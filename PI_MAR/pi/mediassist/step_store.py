"""Durable command receipt and outbox; interrupted actions never replay."""
import json
import sqlite3
import uuid
from .contracts import canonical, digest, StepError
from .store import utcnow

TERMINAL = {"SUCCEEDED", "FAILED", "TIMED_OUT", "CANCELLED", "RECOVERY_REQUIRED"}

class StepStore:
    def __init__(self, path, robot_id):
        self.db = sqlite3.connect(path)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.executescript("""
          CREATE TABLE IF NOT EXISTS worker_identity(robot_id TEXT PRIMARY KEY);
          CREATE TABLE IF NOT EXISTS worker_steps(
            operation TEXT PRIMARY KEY, step_id TEXT UNIQUE NOT NULL,
            job_id TEXT NOT NULL, hash TEXT NOT NULL, body TEXT NOT NULL,
            status TEXT NOT NULL, result TEXT);
          CREATE TABLE IF NOT EXISTS worker_commands(
            command_id TEXT PRIMARY KEY, hash TEXT NOT NULL, ack_id TEXT NOT NULL);
          CREATE TABLE IF NOT EXISTS navigation_guidance(
            step_id TEXT NOT NULL, cue_id TEXT NOT NULL, status TEXT NOT NULL,
            result TEXT, updated_at TEXT NOT NULL, PRIMARY KEY(step_id,cue_id));
          CREATE TABLE IF NOT EXISTS worker_outbox(
            seq INTEGER PRIMARY KEY AUTOINCREMENT, message_id TEXT UNIQUE NOT NULL,
            body TEXT NOT NULL, acked INTEGER NOT NULL DEFAULT 0);
        """)
        columns = {r[1] for r in self.db.execute("PRAGMA table_info(worker_outbox)")}
        if "superseded_by" not in columns:
            with self.db:
                self.db.execute("ALTER TABLE worker_outbox ADD COLUMN superseded_by TEXT")
        identities = [r[0] for r in self.db.execute("SELECT robot_id FROM worker_identity")]
        if identities and identities != [robot_id]:
            raise StepError("JOURNAL_ROBOT_MISMATCH")
        with self.db:
            self.db.execute("INSERT OR IGNORE INTO worker_identity VALUES(?)", (robot_id,))
        self.robot_id, self.session = robot_id, str(uuid.uuid4())
        self.sequence = 0
        for operation, raw in self.db.execute(
                "SELECT operation,body FROM worker_steps WHERE status IN ('ACCEPTED','RUNNING')").fetchall():
            step = json.loads(raw)
            self.finish(step, "RECOVERY_REQUIRED",
                        {"errorCode":"PROCESS_RESTART", "physicalStopConfirmed":False})
            self.emit("STEP_ERROR", {"status":"RECOVERY_REQUIRED", "errorCode":"PROCESS_RESTART",
                      "retryable":False, "details":{"code":"PROCESS_RESTART"}},
                      step, str(uuid.uuid4()))

    def guidance_start(self, step, cue):
        with self.db:
            cursor = self.db.execute('INSERT OR IGNORE INTO navigation_guidance VALUES(?,?,?,?,?)',
                (step['jobStepId'], cue, 'STARTED', None, utcnow()))
        return cursor.rowcount == 1

    def guidance_result(self, step, cue, status, result):
        with self.db:
            self.db.execute('UPDATE navigation_guidance SET status=?,result=?,updated_at=? WHERE step_id=? AND cue_id=?',
                (status, json.dumps(result), utcnow(), step['jobStepId'], cue))

    def emit(self, kind, payload, step=None, correlation=None):
        if self.db.execute("SELECT count(*) FROM worker_outbox WHERE acked=0 AND superseded_by IS NULL").fetchone()[0] >= 10000:
            raise StepError("OUTBOX_FULL")
        mid = str(uuid.uuid4())
        self.sequence += 1
        with self.db:
            cur = self.db.execute("INSERT INTO worker_outbox(message_id,body) VALUES(?,?)", (mid,"{}"))
            body = {"protocolVersion":"1.0", "messageId":mid,
                "operationKey":step["operationKey"] if step else mid,
                "correlationId":correlation or mid, "robotId":self.robot_id,
                "jobId":step["jobId"] if step else None,
                "jobStepId":step["jobStepId"] if step else None,
                "messageType":kind, "occurredAt":utcnow(), "producerSessionId":self.session,
                "sequenceNo":self.sequence, "payload":payload}
            self.db.execute("UPDATE worker_outbox SET body=? WHERE message_id=?", (canonical(body),mid))
        return body

    def transient(self, kind, payload):
        self.sequence += 1
        mid = str(uuid.uuid4())
        return {"protocolVersion":"1.0", "messageId":mid, "operationKey":mid,
            "correlationId":mid, "robotId":self.robot_id, "jobId":None, "jobStepId":None,
            "messageType":kind, "occurredAt":utcnow(), "producerSessionId":self.session,
            "sequenceNo":self.sequence, "payload":payload}

    def remember_command(self, command, event):
        with self.db:
            self.db.execute("INSERT INTO worker_commands VALUES(?,?,?)",
                (command["commandId"],digest(command),event["messageId"]))

    def cached_command(self, command):
        row = self.db.execute("SELECT hash,ack_id FROM worker_commands WHERE command_id=?",
                              (command["commandId"],)).fetchone()
        if not row:
            return None
        if row[0] != digest(command):
            raise StepError("COMMAND_ID_CONFLICT")
        with self.db:
            self.db.execute("UPDATE worker_outbox SET acked=0 WHERE message_id=?", (row[1],))
        return row[1]

    def previous(self, step):
        row = self.db.execute("SELECT hash,status,result FROM worker_steps WHERE operation=? OR step_id=?",
            (step["operationKey"],step["jobStepId"])).fetchone()
        if row:
            if row[0] != digest(step):
                raise StepError("STEP_IDENTITY_CONFLICT")
            return {"status":row[1], "result":json.loads(row[2]) if row[2] else None}
        return None

    def accept(self, step):
        with self.db:
            self.db.execute("INSERT INTO worker_steps VALUES(?,?,?,?,?,?,NULL)",
                (step["operationKey"],step["jobStepId"],step["jobId"],digest(step),canonical(step),"ACCEPTED"))

    def set_running(self, step):
        with self.db:
            self.db.execute("UPDATE worker_steps SET status='RUNNING' WHERE operation=?", (step["operationKey"],))

    def finish(self, step, status, result):
        with self.db:
            self.db.execute("UPDATE worker_steps SET status=?,result=? WHERE operation=?",
                (status, canonical(result), step["operationKey"]))

    def pending(self):
        return [json.loads(r[0]) for r in self.db.execute(
            "SELECT body FROM worker_outbox WHERE acked=0 AND superseded_by IS NULL ORDER BY seq LIMIT 100")]

    def ack(self, mid):
        with self.db:
            self.db.execute("UPDATE worker_outbox SET acked=1 WHERE message_id=?", (mid,))

    def is_acked(self, mid):
        row = self.db.execute("SELECT acked FROM worker_outbox WHERE message_id=?", (mid,)).fetchone()
        return bool(row and row[0])

    def states(self):
        return [{"jobId":r[0],"jobStepId":r[1],"operationKey":r[2],"status":r[3]}
            for r in self.db.execute("SELECT job_id,step_id,operation,status FROM worker_steps WHERE status IN ('ACCEPTED','RUNNING','RECOVERY_REQUIRED') OR rowid IN (SELECT rowid FROM worker_steps ORDER BY rowid DESC LIMIT 30)")]

    def blocked(self):
        return bool(self.db.execute("SELECT 1 FROM worker_steps WHERE status='RECOVERY_REQUIRED' LIMIT 1").fetchone())

    def stopped_move_settlements(self):
        pending = {event.get('jobStepId') for event in self.pending()}
        commands = {}
        for (body,) in self.db.execute('SELECT body FROM worker_outbox ORDER BY seq'):
            event = json.loads(body)
            payload = event.get('payload', {})
            command_id = payload.get('commandId')
            if (event.get('messageType') == 'COMMAND_ACK' and payload.get('commandType') == 'EXECUTE_STEP'
                    and isinstance(command_id,str) and command_id == event.get('correlationId')):
                key = (event.get('jobId'),event.get('jobStepId'),event.get('operationKey'))
                commands.setdefault(key,set()).add(command_id)
        candidates = []
        for sid, raw, status, result in self.db.execute(
                "SELECT step_id,body,status,result FROM worker_steps "
                "WHERE status IN ('RECOVERY_REQUIRED','FAILED','TIMED_OUT','CANCELLED')"):
            if status != 'RECOVERY_REQUIRED' and sid not in pending:
                continue
            step = json.loads(raw)
            evidence = json.loads(result) if result else {}
            if step.get('stepType') != 'MOVE' or evidence.get('physicalStopConfirmed') is not True:
                continue
            command_ids = commands.get((step['jobId'],sid,step['operationKey']),set())
            if len(command_ids) == 1:
                candidates.append(dict(jobId=step['jobId'],jobStepId=sid,
                    commandId=next(iter(command_ids)),operationKey=step['operationKey']))
        return candidates

    def settle_cancelled_move(self, candidate, receipt):
        """Retire a stopped MOVE only after BE confirms its matching terminal outcome."""
        from uuid import UUID
        if (not isinstance(receipt, dict) or receipt.get('robotId') != self.robot_id
                or any(receipt.get(key) != candidate[key] for key in
                       ('jobId','jobStepId','commandId','operationKey'))
                or receipt.get('status') not in ('CANCELLED','FAILED') or receipt.get('physicalStopConfirmed') is not True):
            raise StepError('CANCEL_SETTLEMENT_IDENTITY_MISMATCH')
        try:
            UUID(receipt['receiptId'])
            if receipt['status']=='FAILED':
                from .contracts import parse_timestamp
                if receipt.get('reconciliationOutcome')!='CONFIRM_FAILED':
                    raise ValueError()
                parse_timestamp(receipt['reconciledAt'])
        except (KeyError,ValueError,TypeError,AttributeError):
            raise StepError('CANCEL_SETTLEMENT_RECEIPT_INVALID') from None
        with self.db:
            if candidate not in self.stopped_move_settlements():
                raise StepError('CANCEL_SETTLEMENT_LOCAL_STATE_CHANGED')
            raw, status = self.db.execute('SELECT result,status FROM worker_steps WHERE step_id=?',
                                         (candidate['jobStepId'],)).fetchone()
            result = {**json.loads(raw),'status':receipt['status'],'previousStatus':status,
                      'settlement':receipt}
            self.db.execute('UPDATE worker_steps SET status=?,result=? WHERE step_id=?',
                            (receipt['status'],canonical(result),candidate['jobStepId']))
            superseded = []
            for mid, body in self.db.execute(
                    'SELECT message_id,body FROM worker_outbox WHERE acked=0 AND superseded_by IS NULL'):
                event = json.loads(body)
                if (event.get('jobId') == candidate['jobId'] and event.get('jobStepId') == candidate['jobStepId']
                        and event.get('operationKey') == candidate['operationKey']):
                    superseded.append((receipt['receiptId'],mid))
            self.db.executemany('UPDATE worker_outbox SET superseded_by=? WHERE message_id=? AND acked=0',
                                superseded)

    def reconcile(self, sid, reason, allow_terminal_failed_mapping=False):
        row = self.db.execute("SELECT body,status FROM worker_steps WHERE step_id=?", (sid,)).fetchone()
        if not row:
            raise StepError("STEP_NOT_RECOVERY_REQUIRED")
        step = json.loads(row[0])
        failed_mapping = (allow_terminal_failed_mapping and row[1] in {"FAILED", "TIMED_OUT", "CANCELLED"}
                          and step.get("stepType") == "EXECUTE_SENSING"
                          and step.get("params", {}).get("sensing_type") == "MAPPING")
        if row[1] != "RECOVERY_REQUIRED" and not failed_mapping:
            raise StepError("STEP_NOT_RECOVERY_REQUIRED")
        if not reason.strip():
            raise StepError("RECONCILIATION_REASON_REQUIRED")
        command_id = None
        superseded = []
        for mid, raw, acked in self.db.execute("SELECT message_id,body,acked FROM worker_outbox ORDER BY seq"):
            event = json.loads(raw)
            if event.get("jobStepId") != sid or event.get("operationKey") != step["operationKey"]:
                continue
            payload = event.get("payload", {})
            if event.get("messageType") == "COMMAND_ACK" and payload.get("commandType") == "EXECUTE_STEP":
                candidate = payload.get("commandId")
                if candidate and event.get("correlationId") == candidate:
                    command_id = candidate
            if not acked:
                superseded.append(mid)
        if command_id is None:
            raise StepError("RECONCILIATION_COMMAND_RECEIPT_MISSING")
        # Retain old events for audit; an explicit operator receipt replaces their delivery.
        self.finish(step, "CANCELLED", {"reason":reason,"source":"LOCAL_OPERATOR","previousStatus":row[1]})
        receipt = self.emit("OPERATOR_RECONCILED", {"reason":reason,"status":"CANCELLED"}, step, command_id)
        with self.db:
            self.db.executemany("UPDATE worker_outbox SET superseded_by=? WHERE message_id=? AND acked=0",
                                [(receipt["messageId"], mid) for mid in superseded])
        return receipt

    def close(self):
        self.db.close()
