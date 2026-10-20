"""Durable commands, jobs and outbox. Interrupted jobs never replay physical steps."""
import json
import sqlite3
import uuid
from datetime import datetime, timezone
from .contracts import canonical, digest, StepError


def utcnow():
    return datetime.now(timezone.utc).isoformat()


class Store:
    def __init__(self, path, recover=False):
        self.db = sqlite3.connect(path)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.executescript("""
        CREATE TABLE IF NOT EXISTS commands(id TEXT PRIMARY KEY, hash TEXT NOT NULL, response TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS jobs(id TEXT PRIMARY KEY, hash TEXT NOT NULL, status TEXT NOT NULL, snapshot TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS outbox(seq INTEGER PRIMARY KEY AUTOINCREMENT, id TEXT UNIQUE NOT NULL, body TEXT NOT NULL, ack INTEGER NOT NULL DEFAULT 0);
        """)
        self.session = str(uuid.uuid4())
        for (job_id,) in (self.db.execute("SELECT id FROM jobs WHERE status IN ('RUNNING','ACCEPTED')").fetchall() if recover else []):
            self.status(job_id, "RECOVERY_REQUIRED")
            self.event(job_id, "RECOVERY_REQUIRED", {"reason": "PROCESS_RESTART_NO_AUTO_REPLAY"})

    def cached_command(self, command):
        row = self.db.execute("SELECT hash,response FROM commands WHERE id=?", (command["command_id"],)).fetchone()
        if row and row[0] != digest(command):
            raise StepError("COMMAND_ID_CONFLICT")
        return json.loads(row[1]) if row else None

    def remember_command(self, command, response):
        with self.db:
            self.db.execute("INSERT INTO commands VALUES(?,?,?)", (command["command_id"], digest(command), canonical(response)))

    def add_job(self, job):
        row = self.db.execute("SELECT hash,status FROM jobs WHERE id=?", (job["job_id"],)).fetchone()
        if row:
            if row[0] != digest(job): raise StepError("JOB_ID_CONFLICT")
            return row[1]
        with self.db:
            self.db.execute("INSERT INTO jobs VALUES(?,?,?,?)", (job["job_id"], digest(job), "ACCEPTED", canonical(job)))
        return None

    def status(self, job_id, status):
        with self.db:
            self.db.execute("UPDATE jobs SET status=? WHERE id=?", (status, job_id))

    def event(self, job_id, event_type, data):
        event_id = str(uuid.uuid4())
        body = {"protocol_version": 1, "type": "EVENT", "event_id": event_id,
                "producer_session_id": self.session, "job_id": job_id,
                "event_type": event_type, "occurred_at": utcnow(), "data": data}
        with self.db:
            cursor = self.db.execute("INSERT INTO outbox(id,body) VALUES(?,?)", (event_id, "{}"))
            body["sequence_no"] = cursor.lastrowid
            self.db.execute("UPDATE outbox SET body=? WHERE id=?", (canonical(body), event_id))
        return body

    def pending(self, limit=100):
        return [json.loads(r[0]) for r in self.db.execute("SELECT body FROM outbox WHERE ack=0 ORDER BY seq LIMIT ?", (limit,))]

    def reconcile(self, job_id, reason):
        """Commit the selected legacy job and its audit event in one transaction."""
        if not reason.strip():
            raise StepError("RECONCILIATION_REASON_REQUIRED")
        with self.db:
            changed = self.db.execute(
                "UPDATE jobs SET status='CANCELLED' WHERE id=? AND status='RECOVERY_REQUIRED'", (job_id,))
            if changed.rowcount != 1:
                raise StepError("JOB_NOT_RECOVERY_REQUIRED")
            event_id = str(uuid.uuid4())
            body = {"protocol_version": 1, "type": "EVENT", "event_id": event_id,
                    "producer_session_id": self.session, "job_id": job_id,
                    "event_type": "OPERATOR_RECONCILED", "occurred_at": utcnow(),
                    "data": {"reason": reason.strip(), "status": "CANCELLED", "source": "LOCAL_OPERATOR"}}
            cursor = self.db.execute("INSERT INTO outbox(id,body) VALUES(?,?)", (event_id, "{}"))
            body["sequence_no"] = cursor.lastrowid
            self.db.execute("UPDATE outbox SET body=? WHERE id=?", (canonical(body), event_id))

    def ack(self, event_id):
        with self.db:
            self.db.execute("UPDATE outbox SET ack=1 WHERE id=?", (event_id,))

    def states(self):
        return dict(self.db.execute("SELECT id,status FROM jobs"))

    def close(self):
        self.db.close()
