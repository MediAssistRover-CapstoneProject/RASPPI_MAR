import asyncio
import copy
import time
import uuid
from datetime import datetime, timezone
from .contracts import StepError, resolve, evaluate, validate, digest
from .store import utcnow


class Runtime:
    """Execute one workflow at a time using the installed 11-type catalog."""
    def __init__(self, catalog, store, adapter, system=None):
        self.catalog, self.store, self.adapter = catalog, store, adapter
        self.system = system or {}
        self.cancel = asyncio.Event()
        self.active = None
        if hasattr(adapter, "event_sink"): adapter.event_sink = store.event

    def prepare(self, job):
        wf = self.catalog.get(job["workflow_code"], job["workflow_version"])
        if job["workflow_hash"] != digest(wf):
            raise StepError("WORKFLOW_HASH_MISMATCH")
        validate(job["input"], wf["input_schema"])
        self.adapter.check_capabilities(wf)
        return wf

    async def run(self, job):
        if self.active is not None:
            raise StepError("ROBOT_BUSY")
        wf = self.prepare(job)
        self.active = job["job_id"]
        self.cancel.clear()
        self.store.status(job["job_id"], "RUNNING")
        try:
            result = await asyncio.wait_for(self._workflow(job, wf, 0), wf["max_duration_sec"])
            if result["status"] in ("FAILED", "CANCELLED") and any(s["type"] == "CONTROL_HARDWARE" for s in wf["steps"]):
                result["status"] = "RECOVERY_REQUIRED"
            if getattr(self.adapter, "motion_uncertain", False):
                result["status"], result["error_code"] = "RECOVERY_REQUIRED", "GOAL_STOP_UNCONFIRMED"
            self.store.status(job["job_id"], result["status"])
            return result
        except asyncio.TimeoutError:
            result = {"status": "RECOVERY_REQUIRED", "error_code": "JOB_TIMEOUT"}
            self.store.status(job["job_id"], result["status"])
            return result
        except asyncio.CancelledError:
            result = {"status": "RECOVERY_REQUIRED", "error_code": "CANCELLED_RECONCILE_PHYSICAL_STATE"}
            self.store.status(job["job_id"], "RECOVERY_REQUIRED")
            return result
        except Exception as exc:
            result = {"status": "RECOVERY_REQUIRED", "error_code": getattr(exc, "code", "RUNTIME_ERROR")}
            self.store.status(job["job_id"], result["status"])
            return result
        finally:
            # A stop failure cannot turn an uncertain physical state into success.
            result = locals().get("result", {"status": "RECOVERY_REQUIRED"})
            try:
                stopped = await self.adapter.stop()
                if stopped is False: raise StepError("GOAL_STOP_UNCONFIRMED")
            except Exception:
                result.update(status="RECOVERY_REQUIRED", error_code="GOAL_STOP_UNCONFIRMED")
                self.store.status(job["job_id"], "RECOVERY_REQUIRED")
            finally:
                self.store.event(job["job_id"], "JOB_RESULT", result)
                self.active = None

    async def _workflow(self, job, wf, depth):
        context = {"input": copy.deepcopy(job["input"]), "vars": {}, "outputs": {},
                   "system": copy.deepcopy(self.system), "_auth_grants": {}}
        code, visits, count = wf["start"], {}, 0
        self.store.event(job["job_id"], "JOB_STARTED", {"workflow_code": wf["code"], "version": wf["version"], "simulation": self.adapter.simulated})
        while True:
            if self.cancel.is_set(): raise asyncio.CancelledError
            count += 1
            if count > wf["max_step_visits"]: raise StepError("VISIT_LIMIT")
            step = next(s for s in wf["steps"] if s["code"] == code)
            visits[code] = visits.get(code, 0) + 1
            execution_id = str(uuid.uuid4())
            context["outputs"].pop(code, None)  # Never reuse this step's old result after a failed revisit.
            meta = {"job_id": job["job_id"], "job_step_id": execution_id, "step_code": code,
                    "task_code": step["task_code"],
                    "visit_no": visits[code], "operation_key": execution_id, "attempt_no": 1,
                    "timeout_sec": step["timeout_sec"], "simulation": self.adapter.simulated}
            outcome, error, result = "FAILURE", None, None
            try:
                params = resolve(step["params"], context)
                self.catalog.check_params(step["type"], params)
            except StepError as exc:
                error = exc
                params = None
            if params is not None:
                for attempt in range(1, step["max_attempts"] + 1):
                    if self.cancel.is_set(): raise asyncio.CancelledError
                    meta["attempt_no"] = attempt
                    started, start_mono = utcnow(), time.monotonic()
                    self.store.event(job["job_id"], "STEP_STARTED", {**meta, "type": step["type"], "resolved_input": params})
                    task = asyncio.create_task(self._execute(step["type"], params, context, meta, depth))
                    cancelled = asyncio.create_task(self.cancel.wait())
                    try:
                        done, _ = await asyncio.wait({task, cancelled}, timeout=step["timeout_sec"], return_when=asyncio.FIRST_COMPLETED)
                        if cancelled in done: raise asyncio.CancelledError
                        if task not in done: raise asyncio.TimeoutError
                        data = await task
                        self.catalog.check_output(step["type"], data)
                        if step["type"] == "AUTHENTICATE" and data["authenticated"] is True:
                            context["_auth_grants"][data["verification_id"]] = copy.deepcopy(data)
                        result = {"success": True, "started_at": started, "completed_at": utcnow(),
                                  "duration_ms": round((time.monotonic() - start_mono) * 1000), "attempt": attempt, "data": data}
                        context["outputs"][code] = data
                        outcome, error = "SUCCESS", None
                    except asyncio.TimeoutError:
                        outcome, error = "TIMEOUT", StepError("STEP_TIMEOUT")
                    except asyncio.CancelledError:
                        self.store.event(job["job_id"], "STEP_RESULT", {**meta, "outcome": "CANCELLED", "result": None,
                                         "error": {"code": "INTERRUPTED_REQUIRES_RECONCILIATION", "retryable": False}})
                        raise
                    except StepError as exc:
                        outcome, error = "FAILURE", exc
                    finally:
                        task.cancel()
                        cancelled.cancel()
                        await asyncio.gather(task, cancelled, return_exceptions=True)
                    self.store.event(job["job_id"], "STEP_RESULT", {**meta, "outcome": outcome, "result": result,
                                     "error": None if error is None else {"code": error.code, "retryable": error.retryable}})
                    if outcome == "SUCCESS": break
                    if not error.retryable or error.code not in step["retry_codes"] or attempt == step["max_attempts"]: break
                    await asyncio.sleep(step["retry_delay_sec"])
            else:
                self.store.event(job["job_id"], "STEP_RESULT", {**meta, "outcome": outcome, "result": None, "error": {"code": error.code}})
            scope = {**context, "result": result["data"] if result else None,
                     "error": {"code": error.code} if error else None}
            selected = None
            for tr in sorted(step["transitions"], key=lambda t: t["priority"]):
                if tr["on"] != outcome: continue
                if "error_code" in tr and (error is None or tr["error_code"] != error.code): continue
                if "when" in tr:
                    matched = evaluate(tr["when"], scope)
                    if type(matched) is not bool: raise StepError("CONDITION_NOT_BOOLEAN")
                    if not matched: continue
                selected = tr
                break
            if selected is None: raise StepError("NO_TRANSITION", code)
            self.store.event(job["job_id"], "TRANSITION_SELECTED", {**meta, "transition": selected})
            if "terminal" in selected:
                return {"status": selected["terminal"], "output": resolve(wf["output_mapping"], context) if selected["terminal"] == "SUCCEEDED" else {},
                        "error_code": error.code if error else None}
            code = selected["next"]

    async def _execute(self, kind, p, context, meta, depth):
        if kind == "CONTROL_HARDWARE" and p["action"] == "UNLOCK":
            grant = context["_auth_grants"].get(p["verification_id"])
            if not grant or grant["purpose"] != p["purpose"]:
                raise StepError("AUTH_GRANT_REQUIRED")
            expiry = datetime.fromisoformat(grant["valid_until"].replace("Z", "+00:00"))
            if expiry.tzinfo is None or expiry <= datetime.now(timezone.utc):
                raise StepError("AUTH_GRANT_EXPIRED")
        if kind == "CHECK_CONDITION":
            value = evaluate(p["expression"], context)
            if type(value) is not bool: raise StepError("CONDITION_NOT_BOOLEAN")
            return {"value": value}
        if kind == "SET_CONTEXT":
            values = {k: evaluate(v, context) for k, v in p["assignments"].items()}
            if any(not k.isidentifier() or k.startswith("_") for k in values): raise StepError("INVALID_CONTEXT_KEY")
            context["vars"].update(values)
            return {"changed_keys": list(values), "values": values}
        if kind == "WAIT" and p["wait_type"] == "DURATION":
            start = time.monotonic()
            await asyncio.sleep(p["duration_sec"] if not self.adapter.simulated else min(p["duration_sec"], 0.001))
            return {"completed": True, "wake_reason": "DURATION", "elapsed_sec": time.monotonic() - start, "event": None}
        return await self.adapter.execute(kind, p, meta)
