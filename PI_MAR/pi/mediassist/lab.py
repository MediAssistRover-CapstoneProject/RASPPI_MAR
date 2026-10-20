"""Stage-one exercises: local logic and input validation, without ROS or network."""
import argparse
import asyncio
import json
import math
import uuid
from pathlib import Path
from .contracts import Catalog, StepError, digest, read_json
from .runtime import Runtime
from .store import Store

class LogicOnlyAdapter:
    simulated = False
    def check_capabilities(self, workflow):
        allowed = {"SET_CONTEXT", "CHECK_CONDITION", "WAIT"}
        for step in workflow["steps"]:
            if step["type"] not in allowed:
                raise StepError("LAB_LOGIC_ONLY")
            if step["type"] == "WAIT" and step["params"].get("wait_type") != "DURATION":
                raise StepError("LAB_DURATION_ONLY")
    async def execute(self, kind, params, meta):
        raise StepError("LAB_EXTERNAL_IO_DISABLED")
    async def stop(self):
        pass

def logic_workflow(duration=2.0, condition=True):
    if not math.isfinite(duration) or not 0 <= duration <= 10:
        raise ValueError("duration must be finite and within 0..10 seconds")
    def step(code, kind, params, next_code=None):
        success = {"next": next_code} if next_code else {"terminal":"SUCCEEDED"}
        return {"code":code,"task_code":"MAIN","type":kind,"params":params,
                "timeout_sec":15,"max_attempts":1,"retry_delay_sec":0,"retry_codes":[],
                "transitions":[{"on":"SUCCESS","priority":100,**success},
                               {"on":"FAILURE","priority":100,"terminal":"FAILED"},
                               {"on":"TIMEOUT","priority":100,"terminal":"FAILED"}]}
    steps = [
        step("INIT","SET_CONTEXT",{"assignments":{"phase":"READY","loops":0}},"WAIT_2S"),
        step("WAIT_2S","WAIT",{"wait_type":"DURATION","duration_sec":duration},"CHECK"),
        step("CHECK","CHECK_CONDITION",{"expression":{"op":"eq","args":[
            {"var":"/vars/phase"},"READY" if condition else "NOT_READY"]}})]
    steps[-1]["transitions"] = [
        {"on":"SUCCESS","priority":1,"when":{"var":"/result/value"},"terminal":"SUCCEEDED"},
        {"on":"SUCCESS","priority":100,"terminal":"FAILED"},
        {"on":"FAILURE","priority":100,"terminal":"FAILED"},
        {"on":"TIMEOUT","priority":100,"terminal":"FAILED"}]
    return {"code":"LAB_LOGIC","version":4,"name":"Thu logic cuc bo",
            "deployment_note":"Local logic only, no ROS and no network.",
            "required_capabilities":[],"tasks":[{"code":"MAIN","name":"Logic","display_order":0}],
            "input_schema":{"type":"object","properties":{},"required":[],"additionalProperties":False},
            "start":"INIT","max_duration_sec":30,"max_step_visits":5,
            "output_mapping":{"phase":{"$ref":"/vars/phase"},
                              "wait":{"$ref":"/outputs/WAIT_2S"},
                              "condition":{"$ref":"/outputs/CHECK/value"}},
            "steps":steps}

async def run_logic(root, duration=2.0, condition=True):
    catalog=Catalog(root)
    wf=logic_workflow(duration,condition)
    catalog.check_workflow(wf)
    catalog.workflows[(wf["code"],wf["version"])]=wf
    store=Store(":memory:")
    try:
        runtime=Runtime(catalog,store,LogicOnlyAdapter())
        job={"job_id":str(uuid.uuid4()),"workflow_code":wf["code"],"workflow_version":4,
             "workflow_hash":digest(wf),"input":{}}
        store.add_job(job)
        result=await runtime.run(job)
        events=store.pending(10000)
        return {"test_mode":"LOCAL_LOGIC","hardware_io":False,**result},events
    finally:
        store.close()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root",type=Path,default=Path.cwd())
    sub=parser.add_subparsers(dest="command",required=True)
    logic=sub.add_parser("logic")
    logic.add_argument("--duration-sec",type=float,default=2.0)
    logic.add_argument("--condition",choices=["true","false"],default="true")
    logic.add_argument("--events",type=Path)
    check=sub.add_parser("check")
    check.add_argument("--type",required=True)
    check.add_argument("--params",type=Path,required=True)
    args=parser.parse_args()
    try:
        if args.command=="check":
            catalog=Catalog(args.root)
            if args.type not in catalog.types:raise StepError("UNKNOWN_STEP_TYPE")
            catalog.check_params(args.type,read_json(args.params))
            print(json.dumps({"step_type":args.type,"schema_valid":True,"executed":False}))
            return
        result,events=asyncio.run(run_logic(args.root,args.duration_sec,args.condition=="true"))
        if args.events:
            args.events.parent.mkdir(parents=True,exist_ok=True)
            args.events.write_text("".join(json.dumps(e,ensure_ascii=False)+"\n" for e in events),encoding="utf-8")
        print(json.dumps(result,ensure_ascii=False,indent=2))
        if result["status"]!="SUCCEEDED":raise SystemExit(1)
    except (StepError,ValueError) as exc:
        print(json.dumps({"error_code":getattr(exc,"code","INVALID_VALUE"),"message":str(exc)}))
        raise SystemExit(1)

if __name__=="__main__":
    main()

