"""Write handoff schemas/examples from the worker's actual command definitions."""
from pathlib import Path
import copy
import json
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"pi"))
from mediassist.step_contract import COMMAND_SCHEMA, PARAMS, obj, text, UUID_SCHEMA, CONTRACT
from mediassist.contracts import read_json

def write(name,value):
    (ROOT/"handoff"/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
legacy=read_json(ROOT/"pi/schemas/step_types.json")
server={
 "INTERACT":{"prompt_key":text(),"action":text(["DISPLAY","CAPTURE_INPUT","CONFIRM","SPEAK"]),"data":{"type":"object"}},
 "DATA_SERVICE":{"service_key":text(["QUERY_HOSPITAL_INFORMATION","RESOLVE_TARGETS","SELECT_TARGET","CHECK_TARGET","CREATE_FALL_INCIDENT","COMMIT_MAP_ARTIFACT"]),
                 "mapping_session_id":UUID_SCHEMA,"artifact_id":UUID_SCHEMA,"floor_id":UUID_SCHEMA,"map_name":text(),
                 "question":text(), "endpoint_group_id":UUID_SCHEMA,
                 "targets":{"type":"array","items":{"type":"object"}},
                 "index":{"type":"integer","minimum":0},"endpoint_id":UUID_SCHEMA,
                 "detection_event_id":UUID_SCHEMA},
 "AUTHENTICATE":{"purpose":text(["DELIVERY_SENDER","DELIVERY_RECEIVER"])},
 "HANDLE_PAYLOAD":{"action":text(["LOAD","UNLOAD"]),"service_request_id":UUID_SCHEMA,
                   "compartment_id":text(),"items":{"type":"array","minItems":1,"items":obj({"item_code":text(),"quantity":{"type":"integer","minimum":1}})}},
 "NOTIFY":{"notification_type":text(),"severity":text(["INFO","WARNING","CRITICAL"]),"data":{"type":"object"},"require_ack":{"type":"boolean"}},
 "CHECK_CONDITION":{"expression":obj({"path":text(),"operator":text(["EQ","NE","GT","GTE","LT","LTE","IN","EXISTS"]),"value":{}},["path","operator"])},
 "SET_CONTEXT":{"operations":{"type":"array","minItems":1,"items":obj({"path":{"type":"string","pattern":"^/vars/[A-Za-z][A-Za-z0-9_]*$"},
                        "operation":text(["SET","INCREMENT"]),"value":{}},["path","value"])}}
}
catalog={}
for kind,old in legacy.items():
    pi=kind in PARAMS
    catalog[kind]={"executor":"PI" if pi else "BE",
        "status":"DEFERRED" if kind=="AUTHENTICATE" else "IMPLEMENTED_ON_PI" if pi else "BE_IMPLEMENTATION_REQUIRED",
        "input_schema":PARAMS[kind] if pi else obj(server[kind], ["service_key"] if kind=="DATA_SERVICE" else None),
        "note":""}
catalog["WAIT"]["executor"]="PI_FOR_DURATION_BE_FOR_EVENT"
catalog["WAIT"]["note"]="Pi rejects EVENT. BE owns staff event and correlation."
catalog["CONTROL_HARDWARE"]["note"]="UNLOCK disabled by default; commissioning-only opening is explicitly enabled locally. Staff verification is not implemented."
catalog["EXECUTE_SENSING"]["note"]="Health/US100 implemented. AI requires a real publisher on /mediassist/ai/observations; model weights/inference not included."
catalog["DATA_SERVICE"]["note"]="Resolved flat fields match BE handlers. Required service-specific fields are enforced below."
catalog["DATA_SERVICE"]["input_schema"]["allOf"]=[
    {"if":{"properties":{"service_key":{"const":key}}},"then":{"required":required}}
    for key,required in {"QUERY_HOSPITAL_INFORMATION":["question"],"RESOLVE_TARGETS":["endpoint_group_id"],
    "SELECT_TARGET":["targets","index"],"CHECK_TARGET":["endpoint_id"],"CREATE_FALL_INCIDENT":["detection_event_id"],
    "COMMIT_MAP_ARTIFACT":["mapping_session_id","artifact_id","floor_id","map_name"]}.items()]
catalog["AUTHENTICATE"]["note"]="No successful grant is generated. Delivery with staff authentication is pending."
write("command.schema.json",COMMAND_SCHEMA)
write("step_types.contract.json",{"contract":CONTRACT,"step_types":catalog})
envelope=obj({"protocolVersion":{"const":"1.0"},"messageId":UUID_SCHEMA,"operationKey":text(),
 "correlationId":UUID_SCHEMA,"robotId":UUID_SCHEMA,"jobId":{"type":["string","null"]},
 "jobStepId":{"type":["string","null"]},"messageType":text(["HELLO","HEARTBEAT","COMMAND_ACK","STEP_STARTED","STEP_RESULT","STEP_ERROR","OPERATOR_RECONCILED","PROTOCOL_ERROR"]),
 "occurredAt":{"type":"string","format":"date-time"},"producerSessionId":UUID_SCHEMA,
 "sequenceNo":{"type":"integer","minimum":1},"payload":{"type":"object"}})
write("pi-envelope.schema.json",envelope)
write("server-response.schema.json",{
 "oneOf":[obj({"messageType":{"const":"HELLO_ACK"},"robotId":UUID_SCHEMA,"contract":{"const":CONTRACT},"accepted":{"type":"boolean"}}),
          obj({"messageType":{"const":"EVENT_ACK"},"messageId":UUID_SCHEMA,"accepted":{"type":"boolean"}})]})
robot="11111111-1111-4111-8111-111111111111"
job="22222222-2222-4222-8222-222222222222"
step="33333333-3333-4333-8333-333333333333"
base={"protocolVersion":"1.0","contract":CONTRACT,"messageType":"COMMAND",
 "commandId":"44444444-4444-4444-8444-444444444444",
 "correlationId":"55555555-5555-4555-8555-555555555555","robotId":robot,
 "commandType":"EXECUTE_STEP","expiresAt":"2026-10-08T00:00:00Z",
 "step":{"jobId":job,"jobStepId":step,"stepId":"66666666-6666-4666-8666-666666666666",
 "stepCode":"READ_SIDES","stepType":"EXECUTE_SENSING","visitNo":1,
 "operationKey":job+":READ_SIDES:1","timeoutSec":5,"params":{"sensing_type":"SIDE_CLEARANCE"}}}
write("example-read-sides.json",base)
def variant(name,kind,params,timeout=10):
    item=copy.deepcopy(base)
    item["step"].update(stepCode=name,stepType=kind,params=params,timeoutSec=timeout,operationKey=job+":"+name+":1")
    write("example-"+name.lower().replace("_","-")+".json",item)
variant("WAIT", "WAIT",{"wait_type":"DURATION","duration_sec":2},5)
variant("MOVE","MOVE",{"map_id":"77777777-7777-4777-8777-777777777777",
    "endpoint_id":"88888888-8888-4888-8888-888888888888","mode":"NORMAL","max_speed_mps":0.05},120)
variant("LOCK","CONTROL_HARDWARE",{"compartment_id":"CART_MAIN","action":"LOCK_COMPARTMENT"})
variant("UNLOCK_COMMISSIONING","CONTROL_HARDWARE",{"compartment_id":"CART_MAIN","action":"UNLOCK_COMPARTMENT"})
variant("CROWD","EXECUTE_SENSING",{"sensing_type":"CROWD_DENSITY","zone_id":"99999999-9999-4999-8999-999999999999","window_sec":3},5)
cancel={k:v for k,v in base.items() if k!="step"}
cancel.update(commandType="CANCEL_STEP",commandId="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",jobId=job,jobStepId=step)
write("example-cancel.json",cancel)
write("example-hello-ack.json",{"messageType":"HELLO_ACK","robotId":robot,"contract":CONTRACT,"accepted":True})
write("example-event-ack.json",{"messageType":"EVENT_ACK","messageId":"bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb","accepted":True})
print("Schemas and examples written; example IDs/timestamps must be replaced by BE.")
