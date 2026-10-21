"""MAR-STEP-2: resolved step commands; BE owns workflow transitions."""
from datetime import datetime, timezone
from uuid import UUID
from jsonschema import Draft202012Validator, FormatChecker
from .contracts import StepError, canonical, parse_timestamp

CONTRACT = "MAR-STEP-2"
SUBPROTOCOL = "mar-step-v1"
VERSION = "0.13.0"
UUID_SCHEMA = {"type": "string", "format": "uuid"}

def obj(properties, required=None):
    return {"type": "object", "properties": properties,
            "required": list(properties) if required is None else required, "additionalProperties": False}

def text(enum=None):
    return {"enum": enum} if enum else {"type": "string", "minLength": 1, "maxLength": 2000}

PARAMS = {
    "INTERACT": obj({"prompt_key": text(), "action": {"const": "SPEAK"},
        "data": obj({"text": {"type": "string", "minLength": 1, "maxLength": 500, "pattern": r"\S"},
                     "language_code": text(["vi", "en"]),
                     "speed_wpm": {"type": "integer", "minimum": 80, "maximum": 220},
                     "volume": {"type": "integer", "minimum": 0, "maximum": 100}}, ["text"])}),
    "MOVE": obj({"map_id": UUID_SCHEMA, "endpoint_id": UUID_SCHEMA,
                 "mode": text(["NORMAL", "ESCORT", "PATROL"]),
                 "max_speed_mps": {"type": "number", "exclusiveMinimum": 0, "maximum": 0.3}}),
    "CONTROL_HARDWARE": obj({"compartment_id": text(),
        "action": text(["LOCK_COMPARTMENT", "UNLOCK_COMPARTMENT"]),
        "authorization": obj({"mode": text(["STAFF_VERIFIED"]),
            "verification_id": UUID_SCHEMA, "actor_account_id": UUID_SCHEMA,
            "purpose": text(["DELIVERY_SENDER", "DELIVERY_RECEIVER"]),
            "valid_until": {"type": "string", "format": "date-time"}})},
        ["compartment_id", "action"]),
    "EXECUTE_SENSING": obj({"sensing_type": text(["ROBOT_HEALTH", "SIDE_CLEARANCE",
                            "CROWD_DENSITY", "RESTRICTED_ZONE"]),
        "zone_id": UUID_SCHEMA, "window_sec": {"type": "number", "minimum": 0, "maximum": 30}},
        ["sensing_type"]),
    "WAIT": obj({"wait_type": text(["DURATION", "EVENT"]),
        "duration_sec": {"type": "number", "minimum": 1, "maximum": 3600},
        "event_type": text(), "correlation_key": text()}, ["wait_type"]),
}
SENSING_BASE = PARAMS["EXECUTE_SENSING"]
MAPPING_PARAMS = {
    "oneOf": [obj({"sensing_type":{"const":"MAPPING"},"action":{"const":"START"},
                    "mapping_session_id":UUID_SCHEMA,"preview_rate_hz":{"type":"number","minimum":0.2,"maximum":1}},
                   ["sensing_type","action","mapping_session_id"]),
              obj({"sensing_type":{"const":"MAPPING"},"action":{"const":"SAVE"},
                   "mapping_session_id":UUID_SCHEMA,"artifact_id":UUID_SCHEMA}),
              obj({"sensing_type":{"const":"MAPPING"},"action":{"const":"STOP"},
                   "mapping_session_id":UUID_SCHEMA})]}
PARAMS["EXECUTE_SENSING"] = {"oneOf":[SENSING_BASE,MAPPING_PARAMS]}
STEP_SCHEMA = obj({
    "jobId": UUID_SCHEMA, "jobStepId": UUID_SCHEMA, "stepId": UUID_SCHEMA,
    "stepCode": text(), "stepType": text(["MOVE","INTERACT","DATA_SERVICE","AUTHENTICATE",
        "CONTROL_HARDWARE","HANDLE_PAYLOAD","EXECUTE_SENSING","NOTIFY","WAIT","CHECK_CONDITION","SET_CONTEXT"]),
    "visitNo": {"type": "integer", "minimum": 1},
    "operationKey": {"type": "string", "minLength": 1, "maxLength": 150},
    "timeoutSec": {"type": "number", "minimum": 1, "maximum": 3605},
    "params": {"type": "object"}
})
COMMAND_SCHEMA = obj({
    "protocolVersion": {"const": "1.0"}, "contract": {"const": CONTRACT},
    "messageType": {"const": "COMMAND"}, "commandId": UUID_SCHEMA,
    "correlationId": UUID_SCHEMA, "robotId": UUID_SCHEMA,
    "commandType": text(["EXECUTE_STEP","CANCEL_STEP"]),
    "expiresAt": {"type": "string", "format": "date-time"},
    "step": STEP_SCHEMA, "jobId": UUID_SCHEMA, "jobStepId": UUID_SCHEMA
}, ["protocolVersion","contract","messageType","commandId","correlationId",
    "robotId","commandType","expiresAt"])
COMMAND_SCHEMA["oneOf"] = [
    {"properties":{"commandType":{"const":"EXECUTE_STEP"}}, "required":["step"],
     "not":{"anyOf":[{"required":["jobId"]},{"required":["jobStepId"]}]}},
    {"properties":{"commandType":{"const":"CANCEL_STEP"}}, "required":["jobId","jobStepId"],
     "not":{"required":["step"]}}
]

def validate(value, schema):
    # Reject Python's permissive NaN/Infinity before serialization or ROS calls.
    canonical(value)
    errors = list(Draft202012Validator(schema, format_checker=FormatChecker()).iter_errors(value))
    if errors:
        raise StepError("INVALID_COMMAND", errors[0].message[:300])

def expiry(value):
    try:
        return parse_timestamp(value)
    except ValueError:
        raise StepError("INVALID_EXPIRY") from None

def validate_command(command, robot_id, check_expiry=True):
    validate(command, COMMAND_SCHEMA)
    if command["robotId"] != robot_id:
        raise StepError("WRONG_ROBOT")
    if check_expiry and expiry(command["expiresAt"]) <= datetime.now(timezone.utc):
        raise StepError("COMMAND_EXPIRED")

def validate_step(step, cfg, hardware):
    kind, p = step["stepType"], step["params"]
    if kind == "AUTHENTICATE":
        raise StepError("AUTHENTICATE_NOT_IMPLEMENTED")
    if kind not in PARAMS or (kind == "WAIT" and p.get("wait_type") == "EVENT"):
        raise StepError("SERVER_STEP_REQUIRED")
    validate(p, PARAMS[kind])
    if kind not in cfg["enabled_step_types"]:
        raise StepError("STEP_NOT_ENABLED")
    if kind == "INTERACT":
        if not cfg.get("speech", {}).get("enabled"):
            raise StepError("SPEECH_NOT_ENABLED")
        if step["timeoutSec"] < 5:
            raise StepError("TIMEOUT_TOO_SHORT")
    if kind == "MOVE":
        if "nav2" not in hardware["capabilities"]:
            raise StepError("CAPABILITY_MISSING", "nav2")
        if p["map_id"] != hardware["map_id"] or p["endpoint_id"] not in hardware["endpoints"]:
            raise StepError("ENDPOINT_OR_MAP_NOT_SYNCED")
        if p["max_speed_mps"] > hardware["max_speed_mps"]:
            raise StepError("SPEED_POLICY_EXCEEDED")
    if kind == "CONTROL_HARDWARE":
        if p["compartment_id"] not in cfg["compartments"]:
            raise StepError("COMPARTMENT_NOT_CONFIGURED")
        if p["action"] == "UNLOCK_COMPARTMENT":
            # Staff authentication is explicitly deferred, never silently granted.
            # Only a locally enabled bench/commissioning mode permits opening.
            if cfg["unlock_policy"] != "COMMISSIONING_ONLY":
                raise StepError("AUTHENTICATION_PENDING")
    if kind == "WAIT":
        if set(p) != {"wait_type","duration_sec"} or p["wait_type"] != "DURATION":
            raise StepError("INVALID_WAIT_PARAMS")
        if step["timeoutSec"] < p["duration_sec"] + 1:
            raise StepError("TIMEOUT_TOO_SHORT")
    if kind == "EXECUTE_SENSING":
        if p["sensing_type"] == "MAPPING":
            if not cfg["mapping"]["enabled"]:
                raise StepError("MAPPING_NOT_ENABLED")
            # Starting a stationary scan does not grant a motion lease.
            minimum = {"START":cfg["mapping"]["start_timeout_sec"]+10,"SAVE":60,"STOP":15}[p["action"]]
            if step["timeoutSec"] < minimum:
                raise StepError("TIMEOUT_TOO_SHORT")
            return step
        ai = p["sensing_type"] in ("CROWD_DENSITY","RESTRICTED_ZONE")
        if ai and "zone_id" not in p:
            raise StepError("ZONE_REQUIRED")
        if ai and not cfg["ai"]["enabled"]:
            raise StepError("AI_PROVIDER_NOT_CONFIGURED")
        if step["timeoutSec"] < p.get("window_sec", 0) + 1:
            raise StepError("TIMEOUT_TOO_SHORT")
    return step


def validate_result(step, data):
    canonical(data)
    if not isinstance(data, dict):
        raise StepError("INVALID_STEP_OUTPUT")
    kind, p = step["stepType"], step["params"]
    if kind == "INTERACT" and (data.get("completed") is not True or data.get("action") != "SPEAK"
                               or data.get("prompt_key") != p["prompt_key"] or data.get("source") != "AUDIO_CLIENT"):
        raise StepError("INVALID_SPEECH_OUTPUT")
    if kind == "MOVE" and (data.get("reached") is not True or data.get("endpoint_id") != p["endpoint_id"]):
        raise StepError("INVALID_MOVE_OUTPUT")
    if kind == "CONTROL_HARDWARE":
        expected = 1500 if p["action"] == "LOCK_COMPARTMENT" else 2400
        if (data.get("published") is not True or data.get("verified") is not False
            or data.get("controller_ack") is not False or data.get("position_raw") != expected
            or data.get("ros_servo_id") != 4):
            raise StepError("INVALID_SERVO_OUTPUT")
    if kind == "WAIT" and (data.get("completed") is not True or data.get("wake_reason") != "DURATION"):
        raise StepError("INVALID_WAIT_OUTPUT")
    if kind == "EXECUTE_SENSING":
        if data.get("sensing_type") != p["sensing_type"] or not isinstance(data.get("metrics"), dict):
            raise StepError("INVALID_SENSING_OUTPUT")
        if "captured_at" not in data:
            raise StepError("SENSING_TIMESTAMP_REQUIRED")
        if p["sensing_type"] in ("CROWD_DENSITY", "RESTRICTED_ZONE") and data.get("zone_id") != p["zone_id"]:
            raise StepError("SENSING_ZONE_MISMATCH")
    return data
