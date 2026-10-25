"""Validate hardware profiles before opening ROS, GPIO or network connections."""
import math
from pathlib import Path
from .contracts import StepError, read_json


def number(value, low, high, label):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise StepError("INVALID_CONFIG", label)


def validate_config(cfg):
    try:
        if not isinstance(cfg["robot_id"], str) or not cfg["robot_id"]:
            raise StepError("INVALID_CONFIG", "robot_id")
        # Higher translation is opt-in on M1 Mecanum; other chassis keep their cap.
        mecanum = cfg.get('motion', {}).get('chassis_model') == 'MENTORPI_M1_MECANUM'
        number(cfg["max_speed_mps"], 0.001, 1.0 if mecanum else 0.3, "max_speed_mps")
        for name in ("capabilities", "allowed_workflows"):
            if not isinstance(cfg[name], list) or any(not isinstance(v, str) for v in cfg[name]):
                raise StepError("INVALID_CONFIG", name)
        m, s, u = cfg["motion"], cfg["servo"], cfg["us100"]
        for key in ("commissioned", "cart_footprint_validated", "watchdog_validated"):
            if type(m[key]) is not bool: raise StepError("INVALID_CONFIG", "motion." + key)
        for key in ("sensor_max_age_sec", "lease_timeout_sec", "command_timeout_sec"):
            number(m[key], 0.05, 2, key)
        if type(m.get("us100_required", True)) is not bool:
            raise StepError("INVALID_CONFIG", "motion.us100_required")
        number(m["stop_distance_m"], 0.025, 2, "stop_distance_m")
        number(m["max_angular_speed_rps"], 0.01, 1.0, "max_angular_speed_rps")
        if 'allow_strafe' in m and (type(m['allow_strafe']) is not bool or m['allow_strafe'] and m.get('chassis_model')!='MENTORPI_M1_MECANUM'):
            raise StepError('INVALID_CONFIG','allow_strafe requires Mecanum')
        if m.get('allow_reverse',False):
            if m.get('chassis_model')!='MENTORPI_M1_MECANUM':
                raise StepError('REVERSE_REQUIRES_M1_PROFILE')
            number(m.get('max_reverse_speed_mps',.10),.01,.30,'max_reverse_speed_mps')
            number(m.get('rear_stop_distance_m',.30),.15,1.,'rear_stop_distance_m')
        if 'allow_reverse' in m and type(m['allow_reverse']) is not bool:
            raise StepError('INVALID_CONFIG','allow_reverse')
        if s["model"] != "MG996R" or s["port"] != "S6" or s["ros_id"] != 4:
            raise StepError("INVALID_SERVO_PROFILE")
        if type(s["commissioned"]) is not bool or s["physical_feedback_available"] is not False:
            raise StepError("INVALID_SERVO_PROFILE")
        for key, expected in (("lock_position_raw", 1500), ("unlock_position_raw", 2400)):
            if type(s[key]) is not int or s[key] != expected:
                raise StepError("INVALID_SERVO_PROFILE", key)
        number(s["duration_sec"], 0.1, 3, "servo.duration_sec")
        for side, pins in (("left", (17, 18)), ("right", (22, 23))):
            if (u[side]["trig_bcm"], u[side]["echo_bcm"]) != pins:
                raise StepError("WIRING_PROFILE_MISMATCH", side)
        if u["mode"] != "TRIG_ECHO": raise StepError("WIRING_PROFILE_MISMATCH")
        number(u["fov_deg"], 1, 180, "us100.fov_deg")
        for key, topic in cfg["ros"].items():
            if not isinstance(topic, str) or not topic: raise StepError("INVALID_CONFIG", key)
        if cfg["ros"]["raw_cmd_vel"] == cfg["ros"]["safe_cmd_vel"]:
            raise StepError("VELOCITY_TOPIC_LOOP")
        n = cfg["navigation"]
        number(n["planning_timeout_sec"], 1, 60, "planning_timeout_sec")
        if type(n["max_candidates"]) is not int or not 1 <= n["max_candidates"] <= 8 or n["group_fallback"] != "PLAN_ONLY":
            raise StepError("INVALID_CONFIG", "navigation")
        for key, point in cfg["endpoints"].items():
            if not key or point["map_id"] != cfg["map_id"]: raise StepError("INVALID_CONFIG", "endpoint map")
            for field in ("x_m", "y_m", "yaw_rad"):
                number(point[field], -100000, 100000, field)
        for group in cfg["endpoint_groups"].values():
            ids = group["endpoint_ids"]
            if not ids or len(set(ids)) != len(ids) or not set(ids) <= set(cfg["endpoints"]):
                raise StepError("INVALID_CONFIG", "group members")
            if group["map_id"] != cfg["map_id"] or type(group["revision"]) is not int or group["revision"] < 1:
                raise StepError("INVALID_CONFIG", "group revision/map")
    except (KeyError, TypeError, AttributeError) as exc:
        raise StepError("INVALID_CONFIG", str(exc)) from None
    return cfg


def load_config(path, simulation=False):
    path = Path(path)
    cfg = read_json(path)
    ca_file = cfg.get("tls_ca_file")
    if ca_file is not None:
        if not isinstance(ca_file, str) or not ca_file.strip():
            raise StepError("INVALID_CONFIG", "tls_ca_file")
        ca_path = Path(ca_file).expanduser()
        if not ca_path.is_absolute(): ca_path = path.resolve().parent / ca_path
        if not ca_path.is_file(): raise StepError("TLS_CA_FILE_NOT_FOUND", str(ca_path))
        cfg["tls_ca_file"] = str(ca_path)
    return cfg if simulation else validate_config(cfg)
