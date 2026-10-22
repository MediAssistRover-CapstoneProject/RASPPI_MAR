import os
import ssl
import urllib.request
import json
from pathlib import Path
from urllib.parse import urlparse
from uuid import UUID
from .contracts import read_json, StepError
from .configuration import load_config


def mapping_upload_base(value):
    """Accept a BE HTTPS origin or an explicitly configured upload collection URL."""
    uri=urlparse(value)
    if uri.scheme!='https' or not uri.hostname or uri.username or uri.password or uri.query or uri.fragment:
        raise StepError('INVALID_UPLOAD_URL')
    if uri.path in ('','/'):
        return value.rstrip('/')+'/api/v1/pi/mapping-sessions'
    return value.rstrip('/')

def tls(cfg):
    context = ssl.create_default_context(cafile=cfg.get("tls_ca_file") or None)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    return context

def camera_config(mapping):
    """Validate camera settings; independent stream controls preserve MQTT thumbnails."""
    camera=mapping.setdefault('camera',{})
    for key,value in {'enabled':False,'topic':'/ascamera/camera_publisher/rgb0/image','rate_hz':.5,
                      'max_width':320,'max_height':240,'jpeg_quality':50,'max_jpeg_bytes':65536,
                      'stream_enabled':True,'stream_bind':'0.0.0.0','stream_port':8081,'stream_rate_hz':15.}.items():camera.setdefault(key,value)
    if type(camera['enabled']) is not bool or not isinstance(camera['topic'],str) or not camera['topic'].startswith('/'):
        raise StepError('INVALID_CAMERA_CONFIG')
    for key,lo,hi in (('rate_hz',.2,1),('max_width',64,640),('max_height',48,480),('jpeg_quality',20,80),('max_jpeg_bytes',4096,65536)):
        value=camera[key]
        if type(value) not in (int,float) or not lo<=value<=hi or key!='rate_hz' and type(value) is not int:
            raise StepError('INVALID_CAMERA_CONFIG',key)
    for key in ('max_width','max_height','jpeg_quality'):
        camera.setdefault('stream_'+key,camera[key])
    camera.setdefault('stream_max_frame_age_sec',2.)
    for key,lo,hi in (('stream_max_width',64,640),('stream_max_height',48,480),('stream_jpeg_quality',20,80)):
        if type(camera[key]) is not int or not lo<=camera[key]<=hi:
            raise StepError('INVALID_CAMERA_STREAM_CONFIG',key)
    age=camera['stream_max_frame_age_sec']
    if type(age) not in (int,float) or not .1<=age<=2.:
        raise StepError('INVALID_CAMERA_STREAM_CONFIG','stream_max_frame_age_sec')
    from ipaddress import IPv4Address
    try:
        if not isinstance(camera['stream_bind'], str):
            raise ValueError('IPv4 string required')
        IPv4Address(camera['stream_bind'])
    except (ValueError, TypeError):
        raise StepError('INVALID_CAMERA_STREAM_BIND') from None
    if (type(camera['stream_enabled']) is not bool or type(camera['stream_port']) is not int
        or not 1024 <= camera['stream_port'] <= 65535
        or type(camera['stream_rate_hz']) not in (int,float) or not 1 <= camera['stream_rate_hz'] <= 30):
        raise StepError('INVALID_CAMERA_STREAM_CONFIG')
    return camera


def load(path):
    cfg = read_json(path)
    UUID(cfg["robot_id"])
    if cfg.get("transport") != "mqtt5":
        raise StepError("MQTT5_CONFIG_REQUIRED")
    m=cfg["mqtt"]
    if not isinstance(m.get("host"),str) or not m["host"] or any(x in m["host"] for x in ("/", " ", "@")):
        raise StepError("INVALID_MQTT_HOST")
    if type(m.get("port")) is not int or not 1 <= m["port"] <= 65535:
        raise StepError("INVALID_MQTT_PORT")
    if m.get("username") != cfg["robot_id"]:
        raise StepError("MQTT_USERNAME_MUST_EQUAL_ROBOT_UUID")
    if not Path(m["password_file"]).is_absolute() or not Path(m["password_file"]).is_file():
        raise StepError("MQTT_PASSWORD_FILE_REQUIRED")
    if type(m.get("be_timeout_sec")) not in (int,float) or not 5 <= m["be_timeout_sec"] <= 30:
        raise StepError("INVALID_BE_TIMEOUT")
    m.setdefault("max_publish_per_sec",8)
    if type(m["max_publish_per_sec"]) is not int or not 1 <= m["max_publish_per_sec"] <= 8:
        raise StepError("INVALID_MQTT_PUBLISH_RATE")
    for key in ("hardware_profile","state_path","legacy_state_path"):
        p = Path(cfg[key]).expanduser()
        if not p.is_absolute():
            raise StepError("ABSOLUTE_PATH_REQUIRED",key)
        cfg[key] = str(p)
    if cfg.get("tls_ca_file") and not Path(cfg["tls_ca_file"]).is_file():
        raise StepError("TLS_CA_FILE_NOT_FOUND")
    hardware = load_config(cfg["hardware_profile"])
    hardware["robot_id"] = cfg["robot_id"]
    cfg.setdefault("heartbeat_sec", 3)
    cfg.setdefault("ack_timeout_sec", 15)
    cfg.setdefault("connect_timeout_sec", 10)
    for key, lo, hi in (("heartbeat_sec",1,10),("ack_timeout_sec",3,60),("connect_timeout_sec",3,60)):
        if type(cfg[key]) not in (int,float) or not lo <= cfg[key] <= hi:
            raise StepError("INVALID_CONFIG",key)
    if cfg.get("unlock_policy") not in ("DISABLED","COMMISSIONING_ONLY"):
        raise StepError("INVALID_UNLOCK_POLICY")
    if not isinstance(cfg.get("compartments"),dict) or any(v!="cargo_lock" for v in cfg["compartments"].values()):
        raise StepError("INVALID_COMPARTMENT_MAPPING")
    if (not isinstance(cfg.get("enabled_step_types"),list) or
        not set(cfg["enabled_step_types"]) <= {"MOVE","CONTROL_HARDWARE","EXECUTE_SENSING","WAIT","INTERACT"}):
        raise StepError("INVALID_ENABLED_STEPS")
    speech=cfg.setdefault("speech", {"enabled": False, "device": "auto"})
    if (not isinstance(speech,dict) or type(speech.get("enabled")) is not bool
        or not isinstance(speech.get("device"),str) or not speech["device"]
        or len(speech["device"])>160 or speech["device"].startswith("-")):
        raise StepError("INVALID_SPEECH_CONFIG")
    if speech.get('engine') == 'fpt-be': speech['engine'] = 'be-tts'
    if speech.get('engine', 'espeak-ng') not in ('espeak-ng', 'be-tts'):
        raise StepError('INVALID_SPEECH_ENGINE')
    if speech.get('engine') == 'be-tts':
        from .cloud_speech import validate_cloud_config
        validate_cloud_config(speech)
    ai = cfg.get("ai")
    if not isinstance(ai,dict) or type(ai.get("enabled")) is not bool or not isinstance(ai.get("topic"),str):
        raise StepError("INVALID_AI_CONFIG")
    if type(ai.get("max_age_sec")) not in (int,float) or not 0.1 <= ai["max_age_sec"] <= 10:
        raise StepError("INVALID_AI_CONFIG")
    mapping=cfg["mapping"]
    for key,value in {'fresh_session':True,'reset_service':'/mediassist/slam/new_session','slam_resolution_m':.025,
        'slam_min_travel_m':.05,'slam_min_heading_rad':.10,'slam_min_interval_sec':.15,'slam_map_interval_sec':.5}.items():mapping.setdefault(key,value)
    if type(mapping['fresh_session']) is not bool or not isinstance(mapping['reset_service'],str) or not mapping['reset_service'].startswith('/'):
        raise StepError('INVALID_SLAM_SESSION_CONFIG')
    for key,low,high in (('slam_resolution_m',.01,.1),('slam_min_travel_m',.02,.5),('slam_min_heading_rad',.03,.5),
                         ('slam_min_interval_sec',.05,1),('slam_map_interval_sec',.2,5)):
        value=mapping[key]
        if type(value) not in (int,float) or not low<=value<=high:raise StepError('INVALID_SLAM_SESSION_CONFIG',key)
    mapping.setdefault('control_source','local')
    if mapping['control_source'] not in ('local','mqtt'):raise StepError('INVALID_MAPPING_CONTROL_SOURCE')
    camera_config(mapping)
    mapping.setdefault("max_preview_rate_hz",0.2)
    mapping.setdefault("max_preview_bytes",131072)
    if type(mapping["max_preview_rate_hz"]) not in (int,float) or not 0.2 <= mapping["max_preview_rate_hz"] <= 1:
        raise StepError("INVALID_PREVIEW_RATE")
    if type(mapping["max_preview_bytes"]) is not int or not 1024 <= mapping["max_preview_bytes"] <= 850000:
        raise StepError("INVALID_PREVIEW_LIMIT")
    if type(mapping.get("enabled")) is not bool or not Path(mapping["storage_dir"]).is_absolute():
        raise StepError("INVALID_MAPPING_CONFIG")
    if not isinstance(mapping.get("launch_argv"),list) or any(not isinstance(x,str) or not x for x in mapping["launch_argv"]):
        raise StepError("INVALID_SLAM_LAUNCH_ARGV")
    if not isinstance(mapping.get("map_topic"),str) or not mapping["map_topic"].startswith("/"):
        raise StepError("INVALID_MAP_TOPIC")
    for key,lo,hi in (("max_cells",1,4000000),("start_timeout_sec",5,120),("max_map_age_sec",1,30)):
        if type(mapping.get(key)) not in (int,float) or not lo <= mapping[key] <= hi:
            raise StepError("INVALID_MAPPING_CONFIG",key)
    if mapping.get("upload_base_url"):
        mapping['upload_base_url']=mapping_upload_base(mapping['upload_base_url'])
        if not Path(mapping["upload_token_file"]).is_file():
            raise StepError("UPLOAD_TOKEN_FILE_REQUIRED")
        if mapping.get("upload_ca_file") and not Path(mapping["upload_ca_file"]).is_file():
            raise StepError("UPLOAD_CA_FILE_NOT_FOUND")
    if mapping.get('device_secret_file'):
        secret=Path(mapping['device_secret_file'])
        if not secret.is_absolute() or not secret.is_file() or not mapping.get('robot_code') or not mapping.get('upload_base_url'):
            raise StepError('INVALID_MAP_DEVICE_AUTH')
    from .localization import configuration as localization_configuration
    cfg['localization']=localization_configuration(cfg.get('localization',{}))
    if cfg['localization']['enabled'] and not Path(cfg['localization'].get('status_file','')).is_absolute():
        raise StepError('LOCALIZATION_STATUS_PATH_REQUIRED')
    return cfg, hardware

class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None

def device_token(cfg):
    if not cfg.get("token_url"):
        token = os.environ.get("MEDIASSIST_ROBOT_TOKEN","")
        if len(token) < 16:
            raise StepError("DEVICE_TOKEN_REQUIRED")
        return token
    secret_file = Path(cfg["device_secret_file"])
    secret = secret_file.read_text(encoding="utf-8").strip()
    if not secret or not cfg.get("robot_code"):
        raise StepError("DEVICE_CREDENTIALS_REQUIRED")
    body = json.dumps({"robotCode":cfg["robot_code"],"deviceSecret":secret}).encode()
    request = urllib.request.Request(cfg["token_url"],data=body,
                    headers={"Content-Type":"application/json"},method="POST")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect(),
                                        urllib.request.HTTPSHandler(context=tls(cfg)))
    with opener.open(request,timeout=cfg["connect_timeout_sec"]) as response:
        raw = response.read(65537)
    if len(raw)>65536:
        raise StepError("TOKEN_RESPONSE_TOO_LARGE")
    data=json.loads(raw)
    if data.get("robotId") != cfg["robot_id"] or data.get("tokenType","").lower() != "bearer":
        raise StepError("TOKEN_IDENTITY_MISMATCH")
    token=data.get("accessToken","")
    if not isinstance(token,str) or len(token)<16:
        raise StepError("INVALID_DEVICE_TOKEN")
    return token
