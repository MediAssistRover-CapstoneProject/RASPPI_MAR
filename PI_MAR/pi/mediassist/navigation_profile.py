"""Apply BE endpoints only to the map already installed and loaded in Nav2."""
import copy
import math
import os
import re
from pathlib import Path
from uuid import UUID
from .contracts import StepError, canonical


def validate_profile(message, robot_id, hardware, allow_empty=False):
    if message.get('messageType') != 'NAV_PROFILE' or message.get('robotId') != robot_id:
        raise StepError('INVALID_NAV_PROFILE')
    if message.get('profileVersion') != 'BE-NAV-PROFILE-1':
        raise StepError('NAV_PROFILE_VERSION_UNSUPPORTED')
    if not isinstance(message.get('profileHash'),str) or not re.fullmatch(r'[0-9a-f]{64}',message['profileHash']):
        raise StepError('INVALID_NAV_PROFILE_HASH')
    profile=message.get('profile')
    if not isinstance(profile,dict):
        raise StepError('INVALID_NAV_PROFILE')
    if profile.get('map_id') != hardware.get('map_id'):
        raise StepError('NAV_PROFILE_MAP_MISMATCH')
    if profile.get('navigation_session_id')!=hardware.get('navigation_session_id'):
        raise StepError('NAV_PROFILE_SESSION_MISMATCH')
    UUID(profile['map_id'])
    if (not re.fullmatch(r'[0-9a-f]{64}',str(profile.get('map_asset_sha256',''))) or
            profile['map_asset_sha256'] != hardware.get('map_asset_sha256')):
        raise StepError('NAV_PROFILE_MAP_HASH_MISMATCH')
    if profile.get('frame_id') != hardware['ros']['map_frame']:
        raise StepError('NAV_PROFILE_FRAME_MISMATCH')
    points=profile.get('endpoints')
    if not isinstance(points,dict) or not (0 if allow_empty else 1) <= len(points) <= 10000:
        raise StepError('NAV_PROFILE_ENDPOINTS_REQUIRED')
    for endpoint_id,point in points.items():
        UUID(endpoint_id)
        if not isinstance(point,dict) or point.get('map_id') != profile['map_id']:
            raise StepError('NAV_PROFILE_ENDPOINT_MAP_MISMATCH')
        for key in ('x_m','y_m','yaw_rad'):
            value=point.get(key)
            if type(value) not in (int,float) or not math.isfinite(value) or abs(value)>100000:
                raise StepError('NAV_PROFILE_ENDPOINT_INVALID')
    return copy.deepcopy(profile)


async def apply_profile(message, cfg, hardware, adapter):
    profile=validate_profile(message,cfg['robot_id'],hardware,allow_empty=bool(cfg.get('navigation_session')))
    # Compare the actual map_server YAML + image fingerprint, not only config text.
    await adapter._check_map(profile['map_id'])
    destination=Path(cfg['state_path']).with_suffix('.navigation.json')
    snapshot={'profileVersion':message['profileVersion'],'profileHash':message['profileHash'],
              'robotId':cfg['robot_id'],'profile':profile}
    raw=canonical(snapshot)
    if not destination.is_file() or destination.read_text(encoding='utf-8')!=raw:
        temporary=destination.with_suffix('.new')
        destination.parent.mkdir(parents=True,exist_ok=True)
        with temporary.open('w',encoding='utf-8') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary,destination)
    # BE resolves endpoint groups. The hardware calibration file is never rewritten.
    hardware['endpoints']=profile['endpoints']
    adapter.cfg['endpoints']=profile['endpoints']
    adapter.navigation_profile_hash=message['profileHash']
    return snapshot
