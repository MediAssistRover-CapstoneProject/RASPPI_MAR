"""Validate and install a BE map bundle; no ROS node or movement command is started."""
import hashlib
import io
import json
from pathlib import Path
import re
from uuid import UUID
from zipfile import ZipFile
import yaml
from .contracts import StepError

def verify_bundle(raw, map_id):
    UUID(map_id)
    if len(raw)>6*1024*1024:raise StepError('MAP_BUNDLE_TOO_LARGE')
    try:
        with ZipFile(io.BytesIO(raw)) as z:
            if len(z.infolist())!=3 or set(z.namelist())!={'map.yaml','map.pgm','metadata.json'}:
                raise StepError('INVALID_MAP_BUNDLE_FILES')
            if any(i.file_size>4_100_000 or i.compress_size>6*1024*1024 for i in z.infolist()):
                raise StepError('MAP_BUNDLE_TOO_LARGE')
            files={n:z.read(n) for n in z.namelist()}
        if len(files['map.yaml'])>65536 or len(files['metadata.json'])>65536:raise StepError('MAP_BUNDLE_TOO_LARGE')
        meta=json.loads(files['metadata.json'])
        actual=hashlib.sha256(files['map.yaml']+b'\x00'+files['map.pgm']).hexdigest()
        if meta.get('map_id')!=map_id or meta.get('asset_hash')!=actual:raise StepError('MAP_BUNDLE_HASH_MISMATCH')
        cfg=yaml.safe_load(files['map.yaml'])
        if not isinstance(cfg,dict) or cfg.get('image')!='map.pgm':raise StepError('INVALID_MAP_IMAGE_PATH')
        if not isinstance(cfg.get('resolution'),(int,float)) or cfg['resolution']<=0:raise StepError('INVALID_MAP_RESOLUTION')
        if abs(cfg['resolution']-meta['resolution_m_per_pixel'])>1e-9:raise StepError('MAP_BUNDLE_GEOMETRY_MISMATCH')
        if not isinstance(meta.get('frame_id'),str) or not meta['frame_id']:raise StepError('INVALID_MAP_FRAME')
        return files,meta
    except StepError:raise
    except Exception:raise StepError('INVALID_MAP_BUNDLE') from None
