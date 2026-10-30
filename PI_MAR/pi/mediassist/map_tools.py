import hashlib
from pathlib import Path
import yaml


def map_fingerprint(path):
    path = Path(path).resolve(strict=True)
    raw = path.read_bytes()
    config = yaml.safe_load(raw)
    image_path = (path.parent / config["image"]).resolve(strict=True)
    return hashlib.sha256(raw + b"\x00" + image_path.read_bytes()).hexdigest()


def load_trinary_grid(path):
    """Read Nav2 trinary pixels in ROS bottom-left order; preserve metric geometry."""
    import math
    from PIL import Image
    path=Path(path).resolve(strict=True);cfg=yaml.safe_load(path.read_bytes())
    if cfg.get('mode','trinary')!='trinary':raise ValueError('EXPECTED_TRINARY_MAP')
    res=float(cfg['resolution']);origin=list(cfg['origin'])
    if res<=0 or len(origin)!=3 or not all(math.isfinite(v) for v in [res,*origin]):raise ValueError('INVALID_MAP_GEOMETRY')
    free=float(cfg.get('free_thresh',.196));occupied=float(cfg.get('occupied_thresh',.65))
    if not 0<=free<occupied<=1:raise ValueError('INVALID_MAP_THRESHOLDS')
    with Image.open(path.parent/cfg['image']) as im:
        w,h=im.size
        if not 0<w*h<=4_000_000:raise ValueError('MAP_SIZE_LIMIT')
        pixels=list(im.convert('L').getdata())
    negate=bool(cfg.get('negate',0));data=[]
    for y in range(h-1,-1,-1):
        for pixel in pixels[y*w:(y+1)*w]:
            value=pixel/255. if negate else (255-pixel)/255.
            data.append(100 if value>occupied else 0 if value<free else -1)
    return dict(width=w,height=h,resolution=res,origin=origin,data=data)
