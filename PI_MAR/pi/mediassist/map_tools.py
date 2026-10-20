import hashlib
from pathlib import Path
import yaml


def map_fingerprint(path):
    path = Path(path).resolve(strict=True)
    raw = path.read_bytes()
    config = yaml.safe_load(raw)
    image_path = (path.parent / config["image"]).resolve(strict=True)
    return hashlib.sha256(raw + b"\x00" + image_path.read_bytes()).hexdigest()
