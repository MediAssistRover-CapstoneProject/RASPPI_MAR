"""Convert saved mentorpi_locations poses to a NEW site config; never moves the robot."""
import argparse
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.contracts import read_json, canonical
from mediassist.map_tools import map_fingerprint
from mediassist.configuration import validate_config


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--locations", type=Path, required=True)
    p.add_argument("--map-yaml", type=Path, required=True)
    p.add_argument("--map-id", required=True)
    p.add_argument("--base", type=Path, default=Path("config/site.example.json"))
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    data, cfg = read_json(args.locations), read_json(args.base)
    if data.get("version") != 1 or data.get("frame_id") != "map" or data.get("map_name") != args.map_yaml.stem:
        raise ValueError("Saved locations and map YAML do not identify the same map")
    cfg["map_id"], cfg["map_asset_sha256"] = args.map_id, map_fingerprint(args.map_yaml)
    cfg["endpoints"] = {}
    cfg["endpoint_groups"] = {}
    cfg["motion"]["commissioned"] = False
    for name, point in data["locations"].items():
        if any(type(point[k]) not in (int, float) or not math.isfinite(point[k]) for k in ("x", "y", "yaw_deg")):
            raise ValueError("Non-finite pose")
        cfg["endpoints"][name] = {"map_id": args.map_id, "x_m": point["x"], "y_m": point["y"], "yaw_rad": math.radians(point["yaw_deg"])}
    if not cfg["endpoints"]: raise ValueError("No endpoints recorded")
    validate_config(cfg)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as out:
        out.write(canonical(cfg) + "\n")
    print("Wrote new map profile; old groups cleared; motion commissioning reset. No motion sent.")


if __name__ == "__main__": main()
