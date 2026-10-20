"""Create a separate hardware profile mapped to an already verified BE map."""
import argparse
import json
from pathlib import Path
from uuid import UUID

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--hardware-profile",type=Path,required=True)
    p.add_argument("--manifest",type=Path,required=True,help="JSON from BE catalog/manifest")
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--confirm-same-map",action="store_true")
    args=p.parse_args()
    if not args.confirm_same_map:
        p.error("Confirm that BE endpoints refer to the exact loaded ROS map; coordinates cannot be inferred.")
    if args.output.exists():
        p.error("Output already exists; choose a new filename.")
    cfg=json.loads(args.hardware_profile.read_text(encoding="utf-8-sig"))
    manifest=json.loads(args.manifest.read_text(encoding="utf-8-sig"))
    UUID(manifest["mapId"])
    if not cfg.get("map_asset_sha256") or manifest.get("mapHash")!=cfg["map_asset_sha256"]:
        p.error("Map hash must equal the current Pi map fingerprint. Agree the BE assetHash algorithm first.")
    points={}
    for ep in manifest["endpoints"]:
        if not ep["active"]: continue
        UUID(ep["endpointId"])
        point={"map_id":manifest["mapId"],"x_m":float(ep["xM"]),"y_m":float(ep["yM"]),"yaw_rad":float(ep["yawRad"])}
        import math
        if not all(math.isfinite(point[k]) for k in ("x_m","y_m","yaw_rad")):
            p.error("Non-finite coordinate")
        points[ep["endpointId"]]=point
    if not points: p.error("No active endpoint in manifest")
    legacy=Path(cfg["state_path"])
    if not legacy.is_absolute():
        legacy=args.hardware_profile.resolve().parent.parent/legacy
    cfg["state_path"]=str(legacy.resolve())
    cfg["map_id"]=manifest["mapId"]
    cfg["endpoints"]=points
    cfg["endpoint_groups"]={}  # BE owns group selection in MAR-STEP-1.
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(cfg,indent=2)+"\n",encoding="utf-8")
    print("Created:",args.output,"; calibration unchanged; verify map/frame before enabling MOVE.")
if __name__=="__main__": main()
