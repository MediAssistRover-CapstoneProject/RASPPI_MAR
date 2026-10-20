"""Publish measured US100 fixed transforms; no actuator commands."""
import argparse
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.contracts import read_json


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, required=True)
    args = p.parse_args()
    cfg = read_json(args.config)
    if cfg.get("measured") is not True: raise SystemExit("Fill measured transforms and set measured=true first.")
    for side in ("left", "right"):
        for field in ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"):
            v = cfg[side][field]
            if type(v) not in (int, float) or not math.isfinite(v): raise SystemExit("Invalid transform: " + field)
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    rclpy.init()
    node = rclpy.create_node("mediassist_sensor_mounts")
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for side in ("left", "right"):
        v = cfg[side]
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = node.get_clock().now().to_msg(), cfg["parent_frame"], "us100_" + side
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(v["x_m"]), float(v["y_m"]), float(v["z_m"])
        cr, sr = math.cos(v["roll_rad"]/2), math.sin(v["roll_rad"]/2)
        cp, sp = math.cos(v["pitch_rad"]/2), math.sin(v["pitch_rad"]/2)
        cy, sy = math.cos(v["yaw_rad"]/2), math.sin(v["yaw_rad"]/2)
        q = t.transform.rotation
        q.x, q.y, q.z, q.w = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
        transforms.append(t)
    broadcaster.sendTransform(transforms)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__": main()
