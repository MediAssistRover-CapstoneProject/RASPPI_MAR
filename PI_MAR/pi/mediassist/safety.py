"""Pure safety policy: required US100 failures block motion; optional ranges remain diagnostic."""
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def angular_deadband(yaw):
    """Suppress sub-0.1 rad/s commands; never amplify a collision-limited turn."""
    if not finite(yaw):return 0.0
    # ROS velocity smoothing can represent 0.1 as 0.09999999999999996.
    return yaw if abs(yaw)>=.1-1e-9 else 0.0


def navigation_yaw(yaw, minimum=.1):
    """Make a deliberate Nav2 turn usable by the motor gate's minimum turn speed."""
    if not finite(yaw) or abs(yaw)<.02:return 0.0
    return math.copysign(max(abs(yaw),minimum),yaw)


def gate_velocity(cmd, ranges, now, last_command, last_lease, last_scan, config, max_speed):
    if not config["commissioned"] or not config["cart_footprint_validated"] or not config["watchdog_validated"]:
        return (0.0, 0.0, 0.0), "NOT_COMMISSIONED"
    if now - last_lease > config["lease_timeout_sec"]: return (0.0, 0.0, 0.0), "NO_LEASE"
    if now - last_command > config["command_timeout_sec"]: return (0.0, 0.0, 0.0), "STALE_COMMAND"
    if now - last_scan > config["sensor_max_age_sec"]: return (0.0, 0.0, 0.0), "STALE_LIDAR"
    if config.get("us100_required", True):
        for value, timestamp in ranges:
            if not finite(value) or not 0.02 <= value <= 4.0 or now - timestamp > config["sensor_max_age_sec"]:
                return (0.0, 0.0, 0.0), "STALE_OR_INVALID_US100"
            if value < config["stop_distance_m"]: return (0.0, 0.0, 0.0), "SIDE_OBSTACLE"
    if any(not finite(x) for x in cmd) or not finite(max_speed) or max_speed <= 0:
        return (0.0, 0.0, 0.0), "INVALID_COMMAND"
    x, y, yaw = cmd
    speed = math.hypot(x, y)
    if speed > max_speed: x, y = x * max_speed / speed, y * max_speed / speed
    yaw = min(config["max_angular_speed_rps"], max(-config["max_angular_speed_rps"], yaw))
    return (x, y, angular_deadband(yaw)), "CLEAR"
