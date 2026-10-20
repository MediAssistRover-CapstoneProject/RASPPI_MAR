"""Fail-closed watchdog policy, independent of the main velocity gate process."""


def stop_required(now, last_gate, moving, last_safe, last_odom):
    # A healthy gate publishing nonzero commands also needs live chassis/odometry.
    return (now - last_gate > 0.5 or now - last_safe > 0.5
            or (moving and now - last_odom > 0.5))
