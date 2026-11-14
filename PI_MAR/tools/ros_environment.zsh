# Source inside MentorPi for every project process and diagnostic terminal.
# Keep discovery separate from stale/vendor ROS graphs. MQTT/HTTPS are unchanged.
export ROS_DOMAIN_ID="${MAR_ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${MAR_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY=1
# This robot uses the HP60C RGB topic consumed by MAR, QR, and WebRTC.
export DEPTH_CAMERA_TYPE="${MAR_DEPTH_CAMERA_TYPE:-ascamera}"
# Restore the MS200 used by the measured MAR geometry and /scan_raw profile.
export LIDAR_TYPE="${MAR_LIDAR_TYPE:-MS200}"
# ALSA capture/playback helpers inherit RAM-backed temporary storage.
mar_audio_tmp="/dev/shm/mar-audio-${UID}"
if [[ -L "$mar_audio_tmp" ]] || [[ -e "$mar_audio_tmp" && ( ! -d "$mar_audio_tmp" || ! -O "$mar_audio_tmp" ) ]]; then
    print -u2 'AUDIO_TEMP_OWNER_MISMATCH'
    return 1
fi
mkdir -p -m 700 -- "$mar_audio_tmp" || return 1
chmod 700 -- "$mar_audio_tmp" || return 1
export TMPDIR="$mar_audio_tmp"
unset mar_audio_tmp
if [[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]]; then
    # Do not inherit the vendor shell's missing /etc/cyclonedds/config.xml.
    # Reserve enough distinct localhost participants for the entire ROS stack.
    mar_ros_project_root="${${(%):-%x}:A:h:h}"
    export CYCLONEDDS_URI="${MAR_CYCLONEDDS_URI:-file://$mar_ros_project_root/pi/config/cyclonedds.xml}"
    unset mar_ros_project_root
fi
if [[ "$ROS_DOMAIN_ID" != <-> ]] || (( ROS_DOMAIN_ID < 0 || ROS_DOMAIN_ID > 101 )); then
    print -u2 'INVALID_PROJECT_ROS_DOMAIN: use 0..101'
    return 1
fi
if [[ ! -r "/opt/ros/humble/lib/lib${RMW_IMPLEMENTATION}.so" ]]; then
    print -u2 "PROJECT_RMW_NOT_INSTALLED: $RMW_IMPLEMENTATION; no robot process started"
    return 1
fi
