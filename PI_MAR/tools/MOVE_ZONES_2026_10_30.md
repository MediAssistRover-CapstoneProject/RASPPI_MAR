# MOVE, shutdown and zone patch

Installed on Pi 192.168.1.56 and BE 192.168.1.55 on 2026-10-30.

MOVE on a saved map uses forward/reverse and yaw; lateral velocity is zero in
the DWB profile, velocity smoother and MOVE output. Autonomous mapping keeps
holonomic motion. Manual USB joystick remains holonomic.

One navigation session binds a robot to one map and initialization. Multiple
endpoint jobs can share it until it closes/expires. Mapping and AMCL cannot own
the same map->odom transform at once; stop mapping before loading a saved map.
Changing the laptop used for SSH does not require a new robot registration.

On orderly SIGINT/SIGTERM, worker stops its active step, records FAILED with
PROCESS_SHUTDOWN only when the adapter confirms stop, and allows up to four
seconds for MQTT event acknowledgement before disconnecting. Missing stop proof,
power loss and unresolved hardware actions retain RECOVERY_REQUIRED. Journal
rows are kept for duplicate command protection. Previously completed steps are
never erased or rewritten by stack shutdown.

Create map-coordinate zones through POST /api/v1/zones, with ADMIN authorization:

```json
{
  "mapId": "<saved map UUID>",
  "zoneCode": "KEEP_OUT_01",
  "zoneName": "Forbidden area",
  "zoneType": "KEEP_OUT",
  "polygonJson": {"points": [[0,0], [1,0], [1,1], [0,1]]},
  "active": true,
  "publicVisible": false
}
```

For a slow area use zoneType SPEED and add speedLimitMps, e.g. 0.15. Coordinates
are metres in the saved map frame, including its origin and yaw; display rotation
does not change them. Use PUT /api/v1/zones/{id} to edit. Set up rules while idle.
The next acknowledged NAV_PROFILE carries them; an active MOVE keeps its frozen
profile. KEEP_OUT masks are transient-local OccupancyGrid + CostmapFilterInfo
messages consumed by native Nav2 KeepoutFilter in local and global costmaps.
The mask includes a conservative body radius and cell margin. SPEED rules cap
MOVE output with advance slowdown; overlapping rules use the lowest limit.
These saved-map zones are not enforced on a newly generated SLAM map whose
coordinate frame differs from the saved map.

Current nominal obstacle margin is 0.03 m from the configured body, with a 0.04 m
rear margin. Braking begins earlier according to speed, scan age and the 0.6 m/s²
deceleration assumption. This is not a physically verified 3 cm stopping accuracy.
Validate footprint, sensing coverage and stopping distance on the actual floor.
Restore the previous margin with the stack stopped:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_motion_speed.py --trailer-removed --speed .80 --reverse .80 --angular .80 --stop-margin .05 --apply
```

Verified: 20 targeted Pi tests, two additional MOVE profile/output regressions,
BE image build and 11 navigation target integration tests in an isolated test DB.
An existing localization reacquisition timing test fails; this patch does not
claim to fix AMCL localization or establish clinical deployment readiness.
No motor movement was commanded for patch verification.
