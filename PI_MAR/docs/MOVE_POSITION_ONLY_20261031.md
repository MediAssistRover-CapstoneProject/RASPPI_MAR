# Saved-map MOVE: position-only tracking

`tools/navigation_session.zsh` defaults to POSITION_ONLY. One map/session can
receive successive single-stop or multi-stop Jobs; completing a MOVE does not
restart localization or close the session. A changed map or process restart
requires a new session. Keep one stack and one authoritative BE per robot.

The saved-map session selects SmacPlanner2D with final approach tangent and
Regulated Pure Pursuit with footprint collision checking. The sampled DWB
controller and its per-plan rotation shim are not used in this mode. Valid
replans replace the live path transactionally through CommitNavigationPath;
failed/empty plans cannot overwrite a good path. An unsafe or unavailable
route still stops instead of being replayed blindly.

Parameters in `pi/mediassist/navigation_setup.py`:

- Local MOVE cap: hardware `navigation.max_speed_mps`, currently 0.60 m/s.
- Forward only, no lateral movement. Mapping retains its separate holonomic
  DWB configuration, including reverse/lateral translation.
- Rotate to path before translation when heading error exceeds 0.45 rad;
  desired rotation uses `navigation.max_angular_speed_rps` (1.20 rad/s after
  `configure_vendor_yaw.py`), bounded by the hardware yaw cap (2.00 rad/s).
  There is no absolute turn-angle limit; a rigid Mecanum can rotate in place.
- Nav2 position tolerance 0.06 m; stop and verify fresh stable position within
  0.08 m. No final Spin and no endpoint-yaw requirement in POSITION_ONLY.
- Approach slowing distance 0.45 m, minimum approach setting 0.12 m/s,
  always subordinate to obstacle, zone, curvature and hardware limits.
- Verified pose contains the actual robot yaw. Optional yaw_error_rad is
  omitted when endpoint heading is not verified. No result schema change.

For explicit strict final heading, pass `--arrival-mode POSITION_THEN_SPIN`
when starting the session. That mode remains separate from position-only.

Bare-body footprint, outward-rounded from the Hiwonder URDF/mesh and nominal
212 x 171 mm drawing: front 0.109, rear 0.104, left/right 0.086 metres from
base_footprint. Nav2 padding is 0.03 m. `tools/configure_bare_geometry.py
--trailer-removed --apply` updates the stopped base configuration while
preserving credentials, speed, TLS, maps and journals. A trailer is not covered.

KEEP_OUT applies to the swept oriented body. `NavigationZones.directional_cap`
checks the commanded arc over a braking horizon, rather than restricting
motion solely because a forbidden polygon is beside/behind the robot. SPEED
zones retain their anticipatory cap. Existing overlap and unsafe rotations
remain blocked. Zone edits require a new idle profile acknowledgement before
the next MOVE. On BE's test page, activation of a saved zone is persisted
immediately; polygon/speed edits still require Save.

Files changed in a stopped stack require a new navigation launch; editing
generated files under `pi/state/navigation-sessions` does not update existing
Nav2 nodes. Do not run mapping and saved-map navigation stacks simultaneously.

## Vendor yaw and current path updates (2026-11-01)

The installed Hiwonder `peripherals/launch/joystick_control.launch.py` defaults
to `max_angular=2.0` rad/s. This is a command ceiling, not a measured physical
maximum. Configure the stopped, bare robot with:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_vendor_yaw.py --trailer-removed --manual 2.0 --move 1.2 --mapping 1.2 --apply
```

The helper changes joystick yaw, its acceleration (3 rad/s²), the hardware
yaw ceiling and final LiDAR braking profile together. MOVE yaw is separate
from exploration yaw; rigid autonomous mapping now uses 1.2 rad/s, with
2.4 rad/s² acceleration and 3.2 rad/s² deceleration. Existing scan, TF and
map-jump checks remain active. Linear speeds, geometry, credentials, TLS and maps remain
unchanged. No second vendor joystick/controller process is started.

Clearance remains 0.03 m beyond each body edge, shared through
`footprint.NAVIGATION_PADDING_M` by costmaps, endpoint validation and
KEEP_OUT swept-body validation. This is a nominal extra clearance, not a
guarantee of physical tracking accuracy; map cells/sampling may add space.
The 0.45 m inflation gradient is a soft path cost, not a 45 cm hard exclusion.
Paths still require a traversable corridor and may be rejected if none is found.

SPEED zones anticipate entry along the commanded direction. After the full
padded body clears a zone and is moving away, that zone releases its cap;
RPP receives the original per-step ceiling and accelerates through its smoother.
An overlapping/next zone, obstacle or goal approach may still keep speed low.
Zone limits are linear m/s; they do not redefine the yaw ceiling. Rules are
scoped to the saved map/hash/session; zones from an old map are not applied to
new SLAM exploration coordinates. Pi's `handoff/common.schema.json` and BE's
runtime/documented common schema must both accept manual yaw up to 2 rad/s,
otherwise mapping status/heartbeat messages are rejected.

MOVE RPP and its velocity smoother use 3.2 rad/s² angular acceleration.
When translating, small yaw corrections are preserved through the adapter
and safety gate; removing them distorted the requested path. For stationary
turns the 0.1 rad/s deadband remains. Braking cannot leave a tiny stationary
turn after translation has stopped. Full collision/turn-envelope checks remain.

Successful `/plan` replans are relayed through the existing authenticated
`/api/v1/pi/navigation-session/plan` endpoint at at most 1 Hz. They are bound
to the same map hash, initialization revision, Job, step and endpoint. Wrong
frames/targets, empty paths and invalid zone crossings are discarded. Initial
plan acknowledgement is still mandatory before movement. Later display
updates run in a separate asyncio task with HTTPS in a worker thread, so a
slow display upload does not block the 20 Hz command/lease loop. This adds
one depth-1 subscriber, no new node, process or ROS publisher. The UI shows
the latest accepted global plan; it remains a plan, not an actual driven track.

`NAV_MOTION.odom_twist` logs forward/lateral m/s and yaw rad/s separately;
the older `odom_speed` aggregate is a stop diagnostic and mixes units. Zone
command caps do not alone establish measured ground speed or tracking error.
Supervised physical testing is still needed after installing this change.

Targeted regressions are in `pi/tools/test_move_position_only.py` and
`pi/native/navigation_bt/test_commit_navigation_path.cpp`. These do not replace
a supervised physical motion test. Build the native plugin after source install:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
bash tools/build_navigation_bt.sh
```
