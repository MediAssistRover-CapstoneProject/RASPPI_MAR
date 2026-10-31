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
  desired rotation 0.50 rad/s, bounded by the hardware yaw cap (0.80 rad/s).
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

Targeted regressions are in `pi/tools/test_move_position_only.py` and
`pi/native/navigation_bt/test_commit_navigation_path.cpp`. These do not replace
a supervised physical motion test. Build the native plugin after source install:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
bash tools/build_navigation_bt.sh
```
