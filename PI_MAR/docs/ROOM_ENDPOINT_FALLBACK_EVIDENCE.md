# Evidence for room endpoint fallback

The backend owns endpoint groups, room scope and journey transitions. Pi still
executes one resolved endpoint per MOVE operation. It does not select another
endpoint or change the active goal to a group member.

After an observed navigation gate blockage exhausts the existing stopped
recovery, Pi can report `errorCode: "PATH_UNAVAILABLE"` with this existing
`STEP_ERROR.details` payload:

```json
{
  "code": "PATH_UNAVAILABLE",
  "causeCode": "NAV_GATE_BLOCKED",
  "navigationFailure": {
    "category": "OBSTACLE_BLOCKED",
    "endpointId": "resolved-endpoint-uuid",
    "recoveryExhausted": true,
    "obstacleReason": "LIDAR_BRAKING_STOP",
    "pose": { "xM": 1.0, "yM": 2.0, "yawRad": 0.3 },
    "poseFresh": true,
    "scanFresh": true,
    "tfFresh": true,
    "distanceToTargetM": 0.8
  }
}
```

`causeCode` retains the original terminal code. `obstacleReason` is limited to
`NAV_GATE_BLOCKED`, `BRAKING_OBSTACLE`, `LIDAR_BRAKING_STOP`,
`TURN_ENVELOPE_BLOCKED`, `REAR_OBSTACLE` and `LIVE_ROUTE_OBSTACLE`. Distance is an optional measured
distance to the failed endpoint, not to a proposed replacement.

Evidence requires a current matching navigation session, healthy sensors, a
fresh scan and a finite pose from fresh live map-to-base TF. A generic initial
planning failure or controller abort does not qualify. Localization, stale
commands, TF/scan, network, cancellation and timeout failures retain their own
cause and cannot inherit an earlier obstacle observation.

The worker persists the same details in its terminal journal and outbox, so
reconnect replay retains the evidence. If physical stop cannot be confirmed,
the worker reports `PHYSICAL_STOP_UNCONFIRMED`, removes qualifying evidence and
requires recovery. Backend fallback must require the matching terminal event
with `physicalStopConfirmed: true` before another operation is dispatched.

No MQTT schema extension is needed: `STEP_ERROR.details` and replayed
`COMMAND_ACK.result` already permit nested objects. HELLO publishes operation
states, not terminal result bodies.

A closed approach may stop planning before motion begins. After existing planning
and reposition recovery returns PATH_UNAVAILABLE, LIVE_ROUTE_OBSTACLE requires
a cluster of at least three fresh scan returns spanning 4 cm inside the direct
approach corridor, on floor that was free in the saved map (including a 5 cm
neighbourhood). The backend still requires the robot to be within the configured
radius of the same-room fallback endpoint. A bare planner failure, saved wall,
single scan point, lost localization or stale sensor never qualifies.

An exhausted NAV_ABORTED from the existing MOVE retry loop can also become
PATH_UNAVAILABLE only when the same fresh LIVE_ROUTE_OBSTACLE corroboration
succeeds. Its causeCode remains NAV_ABORTED. Without corroboration the original
exception is retained; generic controller aborts never authorize substitution.

For the b1/b2 demo, the configured 0.75 m radius is measured from the failure
pose to b2, not from b1 to b2. Start the blocked-goal test near b2 with a clear
route to it. A planner failure from far away does not authorize automatically
approaching b2 or completing the room at an arbitrary distant location.
