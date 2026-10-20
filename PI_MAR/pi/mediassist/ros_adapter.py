"""ROS2 Humble adapter for the inspected MentorPi source. Commissioning required."""
import asyncio
import json
import math
import threading
import time
from datetime import datetime, timezone
from .contracts import StepError
from .store import utcnow
from .adapters import validate_remote_result


class RosAdapter:
    simulated = False

    def __init__(self, config, remote):
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from nav2_msgs.action import NavigateToPose, ComputePathToPose
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Range, LaserScan
        from std_msgs.msg import UInt16, String
        from ros_robot_controller_msgs.msg import SetPWMServoState
        from rcl_interfaces.srv import GetParameters
        from tf2_ros import Buffer, TransformListener
        self.rclpy, self.cfg, self.remote = rclpy, config, remote
        from .configuration import validate_config
        validate_config(config)
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node("mediassist_runtime")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.nav = ActionClient(self.node, NavigateToPose, config["ros"]["navigate_action"])
        self.planner = ActionClient(self.node, ComputePathToPose, config["ros"]["compute_path_action"])
        self.plan_goal, self.motion_uncertain, self.event_sink = None, False, None
        self.map_client = self.node.create_client(GetParameters, "/map_server/get_parameters")
        self.pwm = self.node.create_publisher(SetPWMServoState, config["ros"]["pwm"], 10)
        self.lease = self.node.create_publisher(String, config["ros"]["lease"], 10)
        self.lock = threading.Lock()
        self.samples, self.goal, self.lease_speed, self.goal_feedback = {}, None, None, None
        for side in ("left", "right"):
            self.node.create_subscription(Range, config["ros"][side + "_range"], lambda m, s=side: self._sample(s, float(m.range)), qos_profile_sensor_data)
        self.node.create_subscription(UInt16, config["ros"]["battery"], lambda m: self._sample("battery", int(m.data)), qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, config["ros"]["scan"], lambda m: self._sample("scan", bool(m.ranges)), qos_profile_sensor_data)
        self.node.create_subscription(Odometry, config["ros"]["odom"], lambda m: self._sample("odom_speed", max(abs(m.twist.twist.linear.x), abs(m.twist.twist.linear.y), abs(m.twist.twist.angular.z))), qos_profile_sensor_data)
        self.node.create_subscription(String, "/mediassist/safety_state", lambda m: self._sample("gate", m.data), 10)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self.rclpy.spin(self.node)
        except ExternalShutdownException:
            pass
        except Exception:
            import logging
            self.motion_uncertain = True
            logging.exception("ROS executor stopped unexpectedly")

    def _sample(self, key, value):
        with self.lock: self.samples[key] = (value, time.monotonic())

    def telemetry(self):
        with self.lock: snapshot = dict(self.samples)
        now = time.monotonic()
        data = {key: {"value": v if not isinstance(v, float) or math.isfinite(v) else None,
                      "age_sec": max(0, now - at)} for key, (v, at) in snapshot.items()}
        try:
            from rclpy.time import Time
            transform = self.tf_buffer.lookup_transform(self.cfg["ros"]["map_frame"], self.cfg["ros"]["base_frame"], Time())
            t, q = transform.transform.translation, transform.transform.rotation
            age = (self.node.get_clock().now().nanoseconds - Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
            data["map_pose"] = {"value": {"map_id": self.cfg["map_id"], "x_m": t.x, "y_m": t.y,
                                          "yaw_rad": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))},
                                "age_sec": max(0.0, age), "fresh": 0 <= age <= 1.0}
        except Exception:
            data["map_pose"] = {"value": None, "age_sec": None, "fresh": False}
        return data

    def check_capabilities(self, wf):
        if wf["code"] not in self.cfg["allowed_workflows"]: raise StepError("WORKFLOW_NOT_COMMISSIONED")
        missing = set(wf["required_capabilities"]) - set(self.cfg["capabilities"])
        if missing: raise StepError("CAPABILITY_MISSING", ",".join(sorted(missing)))
        if "nav2" in wf["required_capabilities"]:
            if not all(self.cfg["motion"][x] for x in ("commissioned", "cart_footprint_validated", "watchdog_validated")):
                raise StepError("MOTION_NOT_COMMISSIONED")
        if "cargo_lock" in wf["required_capabilities"] and not self.cfg["servo"]["commissioned"]:
            raise StepError("SERVO_NOT_COMMISSIONED")

    async def future(self, future, timeout=5):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline: raise StepError("ROS_RESPONSE_TIMEOUT")
            await asyncio.sleep(0.02)
        return future.result()

    def lease_message(self, enabled, speed=0.0):
        from std_msgs.msg import String
        if not self.rclpy.ok(context=self.node.context):
            if enabled:
                raise StepError("ROS_CONTEXT_UNAVAILABLE")
            return False
        self.lease.publish(String(data=json.dumps({"enabled": enabled, "max_speed_mps": speed})))
        return True

    async def stop(self):
        self.lease_speed = None
        if not self.lease_message(False):
            self.motion_uncertain = True
            return False
        confirmed = True
        for attr in ("goal", "plan_goal"):
            handle = getattr(self, attr)
            if handle is None or not handle.accepted:
                setattr(self, attr, None)
                continue
            try:
                result = handle.get_result_async()
                if not result.done(): await self.future(handle.cancel_goal_async(), timeout=2)
                terminal = await self.future(result, timeout=3)
                if terminal.status not in (4, 5, 6): raise StepError("ACTION_NOT_TERMINAL")
                setattr(self, attr, None)
            except Exception:
                confirmed = False
        if not confirmed: self.motion_uncertain = True
        return confirmed

    def _late_cancel(self, future):
        try:
            handle = future.result()
            if handle.accepted: handle.cancel_goal_async()
        except Exception:
            pass
        # The acceptance/cancel exchange was not observed completely. Require reconciliation.
        self.motion_uncertain = True

    async def _check_map(self, map_id):
        from rcl_interfaces.srv import GetParameters
        from .map_tools import map_fingerprint
        if map_id != self.cfg["map_id"] or not self.cfg["map_asset_sha256"]:
            raise StepError("MAP_NOT_COMMISSIONED")
        if not self.map_client.service_is_ready(): raise StepError("MAP_SERVER_UNAVAILABLE")
        request = GetParameters.Request(names=["yaml_filename"])
        response = await self.future(self.map_client.call_async(request))
        if not response.values or map_fingerprint(response.values[0].string_value) != self.cfg["map_asset_sha256"]:
            raise StepError("MAP_ASSET_MISMATCH")

    def _check_graph(self):
        from .velocity_graph import check_velocity_graph
        check_velocity_graph(self.node, self.cfg['ros'])

    async def _prerequisites(self, p):
        if self.motion_uncertain: raise StepError("GOAL_STOP_UNCONFIRMED")
        if not all(self.cfg["motion"][k] for k in ("commissioned", "cart_footprint_validated", "watchdog_validated")):
            raise StepError("MOTION_NOT_COMMISSIONED")
        if p["max_speed_mps"] > self.cfg["max_speed_mps"]: raise StepError("SPEED_POLICY_EXCEEDED")
        self._check_graph()
        await self._check_map(p["map_id"])
        if not self.health()["sensors_ready"]: raise StepError("SENSORS_NOT_READY")
        if not self.telemetry()["map_pose"]["fresh"]: raise StepError("MAP_TF_NOT_READY")

    def _pose(self, endpoint_id):
        from geometry_msgs.msg import PoseStamped
        endpoint = self.cfg["endpoints"][endpoint_id]
        pose = PoseStamped()
        pose.header.frame_id = self.cfg["ros"]["map_frame"]
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(endpoint["x_m"]), float(endpoint["y_m"])
        pose.pose.orientation.z = math.sin(endpoint["yaw_rad"] / 2)
        pose.pose.orientation.w = math.cos(endpoint["yaw_rad"] / 2)
        return pose

    async def _plan(self, p, endpoint_id):
        from nav2_msgs.action import ComputePathToPose
        await self._prerequisites(p)
        if not self.planner.server_is_ready(): raise StepError("PLANNER_UNAVAILABLE")
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(endpoint_id)
        goal.use_start, goal.planner_id = False, self.cfg["navigation"]["planner_id"]
        sent = self.planner.send_goal_async(goal)
        received = False
        try:
            self.plan_goal = await self.future(sent)
            received = True
            if not self.plan_goal.accepted: raise StepError("PLAN_GOAL_REJECTED")
            result = await self.future(self.plan_goal.get_result_async(), self.cfg["navigation"]["planning_timeout_sec"])
            self.plan_goal = None
            # Humble has no detailed planner error code. Call this PATH_UNAVAILABLE, not BLOCKED.
            if result.status == 6 or (result.status == 4 and not result.result.path.poses):
                raise StepError("PATH_UNAVAILABLE")
            if result.status != 4: raise StepError("PLAN_CANCELLED")
        finally:
            if not received: sent.add_done_callback(self._late_cancel)
            if not await self.stop(): raise StepError("GOAL_STOP_UNCONFIRMED")

    async def _move_one(self, p, endpoint_id):
        from nav2_msgs.action import NavigateToPose
        await self._prerequisites(p)
        if not self.nav.server_is_ready(): raise StepError("NAV2_UNAVAILABLE")
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(endpoint_id)
        from pathlib import Path
        goal.behavior_tree = str((Path(__file__).resolve().parents[1] / "config/navigate_no_recovery.xml").resolve(strict=True))
        self.goal_feedback = None
        sent = self.nav.send_goal_async(goal, feedback_callback=lambda m: setattr(self, "goal_feedback", m.feedback))
        received = False
        try:
            self.goal = await self.future(sent)
            received = True
            if not self.goal.accepted: raise StepError("NAV_GOAL_REJECTED")
            result_future = self.goal.get_result_async()
            while not result_future.done():
                self._check_graph()
                state = self.telemetry()
                if "gate" not in state or state["gate"]["age_sec"] > 0.6: raise StepError("SAFETY_GATE_LOST")
                self.lease_message(True, p["max_speed_mps"])
                await asyncio.sleep(0.1)
            status = result_future.result().status
            self.goal = None
            if status != 4: raise StepError("NAV_CANCELLED" if status == 5 else "NAV_ABORTED")
            feedback, pose = self.goal_feedback, None
            if feedback is not None:
                pos, q = feedback.current_pose.pose.position, feedback.current_pose.pose.orientation
                pose = {"x_m": pos.x, "y_m": pos.y, "yaw_rad": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
            return {"reached": True, "endpoint_id": endpoint_id, "action_status": status, "last_feedback_pose": pose,
                    "distance_remaining_m": float(feedback.distance_remaining) if feedback else None}
        finally:
            self.lease_message(False)
            if not received: sent.add_done_callback(self._late_cancel)
            if not await self.stop(): raise StepError("GOAL_STOP_UNCONFIRMED")

    async def _move(self, p, meta):
        from .navigation import select_and_move
        def report(data):
            if self.event_sink:
                self.event_sink(meta["job_id"], "ENDPOINT_ATTEMPT", {**meta, **data})
        return await select_and_move(p, self.cfg,
            lambda eid: self._plan(p, eid), lambda eid: self._move_one(p, eid), report)

    def health(self):
        data = self.telemetry()
        age = self.cfg["motion"]["sensor_max_age_sec"]
        def fresh(key): return key in data and data[key]["age_sec"] <= age
        def range_ok(key):
            v = data.get(key, {}).get("value")
            return fresh(key) and type(v) in (int, float) and 0.02 <= v <= 4.0
        return {"sensors_ready": range_ok("left") and range_ok("right") and fresh("scan") and data["scan"]["value"] is True,
                "left_m": data.get("left", {}).get("value"), "right_m": data.get("right", {}).get("value"),
                "left_fresh": range_ok("left"), "right_fresh": range_ok("right"),
                "left_age_sec": data.get("left", {}).get("age_sec"), "right_age_sec": data.get("right", {}).get("age_sec"),
                "battery_age_sec": data.get("battery", {}).get("age_sec"),
                "battery_raw": data.get("battery", {}).get("value"), "battery_percent": None, "battery_unit": "UNVERIFIED_UINT16"}

    async def execute(self, kind, p, meta):
        if kind == "MOVE": return await self._move(p, meta)
        if kind == "CONTROL_HARDWARE":
            from ros_robot_controller_msgs.msg import PWMServoState, SetPWMServoState
            if not self.cfg["servo"]["commissioned"]: raise StepError("SERVO_NOT_COMMISSIONED")
            odom = self.telemetry().get("odom_speed")
            if not odom or odom["age_sec"] > 0.5 or odom["value"] > 0.01: raise StepError("ROBOT_NOT_CONFIRMED_STATIONARY")
            if self.pwm.get_subscription_count() < 1: raise StepError("PWM_DRIVER_UNAVAILABLE")
            servo = self.cfg["servo"]
            if p["action"] not in ("LOCK", "UNLOCK"): raise StepError("INVALID_SERVO_ACTION")
            value = servo["lock_position_raw" if p["action"] == "LOCK" else "unlock_position_raw"]
            expected = 1500 if p["action"] == "LOCK" else 2400
            if type(value) is not int or value != expected or servo["ros_id"] != 4: raise StepError("INVALID_SERVO_PROFILE")
            msg = SetPWMServoState()
            item = PWMServoState()
            item.id, item.position, item.offset = [4], [value], []
            msg.state, msg.duration = [item], float(servo["duration_sec"])
            self.pwm.publish(msg)
            await asyncio.sleep(msg.duration)
            return {"published": True, "controller_ack": False, "verified": False, "commanded_state": p["action"], "position_raw": value, "ros_servo_id": 4}
        if kind == "EXECUTE_SENSING" and p["task_type"] in ("ROBOT_HEALTH", "SIDE_CLEARANCE"):
            return {"task_type": p["task_type"], "metrics": self.health(), "captured_at": utcnow(), "evidence_ref": None}
        data = await self.remote.call(kind, p, meta)
        return validate_remote_result(kind, p, meta, data)

    def close(self):
        if getattr(self, "closed", False):
            return
        self.closed = True
        if self.rclpy.ok(context=self.node.context):
            self.rclpy.shutdown(context=self.node.context)
        self.thread.join(timeout=3)
        if not self.thread.is_alive():
            self.node.destroy_node()
