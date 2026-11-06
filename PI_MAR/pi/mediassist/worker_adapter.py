"""Local execution boundary. No business workflow or staff identity logic."""
import asyncio
import json
import logging
import math
import threading
import time
from datetime import datetime, timezone
from .contracts import StepError, parse_timestamp
from .ros_adapter import RosAdapter
from .store import utcnow

class NoRemote:
    async def call(self, *args):
        raise StepError("SERVER_STEP_REQUIRED")

class WorkerAdapter(RosAdapter):
    def __init__(self, hardware, worker):
        self.worker = worker
        self.localization_required=worker.get('localization',{}).get('enabled',False) and not worker['mapping']['enabled']
        if not worker['mapping']['enabled'] and 'MOVE' in worker.get('enabled_step_types',[]) and not self.localization_required:
            raise StepError('CONFIGURE_LOCALIZATION_FIRST')
        self.ai_observations = {}
        self.ai_lock = threading.Lock()
        self.stop_lock = asyncio.Lock()
        self.closed = False
        super().__init__(hardware, NoRemote())
        self.staff_unlock_ready = False
        self.pwm_read_client = None
        from .speech import SpeechPlayer
        self.speech = SpeechPlayer(worker.get("speech", {}))
        self.mapping = None
        if worker["mapping"]["enabled"]:
            from .mapping import MappingManager
            self.mapping = MappingManager(self,worker["mapping"])
        from std_msgs.msg import String
        if worker["ai"]["enabled"]:
            self.node.create_subscription(String, worker["ai"]["topic"], self._ai_sample, 10)

    def _ai_sample(self, msg):
        try:
            if len(msg.data) > 65536:
                return
            data = json.loads(msg.data, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            if not isinstance(data, dict) or data.get("sensing_type") not in ("CROWD_DENSITY","RESTRICTED_ZONE"):
                return
            if data.get("map_id") != self.cfg["map_id"] or not data.get("zone_id") or not data.get("model_version"):
                return
            metrics = data.get("metrics")
            if not isinstance(metrics, dict):
                return
            if data["sensing_type"] == "CROWD_DENSITY":
                if type(metrics.get("count")) is not int or metrics["count"] < 0:
                    return
            elif type(metrics.get("intrusion_detected")) is not bool:
                return
            captured = parse_timestamp(data["captured_at"])
            if captured.tzinfo is None:
                return
            age = (datetime.now(timezone.utc) - captured).total_seconds()
            if not 0 <= age <= self.worker["ai"]["max_age_sec"]:
                return
            with self.ai_lock:
                if len(self.ai_observations) >= 256:
                    oldest = min(self.ai_observations, key=lambda k:self.ai_observations[k][1])
                    self.ai_observations.pop(oldest, None)
                self.ai_observations[(data["sensing_type"],data["zone_id"])] = (data,time.monotonic())
        except (ValueError, KeyError, TypeError):
            logging.warning("Invalid AI observation ignored")

    def telemetry(self):
        data = super().telemetry()
        mapping = getattr(self, 'mapping', None)
        if mapping and mapping.active and data.get('map_pose', {}).get('value'):
            data['map_pose']['value']['map_id'] = None
            data['map_pose']['value']['mapping_session_id'] = mapping.state['session_id']
        elif mapping:
            # Idle SLAM coordinates must not be attributed to an installed
            # navigation map. START sets session_open before requesting TF.
            data['map_pose'] = {'value': None, 'age_sec': None, 'fresh': False}
        return data

    async def stop(self):
        async with self.stop_lock:
            if not self.rclpy.ok(context=self.node.context):
                self.motion_uncertain = True
                return False
            stopped = await super().stop()
            if not stopped:
                return False
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                odom = self.telemetry().get("odom_speed")
                if odom and odom["age_sec"] <= 0.5 and odom["value"] <= 0.01:
                    return True
                await asyncio.sleep(0.05)
            self.motion_uncertain = True
            return False

    def stationary_for_handover(self):
        odom = self.telemetry().get('odom_speed') or {}
        age, speed = odom.get('age_sec'), odom.get('value')
        return (type(age) in (int, float) and 0 <= age <= .5
                and type(speed) in (int, float) and 0 <= speed <= .01)

    async def probe_staff_unlock(self):
        """Advertise staff unlock only after bounded readback from the existing controller."""
        self.staff_unlock_ready = False
        if (not self.worker.get('navigation_session') or self.worker['mapping']['enabled']
                or 'CONTROL_HARDWARE' not in self.worker['enabled_step_types']
                or self.worker.get('compartments', {}).get('CART_MAIN') != 'cargo_lock'
                or not self.cfg['servo']['commissioned'] or not self.stationary_for_handover()):
            return False
        from ros_robot_controller.ros_robot_controller_node import RosRobotController
        from ros_robot_controller_msgs.srv import GetPWMServoState

        topic = self.cfg['ros']['pwm']
        if not getattr(RosRobotController, 'MAR_BOUNDED_PWM_READBACK', False) or not topic.endswith('/set_state'):
            return False
        if self.pwm_read_client is None:
            service = topic.removesuffix('/set_state') + '/get_state'
            self.pwm_read_client = self.node.create_client(GetPWMServoState, service)
        deadline = time.monotonic() + 1.2
        while not self.pwm_read_client.service_is_ready():
            if time.monotonic() >= deadline:
                return False
            await asyncio.sleep(.05)
        position = await self.read_servo_pwm()
        self.staff_unlock_ready = 500 <= position <= 2500
        return self.staff_unlock_ready

    async def read_servo_pwm(self):
        from ros_robot_controller_msgs.msg import GetPWMServoCmd
        from ros_robot_controller_msgs.srv import GetPWMServoState

        if self.pwm_read_client is None or not self.pwm_read_client.service_is_ready():
            raise StepError('PWM_READBACK_UNAVAILABLE')
        request = GetPWMServoState.Request()
        request.cmd = [GetPWMServoCmd(id=4, get_position=1, get_offset=0)]
        future = self.pwm_read_client.call_async(request)
        try:
            result = await self.future(future, timeout=1.2)
        finally:
            if not future.done():future.cancel()
        if (result is None or not result.success or len(result.state) != 1
                or list(result.state[0].id) != [4] or len(result.state[0].position) != 1):
            raise StepError('PWM_READBACK_UNAVAILABLE')
        return int(result.state[0].position[0])

    async def perform(self, step):
        kind, p = step["stepType"], step["params"]
        meta = {"job_id":step["jobId"],"job_step_id":step["jobStepId"],
                "operation_key":step["operationKey"],"step_code":step["stepCode"]}
        if kind == "INTERACT" and p.get("action") == "SPEAK":
            return await self.speech.speak(p)
        if kind == "WAIT":
            started = time.monotonic()
            await asyncio.sleep(p["duration_sec"])
            return {"completed":True,"wake_reason":"DURATION",
                    "elapsed_sec":time.monotonic()-started,"event":None}
        if kind == "MOVE":
            return await self.execute(kind,p,meta)
        if kind == "CONTROL_HARDWARE":
            action = "LOCK" if p["action"] == "LOCK_COMPARTMENT" else "UNLOCK"
            mode = 'DEVICE_COMMAND' if action == 'LOCK' else self.worker['unlock_policy']
            if action == 'UNLOCK' and mode not in ('COMMISSIONING_ONLY','STAFF_VERIFIED'):
                raise StepError('STAFF_VERIFICATION_REQUIRED')
            if mode == 'STAFF_VERIFIED':
                from .step_contract import require_current_unlock_grant
                grant = require_current_unlock_grant(p)
                if (not self.staff_unlock_ready or self.pwm_read_client is None
                        or not self.pwm_read_client.service_is_ready() or not self.stationary_for_handover()):
                    raise StepError('STAFF_UNLOCK_NOT_READY')
                meta['unlock_valid_until'] = grant['valid_until']
            data = await self.execute(kind,{"device_id":"cargo_lock","action":action},meta)
            if mode == 'STAFF_VERIFIED':
                if await self.read_servo_pwm() != data['position_raw']:
                    raise StepError('SERVO_READBACK_MISMATCH')
                data['controller_ack'] = True
            return {**data,"compartment_id":p["compartment_id"],
                "action":p["action"],"authorization_mode":mode}
        if kind == "EXECUTE_SENSING":
            task = p["sensing_type"]
            if task == "AUDIO_LOOPBACK":
                from .voice_capture import loopback
                return await loopback(self.speech,p)
            if task == "VOICE_CAPTURE":
                from .voice_capture import capture
                return await capture(self.worker,self.speech,p)
            if task == "MAPPING":
                if not self.mapping:
                    raise StepError("MAPPING_NOT_ENABLED")
                return await self.mapping.perform(step)
            if task in ("ROBOT_HEALTH","SIDE_CLEARANCE"):
                data = self.health()
                if task == "SIDE_CLEARANCE" and not (data["left_fresh"] and data["right_fresh"]):
                    raise StepError("RANGE_NOT_READY")
                return {"sensing_type":task,"metrics":data,"captured_at":utcnow(),"evidence_ref":None}
            started = time.monotonic()
            window = p.get("window_sec",0)
            while True:
                with self.ai_lock:
                    item = self.ai_observations.get((task,p["zone_id"]))
                if item:
                    data, received = item
                    captured = parse_timestamp(data["captured_at"])
                    if ((received >= started or window == 0) and time.monotonic()-received <= self.worker["ai"]["max_age_sec"]
                        and 0 <= (datetime.now(timezone.utc)-captured).total_seconds() <= self.worker["ai"]["max_age_sec"]):
                        return data
                if time.monotonic()-started >= window:
                    raise StepError("AI_DATA_UNAVAILABLE")
                await asyncio.sleep(0.05)
        raise StepError("SERVER_STEP_REQUIRED")
