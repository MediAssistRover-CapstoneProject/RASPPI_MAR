"""MQTT step worker. The backend owns jobs, branching, retries and staff UI."""
import asyncio
import json
import logging
import time
import uuid
from datetime import datetime, timezone, timedelta
from .mqtt_transport import MqttSession
from .contracts import StepError, canonical
from .step_contract import (COMMISSIONING_UNLOCK_CAPABILITY, CONTRACT, SUBPROTOCOL, VERSION,
                            validate_command, validate_step, validate_result, expiry)
from .worker_config import device_token, tls

class StepWorker:
    def __init__(self, cfg, hardware, store, adapter):
        self.cfg, self.hardware, self.store, self.adapter = cfg, hardware, store, adapter
        self.adapter.path_budget_store = store
        self.task, self.active, self.last_result = None, None, None
        self.connection_ready = False
        self.reconnect_delay_sec = 1
        self.sent = {}
        self.send_lock = asyncio.Lock()
        self.stop_reason = "CANCELLED"
        self.session_fault = False
        self.shutting_down = False
        self.outbox_wakeup = asyncio.Event()
        self.ack_waiters = {}
        self.staff_qr = None
        self.staff_face = None
        self.navigation_audio = {}
        self.voice_publisher=None
        self.cancel_settlement_at=0.
        from .voice_runtime import enabled,StatePublisher
        if enabled(cfg):
            try:self.voice_publisher=StatePublisher(cfg)
            except (OSError,StepError):logging.warning('VOICE_STATE_UNAVAILABLE: voice stays optional')

    def publish_voice_state(self,status=None):
        if self.voice_publisher:
            self.voice_publisher.publish(status or self.status(),
                self.connection_ready and not self.shutting_down and not self.session_fault)

    def advertised_capabilities(self):
        from .staff_qr import CAPABILITIES
        values=[value for value in self.hardware['capabilities']
                if value not in CAPABILITIES and value not in (COMMISSIONING_UNLOCK_CAPABILITY, 'PATROL_FALL', 'continuous-fall-v1', 'continuous-restricted-v1', 'staff-face-verify-v1', 'fall-observation-v1', 'group-path-budget-v1')]
        if self.cfg.get('allow_commissioning_unlock') is True:
            values.append(COMMISSIONING_UNLOCK_CAPABILITY)
        if self.cfg.get('navigation_session') and 'MOVE' in self.cfg.get('enabled_step_types',[]) and 'move-start-commit-v1' not in values:
            values.append('move-start-commit-v1')
        if self.cfg.get('navigation_session') and 'MOVE' in self.cfg.get('enabled_step_types', []):
            values.append('group-path-budget-v1')
        if self.staff_qr is not None:
            values.extend(CAPABILITIES)
        if getattr(self, 'staff_face', None) is not None:
            values.extend(['staff-face-verify-v1', 'secure-unlock-v1'])
        values=list(dict.fromkeys(values))
        observer = getattr(self.adapter, 'patrol_fall_observer', None)
        if observer is not None:
            values.append('PATROL_FALL')
            if observer.config.get('mode') == 'CONTINUOUS':
                values.append('continuous-fall-v1')
                if observer.config.get('observation_pause_enabled', True):
                    values.append('fall-observation-v1')
                if getattr(observer,'restricted_enabled',False):values.append('continuous-restricted-v1')
        return values

    def status(self):
        if self.store.blocked() or getattr(self.adapter,"motion_uncertain",False):
            return "ERROR"
        mapping=self.adapter.mapping
        if mapping and mapping.state.get("state")=="RECOVERY_REQUIRED":
            return "ERROR"
        manual = self.manual_context()
        local_busy = bool(manual and manual.manual_drive.busy(manual.gamepad))
        return "BUSY" if self.active or (mapping and mapping.active) or local_busy else "IDLE"

    def manual_context(self):
        context = self.adapter.mapping or getattr(self.adapter, 'navigation_manual', None)
        return context if context and getattr(context, 'manual_drive', None) else None

    def inhibit_manual(self):
        manual = self.manual_context()
        if not manual or getattr(manual, 'manual_inhibited', False):
            return
        manual.manual_inhibited = True
        manual.gamepad.stop('PROCESS_SHUTDOWN')
        if manual.manual_drive.owned and self.active is None:
            from geometry_msgs.msg import Twist
            manual.velocity.publish(Twist())
            self.adapter.lease_message(False)
            manual.manual_drive.reset()

    async def acknowledged(self, mid):
        deadline=time.monotonic()+self.cfg["ack_timeout_sec"]
        waiter=self.ack_waiters.setdefault(mid,asyncio.Event())
        try:
            while not self.store.is_acked(mid):
                if not self.connection_ready:raise StepError("CONNECTION_LOST")
                remaining=deadline-time.monotonic()
                if remaining<=0:raise StepError("BE_ACK_TIMEOUT")
                try:await asyncio.wait_for(waiter.wait(),remaining)
                except asyncio.TimeoutError:raise StepError("BE_ACK_TIMEOUT") from None
                waiter.clear()
        finally:self.ack_waiters.pop(mid,None)

    def receipt(self, command, accepted, step=None, **extra):
        event=self.store.emit("COMMAND_ACK",
            {"commandId":command["commandId"],"commandType":command["commandType"],
             "accepted":accepted,**extra}, step,command["correlationId"])
        self.store.remember_command(command,event)
        self.outbox_wakeup.set()
        return event

    async def command(self, command):
        validate_command(command,self.cfg["robot_id"],check_expiry=False)
        if self.store.cached_command(command):
            return
        validate_command(command,self.cfg["robot_id"])
        if self.shutting_down and command['commandType']=='EXECUTE_STEP':
            self.receipt(command,False,errorCode='PROCESS_SHUTDOWN')
            return
        if command["commandType"] == "CANCEL_STEP":
            self.store.cancel_operation(command['jobId'], command['jobStepId'])
            step=self.active
            if not step or step["jobStepId"]!=command["jobStepId"] or step["jobId"]!=command["jobId"]:
                previous=self.store.cancellation_state(command['jobId'],command['jobStepId'])
                if previous is None:
                    # A cancel can overtake an unaccepted execute. The durable tombstone prevents it starting later.
                    self.receipt(command,True,status='CANCELLED',physicalStopConfirmed=True)
                else:
                    saved,status,result=previous
                    safe=status in ('CANCELLED','SUCCEEDED','FAILED','TIMED_OUT') and result.get('physicalStopConfirmed') is True
                    self.receipt(command,safe,saved,status=status,physicalStopConfirmed=safe,
                                 **({} if safe else {'errorCode':'STOP_NOT_CONFIRMED'}))
                return
            self.stop_reason="CANCELLED"
            self.task.cancel()
            await asyncio.gather(self.task,return_exceptions=True)
            await self.abandoned("CANCELLED")
            result=self.last_result or {}
            safe=result.get("physicalStopConfirmed") is True and result.get("status")=="CANCELLED"
            self.receipt(command,safe,step,status=result.get("status","RECOVERY_REQUIRED"),
                         physicalStopConfirmed=result.get("physicalStopConfirmed",False))
            return
        step=command["step"]
        if self.store.cancelled(step['jobId'],step['jobStepId']):
            self.receipt(command,False,step,errorCode='OPERATION_CANCELLED')
            return
        if command.get("commissioningUnlock") is True:
            step = {**step, "commissioningUnlock": True}
        previous=self.store.previous(step)
        if previous:
            self.receipt(command,True,step,replayed=True,**previous)
            return
        if self.store.blocked() or getattr(self.adapter,"motion_uncertain",False) or self.session_fault:
            self.receipt(command,False,step,errorCode="RECOVERY_REQUIRED")
            return
        if self.task and not self.task.done():
            self.receipt(command,False,step,errorCode="ROBOT_BUSY")
            return
        mapping=self.adapter.mapping
        params=step['params']
        mapping_handoff=bool(mapping and step['stepType']=='EXECUTE_SENSING'
            and params.get('sensing_type')=='MAPPING' and params.get('action') in ('SAVE','STOP')
            and step['jobId']==mapping.state.get('job_id')
            and params.get('mapping_session_id')==mapping.state.get('session_id'))
        # Scoped SAVE/STOP is an explicit end-of-drive intent. Accepting it
        # inhibits manual input below; SAVE/abort must still confirm zero odometry.
        # Other operations and foreign sessions cannot take over a held stick.
        manual = self.manual_context()
        if not mapping_handoff and manual and manual.manual_drive.busy(manual.gamepad):
            self.receipt(command,False,step,errorCode="RETURN_GAMEPAD_TO_NEUTRAL")
            return
        if mapping and mapping.active:
            p=step["params"]
            if (step["jobId"]!=mapping.state.get("job_id") or
                not (step["stepType"]=="EXECUTE_SENSING" and p.get("sensing_type")=="MAPPING"
                     and p.get("action") in ("SAVE","STOP"))):
                self.receipt(command,False,step,errorCode="MAPPING_SESSION_ACTIVE")
                return
        try:
            validate_step(step,self.cfg,self.hardware)
            if step['stepType']=='MOVE':
                from .navigation_session import command_context
                # Freeze identity now; transient localization is checked before
                # any Nav2 action in the accepted operation. A transient sample
                # must finish with journalled stop evidence, not an orphaned reject.
                command_context(command,self.cfg,self.hardware,self.adapter,require_localization=False)
                self.adapter.active_navigation_context=command.get('navigationContext')
        except (StepError,ValueError) as exc:
            self.receipt(command,False,step,errorCode=getattr(exc,"code","INVALID_PARAMS"))
            return
        self.store.accept(step)
        combined=command.get('startMode')=='ACCEPT_AND_START'
        start={'attemptNo':1,'stepType':step['stepType'],'resolvedInputs':step['params']} if combined else None
        ack=self.receipt(command,True,step,status='START_REQUESTED' if combined else 'ACCEPTED',
                         **({'start':start} if combined else {}))
        self.active=step
        self.publish_voice_state('BUSY')
        if manual:
            manual.gamepad.stop('WORKFLOW_HANDOFF_RETURN_NEUTRAL')
        self.stop_reason="CANCELLED"
        self.task=asyncio.create_task(self.execute(step,command["correlationId"],ack["messageId"],command["expiresAt"],combined=combined))

    async def execute(self, step, correlation, accepted_mid, expires_at,combined=False):
        status, data, error = "FAILED", None, None
        error_details = None
        physical_started=False
        stop_confirmed=None
        try:
            # Combined mode is sent only after mutual capability negotiation.
            # Its ONE receipt ACK commits acceptance AND STARTED on the BE before
            # any physical effect. Legacy commands still require both old ACKs.
            accepted_at=time.monotonic()
            await self.acknowledged(accepted_mid)
            accepted_ack_at=time.monotonic()
            if not combined:
                started=self.store.emit("STEP_STARTED",{"attemptNo":1,"stepType":step["stepType"],
                        "resolvedInputs":step["params"]},step,correlation)
                self.outbox_wakeup.set()
                await self.acknowledged(started["messageId"])
            if expiry(expires_at) <= datetime.now(timezone.utc):
                raise StepError("COMMAND_EXPIRED_BEFORE_EXECUTION")
            logging.info('STEP_START_TIMING job=%s step=%s mode=%s acceptance_ack_ms=%d started_ack_ms=%d',
                         step.get('jobId'),step.get('jobStepId'),'ACCEPT_AND_START' if combined else 'LEGACY',
                         round((accepted_ack_at-accepted_at)*1000),round((time.monotonic()-accepted_ack_at)*1000))
            self.store.set_running(step)
            physical_started=True
            # Route guidance stays on the UI. Robot announcements belong to
            # explicit INTERACT steps before/after MOVE, including ESCORT.
            data=await asyncio.wait_for(self.adapter.perform(step),step["timeoutSec"])
            validate_result(step,data)
            status="SUCCEEDED"
        except asyncio.CancelledError:
            status=("CANCELLED" if self.stop_reason=="CANCELLED" else
                    "FAILED" if self.stop_reason=="PROCESS_SHUTDOWN" else "RECOVERY_REQUIRED")
            error=self.stop_reason
        except asyncio.TimeoutError:
            status,error="TIMED_OUT","STEP_TIMEOUT"
        except Exception as exc:
            error=getattr(exc,"code","STEP_EXECUTION_ERROR")
            # urllib HTTPError.code is an integer, unlike StepError.code.
            if not isinstance(error,str) or not error:
                error="STEP_EXECUTION_ERROR"
            if isinstance(exc,StepError) and isinstance(exc.details,dict):
                error_details=exc.details
            status="RECOVERY_REQUIRED" if error in ("BE_ACK_TIMEOUT","CONNECTION_LOST") else "FAILED"
            logging.warning("Step failed: %s (%s)",error,type(exc).__name__)
        finally:
            if status!="SUCCEEDED" and self.adapter.mapping and self.adapter.mapping.active:
                try:
                    await self.adapter.mapping.abort("ABORTED")
                except Exception:
                    self.adapter.motion_uncertain=True
            is_mapping = step['stepType']=='EXECUTE_SENSING' and step['params'].get('sensing_type')=='MAPPING'
            if step["stepType"] in ("MOVE","CONTROL_HARDWARE") or is_mapping or (status!="SUCCEEDED" and step["stepType"]!="INTERACT"):
                try:
                    stop_confirmed=(await self.adapter.stop()) is True
                except Exception:
                    stop_confirmed=False
                if not stop_confirmed:
                    status,error="RECOVERY_REQUIRED","PHYSICAL_STOP_UNCONFIRMED"
                    error_details=None
            if step["stepType"]=="CONTROL_HARDWARE" and physical_started and status!="SUCCEEDED":
                status="RECOVERY_REQUIRED"
            if (step['stepType']=='MOVE' and stop_confirmed is True and status=='RECOVERY_REQUIRED'
                    and error in ('CONNECTION_LOST','BE_ACK_TIMEOUT')):
                status='FAILED'
            if getattr(self.adapter,"motion_uncertain",False):
                status,error="RECOVERY_REQUIRED","PHYSICAL_STOP_UNCONFIRMED"
                error_details=None
            details=None if not error else error_details or {"code":error}
            result={"status":status,"result":data,"errorCode":error,
                    "details":details,
                    "physicalStopConfirmed":stop_confirmed,
                    "actuatorPositionVerified":False if step["stepType"]=="CONTROL_HARDWARE" else None}
            self.last_result=result
            self.store.finish(step,status,result)
            self.store.emit("STEP_RESULT" if status=="SUCCEEDED" else "STEP_ERROR",
                {"status":status,"result":data,"errorCode":error,"retryable":False,
                 "details":details,
                 "physicalStopConfirmed":stop_confirmed},step,correlation)
            self.outbox_wakeup.set()
            self.active=None
            self.publish_voice_state()

    async def abandoned(self, reason):
        if self.active is None:
            return
        step=self.active
        try:
            stopped=(await self.adapter.stop()) is True
        except Exception:
            stopped=False
        stopped_failure = stopped and (reason=='PROCESS_SHUTDOWN'
            or reason=='CONNECTION_LOST' and step['stepType']=='MOVE')
        status=("CANCELLED" if reason=="CANCELLED" and stopped else
                "FAILED" if stopped_failure else "RECOVERY_REQUIRED")
        result={"status":status,"errorCode":reason,"physicalStopConfirmed":stopped}
        self.last_result=result
        self.store.finish(step,status,result)
        self.active=None
        self.store.emit("STEP_ERROR",{**result,"retryable":False,"details":{"code":reason}},step)
        self.outbox_wakeup.set()

    async def shutdown(self):
        """Stop once, persist the result, and drain it while MQTT is still alive."""
        self.shutting_down=True
        self.inhibit_manual()
        self.publish_voice_state('STOPPING')
        self.stop_reason='PROCESS_SHUTDOWN'
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task,return_exceptions=True)
        await self.abandoned('PROCESS_SHUTDOWN')
        if self.adapter.mapping:
            self.adapter.mapping.manual_inhibited=True
            if self.adapter.mapping.active:
                await self.adapter.mapping.abort('ABORTED')
        deadline=time.monotonic()+4.
        while self.connection_ready and self.store.pending() and time.monotonic()<deadline:
            await asyncio.sleep(.05)

    async def disconnect(self, reason):
        self.connection_ready=False
        self.publish_voice_state('OFFLINE')
        self.stop_reason=reason
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task,return_exceptions=True)
        await self.abandoned(reason)
        if self.adapter.mapping and self.adapter.mapping.active:
            try:
                await self.adapter.mapping.abort("ABORTED")
            except Exception:
                self.adapter.motion_uncertain=True
        try:
            if getattr(self.adapter,"goal",None) is not None or getattr(self.adapter,"plan_goal",None) is not None:
                stopped=await self.adapter.stop()
                logging.info("Connection ended; goal stop confirmation=%s",stopped)
            else:
                # Active execution already performed its stop in execute()/abandoned().
                # An idle connection failure before ROS startup must not invent a running action.
                manual = self.manual_context()
                local = bool(manual and not getattr(manual, 'manual_inhibited', False)
                             and not manual.gamepad.cfg['session_required']
                             and manual.manual_drive.select(manual.gamepad)
                             and reason != 'PROCESS_SHUTDOWN')
                if not local:self.adapter.lease_message(False)
                logging.info("Connection ended; no active worker goal")
        except Exception as exc:
            self.adapter.motion_uncertain=True
            logging.warning("Stop unavailable: %s",type(exc).__name__)

    async def publish(self, send):
        last_heartbeat=0
        while True:
            self.outbox_wakeup.clear()
            now=time.monotonic()
            if now-last_heartbeat>=self.cfg["heartbeat_sec"]:
                status=self.status()
                self.publish_voice_state(status)
                await send(self.store.transient("HEARTBEAT",{
                    "status":status,"runtimeVersion":VERSION,"protocolMajor":1,
                    "capabilities":self.advertised_capabilities(),
                    "activeJobStepId":self.active["jobStepId"] if self.active else None,
                    "currentMapId":None if self.adapter.mapping else self.hardware["map_id"],
                    "mapAssetHash":None if self.adapter.mapping else self.hardware["map_asset_sha256"],
                    "mappingSession":self.adapter.mapping.state if self.adapter.mapping else None,
                    **({'navigationSession':self.cfg['navigation_session']} if self.cfg.get('navigation_session') else {}),
                    "telemetry":self.adapter.telemetry()}))
                last_heartbeat=now
            # Cloud business ACKs can take 5-6 s. Avoid replaying the same
            # durable event during that normal commit window (ACK timeout 15 s).
            # Bound each replay batch so heartbeat cannot be starved by old events.
            count=0
            for event in self.store.pending():
                mid=event["messageId"]
                if mid not in self.sent or time.monotonic()-self.sent[mid]>=8:
                    self.sent[mid]=time.monotonic()
                    # Register before awaiting MQTT PUBACK: BE EVENT_ACK can arrive first.
                    await send(event)
                    count+=1
                    # A slow PUBACK must not postpone the next heartbeat behind
                    # the remaining replay batch. No change to protocol timeouts.
                    if count>=4 or time.monotonic()-last_heartbeat>=self.cfg['heartbeat_sec']:
                        break
            # A new acceptance/start/result wakes this loop immediately. Heartbeat
            # and durable retransmission keep their existing independent deadlines.
            try:await asyncio.wait_for(self.outbox_wakeup.wait(),.2)
            except asyncio.TimeoutError:pass

    async def navigation_profile(self, message, session):
        from .navigation_profile import apply_profile
        accepted,error=False,None
        try:
            if self.status()!='IDLE':
                raise StepError('ROBOT_BUSY')
            if 'MOVE' not in self.cfg['enabled_step_types']:
                raise StepError('MOVE_NOT_ENABLED')
            await apply_profile(message,self.cfg,self.hardware,self.adapter)
            accepted=True
            logging.info('Navigation endpoints synchronized: %s',message['profileHash'])
        except (StepError,ValueError,TypeError,KeyError,OSError) as exc:
            error=getattr(exc,'code','INVALID_NAV_PROFILE')
            logging.warning('Navigation profile rejected: %s',error)
        await session.publish('/profile-acks',{
            'messageType':'NAV_PROFILE_ACK','robotId':self.cfg['robot_id'],
            'transportSessionId':session.id,'producerSessionId':self.store.session,
            'profileHash':message.get('profileHash'), 'accepted':accepted,
            'mapId':self.hardware.get('map_id'),'mapAssetHash':self.hardware.get('map_asset_sha256'),
            'errorCode':error},qos=1)

    async def mapping_control(self, message, session):
        accepted,error=False,None
        try:
            mapping=self.adapter.mapping
            if not mapping:raise StepError('MAPPING_NOT_ENABLED')
            mapping.receive_control(message,session.id,self.connection_ready and not self.session_fault,self.active is None)
            accepted=True
        except (StepError,ValueError,KeyError,TypeError) as exc:
            error=getattr(exc,'code','INVALID_MAPPING_CONTROL')
            logging.warning('Mapping control rejected: code=%s command=%s job=%s reason=%s',
                            error,message.get('commandId'),message.get('jobId'),str(exc)[:240])
        ack={key:message.get(key) for key in ('robotId','jobId','mappingSessionId','commandId','sequenceNo')}
        ack.update(messageType='MAPPING_CONTROL_ACK',transportSessionId=session.id,producerSessionId=self.store.session,
                   accepted=accepted,errorCode=error,receivedAt=datetime.now(timezone.utc).isoformat())
        await session.publish('/mapping/control-acks',ack,qos=0)

    def validate_hello_ack(self, message):
        # The transport has already checked robotId and the current transportSessionId.
        # Keep application validation for both initial and redelivered acknowledgements.
        if (not isinstance(message,dict) or message.get("messageType")!="HELLO_ACK"
            or message.get("accepted") is not True
            or message.get("robotId")!=self.cfg["robot_id"] or message.get("contract")!=CONTRACT):
            raise StepError("HELLO_REJECTED")
        version = message.get('batteryTelemetryVersion')
        self.adapter.battery_telemetry_enabled = type(version) is int and version == 1

    async def connected(self, ws):
        async def send(value):
            async with self.send_lock:
                await ws.send(canonical(value))
        self.sent.clear()
        self.adapter.battery_telemetry_enabled = False
        unlock_policy = self.cfg['unlock_policy']
        self.staff_qr = None
        self.staff_face = None
        mapping_enabled = self.cfg['mapping']['enabled']
        if unlock_policy == 'STAFF_VERIFIED' and mapping_enabled:
            unlock_policy = 'DISABLED'
            self.adapter.staff_unlock_ready = False
            logging.info('STAFF_UNLOCK_DISABLED: mapping mode')
        elif unlock_policy == 'STAFF_VERIFIED':
            try:
                from .staff_qr import StaffQrScanner
                if await self.adapter.probe_staff_unlock():
                    self.staff_qr = StaffQrScanner(self)
            except Exception as exc:
                logging.warning('STAFF_UNLOCK_UNAVAILABLE code=%s', getattr(exc, 'code', type(exc).__name__))
            if self.cfg.get('staff_face_enabled', False):
                try:
                    from .staff_face import StaffFaceScanner
                    if await self.adapter.probe_staff_unlock():
                        self.staff_face = StaffFaceScanner(self)
                except Exception as exc:
                    logging.warning('STAFF_FACE_UNAVAILABLE code=%s', type(exc).__name__)
            if self.staff_qr is None and self.staff_face is None:
                unlock_policy = 'DISABLED'
                self.adapter.staff_unlock_ready = False
                logging.warning('STAFF_UNLOCK_DISABLED: camera and PWM readback must be ready')
        await send(self.store.transient("HELLO",{
            "contract":CONTRACT,"runtimeVersion":VERSION,"simulation":False,
            "enabledStepTypes":self.cfg["enabled_step_types"],
            "unlockPolicy":unlock_policy,"states":self.store.states(),
            "mappingEnabled":self.cfg["mapping"]["enabled"],
            "mappingSession":self.adapter.mapping.state if self.adapter.mapping else None,
            **({'navigationSession':self.cfg['navigation_session']} if self.cfg.get('navigation_session') else {}),
            "capabilities":self.advertised_capabilities()}))
        try:
            raw=await asyncio.wait_for(ws.recv(),self.cfg.get('hello_timeout_sec',20))
        except asyncio.TimeoutError:
            raise StepError('HELLO_ACK_TIMEOUT') from None
        hello=json.loads(raw)
        self.validate_hello_ack(hello)
        self.connection_ready=True
        # Old broker/HELLO failures must not leave a recovered worker waiting
        # the maximum backoff after its next, unrelated connection loss.
        self.reconnect_delay_sec=1
        logging.info("BE MQTT session ready: %s",self.cfg["robot_id"])
        outgoing=asyncio.create_task(self.publish(send))
        scanner = asyncio.create_task(self.staff_qr.run(ws)) if self.staff_qr is not None else None
        face_scanner = asyncio.create_task(self.staff_face.run(ws)) if self.staff_face is not None else None
        try:
            while True:
                incoming=asyncio.create_task(ws.recv())
                try:
                    done,_=await asyncio.wait({incoming,outgoing},return_when=asyncio.FIRST_COMPLETED)
                    if outgoing in done:
                        outgoing.result()
                        raise StepError("OUTBOX_STOPPED")
                    raw=incoming.result()
                finally:
                    incoming.cancel()
                    await asyncio.gather(incoming,return_exceptions=True)
                if not isinstance(raw,str):
                    raise StepError("TEXT_JSON_REQUIRED")
                message=json.loads(raw,parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
                if not isinstance(message,dict):
                    raise StepError("OBJECT_REQUIRED")
                if message.get("messageType")=="HELLO_ACK":
                    # QoS 1 delivery and BE durable ACK retries may repeat HELLO_ACK.
                    # An identical accepted ACK must not restart the worker or its tasks.
                    self.validate_hello_ack(message)
                    logging.debug("Accepted repeated HELLO_ACK for current MQTT session")
                elif message.get("messageType")=="EVENT_ACK":
                    mid=message.get("messageId")
                    if mid in self.sent:
                        if message.get("accepted") is not True:
                            self.session_fault=True
                            raise StepError("BE_EVENT_REJECTED")
                        self.store.ack(mid)
                        self.sent.pop(mid,None)
                        waiter=self.ack_waiters.get(mid)
                        if waiter is not None:waiter.set()
                elif message.get("messageType")=="NAV_PROFILE":
                    await self.navigation_profile(message,ws)
                elif message.get("messageType")=="MAPPING_CONTROL":
                    await self.mapping_control(message,ws)
                elif message.get("messageType")=="COMMAND":
                    try:
                        await self.command(message)
                    except (StepError,ValueError,TypeError,KeyError) as exc:
                        await send(self.store.transient("PROTOCOL_ERROR",
                            {"commandId":message.get("commandId"),
                             "errorCode":getattr(exc,"code","INVALID_MESSAGE")}))
                else:
                    logging.warning("Unsupported BE messageType: %.80r",message.get("messageType"))
                    raise StepError("UNKNOWN_MESSAGE_TYPE")
        finally:
            outgoing.cancel()
            if scanner is not None:
                scanner.cancel()
                await asyncio.gather(scanner, return_exceptions=True)
            if face_scanner is not None:
                face_scanner.cancel()
                await asyncio.gather(face_scanner, return_exceptions=True)
            self.staff_face = None
            self.staff_qr = None
            await asyncio.gather(outgoing,return_exceptions=True)

    async def mapping_loop(self, session):
        context = self.adapter.mapping or getattr(self.adapter, 'navigation_manual', None)
        if not context:
            await asyncio.Future()
        next_tick=time.monotonic()
        while True:
            await context.tick(self.connection_ready and not self.session_fault and not self.shutting_down,
                               self.active is None and not self.store.blocked() and not self.session_fault)
            next_tick+=.05
            now=time.monotonic()
            if next_tick<now-.05:next_tick=now+.05
            await asyncio.sleep(max(0.,next_tick-now))

    async def mapping_media_loop(self, session):
        mapping=self.adapter.mapping
        if not mapping:await asyncio.Future()
        last_preview=0
        while True:
            if self.connection_ready and time.monotonic()-last_preview>=.5:
                preview=await mapping.preview()
                if preview:
                    await session.publish("/mapping/preview",{
                        "robotId":self.cfg["robot_id"],"transportSessionId":session.id,
                        "messageType":"MAP_PREVIEW","payload":preview},qos=0)
                if mapping.camera and mapping.state.get('state')=='MAPPING':
                    session_id=mapping.state['session_id']
                    camera=await mapping.camera.preview(session_id)
                    if camera and mapping.state.get('state')=='MAPPING' and mapping.state.get('session_id')==session_id:
                        await session.publish('/camera/preview',{'robotId':self.cfg['robot_id'],
                            'transportSessionId':session.id,'messageType':'CAMERA_PREVIEW','payload':camera},qos=0)
                last_preview=time.monotonic()
            await asyncio.sleep(.1)

    async def navigation_pose_loop(self, session):
        """Latest ROS pose for viewing only; no extra ROS node, DB write or heartbeat."""
        context=self.cfg.get('navigation_session')
        if not context:await asyncio.Future()
        sequence=0
        while True:
            if self.connection_ready:
                state=self.adapter.telemetry();pose=state.get('map_pose',{});quality=state.get('localization',{})
                value=pose.get('value') or {}
                if (pose.get('fresh') and quality.get('ready') and pose.get('age_sec',99)<=1.
                    and value.get('navigation_session_id')==context['session_id']
                    and value.get('initialization_revision',0)>0):
                    sequence+=1
                    await session.publish('/navigation/pose',{
                        'messageType':'NAVIGATION_POSE','robotId':self.cfg['robot_id'],
                        'transportSessionId':session.id,'producerSessionId':self.store.session,
                        'navigationSessionId':context['session_id'],'mapId':context['map_id'],
                        'mapAssetHash':context['map_asset_hash'],'initializationRevision':value['initialization_revision'],
                        'sequenceNo':sequence,'capturedAt':(datetime.now(timezone.utc)-timedelta(seconds=pose['age_sec'])).isoformat(),
                        'pose':dict(xM=value['x_m'],yM=value['y_m'],yawRad=value['yaw_rad'])},qos=0)
            await asyncio.sleep(.2 if self.active and self.active.get('stepType')=='MOVE' else .5)

    async def settle_cancelled_moves(self):
        if self.active is not None or self.task and not self.task.done():
            return
        if time.monotonic()<self.cancel_settlement_at:
            return
        candidates=self.store.stopped_move_settlements()
        if not candidates:
            return
        self.cancel_settlement_at=time.monotonic()+30.
        from .navigation_session import SessionClient
        try:
            client=SessionClient(self.cfg)
        except (StepError,OSError,ValueError,TypeError,KeyError) as error:
            logging.warning('CANCEL_SETTLEMENT_PENDING code=%s',getattr(error,'code',type(error).__name__))
            return
        for candidate in candidates[:16]:
            try:
                path=('/api/v1/pi/jobs/'+candidate['jobId']+'/operations/'
                      +candidate['jobStepId']+'/settlement')
                body={key:candidate[key] for key in ('commandId','operationKey')}
                raw=await asyncio.to_thread(client.request,path,body=body)
                receipt=json.loads(raw)
                self.store.settle_cancelled_move(candidate,receipt)
                logging.info('TERMINAL_MOVE_SETTLED job=%s step=%s status=%s',
                             candidate['jobId'],candidate['jobStepId'],receipt['status'])
            except (StepError,OSError,ValueError,TypeError,KeyError) as error:
                code=getattr(error,'code',type(error).__name__)
                logging.warning('CANCEL_SETTLEMENT_PENDING step=%s code=%s',candidate['jobStepId'],code)
                if code!='NAVIGATION_SESSION_HTTP_422':
                    break
        if not self.store.blocked() and not self.store.pending():
            self.session_fault=False

    async def fall_monitor_loop(self, observer):
        while True:
            try:
                await observer.monitor(self)
            except asyncio.CancelledError:
                raise
            except Exception as error:
                logging.warning('FALL_MONITOR_RETRY code=%s', getattr(error, 'code', type(error).__name__))
            await asyncio.sleep(3.)

    async def run(self):
        self.store.reissue_operator_reconciliations()
        observer = getattr(self.adapter, 'patrol_fall_observer', None)
        fall_monitor = (asyncio.create_task(self.fall_monitor_loop(observer))
                        if observer is not None and observer.config.get('mode') == 'CONTINUOUS' else None)
        # Local USB input keeps its 20 Hz loop across broker connect/backoff.
        # It does not depend on a BE mapping session or a transport lease.
        manual = self.manual_context()
        local_mode = bool(manual and not manual.gamepad.cfg['session_required'])
        local_mapping=asyncio.create_task(self.mapping_loop(None)) if local_mode else None
        try:
            while True:
                try:
                    await self.settle_cancelled_moves()
                    async with MqttSession(self.cfg) as session:
                        connected=asyncio.create_task(self.connected(session))
                        mapping=local_mapping or asyncio.create_task(self.mapping_loop(session))
                        media=asyncio.create_task(self.mapping_media_loop(session))
                        poses=asyncio.create_task(self.navigation_pose_loop(session))
                        poses.add_done_callback(lambda task:logging.warning('Live pose observation unavailable: %s',type(task.exception()).__name__)
                            if not task.cancelled() and task.exception() else None)
                        try:
                            # Observation failure cannot cancel a physical MOVE. Actual
                            # MQTT failure is still handled by connected/session above.
                            done,_=await asyncio.wait({connected,mapping,media},return_when=asyncio.FIRST_COMPLETED)
                            for task in done:
                                task.result()
                        finally:
                            connected.cancel();media.cancel();poses.cancel()
                            if not local_mode:mapping.cancel()
                            await asyncio.gather(connected,media,poses,*([] if local_mode else [mapping]),return_exceptions=True)
                            await self.disconnect("CONNECTION_LOST")
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    logging.warning("MQTT connection ended: %s; code=%s; retry in %ss",
                        type(exc).__name__,getattr(exc,"code","NETWORK_OR_PROTOCOL"),self.reconnect_delay_sec)
                finally:
                    await self.disconnect("CONNECTION_LOST")
                if local_mapping and local_mapping.done():
                    local_mapping.result()
                    raise StepError('LOCAL_CONTROL_LOOP_STOPPED')
                await asyncio.sleep(self.reconnect_delay_sec)
                self.reconnect_delay_sec=min(self.reconnect_delay_sec*2,30)
        finally:
            if fall_monitor is not None:
                fall_monitor.cancel()
                await asyncio.gather(fall_monitor, return_exceptions=True)
            if local_mapping:
                self.inhibit_manual()
                local_mapping.cancel()
                await asyncio.gather(local_mapping,return_exceptions=True)
            await self.disconnect("PROCESS_SHUTDOWN")
