"""MQTT step worker. The backend owns jobs, branching, retries and staff UI."""
import asyncio
import json
import logging
import time
import uuid
from datetime import datetime, timezone
from .mqtt_transport import MqttSession
from .contracts import StepError, canonical
from .step_contract import CONTRACT, SUBPROTOCOL, VERSION, validate_command, validate_step, validate_result, expiry
from .worker_config import device_token, tls

class StepWorker:
    def __init__(self, cfg, hardware, store, adapter):
        self.cfg, self.hardware, self.store, self.adapter = cfg, hardware, store, adapter
        self.task, self.active, self.last_result = None, None, None
        self.connection_ready = False
        self.sent = {}
        self.send_lock = asyncio.Lock()
        self.stop_reason = "CANCELLED"
        self.session_fault = False

    def status(self):
        if self.store.blocked() or getattr(self.adapter,"motion_uncertain",False):
            return "ERROR"
        mapping=self.adapter.mapping
        if mapping and mapping.state.get("state")=="RECOVERY_REQUIRED":
            return "ERROR"
        local_busy=bool(mapping and getattr(mapping,'manual_drive',None) and mapping.manual_drive.busy(mapping.gamepad))
        return "BUSY" if self.active or (mapping and mapping.active) or local_busy else "IDLE"

    async def acknowledged(self, mid):
        deadline=time.monotonic()+self.cfg["ack_timeout_sec"]
        while not self.store.is_acked(mid):
            if not self.connection_ready:
                raise StepError("CONNECTION_LOST")
            if time.monotonic() >= deadline:
                raise StepError("BE_ACK_TIMEOUT")
            await asyncio.sleep(0.05)

    def receipt(self, command, accepted, step=None, **extra):
        event=self.store.emit("COMMAND_ACK",
            {"commandId":command["commandId"],"commandType":command["commandType"],
             "accepted":accepted,**extra}, step,command["correlationId"])
        self.store.remember_command(command,event)
        return event

    async def command(self, command):
        validate_command(command,self.cfg["robot_id"],check_expiry=False)
        if self.store.cached_command(command):
            return
        validate_command(command,self.cfg["robot_id"])
        if command["commandType"] == "CANCEL_STEP":
            step=self.active
            if not step or step["jobStepId"]!=command["jobStepId"] or step["jobId"]!=command["jobId"]:
                self.receipt(command,False,errorCode="STEP_NOT_ACTIVE")
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
        if mapping and getattr(mapping,'manual_drive',None) and mapping.manual_drive.busy(mapping.gamepad):
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
                command_context(command,self.cfg,self.hardware,self.adapter)
                self.adapter.active_navigation_context=command.get('navigationContext')
        except (StepError,ValueError) as exc:
            self.receipt(command,False,step,errorCode=getattr(exc,"code","INVALID_PARAMS"))
            return
        self.store.accept(step)
        ack=self.receipt(command,True,step,status="ACCEPTED")
        self.active=step
        if mapping and getattr(mapping,'manual_drive',None):
            mapping.gamepad.stop('WORKFLOW_HANDOFF_RETURN_NEUTRAL')
        self.stop_reason="CANCELLED"
        self.task=asyncio.create_task(self.execute(step,command["correlationId"],ack["messageId"],command["expiresAt"]))

    async def execute(self, step, correlation, accepted_mid, expires_at):
        status, data, error = "FAILED", None, None
        error_details = None
        physical_started=False
        stop_confirmed=None
        try:
            # BE must durably acknowledge acceptance and STEP_STARTED before physical effects.
            await self.acknowledged(accepted_mid)
            started=self.store.emit("STEP_STARTED",{"attemptNo":1,"stepType":step["stepType"],
                    "resolvedInputs":step["params"]},step,correlation)
            await self.acknowledged(started["messageId"])
            if expiry(expires_at) <= datetime.now(timezone.utc):
                raise StepError("COMMAND_EXPIRED_BEFORE_EXECUTION")
            self.store.set_running(step)
            physical_started=True
            data=await asyncio.wait_for(self.adapter.perform(step),step["timeoutSec"])
            validate_result(step,data)
            status="SUCCEEDED"
        except asyncio.CancelledError:
            status="CANCELLED" if self.stop_reason=="CANCELLED" else "RECOVERY_REQUIRED"
            error=self.stop_reason
        except asyncio.TimeoutError:
            status,error="TIMED_OUT","STEP_TIMEOUT"
        except Exception as exc:
            error=getattr(exc,"code","STEP_EXECUTION_ERROR")
            # urllib HTTPError.code is an integer, unlike StepError.code.
            if not isinstance(error,str) or not error:
                error="STEP_EXECUTION_ERROR"
            if error.startswith("AUDIO_") and isinstance(getattr(exc,"details",None),dict):
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
            if step["stepType"]=="CONTROL_HARDWARE" and physical_started and status!="SUCCEEDED":
                status="RECOVERY_REQUIRED"
            if getattr(self.adapter,"motion_uncertain",False):
                status,error="RECOVERY_REQUIRED","PHYSICAL_STOP_UNCONFIRMED"
            result={"status":status,"result":data,"errorCode":error,
                    "physicalStopConfirmed":stop_confirmed,
                    "actuatorPositionVerified":False if step["stepType"]=="CONTROL_HARDWARE" else None}
            self.last_result=result
            self.store.finish(step,status,result)
            self.store.emit("STEP_RESULT" if status=="SUCCEEDED" else "STEP_ERROR",
                {"status":status,"result":data,"errorCode":error,"retryable":False,
                 "details":None if not error else error_details or {"code":error},
                 "physicalStopConfirmed":stop_confirmed},step,correlation)
            self.active=None

    async def abandoned(self, reason):
        if self.active is None:
            return
        step=self.active
        try:
            stopped=(await self.adapter.stop()) is True
        except Exception:
            stopped=False
        status="CANCELLED" if reason=="CANCELLED" and stopped else "RECOVERY_REQUIRED"
        result={"status":status,"errorCode":reason,"physicalStopConfirmed":stopped}
        self.last_result=result
        self.store.finish(step,status,result)
        self.active=None
        self.store.emit("STEP_ERROR",{**result,"retryable":False,"details":{"code":reason}},step)

    async def disconnect(self, reason):
        self.connection_ready=False
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
                local=bool(self.adapter.mapping and getattr(self.adapter.mapping,'manual_drive',None)
                           and not self.adapter.mapping.gamepad.cfg['session_required'] and reason!='PROCESS_SHUTDOWN')
                if not local:self.adapter.lease_message(False)
                logging.info("Connection ended; no active worker goal")
        except Exception as exc:
            self.adapter.motion_uncertain=True
            logging.warning("Stop unavailable: %s",type(exc).__name__)

    async def publish(self, send):
        last_heartbeat=0
        while True:
            now=time.monotonic()
            if now-last_heartbeat>=self.cfg["heartbeat_sec"]:
                await send(self.store.transient("HEARTBEAT",{
                    "status":self.status(),"runtimeVersion":VERSION,"protocolMajor":1,
                    "capabilities":self.hardware["capabilities"],
                    "activeJobStepId":self.active["jobStepId"] if self.active else None,
                    "currentMapId":None if self.adapter.mapping else self.hardware["map_id"],
                    "mapAssetHash":None if self.adapter.mapping else self.hardware["map_asset_sha256"],
                    "mappingSession":self.adapter.mapping.state if self.adapter.mapping else None,
                    **({'navigationSession':self.cfg['navigation_session']} if self.cfg.get('navigation_session') else {}),
                    "telemetry":self.adapter.telemetry()}))
                last_heartbeat=now
            # Bound each replay batch so heartbeat cannot be starved by old events.
            count=0
            for event in self.store.pending():
                mid=event["messageId"]
                if mid not in self.sent or time.monotonic()-self.sent[mid]>=5:
                    self.sent[mid]=time.monotonic()
                    # Register before awaiting MQTT PUBACK: BE EVENT_ACK can arrive first.
                    await send(event)
                    count+=1
                    if count>=4:
                        break
            await asyncio.sleep(0.2)

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

    async def connected(self, ws):
        async def send(value):
            async with self.send_lock:
                await ws.send(canonical(value))
        self.sent.clear()
        await send(self.store.transient("HELLO",{
            "contract":CONTRACT,"runtimeVersion":VERSION,"simulation":False,
            "enabledStepTypes":self.cfg["enabled_step_types"],
            "unlockPolicy":self.cfg["unlock_policy"],"states":self.store.states(),
            "mappingEnabled":self.cfg["mapping"]["enabled"],
            "mappingSession":self.adapter.mapping.state if self.adapter.mapping else None,
            **({'navigationSession':self.cfg['navigation_session']} if self.cfg.get('navigation_session') else {}),
            "capabilities":self.hardware["capabilities"]}))
        raw=await asyncio.wait_for(ws.recv(),10)
        hello=json.loads(raw)
        self.validate_hello_ack(hello)
        self.connection_ready=True
        logging.info("BE MQTT session ready: %s",self.cfg["robot_id"])
        outgoing=asyncio.create_task(self.publish(send))
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
            await asyncio.gather(outgoing,return_exceptions=True)

    async def mapping_loop(self, session):
        mapping=self.adapter.mapping
        if not mapping:
            await asyncio.Future()
        next_tick=time.monotonic()
        while True:
            await mapping.tick(self.connection_ready and not self.session_fault,
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

    async def run(self):
        delay=1
        # Local USB input keeps its 20 Hz loop across broker connect/backoff.
        # It does not depend on a BE mapping session or a transport lease.
        local_mode=bool(self.adapter.mapping and getattr(self.adapter.mapping,'manual_drive',None)
                        and not self.adapter.mapping.gamepad.cfg['session_required'])
        local_mapping=asyncio.create_task(self.mapping_loop(None)) if local_mode else None
        try:
            while True:
                try:
                    async with MqttSession(self.cfg) as session:
                        connected=asyncio.create_task(self.connected(session))
                        mapping=local_mapping or asyncio.create_task(self.mapping_loop(session))
                        media=asyncio.create_task(self.mapping_media_loop(session))
                        try:
                            done,_=await asyncio.wait({connected,mapping,media},return_when=asyncio.FIRST_COMPLETED)
                            for task in done:
                                task.result()
                        finally:
                            connected.cancel();media.cancel()
                            if not local_mode:mapping.cancel()
                            await asyncio.gather(connected,media,*([] if local_mode else [mapping]),return_exceptions=True)
                            await self.disconnect("CONNECTION_LOST")
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    logging.warning("MQTT connection ended: %s; code=%s; retry in %ss",
                        type(exc).__name__,getattr(exc,"code","NETWORK_OR_PROTOCOL"),delay)
                finally:
                    await self.disconnect("CONNECTION_LOST")
                if local_mapping and local_mapping.done():
                    local_mapping.result()
                    raise StepError('LOCAL_CONTROL_LOOP_STOPPED')
                await asyncio.sleep(delay)
                delay=min(delay*2,30)
        finally:
            if local_mapping:
                self.adapter.mapping.manual_inhibited=True
                local_mapping.cancel()
                await asyncio.gather(local_mapping,return_exceptions=True)
            await self.disconnect("PROCESS_SHUTDOWN")
