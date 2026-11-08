"""Optional supervised ROS-camera/WebRTC client. One publisher, bounded RAM, no recording."""
import argparse
import asyncio
import json
import logging
import os
import signal
import threading
import time
from uuid import UUID
from .media_runtime import Reservation,configuration
from .media_client import MediaClient

OPEN={'CONNECTING','ACTIVE','RECONNECTING'}


def audio_index(devices,selection):
    if isinstance(selection,int):
        if any(d['index']==selection for d in devices):return selection
    elif selection!='auto':
        matched=[d for d in devices if selection.lower() in d['name'].lower()]
        if len(matched)==1:return matched[0]['index']
    else:
        matched=[d for d in devices if any(x in d['name'].lower() for x in ('usb','wonderecho'))]
        if len(matched)==1:return matched[0]['index']
    raise RuntimeError('MEDIA_AUDIO_DEVICE_AMBIGUOUS_OR_MISSING')


def validate_credentials(value,worker,session):
    if value.get('url','').rstrip('/')!=worker['media']['url'].rstrip('/'):
        raise RuntimeError('MEDIA_ORIGIN_MISMATCH')
    if value.get('room')!='mar-'+session or value.get('identity')!='robot-'+worker['robot_id']:
        raise RuntimeError('MEDIA_IDENTITY_MISMATCH')
    if value.get('session',{}).get('robotId')!=worker['robot_id'] or value['session'].get('sessionId')!=session:
        raise RuntimeError('MEDIA_SESSION_MISMATCH')
    if not isinstance(value.get('token'),str) or not 32<=len(value['token'])<=8192:
        raise RuntimeError('MEDIA_TOKEN_INVALID')
    if value.get('width')!=640 or value.get('height')!=480 or value.get('fps')!=15 or not 100000<=value.get('bitrate',0)<=1500000:
        raise RuntimeError('MEDIA_VIDEO_LIMITS_INVALID')
    return value


class CameraSource:
    def __init__(self,worker,hardware):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from types import SimpleNamespace
        from std_msgs.msg import String
        from nav_msgs.msg import Odometry
        from .camera_preview import CameraPreview
        self.String=String;self.holding=False;self.video_active=False;self.safety={};self.safety_at=0.;self.odom=None;self.odom_at=0.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node=rclpy.create_node('mediassist_media')
        self.publisher=self.node.create_publisher(String,'/mediassist/media_hold',10)
        self.node.create_subscription(String,'/mediassist/safety_state',self.safety_seen,10)
        self.node.create_subscription(Odometry,hardware['ros']['odom'],self.odom_seen,qos_profile_sensor_data)
        settings=dict(worker['mapping']['camera'],sample_every_frame=True,source_max_age_sec=.75)
        self.preview=CameraPreview(SimpleNamespace(node=self.node),settings,lambda:self.video_active)
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        def spin():
            try:self.executor.spin()
            except ExternalShutdownException:pass
            except Exception:logging.exception('MEDIA_ROS_RECEIVER_FAILED')
        self.thread=threading.Thread(target=spin,name='media_ros_latest',daemon=True);self.thread.start()

    def safety_seen(self,msg):
        try:self.safety=json.loads(msg.data);self.safety_at=time.monotonic()
        except (ValueError,TypeError):pass

    def odom_seen(self,msg):
        v=msg.twist.twist;self.odom=(v.linear.x,v.linear.y,v.angular.z);self.odom_at=time.monotonic()

    def hold(self,active):
        self.holding=active
        self.publisher.publish(self.String(data=json.dumps(dict(active=active))))

    async def stopped(self):
        deadline=time.monotonic()+4;settled=None
        while time.monotonic()<deadline:
            self.hold(True)
            good=(time.monotonic()-self.safety_at<.5 and self.safety.get('reason')=='SOS_MEDIA_HOLD'
                  and self.safety.get('moving_command') is False
                  and time.monotonic()-self.odom_at<1. and self.odom is not None
                  and all(abs(v)<.025 for v in self.odom))
            if good:
                settled=settled or time.monotonic()
                if time.monotonic()-settled>=.3:return
            else:settled=None
            await asyncio.sleep(.05)
        raise RuntimeError('MEDIA_SAFETY_NOT_READY')

    async def release_hold(self):
        # Do not resume an old nonzero Nav2/manual command when an ASSIST ends.
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            command=self.safety.get('requested_velocity',[1.,1.,1.])
            if time.monotonic()-self.safety_at<.5 and len(command)==3 and all(abs(v)<.01 for v in command):
                self.hold(False);return
            self.hold(True);await asyncio.sleep(.1)
        logging.error('MEDIA_HOLD_REQUIRES_STACK_STOP: old velocity has not returned to zero')

    def close(self):
        import rclpy
        self.executor.shutdown(timeout_sec=2);self.thread.join(2);self.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


class Bridge:
    def __init__(self,worker,hardware):
        self.worker=worker;self.options=configuration(worker);self.client=MediaClient(worker)
        self.source=CameraSource(worker,hardware);self.queue=asyncio.Queue(maxsize=1);self.call=None;self.current_id=None

    async def run(self):
        loop=asyncio.get_running_loop()
        listener=threading.Thread(target=self.client.listen,args=(loop,self.queue),name='media_https_events',daemon=True)
        listener.start()
        try:
            while True:
                await self.queue.get()
                try:
                    # Treat SSE as a wake-up. Read current committed state once, avoiding snapshot/event races.
                    current=await self.client.current();session=current.get('session') if current.get('enabled') else None
                    target=session.get('sessionId') if session and session.get('state') in OPEN else None
                    if target==self.current_id and self.call is not None and not self.call.done():continue
                    if self.call is not None:
                        self.call.cancel();await asyncio.gather(self.call,return_exceptions=True);self.call=None
                    self.current_id=target
                    if target:
                        UUID(target)
                        self.call=asyncio.create_task(self.connect(session),name='media-call')
                except asyncio.CancelledError:raise
                except Exception as e:
                    logging.warning('MEDIA_SESSION_RETRY code=%s',getattr(e,'code',type(e).__name__))
                    await asyncio.sleep(2)
                    if not self.queue.full():self.queue.put_nowait(True)
        finally:
            if self.call is not None:self.call.cancel();await asyncio.gather(self.call,return_exceptions=True)
            self.client.close();await asyncio.to_thread(listener.join,3);self.source.close()

    async def connect(self,session):
        from livekit import rtc
        from .audio_lock import AudioLock
        from .camera_preview import frame_pixels
        import cv2
        cv2.setNumThreads(1)
        sid=session['sessionId'];assist=session['mode']=='ASSIST'
        reservation=Reservation(sid) if assist else None
        room=rtc.Room();player=None;capture=None;video=None;pulse=None;audio_lock=None
        audio_tasks=set()
        def spawn_audio(coro):
            task=asyncio.create_task(coro);audio_tasks.add(task)
            def done(t):
                audio_tasks.discard(t)
                if not t.cancelled() and t.exception():logging.warning('MEDIA_AUDIO_TRACK_FAILED')
            task.add_done_callback(done)
        async def reserve():
            while True:reservation.refresh();self.source.hold(True);await asyncio.sleep(.2)
        try:
            if assist:
                reservation.refresh();pulse=asyncio.create_task(reserve())
                await self.source.stopped()
                lock=AudioLock(priority=True);await asyncio.wait_for(lock.__aenter__(),5);audio_lock=lock
            credentials=validate_credentials(await self.client.credentials(sid),self.worker,sid)
            rtc_config=rtc.RtcConfiguration(ice_transport_type=rtc.IceTransportType.TRANSPORT_RELAY if self.options['force_relay'] else rtc.IceTransportType.TRANSPORT_ALL)
            @room.on('reconnecting')
            def reconnecting():logging.warning('MEDIA_RECONNECTING session=%s',sid)
            @room.on('track_subscribed')
            def subscribed(track,publication,participant):
                expected='staff-'+str(session['staffAccountId'])
                if assist and participant.identity==expected and track.kind==rtc.TrackKind.KIND_AUDIO and player:
                    spawn_audio(player.add_track(track))
            @room.on('track_unsubscribed')
            def unsubscribed(track,publication,participant):
                if player and track.kind==rtc.TrackKind.KIND_AUDIO:spawn_audio(player.remove_track(track))
            await asyncio.wait_for(room.connect(credentials['url'],credentials['token'],
                options=rtc.RoomOptions(auto_subscribe=True,rtc_config=rtc_config)),15)
            if assist:
                from livekit.rtc.media_devices import MediaDevices
                devices=MediaDevices(blocksize=480,num_channels=1)
                capture=devices.open_input(input_device=audio_index(devices.list_input_devices(),self.options['audio_input']),
                                           enable_aec=True,noise_suppression=True,queue_capacity=10)
                player=devices.open_output(output_device=audio_index(devices.list_output_devices(),self.options['audio_output']))
                await player.start()
                for participant in room.remote_participants.values():
                    if participant.identity=='staff-'+str(session['staffAccountId']):
                        for publication in participant.track_publications.values():
                            if publication.track and publication.track.kind==rtc.TrackKind.KIND_AUDIO:await player.add_track(publication.track)
                audio=rtc.LocalAudioTrack.create_audio_track('patient-microphone',capture.source)
                await room.local_participant.publish_track(audio,rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_MICROPHONE))
            video=rtc.VideoSource(640,480)
            track=rtc.LocalVideoTrack.create_video_track('robot-camera',video)
            await room.local_participant.publish_track(track,rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_CAMERA,
                video_codec=rtc.VideoCodec.VP8,simulcast=False,video_encoding=rtc.VideoEncoding(max_bitrate=credentials['bitrate'],max_framerate=15)))
            self.source.video_active=True
            logging.info('MEDIA_JOINED session=%s mode=%s',sid,session['mode'])
            await self.client.report(sid,'CONNECTED')
            report_at=time.monotonic()+10;last_frame=time.monotonic();camera_warned=False;staff_seen=time.monotonic()
            deadline=time.monotonic()+max(1.,min(1800.,(parse_time(credentials['expiresAt'])-time.time())))
            while time.monotonic()<deadline:
                if 'staff-'+str(session['staffAccountId']) in room.remote_participants:staff_seen=time.monotonic()
                elif time.monotonic()-staff_seen>30:raise RuntimeError('MEDIA_STAFF_DISCONNECTED')
                if time.monotonic()>=report_at:
                    state='CONNECTED' if room.connection_state==rtc.ConnectionState.CONN_CONNECTED else 'RECONNECTING'
                    value=await self.client.report(sid,state)
                    if value.get('state') not in OPEN:return
                    report_at=time.monotonic()+10
                sample=self.source.preview.take_sample(.5)
                if sample:
                    def send():
                        pixels=frame_pixels(sample)
                        if pixels.shape[:2]!=(480,640):pixels=cv2.resize(pixels,(640,480))
                        rgb=cv2.cvtColor(pixels,cv2.COLOR_GRAY2RGB if pixels.ndim==2 else cv2.COLOR_BGR2RGB)
                        video.capture_frame(rtc.VideoFrame(640,480,rtc.VideoBufferType.RGB24,rgb.tobytes()))
                    await asyncio.to_thread(send);last_frame=time.monotonic();camera_warned=False
                elif time.monotonic()-last_frame>15 and not camera_warned:
                    logging.warning('MEDIA_CAMERA_NO_FRAMES session=%s; audio remains available',sid);camera_warned=True
                await asyncio.sleep(1/15.)
        except asyncio.CancelledError:raise
        except Exception as e:
            code=str(e) if str(e).startswith('MEDIA_') and len(str(e))<96 else 'MEDIA_JOIN_FAILED'
            logging.warning('MEDIA_CALL_FAILED session=%s code=%s',sid,code)
            try:await self.client.report(sid,'FAILED',code)
            except Exception:pass
        finally:
            for task in list(audio_tasks):task.cancel()
            self.source.video_active=False
            await asyncio.gather(*list(audio_tasks),return_exceptions=True)
            for resource in (capture,player,room,video):
                if resource:
                    try:await asyncio.wait_for(resource.disconnect() if resource is room else resource.aclose(),3)
                    except Exception:logging.warning('MEDIA_RESOURCE_CLOSE_FAILED')
            if audio_lock:await audio_lock.__aexit__(None,None,None)
            if pulse:pulse.cancel();await asyncio.gather(pulse,return_exceptions=True)
            if reservation:reservation.close();await self.source.release_hold()
            logging.info('MEDIA_LEFT session=%s',sid)


def parse_time(value):
    from datetime import datetime
    return datetime.fromisoformat(value.replace('Z','+00:00')).timestamp()


async def main_async(args):
    from .media_runtime import bridge_lock
    from .worker_config import load
    worker,hardware=load(args.config)
    if not configuration(worker)['enabled']:raise RuntimeError('MEDIA_DISABLED')
    with bridge_lock():
        bridge=Bridge(worker,hardware);task=asyncio.create_task(bridge.run());loop=asyncio.get_running_loop()
        for sig in (signal.SIGINT,signal.SIGTERM):loop.add_signal_handler(sig,task.cancel)
        try:await task
        except asyncio.CancelledError:pass


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',required=True)
    args=parser.parse_args();logging.basicConfig(level=logging.INFO,format='%(asctime)s %(levelname)s %(message)s')
    try:os.nice(5)
    except (OSError,AttributeError):pass
    asyncio.run(main_async(args))


if __name__=='__main__':main()
