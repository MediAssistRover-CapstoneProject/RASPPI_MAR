"""Mapping session, local dead-man teleoperation and portable map artifacts."""
import asyncio
import base64
import hashlib
import json
import math
import os
import signal
import tarfile
import threading
import time
import urllib.request
import zlib
from pathlib import Path
from .contracts import StepError, canonical
from .store import utcnow
from .worker_config import NoRedirect


class MappingManager:
    def __init__(self, adapter, cfg):
        self.adapter, self.cfg = adapter, cfg
        self.root = Path(cfg['storage_dir'])
        self.root.mkdir(parents=True, exist_ok=True)
        self.state_file = self.root/'session.json'
        self.state = json.loads(self.state_file.read_text()) if self.state_file.exists() else {}
        if self.state.get('session_open'):
            self.state['state'] = 'RECOVERY_REQUIRED'
            self.persist()
        self.lock = threading.Lock()
        self.grid = None
        self.revision = 0
        self.last_sent = 0
        self.last_preview = 0
        self.last_graph_check = 0
        self.manual = (0., 0., 0.)
        from .mapping_control import MappingControlLease
        self.remote_control = MappingControlLease()
        self.camera = None
        self.process = None
        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import String
        from geometry_msgs.msg import Twist
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        adapter.node.create_subscription(OccupancyGrid, cfg['map_topic'], self.on_map, qos)
        adapter.node.create_subscription(String, '/mediassist/mapping/manual', self.on_manual, 1)
        self.velocity = adapter.node.create_publisher(Twist, adapter.cfg['ros']['raw_cmd_vel'], 1)
        if cfg.get('camera',{}).get('enabled'):
            from .camera_preview import CameraPreview
            self.camera = CameraPreview(adapter,cfg['camera'],lambda:self.state.get('state')=='MAPPING')

    @property
    def active(self):
        return self.state.get('session_open',False)

    def persist(self):
        temporary = self.state_file.with_suffix('.new')
        with temporary.open('w', encoding='utf-8') as f:
            f.write(canonical(self.state)); f.flush(); os.fsync(f.fileno())
        os.replace(temporary, self.state_file)

    def on_map(self, msg):
        info = msg.info
        if not (0 < info.width*info.height <= self.cfg['max_cells'] and
                len(msg.data) == info.width*info.height and math.isfinite(info.resolution) and info.resolution > 0):
            return
        if msg.header.frame_id != self.adapter.cfg['ros']['map_frame']:
            return
        p, q = info.origin.position, info.origin.orientation
        vals = (p.x,p.y,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(x) for x in vals):
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        snapshot = {'width':info.width, 'height':info.height, 'resolution':info.resolution,
                    'origin':[p.x,p.y,yaw], 'frame_id':msg.header.frame_id,
                    'captured_at':utcnow(), 'data':msg.data.tobytes()}
        with self.lock:
            self.grid = (snapshot,time.monotonic())
            self.revision += 1

    def on_manual(self, msg):
        try:
            if len(msg.data) > 512:
                return
            p = json.loads(msg.data)
            if self.state.get('state') != 'MAPPING' or p['mapping_session_id'] != self.state['session_id']:
                return
            x, z = p['linear_x'], p['angular_z']
            if self.cfg.get('control_source','local') != 'local':
                if x == 0 and z == 0:self.remote_control.stop()
                return
            if type(x) not in (int,float) or type(z) not in (int,float) or not all(math.isfinite(v) for v in (x,z)):
                return
            if not 0 <= x <= .05 or abs(z) > .2:
                return
            with self.lock:
                self.manual = (float(x),float(z),time.monotonic())
        except (ValueError,KeyError,TypeError):
            return

    def receive_control(self, message, transport_id, ready, idle):
        if self.cfg.get('control_source','local') != 'mqtt':raise StepError('MAPPING_CONTROL_SOURCE_LOCAL')
        moving = bool(message.get('linearX') or message.get('angularZ'))
        if moving:
            if not ready or not idle or self.adapter.motion_uncertain:raise StepError('MAPPING_ROBOT_BUSY')
            if not all(self.adapter.cfg['motion'][k] for k in ('commissioned','cart_footprint_validated','watchdog_validated')):
                raise StepError('MOTION_NOT_COMMISSIONED')
            if not self.adapter.health()['sensors_ready']:raise StepError('SENSORS_NOT_READY')
            odom=self.adapter.telemetry().get('odom_speed')
            if not odom or odom['age_sec']>.5:raise StepError('ODOM_NOT_READY')
            self.adapter._check_graph()
        self.remote_control.accept(message,self.adapter.worker['robot_id'],transport_id,self.state)

    def snapshot(self):
        with self.lock:
            item, rev = self.grid, self.revision
        if not item or time.monotonic()-item[1] > self.cfg['max_map_age_sec']:
            raise StepError('MAPPING_MAP_NOT_FRESH')
        return item[0], rev

    async def tick(self, ready, idle):
        from geometry_msgs.msg import Twist
        if not self.active:
            return
        with self.lock:
            x, z, captured = self.manual
        if self.cfg.get('control_source','local') == 'mqtt':
            x,z = self.remote_control.current()
            fresh = True
        else:
            fresh = time.monotonic()-captured < .25
        moving = (ready and idle and not self.adapter.motion_uncertain and self.state.get('state') == 'MAPPING'
                  and fresh and self.adapter.health()['sensors_ready'])
        if not moving:self.remote_control.stop()
        if moving:
            if time.monotonic()-self.last_graph_check >= .2:
                self.adapter._check_graph()
                self.last_graph_check = time.monotonic()
            odom = self.adapter.telemetry().get('odom_speed')
            if not odom or odom['age_sec'] > .5:
                moving = False
        msg = Twist()
        if moving:
            msg.linear.x, msg.angular.z = x, z
        self.velocity.publish(msg)
        self.adapter.lease_message(bool(moving and (x or z)), min(.05,self.adapter.cfg['max_speed_mps']))

    async def preview(self):
        rate = min(self.state.get('preview_rate_hz',0.2), self.cfg.get('max_preview_rate_hz',0.2))
        if self.state.get('state') != 'MAPPING' or time.monotonic()-self.last_preview < 1/rate:
            return None
        try:
            snap, rev = self.snapshot()
        except StepError:
            return None
        if rev == self.last_sent:
            return None
        self.last_preview = time.monotonic()
        encoded = await asyncio.to_thread(lambda: base64.b64encode(zlib.compress(snap['data'],1)).decode('ascii'))
        if len(encoded) > self.cfg.get('max_preview_bytes',131072):
            return None
        self.last_sent = rev
        pose = self.adapter.telemetry()['map_pose']
        # Pose belongs to the live session, not the previously commissioned map UUID.
        if pose.get('value'):
            # The outer payload already identifies the SLAM session. Worker
            # telemetry also includes mapping_session_id; neither identity field
            # belongs in common.pose, which accepts only these three numbers.
            pose = {**pose, 'value':{k:pose['value'][k] for k in ('x_m','y_m','yaw_rad')}}
        return {**{k:v for k,v in snap.items() if k!='data'}, 'revision':rev,
                'mapping_session_id':self.state['session_id'], 'pose':pose,
                'encoding':'zlib-base64-int8-row-major-bottom-up', 'data':encoded}

    def ensure_scope(self, step):
        if self.state.get('session_id') != step['params']['mapping_session_id'] or self.state.get('job_id') != step['jobId']:
            raise StepError('MAPPING_SESSION_MISMATCH')

    async def perform(self, step):
        p = step['params']
        action = p['action']
        if action == 'START':
            if self.active:
                raise StepError('MAPPING_ALREADY_ACTIVE')
            # Existing navigation must be stopped by the local operator first.
            if self.adapter.nav.server_is_ready():
                raise StepError('STOP_NAVIGATION_BEFORE_MAPPING')
            self.state = {'session_id':p['mapping_session_id'], 'job_id':step['jobId'],
                          'state':'STARTING','session_open':True,'preview_rate_hz':min(p.get('preview_rate_hz',0.2),self.cfg.get('max_preview_rate_hz',0.2))}
            self.persist()
            with self.lock:
                self.grid = None
            if self.cfg['launch_argv']:
                log = (self.root/'slam.log').open('ab')
                try:
                    self.process = await asyncio.create_subprocess_exec(*self.cfg['launch_argv'],
                        stdout=log,stderr=log,start_new_session=True)
                finally:
                    log.close()
            deadline = time.monotonic()+self.cfg['start_timeout_sec']
            while True:
                if self.process and self.process.returncode is not None:
                    raise StepError('SLAM_PROCESS_EXITED')
                try:
                    self.snapshot()
                    if self.adapter.telemetry()['map_pose']['fresh']:
                        break
                except StepError:
                    pass
                if time.monotonic() > deadline:
                    raise StepError('SLAM_MAP_OR_TF_NOT_READY')
                await asyncio.sleep(.1)
            self.adapter._check_graph()
            if not await self.adapter.stop():
                raise StepError('PHYSICAL_STOP_UNCONFIRMED')
            self.state['state'] = 'MAPPING'
            self.persist()
            return self.result({'state':'MAPPING','preview_ready':True})
        self.ensure_scope(step)
        if action == 'STOP':
            await self.abort('STOPPED')
            return self.result({'state':self.state['state'],'stopped':True})
        if self.state.get('state') not in ('MAPPING','SAVED_LOCAL','SAVED'):
            raise StepError('MAPPING_NOT_READY_TO_SAVE')
        artifact = p['artifact_id']
        if self.state.get('artifact_id') not in (None,artifact):
            raise StepError('ARTIFACT_ID_MISMATCH')
        self.remote_control.stop()
        self.state['state'] = 'SAVING'
        self.persist()
        with self.lock:
            self.manual = (0.,0.,0.)
        if not await self.adapter.stop():
            raise StepError('PHYSICAL_STOP_UNCONFIRMED')
        folder = self.root/self.state['session_id']/artifact
        if not ((folder/'manifest.json').exists() and (folder/'map.tar.gz').exists()):
            snap, _ = self.snapshot()
            await asyncio.to_thread(self.save_files, folder, snap, artifact)
        metadata = json.loads((folder/'manifest.json').read_text())
        self.state.update(state='SAVED_LOCAL',artifact_id=artifact,artifact_dir=str(folder))
        self.persist()
        uploaded = False
        if self.cfg['upload_base_url']:
            uploaded = await asyncio.to_thread(self.upload, folder, metadata)
        self.state['state'] = 'SAVED' if uploaded else 'SAVED_LOCAL'
        self.persist()
        return self.result({'state':self.state['state'],'saved_locally':True,'uploaded':uploaded,**metadata})

    def save_files(self, folder, snap, artifact):
        import yaml
        folder.mkdir(parents=True,exist_ok=True)
        width, height = snap['width'],snap['height']
        raw = snap['data']
        # Trinary PGM: 0 occupied >=65, 254 free <=25, 205 unknown/intermediate.
        lut = bytes(254 if v <= 25 else 0 if 65 <= v <= 100 else 205 for v in range(256))
        pixels = raw.translate(lut)
        pgm = b'P5\n%d %d\n255\n' % (width,height) + b''.join(
            pixels[row*width:(row+1)*width] for row in range(height-1,-1,-1))
        (folder/'map.pgm').write_bytes(pgm)
        text = yaml.safe_dump({'image':'map.pgm','mode':'trinary','resolution':snap['resolution'],
             'origin':snap['origin'],'negate':0,'occupied_thresh':.65,'free_thresh':.25},sort_keys=False).encode()
        (folder/'map.yaml').write_bytes(text)
        metadata = {'artifact_id':artifact,'asset_hash':hashlib.sha256(text+b'\x00'+pgm).hexdigest(),
            'width_px':width,'height_px':height,'resolution_m_per_pixel':snap['resolution'],
            'width_m':width*snap['resolution'],'height_m':height*snap['resolution'],
            'origin_x_m':snap['origin'][0],'origin_y_m':snap['origin'][1],
            'origin_yaw_rad':snap['origin'][2],'frame_id':snap['frame_id']}
        (folder/'manifest.json').write_text(canonical(metadata),encoding='utf-8')
        with tarfile.open(folder/'map.tar.gz','w:gz',format=tarfile.USTAR_FORMAT) as tar:
            for name in ('map.yaml','map.pgm','manifest.json'):
                tar.add(folder/name,arcname=name)

    def upload(self, folder, metadata):
        import ssl
        url = self.cfg['upload_base_url'].rstrip('/')+'/'+self.state['session_id']+'/artifacts/'+metadata['artifact_id']
        token = Path(self.cfg['upload_token_file']).read_text().strip()
        if not token:
            raise StepError('MAP_UPLOAD_TOKEN_REQUIRED')
        body = (folder/'map.tar.gz').read_bytes()
        context = ssl.create_default_context(cafile=self.cfg.get('upload_ca_file') or None)
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect(),
                                            urllib.request.HTTPSHandler(context=context))
        request = urllib.request.Request(url,data=body,method='PUT',headers={
            'Authorization':'Bearer '+token,'Content-Type':'application/gzip',
            'X-Asset-SHA256':metadata['asset_hash'],'X-Archive-SHA256':hashlib.sha256(body).hexdigest()})
        with opener.open(request,timeout=15) as response:
            receipt = json.loads(response.read(65536))
        if (receipt.get('artifactId') != metadata['artifact_id'] or receipt.get('assetHash') != metadata['asset_hash']
            or receipt.get('stored') is not True):
            raise StepError('MAP_UPLOAD_RECEIPT_MISMATCH')
        return True

    def result(self, metrics):
        return {'sensing_type':'MAPPING','mapping_session_id':self.state['session_id'],
                'metrics':metrics,'captured_at':utcnow(),'evidence_ref':None}

    async def abort(self, reason):
        if not self.state:
            return
        self.remote_control.stop()
        needs_reconciliation = self.state.get('state') == 'RECOVERY_REQUIRED'
        with self.lock:
            self.manual = (0.,0.,0.)
        stopped = await self.adapter.stop()
        if self.process and self.process.returncode is None:
            try:
                os.killpg(self.process.pid,signal.SIGINT)
                await asyncio.wait_for(self.process.wait(),5)
            except asyncio.TimeoutError:
                os.killpg(self.process.pid,signal.SIGKILL)
                await self.process.wait()
        self.process = None
        self.state['session_open'] = not stopped or needs_reconciliation
        if needs_reconciliation:
            self.state['state'] = 'RECOVERY_REQUIRED'
        elif not stopped or self.state.get('state') not in ('SAVED','SAVED_LOCAL'):
            self.state['state'] = reason if stopped else 'RECOVERY_REQUIRED'
        self.persist()
        if not stopped:
            raise StepError('PHYSICAL_STOP_UNCONFIRMED')
