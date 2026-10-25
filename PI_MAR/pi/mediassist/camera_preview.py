"""Sample RGB ROS images; send bounded JPEG thumbnails only during mapping."""
import asyncio
import base64
import logging
import threading
import time
from .store import utcnow

def encode_jpeg(sample, cfg):
    import cv2
    import numpy as np
    data,width,height,step,encoding,captured,received = sample
    channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1}[encoding]
    pixels=np.frombuffer(data,dtype=np.uint8).reshape(height,step)[:,:width*channels]
    frame=pixels.reshape(height,width,channels) if channels>1 else pixels.reshape(height,width)
    colors={'rgb8':cv2.COLOR_RGB2BGR,'rgba8':cv2.COLOR_RGBA2BGR,'bgra8':cv2.COLOR_BGRA2BGR}
    if encoding in colors:frame=cv2.cvtColor(frame,colors[encoding])
    scale=min(1.,cfg['max_width']/width,cfg['max_height']/height)
    if scale<1:frame=cv2.resize(frame,(max(1,int(width*scale)),max(1,int(height*scale))),interpolation=cv2.INTER_AREA)
    for quality in (cfg['jpeg_quality'],30,20):
        ok,encoded=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,quality])
        if ok and len(encoded)<=cfg['max_jpeg_bytes']:
            return {'captured_at':captured,'width':frame.shape[1],'height':frame.shape[0],
                    'jpeg':encoded.tobytes()}
    return None


def encode_frame(sample, cfg):
    frame = encode_jpeg(sample, cfg)
    if frame is None:
        return None
    jpeg = frame.pop('jpeg')
    return {**frame, 'encoding':'jpeg-base64', 'data':base64.b64encode(jpeg).decode('ascii')}

class CameraPreview:
    def __init__(self,adapter,cfg,active,on_receive=None):
        from sensor_msgs.msg import Image
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.cfg,self.active=cfg,active
        self.on_receive=on_receive
        self.source_clock=adapter.node.get_clock() if hasattr(adapter.node,'get_clock') else None
        self.lock=threading.Lock();self.sample=None;self.last_sample=0.;self.revision=0;self.error=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        adapter.node.create_subscription(Image,cfg['topic'],self.on_image,qos)

    def on_image(self,msg):
        now=time.monotonic()
        if not self.active():return
        if not self.cfg.get('sample_every_frame',False) and now-self.last_sample<1/self.cfg['rate_hz']:return
        self.last_sample=now
        channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1}.get(msg.encoding)
        if not channels or not 0<msg.width*msg.height<=4000000 or msg.step<msg.width*channels or len(msg.data)!=msg.step*msg.height or len(msg.data)>16000000:
            if self.error!='CAMERA_FORMAT_UNSUPPORTED':logging.warning('Camera preview: unsupported color encoding or size: %s',msg.encoding)
            self.error='CAMERA_FORMAT_UNSUPPORTED';return
        captured=utcnow()
        # Use camera acquisition timestamps when available. A delayed DDS frame
        # must not look fresh simply because its Python callback just ran.
        clock=getattr(self,'source_clock',None)
        header=getattr(msg,'header',None)
        if clock is not None and header is not None:
            from datetime import datetime,timezone
            stamp=header.stamp.sec*1_000_000_000+header.stamp.nanosec
            age=(clock.now().nanoseconds-stamp)/1e9
            if not -.1<=age<=self.cfg.get('source_max_age_sec',.35):
                self.error='CAMERA_SOURCE_STALE'
                return
            captured=datetime.fromtimestamp(stamp/1e9,timezone.utc).isoformat()
        sample=(bytes(msg.data),msg.width,msg.height,msg.step,msg.encoding,captured,now)
        with self.lock:self.sample=sample
        self.error=None
        observer=getattr(self,'on_receive',None)
        if observer:observer(msg.width,msg.height,now)

    async def preview(self,session_id):
        with self.lock:sample=self.sample;self.sample=None
        if not sample or time.monotonic()-sample[-1]>3:return None
        try:
            frame=await asyncio.to_thread(encode_frame,sample,self.cfg)
            if not frame:return None
            self.revision+=1;self.error=None
            return {**frame,'mapping_session_id':session_id,'revision':self.revision}
        except Exception as exc:
            # A malformed camera frame must not terminate the motion/SLAM session.
            if self.error!=type(exc).__name__:logging.warning('Camera preview unavailable: %s',type(exc).__name__)
            self.error=type(exc).__name__;return None
