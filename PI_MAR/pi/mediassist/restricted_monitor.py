"""Reuse server pose results and the existing registered depth feed; no second model or camera owner."""
import asyncio
import logging
import math
import time
from uuid import uuid4

from .contracts import StepError
from .fall_depth import measure_person, project_ground
from .navigation_zones import signed_distance


def configuration(value):
    value = {} if value is None else value
    if not isinstance(value,dict) or type(value.get('enabled',False)) is not bool:
        raise StepError('INVALID_SHARED_RESTRICTED_CONFIG')
    result = dict(enabled=value.get('enabled',False))
    for key,default,low,high in (('confirm_sec',.75,.4,5.),('cooldown_sec',30.,10.,600.),('margin_m',.08,.02,.30)):
        v=value.get(key,default)
        if type(v) not in (int,float) or not math.isfinite(v) or not low<=v<=high:
            raise StepError('INVALID_SHARED_RESTRICTED_CONFIG',key)
        result[key]=v
    return result


class SharedRestrictedMonitor:
    def __init__(self,observer):
        self.observer=observer
        self.config=configuration(observer.config.get('restricted_monitor'))
        self.enabled=self.config['enabled'] and observer.config.get('depth') is not None
        self.patrol_job=None;self.context=None;self.candidates={};self.cooldown={}
        self.latest=None;self.report_task=None;self.warning_at=0.
        self.zone_profile=None;self.zone_cache=[]

    def zones(self):
        adapter=self.observer.adapter
        zones=getattr(adapter,'zones',None)
        profile=getattr(adapter,'navigation_profile_hash',None)
        if profile!=self.zone_profile or zones is not getattr(self,'zone_source',None):
            self.zone_source=zones;self.zone_profile=profile;self.zone_cache=[]
            for z in getattr(zones,'zones',[]):
                if z.get('kind')=='KEEP_OUT' and z.get('zone_id'):
                    xs,ys=zip(*z['points'])
                    self.zone_cache.append({**z,'bounds':(min(xs),min(ys),max(xs),max(ys))})
        return self.zone_cache

    def project(self,result,pair,size,captured_at,revision):
        if (result.get('ground_people_version')!=1 or pair is None or revision is None
                or self.observer.current_revision()!=revision):
            return None
        adapter=self.observer.adapter
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            tf=adapter.tf_buffer.lookup_transform(adapter.cfg['ros']['map_frame'],pair[0].header.frame_id,
                                                 Time.from_msg(pair[0].header.stamp))
        except TransformException:return None
        people=result.get('ground_people')
        if not isinstance(people,list) or len(people)>8:return None
        mapped=[]
        for person in people:
            if (not isinstance(person,dict) or (person.get('image_width'),person.get('image_height'))!=size
                    or type(person.get('track_id')) is not int or person['track_id']<0
                    or type(person.get('confidence')) not in (int,float)
                    or not .5<=person['confidence']<=1):continue
            try:
                measured=measure_person(pair,person.get('keypoints'),self.observer.config['depth'])
                location=project_ground(measured,tf.transform,self.observer.config['depth']['floor_z_m']) if measured else None
            except (StepError,ValueError,TypeError):continue
            if location:
                session=self.observer.adapter.worker['navigation_session']
                location.update(capturedAt=captured_at,navigationSessionId=revision[0],initializationRevision=revision[1],
                                mapId=session['map_id'],mapAssetHash=session['map_asset_hash'])
                mapped.append(dict(track_id=person['track_id'],confidence=person['confidence'],location=location))
        # Unlocated people make a negative observation unknown, never clear.
        detections=result.get('overlay',{}).get('detections',[])
        complete=isinstance(detections,list) and len(mapped)==len(detections)
        return mapped,complete,tf.transform

    def update(self,worker,result,pair,size,captured_at,revision):
        if not self.enabled:return
        step=worker.active
        if step and step.get('stepType')=='MOVE' and step.get('params',{}).get('mode')=='PATROL':
            self.patrol_job=step['jobId']
        context=(revision,getattr(self.observer.adapter,'navigation_profile_hash',None),step.get('jobId') if step else None)
        if context!=self.context:self.context=context;self.candidates.clear();self.latest=None
        if (not step or step.get('jobId')!=self.patrol_job or step.get('stepType') not in ('MOVE','EXECUTE_SENSING')
                or not self.zones()):return
        projected=self.project(result,pair,size,captured_at,revision)
        if projected is None:self.candidates.clear();return
        people,complete,transform=projected;now=time.monotonic()
        self.latest=dict(at=now,people=people,complete=complete,pair=pair,transform=transform,captured_at=captured_at,
                         algorithm=result.get('algorithm'),context=context)
        seen=set()
        for zone in self.zones():
            for person in people:
                location=person['location']
                x,y=location['xM'],location['yM'];xmin,ymin,xmax,ymax=zone['bounds']
                if not xmin<x<xmax or not ymin<y<ymax:continue
                key=(step['jobId'],zone['zone_id'],person['track_id']);seen.add(key)
                if signed_distance((location['xM'],location['yM']),zone['points'])>=-self.config['margin_m']:
                    self.candidates.pop(key,None);continue
                first,last,count=self.candidates.get(key,(now,now,0))
                if now-last>1.:first,count=now,0
                self.candidates[key]=(first,now,count+1)
                if now-first<self.config['confirm_sec'] or count<2 or now<self.cooldown.get(zone['zone_id'],0):continue
                if self.report_task is not None and not self.report_task.done():continue
                body=dict(eventId=str(uuid4()),jobId=step['jobId'],jobStepId=step['jobStepId'],detectedAt=captured_at,
                          algorithm=result.get('algorithm'),confidence=person['confidence'],zoneId=zone['zone_id'],
                          trackId=person['track_id'],personLocation=location)
                self.report_task=asyncio.create_task(self.report(body))
                self.cooldown[zone['zone_id']]=now+self.config['cooldown_sec']
        self.candidates={key:value for key,value in self.candidates.items() if key in seen}
        self.cooldown={key:value for key,value in self.cooldown.items() if value>now}

    async def report(self,body):
        for attempt in range(3):
            try:
                await asyncio.to_thread(self.observer.be.request,'/api/v1/pi/restricted-zone-events',body)
                logging.warning('RESTRICTED_REVIEW_REQUESTED event=%s zone=%s track=%s range_m=%.2f',
                                body['eventId'],body['zoneId'],body['trackId'],body['personLocation']['rangeM'])
                return
            except (StepError,OSError,ValueError) as error:
                logging.warning('RESTRICTED_REPORT_RETRY event=%s attempt=%d code=%s',body['eventId'],attempt+1,
                                getattr(error,'code',type(error).__name__))
                if attempt<2:await asyncio.sleep(1.)
        logging.warning('RESTRICTED_REPORT_UNCONFIRMED event=%s',body['eventId'])

    async def observe(self,params):
        """The checkpoint consumes this same inference stream, never loading local ONNX."""
        from .patrol_sensing import zone_coverage_points, zone_visible
        worker=getattr(self.observer,'monitor_worker',None)
        step=worker.active if worker is not None else None
        if (not step or step.get('stepType')!='EXECUTE_SENSING'
                or step.get('params',{}).get('sensing_type')!='RESTRICTED_ZONE'
                or step['params'].get('zone_id')!=params['zone_id']):
            raise StepError('AI_RESTRICTED_CHECKPOINT_CHANGED')
        profile=await asyncio.to_thread(self.observer.be.request,
                '/api/v1/pi/restricted-zone-events/profile?zoneId='+params['zone_id'])
        if isinstance(profile,(bytes,str)):
            import json
            profile=json.loads(profile)
        if isinstance(profile,dict) and 'data' in profile:profile=profile['data']
        if not isinstance(profile,dict) or not profile.get('model_version') or not profile.get('producer_ref'):
            raise StepError('AI_RESTRICTED_PROFILE_UNAVAILABLE')
        if self.observer.monitor_worker is not worker or worker.active is not step:
            raise StepError('AI_RESTRICTED_CHECKPOINT_CHANGED')
        # The BE profile endpoint verifies the current running PATROL checkpoint.
        # Rebind after a process restart even when its resumed step is sensing.
        self.patrol_job=step['jobId']
        started=time.monotonic();deadline=started+params['window_sec']
        zone=next((z for z in self.zones() if z['zone_id']==params['zone_id']),None)
        if zone is None:raise StepError('AI_RESTRICTED_ZONE_NOT_SYNCED')
        coverage=zone_coverage_points(zone['points'],self.observer.config['depth']['floor_z_m'])
        stable=0;last=None;previous=None
        while time.monotonic()<deadline:
            await asyncio.sleep(.05);sample=self.latest
            if self.observer.monitor_worker is not worker or worker.active is not step:
                raise StepError('AI_RESTRICTED_CHECKPOINT_CHANGED')
            if not sample or sample['at']<started or sample['at']==last or time.monotonic()-sample['at']>1.:continue
            last=sample['at']
            if sample['algorithm']!=profile['model_version']:
                raise StepError('AI_RESTRICTED_PROFILE_MODEL_MISMATCH')
            intrusion=any(signed_distance((p['location']['xM'],p['location']['yM']),zone['points'])<-self.config['margin_m'] for p in sample['people'])
            if not intrusion and (not sample['complete'] or not zone_visible(sample['pair'],sample['transform'],coverage,self.observer.config['depth'])):
                stable=0;continue
            stable=stable+1 if intrusion==previous else 1;previous=intrusion
            if stable<2:continue
            return dict(sensing_type='RESTRICTED_ZONE',map_id=self.observer.adapter.worker['navigation_session']['map_id'],
                        zone_id=params['zone_id'],frame_id=self.observer.adapter.cfg['ros']['map_frame'],
                        model_version=profile.get('model_version') or sample['algorithm'],
                        producer_ref=profile.get('producer_ref') or 'SHARED_POSE_RGBD',captured_at=sample['captured_at'],
                        metrics={'intrusion_detected':intrusion},evidence_ref=None)
        raise StepError('AI_DATA_UNAVAILABLE','Restricted zone is only partially visible or has no verified ground sample',retryable=True)

    async def close(self):
        if self.report_task is not None:
            self.report_task.cancel();await asyncio.gather(self.report_task,return_exceptions=True)
