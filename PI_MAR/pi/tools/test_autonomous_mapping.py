"""Deterministic autonomy/transport tests; no ROS, GPIO, network or motor access."""
import copy, math, sys, unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.autonomous_mapping import AutoMapper, configuration, scan_view
from mediassist.contracts import StepError
from mediassist.mapping_control import VALIDATOR

POSE=dict(x_m=0.,y_m=0.,yaw_rad=0.)
CLEAR=dict(valid=True,front_m=3.,left_m=3.,right_m=3.,rotation_clear=True)
RUN='11111111-1111-4111-8111-111111111111'


def settings():
    return configuration({'autonomy':dict(enabled=True,footprint=dict(front=.20,rear=.20,left=.15,right=.15))})


class AutonomyTests(unittest.TestCase):
    def setUp(self):
        self.auto=AutoMapper(settings())
        self.message=dict(action='START',runId=RUN,maxDurationSec=180,stopOnReturn=False)

    def start(self):self.auto.accept(self.message,0.,POSE,.9)
    def current(self,t,**kwargs):
        return self.auto.current(t,kwargs.pop('view',CLEAR),kwargs.pop('age',.01),kwargs.pop('pose',POSE),kwargs.pop('allowed',True),**kwargs)

    def test_configuration_requires_measured_geometry_and_finite_bounds(self):
        self.assertFalse(configuration({})['enabled'])
        for value in ({'enabled':True},{'enabled':True,'footprint':dict(front=float('nan'),rear=.2,left=.2,right=.2)}, {'speed_mps':float('inf')}):
            with self.assertRaises(StepError):configuration({'autonomy':value})

    def test_start_renew_and_stop_do_not_restart_finished_run(self):
        self.start();self.assertGreater(self.current(.1)[0],0)
        self.auto.accept({**self.message,'action':'RENEW'},.5,POSE,.9)
        self.assertGreater(self.current(1.)[0],0)
        self.assertEqual(self.current(1.41),(0.,0.))
        self.assertEqual(self.auto.reason,'CONTROL_LEASE_EXPIRED')
        for action in ('RENEW','START'):
            with self.assertRaises(StepError):self.auto.accept({**self.message,'action':action},1.5,POSE,.9)

    def test_duplicate_start_does_not_reset_total_timer_and_options_cannot_change(self):
        self.start();self.auto.accept(self.message,.4,POSE,.9)
        self.assertEqual(self.auto.start_at,0.)
        with self.assertRaises(StepError):self.auto.accept({**self.message,'action':'RENEW','stopOnReturn':True},.5,POSE,.9)

    def test_stop_works_after_sensor_failure_without_sensors(self):
        self.start();self.current(.1,view={'valid':False,'reason':'LIDAR_INVALID'})
        self.assertEqual(self.auto.reason,'LIDAR_INVALID')
        self.auto.accept({**self.message,'action':'STOP'},.2,None,0.)
        self.assertEqual(self.auto.reason,'LIDAR_INVALID')
        self.assertFalse(self.auto.status(.3)['coverage_complete'])

    def test_time_limit_sensor_gate_pose_and_connection_failures_stop(self):
        for args,reason in [({'age':1.},'LIDAR_STALE'),({'pose':None},'MAPPING_POSE_NOT_READY'),
                            ({'allowed':False},'NOT_READY'),({'gate':{'reason':'SIDE_OBSTACLE'}},'GATE_SIDE_OBSTACLE'),
                            ({'gate':[]},'GATE_INVALID_STATUS'),({'pose':{**POSE,'x_m':2}},'MAP_POSE_JUMP')]:
            self.setUp();self.start();self.assertEqual(self.current(.1,**args),(0.,0.));self.assertEqual(self.auto.reason,reason)
        self.setUp();self.message['maxDurationSec']=10;self.start()
        for i in range(1,21):self.auto.accept({**self.message,'action':'RENEW'},i*.5,POSE,.9)
        self.current(10.1);self.assertEqual(self.auto.reason,'TIME_LIMIT')

    def test_clear_corridor_turn_choice_and_rotation_envelope(self):
        self.start()
        x,z=self.current(.1,view={**CLEAR,'front_m':.35,'left_m':2.,'right_m':1.})
        self.assertEqual(x,0);self.assertGreater(z,0)
        self.assertEqual(self.current(.2,view={**CLEAR,'front_m':.35,'rotation_clear':False}),(0.,0.))
        self.assertEqual(self.auto.reason,'ROTATION_ENVELOPE_BLOCKED')

    def test_return_requires_departure_time_travel_and_heading(self):
        self.message['stopOnReturn']=True;self.start();self.current(.1)
        self.assertEqual(self.auto.phase,'RUNNING')
        self.auto.cfg['min_return_sec']=.1;self.auto.cfg['min_return_travel_m']=1.
        for i in range(1,17):
            t=i*.2;self.auto.accept({**self.message,'action':'RENEW'},t,POSE,.9)
            x=i*.1 if i<=8 else (16-i)*.1
            self.current(t,pose={**POSE,'x_m':x})
            if self.auto.phase=='STOPPED':break
        self.assertEqual(self.auto.reason,'RETURNED_TO_START')
        self.assertFalse(self.auto.status(100)['coverage_complete'])

    def test_scan_geometry_coverage_inf_and_obstacles(self):
        scan=SimpleNamespace(angle_min=-math.pi,angle_increment=math.tau/360,range_min=.05,range_max=8.,ranges=[float('inf')]*360)
        self.assertTrue(scan_view(scan,(.05,0,0),settings())['valid'])
        scan.ranges[180]=.55
        view=scan_view(scan,(0.,0.,0.),settings())
        self.assertAlmostEqual(view['front_m'],.35);self.assertTrue(view['rotation_clear'])
        scan.ranges[180]=.25
        self.assertFalse(scan_view(scan,(0,0,0),settings())['rotation_clear'])
        scan.ranges=[float('nan')]*360
        self.assertFalse(scan_view(scan,(0,0,0),settings())['valid'])
        scan.ranges=[float('inf')]*180
        self.assertFalse(scan_view(scan,(0,0,0),settings())['valid'])

    def test_rear_hit_is_not_a_front_obstacle_but_still_blocks_rotation(self):
        scan=SimpleNamespace(angle_min=-math.pi,angle_increment=math.tau/360,range_min=.05,range_max=8.,ranges=[float('inf')]*360)
        scan.ranges[0]=.15
        view=scan_view(scan,(0,0,0),settings())
        self.assertGreater(view['front_m'],1.)
        self.assertFalse(view['rotation_clear'])
        self.assertAlmostEqual(view['nearest_lidar_m'],.15)
        scan.ranges[0]=float('inf');scan.ranges[180]=.25
        self.assertAlmostEqual(scan_view(scan,(0,0,0),settings())['front_m'],.05)

    def test_ten_centimetres_stops_from_any_direction_even_during_plan_hold(self):
        for index in (0,90,180,270):
            self.setUp();self.start()
            scan=SimpleNamespace(angle_min=-math.pi,angle_increment=math.tau/360,range_min=.05,range_max=8.,ranges=[float('inf')]*360)
            scan.ranges[index]=.10
            view=scan_view(scan,(.03,-.01,.8),settings())
            self.assertEqual(self.current(.1,view=view,hold=True),(0.,0.))
            self.assertEqual(self.auto.reason,'LIDAR_OBSTACLE_WITHIN_10CM')
            self.assertFalse(self.auto.status(.1)['coverage_complete'])
        self.setUp();self.start()
        self.assertGreater(self.current(.1,view={**CLEAR,'nearest_lidar_m':.101})[0],0.)

    def test_invalid_returns_are_not_counted_as_valid_nearby_obstacles(self):
        scan=SimpleNamespace(angle_min=-math.pi,angle_increment=math.tau/360,range_min=.05,range_max=8.,ranges=[float('inf')]*360)
        scan.ranges[:5]=[.01,-1.,float('nan'),float('-inf'),9.]
        view=scan_view(scan,(0,0,0),settings())
        self.assertTrue(view['valid']);self.assertEqual(view['nearest_lidar_m'],8.)
        scan.ranges=[float('nan')]*360
        self.assertFalse(scan_view(scan,(0,0,0),settings())['valid'])

    def test_articulated_trailer_uses_only_forward_arc_and_stops_if_no_clear_arc(self):
        self.auto.cfg['vehicle']='articulated';self.start()
        x,z=self.current(.1,view={**CLEAR,'front_m':.35,'arc_left_clear':True,'arc_right_clear':False})
        self.assertGreater(x,0);self.assertGreater(z,0)
        self.assertGreaterEqual(x/abs(z),self.auto.cfg['min_turn_radius_m'])
        self.assertEqual(self.current(.2,view={**CLEAR,'front_m':.35,'arc_left_clear':False,'arc_right_clear':False}),(0.,0.))
        self.assertEqual(self.auto.reason,'TRAILER_FORWARD_ARC_BLOCKED')

    def test_schema_bounds_new_lane_and_keeps_old_manual(self):
        from datetime import datetime,timezone,timedelta
        now=datetime.now(timezone.utc)
        m=dict(messageType='MAPPING_CONTROL',robotId=RUN,transportSessionId=RUN,jobId=RUN,mappingSessionId=RUN,
               commandId=RUN,sequenceNo=1,issuedAt=now.isoformat(),expiresAt=(now+timedelta(seconds=1)).isoformat(),
               linearX=.05,angularZ=0.,durationMs=900)
        self.assertTrue(VALIDATOR.is_valid(m))
        m.update(linearX=0.,autonomy=self.message)
        self.assertTrue(VALIDATOR.is_valid(m))
        m['autonomy']={**self.message,'maxDurationSec':601}
        self.assertFalse(VALIDATOR.is_valid(m))


class MappingIntegrationTests(unittest.IsolatedAsyncioTestCase):
    def manager(self):
        import test_gamepad as fixtures
        from unittest.mock import Mock
        fixture=fixtures.MappingTests()
        item=fixture.manager()
        item.auto=AutoMapper(settings());item.auto_selected=False
        item.cfg['max_map_age_sec']=10
        item.persist=Mock();item.grid=({},10.);item.revision=1
        item.auto_scan=(CLEAR,10.)
        item.adapter.telemetry=lambda:{'odom_speed':{'age_sec':.01},
            'map_pose':{'fresh':True,'value':{**POSE,'map_id':None,'mapping_session_id':item.state['session_id']}},
            'gate':{'value':'{"reason":"CLEAR"}'}}
        return item,fixture

    async def test_start_tick_stop_and_terminal_latch_share_one_motion_publisher(self):
        import sys
        from unittest.mock import patch
        from types import SimpleNamespace as NS
        item,fixture=self.manager()
        packet=fixture.stop_packet(durationMs=900,autonomy=dict(action='START',runId=RUN,maxDurationSec=180,stopOnReturn=False))
        def twist():return NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))
        with patch('mediassist.mapping.time.monotonic',return_value=10.),patch.dict(sys.modules,{
                'geometry_msgs.msg':NS(Twist=twist),'std_msgs.msg':NS(String=lambda **kw:NS(**kw))}):
            item.receive_control(packet,packet['transportSessionId'],True,True)
            self.assertEqual(set(item.auto.start_pose),{'x_m','y_m','yaw_rad'})
            await item.tick(True,True)
            self.assertTrue(item.auto_selected)
            self.assertGreater(item.velocity.publish.call_args.args[0].linear.x,0.)
            self.assertTrue(item.adapter.lease_message.call_args.args[0])
            item.auto.stop('TIME_LIMIT')
            await item.tick(True,True)
            self.assertEqual(item.velocity.publish.call_args.args[0].linear.x,0.)
            self.assertFalse(item.adapter.lease_message.call_args.args[0])
            self.assertTrue(item.auto_selected,'A terminal run cannot fall back to held joystick input')
            item.receive_control(fixture.stop_packet(sequenceNo=2),packet['transportSessionId'],False,False)
            self.assertFalse(item.auto_selected)
            self.assertEqual(item.auto.reason,'TIME_LIMIT')

    async def test_stale_map_or_joystick_override_stops_auto_without_auto_resume(self):
        import sys
        from unittest.mock import patch
        from types import SimpleNamespace as NS
        for cause in ('map','joystick'):
            item,fixture=self.manager()
            item.auto.accept(dict(action='START',runId=RUN,maxDurationSec=180,stopOnReturn=False),10.,POSE,.9)
            item.auto_selected=True
            if cause=='map':item.grid=({},1.)
            else:item.gamepad.axes=(1.,0.)
            with patch('mediassist.mapping.time.monotonic',return_value=10.1),patch.dict(sys.modules,{
                'geometry_msgs.msg':NS(Twist=lambda:NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))),
                'std_msgs.msg':NS(String=lambda **kw:NS(**kw))}):
                await item.tick(True,True)
            self.assertEqual(item.auto.reason,'MAPPING_MAP_STALE' if cause=='map' else 'GAMEPAD_OVERRIDE')
            self.assertEqual(item.velocity.publish.call_args.args[0].linear.x,0.)

    async def test_blocked_frontier_plan_holds_and_reports_map_cause_before_reactive_front_check(self):
        from unittest.mock import Mock,patch
        item,_=self.manager()
        item.auto.cfg['exploration_mode']='frontier'
        item.auto.accept(dict(action='START',runId=RUN,maxDurationSec=180,stopOnReturn=False),10.,POSE,.9)
        item.auto_selected=True
        item.auto_scan=({**CLEAR,'front_m':-.1,'nearest_lidar_m':.2},10.)
        item.exploration=Mock()
        item.exploration.poll.return_value=({'state':'BLOCKED','reason':'ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP'},None)
        item.auto_status=Mock(return_value={})
        def twist():return SimpleNamespace(linear=SimpleNamespace(x=0.,y=0.,z=0.),angular=SimpleNamespace(x=0.,y=0.,z=0.))
        with patch('mediassist.mapping.time.monotonic',return_value=10.1),patch.dict(sys.modules,{
                'geometry_msgs.msg':SimpleNamespace(Twist=twist)}):
            await item.tick(True,True)
        self.assertEqual(item.auto.reason,'ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP')
        self.assertEqual(item.velocity.publish.call_args.args[0].linear.x,0.)
        self.assertFalse(item.adapter.lease_message.call_args.args[0])


if __name__=='__main__':unittest.main()
