"""Offline saved-map regression. No ROS process, network, GPIO or motor output."""
import asyncio,copy,math,sys,threading,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock,AsyncMock,patch
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'pi/tools')]
from mediassist.localization import QualityGate
from mediassist.ros_adapter import RosAdapter
from mediassist.navigation_session import command_context
from mediassist.navigation_setup import nav2_params
from mediassist.contracts import StepError
import test_localization as loc
import test_navigation_session as session

class TrackingTests(unittest.TestCase):
    def test_only_known_plan_commit_races_are_retryable(self):
        import io,json,urllib.error
        from mediassist.navigation_session import SessionClient
        for message,retry in [('Navigation plan session or localization changed',True),
                              ('Navigation plan does not belong to the active MOVE step',True),
                              ('Navigation plan endpoint differs from MOVE target',False),('Other error',False)]:
            client=object.__new__(SessionClient);client.cfg={'robot_id':'robot'};client.auth={};client.origin='https://example.test'
            client.opener=Mock();client.opener.open.side_effect=urllib.error.HTTPError(client.origin,422,'',{},io.BytesIO(json.dumps({'message':message}).encode()))
            with patch('mediassist.navigation_session.upload_token',return_value='test'),self.assertRaises(StepError) as raised:
                client.request('/api/v1/pi/navigation-session/plan',{})
            self.assertEqual(raised.exception.retryable,retry)

    def test_partial_map_does_not_reset_a_verified_wall_match(self):
        gate=self.locked()
        for i in range(8,48):
            sample=loc.evidence(i);sample['known_fraction']=.35
            result=gate.update(i*.5,sample)
            self.assertTrue(result['ready']);self.assertIn('KNOWN_FRACTION_LOW',result['warnings'])
        initial=QualityGate(loc.cfg())
        for i in range(12):self.assertFalse(initial.update(i*.5,sample)['ready'])

    def test_quality_grace_expires_and_never_masks_hard_loss(self):
        gate=self.locked();sample=loc.evidence(8);sample['scan_match']=.5
        self.assertTrue(gate.update(3.6,sample)['ready'])
        self.assertTrue(gate.update(4.3,sample)['ready'])
        self.assertFalse(gate.update(4.6,sample)['ready'])
        for key,value in [('tf_fresh',False),('scan_age_sec',2.),('odom_age_sec',2.),('body_clear',False),('scan_match',.1)]:
            gate=self.locked();bad={**sample,key:value}
            with self.subTest(key=key):self.assertFalse(gate.update(3.6,bad)['ready'])

    def test_amcl_comparison_uses_same_timestamp_without_hiding_jump(self):
        gate=self.locked();sample=loc.evidence(8)
        sample['pose']['yaw_rad']=.45
        sample['amcl_tf_pose']=dict(sample['amcl_pose'])
        self.assertTrue(gate.update(4.,sample)['ready'])
        sample['pose']['yaw_rad']=2.
        self.assertIn('POSE_JUMP',gate.update(4.1,sample)['reasons'])

    def test_gate_never_emits_sub_minimum_yaw_or_amplifies_a_command(self):
        from mediassist.safety import gate_velocity
        config=dict(commissioned=True,cart_footprint_validated=True,watchdog_validated=True,
                    lease_timeout_sec=1.,command_timeout_sec=1.,sensor_max_age_sec=1.,
                    us100_required=False,max_angular_speed_rps=.8)
        for z in (-.8,-.1,-.099,-.033,0.,.033,.099,.1,.8):
            cmd,reason=gate_velocity((.05,0.,z),[],10.,10.,10.,10.,config,.25)
            self.assertEqual(reason,'CLEAR');self.assertTrue(cmd[2]==0 or abs(cmd[2])>=.1)
            self.assertLessEqual(abs(cmd[2]),abs(z))
        self.assertEqual(gate_velocity((0.,0.,.8),[],10.,0.,0.,0.,config,.25)[0],(0.,0.,0.))
    def locked(self):
        gate=QualityGate(loc.cfg())
        for i in range(8):status=gate.update(i*.5,loc.evidence(i))
        self.assertTrue(status['ready']);return gate
    def test_short_loss_stops_immediately_then_reacquires_and_stays_ready(self):
        gate=self.locked();bad=loc.evidence(8);bad['scan_age_sec']=1.
        self.assertFalse(gate.update(3.6,bad)['ready'])
        self.assertFalse(gate.update(3.7,loc.evidence(9))['ready'])
        self.assertTrue(gate.update(4.25,loc.evidence(10))['ready'])
        for i in range(11,18):self.assertTrue(gate.update(4.3+(i-11)*.1,loc.evidence(i))['ready'])
    def test_uncertain_or_long_loss_requires_full_verification(self):
        for hard in (True,False):
            gate=self.locked();bad=loc.evidence(8)
            if hard:bad['covariance'][0]=1.
            else:bad['scan_age_sec']=1.
            self.assertFalse(gate.update(3.6,bad)['ready'])
            start=3.7 if hard else 6.
            self.assertFalse(gate.update(start,loc.evidence(9))['ready'])
            self.assertFalse(gate.update(start+.6,loc.evidence(10))['ready'])
    def test_pause_does_not_authorize_new_command_or_changed_identity(self):
        context={k:session.CTX[k] for k in ('session_id','map_id','map_asset_hash')}
        context.update(initialization_revision=2,profile_hash='b'*64)
        local=dict(ready=False,age_sec=.1,initialization_revision=2)
        a=N(telemetry=lambda:{'localization':local},navigation_profile_hash='b'*64)
        cfg={'navigation_session':session.CTX}
        self.assertFalse(command_context({'navigationContext':context},cfg,{},a,require_localization=False))
        with self.assertRaisesRegex(StepError,'LOCALIZATION_NOT_READY'):command_context({'navigationContext':context},cfg,{},a)
        for field,value in [('session_id',session.ROBOT),('map_id',session.ROBOT),('map_asset_hash','c'*64),
                            ('initialization_revision',1),('profile_hash','c'*64)]:
            with self.subTest(field=field),self.assertRaises(StepError):
                command_context({'navigationContext':{**context,field:value}},cfg,{},a,require_localization=False)
        expired=copy.deepcopy(cfg);expired['navigation_session']={**session.CTX,'expires_at':'2020-01-01T00:00:00Z'}
        with self.assertRaises(StepError):command_context({'navigationContext':context},expired,{},a,require_localization=False)
    def test_speed_limit_uses_controller_absolute_units_and_rejects_invalid_caps(self):
        a=object.__new__(RosAdapter);a.cfg={'max_speed_mps':.25};a.navigation_speed_output=Mock()
        a.node=N(get_clock=lambda:N(now=lambda:N(to_msg=lambda:'stamp')))
        def message():return N(header=N(stamp=None),percentage=None,speed_limit=None)
        with patch.dict(sys.modules,{'nav2_msgs.msg':N(SpeedLimit=message)}):
            a.navigation_speed_limit(.15)
            m=a.navigation_speed_output.publish.call_args.args[0]
            self.assertIs(m.percentage,False);self.assertEqual(m.speed_limit,.15)
            for speed in (0.,-.1,math.nan,.3,True):
                with self.assertRaises(StepError):a.navigation_speed_limit(speed)
    def test_resume_ramps_from_zero_and_stale_velocity_still_stops_immediately(self):
        a=object.__new__(RosAdapter);a.lock=threading.Lock();a.nav_velocity_sample=((.15,0.,.3),10.)
        a.cfg={'max_speed_mps':.25,'motion':{'allow_strafe':True,'max_angular_speed_rps':.6}}
        a.navigation_velocity_output=Mock();a.lease_message=Mock()
        def twist():return N(linear=N(x=0.,y=0.),angular=N(z=0.))
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=twist)}),patch('mediassist.ros_adapter.time.monotonic',return_value=10.):
            a.hold_navigation_velocity();a.lease_message.assert_called_once_with(False)
            self.assertEqual(a.navigation_velocity_output.publish.call_args.args[0].linear.x,0.)
            a.nav_velocity_sample=((.15,0.,.3),10.)
            self.assertTrue(a.publish_navigation_velocity(.15))
            m=a.navigation_velocity_output.publish.call_args.args[0]
            self.assertGreater(m.linear.x,0.);self.assertLessEqual(m.linear.x,.021)
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=twist)}),patch('mediassist.ros_adapter.time.monotonic',return_value=10.3):
            self.assertFalse(a.publish_navigation_velocity(.15))
            self.assertEqual(a.navigation_velocity_output.publish.call_args.args[0].linear.x,0.)
    def test_astar_retains_unknown_obstacle_polygon_and_clears_moving_obstacles(self):
        base={'amcl':{'ros__parameters':{}},'velocity_smoother':{'ros__parameters':{}},
              'planner_server':{'ros__parameters':{'GridBased':{'plugin':'nav2_navfn_planner/NavfnPlanner'}}},
              'bt_navigator':{'ros__parameters':{'plugin_lib_names':['vendor_plugin']}}}
        for name in ('local_costmap','global_costmap'):
            base[name]={name:{'ros__parameters':{'plugins':['obstacle_layer','inflation_layer'],
                'inflation_layer':{},'obstacle_layer':{'observation_sources':'scan','scan':{
                    'topic':'/scan_raw','marking':True,'clearing':True,'raytrace_max_range':2.,'obstacle_max_range':2.5}}}}}
        ctl={'controller_server':{'ros__parameters':{'FollowPath':{'plugin':'dwb_core::DWBLocalPlanner','critics':['BaseObstacle']},'progress_checker':{},'general_goal_checker':{}}}}
        polygon=[[.12,.1],[-.12,.1],[-.12,-.1],[.12,-.1]]
        out=nav2_params(base,ctl,polygon,.15,.6,mecanum=True)
        planner=out['planner_server']['ros__parameters']['GridBased']
        self.assertTrue(planner['use_astar']);self.assertFalse(planner['allow_unknown'])
        for name in ('local_costmap','global_costmap'):
            cm=out[name][name]['ros__parameters'];scan=cm['obstacle_layer']['scan']
            self.assertIn('obstacle_layer',cm['plugins']);self.assertTrue(scan['clearing']);self.assertTrue(scan['marking'])
            self.assertGreater(scan['raytrace_max_range'],scan['obstacle_max_range']);self.assertIn('footprint',cm)
        self.assertIn('vendor_plugin',out['bt_navigator']['ros__parameters']['plugin_lib_names'])
        follow=out['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(follow['PathAlign.scale'],0.)
        self.assertEqual(follow['GoalAlign.scale'],0.)
        self.assertEqual(follow['min_speed_theta'],.1)
        self.assertEqual(follow['vtheta_samples'],13)
        self.assertGreater(follow['min_speed_xy'],0.)
        self.assertNotIn('use_astar',base['planner_server']['ros__parameters']['GridBased'])
    def test_behavior_tree_retries_are_bounded_without_blind_motion_recovery(self):
        import xml.etree.ElementTree as ET
        tree=ET.parse(ROOT/'pi/config/navigate_no_recovery.xml')
        self.assertEqual([n.attrib['number_of_retries'] for n in tree.findall('.//RecoveryNode')],['2','1'])
        for tag in ('Spin','BackUp','DriveOnHeading'):self.assertEqual(tree.findall('.//'+tag),[])
        self.assertEqual(len(tree.findall('.//ClearEntireCostmap')),2)

class MoveTests(unittest.IsolatedAsyncioTestCase):
    async def test_plan_heartbeat_race_retries_same_path_without_dispatching_motion(self):
        a=object.__new__(RosAdapter);a.worker={};a._navigation_context=Mock()
        body={'sessionId':'same','points':[1,2]}
        request=Mock(side_effect=[StepError('NAVIGATION_SESSION_HTTP_422',retryable=True),b'{"accepted":true}'])
        with patch('mediassist.navigation_session.SessionClient',return_value=N(request=request)),patch('mediassist.ros_adapter.asyncio.sleep',new=AsyncMock()):
            await a._publish_plan(body)
        self.assertEqual(request.call_count,2)
        self.assertEqual(request.call_args_list[0],request.call_args_list[1])
        request=Mock(side_effect=StepError('NAVIGATION_SESSION_HTTP_422'))
        with patch('mediassist.navigation_session.SessionClient',return_value=N(request=request)):
            with self.assertRaises(StepError):await a._publish_plan(body)
        self.assertEqual(request.call_count,1)

    async def test_wait_for_nav2_server_is_bounded_and_holds_wheels(self):
        for ready in (True,False):
            clock=[0.];a=object.__new__(RosAdapter);a.hold_navigation_velocity=Mock()
            client=N(server_is_ready=lambda:ready and clock[0]>=.3)
            async def sleep(_):clock[0]+=.1
            with patch('mediassist.ros_adapter.time.monotonic',side_effect=lambda:clock[0]),patch('mediassist.ros_adapter.asyncio.sleep',side_effect=sleep):
                if ready:await a._wait_navigation_server(client,'PLANNER_UNAVAILABLE')
                else:
                    with self.assertRaisesRegex(StepError,'PLANNER_UNAVAILABLE'):await a._wait_navigation_server(client,'PLANNER_UNAVAILABLE')
            self.assertGreater(a.hold_navigation_velocity.call_count,0);self.assertLess(clock[0],8.2)
    async def test_slow_event_replay_yields_to_due_heartbeat(self):
        from mediassist.step_worker import StepWorker
        clock=[10.];sent=[];store=Mock()
        store.transient.side_effect=lambda kind,body:{'messageType':kind}
        store.pending.return_value=[{'messageType':'STEP_RESULT','messageId':str(i)} for i in range(8)]
        a=N(mapping=None,telemetry=lambda:{})
        w=StepWorker({'heartbeat_sec':1.},{'capabilities':[],'map_id':'map','map_asset_sha256':'hash'},store,a)
        w.status=Mock(return_value='BUSY')
        async def send(event):
            sent.append(event['messageType'])
            if len(sent)==3:raise asyncio.CancelledError()
            clock[0]+=1.1 if event['messageType']=='STEP_RESULT' else 0.
        async def sleep(_):clock[0]+=.2
        with patch('mediassist.step_worker.time.monotonic',side_effect=lambda:clock[0]),patch('mediassist.step_worker.asyncio.sleep',side_effect=sleep):
            with self.assertRaises(asyncio.CancelledError):await w.publish(send)
        self.assertEqual(sent,['HEARTBEAT','STEP_RESULT','HEARTBEAT'])

    async def run_move(self,blocked=lambda t:False,finish=.9,velocity=lambda t:True,context_error=False):
        clock=[0.];a=object.__new__(RosAdapter)
        a.cfg={'max_speed_mps':.25,'endpoints':{'endpoint':dict(x_m=0.,y_m=0.,yaw_rad=0.)}}
        a.worker={'navigation_runtime':{'enabled':True}};a.localization_required=True
        a._prerequisites=AsyncMock();a._check_graph=Mock();a._pose=Mock(return_value='endpoint')
        a.clear_navigation_velocity=Mock();a.navigation_speed_limit=Mock();a.lease_message=Mock()
        a.hold_navigation_velocity=Mock();a.stop=AsyncMock(return_value=True)
        a.navigation_diagnostic=Mock()
        result=N(done=lambda:clock[0]>=finish,result=lambda:N(status=4))
        handle=N(accepted=True,get_result_async=lambda:result)
        a.nav=N(server_is_ready=lambda:True,send_goal_async=Mock(return_value=Mock()))
        a.future=AsyncMock(return_value=handle)
        def context(**kwargs):
            if context_error and clock[0]>=.2:raise StepError('LOCALIZATION_REVISION_CHANGED')
            return not blocked(clock[0])
        a._navigation_context=context
        a.telemetry=lambda:{'map_pose':{'fresh':not blocked(clock[0]),'value':dict(x_m=0.,y_m=0.,yaw_rad=0.)},'gate':{'age_sec':0.}}
        a.publish_navigation_velocity=Mock(side_effect=lambda speed:velocity(clock[0]))
        async def sleep(_):clock[0]+=.05
        with patch.dict(sys.modules,{'nav2_msgs.action':N(NavigateToPose=N(Goal=lambda:N()))}),\
             patch('mediassist.ros_adapter.time.monotonic',side_effect=lambda:clock[0]),\
             patch('mediassist.ros_adapter.asyncio.sleep',side_effect=sleep):
            try:return await a._move_one({'max_speed_mps':.15},'endpoint'),a
            finally:self.assertEqual(a.stop.await_count,1)
    async def test_short_quality_loss_keeps_same_goal_with_output_held(self):
        result,a=await self.run_move(blocked=lambda t:.2<=t<.45)
        self.assertTrue(result['reached']);self.assertEqual(a.nav.send_goal_async.call_count,1)
        self.assertGreater(a.hold_navigation_velocity.call_count,0)
    async def test_long_quality_loss_aborts_and_revision_change_cannot_resume(self):
        with self.assertRaisesRegex(StepError,'LOCALIZATION_LOST'):await self.run_move(blocked=lambda t:t>=.2,finish=10.)
        with self.assertRaisesRegex(StepError,'LOCALIZATION_REVISION_CHANGED'):await self.run_move(context_error=True)
    async def test_planning_gap_keeps_lease_disabled_then_recovers(self):
        result,a=await self.run_move(velocity=lambda t:not .2<=t<.6)
        self.assertTrue(result['reached']);self.assertGreater(a.lease_message.call_args_list.count(unittest.mock.call(False)),1)
    async def test_persistent_missing_velocity_does_not_keep_motion_authorized(self):
        with self.assertRaisesRegex(StepError,'NAV2_COMMAND_STALE'):await self.run_move(velocity=lambda t:False,finish=10.)

    async def test_startup_wait_yields_without_sending_any_goal_and_is_bounded(self):
        for recover in (True,False):
            clock=[0.];a=object.__new__(RosAdapter);a.motion_uncertain=False;a.localization_required=True
            a.cfg={'max_speed_mps':.25,'motion':dict(commissioned=True,cart_footprint_validated=True,watchdog_validated=True)}
            a._check_graph=Mock();a._check_map=AsyncMock();a.health=lambda:{'sensors_ready':True}
            a._navigation_context=Mock(return_value=True);a.hold_navigation_velocity=Mock()
            a.telemetry=lambda:{'map_pose':{'fresh':recover and clock[0]>=.3}}
            a.nav=Mock();a.planner=Mock()
            async def sleep(_):clock[0]+=.05
            with patch('mediassist.ros_adapter.time.monotonic',side_effect=lambda:clock[0]),patch('mediassist.ros_adapter.asyncio.sleep',side_effect=sleep):
                if recover:await a._prerequisites({'max_speed_mps':.15,'map_id':'map'})
                else:
                    with self.assertRaisesRegex(StepError,'LOCALIZATION_NOT_READY'):
                        await a._prerequisites({'max_speed_mps':.15,'map_id':'map'})
            self.assertGreater(a.hold_navigation_velocity.call_count,0)
            a.nav.send_goal_async.assert_not_called();a.planner.send_goal_async.assert_not_called()
            self.assertLess(clock[0],2.1)

    async def test_acceptance_journals_transient_localization_failure_with_real_stop(self):
        from mediassist.step_worker import StepWorker
        context={k:session.CTX[k] for k in ('session_id','map_id','map_asset_hash')}
        context.update(initialization_revision=2,profile_hash='b'*64)
        store=Mock();store.cached_command.return_value=False;store.previous.return_value=None
        store.blocked.return_value=False;store.emit.return_value={'messageId':'event'}
        adapter=N(mapping=None,motion_uncertain=False,navigation_profile_hash='b'*64,
            telemetry=lambda:{'localization':dict(ready=False,age_sec=.1,initialization_revision=2)},
            perform=AsyncMock(side_effect=StepError('LOCALIZATION_NOT_READY')),stop=AsyncMock(return_value=True))
        worker=StepWorker({'robot_id':'robot','navigation_session':session.CTX},{},store,adapter)
        worker.acknowledged=AsyncMock()
        command=dict(commandType='EXECUTE_STEP',commandId='cmd',correlationId='correlation',expiresAt='2099-01-01T00:00:00Z',
            navigationContext=context,step=dict(stepType='MOVE',params={},timeoutSec=10))
        with patch('mediassist.step_worker.validate_command'),patch('mediassist.step_worker.validate_step'):
            await worker.command(command);await worker.task
        store.accept.assert_called_once();adapter.stop.assert_awaited_once()
        self.assertEqual(worker.last_result['status'],'FAILED')
        self.assertTrue(worker.last_result['physicalStopConfirmed']);self.assertIsNone(worker.active)
        self.assertEqual(worker.status(),'IDLE')

if __name__=='__main__':unittest.main()
