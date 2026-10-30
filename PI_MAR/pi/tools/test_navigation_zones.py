import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.navigation_zones import rules,distance,keepout_data,NavigationZones
from mediassist.contracts import StepError

class ZonesTest(unittest.TestCase):
    def test_detour_passes_but_direct_and_sparse_crossing_paths_are_rejected(self):
        obj=NavigationZones.__new__(NavigationZones);obj.radius=.15
        obj.zones=rules([dict(zone_type='KEEP_OUT',polygon=self.polygon)])
        # The same start and goal can use a route above the restricted square.
        obj.validate_path([(-.4,.5),(-.4,1.4),(1.4,1.4),(1.4,.5)])
        for path in [[(-.4,.5),(1.4,.5)],[(-.4,1.4),(1.4,-.4)],[(.5,.5)]]:
            with self.assertRaisesRegex(StepError,'NAVIGATION_PATH_INTERSECTS_KEEPOUT'):
                obj.validate_path(path)

    def test_local_mask_does_not_count_body_radius_twice(self):
        zone=rules([dict(zone_type='KEEP_OUT',polygon=[[.4,.4],[.6,.4],[.6,.6],[.4,.6]])])
        grid=dict(width=20,height=20,resolution=.05,origin=[0,0,0])
        global_mask=keepout_data(grid,zone,.18);local_mask=keepout_data(grid,zone,0.)
        self.assertGreater(sum(v==100 for v in global_mask),sum(v==100 for v in local_mask))
        self.assertEqual(local_mask[10*20+5],0)
        self.assertEqual(global_mask[10*20+5],100)

    def test_route_preference_does_not_close_narrow_detours(self):
        zone=rules([dict(zone_type='KEEP_OUT',polygon=[[.4,.4],[.6,.4],[.6,.6],[.4,.6]])])
        grid=dict(width=30,height=30,resolution=.05,origin=[0,0,0])
        core=keepout_data(grid,zone,.15);soft=keepout_data(grid,zone,.15,.25)
        self.assertEqual([v==100 for v in core],[v==100 for v in soft])
        self.assertTrue(any(0<v<99 for v in soft))
        # An overlapping rule must still replace a soft cost with a hard veto.
        overlap=rules([dict(zone_type='KEEP_OUT',polygon=[[.8,.4],[1.,.4],[1.,.6],[.8,.6]])])
        combined=keepout_data(grid,zone+overlap,.15,.25)
        self.assertEqual(combined[10*30+17],100)

    def test_move_zone_cap_is_enforced_after_resume_ramp_and_never_strafes(self):
        from types import SimpleNamespace as N
        from unittest.mock import Mock,patch
        from mediassist.ros_adapter import RosAdapter
        a=RosAdapter.__new__(RosAdapter)
        a.cfg={'max_speed_mps':.8,'motion':{'allow_reverse':True,'max_reverse_speed_mps':.8,'allow_strafe':True,'max_angular_speed_rps':.8}}
        a.navigation_velocity=lambda:((.8,.3,0.),True)
        a.zones=N(zones=[1],cap=lambda pose,speed:.15)
        a.telemetry=lambda:{'map_pose':{'fresh':True}}
        a.navigation_resume_ramp=N(update=lambda velocity,now:(.7,.2,0.))
        a.navigation_velocity_output=Mock()
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
            a.publish_navigation_velocity(.8)
        msg=a.navigation_velocity_output.publish.call_args.args[0]
        self.assertEqual(msg.linear.x,.15);self.assertEqual(msg.linear.y,0.)
    polygon=[[0,0],[1,0],[1,1],[0,1]]
    def test_polygon_and_boundary(self):
        self.assertEqual(distance((.5,.5),self.polygon),0)
        self.assertEqual(distance((0,.5),self.polygon),0)
        self.assertAlmostEqual(distance((2,.5),self.polygon),1)
    def test_keepout_respects_rotated_metric_origin(self):
        zones=rules([dict(zone_type='KEEP_OUT',polygon={'points':self.polygon})])
        grid=dict(width=3,height=3,resolution=1.,origin=[0,0,0])
        self.assertEqual(keepout_data(grid,zones,0.)[0],100)
        self.assertEqual(keepout_data(grid,zones,0.)[-1],0)
        grid['origin']=[5.,5.,1.]
        self.assertEqual(keepout_data(grid,zones,0.),[0]*9)
    def test_speed_rule_and_approach(self):
        obj=NavigationZones.__new__(NavigationZones);obj.radius=.2
        obj.zones=rules([dict(zone_type='SPEED',polygon=self.polygon,speed_limit_mps=.15)])
        self.assertEqual(obj.cap(dict(fresh=True,value=dict(x_m=.5,y_m=.5)),.8),.15)
        self.assertEqual(obj.cap(dict(fresh=True,value=dict(x_m=10,y_m=10)),.8),.8)
        self.assertEqual(obj.cap(None,.8),0.)
    def test_invalid_rules_fail_before_motion(self):
        for p in ([],[[0,0],[1,1],[2,2]],[[0,0],[1,0],[float('nan'),1]]):
            with self.assertRaises(StepError):rules([dict(zone_type='KEEP_OUT',polygon=p)])
        with self.assertRaises(StepError):rules([dict(zone_type='SPEED',polygon=self.polygon)])

    def test_overlapping_speed_zones_take_lowest_and_release_on_exit(self):
        obj=NavigationZones.__new__(NavigationZones);obj.radius=.2
        obj.zones=rules([dict(zone_type='SPEED',polygon=self.polygon,speed_limit_mps=v) for v in (.22,.1)])
        pose=lambda x:dict(fresh=True,value=dict(x_m=x,y_m=.5))
        self.assertEqual(obj.cap(pose(.5),.8),.1)
        self.assertEqual(obj.cap(pose(5.),.8),.8)
        self.assertEqual(obj.cap(pose(.5),.05),.05)

    def test_keepout_stops_body_overlap_and_slows_before_boundary(self):
        obj=NavigationZones.__new__(NavigationZones);obj.radius=.2
        obj.zones=rules([dict(zone_type='KEEP_OUT',polygon=self.polygon)])
        pose=lambda x:dict(fresh=True,value=dict(x_m=x,y_m=.5))
        self.assertEqual(obj.cap(pose(.5),.8),0.)
        self.assertEqual(obj.cap(pose(-.1),.8),0.)
        self.assertGreater(obj.cap(pose(-.5),.8),0.)
        self.assertLess(obj.cap(pose(-.5),.8),.8)

class PlanningTests(unittest.IsolatedAsyncioTestCase):
    async def plan(self,results):
        from types import SimpleNamespace as N
        from unittest.mock import Mock,AsyncMock,patch
        from mediassist.ros_adapter import RosAdapter
        a=RosAdapter.__new__(RosAdapter)
        a.cfg={'ros':{'map_frame':'map'},'navigation':{'planner_id':'GridBased','planning_timeout_sec':3}}
        a._prerequisites=AsyncMock();a._check_endpoint_footprint=Mock();a._wait_navigation_server=AsyncMock()
        a._pose=Mock();a.stop=AsyncMock(return_value=True);a.node=Mock();a.planner=Mock()
        handle=N(accepted=True,get_result_async=lambda:None)
        sequence=[]
        for r in results:sequence.extend([handle,r])
        a.future=AsyncMock(side_effect=sequence)
        with patch.dict(sys.modules,{'nav2_msgs.action':N(ComputePathToPose=N(Goal=N))}),patch('mediassist.ros_adapter.asyncio.sleep',new=AsyncMock()):
            try:await a._plan({},'endpoint')
            finally:self.assertEqual(a.stop.await_count,1)
        return a

    async def test_fresh_costmap_retry_succeeds_before_navigation_is_started(self):
        from types import SimpleNamespace as N
        a=await self.plan([N(status=6),N(status=4,result=N(path=N(header=N(frame_id='map'),poses=[N(pose=N(position=N(x=0.,y=0.)))])))])
        self.assertEqual(a.planner.send_goal_async.call_count,2)

    async def test_blocked_route_has_bounded_retry_and_remains_stopped(self):
        from types import SimpleNamespace as N
        with self.assertRaisesRegex(StepError,'PATH_UNAVAILABLE'):await self.plan([N(status=6),N(status=6)])

if __name__=='__main__':unittest.main()
