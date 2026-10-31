"""Factory yaw/config and live plan isolation; no ROS, network or motor output."""
import asyncio,math,sys,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import AsyncMock
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools'),str(ROOT/'pi/tools')]
from configure_vendor_yaw import tune
from test_motion_speed import MotionSpeedTests
from mediassist.contracts import StepError
from mediassist.ros_adapter import RosAdapter
from mediassist.gamepad import configuration


class YawTests(unittest.TestCase):
    def test_factory_yaw_preserves_translation_tls_and_exploration(self):
        raw,hw=MotionSpeedTests().profile()
        hw['motion']['lidar_braking']={**raw['mapping']['autonomy'],'include_points':True}
        configuration(raw['mapping']).update(layout='vendor_mecanum',linear_axis=1,lateral_axis=0,angular_axis=2)
        before=raw['mapping']['autonomy'].copy()
        cfg,tuned=tune(raw,hw)
        self.assertEqual(tuned['motion']['max_angular_speed_rps'],2.)
        self.assertEqual(tuned['motion']['lidar_braking']['angular_rps'],2.)
        self.assertEqual(tuned['navigation']['max_angular_speed_rps'],1.2)
        self.assertEqual(cfg['mapping']['gamepad']['angular_rps'],2.)
        self.assertEqual(cfg['mapping']['autonomy'],before)
        self.assertEqual(cfg['mapping']['upload_base_url'],raw['mapping']['upload_base_url'])
        self.assertEqual(hw['motion']['max_angular_speed_rps'],.8)
        with self.assertRaises(ValueError):tune(raw,hw,2.1,1.2)


class PlanTests(unittest.IsolatedAsyncioTestCase):
    def adapter(self):
        a=RosAdapter.__new__(RosAdapter);a.lock=threading.Lock();a.zones=None
        a.cfg={'ros':{'map_frame':'map'},'endpoints':{'A':dict(x_m=1.,y_m=1.)}}
        a.navigation_plan_receipt={'sessionId':'session','jobId':'job','jobStepId':'step','endpointId':'A'}
        return a
    def path(self,frame='map',end=(1.,1.)):
        def pose(x,y):return N(pose=N(position=N(x=x,y=y),orientation=N(x=0.,y=0.,z=0.,w=1.)))
        return N(header=N(frame_id=frame),poses=[pose(0.,0.),pose(.4,.6),pose(*end)])
    def test_plan_keeps_scope_and_rejects_wrong_frame_or_goal(self):
        a=self.adapter();receipt=a.navigation_plan_receipt
        self.assertEqual(a._plan_body(self.path(),'A',receipt)['jobStepId'],'step')
        for p in (self.path('odom'),self.path(end=(9.,9.)),self.path(end=(math.nan,1.))):
            with self.assertRaises(StepError):a._plan_body(p,'A',receipt)
    async def test_slow_display_upload_does_not_block_event_loop_and_cancels(self):
        a=self.adapter();started=asyncio.Event();release=asyncio.Event()
        async def publish(body):started.set();await release.wait()
        a._publish_plan=publish
        now=time.monotonic();a.nav_plan_sample=(self.path(),now)
        task=asyncio.create_task(a._relay_navigation_plans('A',now-.1))
        try:
            await asyncio.wait_for(started.wait(),2.)
            # A lease/velocity task can still run while HTTPS is waiting.
            ticks=0
            for _ in range(4):await asyncio.sleep(.01);ticks+=1
            self.assertEqual(ticks,4);self.assertFalse(task.done())
        finally:
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):await task

if __name__=='__main__':unittest.main()
