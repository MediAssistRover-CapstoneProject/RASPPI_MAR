"""Current scan authorizes each escape, independent of a stale exploration grid."""
import unittest
from test_autonomy_speed import inputs,tune
from mediassist.lidar_reactive import LidarReactive

class ReactiveTests(unittest.TestCase):
 def config(self):return tune(*inputs(),1.,.8,mode='reactive')[0]['mapping']['autonomy']
 def view(self,**kw):return dict(front_m=3.,left_m=3.,right_m=3.,lateral_left_m=3.,lateral_right_m=3.,rotation_clear=True,**kw)
 def test_five_centimetre_front_stops_forward_but_uses_clear_side_without_turn_collision(self):
  c=self.config();v=self.view();v.update(front_m=.05,rotation_clear=False,lateral_left_m=1.,lateral_right_m=.03)
  velocity,reason=LidarReactive().current(v,c,.1,1.,'run')
  self.assertEqual(velocity[0],0.);self.assertGreater(velocity[1],0.);self.assertEqual(velocity[2],0.)
  self.assertEqual(reason,'LIDAR_SIDE_ESCAPE')
 def test_obstacle_reduces_forward_speed_and_clear_turn_uses_requested_yaw(self):
  c=self.config();controller=LidarReactive();v=self.view()
  fast,_=controller.current(v,c,.1,1.,'run');v['front_m']=.7
  slow,_=controller.current(v,c,.1,2.,'run');self.assertLess(slow[0],fast[0])
  v['front_m']=.1;turn,reason=controller.current(v,c,.1,3.,'run')
  self.assertEqual(turn[:2],(0.,0.));self.assertEqual(abs(turn[2]),.8)
 def test_enclosed_robot_holds_zero_and_recovers_from_new_clear_scan(self):
  c=self.config();controller=LidarReactive();v=self.view();v.update(front_m=.03,lateral_left_m=.03,lateral_right_m=.03,rotation_clear=False)
  self.assertEqual(controller.current(v,c,.1,1.,'run')[0],(0.,0.,0.))
  self.assertGreater(controller.current(self.view(),c,.1,2.,'run')[0][0],0.)

if __name__=='__main__':unittest.main()
