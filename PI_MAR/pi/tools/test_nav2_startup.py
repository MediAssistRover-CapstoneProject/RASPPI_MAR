"""Lost lifecycle responses must not replay transitions or masquerade as active navigation."""
import sys,unittest
from pathlib import Path
from unittest.mock import Mock
from types import SimpleNamespace as N
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'pi'))
from mediassist.nav2_startup import configure
from mediassist.ros_adapter import RosAdapter

class StartupTests(unittest.TestCase):
    def clock(self):
        self.now=0.
        return lambda:self.now,lambda dt:setattr(self,'now',self.now+dt)
    def test_missing_configure_response_is_confirmed_by_state_without_replay(self):
        clock,pause=self.clock();read=Mock(side_effect=[1,10,2]);change=Mock(side_effect=TimeoutError())
        configure('controller',read,change,clock=clock,pause=pause);change.assert_called_once()
    def test_already_configured_node_is_not_configured_twice(self):
        change=Mock();configure('controller',lambda:2,change);change.assert_not_called()
    def test_stuck_transition_times_out_without_repeated_requests(self):
        clock,pause=self.clock();change=Mock(return_value=True)
        with self.assertRaisesRegex(RuntimeError,'CONFIGURE_TIMEOUT'):
            configure('controller',lambda:1,change,clock=clock,pause=pause,timeout=.3)
        change.assert_called_once()
    def test_rejected_configuration_is_not_hidden(self):
        with self.assertRaisesRegex(RuntimeError,'CONFIGURE_REJECTED'):
            configure('controller',lambda:1,lambda:False)

class NavigationReadyTests(unittest.IsolatedAsyncioTestCase):
    def adapter(self,states):
        a=RosAdapter.__new__(RosAdapter);a.nav2_state_request=lambda:None
        a.nav2_states={n:Mock(service_is_ready=lambda:True,call_async=lambda req,v=v:N(done=lambda:True,result=lambda:N(current_state=N(id=v)))) for n,v in states.items()}
        return a
    async def test_discovered_but_unconfigured_planner_is_not_ready(self):
        a=self.adapter({'controller_server':2,'planner_server':1})
        self.assertFalse(await a._navigation_servers_active());self.assertEqual(a.last_nav2_states['planner_server'],1)
    async def test_every_required_server_must_be_active(self):
        a=self.adapter({'controller_server':3,'planner_server':3,'bt_navigator':3})
        self.assertTrue(await a._navigation_servers_active())
    async def test_active_nodes_do_not_bypass_vendor_bond_confirmation(self):
        a=self.adapter({'controller_server':3,'planner_server':3});a.nav2_manager_request=lambda:None
        a.nav2_manager_active=Mock(service_is_ready=lambda:True,call_async=lambda r:N(done=lambda:True,result=lambda:N(success=False)))
        self.assertFalse(await a._navigation_servers_active())
if __name__=='__main__':unittest.main()
