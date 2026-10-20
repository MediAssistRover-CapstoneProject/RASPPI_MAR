"""Offline graph discovery tests; no ROS nodes or motor commands."""
import asyncio
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench_motion import wait_for_graph, check_bench_inputs, check_test_sensors, StepError


class DiscoveryTests(unittest.TestCase):
    def run_wait(self, check, timeout=2, **kwargs):
        now = [0.0]
        async def sleep(delay):
            now[0] += delay
        with patch('builtins.print'):
            asyncio.run(wait_for_graph(check, timeout, lambda: now[0], sleep, **kwargs))

    def test_waits_for_three_successes_after_missing_publishers(self):
        check = Mock(side_effect=[StepError('UNSAFE_VELOCITY_GRAPH'), None, None, None])
        self.run_wait(check)
        self.assertEqual(check.call_count, 4)

    def test_any_failure_resets_stability(self):
        check = Mock(side_effect=[None, None, StepError('CHASSIS_COUNT_INVALID'), None, None, None])
        self.run_wait(check)
        self.assertEqual(check.call_count, 6)

    def test_persistent_unsafe_graph_times_out(self):
        check = Mock(side_effect=StepError('UNSAFE_VELOCITY_GRAPH', 'publishers=[]'))
        with self.assertRaisesRegex(RuntimeError, 'BENCH_GRAPH_TIMEOUT.*publishers='):
            self.run_wait(check)
        self.assertEqual(check.call_count, 8)

    def test_bypass_never_becomes_ready(self):
        with self.assertRaisesRegex(RuntimeError, 'CHASSIS_BYPASS_DETECTED'):
            self.run_wait(Mock(side_effect=StepError('CHASSIS_BYPASS_DETECTED')))

    def test_unexpected_error_is_not_hidden(self):
        with self.assertRaisesRegex(ValueError, 'broken'):
            self.run_wait(Mock(side_effect=ValueError('broken')))

    def test_delayed_sensor_then_one_second_of_stable_checks(self):
        check=Mock(side_effect=[StepError('BENCH_INPUTS_NOT_READY','right_range')]*8+[None]*5)
        self.run_wait(check,timeout=5,required_successes=5)
        self.assertEqual(check.call_count,13)

    def test_intermittent_sensor_resets_consecutive_readiness(self):
        check=Mock(side_effect=[None]*4+[StepError('BENCH_INPUTS_NOT_READY','right_range')]+[None]*5)
        self.run_wait(check,timeout=4,required_successes=5)
        self.assertEqual(check.call_count,10)

    def test_persistent_missing_echo_remains_a_failure(self):
        with self.assertRaisesRegex(RuntimeError,'BENCH_INPUTS_TIMEOUT.*right_range'):
            self.run_wait(Mock(side_effect=StepError('BENCH_INPUTS_NOT_READY','right_range')),
                          timeout_label='BENCH_INPUTS_TIMEOUT')

class InputTests(unittest.TestCase):
    def ready(self):
        adapter=Mock()
        adapter.cfg={'motion':{'sensor_max_age_sec':.6,'stop_distance_m':.25}}
        adapter.health.return_value={'sensors_ready':True,'left_fresh':True,'right_fresh':True,
                                     'left_m':.5,'right_m':.8}
        adapter.telemetry.return_value={'scan':{'value':True,'age_sec':.1},
            'odom_speed':{'value':0.,'age_sec':.1},'gate':{'value':'NO_LEASE','age_sec':.1}}
        sample={'value':[0.,0.,0.],'at':9.9}
        return adapter,sample

    def test_current_stationary_inputs_pass_without_actuator_calls(self):
        adapter,sample=self.ready()
        check_bench_inputs(adapter,sample,clock=lambda:10.)
        self.assertEqual([c[0] for c in adapter.mock_calls],['_check_graph','health','telemetry'])

    def test_missing_or_stale_right_is_not_substituted_with_clear_distance(self):
        adapter,sample=self.ready()
        adapter.health.return_value.update(sensors_ready=False,right_fresh=False,right_m=None)
        with self.assertRaisesRegex(StepError,'right_range'):
            check_bench_inputs(adapter,sample,clock=lambda:10.)

    def test_obstacle_or_stale_gate_output_prevents_start(self):
        adapter,sample=self.ready()
        adapter.health.return_value['right_m']=.2
        sample['at']=8.
        with self.assertRaisesRegex(StepError,'right_clearance.*zero_safe_velocity'):
            check_bench_inputs(adapter,sample,clock=lambda:10.)

    def test_moving_or_stale_odom_prevents_start(self):
        adapter,sample=self.ready()
        for speed,age in ((.03,.1),(0.,1.)):
            adapter.telemetry.return_value['odom_speed']={'value':speed,'age_sec':age}
            with self.assertRaisesRegex(StepError,'stationary_odom'):
                check_bench_inputs(adapter,sample,clock=lambda:10.)

    def test_sensor_loss_during_fault_injection_cannot_be_recorded_as_pass(self):
        with self.assertRaisesRegex(StepError,'Sensor data became unavailable'):
            check_test_sensors({'sensors_ready':False},.25)

    def test_side_obstacle_during_fault_injection_cannot_be_recorded_as_pass(self):
        with self.assertRaisesRegex(StepError,'Side clearance changed'):
            check_test_sensors({'sensors_ready':True,'left_m':.2,'right_m':.8},.25)
        check_test_sensors({'sensors_ready':True,'left_m':.5,'right_m':.8},.25)


if __name__ == '__main__':
    unittest.main()
