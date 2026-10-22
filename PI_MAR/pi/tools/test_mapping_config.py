"""Offline configuration regression tests; no GPIO, network or motor commands."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools')]
import configure_mapping as tool
from mediassist.configuration import validate_config
from mediassist.contracts import StepError
from mediassist.safety import gate_velocity
from mediassist.worker_config import mapping_upload_base


class MappingConfigTests(unittest.TestCase):
    def hardware(self):
        return json.loads((ROOT/'pi/config/site.example.json').read_text())

    def test_be_origin_maps_to_the_actual_java_upload_collection(self):
        expected='https://192.168.1.11/api/v1/pi/mapping-sessions'
        for value in ('https://192.168.1.11','https://192.168.1.11/',expected,expected+'/'):
            self.assertEqual(mapping_upload_base(value),expected)
        for bad in ('http://192.168.1.11','https://user:secret@be.example','https://be.example/?token=x'):
            with self.assertRaises(StepError):mapping_upload_base(bad)

    def test_025_is_accepted_but_smaller_or_nonfinite_values_are_rejected(self):
        profile=self.hardware()
        profile['motion']['stop_distance_m']=.025
        validate_config(profile)
        for distance in (.024,0.,float('nan'),float('inf'),True):
            bad=copy.deepcopy(profile);bad['motion']['stop_distance_m']=distance
            with self.subTest(distance=distance),self.assertRaises(StepError):validate_config(bad)

    def test_gate_uses_025_without_ignoring_invalid_echo_stale_data_or_speed_cap(self):
        motion=self.hardware()['motion']
        motion.update(commissioned=True,cart_footprint_validated=True,watchdog_validated=True,stop_distance_m=.025)
        velocity,reason=gate_velocity((.1,0.,0.),[(.03,10.),(.03,10.)],10.,10.,10.,10.,motion,.05)
        self.assertAlmostEqual(velocity[0],.05)
        self.assertEqual(velocity[1:],(0.,0.));self.assertEqual(reason,'CLEAR')
        for distance,stamp,reason in ((.024,10.,'SIDE_OBSTACLE'),(.019,10.,'STALE_OR_INVALID_US100'),
                                      (None,10.,'STALE_OR_INVALID_US100'),(.5,9.,'STALE_OR_INVALID_US100')):
            with self.subTest(distance=distance):
                command,actual=gate_velocity((.05,0.,0.),[(distance,stamp),(.5,10.)],10.,10.,10.,10.,motion,.05)
                self.assertEqual(command,(0.,0.,0.));self.assertEqual(actual,reason)

    def scenario(self, directory):
        root=Path(directory)
        hardware=self.hardware();source=root/'original.json';source.write_text(json.dumps(hardware))
        cfg={'hardware_profile':str(source),'state_path':'/unchanged/worker.sqlite',
             'legacy_state_path':'/unchanged/legacy.sqlite','mqtt':{'password_file':'/secret/mqtt.txt'},
             'speech':{'engine':'be-tts','token_file':'/secret/token.txt'},'mapping':{'enabled':True}}
        path=root/'mqtt.worker.json';path.write_text(json.dumps(cfg))
        return path,cfg,hardware,source

    @staticmethod
    def load_candidate(path):
        cfg=json.loads(Path(path).read_text())
        return cfg,validate_config(json.loads(Path(cfg['hardware_profile']).read_text()))

    def test_updates_active_profile_and_preserves_commissioning_gpio_servo_secrets_and_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path,cfg,hardware,source=self.scenario(directory)
            before=source.read_bytes()
            with patch.object(tool,'load',side_effect=self.load_candidate):
                output=tool.write_configuration(path,cfg,hardware,.025)
            tuned=json.loads(Path(output['hardware_profile']).read_text())
            expected=copy.deepcopy(hardware);expected['motion']['stop_distance_m']=.025
            self.assertEqual(tuned,expected)
            self.assertEqual(source.read_bytes(),before)
            for key in ('state_path','legacy_state_path','mqtt','speech'):
                self.assertEqual(output[key],cfg[key])
            self.assertFalse(tuned['motion']['commissioned'])
            self.assertEqual(json.loads(path.read_text()),output)
            self.assertEqual(len(list(Path(directory).glob('mqtt.worker.*.bak'))),1)

    def test_m1_profile_preserves_motion_flags_sensor_threshold_and_wiring(self):
        with tempfile.TemporaryDirectory() as directory:
            path,cfg,hardware,source=self.scenario(directory)
            options={'max_speed_mps':.15,'motion':{'chassis_model':'MENTORPI_M1_MECANUM','allow_reverse':True,
                'max_reverse_speed_mps':.1,'rear_stop_distance_m':.3,'max_angular_speed_rps':.5}}
            with patch.object(tool,'load',side_effect=self.load_candidate):
                output=tool.write_configuration(path,cfg,hardware,motion_options=options)
            updated=json.loads(Path(output['hardware_profile']).read_text())
            self.assertEqual(updated['us100'],hardware['us100']);self.assertEqual(updated['servo'],hardware['servo'])
            self.assertEqual(updated['motion']['stop_distance_m'],hardware['motion']['stop_distance_m'])
            for key in ('commissioned','cart_footprint_validated','watchdog_validated'):
                self.assertEqual(updated['motion'][key],hardware['motion'][key])
            self.assertTrue(updated['motion']['allow_reverse']);self.assertEqual(updated['motion']['max_angular_speed_rps'],.5)

    def test_validation_failure_does_not_replace_config_or_leave_a_new_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            path,cfg,hardware,_=self.scenario(directory)
            before=path.read_bytes()
            with patch.object(tool,'load',side_effect=ValueError('invalid worker configuration')):
                with self.assertRaisesRegex(ValueError,'invalid worker'):
                    tool.write_configuration(path,cfg,hardware,.025)
            self.assertEqual(path.read_bytes(),before)
            self.assertEqual(list(Path(directory).glob('site.motion.*.json')),[])
            self.assertFalse(path.with_suffix('.mapping.new').exists())


if __name__=='__main__':unittest.main()
