"""Real reported MS200 geometries, angular preservation and SLAM-only routing."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.scan_normalizer import normalize, configuration, TOPIC, ManagedNormalizer
from mediassist.slam_session import engine_parameters

SAMPLES = [(447,.011693707,6.275680542,.014044813),
           (446,.006457633,6.279171467,.014095986),
           (445,.009948504,6.273935318,.014108078),
           (445,.004712197,6.272189617,.014115940),
           (446,.003141997,6.271840572,.014086964),
           (449,.002618420,6.273935318,.013998475)]


def scan(sample=SAMPLES[0]):
    n,low,high,step=sample
    return NS(angle_min=low,angle_max=high,angle_increment=step,
              range_min=.05,range_max=12.,ranges=[1.]*n,intensities=[])


class ScanTests(unittest.TestCase):
    def test_real_six_scans_have_identical_output_geometry_and_karto_count(self):
        outputs=[normalize(scan(s)) for s in SAMPLES]
        geometry=[(o['angle_min'],o['angle_max'],o['angle_increment'],len(o['ranges'])) for o in outputs]
        self.assertEqual(len(set(geometry)),1)
        for output in outputs:
            self.assertEqual(len(output['ranges']),360)
            span=output['angle_max']-output['angle_min']
            # Humble's laser_utils: a full-circle endpoint-inclusive grid is not marked periodic.
            is360=abs(span-2*math.pi) < output['angle_increment']-(2**-23*2*math.pi)
            if span>math.radians(350) and round(span/output['angle_increment'])+1==len(output['ranges']):is360=False
            expected=round(span/output['angle_increment']-.5)+(0 if is360 else 1) if is360 else round(span/output['angle_increment'])+1
            self.assertEqual(expected,360)
            self.assertGreaterEqual(sum(math.isfinite(x) for x in output['ranges']),355)

    def test_front_left_back_obstacles_keep_their_angles_not_array_indices(self):
        for sample in SAMPLES:
            msg=scan(sample);msg.ranges=[math.inf]*len(msg.ranges)
            for heading,reading in ((.1,.4),(math.pi/2,.7),(math.pi,1.2)):
                index=round((heading-msg.angle_min)/msg.angle_increment)
                msg.ranges[index]=reading
            output=normalize(msg)
            for heading,reading in ((.1,.4),(math.pi/2,.7),(math.pi,1.2)):
                positions=[i for i,v in enumerate(output['ranges']) if math.isfinite(v) and abs(v-reading)<1e-6]
                self.assertEqual(len(positions),1)
                self.assertLess(abs(positions[0]*output['angle_increment']-heading),.016)

    def test_colliding_rays_keep_nearer_obstacle_and_matching_intensity(self):
        msg=scan();msg.ranges=[math.inf]*len(msg.ranges);msg.intensities=[0.]*len(msg.ranges)
        bins={}
        for i in range(len(msg.ranges)):
            cell=int(math.floor((msg.angle_min+i*msg.angle_increment)*360/(2*math.pi)+.5))%360
            if cell in bins:
                first,second=bins[cell],i;break
            bins[cell]=i
        msg.ranges[first]=2.;msg.ranges[second]=.4;msg.intensities[second]=7.
        output=normalize(msg)
        self.assertAlmostEqual(output['ranges'][cell],.4)
        self.assertEqual(output['intensities'][cell],7.)

    def test_unknown_and_invalid_readings_do_not_become_clear_path(self):
        msg=scan();msg.ranges=[float('nan')]*len(msg.ranges)
        msg.ranges[2]=0.;msg.ranges[3]=-math.inf;msg.ranges[4]=13.
        output=normalize(msg)
        self.assertTrue(all(math.isnan(x) for x in output['ranges']))
        msg.ranges=[math.inf]*len(msg.ranges)
        output=normalize(msg)
        self.assertTrue(all(math.isinf(x) or math.isnan(x) for x in output['ranges']))

    def test_bad_geometry_does_not_get_padded_into_a_fake_scan(self):
        for changes in ({'ranges':[]},{'angle_increment':0.},{'angle_max':1.},
                        {'angle_min':math.nan},{'angle_max':6.1},{'range_max':0.}):
            msg=scan();msg.__dict__.update(changes)
            with self.subTest(changes=changes),self.assertRaises(ValueError):normalize(msg)

    def test_raw_scan_is_not_mutated_and_timing_is_not_fabricated(self):
        msg=scan();before=dict(msg.__dict__);before['ranges']=list(msg.ranges)
        output=normalize(msg)
        self.assertEqual(msg.__dict__,before)
        self.assertEqual(output['time_increment'],0.)

    def test_only_slam_topic_changes_when_explicitly_enabled(self):
        vendor={'/**':{'ros__parameters':{'solver_plugin':'solver_plugins::CeresSolver'}}}
        mapping={'slam_resolution_m':.025,'slam_min_travel_m':.05,'slam_min_heading_rad':.1,
                 'slam_min_interval_sec':.15,'slam_map_interval_sec':.5}
        ros={'scan':'/scan_raw','map_frame':'map','base_frame':'base_footprint'}
        baseline=engine_parameters(vendor,mapping,ros)
        mapping['scan_normalization']={'enabled':True,'bins':360}
        normalized=engine_parameters(vendor,mapping,ros)
        self.assertEqual(normalized['slam_toolbox']['ros__parameters'].pop('scan_topic'),TOPIC)
        self.assertEqual(baseline['slam_toolbox']['ros__parameters'].pop('scan_topic'),'/scan_raw')
        self.assertEqual(normalized,baseline)
        self.assertEqual(ros['scan'],'/scan_raw')
        for bad in ({'enabled':1},{'enabled':True,'bins':445}):
            with self.assertRaises(ValueError):configuration({'scan_normalization':bad})

    def test_normalizer_child_is_owned_and_stopped_without_stopping_drivers(self):
        import tempfile
        with tempfile.TemporaryDirectory() as folder:
            child=Mock(pid=9000);child.poll.return_value=None
            identity={'pid':9000,'ticks':'42'};inspect=Mock(return_value=identity);stop=Mock()
            manager=ManagedNormalizer('worker.json',Path(folder)/'scan.log',(inspect,stop))
            with patch('mediassist.scan_normalizer.subprocess.Popen',return_value=child) as spawn:
                manager.start();manager.check();manager.stop()
            self.assertIn('mediassist.scan_normalizer',spawn.call_args.args[0])
            stop.assert_called_once_with(identity,grace=3.)
            self.assertIsNone(manager.process)


class OriginTests(unittest.TestCase):
    def test_network_update_preserves_broker_secrets_motion_camera_and_storage(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
        from configure_be_origin import updated_config, origin_url
        import copy
        cfg={'mqtt':{'host':'broker.example','password_file':'/private/mqtt'},'tls_ca_file':'/private/broker-ca',
             'hardware_profile':'/profile','mapping':{'storage_dir':'/maps','control_source':'gamepad',
                 'camera':{'stream_rate_hz':20},'device_secret_file':'/private/device',
                 'robot_code':'demo','upload_token_file':'/private/token','upload_base_url':'https://old/collection',
                 'upload_ca_file':'/private/old-ca'},
             'speech':{'engine':'be-tts','synthesize_url':'https://old/tts','ca_file':'/private/old-ca','token_file':'/private/token'}}
        before=copy.deepcopy(cfg)
        result=updated_config(cfg,'https://192.168.1.18/','/private/new-ca')
        self.assertEqual(cfg,before)
        self.assertEqual(result['mqtt'],before['mqtt']);self.assertEqual(result['tls_ca_file'],before['tls_ca_file'])
        for key in before['mapping']:
            if key not in ('upload_base_url','upload_ca_file'):self.assertEqual(result['mapping'][key],before['mapping'][key])
        self.assertEqual(result['mapping']['upload_base_url'],'https://192.168.1.18/api/v1/pi/mapping-sessions')
        self.assertEqual(result['speech']['token_file'],'/private/token')
        for bad in ('http://192.168.1.18','https://user:pass@be/','https://be/path','https://be/?token=x'):
            with self.assertRaises(ValueError):origin_url(bad)


if __name__=='__main__':unittest.main(verbosity=2)
