import copy,hashlib,io,json,sys,unittest
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch
from zipfile import ZipFile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.map_sync import verify_bundle
from mediassist.contracts import StepError

ID='11111111-1111-4111-8111-111111111111'
class MapSyncTests(unittest.TestCase):
    def bundle(self,change=None):
        files={'map.yaml':b'image: map.pgm\nresolution: 0.005\norigin: [0,0,0]\n',
               'map.pgm':b'P5\n2 2\n255\n\x00\xfe\xfe\x00'}
        m={'map_id':ID,'asset_hash':hashlib.sha256(files['map.yaml']+b'\x00'+files['map.pgm']).hexdigest(),
           'resolution_m_per_pixel':.005,'frame_id':'map'}
        if change:change(files,m)
        files['metadata.json']=json.dumps(m).encode();out=io.BytesIO()
        with ZipFile(out,'w') as z:
            for n,b in files.items():z.writestr(n,b)
        return out.getvalue()
    def test_hash_and_identity_verified_before_install(self):
        files,meta=verify_bundle(self.bundle(),ID)
        self.assertEqual(files['map.pgm'][:2],b'P5');self.assertEqual(meta['map_id'],ID)
    def test_modified_map_rejected(self):
        with self.assertRaises(StepError):verify_bundle(self.bundle(lambda f,m:f.update({'map.pgm':b'changed'})),ID)
    def test_path_traversal_rejected(self):
        with self.assertRaises(StepError):verify_bundle(self.bundle(lambda f,m:f.update({'../config.json':b'bad'})),ID)
    def test_wrong_robot_map_rejected(self):
        with self.assertRaises(StepError):verify_bundle(self.bundle(),'22222222-2222-4222-8222-222222222222')
    def test_navigation_requires_commissioning_and_installed_map(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
        from robot_stack import validate_mode
        h={'map_asset_sha256':'a'*64,'endpoints':{'x':{}},'motion':dict(commissioned=False,cart_footprint_validated=False,watchdog_validated=False)}
        cfg={'mapping':{'enabled':False},'localization':{'enabled':True}}
        with self.assertRaises(RuntimeError):validate_mode('run',cfg,h,'navigation')
        h['motion']=dict.fromkeys(h['motion'],True)
        validate_mode('run',cfg,h,'navigation')
        with self.assertRaises(RuntimeError):validate_mode('run',{'mapping':{'enabled':True}},h,'navigation')
    def test_installer_declares_nav2_but_preserves_uncommissioned_motion(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
        import install_be_map as installer
        root=Path(__file__).resolve().parents[2]
        hardware=json.loads((root/'pi/config/site.example.json').read_text())
        flags=copy.deepcopy(hardware['motion'])
        bundle=self.bundle()
        _,meta=verify_bundle(bundle,ID)
        manifest={'profileVersion':'BE-NAV-PROFILE-1','profileHash':'a'*64,
                  'profile':{'map_id':ID,'map_asset_sha256':meta['asset_hash'],'frame_id':'map',
                             'endpoints':{'22222222-2222-4222-8222-222222222222':
                                          {'map_id':ID,'x_m':0.005,'y_m':0.005,'yaw_rad':0}}}}
        with tempfile.TemporaryDirectory() as directory:
            task=Path(directory);config=task/'mqtt.worker.json';config.write_text('{}')
            token=task/'token.txt';token.write_text('test-token')
            cfg={'hardware_profile':'original.json','robot_id':ID,'mapping':{'enabled':True},
                 'enabled_step_types':['WAIT']}
            opener=Mock();opener.open.side_effect=[io.BytesIO(bundle),io.BytesIO(json.dumps(manifest).encode())]
            argv=['install_be_map.py','--config',str(config),'--base-url','https://be.example',
                  '--map-id',ID,'--token-file',str(token)]
            with patch.object(installer,'ROOT',task),patch.object(sys,'argv',argv),\
                 patch.object(installer,'load',return_value=(cfg,hardware)),\
                 patch.object(installer.urllib.request,'build_opener',return_value=opener),\
                 patch('builtins.print'):
                installer.main()
            active=json.loads(config.read_text())
            installed=json.loads(Path(active['hardware_profile']).read_text())
            self.assertIn('nav2',installed['capabilities'])
            self.assertEqual(installed['motion'],flags)
            self.assertFalse(installed['motion']['commissioned'])
            self.assertFalse(active['mapping']['enabled'])
            self.assertIn('MOVE',active['enabled_step_types'])

if __name__=='__main__':unittest.main()
