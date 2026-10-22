"""Device-token renewal with fake credentials/network; never contacts the BE."""
import base64
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.be_token import upload_token,write_private,remaining_seconds,request_device_token
from mediassist.contracts import StepError


def jwt(expires):
    return 'header.'+base64.urlsafe_b64encode(json.dumps({'exp':expires}).encode()).decode().rstrip('=')+'.signature'


class DeviceTokenTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.cfg={'upload_token_file':str(self.root/'token'),'upload_base_url':'https://be.example/api/v1/pi/mapping-sessions','robot_code':'mar-v1'}

    def tearDown(self):self.temp.cleanup()

    def test_fresh_token_never_calls_auth_endpoint(self):
        token=jwt(time.time()+3600);write_private(self.cfg['upload_token_file'],token)
        with patch('mediassist.be_token.request_device_token') as request:
            self.assertEqual(upload_token(self.cfg,'robot'),token);request.assert_not_called()

    def test_expired_token_without_secret_reports_actionable_error(self):
        write_private(self.cfg['upload_token_file'],jwt(time.time()-1))
        with self.assertRaisesRegex(StepError,'Refresh BE device token'):
            upload_token(self.cfg,'robot')

    def test_expired_token_refreshes_privately_for_expected_robot(self):
        write_private(self.cfg['upload_token_file'],jwt(time.time()-1))
        secret=self.root/'secret';write_private(secret,'fake-test-only-secret')
        self.cfg['device_secret_file']=str(secret)
        fresh=jwt(time.time()+3600)
        with patch('mediassist.be_token.request_device_token',return_value={'accessToken':fresh}) as request:
            self.assertEqual(upload_token(self.cfg,'robot'),fresh)
            self.assertEqual(request.call_args.args[-1],'robot')
        self.assertEqual(Path(self.cfg['upload_token_file']).read_text().strip(),fresh)
        self.assertEqual(list(self.root.glob('*.new')),[])

    def test_forced_refresh_handles_revocation_and_masks_network_errors(self):
        write_private(self.cfg['upload_token_file'],jwt(time.time()+3600))
        secret=self.root/'secret';write_private(secret,'fake-test-only-secret');self.cfg['device_secret_file']=str(secret)
        with patch('mediassist.be_token.request_device_token',side_effect=ValueError('fake-test-only-secret')):
            with self.assertRaises(StepError) as caught:upload_token(self.cfg,'robot',force=True)
        self.assertEqual(caught.exception.code,'MAP_UPLOAD_AUTH_REFRESH_FAILED')
        self.assertNotIn('fake-test-only-secret',str(caught.exception))

    def test_credentials_cannot_be_sent_to_http_or_userinfo_url(self):
        for url in ('http://be.example','https://user:password@be.example','https://be.example/?token=x'):
            with self.assertRaises(StepError):request_device_token(url,None,'robot','secret','robot-id')
        self.assertIsNone(remaining_seconds('opaque-test-token'))


if __name__=='__main__':unittest.main()
