"""A transient HTTPS failure never renews or revokes a cached session on its own."""
import sys,threading,unittest,urllib.error,ssl
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'pi'))
from mediassist.navigation_session import SessionPoller
from mediassist.contracts import StepError

class SessionTransportTests(unittest.TestCase):
    def poller(self):
        p=SessionPoller.__new__(SessionPoller);p.lock=threading.Lock()
        p.context={'expires_at':'2099-01-01T00:00:00Z'};p.value={'sessionId':'verified'};p.at=100.;p.error=None
        return p
    def test_timeout_does_not_reset_good_authorization_but_cannot_extend_its_age(self):
        p=self.poller();p.failed(urllib.error.URLError(TimeoutError('timeout')))
        with patch('mediassist.navigation_session.time.monotonic',return_value=104.):
            self.assertEqual(p.current(),({'sessionId':'verified'},None))
        with patch('mediassist.navigation_session.time.monotonic',return_value=114.):
            self.assertEqual(p.current(),({'sessionId':'verified'},None))
        with patch('mediassist.navigation_session.time.monotonic',return_value=116.):
            self.assertIsNone(p.current()[0])
        self.assertEqual(p.at,100.)
    def test_protocol_auth_identity_and_tls_errors_invalidate_immediately(self):
        for e in (StepError('NAVIGATION_SESSION_HTTP_401'),StepError('NAVIGATION_SESSION_HTTP_422'),
                  StepError('NAVIGATION_SESSION_MAP_CHANGED'),StepError('NAVIGATION_SESSION_CLOSED_OR_INVALID'),
                  urllib.error.URLError(ssl.SSLError('certificate'))):
            with self.subTest(error=type(e).__name__):
                p=self.poller();p.failed(e);self.assertIsNone(p.value)
    def test_temporary_server_failure_uses_only_existing_fresh_descriptor(self):
        p=self.poller();p.failed(StepError('NAVIGATION_SESSION_HTTP_503'))
        self.assertIsNotNone(p.value);self.assertEqual(p.at,100.)
        p.context['expires_at']='2020-01-01T00:00:00Z'
        self.assertEqual(p.current(),(None,'NAVIGATION_SESSION_EXPIRED'))
    def test_successful_session_refresh_does_not_change_initialization_revision(self):
        p=self.poller();p.value['initialization']={'revision':3,'mode':'SEED'}
        p.failed(StepError('NAVIGATION_SESSION_HTTP_503'))
        with patch('mediassist.navigation_session.time.monotonic',return_value=110.):
            self.assertEqual(p.current()[0]['initialization']['revision'],3)
        self.assertEqual(p.at,100.)
if __name__=='__main__':unittest.main()
