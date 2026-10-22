"""Regression: timeout on initial lookup must reach repair; failed repair rolls back."""
import contextlib
import importlib.util
import io
from pathlib import Path
import subprocess
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('dns', Path(__file__).with_name('repair_pi_container_dns.py'))
dns = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dns)
old = b'nameserver 116.97.90.124\nsearch demo.local\n'


class HostFile:
    def __init__(self, path): self.path = path
    def exists(self): return False
    def read_text(self):
        assert self.path == '/etc/resolv.conf'
        return 'nameserver 192.168.1.1\n'


def scenario(post_timeout=False, host_timeout=False):
    writes, backups, lookups = [], [], []
    def fake_run(argv, data=None, timeout=12, check=True):
        if argv[:2] == ['docker', 'inspect']: out = b'host\n'
        elif 'cat' in argv and argv[-1].endswith('mqtt.worker.json'): out = b'{"mqtt":{"host":"broker.example"}}'
        elif argv[-2:] == ['cat', '/etc/resolv.conf']: out = old
        elif argv[0] == 'getent':
            if host_timeout: raise subprocess.TimeoutExpired(argv, timeout)
            out = b'192.0.2.1 STREAM\n'
        elif 'getent' in argv:
            lookups.append(argv)
            if len(lookups) == 1 or post_timeout: raise subprocess.TimeoutExpired(argv, timeout)
            out = b'192.0.2.1 STREAM\n'
        elif argv[0] == 'nmcli': out = b'192.168.1.1\n'
        elif argv[-1].startswith('/tmp/mar-resolv-before-'):
            backups.append(data); out = b''
        elif argv[-1] == 'cat > /etc/resolv.conf':
            writes.append(data); out = b''
        else: raise AssertionError(argv)
        return subprocess.CompletedProcess(argv, 0, out, b'')
    output = io.StringIO()
    with patch.object(dns, 'run', fake_run), patch.object(dns, 'Path', HostFile), patch.object(dns.sys, 'platform', 'linux'), patch.object(dns.sys, 'argv', ['repair', '--apply']), contextlib.redirect_stdout(output):
        try:
            dns.main()
        except RuntimeError as error:
            assert post_timeout or host_timeout
            if host_timeout: assert 'HOST_DNS_FAILED' in str(error)
        else:
            assert not post_timeout and not host_timeout
    if host_timeout:
        assert writes == [] and backups == []
    else:
        assert backups == [old]
        assert b'nameserver 192.168.1.1' in writes[0]
        assert b'116.97.90.124' not in writes[0]
        assert 'CONTAINER_DNS_TIMEOUT' in output.getvalue()
        if post_timeout:
            assert len(writes) == 2 and writes[-1] == old
        else:
            assert len(writes) == 1 and 'CONTAINER_DNS_REPAIRED' in output.getvalue()


scenario()
print('PASS initial container timeout -> backup -> repair -> successful DNS verification')
scenario(post_timeout=True)
print('PASS timeout after repair -> original DNS restored')
scenario(host_timeout=True)
print('PASS host DNS timeout -> no container DNS writes')
