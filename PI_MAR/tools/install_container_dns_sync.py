"""Install event-driven MentorPi DNS repair on the Pi HOST; never start robot ROS nodes."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

UNIT = """[Unit]
Description=Synchronize MentorPi DNS after Docker and network startup
Wants=network-online.target
After=docker.service NetworkManager.service network-online.target
ConditionPathExists=/run/docker.sock

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 /usr/local/lib/mediassist-container-dns/repair_pi_container_dns.py --apply
TimeoutStartSec=50

[Install]
WantedBy=multi-user.target docker.service
"""

DISPATCHER = """#!/bin/sh
# MAR_CONTAINER_DNS_SYNC: queue repair after DNS/network changes; never block NetworkManager.
case "$1" in lo|docker*|veth*|br-*) exit 0 ;; esac
case "$2" in
  up|dhcp4-change|dhcp6-change|dns-change)
    /usr/bin/systemctl --no-block start mar-container-dns.service
    ;;
esac
exit 0
"""

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if sys.platform != 'linux' or Path('/.dockerenv').exists():
        parser.error('Run on Pi HOST using sudo python3, outside MentorPi')
    if os.geteuid() != 0:
        parser.error('Root is required to install the systemd/NetworkManager hooks')
    repair = Path(__file__).with_name('repair_pi_container_dns.py').read_bytes()
    # This installation is specific to the verified host-network MentorPi image.
    check = subprocess.run(['docker', 'inspect', 'MentorPi', '--format', '{{.HostConfig.NetworkMode}}'],
                           check=True, capture_output=True, text=True, timeout=10)
    if check.stdout.strip() != 'host':
        raise RuntimeError('EXPECTED_HOST_NETWORK: inspect custom Docker DNS before installing')
    payloads = {
        Path('/usr/local/lib/mediassist-container-dns/repair_pi_container_dns.py'): (repair, 0o644),
        Path('/etc/systemd/system/mar-container-dns.service'): (UNIT.encode(), 0o644),
        Path('/etc/NetworkManager/dispatcher.d/90-mar-container-dns'): (DISPATCHER.encode(), 0o755),
    }
    receipt = Path('/var/lib/mediassist-container-dns/installation.json')
    previous = json.loads(receipt.read_text()) if receipt.exists() else {}
    for target, (content, mode) in payloads.items():
        if target.exists() and target.read_bytes() != content:
            digest = hashlib.sha256(target.read_bytes()).hexdigest()
            if previous.get('files', {}).get(str(target)) != digest:
                raise RuntimeError('EXISTING_FILE_NOT_MANAGED: ' + str(target))
    print('DNS_SYNC_FILES', len(payloads), flush=True)
    if not args.apply:
        print('Read-only; use --apply to install. No Docker/robot restart is required.')
        return
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    old = {}
    for target in payloads:
        old[str(target)] = (target.read_bytes(), target.stat().st_mode & 0o777) if target.exists() else None
    try:
        for target, (content, mode) in payloads.items():
            target.parent.mkdir(parents=True, exist_ok=True)
            temp = target.with_name(target.name + '.mar-new')
            temp.write_bytes(content)
            temp.chmod(mode)
            os.replace(temp, target)
        subprocess.run(['systemd-analyze', 'verify', '/etc/systemd/system/mar-container-dns.service'], check=True, timeout=15)
        subprocess.run(['systemctl', 'daemon-reload'], check=True, timeout=15)
        subprocess.run(['systemctl', 'enable', 'mar-container-dns.service'], check=True, timeout=15)
        receipt.parent.mkdir(parents=True, exist_ok=True)
        receipt.write_text(json.dumps({'installed_at':stamp,
            'files':{str(p):hashlib.sha256(data).hexdigest() for p,(data,_) in payloads.items()}}, indent=2))
        receipt.chmod(0o600)
    except BaseException:
        for target in payloads:
            if old[str(target)] is None:
                target.unlink(missing_ok=True)
            else:
                raw, mode = old[str(target)]
                temp = target.with_name(target.name + '.mar-restore')
                temp.write_bytes(raw); temp.chmod(mode); os.replace(temp, target)
        subprocess.run(['systemctl', 'daemon-reload'], timeout=15, check=False)
        raise
    print('DNS_SYNC_INSTALLED; rollback bytes were held in RAM, no persistent backup created', flush=True)
    print('Run: sudo systemctl start mar-container-dns.service')
    print('No Docker/container/robot restarted. Existing MQTT worker can retry normally.')

if __name__ == '__main__':
    main()
