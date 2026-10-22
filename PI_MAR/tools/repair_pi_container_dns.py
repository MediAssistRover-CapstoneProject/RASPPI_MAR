"""Run on the Pi HOST. Diagnose DNS; optionally sync container DNS from this network.

Only the container's resolv.conf is changed. No host DNS, Docker daemon, runtime,
tokens or MQTT credentials are changed. This is a repair for the current network;
Docker/container recreation or a network change can require a new check.
"""
import argparse
from datetime import datetime, timezone
import ipaddress
import json
from pathlib import Path
import subprocess
import sys


def run(argv, data=None, timeout=12, check=True):
    result = subprocess.run(argv, input=data, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
    if check and result.returncode:
        raise RuntimeError('Command failed: ' + ' '.join(argv[:5]) + ': ' + result.stderr.decode(errors='replace').strip())
    return result


def dns_resolves(argv, label, timeout=12):
    try:
        return run(argv, timeout=timeout, check=False).returncode == 0
    except subprocess.TimeoutExpired:
        print(label + '_DNS_TIMEOUT: treating lookup timeout as DNS failure', flush=True)
        return False


def choose_servers(resolv, extra, mode):
    candidates = [line.split()[1] for line in resolv.splitlines() if line.strip().startswith('nameserver ') and len(line.split()) > 1]
    candidates.extend(extra.split())
    result = []
    for value in candidates:
        try:
            address = ipaddress.ip_address(value.replace('\\:', ':'))
        except ValueError:
            continue
        if address.is_unspecified or address.is_multicast or (mode != 'host' and (address.is_loopback or address.version == 6 and address.is_link_local)):
            continue
        if str(address) not in result:
            result.append(str(address))
    return result[:3]


def replacement(original, servers):
    kept = [line for line in original.decode().splitlines() if not line.lstrip().startswith(('nameserver ', 'options '))]
    return ('\n'.join(kept + ['# MAR DNS snapshot from current Pi host network'] + ['nameserver ' + value for value in servers] + ['options timeout:2 attempts:1']) + '\n').encode()


def apply_and_verify(original, desired, write, resolves):
    try:
        write(desired)
        if not resolves():
            raise RuntimeError('Container DNS still fails after repair')
    except BaseException:
        write(original)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container', default='MentorPi')
    parser.add_argument('--broker', help='Override diagnostic hostname; by default read the active worker config')
    parser.add_argument('--config', default='/home/ubuntu/mediassist-mqtt-2026-10-08/pi/config/mqtt.worker.json')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if sys.platform != 'linux' or Path('/.dockerenv').exists():
        parser.error('Run on Pi HOST using sudo python3, not Windows or inside MentorPi')
    docker = ['docker', 'exec', args.container]
    mode = run(['docker', 'inspect', args.container, '--format', '{{.HostConfig.NetworkMode}}']).stdout.decode().strip()
    if not args.broker:
        cfg = json.loads(run(docker + ['cat', args.config]).stdout.decode('utf-8-sig'))
        args.broker = cfg['mqtt']['host']
    host_resolv = Path('/etc/resolv.conf').read_text()
    original = run(docker + ['cat', '/etc/resolv.conf']).stdout
    print('Broker:', args.broker, flush=True)
    print('NetworkMode:', mode, flush=True)
    print('HOST resolv.conf:\n' + host_resolv, flush=True)
    print('CONTAINER resolv.conf:\n' + original.decode(errors='replace'), flush=True)
    if not dns_resolves(['getent', 'ahostsv4', args.broker], 'HOST', timeout=10):
        raise RuntimeError('HOST_DNS_FAILED: fix Pi host Wi-Fi/Internet/DNS first; container DNS was not changed')
    print('HOST_DNS_OK', flush=True)
    resolves = lambda: dns_resolves(docker + ['getent', 'ahostsv4', args.broker], 'CONTAINER', timeout=12)
    if resolves():
        print('CONTAINER_DNS_OK: no change needed. Check worker reconnect and broker TLS next.')
        return
    print('CONTAINER_DNS_FAILED', flush=True)
    if mode not in ('host', 'bridge', 'default'):
        raise RuntimeError('NETWORK_MODE_NEEDS_INSPECTION: no DNS change made for network ' + mode + '; preserve custom-network service discovery and inspect Docker upstream DNS first')
    extra = ''
    try:
        extra = run(['nmcli', '-g', 'IP4.DNS,IP6.DNS', 'device', 'show'], timeout=6, check=False).stdout.decode()
    except (OSError, subprocess.TimeoutExpired):
        pass
    servers = choose_servers(host_resolv, extra, mode)
    if not servers:
        raise RuntimeError('NO_USABLE_UPSTREAM_DNS: no change made. Need the active network DNS; loopback host DNS cannot be copied into a bridge container.')
    print('DNS candidates from current host network:', ', '.join(servers), flush=True)
    if not args.apply:
        print('Read-only. Run again with --apply to back up and repair container DNS.')
        return
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
    backup = '/tmp/mar-resolv-before-' + stamp + '.conf'
    run(['docker', 'exec', '-i', '-u', '0', args.container, 'sh', '-c', 'umask 077; set -C; cat > "$1"', 'sh', backup], data=original)
    print('Original DNS backup inside container:', backup, flush=True)
    write = lambda content: run(['docker', 'exec', '-i', '-u', '0', args.container, 'sh', '-c', 'cat > /etc/resolv.conf'], data=content)
    apply_and_verify(original, replacement(original, servers), write, resolves)
    print('CONTAINER_DNS_REPAIRED: worker can retry automatically. Keep the existing stack running; do not start a second worker.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        print('DNS_REPAIR_STOPPED:', exc, file=sys.stderr)
        sys.exit(1)
