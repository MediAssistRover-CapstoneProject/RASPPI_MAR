"""Verify a reachable BE HTTPS origin and update only map/TTS URLs and their public CA."""
import argparse
from contextlib import nullcontext
import copy
import hashlib
import json
from pathlib import Path
import ssl
import sys
import urllib.request
from urllib.parse import urlsplit


def origin_url(value):
    uri = urlsplit(value)
    if (uri.scheme != 'https' or not uri.hostname or uri.username or uri.password
            or uri.path not in ('', '/') or uri.query or uri.fragment):
        raise ValueError('Use an HTTPS origin, for example https://192.168.1.18')
    return value.rstrip('/')


def updated_config(cfg, origin, ca_file):
    origin = origin_url(origin)
    candidate = copy.deepcopy(cfg)
    candidate['mapping']['upload_base_url'] = origin+'/api/v1/pi/mapping-sessions'
    candidate['mapping']['upload_ca_file'] = str(ca_file)
    speech = candidate.get('speech', {})
    if speech.get('engine') in ('be-tts', 'fpt-be'):
        speech['synthesize_url'] = origin+'/api/v1/pi/speech/synthesize'
        speech['ca_file'] = str(ca_file)
    return candidate


def main():
    source_root = Path(__file__).resolve().parents[1]
    default_root = source_root if (source_root/'pi/mediassist').is_dir() else Path('/home/ubuntu/mediassist-mqtt-2026-10-08')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=default_root)
    parser.add_argument('--origin', required=True)
    parser.add_argument('--ca-file', required=True, type=Path)
    parser.add_argument('--apply', action='store_true', help='Stop stack first. Default is network/auth check only.')
    args = parser.parse_args()
    root = args.root.resolve(strict=True)
    sys.path[:0] = [str(root/'pi'), str(root/'tools')]
    from configure_mapping import load, stopped_configuration, write_configuration
    from mediassist.be_token import request_device_token
    config = root/'pi/config/mqtt.worker.json'
    origin = origin_url(args.origin)
    ca = args.ca_file.resolve(strict=True)
    with stopped_configuration(config) if args.apply else nullcontext():
        # Use the raw JSON for the write: do not persist unrelated load() defaults.
        cfg = json.loads(config.read_text(encoding='utf-8-sig'))
        _, hardware = load(config)
        mapping = cfg['mapping']
        context = ssl.create_default_context(cafile=str(ca))
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), urllib.request.HTTPSHandler(context=context))
        with opener.open(origin+'/actuator/health', timeout=8) as response:
            health = json.load(response)
            if response.status != 200 or health.get('status') != 'UP':
                raise ValueError('BE_HEALTH_NOT_UP')
        print('PI_TO_BE_HTTPS_OK:', origin)
        secret = Path(mapping['device_secret_file']).read_text(encoding='utf-8').strip()
        result = request_device_token(origin, ca, mapping['robot_code'], secret, cfg['robot_id'], timeout=8)
        print('DEVICE_AUTH_OK:', result['robotId'])
        # No token or secret is printed or replaced. Existing refresh uses the new origin on next SAVE.
        if not args.apply:
            print('CHECK_ONLY: URLs and credentials not changed; add --apply with stack stopped')
            return
        previous_ca = Path(mapping['upload_ca_file']).resolve(strict=True)
        raw = ca.read_bytes()
        destination = previous_ca.parent/('be-ca-network-'+hashlib.sha256(raw).hexdigest()[:12]+'.pem')
        created = not destination.exists()
        if not created and destination.read_bytes() != raw:
            raise ValueError('CA_DESTINATION_CONFLICT')
        try:
            if created:
                destination.write_bytes(raw)
                destination.chmod(0o600)
            candidate = updated_config(cfg, origin, destination)
            if candidate != cfg:
                write_configuration(config, candidate, hardware)
        except BaseException:
            if created: destination.unlink(missing_ok=True)
            raise
        print('BE_ORIGIN_UPDATED:', candidate['mapping']['upload_base_url'])
        print('Broker host/CA, device credentials, camera, motion and map/journal paths preserved.')
        storage = Path(candidate['mapping']['storage_dir'])
        session = storage/'session.json'
        if session.exists():
            state = json.loads(session.read_text())
            print('LOCAL_MAPPING_STATE:', state.get('state'))
            print('LOCAL_ARTIFACT_DIR:', state.get('artifact_dir'))


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('BE_ORIGIN_CHECK_FAILED:', type(exc).__name__, str(exc))
        raise SystemExit(1)
