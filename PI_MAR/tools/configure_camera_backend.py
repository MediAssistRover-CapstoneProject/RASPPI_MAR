"""Switch LAN video backend while stopped; preserve driver FPS and MQTT settings."""
import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import stat
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from mediassist.worker_config import camera_config, load
from mediassist.vendor_camera import vendor_executable, check_port_available, stream_path
from configure_camera import atomic_write
from configure_mapping import stopped_configuration


def plan(raw, backend):
    if backend not in ('python', 'web_video_server'):
        raise ValueError('Unknown camera backend')
    cfg = json.loads(raw.decode('utf-8-sig'))
    camera = cfg.setdefault('mapping', {}).setdefault('camera', {})
    if not camera.get('enabled', False):
        raise RuntimeError('CAMERA_NOT_ENABLED: enable the existing camera configuration first')
    camera.update(stream_backend=backend, stream_enabled=True,mqtt_preview_enabled=backend!='web_video_server',
                  stream_bind='0.0.0.0', stream_port=8080 if backend == 'web_video_server' else 8081)
    camera_config(copy.deepcopy(cfg['mapping']))
    return (json.dumps(cfg, ensure_ascii=False, indent=2) + '\n').encode('utf-8')


def apply(config, backend):
    config = Path(config).resolve(strict=True)
    with stopped_configuration(config):
        load(config)
        before = config.read_bytes()
        after = plan(before, backend)
        if backend == 'web_video_server':
            vendor_executable()
            check_port_available()
        if before == after:
            return None
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        backup = config.parent.parent / 'state/camera-backend-backups' / stamp / config.name
        backup.parent.mkdir(parents=True, mode=0o700)
        backup.parent.chmod(0o700)
        atomic_write(backup, before, 0o600)
        if config.read_bytes() != before:
            raise RuntimeError('CAMERA_CONFIG_CHANGED_DURING_APPLY')
        atomic_write(config, after, stat.S_IMODE(config.stat().st_mode))
        return backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'pi/config/mqtt.worker.json')
    parser.add_argument('--backend', choices=('python', 'web_video_server'), default='web_video_server')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    config = args.config.resolve(strict=True)
    if args.apply:
        if sys.platform != 'linux':
            parser.error('Apply inside MentorPi')
        backup = apply(config, args.backend)
        print('CAMERA_BACKEND_APPLIED; backup:', backup or 'already configured')
    else:
        load(config)
        plan(config.read_bytes(), args.backend)
        print('CAMERA_BACKEND_CHECK_OK; no files changed')
    camera = camera_config(json.loads(plan(config.read_bytes(), args.backend))['mapping'])
    print('Backend:', args.backend)
    print('URL: http://<PI_IP>:' + str(camera['stream_port']) + stream_path(camera))
    print('Driver FPS/resolution, MQTT thumbnails, BE origin, credentials and motion configuration preserved.')
    print('Stop the manual web_video_server with Ctrl+C before apply; then run the existing robot.zsh run once.')
    if args.backend == 'web_video_server':
        print('Native server has no custom /health endpoint; check camera.log and real HTTP FPS instead.')


if __name__ == '__main__':
    main()
