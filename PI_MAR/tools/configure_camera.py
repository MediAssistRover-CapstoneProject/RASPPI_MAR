"""Prepare a measured HP60C 15/20 FPS camera trial; stop the robot stack before apply."""
import argparse
import ast
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from mediassist.worker_config import camera_config, load
from configure_mapping import stopped_configuration


def vendor_launch():
    if os.environ.get('DEPTH_CAMERA_TYPE') != 'ascamera':
        raise RuntimeError('HP60C_REQUIRED: check DEPTH_CAMERA_TYPE; no files changed')
    if os.environ.get('need_compile') == 'True':
        from ament_index_python.packages import get_package_share_directory
        folder = Path(get_package_share_directory('peripherals'))
    elif os.environ.get('need_compile') == 'False':
        folder = Path('/home/ubuntu/ros2_ws/src/peripherals')
    else:
        raise RuntimeError('SOURCE_ROS_SHELL_FIRST: need_compile must be True or False')
    return folder / 'launch/include/ascamera.launch.py'


def patch_driver(raw, fps):
    """Change only literal FPS/resolutions of the known ascamera node; keep TF/depth features."""
    if type(fps) is not int or fps not in (15, 20):
        raise ValueError('HP60C trial supports 15 or 20 FPS')
    source = raw.decode('utf-8-sig')
    tree = ast.parse(source)
    nodes = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        kw = {item.arg: item.value for item in node.keywords}
        if (isinstance(kw.get('package'), ast.Constant) and kw['package'].value == 'ascamera'
                and isinstance(kw.get('executable'), ast.Constant) and kw['executable'].value == 'ascamera_node'):
            nodes.append(kw)
    if len(nodes) != 1 or not isinstance(nodes[0].get('parameters'), ast.List):
        raise RuntimeError('CAMERA_LAUNCH_NOT_RECOGNIZED: expected one ascamera_node with literal parameters')
    wanted = {'fps': fps, 'rgb_width': 640, 'rgb_height': 480, 'depth_width': 640, 'depth_height': 480}
    matches = {key: [] for key in wanted}
    for item in nodes[0]['parameters'].elts:
        if isinstance(item, ast.Dict):
            for key, value in zip(item.keys, item.values):
                if isinstance(key, ast.Constant) and key.value in matches:
                    matches[key.value].append(value)
    # AST column offsets count UTF-8 bytes; replace in bytes to retain all other text.
    data = source.encode('utf-8')
    lines = data.splitlines(keepends=True)
    offsets = [0]
    for line in lines:
        offsets.append(offsets[-1] + len(line))
    edits = []
    for key, value in wanted.items():
        found = matches[key]
        if len(found) != 1 or not isinstance(found[0], ast.Constant) or type(found[0].value) is not int:
            raise RuntimeError('CAMERA_LAUNCH_NOT_RECOGNIZED: literal parameter ' + key)
        item = found[0]
        edits.append((offsets[item.lineno-1]+item.col_offset,
                      offsets[item.end_lineno-1]+item.end_col_offset, str(value).encode()))
    for start, end, replacement in sorted(edits, reverse=True):
        data = data[:start] + replacement + data[end:]
    ast.parse(data, feature_version=(3, 10))
    return data


def plan(config, driver, fps=20, width=320, height=240, quality=50):
    config, driver = Path(config).resolve(strict=True), Path(driver).resolve(strict=True)
    if config == driver:
        raise ValueError('Worker config and camera driver must be different files')
    load(config)
    original_config, original_driver = config.read_bytes(), driver.read_bytes()
    candidate = json.loads(original_config.decode('utf-8-sig'))
    settings = candidate.setdefault('mapping', {}).setdefault('camera', {})
    settings.update(stream_enabled=True, stream_rate_hz=fps, stream_max_width=width,
                    stream_max_height=height, stream_jpeg_quality=quality, stream_max_frame_age_sec=.35)
    camera_config(copy.deepcopy(candidate['mapping']))
    updated_driver = patch_driver(original_driver, fps)
    updated_config = (json.dumps(candidate, ensure_ascii=False, indent=2) + '\n').encode()
    return {driver: (original_driver, updated_driver), config: (original_config, updated_config)}


def atomic_write(path, raw, mode):
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def apply(config, driver, fps=20, width=320, height=240, quality=50):
    config = Path(config).resolve(strict=True)
    with stopped_configuration(config):
        prepared = plan(config, driver, fps, width, height, quality)
        # Validate the complete candidate with the same loader as the worker.
        fd, name = tempfile.mkstemp(suffix='.camera.new', dir=str(config.parent))
        try:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(prepared[config][1])
            load(Path(name))
        finally:
            Path(name).unlink(missing_ok=True)
        changed = {path: pair for path, pair in prepared.items() if pair[0] != pair[1]}
        if not changed:
            return None
        backup = config.parent.parent / 'state/camera-backups' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        backup.mkdir(parents=True, mode=0o700)
        backup.chmod(0o700)
        modes, manifest = {}, []
        for index, (path, (before, after)) in enumerate(changed.items()):
            if path.read_bytes() != before:
                raise RuntimeError('CAMERA_CONFIG_CHANGED_DURING_PREPARE')
            modes[path] = stat.S_IMODE(path.stat().st_mode)
            saved = backup / (str(index) + '-' + path.name)
            atomic_write(saved, before, 0o600)
            manifest.append({'path': str(path), 'backup': saved.name,
                             'beforeSha256': hashlib.sha256(before).hexdigest(),
                             'afterSha256': hashlib.sha256(after).hexdigest()})
        atomic_write(backup / 'manifest.json', json.dumps(manifest, indent=2).encode(), 0o600)
        applied = []
        try:
            for path, (before, after) in changed.items():
                if path.read_bytes() != before:
                    raise RuntimeError('CAMERA_CONFIG_CHANGED_DURING_APPLY')
                atomic_write(path, after, modes[path])
                applied.append(path)
        except BaseException:
            for path in reversed(applied):
                atomic_write(path, changed[path][0], modes[path])
            raise
        return backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'pi/config/mqtt.worker.json')
    parser.add_argument('--vendor-launch', type=Path, help='Override only after verifying this is the active ascamera launch')
    parser.add_argument('--fps', type=int, choices=(15, 20), default=20)
    parser.add_argument('--width', type=int, choices=(320, 640), default=320)
    parser.add_argument('--quality', type=int, choices=range(20, 81), default=50, metavar='20..80')
    parser.add_argument('--apply', action='store_true', help='Write after checking stack stopped; default is read-only')
    args = parser.parse_args()
    driver = args.vendor_launch or vendor_launch()
    height = args.width * 3 // 4
    if args.apply:
        if sys.platform != 'linux':
            parser.error('Apply inside MentorPi; use read-only mode on a laptop')
        backup = apply(args.config, driver, args.fps, args.width, height, args.quality)
        print('CAMERA_APPLIED; backup:', backup if backup else 'already configured')
    else:
        prepared = plan(args.config, driver, args.fps, args.width, height, args.quality)
        print('CAMERA_CHECK_OK: no files changed')
        print('Driver launch:', next(iter(prepared)))
    print('HP60C source RGB/depth: 640x480 @', args.fps, 'FPS (shared driver setting)')
    print('LAN MJPEG:', str(args.width)+'x'+str(height), '@', args.fps, 'FPS; JPEG quality:', args.quality)
    print('MQTT thumbnail settings preserved. Frame age limit: 350 ms from ROS receipt, not exposure/display latency.')
    print('Restart the existing stack once; keep one video tab open and inspect /health.')


if __name__ == '__main__':
    main()
