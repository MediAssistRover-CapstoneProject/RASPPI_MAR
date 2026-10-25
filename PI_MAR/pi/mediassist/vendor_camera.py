"""Run the installed Hiwonder video server, without starting camera/motor drivers."""
import argparse
import os
from pathlib import Path
import socket
from urllib.parse import urlencode


def camera_argv(python, config, camera):
    module = 'mediassist.vendor_camera' if camera['stream_backend'] == 'web_video_server' else 'mediassist.camera_stream'
    return [str(python), '-m', module, '--config', str(config)]


def stream_path(camera):
    params = {'topic': camera['topic']}
    if camera['stream_backend'] == 'web_video_server':
        params.update(type='mjpeg', width=camera['stream_max_width'],
                      height=camera['stream_max_height'], quality=camera['stream_jpeg_quality'])
    return '/stream?' + urlencode(params)


def vendor_executable(prefix=None):
    if prefix is None:
        try:
            from ament_index_python.packages import get_package_prefix
            prefix = get_package_prefix('web_video_server')
        except Exception as exc:
            raise RuntimeError('WEB_VIDEO_SERVER_NOT_INSTALLED: source ROS and check ros2 pkg prefix web_video_server') from exc
    executable = Path(prefix) / 'lib/web_video_server/web_video_server'
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError('WEB_VIDEO_SERVER_EXECUTABLE_MISSING: ' + str(executable))
    return executable


def check_port_available():
    # Never adopt/kill an unrelated listener, including a manual A/B server.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
        connection.settimeout(.5)
        if connection.connect_ex(('127.0.0.1',8080)) == 0:
            raise RuntimeError('CAMERA_PORT_8080_BUSY: stop the manual video server first; no process killed')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(('0.0.0.0', 8080))
        except OSError as exc:
            raise RuntimeError('CAMERA_PORT_8080_BUSY: stop the manual video server first; no process killed') from exc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    from .worker_config import load
    cfg, _ = load(args.config)
    camera = cfg['mapping']['camera']
    if camera['stream_backend'] != 'web_video_server' or not camera['enabled'] or not camera['stream_enabled']:
        raise RuntimeError('VENDOR_CAMERA_NOT_ENABLED')
    executable = vendor_executable()
    check_port_available()
    print('VENDOR_CAMERA_START: ' + str(executable) + '; HTTP default 0.0.0.0:8080; see following server bind log', flush=True)
    # Same executable/arguments as the successful ros2 run test. No node-name
    # remaps or port overrides: this installed build ignored the latter.
    # exec keeps the supervisor PID and avoids a second ROS launcher process.
    os.execv(str(executable), [str(executable)])


if __name__ == '__main__':
    main()
