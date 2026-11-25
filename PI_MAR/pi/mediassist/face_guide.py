"""Loopback-only face guide for a display attached to the robot.

The page never receives a biometric template or device credential. The worker
writes only a short-lived, non-identifying status beside its stack logs.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import time

PORT = 8096
STATE_FILE = 'face-guide.json'


def state_path(cfg):
    directory = cfg.get('stack_state_dir') or str(Path(cfg['state_path']).parent / 'stack')
    return Path(directory) / STATE_FILE


def write_state(path, reason, expires_at=None):
    """Publish no image, account, job or template; replace atomically."""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {'reason': reason, 'updatedAt': time.time(), 'expiresAt': expires_at}
    temporary = path.with_name(path.name + '.new')
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC | getattr(os, 'O_NOFOLLOW', 0)
    with os.fdopen(os.open(temporary, flags, 0o600), 'w', encoding='utf-8') as stream:
        json.dump(payload, stream, separators=(',', ':'))
    os.replace(temporary, path)

def current_state(path):
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        now = time.time()
        updated = float(value['updatedAt'])
        expires = float(value['expiresAt'])
        if (value.get('reason') not in ('WAITING', 'NO_FACE', 'MULTIPLE_FACES',
                                       'LOW_QUALITY', 'FACE_OUTSIDE_FRAME', 'NO_MATCH',
                                       'CAMERA_UNAVAILABLE', 'VERIFIED')
                or not math.isfinite(updated) or not math.isfinite(expires)
                or now - updated > 8 or updated - now > 2 or now >= expires):
            return {'active': False, 'reason': 'IDLE'}
        return {'active': True, 'reason': value['reason']}
    except (OSError, ValueError, TypeError, KeyError, AttributeError):
        return {'active': False, 'reason': 'IDLE'}


PAGE = """<!doctype html><html lang="vi"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Xác thực khuôn mặt</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#08131f;color:#fff;font:clamp(20px,2.6vw,36px) system-ui,sans-serif}
main{min-height:100vh;display:grid;place-items:center;padding:16px}.panel{text-align:center;width:min(100%,110vh)}
h1{font-size:1.1em;margin:.2em 0 .5em}.camera{position:relative;aspect-ratio:4/3;background:#172637;border-radius:18px;overflow:hidden}
img{width:100%;height:100%;object-fit:contain;display:none}.frame{display:none;position:absolute;left:20%;top:12%;width:60%;height:76%;border:6px solid #fff;border-radius:24px;box-shadow:0 0 0 999px #06142255,inset 0 0 20px #06142288}
.active img,.active .frame{display:block}.verified .frame{border-color:#25ce77}.warn .frame{border-color:#ffbc47}
p{min-height:2.8em;margin:.5em 0;font-weight:650}
</style><main><section class="panel"><h1>Xác thực nhân viên</h1><div class="camera" id="camera">
<img id="video" alt="Camera robot"><div class="frame" aria-hidden="true"></div></div>
<p id="message">Đang chờ robot đến bước xác thực khuôn mặt.</p></section></main>
<script>
const messages={WAITING:'Đưa khuôn mặt vào khung và nhìn thẳng vào camera.',
NO_FACE:'Chưa thấy khuôn mặt. Vui lòng nhìn vào camera.',
MULTIPLE_FACES:'Chỉ một người đứng trước camera.',
LOW_QUALITY:'Đứng gần hơn và giữ yên để ảnh rõ nét.',
FACE_OUTSIDE_FRAME:'Đưa toàn bộ khuôn mặt vào trong khung.',
CAMERA_UNAVAILABLE:'Camera chưa sẵn sàng. Vui lòng chờ hoặc báo nhân viên hỗ trợ.',
NO_MATCH:'Chưa đúng nhân viên được giao. Vui lòng thử lại.',
VERIFIED:'Xác thực thành công. Đang chờ mở ngăn.'};
const camera=document.getElementById('camera'),video=document.getElementById('video'),message=document.getElementById('message');
let active=false;
async function refresh(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error('status');
const s=await r.json();camera.className='camera'+(s.active?' active':'')+(s.reason==='VERIFIED'?' verified':(['NO_FACE','MULTIPLE_FACES','LOW_QUALITY','FACE_OUTSIDE_FRAME','NO_MATCH','CAMERA_UNAVAILABLE'].includes(s.reason)?' warn':''));
message.textContent=s.active?messages[s.reason]||messages.WAITING:'Đang chờ robot đến bước xác thực khuôn mặt.';
if(s.active&&!active){video.src=STREAM_URL;active=true}else if(!s.active&&active){video.removeAttribute('src');active=false}
}catch(e){camera.className='camera';video.removeAttribute('src');active=false;message.textContent='Mất kết nối với bộ quét khuôn mặt.'}}
setInterval(refresh,500);refresh();
</script></html>"""


def handler_for(path, stream_url):
    page = PAGE.replace('const messages=', 'const STREAM_URL=' + json.dumps(stream_url) + ';\nconst messages=')
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            if self.path == '/status':
                body = json.dumps(current_state(path)).encode('utf-8')
                content_type = 'application/json'
            elif self.path == '/':
                body = page.encode('utf-8')
                content_type = 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Security-Policy', "default-src 'none'; connect-src 'self'; img-src http://127.0.0.1:*; script-src 'unsafe-inline'; style-src 'unsafe-inline'")
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    from .worker_config import load
    from .vendor_camera import stream_path
    cfg, _ = load(args.config)
    camera = cfg['mapping']['camera']
    if not cfg.get('staff_face_enabled') or not camera['enabled'] or not camera['stream_enabled']:
        raise RuntimeError('FACE_GUIDE_REQUIRES_FACE_AND_CAMERA')
    path = state_path(cfg)
    stream = f"http://127.0.0.1:{camera['stream_port']}" + stream_path(camera)
    with ThreadingHTTPServer(('127.0.0.1', PORT), handler_for(path, stream)) as server:
        print(f'FACE_GUIDE_READY http://127.0.0.1:{PORT}/', flush=True)
        server.serve_forever()


if __name__ == '__main__':
    main()
