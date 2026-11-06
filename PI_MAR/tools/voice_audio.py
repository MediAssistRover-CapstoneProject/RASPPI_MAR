"""Commission micro/loa and BE STT/TTS. Never starts ROS, MQTT or a workflow."""
import argparse
import array
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import ssl
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import wave

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from mediassist.be_token import NoRedirect,upload_token
from mediassist.contracts import StepError
from mediassist.speech import audio_device
from mediassist.worker_config import load
from mediassist.voice_client import VoiceClient,describe_audio

MAX_WAV=4*1024*1024


def command(argv,timeout):
    if not shutil.which(argv[0]):raise StepError('VOICE_ALSA_TOOLS_MISSING','Install alsa-utils inside MentorPi')
    try:result=subprocess.run(argv,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,timeout=timeout)
    except subprocess.TimeoutExpired:raise StepError('VOICE_AUDIO_PROCESS_TIMEOUT') from None
    if result.returncode:
        # ALSA diagnostics contain device names, not tokens or captured speech.
        print(result.stderr.decode(errors='replace')[-1000:],file=sys.stderr)
        raise StepError('VOICE_'+Path(argv[0]).name.upper()+'_FAILED')


def capture(device,seconds):
    with tempfile.TemporaryDirectory(prefix='mar-mic-') as folder:
        path=Path(folder)/'capture.wav'
        print('RECORDING: hãy nói một câu ngắn; thời gian '+str(seconds)+' giây.',flush=True)
        command(['arecord','-q','-D',device,'-t','wav','-f','S16_LE','-r','16000','-c','1','-d',str(seconds),str(path)],seconds+5)
        if path.stat().st_size>1024*1024:raise StepError('VOICE_RECORDING_TOO_LARGE')
        raw=path.read_bytes()
    info=describe_audio(raw);print('CAPTURE',json.dumps(info,ensure_ascii=False),flush=True)
    return raw


def save_audio(path,raw):
    if path is None:return
    path=path.expanduser().absolute()
    if path.is_symlink() or path.suffix.lower()!='.wav':raise StepError('VOICE_OUTPUT_MUST_BE_WAV_FILE')
    fd,temporary=tempfile.mkstemp(prefix='.mar-voice-',dir=path.parent)
    try:
        with os.fdopen(fd,'wb') as stream:stream.write(raw)
        os.replace(temporary,path)
    finally:
        if os.path.exists(temporary):os.unlink(temporary)
    print('WAV_SAVED',path)


def playback(device,raw):
    info=describe_audio(raw)
    with tempfile.TemporaryDirectory(prefix='mar-speaker-') as folder:
        path=Path(folder)/'reply.wav';path.write_bytes(raw)
        command(['aplay','-q','-D',device,str(path)],math.ceil(info['duration_sec'])+5)
    print('PLAYBACK_COMPLETED: ALSA đã phát xong; cần nghe thực tế để xác nhận loa.',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'pi/config/mqtt.worker.json')
    commands=parser.add_subparsers(dest='action',required=True)
    commands.add_parser('devices');commands.add_parser('status')
    for name in ('record','roundtrip'):
        p=commands.add_parser(name);p.add_argument('--seconds',type=int,choices=range(1,16),default=5)
        p.add_argument('--input-device',default='auto');p.add_argument('--output',type=Path)
        p.add_argument('--output-device',default='auto');p.add_argument('--play',action='store_true')
        if name=='roundtrip':p.add_argument('--language',choices=('vi','en'),default='vi')
    p=commands.add_parser('stt');p.add_argument('--file',type=Path,required=True);p.add_argument('--language',choices=('vi','en'),default='vi')
    p=commands.add_parser('tts');p.add_argument('--text',required=True);p.add_argument('--language',choices=('vi','en'),default='vi')
    p.add_argument('--output-device',default='auto');p.add_argument('--output',type=Path);p.add_argument('--play',action='store_true')
    p=commands.add_parser('chat',help='Hello/wake -> BE STT -> laptop RAG -> BE TTS; user audio stays in RAM')
    p.add_argument('--ai-config',type=Path,default=Path('/home/ubuntu/mar-secrets/voice-ai.json'))
    p.add_argument('--start-now',action='store_true',help='Start one conversation without waiting for the hardware wake event')
    p.add_argument('--once',action='store_true',help='Exit when the conversation ends instead of waiting for another wake')
    p.add_argument('--max-turns',type=int,choices=range(1,21),default=10)
    commands.add_parser('chat-daemon',help='Managed navigation idle listener; no extra ROS/MQTT worker')
    args=parser.parse_args()
    if args.action=='devices':
        for executable in ('arecord','aplay'):
            if not shutil.which(executable):raise StepError('VOICE_ALSA_TOOLS_MISSING')
            subprocess.run([executable,'-l'],check=True)
        return
    if args.action=='record':
        raw=capture(audio_device({'device':args.input_device}),args.seconds);save_audio(args.output,raw)
        if args.play:playback(audio_device({'device':args.output_device}),raw)
        return
    cfg,_=load(args.config.resolve(strict=True))
    if args.action=='chat-daemon':
        import asyncio,logging
        from mediassist.voice_runtime import run_daemon
        logging.basicConfig(level=logging.INFO,format='%(asctime)s %(levelname)s %(message)s')
        asyncio.run(run_daemon(cfg));return
    if args.action=='chat':
        import asyncio
        from mediassist.voice_chat import run_chat
        asyncio.run(run_chat(cfg,args.ai_config,args.start_now,args.once,args.max_turns));return
    client=VoiceClient(cfg,verbose=True)
    if args.action=='status':print(json.dumps(client.status(),ensure_ascii=False,indent=2));return
    if args.action=='stt':
        if args.file.stat().st_size>1024*1024:raise StepError('VOICE_RECORDING_TOO_LARGE')
        client.transcribe(args.file.read_bytes(),args.language);return
    if args.action=='roundtrip':
        raw=capture(audio_device({'device':args.input_device}),args.seconds)
        text=client.transcribe(raw,args.language)
    else:text=args.text
    # Roundtrip repeats only the recognized text; it never dispatches its intent.
    output=client.synthesize(text,args.language);save_audio(args.output,output)
    if args.play:playback(audio_device({'device':args.output_device}),output)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:
        print('VOICE_CHAT_STOPPED')
    except (StepError,OSError,ValueError,subprocess.SubprocessError) as exc:
        print('VOICE_CHECK_FAILED:',getattr(exc,'code',type(exc).__name__),str(exc),file=sys.stderr)
        sys.exit(1)
