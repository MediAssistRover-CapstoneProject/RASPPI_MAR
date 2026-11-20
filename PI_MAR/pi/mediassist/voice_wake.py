"""Speech/noise gating for wake capture; bounded RAM, no audio files."""
import array
import asyncio
from collections import deque
import logging
import math
import shutil
import sys
import time
import uuid
from .contracts import StepError
from .speech import audio_device

SAMPLE_RATE = 16000
FRAME_SECONDS = .03
FRAME_BYTES = int(SAMPLE_RATE * FRAME_SECONDS) * 2


class PcmConditioner:
    """One-pole high-pass for DC/low-frequency rumble, never source separation.

    Retain filter state across 30 ms frames so frame boundaries do not click.
    Speech above the cut-off keeps its waveform; no blanket gain/noise gate
    clips the quiet initial consonants before the speech detector sees them.
    """
    def __init__(self,cutoff_hz=70):
        self.previous_input=0.;self.previous_output=0.
        self.coefficient=math.exp(-2*math.pi*cutoff_hz/SAMPLE_RATE)

    def process(self,frame):
        samples=array.array('h',frame)
        if sys.byteorder!='little':samples.byteswap()
        for index,value in enumerate(samples):
            filtered=self.coefficient*(self.previous_output+value-self.previous_input)
            self.previous_input=value;self.previous_output=filtered
            samples[index]=max(-32768,min(32767,round(filtered)))
        if sys.byteorder!='little':samples.byteswap()
        return samples.tobytes()


class WakeSegmenter:
    """Require speech above the background; loud non-speech cannot trigger alone.

    A single mono channel cannot identify or isolate the loudest speaker. The
    energy margin only prefers speech that stands out from the ambient noise.
    """
    def __init__(self, settings, detector=None):
        self.minimum = settings.get('vad_threshold', .004)
        silence = settings.get('silence_sec', .65)
        seconds = settings.get('max_seconds', 4)
        margin = settings.get('noise_margin_db', 6.)
        mode = settings.get('vad_mode', 2)
        filtered=settings.get('noise_filter_enabled',True)
        cutoff=settings.get('highpass_hz',70)
        if (not self.valid_number(self.minimum, .002, .1)
                or not self.valid_number(silence, .3, 1)
                or type(seconds) is not int or not 2 <= seconds <= 4
                or not self.valid_number(margin, 3, 20)
                or type(mode) is not int or not 0 <= mode <= 3
                or type(filtered) is not bool or not self.valid_number(cutoff,40,150)):
            raise StepError('VOICE_WAKE_CONFIG_INVALID')
        if detector is None:
            try:
                import webrtcvad
            except ImportError:
                raise StepError('VOICE_VAD_DEPENDENCY_MISSING',
                                'Install the pinned webrtcvad-wheels dependency') from None
            detector = webrtcvad.Vad(mode)
        self.detector = detector
        self.margin = 10 ** (margin / 20)
        self.silence_frames = math.ceil(silence / FRAME_SECONDS)
        self.frame_limit = int(seconds / FRAME_SECONDS)
        self.noise = deque(maxlen=67)
        self.noise_floor = self.minimum / self.margin
        self.last_report = 0.
        self.counts = dict(short=0, overlong=0)
        self.conditioner=PcmConditioner(cutoff) if filtered else None
        self.reset()

    @staticmethod
    def valid_number(value, low, high):
        return type(value) in (int, float) and math.isfinite(value) and low <= value <= high

    def reset(self):
        # Retain only numeric noise estimates between captures, never old audio.
        self.lead = deque(maxlen=10)
        self.onset = deque(maxlen=3)
        self.frames = []
        self.voiced = self.quiet = self.clipped = 0
        self.peak = 0.
        self.overlong = False

    def push(self, frame):
        if len(frame) != FRAME_BYTES:
            raise StepError('VOICE_MIC_STREAM_FAILED')
        original=array.array('h',frame)
        if sys.byteorder!='little':original.byteswap()
        peak=max(abs(value) for value in original)/32768.
        if self.conditioner:frame=self.conditioner.process(frame)
        samples = array.array('h', frame)
        if sys.byteorder != 'little':
            samples.byteswap()
        level = math.sqrt(sum(value * value for value in samples) / len(samples)) / 32768.
        speech = self.detector.is_speech(frame, SAMPLE_RATE)
        if not speech:
            # A lower quantile prevents a single bang from raising the noise gate.
            self.noise.append(level)
            floor = sorted(self.noise)[int((len(self.noise) - 1) * .3)]
            self.noise_floor += .1 * (floor - self.noise_floor)
        threshold = max(self.minimum, self.noise_floor * self.margin)
        # Keep softer phonemes after onset without letting ambient noise hold it open.
        release = max(self.minimum * .75, self.noise_floor * 1.25)
        active = speech and level >= (release if self.frames else threshold)
        self.quiet = 0 if active else self.quiet + 1
        if self.overlong:
            if self.quiet >= self.silence_frames:
                self.reset()
            self.report('WAIT_QUIET', threshold)
            return None
        if not self.frames:
            self.lead.append((frame, active, peak))
            self.onset.append(active)
            if sum(self.onset) >= 2:
                self.frames = [item[0] for item in self.lead]
                self.voiced = sum(item[1] for item in self.lead)
                self.peak = max(item[2] for item in self.lead)
                self.clipped = sum(item[2] >= .999 for item in self.lead)
                self.lead.clear()
            self.report('LISTENING', threshold)
            return None

        self.frames.append(frame)
        self.voiced += int(active)
        self.peak = max(self.peak, peak)
        self.clipped += int(peak >= .999)
        if len(self.frames) >= self.frame_limit:
            # Do not accept a clipped prefix of a long/background conversation.
            self.counts['overlong'] += 1
            self.report('TOO_LONG', threshold, force=True)
            self.frames.clear()
            self.overlong = True
        elif self.quiet >= self.silence_frames:
            if self.voiced >= 6:
                pcm = b''.join(self.frames)
                self.report('CANDIDATE', threshold, force=True)
                self.reset()
                return pcm
            self.counts['short'] += 1
            self.reset()
        return None

    def report(self, state, threshold, force=False):
        now = time.monotonic()
        if not force and now - self.last_report < 30:
            return
        logging.info('VOICE_WAKE_AUDIO state=%s noise_rms=%.5f threshold=%.5f '
                     'audio_ms=%d speech_ms=%d peak=%.4f clipped_frames=%d '
                     'short_dropped=%d long_dropped=%d',
                     state, self.noise_floor, threshold,
                     round(len(self.frames) * FRAME_SECONDS * 1000),
                     round(self.voiced * FRAME_SECONDS * 1000), self.peak, self.clipped,
                     self.counts['short'], self.counts['overlong'])
        self.last_report = now


async def capture_phrase(player, settings, segmenter=None):
    from .voice_chat import stop_process
    segmenter = segmenter or WakeSegmenter(settings)
    if segmenter.frames:
        raise StepError('VOICE_WAKE_CONFIG_INVALID')
    recorder = shutil.which('arecord')
    if not recorder:
        raise StepError('VOICE_ALSA_TOOLS_MISSING')

    async with player.lock:
        proc = await asyncio.create_subprocess_exec(
            recorder, '-q', '-D', audio_device(player.cfg), '-t', 'raw',
            '-f', 'S16_LE', '-r', str(SAMPLE_RATE), '-c', '1',
            stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        try:
            while True:
                try:
                    frame = await asyncio.wait_for(proc.stdout.readexactly(FRAME_BYTES), 1.5)
                except (asyncio.IncompleteReadError, asyncio.TimeoutError):
                    raise StepError('VOICE_MIC_STREAM_FAILED') from None

                pcm = segmenter.push(frame)
                if pcm is not None:
                    return pcm
        finally:
            try:
                await stop_process(proc)
            finally:
                segmenter.reset()


async def wait_phrase(player,ai,settings):
    phrase = settings.get('phrase', 'M A')
    if not isinstance(phrase, str) or not 1 <= len(phrase.strip()) <= 80:
        raise StepError('VOICE_WAKE_CONFIG_INVALID')
    segmenter = WakeSegmenter(settings)
    max_age=settings.get('candidate_max_age_sec',2.5)
    if not WakeSegmenter.valid_number(max_age,.5,5):
        raise StepError('VOICE_WAKE_CONFIG_INVALID')
    recorder=shutil.which('arecord')
    if not recorder:raise StepError('VOICE_ALSA_TOOLS_MISSING')
    queue=asyncio.Queue(maxsize=2)
    print(f'VOICE_WAKE_LISTENING: {phrase}', flush=True)

    async def capture():
        from .voice_chat import stop_process
        async with player.lock:
            proc=await asyncio.create_subprocess_exec(recorder,'-q','-D',audio_device(player.cfg),'-t','raw',
                '-f','S16_LE','-r',str(SAMPLE_RATE),'-c','1',stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.DEVNULL)
            try:
                while True:
                    try:frame=await asyncio.wait_for(proc.stdout.readexactly(FRAME_BYTES),1.5)
                    except (asyncio.IncompleteReadError,asyncio.TimeoutError):
                        raise StepError('VOICE_MIC_STREAM_FAILED') from None
                    pcm=segmenter.push(frame)
                    if pcm is not None:
                        if queue.full():
                            queue.get_nowait()
                            logging.info('VOICE_WAKE_DROPPED reason=QUEUE_REPLACED')
                        queue.put_nowait((time.monotonic(),str(uuid.uuid4()),pcm))
            finally:
                await stop_process(proc)
                segmenter.reset()

    producer=asyncio.create_task(capture())
    getter=None;completed=False
    try:
        while True:
            getter=asyncio.create_task(queue.get())
            done,_=await asyncio.wait((producer,getter),return_when=asyncio.FIRST_COMPLETED)
            if producer in done:
                producer.result()
                raise StepError('VOICE_MIC_STREAM_FAILED')
            captured,request_id,pcm=getter.result();getter=None
            queued_ms=round((time.monotonic()-captured)*1000)
            if queued_ms>max_age*1000:
                logging.info('VOICE_WAKE_DROPPED request_id=%s reason=STALE queued_ms=%d',request_id,queued_ms)
                del pcm
                continue
            started=time.monotonic()
            try:
                matched=await asyncio.to_thread(ai.detect_wake,pcm,request_id)
                logging.info('VOICE_WAKE_PIPELINE request_id=%s queued_ms=%d roundtrip_ms=%d candidate_ms=%d',
                             request_id,queued_ms,round((time.monotonic()-started)*1000),round(len(pcm)/32))
                if matched:
                    completed=True
                    return
            finally:del pcm
    finally:
        if getter is not None:getter.cancel()
        producer.cancel()
        await asyncio.gather(producer,*([getter] if getter else []),return_exceptions=True)
        cancel=getattr(ai,'cancel_pending',None)
        if cancel and not completed:cancel()
        while not queue.empty():queue.get_nowait()
