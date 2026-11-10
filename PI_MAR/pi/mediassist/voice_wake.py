"""Speech/noise gating for wake capture; bounded RAM, no audio files."""
import array
import asyncio
from collections import deque
import logging
import math
import shutil
import sys
import time
from .contracts import StepError
from .speech import audio_device

SAMPLE_RATE = 16000
FRAME_SECONDS = .03
FRAME_BYTES = int(SAMPLE_RATE * FRAME_SECONDS) * 2


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
        if (not self.valid_number(self.minimum, .002, .1)
                or not self.valid_number(silence, .3, 1)
                or type(seconds) is not int or not 2 <= seconds <= 4
                or not self.valid_number(margin, 3, 20)
                or type(mode) is not int or not 0 <= mode <= 3):
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
        samples = array.array('h', frame)
        if sys.byteorder != 'little':
            samples.byteswap()
        level = math.sqrt(sum(value * value for value in samples) / len(samples)) / 32768.
        peak = max(abs(value) for value in samples) / 32768.
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
    print(f'VOICE_WAKE_LISTENING: {phrase}', flush=True)
    while True:
        pcm = await capture_phrase(player, settings, segmenter)
        try:
            if await asyncio.to_thread(ai.detect_wake, pcm):
                return
        finally:
            del pcm
