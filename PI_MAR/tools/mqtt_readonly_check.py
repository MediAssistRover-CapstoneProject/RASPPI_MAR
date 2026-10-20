#!/usr/bin/env python3
"""Passive MQTT5/TLS subscribe test for MAR MentorPi. NO publishes or motor commands.

Run in the MentorPi container, from the Pi project directory:
  cd /home/ubuntu/mediassist-mqtt-2026-10-08/pi
  .venv/bin/python ../tools/mqtt_readonly_check.py
"""
import json
import sys
import threading
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))
import paho.mqtt.client as mqtt
from mediassist.worker_config import load, tls

CONFIG = Path('config/mqtt.worker.json')
DURATION_SECONDS = 25

def main():
    cfg, _ = load(CONFIG)
    rid = cfg['robot_id']
    prefix = f'mar/robots/{rid}'
    # Unique diagnostic ID: MUST NOT take over the production worker client ID.
    client_id = f'mar-diag-{uuid.uuid4().hex[:12]}'
    ready = threading.Event()
    outcome = {'suback': False, 'count': 0, 'failed': False}
    topics = [
        (prefix + '/commands', mqtt.SubscribeOptions(qos=1, retainHandling=2)),
        (prefix + '/event-acks', mqtt.SubscribeOptions(qos=1, retainHandling=2)),
        (prefix + '/mapping/control', mqtt.SubscribeOptions(qos=0, retainHandling=2)),
    ]
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id,
                         protocol=mqtt.MQTTv5, reconnect_on_failure=False)
    client.tls_set_context(tls(cfg))
    secret = Path(cfg['mqtt']['password_file']).read_text(encoding='utf-8').strip()
    client.username_pw_set(cfg['mqtt']['username'], secret)

    def on_connect(c, userdata, flags, reason, properties):
        if reason.is_failure:
            print('MQTT CONNACK REJECTED:', reason, flush=True)
            outcome['failed'] = True
            ready.set()
            return
        print('MQTT CONNECTED (CONNACK OK), subscribing:', flush=True)
        for topic, _ in topics:
            print(' ', topic, flush=True)
        rc, _ = c.subscribe(topics)
        if rc != mqtt.MQTT_ERR_SUCCESS:
            print('MQTT SUBSCRIBE REQUEST FAILED: rc=', rc, flush=True)
            outcome['failed'] = True
            ready.set()

    def on_subscribe(c, userdata, mid, reasons, properties):
        for (topic, _), reason in zip(topics, reasons):
            print('SUBACK:', topic.split('/')[-1], '=>', reason, flush=True)
        outcome['suback'] = len(reasons) == len(topics) and all(not r.is_failure for r in reasons)
        if not outcome['suback']:
            outcome['failed'] = True
        print('ALL_SUBSCRIPTIONS_ALLOWED:', outcome['suback'], flush=True)
        ready.set()

    def on_message(c, userdata, msg):
        outcome['count'] += 1
        try:
            envelope = json.loads(msg.payload.decode('utf-8'))
            kind = envelope.get('messageType', '(no messageType)')
            accepted = envelope.get('accepted', None)
        except (UnicodeError, ValueError, AttributeError):
            kind, accepted = '(non-JSON)', None
        # Deliberately never print raw payloads: commands may contain sensitive data.
        print('RECEIVED:', msg.topic, 'type=', kind,
              'bytes=', len(msg.payload), 'qos=', msg.qos,
              'retained=', msg.retain, 'accepted=', accepted, flush=True)

    def on_disconnect(c, userdata, flags, reason, properties):
        print('MQTT DISCONNECTED:', reason, flush=True)
        if not ready.is_set():
            outcome['failed'] = True
            ready.set()

    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    print('Testing broker', cfg['mqtt']['host'], 'port', cfg['mqtt']['port'], flush=True)
    print('Read-only diagnostic client:', client_id, '(not the production worker)', flush=True)
    try:
        client.connect(cfg['mqtt']['host'], cfg['mqtt']['port'], keepalive=10)
        client.loop_start()
        if not ready.wait(12):
            print('TIMEOUT waiting for CONNACK/SUBACK', flush=True)
            return 2
        if not outcome['suback']:
            print('SUBSCRIBE NOT CONFIRMED; inspect broker ACL/client-id restrictions.', flush=True)
            return 3
        print(f'Listening for {DURATION_SECONDS}s (no messages expected unless BE publishes)...', flush=True)
        time.sleep(DURATION_SECONDS)
        print('Messages observed:', outcome['count'], flush=True)
        print('NOTE: 0 messages does NOT mean subscribe failed; the broker may be idle.', flush=True)
        print('NOTE: this separate test client does not prove the production worker is online.', flush=True)
        return 0
    except Exception as exc:
        print('ERROR:', type(exc).__name__, str(exc), flush=True)
        return 1
    finally:
        try:
            client.disconnect()
            client.loop_stop()
        except Exception:
            pass

if __name__ == '__main__':
    raise SystemExit(main())
