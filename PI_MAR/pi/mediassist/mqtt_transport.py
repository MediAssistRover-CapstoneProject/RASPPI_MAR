"""MQTT 5 transport. Application acknowledgements remain in StepStore."""
import asyncio
import json
import logging
import time
import uuid
from pathlib import Path
import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes
from .contracts import StepError, canonical
from .worker_config import tls


class MqttSession:
    def __init__(self, cfg):
        self.cfg = cfg
        self.loop = asyncio.get_running_loop()
        self.id = str(uuid.uuid4())
        self.prefix = 'mar/robots/' + cfg['robot_id']
        self.queue = asyncio.Queue(maxsize=64)
        self.ready = asyncio.Event()
        self.failure = None
        self.last_be = time.monotonic()
        self.publish_lock = asyncio.Lock()
        self.next_publish = 0.0
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
            client_id='mar-pi-' + cfg['robot_id'], protocol=mqtt.MQTTv5,
            reconnect_on_failure=False)
        self.client.connect_timeout = cfg['connect_timeout_sec']
        self.client.max_inflight_messages_set(16)
        self.client.max_queued_messages_set(64)
        self.client.tls_set_context(tls(cfg))
        secret = Path(cfg['mqtt']['password_file']).read_text(encoding='utf-8').strip()
        if not secret:
            raise StepError('MQTT_PASSWORD_EMPTY')
        self.client.username_pw_set(cfg['mqtt']['username'], secret)
        self.client.will_set(self.prefix + '/presence', canonical({
            'robotId': cfg['robot_id'], 'transportSessionId': self.id,
            'online': False}), qos=1, retain=True)
        self.client.on_connect = self.on_connect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    def fail(self, code):
        self.failure = code
        self.ready.set()

    def on_connect(self, client, userdata, flags, reason, properties):
        if reason.is_failure:
            self.loop.call_soon_threadsafe(self.fail, 'MQTT_CONNECT_REJECTED')
            return
        options = mqtt.SubscribeOptions(qos=1, retainAsPublished=True, retainHandling=2)
        rc, _ = client.subscribe([(self.prefix+'/commands', options),
                                  (self.prefix+'/event-acks', options),
                                  (self.prefix+'/mapping/control', mqtt.SubscribeOptions(qos=0,retainAsPublished=True,retainHandling=2))])
        if rc != mqtt.MQTT_ERR_SUCCESS:
            self.loop.call_soon_threadsafe(self.fail, 'MQTT_SUBSCRIBE_FAILED')

    def on_subscribe(self, client, userdata, mid, reasons, properties):
        if any(r.is_failure for r in reasons):
            self.loop.call_soon_threadsafe(self.fail, 'MQTT_SUBSCRIBE_REJECTED')
        else:
            self.loop.call_soon_threadsafe(self.ready.set)

    def on_disconnect(self, client, userdata, flags, reason, properties):
        self.loop.call_soon_threadsafe(self.fail, 'MQTT_DISCONNECTED')

    def on_message(self, client, userdata, message):
        if message.retain or len(message.payload) > 1048576:
            self.loop.call_soon_threadsafe(self.fail, 'RETAINED_OR_OVERSIZE_MESSAGE')
            return
        if message.topic.endswith('/mapping/control') and (message.qos != 0 or len(message.payload)>4096):
            self.loop.call_soon_threadsafe(self.fail, 'INVALID_MAPPING_CONTROL_LANE')
            return
        self.loop.call_soon_threadsafe(self.enqueue, message.topic, bytes(message.payload))

    def enqueue(self, topic, raw):
        if self.queue.full():
            self.fail('MQTT_RECEIVE_QUEUE_FULL')
        else:
            self.queue.put_nowait((topic, raw))

    async def __aenter__(self):
        props = Properties(PacketTypes.CONNECT)
        props.SessionExpiryInterval = 0
        props.MaximumPacketSize = 1048576
        props.ReceiveMaximum = 16
        try:
            await asyncio.to_thread(self.client.connect, self.cfg['mqtt']['host'],
                self.cfg['mqtt']['port'], 10, clean_start=True, properties=props)
            self.client.loop_start()
            await asyncio.wait_for(self.ready.wait(), self.cfg['connect_timeout_sec'])
            if self.failure:
                raise StepError(self.failure)
            await self.publish('/presence', {'robotId': self.cfg['robot_id'],
                'transportSessionId': self.id, 'online': True}, qos=1, retain=True)
            return self
        except BaseException:
            await self.__aexit__(None, None, None)
            raise

    async def publish(self, suffix, value, qos=1, retain=False):
        if self.failure:
            raise StepError(self.failure)
        payload = canonical(value)
        if len(payload.encode('utf-8')) > 1000000:
            raise StepError('MQTT_PAYLOAD_TOO_LARGE')
        props = Properties(PacketTypes.PUBLISH)
        props.PayloadFormatIndicator = 1
        props.ContentType = 'application/json'
        # Telemetry expires quickly. Durable events are replayed by StepStore.
        if not retain:
            props.MessageExpiryInterval = 15 if qos == 0 else 60
        # A shared gate covers events, heartbeat, presence and map previews.
        # EMQX Serverless accepts at most 10 application publishes/client/second.
        async with self.publish_lock:
            await asyncio.sleep(max(0.0, self.next_publish-time.monotonic()))
            if self.failure:
                raise StepError(self.failure)
            info = self.client.publish(self.prefix+suffix, payload, qos=qos, retain=retain, properties=props)
            self.next_publish = time.monotonic()+1/self.cfg['mqtt'].get('max_publish_per_sec',8)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            raise StepError('MQTT_PUBLISH_FAILED')
        if qos:
            deadline = time.monotonic() + self.cfg['connect_timeout_sec']
            while not info.is_published():
                if self.failure or time.monotonic() >= deadline:
                    raise StepError(self.failure or 'MQTT_PUBLISH_TIMEOUT')
                await asyncio.sleep(0.02)

    async def send(self, raw):
        value = json.loads(raw)
        value['transportSessionId'] = self.id
        heartbeat = value.get('messageType') == 'HEARTBEAT'
        await self.publish('/heartbeat' if heartbeat else '/events', value, qos=0 if heartbeat else 1)

    async def recv(self):
        while True:
            if self.failure:
                raise StepError(self.failure)
            if time.monotonic()-self.last_be > self.cfg['mqtt']['be_timeout_sec']:
                raise StepError('BE_HEARTBEAT_TIMEOUT')
            try:
                topic, raw = await asyncio.wait_for(self.queue.get(), 0.25)
            except asyncio.TimeoutError:
                continue
            try:
                value = json.loads(raw.decode('utf-8'), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            except (ValueError, UnicodeError):
                raise StepError('INVALID_MQTT_JSON') from None
            if not isinstance(value, dict):
                raise StepError('OBJECT_REQUIRED')
            if value.get('robotId') != self.cfg['robot_id']:
                raise StepError('WRONG_ROBOT')
            if value.get('transportSessionId') != self.id:
                # Stale packets from an earlier connection cannot start a physical action.
                logging.warning('Ignored packet for an old transport session')
                continue
            kind = value.get('messageType')
            allowed = ({'MAPPING_CONTROL'} if topic.endswith('/mapping/control') else
                       {'COMMAND'} if topic.endswith('/commands') else {'HELLO_ACK','EVENT_ACK','SERVER_HEARTBEAT','NAV_PROFILE'})
            if kind not in allowed:
                raise StepError('WRONG_MQTT_TOPIC')
            self.last_be = time.monotonic()
            if kind == 'SERVER_HEARTBEAT':
                continue
            if kind != 'MAPPING_CONTROL':value.pop('transportSessionId')
            # Command schema already has robotId; legacy EVENT_ACK handler accepts it too.
            return canonical(value)

    async def __aexit__(self, *args):
        try:
            if self.client.is_connected() and not self.failure:
                await self.publish('/presence', {'robotId': self.cfg['robot_id'],
                    'transportSessionId': self.id, 'online': False}, qos=1, retain=True)
        finally:
            self.client.disconnect()
            await asyncio.to_thread(self.client.loop_stop)
