# Cancellation regression checks

From the repository root, using the Python environment with the Pi dependencies:

```powershell
python -m unittest discover -s PI_MAR/pi/tools -p test_cancel_mqtt.py
python -m unittest discover -s PI_MAR/pi/tools -p test_mqtt_bridge.py
python -m unittest discover -s PI_MAR/pi/tools -p test_step_recovery.py
python -m unittest discover -s PI_MAR/pi/tools -p test_safety_shutdown.py
```

For a real local broker on port 18884 (do not use the robot's production broker):

```powershell
$env:MAR_TEST_MQTT_PORT = '18884'
python -m unittest discover -s PI_MAR/pi/tools -p test_cancel_broker.py
```

The broker test uses a unique topic namespace, real Paho MQTT and a temporary SQLite
journal. Hardware stop is simulated. It verifies delivery, cancellation receipt,
worker cleanup and duplicate command handling. It is skipped unless the port is set.

Cancellation tombstones persist in the journal, preventing a delayed EXECUTE_STEP
from restarting a cancelled job/step. Do not clear the journal to release a robot
whose physical stop is unconfirmed. Use the existing operator recovery procedure.
Keep continuous fall detection enabled when cancelling an individual fall incident.
