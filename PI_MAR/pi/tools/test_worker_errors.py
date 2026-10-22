"""A numeric HTTP error must finish the step and report failure, not crash handling."""
from datetime import datetime, timedelta, timezone
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock
from urllib.error import HTTPError

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.contracts import StepError
from mediassist.step_worker import StepWorker


class WorkerErrorTests(unittest.IsolatedAsyncioTestCase):
    async def execute_failure(self, error, step_type='INTERACT', stop_confirmed=True):
        store = Mock()
        store.emit.return_value = {'messageId': 'started'}
        adapter = SimpleNamespace(mapping=None, motion_uncertain=False,
                                  perform=AsyncMock(side_effect=error),
                                  stop=AsyncMock(return_value=stop_confirmed))
        worker = StepWorker({}, {}, store, adapter)
        worker.acknowledged = AsyncMock()
        step = {'stepType': step_type, 'params': {}, 'timeoutSec': 10}
        worker.active = step
        await worker.execute(step, 'correlation', 'accepted',
                             (datetime.now(timezone.utc) + timedelta(minutes=1)).isoformat())
        self.assertIsNone(worker.active)
        store.finish.assert_called_once_with(step, worker.last_result['status'], worker.last_result)
        self.assertEqual(store.emit.call_args.args[0], 'STEP_ERROR')
        return worker, adapter, store.emit.call_args.args[1]

    async def test_http_401_records_failure_and_returns_without_attribute_error(self):
        worker, _, event = await self.execute_failure(HTTPError('https://be.example', 401, 'Unauthorized', {}, None))
        self.assertEqual(worker.last_result['status'], 'FAILED')
        self.assertEqual(event['errorCode'], 'STEP_EXECUTION_ERROR')

    async def test_audio_error_keeps_structured_details(self):
        error = StepError('AUDIO_PLAYBACK_FAILED')
        error.details = {'code': 'AUDIO_PLAYBACK_FAILED', 'exitCode': 1}
        _, _, event = await self.execute_failure(error)
        self.assertEqual(event['errorCode'], error.code)
        self.assertEqual(event['details'], error.details)

    async def test_move_http_failure_still_requires_physical_stop_confirmation(self):
        worker, adapter, event = await self.execute_failure(
            HTTPError('https://be.example', 500, 'Failed', {}, None), 'MOVE', False)
        adapter.stop.assert_awaited_once()
        self.assertEqual(worker.last_result['status'], 'RECOVERY_REQUIRED')
        self.assertEqual(event['errorCode'], 'PHYSICAL_STOP_UNCONFIRMED')


if __name__ == '__main__':
    unittest.main()
