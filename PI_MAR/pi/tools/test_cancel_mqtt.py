"""Cancellation races and persistent tombstones; real SQLite, no robot motion."""
import asyncio
import tempfile
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.step_store import StepStore
from mediassist.step_worker import StepWorker
from mediassist.contracts import StepError


class CancellationTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = str(Path(self.temp.name) / 'steps.db')
        self.store = StepStore(self.path, 'robot')
        self.step = dict(jobId=str(uuid4()), jobStepId=str(uuid4()), operationKey='job:move:1', stepType='MOVE', params={})
        self.worker = StepWorker.__new__(StepWorker)
        self.worker.cfg = {'robot_id': 'robot'}
        self.worker.store = self.store
        self.worker.shutting_down = False
        self.worker.active = None
        self.worker.outbox_wakeup = asyncio.Event()

    def tearDown(self):
        self.store.close()
        self.temp.cleanup()

    def cancel(self):
        identity = str(uuid4())
        return dict(commandId=identity, correlationId=identity, commandType='CANCEL_STEP',
                    jobId=self.step['jobId'], jobStepId=self.step['jobStepId'])

    async def test_cancel_overtakes_execute_and_survives_restart(self):
        command = self.cancel()
        with patch('mediassist.step_worker.validate_command'):
            await self.worker.command(command)
            await self.worker.command(command)
        receipt = self.store.pending()[0]['payload']
        self.assertTrue(receipt['accepted'])
        self.assertTrue(receipt['physicalStopConfirmed'])
        self.assertEqual(len(self.store.pending()), 1)
        self.store.close()
        self.store = StepStore(self.path, 'robot')
        with self.assertRaisesRegex(StepError, 'OPERATION_CANCELLED'):
            self.store.accept(self.step)

    async def test_late_cancel_of_stopped_operation_returns_proof(self):
        self.store.accept(self.step)
        self.store.finish(self.step, 'FAILED', {'physicalStopConfirmed': True})
        with patch('mediassist.step_worker.validate_command'):
            await self.worker.command(self.cancel())
        result = self.store.pending()[0]['payload']
        self.assertTrue(result['accepted'])
        self.assertTrue(result['physicalStopConfirmed'])

    async def test_unknown_stop_is_not_acknowledged_as_safe(self):
        self.store.accept(self.step)
        self.store.finish(self.step, 'RECOVERY_REQUIRED', {'physicalStopConfirmed': False})
        with patch('mediassist.step_worker.validate_command'):
            await self.worker.command(self.cancel())
        result = self.store.pending()[0]['payload']
        self.assertFalse(result['accepted'])
        self.assertFalse(result['physicalStopConfirmed'])
        self.assertTrue(self.store.blocked())

    async def test_foreign_cancel_does_not_cancel_active_task(self):
        self.worker.active = dict(self.step, jobId=str(uuid4()), jobStepId=str(uuid4()))
        with patch('mediassist.step_worker.validate_command'):
            await self.worker.command(self.cancel())
        self.assertIsNotNone(self.worker.active)


if __name__ == '__main__':
    unittest.main()
