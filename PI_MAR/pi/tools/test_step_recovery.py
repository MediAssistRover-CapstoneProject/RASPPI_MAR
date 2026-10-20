"""Durable, scoped recovery receipts; no hardware commands or network."""
import json
import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mediassist.step_store import StepStore
from mediassist.contracts import StepError


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.store=StepStore(str(Path(self.temp.name)/'steps.db'),'robot-test')
        self.step={'operationKey':'job:SAVE:1','jobStepId':'step-save','jobId':'job',
                   'stepType':'EXECUTE_SENSING','params':{'sensing_type':'MAPPING','action':'SAVE'}}
        self.store.accept(self.step)

    def tearDown(self):
        self.store.close();self.temp.cleanup()

    def receipt(self):
        return self.store.emit('COMMAND_ACK',{'commandType':'EXECUTE_STEP','commandId':'command-save'},self.step,'command-save')

    def test_failed_mapping_operator_receipt_uses_original_command_and_preserves_old_events(self):
        ack=self.receipt()
        old=self.store.emit('STEP_ERROR',{'errorCode':'MAP_UPLOAD_UNAUTHORIZED'},self.step,'command-save')
        self.store.finish(self.step,'FAILED',{'errorCode':'MAP_UPLOAD_UNAUTHORIZED'})
        receipt=self.store.reconcile('step-save','Robot and SLAM stopped; local map retained',True)
        self.assertEqual(receipt['correlationId'],'command-save')
        self.assertEqual([r['messageId'] for r in self.store.pending()],[receipt['messageId']])
        self.assertFalse(self.store.is_acked(old['messageId']))
        row=self.store.db.execute('SELECT body,superseded_by FROM worker_outbox WHERE message_id=?',(ack['messageId'],)).fetchone()
        self.assertEqual(json.loads(row[0]),ack);self.assertEqual(row[1],receipt['messageId'])
        self.assertEqual(self.store.previous(self.step)['status'],'CANCELLED')
        self.assertNotIn('physicalStopConfirmed',receipt['payload'])

    def test_generic_recovery_keeps_command_correlation(self):
        self.receipt();self.store.finish(self.step,'RECOVERY_REQUIRED',{})
        result=self.store.reconcile('step-save','Inspected robot after restart')
        self.assertEqual(result['correlationId'],'command-save')

    def test_failed_mapping_requires_explicit_option(self):
        self.receipt();self.store.finish(self.step,'FAILED',{})
        with self.assertRaises(StepError):self.store.reconcile('step-save','Stopped')

    def test_success_cannot_be_rewritten(self):
        self.receipt();self.store.finish(self.step,'SUCCEEDED',{})
        with self.assertRaises(StepError):self.store.reconcile('step-save','Stopped',True)
        self.assertEqual(self.store.previous(self.step)['status'],'SUCCEEDED')

    def test_failed_non_mapping_cannot_use_mapping_override(self):
        step=dict(self.step,operationKey='job:WAIT:1',jobStepId='step-wait',stepType='WAIT')
        self.store.accept(step);self.store.finish(step,'FAILED',{})
        with self.assertRaises(StepError):self.store.reconcile('step-wait','Stopped',True)

    def test_missing_original_receipt_refuses_without_changing_status(self):
        self.store.finish(self.step,'RECOVERY_REQUIRED',{})
        with self.assertRaisesRegex(StepError,'RECONCILIATION_COMMAND_RECEIPT_MISSING'):
            self.store.reconcile('step-save','Stopped')
        self.assertEqual(self.store.previous(self.step)['status'],'RECOVERY_REQUIRED')

    def test_legacy_database_migration_keeps_rows(self):
        target=str(Path(self.temp.name)/'legacy.db')
        db=sqlite3.connect(target)
        db.execute('CREATE TABLE worker_outbox(seq INTEGER PRIMARY KEY AUTOINCREMENT,message_id TEXT UNIQUE NOT NULL,body TEXT NOT NULL,acked INTEGER NOT NULL DEFAULT 0)')
        db.execute('INSERT INTO worker_outbox(message_id,body) VALUES(?,?)',('old-event','{"messageId":"old-event"}'))
        db.commit();db.close()
        migrated=StepStore(target,'robot-test')
        try:self.assertEqual(migrated.pending(),[{'messageId':'old-event'}])
        finally:migrated.close()


if __name__=='__main__':unittest.main()
