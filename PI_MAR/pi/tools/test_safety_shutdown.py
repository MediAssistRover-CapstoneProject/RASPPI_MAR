"""Offline shutdown regression tests; never access ROS or motors."""
from pathlib import Path
import signal
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.safety_node import spin_and_stop


class ShutdownTests(unittest.TestCase):
    def exercise(self, action, cleanup_error=False):
        events, handlers = [], {}
        ros, node = Mock(), Mock()
        alive = [True]
        ros.ok.side_effect = lambda: alive[0]
        ros.shutdown.side_effect = lambda: (events.append('shutdown'), alive.__setitem__(0, False))
        node.destroy_node.side_effect = lambda: events.append('destroy')
        node.get_logger.return_value.error.side_effect = lambda _: events.append('cannot_send')
        old = {signal.SIGINT: object(), signal.SIGTERM: object()}

        def set_signal(sig, handler):
            previous = handlers.get(sig, old[sig])
            handlers[sig] = handler
            return previous

        def spin(*args, **kwargs):
            if action == 'external_shutdown':
                alive[0] = False
            elif action == 'exception':
                raise ValueError('callback failed')
            else:
                handlers[action](action, None)
                handlers[action](action, None)

        def cleanup():
            self.assertTrue(alive[0], 'zero must precede ROS shutdown')
            events.append('zero')
            if cleanup_error:
                raise RuntimeError('publish failed')

        ros.spin_once.side_effect = spin
        with patch('mediassist.safety_node.signal.signal', side_effect=set_signal):
            if cleanup_error:
                with self.assertRaisesRegex(RuntimeError, 'publish failed'):
                    spin_and_stop(ros, node, cleanup)
            elif action == 'exception':
                with self.assertRaisesRegex(ValueError, 'callback failed'):
                    spin_and_stop(ros, node, cleanup)
            else:
                spin_and_stop(ros, node, cleanup)
        self.assertEqual(handlers, old)
        return events

    def test_sigint_sends_zero_before_destroy_and_shutdown(self):
        self.assertEqual(self.exercise(signal.SIGINT), ['zero', 'destroy', 'shutdown'])

    def test_sigterm_sends_zero_before_destroy_and_shutdown(self):
        self.assertEqual(self.exercise(signal.SIGTERM), ['zero', 'destroy', 'shutdown'])

    def test_callback_error_still_stops_and_propagates(self):
        self.assertEqual(self.exercise('exception'), ['zero', 'destroy', 'shutdown'])

    def test_publish_failure_is_not_reported_as_success(self):
        self.assertEqual(self.exercise(signal.SIGINT, True), ['zero', 'destroy', 'shutdown'])

    def test_external_shutdown_reports_zero_not_sent(self):
        self.assertEqual(self.exercise('external_shutdown'), ['cannot_send', 'destroy'])


if __name__ == '__main__':
    unittest.main()
