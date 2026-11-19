import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from pymavlink import mavutil
from signal_trace import SignalReporter, trace_connection


class SignalTraceTests(unittest.TestCase):
    def test_records_match_encoded_command_not_a_prediction(self):
        class Wire:
            def __init__(self):
                self.data = bytearray()
            def write(self, data):
                self.data.extend(data)
        wire = Wire()
        reporter = SignalReporter('test')  # no observer socket needed
        connection = SimpleNamespace(
            mav=mavutil.mavlink.MAVLink(wire, srcSystem=252, srcComponent=191),
            recv_match=lambda **kwargs: None)
        trace_connection(connection, reporter)
        connection.mav.command_long_send(1, 1, 209, 0, 1, 0, 0, 0, 1, 0, 0)
        event = reporter.queue.get_nowait()
        decoded = mavutil.mavlink.MAVLink(None).parse_buffer(bytes(wire.data))[0]
        self.assertEqual(event['outcome'], 'sent')
        self.assertEqual(event['name'], 'MOTOR_TEST')
        for field in ('target_system', 'target_component', 'command', 'param1', 'param3', 'param4', 'param5'):
            self.assertEqual(event['payload'][field], getattr(decoded, field))
        self.assertEqual(event['payload']['source_system'], 252)

    def test_send_error_recorded_as_failure_and_still_raised(self):
        reporter = SignalReporter('test')
        mav = Mock(srcSystem=252, srcComponent=191)
        mav.command_long_send.side_effect = OSError('link gone')
        connection = SimpleNamespace(mav=mav, recv_match=lambda: None)
        trace_connection(connection, reporter)
        with self.assertRaises(OSError):
            connection.mav.command_long_send(1, 1, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
        event = reporter.queue.get_nowait()
        self.assertEqual(event['outcome'], 'failed')
        self.assertEqual(event['payload']['error'], 'link gone')

    def test_full_observer_queue_does_not_block_commands(self):
        reporter = SignalReporter('test')
        for i in range(600):
            reporter.record('pi_to_fc', 'HEARTBEAT', 'sent', {})
        mav = Mock(srcSystem=252, srcComponent=191)
        connection = SimpleNamespace(mav=mav, recv_match=lambda: None)
        trace_connection(connection, reporter)
        connection.mav.command_long_send(1, 1, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
        mav.command_long_send.assert_called_once()
        self.assertEqual(reporter.queue.qsize(), 512)
        self.assertGreater(reporter.dropped, 0)

    def test_ack_is_separate_from_command_and_preserves_source(self):
        reporter = SignalReporter('test')
        ack = Mock()
        ack.get_type.return_value = 'COMMAND_ACK'
        ack.get_srcSystem.return_value = 1
        ack.get_srcComponent.return_value = 1
        ack.to_dict.return_value = {'command': 209, 'result': 2}
        connection = SimpleNamespace(mav=Mock(), recv_match=lambda **kwargs: ack)
        trace_connection(connection, reporter)
        self.assertIs(connection.recv_match(blocking=False), ack)
        event = reporter.queue.get_nowait()
        self.assertEqual(event['direction'], 'fc_to_pi')
        self.assertEqual(event['outcome'], 'received')
        self.assertEqual(event['payload']['result'], 2)
        self.assertEqual(event['payload']['source_system'], 1)

    def test_proxy_preserves_parser_configuration(self):
        reporter = SignalReporter('test')
        mav = Mock()
        connection = SimpleNamespace(mav=mav, recv_match=lambda: None)
        trace_connection(connection, reporter)
        connection.mav.robust_parsing = True
        self.assertTrue(mav.robust_parsing)

    def test_mavlink_version_switch_does_not_remove_capture(self):
        reporter = SignalReporter('test')
        replacement = Mock(srcSystem=252, srcComponent=191)
        connection = SimpleNamespace(mav=Mock())
        def negotiate():
            connection.mav = replacement
            return None
        connection.recv_match = negotiate
        trace_connection(connection, reporter)
        connection.recv_match()
        connection.mav.command_long_send(1, 1, 209, 0, 1, 0, 0, 0, 1, 0, 0)
        replacement.command_long_send.assert_called_once()
        self.assertEqual(reporter.queue.get_nowait()['name'], 'MOTOR_TEST')


if __name__ == '__main__':
    unittest.main()
