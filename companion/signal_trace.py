"""Best-effort application signal records; never a prerequisite for control.

Capture send completion/failure, not physical UART delivery or actuator action.
Bounded, nonblocking producer queue; a separate thread sends observer UDP.
"""
import json
import queue
import socket
import threading
import time
import uuid


class SignalReporter:
    def __init__(self, source, host=None, port=5001):
        self.source = source
        self.target = (host, port) if host else None
        self.port = port
        self.session = uuid.uuid4().hex[:12]
        self.sequence = 0
        self.dropped = 0
        self.queue = queue.Queue(maxsize=512)
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def set_host(self, host):
        self.target = (host, self.port)

    def record(self, direction, name, outcome, payload):
        # Monitoring must not turn a successfully sent command into an error.
        try:
            with self.lock:
                self.sequence += 1
                event = dict(version=1, type='SIGNAL', id=f'{self.session}:{self.sequence}',
                             source=self.source, direction=direction, name=name,
                             outcome=outcome, timestamp_ms=int(time.time() * 1000),
                             payload=payload, observer_dropped=self.dropped)
            self.queue.put_nowait(event)
        except queue.Full:
            self.dropped += 1
        except Exception:
            self.dropped += 1

    def _run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.setblocking(False)
                while not self.stopping.is_set() or not self.queue.empty():
                    try:
                        event = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    try:
                        if self.target is None:
                            self.dropped += 1
                            continue
                        data = json.dumps(event, separators=(',', ':')).encode()
                        if len(data) > 8000:
                            self.dropped += 1
                            continue
                        sock.sendto(data, self.target)
                    except (OSError, TypeError, ValueError):
                        self.dropped += 1
        except OSError:
            self.dropped += 1

    def close(self):
        self.stopping.set()
        if self.thread:
            self.thread.join(timeout=0.5)


COMMAND_NAMES = {209: 'MOTOR_TEST', 400: 'ARM_DISARM',
                 511: 'SET_MESSAGE_INTERVAL', 512: 'REQUEST_MESSAGE'}


class TracedMav:
    def __init__(self, mav, reporter):
        self._mav = mav
        self._reporter = reporter

    def __getattr__(self, name):
        return getattr(self._mav, name)

    def __setattr__(self, name, value):
        if name.startswith('_'):
            object.__setattr__(self, name, value)
        else:
            setattr(self._mav, name, value)

    def _send(self, method, name, payload, args, kwargs):
        try:
            result = getattr(self._mav, method)(*args, **kwargs)
        except Exception as error:
            self._reporter.record('pi_to_fc', name, 'failed', {**payload, 'error': str(error)})
            raise
        self._reporter.record('pi_to_fc', name, 'sent', payload)
        return result

    def command_long_send(self, *args, **kwargs):
        keys = ['target_system', 'target_component', 'command', 'confirmation',
                'param1', 'param2', 'param3', 'param4', 'param5', 'param6', 'param7']
        payload = {**dict(zip(keys, args)), **kwargs,
                   'source_system': self._mav.srcSystem, 'source_component': self._mav.srcComponent}
        return self._send('command_long_send', COMMAND_NAMES.get(payload.get('command'), 'COMMAND_LONG'),
                          payload, args, kwargs)

    def heartbeat_send(self, *args, **kwargs):
        payload = {**dict(zip(['type', 'autopilot', 'base_mode', 'custom_mode', 'system_status'], args)),
                   **kwargs, 'source_system': self._mav.srcSystem,
                   'source_component': self._mav.srcComponent}
        return self._send('heartbeat_send', 'HEARTBEAT', payload, args, kwargs)


def trace_connection(connection, reporter):
    if reporter is None:
        return connection
    connection.mav = TracedMav(connection.mav, reporter)
    original_recv = connection.recv_match

    def recv_match(*args, **kwargs):
        try:
            message = original_recv(*args, **kwargs)
        finally:
            # pymavlink replaces its encoder when the first MAVLink 2 packet
            # arrives. Keep capture installed after protocol negotiation.
            if not isinstance(connection.mav, TracedMav):
                connection.mav = TracedMav(connection.mav, reporter)
        if message is not None and message.get_type() in ('COMMAND_ACK', 'STATUSTEXT'):
            reporter.record('fc_to_pi', message.get_type(), 'received',
                            {**message.to_dict(), 'source_system': message.get_srcSystem(),
                             'source_component': message.get_srcComponent()})
        return message

    connection.recv_match = recv_match
    return connection
