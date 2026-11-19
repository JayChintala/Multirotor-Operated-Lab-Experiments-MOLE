#!/usr/bin/env python3
"""Manual-only bench kill receiver for control_center_backend.py (UDP 5002).

KILL latches repeated motor-test cancellation and force-disarm until reset.
KILL_HEARTBEAT only requests status; missing heartbeats do not trigger a stop.
No armed-state or heartbeat check gates transmission. RPM is not inferred.
"""
import argparse
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import json
import socket
import threading
import time

from pymavlink import mavutil
from signal_trace import SignalReporter, trace_connection

DEFAULT_KILL_FILE = Path('/tmp') / ('mole-bench-kill-%d' % os.getuid())


@contextmanager
def bench_command_lock(marker):
    """Serialize latching with the bench producer's check-and-send operation."""
    with Path(str(marker) + '.lock').open('a') as lock_file:
        fcntl.flock(lock_file, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock_file, fcntl.LOCK_UN)


class ManualKill:
    def __init__(self, target_system=1, target_component=1, kill_file=None, reporter=None):
        self.reporter = reporter
        self.target_system = target_system
        self.target_component = target_component
        self.latched = threading.Event()
        self.lock = threading.Lock()
        self.sent = 0
        self.last_ack = None
        self.armed = None
        self.last_fc_hb = 0.0
        self.error = None
        self.kill_file = Path(kill_file) if kill_file is not None else None
        self.motor_test_ack = None
        if self.kill_file is not None and self.kill_file.exists():
            self.latched.set()

    def receive(self, packet):
        if not isinstance(packet, dict) or packet.get('version') != 1:
            return False
        if packet.get('type') == 'KILL':
            if not self.latched.is_set():
                print('KILL latched: cancelling motor tests and force-disarming', flush=True)
            self.latched.set()
            if self.kill_file is not None:
                try:
                    with bench_command_lock(self.kill_file):
                        self.kill_file.touch(exist_ok=True)
                except OSError as error:
                    # Never let a marker failure suppress the FC stop command.
                    with self.lock:
                        self.error = 'Cannot inhibit bench producer: %s' % error
            return True
        return packet.get('type') == 'KILL_HEARTBEAT'

    def send_stop(self, connection):
        # ArduCopter has one active motor-test state. Replace it with a
        # zero-duration, single-motor test: its next output cycle stops the
        # entire test, including any queued motor sequence. Same path as
        # motor_test.py:stop_motor. Do NOT pass the frame's motor count here.
        connection.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
            1, mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT, 0, 0, 1, 0, 0)
        connection.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196, 0, 0, 0, 0, 0)
        with self.lock:
            self.sent += 1

    def feedback(self, msg):
        if (msg.get_srcSystem(), msg.get_srcComponent()) != (
                self.target_system, self.target_component):
            return
        with self.lock:
            if msg.get_type() == 'HEARTBEAT':
                self.last_fc_hb = time.monotonic()
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif (msg.get_type() == 'COMMAND_ACK' and
                  msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                if self.last_ack != msg.result:
                    print('FC disarm ACK result:', msg.result, flush=True)
                self.last_ack = msg.result
            elif (msg.get_type() == 'COMMAND_ACK' and
                  msg.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST):
                if self.motor_test_ack != msg.result:
                    print('FC motor-test stop ACK result:', msg.result, flush=True)
                self.motor_test_ack = msg.result

    def status(self):
        with self.lock:
            fresh = bool(self.last_fc_hb and time.monotonic() - self.last_fc_hb < 3)
            return dict(version=1, type='KILL_STATUS', time_ms=int(time.time()*1000),
                        armed=self.armed if fresh else None, fc_linked=fresh,
                        killing='manual kill' if self.latched.is_set() else None,
                        kill_latched=self.latched.is_set(), mode='manual_only',
                        timeout_s=None, disarm_commands_sent=self.sent,
                        disarm_ack_result=self.last_ack, motor_rpm=None,
                        motor_test_ack_result=self.motor_test_ack,
                        error=self.error)


def fc_worker(state, args, stopping):
    """Keep UDP reception independent of connection startup/reconnect delays."""
    while not stopping.is_set():
        connection = None
        try:
            connection = mavutil.mavlink_connection(
                args.connection, baud=args.baud, source_system=args.source_system,
                source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
                retries=1)
            trace_connection(connection, state.reporter)
            with state.lock:
                state.error = None
            print('MAVLink connected; target %d/%d' % (
                state.target_system, state.target_component), flush=True)
            next_stop = next_heartbeat = 0.0
            while not stopping.is_set():
                now = time.monotonic()
                # Announce our endpoint for routing; never wait for a reply.
                if now >= next_heartbeat:
                    connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    next_heartbeat = now + 1
                if state.latched.is_set() and now >= next_stop:
                    state.send_stop(connection)
                    next_stop = now + 1 / args.rate
                # Bound telemetry work so incoming streams cannot starve KILL.
                for _ in range(32):
                    msg = connection.recv_match(blocking=False)
                    if msg is None:
                        break
                    state.feedback(msg)
                stopping.wait(0.005)
        except Exception as error:
            with state.lock:
                state.error = str(error)
            print('MAVLink error; retaining KILL latch:', error, flush=True)
        finally:
            if connection is not None:
                connection.close()
        stopping.wait(0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='tcp:127.0.0.1:5760')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--port', type=int, default=5002)
    parser.add_argument('--target-system', type=int, default=1)
    parser.add_argument('--target-component', type=int, default=1)
    parser.add_argument('--source-system', type=int, default=252)
    parser.add_argument('--rate', type=float, default=20, help='stop command pairs/second (1–100)')
    parser.add_argument('--signal-port', type=int, default=5001, help='laptop observer UDP port')
    parser.add_argument('--reset-latch', action='store_true',
                        help='remove persistent bench inhibit and exit; stop receiver and producer first')
    args = parser.parse_args()
    if args.reset_latch:
        DEFAULT_KILL_FILE.unlink(missing_ok=True)
        print('Bench inhibit cleared. Restart receiver and backend before another test.')
        return
    if not 1 <= args.rate <= 100:
        parser.error('--rate must be between 1 and 100')
    if not all(1 <= n <= 255 for n in (
            args.target_system, args.target_component, args.source_system)):
        parser.error('system/component IDs must be between 1 and 255')
    reporter = SignalReporter('kill_receiver', port=args.signal_port).start()
    state = ManualKill(args.target_system, args.target_component, DEFAULT_KILL_FILE, reporter)
    stopping = threading.Event()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('0.0.0.0', args.port))
        sock.settimeout(0.2)
        worker = threading.Thread(target=fc_worker, args=(state, args, stopping), daemon=True)
        worker.start()
        print('Manual KILL listening on UDP %d; no link-loss shutdown' % args.port, flush=True)
        try:
            while True:
                try:
                    data, address = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                try:
                    packet = json.loads(data)
                except (ValueError, UnicodeDecodeError):
                    continue
                if isinstance(packet, dict) and packet.get('version') == 1 and packet.get('type') in ('KILL', 'KILL_HEARTBEAT'):
                    reporter.set_host(address[0])
                    reporter.record('laptop_to_pi', packet['type'], 'received', packet)
                if state.receive(packet):
                    try:
                        sock.sendto(json.dumps(state.status()).encode(), address)
                    except OSError as error:
                        print('Status reply failed:', error, flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            stopping.set()
            worker.join(timeout=2)
            reporter.close()


if __name__ == '__main__':
    main()
