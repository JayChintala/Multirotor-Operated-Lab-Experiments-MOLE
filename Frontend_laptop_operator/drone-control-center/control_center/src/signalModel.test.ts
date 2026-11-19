import assert from 'node:assert/strict';
import test from 'node:test';
import {demoSignals, summary, isHeartbeat} from './signalModel.ts';

test('demo separates send, Pi receipt, cancellation and FC ACK', () => {
  const events = demoSignals(100000);
  assert.equal(events.length, 8);
  assert.equal(events.filter(e => e.name === 'KILL').length, 2);
  assert.equal(events.filter(e => e.name === 'COMMAND_ACK').length, 3);
  assert.ok(events.every(e => e.id.startsWith('demo-')));
  assert.match(summary(events.find(e => e.name === 'MOTOR_TEST' && e.payload.param3 === 0)!), /0 % throttle · 0s · count 1/);
});

test('ACK refusal is not displayed as a successful motor stop', () => {
  const event = demoSignals()[0];
  event.payload.result = 2;
  assert.match(summary(event), /DENIED/);
  event.outcome = 'failed'; event.payload.error = 'connection reset';
  assert.equal(summary(event), 'connection reset');
});

test('control values and heartbeat filtering preserve actual data', () => {
  const event = demoSignals()[0];
  event.name = 'CONTROL'; event.payload = {flight: {roll: -.2, pitch: .3, yaw: 0, throttle: .7}};
  assert.equal(summary(event), 'R -0.2 · P 0.3 · Y 0 · throttle 0.7');
  event.name = 'KILL_HEARTBEAT'; assert.ok(isHeartbeat(event));
  event.name = 'KILL'; assert.ok(!isHeartbeat(event));
});
