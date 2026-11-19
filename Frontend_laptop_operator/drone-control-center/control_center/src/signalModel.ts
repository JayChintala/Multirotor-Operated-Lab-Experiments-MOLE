export type SignalEvent = {
  id: string; capture_id?: number; source: string;
  direction: "laptop_to_pi" | "pi_to_fc" | "fc_to_pi";
  name: string; outcome: string; timestamp_ms: number;
  bridge_received_time_ms: number; payload: Record<string, any>;
  observer_dropped?: number;
};

export const ACK_RESULTS: Record<number, string> = {
  0: "ACCEPTED", 1: "TEMPORARILY REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
  4: "FAILED", 5: "IN PROGRESS", 6: "CANCELLED",
};
export const routeLabel = (direction: string) => ({
  laptop_to_pi: "Laptop → Pi", pi_to_fc: "Pi → FC", fc_to_pi: "FC → Pi",
}[direction] ?? direction);
export const isHeartbeat = (event: SignalEvent) => /HEARTBEAT/.test(event.name);
export function summary(event: SignalEvent): string {
  const p = event.payload;
  if (event.outcome === "failed") return p.error ?? "Send failed";
  if (event.name === "CONTROL" && p.flight) {
    return `R ${p.flight.roll} · P ${p.flight.pitch} · Y ${p.flight.yaw} · throttle ${p.flight.throttle}`;
  }
  if (event.name === "TRIGGER") return p.name ?? "Trigger";
  if (event.name === "MOTOR_TEST") {
    const unit = p.param2 === 0 ? "% throttle" : p.param2 === 1 ? "PWM" : "(type " + p.param2 + ")";
    return `Motor ${p.param1} · ${p.param3} ${unit} · ${p.param4}s · count ${p.param5}`;
  }
  if (event.name === "ARM_DISARM") return `${p.param1 === 0 ? "Disarm" : "Arm"} · param2 ${p.param2}`;
  if (event.name === "COMMAND_ACK") return `Command ${p.command} · ${ACK_RESULTS[p.result] ?? p.result}`;
  if (event.name === "SET_MESSAGE_INTERVAL") return `Message ${p.param1} · interval ${p.param2} μs`;
  if (event.name === "REQUEST_MESSAGE") return `Message ${p.param1}`;
  if (event.name === "STATUSTEXT") return p.text ?? "FC status";
  if (event.name === "KILL") return "Manual stop request";
  return p.destination ?? "Connection announcement / status poll";
}

// Illustrative records only. Never passed to a transport or command sender.
export function demoSignals(now = Date.now()): SignalEvent[] {
  const rows = [
    ["motor_test", "pi_to_fc", "MOTOR_TEST", "sent", {command: 209, param1: 1, param2: 0, param3: 7, param4: 5, param5: 0}],
    ["motor_test", "fc_to_pi", "COMMAND_ACK", "received", {command: 209, result: 0}],
    ["laptop_backend", "laptop_to_pi", "KILL", "sent", {version: 1, type: "KILL", destination: "10.42.0.1:5002"}],
    ["kill_receiver", "laptop_to_pi", "KILL", "received", {version: 1, type: "KILL"}],
    ["kill_receiver", "pi_to_fc", "MOTOR_TEST", "sent", {command: 209, param1: 1, param2: 0, param3: 0, param4: 0, param5: 1}],
    ["kill_receiver", "pi_to_fc", "ARM_DISARM", "sent", {command: 400, param1: 0, param2: 21196}],
    ["kill_receiver", "fc_to_pi", "COMMAND_ACK", "received", {command: 209, result: 0}],
    ["kill_receiver", "fc_to_pi", "COMMAND_ACK", "received", {command: 400, result: 0}],
  ];
  return rows.map(([source, direction, name, outcome, payload], i) => ({
    id: `demo-${i}`, source, direction, name, outcome, payload,
    timestamp_ms: now - 1500 + i * 100, bridge_received_time_ms: now - 1500 + i * 100,
  } as SignalEvent)).reverse();
}
