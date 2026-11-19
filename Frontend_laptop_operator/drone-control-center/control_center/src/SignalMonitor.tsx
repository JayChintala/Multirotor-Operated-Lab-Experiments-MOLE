import React, { useEffect, useMemo, useRef, useState } from "react";
import { SignalEvent, demoSignals, isHeartbeat, routeLabel, summary } from "./signalModel";
import "./signalMonitor.css";

const timeLabel = (time: number) => new Date(time).toLocaleTimeString([], {hour12: false}) + "." + String(time % 1000).padStart(3, "0");

export default function SignalMonitor({events, connected, dropped, onClose, onKill}: {
  events: SignalEvent[]; connected: boolean; dropped: number;
  onClose: () => void; onKill: () => void;
}) {
  const [paused, setPaused] = useState<SignalEvent[] | null>(null);
  const [demo, setDemo] = useState<SignalEvent[] | null>(null);
  const [route, setRoute] = useState("all");
  const [query, setQuery] = useState("");
  const [heartbeats, setHeartbeats] = useState(false);
  const [selected, setSelected] = useState<SignalEvent | null>(null);
  const closeRef = useRef<HTMLButtonElement>(null);
  const dialogRef = useRef<HTMLElement>(null);
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    const previousFocus = document.activeElement as HTMLElement | null;
    closeRef.current?.focus();
    const timer = window.setInterval(() => setNow(Date.now()), 250);
    return () => { window.clearInterval(timer); previousFocus?.focus(); };
  }, []);
  const displayed = demo ?? paused ?? events;
  const filtered = useMemo(() => displayed.filter(event =>
    (route === "all" || event.direction === route) && (heartbeats || !isHeartbeat(event)) &&
    `${event.name} ${event.source} ${event.outcome} ${summary(event)}`.toLowerCase().includes(query.toLowerCase())
  ), [displayed, route, heartbeats, query]);
  const detail = selected ?? filtered[0];
  const exportCapture = () => {
    const blob = new Blob([JSON.stringify({mode: demo ? "illustration" : "live_capture", dropped, events: displayed}, null, 2)], {type: "application/json"});
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url; link.download = `signals-${demo ? "example" : "capture"}-${Date.now()}.json`; link.click();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  const observerDrops = Object.values(displayed.reduce<Record<string, number>>((acc, event) => {
    acc[event.source] = Math.max(acc[event.source] ?? 0, event.observer_dropped ?? 0); return acc;
  }, {})).reduce((a, b) => a + b, 0);
  return <section ref={dialogRef} className="signal-monitor" role="dialog" aria-modal="true" aria-label="Signal monitor"
    onKeyDown={event => {
      event.stopPropagation();
      if (event.key === "Escape") onClose();
      if (event.key === "Tab") {
        const elements = dialogRef.current?.querySelectorAll<HTMLElement>('button:not(:disabled), input');
        if (!elements?.length) return;
        const first = elements[0], last = elements[elements.length - 1];
        if (event.shiftKey && document.activeElement === first) { event.preventDefault(); last.focus(); }
        if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first.focus(); }
      }
    }}>
    <header className="signal-header">
      <div><div className="signal-eyebrow">EMSD / BENCH DIAGNOSTICS</div><h1>Signal monitor</h1></div>
      <div className="signal-actions">
        <span className={`signal-connection ${connected ? "online" : ""}`}>{connected ? "Backend connected" : "Backend offline"}</span>
        <button className="signal-kill" onClick={onKill} disabled={!connected || !!demo}>KILL MOTORS</button>
        <button ref={closeRef} onClick={onClose}>Close monitor</button>
      </div>
    </header>
    <p className="signal-intro">Watch commands cross each link. “Sent” means the sending application completed its write, not that the motors responded.</p>
    {demo && <div className="signal-banner">ILLUSTRATION ONLY — example mode sends no commands. Existing live control continues. <button onClick={() => {setDemo(null); setSelected(null);}}>Return to live capture</button></div>}
    {paused && !demo && <div className="signal-banner">Display frozen. Commands and live capture continue normally.</div>}
    <div className="signal-routes">
      {(["laptop_to_pi", "pi_to_fc", "fc_to_pi"] as const).map(direction => {
        const rows = displayed.filter(e => e.direction === direction);
        const latest = rows[0];
        const age = latest ? Math.max(0, now - latest.bridge_received_time_ms) : null;
        return <button key={direction} className={`signal-route ${route === direction ? "selected" : ""}`} onClick={() => setRoute(route === direction ? "all" : direction)}>
          <span>{routeLabel(direction)}</span><strong>{rows.length}<small> records retained</small></strong>
          <div>{direction === "laptop_to_pi" ? "UDP commands + Pi receipt" : direction === "pi_to_fc" ? "MAVLink application writes" : "ACK + FC status messages"}</div>
          <em>{age === null ? "No records yet" : `Last received ${age < 1000 ? `${age} ms` : `${(age / 1000).toFixed(1)} s`} ago`}</em>
        </button>;
      })}
    </div>
    <div className="signal-tools">
      <input aria-label="Filter signal records" placeholder="Filter command, source, result…" value={query} onChange={e => setQuery(e.target.value)} />
      <button onClick={() => setRoute("all")} aria-pressed={route === "all"}>All links</button>
      <label><input type="checkbox" checked={heartbeats} onChange={e => setHeartbeats(e.target.checked)} /> Heartbeats</label>
      <button disabled={!!demo} onClick={() => { setPaused(paused ? null : [...events]); setSelected(null); }}>{paused ? "Resume display" : "Freeze display"}</button>
      <button onClick={exportCapture}>Export JSON</button>
      <button onClick={() => {setDemo(demo ? null : demoSignals()); setSelected(null);}}> {demo ? "Live capture" : "Show example"}</button>
    </div>
    <div className="signal-body">
      <div className="signal-table-wrap">
        <table><thead><tr><th>Received (laptop)</th><th>Link / source</th><th>Signal</th><th>Result</th><th>Values</th></tr></thead>
          <tbody>{filtered.map(event => <tr key={`${event.capture_id ?? event.id}:${event.id}`} className={detail?.id === event.id ? "active" : ""}>
            <td><button className="signal-row-select" onClick={() => setSelected(event)} aria-label={`Inspect ${event.name} at ${timeLabel(event.bridge_received_time_ms)}`}>{timeLabel(event.bridge_received_time_ms)}</button></td>
            <td>{routeLabel(event.direction)}<small>{event.source}</small></td><td>{event.name}</td>
            <td><span className={`signal-result ${event.outcome}`}>{event.outcome}</span></td><td>{summary(event)}</td>
          </tr>)}</tbody></table>
        {!filtered.length && <div className="signal-empty"><h2>{events.length ? "No matching signals" : "Waiting for signal records"}</h2><p>Start the updated backend and companion scripts. For bench start commands, run motor_test.py with --signal-ip set to this laptop.</p><p>Use “Show example” to inspect the display without hardware.</p></div>}
      </div>
      <aside className="signal-detail"><h2>Inspect signal</h2>{detail ? <>
        <h3>{detail.name}</h3><p>{summary(detail)}</p>
        <dl><dt>Recorded by</dt><dd>{detail.source}</dd><dt>Sender timestamp</dt><dd>{timeLabel(detail.timestamp_ms)}</dd><dt>Laptop receipt</dt><dd>{timeLabel(detail.bridge_received_time_ms)}</dd><dt>Event ID</dt><dd>{detail.id}</dd></dl>
        <pre>{JSON.stringify(detail.payload, null, 2)}</pre>
        {detail.name === "MOTOR_TEST" && <p>param1 motor · param2 throttle type · param3 output · param4 duration (s) · param5 count. Percentage is not measured RPM.</p>}
        {detail.name === "ARM_DISARM" && <p>param1: 0 disarms. param2: 21196 forces disarm.</p>}
        {detail.name === "COMMAND_ACK" && <p>An ACK identifies the command, not a unique retry. Copies can be observed by multiple companion scripts.</p>}
      </> : <p>Select a timestamp to inspect the original values.</p>}</aside>
    </div>
    <footer className="signal-footer"><span>{filtered.length} visible / {displayed.length} retained · latest 1,000 records · local observer drops {dropped + observerDrops}</span><span>UDP monitor records can be lost. Pi/laptop clocks may differ. This is not an electrical signal capture.</span></footer>
  </section>;
}
