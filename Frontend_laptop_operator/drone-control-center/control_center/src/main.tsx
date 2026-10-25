import React, { useEffect, useMemo, useRef, useState } from "react";
import ReactDOM from "react-dom/client";
import "./style.css";

const WS_URL = "ws://127.0.0.1:5173";
const CONTROL_HZ = 20;
const STALE_AFTER_MS = 1500;
const GAMEPAD_DEADZONE = 0.08;
const MOTOR_COUNT = 4;

type IMUData = {
  roll?: number;
  pitch?: number;
  yaw?: number;
  ax?: number;
  ay?: number;
  az?: number;
  gx?: number;
  gy?: number;
  gz?: number;
};

type Telemetry = {
  type: string;
  fc?: {
    attitude?: { roll?: number; pitch?: number; yaw?: number };
    acceleration?: { x?: number; y?: number; z?: number };
    angular_velocity?: { x?: number; y?: number; z?: number };
    position?: {
      x?: number;
      y?: number;
      z?: number;
      lat?: number;
      lon?: number;
      alt?: number;
    };
    battery?: { voltage?: number; remaining?: number };
    flight_mode?: string;
    armed?: boolean;
    motors?: { output?: number; rpm?: number; temperature?: number }[];
  };
  water_contact?: boolean;
  sensors?: {
    water_detected?: boolean;
  };
  bridge_received_time_ms?: number;
};

type ControlCommand = {
  roll: number;
  pitch: number;
  yaw: number;
  throttle: number;
};

type LogEntry = {
  time: string;
  message: string;
};

type TrajectoryPoint = {
  x: number;
  y: number;
  z: number;
};

const neutralControl: ControlCommand = {
  roll: 0,
  pitch: 0,
  yaw: 0,
  throttle: 0,
};

function App() {
  const [socket, setSocket] = useState<WebSocket | null>(null);
  const [, setClock] = useState(Date.now());
  const [connected, setConnected] = useState(false);
  const [videoFrame, setVideoFrame] = useState<string | null>(null);
  const [videoWaterDetected, setVideoWaterDetected] = useState(false);
  const [lastVideoTime, setLastVideoTime] = useState<number | null>(null);
  const [telemetry, setTelemetry] = useState<Telemetry | null>(null);
  const [lastTelemetryTime, setLastTelemetryTime] = useState<number | null>(null);
  const [control, setControl] = useState(neutralControl);
  const controlRef = useRef(control);
  const [gamepadName, setGamepadName] = useState<string | null>(null);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [trajectory, setTrajectory] = useState<TrajectoryPoint[]>([]);
  const gpsOriginRef = useRef<{ lat: number; lon: number } | null>(null);

  useEffect(() => {
    const timer = window.setInterval(() => setClock(Date.now()), 250);
    return () => window.clearInterval(timer);
  }, []);

  const addLog = (message: string) => {
    setLogs((previous) => [
      { time: new Date().toLocaleTimeString(), message },
      ...previous,
    ].slice(0, 60));
  };

  useEffect(() => {
    controlRef.current = control;
  }, [control]);

  useEffect(() => {
    let active = true;
    let reconnectTimer: number | undefined;
    let currentSocket: WebSocket | null = null;

    const connect = () => {
      if (!active) return;
      currentSocket = new WebSocket(WS_URL);
      setSocket(currentSocket);
      currentSocket.onopen = () => {
        setConnected(true);
        addLog("Connected to laptop backend");
        currentSocket?.send(JSON.stringify({
          version: 1,
          type: "CONFIG",
          desired_control_hz: CONTROL_HZ,
          video_fps: 12,
        }));
      };
      currentSocket.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          if (message.type === "VIDEO_FRAME") {
            setVideoFrame(`data:${message.content_type};base64,${message.data}`);
            setVideoWaterDetected(
              message.water_contact === true || message.sensors?.water_detected === true,
            );
            setLastVideoTime(Date.now());
          } else if (message.type === "TELEMETRY") {
            setTelemetry(message);
            setLastTelemetryTime(Date.now());
          } else if (message.type === "ERROR") {
            addLog(`Backend error: ${message.message}`);
          }
        } catch {
          addLog("Received an invalid backend message");
        }
      };
      currentSocket.onerror = () => setConnected(false);
      currentSocket.onclose = () => {
        setConnected(false);
        setSocket(null);
        if (active) reconnectTimer = window.setTimeout(connect, 1000);
      };
    };

    connect();
    return () => {
      active = false;
      window.clearTimeout(reconnectTimer);
      currentSocket?.close();
    };
  }, []);

  useEffect(() => {
    const sendControl = () => {
      if (!socket || socket.readyState !== WebSocket.OPEN) return;
      socket.send(JSON.stringify({
        version: 1,
        type: "CONTROL",
        time_ms: Date.now(),
        flight: controlRef.current,
        source: "new_frontend",
      }));
    };
    const timer = window.setInterval(sendControl, 1000 / CONTROL_HZ);
    return () => window.clearInterval(timer);
  }, [socket]);

  useEffect(() => {
    const keyMap: Record<string, keyof ControlCommand> = {
      w: "pitch",
      s: "pitch",
      a: "roll",
      d: "roll",
      ArrowUp: "throttle",
      ArrowDown: "throttle",
    };
    const onKeyDown = (event: KeyboardEvent) => {
      const axis = keyMap[event.key];
      if (!axis || event.repeat) return;
      event.preventDefault();
      const value = event.key === "w" || event.key === "d" || event.key === "ArrowUp" ? 1 : -1;
      setControl((previous) => ({ ...previous, [axis]: value }));
      addLog(`Keyboard ${event.key} -> ${axis} ${value > 0 ? "+" : "-"}`);
    };
    const onKeyUp = (event: KeyboardEvent) => {
      const axis = keyMap[event.key];
      if (!axis) return;
      event.preventDefault();
      setControl((previous) => ({ ...previous, [axis]: 0 }));
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
  }, []);

  // Mode 2 sticks on the standard gamepad mapping: left = throttle/yaw,
  // right = pitch/roll. While a gamepad is connected it overrides the keyboard.
  useEffect(() => {
    const onConnect = (event: GamepadEvent) => addLog(`Gamepad connected: ${event.gamepad.id}`);
    const onDisconnect = (event: GamepadEvent) => {
      addLog(`Gamepad disconnected: ${event.gamepad.id}`);
      controlRef.current = neutralControl;
      setControl(neutralControl);
    };
    const poll = () => {
      const pad = Array.from(navigator.getGamepads?.() ?? []).find((p): p is Gamepad => !!p?.connected);
      setGamepadName(pad ? pad.id : null);
      if (!pad || pad.axes.length < 4) return;
      const axis = (index: number) => {
        const value = pad.axes[index] ?? 0;
        return Math.abs(value) < GAMEPAD_DEADZONE ? 0 : Math.round(value * 100) / 100;
      };
      const next: ControlCommand = {
        yaw: axis(0),
        throttle: Math.max(0, -axis(1)),
        roll: axis(2),
        pitch: -axis(3) || 0,
      };
      controlRef.current = next;
      setControl(next);
    };
    window.addEventListener("gamepadconnected", onConnect);
    window.addEventListener("gamepaddisconnected", onDisconnect);
    const timer = window.setInterval(poll, 1000 / CONTROL_HZ);
    return () => {
      window.removeEventListener("gamepadconnected", onConnect);
      window.removeEventListener("gamepaddisconnected", onDisconnect);
      window.clearInterval(timer);
    };
  }, []);

  const cameraLive = lastVideoTime !== null && Date.now() - lastVideoTime < STALE_AFTER_MS;
  const imuLive = lastTelemetryTime !== null && Date.now() - lastTelemetryTime < STALE_AFTER_MS;
  const waterDetected = videoWaterDetected
    || telemetry?.water_contact === true
    || telemetry?.sensors?.water_detected === true;
  const imu: IMUData = useMemo(() => ({
    roll: telemetry?.fc?.attitude?.roll,
    pitch: telemetry?.fc?.attitude?.pitch,
    yaw: telemetry?.fc?.attitude?.yaw,
    ax: telemetry?.fc?.acceleration?.x,
    ay: telemetry?.fc?.acceleration?.y,
    az: telemetry?.fc?.acceleration?.z,
    gx: telemetry?.fc?.angular_velocity?.x,
    gy: telemetry?.fc?.angular_velocity?.y,
    gz: telemetry?.fc?.angular_velocity?.z,
  }), [telemetry]);

  const currentPosition = useMemo(() => {
    const position = telemetry?.fc?.position;
    if (!position) return null;

    if ([position.x, position.y, position.z].every((value) => typeof value === "number" && Number.isFinite(value))) {
      return { x: position.x!, y: position.y!, z: position.z! };
    }

    if (
      typeof position.lat !== "number" ||
      typeof position.lon !== "number" ||
      typeof position.alt !== "number" ||
      !Number.isFinite(position.lat) ||
      !Number.isFinite(position.lon) ||
      !Number.isFinite(position.alt)
    ) {
      return null;
    }

    if (!gpsOriginRef.current) {
      gpsOriginRef.current = { lat: position.lat, lon: position.lon };
    }
    const earthRadius = 6371000;
    const origin = gpsOriginRef.current;
    return {
      x: (position.lon - origin.lon) * Math.PI / 180 * earthRadius * Math.cos(origin.lat * Math.PI / 180),
      y: (position.lat - origin.lat) * Math.PI / 180 * earthRadius,
      z: position.alt,
    };
  }, [telemetry]);

  useEffect(() => {
    if (!currentPosition) return;
    setTrajectory((previous) => {
      const last = previous[previous.length - 1];
      if (last && last.x === currentPosition.x && last.y === currentPosition.y && last.z === currentPosition.z) {
        return previous;
      }
      return [...previous, currentPosition].slice(-500);
    });
  }, [currentPosition]);

  const resetTrajectory = () => {
    setTrajectory([]);
    gpsOriginRef.current = null;
  };

  const sendTrigger = (name: string) => {
    if (socket?.readyState !== WebSocket.OPEN) {
      addLog(`Trigger ${name} not sent: backend disconnected`);
      return;
    }
    socket.send(JSON.stringify({
      version: 1,
      type: "TRIGGER",
      name,
      time_ms: Date.now(),
    }));
    addLog(`Trigger sent: ${name}`);
  };

  return (
    <main className="screen">
      <section className="panel camera-panel">
        <PanelTitle title="CAMERA" status={cameraLive ? "LIVE" : "NO SIGNAL"} live={cameraLive} />
        <div className="camera-content">
          {cameraLive && videoFrame ? <img src={videoFrame} alt="Live camera feed" /> : <span>No camera detected</span>}
        </div>
      </section>

      <section className="panel imu-panel">
        <PanelTitle title="IMU READINGS" status={imuLive ? "LIVE" : "OFFLINE"} live={imuLive} />
        {imuLive ? (
          <>
            <ReadingGroup title="Orientation" values={[
              ["Roll", format(imu.roll, "°")], ["Pitch", format(imu.pitch, "°")], ["Yaw", format(imu.yaw, "°")],
            ]} />
            <ReadingGroup title="Acceleration" values={[
              ["X", format(imu.ax, " m/s²")], ["Y", format(imu.ay, " m/s²")], ["Z", format(imu.az, " m/s²")],
            ]} />
            <ReadingGroup title="Angular velocity" values={[
              ["X", format(imu.gx, " rad/s")], ["Y", format(imu.gy, " rad/s")], ["Z", format(imu.gz, " rad/s")],
            ]} />
            <MotorGroup motors={telemetry?.fc?.motors} />
          </>
        ) : <div className="offline-message">Drone not connected</div>}
      </section>

      <section className="panel trajectory-panel">
        <PanelTitle title="3D TRAJECTORY / TELEMETRY" status={trajectory.length ? `${trajectory.length} POINTS` : "WAITING"} live={trajectory.length > 0} />
        <div className="trajectory-content">
          <div className="telemetry-grid">
            <Metric label="Mode" value={telemetry?.fc?.flight_mode ?? "—"} />
            <Metric label="Armed" value={telemetry?.fc?.armed ? "YES" : "NO"} />
            <Metric label="Latitude" value={format(telemetry?.fc?.position?.lat)} />
            <Metric label="Longitude" value={format(telemetry?.fc?.position?.lon)} />
            <Metric label="Altitude" value={format(telemetry?.fc?.position?.alt, " m")} />
            <Metric label="Battery" value={format(telemetry?.fc?.battery?.remaining, "%")} />
          </div>
          <TrajectoryPlot points={trajectory} />
          <div className="trajectory-footer">
            <span className="muted">X east · Y north · Z altitude (metres)</span>
            <button className="reset-button" onClick={resetTrajectory}>RESET TRAIL</button>
          </div>
        </div>
      </section>

      <section className="panel controls-panel">
        <PanelTitle title="CONTROLLER INPUTS" status={connected ? "TX 20 HZ" : "DISCONNECTED"} live={connected} />
        <div className="controls-content">
          <div className="stick-row">
            <Stick label="Left stick" x={control.yaw} y={control.throttle} />
            <div className="control-readout">
              <Metric label="Throttle" value={control.throttle.toFixed(2)} />
              <Metric label="Yaw" value={control.yaw.toFixed(2)} />
              <Metric label="Pitch" value={control.pitch.toFixed(2)} />
              <Metric label="Roll" value={control.roll.toFixed(2)} />
            </div>
            <Stick label="Right stick" x={control.roll} y={control.pitch} />
          </div>
          <div className="key-hint">
            {gamepadName
              ? `Gamepad: ${gamepadName}`
              : "No gamepad: press a button on it to connect · keyboard W A S D, ↑ ↓"}
          </div>
          <div className="log-list">
            {logs.length === 0 ? <span className="muted">Waiting for control activity…</span> :
              logs.map((entry, index) => <div className="log-line" key={`${entry.time}-${index}`}><span>{entry.time}</span>{entry.message}</div>)}
          </div>
        </div>
      </section>

      <section className="panel water-panel">
        <PanelTitle title="WATER CONTACT" />
        <div className={`water-status ${waterDetected ? "detected" : ""}`}>
          {waterDetected ? "CONTACT DETECTED" : imuLive ? "NO CONTACT" : "NO DATA"}
        </div>
      </section>

      <section className="panel trigger-panel">
        <PanelTitle title="TRIGGERS" />
        <div className="trigger-buttons">
          <button onClick={() => sendTrigger("valve_1_on")}>VALVE 1 ON</button>
          <button onClick={() => sendTrigger("valve_2_on")}>VALVE 2 ON</button>
          <button onClick={() => sendTrigger("valve_1_off")}>VALVE 1 OFF</button>
          <button onClick={() => sendTrigger("valve_2_off")}>VALVE 2 OFF</button>
        </div>
      </section>
    </main>
  );
}

function PanelTitle({ title, status, live }: { title: string; status?: string; live?: boolean }) {
  return <div className="panel-title">{title}{status && <span className={`panel-status ${live ? "live" : ""}`}>{status}</span>}</div>;
}

function ReadingGroup({ title, values }: { title: string; values: [string, string][] }) {
  return <div className="imu-section"><h2>{title}</h2><div className="readings">
    {values.map(([label, value]) => <Reading key={label} label={label} value={value} />)}
  </div></div>;
}

function Reading({ label, value }: { label: string; value: string }) {
  return <div className="reading"><span>{label}</span><strong>{value}</strong></div>;
}

function Metric({ label, value }: { label: string; value: string }) {
  return <div className="metric"><span>{label}</span><strong>{value}</strong></div>;
}

function MotorGroup({ motors }: { motors?: { output?: number; rpm?: number; temperature?: number }[] }) {
  return <div className="imu-section"><h2>Motors</h2><div className="readings motor-readings">
    {Array.from({ length: MOTOR_COUNT }, (_, index) => {
      const motor = motors?.[index];
      return <div className="reading" key={index}>
        <span>
          M{index + 1} · {motor?.output !== undefined ? `${motor.output.toFixed(0)}%` : "—"}
          {motor?.temperature !== undefined ? ` · ${motor.temperature}°C` : ""}
        </span>
        <strong>{motor?.rpm !== undefined ? `${motor.rpm} rpm` : "—"}</strong>
        <div className="motor-bar"><div style={{ width: `${motor?.output ?? 0}%` }} /></div>
      </div>;
    })}
  </div></div>;
}

function Stick({ label, x, y }: { label: string; x: number; y: number }) {
  const clamp = (value: number) => Math.min(Math.max(value, -1), 1);
  return <div className="stick" aria-label={label}>
    <div className="stick-dot" style={{ left: `${50 + clamp(x) * 50}%`, top: `${50 - clamp(y) * 50}%` }} />
  </div>;
}

function TrajectoryPlot({ points }: { points: TrajectoryPoint[] }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const container = canvas.parentElement;
    if (!container) return;
    const context = canvas.getContext("2d");
    if (!context) return;

    const draw = () => {
      const width = container.clientWidth;
      const height = container.clientHeight;
      const dpr = window.devicePixelRatio || 1;
      canvas.width = width * dpr;
      canvas.height = height * dpr;
      canvas.style.width = `${width}px`;
      canvas.style.height = `${height}px`;
      context.setTransform(dpr, 0, 0, dpr, 0, 0);
      context.clearRect(0, 0, width, height);
      context.fillStyle = "#0b1116";
      context.fillRect(0, 0, width, height);

      const allPoints = points.length ? points : [{ x: 0, y: 0, z: 0 }];
      const maxRange = Math.max(
        ...allPoints.map((point) => Math.max(Math.abs(point.x), Math.abs(point.y), Math.abs(point.z))),
        1,
      );
      const scale = Math.min(width, height) * 0.34 / maxRange;
      const centerX = width / 2;
      const centerY = height / 2 + 12;
      const project = (point: TrajectoryPoint) => ({
        x: centerX + (point.x - point.y) * scale * 0.72,
        y: centerY + (point.x + point.y) * scale * 0.36 - point.z * scale * 0.82,
      });

      const origin = project({ x: 0, y: 0, z: 0 });
      const axes = [
        { point: { x: maxRange, y: 0, z: 0 }, label: "X", color: "#e58c7a" },
        { point: { x: 0, y: maxRange, z: 0 }, label: "Y", color: "#8fd694" },
        { point: { x: 0, y: 0, z: maxRange }, label: "Z", color: "#78a9d1" },
      ];
      context.lineWidth = 1;
      axes.forEach(({ point, label, color }) => {
        const end = project(point);
        context.strokeStyle = color;
        context.beginPath();
        context.moveTo(origin.x, origin.y);
        context.lineTo(end.x, end.y);
        context.stroke();
        context.fillStyle = color;
        context.font = "11px Arial";
        context.fillText(label, end.x + 5, end.y - 4);
      });

      if (points.length > 1) {
        context.strokeStyle = "#f2bb67";
        context.lineWidth = 2;
        context.beginPath();
        points.forEach((point, index) => {
          const projected = project(point);
          if (index === 0) context.moveTo(projected.x, projected.y);
          else context.lineTo(projected.x, projected.y);
        });
        context.stroke();
      }

      const latest = project(allPoints[allPoints.length - 1]);
      context.fillStyle = "#f2bb67";
      context.beginPath();
      context.arc(latest.x, latest.y, 5, 0, Math.PI * 2);
      context.fill();
      context.fillStyle = "#777";
      context.font = "11px Arial";
      context.fillText(points.length ? "LIVE POSITION" : "WAITING FOR POSITION TELEMETRY", 14, 20);
    };

    draw();
    const observer = new ResizeObserver(draw);
    observer.observe(container);
    return () => observer.disconnect();
  }, [points]);

  return <canvas ref={canvasRef} className="trajectory-canvas" aria-label="Live 3D drone trajectory plot" />;
}

function format(value: number | undefined, suffix = "") {
  return value === undefined || Number.isNaN(value) ? "—" : `${value.toFixed(2)}${suffix}`;
}

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode><App /></React.StrictMode>,
);
