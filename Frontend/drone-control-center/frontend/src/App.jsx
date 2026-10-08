import { useCallback, useEffect, useMemo, useRef, useState } from 'react'

const DEFAULT_WS_URL = 'ws://127.0.0.1:8765'
const CONTROL_HZ = 20

const clamp = (value, min, max) => Math.min(max, Math.max(min, value))

const neutralFlight = {
  roll: 0,
  pitch: 0,
  yaw: 0,
  throttle: 0,
}

const neutralPico = {
  output0: 0,
  output1: 0,
  output2: 0,
  output3: 0,
}

function App() {
  const [wsUrl, setWsUrl] = useState(() => localStorage.getItem('wsUrl') || DEFAULT_WS_URL)
  const [connectionState, setConnectionState] = useState('DISCONNECTED')
  const [controlEnabled, setControlEnabled] = useState(false)
  const [flight, setFlight] = useState(neutralFlight)
  const [pico, setPico] = useState(neutralPico)
  const [telemetry, setTelemetry] = useState(null)
  const [bridgeStatus, setBridgeStatus] = useState(null)
  const [logLines, setLogLines] = useState([])
  const [lastTx, setLastTx] = useState(null)
  const [sequence, setSequence] = useState(0)
  const [stats, setStats] = useState({ tx: 0, rx: 0, dropped: 0 })

  const socketRef = useRef(null)
  const reconnectTimerRef = useRef(null)
  const sequenceRef = useRef(0)
  const controlRef = useRef(false)
  const flightRef = useRef(flight)
  const picoRef = useRef(pico)
  const stateRef = useRef(connectionState)

  useEffect(() => {
    controlRef.current = controlEnabled
  }, [controlEnabled])

  useEffect(() => {
    flightRef.current = flight
  }, [flight])

  useEffect(() => {
    picoRef.current = pico
  }, [pico])

  useEffect(() => {
    stateRef.current = connectionState
  }, [connectionState])

  const addLog = useCallback((message) => {
    setLogLines((previous) => [
      `${new Date().toLocaleTimeString()}  ${message}`,
      ...previous,
    ].slice(0, 80))
  }, [])

  const sendJson = useCallback((message) => {
    const socket = socketRef.current
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      return false
    }
    socket.send(JSON.stringify(message))
    return true
  }, [])

  const connect = useCallback(() => {
    if (socketRef.current?.readyState === WebSocket.OPEN || socketRef.current?.readyState === WebSocket.CONNECTING) {
      return
    }

    setConnectionState('CONNECTING')
    addLog(`Connecting to ${wsUrl}`)

    const socket = new WebSocket(wsUrl)
    socketRef.current = socket

    socket.onopen = () => {
      setConnectionState('CONNECTED')
      addLog('WebSocket connected to laptop backend')
      sendJson({
        version: 1,
        type: 'CONFIG',
        desired_control_hz: CONTROL_HZ,
      })
    }

    socket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data)

        if (message.type === 'TELEMETRY') {
          setTelemetry(message)
          setStats((previous) => ({ ...previous, rx: previous.rx + 1 }))
        } else if (message.type === 'BRIDGE_STATUS') {
          setBridgeStatus(message)
        } else if (message.type === 'ERROR') {
          addLog(`Backend error: ${message.message}`)
        }
      } catch (error) {
        addLog(`Invalid message from backend: ${error.message}`)
      }
    }

    socket.onerror = () => {
      addLog('WebSocket error')
    }

    socket.onclose = () => {
      if (socketRef.current === socket) {
        socketRef.current = null
      }
      setConnectionState('DISCONNECTED')
      addLog('WebSocket disconnected')

      clearTimeout(reconnectTimerRef.current)
      reconnectTimerRef.current = setTimeout(() => {
        if (stateRef.current === 'DISCONNECTED') {
          connect()
        }
      }, 1500)
    }
  }, [addLog, sendJson, wsUrl])

  const disconnect = useCallback(() => {
    clearTimeout(reconnectTimerRef.current)
    const socket = socketRef.current
    socketRef.current = null
    if (socket) {
      socket.close()
    }
    setConnectionState('DISCONNECTED')
  }, [])

  useEffect(() => {
    localStorage.setItem('wsUrl', wsUrl)
  }, [wsUrl])

  useEffect(() => {
    connect()
    return () => {
      clearTimeout(reconnectTimerRef.current)
      const socket = socketRef.current
      socketRef.current = null
      socket?.close()
    }
  }, [connect])

  useEffect(() => {
    // React state is deliberately kept separate from the 20 Hz control loop.
    // This prevents a slider render from being the thing that determines the
    // packet timing. The browser still isn't a hard-real-time environment.
    const timer = setInterval(() => {
      if (socketRef.current?.readyState !== WebSocket.OPEN) {
        return
      }

      const nextSequence = sequenceRef.current
      sequenceRef.current = (sequenceRef.current + 1) % 256
      setSequence(nextSequence)

      const flightCommand = controlRef.current ? flightRef.current : neutralFlight
      const picoCommand = controlRef.current ? picoRef.current : neutralPico

      const packet = {
        version: 1,
        type: 'CONTROL',
        seq: nextSequence,
        time_ms: Date.now(),
        flight: flightCommand,
        pico: picoCommand,
      }

      const sent = sendJson(packet)
      if (sent) {
        setLastTx(packet)
        setStats((previous) => ({ ...previous, tx: previous.tx + 1 }))
      }
    }, 1000 / CONTROL_HZ)

    return () => clearInterval(timer)
  }, [sendJson])

  const setFlightValue = (name, value) => {
    setFlight((previous) => ({
      ...previous,
      [name]: Number(value),
    }))
  }

  const setPicoValue = (name, value) => {
    setPico((previous) => ({
      ...previous,
      [name]: Number(value),
    }))
  }

  const telemetryAge = useMemo(() => {
    if (!telemetry?.bridge_received_time_ms) return null
    return Math.max(0, Date.now() - telemetry.bridge_received_time_ms)
  }, [telemetry])

  const piAlive = bridgeStatus?.pi_telemetry_age_ms != null && bridgeStatus.pi_telemetry_age_ms < 1500

  const flightMode = telemetry?.fc?.flight_mode ?? '—'
  const armed = telemetry?.fc?.armed ?? false
  const attitude = telemetry?.fc?.attitude ?? {}
  const battery = telemetry?.fc?.battery ?? {}
  const position = telemetry?.fc?.position ?? {}

  return (
    <div className="app-shell">
      <header className="topbar">
        <div>
          <div className="eyebrow">ROBOTICS / CONTROL CENTER</div>
          <h1>Drone Ground Station</h1>
          <p className="subhead">Laptop → WebSocket → Python bridge → UDP → Raspberry Pi → MAVLink / SPI</p>
        </div>
        <div className="connection-cluster">
          <StatusPill label="Browser ↔ Backend" state={connectionState} />
          <StatusPill label="Pi telemetry" state={piAlive ? 'LIVE' : 'NO DATA'} />
          <StatusPill label="FC" state={armed ? 'ARMED' : 'DISARMED'} />
        </div>
      </header>

      <main className="dashboard">
        <section className="card span-2">
          <div className="section-heading">
            <div>
              <span className="section-kicker">01</span>
              <h2>Connection</h2>
            </div>
            <span className="muted">Local browser ↔ laptop backend</span>
          </div>

          <div className="connection-form">
            <label>
              WebSocket URL
              <input
                value={wsUrl}
                onChange={(event) => setWsUrl(event.target.value)}
                placeholder="ws://127.0.0.1:8765"
              />
            </label>
            <div className="button-row">
              <button className="secondary" onClick={connect}>Connect</button>
              <button className="ghost" onClick={disconnect}>Disconnect</button>
            </div>
          </div>
          <p className="hint">Run the laptop Python backend first. The browser never talks directly to the Raspberry Pi.</p>
        </section>

        <section className={`card control-card ${controlEnabled ? 'active' : ''}`}>
          <div className="section-heading">
            <div>
              <span className="section-kicker">02</span>
              <h2>Flight controls</h2>
            </div>
            <button
              className={controlEnabled ? 'danger' : 'primary'}
              onClick={() => setControlEnabled((previous) => !previous)}
            >
              {controlEnabled ? 'Disable controls' : 'Enable controls'}
            </button>
          </div>

          <div className="control-warning">
            {controlEnabled
              ? 'Live control packet stream enabled. Keep propulsion disabled during bench testing.'
              : 'Controls disabled: the UI is sending neutral flight/Pico values.'}
          </div>

          <div className="axis-list">
            <AxisControl label="Roll" value={flight.roll} min={-1} max={1} step={0.01} onChange={(value) => setFlightValue('roll', value)} />
            <AxisControl label="Pitch" value={flight.pitch} min={-1} max={1} step={0.01} onChange={(value) => setFlightValue('pitch', value)} />
            <AxisControl label="Yaw" value={flight.yaw} min={-1} max={1} step={0.01} onChange={(value) => setFlightValue('yaw', value)} />
            <AxisControl label="Throttle" value={flight.throttle} min={0} max={1} step={0.01} onChange={(value) => setFlightValue('throttle', value)} />
          </div>

          <div className="control-readout">
            {Object.entries(flight).map(([name, value]) => (
              <div key={name}>
                <span>{name}</span>
                <strong>{Number(value).toFixed(2)}</strong>
              </div>
            ))}
          </div>
        </section>

        <section className="card">
          <div className="section-heading">
            <div>
              <span className="section-kicker">03</span>
              <h2>Pico outputs</h2>
            </div>
            <span className="muted">Signed int16</span>
          </div>

          <div className="pico-grid">
            {Object.entries(pico).map(([name, value]) => (
              <label key={name} className="pico-input">
                <span>{name}</span>
                <input
                  type="number"
                  min={-1000}
                  max={1000}
                  step={1}
                  value={value}
                  onChange={(event) => setPicoValue(name, clamp(Number(event.target.value), -1000, 1000))}
                />
              </label>
            ))}
          </div>
          <p className="hint">These become the four int16 values in the Pi → Pico SPI packet. Change the mapping later in the Python gateway and Pico firmware together.</p>
        </section>

        <section className="card telemetry-card">
          <div className="section-heading">
            <div>
              <span className="section-kicker">04</span>
              <h2>Flight telemetry</h2>
            </div>
            <span className={telemetryAge != null && telemetryAge < 1500 ? 'live-text' : 'muted'}>
              {telemetryAge == null ? 'No telemetry' : `${telemetryAge} ms old`}
            </span>
          </div>

          <div className="telemetry-grid">
            <Metric label="Mode" value={flightMode} />
            <Metric label="Armed" value={armed ? 'YES' : 'NO'} />
            <Metric label="Battery" value={battery.voltage_v != null ? `${battery.voltage_v.toFixed(2)} V` : '—'} />
            <Metric label="Remaining" value={battery.remaining_percent != null ? `${battery.remaining_percent}%` : '—'} />
            <Metric label="Roll" value={formatNumber(attitude.roll)} />
            <Metric label="Pitch" value={formatNumber(attitude.pitch)} />
            <Metric label="Yaw" value={formatNumber(attitude.yaw)} />
            <Metric label="Altitude" value={position.relative_alt_m != null ? `${position.relative_alt_m.toFixed(2)} m` : '—'} />
          </div>
        </section>

        <section className="card stats-card">
          <div className="section-heading">
            <div>
              <span className="section-kicker">05</span>
              <h2>Link statistics</h2>
            </div>
          </div>
          <div className="stats-grid">
            <Metric label="Control TX" value={stats.tx} />
            <Metric label="Telemetry RX" value={stats.rx} />
            <Metric label="Last seq" value={sequence} />
            <Metric label="Backend→Pi" value={bridgeStatus?.last_control_seq ?? '—'} />
          </div>
          <div className="bridge-note">
            <div><span>Pi IP</span><strong>{bridgeStatus?.pi_ip ?? '—'}</strong></div>
            <div><span>Pi telemetry age</span><strong>{bridgeStatus?.pi_telemetry_age_ms != null ? `${bridgeStatus.pi_telemetry_age_ms} ms` : '—'}</strong></div>
            <div><span>Last control age</span><strong>{bridgeStatus?.last_control_age_ms != null ? `${bridgeStatus.last_control_age_ms} ms` : '—'}</strong></div>
          </div>
        </section>

        <section className="card span-2">
          <div className="section-heading">
            <div>
              <span className="section-kicker">06</span>
              <h2>Protocol preview</h2>
            </div>
            <span className="muted">What the browser is actually sending</span>
          </div>

          <pre className="packet-preview">{lastTx ? JSON.stringify(lastTx, null, 2) : 'Waiting for first control packet…'}</pre>
        </section>

        <section className="card span-2">
          <div className="section-heading">
            <div>
              <span className="section-kicker">07</span>
              <h2>Event log</h2>
            </div>
          </div>
          <div className="log-window">
            {logLines.length === 0 ? <span className="muted">No events yet.</span> : logLines.map((line, index) => <div key={`${line}-${index}`}>{line}</div>)}
          </div>
        </section>
      </main>
    </div>
  )
}

function StatusPill({ label, state }) {
  const liveStates = new Set(['CONNECTED', 'LIVE'])
  const warningStates = new Set(['CONNECTING'])
  const tone = liveStates.has(state) ? 'live' : warningStates.has(state) ? 'warning' : 'idle'

  return (
    <div className={`status-pill ${tone}`}>
      <span className="status-dot" />
      <div>
        <small>{label}</small>
        <strong>{state}</strong>
      </div>
    </div>
  )
}

function AxisControl({ label, value, min, max, step, onChange }) {
  return (
    <label className="axis-control">
      <div className="axis-heading">
        <span>{label}</span>
        <strong>{Number(value).toFixed(2)}</strong>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(event) => onChange(event.target.value)}
      />
      <div className="range-labels"><span>{min}</span><span>{max}</span></div>
    </label>
  )
}

function Metric({ label, value }) {
  return (
    <div className="metric">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  )
}

function formatNumber(value) {
  return typeof value === 'number' ? value.toFixed(3) : '—'
}

export default App
