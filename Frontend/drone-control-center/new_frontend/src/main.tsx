import React from "react";
import ReactDOM from "react-dom/client";
import "./style.css";

type IMUData = {
  roll: number;
  pitch: number;
  yaw: number;
  ax: number;
  ay: number;
  az: number;
  gx: number;
  gy: number;
  gz: number;
};

const imu: IMUData = {
  roll: 2.31,
  pitch: -1.17,
  yaw: 127.42,
  ax: 0.03,
  ay: -0.01,
  az: 9.79,
  gx: 0.12,
  gy: -0.08,
  gz: 0.21,
};

function App() {
  return (
    <main className="screen">
      <section className="panel camera-panel">
        <div className="panel-title">CAMERA FEED</div>
        <div className="camera-placeholder">
          <span>NO VIDEO SIGNAL</span>
        </div>
      </section>

      <section className="panel imu-panel">
        <div className="panel-title">IMU</div>

        <div className="imu-section">
          <h2>Orientation</h2>
          <div className="readings">
            <Reading label="Roll" value={`${imu.roll.toFixed(2)}°`} />
            <Reading label="Pitch" value={`${imu.pitch.toFixed(2)}°`} />
            <Reading label="Yaw" value={`${imu.yaw.toFixed(2)}°`} />
          </div>
        </div>

        <div className="imu-section">
          <h2>Acceleration</h2>
          <div className="readings">
            <Reading label="X" value={`${imu.ax.toFixed(2)} m/s²`} />
            <Reading label="Y" value={`${imu.ay.toFixed(2)} m/s²`} />
            <Reading label="Z" value={`${imu.az.toFixed(2)} m/s²`} />
          </div>
        </div>

        <div className="imu-section">
          <h2>Angular Velocity</h2>
          <div className="readings">
            <Reading label="X" value={`${imu.gx.toFixed(2)} rad/s`} />
            <Reading label="Y" value={`${imu.gy.toFixed(2)} rad/s`} />
            <Reading label="Z" value={`${imu.gz.toFixed(2)} rad/s`} />
          </div>
        </div>
      </section>

      <section className="panel empty-panel">
        <div className="panel-title">MAP / TELEMETRY</div>
      </section>

      <section className="panel empty-panel">
        <div className="panel-title">CONTROLS</div>
      </section>
    </main>
  );
}

function Reading({ label, value }: { label: string; value: string }) {
  return (
    <div className="reading">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
