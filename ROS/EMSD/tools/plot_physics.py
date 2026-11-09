"""Plot recorded physical state, references and motor outputs from validation."""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/emsd_matplotlib')
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parents[1]/'analysis/physics_validation'
data = np.genfromtxt(OUT/'trajectory/telemetry.csv', delimiter=',', names=True)
t = data['time_s']
fig = plt.figure(figsize=(12, 9), constrained_layout=True)
ax = fig.add_subplot(2, 2, 1, projection='3d')
ax.plot(data['ref_x_m'], data['ref_y_m'], data['ref_z_m'], '--', label='Reference')
ax.plot(data['x_m'], data['y_m'], data['z_m'], label='DART physical state')
ax.set(xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]', title='Takeoff and figure-eight')
ax.legend(fontsize=8)
ax = fig.add_subplot(2, 2, 2)
ax.plot(t, data['ref_z_m'], '--', label='Reference')
ax.plot(t, data['z_m'], label='Physical altitude')
ax.set(xlabel='Simulation time [s]', ylabel='Altitude [m]', title='Closed-loop altitude')
ax.legend()
ax.grid(alpha=.25)
ax = fig.add_subplot(2, 2, 3)
for key, label in [('roll_rad', 'Roll'), ('pitch_rad', 'Pitch'), ('yaw_rad', 'Yaw')]:
    ax.plot(t, np.degrees(data[key]), label=label)
ax.set(xlabel='Simulation time [s]', ylabel='Angle [degrees]', title='Orientation produced by motor forces')
ax.legend()
ax.grid(alpha=.25)
ax = fig.add_subplot(2, 2, 4)
for i in range(1, 5):
    ax.plot(t, data[f'thrust_{i}_n'], label=f'Motor {i}', linewidth=1)
ax.set(xlabel='Simulation time [s]', ylabel='Thrust [N]', title='Individual motor outputs')
ax.legend(ncol=2)
ax.grid(alpha=.25)
fig.suptitle('700 g drone · Gazebo DART physics · CAD-derived approximate inertia', fontsize=14)
for extension in ('png', 'pdf'):
    filename = OUT/f'physics_flight.{extension}'
    fig.savefig(filename, dpi=170)
    print(filename)
