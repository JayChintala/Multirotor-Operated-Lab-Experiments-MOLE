#!/usr/bin/env python3
"""Generate models/mole_quad/model.sdf from config/vehicle.json.

All physical numbers live in vehicle.json; this script turns them into the
Gazebo model: combined rigid-body mass and inertia, four propeller joints with
blade lift/drag sized to the measured thrust and torque curves, the IMU, and
the ArduPilot plugin wired to the real flight controller's output channels.

    python3 scripts/gen_model.py          # from the package directory
"""
import json
import math
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def box_inertia(m, sx, sy, sz):
    return m / 12 * np.diag([sy * sy + sz * sz, sx * sx + sz * sz, sx * sx + sy * sy])


def combine(parts):
    """parts: (mass, com, inertia about own com) -> total mass, com, inertia about com."""
    m = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / m
    inertia = np.zeros((3, 3))
    for mi, ri, Ii in parts:
        d = ri - com
        inertia += Ii + mi * ((d @ d) * np.eye(3) - np.outer(d, d))
    return m, com, inertia


def inertia_xml(I, indent):
    pad = ' ' * indent
    names = [('ixx', 0, 0), ('ixy', 0, 1), ('ixz', 0, 2), ('iyy', 1, 1), ('iyz', 1, 2), ('izz', 2, 2)]
    return '\n'.join(f'{pad}<{n}>{I[i, j]:.6g}</{n}>' for n, i, j in names)


def material(r, g, b, a=1.0):
    return (f'<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>'
            f'<specular>0.2 0.2 0.2 1</specular></material>')


def main():
    v = json.loads((PKG / 'config/vehicle.json').read_text())
    fr, bat, prop, mot, aero = v['frame'], v['battery'], v['props'], v['motors'], v['aero']

    # Frame (CAD axes) rotated into body axes, plus the battery: one rigid body.
    R = rot_z(fr['cad_yaw_deg'])
    i = fr['inertia_cad_kg_m2']
    I_cad = np.array([[i['ixx'], i['ixy'], i['ixz']], [i['ixy'], i['iyy'], i['iyz']], [i['ixz'], i['iyz'], i['izz']]])
    frame_part = (fr['mass_kg'], R @ np.array(fr['com_cad_m']), R @ I_cad @ R.T)
    bx, by, bz = bat['size_m']
    bat_com = np.array([0.0, 0.0, bat['bottom_z_m'] + bz / 2])
    bat_part = (bat['mass_kg'], bat_com, box_inertia(bat['mass_kg'], bx, by, bz))
    base_m, base_com, base_I = combine([frame_part, bat_part])

    # Props as separate spinning links (thin rod about z for the spin inertia).
    pm, pr, pj = prop['mass_kg'], prop['diameter_m'] / 2, prop['spin_inertia_kg_m2']
    prop_I = np.diag([1e-7, pj, pj])
    total_m, total_com, total_I = combine(
        [(base_m, base_com, base_I)]
        + [(pm, np.array([*m['xy_m'], prop['z_m']]), prop_I) for m in mot['list']])

    # Blade lift/drag: two blades per rotor at cp = 0.7 R.
    # Hover thrust T = rho (w r)^2 A cla a0  ->  A; torque/thrust = cda r / cla -> cda.
    rho, cla, a0 = aero['air_density_kg_m3'], aero['cla'], aero['a0']
    r_cp = aero['cp_fraction_of_radius'] * pr
    kT, kQ = mot['thrust_coeff_n_per_rad_s2'], mot['torque_coeff_nm_per_rad_s2']
    blade_area = kT / (rho * r_cp ** 2 * cla * a0)
    cda = (kQ / kT) * cla / r_cp

    # Rotor speed loop: PI on joint torque giving the motor time constant.
    tau = mot['time_constant_s']
    p_gain = pj / tau
    i_gain = p_gain ** 2 / (4 * 0.8 ** 2 * pj)          # damping ratio 0.8
    # Motor torque limit (current limit): caps rotor speed and thrust at the measured maximum.
    cmd_max = mot['max_torque_nm']
    w_max = math.sqrt(cmd_max / kQ)
    torque_max = cmd_max

    rotors, liftdrag, controls = [], [], []
    for k, m in enumerate(mot['list']):
        x, y = m['xy_m']
        sign = 1 if m['spin'] == 'ccw' else -1          # +z rotation = CCW seen from above
        colour = (0.15, 0.4, 0.95) if sign > 0 else (1.0, 0.5, 0.1)
        name = f'rotor_{m["name"]}'
        rotors.append(f'''
    <link name="{name}">
      <pose>{x} {y} {prop['z_m']} 0 0 0</pose>
      <inertial>
        <mass>{pm}</mass>
        <inertia>
{inertia_xml(prop_I, 10)}
        </inertia>
      </inertial>
      <visual name="blades">
        <geometry><box><size>{2 * pr:.4f} 0.018 0.003</size></box></geometry>
        {material(*colour)}
      </visual>
      <visual name="disc">
        <geometry><cylinder><radius>{pr:.4f}</radius><length>0.001</length></cylinder></geometry>
        <transparency>0.75</transparency>
        {material(*colour, 0.25)}
      </visual>
    </link>
    <joint name="{name}_joint" type="revolute">
      <parent>base_link</parent>
      <child>{name}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>1e-6</damping></dynamics>
      </axis>
    </joint>''')
        for side in (1, -1):
            liftdrag.append(f'''
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <link_name>{name}</link_name>
      <cp>{side * r_cp:.5f} 0 0</cp>
      <forward>0 {side * sign} 0</forward>
      <upward>0 0 1</upward>
      <a0>{a0}</a0>
      <cla>{cla}</cla>
      <cda>{cda:.5f}</cda>
      <cma>0</cma>
      <alpha_stall>1.4</alpha_stall>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0</cda_stall>
      <cma_stall>0</cma_stall>
      <area>{blade_area:.6g}</area>
      <air_density>{rho}</air_density>
    </plugin>''')
        controls.append(f'''
      <!-- SERVO{m["servo_output"]} = ArduPilot motor {m["ardupilot_motor"]} ({m["name"].replace("_", "-")}, {m["spin"].upper()}) -->
      <control channel="{m['servo_output'] - 1}">
        <jointName>{name}_joint</jointName>
        <useForce>1</useForce>
        <type>VELOCITY</type>
        <multiplier>{sign * mot['speed_per_cmd_rad_s']}</multiplier>
        <offset>{mot['speed_offset']}</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
        <p_gain>{p_gain:.6g}</p_gain>
        <i_gain>{i_gain:.6g}</i_gain>
        <d_gain>0</d_gain>
        <i_max>{cmd_max:.4g}</i_max>
        <i_min>{-cmd_max:.4g}</i_min>
        <cmd_max>{cmd_max:.4g}</cmd_max>
        <cmd_min>{-cmd_max:.4g}</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>''')

    fx, fy = fr['footprint_m']
    plate_h = fr['top_z_m'] - fr['bottom_z_m']
    yaw = math.radians(fr['cad_yaw_deg'])
    motor_visuals = ''.join(f'''
      <visual name="motor_{m['name']}">
        <pose>{m['xy_m'][0]} {m['xy_m'][1]} 0.0075 0 0 0</pose>
        <geometry><cylinder><radius>0.014</radius><length>0.020</length></cylinder></geometry>
        {material(0.08, 0.08, 0.08)}
      </visual>''' for m in mot['list'])

    sdf = f'''<?xml version="1.0"?>
<!-- GENERATED by scripts/gen_model.py from config/vehicle.json. Do not edit by hand. -->
<!-- Total {total_m:.3f} kg; CoG {np.round(total_com * 1000, 1).tolist()} mm; inertia diag {np.round(np.diag(total_I), 5).tolist()} kg m^2 -->
<sdf version="1.9">
  <model name="mole_quad">
    <link name="base_link">
      <inertial>
        <pose>{base_com[0]:.6f} {base_com[1]:.6f} {base_com[2]:.6f} 0 0 0</pose>
        <mass>{base_m:.4f}</mass>
        <inertia>
{inertia_xml(base_I, 10)}
        </inertia>
      </inertial>
      <collision name="frame_collision">
        <pose>0 0 {(fr['top_z_m'] + fr['bottom_z_m']) / 2:.4f} 0 0 0</pose>
        <geometry><box><size>{fx} {fy} {plate_h:.4f}</size></box></geometry>
      </collision>
      <collision name="battery_collision">
        <pose>{bat_com[0]} {bat_com[1]} {bat_com[2]:.4f} 0 0 0</pose>
        <geometry><box><size>{bx} {by} {bz}</size></box></geometry>
      </collision>
      <visual name="frame">
        <pose>0 0 0 0 0 {yaw:.6f}</pose>
        <geometry><mesh><uri>model://mole_quad/{fr['mesh']}</uri></mesh></geometry>
        {material(0.24, 0.30, 0.38)}
      </visual>
      <visual name="battery">
        <pose>{bat_com[0]} {bat_com[1]} {bat_com[2]:.4f} 0 0 0</pose>
        <geometry><box><size>{bx} {by} {bz}</size></box></geometry>
        {material(0.85, 0.75, 0.1)}
      </visual>
      <visual name="front_marker">
        <pose>{fx / 2 + 0.01:.4f} 0 {fr['top_z_m']:.4f} 0 0 0</pose>
        <geometry><sphere><radius>0.012</radius></sphere></geometry>
        {material(0.9, 0.1, 0.1)}
      </visual>{motor_visuals}
      <sensor name="imu_sensor" type="imu">
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on>
        <update_rate>1000</update_rate>
      </sensor>
    </link>
{''.join(rotors)}
{''.join(liftdrag)}

    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>

    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>base_link::imu_sensor</imuName>
{''.join(controls)}
    </plugin>
  </model>
</sdf>
'''
    out = PKG / 'models/mole_quad/model.sdf'
    out.write_text(sdf)
    (PKG / 'models/mole_quad/model.config').write_text('''<?xml version="1.0"?>
<model>
  <name>mole_quad</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>MOLE quadcopter (QAV-RXL2 frame, 6S, 8 inch props) for ArduPilot SITL. Generated by scripts/gen_model.py.</description>
</model>
''')
    print(f'Wrote {out}')
    print(f'  total mass {total_m:.3f} kg, CoG {np.round(total_com * 1000, 1)} mm')
    print(f'  inertia diag {np.round(np.diag(total_I), 5)} kg m^2')
    print(f'  blade area {blade_area:.3e} m^2 at r_cp {r_cp:.4f} m, cda {cda:.4f}')
    print(f'  rotor PI p={p_gain:.4g} i={i_gain:.4g}; torque cap {torque_max:.3f} Nm -> max {w_max:.0f} rad/s, '
          f'{kT * w_max ** 2:.2f} N ({kT * w_max ** 2 / 9.80665:.2f} kgf) per motor, '
          f'thrust/weight {4 * kT * w_max ** 2 / (total_m * 9.80665):.2f}')
    w_h = math.sqrt(total_m * 9.80665 / 4 / kT)
    print(f'  expected hover: {w_h:.0f} rad/s, cmd {w_h / mot["speed_per_cmd_rad_s"] - mot["speed_offset"]:.3f}')


if __name__ == '__main__':
    main()
