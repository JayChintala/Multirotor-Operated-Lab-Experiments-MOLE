#!/usr/bin/env python3
"""Generate models/mole_quad/model.sdf from config/vehicle.json.

All physical numbers live in vehicle.json; this script turns them into the
Gazebo model: combined rigid-body mass and inertia, four propeller joints with
blade lift/drag sized to the measured thrust and torque curves, the IMU, and
the ArduPilot plugin wired to the real flight controller's output channels.

    python3 scripts/gen_model.py                    # from the package directory
    python3 scripts/gen_model.py --load water=0     # override a load's mass (0 = empty)
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
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


def stl_points(path):
    """Vertices of a binary STL (as written by import_water_module.py), shape (n, 3)."""
    b = path.read_bytes()
    n = int.from_bytes(b[80:84], 'little')
    rec = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('x', '<u2')]))
    return rec['v'].reshape(-1, 3).astype(float)


def urdf_mass_properties(path):
    """Total mass, CoM and inertia about the CoM of a URDF whose links hang off the root by fixed joints."""
    root = ET.parse(path).getroot()
    offset = {}
    for j in root.findall('joint'):
        if j.get('type') != 'fixed':
            raise ValueError(f'{path}: joint {j.get("name")} is not fixed')
        o = j.find('origin')
        if o is not None and any(float(a) for a in o.get('rpy', '0 0 0').split()):
            raise ValueError(f'{path}: rotated joints are not supported')
        offset[j.find('child').get('link')] = (j.find('parent').get('link'),
                                               np.array([float(a) for a in (o.get('xyz') if o is not None else '0 0 0').split()]))

    def link_pos(name):
        pos = np.zeros(3)
        while name in offset:
            name, xyz = offset[name]
            pos = pos + xyz
        return pos

    parts = []
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        o = inertial.find('origin')
        com = np.array([float(a) for a in (o.get('xyz') if o is not None else '0 0 0').split()])
        a = inertial.find('inertia').attrib
        I = np.array([[float(a['ixx']), float(a['ixy']), float(a['ixz'])],
                      [float(a['ixy']), float(a['iyy']), float(a['iyz'])],
                      [float(a['ixz']), float(a['iyz']), float(a['izz'])]])
        parts.append((float(inertial.find('mass').get('value')), link_pos(link.get('name')) + com, I))
    return combine(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--load', action='append', default=[], metavar='NAME=KG',
                        help='override the mass of a load from vehicle.json (0 removes it)')
    args = parser.parse_args()
    overrides = {}
    for item in args.load:
        name, kg = item.split('=')
        overrides[name] = float(kg)
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
    payloads = []
    for pl in v.get('payloads', []):
        if not pl.get('enabled', True):
            continue
        pm_, pcom_mod, pI_mod = urdf_mass_properties(PKG / pl['urdf'])
        Rp = rot_z(pl['yaw_deg'])
        # Module origin placed so its top face (top_z_m in module axes) sits at mount_top_z_m in body axes.
        origin = np.array([*pl['mount_xy_m'], pl['mount_top_z_m'] - pl['top_z_m']])
        pcom = origin + Rp @ pcom_mod
        size = np.array(pl['box_size_m'])                     # body axes
        box_centre = np.array([*pl['mount_xy_m'], pl['mount_top_z_m'] - size[2] / 2])
        p = {'name': pl['name'], 'mass': pm_, 'com': pcom, 'I': Rp @ pI_mod @ Rp.T,
             'size': size, 'centre': box_centre, 'origin': origin, 'R': Rp, 'yaw': math.radians(pl['yaw_deg']),
             'visuals': pl.get('visual_meshes', []), 'collisions': pl.get('collision_meshes', []),
             'mesh_dir': pl.get('mesh_dir', '')}
        if p['visuals']:
            # Lowest point from the collision meshes (module axes -> body axes).
            pts = np.concatenate([stl_points(PKG / 'models/mole_quad' / p['mesh_dir'] / f'{c}.stl')
                                  for c in p['collisions']])
            p['lowest'] = (origin + pts @ Rp.T)[:, 2].min()
        else:
            p['lowest'] = box_centre[2] - size[2] / 2
        payloads.append(p)
    # Landing gear: vertical legs (cylinders) with a ball foot, fixed to the frame.
    gear = v.get('landing_gear', {})
    legs = []
    if gear.get('enabled'):
        top, bottom, r, rf = gear['top_z_m'], gear['bottom_z_m'], gear['leg_radius_m'], gear['foot_radius_m']
        h = top - (bottom + rf)                                   # leg ends at the foot's centre
        mg = gear['mass_kg_each']
        I_leg = np.diag([mg / 12 * (3 * r * r + h * h)] * 2 + [mg * r * r / 2])
        for k, (x, y) in enumerate(gear['legs_xy_m']):
            legs.append({'name': f'leg_{k + 1}', 'x': x, 'y': y, 'z': top - h / 2, 'h': h, 'r': r,
                         'foot_z': bottom + rf, 'rf': rf, 'mass': mg, 'I': I_leg})
    base_m, base_com, base_I = combine([frame_part, bat_part]
                                       + [(p['mass'], p['com'], p['I']) for p in payloads]
                                       + [(lg['mass'], np.array([lg['x'], lg['y'], lg['z']]), lg['I']) for lg in legs])

    # Loads carried inside a payload (e.g. water): separate links, fixed to base_link for now.
    loads = []
    for ld in v.get('loads', []):
        mass = overrides.get(ld['name'], ld['mass_kg'] if ld.get('enabled', True) else 0.0)
        if mass <= 0:
            continue
        host = next(p for p in payloads if p['name'] == ld['inside'])
        if 'tank_min_m' in ld:
            # Water fills the measured tank (module axes) from the floor up.
            lo, hi = np.array(ld['tank_min_m']), np.array(ld['tank_max_m'])
            footprint = (hi[0] - lo[0]) * (hi[1] - lo[1])
            volume = mass / ld.get('density_kg_m3', 1000.0)
            capacity = footprint * (hi[2] - lo[2])
            if volume > capacity * 1.001:
                print(f'WARNING: {ld["name"]}: {volume * 1e6:.0f} ml does not fit in the {capacity * 1e6:.0f} ml tank; '
                      f'modelled as a full tank carrying {mass * 1000:.0f} g')
            height = min(volume / footprint, hi[2] - lo[2])
            size_mod = np.array([hi[0] - lo[0], hi[1] - lo[1], height])
            centre_mod = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2] + height / 2])
            centre = host['origin'] + host['R'] @ centre_mod
            size = np.abs(host['R']) @ size_mod                    # exact for yaw multiples of 90 deg
            I = host['R'] @ box_inertia(mass, *size_mod) @ host['R'].T
        else:
            size = host['size'] * np.array([1.0, 1.0, ld['fill_height_fraction']])
            bottom = host['centre'][2] - host['size'][2] / 2
            centre = np.array([host['centre'][0], host['centre'][1], bottom + size[2] / 2])
            I = box_inertia(mass, *size)
        loads.append({'name': ld['name'], 'mass': mass, 'size': size, 'centre': centre, 'I': I})

    # Props as separate spinning links (thin rod about z for the spin inertia).
    pm, pr, pj = prop['mass_kg'], prop['diameter_m'] / 2, prop['spin_inertia_kg_m2']
    prop_I = np.diag([1e-7, pj, pj])
    total_m, total_com, total_I = combine(
        [(base_m, base_com, base_I)]
        + [(pm, np.array([*m['xy_m'], prop['z_m']]), prop_I) for m in mot['list']]
        + [(ld['mass'], ld['centre'], ld['I']) for ld in loads])

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
    if mot.get('max_torque_nm'):
        # Motor torque limit (current limit): caps rotor speed and thrust.
        cmd_max = mot['max_torque_nm']
        w_max = math.sqrt(cmd_max / kQ)
    else:
        # Real hardware, no current limit: full output reaches the extrapolated top speed.
        # Limits on the real drone are set in firmware (MOT_SPIN_MAX), which SITL applies too.
        w_max = mot['speed_per_cmd_rad_s'] * (1 + mot['speed_offset'])
        cmd_max = 2.0 * kQ * w_max ** 2                       # torque headroom to accelerate the rotor
    torque_max = kQ * w_max ** 2

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

    def mesh_payload_xml(p):
        pose = f"{p['origin'][0]:.4f} {p['origin'][1]:.4f} {p['origin'][2]:.4f} 0 0 {p['yaw']:.6f}"
        uri = f"model://mole_quad/{p['mesh_dir']}"
        xml = ''.join(f'''
      <collision name="{p['name']}_{c}">
        <pose>{pose}</pose>
        <geometry><mesh><uri>{uri}/{c}.stl</uri></mesh></geometry>
      </collision>''' for c in p['collisions'])
        for vm in p['visuals']:
            rgba = vm['rgba']
            xml += f'''
      <visual name="{p['name']}_{vm['mesh']}">
        <pose>{pose}</pose>
        <geometry><mesh><uri>{uri}/{vm['mesh']}.stl</uri></mesh></geometry>{
            f"""
        <transparency>{1 - rgba[3]:.2f}</transparency>""" if rgba[3] < 1 else ''}
        {material(*rgba)}
      </visual>'''
        return xml

    payload_xml = ''.join(mesh_payload_xml(p) for p in payloads if p['visuals']) + ''.join(f'''
      <collision name="{p['name']}_collision">
        <pose>{p['centre'][0]:.4f} {p['centre'][1]:.4f} {p['centre'][2]:.4f} 0 0 0</pose>
        <geometry><box><size>{p['size'][0]} {p['size'][1]} {p['size'][2]}</size></box></geometry>
      </collision>
      <visual name="{p['name']}">
        <pose>{p['centre'][0]:.4f} {p['centre'][1]:.4f} {p['centre'][2]:.4f} 0 0 0</pose>
        <geometry><box><size>{p['size'][0]} {p['size'][1]} {p['size'][2]}</size></box></geometry>
        <transparency>0.6</transparency>
        {material(0.75, 0.78, 0.82, 0.4)}
      </visual>''' for p in payloads if not p['visuals'])

    gear_xml = ''.join(f'''
      <collision name="{lg['name']}_foot">
        <pose>{lg['x']} {lg['y']} {lg['foot_z']:.4f} 0 0 0</pose>
        <geometry><sphere><radius>{lg['rf']}</radius></sphere></geometry>
      </collision>
      <visual name="{lg['name']}">
        <pose>{lg['x']} {lg['y']} {lg['z']:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{lg['r']}</radius><length>{lg['h']:.4f}</length></cylinder></geometry>
        {material(0.1, 0.1, 0.1)}
      </visual>
      <visual name="{lg['name']}_foot">
        <pose>{lg['x']} {lg['y']} {lg['foot_z']:.4f} 0 0 0</pose>
        <geometry><sphere><radius>{lg['rf']}</radius></sphere></geometry>
        {material(0.9, 0.3, 0.1)}
      </visual>''' for lg in legs)

    load_xml = ''.join(f'''
    <link name="{ld['name']}">
      <pose>{ld['centre'][0]:.4f} {ld['centre'][1]:.4f} {ld['centre'][2]:.4f} 0 0 0</pose>
      <inertial>
        <mass>{ld['mass']:.4f}</mass>
        <inertia>
{inertia_xml(ld['I'], 10)}
        </inertia>
      </inertial>
      <visual name="{ld['name']}_visual">
        <geometry><box><size>{ld['size'][0] * 0.98:.4f} {ld['size'][1] * 0.98:.4f} {ld['size'][2]:.4f}</size></box></geometry>
        <transparency>0.3</transparency>
        {material(0.1, 0.35, 0.95, 0.7)}
      </visual>
    </link>
    <joint name="{ld['name']}_mount" type="fixed">
      <parent>base_link</parent>
      <child>{ld['name']}</child>
    </joint>''' for ld in loads)

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
{payload_xml}{gear_xml}
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
{load_xml}
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
    for p in payloads:
        print(f'  payload {p["name"]}: {p["mass"] * 1000:.1f} g, CoM {np.round(p["com"] * 1000, 1)} mm, '
              f'box {np.round(p["size"] * 1000).astype(int).tolist()} mm')
    for ld in loads:
        vol = np.prod(ld['size']) * 1e6
        print(f'  load {ld["name"]}: {ld["mass"] * 1000:.0f} g in a {np.round(ld["size"] * 1000, 1).tolist()} mm '
              f'block ({vol:.0f} ml), centre {np.round(ld["centre"] * 1000, 1).tolist()} mm')
    print(f'  total mass {total_m:.3f} kg, CoG {np.round(total_com * 1000, 1)} mm')
    print(f'  inertia diag {np.round(np.diag(total_I), 5)} kg m^2')
    print(f'  blade area {blade_area:.3e} m^2 at r_cp {r_cp:.4f} m, cda {cda:.4f}')
    print(f'  rotor PI p={p_gain:.4g} i={i_gain:.4g}; full-output torque {torque_max:.3f} Nm, {w_max:.0f} rad/s, '
          f'{kT * w_max ** 2:.2f} N ({kT * w_max ** 2 / 9.80665:.2f} kgf) per motor, '
          f'thrust/weight {4 * kT * w_max ** 2 / (total_m * 9.80665):.2f}')
    w_h = math.sqrt(total_m * 9.80665 / 4 / kT)
    hover_cmd = w_h / mot['speed_per_cmd_rad_s'] - mot['speed_offset']
    print(f'  expected hover: {w_h:.0f} rad/s, cmd {hover_cmd:.3f}')
    lowest = min([fr['bottom_z_m']] + [p['lowest'] for p in payloads] + [lg['foot_z'] - lg['rf'] for lg in legs])
    if legs:
        print(f'  landing gear (ASSUMED): {len(legs)} legs x {legs[0]["mass"] * 1000:.0f} g, '
              f'feet at {np.round((legs[0]["foot_z"] - legs[0]["rf"]) * 1000)} mm')
    (PKG / 'models/mole_quad/predicted.json').write_text(json.dumps({
        '_comment': 'Written by gen_model.py; read by smoke_test.py',
        'total_mass_kg': round(total_m, 4), 'hover_cmd': round(hover_cmd, 4),
        'payloads': [p['name'] for p in payloads] + [f"{ld['name']} {ld['mass'] * 1000:.0f} g" for ld in loads],
        'lowest_point_z_m': round(lowest, 4)}, indent=2) + '\n')
    print(f'  lowest point {lowest * 1000:.0f} mm below the frame centre: spawn the model at least that high')


if __name__ == '__main__':
    main()
