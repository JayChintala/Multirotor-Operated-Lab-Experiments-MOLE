"""Derive rigid-body mass properties from CAD and the supplied component masses.

Run with .venv/bin/python (CadQuery). All tensors use original CAD axes and SI.
Arm IDs below are the four matching long, flat CAD solids, using zero-based
indices. Electronics is a point at the assembled CoM; motors are point masses
at the configured mounts because the STEP assembly contains no motor geometry.
"""
import json
from pathlib import Path

import cadquery as cq
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ARM_IDS = [4, 12, 14, 18]
ARM_MASS_KG = 0.042
MOTOR_MASS_KG = 0.072
FRAME_ELECTRONICS_KG = 0.244


def main():
    config_path = ROOT / 'control/config/drone.json'
    config = json.loads(config_path.read_text())
    assembly = cq.Compound.makeCompound(cq.importers.importStep(
        str(ROOT / 'cad/QAV-RXL2_8inch.step')).vals()).scale(0.001)
    solids = assembly.Solids()
    if len(solids) != 19:
        raise ValueError('CAD assembly changed: inspect solids and update arm classification')
    volumes = np.array([s.Volume() for s in solids])
    arm_volumes = volumes[ARM_IDS]
    if not np.allclose(arm_volumes, arm_volumes.mean(), rtol=1e-6):
        raise ValueError('Expected four matching arm solids')
    density = ARM_MASS_KG / arm_volumes.mean()
    masses = volumes*density
    frame_mass = float(masses.sum()-4*ARM_MASS_KG)
    electronics_mass = FRAME_ELECTRONICS_KG-frame_mass
    if electronics_mass < 0:
        raise ValueError('CAD frame mass exceeds supplied frame-plus-electronics mass')
    components = []
    for i, solid in enumerate(solids):
        components.append({
            'name': f'cad_solid_{i}', 'kind': 'arm' if i in ARM_IDS else 'central_frame',
            'mass_kg': float(masses[i]), 'volume_m3': float(volumes[i]),
            'center_m': list(solid.Center().toTuple()),
            # OCCT's unit-density central inertia has units m^5.
            'central_inertia_kg_m2': (np.asarray(cq.Shape.matrixOfInertia(solid))*density).tolist()
        })
    for i, position in enumerate(config['motor_positions_m']):
        components.append({'name': f'motor_{i+1}', 'kind': 'motor_point_mass',
                           'mass_kg': MOTOR_MASS_KG, 'center_m': position,
                           'central_inertia_kg_m2': np.zeros((3,3)).tolist()})
    structural_mass = sum(part['mass_kg'] for part in components)
    com = sum(part['mass_kg']*np.asarray(part['center_m']) for part in components)/structural_mass
    # A point mass at the final CoM does not move that CoM or add central inertia.
    tensor = np.zeros((3,3))
    for part in components:
        r = np.asarray(part['center_m'])-com
        tensor += np.asarray(part['central_inertia_kg_m2']) + part['mass_kg']*(r@r*np.eye(3)-np.outer(r,r))
    total = structural_mass+electronics_mass
    eigenvalues = np.linalg.eigvalsh(tensor)
    if np.any(eigenvalues <= 0) or np.any(2*eigenvalues > eigenvalues.sum()):
        raise ValueError('Nonphysical inertia tensor')
    assumptions = [
        '244 g is approximate central frame plus electronics, excluding four arms and four motors.',
        'Uniform shared density inferred from four CAD arm volumes and 42 g per arm.',
        'All 15 non-arm CAD solids use this density, including small brackets and standoffs.',
        'Electronics is the residual of 244 g minus CAD central-frame mass, modeled as a point at total CoM.',
        'Four 72 g motors modeled as points at provisional mount coordinates; motor own-body/rotor inertia omitted.',
        'Original CAD axes assumed controller body axes; added batteries, propellers or payloads are not separately modeled.'
    ]
    report = {'cad_source': 'cad/QAV-RXL2_8inch.step', 'arm_solid_indices_zero_based': ARM_IDS,
              'density_kg_m3': float(density), 'central_frame_mass_kg': frame_mass,
              'electronics_point_mass_kg': electronics_mass, 'arm_mass_each_kg': ARM_MASS_KG,
              'motor_mass_each_kg': MOTOR_MASS_KG, 'total_mass_kg': total,
              'center_of_mass_cad_m': com.tolist(), 'inertia_about_com_kg_m2': tensor.tolist(),
              'principal_inertias_kg_m2': eigenvalues.tolist(), 'assumptions': assumptions,
              'components': components}
    (ROOT/'cad/mass_properties.json').write_text(json.dumps(report, indent=2)+'\n')
    config['mass_kg'] = total
    config['com_m'] = com.tolist()
    config['inertia_kg_m2'] = np.diag(tensor).tolist()
    config['inertia_products_kg_m2'] = [float(tensor[0,1]), float(tensor[0,2]), float(tensor[1,2])]
    config['assumptions'] = ' '.join(assumptions) + ' Motor lag and drag are assumed, not identified.'
    config_path.write_text(json.dumps(config, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'components'}, indent=2))


if __name__ == '__main__':
    main()
