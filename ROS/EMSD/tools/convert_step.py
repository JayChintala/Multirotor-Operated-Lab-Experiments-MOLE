#!/usr/bin/env python3
"""Requires CadQuery. Preserve assembly placement; convert OCCT millimetres to ROS metres."""
import json
from pathlib import Path
import cadquery as cq

root = Path(__file__).resolve().parents[1]
source = root / 'cad/QAV-RXL2_8inch.step'
parts = cq.importers.importStep(str(source)).vals()
shape = cq.Compound.makeCompound(parts)
# STEP import performs the STEP unit conversion to OCCT's millimetre convention.
shape = shape.scale(0.001)
mesh = root / 'ros2_ws/src/emsd_drone_description/meshes/frame.stl'
cq.exporters.export(shape, str(mesh), tolerance=0.0002, angularTolerance=0.15)
b = shape.BoundingBox()
report = {'source': source.name, 'mesh_units': 'metres',
          'dimensions_m': [b.xlen, b.ylen, b.zlen],
          'bounds_min_m': [b.xmin, b.ymin, b.zmin],
          'bounds_max_m': [b.xmax, b.ymax, b.zmax],
          'solid_count': len(shape.Solids()),
          'orientation': 'Original CAD axes; verify Z-up and X-forward before simulation.'}
(root / 'cad/conversion_report.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
print('Exported', mesh)
