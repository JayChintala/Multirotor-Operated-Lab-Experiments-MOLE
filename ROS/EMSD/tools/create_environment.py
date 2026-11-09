"""Generate metre-scale ground and hemispherical pond meshes for Gazebo."""
from pathlib import Path
import math

MESHES = Path(__file__).resolve().parents[1] / 'ros2_ws/src/emsd_drone_description/meshes'


def write_stl(name, triangles):
    with (MESHES / name).open('w') as out:
        out.write('solid environment\n')
        for a, b, c in triangles:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
            length = math.sqrt(sum(x*x for x in n))
            if length < 1e-12:
                continue
            out.write('facet normal ' + ' '.join(str(x/length) for x in n) + '\nouter loop\n')
            for point in (a, b, c):
                out.write('vertex ' + ' '.join(map(str, point)) + '\n')
            out.write('endloop\nendfacet\n')
        out.write('endsolid environment\n')


def generate():
    angles = sorted(set([2*math.pi*i/256 for i in range(256)] +
                        [math.atan2(y-3, x-2) % (2*math.pi) for x, y in [(-5,-5),(5,-5),(5,5),(-5,5)]]))
    inner, outer = [], []
    for angle in angles:
        dx, dy = math.cos(angle), math.sin(angle)
        limits = []
        if abs(dx) > 1e-12:
            limits.append(((5 if dx > 0 else -5)-2)/dx)
        if abs(dy) > 1e-12:
            limits.append(((5 if dy > 0 else -5)-3)/dy)
        distance = min(limits)
        inner.append((2+2*dx, 3+2*dy, 0))
        outer.append((2+distance*dx, 3+distance*dy, 0))
    ground, bowl = [], []
    for i in range(len(angles)):
        j = (i+1) % len(angles)
        ground.extend([(inner[i], outer[i], outer[j]), (inner[i], outer[j], inner[j])])
    rings = [[(2*math.cos(k*math.pi/64)*math.cos(a),
               2*math.cos(k*math.pi/64)*math.sin(a),
               -2*math.sin(k*math.pi/64)) for a in angles] for k in range(33)]
    for top, bottom in zip(rings, rings[1:]):
        for i in range(len(angles)):
            j = (i+1) % len(angles)
            bowl.extend([(top[i], top[j], bottom[j]), (top[i], bottom[j], bottom[i])])
    write_stl('ground_pond.stl', ground)
    write_stl('pond_bowl.stl', bowl)


if __name__ == '__main__':
    generate()
