#!/usr/bin/env python3
"""Reproducible conservative 5 cm map from fixed, planar SDF collisions.

Only literal world/model geometry is included, never the xacro robot macro or
movable presentation props. Unknown shapes/tilted boxes fail instead of silently
omitting a physical obstacle. Floor and flush dock pads are below the slice.
"""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def pose(element):
    values = list(map(float, element.findtext('pose', '0 0 0 0 0 0').split()))
    if abs(values[3]) > 1e-6 or abs(values[4]) > 1e-6:
        raise ValueError('map generator requires planar fixed geometry')
    return values[0], values[1], values[2], values[5]


def compose(a, b):
    x, y, z, yaw = a
    dx, dy, dz, dyaw = b
    return (x + math.cos(yaw)*dx - math.sin(yaw)*dy,
            y + math.sin(yaw)*dx + math.cos(yaw)*dy, z+dz, yaw+dyaw)


def obstacles(world):
    result = []
    for model in ET.parse(world).getroot().findall('world/model'):
        if model.findtext('static') != 'true' or model.get('name') == 'floor':
            continue
        for link in model.findall('link'):
            parent = compose(pose(model), pose(link))
            for collision in link.findall('collision'):
                x, y, z, yaw = compose(parent, pose(collision))
                size = collision.findtext('geometry/box/size')
                if size is None:
                    raise ValueError(f'unsupported collision: {model.get("name")}')
                sx, sy, sz = map(float, size.split())
                if z + sz/2 <= 0.05 or z - sz/2 >= 0.50:
                    continue
                result.append((x, y, sx, sy, yaw, model.get('name')))
    return result


def clearance(x, y, items):
    distances = []
    for cx, cy, sx, sy, yaw, _ in items:
        dx, dy = x-cx, y-cy
        lx = abs(math.cos(yaw)*dx + math.sin(yaw)*dy)-sx/2
        ly = abs(-math.sin(yaw)*dx + math.cos(yaw)*dy)-sy/2
        distances.append(math.hypot(max(0, lx), max(0, ly)))
    return min(distances)


def generate(world, output):
    items = obstacles(world)
    resolution, origin, width = 0.05, -5.5, 220
    cells = []
    for row in range(width):
        y = origin + (width-row-0.5)*resolution
        cells.append(['0' if clearance(origin+(col+0.5)*resolution,y,items)
                      <= resolution*math.sqrt(2)/2 else '254' for col in range(width)])
    output.write_text('P2\n# Generated from indoor_room.sdf.xacro fixed collisions\n'
                      f'{width} {width}\n255\n' + '\n'.join(' '.join(r) for r in cells)+'\n')
    output.with_suffix('.yaml').write_text(
        f'image: {output.name}\nmode: trinary\nresolution: {resolution}\n'
        f'origin: [{origin}, {origin}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'Generated {width}x{width} map with {len(items)} collision boxes: {output}')


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, default=root/'src/navcraft_simulation/worlds/indoor_room.sdf.xacro')
    parser.add_argument('--output', type=Path, default=root/'src/navcraft_navigation/maps/indoor_room.pgm')
    args = parser.parse_args()
    generate(args.world, args.output)
