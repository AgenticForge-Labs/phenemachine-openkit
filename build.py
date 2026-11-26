"""Build the OpenKit STEP and STL files from the CAD source.

    uv run python build.py --out hardware
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cadquery as cq

from cad import carrier as K
from cad import floor as F
from cad import imager as I
from cad import rail as R
from cad.candidate import parts as P
from cad.robot_stand import robot_stand

ROOT = Path(__file__).resolve().parent


def kit_parts(p) -> dict:
    fork = P.print_oriented('robot_fork_body', P.robot_fork_body(p))
    return {
        'floor_tile': F.floor_tile(),
        'robot_stand_30': robot_stand(30.0),
        'robot_stand_45': robot_stand(45.0),
        'robot_stand_55': robot_stand(55.0),
        'robot_fork_body': fork,
        'rack_foot': R.rack_foot(p),
        'keyed_dock': R.keyed_dock(p),
        'top_cap': R.top_cap(p),
        'imager_base': I.imager_base(p),
        'imager_light_platform': I.light_platform(p),
        'imager_camera_gantry': I.camera_gantry(p),
        'imaging_dock': I.imaging_dock(p),
        K.carrier_name(p): K.openkit_carrier(p),
        K.carrier_name(p, float(p['dish']['max_base_diameter'])):
            K.openkit_carrier(p, float(p['dish']['max_base_diameter'])),
        'rail_fit_coupon': R.printed_rail(p, 320.0),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, default=ROOT / 'hardware')
    args = ap.parse_args()
    p = P.load(ROOT / 'parameters' / 'openkit.toml')
    parts = kit_parts(p)
    for kind in ('step', 'stl'):
        folder = args.out / kind
        folder.mkdir(parents=True, exist_ok=True)
        for name, shape in parts.items():
            options = {'tolerance': 0.02, 'angularTolerance': 0.15} if kind == 'stl' else {}
            cq.exporters.export(shape, str(folder / f'{name}.{kind}'), **options)
    print(f'Wrote {len(parts)} parts to {args.out}')


if __name__ == '__main__':
    main()
