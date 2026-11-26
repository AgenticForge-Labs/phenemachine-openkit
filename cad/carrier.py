"""OpenKit round dish carrier.

The compact clipped-corner (hexagonal) plate with the four underside grooves at
+/-dock_spacing/2 and +/-robot_spacing/2 and four finger scallops. The dish base
rests on a 1.5 mm radial lip around a round window that is open from below, so a
backlight shows most of the base. The outer stays 140 x 100 mm (sized from
`dish.max_base_diameter`); each round dish family is named by its base diameter
and keeps the 1.5 mm lip.

Concept CAD in the dock datum; fit, strength and print flatness are not qualified.
"""
from __future__ import annotations

import cadquery as cq

from cad.candidate import parts as P


def carrier_name(p, base_diameter=None):
    """Canonical export name for a round carrier based on its base diameter (mm)."""
    base = P.dish_diameter(p) if base_diameter is None else float(base_diameter)
    tag = f'{base:g}'.replace('.', '_')
    return f'round_carrier_{tag}mm'


def nest_and_window(p, base_diameter=None):
    """Nest and window diameters for a round base (default: the configured kit dish)."""
    c = p['carrier']
    base = P.dish_diameter(p) if base_diameter is None else float(base_diameter)
    nest = base + 2 * c['fit_clearance']
    lip = c.get('openkit_lip', 1.5)
    return nest, nest - 2 * lip


def max_nest(p):
    """Maximum circular nest diameter that fits inside the shared carrier outer."""
    c, d = p['carrier'], p['dish']
    length = c.get('openkit_length')
    if length is not None:
        return length - 2 * c.get('openkit_rim_wall', c['wall'])
    max_base = float(d.get('max_base_diameter', P.dish_diameter(p)))
    return max_base + 2 * max(c['fit_clearance'], 0.5)


def outer_size(p):
    """Carrier (width, length): sized from the shared OpenKit max envelope."""
    c = p['carrier']
    width = c.get('openkit_width') or P.carrier_outline(p)[0]
    length = c.get('openkit_length') or (max_nest(p) + 2 * c.get('openkit_rim_wall', c['wall']))
    return width, length


def openkit_carrier(p, base_diameter=None):
    """Round carrier; `base_diameter` re-sizes only the nest and window while keeping a 1.5 mm lip."""
    i, c, rib = p['interface'], p['carrier'], p['rib']
    width, length = outer_size(p)
    nest, window = nest_and_window(p, base_diameter)
    if nest > max_nest(p) + 1e-9:
        raise ValueError('dish base larger than the carrier is sized for')
    if window <= 0:
        raise ValueError('dish too small for a window')
    T, sz = c['thickness'], c['seat_z']
    x, y = width / 2, length / 2
    a, b = c['corner_cut_x'], c['corner_cut_y']
    points = [(-x + a, -y), (x - a, -y), (x, -y + b), (x, y - b),
              (x - a, y), (-x + a, y), (-x, y - b), (-x, -y + b)]

    body = cq.Workplane('XY').polyline(points).close().extrude(T)
    body = body.edges('|Z').fillet(c['corner_radius'])
    body = body.faces('<Z').chamfer(c['edge_chamfer'])

    # Backlight window (through) and the lip recess above the seat.
    body = body.cut(P.cyl(window, -1, T + 1, 0, 0))
    body = body.cut(P.cyl(nest, sz - 0.01, T + 1, 0, 0))

    # Four finger scallops on the front/back rim.
    for sx in (-1, 1):
        for sy in (-1, 1):
            body = body.cut(P.rbox(c['grip_width'], c['wall'] + 6.0, sz + c.get('grip_floor', 0),
                                   T + 1, 0, (sx * c['grip_x'], sy * (length / 2 - c['wall'] / 2))))

    # Four underside locating grooves.
    for spacing in (i['dock_spacing'], i['robot_spacing']):
        for sign in (-1, 1):
            body = body.cut(P.groove(rib, sign * spacing / 2))
    return P.solid(body.clean())
