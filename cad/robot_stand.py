"""Standalone drop-on SO-ARM102 robot stand for OpenKit floor tiles.

Drops onto the four protruding M5 posts on a standard floor tile (PCD 100 mm)
and fastens from above with M5 washers and nuts. Local datum: (0, 0) at the
Robot J1 axis, z = 0.0 at the stand bottom sitting flat on the floor tile top,
and z = rise_mm at the robot base mounting surface.

Form: one hollow, sculpted shell printed upside down (robot face on the bed, no
supports). The outer skin is a loft between two same-topology rounded-rectangle
sketches (130 x 130 R50 at the floor, 150 x 128 R24 at rise-6) capped by a 6 mm
top plate, with a 3.2 mm wall, a 2.0 mm 45 deg chamfer on the top (bed) edge and
a small fillet on the open rim. Four hollow towers (OD 25) sit on the cardinal
post bores; each has a 5 mm Ø25 pad with a Ø5.5 bore that bears on the tile and a
Ø17 socket well running from the pad up through the top plate, so a 5/16 in or
8 mm socket reaches the M5 nut (washer z 5..6, nut 6..10.7, bolt tip ~12.2).

Hardware: the RoboNine/SO-ARM102 base has Ø8.2 mm holes on an 80 mm square, much
larger than the M4 screws. Use M4 LARGE washers (ISO 7093-1, OD >= 12 mm) under
the M4 screw heads, or the heads pull through the base holes.
"""
from __future__ import annotations

import cadquery as cq

from cad.candidate import parts as P

ALLOWED_RISES = (30.0, 45.0, 55.0)

# Square M4 robot base mounting hole pattern about the J1 axis.
HOLES = ((-40.0, -40.0), (-40.0, 40.0), (40.0, -40.0), (40.0, 40.0))

# Cardinal M5 floor-post bores on the floor PCD (r = 50). Clocking the stand
# +45 deg lands on diagonal tile posts; 0 deg lands on cardinal tile posts.
POST_HOLES = ((50.0, 0.0), (-50.0, 0.0), (0.0, 50.0), (0.0, -50.0))

BASE_SIZE = 130.0
BASE_R = 50.0
TOP_W, TOP_D, TOP_R = 150.0, 128.0, 24.0
PLATE_T = 6.0
WALL = 3.2
TOWER_OD = 25.0
PAD_T = 5.0
WELL_D = 17.0


def _loft(w0, d0, r0, w1, d1, r1, z0, z1):
    s0 = cq.Sketch().rect(w0, d0).vertices().fillet(r0)
    s1 = cq.Sketch().rect(w1, d1).vertices().fillet(r1).moved(cq.Location(cq.Vector(0, 0, z1 - z0)))
    wp = cq.Workplane("XY", origin=(0, 0, z0))
    return wp.placeSketch(s0, s1).loft(combine=True)


def robot_stand(rise_mm: float = 55.0):
    """Return a hollow sculpted drop-on robot stand at 30, 45, or 55 mm rise.

    55 mm is the default: the 40 mm four-sided rack foot puts shelf 0 at 51.5 mm above the floor, which the
    robot base at 55 mm is chosen to reach.
    """
    from cad import floor as F

    if isinstance(rise_mm, bool) or rise_mm not in ALLOWED_RISES:
        raise ValueError(f"rise_mm must be one of {ALLOWED_RISES}")
    rise_mm = float(rise_mm)
    pb = rise_mm - PLATE_T  # plate underside / loft top
    post_d = F.POST_DIAMETER

    # Outer skin: loft + top plate, chamfered bed edge, softened open rim.
    outer = _loft(BASE_SIZE, BASE_SIZE, BASE_R, TOP_W, TOP_D, TOP_R, 0.0, pb)
    plate = cq.Workplane("XY", origin=(0, 0, pb)).placeSketch(
        cq.Sketch().rect(TOP_W, TOP_D).vertices().fillet(TOP_R)
    ).extrude(PLATE_T)
    outer = outer.union(plate)
    outer = outer.faces(">Z").edges().chamfer(2.0)
    outer = outer.faces("<Z").edges().fillet(1.5)

    # Hollow it: inner loft (bottom open) leaves the wall and the plate.
    t = WALL
    inner = _loft(BASE_SIZE - 2 * t, BASE_SIZE - 2 * t, BASE_R - t,
                  TOP_W - 2 * t, TOP_D - 2 * t, TOP_R - t, 0.0, pb)
    inner = inner.union(
        cq.Workplane("XY", origin=(0, 0, -1.0)).placeSketch(
            cq.Sketch().rect(BASE_SIZE - 2 * t, BASE_SIZE - 2 * t).vertices().fillet(BASE_R - t)
        ).extrude(1.0)
    )
    body = outer.cut(inner)

    # Post towers (pad + tube, merged into the wall and plate).
    for x, y in POST_HOLES:
        body = body.union(P.cyl(TOWER_OD, 0.0, pb + 0.1, x, y))

    # Cuts: Ø17 socket wells (z = pad top .. through plate, 1.0 chamfer on top),
    # Ø5.5 pad bores.
    for x, y in POST_HOLES:
        body = body.cut(P.cyl(WELL_D, PAD_T, rise_mm + 0.1, x, y))
        body = body.cut(P.cyl(post_d, -0.5, PAD_T + 0.5, x, y))
        cone = cq.Solid.makeCone(WELL_D / 2.0, WELL_D / 2.0 + 2.0, 2.0,
                                 cq.Vector(x, y, rise_mm - 1.0))
        body = body.cut(cq.Workplane("XY").add(cone))

    # M4 robot-mount through-bores and underside captured hex nut pockets.
    for x, y in HOLES:
        body = body.cut(P.cyl(4.5, pb - 0.1, rise_mm + 0.1, x, y))
        pocket = (
            cq.Workplane("XY", origin=(x, y, pb - 0.1))
            .polygon(6, 7.4, circumscribed=True)
            .extrude(3.5)
        )
        body = body.cut(pocket)

    return P.solid(body.clean())
