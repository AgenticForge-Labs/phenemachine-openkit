"""OpenKit imaging station: floor base, T-slot spine, keyed dock, light platform, camera gantry.

Built in the dock datum (origin on the carrier underside support plane at the
seated carrier centre, +Y away from the robot, +Z up). The station is a
generic "light under the dish, camera above the dish" stand:

* imager_base      - four-sided rack foot plus a flat front plate. Bolts to four
                     floor posts (diagonal pattern about (0, post_center_y)) with
                     M5 washers and nuts. Prints flat.
* spine            - the printed 20 x 40 T-slot rail (rail.rail_section).
* dock             - the keyed shelf the carrier seats on (rail.keyed_dock).
* light_platform   - separate drop-in plastic platform that holds any flat light
                     panel (up to 99 x 99 mm) 22 mm under the carrier datum. It is
                     installed after the base nuts are tightened, slid in under
                     the dock (lifted over the 3 mm fence); the dock blocks a
                     vertical drop.
* camera_gantry    - sleeve over the spine top bolted to the two front T-nuts, a
                     boom reaching over the dish and the existing board-camera head
                     (optical centre (0, 0, 165)). Prints upside down.

The light panel and the camera are kept generic on purpose. Covers and light
shielding are out of scope for now; the base front plate and the gantry sleeve
leave room for a future clip-on shroud.

Concept CAD. Camera, illumination, reflections, printability and the physical
light are not qualified.
"""
from __future__ import annotations

import cadquery as cq

from cad import floor as F
from cad import rail as R
from cad.candidate import parts as P

DEFAULT_DOCK_Z = 177.5     # imaging-dock datum above the floor top, mm (shelf level 4)
BASE_T = 5.0               # base plate thickness, same as the rack foot
POST_ROW_OFFSET = 10.0     # rack_foot post pattern is centred 10 mm behind the rail front face
CAMERA_ARM_SKIN = 3.0      # roof over the spine top

# Light platform (all in the dock datum).
LIGHT_COL = 88.0           # outer column width/depth
LIGHT_WALL = 3.2
LIGHT_FLARE_Z0 = -42.0     # column top / flare start
LIGHT_FLARE_Z1 = -33.0     # flare end (45 deg: +-44 -> +-53)
LIGHT_RING_TOP = -30.0     # light panel seats here
LIGHT_RIM_TOP = -26.0
LIGHT_OUTER_TOP = 106.0    # flare/ring/rim outer width
LIGHT_RIM_IN = 99.0        # light pocket inside the rim (98 mm panel + 0.5 per side)
FENCE_INNER = 44.3         # base fence inner half-width (0.3 clearance round the column)


def dock_height(p):
    """Imaging dock datum height above the floor top (mm), from the TOML."""
    return float(p['imager'].get('dock_height_above_floor', DEFAULT_DOCK_Z))


def post_center_y(p):
    """Dock-frame Y of the centre of the four-post pattern."""
    return R.rail_front_y(p) + POST_ROW_OFFSET


def _dz(p, dock_z):
    return dock_height(p) if dock_z is None else float(dock_z)


def spine_top_z(p):
    """Spine top: the camera arm underside is 3 mm above it (the roof of the sleeve)."""
    im = p['imager']
    return im['camera_height_above_datum'] + im['arm_thickness_z'] - CAMERA_ARM_SKIN


def spine_bottom_z(p, dock_z=None):
    return -_dz(p, dock_z) + BASE_T


def imager_spine(p, dock_z=None):
    """Plain T-slot rail (front face at y=0) from the base top to the sleeve roof."""
    return R.rail_section(p, spine_bottom_z(p, dock_z), spine_top_z(p), hollow=True)


def imaging_dock(p):
    """Keyed shelf dock at the dock datum (z=0); the dish/carrier seats here."""
    return R.keyed_dock(p)


# ---------------------------------------------------------------------- base

def imager_base(p, dock_z=None):
    """Rack foot + 112 x 114 mm front plate, with a locating fence for the light platform.

    The rack foot supplies the collar, four collar T-nut bolts and the diagonal post
    holes about (0, post_center_y). The two front posts (y = post_center_y - 35.4)
    stand in the front plate, so the light platform column stays clear of their
    nuts. The fence (2 x 3 mm, inner edge +-44.3) locates the light platform on the
    front and both sides; at the rear only |x| <= 20 is fenced so an 8 mm socket
    still reaches the front post nuts. Prints flat, plate down.
    """
    dz = _dz(p, dock_z)
    face = R.rail_front_y(p)
    z0 = -dz
    z1 = z0 + BASE_T
    base = cq.Workplane('XY').add(R.rack_foot(p)).translate((0, face, z1))
    plate = P.rbox(112.0, 114.0, z0, z1, 12.0, (0.0, 3.0))      # y -54 .. 60
    base = base.union(plate)
    # Re-cut the front post holes (the rack foot cut all four; the front plate refills the front pair).
    pcy = post_center_y(p)
    r = F.FLOOR_POST_PCD / 2.0 * 0.70710678118654752
    for sx in (-1, 1):
        base = base.cut(P.cyl(F.POST_DIAMETER, z0 - 1, z1 + 1, sx * r, pcy - r))
    # Light platform fence.
    t, h, a = 2.0, 3.0, FENCE_INNER
    fy = -a - t / 2
    base = base.union(P.rbox(2 * (a + t), t, z1, z1 + h, 0.0, (0, fy)))
    for sx in (-1, 1):
        base = base.union(P.rbox(t, 2 * a, z1, z1 + h, 0.0, (sx * (a + t / 2), 0.0)))
    base = base.union(P.rbox(40.0, t, z1, z1 + h, 0.0, (0, a + t / 2)))
    return P.solid(base.clean())


# ------------------------------------------------------------ light platform

def _rr_sketch(w, r):
    return cq.Sketch().rect(w, w).vertices().fillet(r)


def _loft(w0, w1, r, z0, z1):
    return (cq.Workplane('XY', origin=(0, 0, z0)).placeSketch(
        _rr_sketch(w0, r), _rr_sketch(w1, r).moved(cq.Location(cq.Vector(0, 0, z1 - z0)))).loft())


def light_platform(p, dock_z=None):
    """Drop-in platform holding a generic flat light panel under the dish.

    Hollow 88 x 88 column from the base top to z -42, a 45 degree flare to +-53 mm,
    a 3 mm ring the panel rests on (centre open for the cable) and a 4 mm rim
    locating the panel (24 mm cable notch at the rear centre, 30 x 15 slot in the
    rear wall at the bottom). Open at the bottom; after the base nuts are tightened
    it slides in under the dock from the front or a side (lifted >= 3.2 mm over the
    fence) and drops into the fence. A vertical drop from above is blocked by the dock. Prints upright. Panel: any flat light up to
    99 x 99 mm whose top is 22 mm under the carrier datum.
    """
    z_bot = -_dz(p, dock_z) + BASE_T
    zf0, zf1, zr, zt = LIGHT_FLARE_Z0, LIGHT_FLARE_Z1, LIGHT_RING_TOP, LIGHT_RIM_TOP
    half_in = LIGHT_COL / 2 - LIGHT_WALL
    body = _loft(LIGHT_COL, LIGHT_COL, 6.0, z_bot, zf0)
    body = body.union(_loft(LIGHT_COL, LIGHT_OUTER_TOP, 6.0, zf0, zf1))
    body = body.union(P.rbox(LIGHT_OUTER_TOP, LIGHT_OUTER_TOP, zf1 - 0.01, zr, 6.0))
    # Cavity: straight, then flaring under the ring; centre opening through the ring.
    cav = _loft(2 * half_in, 2 * half_in, 2.8, z_bot - 1.0, zf0)
    cav = cav.union(_loft(2 * half_in, 97.0, 2.8, zf0, zf1))
    body = body.cut(cav)
    body = body.cut(P.rbox(2 * half_in, 2 * half_in, zf1 - 1.0, zr + 1.0, 2.8))
    # Rim round the panel.
    rim = P.rbox(LIGHT_OUTER_TOP, LIGHT_OUTER_TOP, zr - 0.01, zt, 6.0)
    rim = rim.cut(P.rbox(LIGHT_RIM_IN + 0.5, LIGHT_RIM_IN + 0.5, zr - 1.0, zt + 1.0, 2.5))
    rim = rim.cut(P.rbox(24.0, 8.0, zr - 1.0, zt + 1.0, 0.0, (0, LIGHT_OUTER_TOP / 2 - 2.0)))
    body = body.union(rim)
    # Cable slot through the rear wall at the bottom.
    body = body.cut(P.rbox(30.0, 8.0, z_bot - 1.0, z_bot + 15.0, 0.0, (0, LIGHT_COL / 2 - 1.5)))
    return P.solid(body.clean())


def backlight_block(p):
    """Printable stand-in for the rectangular backlight (for fit checks)."""
    im = p['imager']
    size, th = im['backlight_size'], im['backlight_thickness']
    z_top = -im['backlight_below_datum']
    return P.solid(P.rbox(size, size, z_top - th, z_top, 2.0).clean())


# ------------------------------------------------------------ camera gantry

def _camera_head_blank(p):
    """Camera cradle head centred at (0, 0), copied from parts.imager_camera_mount."""
    im = p['imager']
    z_cam, arm_t = im['camera_height_above_datum'], im['arm_thickness_z']
    head_sz = im['board_outer_size'] + 8.0
    return P.rbox(head_sz, head_sz, z_cam - 4.0, z_cam + arm_t, 4.0, (0, 0))


def _camera_head_cutters(p):
    """PCB recess, M12 lens bore and M2.5 hole patterns for the 32/38 mm board cameras."""
    im = p['imager']
    z_cam, arm_t = im['camera_height_above_datum'], im['arm_thickness_z']
    pcb = im['board_outer_size'] + 0.6
    cut = P.rbox(pcb, pcb, z_cam - 5.0, z_cam, 1.0, (0, 0))
    cut = cut.union(P.cyl(im['lens_bore_diameter'], z_cam - 6.0, z_cam + arm_t + 1.0, 0, 0))
    for spacing in (im['hole_spacing_inner'], im['hole_spacing_outer']):
        h = spacing / 2
        for sx in (-1, 1):
            for sy in (-1, 1):
                cut = cut.union(P.cyl(2.6, z_cam - 6.0, z_cam + arm_t + 1.0, sx * h, sy * h))
    return cut


def _camera_head(p):
    """Head with its cuts (private; the parts version is left untouched)."""
    return P.solid(_camera_head_blank(p).cut(_camera_head_cutters(p)).clean())


def camera_gantry(p):
    """Sleeve over the spine top + boom + camera head. Prints upside down (roof on the bed).

    The sleeve wraps the spine top with 0.3 mm clearance (4 sided pocket 40.6 x 20.6)
    and a 3 mm roof. Two recessed M5 bolts through the 7.5 mm front wall clamp it to
    T-nuts in the spine front slots. The 36 mm boom reaches over the dish; an 8 mm
    thick 45 degree gusset braces it to the front wall. The optical centre is
    (0, 0, camera_height_above_datum).
    """
    im, r = p['imager'], p['rail']
    z_cam, arm_t = im['camera_height_above_datum'], im['arm_thickness_z']
    z_roof = z_cam + arm_t
    face, depth = R.rail_front_y(p), r['height']
    pc, side_wall = 0.30, 4.0
    front_wall = R.foot_long_wall(p)
    hx, hy = r['face'] / 2 + pc, depth / 2 + pc
    y_mid = face + depth / 2
    y_out0 = face - pc - front_wall
    y_out1 = face + depth + pc + side_wall
    z_sleeve0 = spine_top_z(p) - 40.0
    sleeve = P.rbox(2 * (hx + side_wall), y_out1 - y_out0, z_sleeve0, z_roof, 2.0, (0.0, (y_out0 + y_out1) / 2))
    sleeve = sleeve.cut(P.rbox(2 * hx, 2 * hy, z_sleeve0 - 1.0, spine_top_z(p), 1.0, (0, y_mid)))

    boom = P.rbox(im['arm_width_x'], 72.0, z_cam, z_roof, 3.0, (0, 36.0))
    g = 8.0
    y_g0 = 57.0
    gusset = (cq.Workplane('YZ', origin=(-g / 2, 0, 0))
              .polyline([(y_g0, z_cam), (y_out0 + 1.0, z_cam), (y_out0 + 1.0, z_cam - (y_out0 + 1.0 - y_g0))])
              .close().extrude(g))
    body = sleeve.union(boom).union(gusset).union(_camera_head_blank(p))
    body = body.cut(_camera_head_cutters(p))
    if p['dock'].get('t_nut_bolt_diameter', 0.0) > 0:
        body = R._cut_front_tnut_bores(body, p, y_out0, face, 152.0)
    return P.solid(body.clean())


def imager_station(p, dock_z=None):
    """Nominal imaging station in the dock datum (base placed with its bottom at -dock_z)."""
    face = R.rail_front_y(p)
    return {
        'base': imager_base(p, dock_z),
        'spine': imager_spine(p, dock_z).translate((0, face, 0)),
        'dock': imaging_dock(p),
        'light_platform': light_platform(p, dock_z),
        'camera_gantry': camera_gantry(p),
    }
