"""OpenKit hermaphroditic puzzle-snap floor-tile system, millimetres.

Four identical 261 x 261 x 14 mm tiles (285 x 285 mm nominal across puzzle
tabs) snap together into a 522 x 522 mm 2 x 2 workcell floor with zero loose
keys or pins. Each tile carries an asymmetric octagram of underside M5 hex-bolt
pockets (PCD 100 mm about MOUNT_CENTER = (54, -54) mm) so modules (robot stand,
rack1, rack2, imager) drop onto four protruding M5 x 20 mm hex bolts and fasten
from above with M5 washers and hex nuts.

Optional seam clamps: at edge coordinate +-40 mm on every tile edge a Ø5.5 bore,
a split underside hex pocket and a split Ø16 top counterbore straddle the seam
line. An M5 x 10 hex bolt (head in the underside pocket), washer and nut in the
counterbore clamp both neighbouring tiles from below and above. There is no
overlap in plan, so any drop order works and nothing overhangs. Only the 8 clamp
positions on internal seams are used; the 8 half-features on the outer edges are
cosmetic. The snap joint holds without any seam clamp.

Concept CAD only. Joint snap fatigue, print flatness, anchor loads and module
structural capacity are not qualified.
"""
from __future__ import annotations

import math

import cadquery as cq

from cad.candidate import parts as P

# ---------------------------------------------------------------- tile envelope
TILE_SIZE = 261.0
TILE_X = TILE_SIZE
TILE_Y = TILE_SIZE
TILE_T = 14.0
CORNER_R = 4.0

# ------------------------------------------------------------ puzzle & snap joint
TAB_OFFSET = 65.0          # +65.0 male, -65.0 female in CCW edge coordinate
TAB_PROJ = 12.0            # outward projection (285.0 x 285.0 mm nominal envelope)
TAB_NECK_W = 14.0
TAB_NECK_L = 3.0
TAB_HEAD_W = 22.0
TAB_FLARE_L = 4.0
TAB_TIP_R = 2.0
TAB_ROOT_R = 1.5
PUZZLE_CLEARANCE = 0.25    # per-side lateral clearance (valid range 0.15..0.50 mm)

SNAP_BACK_CLEARANCE = 0.65
SNAP_PANEL_T = 1.20
SNAP_SLOT_L = 26.0
SNAP_SLOT_W = 1.00

# ---------------------------------------------------------- mounting & anchors
MOUNT_CENTER = (54.0, -54.0)
FLOOR_POST_PCD = 100.0
POST_DIAMETER = 5.5
HEX_POCKET_AF = 8.40
HEX_POCKET_DEPTH = 6.2     # 7.8 mm web above the post pocket; post bolt tip 12.2 mm above the top
ANCHOR_INSET = 16.0
ANCHOR_DIAMETER = 5.5

# ------------------------------------------------------------------ seam clamps
SEAM_BOLT_OFFSET = 40.0    # edge coordinate +-40 on every edge (matches the neighbour under any rotation)
SEAM_HEX_DEPTH = 4.2       # underside hex pocket depth (head bearing face z)
SEAM_CB_D = 16.0           # top counterbore diameter (washer + nut)
SEAM_CB_DEPTH = 6.0
SEAM_CB_CHAMFER = 0.5

_DEFAULT_PARAMS = P.ROOT / "parameters" / "openkit.toml"


def _resolve_params(p=None):
    return P.load(_DEFAULT_PARAMS) if p is None else p


def post_points(mode="all", center=MOUNT_CENTER):
    """Return M5 post XY centres on the PCD 100 mm octagram."""
    modes = {
        "cardinal": (0, 90, 180, 270),
        "diagonal": (45, 135, 225, 315),
        "all": (0, 45, 90, 135, 180, 225, 270, 315),
    }
    if mode not in modes:
        raise ValueError(f"mode must be one of {tuple(modes)}, got {mode!r}")
    r = FLOOR_POST_PCD / 2.0
    cx, cy = center
    return [
        (cx + r * math.cos(math.radians(ang)), cy + r * math.sin(math.radians(ang)))
        for ang in modes[mode]
    ]


def anchor_points():
    """Return the four corner tabletop anchor bore XY centres."""
    off = TILE_SIZE / 2.0 - ANCHOR_INSET
    return [(sx * off, sy * off) for sx in (-1, 1) for sy in (-1, 1)]


# ------------------------------------------------------------------------- tile

def _male_tab_solid():
    """Male puzzle tab template on the +Y edge (x = -TAB_OFFSET, y = H .. H + TAB_PROJ)."""
    h = TILE_SIZE / 2.0
    xc = -TAB_OFFSET
    hw_n = TAB_NECK_W / 2.0
    hw_h = TAB_HEAD_W / 2.0
    pts = [
        (xc - hw_n - TAB_ROOT_R - 1.0, h - 1.0),
        (xc + hw_n + TAB_ROOT_R + 1.0, h - 1.0),
        (xc + hw_n + TAB_ROOT_R + 1.0, h),
        (xc + hw_n, h),
        (xc + hw_n, h + TAB_NECK_L),
        (xc + hw_h, h + TAB_NECK_L + TAB_FLARE_L),
        (xc + hw_h, h + TAB_PROJ),
        (xc - hw_h, h + TAB_PROJ),
        (xc - hw_h, h + TAB_NECK_L + TAB_FLARE_L),
        (xc - hw_n, h + TAB_NECK_L),
        (xc - hw_n, h),
        (xc - hw_n - TAB_ROOT_R - 1.0, h),
    ]
    tab = cq.Workplane("XY").polyline(pts).close().extrude(TILE_T)
    tab = tab.edges("|Z").edges(
        cq.selectors.BoxSelector((xc - hw_n - 0.1, h - 0.1, -1.0), (xc - hw_n + 0.1, h + 0.1, TILE_T + 1.0))
    ).fillet(TAB_ROOT_R)
    tab = tab.edges("|Z").edges(
        cq.selectors.BoxSelector((xc + hw_n - 0.1, h - 0.1, -1.0), (xc + hw_n + 0.1, h + 0.1, TILE_T + 1.0))
    ).fillet(TAB_ROOT_R)
    tab = tab.edges("|Z").edges(
        cq.selectors.BoxSelector((xc - hw_h - 0.1, h + TAB_PROJ - 0.1, -1.0), (xc - hw_h + 0.1, h + TAB_PROJ + 0.1, TILE_T + 1.0))
    ).fillet(TAB_TIP_R)
    tab = tab.edges("|Z").edges(
        cq.selectors.BoxSelector((xc + hw_h - 0.1, h + TAB_PROJ - 0.1, -1.0), (xc + hw_h + 0.1, h + TAB_PROJ + 0.1, TILE_T + 1.0))
    ).fillet(TAB_TIP_R)
    return tab


def _male_detent_ridge():
    """Male tip detent ridge on y = H + TAB_PROJ, width 10.0 mm, +0.55 mm outward."""
    y_face = TILE_SIZE / 2.0 + TAB_PROJ
    pts = [
        (y_face - 0.5, 1.90),
        (y_face, 1.90),
        (y_face + 0.55, 2.85),
        (y_face + 0.55, 3.15),
        (y_face, 3.70),
        (y_face - 0.5, 3.70),
    ]
    return cq.Workplane("YZ", origin=(-TAB_OFFSET - 5.0, 0.0, 0.0)).polyline(pts).close().extrude(10.0)


def _female_pocket_cutter(clearance):
    """Female pocket void template on the +Y edge (x = +TAB_OFFSET, inward to H - d_back)."""
    h = TILE_SIZE / 2.0
    xc = TAB_OFFSET
    hw_n = TAB_NECK_W / 2.0 + clearance
    hw_h = TAB_HEAD_W / 2.0 + clearance
    d_back = TAB_PROJ + SNAP_BACK_CLEARANCE
    r_tip = TAB_TIP_R + clearance
    pts = [
        (xc - hw_n - TAB_ROOT_R - 1.0, h + 1.0),
        (xc + hw_n + TAB_ROOT_R + 1.0, h + 1.0),
        (xc + hw_n + TAB_ROOT_R + 1.0, h),
        (xc + hw_n, h),
        (xc + hw_n, h - TAB_NECK_L),
        (xc + hw_h, h - TAB_NECK_L - TAB_FLARE_L),
        (xc + hw_h, h - d_back),
        (xc - hw_h, h - d_back),
        (xc - hw_h, h - TAB_NECK_L - TAB_FLARE_L),
        (xc - hw_n, h - TAB_NECK_L),
        (xc - hw_n, h),
        (xc - hw_n - TAB_ROOT_R - 1.0, h),
    ]
    pkt = cq.Workplane("XY", origin=(0.0, 0.0, -0.1)).polyline(pts).close().extrude(TILE_T + 0.2)
    pkt = pkt.edges("|Z").edges(
        cq.selectors.BoxSelector((xc - hw_n - 0.1, h - 0.1, -1.0), (xc - hw_n + 0.1, h + 0.1, TILE_T + 1.0))
    ).fillet(TAB_ROOT_R)
    pkt = pkt.edges("|Z").edges(
        cq.selectors.BoxSelector((xc + hw_n - 0.1, h - 0.1, -1.0), (xc + hw_n + 0.1, h + 0.1, TILE_T + 1.0))
    ).fillet(TAB_ROOT_R)
    pkt = pkt.edges("|Z").edges(
        cq.selectors.BoxSelector((xc - hw_h - 0.1, h - d_back - 0.1, -1.0), (xc - hw_h + 0.1, h - d_back + 0.1, TILE_T + 1.0))
    ).fillet(r_tip)
    pkt = pkt.edges("|Z").edges(
        cq.selectors.BoxSelector((xc + hw_h - 0.1, h - d_back - 0.1, -1.0), (xc + hw_h + 0.1, h - d_back + 0.1, TILE_T + 1.0))
    ).fillet(r_tip)
    return pkt


def _female_detent_ridge():
    """Female spring-wall detent ridge on back wall (y = H - 12.65), +0.55 mm toward mouth."""
    y_back = TILE_SIZE / 2.0 - (TAB_PROJ + SNAP_BACK_CLEARANCE)
    pts = [
        (y_back - 0.5, 4.20),
        (y_back, 4.20),
        (y_back + 0.55, 4.75),
        (y_back + 0.55, 5.05),
        (y_back, 6.00),
        (y_back - 0.5, 6.00),
    ]
    return cq.Workplane("YZ", origin=(TAB_OFFSET - 5.0, 0.0, 0.0)).polyline(pts).close().extrude(10.0)


def _spring_relief_slot():
    """Vertical through relief slot behind the 1.20 mm female spring wall."""
    y_slot = TILE_SIZE / 2.0 - (TAB_PROJ + SNAP_BACK_CLEARANCE + SNAP_PANEL_T + SNAP_SLOT_W / 2.0)
    return (
        cq.Workplane("XY", origin=(TAB_OFFSET, y_slot, -0.1))
        .slot2D(SNAP_SLOT_L, SNAP_SLOT_W)
        .extrude(TILE_T + 0.2)
    )


def _seam_clamp_cutters():
    """Cutters for the two seam clamps on the +Y edge (x = +-SEAM_BOLT_OFFSET), centred on y = H.

    Each cutter is a full feature (bore, hex pocket with elephant-foot lead-in,
    counterbore with top chamfer); only the half inside the tile removes
    material, the other half is the neighbour's. Hex vertices lie along the edge
    so facing edges of two tiles agree in any rotation.
    """
    h = TILE_SIZE / 2.0
    out = None
    for x in (-SEAM_BOLT_OFFSET, SEAM_BOLT_OFFSET):
        bore = P.cyl(POST_DIAMETER, -0.5, TILE_T + 0.5, x, h)
        pocket = (
            cq.Workplane("XY", origin=(x, h, -0.1))
            .polygon(6, HEX_POCKET_AF, circumscribed=True)
            .extrude(SEAM_HEX_DEPTH + 0.1)
        )
        foot = (
            cq.Workplane("XY", origin=(x, h, -0.1))
            .polygon(6, HEX_POCKET_AF + 1.0, circumscribed=True)
            .workplane(offset=0.5)
            .polygon(6, HEX_POCKET_AF, circumscribed=True)
            .loft()
        )
        cb = (
            cq.Workplane("XY", origin=(x, h, TILE_T - SEAM_CB_DEPTH))
            .circle(SEAM_CB_D / 2.0)
            .extrude(SEAM_CB_DEPTH + 0.1)
        )
        cham = (
            cq.Workplane("XY", origin=(x, h, TILE_T - SEAM_CB_CHAMFER))
            .circle(SEAM_CB_D / 2.0)
            .workplane(offset=SEAM_CB_CHAMFER + 0.1)
            .circle(SEAM_CB_D / 2.0 + SEAM_CB_CHAMFER + 0.1)
            .loft()
        )
        cut = bore.union(pocket).union(foot).union(cb).union(cham)
        out = cut if out is None else out.union(cut)
    return out


def floor_tile(clearance=PUZZLE_CLEARANCE):
    """One 261 x 261 x 14 mm hermaphroditic puzzle-snap tile with underside M5 hex pockets."""
    if not 0.15 <= clearance <= 0.50:
        raise ValueError("clearance must be between 0.15 and 0.50 mm")

    h = TILE_SIZE / 2.0
    d_back = TAB_PROJ + SNAP_BACK_CLEARANCE
    tile = cq.Workplane("XY").box(TILE_SIZE, TILE_SIZE, TILE_T, centered=(True, True, False))
    tile = tile.edges("|Z").fillet(CORNER_R)

    # 1. Male puzzle tabs on all 4 edges + 0.8 mm bottom lead-in chamfer + detent ridges.
    tab = _male_tab_solid()
    for ang in (0.0, 90.0, 180.0, 270.0):
        tile = tile.union(tab.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), ang))

    tab_bot_edges = [
        e for e in tile.edges("<Z").vals()
        if max(abs(e.Center().x), abs(e.Center().y)) > h + 0.001
    ]
    tile = tile.newObject(tab_bot_edges).chamfer(0.8).clean()

    m_ridge = _male_detent_ridge()
    for ang in (0.0, 90.0, 180.0, 270.0):
        tile = tile.union(m_ridge.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), ang))
    tile = tile.clean()

    # 2. Female pockets + mouth chamfers (0.8 top / 0.4 bottom) + spring ridges + relief slots.
    pkt = _female_pocket_cutter(clearance)
    for ang in (0.0, 90.0, 180.0, 270.0):
        tile = tile.cut(pkt.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), ang))

    def _is_pocket_edge(e):
        c = e.Center()
        return (
            (abs(abs(c.x) - TAB_OFFSET) < 13.0 and h - d_back - 0.1 < abs(c.y) < h - 0.001)
            or (abs(abs(c.y) - TAB_OFFSET) < 13.0 and h - d_back - 0.1 < abs(c.x) < h - 0.001)
        )

    pkt_top_edges = [e for e in tile.edges(">Z").vals() if _is_pocket_edge(e)]
    pkt_bot_edges = [e for e in tile.edges("<Z").vals() if _is_pocket_edge(e)]
    tile = tile.newObject(pkt_top_edges).chamfer(0.8).clean()
    tile = tile.newObject(pkt_bot_edges).chamfer(0.4).clean()

    f_ridge = _female_detent_ridge()
    slot = _spring_relief_slot()
    for ang in (0.0, 90.0, 180.0, 270.0):
        tile = tile.union(f_ridge.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), ang))
        tile = tile.cut(slot.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), ang))

    # 3. 8x octagram M5 through-bores + underside hex pockets + 0.4 mm elephant-foot relief.
    for x, y in post_points("all"):
        tile = tile.cut(P.cyl(POST_DIAMETER, -0.5, TILE_T + 0.5, x, y))
        pocket = (
            cq.Workplane("XY", origin=(x, y, 0.0))
            .polygon(6, HEX_POCKET_AF, circumscribed=True)
            .extrude(HEX_POCKET_DEPTH)
        )
        tile = tile.cut(pocket)
        frustum = (
            cq.Workplane("XY", origin=(x, y, -0.1))
            .polygon(6, HEX_POCKET_AF + 1.0, circumscribed=True)
            .workplane(offset=0.5)
            .polygon(6, HEX_POCKET_AF, circumscribed=True)
            .loft()
        )
        tile = tile.cut(frustum)

    # 3b. Seam-clamp half-features on every edge (split across the edge line).
    seam = _seam_clamp_cutters()
    for ang in (0.0, 90.0, 180.0, 270.0):
        tile = tile.cut(seam.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), ang))

    # 4. 4x corner tabletop anchor bores.
    for x, y in anchor_points():
        tile = tile.cut(P.cyl(ANCHOR_DIAMETER, -0.5, TILE_T + 0.5, x, y))

    # 5. Subtle orientation arrow on top near the +X/+Y corner.
    mark = (
        cq.Workplane("XY", origin=(h - 30.0, h - 30.0, TILE_T - 0.6))
        .polyline([(-6.0, -4.0), (6.0, -4.0), (0.0, 5.0)])
        .close()
        .extrude(0.7)
    )
    tile = tile.cut(mark)

    return P.solid(tile.clean())


# --------------------------------------------------------------------- hardware

def m5_hex_bolt(p=None, length=None):
    """ISO 4017 M5 hex bolt (origin at head bearing face z = 0.0).

    ``length`` defaults to ``hardware.floor_bolt_length`` (M5 x 20, module posts);
    the seam clamps pass ``hardware.seam_bolt_length`` (M5 x 10).
    """
    hw = _resolve_params(p)["hardware"]
    af = float(hw["floor_bolt_hex_af"])
    head_h = float(hw["floor_bolt_head_height"])
    length = float(hw["floor_bolt_length"]) if length is None else float(length)
    head = (
        cq.Workplane("XY", origin=(0.0, 0.0, -head_h))
        .polygon(6, af, circumscribed=True)
        .extrude(head_h)
    )
    shank = P.cyl(4.9, -0.1, length)
    return P.solid(head.union(shank).clean())


def m5_washer(p=None):
    """ISO 7089 M5 flat washer (z = 0.0 .. floor_washer_thickness)."""
    hw = _resolve_params(p)["hardware"]
    od = float(hw["floor_washer_od"])
    id_ = float(hw["floor_washer_id"])
    th = float(hw["floor_washer_thickness"])
    washer = cq.Workplane("XY").circle(od / 2.0).circle(id_ / 2.0).extrude(th)
    return P.solid(washer.clean())


def m5_hex_nut(p=None):
    """ISO 4032 M5 hex nut (z = 0.0 .. floor_nut_height)."""
    hw = _resolve_params(p)["hardware"]
    af = float(hw["floor_nut_hex_af"])
    nut_h = float(hw["floor_nut_height"])
    nut = (
        cq.Workplane("XY")
        .polygon(6, af, circumscribed=True)
        .circle(5.1 / 2.0)
        .extrude(nut_h)
    )
    return P.solid(nut.clean())


def post_hardware_set(p=None, flange_thickness=5.0):
    """Return {bolt, washer, nut} positioned in the tile-Z frame for one post."""
    p = _resolve_params(p)
    washer_t = float(p["hardware"]["floor_washer_thickness"])
    z_washer = TILE_T + float(flange_thickness)
    return {
        "bolt": m5_hex_bolt(p).translate((0.0, 0.0, HEX_POCKET_DEPTH)),
        "washer": m5_washer(p).translate((0.0, 0.0, z_washer)),
        "nut": m5_hex_nut(p).translate((0.0, 0.0, z_washer + washer_t)),
    }


def seam_hardware_set(p=None):
    """Return {bolt, washer, nut} for one seam clamp at the origin (tile-Z frame).

    Bolt head bearing face at z = SEAM_HEX_DEPTH, washer at TILE_T - SEAM_CB_DEPTH,
    nut on the washer. Hex vertices lie along world X (seams on y = const lines).
    """
    p = _resolve_params(p)
    hw = p["hardware"]
    washer_t = float(hw["floor_washer_thickness"])
    z_washer = TILE_T - SEAM_CB_DEPTH
    return {
        "bolt": m5_hex_bolt(p, length=float(hw["seam_bolt_length"])).translate((0.0, 0.0, SEAM_HEX_DEPTH)),
        "washer": m5_washer(p).translate((0.0, 0.0, z_washer)),
        "nut": m5_hex_nut(p).translate((0.0, 0.0, z_washer + washer_t)),
    }


# ----------------------------------------------------------- layout & assembly

def tile_placements():
    """Return {tile_name: (cx, cy, rot_deg)} for the 2 x 2 floor."""
    h = TILE_SIZE / 2.0
    return {
        "front_right": (h, -h, 0.0),
        "front_left": (-h, -h, -90.0),
        "back_left": (-h, h, 0.0),
        "back_right": (h, h, 90.0),
    }


def _tile_mount_xy(tile_name):
    cx, cy, rot_deg = tile_placements()[tile_name]
    rad = math.radians(rot_deg)
    lx, ly = MOUNT_CENTER
    return (
        cx + lx * math.cos(rad) - ly * math.sin(rad),
        cy + lx * math.sin(rad) + ly * math.cos(rad),
    )


def seam_points():
    """Return the 8 internal seam-clamp XY centres in the world frame.

    Two per seam at edge coordinate +-SEAM_BOLT_OFFSET about each tile-edge centre.
    """
    h = TILE_SIZE / 2.0
    o = SEAM_BOLT_OFFSET
    pts = []
    for s in (-1.0, 1.0):
        pts += [(0.0, s * h - o), (0.0, s * h + o)]   # seams on the x = 0 line
    for s in (-1.0, 1.0):
        pts += [(s * h - o, 0.0), (s * h + o, 0.0)]   # seams on the y = 0 line
    return pts


def seam_hardware(p=None):
    """Return {seam{i}_bolt, seam{i}_washer, seam{i}_nut} for the 8 seam clamps.

    Bolt hex orientation follows the pocket (vertices along the seam edge).
    """
    p = _resolve_params(p)
    hw_set = seam_hardware_set(p)
    out = {}
    for i, (x, y) in enumerate(seam_points()):
        ang = 90.0 if abs(x) < 1e-6 else 0.0
        for kind, solid in hw_set.items():
            out[f"seam{i}_{kind}"] = (
                solid.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), ang).translate((x, y, 0.0))
            )
    return out


def module_placements(robot_rot_deg=45.0):
    """Return {module: (mx, my, rot_deg, post_mode)} in the 2 x 2 floor frame."""
    robot_mode = "diagonal" if abs((robot_rot_deg % 90.0) - 45.0) < 1e-6 else "cardinal"
    specs = {
        "robot": ("front_right", float(robot_rot_deg), robot_mode),
        "rack1": ("front_left", 90.0, "diagonal"),
        "rack2": ("back_left", 45.0, "cardinal"),
        "imager": ("back_right", 0.0, "diagonal"),
    }
    out = {}
    for mod, (tile_name, rot_deg, post_mode) in specs.items():
        mx, my = _tile_mount_xy(tile_name)
        out[mod] = (mx, my, rot_deg, post_mode)
    return out


def floor_layout(clearance=PUZZLE_CLEARANCE):
    """Four identical hermaphroditic puzzle tiles snapped into a 2 x 2 floor."""
    base = floor_tile(clearance)
    return {
        name: base.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), rot).translate((cx, cy, 0.0))
        for name, (cx, cy, rot) in tile_placements().items()
    }


def floor_hardware(p=None, robot_rot_deg=45.0, flange_thickness=5.0):
    """Return all 48 M5 bolts, washers, and nuts at every module active post."""
    p = _resolve_params(p)
    hw_set = post_hardware_set(p, flange_thickness=flange_thickness)
    tiles = tile_placements()
    mod_to_tile = {
        "robot": "front_right",
        "rack1": "front_left",
        "rack2": "back_left",
        "imager": "back_right",
    }
    out = {}
    for mod, (mx, my, _rot_deg, post_mode) in module_placements(robot_rot_deg).items():
        tile_rot = tiles[mod_to_tile[mod]][2]
        for i, (px, py) in enumerate(post_points(post_mode, center=(mx, my))):
            for kind, solid in hw_set.items():
                out[f"{mod}_post{i}_{kind}"] = (
                    solid.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), tile_rot)
                    .translate((px, py, 0.0))
                )
    return out


def workcell_floor_assembly(p=None, rise_mm=55.0, robot_rot_deg=45.0):
    """Assemble the 2 x 2 puzzle floor, module feet/stand/imager, 48 post fasteners and 8 optional seam clamps."""
    from cad import imager as I
    from cad import rail as R
    from cad import robot_stand as RS

    p = _resolve_params(p)
    parts = dict(floor_layout())
    mods = module_placements(robot_rot_deg=robot_rot_deg)

    rx, ry, rrot, _ = mods["robot"]
    parts["robot_stand"] = (
        RS.robot_stand(rise_mm)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), rrot)
        .translate((rx, ry, TILE_T))
    )

    for name in ("rack1", "rack2"):
        mx, my, mrot, _ = mods[name]
        parts[f"{name}_foot"] = (
            R.rack_foot(p)
            .translate((0.0, -10.0, TILE_T + 5.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), mrot)
            .translate((mx, my, 0.0))
        )

    ix, iy, irot, _ = mods["imager"]
    station = I.imager_station(p)
    for key in ("base", "spine", "dock", "light_platform", "camera_gantry"):
        parts[f"imager_{key}"] = (
            station[key]
            .translate((0.0, -I.post_center_y(p), TILE_T + I.dock_height(p)))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), irot)
            .translate((ix, iy, 0.0))
        )

    parts.update(seam_hardware(p))
    parts.update(floor_hardware(p, robot_rot_deg=robot_rot_deg))
    return parts


def floor_envelope():
    """Overall 2 x 2 floor and single-tile envelope dimensions in mm."""
    return {
        "x": 2.0 * TILE_SIZE,
        "y": 2.0 * TILE_SIZE,
        "thickness": TILE_T,
        "tile_bbox_x": TILE_SIZE + 2.0 * TAB_PROJ,
        "tile_bbox_y": TILE_SIZE + 2.0 * TAB_PROJ,
    }
