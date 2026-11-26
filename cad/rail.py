"""OpenKit upright 20 x 40 T-slot rail, four-sided foot, and front-bolted shelves.

Built in the dock datum (origin on the carrier underside support plane at the
seated carrier centre, +Y away from the robot). A shelf dock meets the rail
front face at rail_face_y; its recessed front M5 bolts clamp directly into
standard 20-series T-nuts in the two front slots while short lower rear T-keys
and stacking tenons align the column. The four-sided foot collar wraps the
20 x 40 profile on all four sides and bolts to T-nuts on both long (40 mm) faces.

Concept CAD only. Printed-rail creep, bolt preload, key fit and physical
load capacity are not qualified.
"""
from __future__ import annotations

import math

import cadquery as cq

from cad.candidate import parts as P
from cad import floor as F

# Floor-tile mounting: 4-post M5 pattern matching cad.floor.
FOOT_BOLT_ANGLES = (45.0, 135.0, 225.0, 315.0)


def rail_front_y(p):
    return P.rail_face_y(p)


def slot_centres(p):
    half = p['rail']['slot_pitch'] / 2
    return -half, half


def slot_outline(r, margin=1.0):
    """T-slot void as (across, depth) points: opening, straight, 45deg to floor."""
    o, c, f = r['slot_opening'] / 2, r['slot_cavity_width'] / 2, r['slot_floor_width'] / 2
    lip = r['lip_thickness']
    knee = lip + r['slot_shoulder']
    floor = r['slot_depth']
    right = [(o, -margin), (o, lip), (c, lip), (c, knee), (f, floor)]
    return right + [(-u, d) for u, d in reversed(right)]


def _prism(points, z0, z1):
    return cq.Workplane('XY', origin=(0, 0, z0)).polyline(points).close().extrude(z1 - z0)


def rail_section(p, z0, z1, hollow=True):
    """One 20 x 40 T-slot section, front face at y=0, from z0 to z1 (dock datum)."""
    r = p['rail']
    height, face = r['height'], r['face']
    mid = height / 2
    body = P.rbox(face, height, z0, z1, r['corner_radius'], (0, mid))
    outline = slot_outline(r)
    for x in slot_centres(p):
        body = body.cut(_prism([(x + u, 0 + d) for u, d in outline], z0, z1))            # front
        body = body.cut(_prism([(x + u, height - d) for u, d in outline], z0, z1))        # back
        if hollow:
            body = body.cut(P.cyl(r['bore_diameter'], z0, z1, x, mid))
    for sign in (-1, 1):
        body = body.cut(_prism([(sign * (face / 2 - d), mid + u) for u, d in outline], z0, z1))
    if hollow:
        e, m, h = r['chamber_end_width'] / 2, r['chamber_mid_width'] / 2, r['chamber_height'] / 2
        knee = h - r['chamber_end_straight']
        quarter = [(e, h), (e, knee), (m, knee - (m - e))]
        half = quarter + [(u, -v) for u, v in reversed(quarter)]
        body = body.cut(_prism([(u, mid + v) for u, v in half + [(-u, v) for u, v in reversed(half)]], z0, z1))
    return P.solid(body.clean())


def rack_bottom_z(p):
    return P.bed_z(p) - p['rail']['below']


def printed_rail(p, length=320.0):
    """Plain 320 mm (default) upright rail coupon; prints vertically with a brim."""
    z0 = rack_bottom_z(p)
    return rail_section(p, z0, z0 + length, hollow=True)


def rack_rail(p, shelves=None):
    """Full-height rack rail: 40 mm foot collar zone, all shelves, and top margin."""
    shelves = p['stack']['shelves'] if shelves is None else shelves
    z0 = rack_bottom_z(p)
    z1 = P.bed_z(p) + shelves * p['stack']['pitch'] + p['rail']['above']
    return rail_section(p, z0, z1, hollow=True)


# ------------------------------------------------------------------- T-keys

def _key_solids(p, z0, z1, face=None, neck_anchor=0.02, chamfer_bottom=True, chamfer_top=True):
    """Two male T-keys protruding +Y from `face`, running z0..z1 (dock datum)."""
    r, k = p['rail'], p['dock']
    face = rail_front_y(p) if face is None else face
    neck_depth = r['lip_thickness'] + k['key_neck_clearance']
    half = k['key_head_width'] / 2
    y0 = face + neck_depth
    y1 = y0 + k['key_head_shoulder']
    y2 = face + r['slot_depth'] - k['key_floor_clearance']
    end = half - (y2 - y1)
    keys = []
    for x in slot_centres(p):
        neck = P.rbox(k['key_neck_width'], neck_depth + neck_anchor + 0.02, z0, z1, 0,
                      (x, face + (neck_depth - neck_anchor) / 2))
        head = (cq.Workplane('XY', origin=(0, 0, z0))
                .polyline([(x - half, y0), (x + half, y0), (x + half, y1),
                           (x + end, y2), (x - end, y2), (x - half, y1)])
                .close().extrude(z1 - z0))
        ch = k['key_end_chamfer']
        if chamfer_bottom:
            head = head.faces('<Z').chamfer(ch)
        if chamfer_top:
            head = head.faces('>Z').chamfer(ch)
        keys.append((neck, head))
    return keys


def _cut_front_tnut_bores(body, p, y_front, y_back, z):
    """Cut two front-to-back recessed M5 bolt holes at slot_centres (x = +/-10 mm)."""
    k = p['dock']
    bolt_d = k['t_nut_bolt_diameter']
    cb_d = k['t_nut_counterbore_diameter']
    flange = k['t_nut_flange_thickness']
    through_len = (y_back - y_front) + 2.0
    cb_len = (y_back - flange) - (y_front - 1.0)
    for x in slot_centres(p):
        hole = cq.Workplane('XZ', origin=(x, y_front - 1.0, z)).circle(bolt_d / 2).extrude(-through_len)
        bore = cq.Workplane('XZ', origin=(x, y_front - 1.0, z)).circle(cb_d / 2).extrude(-cb_len)
        body = body.cut(hole).cut(bore)
    return body


def keyed_dock(p):
    """Shelf dock with mid-height recessed M5 bores and upper/lower T-keys capturing the T-nut."""
    k = p['dock']
    bed, pitch = P.bed_z(p), p['stack']['pitch']
    root, face = p['tine']['dock_root_y'], rail_front_y(p)
    z_bolt = bed + k['t_nut_z_from_bed']
    body = cq.Workplane('XY').add(P.dock_body(p))  # dock datum
    body = _cut_front_tnut_bores(body, p, root, face, z_bolt)
    bay_half = k['t_nut_pocket_height'] / 2
    margin = k['key_z_min_above_bed']
    # Lower T-key acts as a flat seat under the T-nut; upper T-key aligns & captures it above.
    for z0, z1, ch_bot, ch_top in (
        (bed + margin, z_bolt - bay_half, True, False),
        (z_bolt + bay_half, bed + pitch - margin, False, True),
    ):
        for neck, head in _key_solids(p, z0, z1, chamfer_bottom=ch_bot, chamfer_top=ch_top):
            body = body.union(cq.Workplane('XY').add(neck)).union(cq.Workplane('XY').add(head))
    return P.solid(body.clean())


# ----------------------------------------------------------- four-sided foot

def foot_long_wall(p):
    """Collar thickness on the two 40 mm faces: flange + recessed button-head counterbore."""
    return p['dock']['t_nut_flange_thickness'] + max(p['hardware']['m5_head_height'] + 0.75, 3.5)


def _side_key_solids(p, z0, z1, pc):
    """Two male T-keys protruding inward from the collar side walls (x = -/+ face/2).

    Same neck and head section as the shelf keys, bridged across the pocket
    clearance `pc` to the wall, with a lead-in chamfer on the top end because
    the rail drops in from above. Rail frame: front face y = 0, depth 0..height.
    """
    r, k = p['rail'], p['dock']
    face, h = r['face'], r['height']
    neck_depth = r['lip_thickness'] + k['key_neck_clearance']
    half = k['key_head_width'] / 2
    y0 = neck_depth
    y1 = y0 + k['key_head_shoulder']
    y2 = r['slot_depth'] - k['key_floor_clearance']
    end = half - (y2 - y1)
    head_uv = [(y0, -half), (y0, half), (y1, half), (y2, end), (y2, -end), (y1, -half)]
    nw = k['key_neck_width'] / 2
    neck_uv = [(-pc - 0.5, -nw), (y0 + 0.02, -nw), (y0 + 0.02, nw), (-pc - 0.5, nw)]
    keys = []
    for s in (-1, 1):
        def xy(uv):
            return [(s * (face / 2 - u), h / 2 + v) for u, v in uv]
        neck = cq.Workplane('XY', origin=(0, 0, z0)).polyline(xy(neck_uv)).close().extrude(z1 - z0)
        head = cq.Workplane('XY', origin=(0, 0, z0)).polyline(xy(head_uv)).close().extrude(z1 - z0)
        head = head.faces('>Z').chamfer(k['key_end_chamfer'])
        keys.append(neck.union(head))
    return keys


def rack_foot(p, height=40.0, base_thickness=5.0):
    """Four-sided gusseted collar capturing all four faces of the 20 x 40 rail.

    Local datum: origin at the rail front-centre on its bottom face. The collar
    surrounds the 20 x 40 profile on all four sides (0.30 mm clearance per side).
    On the two long (40 mm) faces, recessed M5 bolt holes at x = +/-10 mm clamp
    into standard 20-series T-nuts inside the front and rear slots. The 96 x 96 mm
    baseplate mounts directly onto four protruding M5 floor-tile posts (PCD
    100 mm about (0, 10) mm) and fastens from above with M5 washers and nuts.
    Male T-keys on both short side walls run in the single 20 mm side slots, so
    the rail drops in over them; the T-nuts load from the rail's bottom end into
    the front and rear slots before the rail goes in.
    """
    r, k = p['rail'], p['dock']
    face, h = r['face'], r['height']          # 40 across X, 20 deep Y
    pc = 0.30                                  # pocket clearance per side
    side_wall = 6.0                            # 20 mm short sides
    long_wall = foot_long_wall(p)              # 40 mm long sides (10.5 mm)
    hx, hy = face / 2 + pc, h / 2 + pc         # pocket half-widths around (0, h/2)
    base_z0, base_z1 = -base_thickness, 0.0
    body = P.rbox(96.0, 96.0, base_z0, base_z1, 12.0, (0.0, 10.0))

    # Full four-sided collar around the 20 x 40 profile.
    collar = P.rbox(2 * (hx + side_wall), 2 * (hy + long_wall), base_z1, height, 2.0, (0, h / 2))
    pocket = P.rbox(2 * hx, 2 * hy, base_z1 - 0.1, height + 1.0, 1.0, (0, h / 2))
    body = body.union(collar.cut(pocket))

    # Recessed M5 bolt holes on both long (40 mm) faces at mid-height.
    z_bolt = height / 2
    bolt_d = k['t_nut_bolt_diameter']
    cb_d = k['t_nut_counterbore_diameter']
    flange = k['t_nut_flange_thickness']
    y_front_outer = -pc - long_wall
    y_back_outer = h + pc + long_wall
    y_front_cb_end = -pc - flange
    y_back_cb_start = h + pc + flange
    for x in slot_centres(p):
        body = body.cut(cq.Workplane('XZ', origin=(x, y_front_outer - 1.0, z_bolt))
                        .circle(bolt_d / 2).extrude(-(y_back_outer - y_front_outer + 2.0)))
        body = body.cut(cq.Workplane('XZ', origin=(x, y_front_outer - 1.0, z_bolt))
                        .circle(cb_d / 2).extrude(-(y_front_cb_end - (y_front_outer - 1.0))))
        body = body.cut(cq.Workplane('XZ', origin=(x, y_back_cb_start, z_bolt))
                        .circle(cb_d / 2).extrude(-(y_back_outer + 1.0 - y_back_cb_start)))

    # Male T-keys from both short side walls into the single 20 mm side slots.
    # They centre the rail and stop it rocking before the bolts are tight, and
    # leave the front and rear slots free for the T-nuts.
    for key in _side_key_solids(p, base_z1, height, pc):
        body = body.union(key)

    # 45-degree gusset ribs bracing the outside of each side wall to the base.
    g = 18.0
    depth = 26.0
    y_gusset_end = h / 2 + depth / 2
    for sign in (-1, 1):
        web = (cq.Workplane('XZ', origin=(0, y_gusset_end, 0))
               .polyline([(sign * (hx + side_wall), base_z1), (sign * (hx + side_wall), base_z1 + g),
                          (sign * (hx + side_wall + g), base_z1)]).close().extrude(depth))
        body = body.union(web)

    # Direct 4-post M5 floor-tile mounting bores on PCD 100 mm about (0, 10).
    for ang in FOOT_BOLT_ANGLES:
        a = math.radians(ang)
        bx, by = F.FLOOR_POST_PCD / 2 * math.cos(a), 10.0 + F.FLOOR_POST_PCD / 2 * math.sin(a)
        body = body.cut(P.cyl(F.POST_DIAMETER, base_z0 - 1, base_z1 + 1, bx, by))
    return P.solid(body.clean())


# ------------------------------------------------------------------- top cap

def top_cap_height(p):
    """Socket depth for the top shelf tenons plus a solid roof clearing the rail top."""
    k, r = p['dock'], p['rail']
    roof = min(k['tenon_width_x'], k['tenon_depth_y']) / 2 + k['socket_clearance_xy'] - 0.8
    return round(max(k['tenon_height'] + k['socket_roof_gap'] + roof + 4.4, r['above'] + 6.0), 1)


def top_cap_bolt_z(p):
    """Vertical axis of the two rear T-nut bolts on top_cap in shelf-0 datum."""
    return P.bed_z(p) + p['stack']['pitch'] + p['rail']['above'] / 2


def top_cap(p):
    """Four-sided cap fitting over the top shelf tenons and rail top with front T-keys and rear T-nut bores.

    Local datum: seated on the top shoulder of shelf 0 (z0 = bed + pitch);
    rack_assembly translates it up by (shelves - 1) * pitch. Prints upside down
    with sockets, 4-sided rail pocket, and male T-keys facing upward (no supports).
    """
    k, r = p['dock'], p['rail']
    bed, pitch = P.bed_z(p), p['stack']['pitch']
    z0 = bed + pitch
    height = top_cap_height(p)
    if r['above'] + 0.5 > height - 3.0:
        raise ValueError('rail top too high for the top cap')
    root, face = p['tine']['dock_root_y'], rail_front_y(p)
    mid_y = P.column_mid_y(p)
    pc = 0.30
    back_wall = foot_long_wall(p)
    cap_y0 = root
    cap_y1 = face + r['height'] + pc + back_wall
    body = P.rbox(k['column_width'], cap_y1 - cap_y0, z0, z0 + height, 2.0, (0, (cap_y0 + cap_y1) / 2))
    for tx in P.tenon_centres(p):
        body = body.cut(P._socket_void(k, tx, mid_y, z0))
    body = body.cut(P.rbox(r['face'] + 2 * pc, r['height'] + 2 * pc, z0 - 0.1, z0 + r['above'] + 0.5, 1.0,
                           (0, face + r['height'] / 2)))
    for neck, head in _key_solids(p, z0 + 0.8, z0 + r['above'] - 0.5, face=face,
                                  neck_anchor=1.5, chamfer_bottom=True, chamfer_top=True):
        body = body.union(cq.Workplane('XY').add(neck)).union(cq.Workplane('XY').add(head))
    # Two recessed M5 button-head T-nut bores on the back side into the rear 20 x 40 slots.
    z_bolt = top_cap_bolt_z(p)
    bolt_d = k['t_nut_bolt_diameter']
    cb_d = k['t_nut_counterbore_diameter']
    flange = k['t_nut_flange_thickness']
    y_back_in = face + r['height'] + pc
    y_cb_start = y_back_in + flange
    for x in slot_centres(p):
        body = body.cut(cq.Workplane('XZ', origin=(x, y_back_in - 1.0, z_bolt))
                        .circle(bolt_d / 2).extrude(-(back_wall + 2.0)))
        body = body.cut(cq.Workplane('XZ', origin=(x, y_cb_start, z_bolt))
                        .circle(cb_d / 2).extrude(-(cap_y1 + 1.0 - y_cb_start)))
    return P.solid(body.clean())


# ----------------------------------------------------------- hardware solids

def t_nut(p):
    """Nominal 20-series M5 T-nut in slot datum (origin on rail face, +Y into slot)."""
    r, hw = p['rail'], p['hardware']
    lip = r['lip_thickness']
    w = hw['t_nut_width'] / 2
    length = hw['t_nut_length']
    neck_w = r['slot_opening'] / 2 - 0.3
    y0 = lip + 0.2
    y_knee = lip + r['slot_shoulder']
    y_floor = r['slot_depth'] - 0.5
    top_w = min(w, r['slot_floor_width'] / 2 - 0.2)
    neck_len = min(length, 6.0)
    right = [(w, y0), (w, y_knee), (top_w, y_floor)]
    pts = right + [(-u, d) for u, d in reversed(right)]
    body = cq.Workplane('XY', origin=(0, 0, -length / 2)).polyline(pts).close().extrude(length)
    neck = (cq.Workplane('XY', origin=(0, 0, -neck_len / 2))
            .rect(2 * neck_w, lip).extrude(neck_len).translate((0, 0.2 + lip / 2, 0)))
    nut = body.union(neck).cut(
        cq.Workplane('XZ', origin=(0, 0, 0)).circle(2.55).extrude(-(y_floor + 1.0))
    )
    return P.solid(nut.clean())


def m5_bolt(p):
    """Nominal M5 x 8 mm ISO 7380 button-head screw (y=0 at flange bearing face, shank +Y)."""
    hw = p['hardware']
    head_r = (hw['m5_head_diameter'] - 0.1) / 2
    head_h = hw['m5_head_height']
    shank_l = hw['m5_length']
    skirt = min(0.8, head_h * 0.3)
    head = (cq.Workplane('XZ', origin=(0, -skirt, 0)).circle(head_r).extrude(-skirt)
            .union(cq.Workplane('XZ', origin=(0, -skirt, 0)).circle(head_r)
                   .workplane(offset=head_h - skirt).circle(head_r * 0.52).loft()))
    shank = cq.Workplane('XZ', origin=(0, 0, 0)).circle(4.9 / 2).extrude(-shank_l)
    return P.solid(head.union(shank).clean())


def rack_hardware(p, shelves=None):
    """M5 x 8 button-head bolts and 20-series T-nuts for the four-sided foot and every shelf."""
    shelves = p['stack']['shelves'] if shelves is None else shelves
    pitch = p['stack']['pitch']
    face = rail_front_y(p)
    back_face = face + p['rail']['height']
    flange = p['dock']['t_nut_flange_thickness']
    pc = 0.30
    z_foot = rack_bottom_z(p) + 20.0
    z_shelf0 = P.bed_z(p) + p['dock']['t_nut_z_from_bed']
    nut = t_nut(p)
    bolt = m5_bolt(p)
    hw = {}
    for side, x in (('l', slot_centres(p)[0]), ('r', slot_centres(p)[1])):
        hw[f'foot_front_nut_{side}'] = nut.translate((x, face, z_foot))
        hw[f'foot_front_bolt_{side}'] = bolt.translate((x, face - pc - flange, z_foot))
        hw[f'foot_back_nut_{side}'] = nut.rotate((0, 0, 0), (0, 0, 1), 180).translate((x, back_face, z_foot))
        hw[f'foot_back_bolt_{side}'] = bolt.rotate((0, 0, 0), (0, 0, 1), 180).translate((x, back_face + pc + flange, z_foot))
        for i in range(shelves):
            z = z_shelf0 + i * pitch
            hw[f'shelf{i}_nut_{side}'] = nut.translate((x, face, z))
            hw[f'shelf{i}_bolt_{side}'] = bolt.translate((x, face - flange, z))
        z_cap = top_cap_bolt_z(p) + (shelves - 1) * pitch
        hw[f'top_cap_back_nut_{side}'] = nut.rotate((0, 0, 0), (0, 0, 1), 180).translate((x, back_face, z_cap))
        hw[f'top_cap_back_bolt_{side}'] = bolt.rotate((0, 0, 0), (0, 0, 1), 180).translate((x, back_face + pc + flange, z_cap))
    return hw


# ------------------------------------------------------------- assembly checks

def rack_assembly(p, shelves=None):
    """Nominal rack: 4-sided foot + rail + front-bolted shelves + keyed top cap.

    Returned in the dock datum with the foot collar resting on z = rack_bottom_z.
    """
    shelves = p['stack']['shelves'] if shelves is None else shelves
    pitch = p['stack']['pitch']
    z0 = rack_bottom_z(p)
    face = rail_front_y(p)
    parts = {'foot': rack_foot(p).translate((0, face, z0))}
    parts['rail'] = rack_rail(p, shelves).translate((0, face, 0))
    for i in range(shelves):
        parts[f'shelf{i}'] = keyed_dock(p).translate((0, 0, i * pitch))
    parts['top_cap'] = top_cap(p).translate((0, 0, (shelves - 1) * pitch))
    return parts
