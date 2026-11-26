"""Core parametric parts for PheneMachine OpenKit:
- 4th-motor (wrist_flex) SO-ARM101 clevis robot fork (`robot_fork_body`)
- Compact reversible carrier (`carrier`) for 100 mm round Petri dishes
- 100 mm round Petri dish base and lid (`dish_base`, `dish_lid`, `plate`)
- Self-stacking modular tower shelf fork (`dock_body`) + printed tower base (`tower_base` / `rail_spacer`)
- Stack-on SO-ARM101 USB camera imager mount (`imager_camera_mount`)

All solids are built directly in the six-STEP dock datum:
X across the tines, +Y into the dock (radially outward from the robot), +Z up,
origin on the carrier underside support plane at the seated carrier centre.
"""
from __future__ import annotations

import math
import tomllib
from math import radians, tan
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / "parameters" / "openkit.toml"


def load(path=DEFAULT, robot_spacing=None, overrides=None):
    with Path(path).open("rb") as stream:
        p = tomllib.load(stream)
    if robot_spacing is not None:
        p["interface"]["robot_spacing"] = float(robot_spacing)
    for dotted, value in (overrides or {}).items():
        section, key = dotted.split(".")
        if key not in p[section]:
            raise KeyError(dotted)
        p[section][key] = value
    validate_mount(p)
    return p


def validate_mount(p):
    r = p["robot"]
    if r.get("mount_style", "so101") not in ("so101", "so102_candidate"): raise ValueError("unknown mount style")
    if r.get("mount_style") != "so102_candidate": return
    span, pcd, screw, pilot, h, travel = (r[k] for k in ("horn_face_span", "horn_screw_pcd", "horn_screw_diameter", "horn_pilot_diameter", "ear_height", "horn_slot_radial_travel"))
    reach = pcd / 2 + travel / 2 + screw / 2 + 2
    if not (25 <= span <= 60 and 8 <= pcd <= 24 and 2 <= screw <= 5 and 3.2 <= pilot <= 12 and 0 <= travel <= 3 and 6 <= r["clevis_ear_thickness"] <= 12 and h <= 50 and h / 2 >= reach and r["pad_size"] / 2 >= reach and pcd / 2 - travel / 2 - screw / 2 >= pilot / 2 + 2 and p["tine"]["robot_root_y"] - r["bridge_depth_y"] >= r["flange_y"] + reach): raise ValueError("invalid SO102 mount dimensions or less than 2 mm pattern wall")


def solid(workplane):
    shape = workplane.val() if isinstance(workplane, cq.Workplane) else workplane
    solids = shape.Solids()
    if len(solids) != 1 or not shape.isValid():
        raise ValueError(f"expected one valid solid, found {len(solids)}")
    return solids[0]


def compound(*shapes):
    solids = [s for shape in shapes for s in shape.Solids()]
    return cq.Compound.makeCompound(solids)


def rbox(x, y, z0, z1, radius=0.0, center=(0.0, 0.0)):
    """Box centred in XY with optional vertical-edge radius."""
    w = cq.Workplane("XY", origin=(center[0], center[1], z0)).rect(x, y).extrude(z1 - z0)
    return w.edges("|Z").fillet(radius) if radius else w


def cyl(diameter, z0, z1, x=0.0, y=0.0):
    return cq.Workplane("XY", origin=(x, y, z0)).circle(diameter / 2).extrude(z1 - z0)


# ---------------------------------------------------------------- shared tine

def tine_plan(t, length):
    """Shared tine outline, root on y=0, pointing +Y: taper plus rounded tip."""
    wr, wt = t["root_width"] / 2, t["tip_width"] / 2
    return (cq.Workplane("XY").moveTo(-wr, 0).lineTo(-wt, length - wt)
            .threePointArc((0, length), (wt, length - wt)).lineTo(wr, 0).close())


def tine(t, length, top_z):
    """One tine, root face on y=0, top on top_z, with the tip lead-in ramp."""
    body = tine_plan(t, length).extrude(-t["depth"]).translate((0, 0, top_z))
    lead, drop = t["tip_lead_length"], t["tip_lead_drop"]
    ramp = (cq.Workplane("YZ").moveTo(length - lead, top_z).lineTo(length + 1, top_z - drop * (lead + 1) / lead)
            .lineTo(length + 1, top_z + 1).lineTo(length - lead, top_z + 1).close()
            .extrude(t["root_width"], both=True))
    return body.cut(ramp)


def fork_body(t, spacing, length, bridge, top_z, pad=None):
    """Two shared-profile tines on a bridge. Local frame: roots on y=0, tines +Y."""
    width = spacing + t["root_width"] + 2 * bridge["margin_x"]
    body = (cq.Workplane("XY", origin=(0, -bridge["depth_y"] / 2, top_z - t["depth"]))
            .rect(width, bridge["depth_y"]).extrude(t["depth"]))
    if pad:
        body = body.union(cq.Workplane("XY", origin=(0, (pad["y0"] + pad["y1"]) / 2, top_z - t["depth"]))
                          .rect(2 * pad["half_width"], pad["y1"] - pad["y0"]).extrude(t["depth"]))
    for sign in (-1, 1):
        body = body.union(tine(t, length, top_z).translate((sign * spacing / 2, 0, 0)))
    if pad and pad.get("fillet"):
        for sign in (-1, 1):
            sel = cq.selectors.BoxSelector((sign * pad["half_width"] - 0.3, -bridge["depth_y"] - 0.3, top_z - t["depth"] - 1),
                                           (sign * pad["half_width"] + 0.3, -bridge["depth_y"] + 0.3, top_z + 1))
            body = body.edges(sel).fillet(pad["fillet"])
    r = t["root_fillet"]
    if r:
        for sign in (-1, 1):
            for side in (-1, 1):
                x = sign * spacing / 2 + side * t["root_width"] / 2
                if abs(x) > width / 2 - 0.5:
                    continue
                sel = cq.selectors.BoxSelector((x - 0.3, -0.3, top_z - t["depth"] - 1),
                                               (x + 0.3, 0.3, top_z + 1))
                body = body.edges(sel).fillet(r)
    return body


def frustum(width, length, z0, z1, slope, x=0.0, y=0.0):
    """Hip-roof prism: width x length at z=0, every side leaning in by `slope` per mm of height."""
    def rect(z):
        return width - 2 * slope * z, length - 2 * slope * z
    if min(rect(z1)) <= 0.3:
        raise ValueError("ridge closes before its crest")
    w = cq.Workplane("XY", origin=(x, y, z0)).rect(*rect(z0))
    return w.workplane(offset=z1 - z0).rect(*rect(z1)).loft(combine=True)


def rib_slope(rib):
    return tan(radians(rib["flank_angle_deg"]))


def rib_run(rib):
    """Horizontal run of the 45-degree lead-in above the vertical part: the capture range per side."""
    return (rib["height"] - rib["vertical_height"]) * rib_slope(rib)


def rib_base_width(rib):
    return rib["top_width"] + 2 * rib_run(rib)


def rib_width_at(rib, z, secondary=False, clearance=0.0):
    """Ridge width in X at height z (groove width with clearance)."""
    base = rib_base_width(rib) + 2 * clearance - (2 * rib["secondary_relief"] if secondary else 0)
    return base - 2 * max(0.0, z - rib["vertical_height"]) * rib_slope(rib)


def walled_frustum(width, length, vertical, height, slope, z0=0.0, x=0.0, y=0.0):
    """Vertical walls up to `vertical`, then every side leans in by `slope` up to `height`."""
    wall = rbox(width, length, z0, vertical)
    top = frustum(width, length, 0.0, height - vertical, slope).translate((0, 0, vertical - 0.001))
    return wall.union(top).translate((x, y, 0.0))


def ridge(rib, x, base_z=0.0, secondary=False, y=0.0):
    """Locating ridge on a tine top. The secondary one is narrower in X only."""
    slope = rib_slope(rib)
    length = 2 * rib["half_length"]
    v = rib["vertical_height"]
    if not secondary:
        return walled_frustum(rib_base_width(rib), length, v, rib["height"], slope).translate((x, y, base_z))
    width = rib_base_width(rib) - 2 * rib["secondary_relief"]
    lead = (width - rib["top_width"]) / (2 * slope)
    if lead < 0.8:
        raise ValueError("secondary ridge lead-in is too low")
    return walled_frustum(width, length, v, v + lead, slope).translate((x, y, base_z))


def groove(rib, x, y=0.0):
    """Carrier underside groove: the primary ridge offset by the clearance, plus a roof gap."""
    c = rib["clearance"]
    return walled_frustum(rib_base_width(rib) + 2 * c, 2 * rib["half_length"] + 2 * c, rib["vertical_height"],
                          rib["height"] + rib["roof_gap"], rib_slope(rib), z0=-0.01).translate((x, y, 0))


# ----------------------------------------------- carrier & 100 mm round dish

def dish_diameter(p):
    d = p["dish"]
    return float(d.get("base_diameter", d["base_x"]))


def carrier_outline(p):
    i, t, c, d = p["interface"], p["tine"], p["carrier"], p["dish"]
    width = i["robot_spacing"] + t["root_width"] + 2 * i["carrier_margin"]
    nest_d = dish_diameter(p) + 2 * c["fit_clearance"]
    return width, nest_d + 2 * c["wall"]


def carrier_plan(p, external_length=None):
    """Vertices of the unfilleted clipped carrier outline, or None for legacy plans."""
    c = p["carrier"]
    if "corner_cut_x" not in c:
        return None
    width, length = carrier_outline(p)
    x, y = width / 2, (external_length or length) / 2
    a, b = c["corner_cut_x"], c["corner_cut_y"]
    return [(-x+a,-y),(x-a,-y),(x,-y+b),(x,y-b),
            (x-a,y),(-x+a,y),(-x,y-b),(-x,-y+b)]


def carrier(p, external_length=None):
    """Compact reversible carrier with a circular nest for 100 mm round Petri dishes."""
    i, c, rib = p["interface"], p["carrier"], p["rib"]
    width, length = carrier_outline(p)
    length = external_length or length
    nest_d = dish_diameter(p) + 2 * c["fit_clearance"]
    win_d = float(c.get("window_diameter", nest_d - 2 * c["ledge_width"]))

    if "corner_cut_x" in c:
        x, y = width / 2, length / 2
        a, b = c["corner_cut_x"], c["corner_cut_y"]
        if not (0 < a < x and 0 < b < y and (x-a)/a + (y-b)/b > 0):
            raise ValueError("invalid corner cuts")
        # Distance from the origin to each clipped corner line.
        if (x/a + y/b - 1) / math.hypot(1/a, 1/b) <= nest_d/2:
            raise ValueError("corner clip intersects dish nest")
        gz = rib["height"] + rib["roof_gap"]
        if not (0 < c["seat_z"] < gz and c["thickness"] - gz >= 0.6 - 1e-9
                and win_d / 2 <= dish_diameter(p)/2 - 1
                and i["dock_spacing"]/2 - rib_base_width(rib)/2 - rib["clearance"] > nest_d/2):
            raise ValueError("invalid low seat, ring, or groove clearance")
        points = [(-x+a,-y),(x-a,-y),(x,-y+b),(x,y-b),
                  (x-a,y),(-x+a,y),(-x,y-b),(-x,-y+b)]
        body = cq.Workplane("XY").polyline(points).close().extrude(c["thickness"])
        body = body.edges("|Z").fillet(c["corner_radius"])
    else:
        body = rbox(width, length, 0, c["thickness"], c["corner_radius"])
    body = body.faces("<Z").chamfer(c["edge_chamfer"])

    # Central circular through-opening for backlighting / bottom inspection (inside the inner dock tines)
    body = body.cut(cyl(win_d, -1, c["thickness"] + 1, 0, 0))
    # Circular recess for the round 100 mm Petri dish above seat_z
    body = body.cut(cyl(nest_d, c["seat_z"], c["thickness"] + 1, 0, 0))

    # Four finger scallops on the front/back rim so the round dish can be lifted out manually
    for sx in (-1, 1):
        for sy in (-1, 1):
            body = body.cut(rbox(c["grip_width"], c["wall"] + 6.0, c["seat_z"] + c.get("grip_floor", 0), c["thickness"] + 1, 0,
                                 (sx * c["grip_x"], sy * (length / 2 - c["wall"] / 2))))

    # Four identical grooves centred on the X axis, one over each dock and robot tine,
    # so a 180-deg half-turn about Z maps every groove onto another groove.
    for spacing in (i["dock_spacing"], i["robot_spacing"]):
        for sign in (-1, 1):
            body = body.cut(groove(rib, sign * spacing / 2))
    return solid(body.clean())


def dish_base(p):
    """Printable stand-in for the 100 mm round Petri dish base."""
    c, d = p["carrier"], p["dish"]
    z = c["seat_z"]
    diam = dish_diameter(p)
    w = d["base_wall"]
    tray = cyl(diam, z, z + d["base_height"]).cut(cyl(diam - 2 * w, z + d["base_floor"], z + d["base_height"] + 1))
    return solid(tray.clean())


def dish_lid(p):
    """Printable stand-in for the 100 mm round Petri dish lid."""
    c, d = p["carrier"], p["dish"]
    z = c["seat_z"]
    outer = d["lid_outer"]
    inner = outer - 2 * d["lid_wall"]
    lid = cyl(outer, z + d["lid_skirt_above_seat"], z + d["base_height"] + d["lid_top"]).cut(
        cyl(inner, z - 1, z + d["base_height"])
    )
    return solid(lid.clean())


def plate(p):
    """Lidded 100 mm round Petri dish: base tray first, lid second."""
    return compound(dish_base(p), dish_lid(p))


# -------------------------------------------- self-stacking tower shelf dock

def bed_z(p):
    """Underside of every dock: the print bed face and the stacking shoulder datum."""
    return -p["tine"]["depth"]


def rail_face_y(p):
    return p["tine"]["dock_root_y"] + p["dock"]["beam_depth_y"]


def column_mid_y(p):
    return (p["tine"]["dock_root_y"] + rail_face_y(p)) / 2


def tenon_centres(p):
    half = p["dock"].get("tenon_pitch_x", 32.0) / 2
    return -half, half


def slot_centres(p):
    half = p["rail"]["slot_pitch"] / 2
    return -half, half


def lock_axis_z(p):
    return bed_z(p) + p["stack"]["pitch"] - p["dock"]["lock_z_from_top"]


def dock_pads(p):
    """Four support pads on the datum plane, two per tine, beyond the ridge ends."""
    half, y = p["interface"]["dock_spacing"] / 2, p["rib"]["pad_y"]
    return [(sx * half, sy * y) for sx in (-1, 1) for sy in (-1, 1)]


def _tenon_solid(k, x, y, z0):
    """One tapered male stacking tenon rising from z0 to z0 + tenon_height."""
    wx, wy = k["tenon_width_x"], k["tenon_depth_y"]
    th, ch = k["tenon_height"], k["tenon_chamfer"]
    v = th - ch
    tenon = walled_frustum(wx, wy, v, th, 1.0, z0=-0.02, x=x, y=y).translate((0, 0, z0))
    if k.get("stack_joint") == "deep_keyed":
        tenon = tenon.cut(_cross_hole(x, y - wy / 2 - 1, z0 + 4, wy + 2, 3.4))
    return tenon


def _socket_void(k, x, y, z0):
    """Matching female socket cut into the underside at z0 with a 45-degree self-supporting roof."""
    cl = k["socket_clearance_xy"]
    wx = k["tenon_width_x"] + 2 * cl
    wy = k["tenon_depth_y"] + 2 * cl
    th = k["tenon_height"] + k["socket_roof_gap"]
    ch = k["tenon_chamfer"] + cl
    v = th if k.get("stack_joint") == "deep_keyed" else th - ch
    # Roof tapers to a narrow crest at 45 degrees so it bridges cleanly with zero supports
    roof_h = min(wx, wy) / 2 - 0.8
    void = walled_frustum(wx, wy, v, v + roof_h, 1.0, z0=-0.05, x=x, y=y).translate((0, 0, z0))
    if k.get("stack_joint") == "deep_keyed":
        void = void.union(frustum(wx + 0.6, wy + 0.6, 0, 0.3, 1.0).translate((x, y, z0 - 0.01)))
    return void


def _cross_hole(x, y, z, length, diameter):
    return cq.Workplane("XZ", origin=(x, y, z)).circle(diameter / 2).extrude(-length)


def _joint_receiver_holes(body, p, z0):
    if p["dock"].get("stack_joint") != "deep_keyed":
        return body
    k = p["dock"]
    for x in tenon_centres(p):
        body = body.cut(_cross_hole(x, p["tine"]["dock_root_y"] + 2.5, z0 + 4, k["beam_depth_y"], 3.4))
    return body.cut(_cross_hole(0, rail_face_y(p) - 5, z0 + 4, 6, 2.5))


def stack_pin(p):
    return solid(cyl(3.0, 0, 19.0).clean())


def stack_pin_retainer(p):
    return solid(rbox(42, 7, 0, 2, 1).cut(cyl(3.4, -1, 3)).clean())


def dock_body(p, ribs=True):
    """One self-stacking shelf fork block:
    - Two dock tines with 4 support pads and locating ridges
    - Wide self-supporting tower column of height = stack.pitch
    - Two top male tenons + two bottom female sockets so blocks stack securely into a tower without a rail
    - Central vertical M5 tie-rod bore + horizontal M3 lock-pin hole
    """
    i, t, rib, k = (p[s] for s in ("interface", "tine", "rib", "dock"))
    relief_z = -rib["pad_relief"]
    bed, pitch = bed_z(p), p["stack"]["pitch"]
    root, face = t["dock_root_y"], rail_face_y(p)
    mid_y = column_mid_y(p)

    local = fork_body(t, i["dock_spacing"], t["dock_length"],
                      {"margin_x": 0.0, "depth_y": k["beam_depth_y"]}, relief_z)
    body = local.rotate((0, 0, 0), (0, 0, 1), 180).translate((0, root, 0))
    body = body.cut(rbox(600, 600, bed - 5, bed, 0))

    # Main stacking tower column (flat shoulder at bed + pitch supports the shelf block above)
    body = body.union(rbox(k["column_width"], k["beam_depth_y"], bed, bed + pitch, 2.0, (0, mid_y)))

    # 45-degree side gussets bracing the column to the dock tine roots
    g, half_col = k["gusset"], k["column_width"] / 2
    for sign in (-1, 1):
        max_g = min(g, (i["dock_spacing"] + t["root_width"]) / 2 - half_col)
        if max_g > 1.0:
            web = (cq.Workplane("XZ", origin=(0, face, 0))
                   .polyline([(sign * half_col, relief_z), (sign * (half_col + max_g), relief_z),
                              (sign * half_col, relief_z + max_g)])
                   .close().extrude(k["beam_depth_y"]))
            body = body.union(web)

    # Four support pads at Z = 0 and two locating ridges on the dock tines
    for x, y in dock_pads(p):
        pad = rbox(t["root_width"], rib["pad_length"], relief_z - 0.01, 0.0, 0, (x, y))
        tine_vol = tine_plan(t, t["dock_length"]).extrude(2 * t["depth"]).rotate((0, 0, 0), (0, 0, 1), 180)
        tine_vol = tine_vol.translate((x, root, -t["depth"]))
        body = body.union(pad.intersect(tine_vol))
    for sign in (-1, 1) if ribs else ():
        body = body.union(ridge(rib, sign * i["dock_spacing"] / 2, relief_z - 0.01, secondary=sign > 0))

    # Top male interlocking tenons + bottom female interlocking sockets
    for tx in tenon_centres(p):
        body = body.union(_tenon_solid(k, tx, mid_y, bed + pitch))
        body = body.cut(_socket_void(k, tx, mid_y, bed))

    body = _joint_receiver_holes(body, p, bed)

    # Central vertical tie-rod bore (M5 clearance) through the column
    tie_d = k.get("tie_rod_bore", 0.0)
    if tie_d > 0:
        body = body.cut(cyl(tie_d, bed - 1, bed + pitch + k["tenon_height"] + 1, 0.0, mid_y))

    # Horizontal M3 lock-screw / pin hole with front counterbore (lock_hole = 0 omits it, e.g. when a
    # tie rod clamps the stack: the hole would cross the tie-rod bore)
    if k["lock_hole"] <= 0:
        return solid(body.clean())
    z = lock_axis_z(p)
    hole = cq.Workplane("XZ", origin=(0, root - 1, z)).circle(k["lock_hole"] / 2).extrude(-(k["beam_depth_y"] + 2))
    bore = (cq.Workplane("XZ", origin=(0, root - 1, z)).circle(k["counterbore_diameter"] / 2)
            .extrude(-(k["counterbore_depth"] + 1)))
    return solid(body.cut(hole).cut(bore).clean())


def dock(p):
    return dock_body(p)


def tower_base(p, spacer_height=14.0, base_plate_thickness=5.0, base_plate_x=86.0, base_plate_y=64.0):
    """3D-printable self-supporting tower base plate + bottom spacer with top male tenons for Shelf 0."""
    k = p["dock"]
    bed = bed_z(p)
    mid_y = column_mid_y(p)
    z_foot0 = bed - spacer_height - base_plate_thickness
    z_foot1 = bed - spacer_height

    # Wide flanged base plate with 4 x M4 table mounting holes
    foot = rbox(base_plate_x, base_plate_y, z_foot0, z_foot1, 4.0, (0, mid_y))
    for sx in (-1, 1):
        for sy in (-1, 1):
            foot = foot.cut(cyl(4.5, z_foot0 - 1, z_foot1 + 1,
                                sx * (base_plate_x / 2 - 8.0), mid_y + sy * (base_plate_y / 2 - 8.0)))

    # Bottom riser column up to Shelf 0's bed_z
    col = rbox(k["column_width"], k["beam_depth_y"], z_foot1 - 0.01, bed, 2.0, (0, mid_y))
    body = foot.union(col)

    # Top male tenons that lock into Shelf 0's bottom female sockets
    for tx in tenon_centres(p):
        body = body.union(_tenon_solid(k, tx, mid_y, bed))

    tie_d = k.get("tie_rod_bore", 0.0)
    if tie_d > 0:
        body = body.cut(cyl(tie_d, z_foot0 - 1, bed + k["tenon_height"] + 1, 0.0, mid_y))
    return solid(body.clean())


def rail_spacer(p, height=14.0):
    """Bottom tower spacer block with top male tenons that mate into the lowest dock_body."""
    k = p["dock"]
    bed = bed_z(p)
    mid_y = column_mid_y(p)
    body = rbox(k["column_width"], k["beam_depth_y"], bed - height, bed, 2.0, (0, mid_y))
    for tx in tenon_centres(p):
        body = body.union(_tenon_solid(k, tx, mid_y, bed))
    tie_d = k.get("tie_rod_bore", 0.0)
    if tie_d > 0:
        body = body.cut(cyl(tie_d, bed - height - 1, bed + k["tenon_height"] + 1, 0.0, mid_y))
    return solid(body.clean())


def imager_camera_mount(p):
    """3D-printable SO-ARM101 USB camera mount that stacks directly onto the imaging dock's top tenons.

    Accommodates both the standard 32x32 / 38x38 mm UVC USB camera board shipped with SO-ARM101 / LeRobot kits
    (28x28 mm and 34x34 mm M2.5 hole patterns + M12 lens collar) and a 1/4\"-20 webcam screw.
    """
    k, im = p["dock"], p["imager"]
    bed, pitch = bed_z(p), p["stack"]["pitch"]
    z0 = bed + pitch
    z_cam = im["camera_height_above_datum"]
    mid_y = column_mid_y(p)
    root, face = p["tine"]["dock_root_y"], rail_face_y(p)

    # Vertical riser mast from the top of the imaging dock (z0) to z_cam + arm_thickness_z
    arm_t = im["arm_thickness_z"]
    mast = rbox(k["column_width"], k["beam_depth_y"], z0, z_cam + arm_t, 2.0, (0, mid_y))
    for tx in tenon_centres(p):
        mast = mast.cut(_socket_void(k, tx, mid_y, z0))

    mast = _joint_receiver_holes(mast, p, z0)

    # Overhanging camera boom from the mast out to the dish centre (x=0, y=0)
    boom_w = im["arm_width_x"]
    boom = rbox(boom_w, face, z_cam, z_cam + arm_t, 3.0, (0, face / 2))

    # Camera cradle head centred at (x=0, y=0)
    head_sz = im["board_outer_size"] + 8.0
    head = rbox(head_sz, head_sz, z_cam - 4.0, z_cam + arm_t, 4.0, (0, 0))
    body = mast.union(boom).union(head)

    # Recess for the 38x38 mm USB camera PCB on the underside + central M12 lens bore / 1/4"-20 slot
    pcb_sz = im["board_outer_size"] + 0.6
    body = body.cut(rbox(pcb_sz, pcb_sz, z_cam - 5.0, z_cam, 1.0, (0, 0)))
    body = body.cut(cyl(im["lens_bore_diameter"], z_cam - 6.0, z_cam + arm_t + 1.0, 0, 0))

    # M2.5 mounting holes for both 28x28 mm (32mm module) and 34x34 mm (38mm module) SO-ARM101 USB cameras
    for spacing in (im["hole_spacing_inner"], im["hole_spacing_outer"]):
        h = spacing / 2
        for sx in (-1, 1):
            for sy in (-1, 1):
                body = body.cut(cyl(2.6, z_cam - 6.0, z_cam + arm_t + 1.0, sx * h, sy * h))

    tie_d = k.get("tie_rod_bore", 0.0)
    if tie_d > 0:
        body = body.cut(cyl(tie_d, z0 - 1, z_cam + arm_t + 1, 0.0, mid_y))
    return solid(body.clean())


def rail_section(p, z0, z1, hollow=True):
    """Tower spine column section between z0 and z1 (kept compatible with scene.py and cad_audit.py)."""
    k = p["dock"]
    mid_y = column_mid_y(p)
    body = rbox(k["column_width"], k["beam_depth_y"], z0, z1, 2.0, (0, mid_y))
    if hollow and k.get("tie_rod_bore", 0.0) > 0:
        body = body.cut(cyl(k["tie_rod_bore"], z0 - 1, z1 + 1, 0.0, mid_y))
    return solid(body.clean())


def extrusion(p, shelves=1):
    """Reference tower support column below the lowest dock."""
    r = p["rail"]
    z0 = bed_z(p) - r["below"]
    z1 = bed_z(p)
    return rail_section(p, z0, z1, hollow=True)


def printed_rail(p, shelves=None):
    return tower_base(p, spacer_height=p["printed_rail"]["below"])


def lock_stack(p):
    k, h = p["dock"], p["hardware"]
    thread = k["beam_depth_y"] - k["counterbore_depth"]
    return {
        "screw": "ISO 4762 M3 x 16 + M5 tower tie-rod",
        "plain_hole_mm": k["lock_hole"],
        "thread_formed_in_petg_mm": thread,
        "head_underside_above_counterbore_floor_mm": 0.0,
        "head_top_below_front_face_mm": 1.0,
        "dock_shift_toward_robot_when_locked_mm": 0.0,
        "lock_axis_z_mm": lock_axis_z(p),
    }


def lock_screw(p):
    k, t = p["dock"], p["tine"]
    z = lock_axis_z(p)
    y_head = t["dock_root_y"] - 1.0
    screw = (cq.Workplane("XZ", origin=(0, y_head, z)).circle(2.75).extrude(-3.0)
             .union(cq.Workplane("XZ", origin=(0, y_head + 2.99, z)).circle(1.5).extrude(-12.0)))
    return solid(screw)


# ----------------------------------------- 4th-motor SO-ARM101 robot fork side

def flange_points(p):
    f = p["flange"]
    half = f["pcd"] / 2 * 0.7071067811865476
    return [(sx * half, sy * half) for sx in (-1, 1) for sy in (-1, 1)]


def robot_fork_body(p, ribs=True):
    """One-piece 3D-printable robot fork for the 4th motor (`wrist_flex`) of the SO-ARM101.

    Prints completely flat on Z = -tine.depth without supports:
    - Two tapered tines with locating ridges
    - Cross-bridge at robot_root_y
    - Two rear clevis ears (inner span = 36.2 mm) that clamp directly onto Motor 4's active and passive horns
    """
    i, t, rib, rb = (p[s] for s in ("interface", "tine", "rib", "robot"))
    y_axis = rb["flange_y"]
    bridge_y0 = t["robot_root_y"] - rb["bridge_depth_y"]
    ear_y0 = y_axis - rb["pad_size"] / 2
    ear_y1 = bridge_y0 + 0.02
    candidate = rb.get("mount_style") == "so102_candidate"
    if candidate: validate_mount(p)
    bottom_z = -rb["ear_height"] / 2 if candidate else -t["depth"]
    top_z = rb["ear_height"] / 2 if candidate else rb.get("clevis_top_z", 7.0)
    inner_x = rb["horn_face_span"] / 2 if candidate else rb.get("clevis_inner_half_x", 18.1)
    ear_t = rb.get("clevis_ear_thickness", 6.0)
    outer_x = inner_x + ear_t

    local = fork_body(t, i["robot_spacing"], t["robot_length"],
                      {"margin_x": 0.0, "depth_y": rb["bridge_depth_y"]}, 0.0)
    body = local.translate((0, t["robot_root_y"], 0))

    # Two Motor-4 clevis ears extending from bridge_y0 back to ear_y0, flat on bottom_z
    for sign in (-1, 1):
        ear_cx = sign * (inner_x + ear_t / 2)
        ear = rbox(ear_t, ear_y1 - ear_y0, bottom_z, top_z, 1.5, (ear_cx, (ear_y0 + ear_y1) / 2))
        # Gusset Ramp from top of clevis ear (top_z at bridge_y0) down to bridge top (0.0 at robot_root_y)
        ramp = (cq.Workplane("YZ", origin=(sign * inner_x, 0, 0))
                .moveTo(bridge_y0, 0.0).lineTo(bridge_y0, top_z).lineTo(t["robot_root_y"], 0.0).close()
                .extrude(sign * ear_t))
        body = body.union(ear).union(ramp)

    for sign in (-1, 1) if ribs else ():
        body = body.union(ridge(rib, sign * i["robot_spacing"] / 2, -0.01, secondary=sign > 0))

    if candidate:
        return solid(_so102_pattern(body, rb, y_axis, outer_x).clean())

    # Horizontal Motor-4 (wrist_flex) horn center boss bore and 4 x M2.5 horn screw holes along the X axis
    axis_z = 0.0
    pilot_hole = (cq.Workplane("YZ", origin=(-outer_x - 1, 0, 0))
                  .moveTo(y_axis, axis_z).circle(rb["pilot_diameter"] / 2)
                  .extrude(2 * outer_x + 2))
    body = body.cut(pilot_hole)

    pcd_r = rb.get("horn_pcd", 14.0) / 2
    screw_r = rb.get("horn_screw_clearance", 2.7) / 2
    for angle_deg in (45.0, 135.0, 225.0, 315.0):
        ang = radians(angle_deg + rb.get("pitch_offset_deg", 0.0))
        dy = pcd_r * math.cos(ang)
        dz = pcd_r * math.sin(ang)
        sh = (cq.Workplane("YZ", origin=(-outer_x - 1, 0, 0))
              .moveTo(y_axis + dy, axis_z + dz).circle(screw_r)
              .extrude(2 * outer_x + 2))
        body = body.cut(sh)

    return solid(body.clean())


def _so102_pattern(body, rb, y_axis, outer_x):
    plane = lambda: cq.Workplane("YZ", origin=(-outer_x - 1, 0, 0))
    body = body.cut(plane().moveTo(y_axis, 0).circle(rb["horn_pilot_diameter"] / 2).extrude(2 * outer_x + 2))
    for angle in (45, 135, 225, 315):
        a = angle + rb.get("pitch_offset_deg", 0)
        r = rb["horn_screw_pcd"] / 2
        slot = plane().center(y_axis + r * math.cos(radians(a)), r * math.sin(radians(a))).slot2D(rb["horn_screw_diameter"] + rb["horn_slot_radial_travel"], rb["horn_screw_diameter"], a).extrude(2 * outer_x + 2)
        body = body.cut(slot)
    return body


def flange_stack(p):
    rb = p["robot"]
    if rb.get("mount_style") == "so102_candidate":
        return {"status": "Parametric candidate; physical fit unverified", "screw": "Metal horn screws and washers; measure size and engagement", "assumed_dimensions_mm": {k: rb[k] for k in ("horn_face_span", "horn_screw_pcd", "horn_screw_diameter", "horn_pilot_diameter", "ear_height", "horn_slot_radial_travel")}}
    return {
        "screw": "Feetech STS3215 M2.5 x 6 horn screws",
        "quantity": 4,
        "clevis_ear_thickness_mm": rb.get("clevis_ear_thickness", 6.0),
        "clevis_inner_span_mm": 2 * rb.get("clevis_inner_half_x", 18.1),
        "horn_pcd_mm": rb.get("horn_pcd", 14.0),
    }


def robot_hardware(p):
    rb = p["robot"]
    y_axis = rb["flange_y"]
    if rb.get("mount_style") == "so102_candidate": return []
    inner_x = rb.get("clevis_inner_half_x", 18.1)
    ear_t = rb.get("clevis_ear_thickness", 6.0)
    pcd_r = rb.get("horn_pcd", 14.0) / 2
    screws = []
    for sign in (-1, 1):
        x0 = sign * (inner_x + ear_t)
        s = (cq.Workplane("YZ", origin=(x0, 0, 0))
             .moveTo(y_axis, 0.0).circle(2.5).extrude(-sign * 6.0))
        screws.append(solid(s))
    return screws


def robot_fork(p):
    return robot_fork_body(p)


def flange_puck(p):
    """Test clevis gauge to check fit on SO-ARM101 Motor 4 before printing the full fork."""
    rb = p["robot"]
    inner_x = rb.get("clevis_inner_half_x", 18.1)
    ear_t = rb.get("clevis_ear_thickness", 6.0)
    if rb.get("mount_style") == "so102_candidate":
        validate_mount(p)
        inner_x = rb["horn_face_span"] / 2
        outer_x = inner_x + ear_t
        h, size = rb["ear_height"], rb["pad_size"]
        puck = rbox(2 * outer_x, size + 6, -h / 2, h / 2).cut(rbox(2 * inner_x, size + 0.01, -h / 2 - 1, h / 2 + 1, center=(0, -3)))
        return solid(_so102_pattern(puck, rb, 0, outer_x).clean())
    outer_x = inner_x + ear_t
    puck = rbox(2 * outer_x, 22.0, -7.0, 7.0, 1.5, (0, 0)).cut(
        rbox(2 * inner_x, 18.0, -8.0, 8.0, 0, (0, -3.0))
    )
    hole = (cq.Workplane("YZ", origin=(-outer_x - 1, 0, 0))
            .moveTo(0, 0).circle(rb["pilot_diameter"] / 2).extrude(2 * outer_x + 2))
    return solid(puck.cut(hole).clean())


def hardware(p):
    return compound(lock_screw(p), *robot_hardware(p))


def flange(p):
    """Reference envelope of SO-ARM101 Motor 4 (`sts3215_03a_v1_4`) at `wrist_flex`, between the clevis ears."""
    f, rb = p["flange"], p["robot"]
    y0 = rb["flange_y"]
    w_x = (rb["horn_face_span"] if rb.get("mount_style") == "so102_candidate" else f.get("servo_width_x", 36.2)) - 0.8
    front = f.get("servo_front_from_axis", 10.2)
    back = f.get("servo_back_from_axis", 35.2)
    hz = f.get("servo_height_z", 24.8) / 2
    ymid = y0 + (front - back) / 2
    ylen = front + back
    body = rbox(w_x, ylen, -hz, hz, 2.0, (0.0, ymid))
    return solid(body.clean())


def build(p):
    """The six interchange solids at the pickup-contact datum, one shelf."""
    return {
        "robot_fork": robot_fork(p),
        "carrier": carrier(p),
        "dock": dock(p),
        "plate": plate(p),
        "flange": flange(p),
        "extrusion": extrusion(p),
    }


def printable(p):
    """All 3D-printable plastic parts in the dock datum."""
    result = {
        "robot_fork_body": robot_fork_body(p),
        "carrier": carrier(p),
        "dock_body": dock_body(p),
        "dish_base": dish_base(p),
        "dish_lid": dish_lid(p),
        "tower_base": tower_base(p),
        "imager_camera_mount": imager_camera_mount(p),
    }
    if p["dock"].get("stack_joint") == "deep_keyed":
        result.update(stack_pin=stack_pin(p), stack_pin_retainer=stack_pin_retainer(p))
    return result


def print_oriented(name, shape):
    """Move one printable part to its export print pose on Z = 0; assess support separately."""
    if name in ("dish_lid", "imager_camera_mount"):
        shape = shape.rotate((0, 0, 0), (1, 0, 0), 180)
    box = shape.BoundingBox()
    return shape.translate((-(box.xmin + box.xmax) / 2, -(box.ymin + box.ymax) / 2, -box.zmin))
