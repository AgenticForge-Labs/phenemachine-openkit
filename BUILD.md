# OpenKit build guide (concept)

Two ten-shelf racks and one USB imaging station on a four-tile floor, for round Petri dishes up to a 94 mm base, 100 mm lid and 16 mm height. Square dishes are not supported.

> Nothing here has been printed, measured, load-tested or run on a robot. Treat every mass and clearance as a CAD estimate. Parts are listed in [PARTS.md](PARTS.md).

## 1. Print orientation and counts

| Part | Orientation | Est. print mass* |
| --- | --- | --- |
| Floor tile (`286.1 × 286.1 × 14 mm`) | flat on the 261.0 × 261.0 mm base (`285.0 × 285.0 mm` nominal tab-to-tab; `286.1 × 286.1 mm` incl. `0.55 mm` detent ridges) | see [PARTS.md](PARTS.md) |
| Robot stand (30/45/55 mm, `150 × 130 × rise`) | upside down: the robot mounting face on the bed, no supports (`2 mm` bed chamfer) | see [PARTS.md](PARTS.md) |
| Rack foot | base down (`96 × 96 × 5 mm` 4-post flange), collar walls and `18 mm` gussets up | see [PARTS.md](PARTS.md) |
| Imager base (`112 × 193 × 45 mm`) | plate down, collar and fence up | see [PARTS.md](PARTS.md) |
| Imager light platform (`106 × 106 × 146.5 mm`) | upright, top ring up | see [PARTS.md](PARTS.md) |
| Imager camera gantry | upside down (the `z = 175` roof face on the bed) | see [PARTS.md](PARTS.md) |
| Rack rail | not printed: purchased aluminium 20 × 40 profile | — |
| Keyed dock | tines up, keys vertical | see [PARTS.md](PARTS.md) |
| Top cap | upside down (top face on bed) | see [PARTS.md](PARTS.md) |
| Imager spine | not printed: purchased TNUTZ EXM-2040 | — |

Estimated print masses are in [PARTS.md](PARTS.md). Slice the floor tile first and check it fits your printer, with brim and exclusion zones.

**Rails:** all three rails are bought, not printed: TNUTZ EXM-2040 20 × 40
aluminium extrusion [EX-TNUTZ], two cut to 370.0 mm for the racks and one cut to
344.5 mm for the imager spine, with TNUTZ ST-020-M5 T-nuts throughout. One
1,085 mm length (2 × 370 + 344.5 plus saw kerf) covers a kit. The 320 mm printed
rail export is a fit coupon for checking the shelf keys and T-nuts, not a rail.
The shelf keys and T-nut bays follow the printed copy of this profile, not a
measured extrusion: fit one shelf and one T-nut on the real rail first.

## 2. Fasteners

- **Floor-to-module attachment (Robot stand, Rack 1, Rack 2, Imager):** **16 × M5 × 20 mm
  ISO 4017 hex-head bolts + 16 × ISO 7089 M5 flat washers + 16 × ISO 4032 M5 hex nuts**
  (same **M5** size family as the 20-series rail T-nut kit). Four bolts per module
  drop upward from the floor tile underside into trapped **`8.40 mm AF × 6.2 mm`**
  hex receptacles (bolt heads sit recessed clear of the tabletop with a
  `7.8 mm` bearing web above each pocket), presenting four protruding M5 threaded
  posts above the tile top (`PCD = 100.0 mm` on each tile's 8-hole octagram at
  `(+54.0, -54.0)`). Modules drop over the four posts and tighten from above with
  an 8 mm socket or wrench.
- **Seam clamps (optional, 8 sets):** `M5 × 10 mm` ISO 4017 hex bolt, ISO 7089 washer and ISO 4032 nut per clamp. The head sits in a split underside hex pocket, the washer and nut in a split `Ø16 mm` top counterbore; the nut finishes `0.3 mm` below the tile top. Snug-tight only (untested).
- **Robot base to stand:** four M4 through-bolts + underside captured hex nut pockets
  (`±40 mm` square pattern around J1 in the stand's top plate). The RoboNine base holes are `Ø8.2 mm`, so use **M4 large washers (ISO 7093-1)** under the bolt heads or nuts on the base side. Measure the physical
  SO-ARM102 base before drilling; the CAD pattern is the RoboNine reference, not a
  measurement.
- **Foot, shelf & top-cap rail mounting (per rack):** 26 × standard 20-series M5 T-nuts +
  26 × M5 × 8 mm ISO 7380 button-head cap screws (`hardware.m5_length = 8.0`).
  Four bolts clamp the 4-sided foot collar from its front and rear 40 mm faces
  (`x = ±10 mm`), two bolts per shelf enter through `Ø11.0 mm` recessed front
  counterbores (`4.0 mm` bearing flange) into the two front slots, and two bolts
  clamp the rear of the keyed top cap into the top of the two rear slots.
- **Backlight keeper:** one M3 screw or printed clip at the rear of the C-sleeve.
- **Table anchors:** optional M5 through the corner holes, with a suitable support
  and edge distance.

## 3. Assembly

1. **Floor & hex posts.** Drop 4 × `M5 × 20 mm` hex bolts from the underside into
   the active 4 holes of each tile's 8-hole octagram (diagonal for `robot` if
   clocked `+45°`, `rack1`, and `imager`; cardinal for `rack2` — or cardinal for
   `robot` at `0°`). Verify all bolt heads sit recessed inside the `8.40 mm AF × 6.2 mm`
   hex pockets, then lay the tiles down and snap the four identical tiles top-down
   (`-Z`) into the `522.0 × 522.0 mm` 2 × 2 puzzle floor (no loose keys or pins).
   Optionally clamp each seam: before snapping a tile, stand an `M5 × 10 mm` bolt on its head in the half pocket of the tile already down at each of that seam's two positions, so the next tile drops over it (the split bore cannot hold a bolt on its own, and the heads are not reachable once the floor is down); then fit washer and nut from above (snug-tight). Fit the optional table anchors last.
2. **Bolt modules over 4 posts.** Drop `robot_stand` (clocked `+45°` to point
   `J1 = 0°` toward `rack2` for `(-45°, 0°, +45°)` workcell azimuths, or `0°` for
   `(-90°, -45°, 0°)`), `rack_foot` × 2, and `imager_base` onto their four
   protruding M5 posts. Drop an ISO 7089 M5 flat washer and ISO 4032 M5 hex nut
   onto each post and tighten with an 8 mm socket (reaching through the stand's
   `Ø17 mm` post wells on `robot_stand`). Bolt the SO-ARM102 base onto
   `robot_stand` using four M4 bolts, M4 large washers and the underside captured hex pockets.
3. **Rail.** Slide four M5 T-nuts into the front and rear slots from the rail's
   bottom end (two per 40 mm face), then drop the rail into the 4-sided foot
   collar so its two side slots run down over the collar's side T-keys until it
   seats on the base. Line the T-nuts up with the collar holes and bolt both long
   (40 mm) faces. The side keys centre the rail and stop it twisting; they never
   touch the T-nut slots.
4. **Shelves & top cap.** For each shelf from `shelf 0` upward, start two M5 × 8 mm
   button-head bolts 1–2 turns into two M5 T-nuts nestled in the mid-height bay
   (`+15.75 mm` above shelf underside) between the upper and lower T-keys, slide
   the shelf + T-nuts down the rail slots onto the tenons below, and tighten.
   Finish by pressing the keyed top cap over shelf 9 tenons and the top 14.5 mm of the rail, then tightening two M5 × 8 mm button-head bolts into rear T-nuts on the back of the cap.
5. **Imager.** With the base nuts tight, drop the imager spine into the `imager_base` collar and
   clamp it with four M5 T-nuts as in step 3. Slide the imaging dock to the
   `177.5 mm` level and bolt it with two front M5 × 8 mm button-head + T-nuts, then
   slip the camera gantry sleeve over the spine top, bolt it to the two front T-nuts, and plug in
   the USB camera.
6. **Light platform.** Last, slide the `imager_light_platform` in under the dock from the front (or a side), lifted just over the `3 mm` fence, and lower it into the fence (it is held by gravity; the dock blocks a vertical drop)
   and set the flat backlight on its rim, cable out through the rear notch.

## 4. Alignment checks

- Puzzle seams sit flush top and bottom; no rock between tiles and all four snap joints fully seated.
- Underside M5 hex bolt heads sit recessed inside their tile pockets.
- If fitted, each seam clamp nut sits below the tile top and the seam stays flush.
- M5 floor-post nuts and washers are tight on all four module flanges (`robot_stand`, `rack_foot` × 2, `imager_base`) with no rock.
- Rail upright within ~1°; lowest and highest keys fully engaged.
- Shelf pitch 31.5 mm; every shoulder seated on the one below, no gap at any tenon.
- Foot and shelf T-nut bolts tight: no play at the 4-sided foot collar or any shelf joint.
- Light platform seated flat in the base fence; backlight centred under the dish (check the projected window, not by eye).
- Dock datum `280 mm` from J1 and `177.5 mm` above the floor top.
- Camera field of view covers the full 100 mm round lid with margin.
- With every retainer fitted, the fork can reach the lowest and highest shelf.

## 5. Carrier (round dishes) and dishes

The carrier is the compact **clipped-corner (hexagonal)** plate, **140 × 100 ×
6.2 mm**, with 6.0 mm corner radii, 22/24 mm corner cuts, the four underside
grooves (at ±104/2 and ±128/2) and four finger scallops (`cad/carrier.py`).
Every round carrier keeps a **1.5 mm radial lip** (`openkit_lip = 1.5 mm`) from
the nest wall around a round backlight window that is open from below, and is
named by its base diameter via `carrier_name(p, base_diameter)` (for example
`round_carrier_87_7mm`, `round_carrier_94mm`).

The outer **140 × 100 mm** plate is sized for the largest supported round dish
envelope ([PD-ROUND-OK], `max_base_diameter = 94.0 mm`, up to 100 mm lid and
16 mm height). The **default** carrier built by `openkit_carrier(p)` and
exported as `round_carrier_87_7mm` is tuned specifically for **CELLTREAT
#229697**:

- **Snug nest (`Ø88.2 mm`) & `1.5 mm` lip (`Ø85.2 mm` window):** On the
  CELLTREAT #229697 drawing, the upper base body is `Ø88.7 mm` while the exposed
  6.8 mm skirt below the lid tapers from `~Ø87.8 mm` down to `~Ø86.4 mm` at the
  bottom face. Sizing the default `5.2 mm`-deep pocket to `base_diameter = 87.7 mm`
  with a tightened **`0.25 mm` radial clearance** (`fit_clearance = 0.25 mm`)
  gives a `Ø88.2 mm` nest (`0.25 mm`/side around the dish wall at the pocket
  rim to minimize wobble/sliding during robot transfers) and an `Ø85.2 mm`
  window (`1.5 mm` radial lip; `~0.60 mm`/side bearing overlap under the `~Ø86.4 mm`
  bottom rim so it cannot drop through).
- **Other round dishes (`round_carrier_<base>mm`):** Any round dish up to
  `Ø94.0 mm` base O.D., `Ø100.0 mm` lid O.D., and `16.0 mm` lidded height gets
  its own carrier on the same 140 × 100 mm outer and 1.5 mm lip via
  `openkit_carrier(p, base_diameter=...)` (for instance `round_carrier_94mm` is
  also exported with a `Ø94.5 mm` nest and `Ø91.5 mm` window). **Measure the
  physical base O.D. near both the bottom edge and 5 mm up before printing a
  production batch.**

Tines are **6 mm** deep (were 7) on both the robot fork and the shelf docks. That
1 mm keeps dishes up to 16 mm tall clear of the shelf above at the 5.7 mm lift
and the 31.5 mm pitch (`15.6 mm` for CELLTREAT #229697). A 6 mm printed tine
bends about 1.6× as much as a 7 mm one; the load is small, but measure the sag
of a printed shelf tine.

### Dish dimensions

| Feature | CELLTREAT #229697 (`PD-CELLTREAT-229697` drawing) | OpenKit max envelope (`[PD-ROUND-OK]`) |
| --- | --- | --- |
| Dish base O.D. | **Ø88.7 mm** upper body (`~Ø87.7 mm` at 5.2 mm nest rim, `~Ø86.4 mm` bottom rim) | **≤ Ø94.0 mm** |
| Lid O.D. | **Ø92.3 mm** (assembly view) / **Ø92.4 mm** (lid detail view) | **≤ Ø100.0 mm** |
| Overall lidded height | **15.6 mm** | **≤ 16.0 mm** |
| Base height / lid skirt | **14.1 mm** base profile height · **8.8 mm** lid skirt height | 15.0 mm max base · 6.0 mm ref skirt above seat |
| Automation & vent features | `ISO mark` target (`26.39 × 14.94 mm` offset, `6.27 mm`); `1.8` / `2.7 mm` lid vent/lug details | — |
| Nominal catalog spec | `3.63 × 3.63 × 0.61 in` (~`92.2 × 92.2 × 15.5 mm`), `15–16 mL` working vol., `55 cm²` | `100 × 15 mm` nominal class |
| Default carrier export | **`round_carrier_87_7mm`** (`Ø88.2 mm` nest, `Ø85.2 mm` window, `1.5 mm` lip) | `round_carrier_94mm` (`Ø94.5` / `Ø91.5 mm`) |
