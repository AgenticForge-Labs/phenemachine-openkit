# PheneMachine OpenKit hardware

Open hardware for a small Petri-dish imaging kit built around the SO-ARM102 arm: a four-tile floor, a robot stand,
two ten-shelf dish racks, dish carriers and a camera-over-backlight imaging station on 20 × 40 aluminium extrusion.

> **Status: concept design.** Nothing here has been printed, measured, load-tested or run on a robot. It is not an
> industrial platform. Check every fit with a test print before building a full kit.

## Contents

| Path | What it is |
| --- | --- |
| [BUILD.md](BUILD.md) | Print orientation, fasteners, assembly and alignment checks |
| [PARTS.md](PARTS.md) | Printed parts and purchased parts for one kit |
| `hardware/stl/`, `hardware/step/` | Printable and editable 3D files |
| `cad/`, `parameters/openkit.toml` | Parametric CAD source (CadQuery) |
| [REFERENCES.md](REFERENCES.md) | Sources of external dimensions |

## Rebuild the 3D files

```sh
uv run python build.py --out hardware
```

## Scope

This repository holds hardware only.

OpenKit carriers, forks and shelves are **not compatible with PheneMachine Workcell**. Workcell and assembled
OpenKit systems are sold by PheneMachine, which also offers a subscription service for running experiments.

## Licence and name

Hardware design files and CAD source are licensed under the
[CERN Open Hardware Licence Version 2 – Strongly Reciprocal](LICENSE) (CERN-OHL-S-2.0). See [NOTICE.md](NOTICE.md).

The licence does not grant any right to use the PheneMachine name or logo. Do not present your own build or
product as a PheneMachine product.
