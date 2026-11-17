# Stackable checker S40: CAD and print files

Copyright (c) 2026 Tico Hannan. BSD-3-Clause.

The S40 piece as built in SolidWorks: Ø25 mm, 22.13 mm tall, a 12 mm grip band, a 40° cone on a 2 mm
flat shoulder with a Ø4 flat tip, and a matching 40° socket underneath (mouth Ø22.0, 0.5 mm radial
clearance). Two pieces stack with a 12 mm pitch; a king is 34.13 mm tall. Volume 5,659.463 mm³,
about 7.0 g of solid PLA.

| File | What it is | Open with |
| --- | --- | --- |
| `checker_s40.step` | Solid (AP203), mm. Volume 5,659.463 mm³, 25.00 × 25.00 × 22.13 mm | Any CAD program (Fusion, Onshape, FreeCAD, SolidWorks) |
| `checker_s40.SLDPRT` | Native SolidWorks part. Every size is a Global Variable (Tools > Equations): `D`, `HC`, `SH`, `TIP_D`, `CONE_H`, `CLR`, `VCLR`, `CT`, `CB`; `SLOPE`, `MOUTH_D`, `SOCK_H` and `ROOF_D` are equations | SolidWorks 2025 or newer (older versions cannot open it) |
| `checker_s40.3mf` | One piece, already standing upright with the socket on the bed | Bambu Studio (or any slicer) |
| `checker_s40_plate2.3mf` | Two pieces on one plate, 10 mm apart | Bambu Studio |
| `checker_s40_partspec.json` | The parameters the SolidWorks part was built from (one revolve for the body, one revolve cut for the socket) | Text editor; same numbers as `stackpieces.export.partspec` for preset `S40` |
| `preview.png` | Picture of the part | Any image viewer |

The cone angle is set through `CONE_H` (10.13 mm gives 40° with `SH` = 2 and `TIP_D` = 4).

## Printing (Bambu X2D, PLA)

1. Open `checker_s40_plate2.3mf` (or `checker_s40.3mf`) in Bambu Studio. The pieces are already
   oriented: bottom face with the socket on the bed, cone tip up.
2. Choose PLA, 0.4 mm nozzle, 0.2 mm layers (Tico's preset `kit 0.20 functional`), **no supports**.
   The socket wall leans 40° from vertical and the socket roof is a Ø4.2 mm bridge, both printable
   without support.
3. Only the 1.1 mm rim touches the bed (about 80 mm²). Add a brim if the pieces lift.
4. Check the first pieces:
   - band diameter at mid-height: 25.0 mm (round outer shapes may print about 0.2 mm small);
   - socket mouth: 22.0 mm;
   - stack two pieces by hand: the top one should sit flat on the shoulder without rocking, and the
     pair should be 34.1 ± 0.3 mm tall.
5. If a piece binds or sits high on another, the socket printed small: open `CLR` by 0.2 mm in
   SolidWorks (or in `checker_s40_partspec.json`) and reprint.

The simulations (`../../README.md`) suggest lightly sanding the cone of some pieces to compare: the
printed layer ridges on the cone and in the socket catch on each other and shorten the self-centering
range.
