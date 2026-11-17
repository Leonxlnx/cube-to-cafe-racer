# Shared brief for part-modelling agents

## The film
45 s, 1080p, photoreal product-ad CGI rendered in Blender 5.2 / Cycles (CPU): a plain brushed-aluminium
1.1 m cube unfolds mechanically into a cafe-racer motorcycle (no rider). Look reference images:
`E:\fable3ddemos\3ddemo4\refs\1.webp, 2.webp, 3.webp, 10.webp` (finished bike), `4.webp` (mechanism macro).
Palette: brushed / machined aluminium, orange anodised structure, black engine, black rubber, chrome accents.
The camera will go into macro close-ups of your part, so it must hold up at hero quality: real bevels on every
edge that would catch light (no razor-sharp CG edges), fasteners, machined pockets, parting lines, believable
proportions. Clean and premium, not greebled chaos. No brand names, logos or readable text anywhere.

## Conventions (read `src/common.py` first: all dimensions and hard points live there)
- Metres. X forward (bike nose), Y left, Z up, floor z = 0. Final (unfolded, ride height) pose in world coordinates.
- Materials: ONLY `mats.get("name")` from `src/mats.py` (read its docstring: brushing direction follows the object's
  local X axis, "*_turned" materials brush around the object's local Z axis, so orient OBJECTS accordingly; a lathe
  part whose axis is not Z needs its object rotated, not its mesh data). If something is truly missing, define ONE
  extra Principled material inside your module with a unique name and say so in your report.
- Helpers: `src/geo.py` (box, cyl, lathe, prism, tube, torus, bolt, join, bevel_mod, parent, polar_copies ...).
  Do not edit `common.py`, `mats.py`, `geo.py`, `studio.py`, `preview_part.py`; if a helper misbehaves, work around it
  inside your own module and report it. Write only your module file(s) under `src/parts/` (helper/test files there
  must start with your module name).
- Interface: `def build(col) -> dict`. Create every object in collection `col`. Return the dict of root empties
  described in your task. Every mesh object must be parented (keeping its world transform, `geo.parent`) to one of
  those roots. The roots sit at the specified origin/orientation. I re-parent / animate the roots, nothing else.
- Rigid objects only: no armatures, shape keys, constraints, drivers, physics, particles, geometry nodes. Bevel
  modifiers are fine, subdivision surface at most level 2. Keep the object count low (join meshes that share a
  material and never move relative to each other; instancing identical small parts with linked copies is fine).
- Blender 5.2 API reminders: there is no `mesh.use_auto_smooth` (use `geo.set_smooth(obj, angle)`), collections are
  linked explicitly, use bmesh rather than `bpy.ops` wherever possible (we run headless with `-b`).
- Do not animate anything.

## Previewing (the machine is shared with other renders: be gentle)
```
"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" -b --factory-startup --python E:\fable3ddemos\3ddemo4\src\preview_part.py -- --module parts.<yourmodule> --out E:\fable3ddemos\3ddemo4\renders\preview\<name>.png --az 35 --el 12 --res 960x540 --spp 24 --threads 4
```
Options: `--az/--el` camera azimuth/elevation in degrees (az 0 = looking from +X at the nose, 90 = from the left),
`--dist` distance factor, `--lens`, `--target x,y,z --radius r` to frame a detail, `--post "python"` (namespace:
`res` = your returned dict, `st`, `bpy`, `geo`, `mats`, `math`, `Vector`). It prints bounds and triangle count.
Use absolute output paths. Look at the PNG with the Read tool. In Git Bash use forward slashes or quote the paths.
Rules: never more than ONE Blender process of yours at a time, `--threads 4`, at most ~10 preview renders in total
(mix wide views and close-ups), never kill or touch processes you did not start. A preview takes 30-90 s.

## Working style
Fully autonomous, no questions. Budget roughly 60-75 minutes. Get a complete, correct, good-looking version
first, then spend remaining effort on realism details. Check your own renders critically against the reference
images (proportions, bevel highlights, material assignment, floating or intersecting parts, flipped normals).

## Final report (short)
Dict keys returned + what each root contains, bounding boxes, triangle count, any deviation from the spec,
known weaknesses, and the preview PNG paths worth looking at.
