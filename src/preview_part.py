"""Render a quick studio preview of one part module.

"C:\\Program Files\\Blender Foundation\\Blender 5.2\\blender.exe" -b --factory-startup ^
    --python E:\\fable3ddemos\\3ddemo4\\src\\preview_part.py -- ^
    --module parts.engine --out E:\\fable3ddemos\\3ddemo4\\renders\\preview\\engine_a.png ^
    --az 35 --el 12 [--dist 1.0] [--lens 70] [--res 960x540] [--spp 24] [--threads 4]
    [--target x,y,z] [--radius r] [--fn build] [--kw '{"a": 1}'] [--post "python code"]

The module must expose  build(col) -> dict  (or the function named by --fn).  The camera looks at
the bounding-box centre of everything the module created (or --target) from azimuth --az
(degrees, 0 = from +X i.e. looking at the bike nose, 90 = from +Y the bike's left side) and
elevation --el.  Prints bounds and triangle count.  ABSOLUTE output paths only.
"""
import sys
import os
import math
import json
import time
import argparse
import importlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
from mathutils import Vector

import geo
import mats
import studio


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--module", required=True)
    ap.add_argument("--fn", default="build")
    ap.add_argument("--kw", default="{}")
    ap.add_argument("--out", required=True)
    ap.add_argument("--az", type=float, default=35.0)
    ap.add_argument("--el", type=float, default=12.0)
    ap.add_argument("--dist", type=float, default=1.0)
    ap.add_argument("--lens", type=float, default=70.0)
    ap.add_argument("--res", default="960x540")
    ap.add_argument("--spp", type=int, default=24)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--target", default="")
    ap.add_argument("--radius", type=float, default=0.0)
    ap.add_argument("--post", default="")
    ap.add_argument("--nofloor", action="store_true")
    a = ap.parse_args(argv)

    t0 = time.time()
    studio.clear_scene()
    st = studio.build_open()
    if a.nofloor:
        st["cove"].hide_render = True
    col = studio.new_collection("part")
    geo.DEFAULT_COL = col
    mod = importlib.import_module(a.module)
    res = getattr(mod, a.fn)(col, **json.loads(a.kw))
    if a.post:
        exec(a.post, {"res": res, "st": st, "studio": studio, "bpy": bpy, "geo": geo, "mats": mats, "math": math, "Vector": Vector})
    bpy.context.view_layer.update()
    objs = [o for o in col.all_objects if o.type == "MESH"]
    mn, mx = geo.bounds(objs)
    print("PREVIEW bounds min (%.3f %.3f %.3f) max (%.3f %.3f %.3f)  objects %d  tris %d  build %.1fs" % (
        mn.x, mn.y, mn.z, mx.x, mx.y, mx.z, len(objs), geo.tri_count(objs), time.time() - t0))
    ctr = (mn + mx) / 2
    rad = (mx - mn).length / 2
    if a.target:
        ctr = Vector([float(v) for v in a.target.split(",")])
    if a.radius > 0:
        rad = a.radius
    w, h = [int(v) for v in a.res.split("x")]
    cam = studio.add_camera(st["col"], lens=a.lens)
    fov = 2 * math.atan(18.0 / a.lens * (h / w) * 1.0)       # vertical fov (sensor 36 wide)
    d = rad / math.sin(min(fov, 2 * math.atan(18.0 / a.lens)) / 2) * 1.08 * a.dist
    az, el = math.radians(a.az), math.radians(a.el)
    loc = ctr + Vector((math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el))) * d
    if loc.z < 0.03:
        loc.z = 0.03
    studio.aim_camera(cam, loc, ctr)
    sc = studio.setup_render((w, h), a.spp, a.threads, adaptive=0.05, min_spp=8, quality="FAST")
    sc.render.filepath = a.out
    t1 = time.time()
    bpy.ops.render.render(write_still=True)
    print("PREVIEW wrote %s  render %.1fs" % (a.out, time.time() - t1))


if __name__ == "__main__":
    main()
