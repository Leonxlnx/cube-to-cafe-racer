"""Render stills of the film rig at given animation times.

blender -b --factory-startup --python render_stills.py -- --times 0,6,8 --cam "az,el,dist,tx,ty,tz,lens"
        --out E:/.../renders/preview/anim --res 640x360 --spp 12 --threads 6 [--engine cycles|workbench]
        [--noparts] [--sheet 4]   (writes <out>_000.png ... and <out>_sheet.png via ffmpeg if --sheet N cols)
"""
import sys
import os
import math
import time
import argparse
import subprocess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
from mathutils import Vector

import studio
import film


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--times", default="0")
    ap.add_argument("--cam", default="35,14,5.2,0,0,0.5,50")
    ap.add_argument("--out", required=True)
    ap.add_argument("--res", default="640x360")
    ap.add_argument("--spp", type=int, default=12)
    ap.add_argument("--threads", type=int, default=6)
    ap.add_argument("--engine", default="cycles")
    ap.add_argument("--noparts", action="store_true")
    ap.add_argument("--sheet", type=int, default=0)
    ap.add_argument("--py", default="")
    ap.add_argument("--quality", default="FAST")
    ap.add_argument("--fstop", type=float, default=0.0)
    a = ap.parse_args(argv)

    t0 = time.time()
    studio.clear_scene()
    st = studio.build_open()
    fm = film.Film(use_parts=not a.noparts)
    print("STILLS built in %.1fs  parts %s" % (time.time() - t0, fm.parts_ok))
    w, h = [int(v) for v in a.res.split("x")]
    cam = studio.add_camera(st["col"])
    sc = studio.setup_render((w, h), a.spp, a.threads, adaptive=0.05, min_spp=6, quality=a.quality)
    if a.engine == "workbench":
        sc.render.engine = "BLENDER_WORKBENCH"
        sc.display.shading.light = "STUDIO"
        sc.display.shading.color_type = "MATERIAL"
        sc.display.shading.show_cavity = True
        sc.display.shading.show_shadows = False
        sc.view_settings.view_transform = "Standard"
    if a.py:
        code = open(a.py, encoding="utf-8").read() if a.py.endswith(".py") else a.py
        exec(code, {"st": st, "fm": fm, "bpy": bpy, "film": film, "studio": studio, "sc": sc, "math": math, "Vector": Vector})
    cams = [c for c in a.cam.split(";") if c]
    times = [float(v) for v in a.times.split(",")]
    k = 0
    for ci, cs in enumerate(cams):
        az, el, dist, tx, ty, tz, lens = [float(v) for v in cs.split(",")]
        azr, elr = math.radians(az), math.radians(el)
        tgt = Vector((tx, ty, tz))
        loc = tgt + Vector((math.cos(azr) * math.cos(elr), math.sin(azr) * math.cos(elr), math.sin(elr))) * dist
        studio.aim_camera(cam, loc, tgt, lens=lens, fstop=(a.fstop or None))
        for T in times:
            fm.pose(T)
            sc.render.filepath = "%s_%03d.png" % (a.out, k)
            t1 = time.time()
            c1 = time.process_time()
            bpy.ops.render.render(write_still=True)
            print("STILLS %03d cam %d T=%.2f  %.1fs  cpu %.0fs" % (k, ci, T, time.time() - t1, time.process_time() - c1))
            k += 1
    if a.sheet and k > 1:
        rows = (k + a.sheet - 1) // a.sheet
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-start_number", "0", "-i", a.out + "_%03d.png",
                        "-vf", "tile=%dx%d" % (a.sheet, rows), "-frames:v", "1", a.out + "_sheet.png"])
        print("STILLS sheet", a.out + "_sheet.png")


if __name__ == "__main__":
    main()
