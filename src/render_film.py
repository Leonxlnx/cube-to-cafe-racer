"""Film render driver (run inside Blender).

blender -b --factory-startup --python render_film.py -- --mode preview|final [--shots s01_cube,s05_rows]
        [--pass all|A|B] [--threads 8] [--force] [--frames a:b] [--out frames] [--spp N] [--only plate|frames]

final mode
  still   -> <out>/<shot>/still.png  (W*ss x H*ss, 16 bit)
  locked  -> <out>/<shot>/plate.png + reg_####.png (cropped region, fixed seed) + meta.json
             (shots flagged full=True: f_####.png full frames at W*ss x H*ss)
  moving  -> <out>/<shot>/f_####.png
  --pass A renders the even frames (and for interp=2 shots that is all that is needed for the fallback),
  --pass B the odd ones, --pass all both.
preview mode: small full frames, every 3rd frame -> <out>_prev/<shot>/f_####.png
Resumable: existing files are skipped unless --force.
"""
import sys
import os
import time
import json
import math
import argparse
import ctypes

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

import studio
import film
import shots as S

ROOT = "E:/fable3ddemos/3ddemo4"

Q = dict(
    still=dict(adaptive=0.012, min_spp=32, quality="HIGH"),
    plate=dict(spp=28, adaptive=0.03, min_spp=12, quality="BALANCED"),
    region=dict(spp=14, adaptive=0.05, min_spp=8, quality="FAST", gb=3),
    full=dict(spp=14, adaptive=0.05, min_spp=8, quality="FAST", gb=3),
    moving=dict(spp=14, adaptive=0.05, min_spp=8, quality="FAST", gb=3),
)


OVR = {}


def keep_awake():
    try:
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | 0x00000001)
    except Exception:
        pass


def set_cam(cam, c):
    studio.aim_camera(cam, c["loc"], c["tgt"], lens=c["lens"], fstop=c.get("fstop"), focus=c.get("focus"))


def configure(res, spp, threads, cfg, seed=0):
    sc = studio.setup_render(res, spp, threads, adaptive=cfg.get("adaptive"), min_spp=cfg.get("min_spp", 8),
                             denoise=True, quality=cfg.get("quality", "FAST"))
    sc.cycles.seed = seed
    if cfg.get("gb"):
        sc.cycles.glossy_bounces = cfg["gb"]
        sc.cycles.max_bounces = cfg["gb"] + 1
    if OVR.get("gb"):
        sc.cycles.glossy_bounces = OVR["gb"]
        sc.cycles.max_bounces = OVR["gb"] + 1
    if OVR.get("th") and cfg.get("adaptive") is not None and cfg is not Q["still"] and cfg is not Q["plate"]:
        sc.cycles.adaptive_threshold = OVR["th"]
    if OVR.get("dq") and cfg is not Q["still"] and cfg is not Q["plate"]:
        try:
            sc.cycles.denoising_quality = OVR["dq"]
            sc.cycles.denoising_prefilter = "ACCURATE" if OVR["dq"] == "HIGH" else "FAST"
        except Exception:
            pass
    try:
        sc.cycles.use_animated_seed = False
    except Exception:
        pass
    sc.render.image_settings.color_depth = "16"
    sc.render.image_settings.compression = 15
    sc.render.use_border = False
    sc.render.use_crop_to_border = False
    return sc


def region_of(fm, cam, shot, res, margin_px=26, floor_margin=0.55, frames=None):
    """Normalised (x0, x1, y0, y1) (y from bottom) covering everything that changes during the shot,
    including floor reflections and the contact shadow halo."""
    sc = bpy.context.scene
    objs = [o for o in list(fm.col.all_objects) + list(fm.pcol.all_objects) if o.type == "MESH"]
    xs, ys = [], []
    n = shot["n"]
    for i in (frames if frames is not None else list(range(0, n, 3)) + [n - 1]):
        fm.pose(S.frame_T(shot, i))
        bpy.context.view_layer.update()
        mn = Vector((1e9, 1e9, 1e9))
        mx = Vector((-1e9, -1e9, -1e9))
        for o in objs:
            mw = o.matrix_world
            for c in o.bound_box:
                w = mw @ Vector(c)
                for p in (w, Vector((w.x, w.y, -w.z))):
                    co = world_to_camera_view(sc, cam, p)
                    if co.z > 0:
                        xs.append(co.x)
                        ys.append(co.y)
                mn = Vector((min(mn.x, w.x), min(mn.y, w.y), min(mn.z, w.z)))
                mx = Vector((max(mx.x, w.x), max(mx.y, w.y), max(mx.z, w.z)))
        for x in (mn.x - floor_margin, mx.x + floor_margin):
            for y in (mn.y - floor_margin, mx.y + floor_margin):
                co = world_to_camera_view(sc, cam, Vector((x, y, 0.0)))
                if co.z > 0:
                    xs.append(co.x)
                    ys.append(co.y)
    w, h = res
    x0 = max(0.0, min(xs) - margin_px / w)
    x1 = min(1.0, max(xs) + margin_px / w)
    y0 = max(0.0, min(ys) - margin_px / h)
    y1 = min(1.0, max(ys) + margin_px / h)
    # snap to even pixels
    ix0 = int(math.floor(x0 * w / 2.0)) * 2
    ix1 = min(w, int(math.ceil(x1 * w / 2.0)) * 2)
    iy0 = int(math.floor(y0 * h / 2.0)) * 2
    iy1 = min(h, int(math.ceil(y1 * h / 2.0)) * 2)
    return ix0, ix1, iy0, iy1


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="preview")
    ap.add_argument("--shots", default="all")
    ap.add_argument("--pass", dest="pas", default="all")
    ap.add_argument("--threads", type=int, default=8)
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--frames", default="")
    ap.add_argument("--out", default="frames")
    ap.add_argument("--spp", type=int, default=0)
    ap.add_argument("--only", default="")
    ap.add_argument("--prev_scale", type=float, default=1.0 / 3.0)
    ap.add_argument("--prev_step", type=int, default=3)
    ap.add_argument("--prev_spp", type=int, default=6)
    ap.add_argument("--th", type=float, default=0.0)
    ap.add_argument("--gb", type=int, default=0)
    ap.add_argument("--dq", default="")
    ap.add_argument("--ao", type=int, default=0)
    ap.add_argument("--tag", default="")
    a = ap.parse_args(argv)
    keep_awake()
    OVR.update(th=a.th, gb=a.gb, dq=a.dq)
    final = a.mode == "final"
    outroot = os.path.join(ROOT, a.out if final else a.out + "_prev")
    os.makedirs(outroot, exist_ok=True)
    logf = open(os.path.join(outroot, "render_log.txt"), "a", encoding="utf-8")

    def log(msg):
        line = time.strftime("%H:%M:%S ") + msg
        print("FILM " + line)
        logf.write(line + "\n")
        logf.flush()

    t0 = time.time()
    studio.clear_scene()
    st = studio.build_open()
    fm = film.Film()
    cam = studio.add_camera(st["col"])
    if True:
        for n_ in st["floor"].data.materials[0].node_tree.nodes:
            if n_.type == "AMBIENT_OCCLUSION":
                n_.samples = a.ao or 2
    log("built scene in %.0fs  parts %s  mode %s pass %s threads %d" % (time.time() - t0, fm.parts_ok, a.mode, a.pas, a.threads))
    names = [s["name"] for s in S.SHOTS] if a.shots == "all" else a.shots.split(",")

    def want(i, shot):
        if a.pas == "A":
            return i % 2 == 0 or i == shot["n"] - 1
        if a.pas == "B":
            return i % 2 == 1 and i != shot["n"] - 1
        return True

    def exists(p):
        return (not a.force) and os.path.exists(p) and os.path.getsize(p) > 0

    def render(sc, path):
        sc.render.filepath = path
        t1 = time.time()
        bpy.ops.render.render(write_still=True)
        keep_awake()
        return time.time() - t1

    for name in names:
        shot = S.by_name(name)
        odir = os.path.join(outroot, name)
        os.makedirs(odir, exist_ok=True)
        n = shot["n"]
        f0, f1 = 0, n
        if a.frames:
            p = a.frames.split(":")
            f0, f1 = int(p[0]), min(n, int(p[1]))
        kind = shot["kind"]
        # ------------------------------------------------------------- preview
        if not final:
            res = (int(S.W * a.prev_scale / 2) * 2, int(S.H * a.prev_scale / 2) * 2)
            sc = configure(res, a.prev_spp, a.threads, dict(adaptive=None, quality="FAST"), seed=1)
            sc.render.image_settings.color_depth = "8"
            frames = [0] if kind == "still" else [i for i in range(f0, f1) if i % a.prev_step == 0 or i == n - 1]
            for i in frames:
                fn = os.path.join(odir, "f_%04d.png" % i)
                if exists(fn):
                    continue
                set_cam(cam, S.cam_at(shot, S.frame_u(shot, i)))
                fm.pose(S.frame_T(shot, i))
                dt = render(sc, fn)
                log("  prev %s %04d %.1fs" % (name, i, dt))
            continue
        # ------------------------------------------------------------- final
        ss = shot["ss"]
        res = (int(round(S.W * ss / 2)) * 2, int(round(S.H * ss / 2)) * 2)
        if kind == "still":
            fn = os.path.join(odir, "still.png")
            if exists(fn):
                log("shot %s still exists" % name)
                continue
            cfg = dict(Q["still"])
            sc = configure(res, a.spp or shot.get("spp", 96), a.threads, cfg, seed=3)
            set_cam(cam, S.cam_at(shot, 0.0))
            fm.pose(shot["T0"])
            dt = render(sc, fn)
            json.dump(dict(name=name, kind=kind, res=list(res)), open(os.path.join(odir, "meta.json"), "w"))
            log("shot %s still %dx%d %.0fs" % (name, res[0], res[1], dt))
            continue
        if shot["full"]:
            cfg = dict(Q["moving" if kind == "moving" else "full"])
            sc = configure(res, a.spp or cfg["spp"], a.threads, cfg, seed=5)
            json.dump(dict(name=name, kind=kind, res=list(res), full=True), open(os.path.join(odir, "meta.json"), "w"))
            todo = [i for i in range(f0, f1) if want(i, shot) and not exists(os.path.join(odir, "f_%04d.png" % i))]
            log("shot %s (%s full) %d frames to do of %d  res %dx%d" % (name, kind, len(todo), n, res[0], res[1]))
            for i in todo:
                set_cam(cam, S.cam_at(shot, S.frame_u(shot, i)))
                fm.pose(S.frame_T(shot, i))
                dt = render(sc, os.path.join(odir, "f_%04d.png" % i))
                log("  %s %04d %.1fs" % (name, i, dt))
            continue
        if kind == "moving":
            # moving camera: a few full plates (cross-faded in post) + per-frame region around the subject
            metap = os.path.join(odir, "meta.json")
            if os.path.exists(metap) and not a.force:
                meta = json.load(open(metap))
            else:
                regs = []
                for i in range(n):
                    set_cam(cam, S.cam_at(shot, S.frame_u(shot, i)))
                    bpy.context.view_layer.update()
                    regs.append(list(region_of(fm, cam, shot, res, frames=[i])))
                npl = shot.get("plates", 3)
                plates = sorted(set(int(round(k * (n - 1) / float(npl - 1))) for k in range(npl)))
                meta = dict(name=name, kind=kind, res=list(res), regions=regs, plates=plates)
                json.dump(meta, open(metap, "w"))
            sc = configure(res, Q["plate"]["spp"], a.threads, Q["plate"], seed=7)
            if a.only != "frames":
                for i in meta["plates"]:
                    fn = os.path.join(odir, "plate_%04d.png" % i)
                    if exists(fn):
                        continue
                    set_cam(cam, S.cam_at(shot, S.frame_u(shot, i)))
                    fm.pose(S.frame_T(shot, i))
                    dt = render(sc, fn)
                    log("shot %s plate %d %.0fs" % (name, i, dt))
            if a.only == "plate":
                continue
            cfg = dict(Q["region"])
            sc = configure(res, a.spp or cfg["spp"], a.threads, cfg, seed=7)
            sc.render.use_border = True
            sc.render.use_crop_to_border = True
            todo = [i for i in range(f0, f1) if want(i, shot) and not exists(os.path.join(odir, "reg_%04d%s.png" % (i, a.tag)))]
            fr = sum((r[1] - r[0]) * (r[3] - r[2]) for r in meta["regions"]) / float(len(meta["regions"]) * res[0] * res[1])
            log("shot %s (moving, region) %d frames to do of %d  mean region %.0f%%" % (name, len(todo), n, fr * 100))
            for i in todo:
                ix0, ix1, iy0, iy1 = meta["regions"][i]
                sc.render.border_min_x = ix0 / res[0]
                sc.render.border_max_x = ix1 / res[0]
                sc.render.border_min_y = iy0 / res[1]
                sc.render.border_max_y = iy1 / res[1]
                set_cam(cam, S.cam_at(shot, S.frame_u(shot, i)))
                fm.pose(S.frame_T(shot, i))
                dt = render(sc, os.path.join(odir, "reg_%04d%s.png" % (i, a.tag)))
                log("  %s %04d %.1fs" % (name, i, dt))
            sc.render.use_border = False
            sc.render.use_crop_to_border = False
            continue
        # locked: plate + region
        set_cam(cam, S.cam_at(shot, 0.0))
        bpy.context.view_layer.update()
        sc = configure(res, Q["plate"]["spp"], a.threads, Q["plate"], seed=7)
        metap = os.path.join(odir, "meta.json")
        if os.path.exists(metap) and not a.force:
            meta = json.load(open(metap))
            ix0, ix1, iy0, iy1 = meta["region"]
        else:
            ix0, ix1, iy0, iy1 = region_of(fm, cam, shot, res)
            meta = dict(name=name, kind=kind, res=list(res), region=[ix0, ix1, iy0, iy1])
            json.dump(meta, open(metap, "w"))
        frac = (ix1 - ix0) * (iy1 - iy0) / float(res[0] * res[1])
        if frac > 0.90:
            meta["full"] = True
            json.dump(meta, open(metap, "w"))
            cfg = dict(Q["full"])
            sc = configure(res, a.spp or cfg["spp"], a.threads, cfg, seed=7)
            todo = [i for i in range(f0, f1) if want(i, shot) and not exists(os.path.join(odir, "f_%04d.png" % i))]
            log("shot %s (locked, full frame: region %.0f%%) %d frames to do of %d" % (name, frac * 100, len(todo), n))
            for i in todo:
                fm.pose(S.frame_T(shot, i))
                dt = render(sc, os.path.join(odir, "f_%04d.png" % i))
                log("  %s %04d %.1fs" % (name, i, dt))
            continue
        plate = os.path.join(odir, "plate.png")
        if a.only != "frames" and not exists(plate):
            fm.pose(shot["T0"])
            dt = render(sc, plate)
            log("shot %s plate %.0fs  region %s = %.0f%% of frame" % (name, dt, [ix0, ix1, iy0, iy1], frac * 100))
        if a.only == "plate":
            continue
        cfg = dict(Q["region"])
        sc = configure(res, a.spp or cfg["spp"], a.threads, cfg, seed=7)
        sc.render.use_border = True
        sc.render.use_crop_to_border = True
        sc.render.border_min_x = ix0 / res[0]
        sc.render.border_max_x = ix1 / res[0]
        sc.render.border_min_y = iy0 / res[1]
        sc.render.border_max_y = iy1 / res[1]
        todo = [i for i in range(f0, f1) if want(i, shot) and not exists(os.path.join(odir, "reg_%04d%s.png" % (i, a.tag)))]
        log("shot %s (locked) %d frames to do of %d  region %.0f%%" % (name, len(todo), n, frac * 100))
        for i in todo:
            fm.pose(S.frame_T(shot, i))
            dt = render(sc, os.path.join(odir, "reg_%04d%s.png" % (i, a.tag)))
            log("  %s %04d %.1fs" % (name, i, dt))
        sc.render.use_border = False
        sc.render.use_crop_to_border = False
    log("ALL DONE (%s, pass %s) in %.0f min" % (a.mode, a.pas, (time.time() - t0) / 60.0))


if __name__ == "__main__":
    main()
