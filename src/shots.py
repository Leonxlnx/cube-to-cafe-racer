"""The edit: shots, cameras, time mapping.  Film: 45 s, 30 fps, 1350 frames.

Cuts sit on the beat grid of the music ("Punctus XII", 112 BPM, track start at film 4.497 s,
beats every 0.536 s from 4.63 s, big hit at film 25.00 s).

kinds
  still   static scene at time T0, one high-res render, 2D move in post (zoom / pan / roll)
  locked  locked camera, scene animates (T0 -> T1 over the shot); plate + region render; 2D push in post
  moving  true camera move; a few plates + per-frame region render
"""
import math

FPS = 30
W, H = 1920, 1080
TOTAL = 45.0


def orbit(az, el, dist, tgt):
    a, e = math.radians(az), math.radians(el)
    return (tgt[0] + math.cos(a) * math.cos(e) * dist, tgt[1] + math.sin(a) * math.cos(e) * dist, tgt[2] + math.sin(e) * dist)


def cam_orbit(az, el, dist, tgt, lens, fstop=None, focus=None):
    return dict(loc=orbit(az, el, dist, tgt), tgt=tgt, lens=lens, fstop=fstop, focus=focus)


def cam_free(loc, tgt, lens, fstop=None, focus=None):
    return dict(loc=loc, tgt=tgt, lens=lens, fstop=fstop, focus=focus)


def lerp(a, b, u):
    return a + (b - a) * u


def smooth(u):
    return u * u * (3 - 2 * u)


END = 25.0      # animation time of the finished bike (timeline.TL["end"])
OFF = 0.4       # film time = T - OFF during the transformation


def L(name, t, cam, move=(1.00, 1.05, 0, 0, 0), **kw):
    """Locked shot whose animation time runs 1:1 with film time (T = t + OFF)."""
    d = dict(name=name, t=t, kind="locked", cam=cam, move=move)
    d.update(kw)
    return d


SHOTS = [
    dict(name="s01_cube", t=0.0, kind="moving", T0=0.0, T1=0.0,
         cam=lambda u: cam_orbit(lerp(42, 35, u), lerp(11.5, 10.5, u), lerp(6.7, 5.9, u), (0, 0, 0.50), 70)),
    dict(name="s02_edge", t=2.3, kind="still", T0=0.0, cam=cam_free((1.55, 1.25, 1.36), (0.52, 0.30, 1.07), 95, 2.8),
         move=(1.10, 1.10, 0.05, -0.012, 0.0), ss=1.25, spp=96),
    L("s03a_seams", 4.00, cam_free((1.08, 0.98, 0.99), (0.55, 0.47, 0.87), 100, 3.2), move=(1.00, 1.04, 0, 0, 0),
      full=True, ss=0.75),
    L("s03_breath", 5.17, cam_orbit(32, 14, 5.6, (0.10, 0, 0.50), 60)),
    L("s04_doors", 6.24, cam_orbit(-62, 66, 7.4, (0, 0, 0.55), 50), move=(1.00, 1.05, 0, 0, 1.5)),
    L("s05_rows", 8.39, cam_orbit(78, 6, 5.4, (0, 0, 0.52), 60), move=(1.04, 1.00, 0, 0, 0)),
    L("s06_rear", 10.53, cam_orbit(203, 10, 5.0, (-0.42, 0, 0.45), 55)),
    L("s06b_lock", 12.67, cam_free((-1.28, 0.98, 0.50), (-0.60, 0.13, 0.35), 90, 4.0), move=(1.00, 1.03, 0, 0, 0),
      full=True, ss=0.75),
    L("s07_front", 13.21, cam_orbit(27, 7, 4.5, (0.36, 0, 0.50), 60)),
    L("s08_touch", 15.35, cam_free((1.62, 0.92, 0.13), (0.70, 0.0, 0.17), 75, 4.0), move=(1.00, 1.03, 0, 0, 0),
      full=True, ss=0.75),
    L("s09_rise", 16.43, cam_orbit(90, 4, 6.2, (-0.05, 0, 0.54), 70), move=(1.00, 1.04, 0, 0, 0)),
    L("s10_tank", 17.50, cam_orbit(138, 30, 5.0, (-0.28, 0, 0.78), 60)),
    L("s11_cap", 19.64, cam_free((0.52, 0.58, 1.36), (0.04, 0.0, 0.93), 85, 4.0), move=(1.00, 1.04, 0, 0, 0),
      full=True, ss=0.75),
    L("s12_keel", 21.25, cam_orbit(-36, 5, 4.6, (0.0, 0, 0.36), 60)),
    L("s13_light", 23.39, cam_orbit(-16, 4, 4.4, (0.42, 0, 0.64), 85), move=(1.00, 1.04, 0, 0, 0)),
    dict(name="s14_hero", t=25.00, kind="moving", T0=END, T1=END,
         cam=lambda u: cam_orbit(lerp(-58, -27, smooth(u)), lerp(11, 8.5, u), lerp(5.4, 5.05, u), (-0.03, 0, 0.50), 60)),
    dict(name="s15_engine", t=29.29, kind="still", T0=END, cam=cam_free((0.80, -1.35, 0.60), (0.04, -0.10, 0.42), 85, 4.0),
         move=(1.08, 1.08, 0.06, 0.0, 0.0), ss=1.25, spp=96),
    dict(name="s16_rearhub", t=31.43, kind="still", T0=END, cam=cam_free((-1.62, -1.12, 0.40), (-0.74, -0.06, 0.33), 100, 4.0),
         move=(1.08, 1.08, -0.05, 0.0, 0.0), ss=1.25, spp=96),
    dict(name="s17_top", t=33.58, kind="still", T0=END, cam=cam_free((-0.05, -0.002, 7.6), (-0.05, 0, 0.5), 85),
         move=(1.22, 1.34, 0.0, 0.0, 7.0), ss=1.4, spp=96),
    dict(name="s18_wheel", t=35.72, kind="still", T0=END, cam=cam_free((1.55, 0.95, 0.44), (0.70, 0.05, 0.37), 100, 4.0),
         move=(1.08, 1.08, 0.0, 0.05, 0.0), ss=1.25, spp=96),
    dict(name="s19_cockpit", t=37.86, kind="still", T0=END, cam=cam_free((-0.47, 0.64, 1.384), (0.38, 0.03, 0.965), 75, 4.0),
         move=(1.08, 1.08, 0.04, 0.0, 0.0), ss=1.25, spp=96),
    dict(name="s19b_front34", t=40.01, kind="still", T0=END, cam=cam_orbit(31, 4.5, 5.15, (0.02, 0, 0.52), 70),
         move=(1.08, 1.20, 0.0, 0.0, 0.0), ss=1.25, spp=96),
    dict(name="s20_final", t=42.15, kind="still", T0=END, cam=cam_orbit(270, 3.5, 7.2, (-0.05, 0, 0.53), 85),
         move=(1.10, 1.00, 0.0, 0.0, 0.0), ss=1.25, spp=128),
]

ANIM_SS = 0.8334      # animated shots render at 1600x900 and are upscaled in post (render-time budget)

for i, s in enumerate(SHOTS):
    if s["kind"] in ("locked", "moving") and "ss" not in s:
        s["ss"] = ANIM_SS
    t1 = SHOTS[i + 1]["t"] if i + 1 < len(SHOTS) else TOTAL
    s["f0"] = int(round(s["t"] * FPS))
    s["f1"] = int(round(t1 * FPS))
    s["n"] = s["f1"] - s["f0"]
    if s["kind"] == "locked":
        s["T0"] = s["f0"] / FPS + OFF
        s["T1"] = s["f1"] / FPS + OFF
    s.setdefault("T1", s["T0"])
    s.setdefault("move", (1, 1, 0, 0, 0))
    s.setdefault("ss", 1.0)
    s.setdefault("interp", 1)
    s.setdefault("full", False)


# Optional upgrades: true camera moves that replace a still shot when their frames exist (see post.py).
ALT = [
    dict(name="s20m_final", base="s20_final", kind="moving", T0=END, T1=END,
         cam=lambda u: cam_orbit(lerp(263.0, 275.0, u), 3.5, lerp(7.7, 7.15, u), (-0.05, 0, 0.53), 85)),
    dict(name="s19m_front34", base="s19b_front34", kind="moving", T0=END, T1=END,
         cam=lambda u: cam_orbit(lerp(37.0, 27.0, u), 4.5, lerp(5.25, 4.95, u), (0.02, 0, 0.52), 70)),
]
for _a in ALT:
    _b = [x for x in SHOTS if x["name"] == _a["base"]][0]
    for _k in ("t", "f0", "f1", "n"):
        _a[_k] = _b[_k]
    _a.setdefault("move", (1, 1, 0, 0, 0))
    _a.setdefault("ss", ANIM_SS)
    _a.setdefault("interp", 1)
    _a.setdefault("full", False)


def by_name(name):
    for s in SHOTS + ALT:
        if s["name"] == name:
            return s
    raise KeyError(name)


def frame_u(shot, i):
    return i / max(shot["n"] - 1, 1)


def frame_T(shot, i):
    return shot["T0"] + (shot["T1"] - shot["T0"]) * (i / max(shot["n"], 1))


def cam_at(shot, u):
    c = shot["cam"]
    return c(u) if callable(c) else c


def shot_of_frame(n):
    for s in SHOTS:
        if s["f0"] <= n < s["f1"]:
            return s, n - s["f0"]
    return SHOTS[-1], SHOTS[-1]["n"] - 1
