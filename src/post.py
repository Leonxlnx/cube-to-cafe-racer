"""Assemble the film from the rendered shots (system python3 with numpy + OpenCV).

python3 post.py --src frames --out out/film.mp4 [--size 1920x1080] [--prev] [--audio audio/mix.wav]
                [--range 0:1350] [--sheet out/sheet.png] [--nograde]
--prev : sources are small full frames from `render_film.py --mode preview` (missing frames are held).
"""
import os
import sys
import json
import math
import argparse
import subprocess

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shots as S

ROOT = "E:/fable3ddemos/3ddemo4"
cv2.setNumThreads(3)


def imread(p):
    im = cv2.imread(p, cv2.IMREAD_UNCHANGED)
    if im is None:
        return None
    if im.ndim == 2:
        im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    if im.shape[2] == 4:
        im = im[:, :, :3]
    if im.dtype == np.uint16:
        return im.astype(np.float32) / 65535.0
    return im.astype(np.float32) / 255.0


class ShotSrc:
    def __init__(self, shot, src, prev):
        self.shot = shot
        self.dir = os.path.join(src, shot["name"])
        self.prev = prev
        self.cache = {}
        self.meta = None
        mp = os.path.join(self.dir, "meta.json")
        if os.path.exists(mp):
            self.meta = json.load(open(mp))
        self.plate = None
        self.mask = None
        self.still = None
        self.kind = shot["kind"]
        self.full = prev or shot["full"] or bool((self.meta or {}).get("full", False))
        self.plates = {}
        self.avail = None

    def _available(self):
        if self.avail is None:
            pre = "f_" if self.full else "reg_"
            self.avail = sorted(int(f[len(pre):len(pre) + 4]) for f in os.listdir(self.dir)
                                if f.startswith(pre) and f.endswith(".png")) if os.path.isdir(self.dir) else []
        return self.avail

    def _raw(self, i):
        """Rendered frame i (composited over the plate if needed) or None."""
        if i in self.cache:
            return self.cache[i]
        if self.full:
            im = imread(os.path.join(self.dir, "f_%04d.png" % i))
        else:
            reg = imread(os.path.join(self.dir, "reg_%04d.png" % i))
            if reg is None:
                im = None
            else:
                if "regions" in self.meta:                    # moving camera: cross-faded plates
                    x0, x1, y0, y1 = self.meta["regions"][i]
                    pl = self.meta["plates"]
                    lo = max([k for k in pl if k <= i] or [pl[0]])
                    hi = min([k for k in pl if k >= i] or [pl[-1]])
                    for k in (lo, hi):
                        if k not in self.plates:
                            self.plates[k] = imread(os.path.join(self.dir, "plate_%04d.png" % k))
                    if hi == lo:
                        base = self.plates[lo].copy()
                    else:
                        w_ = (i - lo) / float(hi - lo)
                        base = self.plates[lo] * (1 - w_) + self.plates[hi] * w_
                else:
                    if self.plate is None:
                        self.plate = imread(os.path.join(self.dir, "plate.png"))
                    x0, x1, y0, y1 = self.meta["region"]
                    base = self.plate.copy()
                hh, ww = base.shape[:2]
                key = (x0, x1, y0, y1)
                if self.mask is None or self.mask[0] != key:
                    m = np.ones((y1 - y0, x1 - x0), np.float32)
                    fe = 22
                    ramp = np.linspace(0.0, 1.0, fe, dtype=np.float32)
                    if x0 > 0:
                        m[:, :fe] *= ramp[None, :]
                    if x1 < ww:
                        m[:, -fe:] *= ramp[::-1][None, :]
                    if y1 < hh:          # top edge of the region (rows start at the top)
                        m[:fe, :] *= ramp[:, None]
                    if y0 > 0:
                        m[-fe:, :] *= ramp[::-1][:, None]
                    self.mask = (key, m[:, :, None])
                mk = self.mask[1]
                r0, r1 = hh - y1, hh - y0
                if reg.shape[0] != (r1 - r0) or reg.shape[1] != (x1 - x0):
                    reg = cv2.resize(reg, (x1 - x0, r1 - r0), interpolation=cv2.INTER_LINEAR)
                im = base
                im[r0:r1, x0:x1] = im[r0:r1, x0:x1] * (1 - mk) + reg * mk
        if len(self.cache) > 6:
            self.cache.pop(next(iter(self.cache)))
        self.cache[i] = im
        return im

    def frame(self, i):
        if self.kind == "still":
            if self.still is None:
                self.still = imread(os.path.join(self.dir, "f_0000.png" if self.prev else "still.png"))
            return self.still
        av = self._available()
        if not av:
            return None
        if i in av:
            return self._raw(i)
        lo = [k for k in av if k < i]
        hi = [k for k in av if k > i]
        if not lo:
            return self._raw(hi[0])
        if not hi:
            return self._raw(lo[-1])
        a, b = lo[-1], hi[0]
        if self.prev or (b - a) > 4:
            return self._raw(a if (i - a) <= (b - i) else b)
        return flow_interp(self._raw(a), self._raw(b), (i - a) / float(b - a))


_dis = None


def flow_interp(a, b, t):
    """In-between of two rendered frames: motion compensated where the optical flow is trustworthy
    (forward / backward flows agree), plain cross-blend elsewhere (reads as motion blur instead of warping)."""
    global _dis
    if _dis is None:
        _dis = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM)
    ga = cv2.cvtColor((np.clip(a, 0, 1) * 255).astype(np.uint8), cv2.COLOR_BGR2GRAY)
    gb = cv2.cvtColor((np.clip(b, 0, 1) * 255).astype(np.uint8), cv2.COLOR_BGR2GRAY)
    fab = _dis.calc(ga, gb, None)
    fba = _dis.calc(gb, ga, None)
    h, w = ga.shape
    gx, gy = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    wa = cv2.remap(a, gx - t * fab[..., 0], gy - t * fab[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    wb = cv2.remap(b, gx - (1 - t) * fba[..., 0], gy - (1 - t) * fba[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    mc = wa * (1 - t) + wb * t
    # consistency: follow a->b then b->a, should come back
    bx = cv2.remap(fba[..., 0], gx + fab[..., 0], gy + fab[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    by = cv2.remap(fba[..., 1], gx + fab[..., 0], gy + fab[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    err = np.sqrt((fab[..., 0] + bx) ** 2 + (fab[..., 1] + by) ** 2)
    mag = np.sqrt(fab[..., 0] ** 2 + fab[..., 1] ** 2)
    # also distrust places where the two warped images disagree
    diff = np.abs(wa - wb).max(axis=2)
    bad = np.clip((err - 1.0) / 2.0, 0, 1)
    bad = np.maximum(bad, np.clip((diff - 0.06) / 0.10, 0, 1))
    bad = np.maximum(bad, np.clip((mag - 28.0) / 12.0, 0, 1))
    bad = cv2.GaussianBlur(cv2.dilate(bad, np.ones((9, 9), np.uint8)), (0, 0), 6.0)[:, :, None]
    blend = a * (1 - t) + b * t
    return mc * (1 - bad) + blend * bad


def move2d(im, shot, u, W, H):
    z0, z1, px, py, roll = shot["move"]
    sh, sw = im.shape[:2]
    z = z0 + (z1 - z0) * u
    cx = sw / 2.0 + px * (u - 0.5) * sw
    cy = sh / 2.0 + py * (u - 0.5) * sh
    s = (W / float(sw)) * z
    ang = roll * (u - 0.5)
    M = cv2.getRotationMatrix2D((cx, cy), ang, s)
    M[0, 2] += W / 2.0 - cx
    M[1, 2] += H / 2.0 - cy
    if abs(s - 1.0) < 1e-6 and abs(ang) < 1e-9 and abs(px) < 1e-9 and abs(py) < 1e-9 and sw == W:
        return im
    if s < 0.75:
        k = max(1, int(round(0.5 / s)) * 2 + 1)
        im = cv2.GaussianBlur(im, (k, k), 0.45 / s)
    out = cv2.warpAffine(im, M, (W, H), flags=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_REPLICATE)
    if s > 1.06:           # upscaled material: restore some edge contrast
        blur = cv2.GaussianBlur(out, (0, 0), 1.1)
        out = np.clip(out + (out - blur) * min(0.55, 0.9 * (s - 1.0) + 0.25), 0.0, 1.0)
    return out


_vig = {}
_rng = np.random.default_rng(7)


def grade(im, W, H, n, total, nograde=False):
    im = np.clip(im, 0.0, 1.0)
    if not nograde:
        # soft glow on the brightest parts
        small = cv2.resize(im, (W // 4, H // 4), interpolation=cv2.INTER_AREA)
        hi = np.clip(small - 0.80, 0, 1) * 2.2
        g = cv2.GaussianBlur(hi, (0, 0), max(1.0, W / 1920.0 * 5.0))
        g = cv2.resize(g, (W, H), interpolation=cv2.INTER_LINEAR)
        im = 1.0 - (1.0 - im) * (1.0 - 0.22 * g)
        # gentle vignette
        key = (W, H)
        if key not in _vig:
            yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
            r2 = ((xx - W / 2) / (W / 2)) ** 2 + ((yy - H / 2) / (H / 2)) ** 2
            _vig[key] = (1.0 - 0.055 * np.clip(r2 - 0.25, 0, 2) ** 1.2)[:, :, None]
        im = im * _vig[key]
        # fine grain (also dithers the smooth backdrop)
        gr = _rng.normal(0.0, 0.0055, (H, W, 1)).astype(np.float32)
        im = im + gr * (0.6 + 0.8 * (1.0 - im))
    # fades to / from white
    fi = 10
    fo = 22
    if n < fi:
        k = 1.0 - n / float(fi)
        im = im + (1.0 - im) * (k * k)
    if n >= total - fo:
        k = (n - (total - fo)) / float(fo - 1)
        im = im + (1.0 - im) * (k * k * (3 - 2 * k))
    return np.clip(im, 0.0, 1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="frames")
    ap.add_argument("--out", default="out/film.mp4")
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--prev", action="store_true")
    ap.add_argument("--audio", default="")
    ap.add_argument("--range", default="")
    ap.add_argument("--sheet", default="")
    ap.add_argument("--nograde", action="store_true")
    ap.add_argument("--crf", type=int, default=14)
    ap.add_argument("--dump", default="", help="comma list of film frame numbers to also save as PNG (out/qa_####.png)")
    ap.add_argument("--only_dump", action="store_true", help="process only the --dump frames, write no video")
    a = ap.parse_args()
    W, H = [int(v) for v in a.size.split("x")]
    src = os.path.join(ROOT, a.src)
    out = os.path.join(ROOT, a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    total = int(round(S.TOTAL * S.FPS))
    n0, n1 = 0, total
    if a.range:
        p = a.range.split(":")
        n0, n1 = int(p[0]), int(p[1])
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-threads", "4", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", "%dx%d" % (W, H),
           "-r", str(S.FPS), "-i", "-"]
    if a.audio:
        cmd += ["-i", os.path.join(ROOT, a.audio)]
    cmd += ["-vf", "scale=out_color_matrix=bt709:out_range=tv", "-c:v", "libx264", "-threads", "4", "-preset", "slow", "-crf", str(a.crf),
            "-pix_fmt", "yuv420p", "-colorspace", "bt709", "-color_primaries", "bt709", "-color_trc", "bt709",
            "-color_range", "tv", "-movflags", "+faststart"]
    if a.audio:
        cmd += ["-c:a", "aac", "-b:a", "256k", "-shortest"]
    cmd += [out]
    dump = set(int(v) for v in a.dump.split(",") if v)
    ff = None if a.only_dump else subprocess.Popen(cmd, stdin=subprocess.PIPE)
    # tiny camera shake on the heavy mechanical impacts (timing from the audio cue sheet)
    shakes = []
    try:
        cj = json.load(open(os.path.join(ROOT, "audio", "cues.json")))
        for c in cj["cues"]:
            if c["kind"] == "clack_lock" and c.get("gain_db", -9) >= -3:
                shakes.append((c["t"], 2.6))
            elif c["kind"] == "suspension":
                shakes.append((c["t"], 4.0))
            elif c["kind"] == "impact_low" and "slaps" in c.get("desc", ""):
                shakes.append((c["t"], 3.0))
    except Exception as e:
        print("POST no shake:", e)

    def shake_at(tf):
        dx = dy = 0.0
        for (t0_, amp) in shakes:
            dt = tf - t0_
            if 0.0 <= dt < 0.32:
                k = amp * math.exp(-dt * 13.0) * (W / 1920.0)
                dx += k * math.sin(2 * math.pi * 11.0 * dt + 0.6)
                dy += k * math.cos(2 * math.pi * 13.0 * dt)
        return dx, dy

    srcs = {}
    sheet = []
    missing = {}
    last = np.full((H, W, 3), 0.8, np.float32)
    alt_ok = {}

    def effective(shot):
        """Use the upgraded (moving camera) version of a still shot if enough of its frames are rendered."""
        nm = shot["name"]
        if nm not in alt_ok:
            alt_ok[nm] = shot
            if not a.prev:
                for alt in S.ALT:
                    d = os.path.join(src, alt["name"])
                    if alt["base"] == nm and os.path.isdir(d) and os.path.exists(os.path.join(d, "meta.json")):
                        have = len([f for f in os.listdir(d) if f.startswith("reg_")])
                        plates = len([f for f in os.listdir(d) if f.startswith("plate_")])
                        if have >= (alt["n"] + 1) // 2 and plates >= 2:
                            alt_ok[nm] = alt
                            print("POST using upgraded shot", alt["name"], "(%d frames)" % have, flush=True)
        return alt_ok[nm]

    for n in (sorted(dump) if a.only_dump else range(n0, n1)):
        shot, i = S.shot_of_frame(n)
        shot = effective(shot)
        if shot["name"] not in srcs:
            srcs.clear()
            srcs[shot["name"]] = ShotSrc(shot, src, a.prev)
        im = srcs[shot["name"]].frame(i)
        if im is None:
            missing[shot["name"]] = missing.get(shot["name"], 0) + 1
            fr = last
        else:
            u = i / float(max(shot["n"] - 1, 1))
            fr = move2d(im, shot, u, W, H)
            if fr.shape[1] != W or fr.shape[0] != H:
                fr = cv2.resize(fr, (W, H), interpolation=cv2.INTER_LANCZOS4)
            if shot["kind"] != "still":
                sdx, sdy = shake_at(n / float(S.FPS))
                if abs(sdx) + abs(sdy) > 0.05:
                    fr = cv2.warpAffine(fr, np.float32([[1, 0, sdx], [0, 1, sdy]]), (W, H), flags=cv2.INTER_LINEAR,
                                        borderMode=cv2.BORDER_REPLICATE)
            last = fr
        g = grade(fr, W, H, n, total, a.nograde)
        if ff is not None:
            ff.stdin.write((g * 255.0 + 0.5).astype(np.uint8).tobytes())
        if n in dump:
            cv2.imwrite(os.path.join(ROOT, "out", "qa_%04d.png" % n), (g * 255.0 + 0.5).astype(np.uint8))
        if a.sheet and (n % 45 == 22):
            sheet.append(cv2.resize((g * 255).astype(np.uint8), (384, 216), interpolation=cv2.INTER_AREA))
        if n % 150 == 0:
            print("POST frame %d / %d  (%s)" % (n, n1, shot["name"]), flush=True)
    if ff is not None:
        ff.stdin.close()
        ff.wait()
    if a.sheet and sheet:
        cols = 6
        rows = (len(sheet) + cols - 1) // cols
        canvas = np.full((rows * 216, cols * 384, 3), 255, np.uint8)
        for k, t in enumerate(sheet):
            r, c = divmod(k, cols)
            canvas[r * 216:(r + 1) * 216, c * 384:(c + 1) * 384] = t
        cv2.imwrite(os.path.join(ROOT, a.sheet), canvas)
    print("POST wrote", out, " missing:", missing)


if __name__ == "__main__":
    main()
