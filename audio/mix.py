"""Final soundtrack mix for the aluminium-cube -> cafe-racer film.

Run:  python3 mix.py            (from anywhere; paths are relative to this file)
Reads cues.json + manifest.json, writes mix.wav, mix_noengine.wav and mix_report.md.
Only downloaded audio is used (see manifest.json / CREDITS.md); processing = trim, fade, gain, pan,
resample (pitch +-4 %), simple filtering, ducking, static gain and (only if needed) a peak limiter.
Re-runnable: everything is derived from cues.json, so regenerated cue times are picked up automatically.
"""
import os, sys, json, re, subprocess, zlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SR = 48000
WORK = os.path.join(HERE, "_work")
os.makedirs(WORK, exist_ok=True)

# ----------------------------------------------------------------------------------------------- settings
MUSIC_PREFIX = "music_01"            # Punctus XII
MUSIC_HIT_SEARCH = (20.3, 20.9)      # the big hit of Punctus XII is searched in this track window
MUSIC_FILM_START_FALLBACK = 4.497    # used only if the hit detection fails (library analysis: hit ~20.50)
MUSIC_FILM_START_OVERRIDE = None     # set e.g. 4.497 to force a fixed placement instead of hit detection
ROOMTONE_PREFIX = "room_tone_02"     # CC0 stereo house room tone (124 s)
ROOMTONE_DBFS = -50.0                # RMS of the bed in the final mix
TARGET_LUFS = -15.0
TP_CEIL = -1.0                       # dBTP
END_FADE = 1.8                       # master fade over the last 1.8 s
PAN_SCALE = 0.35                     # use 35 % of the cue pan
SFX_REF_OFFSET = -3.5                # gain_db 0 accent = music loud-section level - 6 dB
CASCADE = (8.35, 10.45)              # wall-row cascade window (film s)
CASCADE_ATTEN = -5.0                 # extra attenuation for cascade clicks/clacks/slides
POST_HIT_REL = -14.0                 # post-hit whooshes (gain -12): RMS this far under the local music RMS
ENGINE_REL = -12.0                   # engine (gain -8): RMS this far under the local music RMS
DUCK_MAX = 3.0                       # max music ducking under loud SFX (dB)

TRANSIENT_KINDS = {"latch_click", "clack_lock", "impact_low", "metal_ring", "switch_light", "suspension"}
CONTINUOUS_KINDS = {"servo_whir", "metal_slide", "pneumatic_hiss", "hinge_swing", "ratchet"}

# Per-cue file rules: (kind, regex on desc, [file prefixes in rotation order], options)
RULES = [
    ("latch_click", r"seams pop|ripple", ["latch_click_01", "latch_click_03", "latch_click_04", "latch_click_08", "latch_click_06", "latch_click_07"], {"ripple": True}),
    ("latch_click", r"wall rows? unlatch", ["latch_click_03", "latch_click_06", "latch_click_01", "latch_click_04", "latch_click_08", "latch_click_05"], {"cascade": True}),
    ("latch_click", r"handlebar", ["latch_click_03", "latch_click_04"], {}),
    ("latch_click", r"fuel cap", ["latch_click_02"], {}),
    ("latch_click", r"seat carriage", ["latch_click_06", "latch_click_05"], {}),
    ("latch_click", r".", ["latch_click_01", "latch_click_07", "latch_click_03", "latch_click_04", "latch_click_06"], {}),
    ("clack_lock", r"row seats|seats on the bottom|top row", ["clack_lock_08", "clack_lock_01", "clack_lock_02", "clack_lock_03", "clack_lock_06", "clack_lock_04"], {"cascade": True}),
    ("clack_lock", r"door lands", ["clack_lock_05", "clack_lock_04", "clack_lock_09", "clack_lock_03"], {}),
    ("clack_lock", r"ka-chunk|solid|snaps up|locks straight|struts lock", ["clack_lock_06", "clack_lock_01", "clack_lock_04", "clack_lock_03"], {"heavy": True}),
    ("clack_lock", r"docks|tail box", ["clack_lock_08", "clack_lock_02", "clack_lock_04"], {}),
    ("clack_lock", r"cover locks", ["clack_lock_10", "clack_lock_08"], {}),
    ("clack_lock", r".", ["clack_lock_04", "clack_lock_02", "clack_lock_08"], {}),
    ("servo_whir", r"swingarm|long, powerful", ["servo_whir_08"], {}),
    ("servo_whir", r"fork|rake", ["servo_whir_09"], {}),
    ("servo_whir", r"front door", ["servo_whir_07"], {}),
    ("servo_whir", r"rear door", ["servo_whir_10"], {}),
    ("servo_whir", r"lid rises", ["servo_whir_01"], {}),
    ("servo_whir", r"turns straight|light, smooth", ["servo_whir_03"], {}),
    ("servo_whir", r"tiny|feet", ["servo_whir_02"], {}),
    ("servo_whir", r"tank panels travel", ["servo_whir_05"], {}),
    ("servo_whir", r"tank shell lowers", ["servo_whir_04"], {}),
    ("servo_whir", r"stack lifts", ["servo_whir_11", "servo_whir_12"], {}),
    ("servo_whir", r"side covers", ["servo_whir_06", "servo_whir_07"], {}),
    ("servo_whir", r".", ["servo_whir_07", "servo_whir_12", "servo_whir_09", "servo_whir_01", "servo_whir_03"], {}),
    ("metal_slide", r"wall rows? telescope", ["metal_slide_05", "metal_slide_06", "metal_slide_07", "metal_slide_02", "metal_slide_03"], {"cascade": True}),
    ("metal_slide", r"seat carriage", ["metal_slide_01"], {}),
    ("metal_slide", r"stack slides", ["metal_slide_04", "metal_slide_08", "metal_slide_03"], {}),
    ("metal_slide", r".", ["metal_slide_01", "metal_slide_03", "metal_slide_04"], {}),
    ("hinge_swing", r"front door", ["hinge_swing_04"], {}),
    ("hinge_swing", r"rear door", ["hinge_swing_01"], {}),
    ("hinge_swing", r"fold", ["hinge_swing_07"], {}),
    ("hinge_swing", r".", ["hinge_swing_06", "hinge_swing_04", "hinge_swing_07"], {}),
    ("pneumatic_hiss", r"seam", ["pneumatic_hiss_04"], {}),
    ("pneumatic_hiss", r"fork legs", ["pneumatic_hiss_05"], {}),
    ("pneumatic_hiss", r"fuel cap|puff", ["pneumatic_hiss_03"], {}),
    ("pneumatic_hiss", r".", ["pneumatic_hiss_03", "pneumatic_hiss_06"], {}),
    ("ratchet", r"fuel cap|quarter", ["ratchet_01"], {}),
    ("ratchet", r".", ["ratchet_02", "ratchet_04"], {}),
    ("metal_ring", r".", ["metal_ring_06", "metal_ring_03"], {}),
    ("suspension", r"tyres? touch|settle", ["suspension_01"], {"layer": ("suspension_06", -8.0)}),
    ("suspension", r".", ["suspension_02"], {}),
    ("impact_low", r"REVEAL", ["impact_low_01"], {"reveal": True}),
    ("impact_low", r"slap", ["impact_low_13"], {}),
    ("impact_low", r"thud|touchdown", ["impact_low_12"], {}),
    ("impact_low", r".", ["impact_low_11"], {}),
    ("switch_light", r"headlight", ["switch_light_06"], {"layer": ("switch_light_02", -4.0)}),
    ("switch_light", r".", ["switch_light_01"], {}),
    ("whoosh", r"wall panels", ["whoosh_06", "whoosh_02"], {}),
    ("whoosh", r".", ["whoosh_08", "whoosh_06", "whoosh_03", "whoosh_07", "whoosh_02", "whoosh_01"], {}),
    ("riser", r".", ["riser_03"], {}),
    ("engine", r".", ["engine_02"], {"engine": True}),
]
HEAVY_LAYER = ("clack_lock_09", -6.0)   # low tailgate thunk layered under the heavy locks
INTRO_RISER = ("riser_04", -18.0)        # very soft reverse cymbal into the first sound

# ----------------------------------------------------------------------------------------------- io helpers
man = json.load(open(os.path.join(HERE, "manifest.json"), encoding="utf8"))["files"]
BYPREFIX = {}
for m in man:
    stem = os.path.basename(m["path"])
    # key like 'clack_lock_05' (category + number) or 'music_01'
    mm = re.match(r"([a-z_]+?_\d\d)_", stem)
    BYPREFIX[mm.group(1) if mm else stem[:8]] = m
_cache = {}


def entry(prefix):
    if prefix not in BYPREFIX:
        raise KeyError("no manifest file for " + prefix)
    return BYPREFIX[prefix]


def load(prefix, max_s=None):
    if prefix in _cache:
        return _cache[prefix]
    m = entry(prefix)
    path = os.path.join(HERE, m["path"])
    ch = int(m.get("channels", 2))
    lim = ["-t", "%.3f" % max_s] if max_s else []
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", path] + lim + ["-map", "0:a:0", "-ar", str(SR), "-f", "f32le", "-"],
                         stdout=subprocess.PIPE, check=True).stdout
    x = np.frombuffer(raw, np.float32).reshape(-1, ch).astype(np.float64)
    _cache[prefix] = x
    return x


def db(v):
    return 20 * np.log10(np.maximum(v, 1e-12))


def env_db(mono, win, hop):
    """RMS envelope in dB, window/hop in samples, frame i covers [i*hop, i*hop+win)."""
    c = np.concatenate([[0.0], np.cumsum(mono * mono)])
    n = max(1, (len(mono) - 1) // hop + 1)
    s = np.arange(n) * hop
    e = np.minimum(s + win, len(mono))
    return 10 * np.log10((c[e] - c[s]) / np.maximum(e - s, 1) + 1e-20)


def max_rms(x, win_s=0.05):
    mono = x.mean(1) if x.ndim == 2 else x
    w = int(win_s * SR)
    if len(mono) <= w:
        return float(np.sqrt(np.mean(mono ** 2)) + 1e-12)
    return float(10 ** (env_db(mono, w, w // 4).max() / 20))


def resample(x, ratio):
    """Pitch/speed change by band-limited (FFT) resampling. ratio>1 -> higher & shorter."""
    if abs(ratio - 1) < 1e-4:
        return x
    n = len(x)
    pad = int(0.05 * n) + 256
    xp = np.concatenate([x, np.zeros((pad, x.shape[1]))])
    N = len(xp)
    M = int(round(N / ratio))
    X = np.fft.rfft(xp, axis=0)
    K = M // 2 + 1
    Y = np.zeros((K, x.shape[1]), complex)
    k = min(K, X.shape[0])
    Y[:k] = X[:k]
    y = np.fft.irfft(Y, n=M, axis=0) * (M / N)
    return y[: int(round(n / ratio))]


def hrand(*keys):
    """Deterministic pseudo-random in [0,1) from keys (stable across runs)."""
    return (zlib.crc32(("|".join(str(k) for k in keys)).encode()) % 100000) / 100000.0


def anchor_of(x, mode, thr=6.0):
    """Return anchor sample index. transient: attack of the first strong peak; start: first audible
    sample; peak: loudest 20 ms; end_peak: loudest point (risers)."""
    mono = np.abs(x).max(1) if x.ndim == 2 else np.abs(x)
    e = env_db(x.mean(1) if x.ndim == 2 else x, 96, 48)          # 2 ms window, 1 ms hop
    emax = e.max()
    if mode == "start":
        i = int(np.argmax(e > emax - 40))
        return max(0, i * 48 - 48)
    if mode in ("peak", "end_peak"):
        e20 = env_db(x.mean(1) if x.ndim == 2 else x, 960, 48)
        return int(np.argmax(e20) * 48)
    # transient: attack of the first peak within 6 dB of the loudest point (weaker lead-ins such as a
    # drawer slide or a lever wind-up are kept as pre-roll and land before the cue time)
    strong = np.where(e > emax - thr)[0]
    i0 = strong[0]
    j = i0
    while j + 1 < len(e) and e[j + 1] >= e[j]:
        j += 1                                                    # climb to the local peak
    k = j
    while k > 0 and e[k] > e[j] - 12:
        k -= 1                                                    # walk back to the attack start
    return int(k * 48)


def fade(y, fin, fout):
    y = y.copy()
    n = len(y)
    fi, fo = min(int(fin * SR), n // 2), min(int(fout * SR), n // 2)
    if fi > 1:
        y[:fi] *= (np.sin(np.linspace(0, np.pi / 2, fi)) ** 2)[:, None]
    if fo > 1:
        y[-fo:] *= (np.cos(np.linspace(0, np.pi / 2, fo)) ** 2)[:, None]
    return y


def pan_gains(nch, p):
    p = float(np.clip(p, -1, 1))
    if nch == 1:
        th = (p + 1) * np.pi / 4
        return np.cos(th) * np.sqrt(2), np.sin(th) * np.sqrt(2)
    return min(1.0, 1 - p), min(1.0, 1 + p)


def to_stereo(y, p):
    gl, gr = pan_gains(y.shape[1], p)
    if y.shape[1] == 1:
        return np.stack([y[:, 0] * gl, y[:, 0] * gr], 1)
    return np.stack([y[:, 0] * gl, y[:, 1] * gr], 1)


def fft_highpass(x, fc, slope_oct=1.0):
    """Zero-phase high-pass via FFT with a smooth (raised-cosine) transition one octave wide."""
    N = len(x)
    X = np.fft.rfft(x, axis=0)
    f = np.fft.rfftfreq(N, 1 / SR)
    lo, hi = fc / (2 ** slope_oct), fc
    g = np.clip((np.log2(np.maximum(f, 1e-3)) - np.log2(lo)) / (np.log2(hi) - np.log2(lo)), 0, 1)
    g = np.sin(g * np.pi / 2) ** 2
    return np.fft.irfft(X * g[:, None], n=N, axis=0)


# ----------------------------------------------------------------------------------------------- cue planning
cues_doc = json.load(open(os.path.join(HERE, "cues.json"), encoding="utf8"))
DUR = float(cues_doc["duration"])
REVEAL = float(cues_doc.get("reveal", 25.0))
NS = int(round(DUR * SR))
cuts = sorted(c["t"] for c in cues_doc["cues"] if c["kind"] == "cut")
cues = [c for c in cues_doc["cues"] if c["kind"] != "cut"]
cues.sort(key=lambda c: c["t"])


def rule_for(c):
    for kind, rx, files, opt in RULES:
        if kind == c["kind"] and re.search(rx, c.get("desc", ""), re.I):
            return files, opt
    raise ValueError("no rule for cue %s" % c)


# ---- music
mus_entry = entry(MUSIC_PREFIX)
music_raw = load(MUSIC_PREFIX, max_s=DUR + 1.0)


def detect_hit(x):
    """Broadband (>150 Hz) hit: largest sustained level jump (next 35 ms vs previous 100 ms, power average) in the
    search window, then walked back from the local peak to its -12 dB attack start."""
    a, b = int(MUSIC_HIT_SEARCH[0] * SR), int(MUSIC_HIT_SEARCH[1] * SR)
    seg = fft_highpass(x[a - SR // 2: b + SR // 2].mean(1)[:, None], 150.0)[:, 0]
    e = env_db(seg, 96, 48)                       # 1 ms frames
    P = 10 ** (e / 10)
    c = np.concatenate([[0.0], np.cumsum(P)])
    o = SR // 2 // 48
    best, bi = -1e9, o
    for i in range(o, len(e) - o):
        post = (c[i + 35] - c[i]) / 35
        pre = (c[i - 5] - c[i - 105]) / 100
        j = 10 * np.log10(post / pre)
        if j > best:
            best, bi = j, i
    j = bi + int(np.argmax(e[bi:bi + 20]))
    k = j
    while k > bi - 15 and e[k] > e[j] - 12:
        k -= 1
    return (a - SR // 2) / SR + k / 1000.0, float(best)


HIT_TRACK, HIT_RISE = detect_hit(music_raw)
if not (MUSIC_HIT_SEARCH[0] < HIT_TRACK < MUSIC_HIT_SEARCH[1]) or HIT_RISE < 12:
    HIT_TRACK = REVEAL - MUSIC_FILM_START_FALLBACK
MUSIC_FILM_START = REVEAL - HIT_TRACK         # track 0.000 lands here, so the hit lands exactly on the reveal
if MUSIC_FILM_START_OVERRIDE is not None:
    MUSIC_FILM_START = float(MUSIC_FILM_START_OVERRIDE)
music = np.zeros((NS, 2))
o = int(round(MUSIC_FILM_START * SR))
seg = music_raw[: NS - o]
music[o:o + len(seg)] = seg[:, :2]
# references from the loud section after the hit (film reveal .. end-2)
a, b = int((REVEAL + 0.3) * SR), int((DUR - 2.5) * SR)
M_PEAK = float(np.abs(music[a:b]).max())
mono_m = music[a:b].mean(1)
e50 = env_db(mono_m, int(0.05 * SR), int(0.0125 * SR))
M_50 = float(10 ** (np.percentile(e50, 95) / 20))          # loud-section 50 ms RMS (95th pct)
# lift the quiet build before the hit a little so the transformation is not much quieter than the rest
PRE_HIT_BOOST_DB = 4.5
if PRE_HIT_BOOST_DB:
    _tt = np.arange(NS) / SR
    _gdb = np.where(_tt < REVEAL - 0.09, PRE_HIT_BOOST_DB,
                    np.clip((REVEAL - 0.01 - _tt) / 0.08, 0.0, 1.0) * PRE_HIT_BOOST_DB)
    music *= (10 ** (_gdb / 20.0))[:, None]

# ---- place SFX
sfx = np.zeros((NS, 2))
eng = np.zeros((NS, 2))
report = []
used_files = {}
last_used = {}          # rotation memory per rule
counter = {}


def add(bus, y, start_idx):
    s = max(0, start_idx)
    off = s - start_idx
    e = min(NS, start_idx + len(y))
    if e > s:
        bus[s:e] += y[off:off + (e - s)]


def prepare(prefix, kind, cue, opt, pitch, mode=None):
    x = load(prefix)
    x = resample(x, pitch)
    dur = cue.get("dur") or 0.3
    cascade = opt.get("cascade", False)
    if kind == "whoosh":
        mode = mode or "peak"
    elif kind == "riser":
        mode = "end_peak"
    elif kind in CONTINUOUS_KINDS:
        mode = mode or "start"
    else:
        mode = mode or "transient"
    anc = anchor_of(x, mode, thr=15.0 if kind == "impact_low" else 6.0)
    if kind in TRANSIENT_KINDS:
        pre = min(anc, int((0.015 if cascade else 0.2) * SR))
        lims = {"latch_click": (0.12, 0.45), "clack_lock": (0.25, 0.8), "impact_low": (0.5, 3.0), "metal_ring": (0.6, 1.0),
                "switch_light": (0.8, 1.8), "suspension": (dur + 0.4, dur + 0.9)}[kind]
        if cascade:
            lims = (0.1, 0.2) if kind == "latch_click" else (0.18, 0.26)
        post = int(np.clip(dur + 0.1, *lims) * SR)
        if kind == "impact_low" and opt.get("reveal"):
            post = int(max(dur, 2.0) * SR)
        if kind in ("clack_lock", "latch_click"):
            # keep one event: stop before any later strong attack (> 0.25 s after the anchor) such as a second slam
            e = env_db(x.mean(1), 96, 48)
            emax = e.max()
            i0 = (anc + int(0.25 * SR)) // 48
            for i in range(i0, min(len(e) - 5, (anc + post) // 48)):
                if e[i] > emax - 6 and e[i] - e[i - 5] > 10:
                    post = max(int(0.12 * SR), i * 48 - anc - int(0.015 * SR))
                    break
        y = x[anc - pre: anc + post]
        y = fade(y, 0.004 if pre < int(0.02 * SR) else 0.02, 0.03 if cascade else min(0.25, 0.35 * len(y) / SR))
        return y, pre, anc, mode
    if kind in CONTINUOUS_KINDS:
        post = int((dur + (0.08 if cascade else 0.15)) * SR)
        y = x[anc: anc + post]
        y = fade(y, 0.006, min(0.25 if not cascade else 0.12, 0.3 * len(y) / SR))
        return y, 0, anc, mode
    if kind == "whoosh":
        pre = min(anc, int(0.45 * SR))
        y = x[anc - pre: anc + int(0.4 * SR)]
        y = fade(y, 0.03, 0.15)
        return y, pre, anc, mode
    if kind == "riser":
        pre = min(anc, int(dur * SR))
        y = x[anc - pre: anc]
        y = fade(y, 0.25, 0.004)
        return y, pre, anc, mode
    raise ValueError(kind)


def level_gain(y, kind, g):
    """Gain so the sound sits at (music loud-section ref - 6 dB + cue gain)."""
    if kind in TRANSIENT_KINDS:
        cur = float(np.abs(y).max())
        tgt = M_PEAK * 10 ** ((SFX_REF_OFFSET + g) / 20)
    else:
        cur = max_rms(y, 0.05)
        tgt = M_50 * 10 ** ((SFX_REF_OFFSET + g) / 20)
    return float(np.clip(20 * np.log10(tgt / max(cur, 1e-9)), -40, 24))


def place(prefix, kind, cue, opt, t_land, gain_extra=0.0, pitch=1.0, pan=0.0, bus=None, note=""):
    y, pre, anc, mode = prepare(prefix, kind, cue, opt, pitch)
    if "rel_music" in opt:
        # level relative to the music actually playing around the landing point (RMS over the sound's span)
        a0 = max(0, int(round(t_land * SR)) - pre)
        b0 = min(NS, a0 + len(y))
        m_rms = float(np.sqrt(np.mean(music[a0:b0] ** 2)) + 1e-9)
        y_rms = float(np.sqrt(np.mean(y ** 2)) + 1e-12)
        gdb = 20 * np.log10(m_rms / y_rms) + opt["rel_music"] + gain_extra
    else:
        gdb = level_gain(y, kind, float(cue.get("gain_db", 0))) + gain_extra
    y = y * 10 ** (gdb / 20)
    y = to_stereo(y, pan * PAN_SCALE)
    start = int(round(t_land * SR)) - pre
    add(bus if bus is not None else sfx, y, start)
    m = entry(prefix)
    used_files[m["path"]] = m
    report.append(dict(t=cue["t"], land=round(t_land, 3), kind=kind, desc=cue.get("desc", ""), file=os.path.basename(m["path"]),
                       anchor_mode=mode, anchor_in_file_s=round(anc / SR * pitch, 3), pitch=round(pitch, 3),
                       gain_db=round(gdb, 1), pan=round(pan * PAN_SCALE, 2), len_s=round(len(y) / SR, 2), note=note,
                       peak_pre_master=float(np.abs(y).max())))
    return y


def pick(files, key):
    """Rotate: first file not used in the last two picks of this rule, else the least recently used."""
    lu = last_used.setdefault(key, [])
    for f in files:
        if f not in lu[-2:] and f not in lu:
            lu.append(f)
            return f
    last = {f: max([i for i, q in enumerate(lu) if q == f] or [-1]) for f in files}
    f = min(files, key=lambda q: last[q])
    lu.append(f)
    return f


def next_cut_after(t, within):
    for c in cuts:
        if t < c <= t + within:
            return c
    return None


first_t = cues[0]["t"] if cues else 0
for ci, c in enumerate(cues):
    kind, t, g = c["kind"], float(c["t"]), float(c.get("gain_db", 0))
    files, opt = rule_for(c)
    key = kind + "|" + str(files)
    in_cascade = opt.get("cascade", False) and CASCADE[0] <= t <= CASCADE[1]
    if opt.get("engine"):
        continue  # handled below
    if opt.get("ripple"):
        # fast ripple of 6 crisp clicks over ~0.35 s, first one strongest
        offs = [0.0, 0.055, 0.12, 0.185, 0.26, 0.335]
        for k, off in enumerate(offs):
            f = files[k % len(files)]
            pr = 1 + 0.06 * (hrand(t, k, "p") - 0.5)
            pn = (hrand(t, k, "pan") - 0.5) * 0.6
            place(f, kind, dict(c, dur=0.12), opt, t + off, gain_extra=-1.5 * k ** 0.8, pitch=pr, pan=pn,
                  note="seam ripple %d/6" % (k + 1))
        continue
    f = pick(files, key)
    jitter = (hrand(t, f, "g") - 0.5) * 3.0 if in_cascade else 0.0
    pr = 1 + (0.08 if in_cascade else 0.04) * (hrand(t, f, "p") - 0.5)
    extra = (CASCADE_ATTEN if in_cascade else 0.0) + jitter
    t_land = t
    if kind == "whoosh":
        nc = next_cut_after(t, 0.3)
        t_land = nc if nc is not None else t + 0.6 * float(c.get("dur") or 0.5)
        if t > REVEAL + 0.5:
            opt = dict(opt, rel_music=POST_HIT_REL + (g + 12.0))
    if kind == "riser":
        t_land = t + float(c.get("dur") or 2.2)       # riser peak/end lands at t+dur (the reveal)
        pr = 1.0
    if opt.get("reveal"):
        pr = 1.0
    place(f, kind, c, opt, t_land, gain_extra=extra, pitch=pr, pan=float(c.get("pan", 0)),
          note=("cascade" if in_cascade else ""))
    if opt.get("heavy"):
        place(HEAVY_LAYER[0], "clack_lock", dict(c, dur=0.4), {}, t, gain_extra=HEAVY_LAYER[1], pitch=0.97,
              pan=float(c.get("pan", 0)), note="weight layer under heavy lock")
    if opt.get("layer"):
        lp, lg = opt["layer"]
        lk = "suspension" if lp.startswith("suspension") else ("switch_light" if lp.startswith("switch") else kind)
        place(lp, lk, c, {}, t + (0.05 if lk == "suspension" else 0.0), gain_extra=lg, pitch=1.0, pan=float(c.get("pan", 0)),
              note="layer")

# very soft intro riser ending on the first sound (before the music starts)
if cues:
    rc = dict(t=first_t - 2.2, kind="riser", desc="(auto) very soft reverse cymbal into the first seam pop", dur=2.2, gain_db=INTRO_RISER[1], pan=0)
    place(INTRO_RISER[0], "riser", rc, {}, first_t - 0.01, pitch=1.0, pan=0.0, note="intro riser (not a cue)")

# ---- engine: starter + catch from the BMW take, crossfaded into the start of its rev (same take)
eng_cues = [c for c in cues if c["kind"] == "engine"]
for c in eng_cues:
    x = load("engine_02")
    mono = x.mean(1)
    e = env_db(mono, 960, 480)                  # 20 ms / 10 ms
    t_e = np.arange(len(e)) * 0.01
    # starter: first strong rise after 0.5 s
    rel = e - e.max()
    st = int(np.argmax((t_e > 0.6) & (rel > -20)) * 480)
    # rev onset: biggest 0.5 s level jump between 30 and 37 s
    win = 50
    best, ri = -1e9, None
    for i in range(int(30 / 0.01), int(37 / 0.01)):
        jmp = e[i:i + win].mean() - e[i - win:i].mean()
        if jmp > best:
            best, ri = jmp, i
    rv = ri * 480
    A = x[st - int(0.03 * SR): st + int(1.75 * SR)]
    B = x[rv - int(0.35 * SR): rv + int(1.6 * SR)]
    xf = int(0.12 * SR)
    ramp = np.linspace(0, 1, xf)[:, None]
    y = np.concatenate([A[:-xf], A[-xf:] * (1 - ramp) + B[:xf] * ramp, B[xf:]])
    dur = float(c.get("dur") or 3.2)
    end_by = min(DUR - 0.4, float(c["t"]) + dur + 0.1)
    y = y[: int((end_by - float(c["t"])) * SR) + int(0.03 * SR)]
    y = fade(y, 0.01, 0.8)
    a0 = int(round(float(c["t"]) * SR))
    m_rms = float(np.sqrt(np.mean(music[a0:a0 + len(y)] ** 2)) + 1e-9)
    y_rms = float(np.sqrt(np.mean(y[: int(2.4 * SR)] ** 2)) + 1e-12)       # starter + rev, before the fade-out
    gdb = 20 * np.log10(m_rms / y_rms) + ENGINE_REL + (float(c.get("gain_db", -8)) + 8.0)
    y = to_stereo(y * 10 ** (gdb / 20), float(c.get("pan", 0)) * PAN_SCALE)
    add(eng, y, int(round(float(c["t"]) * SR)) - int(0.03 * SR))
    m = entry("engine_02")
    used_files[m["path"]] = m
    report.append(dict(t=c["t"], land=c["t"], kind="engine", desc=c.get("desc", ""), file=os.path.basename(m["path"]),
                       anchor_mode="start", anchor_in_file_s=round(st / SR, 3), pitch=1.0, gain_db=round(gdb, 1), pan=0.0,
                       len_s=round(len(y) / SR, 2), note="edit: starter %.2f-%.2f s + rev from %.2f s (120 ms crossfade)" % (
                           (st / SR) - 0.03, st / SR + 1.75, rv / SR - 0.35), peak_pre_master=float(np.abs(y).max())))

# ---- ducking: music dips (max DUCK_MAX dB) when the SFX bus gets close to it; not around the reveal hit
hop = int(0.01 * SR)
es = env_db((sfx + eng).mean(1), int(0.05 * SR), hop)
em = env_db(music.mean(1), int(0.05 * SR), hop)
red = np.clip(0.5 * (es - em + 2.0), 0, DUCK_MAX)
tt = np.arange(len(red)) * 0.01
red[(tt > REVEAL - 0.1) & (tt < REVEAL + 2.5)] = 0
sm = np.zeros_like(red)
att, rel_c = np.exp(-1 / 3.0), np.exp(-1 / 30.0)     # 30 ms attack, 300 ms release (10 ms frames)
cur = 0.0
for i, r in enumerate(red):
    cur = r + (cur - r) * (att if r > cur else rel_c)
    sm[i] = cur
duck_db = np.interp(np.arange(NS) / SR, tt, sm)
music_d = music * (10 ** (-duck_db / 20))[:, None]

# ---- room tone bed (high-passed, level set after the master gain)
rt = load(ROOMTONE_PREFIX, max_s=DUR + 1.0)
rt = rt[: NS] if len(rt) >= NS else np.tile(rt, (NS // len(rt) + 1, 1))[:NS]
if rt.shape[1] == 1:
    rt = np.repeat(rt, 2, 1)
rt = fft_highpass(rt, 60.0)
rt = fade(rt, 0.05, 0.05)
used_files[entry(ROOMTONE_PREFIX)["path"]] = entry(ROOMTONE_PREFIX)
used_files[mus_entry["path"]] = mus_entry


def master_fade(x):
    x = x.copy()
    n = int(END_FADE * SR)
    e = NS - int(0.02 * SR)
    x[e - n:e] *= (np.cos(np.linspace(0, np.pi / 2, n)) ** 2)[:, None]
    x[e:] = 0
    return x


def ffmpeg_measure(x, tag):
    p = os.path.join(WORK, "_mixtmp_%s.wav" % tag)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-f", "f32le", "-ar", str(SR), "-ac", "2", "-i", "-", "-c:a", "pcm_f32le", p],
                   input=x.astype(np.float32).tobytes(), check=True)
    r = subprocess.run(["ffmpeg", "-nostats", "-i", p, "-af", "ebur128=peak=true", "-f", "null", "-"], stderr=subprocess.PIPE,
                       text=True, errors="replace").stderr
    os.remove(p)
    tail = r[r.rfind("Summary:"):]
    g = lambda pat: float(re.search(pat, tail).group(1))
    return dict(I=g(r"I:\s*(-?[\d.]+) LUFS"), LRA=g(r"LRA:\s*([\d.]+) LU"), TP=g(r"Peak:\s*(-?[\d.]+) dBFS"))


LIMITER_LOG = []


def os_peaks(x, chunk=SR, padn=1024):
    """Per-sample peak of the 4x oversampled signal (chunked FFT upsampling)."""
    out = np.zeros(len(x))
    for s in range(0, len(x), chunk):
        a, b = max(0, s - padn), min(len(x), s + chunk + padn)
        up = resample(x[a:b], 0.25)
        pk = np.abs(up).max(1)
        n = (len(pk) // 4) * 4
        pk = pk[:n].reshape(-1, 4).max(1)
        seg = pk[s - a: s - a + min(chunk, len(x) - s)]
        out[s:s + len(seg)] = seg
    return out


def true_peak_limiter(x, ceil_db):
    """Look-ahead peak limiter on a 4x oversampled envelope (only used if the static gain alone exceeds the ceiling).
    Vectorised: 1 ms blocks, 10 ms look-ahead, 40 ms hold, 20 ms Hann smoothing (the min-filter covers the smoothing
    support, so the smoothed gain never exceeds what a peak needs)."""
    from numpy.lib.stride_tricks import sliding_window_view
    ceil = 10 ** (ceil_db / 20)
    pk = os_peaks(x)
    need = np.minimum(1.0, ceil / np.maximum(pk, 1e-9))
    B = 48
    nb = -(-len(need) // B)
    nd = np.concatenate([need, np.ones(nb * B - len(need))]).reshape(nb, B).min(1)
    LA, HOLD, SM = 10, 40, 10
    padded = np.concatenate([np.ones(HOLD), nd, np.ones(LA)])
    gb = sliding_window_view(padded, HOLD + LA + 1).min(1)[:nb]
    k = np.hanning(2 * SM + 3)[1:-1]
    k /= k.sum()
    gb = np.convolve(np.concatenate([np.ones(SM), gb, np.ones(SM)]), k, mode="same")[SM:SM + nb]
    g = np.interp(np.arange(len(x)), np.arange(nb) * B + B / 2, gb)
    g = np.minimum(g, 1.0)
    act = gb < 10 ** (-0.3 / 20)
    LIMITER_LOG.append(dict(ms_active=int(act.sum()), where_s=[round(float(i * B / SR), 2) for i in np.where(act & ~np.roll(act, 1))[0]][:20],
                            max_db=round(float(db(gb.min())), 2)))
    return x * g[:, None], float(db(g.min()))


def render(with_engine, gain_db=None):
    full = music_d + sfx + (eng if with_engine else 0)
    full = master_fade(full)
    if gain_db is None:
        meas = ffmpeg_measure(full, "pre")
        gain_db = TARGET_LUFS - meas["I"]
    out = full * 10 ** (gain_db / 20)
    # room tone at ROOMTONE_DBFS RMS in the final scale
    rt_rms = np.sqrt(np.mean(rt ** 2))
    out = out + master_fade(rt * (10 ** (ROOMTONE_DBFS / 20) / rt_rms))
    lim = 0.0
    m2 = ffmpeg_measure(out, "post")
    if m2["TP"] > TP_CEIL - 0.3:
        out, lim = true_peak_limiter(out, TP_CEIL - 0.5)
        m2 = ffmpeg_measure(out, "lim")
    return out, gain_db, lim, m2


def write_wav(x, name):
    p = os.path.join(HERE, name)
    x = x[:NS]
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-f", "f32le", "-ar", str(SR), "-ac", "2", "-i", "-", "-c:a", "pcm_s24le", p],
                   input=np.clip(x, -1, 1).astype(np.float32).tobytes(), check=True)
    return p


print("placed %d sounds; music hit at track %.3f -> music starts at film %.3f" % (len(report), HIT_TRACK, MUSIC_FILM_START), flush=True)
mix, G, LIM, meas = render(True)
print("rendered mix (gain %+.2f dB, limiter %.1f dB)" % (G, LIM), flush=True)
mix_ne, _, LIM_NE, meas_ne = render(False, gain_db=G)
write_wav(mix, "mix.wav")
write_wav(mix_ne, "mix_noengine.wav")
print("wrote mix.wav / mix_noengine.wav; verifying", flush=True)

# ----------------------------------------------------------------------------------------------- verification
gl = 10 ** (G / 20)
sfx_m = master_fade(sfx) * gl
mus_m = master_fade(music_d) * gl
eng_m = master_fade(eng) * gl


def attack_times(x):
    """1 ms-hop, 2 ms-window envelope (dB) of the signal high-passed at 150 Hz (music bass removed)."""
    mono = fft_highpass(x.mean(1)[:, None], 150.0)[:, 0]
    return env_db(mono, 96, 48)


def attack_near(e, t0, w=0.05):
    """Steepest 5 ms rise within +-w of t0, then walk back to the -12 dB attack start (same rule as the anchors)."""
    a, b = int((t0 - w) * 1000), int((t0 + w) * 1000)
    if a < 5 or b + 5 >= len(e):
        return None
    rise = e[a + 5:b + 5] - e[a:b]
    i = a + int(np.argmax(rise)) + 5
    j = i
    while j + 1 < len(e) and e[j + 1] >= e[j]:
        j += 1
    k = j
    while k > 0 and e[k] > e[j] - 12:
        k -= 1
    return k / 1000.0


e_sfx = attack_times(sfx_m)
e_mix = attack_times(mix)
e_mus = attack_times(mus_m)
MUSIC_HIT_DEV = attack_near(e_mus, REVEAL, 0.06)
MUSIC_HIT_DEV = None if MUSIC_HIT_DEV is None else round((MUSIC_HIT_DEV - REVEAL) * 1000)
checks = []
trans = [r for r in report if r["kind"] in TRANSIENT_KINDS]
for r in trans:
    if "layer" in r["note"]:
        continue
    others_t = [q["land"] for q in trans if q is not r and abs(q["land"] - r["land"]) > 1e-6]
    others_c = [q["land"] for q in report if q["kind"] not in TRANSIENT_KINDS and q["kind"] not in ("whoosh", "riser")]
    iso = (min([abs(r["land"] - q) for q in others_t] or [9]) > 0.15) and (min([abs(r["land"] - q) for q in others_c] or [9]) > 0.1)
    o1 = attack_near(e_sfx, r["land"])
    o2 = attack_near(e_mix, r["land"]) if iso else None
    checks.append(dict(t=r["land"], kind=r["kind"], file=r["file"], sfx_stem_dev_ms=None if o1 is None else round((o1 - r["land"]) * 1000),
                       mix_dev_ms=None if o2 is None else round((o2 - r["land"]) * 1000), isolated=iso))

# level stats
def win_peak(x, t0, t1):
    a, b = int(t0 * SR), int(t1 * SR)
    return float(db(np.abs(x[a:b]).max()))


def win_rms(x, t0, t1):
    a, b = int(t0 * SR), int(t1 * SR)
    return float(db(np.sqrt(np.mean(x[a:b] ** 2))))


music_loud_peak = win_peak(mus_m, REVEAL + 0.3, DUR - 2.5)
music_loud_rms = win_rms(mus_m, REVEAL + 0.3, DUR - 2.5)
music_trans_rms = win_rms(mus_m, MUSIC_FILM_START + 0.5, REVEAL - 0.2)
accent_peaks = []
for r in report:
    if r["kind"] in ("clack_lock", "latch_click", "switch_light", "impact_low", "metal_ring") and "layer" not in r["note"]:
        pk = float(db(r["peak_pre_master"] * gl))
        accent_peaks.append((r["t"], r["kind"], r["file"], round(pk, 1), r["note"]))
casc = [p for p in accent_peaks if "cascade" in p[4]]
heavy = [p for p in accent_peaks if p[1] == "clack_lock" and "cascade" not in p[4]]
post_hit = [r for r in report if r["land"] > REVEAL + 0.5]
post_stats = []
for r in post_hit:
    a, b = r["land"] - 0.3, r["land"] + 0.3
    if r["kind"] == "engine":
        a, b = r["land"] + 0.3, r["land"] + 2.6
    s_r = win_rms(sfx_m + eng_m, a, b)
    m_r = win_rms(mus_m, a, b)
    post_stats.append((r["t"], r["kind"], r["file"], round(s_r, 1), round(m_r, 1), round(s_r - m_r, 1)))

# write report
L = []
w = L.append
w("# Mix report")
w("")
w("Generated by `mix.py` from `cues.json` (%d sound cues, %d picture cuts) and `manifest.json`." % (len(cues), len(cuts)))
w("")
w("## Outputs")
w("")
for name, ms, lim in (("mix.wav", meas, LIM), ("mix_noengine.wav", meas_ne, LIM_NE)):
    p = os.path.join(HERE, name)
    pr = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "stream=sample_rate,channels,sample_fmt,bits_per_sample:format=duration", "-of", "json", p],
                        stdout=subprocess.PIPE, text=True)
    jj = json.loads(pr.stdout)
    w("- `%s`: %s s, %s Hz, %s ch, 24-bit PCM. Integrated **%.1f LUFS**, LRA %.1f LU, true peak **%.1f dBTP**, limiter gain reduction %s." % (
        name, jj["format"]["duration"], jj["streams"][0]["sample_rate"], jj["streams"][0]["channels"], ms["I"], ms["LRA"], ms["TP"],
        "none (not needed)" if lim == 0 else "max %.1f dB" % lim))
for k, L_ in enumerate(LIMITER_LOG):
    w("- Limiter pass %d (%s): gain reduction > 0.3 dB during %d ms in total, max %.2f dB; regions start at film %s s." % (
        k + 1, "mix.wav" if k == 0 else "mix_noengine.wav", L_["ms_active"], L_["max_db"], ", ".join("%.2f" % v for v in L_["where_s"])))
w("- Static master gain: %+.2f dB (same for both files). Room-tone bed at %.0f dBFS RMS (high-passed at 60 Hz). Master fade over the last %.1f s; the last 20 ms are digital silence." % (G, ROOMTONE_DBFS, END_FADE))
w("")
w("## Levels (final scale)")
w("")
w("- Music (Punctus XII) loud section after the hit (film %.1f-%.1f): peak %.1f dBFS, RMS %.1f dBFS. During the transformation (film %.1f-%.1f): RMS %.1f dBFS." % (
    REVEAL + 0.3, DUR - 2.5, music_loud_peak, music_loud_rms, MUSIC_FILM_START + 0.5, REVEAL - 0.2, music_trans_rms))
w("- Lock/latch accents (non-cascade) peaks: %s dBFS (target about %.1f, i.e. music loud peak - 6 dB for gain 0)." % (
    ", ".join("%.1f" % p[3] for p in heavy), music_loud_peak - 6))
if casc:
    w("- Wall-row cascade (film %.1f-%.1f): %d clicks/clacks, peaks %.1f to %.1f dBFS (max %.1f dB below the loudest lock accent)." % (
        CASCADE[0], CASCADE[1], len(casc), min(p[3] for p in casc), max(p[3] for p in casc),
        max(p[3] for p in heavy) - max(p[3] for p in casc) if heavy else 0))
w("- Music ducking: max %.1f dB (median while active %.1f dB)." % (float(duck_db.max()), float(np.median(duck_db[duck_db > 0.2])) if (duck_db > 0.2).any() else 0))
w("- Post-hit SFX vs music (RMS around each cue): " + "; ".join("%.2f %s %+.1f dB" % (p[0], p[1], p[5]) for p in post_stats))
w("")
w("## Timing check (transient lands on cue time)")
w("")
devs = [c["sfx_stem_dev_ms"] for c in checks if c["sfx_stem_dev_ms"] is not None and c["isolated"]]
devm = [c["mix_dev_ms"] for c in checks if c["mix_dev_ms"] is not None]
w("Attack detection (2 ms envelope, 1 ms hop, 150 Hz high-pass; steepest 5 ms rise within +-50 ms, walked back to its -12 dB start) "
  "on the SFX stem for isolated transient cues (n=%d): median |dev| %d ms, max %d ms (dense passages are listed but not counted: the steepest rise there can belong to a neighbouring cue). "
  "On the full mix for isolated cues (no other transient within 0.15 s, no whir/slide/hiss "
  "starting within 0.1 s; n=%d): median |dev| %d ms, max %d ms." % (
      len(devs), int(np.median(np.abs(devs))) if devs else -1, int(np.max(np.abs(devs))) if devs else -1,
      len(devm), int(np.median(np.abs(devm))) if devm else -1, int(np.max(np.abs(devm))) if devm else -1))
w("")
w("Music hit check: attack detected on the music stem %s ms from film %.3f s." % (MUSIC_HIT_DEV, REVEAL))
w("")
w("| cue t | kind | file | isolated | stem dev (ms) | mix dev (ms) |")
w("|---|---|---|---|---|---|")
for c in checks:
    w("| %.3f | %s | %s | %s | %s | %s |" % (c["t"], c["kind"], c["file"], "yes" if c["isolated"] else "no", c["sfx_stem_dev_ms"], "-" if c["mix_dev_ms"] is None else c["mix_dev_ms"]))
w("")
w("## Cue sheet (file used per cue)")
w("")
w("Music: `%s`, track 0.000 at film %.3f s; its hit (broadband attack detected at track %.3f s, level jump %.0f dB) lands at film %.3f s. No time-stretch." % (
    os.path.basename(mus_entry["path"]), MUSIC_FILM_START, HIT_TRACK, HIT_RISE, MUSIC_FILM_START + HIT_TRACK))
w("")
w("| cue t | lands at | kind | on screen | file | anchor | pitch | gain dB | pan | len s | note |")
w("|---|---|---|---|---|---|---|---|---|---|---|")
for r in sorted(report, key=lambda r: r["land"]):
    w("| %.3f | %.3f | %s | %s | %s | %s @%.3f s | %.3f | %+.1f | %+.2f | %.2f | %s |" % (
        r["t"], r["land"], r["kind"], r["desc"][:60], r["file"], r["anchor_mode"], r["anchor_in_file_s"], r["pitch"], r["gain_db"], r["pan"], r["len_s"], r["note"]))
w("")
w("## Credits required for files used in this mix")
w("")
need = [m for m in used_files.values() if m.get("attribution_required")]
if need:
    w("```")
    for m in sorted(need, key=lambda m: m["path"]):
        w(m.get("attribution_text", "%s by %s" % (m.get("title"), m.get("author"))))
    w("```")
else:
    w("None.")
w("")
w("Files used without attribution requirement (CC0 / Mixkit licence): " + ", ".join(sorted(os.path.basename(m["path"]) for m in used_files.values() if not m.get("attribution_required"))))
w("")
w("_mix_noengine.wav uses the same gain and omits the engine cue only; its credits are the same minus %s._" % ", ".join(
    sorted({r["file"] for r in report if r["kind"] == "engine"})))
open(os.path.join(HERE, "mix_report.md"), "w", encoding="utf8").write("\n".join(L) + "\n")
print("mix.wav: I %.1f LUFS, TP %.1f dBTP, LRA %.1f; noengine: I %.1f, TP %.1f; gain %+.2f dB; limiter %.1f/%.1f dB" % (
    meas["I"], meas["TP"], meas["LRA"], meas_ne["I"], meas_ne["TP"], G, LIM, LIM_NE))
print("music loud peak %.1f / rms %.1f; transformation rms %.1f; heavy accents %s; cascade max %s" % (
    music_loud_peak, music_loud_rms, music_trans_rms, [p[3] for p in heavy], max([p[3] for p in casc]) if casc else None))
print("timing: stem median %s max %s; mix median %s max %s" % (
    np.median(np.abs(devs)) if devs else None, np.max(np.abs(devs)) if devs else None, np.median(np.abs(devm)) if devm else None, np.max(np.abs(devm)) if devm else None))
print("post-hit:", post_stats)
