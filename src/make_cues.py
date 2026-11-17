"""Write audio/cues.json: every sound-relevant event of the film in FILM time (seconds), from the
animation timeline and the shot list.  python3 make_cues.py"""
import os
import sys
import json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from timeline import TL
import shots as S

OFF = S.OFF


def f(T):
    return round(T - OFF, 3)


cues = []


def add(t, kind, desc, dur=None, gain=0.0, pan=0.0):
    cues.append(dict(t=round(t, 3), kind=kind, desc=desc, dur=dur, gain_db=gain, pan=pan))


# cuts
for sh in S.SHOTS[1:]:
    add(sh["t"], "cut", "cut to " + sh["name"])

b0 = TL["breath"][0]
add(f(b0), "latch_click", "all seams pop open: fast ripple of 5-7 crisp clicks over 0.35 s", 0.4, 0)
add(f(b0) + 0.05, "pneumatic_hiss", "short air release with the seam pop", 0.5, -8)
for nm, key, pan in (("front", "door_F", 0.25), ("rear", "door_B", -0.25)):
    d0, d1 = TL[key]
    add(f(d0), "servo_whir", "%s door leaves swing 270 degrees around the cube corners" % nm, d1 - d0, -6, pan)
    add(f(d0), "hinge_swing", "air movement of the big %s door panels" % nm, d1 - d0, -10, pan)
    add(f(d1) - 0.03, "clack_lock", "%s left door lands flat on the side wall" % nm, 0.3, -3, pan + 0.3)
    add(f(d1) + 0.09, "clack_lock", "%s right door lands flat on the side wall" % nm, 0.3, -3, pan - 0.3)
l0, l1 = TL["lid_up"]
add(f(l0), "servo_whir", "lid rises 10 cm on four arms", l1 - l0, -8)
r0, r1 = TL["rows"]
for c in range(4):
    for sgn, sp in ((1, 0.0), (-1, 0.08)):
        st = r0 + (3 - c) * 0.16 + sp
        add(f(st), "latch_click", "wall rows unlatch (column %d)" % c, 0.15, -9, 0.4 * sgn)
        add(f(st + 0.2), "metal_slide", "wall rows telescope down (column %d)" % c, 0.9, -8, 0.4 * sgn)
        add(f(st + 0.2 + 0.55 + 0.28), "clack_lock", "middle row seats on the bottom row", 0.2, -8, 0.4 * sgn)
        add(f(st + 0.2 + 0.55 + 0.56), "clack_lock", "top row seats", 0.2, -6, 0.4 * sgn)
a0, a1 = TL["rear"]
add(f(a0), "servo_whir", "rear wheel swings out on the folding swingarm (long, powerful)", a1 - a0, -3, -0.2)
add(f(a1), "clack_lock", "swingarm elbow locks straight (heavy ka-chunk)", 0.4, 0, -0.2)
c0, c1 = TL["carr"]
add(f(c0), "metal_slide", "seat carriage slides back on its rails", c1 - c0, -7, -0.3)
add(f(c1), "latch_click", "seat carriage latches", 0.2, -4, -0.3)
k0, k1 = TL["rake"]
add(f(k0), "servo_whir", "fork and front wheel rake forward out of the cube", k1 - k0, -4, 0.2)
add(f(k0) + 0.2, "ratchet", "ratchet while the front end rakes out", k1 - k0 - 0.4, -10, 0.2)
add(f(k1), "clack_lock", "head struts lock", 0.35, -1, 0.2)
s0, s1 = TL["steer"]
add(f(s0), "servo_whir", "front wheel turns straight (light, smooth)", s1 - s0, -9, 0.2)
p0, p1 = TL["clip"]
add(f(p0) + 0.55, "latch_click", "left handlebar flips out and clicks", 0.2, -3, 0.25)
add(f(p0) + 0.73, "latch_click", "right handlebar flips out and clicks", 0.2, -3, 0.15)
e0, e1 = TL["ext"]
add(f(e0), "pneumatic_hiss", "fork legs extend", e1 - e0, -5)
add(f(TL["land"]), "suspension", "tyres touch the floor, bike rises and settles on its springs (soft thud + spring)", 1.2, -2)
add(f(TL["land"]), "impact_low", "soft low thud of the touchdown", 0.8, -10)
t0, t1 = TL["stand"]
add(f(t0), "servo_whir", "stand feet retract (tiny)", t1 - t0, -14)
b0_, b1_ = TL["tank_back"]
add(f(b0_), "servo_whir", "tank panels travel back above the bike", b1_ - b0_, -8)
h0, h1 = TL["tank_fold"]
add(f(h0) + 0.1, "hinge_swing", "tank side panels fold down", h1 - h0, -10)
add(f(h0) + 0.9 * (h1 - h0) * 0.75, "latch_click", "tank shoulder panels click into angle", 0.2, -5)
add(f(h1), "latch_click", "tank side panels click into angle", 0.2, -4)
d0, d1 = TL["tank_down"]
add(f(d0), "servo_whir", "tank shell lowers onto the frame", d1 - d0, -7)
add(f(d1) - 0.12, "clack_lock", "tank shell seats on the frame (solid clunk)", 0.4, -1)
add(f(TL["tail_close"][1]) - 0.08, "clack_lock", "tail box closes under the subframe", 0.3, -5, -0.4)
add(f(TL["nose"][0]) + 0.2, "latch_click", "tank nose facet clicks down", 0.15, -6, 0.2)
c0, c1 = TL["cap"]
add(f(c0), "pneumatic_hiss", "fuel cap pops up (tiny air puff)", 0.25, -10)
add(f(c0) + 0.1, "latch_click", "fuel cap pops up", 0.15, -2)
add(f(c0) + 0.35, "ratchet", "fuel cap twists a quarter turn (fine clicks)", 0.4, -6)
add(f(c1), "metal_ring", "fuel cap locks (bright metallic shing)", 0.6, -4)
for nm, key, pan in (("left", "keel_L", 0.35), ("right", "keel_R", -0.35)):
    q0, q1 = TL[key]
    d = q1 - q0
    add(f(q0), "whoosh", "%s wall panels tip inward and fall" % nm, 0.34 * d, -8, pan)
    add(f(q0 + 0.34 * d), "impact_low", "%s panel stack slaps flat on the floor (metal sheet slap)" % nm, 0.4, -5, pan)
    add(f(q0 + 0.34 * d + 0.1), "metal_slide", "%s stack slides under the engine" % nm, 0.4 * d, -4, pan)
    add(f(q1) - 0.1, "clack_lock", "%s stack snaps up into the belly keel" % nm, 0.35, -1, pan * 0.3)
for nm, key, pan in (("left", "tray_L", 0.3), ("right", "tray_R", -0.3)):
    q0, q1 = TL[key]
    add(f(q0), "servo_whir", "%s rear stack lifts and swings under the seat" % nm, q1 - q0, -10, pan)
    add(f(q1) - 0.05, "clack_lock", "%s stack docks under the seat" % nm, 0.25, -6, pan)
q0, q1 = TL["side"]
add(f(q0), "servo_whir", "side covers swing in on their arms", q1 - q0 + 0.25, -7)
add(f(q1) - 0.05, "clack_lock", "left side cover locks", 0.3, -3, 0.3)
add(f(q1) + 0.2, "clack_lock", "right side cover locks", 0.3, -3, -0.3)
add(f(TL["lights"]), "switch_light", "headlight and tail light switch on", 0.6, 0)
hero = S.by_name("s14_hero")["t"]
add(hero, "impact_low", "REVEAL: finished bike hero shot. Music hit lands here", 2.5, 0)
add(hero - 2.2, "riser", "riser into the reveal", 2.2, -4)
for sh in S.SHOTS:
    if sh["t"] > hero:
        add(sh["t"] - 0.12, "whoosh", "soft transition whoosh into " + sh["name"], 0.5, -12)
add(41.2, "engine", "optional ending accent: starter + one rev, fades out before the end", 3.2, -8)
cues.sort(key=lambda c: c["t"])
out = dict(fps=S.FPS, duration=S.TOTAL, reveal=hero,
           shots=[dict(name=s["name"], start=s["t"], frames=s["n"], kind=s["kind"]) for s in S.SHOTS], cues=cues)
dst = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "audio", "cues.json")
json.dump(out, open(dst, "w"), indent=1)
print("wrote", dst, len(cues), "cues")
