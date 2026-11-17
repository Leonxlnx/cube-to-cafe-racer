"""The film rig: builds everything and poses it as a pure function of animation time T (seconds)."""
import math
import importlib
import bpy
from mathutils import Matrix, Vector

import common as C
import geo
import mats
import studio
import cube
import chassis as CHS
from rigbase import (Node, T as Tr, R, rot_about, frame, mlerp, path, ph, smooth, smoother, ease_in, ease_out,
                     ease_io, back_out, settle, mech, lerp, ik2, aim, I4, clamp01)

t = C.T_PLATE
S, H = C.S, C.H
ROWH = cube.ROWH
RAKE = C.RAKE
SA = CHS.SA_ANGLE
CARR_TRAVEL = 0.545
TANK_SHIFT = -0.25
LID_UP = 0.10
POP = 0.003
GLOW_W = 110.0

from timeline import TL


def aimy(a, b, n):
    """Matrix at a, local X toward b, local Y as close to n as possible."""
    a = Vector(a)
    x = Vector(b) - a
    if x.length < 1e-9:
        x = Vector((1, 0, 0))
    x.normalize()
    y = Vector(n) - x * Vector(n).dot(x)
    if y.length < 1e-6:
        y = x.orthogonal()
    y.normalize()
    z = x.cross(y)
    return Matrix(((x.x, y.x, z.x, a.x), (x.y, y.y, z.y, a.y), (x.z, y.z, z.z, a.z), (0, 0, 0, 1)))


class Arm:
    """Two-link orange arm between a world anchor point and a world mount point, with a small hydraulic strut."""

    def __init__(self, name, l1, l2, col, w=0.024, th=0.008, strut=True):
        self.l1, self.l2 = l1, l2
        self.a = CHS.arm_link(name + "_a", l1, col, w, th)
        self.b = CHS.arm_link(name + "_b", l2, col, w * 0.9, th)
        self.strut = None
        if strut and min(l1, l2) > 0.12:
            self.strut = CHS.arm_strut(name + "_s", (l1 + l2) / 2.0, col)

    def solve(self, pa, pb, pole, side=1.0):
        pa, pb = Vector(pa), Vector(pb)
        e = ik2(pa, pb, self.l1, self.l2, pole)
        n = (pb - pa).cross(Vector(pole))
        if n.length < 1e-6:
            n = Vector((0, 1, 0))
        n.normalize()
        off = n * (0.0095 * side)
        self.a.matrix_world = aimy(pa + off, e + off, n)
        self.b.matrix_world = aimy(e - off, pb - off, n)
        if self.strut is not None:
            so = n * (0.0225 * side)
            p1 = e + (pa - e) * 0.20 + so
            p2 = e + (pb - e) * 0.85 + so
            self.strut[0].matrix_world = aimy(p1, p2, n)
            self.strut[1].matrix_world = aimy(p2, p1, n)


class Film:
    def __init__(self, use_parts=True, detail=True):
        self.col = studio.new_collection("bike")
        self.pcol = studio.new_collection("plates")
        geo.DEFAULT_COL = self.col
        self.parts_ok = {}
        self._build_nodes()
        self.pose_nodes_final()
        bpy.context.view_layer.update()
        self._build_chassis(use_parts)
        self._build_plates()
        self._build_arms()
        # warm work light inside the cube: leaks through the seams when they pop, floods out with the doors
        li = bpy.data.lights.new("core_glow", "POINT")
        li.color = (1.0, 0.56, 0.22)
        li.shadow_soft_size = 0.06
        li.energy = 0.0
        self.glow = bpy.data.objects.new("core_glow", li)
        self.col.objects.link(self.glow)
        self.glow.parent = self.CH.ob
        self.glow.location = (0.02, 0.0, 0.93)
        # parts that would poke through the closed walls while folded (hose loops, mirrors): they stay
        # hidden until the doors have started to swing away
        self.offenders = []
        self.pose(0.0)
        bpy.context.view_layer.update()
        lim = H - 0.0054
        for o in self.col.all_objects:
            if o.type != "MESH":
                continue
            mw = o.matrix_world
            nv = len(o.data.vertices)
            step = 7 if nv > 4000 else 1
            for vi in range(0, nv, step):
                w = mw @ o.data.vertices[vi].co
                if abs(w.x) > lim or abs(w.y) > lim or w.z > S - 0.0054:
                    self.offenders.append(o)
                    break
        print("FILM hidden while the cube is closed:", [o.name for o in self.offenders])
        self.pose(TL["end"])

    # ------------------------------------------------------------------ nodes
    def _build_nodes(self):
        c = self.col
        N = lambda name, parent=None, local=None: Node(name, parent, local, c)
        self.CH = N("CH")
        Cw = Vector(C.fork_to_world(C.HINGE_C_F))
        self.Cw = Cw
        self.F = Tr(C.F_O) @ R("Y", -RAKE)
        self.HEAD = N("HEAD", self.CH, Tr(Cw))
        self.STEERB = N("STEERB", self.HEAD, Tr(Cw).inverted() @ self.F)
        self.STEER = N("STEER", self.STEERB)
        self.LOWER = N("LOWER", self.STEER)
        self.FW = N("FW", self.LOWER, Tr(C.FORK_OFFSET, 0, -C.FORK_LEN))
        self.CLIP = {1: N("CLIP_L", self.STEER, Tr(C.FORK_OFFSET, C.FORK_HALF_SPAN, -0.055)),
                     -1: N("CLIP_R", self.STEER, Tr(C.FORK_OFFSET, -C.FORK_HALF_SPAN, -0.055))}
        self.SAP = N("SAP", self.CH, Tr(C.PIVOT))
        self.ARMB = N("ARMB", self.SAP, Tr(0, C.ARM_Y, 0) @ R("Y", -SA))
        self.L1 = N("L1", self.ARMB)
        self.L2 = N("L2", self.L1, Tr(-C.LINK1, 0, 0))
        self.RW = N("RW", self.L2, Tr(-C.LINK2, -C.ARM_Y, 0))
        self.CARR = N("CARR", self.CH)
        # lid nets
        zt = C.TANK_TOP_Z - t / 2
        self.TN = N("TN", self.CH, Tr(0.17 + TANK_SHIFT, 0, zt))
        P = cube.TANK_BANDS
        self.tank_h = {}
        for s in (1, -1):
            sg = "+" if s > 0 else "-"
            hS = N("hS" + sg, self.TN, Tr(0.05, s * 0.10, -t / 2))
            pS = N("pS" + sg, hS, Tr(0, s * 0.045, t / 2))
            hV = N("hV" + sg, pS, Tr(0, s * 0.045, -t / 2))
            pV = N("pV" + sg, hV, Tr(0, s * 0.09, t / 2))
            hU = N("hU" + sg, pV, Tr(0, s * 0.09, -t / 2))
            pU = N("pU" + sg, hU, Tr(0, s * 0.09, t / 2))
            self.tank_h[s] = dict(hS=hS, pS=pS, hV=hV, pV=pV, hU=hU, pU=pU)
        self.hN = N("hN", self.TN, Tr(0.28, 0, -t / 2))
        self.pN = N("pN", self.hN, Tr(0.05, 0, t / 2))
        self.CAP = N("CAP", self.TN, Tr(cube.CAP_POS[0], cube.CAP_POS[1], 0))
        self.TT = N("TT", self.CARR, Tr(-0.33 - CARR_TRAVEL, 0, 0.935 - t / 2))
        self.tail_h = {}
        B = cube.TAIL_BANDS
        wS = B["TS"][1] - B["TS"][0]
        w1 = B["TB1"][1] - B["TB1"][0]
        w2 = B["TB2"][1] - B["TB2"][0]
        self.tail_w = (wS, w1, w2)
        for s in (1, -1):
            sg = "+" if s > 0 else "-"
            hS = N("thS" + sg, self.TT, Tr(0, s * 0.09, -t / 2))
            pS = N("tpS" + sg, hS, Tr(0, s * wS / 2, t / 2))
            h1 = N("th1" + sg, pS, Tr(0, s * wS / 2, -t / 2))
            p1 = N("tp1" + sg, h1, Tr(0, s * w1 / 2, t / 2))
            h2 = N("th2" + sg, p1, Tr(0, s * w1 / 2, t / 2))
            p2 = N("tp2" + sg, h2, Tr(0, s * w2 / 2, -t / 2))
            self.tail_h[s] = dict(hS=hS, pS=pS, h1=h1, p1=p1, h2=h2, p2=p2)
        self.STAND = {s: N("STAND%d" % s, self.CH, Tr(0.04, s * 0.11, 0.190)) for s in (1, -1)}

    def pose_nodes_final(self):
        for n in Node.registry:
            n.set(n.rest)

    # ------------------------------------------------------------------ chassis geometry
    def _attach_all(self, node, objs):
        for o in objs:
            node.attach(o)

    def _local_all(self, node, objs):
        for o in objs:
            o.parent = node.ob
            o.matrix_parent_inverse = Matrix.Identity(4)

    def _build_chassis(self, use_parts):
        c = self.col
        self._attach_all(self.CH, CHS.build_frame(c))
        # head carrier (built in fork frame)
        for o in CHS.build_head(c):
            o.matrix_world = self.F @ o.matrix_world
            self.HEAD.attach(o)
        self._attach_all(self.SAP, CHS.build_sa_block(c))
        self._local_all(self.L1, CHS.build_link1(c))
        self._local_all(self.L2, CHS.build_link2(c))
        self.sh_top, self.sh_bot, self.sh_spring, self.sh_L0, self.sh_Ls = CHS.build_shock(c)
        objs, self.led, lens = CHS.build_carriage(c)
        self._attach_all(self.CARR, objs + [self.led, lens])
        for s in (1, -1):
            leg = CHS.build_stand_leg(c)
            leg.parent = self.STAND[s].ob
            leg.matrix_parent_inverse = Matrix.Identity(4)
        # ---- parts made by the modelling agents (or proxies)
        self.head_emit = None
        self._part_engine(use_parts)
        self._part_wheels(use_parts)
        self._part_front(use_parts)
        self._part_detail(use_parts)

    def _try(self, modname, use_parts):
        if not use_parts:
            return None
        try:
            mod = importlib.import_module(modname)
            res = mod.build(self.col)
            self.parts_ok[modname] = True
            return res
        except Exception as e:                       # noqa
            import traceback
            traceback.print_exc()
            print("FILM part %s failed (%s) -> proxy" % (modname, e))
            self.parts_ok[modname] = False
            return None

    def _part_detail(self, use_parts):
        """Extra chassis detail: dict of lists keyed by rig group.  CH / CARR / SAP lists are built in
        final-pose world coordinates, L1 / L2 in the link's local frame, HEAD in fork-frame coordinates."""
        res = self._try("parts.chassis_detail", use_parts)
        if not res:
            return
        for key, node in (("CH", self.CH), ("CARR", self.CARR), ("SAP", self.SAP)):
            for o in res.get(key, []):
                node.attach(o)
        self._local_all(self.L1, res.get("L1", []))
        self._local_all(self.L2, res.get("L2", []))
        for o in res.get("HEAD", []):
            o.matrix_world = self.F @ o.matrix_world
            self.HEAD.attach(o)

    def _part_engine(self, use_parts):
        res = self._try("parts.engine", use_parts)
        if res:
            self.CH.attach(res["engine"])
            return
        e = geo.box("eng_proxy", C.ENG_X[1] - C.ENG_X[0], 0.40, C.ENG_Z[1] - C.ENG_Z[0], bevel=0.02, col=self.col,
                    mat=mats.get("engine_black"),
                    loc=((C.ENG_X[0] + C.ENG_X[1]) / 2, 0, (C.ENG_Z[0] + C.ENG_Z[1]) / 2))
        m = geo.cyl("muf_proxy", 0.048, 0.34, 32, axis="X", col=self.col, mat=mats.get("titanium"), loc=(-0.14, -0.255, 0.205))
        self.CH.attach(e)
        self.CH.attach(m)

    def _part_wheels(self, use_parts):
        res = self._try("parts.wheels", use_parts)
        if res:
            self.FW.attach(res["front_wheel"])
            self.RW.attach(res["rear_wheel"])
            self.L2.attach(res["rear_drive"])
            return
        for nm, node, ctr, r, w in (("fw", self.FW, C.FW_C, C.FW_R, C.FW_W), ("rw", self.RW, C.RW_C, C.RW_R, C.RW_W)):
            tyre = geo.torus(nm + "_tyre", r - w / 2, w / 2, 64, 16, "Y", self.col, mats.get("rubber_tire"), loc=ctr)
            rim = geo.cyl(nm + "_rim", 0.22, w * 0.7, 48, axis="Y", col=self.col, mat=mats.get("alu_turned"), loc=ctr)
            node.attach(tyre)
            node.attach(rim)
        hub = geo.cyl("rd_proxy", 0.085, 0.09, 32, axis="Y", col=self.col, mat=mats.get("alu_cast"),
                      loc=(C.RW_C[0], 0.115, C.RW_C[2]))
        self.L2.attach(hub)

    def _part_front(self, use_parts):
        res = self._try("parts.front", use_parts)
        if res:
            self.STEER.attach(res["steer"])
            self.LOWER.attach(res["lower"])
            self.CLIP[1].attach(res["clipon_L"])
            self.CLIP[-1].attach(res["clipon_R"])
            self.head_emit = bpy.data.objects.get("headlight_emit")
            return
        F = self.F
        col = self.col
        for s in (1, -1):
            o = geo.cyl("fk_outer", C.FORK_OUTER_R, 0.45, 32, col=col, mat=mats.get("ano_titan"), loc=(0.032, s * 0.105, -0.215))
            o.matrix_world = F @ o.matrix_world
            self.STEER.attach(o)
            o = geo.cyl("fk_inner", C.FORK_INNER_R, 0.46, 32, col=col, mat=mats.get("chrome"), loc=(0.032, s * 0.105, -0.45))
            o.matrix_world = F @ o.matrix_world
            self.LOWER.attach(o)
            o = geo.cyl("clip", 0.011, 0.27, 24, axis="Y", col=col, mat=mats.get("rubber"), loc=(0, s * 0.16, 0))
            o.matrix_world = self.CLIP[s].world() @ o.matrix_world
            self.CLIP[s].attach(o)
        for z in (-0.015, -0.195):
            o = geo.box("clamp", 0.07, 0.27, 0.03, bevel=0.004, col=col, mat=mats.get("alu_machined"), loc=(0.016, 0, z))
            o.matrix_world = F @ o.matrix_world
            self.STEER.attach(o)
        o = geo.cyl("hl_proxy", 0.095, 0.10, 48, axis="X", col=col, mat=mats.get("alu_brushed"), loc=(0.215, 0, -0.105))
        o.matrix_world = F @ o.matrix_world
        self.STEER.attach(o)

    # ------------------------------------------------------------------ plates
    def _build_plates(self):
        pc = self.pcol
        self.walls = cube.build_walls(pc)
        self.lid = cube.build_lid(pc)
        # lid plates -> nodes (local, centred)
        L = self.lid

        def put(name, node):
            o = L[name]["obj"]
            o.parent = node.ob
            o.matrix_parent_inverse = Matrix.Identity(4)
            o.matrix_basis = Matrix.Identity(4)

        put("T", self.TN)
        put("N", self.pN)
        put("TT", self.TT)
        for s in (1, -1):
            sg = "+" if s > 0 else "-"
            put("S" + sg, self.tank_h[s]["pS"])
            put("V" + sg, self.tank_h[s]["pV"])
            put("U" + sg, self.tank_h[s]["pU"])
            put("TS" + sg, self.tail_h[s]["pS"])
            put("TB1" + sg, self.tail_h[s]["p1"])
            put("TB2" + sg, self.tail_h[s]["p2"])
        cap = cube.build_cap(pc)
        cap.parent = self.CAP.ob
        cap.matrix_parent_inverse = Matrix.Identity(4)
        self.cap = cap
        # wall plates: precompute swing-end, cradle matrices and stack roots
        for p in self.walls:
            s = p["side"]
            if p["leaf"]:
                hx, hy = p["hinge"]
                p["Msw"] = rot_about((hx, hy, 0), "Z", p["swing"]) @ p["M0"]
                k = p["r"]
            else:
                p["Msw"] = p["M0"].copy()
                k = -p["r"]
            p["k"] = k
            p["Mc"] = Tr(0, s * k * t, -p["r"] * ROWH) @ p["Msw"]
            xc = -H + (p["c"] + 0.5) * cube.COLW_SIDE
            p["layer"] = (2.5 + k) if p["leaf"] else (2.5 + k) - 0.0   # 0..5 from the inside out
            # layer index: wall r2 -> 0, wall r1 -> 1, wall r0 -> 2, leaf r0 -> 3, leaf r1 -> 4, leaf r2 -> 5
            p["layer"] = (2 - p["r"]) if not p["leaf"] else (3 + p["r"])
        self._build_corner_hinges(pc)
        self.stack_roots = {}
        for s in (1, -1):
            # keel unit: columns 2 + 3
            self.stack_roots[(s, "keel")] = frame((0.275, s * H, ROWH / 2), (1, 0, 0), (0, s, 0))
            self.stack_roots[(s, 1)] = frame((-H + 1.5 * cube.COLW_SIDE, s * H, ROWH / 2), (1, 0, 0), (0, s, 0))
            self.stack_roots[(s, 0)] = frame((-H + 0.5 * cube.COLW_SIDE, s * H, ROWH / 2), (1, 0, 0), (0, s, 0))
        for p in self.walls:
            key = (p["side"], "keel") if p["c"] >= 2 else (p["side"], p["c"])
            p["stack"] = key
            p["Ls"] = self.stack_roots[key].inverted() @ p["Mc"]

    def _build_corner_hinges(self, pc):
        """Segmented hinge barrels along the four vertical cube edges (door leaf hinges)."""
        rad = 0.0036
        nseg = 6
        look = {}
        for p in self.walls:
            look[(p["leaf"], p["side"], p["c"], p["r"])] = p
        for sx in (1, -1):
            for sy in (1, -1):
                cidx = 3 if sx > 0 else 0
                for r in range(3):
                    seg = (ROWH - 0.004) / nseg
                    for par in (0, 1):
                        parts = []
                        for k in range(par, nseg, 2):
                            z = r * ROWH + 0.002 + (k + 0.5) * seg
                            parts.append(geo.cyl("hk", rad, seg - 0.0012, 20, bevel=0.0005, bsegs=1, col=pc,
                                                 mat=mats.get("steel_polished"), loc=(sx * H, sy * H, z)))
                        ob = geo.join(parts, "hinge_%d_%d_%d_%d" % (sx, sy, r, par))
                        pl = look[(par == 1, sy, cidx, r)]
                        ob.parent = pl["obj"]
                        ob.matrix_parent_inverse = Matrix.Identity(4)
                        ob.matrix_basis = pl["M0"].inverted()

    def _build_arms(self):
        c = self.col
        self.arms = {}
        # head struts
        Fm = self.F
        for s in (1, -1):
            pf = Vector((CHS.STRUT_FRAME[0], s * CHS.STRUT_FRAME[1], CHS.STRUT_FRAME[2]))
            ph_ = Fm @ Vector((CHS.STRUT_HEAD_F[0], s * CHS.STRUT_HEAD_F[1], CHS.STRUT_HEAD_F[2]))
            d = (ph_ - pf).length
            self.arms[("strut", s)] = Arm("strut%d" % s, d * 0.507, d * 0.507, c, w=0.026, th=0.009)
        for s in (1, -1):
            self.arms[("tankR", s)] = Arm("tankR%d" % s, 0.315, 0.315, c)
            self.arms[("tankF", s)] = Arm("tankF%d" % s, 0.275, 0.275, c)
            self.arms[("tail", s)] = Arm("tail%d" % s, 0.255, 0.255, c)
            self.arms[("side", s)] = Arm("side%d" % s, 0.305, 0.305, c, w=0.026, th=0.009)
            self.arms[("tray", s)] = Arm("tray%d" % s, 0.275, 0.275, c, w=0.024, th=0.008)

    # ------------------------------------------------------------------ pose
    def chassis_dz(self, T):
        """Vertical offset of the chassis (0 at ride height)."""
        tl = TL["land"]
        if T <= tl:
            return -C.DROP
        tau = T - tl
        w = 2 * math.pi * 1.45
        z = 3.1
        s = 1.0 - math.exp(-z * tau) * (math.cos(w * tau) + (z / w) * math.sin(w * tau))
        return -C.DROP * (1.0 - s)

    def pose(self, T):
        self.T = T
        dz = self.chassis_dz(T)
        self.CH.set(Tr(0, 0, dz))
        # ---------------- front end
        rake_u = smoother(ph(T, *TL["rake"]))
        rake_u = rake_u + 0.012 * math.sin(math.pi * clamp01((T - TL["rake"][1]) / 0.35)) * (1 if T < TL["rake"][1] + 0.35 else 0)
        self.HEAD.set(Tr(self.Cw) @ R("Y", RAKE * (1.0 - rake_u)))
        steer = math.radians(90.0) * (1.0 - smoother(ph(T, *TL["steer"])))
        self.STEER.set(R("Z", steer))
        c_contact = -dz / math.cos(RAKE)
        if T < TL["land"]:
            comp = lerp(C.FORK_COMPRESS_FOLDED, c_contact, ease_io(ph(T, *TL["ext"]), 2.2))
        else:
            comp = c_contact
        self.LOWER.set(Tr(0, 0, comp))
        self.FW.set(Tr(C.FORK_OFFSET, 0, -C.FORK_LEN))
        for s in (1, -1):
            u = smoother(ph(T, TL["clip"][0] + (0.0 if s > 0 else 0.18), TL["clip"][1] + (0.0 if s > 0 else 0.18) - 0.18))
            fold = math.radians(84.0) if s > 0 else math.radians(96.0)
            ang = -fold * (1.0 - back_out(u, 1.0) if u < 1 else 0.0)
            self.CLIP[s].set(Tr(C.FORK_OFFSET, s * C.FORK_HALF_SPAN, -0.055) @ R("Z", ang))
        # ---------------- rear arm
        if T < TL["land"]:
            th_touch = math.asin(clamp01((C.PIVOT[2] + dz - C.RW_R) / (C.LINK1 + C.LINK2)))
            droop = th_touch * ease_io(ph(T, *TL["ext"]), 2.2)
        else:
            droop = math.asin(max(-1.0, min(1.0, (C.PIVOT[2] + dz - C.RW_R) / (C.LINK1 + C.LINK2))))
        self.SAP.set(Tr(C.PIVOT) @ R("Y", SA - droop))
        ur = ph(T, *TL["rear"])
        u1 = smoother(clamp01(ur / 0.86))
        u2 = smoother(clamp01((ur - 0.10) / 0.90))
        ring = 0.02 * math.sin(math.pi * clamp01((T - TL["rear"][1]) / 0.3)) if TL["rear"][1] < T < TL["rear"][1] + 0.3 else 0.0
        self.L1.set(R("Z", math.radians(90.0) * (1.0 - u1)))
        self.L2.set(Tr(-C.LINK1, 0, 0) @ R("Z", -math.radians(180.0) * (1.0 - u2) + ring))
        self.RW.set(Tr(-C.LINK2, -C.ARM_Y, 0))
        # ---------------- carriage
        uc = mech(ph(T, *TL["carr"]), over=0.006)
        self.CARR.set(Tr(CARR_TRAVEL * (1.0 - uc), 0, 0))
        # ---------------- stand
        us = smoother(ph(T, *TL["stand"]))
        for s in (1, -1):
            self.STAND[s].set(Tr(0.04, s * 0.11, 0.190) @ R("Y", math.radians(-90.0) * us))
        # ---------------- lid
        self._pose_lid(T)
        # ---------------- walls
        self._pose_walls(T)
        # ---------------- IK things (need world matrices)
        self._pose_ik(T)
        # ---------------- parts hidden inside the closed cube
        closed = T < TL["door_F"][0] + 0.28
        for o in self.offenders:
            o.hide_render = closed
        # ---------------- lights
        gk = smooth(ph(T, TL["breath"][0] - 0.05, TL["breath"][0] + 0.45)) * (1.0 - smooth(ph(T, TL["rows"][0] + 0.6, TL["rows"][1] + 0.3)))
        self.glow.data.energy = GLOW_W * gk
        on = smooth(ph(T, TL["lights"], TL["lights"] + 0.25))
        if self.head_emit is not None:
            self.head_emit["emit"] = on * 5.0
        self.led["emit"] = on * 0.55

    # .................................................................. lid
    def _pop(self, T, delay=0.0):
        b0 = TL["breath"][0] + delay
        return back_out(ph(T, b0, b0 + 0.32), 2.2)

    def _pose_lid(self, T):
        zl = S - t / 2 + C.DROP + POP * self._pop(T, 0.0) + LID_UP * smoother(ph(T, *TL["lid_up"]))
        zt = C.TANK_TOP_Z - t / 2
        # tank net
        ub = smoother(ph(T, *TL["tank_back"]))
        ud = ph(T, *TL["tank_down"])
        udm = mech(ud, over=0.012)
        x = 0.17 + TANK_SHIFT * ub
        z = lerp(zl, zt, udm)
        self.TN.set(Tr(x, 0, z))
        uh = smoother(ph(T, *TL["tank_hem"]))
        uf = ph(T, *TL["tank_fold"])
        aS = math.radians(40.0) * smoother(clamp01(uf / 0.75))
        aV = math.radians(40.0) * smoother(clamp01((uf - 0.2) / 0.8))
        aU = math.radians(180.0) * uh
        for s in (1, -1):
            h = self.tank_h[s]
            h["hS"].set(Tr(0.05, s * 0.10, -t / 2) @ R("X", -s * aS))
            h["hV"].set(Tr(0, s * 0.045, -t / 2) @ R("X", -s * aV))
            h["hU"].set(Tr(0, s * 0.09, -t / 2) @ R("X", -s * aU))
        self.hN.set(Tr(0.28, 0, -t / 2) @ R("Y", math.radians(9.0) * back_out(ph(T, *TL["nose"]), 1.5)))
        ucap = ph(T, *TL["cap"])
        self.CAP.set(Tr(cube.CAP_POS[0], cube.CAP_POS[1], 0.011 * back_out(clamp01(ucap / 0.45), 2.0))
                     @ R("Z", math.radians(90.0) * smoother(clamp01((ucap - 0.35) / 0.65))))
        # tail net (rides on the carriage)
        ztail = 0.935 - t / 2
        ud2 = mech(ph(T, *TL["tail_down"]), over=0.012)
        self.TT.set(Tr(-0.33 - CARR_TRAVEL, 0, lerp(zl, ztail, ud2)))
        wS, w1, w2 = self.tail_w
        a2 = math.radians(180.0) * smoother(ph(T, *TL["tail_hem"]))
        aS2 = math.radians(72.0) * smoother(ph(T, *TL["tail_side"]))
        a1 = math.radians(108.0) * back_out(ph(T, *TL["tail_close"]), 0.8)
        for s in (1, -1):
            h = self.tail_h[s]
            h["hS"].set(Tr(0, s * 0.09, -t / 2) @ R("X", -s * aS2))
            h["h1"].set(Tr(0, s * wS / 2, -t / 2) @ R("X", -s * a1))
            h["h2"].set(Tr(0, s * w1 / 2, t / 2) @ R("X", s * a2))

    # .................................................................. walls
    def _stack_pose(self, key, T):
        """World matrix of a stack root + per-layer extra offset function."""
        s = key[0]
        R0 = self.stack_roots[key]
        if key[1] == "keel":
            t0, t1 = TL["keel_L"] if s > 0 else TL["keel_R"]
            d = t1 - t0
            ta, tb, tc = t0 + 0.34 * d, t0 + 0.74 * d, t1
            ufall = ph(T, t0, ta)
            fall = ease_in(ufall, 2.4)
            if ufall >= 1.0:
                fall = 1.0 + 0.0
            pivot = (0.0, s * (H - 3 * t), 0.0)
            M = rot_about(pivot, "X", s * math.radians(90.0) * fall) @ R0
            # small bounce when it slaps the floor
            if ta <= T < ta + 0.22:
                b = math.sin(math.pi * (T - ta) / 0.22) * 0.035
                M = rot_about(pivot, "X", s * math.radians(90.0) * (1.0 - b)) @ R0
            us = smoother(ph(T, ta + 0.10, tb))
            dx = (sum(C.KEEL_X) / 2 - 0.275) * us
            dy = -s * ((H - 3 * t) - ROWH / 2) * us
            ul = back_out(ph(T, tb, tc), 1.4)
            zc = (0.145 if s > 0 else 0.115) - 3 * t
            M = Tr(dx, dy, zc * ul) @ M
            stag = smoother(ph(T, tb - 0.2, tc))
            return M, (lambda p, st=stag, ss=s: Tr((-(5 - p["layer"]) - (0 if ss > 0 else 6)) * 0.0055 * st + 0.030 * st, 0, 0))
        if key[1] == 1:       # side cover
            t0, t1 = TL["side"]
            t0 += 0.0 if s > 0 else 0.25
            t1 += 0.0 if s > 0 else 0.25
            rot0 = R0.to_3x3().to_4x4()
            Rf = Tr(-0.335, s * 0.245, 0.515) @ R("Y", math.radians(14.0)) @ rot0
            Rm1 = Tr(R0.translation) @ Tr(0.0, s * 0.11, 0.06) @ rot0
            Rm2 = Tr(-0.30, s * 0.42, 0.50) @ R("Y", math.radians(8.0)) @ rot0
            d = t1 - t0
            M = path([(t0, R0), (t0 + 0.22 * d, Rm1, smooth), (t0 + 0.66 * d, Rm2, smoother),
                      (t1, Rf, lambda u: back_out(u, 1.1))], T)
            fan = smoother(ph(T, t0 + 0.5 * d, t1))
            return M, (lambda p, f=fan, ss=s: Tr((p["layer"] - 2.5) * 0.012 * f, ss * (p["layer"] - 2.5) * 0.008 * f, 0))
        # tray under the seat
        t0, t1 = TL["tray_L"] if s > 0 else TL["tray_R"]
        rot0 = R0.to_3x3().to_4x4()
        zs = 0.734 if s > 0 else 0.704
        Rf = frame((-0.5175, 0.0, zs), (0, s, 0), (0, 0, 1))
        Rh = frame((-0.5175, s * 0.46, zs), (0, s, 0), (0, 0, 1))
        Rv = Tr(-0.5175, s * 0.60, 0.62) @ rot0
        Rm1 = Tr(R0.translation) @ Tr(0.0, s * 0.10, 0.05) @ rot0
        d = t1 - t0
        M = path([(t0, R0), (t0 + 0.18 * d, Rm1, smooth), (t0 + 0.50 * d, Rv, smoother), (t0 + 0.72 * d, Rh, smoother),
                  (t1, Rf, lambda u: back_out(u, 0.9))], T)
        return M, (lambda p: I4)

    def _pose_walls(self, T):
        cache = {}
        r0, r1 = TL["rows"]
        for p in self.walls:
            s = p["side"]
            M0 = p["M0"]
            delay = 0.30 * (1.0 - (M0.translation.z / S)) + (0.06 if p["leaf"] else 0.0)
            pop = self._pop(T, delay)
            if p["leaf"]:
                d0, d1 = TL["door_F"] if p["end"] == "F" else TL["door_B"]
                d0 += 0.0 if s > 0 else 0.12
                d1 += 0.0 if s > 0 else 0.12
                u = ph(T, d0, d1)
                e = smoother(clamp01(u / 0.9))
                if u >= 0.9:                       # arrive: tiny rebound then seated
                    v = (u - 0.9) / 0.1
                    e = 1.0 - 0.012 * math.sin(math.pi * v) * (1 - v)
                off = POP * (pop - smooth(ph(T, d0, d0 + 0.45)))
                hx, hy = p["hinge"]
                Msw = rot_about((hx, hy, 0), "Z", p["swing"] * e) @ M0 @ Tr(0, 0, off)
            else:
                dA = TL["door_F"][1] if p["c"] >= 2 else TL["door_B"][1]
                off = POP * (pop - smooth(ph(T, dA - 0.5, dA - 0.05)))
                Msw = M0 @ Tr(0, 0, off)
            # telescoping rows
            c = p["c"]
            st = r0 + (3 - c) * 0.16 + (0.0 if s > 0 else 0.08)
            ua = smooth(ph(T, st, st + 0.22))
            dur = 0.55 + 0.28 * p["r"]
            ud = ph(T, st + 0.20, st + 0.20 + dur)
            drop = ease_in(ud, 2.0)
            if ud >= 1.0 and p["r"] > 0:
                tau = T - (st + 0.20 + dur)
                drop = 1.0 - 0.012 * math.exp(-9.0 * tau) * abs(math.sin(2 * math.pi * 3.2 * tau))
            M = Tr(0, s * p["k"] * t * ua, -p["r"] * ROWH * drop) @ Msw
            # finale
            key = p["stack"]
            if key not in cache:
                cache[key] = self._stack_pose(key, T)
            Rs, extra = cache[key]
            kt0 = self._stack_t0(key)
            if T > kt0:
                M = Rs @ extra(p) @ p["Ls"]
            p["obj"].matrix_world = M

    def _stack_t0(self, key):
        s = key[0]
        if key[1] == "keel":
            return (TL["keel_L"] if s > 0 else TL["keel_R"])[0]
        if key[1] == 1:
            return TL["side"][0] + (0.0 if s > 0 else 0.25)
        return (TL["tray_L"] if s > 0 else TL["tray_R"])[0]

    # .................................................................. IK
    def _pose_ik(self, T):
        CHw = self.CH.world()
        # shock
        pa = CHw @ Vector(CHS.SHOCK_TOP)
        pb = self.SAP.world() @ (Vector(CHS.SHOCK_BOT) - Vector(C.PIVOT))
        up = Vector((0, 0, 1))
        self.sh_top.matrix_world = aim(pa, pb, up)
        self.sh_bot.matrix_world = aim(pb, pa, up)
        L = (pb - pa).length
        ms = aim(pa, pb, up) @ Tr(0.045, 0, 0) @ Matrix.Diagonal(((L - 0.077) / self.sh_Ls, 1, 1, 1))
        self.sh_spring.matrix_world = ms
        # head struts
        Fw = self.STEERB.world()
        for s in (1, -1):
            pf = CHw @ Vector((CHS.STRUT_FRAME[0], s * CHS.STRUT_FRAME[1], CHS.STRUT_FRAME[2]))
            phd = Fw @ Vector((CHS.STRUT_HEAD_F[0], s * CHS.STRUT_HEAD_F[1], CHS.STRUT_HEAD_F[2]))
            self.arms[("strut", s)].solve(pf, phd, (-0.25, 0, 1.0), side=s)
        # tank arms
        TNw = self.TN.world()
        for s in (1, -1):
            a = CHw @ Vector((-0.33, s * 0.143, 0.745))
            m = TNw @ Vector((-0.20, s * 0.072, -t / 2 - 0.013))
            self.arms[("tankR", s)].solve(a, m, (1.0, 0, 0.15), side=s)
            a = CHw @ Vector((0.250, s * 0.123, 0.745))
            m = TNw @ Vector((0.20, s * 0.072, -t / 2 - 0.013))
            self.arms[("tankF", s)].solve(a, m, (-1.0, 0, 0.15), side=s)
        # tail arms
        Cw_ = self.CARR.world()
        TTw = self.TT.world()
        for s in (1, -1):
            a = Cw_ @ Vector((-0.94, s * 0.10, CHS.CARR_Z + 0.012))
            m = TTw @ Vector((-0.06, s * 0.062, -t / 2 - 0.013))
            self.arms[("tail", s)].solve(a, m, (1.0, 0, 0.1), side=s)
        # side cover arms
        for s in (1, -1):
            key = (s, 1)
            Rs = self._stack_pose(key, T)[0] if T > self._stack_t0(key) else self.stack_roots[key]
            a = CHw @ Vector((-0.30, s * 0.196, 0.615))
            m = Rs @ Vector((0.0, 0.0, -3 * t - 0.004))
            self.arms[("side", s)].solve(a, m, (0.55, 0.0, -1.0), side=s)
            key = (s, 0)
            Rs = self._stack_pose(key, T)[0] if T > self._stack_t0(key) else self.stack_roots[key]
            a = CHw @ Vector((-0.262, s * 0.226, 0.555))
            m = Rs @ Vector((0.0, 0.0, -3 * t - 0.004))
            self.arms[("tray", s)].solve(a, m, (-0.45, 0.0, -1.0), side=-s)
