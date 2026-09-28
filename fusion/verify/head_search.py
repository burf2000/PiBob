#!/usr/bin/env python3
"""Exhaustive search of head assemblies: every axis-aligned orientation of each part.

Chain: beam -> Head-BottomServoHolder (BH) -> pan 9 g servo (in BH's pocket) ->
Head-MiddleServoHolder (MH, floor hub hole on the pan spline) -> tilt 9 g servo (in MH's
pocket) -> Head-CameraHolder (CH, arm hub hole on the tilt spline).

Interpenetration is measured with 0.5 mm voxels (mm^3). All units mm, world frame =
robot (X fwd, Y left, Z up), beam as in the URDF (centre z=300, top z=335.5, 20x20 head
hole centred on X=Y=0).
"""
import itertools
import json
import math
import os
import sys

import numpy as np
import trimesh

MESH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2", "pibob_description", "meshes")
PITCH = 0.5
BEAM_TOP = 335.5


def rots():
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, (p, s) in enumerate(zip(perm, signs)):
                R[p, i] = s
            if np.linalg.det(R) > 0:
                out.append(R)
    return out


ROTS = rots()
assert len(ROTS) == 24


class Vox:
    def __init__(self, name, sample=1):
        m = trimesh.load(os.path.join(MESH, name + ".stl"), force="mesh")
        v = m.voxelized(PITCH).fill()
        from scipy.ndimage import binary_erosion
        # inside-tests use the grid eroded by one voxel, so parts that merely TOUCH
        # (face on face) don't count as interpenetration; only > ~0.5 mm does
        self.M = binary_erosion(v.matrix)
        full = v.matrix
        self.o = v.transform[:3, 3]
        idx = np.argwhere(full)
        self.pts = (self.o + idx * PITCH)[::sample]          # voxel centres, native mm
        self.w = sample                                       # each sample stands for w voxels
        self.lo, self.hi = m.bounds

    def inside(self, p):
        i = np.round((p - self.o) / PITCH).astype(int)
        ok = np.all((i >= 0) & (i < self.M.shape), axis=1)
        r = np.zeros(len(p), bool)
        r[ok] = self.M[i[ok, 0], i[ok, 1], i[ok, 2]]
        return r


def overlap(A, TA, B, TB):
    """mm^3 of A inside B. T* = (R, t) native -> world."""
    Ra, ta = TA
    Rb, tb = TB
    pw = A.pts @ Ra.T + ta
    pb = (pw - tb) @ Rb                       # world -> B native
    return float(B.inside(pb).sum()) * A.w * PITCH ** 3


BEAM = Vox("Shoulder-Beam", 4)
BH = Vox("Head-BottomServoHolder", 2)
MH = Vox("Head-MiddleServoHolder", 2)
CH = Vox("Head-CameraHolder", 2)
SV = Vox("Servo-9g", 2)
T_BEAM = (np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.]]), np.array([0, 0, 300.]))   # rpy (pi/2,0,0)

# Servo native: origin on the shaft at the top face, +z out along the shaft, body z in [-22.7, 0]
SV_BODY = np.array([[-5.85, -6.1, -26.7], [16.95, 6.1, -4.0]])   # case (boss excluded), servo native


def servo_in_pocket(holder, T_h, pocket_lo, pocket_hi, open_axis, open_sign, max_overlap=3.0):
    """All ways a 9 g servo sits in a holder pocket (holder native coords): body inside
    the pocket on the two closed axes, slid along the open axis to the deepest position
    with <= max_overlap mm^3 interpenetration. Returns [(R_s_native, t_s_native, overlap)]."""
    res = []
    closed = [a for a in range(3) if a != open_axis]
    for R in ROTS:
        corners = np.array([[x, y, z] for x in SV_BODY[:, 0] for y in SV_BODY[:, 1] for z in SV_BODY[:, 2]]) @ R.T
        blo, bhi = corners.min(0), corners.max(0)
        ext = bhi - blo
        if any(ext[a] > (pocket_hi[a] - pocket_lo[a]) + 0.5 for a in closed):
            continue
        base = np.zeros(3)
        for a in closed:                         # centre the body in the pocket on the closed axes
            base[a] = (pocket_lo[a] + pocket_hi[a]) / 2 - (blo[a] + bhi[a]) / 2
        # along the open axis: start with the body bottom on the pocket floor, slide outward
        floor = pocket_lo[open_axis] if open_sign > 0 else pocket_hi[open_axis]
        start = floor - (blo[open_axis] if open_sign > 0 else bhi[open_axis])
        best = None
        for k in range(0, 61):                    # up to 30 mm out
            t = base.copy()
            t[open_axis] = start + open_sign * k * 0.5
            Ts_world = (T_h[0] @ R, T_h[0] @ t + T_h[1])
            ov = overlap(SV, Ts_world, holder, T_h)
            if ov <= max_overlap:
                best = (R, t, ov, k * 0.5)
                break
        if best:
            res.append(best)
    return res


def compose(T_parent, R, t):
    return (T_parent[0] @ R, T_parent[0] @ t + T_parent[1])


def run(bh_filter=None):
    rows = []
    for ib, Rb in enumerate(ROTS):
        down = -Rb.T @ np.array([0, 0, 1.])      # world down, in BH native
        peg_down = np.allclose(down, [-1, 0, 0])
        pin_down = np.allclose(down, [0, -1, 0])
        if bh_filter == "peg_down" and not peg_down:
            continue
        # place BH on the beam
        if peg_down:
            anchor = np.array([-9.75, 2.5, 0.0])      # box bottom, above the peg centre
        elif pin_down:
            anchor = np.array([4.1, -7.5, 0.0])       # box face the pin leaves, on the pin axis
        else:
            c = (BH.lo + BH.hi) / 2
            lowest = (np.array([[x, y, z] for x in (BH.lo[0], BH.hi[0]) for y in (BH.lo[1], BH.hi[1])
                                for z in (BH.lo[2], BH.hi[2])]) @ Rb.T)[:, 2].min()
            anchor = None
        if anchor is not None:
            t_b = np.array([0, 0, BEAM_TOP]) - Rb @ anchor
        else:
            cw = Rb @ c
            t_b = np.array([-cw[0], -cw[1], BEAM_TOP - lowest])
        T_b = (Rb, t_b)
        ov_beam = overlap(BH, T_b, BEAM, T_BEAM)
        # pan servo in BH pocket (open +y native)
        for Rs, ts, ov_s, slid in servo_in_pocket(BH, T_b, np.array([-1.75, -4.75, -6.1]),
                                                  np.array([21.25, 12.5, 6.1]), 1, +1):
            T_s = compose(T_b, Rs, ts)
            pan_axis = T_s[0][:, 2]
            pan_root = T_s[1]
            ov_s_beam = overlap(SV, T_s, BEAM, T_BEAM)
            # MH: native +z along the pan shaft, hub (0,-0.5,-10) at the shaft root
            for Rm in ROTS:
                if not np.allclose(Rm[:, 2], pan_axis):
                    continue
                t_m = pan_root - Rm @ np.array([0, -0.5, -10.0])
                T_m = (Rm, t_m)
                ov_m = overlap(MH, T_m, BH, T_b) + overlap(MH, T_m, SV, T_s) + overlap(MH, T_m, BEAM, T_BEAM)
                # tilt servo in MH pocket (open +y native)
                for Rt, tt, ov_t, slid_t in servo_in_pocket(MH, T_m, np.array([-9.0, -8.0, -5.0]),
                                                            np.array([14.0, 10.0, 7.1]), 1, +1):
                    T_t = compose(T_m, Rt, tt)
                    tilt_axis = T_t[0][:, 2]
                    tilt_root = T_t[1]
                    ov_t2 = overlap(SV, T_t, BH, T_b) + overlap(SV, T_t, SV, T_s)
                    # CH: native +y along the tilt shaft, hub (28.67, 17.75, 0) at the shaft root
                    for Rc in ROTS:
                        if not np.allclose(Rc[:, 1], tilt_axis):
                            continue
                        t_c = tilt_root - Rc @ np.array([28.67, 17.75, 0.0])
                        T_c = (Rc, t_c)
                        ov_c = sum(overlap(CH, T_c, X, TX) for X, TX in
                                   ((BH, T_b), (SV, T_s), (MH, T_m), (SV, T_t), (BEAM, T_BEAM)))
                        frame_up = float(Rc[:, 0] @ np.array([0, 0, -1.0]))   # U opens up if native -x = +Z
                        facing = abs(float(Rc[:, 2] @ np.array([1, 0, 0.])))  # board plane normal (native z) = +-X
                        sv_len_t = T_t[0][:, 0]                               # tilt servo body length axis
                        rows.append(dict(
                            bh=ib, peg_in_beam=bool(peg_down), pin_down=bool(pin_down),
                            pan_axis=pan_axis.round(3).tolist(), tilt_axis=tilt_axis.round(3).tolist(),
                            pan_vertical=bool(abs(pan_axis[2]) > 0.99), pan_up=bool(pan_axis[2] > 0.99),
                            tilt_horizontal=bool(abs(tilt_axis[2]) < 0.01),
                            tilt_servo_upright=bool(abs(sv_len_t[2]) > 0.99),
                            camera_frame_up=frame_up > 0.99, camera_faces_fwd=facing > 0.99,
                            pan_servo_out_mm=slid, tilt_servo_out_mm=slid_t,
                            ov_bh_beam=round(ov_beam, 1), ov_pan=round(ov_s + ov_s_beam, 1),
                            ov_mh=round(ov_m, 1), ov_tilt=round(ov_t + ov_t2, 1), ov_cam=round(ov_c, 1),
                            T=dict(bh=[Rb.tolist(), t_b.tolist()], pan=[T_s[0].tolist(), T_s[1].tolist()],
                                   mh=[Rm.tolist(), t_m.tolist()], tilt=[T_t[0].tolist(), T_t[1].tolist()],
                                   cam=[Rc.tolist(), t_c.tolist()])))
    return rows


def score(r):
    ov = r["ov_bh_beam"] + r["ov_pan"] + r["ov_mh"] + r["ov_tilt"] + r["ov_cam"]
    checks = [r["peg_in_beam"], r["pan_up"], r["tilt_horizontal"], r["camera_frame_up"], r["camera_faces_fwd"]]
    return ov, sum(not c for c in checks)


if __name__ == "__main__":
    rows = run()
    for r in rows:
        r["ov_total"], r["failed_checks"] = score(r)
    rows.sort(key=lambda r: (r["failed_checks"] > 0 and r["ov_total"] > 50, r["ov_total"] + 200 * r["failed_checks"]))
    json.dump(rows, open(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "head_search_all.json"), "w"))
    print(len(rows), "complete assemblies")
