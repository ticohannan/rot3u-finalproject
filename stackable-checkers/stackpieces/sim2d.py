"""2D section settling simulation (DD-STACK SP-6, SP-7): Numba port of web/sim2d.js.

Same algorithm and operation order as the JavaScript reference: separating-axis polygon contacts
with reference-face clipping, sequential impulses with Coulomb friction, Baumgarte position
correction and restitution. test_parity.py runs both on the same cases.

Units mm, g, s. Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
"""
from __future__ import annotations

import math

import numpy as np

try:
    from numba import njit
    HAVE_NUMBA = True
except ImportError:  # pragma: no cover - works, only about 100x slower
    HAVE_NUMBA = False

    def njit(*a, **k):
        if a and callable(a[0]):
            return a[0]
        return lambda f: f

G = 9810.0
MAXV = 12
MAXC = 8192


def pack_parts(parts, zc):
    """Section polygons (piece frame) -> (P, MAXV, 2) array relative to the CoM, and vertex counts."""
    P = len(parts)
    loc = np.zeros((P, MAXV, 2))
    nv = np.zeros(P, dtype=np.int64)
    for i, poly in enumerate(parts):
        if len(poly) > MAXV:
            raise ValueError(f"polygon with {len(poly)} vertices; at most {MAXV}")
        nv[i] = len(poly)
        for k, (x, z) in enumerate(poly):
            loc[i, k, 0] = x
            loc[i, k, 1] = z - zc
    return loc, nv


@njit(cache=True)
def _world(bx, bz, ba, loc, nv, i0, i1, wv, wn, wbox, bbox, b):
    c = math.cos(ba)
    s = math.sin(ba)
    X0 = 1e9; X1 = -1e9; Z0 = 1e9; Z1 = -1e9
    for i in range(i0, i1):
        x0 = 1e9; x1 = -1e9; z0 = 1e9; z1 = -1e9
        n = nv[i]
        for k in range(n):
            px = loc[i, k, 0]; pz = loc[i, k, 1]
            x = bx + c * px - s * pz
            z = bz + s * px + c * pz
            wv[i, k, 0] = x; wv[i, k, 1] = z
            if x < x0: x0 = x
            if x > x1: x1 = x
            if z < z0: z0 = z
            if z > z1: z1 = z
        if x0 < X0: X0 = x0
        if x1 > X1: X1 = x1
        if z0 < Z0: Z0 = z0
        if z1 > Z1: Z1 = z1
        for k in range(n):
            k2 = (k + 1) % n
            dx = wv[i, k2, 0] - wv[i, k, 0]
            dz = wv[i, k2, 1] - wv[i, k, 1]
            l = math.hypot(dx, dz)
            if l == 0.0:
                l = 1e-9
            wn[i, k, 0] = dz / l
            wn[i, k, 1] = -dx / l
        wbox[i, 0] = x0; wbox[i, 1] = x1; wbox[i, 2] = z0; wbox[i, 3] = z1
    bbox[b, 0] = X0; bbox[b, 1] = X1; bbox[b, 2] = Z0; bbox[b, 3] = Z1


@njit(cache=True)
def _ov(a0, a1, a2, a3, b0, b1, b2, b3):
    return a0 <= b1 + 0.02 and b0 <= a1 + 0.02 and a2 <= b3 + 0.02 and b2 <= a3 + 0.02


@njit(cache=True)
def _maxsep(wv, wn, nv, A, B):
    best = -1e9
    bi = 0
    for i in range(nv[A]):
        nx = wn[A, i, 0]; nz = wn[A, i, 1]
        px = wv[A, i, 0]; pz = wv[A, i, 1]
        mn = 1e9
        for k in range(nv[B]):
            d = nx * (wv[B, k, 0] - px) + nz * (wv[B, k, 1] - pz)
            if d < mn:
                mn = d
        if mn > best:
            best = mn
            bi = i
    return best, bi


@njit(cache=True)
def _clip(p, n_in, nx, nz, off, out):
    # p: (2, 2) input points; out: (3, 2); returns count (JS order: kept p0, kept p1, intersection)
    d0 = nx * p[0, 0] + nz * p[0, 1] - off
    d1 = nx * p[1, 0] + nz * p[1, 1] - off
    m = 0
    if d0 <= 0:
        out[m, 0] = p[0, 0]; out[m, 1] = p[0, 1]; m += 1
    if d1 <= 0:
        out[m, 0] = p[1, 0]; out[m, 1] = p[1, 1]; m += 1
    if d0 * d1 < 0:
        t = d0 / (d0 - d1)
        out[m, 0] = p[0, 0] + t * (p[1, 0] - p[0, 0])
        out[m, 1] = p[0, 1] + t * (p[1, 1] - p[0, 1])
        m += 1
    return m


@njit(cache=True)
def _collide(wv, wn, nv, A, B, res):
    """Contacts between polygons A and B. res: (2, 5) px, pz, nx, nz, pen (normal from A to B)."""
    sa, ia = _maxsep(wv, wn, nv, A, B)
    if sa > 0:
        return 0
    sb, ib = _maxsep(wv, wn, nv, B, A)
    if sb > 0:
        return 0
    if sb > sa + 1e-5:
        ref = B; inc = A; ri = ib; flip = True
    else:
        ref = A; inc = B; ri = ia; flip = False
    nx = wn[ref, ri, 0]; nz = wn[ref, ri, 1]
    ii = 0
    mn = 1e9
    for i in range(nv[inc]):
        d = wn[inc, i, 0] * nx + wn[inc, i, 1] * nz
        if d < mn:
            mn = d
            ii = i
    p = np.empty((2, 2))
    i2 = (ii + 1) % nv[inc]
    p[0, 0] = wv[inc, ii, 0]; p[0, 1] = wv[inc, ii, 1]
    p[1, 0] = wv[inc, i2, 0]; p[1, 1] = wv[inc, i2, 1]
    r2 = (ri + 1) % nv[ref]
    v1x = wv[ref, ri, 0]; v1z = wv[ref, ri, 1]
    v2x = wv[ref, r2, 0]; v2z = wv[ref, r2, 1]
    tx = v2x - v1x; tz = v2z - v1z
    tl = math.hypot(tx, tz)
    tx /= tl; tz /= tl
    out = np.empty((3, 2))
    m = _clip(p, 2, -tx, -tz, -(tx * v1x + tz * v1z), out)
    if m < 2:
        return 0
    p[0, 0] = out[0, 0]; p[0, 1] = out[0, 1]; p[1, 0] = out[1, 0]; p[1, 1] = out[1, 1]
    m = _clip(p, 2, tx, tz, tx * v2x + tz * v2z, out)
    if m < 2:
        return 0
    cnt = 0
    for k in range(2):
        px = out[k, 0]; pz = out[k, 1]
        sep = nx * (px - v1x) + nz * (pz - v1z)
        if sep <= 0:
            res[cnt, 0] = px - nx * sep / 2
            res[cnt, 1] = pz - nz * sep / 2
            if flip:
                res[cnt, 2] = -nx; res[cnt, 3] = -nz
            else:
                res[cnt, 2] = nx; res[cnt, 3] = nz
            res[cnt, 4] = -sep
            cnt += 1
    return cnt


@njit(cache=True)
def _run(loc, nv, zc, mass, inertia, H, e, d, tilt_deg, vx0, mu, rest, dt, iters, T, record, rec_every):
    P = loc.shape[0]
    NP = 1 + 2 * P
    L = np.zeros((NP, MAXV, 2))
    NV = np.zeros(NP, dtype=np.int64)
    gp = np.array([[-300.0, -20.0], [300.0, -20.0], [300.0, 0.0], [-300.0, 0.0]])
    for k in range(4):
        L[0, k, 0] = gp[k, 0]; L[0, k, 1] = gp[k, 1]
    NV[0] = 4
    for i in range(P):
        NV[1 + i] = nv[i]; NV[1 + P + i] = nv[i]
        for k in range(nv[i]):
            L[1 + i, k, 0] = loc[i, k, 0]; L[1 + i, k, 1] = loc[i, k, 1]
            L[1 + P + i, k, 0] = loc[i, k, 0]; L[1 + P + i, k, 1] = loc[i, k, 1]
    start = np.array([0, 1, 1 + P]); stop = np.array([1, 1 + P, 1 + 2 * P])
    bx = np.zeros(3); bz = np.zeros(3); ba = np.zeros(3)
    vx = np.zeros(3); vz = np.zeros(3); w = np.zeros(3)
    invM = np.array([0.0, 1.0 / mass, 1.0 / mass]); invI = np.array([0.0, 1.0 / inertia, 1.0 / inertia])
    bz[1] = zc
    bx[2] = e; bz[2] = zc; ba[2] = -tilt_deg * math.pi / 180.0
    wv = np.zeros((NP, MAXV, 2)); wn = np.zeros((NP, MAXV, 2)); wbox = np.zeros((NP, 4)); bbox = np.zeros((3, 4))
    _world(bx[2], bz[2], ba[2], L, NV, start[2], stop[2], wv, wn, wbox, bbox, 2)
    zmin = 1e9
    for i in range(start[2], stop[2]):
        for k in range(NV[i]):
            if wv[i, k, 1] < zmin:
                zmin = wv[i, k, 1]
    bz[2] += H + d - zmin
    vx[2] = vx0
    # contacts
    cA = np.zeros(MAXC, dtype=np.int64); cB = np.zeros(MAXC, dtype=np.int64)
    rAx = np.zeros(MAXC); rAz = np.zeros(MAXC); rBx = np.zeros(MAXC); rBz = np.zeros(MAXC)
    cnx = np.zeros(MAXC); cnz = np.zeros(MAXC); ctx = np.zeros(MAXC); ctz = np.zeros(MAXC)
    mN = np.zeros(MAXC); mT = np.zeros(MAXC); bias = np.zeros(MAXC); Pn = np.zeros(MAXC); Pt = np.zeros(MAXC)
    res = np.zeros((2, 5))
    pa = np.array([0, 0, 1]); pb = np.array([1, 2, 2])
    la = np.zeros(NP, dtype=np.int64); lb = np.zeros(NP, dtype=np.int64)
    nrec_max = int(T / dt) // rec_every + 3 if record else 1
    rec = np.zeros((nrec_max, 7)); nrec = 0
    calm = 0.0
    t = 0.0
    N = int(round(T / dt))
    max_contacts = 0
    for step in range(N):
        t = step * dt
        vz[1] -= G * dt; vz[2] -= G * dt
        for b in range(3):
            _world(bx[b], bz[b], ba[b], L, NV, start[b], stop[b], wv, wn, wbox, bbox, b)
        nc = 0
        for q in range(3):
            A = pa[q]; B = pb[q]
            if not _ov(bbox[A, 0], bbox[A, 1], bbox[A, 2], bbox[A, 3], bbox[B, 0], bbox[B, 1], bbox[B, 2], bbox[B, 3]):
                continue
            na = 0
            for i in range(start[A], stop[A]):
                if _ov(wbox[i, 0], wbox[i, 1], wbox[i, 2], wbox[i, 3], bbox[B, 0], bbox[B, 1], bbox[B, 2], bbox[B, 3]):
                    la[na] = i; na += 1
            nb = 0
            for i in range(start[B], stop[B]):
                if _ov(wbox[i, 0], wbox[i, 1], wbox[i, 2], wbox[i, 3], bbox[A, 0], bbox[A, 1], bbox[A, 2], bbox[A, 3]):
                    lb[nb] = i; nb += 1
            for ia_ in range(na):
                PA = la[ia_]
                for ib_ in range(nb):
                    PB = lb[ib_]
                    if not _ov(wbox[PA, 0], wbox[PA, 1], wbox[PA, 2], wbox[PA, 3], wbox[PB, 0], wbox[PB, 1], wbox[PB, 2], wbox[PB, 3]):
                        continue
                    m = _collide(wv, wn, NV, PA, PB, res)
                    for k in range(m):
                        if nc >= MAXC:
                            break
                        px = res[k, 0]; pz = res[k, 1]; nx = res[k, 2]; nz = res[k, 3]; pen = res[k, 4]
                        cA[nc] = A; cB[nc] = B
                        rAx[nc] = px - bx[A]; rAz[nc] = pz - bz[A]; rBx[nc] = px - bx[B]; rBz[nc] = pz - bz[B]
                        cnx[nc] = nx; cnz[nc] = nz
                        rnA = rAx[nc] * nz - rAz[nc] * nx; rnB = rBx[nc] * nz - rBz[nc] * nx
                        mN[nc] = 1.0 / (invM[A] + invM[B] + invI[A] * rnA * rnA + invI[B] * rnB * rnB)
                        ctx[nc] = nz; ctz[nc] = -nx
                        rtA = rAx[nc] * ctz[nc] - rAz[nc] * ctx[nc]; rtB = rBx[nc] * ctz[nc] - rBz[nc] * ctx[nc]
                        mT[nc] = 1.0 / (invM[A] + invM[B] + invI[A] * rtA * rtA + invI[B] * rtB * rtB)
                        rvx = vx[B] - w[B] * rBz[nc] - (vx[A] - w[A] * rAz[nc])
                        rvz = vz[B] + w[B] * rBx[nc] - (vz[A] + w[A] * rAx[nc])
                        vn = rvx * nx + rvz * nz
                        bias[nc] = 0.2 / dt * max(0.0, pen - 0.005)
                        if vn < -60:
                            bias[nc] = max(bias[nc], -rest * vn)
                        Pn[nc] = 0.0; Pt[nc] = 0.0
                        nc += 1
        if nc > max_contacts:
            max_contacts = nc
        for it in range(iters):
            for c in range(nc):
                A = cA[c]; B = cB[c]
                rvx = vx[B] - w[B] * rBz[c] - (vx[A] - w[A] * rAz[c])
                rvz = vz[B] + w[B] * rBx[c] - (vz[A] + w[A] * rAx[c])
                vn = rvx * cnx[c] + rvz * cnz[c]
                dPn = mN[c] * (-vn + bias[c])
                P0 = Pn[c]
                Pn[c] = max(P0 + dPn, 0.0)
                dPn = Pn[c] - P0
                Px = cnx[c] * dPn; Pz = cnz[c] * dPn
                vx[A] -= Px * invM[A]; vz[A] -= Pz * invM[A]; w[A] -= invI[A] * (rAx[c] * Pz - rAz[c] * Px)
                vx[B] += Px * invM[B]; vz[B] += Pz * invM[B]; w[B] += invI[B] * (rBx[c] * Pz - rBz[c] * Px)
                rvx = vx[B] - w[B] * rBz[c] - (vx[A] - w[A] * rAz[c])
                rvz = vz[B] + w[B] * rBx[c] - (vz[A] + w[A] * rAx[c])
                vt = rvx * ctx[c] + rvz * ctz[c]
                dPt = -mT[c] * vt
                mx = mu * Pn[c]
                T0 = Pt[c]
                Pt[c] = min(max(T0 + dPt, -mx), mx)
                dPt = Pt[c] - T0
                Px = ctx[c] * dPt; Pz = ctz[c] * dPt
                vx[A] -= Px * invM[A]; vz[A] -= Pz * invM[A]; w[A] -= invI[A] * (rAx[c] * Pz - rAz[c] * Px)
                vx[B] += Px * invM[B]; vz[B] += Pz * invM[B]; w[B] += invI[B] * (rBx[c] * Pz - rBz[c] * Px)
        for b in range(1, 3):
            bx[b] += vx[b] * dt; bz[b] += vz[b] * dt; ba[b] += w[b] * dt
        if record and step % rec_every == 0 and nrec < nrec_max - 1:
            rec[nrec, 0] = t
            for b in range(1, 3):
                rec[nrec, 3 * b - 2] = bx[b] + zc * math.sin(ba[b])
                rec[nrec, 3 * b - 1] = bz[b] - zc * math.cos(ba[b])
                rec[nrec, 3 * b] = ba[b]
            nrec += 1
        moving = (math.hypot(vx[1], vz[1]) > 2 or abs(w[1]) > 0.05 or math.hypot(vx[2], vz[2]) > 2 or abs(w[2]) > 0.05)
        calm = 0.0 if moving else calm + dt
        if calm > 0.08 and t > 0.05:
            break
    out = np.zeros(7)
    out[0] = t
    for b in range(1, 3):
        out[3 * b - 2] = bx[b] + zc * math.sin(ba[b])
        out[3 * b - 1] = bz[b] - zc * math.cos(ba[b])
        out[3 * b] = ba[b]
    if record:
        rec[nrec, :] = out
        nrec += 1
    return out, rec[:nrec], max_contacts


def classify(g, low, up):
    """seated | perched | fell | knocked, as web/sim2d.js."""
    deg = 180 / math.pi
    dx = up[0] - low[0]
    dz = up[1] - low[1]
    tilt = abs(up[2] * deg)
    ltilt = abs(low[2] * deg)
    if ltilt > 10:
        code = "knocked"
    elif up[1] < g["band_h"] * 0.6 or tilt > 45:
        code = "fell"
    elif abs(dx) < 1.0 and tilt < 3 and abs(dz - g["seat"]) < 0.6:
        code = "seated"
    else:
        code = "perched"
    return dict(code=code, dx=float(dx), dz=float(dz), tilt=float(tilt), ltilt=float(ltilt))


class Section2D:
    """A piece and its section polygons, packed once for repeated simulations."""

    def __init__(self, g, parts):
        self.g = g
        self.loc, self.nv = pack_parts(parts, g["zc"])

    def simulate(self, e, d=1.0, tilt=0.0, vx=0.0, mu=0.4, rest=0.15, dt=2e-4, iters=16, T=1.2, record=False):
        """Release a piece at offset e (mm) over an identical piece on the board (SP-6)."""
        out, rec, mc = _run(self.loc, self.nv, self.g["zc"], self.g["m"], self.g["I"], self.g["H"],
                            float(e), float(d), float(tilt), float(vx), float(mu), float(rest),
                            dt, iters, T, record, int(round((1 / 120) / dt)))
        r = classify(self.g, out[1:4], out[4:7])
        r.update(t=float(out[0]), low=out[1:4].tolist(), up=out[4:7].tolist(), max_contacts=int(mc))
        if record:
            r["frames"] = rec.tolist()
        return r
