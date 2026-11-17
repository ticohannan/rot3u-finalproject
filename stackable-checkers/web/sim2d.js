// 2D section settling simulation (DD-STACK SP-6, SP-9). Reference engine; stackpieces/sim2d.py is
// a Numba port checked against it (test_parity.py). Units mm, g, s. Browser: window.StackSim2D.
// Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
(function (root, factory) {
  if (typeof module !== 'undefined' && module.exports) module.exports = factory();
  else root.StackSim2D = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';
  const G = 9810;

  function body(g, parts, x, z, a) {
    return { g, x, z: z + g.zc, a, vx: 0, vz: 0, w: 0, invM: 1 / g.m, invI: 1 / g.I,
      parts: parts.map(P => P.map(([px, pz]) => [px, pz - g.zc])), wp: null, box: null };
  }
  function ground() {
    return { x: 0, z: 0, a: 0, vx: 0, vz: 0, w: 0, invM: 0, invI: 0, parts: [[[-300, -20], [300, -20], [300, 0], [-300, 0]]], wp: null, box: null };
  }
  function worldPolys(b) {
    const c = Math.cos(b.a), s = Math.sin(b.a);
    let X0 = 1e9, X1 = -1e9, Z0 = 1e9, Z1 = -1e9;
    b.wp = b.parts.map(poly => {
      const v = poly.map(([px, pz]) => [b.x + c * px - s * pz, b.z + s * px + c * pz]);
      let x0 = 1e9, x1 = -1e9, z0 = 1e9, z1 = -1e9;
      for (const [x, z] of v) { if (x < x0) x0 = x; if (x > x1) x1 = x; if (z < z0) z0 = z; if (z > z1) z1 = z; }
      if (x0 < X0) X0 = x0; if (x1 > X1) X1 = x1; if (z0 < Z0) Z0 = z0; if (z1 > Z1) Z1 = z1;
      const n = v.map((p, i) => { const q = v[(i + 1) % v.length]; const dx = q[0] - p[0], dz = q[1] - p[1]; const l = Math.hypot(dx, dz) || 1e-9; return [dz / l, -dx / l]; });
      return { v, n, box: [x0, x1, z0, z1] };
    });
    b.box = [X0, X1, Z0, Z1];
  }
  function maxSep(A, B) {
    let best = -1e9, bi = 0;
    for (let i = 0; i < A.v.length; i++) {
      const n = A.n[i], p = A.v[i]; let mn = 1e9;
      for (const q of B.v) { const d = n[0] * (q[0] - p[0]) + n[1] * (q[1] - p[1]); if (d < mn) mn = d; }
      if (mn > best) { best = mn; bi = i; }
    }
    return [best, bi];
  }
  function clip(pts, nrm, off) {
    const out = [];
    const d0 = nrm[0] * pts[0][0] + nrm[1] * pts[0][1] - off, d1 = nrm[0] * pts[1][0] + nrm[1] * pts[1][1] - off;
    if (d0 <= 0) out.push(pts[0]);
    if (d1 <= 0) out.push(pts[1]);
    if (d0 * d1 < 0) { const t = d0 / (d0 - d1); out.push([pts[0][0] + t * (pts[1][0] - pts[0][0]), pts[0][1] + t * (pts[1][1] - pts[0][1])]); }
    return out;
  }
  function collide(A, B) {
    const [sa, ia] = maxSep(A, B); if (sa > 0) return [];
    const [sb, ib] = maxSep(B, A); if (sb > 0) return [];
    let ref, inc, ri, flip;
    if (sb > sa + 1e-5) { ref = B; inc = A; ri = ib; flip = true; } else { ref = A; inc = B; ri = ia; flip = false; }
    const n = ref.n[ri]; let ii = 0, mn = 1e9;
    for (let i = 0; i < inc.n.length; i++) { const d = inc.n[i][0] * n[0] + inc.n[i][1] * n[1]; if (d < mn) { mn = d; ii = i; } }
    let pts = [inc.v[ii], inc.v[(ii + 1) % inc.v.length]];
    const v1 = ref.v[ri], v2 = ref.v[(ri + 1) % ref.v.length];
    let tx = v2[0] - v1[0], tz = v2[1] - v1[1]; const tl = Math.hypot(tx, tz); tx /= tl; tz /= tl;
    pts = clip(pts, [-tx, -tz], -(tx * v1[0] + tz * v1[1])); if (pts.length < 2) return [];
    pts = clip(pts, [tx, tz], tx * v2[0] + tz * v2[1]); if (pts.length < 2) return [];
    const out = [];
    for (let k = 0; k < 2; k++) {
      const p = pts[k];
      const sep = n[0] * (p[0] - v1[0]) + n[1] * (p[1] - v1[1]);
      if (sep <= 0) out.push({ p: [p[0] - n[0] * sep / 2, p[1] - n[1] * sep / 2], n: flip ? [-n[0], -n[1]] : [n[0], n[1]], pen: -sep });
    }
    return out;
  }
  const ov = (a, b) => a[0] <= b[1] + 0.02 && b[0] <= a[1] + 0.02 && a[2] <= b[3] + 0.02 && b[2] <= a[3] + 0.02;
  function relV(c) { const A = c.A, B = c.B; return [B.vx - B.w * c.rB[1] - (A.vx - A.w * c.rA[1]), B.vz + B.w * c.rB[0] - (A.vz + A.w * c.rA[0])]; }
  function apply(c, Px, Pz) {
    const A = c.A, B = c.B;
    A.vx -= Px * A.invM; A.vz -= Pz * A.invM; A.w -= A.invI * (c.rA[0] * Pz - c.rA[1] * Px);
    B.vx += Px * B.invM; B.vz += Pz * B.invM; B.w += B.invI * (c.rB[0] * Pz - c.rB[1] * Px);
  }
  function basePose(b) { return [b.x + b.g.zc * Math.sin(b.a), b.z - b.g.zc * Math.cos(b.a), b.a]; }

  // g: piece (makePiece); parts: section polygons (StackGeom.section)
  // sc: {e, d (release height above the lower tip), tilt (deg, outer side down), vx, mu, rest}
  // opts: {dt, iters, T, record}
  function simulate(g, parts, sc, opts = {}) {
    const dt = opts.dt || 2e-4, iters = opts.iters || 16, T = opts.T || 1.2;
    const rest = sc.rest == null ? 0.15 : sc.rest, mu = sc.mu;
    const gr = ground(), low = body(g, parts, 0, 0, 0), up = body(g, parts, sc.e, 0, 0);
    up.a = -(sc.tilt || 0) * Math.PI / 180;   // outer (+x) side down = clockwise = negative angle
    worldPolys(up);
    let zmin = 1e9; for (const P of up.wp) for (const v of P.v) if (v[1] < zmin) zmin = v[1];
    up.z += g.H + sc.d - zmin; up.vx = sc.vx || 0;
    const bodies = [gr, low, up], pairs = [[gr, low], [gr, up], [low, up]];
    const rec = opts.record ? [] : null, recEvery = Math.round((1 / 120) / dt);
    let calm = 0, t = 0, maxContacts = 0;
    const N = Math.round(T / dt);
    for (let step = 0; step < N; step++) {
      t = step * dt;
      low.vz -= G * dt; up.vz -= G * dt;
      for (const b of bodies) worldPolys(b);
      const cs = [];
      for (const [A, B] of pairs) {
        if (!ov(A.box, B.box)) continue;
        const la = A.wp.filter(P => ov(P.box, B.box)), lb = B.wp.filter(P => ov(P.box, A.box));
        for (const PA of la) for (const PB of lb) {
          if (!ov(PA.box, PB.box)) continue;
          for (const c of collide(PA, PB)) {
            c.A = A; c.B = B; c.rA = [c.p[0] - A.x, c.p[1] - A.z]; c.rB = [c.p[0] - B.x, c.p[1] - B.z];
            const rnA = c.rA[0] * c.n[1] - c.rA[1] * c.n[0], rnB = c.rB[0] * c.n[1] - c.rB[1] * c.n[0];
            c.mN = 1 / (A.invM + B.invM + A.invI * rnA * rnA + B.invI * rnB * rnB);
            c.t = [c.n[1], -c.n[0]];
            const rtA = c.rA[0] * c.t[1] - c.rA[1] * c.t[0], rtB = c.rB[0] * c.t[1] - c.rB[1] * c.t[0];
            c.mT = 1 / (A.invM + B.invM + A.invI * rtA * rtA + B.invI * rtB * rtB);
            const rv = relV(c), vn = rv[0] * c.n[0] + rv[1] * c.n[1];
            c.bias = 0.2 / dt * Math.max(0, c.pen - 0.005);
            if (vn < -60) c.bias = Math.max(c.bias, -rest * vn);
            c.Pn = 0; c.Pt = 0; cs.push(c);
          }
        }
      }
      if (cs.length > maxContacts) maxContacts = cs.length;
      for (let k = 0; k < iters; k++) for (const c of cs) {
        let dv = relV(c); const vn = dv[0] * c.n[0] + dv[1] * c.n[1];
        let dPn = c.mN * (-vn + c.bias); const P0 = c.Pn; c.Pn = Math.max(P0 + dPn, 0); dPn = c.Pn - P0;
        apply(c, c.n[0] * dPn, c.n[1] * dPn);
        dv = relV(c); const vt = dv[0] * c.t[0] + dv[1] * c.t[1];
        let dPt = -c.mT * vt; const mx = mu * c.Pn, T0 = c.Pt; c.Pt = Math.min(Math.max(T0 + dPt, -mx), mx); dPt = c.Pt - T0;
        apply(c, c.t[0] * dPt, c.t[1] * dPt);
      }
      for (const b of [low, up]) { b.x += b.vx * dt; b.z += b.vz * dt; b.a += b.w * dt; }
      if (rec && step % recEvery === 0) rec.push([t].concat(basePose(low), basePose(up)));
      const moving = Math.hypot(low.vx, low.vz) > 2 || Math.abs(low.w) > 0.05 || Math.hypot(up.vx, up.vz) > 2 || Math.abs(up.w) > 0.05;
      calm = moving ? 0 : calm + dt;
      if (calm > 0.08 && t > 0.05) break;
    }
    if (rec) rec.push([t].concat(basePose(low), basePose(up)));
    const L = basePose(low), U = basePose(up);
    return Object.assign(classify(g, L, U), { t, frames: rec, maxContacts });
  }
  function classify(g, L, U) {
    const deg = 180 / Math.PI, dx = U[0] - L[0], dz = U[1] - L[1], tilt = Math.abs(U[2] * deg), ltilt = Math.abs(L[2] * deg);
    let code;
    if (ltilt > 10) code = 'knocked';
    else if (U[1] < g.band_h * 0.6 || tilt > 45) code = 'fell';
    else if (Math.abs(dx) < 1.0 && tilt < 3 && Math.abs(dz - g.seat) < 0.6) code = 'seated';
    else code = 'perched';
    return { code, dx, dz, tilt, ltilt, low: L, up: U };
  }

  // Seeded generator shared with stackpieces/montecarlo.py (32-bit LCG, Box-Muller).
  function rng(seed) { let s = seed >>> 0; return () => { s = (Math.imul(s, 1664525) + 1013904223) >>> 0; return s / 4294967296; }; }
  function randn(r) { const u = r(), v = r(); return Math.sqrt(-2 * Math.log(u + 1e-12)) * Math.cos(2 * Math.PI * v); }
  // release conditions for offset index i (constraints.physics)
  function sampleReleases(phys, seed, i, n) {
    const r = rng(seed + i * 7919), out = [];
    const [m0, m1] = phys.friction_range, [d0, d1] = phys.release.height_above_tip_mm;
    for (let k = 0; k < n; k++) {
      const mu = m0 + (m1 - m0) * r(), d = d0 + (d1 - d0) * r();
      const tilt = phys.release.tilt_sigma_deg * randn(r), vx = phys.release.sideways_speed_sigma_mm_s * randn(r);
      out.push({ mu, d, tilt, vx, rest: phys.restitution });
    }
    return out;
  }
  return { simulate, classify, rng, randn, sampleReleases, G };
});
