// Piece geometry and printed-surface model (DD-STACK SP-2, SP-3, SP-5). Same algorithms as
// stackpieces/geometry.py and surface.py; test_parity.py checks they agree.
// Browser: window.StackGeom. Node: require('./geometry.js').
// Copyright (c) 2026, Tico Hannan. BSD-3-Clause.
(function (root, factory) {
  if (typeof module !== 'undefined' && module.exports) module.exports = factory();
  else root.StackGeom = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';
  const rad = d => d * Math.PI / 180;
  const PRESETS = {
    S40: { family: 'shoulder', D: 25, band_h: 12, shoulder: 2, tip_d: 4, alpha: 40, clr: 0.5, vclr: 0.5, ct: 0.5, cb: 0.4 },
    N35: { family: 'nesting', D: 25, band_h: 12, tip_d: 1.6, alpha: 35, land: 1, vclr: 0.6, ctip: 0.4, cb: 0.4 },
    CYL: { family: 'cylinder', D: 25, band_h: 15, ct: 0.5, cb: 0.4 }
  };

  function makePiece(p, density = 1.24e-3) {
    let g;
    if (p.family === 'shoulder') g = shoulder(p);
    else if (p.family === 'nesting') g = nesting(p);
    else if (p.family === 'cylinder') g = cylinder(p);
    else throw new Error('unknown family ' + p.family);
    g.params = Object.assign({}, p); g.family = p.family;
    Object.assign(g, massProps(g.profile, density));
    g.king_h = g.seat + g.H;
    return g;
  }
  function shoulder(p) {
    const R = p.D / 2, Hc = p.band_h, sh = p.shoulder, rt = p.tip_d / 2, ct = p.ct, cb = p.cb;
    const ta = Math.tan(rad(p.alpha)), Rb = R - sh;
    if (!(0 < rt && rt < Rb && Rb < R - ct)) throw new Error('need 0 < tip radius < cone base radius < D/2 - ct');
    const h = (Rb - rt) / ta, H = Hc + h, Rm = Rb + p.clr, hs = h + p.vclr, rs = Rm - hs * ta;
    if (rs <= 0.05) throw new Error('socket roof radius <= 0: increase clr or tip_d');
    if (hs >= Hc) throw new Error('socket depth >= band_h: the socket does not fit in the band');
    const profile = [[0, hs], [rs, hs], [Rm, 0], [R - cb, 0], [R, cb], [R, Hc - ct], [R - ct, Hc], [Rb, Hc], [rt, H], [0, H]];
    const left = [[-R + cb, 0], [-Rm, 0], [-rs, hs], [-rs, Hc], [-R + ct, Hc], [-R, Hc - ct], [-R, cb]];
    const right = left.map(([x, z]) => [-x, z]).reverse();
    const section = [left, right, [[-rs, hs], [rs, hs], [rs, Hc], [-rs, Hc]], [[-Rb, Hc], [Rb, Hc], [rt, H], [-rt, H]]];
    return { R, H, band_h: Hc, h, Rb, rt, Rm, rs, hs, seat: Hc, cone_start: Hc, capture: Rm - rt, profile, section,
      socket_overhang_deg: p.alpha, bridge: 2 * rs, rim: R - cb - Rm, roof: Hc - hs, straight_band: Hc - ct - cb };
  }
  function nesting(p) {
    const R = p.D / 2, Hb = p.band_h, rt = p.tip_d / 2, land = p.land, cb = p.cb;
    const a = rad(p.alpha), ta = Math.tan(a);
    const H = Hb + (R - rt) / ta, Rm = R - land, seat = Hb + land / ta, roof = H - seat + p.vclr;
    const rs = rt + p.ctip, ts = (Rm - rs) / roof, ro = R - (roof - Hb) * ta;
    if (!(roof > Hb && ro > rs)) throw new Error('socket roof must be above band_h and inside the cone');
    const profile = [[0, roof], [rs, roof], [Rm, 0], [R - cb, 0], [R, cb], [R, Hb], [rt, H], [0, H]];
    const strip = [[rs, roof], [Rm, 0], [R - cb, 0], [R, cb], [R, Hb], [ro, roof]];
    const left = strip.map(([x, z]) => [-x, z]).reverse();
    const section = [left, strip, [[-ro, roof], [ro, roof], [rt, H], [-rt, H]]];
    const wall = Math.min(ro - rs, R - (Rm - Hb * ts)) * Math.cos(a);
    return { R, H, band_h: Hb, h: H - Hb, rt, Rm, rs, hs: roof, ro, seat, cone_start: Hb, capture: Rm - rt, profile, section,
      socket_overhang_deg: Math.atan(ts) * 180 / Math.PI, bridge: 2 * rs, rim: land - cb, roof: H - roof, shell_wall: wall, straight_band: Hb - cb };
  }
  function cylinder(p) {
    const R = p.D / 2, H = p.band_h, ct = p.ct, cb = p.cb;
    const profile = [[0, 0], [R - cb, 0], [R, cb], [R, H - ct], [R - ct, H], [0, H]];
    const section = [[[-R + cb, 0], [R - cb, 0], [R, cb], [R, H - ct], [R - ct, H], [-R + ct, H], [-R, H - ct], [-R, cb]]];
    return { R, H, band_h: H, h: 0, rt: 0, Rm: 0, seat: H, cone_start: H, capture: 0, profile, section,
      socket_overhang_deg: 0, bridge: 0, rim: R - cb, roof: H, straight_band: H - ct - cb };
  }
  function inPoly(x, y, poly) {
    let c = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
      if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) c = !c;
    }
    return c;
  }
  // same grid as numpy.arange(d/2, max, d)
  function massProps(profile, density = 1.24e-3, d = 0.05) {
    let rmax = 0, zmax = 0; profile.forEach(([r, z]) => { rmax = Math.max(rmax, r); zmax = Math.max(zmax, z); });
    const nr = Math.ceil((rmax - d / 2) / d), nz = Math.ceil((zmax - d / 2) / d);
    let V = 0, Mz = 0, Srr = 0; const zs = [], ws = [];
    for (let j = 0; j < nz; j++) {
      const z = d / 2 + j * d; let w = 0, wr = 0;
      for (let i = 0; i < nr; i++) { const r = d / 2 + i * d; if (inPoly(r, z, profile)) { const dv = 2 * Math.PI * r * d * d; w += dv; wr += dv * r * r; } }
      V += w; Mz += w * z; Srr += wr; zs.push(z); ws.push(w);
    }
    const zc = Mz / V; let Szz = 0; for (let j = 0; j < nz; j++) Szz += ws[j] * (zs[j] - zc) * (zs[j] - zc);
    return { V, zc, m: V * density, I: density * (Srr / 2 + Szz), Iz: density * Srr };
  }
  function radiiAt(profile, z) {
    const xs = [];
    for (let i = 0; i < profile.length; i++) {
      const [r1, z1] = profile[i], [r2, z2] = profile[(i + 1) % profile.length];
      if ((z1 > z) !== (z2 > z)) xs.push(r1 + (r2 - r1) * (z - z1) / (z2 - z1));
    }
    return xs.sort((a, b) => a - b);
  }
  function stadium(x0, x1, z0, t) {
    const r = t / 2, zc = z0 + r, pts = [];
    for (let k = -2; k <= 2; k++) { const a = k * Math.PI / 4; pts.push([x1 - r + r * Math.cos(a), zc + r * Math.sin(a)]); }
    for (let k = 2; k <= 6; k++) { const a = k * Math.PI / 4; pts.push([x0 + r + r * Math.cos(a), zc + r * Math.sin(a)]); }
    return pts;
  }
  function layers(g, t) {
    const out = [], n = Math.round(g.H / t);
    for (let i = 0; i < n; i++) {
      const z0 = i * t, zm = z0 + t / 2; if (zm >= g.H) break;
      const xs = radiiAt(g.profile, zm);
      for (let k = 0; k + 1 < xs.length; k += 2) {
        const a = xs[k], b = xs[k + 1]; if (b - a < t * 0.5) continue;
        if (a < 1e-9) out.push(stadium(-b, b, z0, t)); else { out.push(stadium(-b, -a, z0, t)); out.push(stadium(a, b, z0, t)); }
      }
    }
    return out;
  }
  function section(g, variant = 'smooth', t = 0.2) {
    if (variant === 'smooth') return g.section.map(P => P.map(v => v.slice()));
    if (variant === 'layered') return layers(g, t);
    if (variant === 'hybrid') {
      if (g.family === 'nesting') throw new Error('hybrid is defined for the shoulder and cylinder families only');
      const below = layers(g, t).filter(P => P[0][1] < g.cone_start - 1e-6);
      if (g.family === 'shoulder') below.push(g.section[3].map(v => v.slice()));
      return below;
    }
    throw new Error('unknown surface variant ' + variant);
  }

  // Print and robot checks (same as stackpieces.geometry.checks)
  function checks(g, c) {
    const pr = c.printer, gr = c.gripper, bd = c.board, goal = c.goal, out = [];
    const add = (id, ok, value, limit, note) => out.push({ id, ok: !!ok, value: Math.round(value * 1000) / 1000, limit, note });
    if (g.family !== 'cylinder') {
      add('overhang', g.socket_overhang_deg <= pr.overhang_max_deg + 1e-9, g.socket_overhang_deg, `<= ${pr.overhang_max_deg} deg from vertical`, 'socket wall printed upright without support');
      add('bridge', g.bridge <= pr.bridge_max_mm, g.bridge, `<= ${pr.bridge_max_mm} mm`, 'socket roof diameter');
      add('roof', g.roof >= pr.min_wall_mm, g.roof, `>= ${pr.min_wall_mm} mm`, 'material above the socket roof');
    }
    add('rim', g.rim >= pr.min_wall_mm, g.rim, `>= ${pr.min_wall_mm} mm`, 'flat ring on the bed / board (thinnest wall at the bottom)');
    if (g.family === 'nesting') add('shell_wall', g.shell_wall >= pr.min_wall_mm, g.shell_wall, `>= ${pr.min_wall_mm} mm`, 'cone shell thickness');
    add('grip_band_min', g.band_h >= gr.grip_band_min_mm, g.band_h, `>= ${gr.grip_band_min_mm} mm`, 'cylindrical band the fingers grip');
    add('grip_band_max', g.band_h <= gr.grip_band_max_mm + 1e-9, g.band_h, `<= ${gr.grip_band_max_mm} mm`, "Tico's band range");
    add('height', g.H <= goal.max_height_mm + 1e-9, g.H, `<= ${goal.max_height_mm} mm`, 'single piece height');
    const gap = (bd.pieces_on_diagonals_only !== false ? bd.square_mm * Math.SQRT2 : bd.square_mm) - 2 * g.R;
    const ft = gr.finger_thickness_mm;
    add('finger_room', ft != null && gap >= ft, gap, `>= finger thickness (${ft == null ? 'not measured' : ft + ' mm'})`, 'gap between pieces on diagonal neighbor squares');
    return out;
  }
  // Watertight revolve mesh and ASCII STL (same as stackpieces.export.revolve_mesh)
  function revolveMesh(profile, seg = 128) {
    const V = [], F = [], idx = [];
    for (const [r, z] of profile) { idx.push(V.length); if (r < 1e-9) V.push([0, 0, z]); else for (let j = 0; j < seg; j++) V.push([r * Math.cos(2 * Math.PI * j / seg), r * Math.sin(2 * Math.PI * j / seg), z]); }
    const n = profile.length;
    for (let i = 0; i < n; i++) {
      const a = i, b = (i + 1) % n, ra = profile[a][0], rb = profile[b][0];
      if (ra < 1e-9 && rb < 1e-9) continue;
      for (let j = 0; j < seg; j++) { const j2 = (j + 1) % seg;
        if (ra < 1e-9) F.push([idx[a], idx[b] + j2, idx[b] + j]);
        else if (rb < 1e-9) F.push([idx[a] + j, idx[a] + j2, idx[b]]);
        else { F.push([idx[a] + j, idx[a] + j2, idx[b] + j2]); F.push([idx[a] + j, idx[b] + j2, idx[b] + j]); } }
    }
    let vol = 0; for (const [i, j, k] of F) { const A = V[i], B = V[j], C = V[k]; vol += (A[0] * (B[1] * C[2] - B[2] * C[1]) - A[1] * (B[0] * C[2] - B[2] * C[0]) + A[2] * (B[0] * C[1] - B[1] * C[0])) / 6; }
    if (vol < 0) { F.forEach(f => f.reverse()); vol = -vol; }
    return { V, F, volume: vol };
  }
  function stlAscii(profile, name = 'piece', seg = 128) {
    const { V, F } = revolveMesh(profile, seg); const L = ['solid ' + name];
    for (const [i, j, k] of F) { const A = V[i], B = V[j], C = V[k];
      const u = [B[0] - A[0], B[1] - A[1], B[2] - A[2]], v = [C[0] - A[0], C[1] - A[1], C[2] - A[2]];
      let nx = u[1] * v[2] - u[2] * v[1], ny = u[2] * v[0] - u[0] * v[2], nz = u[0] * v[1] - u[1] * v[0]; const l = Math.hypot(nx, ny, nz) || 1;
      L.push(` facet normal ${(nx / l).toFixed(6)} ${(ny / l).toFixed(6)} ${(nz / l).toFixed(6)}`, '  outer loop');
      for (const P of [A, B, C]) L.push(`   vertex ${P[0].toFixed(5)} ${P[1].toFixed(5)} ${P[2].toFixed(5)}`);
      L.push('  endloop', ' endfacet'); }
    L.push('endsolid ' + name); return L.join('\n');
  }
  return { PRESETS, makePiece, massProps, radiiAt, layers, section, stadium, checks, revolveMesh, stlAscii };
});
