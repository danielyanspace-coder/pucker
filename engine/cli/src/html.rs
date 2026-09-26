//! Self-contained 3D report (Three.js from CDN) to eyeball a packing result.

use pucker_core::{PackingRequest, PackingResult};
use serde_json::json;

pub fn render(req: &PackingRequest, res: &PackingResult) -> String {
    let names: std::collections::HashMap<&str, &str> = req.items.iter().map(|i| (i.id.as_str(), i.name.as_str())).collect();
    let bins: Vec<_> = res
        .bins
        .iter()
        .map(|b| {
            json!({
                "id": b.bin_id, "name": b.place_name, "type": b.place_type,
                "w": b.width, "d": b.depth, "h": b.height, "base": b.base_z,
                "stats": {
                    "items": b.placed_items.len(), "weight": b.total_weight, "height": b.used_height,
                    "utilization": b.utilization, "compactness": b.compactness,
                    "stability": b.stability_score, "cg": b.cg_offset_mm,
                },
                "boxes": b.placed_items.iter().map(|p| json!([
                    p.x, p.y, p.z, p.width, p.depth, p.height, p.sku,
                    names.get(p.item_id.as_str()).copied().unwrap_or(""),
                    p.fragility, p.weight, (p.support_ratio * 100.0).round(),
                    (p.received_top_load * 10.0).round() / 10.0, (p.max_top_load * 10.0).round() / 10.0,
                ])).collect::<Vec<_>>(),
            })
        })
        .collect();
    let data = json!({
        "bins": bins,
        "warnings": res.warnings.iter().map(|w| json!({"s": w.severity, "m": w.message})).collect::<Vec<_>>(),
        "summary": res.summary,
        "valid": res.validation.valid,
        "violations": res.validation.violations.len(),
        "time": res.diagnostics.calculation_time,
    });
    TEMPLATE.replace("__DATA__", &data.to_string().replace("</", "<\\/"))
}

const TEMPLATE: &str = r##"<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pucker — раскладка</title>
<style>
  :root { --bg:#f6f6f4; --panel:#ffffff; --text:#1d1d1b; --muted:#6b6b66; --line:#e2e2dc; --accent:#2f6fde; --bad:#c83a2e; --warn:#b7791f; }
  @media (prefers-color-scheme: dark) { :root { --bg:#161615; --panel:#20201e; --text:#ececea; --muted:#9a9a94; --line:#33332f; --accent:#6d9cff; --bad:#ff7a6e; --warn:#e0b050; } }
  * { box-sizing: border-box; }
  body { margin:0; font:14px/1.45 system-ui, sans-serif; background:var(--bg); color:var(--text); display:flex; height:100vh; }
  aside { width:340px; flex:none; overflow:auto; padding:16px; background:var(--panel); border-right:1px solid var(--line); }
  main { flex:1; position:relative; min-width:0; }
  canvas { display:block; }
  h1 { font-size:18px; margin:0 0 4px; }
  .muted { color:var(--muted); }
  .kpis { display:grid; grid-template-columns:1fr 1fr; gap:8px; margin:12px 0; }
  .kpi { border:1px solid var(--line); border-radius:8px; padding:8px; }
  .kpi b { display:block; font-size:18px; font-variant-numeric:tabular-nums; }
  select, input[type=range] { width:100%; }
  label { display:block; margin:10px 0 4px; color:var(--muted); font-size:12px; }
  .warn { border-left:3px solid var(--warn); padding:6px 8px; margin:8px 0; background:color-mix(in srgb, var(--warn) 10%, transparent); }
  .warn.error { border-color:var(--bad); background:color-mix(in srgb, var(--bad) 10%, transparent); }
  #tip { position:absolute; pointer-events:none; background:var(--panel); border:1px solid var(--line); border-radius:6px; padding:6px 8px; font-size:12px; display:none; max-width:280px; }
  .ok { color:#2e8b57; } .bad { color:var(--bad); }
  @media (max-width: 720px) { body { flex-direction:column; } aside { width:auto; max-height:45vh; border-right:0; border-bottom:1px solid var(--line); } }
</style>
</head>
<body>
<aside>
  <h1>Pucker</h1>
  <div class="muted" id="head"></div>
  <div class="kpis" id="kpis"></div>
  <label>Место погрузки</label><select id="bin"></select>
  <label>Цвет</label>
  <select id="color"><option value="frag">По хрупкости (1 — красный, прочные — синие)</option><option value="sku">По артикулу</option><option value="load">По нагрузке сверху</option></select>
  <label>Порядок погрузки: <span id="stepv"></span></label><input type="range" id="step" min="0" value="0">
  <div id="warns"></div>
</aside>
<main><div id="tip"></div></main>
<script type="importmap">{ "imports": { "three": "https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js", "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.170.0/examples/jsm/" } }</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const D = __DATA__;
const $ = id => document.getElementById(id);
const pct = x => (x*100).toFixed(1) + '%';
const s = D.summary;
$('head').textContent = `Коробок ${s.items_total}, размещено ${s.items_placed}, мест ${s.bins_used} · ${D.time.toFixed(2)} с`;
$('kpis').innerHTML = `<div class="kpi">Заполнение<b>${pct(s.utilization)}</b></div><div class="kpi">Плотность блока<b>${pct(s.compactness)}</b></div>`
  + `<div class="kpi">Не поместилось<b>${s.items_unplaced}</b></div><div class="kpi">Проверка<b class="${D.valid?'ok':'bad'}">${D.valid?'OK':D.violations+' ошибок'}</b></div>`;
$('warns').innerHTML = D.warnings.map(w => `<div class="warn ${w.s}">${w.m}</div>`).join('');
D.bins.forEach((b,i) => $('bin').add(new Option(`${b.id} — ${b.stats.items} кор., ${pct(b.stats.compactness)}`, i)));

const main = document.querySelector('main');
const renderer = new THREE.WebGLRenderer({ antialias:true });
renderer.setPixelRatio(devicePixelRatio);
main.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const dark = matchMedia('(prefers-color-scheme: dark)').matches;
scene.background = new THREE.Color(dark ? 0x161615 : 0xf6f6f4);
const camera = new THREE.PerspectiveCamera(40, 1, 10, 200000);
const controls = new OrbitControls(camera, renderer.domElement);
scene.add(new THREE.HemisphereLight(0xffffff, 0x888877, 2.2));
const sun = new THREE.DirectionalLight(0xffffff, 1.4); sun.position.set(1, 2, 1.5); scene.add(sun);
let group = null, mesh = null, bin = null;

function colorOf(bx, mode) {
  if (mode === 'frag') return new THREE.Color().setHSL(0.0 + Math.min(bx[8]-1, 9) / 9 * 0.62, 0.62, 0.55);
  if (mode === 'load') { const r = bx[12] > 0 ? Math.min(bx[11] / bx[12], 1) : 0; return new THREE.Color().setHSL(0.33 * (1 - r), 0.7, 0.5); }
  let h = 0; for (const c of String(bx[6])) h = (h * 31 + c.charCodeAt(0)) >>> 0;
  return new THREE.Color().setHSL((h % 360) / 360, 0.55, 0.58);
}

function build() {
  if (group) scene.remove(group);
  bin = D.bins[+$('bin').value]; if (!bin) return;
  group = new THREE.Group();
  const mode = $('color').value;
  const n = bin.boxes.length;
  // Engine X,Y,Z (width, depth, height) -> three x, z, y.
  const geo = new THREE.BoxGeometry(1,1,1);
  mesh = new THREE.InstancedMesh(geo, new THREE.MeshStandardMaterial({ roughness:0.8 }), n);
  const m = new THREE.Matrix4(), edges = [];
  const shrink = 0.6;
  bin.boxes.forEach((b,i) => {
    const [x,y,z,w,d,h] = b;
    m.makeScale(w - shrink, h - shrink, d - shrink).setPosition(x + w/2, z + h/2, y + d/2);
    mesh.setMatrixAt(i, m); mesh.setColorAt(i, colorOf(b, mode));
    const X=[x,x+w], Y=[z,z+h], Z=[y,y+d];
    for (const a of [0,1]) for (const c of [0,1]) {
      edges.push(X[0],Y[a],Z[c], X[1],Y[a],Z[c]);
      edges.push(X[a],Y[0],Z[c], X[a],Y[1],Z[c]);
      edges.push(X[a],Y[c],Z[0], X[a],Y[c],Z[1]);
    }
  });
  group.add(mesh);
  const eg = new THREE.BufferGeometry(); eg.setAttribute('position', new THREE.Float32BufferAttribute(edges, 3));
  const lines = new THREE.LineSegments(eg, new THREE.LineBasicMaterial({ color: dark ? 0x111111 : 0x333333, transparent:true, opacity:0.35 }));
  group.add(lines); mesh.userData.lines = lines;
  if (bin.base > 0) {
    const pal = new THREE.Mesh(new THREE.BoxGeometry(bin.w, bin.base, bin.d), new THREE.MeshStandardMaterial({ color:0xb08a5a, roughness:1 }));
    pal.position.set(bin.w/2, bin.base/2, bin.d/2); group.add(pal);
  }
  const cage = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(bin.w, bin.h, bin.d)), new THREE.LineBasicMaterial({ color: 0x2f6fde }));
  cage.position.set(bin.w/2, bin.h/2, bin.d/2); group.add(cage);
  const floor = new THREE.GridHelper(Math.max(bin.w, bin.d) * 1.6, 16, 0x999999, dark ? 0x2a2a28 : 0xdddddd);
  floor.position.set(bin.w/2, 0, bin.d/2); group.add(floor);
  scene.add(group);
  $('step').max = n; $('step').value = n; showStep();
  const r = Math.max(bin.w, bin.d, bin.h);
  controls.target.set(bin.w/2, bin.h/3, bin.d/2);
  camera.position.set(bin.w/2 + r*1.3, r*1.1, bin.d/2 + r*1.5);
  controls.update();
}
function showStep() {
  const k = +$('step').value; mesh.count = k;
  mesh.userData.lines.geometry.setDrawRange(0, k * 24);
  $('stepv').textContent = `${k} из ${bin.boxes.length}`;
}
function resize() { const w = main.clientWidth, h = main.clientHeight; renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix(); }
const ray = new THREE.Raycaster(), mouse = new THREE.Vector2(), tip = $('tip');
renderer.domElement.addEventListener('pointermove', e => {
  const r = renderer.domElement.getBoundingClientRect();
  mouse.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
  ray.setFromCamera(mouse, camera);
  const hit = mesh && ray.intersectObject(mesh)[0];
  if (!hit) { tip.style.display = 'none'; return; }
  const b = bin.boxes[hit.instanceId];
  tip.innerHTML = `<b>${b[6]}</b> ${b[7]}<br>${b[3]}×${b[4]}×${b[5]} мм, ${b[9]} кг, хрупкость ${b[8]}<br>опора ${b[10]}%, нагрузка сверху ${b[11]} / ${b[12]} кг<br>порядок №${hit.instanceId + 1}`;
  tip.style.display = 'block'; tip.style.left = (e.clientX - r.left + 14) + 'px'; tip.style.top = (e.clientY - r.top + 14) + 'px';
});
$('bin').onchange = build; $('color').onchange = build; $('step').oninput = showStep;
addEventListener('resize', resize);
resize(); build();
renderer.setAnimationLoop(() => renderer.render(scene, camera));
</script>
</body>
</html>
"##;
