import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const state = {
  project: { name: 'Factory Cable Routing', units: 'mm', schema_version: 1 },
  objects: [], selected: null, tool: 'select', drawing: null, modelRoots: new Map(), measureStart: null
};
const $ = id => document.getElementById(id);

// Register model loading before WebGL initialization so the native Windows
// picker remains usable even if the graphics context cannot initialize.
window.CableTrayAcceptModel = async function(selected) {
  if (!selected || !selected.url) throw new Error('The model picker did not return a loadable model.');
  const url = new URL(selected.url, window.location.href).href;
  const nativeFormat = selected.native_format || selected.format || 'STL';
  window.CableTrayDebugLog && window.CableTrayDebugLog('INFO', 'MODEL_ENGINE_START', {
    name: selected.name,
    format: selected.format,
    nativeFormat: nativeFormat,
    url: url
  });
  status('Loading model...');
  await importModelFile(url, selected.name, selected.format || 'STL', nativeFormat);
};

const viewport = $('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x080d12);
const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 1000000);
camera.position.set(12000, 9000, 12000);
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.07;
controls.target.set(0, 1500, 0);
scene.add(new THREE.HemisphereLight(0xbfd8ef, 0x1a2632, 2.2));
const sun = new THREE.DirectionalLight(0xffffff, 1.8);
sun.position.set(5000, 10000, 5000);
scene.add(sun);
const grid = new THREE.GridHelper(30000, 60, 0x324252, 0x1a252f);
scene.add(grid);
const axes = new THREE.AxesHelper(1500);
scene.add(axes);
const ground = new THREE.Mesh(new THREE.PlaneGeometry(30000, 30000), new THREE.MeshBasicMaterial({ transparent: true, opacity: 0 }));
ground.rotation.x = -Math.PI / 2;
scene.add(ground);
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

function resize() {
  const r = viewport.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = Math.max(0.1, r.width / Math.max(1, r.height));
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

function id(prefix) { return prefix + '_' + Date.now() + '_' + Math.floor(Math.random() * 100000); }
function mmToScene(v) { return Number(v) / 10; }
function sceneToM(v) { return Number(v) * 0.01; }
function lengthOf(points) { let n = 0; for (let i = 1; i < points.length; i++) n += points[i - 1].distanceTo(points[i]); return sceneToM(n); }
function elbows(points) { let n = 0; for (let i = 1; i < points.length - 1; i++) { const a = points[i].clone().sub(points[i - 1]).normalize(); const b = points[i + 1].clone().sub(points[i]).normalize(); if (a.dot(b) < 0.999) n++; } return n; }
function esc(v) { return String(v == null ? '' : v).replace(/[&<>"']/g, function(c){ return ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]); }); }
function toast(message) { const e = $('toast'); e.textContent = message; e.classList.add('show'); clearTimeout(toast.timer); toast.timer = setTimeout(function(){ e.classList.remove('show'); }, 1800); }
function status(message) { $('statusText').textContent = message; }

function setTool(tool) {
  state.tool = tool;
  state.drawing = (tool === 'cable' || tool === 'tray') ? { type: tool, points: [] } : null;
  state.measureStart = null;
  document.querySelectorAll('.tool').forEach(function(b){ b.classList.toggle('active', b.dataset.tool === tool); });
  const hint = {
    select: 'Select an object in the 3D scene or Scene panel.',
    cable: 'Click route points. Press Enter to finish.',
    tray: 'Click route points. Press Enter to finish.',
    model: 'Use Load Model for 3D, SolidWorks, AutoCAD DWG or DXF files.',
    measure: 'Click two points on the ground plane to measure.'
  };
  $('toolHint').textContent = hint[tool] || '';
  $('routeOverlay').classList.toggle('hidden', tool !== 'cable' && tool !== 'tray');
  status(tool === 'cable' || tool === 'tray' ? 'Drawing ' + tool + ' route' : 'Ready');
}
document.querySelectorAll('.tool').forEach(function(b){ b.addEventListener('click', function(){ setTool(b.dataset.tool); }); });
$('projectName').addEventListener('input', function(e){ state.project.name = e.target.value; });
$('unitSystem').addEventListener('change', function(e){ state.project.units = e.target.value; });

function pointerRay(event) {
  const r = renderer.domElement.getBoundingClientRect();
  mouse.x = ((event.clientX - r.left) / r.width) * 2 - 1;
  mouse.y = -((event.clientY - r.top) / r.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);
}

function groundPoint(event) {
  pointerRay(event);
  const hit = raycaster.intersectObject(ground, false)[0];
  return hit ? hit.point.clone() : null;
}

function routePoint(event) {
  pointerRay(event);
  const modelRoots = Array.from(state.modelRoots.values());
  if (modelRoots.length) {
    const modelHit = raycaster.intersectObjects(modelRoots, true)[0];
    if (modelHit) return modelHit.point.clone();
  }
  const groundHit = raycaster.intersectObject(ground, false)[0];
  return groundHit ? groundHit.point.clone() : null;
}

function createRoute(type, points) {
  const cable = type === 'cable';
  const dia = Number($('defaultCableDiameter').value) || 24;
  const width = Number($('defaultTrayWidth').value) || 300;
  const height = Number($('defaultTrayHeight').value) || 100;
  const elevation = Number($('defaultElevation').value) || 3000;
  const p = points.map(function(v){ return new THREE.Vector3(v.x, mmToScene(elevation), v.z); });
  const obj = {
    id: id(type), kind: type,
    name: (cable ? 'Cable-' : 'Tray-') + (state.objects.filter(function(o){ return o.kind === type; }).length + 1),
    points: p,
    diameter_mm: dia, width_mm: width, height_mm: height,
    specification: cable ? 'POWER-CABLE' : width + 'x' + height + ' TRAY',
    material: cable ? 'Copper/PVC' : 'Galvanized Steel'
  };
  state.objects.push(obj);
  return obj;
}
function routeVisual(obj) {
  const g = new THREE.Group();
  g.userData.objectId = obj.id;
  const color = obj.kind === 'cable' ? 0xffb347 : 0x55b6ff;
  const mat = new THREE.MeshStandardMaterial({ color: color, roughness: 0.72, metalness: 0.25 });
  if (obj.kind === 'cable') {
    const curve = new THREE.CatmullRomCurve3(obj.points, false, 'centripetal', 0.1);
    curve.arcLengthDivisions = Math.max(32, obj.points.length * 16);
    const tube = new THREE.Mesh(new THREE.TubeGeometry(curve, Math.max(8, obj.points.length * 16), Math.max(0.2, mmToScene(obj.diameter_mm) / 2), 10, false), mat);
    tube.userData.objectId = obj.id;
    g.add(tube);
  } else {
    for (let i = 1; i < obj.points.length; i++) {
      const a = obj.points[i - 1], b = obj.points[i];
      const len = a.distanceTo(b);
      const box = new THREE.Mesh(new THREE.BoxGeometry(mmToScene(obj.width_mm), mmToScene(obj.height_mm), len), mat);
      box.position.copy(a.clone().add(b).multiplyScalar(0.5));
      box.lookAt(b);
      box.userData.objectId = obj.id;
      g.add(box);
    }
  }
  const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(obj.points), new THREE.LineBasicMaterial({ color: 0xdceeff, transparent: true, opacity: 0.35 }));
  line.userData.objectId = obj.id;
  g.add(line);
  return g;
}
function rebuildRoutes() {
  const remove = [];
  scene.traverse(function(o){ if (o.userData && o.userData.routeVisual) remove.push(o); });
  remove.forEach(function(o){ if (o.parent) o.parent.remove(o); });
  state.objects.filter(function(o){ return o.kind === 'cable' || o.kind === 'tray'; }).forEach(function(o){ const g = routeVisual(o); g.userData.routeVisual = true; scene.add(g); });
}
function finishRoute() {
  if (!state.drawing) return;
  const d = state.drawing;
  state.drawing = null;
  if (d.points.length < 2) { setTool('select'); return; }
  const obj = createRoute(d.type, d.points);
  state.selected = obj.id;
  rebuildRoutes();
  render();
  setTool('select');
  toast(obj.name + ' created');
}
renderer.domElement.addEventListener('click', function(e){
  if (state.tool === 'cable' || state.tool === 'tray') {
    const p = routePoint(e); if (!p) return;
    p.y = mmToScene(Number($('defaultElevation').value) || 3000);
    state.drawing.points.push(p);
    status(state.drawing.type + ' point ' + state.drawing.points.length);
    return;
  }
  if (state.tool === 'measure') {
    const p = groundPoint(e); if (!p) return;
    if (!state.measureStart) { state.measureStart = p; toast('Measurement start set'); }
    else { toast('Distance: ' + (sceneToM(state.measureStart.distanceTo(p))).toFixed(3) + ' m'); state.measureStart = null; }
    return;
  }
  if (state.tool !== 'select') return;
  const r = renderer.domElement.getBoundingClientRect();
  mouse.x = ((e.clientX - r.left) / r.width) * 2 - 1;
  mouse.y = -((e.clientY - r.top) / r.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);
  const hits = raycaster.intersectObjects(scene.children, true);
  const hit = hits.find(function(x){ return x.object.userData && x.object.userData.objectId; });
  state.selected = hit ? hit.object.userData.objectId : null;
  render();
});
document.addEventListener('keydown', function(e){
  if (e.key === 'Enter' && state.drawing) { e.preventDefault(); finishRoute(); }
  if (e.key === 'Escape' && state.drawing) setTool('select');
  if ((e.key === 'Delete' || e.key === 'Backspace') && state.selected) deleteSelected();
});

function deleteSelected() {
  const idx = state.objects.findIndex(function(o){ return o.id === state.selected; });
  if (idx < 0) return;
  const root = state.modelRoots.get(state.selected);
  if (root) { scene.remove(root); state.modelRoots.delete(state.selected); }
  state.objects.splice(idx, 1);
  state.selected = null;
  rebuildRoutes(); render(); toast('Object deleted');
}
function renderScene() {
  const box = $('sceneList');
  if (!state.objects.length) { box.innerHTML = '<div class="hint" style="padding:10px">No objects yet.</div>'; return; }
  box.innerHTML = state.objects.map(function(o){
    const qty = (o.kind === 'cable' || o.kind === 'tray') ? lengthOf(o.points).toFixed(2) + ' m' : o.format;
    return '<div class="scene-item ' + (o.id === state.selected ? 'active' : '') + '" data-id="' + esc(o.id) + '"><div><div class="scene-name">' + esc(o.name) + '</div><div class="scene-type">' + esc(String(o.kind).toUpperCase()) + '</div></div><div class="scene-type">' + esc(qty) + '</div></div>';
  }).join('');
  box.querySelectorAll('.scene-item').forEach(function(n){ n.addEventListener('click', function(){ state.selected = n.dataset.id; render(); }); });
}
function renderProperties() {
  const el = $('properties');
  const o = state.objects.find(function(x){ return x.id === state.selected; });
  if (!o) { el.className = 'properties empty'; el.textContent = 'Select a route or imported model.'; return; }
  el.className = 'properties';
  if (o.kind === 'model') {
    el.innerHTML = '<div class="prop-row"><div class="prop-label">Name</div><input class="prop-value" id="p_name" value="' + esc(o.name) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Format</div><div class="prop-value">' + esc(o.format) + '</div></div>' +
      '<div class="prop-row"><div class="prop-label">Source</div><div class="prop-value">' + esc(o.source) + '</div></div>' +
      '<button class="small danger" id="deleteObjectBtn">Delete</button>';
    $('p_name').addEventListener('input', function(e){ o.name = e.target.value; renderScene(); });
  } else {
    const cable = o.kind === 'cable';
    el.innerHTML = '<div class="prop-row"><div class="prop-label">Name</div><input class="prop-value" id="p_name" value="' + esc(o.name) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Specification</div><input class="prop-value" id="p_spec" value="' + esc(o.specification) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">' + (cable ? 'Diameter' : 'Width') + ' (mm)</div><input class="prop-value" id="p_a" type="number" value="' + (cable ? o.diameter_mm : o.width_mm) + '"></div>' +
      (cable ? '' : '<div class="prop-row"><div class="prop-label">Height (mm)</div><input class="prop-value" id="p_h" type="number" value="' + o.height_mm + '"></div>') +
      '<div class="prop-row"><div class="prop-label">Material</div><input class="prop-value" id="p_material" value="' + esc(o.material) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Length</div><div class="prop-value">' + lengthOf(o.points).toFixed(2) + ' m</div></div>' +
      '<div class="prop-row"><div class="prop-label">Elbows</div><div class="prop-value">' + elbows(o.points) + '</div></div>' +
      '<button class="small danger" id="deleteObjectBtn">Delete</button>';
    $('p_name').addEventListener('input', function(e){ o.name = e.target.value; renderScene(); });
    $('p_spec').addEventListener('input', function(e){ o.specification = e.target.value; renderBoq(); });
    $('p_material').addEventListener('input', function(e){ o.material = e.target.value; renderBoq(); });
    $('p_a').addEventListener('input', function(e){ if (cable) o.diameter_mm = Number(e.target.value) || 1; else o.width_mm = Number(e.target.value) || 1; rebuildRoutes(); renderBoq(); });
    if (!cable) $('p_h').addEventListener('input', function(e){ o.height_mm = Number(e.target.value) || 1; rebuildRoutes(); renderBoq(); });
  }
  $('deleteObjectBtn').addEventListener('click', deleteSelected);
}
function renderBoq() {
  const routes = state.objects.filter(function(o){ return o.kind === 'cable' || o.kind === 'tray'; });
  const cableM = routes.filter(function(o){ return o.kind === 'cable'; }).reduce(function(s,o){ return s + lengthOf(o.points); }, 0);
  const trayM = routes.filter(function(o){ return o.kind === 'tray'; }).reduce(function(s,o){ return s + lengthOf(o.points); }, 0);
  const elbowN = routes.reduce(function(s,o){ return s + elbows(o.points); }, 0);
  $('totalCable').textContent = cableM.toFixed(2) + ' m';
  $('totalTray').textContent = trayM.toFixed(2) + ' m';
  $('totalElbows').textContent = String(elbowN);
  $('boqTableWrap').innerHTML = routes.length ? '<table><thead><tr><th>Item</th><th>Specification</th><th>Qty</th><th>Unit</th><th>Elbow</th></tr></thead><tbody>' + routes.map(function(o){ return '<tr><td>' + esc(o.kind === 'cable' ? 'Cable' : 'Cable Tray') + '</td><td>' + esc(o.specification) + '</td><td>' + lengthOf(o.points).toFixed(2) + '</td><td>m</td><td>' + elbows(o.points) + '</td></tr>'; }).join('') + '</tbody></table>' : '<div class="hint" style="padding:10px">No routing quantities yet.</div>';
}
function render() { renderScene(); renderProperties(); renderBoq(); }

async function importModelFile(fileUrl, displayName, format, nativeFormat) {
  const url = fileUrl;
  try {
    let root;
    if (format === 'GLB' || format === 'GLTF') {
      root = (await new GLTFLoader().loadAsync(url)).scene;
    } else if (format === 'OBJ') {
      const text = await fetch(url).then(function(r){ if (!r.ok) throw new Error('Model could not be read'); return r.text(); });
      root = new OBJLoader().parse(text);
    } else if (format === 'STL') {
      const data = await fetch(url).then(function(r){ if (!r.ok) throw new Error('Model could not be read'); return r.arrayBuffer(); });
      const geometry = new STLLoader().parse(data);
      if (!geometry.attributes.position || geometry.attributes.position.count === 0) {
        throw new Error('STL contains no geometry.');
      }
      geometry.computeVertexNormals();
      root = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: 0x9ca8b2, metalness: 0.45, roughness: 0.65 }));
    } else {
      throw new Error('Unsupported model format: ' + format);
    }

    // Routing coordinates use a scene scale of 10 mm per scene unit.
    // Native CAD bridges export STL geometry in millimetre coordinates, so
    // scale those results to the same scene coordinate system.
    const nativeCadFormats = ['SLDASM', 'SLDPRT', 'DWG', 'DXF'];
    if (format === 'STL' && nativeCadFormats.indexOf(String(nativeFormat || '').toUpperCase()) >= 0) {
      root.scale.setScalar(0.1);
      root.updateMatrixWorld(true);
      window.CableTrayDebugLog && window.CableTrayDebugLog('INFO', 'CAD_MODEL_SCALE_APPLIED', {
        nativeFormat: nativeFormat,
        scale: 0.1,
        sceneMillimetersPerUnit: 10
      });
    }

    const objectId = id('model');
    root.userData.objectId = objectId;
    root.traverse(function(n){ if (n.isMesh) n.userData.objectId = objectId; });
    scene.add(root);
    state.modelRoots.set(objectId, root);
    state.objects.push({ id: objectId, kind: 'model', name: displayName, source: displayName, format: format });
    state.selected = objectId;
    fitAllScene();
    render();
    status('Model loaded');
    toast(displayName + ' loaded');
  } catch (err) {
    console.error(err);
    status('Ready');
    toast('Import failed: ' + err.message);
    throw err;
  }
}

function projectData() {
  return { schema: 'cable-tray-project', schema_version: 1, project: state.project, objects: state.objects.filter(function(o){ return o.kind !== 'model'; }).map(function(o){ return { id:o.id, kind:o.kind, name:o.name, points:o.points.map(function(p){ return {x:p.x*10,y:p.y*10,z:p.z*10}; }), diameter_mm:o.diameter_mm, width_mm:o.width_mm, height_mm:o.height_mm, specification:o.specification, material:o.material }; }) };
}
$('saveProjectBtn').addEventListener('click', function(){
  const blob = new Blob([JSON.stringify(projectData(), null, 2)], { type: 'application/json' });
  const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = slug(state.project.name) + '.json'; a.click(); URL.revokeObjectURL(a.href);
});
function loadProject(data) {
  if (!data || data.schema !== 'cable-tray-project') throw new Error('Not a Cable_tray project');
  state.modelRoots.forEach(function(root){ scene.remove(root); }); state.modelRoots.clear();
  state.objects = (data.objects || []).map(function(o){ return { ...o, points:(o.points || []).map(function(p){ return new THREE.Vector3(mmToScene(p.x), mmToScene(p.y), mmToScene(p.z)); }) }; });
  state.project = { ...state.project, ...(data.project || {}) };
  $('projectName').value = state.project.name || 'Factory Cable Routing';
  $('unitSystem').value = state.project.units || 'mm';
  state.selected = null; rebuildRoutes(); render();
}
$('newProjectBtn').addEventListener('click', function(){
  if (!confirm('Clear the current design?')) return;
  state.modelRoots.forEach(function(root){ scene.remove(root); }); state.modelRoots.clear(); state.objects = []; state.selected = null; render(); toast('New project created');
});
$('exportBoqBtn').addEventListener('click', function(){
  const rows = [['Item','Specification','Name','Quantity','Unit','Elbows']];
  state.objects.filter(function(o){ return o.kind === 'cable' || o.kind === 'tray'; }).forEach(function(o){ rows.push([o.kind === 'cable' ? 'Cable' : 'Cable Tray', o.specification, o.name, lengthOf(o.points).toFixed(3), 'm', String(elbows(o.points))]); });
  const csv = rows.map(function(r){ return r.map(function(v){ return '"' + String(v).replace(/"/g,'""') + '"'; }).join(','); }).join('\\n');
  const blob = new Blob([csv], { type:'text/csv;charset=utf-8' }); const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = slug(state.project.name) + '_BOQ.csv'; a.click(); URL.revokeObjectURL(a.href);
});
function slug(v){ return String(v || 'cable-tray-project').toLowerCase().replace(/[^a-z0-9]+/g,'-').replace(/(^-|-$)/g,'') || 'cable-tray-project'; }
function fitObject(root) {
  const box = new THREE.Box3().setFromObject(root); if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3()); const size = box.getSize(new THREE.Vector3()); const d = Math.max(size.x,size.y,size.z,1000) * 1.6;
  camera.position.copy(center).add(new THREE.Vector3(d*0.8,d*0.6,d*0.8)); controls.target.copy(center); controls.update();
}
function fitAllScene() {
  const box = new THREE.Box3();
  scene.traverse(function(o){ if ((o.isMesh || o.isLine) && o !== ground && o !== grid && o !== axes) box.expandByObject(o); });
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3()); const size = box.getSize(new THREE.Vector3()); const d = Math.max(size.x,size.y,size.z,1000) * 1.7;
  camera.position.copy(center).add(new THREE.Vector3(d*0.7,d*0.55,d*0.7)); controls.target.copy(center); controls.update();
}
$('fitBtn').addEventListener('click', fitAllScene);
$('topBtn').addEventListener('click', function(){ camera.position.set(0,18000,0.01); controls.target.set(0,0,0); controls.update(); });
$('frontBtn').addEventListener('click', function(){ camera.position.set(0,5000,18000); controls.target.set(0,0,0); controls.update(); });
$('isoBtn').addEventListener('click', function(){ camera.position.set(12000,9500,12000); controls.target.set(0,1500,0); controls.update(); });
setTool('select'); render(); animate();

function animate(){ requestAnimationFrame(animate); controls.update(); renderer.render(scene,camera); }
