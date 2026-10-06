import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { OBJExporter } from 'three/addons/exporters/OBJExporter.js';
import { STLExporter } from 'three/addons/exporters/STLExporter.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';

const state = {
  project: { name: 'Factory Cable Routing', units: 'mm', schema_version: 1 },
  objects: [], selected: null, tool: 'select', drawing: null,
  modelRoots: new Map(), routeRoots: new Map(), measureStart: null,
  sourceModels: [], dragging: null, skipClick: false,
  surfacePickMode: false, surfacePick: null,
  measureStart: null, autoRouteStart: null, autoRoutePendingEnd: null, autoRoutePreviewRoot: null, measurements: [], measurementsVisible: true,
  selectedMeasurementId: null,
  surfaceAlignStart: null,
  clipboard: null,
  undoStack: [], redoStack: [],
  restoringHistory: false
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
  await importModelFile(url, selected.name, selected.format || 'STL', nativeFormat, selected.path || '');
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
const measurementRoot = new THREE.Group();
measurementRoot.name = 'MeasurementAnnotations';
scene.add(measurementRoot);
const surfaceSelectionRoot = new THREE.Group();
surfaceSelectionRoot.name = 'SurfaceSelection';
scene.add(surfaceSelectionRoot);
const autoRoutePreviewRoot = new THREE.Group();
autoRoutePreviewRoot.name = 'AutoRoutePreview';
scene.add(autoRoutePreviewRoot);
state.autoRoutePreviewRoot = autoRoutePreviewRoot;
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

function cloneRouteData(o) {
  return {
    ...o,
    points: (o.points || []).map(function(p){ return p.clone(); }),
    rotation_deg: {
      x: Number(o.rotation_deg && o.rotation_deg.x) || 0,
      y: Number(o.rotation_deg && o.rotation_deg.y) || 0,
      z: Number(o.rotation_deg && o.rotation_deg.z) || 0
    },
    surface_alignment: o.surface_alignment ? {
      mode: o.surface_alignment.mode,
      angle_deg: Number(o.surface_alignment.angle_deg) || 0,
      normal: o.surface_alignment.normal ? {
        x: Number(o.surface_alignment.normal.x) || 0,
        y: Number(o.surface_alignment.normal.y) || 0,
        z: Number(o.surface_alignment.normal.z) || 0
      } : null
    } : null
  };
}

function captureDesignState() {
  return {
    project: { ...state.project },
    routes: state.objects
      .filter(function(o){ return o.kind === 'cable' || o.kind === 'tray'; })
      .map(cloneRouteData),
    models: state.objects
      .filter(function(o){ return o.kind === 'model'; })
      .map(function(o){
        const root = state.modelRoots.get(o.id);
        if (!root) return { id:o.id, transform:null };
        return {
          id:o.id,
          transform:{
            position:{x:root.position.x,y:root.position.y,z:root.position.z},
            quaternion:{x:root.quaternion.x,y:root.quaternion.y,z:root.quaternion.z,w:root.quaternion.w},
            scale:{x:root.scale.x,y:root.scale.y,z:root.scale.z}
          }
        };
      }),
    selected: state.selected
  };
}

function recordHistory(before) {
  if (!before || state.restoringHistory) return;
  const after = captureDesignState();
  if (JSON.stringify(before) === JSON.stringify(after)) return;
  state.undoStack.push(before);
  if (state.undoStack.length > 100) state.undoStack.shift();
  state.redoStack.length = 0;
}

function resetHistory() {
  state.undoStack.length = 0;
  state.redoStack.length = 0;
}

function restoreDesignState(snapshot) {
  if (!snapshot) return;
  state.restoringHistory = true;
  try {
    const modelObjects = state.objects.filter(function(o){ return o.kind === 'model'; });
    const restoredRoutes = (snapshot.routes || []).map(cloneRouteData);
    state.objects = modelObjects.concat(restoredRoutes);
    state.project = { ...state.project, ...(snapshot.project || {}) };
    const selectedExists = state.objects.some(function(o){ return o.id === snapshot.selected; });
    state.selected = selectedExists ? snapshot.selected : null;
    rebuildRoutes();
    (snapshot.models || []).forEach(function(saved){
      const root = state.modelRoots.get(saved.id);
      if (!root || !saved.transform) return;
      root.position.set(saved.transform.position.x, saved.transform.position.y, saved.transform.position.z);
      root.quaternion.set(saved.transform.quaternion.x, saved.transform.quaternion.y, saved.transform.quaternion.z, saved.transform.quaternion.w);
      root.scale.set(saved.transform.scale.x, saved.transform.scale.y, saved.transform.scale.z);
      root.updateMatrixWorld(true);
    });
    syncMeasurements();
    rebuildMeasurements();
    render();
  } finally {
    state.restoringHistory = false;
  }
}

function undo() {
  if (state.surfacePickMode) cancelSurfacePick();
  if (state.drawing) {
    if (state.drawing.points.length) {
      state.drawing.points.pop();
      status('Removed last route point');
    }
    return;
  }
  const before = state.undoStack.pop();
  if (!before) {
    toast('Nothing to undo');
    return;
  }
  state.redoStack.push(captureDesignState());
  restoreDesignState(before);
  toast('Undo');
}

function redo() {
  if (state.surfacePickMode || state.drawing) return;
  const next = state.redoStack.pop();
  if (!next) {
    toast('Nothing to redo');
    return;
  }
  state.undoStack.push(captureDesignState());
  restoreDesignState(next);
  toast('Redo');
}

function saveProjectFile() {
  const blob = new Blob([JSON.stringify(projectData(), null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = slug(state.project.name) + '.json';
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(function(){ URL.revokeObjectURL(a.href); }, 1000);
}

function setTool(tool) {
  state.surfacePickMode = false;
  state.surfacePick = null;
  state.surfaceAlignStart = null;
  state.autoRouteStart = null;
  state.autoRoutePendingEnd = null;
  autoRoutePreviewRoot.clear();
  state.tool = tool;
  state.drawing = (tool === 'cable' || tool === 'tray') ? { type: tool, points: [] } : null;
  state.measureStart = null;
  surfaceSelectionRoot.clear();
  document.querySelectorAll('.tool').forEach(function(b){ b.classList.toggle('active', b.dataset.tool === tool); });
  const autoRouteTypeRow = $('autoRouteTypeRow');
  const autoRouteActions = $('autoRouteActions');
  if (autoRouteTypeRow) autoRouteTypeRow.classList.toggle('hidden', tool !== 'auto-route');
  if (autoRouteActions) autoRouteActions.classList.add('hidden');
  const hint = {
    select: 'Click a route to select it. Drag a selected tray or cable to move it in 3D; edit Position/Slope in Properties.',
    cable: 'Click route points. Press Enter to finish.',
    tray: 'Click route points. Press Enter to finish.',
    model: 'Use Load Model for 3D, SolidWorks, AutoCAD DWG or DXF files.',
    measure: 'Click two points, or two CAD/model surfaces, to create a persistent distance dimension.',
    'auto-route': 'Select Cable or Tray, then click the start point and end point. A straight shortest route is created between them.'
  };
  $('toolHint').textContent = hint[tool] || '';
  $('routeOverlay').classList.toggle('hidden', tool !== 'cable' && tool !== 'tray');
  status(tool === 'cable' || tool === 'tray' ? 'Drawing ' + tool + ' route' : tool === 'auto-route' ? 'Auto Route: select start point' : 'Ready');
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
    if (modelHit) {
      const point = modelHit.point.clone();
      point.__routeSnapToModel = true;
      return point;
    }
  }
  const groundHit = raycaster.intersectObject(ground, false)[0];
  return groundHit ? groundHit.point.clone() : null;
}

function autoRoutePoint(event) {
  pointerRay(event);
  const roots = Array.from(state.modelRoots.values()).concat(Array.from(state.routeRoots.values()));
  if (roots.length) {
    const hits = [];
    roots.forEach(function(root){
      raycaster.intersectObject(root, true).forEach(function(hit){ hits.push(hit); });
    });
    hits.sort(function(a,b){ return a.distance - b.distance; });
    const hit = hits[0];
    if (hit && hit.point) {
      const point = hit.point.clone();
      point.__routeSnapToModel = true;
      return point;
    }
  }

  const groundHit = raycaster.intersectObject(ground, false)[0];
  if (!groundHit) return null;

  const point = groundHit.point.clone();
  point.y = mmToScene(Number($('defaultElevation').value) || 3000);
  point.__routeSnapToModel = true;
  return point;
}

function clearAutoRoutePreview() {
  autoRoutePreviewRoot.clear();
}

function showAutoRoutePreview(start, end) {
  clearAutoRoutePreview();

  const markerRadius = Math.max(2, mmToScene(40));
  const startMarker = new THREE.Mesh(
    new THREE.SphereGeometry(markerRadius, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0xffd45a, depthTest: false })
  );
  startMarker.position.copy(start);
  startMarker.renderOrder = 40;

  const endMarker = new THREE.Mesh(
    new THREE.SphereGeometry(markerRadius, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x66d9ff, depthTest: false })
  );
  endMarker.position.copy(end);
  endMarker.renderOrder = 40;

  const lineGeometry = new THREE.BufferGeometry().setFromPoints([start.clone(), end.clone()]);
  const lineMaterial = new THREE.LineDashedMaterial({
    color: 0x66d9ff,
    dashSize: Math.max(1.5, mmToScene(120)),
    gapSize: Math.max(0.8, mmToScene(70)),
    transparent: true,
    opacity: 0.95,
    depthTest: false
  });
  const line = new THREE.Line(lineGeometry, lineMaterial);
  line.computeLineDistances();
  line.renderOrder = 39;

  autoRoutePreviewRoot.add(line);
  autoRoutePreviewRoot.add(startMarker);
  autoRoutePreviewRoot.add(endMarker);

  const distance = sceneToM(start.distanceTo(end));
  const lengthElement = $('autoRoutePreviewLength');
  if (lengthElement) lengthElement.textContent = distance.toFixed(3) + ' m';
  const actions = $('autoRouteActions');
  if (actions) actions.classList.remove('hidden');
}

function createAutoRouteFromPreview() {
  const start = state.autoRouteStart;
  const end = state.autoRoutePendingEnd;
  if (!start || !end) return;

  const distance = start.distanceTo(end);
  if (distance < 1e-8) {
    toast('Start and end points must be different');
    return;
  }

  const type = $('autoRouteType').value === 'tray' ? 'tray' : 'cable';
  const beforeHistory = captureDesignState();
  const obj = createRoute(type, [start.clone(), end.clone()]);

  recordHistory(beforeHistory);
  state.selected = obj.id;
  setTool('select');
  render();
  toast(obj.name + ' created — shortest straight route: ' + sceneToM(distance).toFixed(3) + ' m');
}

function resetAutoRoutePoints() {
  state.autoRouteStart = null;
  state.autoRoutePendingEnd = null;
  clearAutoRoutePreview();
  const actions = $('autoRouteActions');
  if (actions) actions.classList.add('hidden');
  status('Auto Route: select start point');
  toast('Auto Route points cleared');
}

function createRoute(type, points) {
  const cable = type === 'cable';
  const dia = Number($('defaultCableDiameter').value) || 24;
  const width = Number($('defaultTrayWidth').value) || 300;
  const height = Number($('defaultTrayHeight').value) || 100;
  const elevation = Number($('defaultElevation').value) || 3000;
  const p = points.map(function(v){
    // A point picked on imported CAD geometry already has exact world X/Y/Z.
    // Points falling back to the ground plane use the configured elevation.
    return new THREE.Vector3(v.x, v.__routeSnapToModel ? v.y : mmToScene(elevation), v.z);
  });
  const obj = {
    id: id(type), kind: type,
    name: (cable ? 'Cable-' : 'Tray-') + (state.objects.filter(function(o){ return o.kind === type; }).length + 1),
    points: p,
    diameter_mm: dia, width_mm: width, height_mm: height,
    specification: cable ? 'POWER-CABLE' : width + 'x' + height + ' TRAY',
    material: cable ? 'Copper/PVC' : 'Galvanized Steel',
    rotation_deg: { x: 0, y: 0, z: 0 },
    surface_alignment: null
  };
  state.objects.push(obj);
  return obj;
}

function routeCenter(points) {
  const center = new THREE.Vector3();
  if (!points.length) return center;
  points.forEach(function(p){ center.add(p); });
  return center.multiplyScalar(1 / points.length);
}

function roundedRouteCurve(points, radius) {
  const path = new THREE.CurvePath();
  if (points.length < 2) return path;
  if (points.length === 2) {
    path.add(new THREE.LineCurve3(points[0], points[1]));
    return path;
  }

  const entries = new Array(points.length);
  const exits = new Array(points.length);
  entries[0] = points[0].clone();
  exits[0] = points[0].clone();
  entries[points.length - 1] = points[points.length - 1].clone();
  exits[points.length - 1] = points[points.length - 1].clone();

  for (let i = 1; i < points.length - 1; i++) {
    const prev = points[i - 1];
    const cur = points[i];
    const next = points[i + 1];
    const inLength = cur.distanceTo(prev);
    const outLength = next.distanceTo(cur);
    const localRadius = Math.max(0, Math.min(radius, inLength * 0.35, outLength * 0.35));
    const inDir = cur.clone().sub(prev).normalize();
    const outDir = next.clone().sub(cur).normalize();
    entries[i] = cur.clone().sub(inDir.multiplyScalar(localRadius));
    exits[i] = cur.clone().add(outDir.multiplyScalar(localRadius));
  }

  let cursor = points[0].clone();
  for (let i = 1; i < points.length - 1; i++) {
    path.add(new THREE.LineCurve3(cursor, entries[i]));
    path.add(new THREE.QuadraticBezierCurve3(entries[i], points[i], exits[i]));
    cursor = exits[i].clone();
  }
  path.add(new THREE.LineCurve3(cursor, points[points.length - 1]));
  return path;
}

function routeVisual(obj) {
  const g = new THREE.Group();
  g.userData.objectId = obj.id;
  g.userData.routeVisual = true;

  const center = routeCenter(obj.points);
  const localPoints = obj.points.map(function(p){ return p.clone().sub(center); });
  const rot = obj.rotation_deg || { x: 0, y: 0, z: 0 };
  g.position.copy(center);
  g.rotation.order = 'XYZ';
  g.rotation.set(
    THREE.MathUtils.degToRad(Number(rot.x) || 0),
    THREE.MathUtils.degToRad(Number(rot.y) || 0),
    THREE.MathUtils.degToRad(Number(rot.z) || 0)
  );

  const isCable = obj.kind === 'cable';
  const baseColor = isCable ? 0xffb347 : 0xb6bec7;
  const mat = new THREE.MeshStandardMaterial({
    color: baseColor,
    roughness: isCable ? 0.72 : 0.28,
    metalness: isCable ? 0.25 : 0.78
  });
  g.userData.baseColor = baseColor;
  g.userData.meshMaterial = mat;

  const curveRadius = obj.kind === 'cable'
    ? Math.max(0.8, mmToScene(obj.diameter_mm * 3))
    : Math.max(1.0, mmToScene(Math.min(obj.width_mm, obj.height_mm) * 0.8));
  const roundedCurve = roundedRouteCurve(localPoints, curveRadius);

  if (obj.kind === 'cable') {
    const tube = new THREE.Mesh(
      new THREE.TubeGeometry(
        roundedCurve,
        Math.max(24, localPoints.length * 20),
        Math.max(0.2, mmToScene(obj.diameter_mm) / 2),
        12,
        false
      ),
      mat
    );
    tube.userData.objectId = obj.id;
    g.add(tube);
  } else {
    // Solid-bottom industrial tray: continuous floor and continuous sidewalls.
    // No perforations, slots, ladder rungs, or mesh openings anywhere.
    const width = mmToScene(obj.width_mm);
    const height = mmToScene(obj.height_mm);
    const sheet = Math.max(0.18, mmToScene(Math.min(obj.width_mm, obj.height_mm) * 0.035));
    const rail = Math.max(0.22, sheet * 1.4);
    const lipWidth = Math.max(rail * 1.2, mmToScene(Math.min(obj.width_mm, 80) * 0.045));

    function addOrientedPart(geometry, position, direction, upDirection) {
      const mesh = new THREE.Mesh(geometry, mat);
      mesh.position.copy(position);

      const zAxis = direction.clone().normalize();
      let yAxis = (upDirection || new THREE.Vector3(0, 1, 0)).clone();
      yAxis.sub(zAxis.clone().multiplyScalar(yAxis.dot(zAxis)));

      if (yAxis.lengthSq() < 1e-8) {
        yAxis = Math.abs(zAxis.y) < 0.999
          ? new THREE.Vector3(0, 1, 0)
          : new THREE.Vector3(0, 0, 1);
        yAxis.sub(zAxis.clone().multiplyScalar(yAxis.dot(zAxis)));
      }

      yAxis.normalize();
      const xAxis = zAxis.clone().cross(yAxis).normalize();
      const basis = new THREE.Matrix4().makeBasis(xAxis, yAxis, zAxis);
      mesh.quaternion.setFromRotationMatrix(basis);

      mesh.userData.objectId = obj.id;
      g.add(mesh);
      return mesh;
    }

    for (let i = 1; i < localPoints.length; i++) {
      const a = localPoints[i - 1], b = localPoints[i];
      const len = a.distanceTo(b);
      if (len < 0.001) continue;

      const dir = b.clone().sub(a).normalize();
      const centerSeg = a.clone().add(b).multiplyScalar(0.5);

      // World Y is the normal/up direction for horizontal tray runs.
      // For a vertical run, use world Z as the tray's local up direction
      // so the cross-section remains well-defined instead of collapsing.
      let trayUp = new THREE.Vector3(0, 1, 0);
      if (Math.abs(trayUp.dot(dir)) > 0.999) trayUp.set(0, 0, 1);
      trayUp.sub(dir.clone().multiplyScalar(trayUp.dot(dir))).normalize();

      const sideDir = dir.clone().cross(trayUp).normalize();

      const depth = Math.max(len, sheet * 2);
      const edgeRadius = Math.min(sheet * 0.5, Math.max(0.05, depth * 0.03));

      // Continuous solid floor. All offsets are relative to the segment's
      // own cross-section so vertical segments are positioned correctly too.
      const floorPosition = centerSeg.clone()
        .add(trayUp.clone().multiplyScalar(-height * 0.5 + sheet * 0.5));

      addOrientedPart(
        new RoundedBoxGeometry(
          Math.max(width, 0.2),
          sheet,
          depth,
          5,
          edgeRadius
        ),
        floorPosition,
        dir,
        trayUp
      );

      // Continuous solid sidewalls.
      for (const side of [-1, 1]) {
        const wallOffset = sideDir.clone().multiplyScalar(side * (width * 0.5 - sheet * 0.5));

        addOrientedPart(
          new RoundedBoxGeometry(
            sheet,
            Math.max(height, sheet),
            depth,
            5,
            edgeRadius
          ),
          centerSeg.clone().add(wallOffset),
          dir,
          trayUp
        );

        // Rolled top lip for a cleaner industrial profile.
        const lipOffset = sideDir.clone().multiplyScalar(side * (width * 0.5 + lipWidth * 0.5));
        const lipPosition = centerSeg.clone()
          .add(lipOffset)
          .add(trayUp.clone().multiplyScalar(height * 0.5 - lipWidth * 0.35));

        addOrientedPart(
          new RoundedBoxGeometry(
            lipWidth,
            Math.max(sheet, lipWidth * 0.65),
            depth,
            5,
            Math.min(lipWidth * 0.3, depth * 0.035)
          ),
          lipPosition,
          dir,
          trayUp
        );

        // Subtle lower return flange: solid, continuous, and hole-free.
        const lowerFlangeOffset = sideDir.clone().multiplyScalar(side * (width * 0.5 - sheet * 0.5));
        const lowerFlangePosition = centerSeg.clone()
          .add(lowerFlangeOffset)
          .add(trayUp.clone().multiplyScalar(-height * 0.5 + sheet * 1.45));

        addOrientedPart(
          new RoundedBoxGeometry(
            Math.max(sheet * 1.8, lipWidth * 0.9),
            sheet,
            depth,
            5,
            Math.min(sheet * 0.5, depth * 0.03)
          ),
          lowerFlangePosition,
          dir,
          trayUp
        );
      }
    }

    // Smooth solid transition blocks at bends; no perforation or open grid.
    for (let i = 1; i < localPoints.length - 1; i++) {
      const corner = localPoints[i];
      const inDir = corner.clone().sub(localPoints[i - 1]).normalize();
      const outDir = localPoints[i + 1].clone().sub(corner).normalize();
      let bisector = inDir.clone().add(outDir);
      if (bisector.lengthSq() < 1e-6) bisector = outDir.clone();
      bisector.normalize();

      const elbowDepth = Math.max(width * 0.55, height);
      const elbow = new THREE.Mesh(
        new RoundedBoxGeometry(
          Math.max(width, 0.2),
          Math.max(height, sheet),
          elbowDepth,
          7,
          Math.min(Math.max(width, height) * 0.16, elbowDepth * 0.16)
        ),
        mat
      );
      elbow.position.copy(corner);
      elbow.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), bisector);
      elbow.userData.objectId = obj.id;
      g.add(elbow);
    }
  }

  const linePoints = roundedCurve.getPoints(Math.max(16, localPoints.length * 16));
  const lineMaterial = new THREE.LineBasicMaterial({
    color: 0xdceeff,
    transparent: true,
    opacity: 0.34
  });
  const line = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(linePoints),
    lineMaterial
  );
  line.userData.objectId = obj.id;
  g.userData.lineMaterial = lineMaterial;
  g.add(line);

  applyRouteSelectionVisual(g, obj.id === state.selected);
  return g;
}

function applyRouteSelectionVisual(root, selected) {
  if (!root) return;
  const baseColor = root.userData.baseColor || 0x55b6ff;
  root.traverse(function(node){
    if (node.isMesh && node.material && node.material.color) {
      node.material.color.setHex(selected ? 0xffdf59 : baseColor);
      if (node.material.emissive) {
        node.material.emissive.setHex(selected ? 0x6b4e00 : 0x000000);
        node.material.emissiveIntensity = selected ? 0.55 : 0;
      }
    }
  });
  if (root.userData.lineMaterial) {
    root.userData.lineMaterial.color.setHex(selected ? 0xfff3a6 : 0xdceeff);
    root.userData.lineMaterial.opacity = selected ? 0.9 : 0.34;
  }
}

function updateRouteSelectionVisuals() {
  state.routeRoots.forEach(function(root, objectId){
    applyRouteSelectionVisual(root, objectId === state.selected);
  });
}

function rebuildRoutes() {
  state.routeRoots.forEach(function(root){
    if (root.parent) root.parent.remove(root);
  });
  state.routeRoots.clear();

  state.objects
    .filter(function(o){ return o.kind === 'cable' || o.kind === 'tray'; })
    .forEach(function(o){
      const g = routeVisual(o);
      scene.add(g);
      state.routeRoots.set(o.id, g);
    });
}
function finishRoute() {
  if (!state.drawing) return;
  const d = state.drawing;
  state.drawing = null;
  if (d.points.length < 2) { setTool('select'); return; }
  const beforeHistory = captureDesignState();
  const obj = createRoute(d.type, d.points);
  recordHistory(beforeHistory);
  state.selected = obj.id;
  rebuildRoutes();
  render();
  setTool('select');
  toast(obj.name + ' created');
}
function routeHitAtEvent(event) {
  pointerRay(event);
  const roots = Array.from(state.routeRoots.values());
  if (!roots.length) return null;
  const hits = raycaster.intersectObjects(roots, true);
  const hit = hits[0];
  if (!hit || !hit.object || !hit.object.userData) return null;
  const objectId = hit.object.userData.objectId;
  const root = objectId ? state.routeRoots.get(objectId) : null;
  if (!root) return null;
  return { hit: hit, objectId: objectId, root: root };
}

function finishRouteDrag() {
  if (!state.dragging) return;
  const drag = state.dragging;
  state.dragging = null;
  controls.enabled = true;

  if (drag.moved) {
    const delta = drag.root.position.clone().sub(drag.startPosition);
    if (delta.lengthSq() > 1e-10) {
      const obj = state.objects.find(function(o){ return o.id === drag.objectId; });
      if (obj) {
        const beforeHistory = drag.historyBefore;
        obj.points.forEach(function(p){ p.add(delta); });
        recordHistory(beforeHistory);
      }
    }
    rebuildRoutes();
    render();
    state.skipClick = true;
    status('Route moved');
    return;
  }

  state.selected = drag.objectId;
  render();
}

renderer.domElement.addEventListener('pointerdown', function(e){
  if (state.surfacePickMode) return;
  if (state.tool !== 'select' || e.button !== 0) return;
  const picked = routeHitAtEvent(e);
  if (!picked) return;

  state.selected = picked.objectId;
  render();

  const normal = camera.getWorldDirection(new THREE.Vector3()).normalize();
  const dragPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, picked.hit.point);
  state.dragging = {
    objectId: picked.objectId,
    root: picked.root,
    startPosition: picked.root.position.clone(),
    historyBefore: captureDesignState(),
    pointerOffset: picked.hit.point.clone().sub(picked.root.position),
    plane: dragPlane,
    moved: false,
    startPointer: { x: e.clientX, y: e.clientY }
  };
  controls.enabled = false;
  try { renderer.domElement.setPointerCapture(e.pointerId); } catch (_) {}
});

renderer.domElement.addEventListener('pointermove', function(e){
  if (!state.dragging) return;
  const drag = state.dragging;
  const r = renderer.domElement.getBoundingClientRect();
  mouse.x = ((e.clientX - r.left) / r.width) * 2 - 1;
  mouse.y = -((e.clientY - r.top) / r.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);

  const point = new THREE.Vector3();
  if (!raycaster.ray.intersectPlane(drag.plane, point)) return;

  const nextPosition = point.sub(drag.pointerOffset);
  if (!drag.moved) {
    const px = e.clientX - drag.startPointer.x;
    const py = e.clientY - drag.startPointer.y;
    drag.moved = (px * px + py * py) >= 16;
  }
  if (!drag.moved) return;

  drag.root.position.copy(nextPosition);
  status('Moving ' + (state.objects.find(function(o){ return o.id === drag.objectId; }) || {}).name);
});

renderer.domElement.addEventListener('pointerup', function(){
  finishRouteDrag();
});

renderer.domElement.addEventListener('pointercancel', function(){
  if (state.dragging) {
    state.dragging = null;
    controls.enabled = true;
  }
});

renderer.domElement.addEventListener('click', function(e){
  if (state.skipClick) {
    state.skipClick = false;
    return;
  }

  if (state.surfacePickMode) {
    pickSurfaceFromEvent(e);
    return;
  }

  if (state.tool === 'cable' || state.tool === 'tray') {
    const p = routePoint(e); if (!p) return;
    state.drawing.points.push(p);
    status(state.drawing.type + ' point ' + state.drawing.points.length);
    return;
  }

  if (state.tool === 'auto-route') {
    const point = autoRoutePoint(e);
    if (!point) {
      toast('Click a valid start or end point in the 3D view');
      return;
    }

    if (!state.autoRouteStart) {
      state.autoRouteStart = point;
      state.autoRoutePendingEnd = null;
      clearAutoRoutePreview();
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(Math.max(2, mmToScene(40)), 16, 16),
        new THREE.MeshBasicMaterial({ color: 0xffd45a, depthTest: false })
      );
      marker.position.copy(point);
      marker.renderOrder = 40;
      autoRoutePreviewRoot.add(marker);
      status('Auto Route: select end point');
      toast('Start point selected — click the end point');
    } else {
      state.autoRoutePendingEnd = point;
      showAutoRoutePreview(state.autoRouteStart, point);
      status('Shortest route preview shown');
      toast('Shortest route preview shown — review it before creating');
    }
    return;
  }

  if (state.tool === 'align') {
    const target = measurementTargetFromEvent(e);
    if (!target) return;
    if (!state.surfaceAlignStart) {
      state.surfaceAlignStart = target;
      clearSurfaceSelectionVisuals();
      addSurfaceSelectionVisual(target, 0xffd45a);
      toast('Reference surface selected — click the second surface');
    } else {
      addSurfaceSelectionVisual(target, 0x7ae6ff);
      const reference = state.surfaceAlignStart;
      state.surfaceAlignStart = null;
      alignObjectToSurface(reference, target);
    }
    return;
  }

  if (state.tool === 'measure') {
    const target = measurementTargetFromEvent(e);
    if (!target) return;
    if (!state.measureStart) {
      state.measureStart = target;
      if (target.kind === 'surface') {
        clearSurfaceSelectionVisuals();
        addSurfaceSelectionVisual(target, 0xffd45a);
        toast('First surface selected — click the second surface');
      } else {
        toast('First measurement point set');
      }
    } else {
      if (target.kind === 'surface') addSurfaceSelectionVisual(target, 0x7ae6ff);
      addMeasurement(state.measureStart, target);
      state.measureStart = null;
      clearSurfaceSelectionVisuals();
    }
    return;
  }
  if (state.tool !== 'select') return;

  pointerRay(e);
  const hits = raycaster.intersectObjects(scene.children, true);
  const hit = hits.find(function(x){ return x.object.userData && x.object.userData.objectId; });
  state.selectedMeasurementId = null;
  state.selected = hit ? hit.object.userData.objectId : null;
  render();
});
function handleKeyboardShortcut(e) {
  if (e.repeat) return;
  const key = String(e.key || '').toLowerCase();
  const modifier = e.ctrlKey || e.metaKey;
  const tag = e.target && e.target.tagName ? e.target.tagName.toUpperCase() : '';
  const isTextEditing = tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || !!(e.target && e.target.isContentEditable);

  if (modifier && key === 'r') {
    e.preventDefault(); e.stopPropagation();
    if (e.stopImmediatePropagation) e.stopImmediatePropagation();
    rebuildRoutes(); render(); status('3D view refreshed'); toast('3D view refreshed'); return;
  }

  if (modifier && key === 's') {
    e.preventDefault(); e.stopPropagation();
    if (e.stopImmediatePropagation) e.stopImmediatePropagation();
    if (e.shiftKey) {
      try { exportModel($('exportModelFormat').value); }
      catch (err) { console.error(err); status('Ready'); toast('Export failed: ' + err.message); }
    } else {
      saveProjectFile(); toast('Project saved');
    }
    return;
  }

  if (key === 'escape') {
    if (state.surfacePickMode) { cancelSurfacePick(); e.preventDefault(); return; }
    if (state.surfaceAlignStart) {
      state.surfaceAlignStart = null;
      clearSurfaceSelectionVisuals();
      e.preventDefault();
      return;
    }
    if (state.autoRouteStart) {
      resetAutoRoutePoints();
      e.preventDefault();
      return;
    }
    if (state.drawing) { setTool('select'); e.preventDefault(); }
    return;
  }

  if (modifier && !isTextEditing) {
    if (key === 'z' && !e.shiftKey) { e.preventDefault(); e.stopPropagation(); if(e.stopImmediatePropagation)e.stopImmediatePropagation(); undo(); return; }
    if ((key === 'z' && e.shiftKey) || key === 'y') { e.preventDefault(); e.stopPropagation(); if(e.stopImmediatePropagation)e.stopImmediatePropagation(); redo(); return; }
    if (key === 'c') { e.preventDefault(); e.stopPropagation(); if(e.stopImmediatePropagation)e.stopImmediatePropagation(); copySelected(); return; }
    if (key === 'v') { e.preventDefault(); e.stopPropagation(); if(e.stopImmediatePropagation)e.stopImmediatePropagation(); pasteClipboard(); return; }
    if (key === 'x') { e.preventDefault(); e.stopPropagation(); if(e.stopImmediatePropagation)e.stopImmediatePropagation(); cutSelected(); return; }
    if (key === 'd') { e.preventDefault(); e.stopPropagation(); if(e.stopImmediatePropagation)e.stopImmediatePropagation(); copySelected(); pasteClipboard(); return; }
  }

  if (key === 'enter' && state.drawing) { e.preventDefault(); finishRoute(); return; }
  if ((key === 'delete' || key === 'backspace') && state.selectedMeasurementId && !isTextEditing) {
    e.preventDefault();
    deleteMeasurement(state.selectedMeasurementId);
    return;
  }
  if (key === 'f' && !modifier && !isTextEditing && !state.drawing) { e.preventDefault(); fitAllScene(); return; }
  if (key === '1' && !modifier && !isTextEditing && !state.drawing) { e.preventDefault(); $('topBtn').click(); return; }
  if (key === '2' && !modifier && !isTextEditing && !state.drawing) { e.preventDefault(); $('frontBtn').click(); return; }
  if (key === '3' && !modifier && !isTextEditing && !state.drawing) { e.preventDefault(); $('isoBtn').click(); return; }
  if ((key === 'delete' || key === 'backspace') && state.selectedMeasurementId && !isTextEditing) {
    e.preventDefault();
    deleteMeasurement(state.selectedMeasurementId);
    return;
  }
  if ((key === 'delete' || key === 'backspace') && state.selected && !isTextEditing) {
    e.preventDefault();
    deleteSelected();
  }
}
window.addEventListener('keydown', handleKeyboardShortcut, true);

function deleteSelected() {
  const idx = state.objects.findIndex(function(o){ return o.id === state.selected; });
  if (idx < 0) return;
  const object = state.objects[idx];
  const beforeHistory = object.kind === 'cable' || object.kind === 'tray' ? captureDesignState() : null;
  const root = state.modelRoots.get(state.selected);
  if (root) { scene.remove(root); state.modelRoots.delete(state.selected); }
  state.objects.splice(idx, 1);
  state.selected = null;
  if (beforeHistory) recordHistory(beforeHistory);
  rebuildRoutes(); render(); toast('Object deleted');
}

function bindPropertyHistoryInput(id, onInput) {
  const field = $(id);
  if (!field) return;
  let before = null;
  field.addEventListener('focus', function(){ before = captureDesignState(); });
  field.addEventListener('input', onInput);
  field.addEventListener('change', function(){
    if (before) recordHistory(before);
    before = null;
  });
}

function bindPropertyHistoryChange(id, onChange) {
  const field = $(id);
  if (!field) return;
  field.addEventListener('focus', function(){ field.__historyBefore = captureDesignState(); });
  field.addEventListener('change', function(e){
    const before = field.__historyBefore || captureDesignState();
    onChange(e);
    recordHistory(before);
    field.__historyBefore = null;
  });
}

function copySelected() {
  const o = state.objects.find(function(x){ return x.id === state.selected; });
  if (!o || (o.kind !== 'cable' && o.kind !== 'tray')) {
    toast('Select a cable or tray to copy');
    return;
  }
  state.clipboard = cloneRouteData(o);
  try {
    if (navigator.clipboard) {
      navigator.clipboard.writeText(JSON.stringify({
        application: 'CableTrayDesigner',
        kind: 'route',
        object: state.clipboard
      }));
    }
  } catch (_) {}
  toast(o.name + ' copied');
}

function pasteClipboard() {
  if (!state.clipboard) {
    toast('Nothing to paste');
    return;
  }
  const beforeHistory = captureDesignState();
  const o = cloneRouteData(state.clipboard);
  o.id = id(o.kind);
  o.name = (o.name || (o.kind === 'tray' ? 'Tray' : 'Cable')) + ' Copy';
  const offset = new THREE.Vector3(mmToScene(500), 0, mmToScene(500));
  o.points = o.points.map(function(p){ return p.clone().add(offset); });
  o.surface_alignment = null;
  state.objects.push(o);
  state.selected = o.id;
  recordHistory(beforeHistory);
  rebuildRoutes();
  render();
  toast(o.name + ' pasted');
}

function cutSelected() {
  const o = state.objects.find(function(x){ return x.id === state.selected; });
  if (!o || (o.kind !== 'cable' && o.kind !== 'tray')) {
    toast('Select a cable or tray to cut');
    return;
  }
  copySelected();
  deleteSelected();
}

function formatDistance(meters) {
  const mm = Math.abs(Number(meters) || 0) * 1000;
  return mm >= 1000 ? (mm / 1000).toFixed(3) + ' m' : mm.toFixed(1) + ' mm';
}

function getProjectRoot(objectId) {
  return state.modelRoots.get(objectId) || state.routeRoots.get(objectId) || null;
}

function faceGeometryData(hit, root) {
  if (!hit || !hit.face || !hit.object || !root) return null;
  const geometry = hit.object.geometry;
  const position = geometry && geometry.attributes ? geometry.attributes.position : null;
  if (!position) return null;

  const indices = geometry.index;
  const vertexIndices = indices
    ? [
        indices.getX(hit.faceIndex * 3),
        indices.getX(hit.faceIndex * 3 + 1),
        indices.getX(hit.faceIndex * 3 + 2)
      ]
    : [hit.faceIndex * 3, hit.faceIndex * 3 + 1, hit.faceIndex * 3 + 2];

  if (vertexIndices.some(function(i){ return !Number.isInteger(i) || i < 0 || i >= position.count; })) return null;

  const worldVertices = vertexIndices.map(function(i){
    return new THREE.Vector3()
      .fromBufferAttribute(position, i)
      .applyMatrix4(hit.object.matrixWorld);
  });

  const geometricNormal = worldVertices[1].clone().sub(worldVertices[0])
    .cross(worldVertices[2].clone().sub(worldVertices[0]));
  if (geometricNormal.lengthSq() < 1e-12) return null;
  geometricNormal.normalize();

  const inverseRoot = root.matrixWorld.clone().invert();
  const localNormal = geometricNormal.clone().transformDirection(inverseRoot).normalize();
  const localPoint = root.worldToLocal(hit.point.clone());
  const localTriangle = worldVertices.map(function(p){
    const v = root.worldToLocal(p.clone());
    return {x:v.x,y:v.y,z:v.z};
  });

  return {
    point: hit.point.clone(),
    normal: geometricNormal,
    localPoint: {x:localPoint.x,y:localPoint.y,z:localPoint.z},
    localNormal: {x:localNormal.x,y:localNormal.y,z:localNormal.z},
    localTriangle: localTriangle,
    object: hit.object,
    faceIndex: hit.faceIndex
  };
}

function measurementTargetFromEvent(event) {
  pointerRay(event);

  const roots = Array.from(state.modelRoots.values()).concat(Array.from(state.routeRoots.values()));
  if (!roots.length) {
    toast('No project surfaces are available for measurement');
    return null;
  }

  const hits = [];
  roots.forEach(function(root){
    raycaster.intersectObject(root, true).forEach(function(hit){
      if (hit && hit.face) hits.push(hit);
    });
  });
  hits.sort(function(a,b){ return a.distance - b.distance; });

  const hit = hits[0];
  if (!hit || !hit.face) {
    toast('Click a surface that belongs to the project');
    return null;
  }

  const objectId = hit.object.userData && hit.object.userData.objectId ? hit.object.userData.objectId : null;
  const modelObject = objectId ? state.objects.find(function(o){ return o.id === objectId; }) : null;
  const root = objectId ? getProjectRoot(objectId) : null;
  if (!root) {
    toast('The selected surface is not a project object');
    return null;
  }

  const face = faceGeometryData(hit, root);
  if (!face) {
    toast('The selected face has no usable surface geometry');
    return null;
  }

  return {
    kind:'surface',
    point:face.point,
    normal:face.normal,
    modelId:modelObject && modelObject.kind === 'model' ? objectId : null,
    objectId:objectId,
    root:root,
    localPoint:face.localPoint,
    localNormal:face.localNormal,
    localTriangle:face.localTriangle,
    object:face.object,
    faceIndex:face.faceIndex
  };
}

function makeMeasurementAnchor(target) {
  if (!target || !target.objectId || !target.localPoint) return null;
  return {
    objectId:target.objectId,
    localPoint:{x:target.localPoint.x,y:target.localPoint.y,z:target.localPoint.z},
    localNormal:target.localNormal ? {
      x:target.localNormal.x,y:target.localNormal.y,z:target.localNormal.z
    } : null,
    localTriangle:Array.isArray(target.localTriangle)
      ? target.localTriangle.map(function(v){ return {x:v.x,y:v.y,z:v.z}; })
      : null
  };
}

function resolveMeasurementAnchor(anchor, fallback) {
  if (!anchor || !anchor.objectId || !anchor.localPoint) return fallback ? fallback.clone() : new THREE.Vector3();
  const root = getProjectRoot(anchor.objectId);
  if (!root) return fallback ? fallback.clone() : new THREE.Vector3();
  return root.localToWorld(new THREE.Vector3(anchor.localPoint.x,anchor.localPoint.y,anchor.localPoint.z));
}

function resolveMeasurementNormal(anchor, fallback) {
  if (!anchor || !anchor.objectId || !anchor.localNormal) return fallback ? fallback.clone().normalize() : null;
  const root = getProjectRoot(anchor.objectId);
  if (!root) return fallback ? fallback.clone().normalize() : null;
  return new THREE.Vector3(anchor.localNormal.x,anchor.localNormal.y,anchor.localNormal.z)
    .transformDirection(root.matrixWorld).normalize();
}

function resolveMeasurementTriangle(anchor) {
  if (!anchor || !anchor.objectId || !Array.isArray(anchor.localTriangle) || anchor.localTriangle.length !== 3) return null;
  const root = getProjectRoot(anchor.objectId);
  if (!root) return null;
  return anchor.localTriangle.map(function(v){
    return root.localToWorld(new THREE.Vector3(v.x,v.y,v.z));
  });
}

function closestPointsOnSegments(p1,q1,p2,q2) {
  const d1=q1.clone().sub(p1);
  const d2=q2.clone().sub(p2);
  const r=p1.clone().sub(p2);
  const a=d1.dot(d1);
  const e=d2.dot(d2);
  const f=d2.dot(r);
  let s=0;
  let t=0;

  if(a<=1e-12 && e<=1e-12) {
    return {a:p1.clone(),b:p2.clone(),distance:p1.distanceTo(p2)};
  }

  if(a<=1e-12) {
    s=0;
    t=Math.max(0,Math.min(1,f/e));
  } else {
    const c=d1.dot(r);
    if(e<=1e-12) {
      t=0;
      s=Math.max(0,Math.min(1,-c/a));
    } else {
      const b=d1.dot(d2);
      const denom=a*e-b*b;
      if(Math.abs(denom)>1e-12) s=Math.max(0,Math.min(1,(b*f-c*e)/denom));
      const tNom=b*s+f;
      if(tNom<=0) {
        t=0;
        s=Math.max(0,Math.min(1,-c/a));
      } else if(tNom>=e) {
        t=1;
        s=Math.max(0,Math.min(1,(b-c)/a));
      } else {
        t=tNom/e;
      }
    }
  }

  const aPoint=p1.clone().add(d1.multiplyScalar(s));
  const bPoint=p2.clone().add(d2.multiplyScalar(t));
  return {a:aPoint,b:bPoint,distance:aPoint.distanceTo(bPoint)};
}

function closestPointToTriangle(point,a,b,c) {
  return new THREE.Triangle(a,b,c).closestPointToPoint(point.clone(),new THREE.Vector3());
}

function closestPointsSegmentTriangle(p0,p1,a,b,c) {
  let best=null;
  const direction=p1.clone().sub(p0);
  const length=direction.length();

  if(length>1e-12) {
    direction.normalize();
    const hit=new THREE.Vector3();
    const ray=new THREE.Ray(p0.clone(),direction);
    if(ray.intersectTriangle(a,b,c,false,hit)) {
      const travelled=p0.distanceTo(hit);
      if(travelled<=length+1e-8) return {a:hit.clone(),b:hit.clone(),distance:0};
    }
  }

  [p0,p1].forEach(function(endpoint){
    const closest=closestPointToTriangle(endpoint,a,b,c);
    const candidate={a:endpoint.clone(),b:closest,distance:endpoint.distanceTo(closest)};
    if(!best || candidate.distance<best.distance) best=candidate;
  });

  [[a,b],[b,c],[c,a]].forEach(function(edge){
    const candidate=closestPointsOnSegments(p0,p1,edge[0],edge[1]);
    if(!best || candidate.distance<best.distance) {
      best={a:candidate.a,b:candidate.b,distance:candidate.distance};
    }
  });

  return best;
}

function closestPointsBetweenTriangles(a0,a1,a2,b0,b1,b2) {
  const edgesA=[[a0,a1],[a1,a2],[a2,a0]];
  const edgesB=[[b0,b1],[b1,b2],[b2,b0]];
  let best=null;

  edgesA.forEach(function(edge){
    const candidate=closestPointsSegmentTriangle(edge[0],edge[1],b0,b1,b2);
    if(!best || candidate.distance<best.distance) best=candidate;
  });
  edgesB.forEach(function(edge){
    const candidate=closestPointsSegmentTriangle(edge[0],edge[1],a0,a1,a2);
    if(!best || candidate.distance<best.distance) {
      best={a:candidate.b,b:candidate.a,distance:candidate.distance};
    }
  });

  return best;
}

function syncMeasurements() {
  state.measurements.forEach(function(m){
    const anchoredStart=resolveMeasurementAnchor(m.start_anchor,m.start);
    const anchoredEnd=resolveMeasurementAnchor(m.end_anchor,m.end);
    const startNormal=resolveMeasurementNormal(m.start_anchor,m.start_normal);
    const endNormal=resolveMeasurementNormal(m.end_anchor,m.end_normal);

    let displayStart=anchoredStart.clone();
    let displayEnd=anchoredEnd.clone();
    let distance=anchoredStart.distanceTo(anchoredEnd);

    if(m.kind==='surface' && startNormal && endNormal) {
      const parallel=Math.abs(startNormal.dot(endNormal))>=0.9995;
      if(parallel) {
        const signed=anchoredEnd.clone().sub(anchoredStart).dot(startNormal);
        displayEnd=anchoredStart.clone().add(startNormal.clone().multiplyScalar(signed));
        distance=Math.abs(signed);
      } else {
        const triangleA=resolveMeasurementTriangle(m.start_anchor);
        const triangleB=resolveMeasurementTriangle(m.end_anchor);
        const closest=triangleA && triangleB
          ? closestPointsBetweenTriangles(
              triangleA[0],triangleA[1],triangleA[2],
              triangleB[0],triangleB[1],triangleB[2]
            )
          : null;
        if(closest) {
          displayStart=closest.a.clone();
          displayEnd=closest.b.clone();
          distance=closest.distance;
        } else {
          const signed=anchoredEnd.clone().sub(anchoredStart).dot(startNormal);
          displayEnd=anchoredStart.clone().add(startNormal.clone().multiplyScalar(signed));
          distance=Math.abs(signed);
        }
      }
    }

    m.start=anchoredStart;
    m.end=anchoredEnd;
    m.start_normal=startNormal || m.start_normal || null;
    m.end_normal=endNormal || m.end_normal || null;
    m.displayStart=displayStart;
    m.dimensionEnd=displayEnd;
    m.distance_m=sceneToM(distance);

    if(m.line && m.line.geometry && m.line.geometry.attributes && m.line.geometry.attributes.position) {
      const positions=m.line.geometry.attributes.position;
      positions.setXYZ(0,displayStart.x,displayStart.y,displayStart.z);
      positions.setXYZ(1,displayEnd.x,displayEnd.y,displayEnd.z);
      positions.needsUpdate=true;
      m.line.geometry.computeBoundingSphere();
    }
    if(m.label) {
      m.label.textContent=formatDistance(m.distance_m);
    }
    if(m.listValue) {
      m.listValue.textContent=formatDistance(m.distance_m);
    }
  });
}

function clearSurfaceSelectionVisuals() {
  surfaceSelectionRoot.clear();
}

function addSurfaceSelectionVisual(target, color) {
  if (!target || target.kind !== 'surface' || !target.object || target.faceIndex == null) return;
  const geometry = target.object.geometry;
  const position = geometry && geometry.attributes ? geometry.attributes.position : null;
  if (!position) return;

  const indices = geometry.index;
  const tri = indices
    ? [indices.getX(target.faceIndex * 3), indices.getX(target.faceIndex * 3 + 1), indices.getX(target.faceIndex * 3 + 2)]
    : [target.faceIndex * 3, target.faceIndex * 3 + 1, target.faceIndex * 3 + 2];

  if (tri.some(function(i){ return !Number.isInteger(i) || i < 0 || i >= position.count; })) return;

  const points = tri.map(function(i){
    return new THREE.Vector3()
      .fromBufferAttribute(position, i)
      .applyMatrix4(target.object.matrixWorld)
      .add(target.normal.clone().multiplyScalar(0.03));
  });

  const overlayGeometry = new THREE.BufferGeometry().setFromPoints(points);
  overlayGeometry.setIndex([0, 1, 2]);
  const overlayMaterial = new THREE.MeshBasicMaterial({
    color: color || 0xffd45a,
    transparent: true,
    opacity: 0.48,
    side: THREE.DoubleSide,
    depthTest: false
  });
  const overlay = new THREE.Mesh(overlayGeometry, overlayMaterial);
  overlay.renderOrder = 30;
  surfaceSelectionRoot.add(overlay);
}

function selectMeasurement(id) {
  state.selectedMeasurementId = id;
  rebuildMeasurements();
}

function deleteMeasurement(id) {
  const index = state.measurements.findIndex(function(m){ return m.id === id; });
  if (index < 0) return;
  const deleted = state.measurements[index];
  state.measurements.splice(index, 1);
  if (state.selectedMeasurementId === id) state.selectedMeasurementId = null;
  rebuildMeasurements();
  toast('Dimension deleted: ' + formatDistance(deleted.distance_m));
}

function moveMeasurementObject(objectId, delta) {
  if (!objectId || !delta || delta.lengthSq() < 1e-16) return false;
  const object = state.objects.find(function(o){ return o.id === objectId; });
  if (!object) return false;

  if (object.kind === 'cable' || object.kind === 'tray') {
    object.points.forEach(function(p){ p.add(delta); });
    rebuildRoutes();
    return true;
  }

  if (object.kind === 'model') {
    const root = state.modelRoots.get(objectId);
    if (!root) return false;
    root.position.add(delta);
    root.updateMatrixWorld(true);
    return true;
  }

  return false;
}

function setMeasurementTargetDistance(id, targetMm) {
  const measurement = state.measurements.find(function(m){ return m.id === id; });
  if (!measurement || measurement.kind !== 'surface') {
    toast('Select a surface-to-surface measurement first');
    return;
  }

  const value = Number(targetMm);
  if (!Number.isFinite(value) || value < 0) {
    toast('Enter a valid distance in mm');
    return;
  }

  const startAnchor = measurement.start_anchor;
  const endAnchor = measurement.end_anchor;
  if (!startAnchor || !endAnchor || !startAnchor.objectId || !endAnchor.objectId) {
    toast('This measurement cannot be adjusted from the panel');
    return;
  }

  if (startAnchor.objectId === endAnchor.objectId) {
    toast('The two selected surfaces must belong to different objects');
    return;
  }

  const before = captureDesignState();
  const targetScene = mmToScene(value);
  const sourceObjectId = endAnchor.objectId;
  let moved = false;

  // Move only the object carrying the second selected surface.
  // The existing syncMeasurements() remains the single source of truth for
  // the actual shortest surface-to-surface distance.
  for (let i = 0; i < 12; i++) {
    syncMeasurements();

    const currentScene = Math.max(0, Number(measurement.distance_m) / 0.01);
    const error = targetScene - currentScene;
    if (Math.abs(error) <= 1e-7) break;

    let direction = (measurement.dimensionEnd || measurement.end)
      .clone()
      .sub(measurement.displayStart || measurement.start);

    if (direction.lengthSq() < 1e-12) {
      const normal = measurement.start_normal && measurement.start_normal.lengthSq()
        ? measurement.start_normal.clone().normalize()
        : null;
      if (!normal) {
        toast('Could not determine the measurement direction');
        return;
      }
      direction = normal;
    } else {
      direction.normalize();
    }

    if (!moveMeasurementObject(sourceObjectId, direction.multiplyScalar(error))) {
      toast('The second surface object cannot be moved');
      return;
    }
    moved = true;
  }

  syncMeasurements();
  const actualMm = Math.abs(Number(measurement.distance_m) || 0) * 1000;
  const remainingMm = Math.abs(actualMm - value);

  if (!moved) {
    toast('Distance is already ' + actualMm.toFixed(1) + ' mm');
    return;
  }

  recordHistory(before);
  rebuildMeasurements();
  renderScene();
  renderBoq();
  renderProperties();

  if (remainingMm <= 0.1) {
    toast('Surface distance set to ' + actualMm.toFixed(1) + ' mm');
  } else {
    toast('Closest achievable distance: ' + actualMm.toFixed(1) + ' mm');
  }
}

function renderMeasurementList() {
  const box = $('measurementList');
  if (!box) return;
  if (!state.measurements.length) {
    box.innerHTML = '<div class="hint" style="padding:8px">No dimensions yet.</div>';
    return;
  }

  box.innerHTML = state.measurements.map(function(m, index){
    const selected = m.id === state.selectedMeasurementId;
    return '<div class="measurement-item ' + (selected ? 'active' : '') + '" data-id="' + esc(m.id) + '">' +
      '<button class="measurement-select" type="button"><span>' + (index + 1) + '</span><b>' + esc(formatDistance(m.distance_m)) + '</b></button>' +
      '<button class="small danger measurement-delete" type="button">Delete</button>' +
    '</div>';
  }).join('');

  box.querySelectorAll('.measurement-item').forEach(function(row){
    const id = row.dataset.id;
    const measurement = state.measurements.find(function(m){ return m.id === id; });
    if (measurement) measurement.listValue = row.querySelector('.measurement-select b');
    row.querySelector('.measurement-select').addEventListener('click', function(){ selectMeasurement(id); });
    row.querySelector('.measurement-delete').addEventListener('click', function(){ deleteMeasurement(id); });
  });

  const editor = $('measurementEditor');
  if (!editor) return;

  const selected = state.measurements.find(function(m){ return m.id === state.selectedMeasurementId; });
  if (!selected) {
    editor.innerHTML = '<div class="hint">Select a surface-to-surface measurement to set an exact distance.</div>';
    return;
  }

  if (selected.kind !== 'surface') {
    editor.innerHTML = '<div class="hint">Exact target distance is available only for surface-to-surface measurements.</div>';
    return;
  }

  editor.innerHTML =
    '<div class="measurement-editor-title">Exact Surface Distance</div>' +
    '<label class="field"><span>Target Distance (mm)</span>' +
      '<input id="measurementTargetDistance" class="measurement-target" type="number" min="0" step="0.1" value="' +
        (Math.abs(Number(selected.distance_m) || 0) * 1000).toFixed(1) +
      '">' +
    '</label>' +
    '<button id="setMeasurementTargetBtn" class="small">Set Distance</button>' +
    '<div class="hint">The first selected surface stays fixed. The object containing the second selected surface is moved until the measured surface distance reaches the target.</div>';

  const input = $('measurementTargetDistance');
  const button = $('setMeasurementTargetBtn');
  const apply = function(){
    setMeasurementTargetDistance(selected.id, input.value);
  };
  button.addEventListener('click', apply);
  input.addEventListener('keydown', function(event){
    if (event.key === 'Enter') {
      event.preventDefault();
      apply();
    }
  });
}

function rebuildMeasurements() {
  measurementRoot.clear();
  const overlay=$('measurementOverlay');
  if(overlay) overlay.innerHTML='';

  state.measurements.forEach(function(m){
    const selected=m.id===state.selectedMeasurementId;
    const geometry=new THREE.BufferGeometry().setFromPoints([
      m.displayStart || m.start,
      m.dimensionEnd || m.end
    ]);
    const material=new THREE.LineBasicMaterial({
      color:selected?0xffffff:(m.kind==='surface'?0xffc857:0x66d9ff),
      transparent:true,
      opacity:selected?1:0.95
    });
    const line=new THREE.Line(geometry,material);
    line.userData.measurementId=m.id;
    m.line=line;
    measurementRoot.add(line);

    if(overlay){
      const label=document.createElement('div');
      label.className='measurement-label'+(selected?' active':'');
      label.textContent=formatDistance(m.distance_m);
      label.title=m.kind==='surface'?'Surface distance':'Point distance';
      label.addEventListener('click',function(event){
        event.stopPropagation();
        selectMeasurement(m.id);
      });
      overlay.appendChild(label);
      m.label=label;
    }
  });

  measurementRoot.visible=state.measurementsVisible;
  updateMeasurementOverlay();
  renderMeasurementList();
}
function updateMeasurementOverlay() {
  const overlay=$('measurementOverlay');
  if(!overlay)return;
  overlay.style.display=state.measurementsVisible?'block':'none';
  if(!state.measurementsVisible)return;
  const rect=renderer.domElement.getBoundingClientRect();
  state.measurements.forEach(function(m){
    if(!m.label)return;
    m.label.textContent=formatDistance(m.distance_m);
    const start=m.displayStart || m.start;
    const end=m.dimensionEnd || m.end;
    const mid=start.clone().add(end).multiplyScalar(0.5);
    const projected=mid.project(camera);
    const visible=projected.z>=-1 && projected.z<=1 && projected.x>=-1.15 && projected.x<=1.15 && projected.y>=-1.15 && projected.y<=1.15;
    m.label.style.display=visible?'block':'none';
    if(visible){
      m.label.style.left=((projected.x*0.5+0.5)*rect.width)+'px';
      m.label.style.top=((-projected.y*0.5+0.5)*rect.height)+'px';
    }
  });
}

function addMeasurement(first,second) {
  const m={
    id:id('measure'),
    kind:first.kind==='surface' && second.kind==='surface'?'surface':'point',
    start:first.point.clone(),
    end:second.point.clone(),
    distance_m:sceneToM(first.point.distanceTo(second.point)),
    start_normal:first.normal?first.normal.clone():null,
    end_normal:second.normal?second.normal.clone():null,
    start_anchor:makeMeasurementAnchor(first),
    end_anchor:makeMeasurementAnchor(second)
  };
  state.measurements.push(m);
  state.selectedMeasurementId=m.id;
  syncMeasurements();
  clearSurfaceSelectionVisuals();
  rebuildMeasurements();
  renderMeasurementsToggle();
  toast((m.kind==='surface'?'Surface distance: ':'Distance: ')+formatDistance(m.distance_m));
}

function toggleMeasurements() {
  state.measurementsVisible=!state.measurementsVisible;
  measurementRoot.visible=state.measurementsVisible;
  updateMeasurementOverlay();
  renderMeasurementsToggle();
  renderMeasurementList();
}

function clearAllMeasurements() {
  state.measurements = [];
  state.selectedMeasurementId = null;
  state.measureStart = null;
  clearSurfaceSelectionVisuals();
  rebuildMeasurements();
  renderMeasurementsToggle();
  renderMeasurementList();
  toast('All dimensions cleared');
}

function renderMeasurementsToggle() {
  const button=$('toggleMeasurementsBtn');
  if(button){
    button.textContent=state.measurementsVisible?'Hide Dimensions':'Show Dimensions';
    button.title=state.measurementsVisible?'Hide all measurement numbers':'Show all measurement numbers';
  }
}

function cancelSurfacePick() {
  state.surfacePickMode = false;
  state.surfacePick = null;
  status('Ready');
  renderProperties();
  toast('Surface pick cancelled');
}

function pickSurfaceFromEvent(event) {
  pointerRay(event);
  const roots=Array.from(state.modelRoots.values());
  if(!roots.length){
    cancelSurfacePick();
    toast('Load a CAD/model surface first');
    return;
  }

  const hits=[];
  roots.forEach(function(root){
    raycaster.intersectObject(root,true).forEach(function(hit){ hits.push(hit); });
  });
  hits.sort(function(a,b){ return a.distance-b.distance; });
  const hit=hits[0];

  const o=state.objects.find(function(x){ return x.id===state.selected; });
  if(!o || o.kind!=='tray'){
    cancelSurfacePick();
    toast('Select a tray before picking a surface');
    return;
  }
  if(!hit || !hit.face){
    toast('No model surface selected');
    return;
  }

  const modelId=hit.object.userData && hit.object.userData.objectId ? hit.object.userData.objectId : null;
  const modelRoot=modelId ? getProjectRoot(modelId) : null;
  const face=faceGeometryData(hit,modelRoot);
  if(!face){
    toast('The selected model face has no usable surface geometry');
    return;
  }

  state.surfacePick={
    routeId:o.id,
    point:face.point,
    normal:face.normal,
    modelId:modelId
  };
  state.surfacePickMode=false;
  status('Surface selected');
  renderProperties();
  toast('Surface selected');
}

function surfaceAlignmentRotation(o, surfaceNormal, mode, angleDeg) {
  const rot = o.rotation_deg || { x: 0, y: 0, z: 0 };
  const currentEuler = new THREE.Euler(
    THREE.MathUtils.degToRad(Number(rot.x) || 0),
    THREE.MathUtils.degToRad(Number(rot.y) || 0),
    THREE.MathUtils.degToRad(Number(rot.z) || 0),
    'XYZ'
  );
  const currentQ = new THREE.Quaternion().setFromEuler(currentEuler);
  const currentUp = new THREE.Vector3(0, 1, 0).applyQuaternion(currentQ).normalize();

  const rawRoute = o.points.length > 1
    ? o.points[o.points.length - 1].clone().sub(o.points[0]).normalize()
    : new THREE.Vector3(0, 0, 1);
  const currentTangent = rawRoute.applyQuaternion(currentQ).normalize();

  const normal = surfaceNormal.clone().normalize();
  let referenceTangent = currentTangent.clone().projectOnPlane(normal);
  if (referenceTangent.lengthSq() < 1e-8) {
    referenceTangent = new THREE.Vector3(0, 1, 0).cross(normal);
    if (referenceTangent.lengthSq() < 1e-8) referenceTangent = new THREE.Vector3(1, 0, 0).cross(normal);
  }
  referenceTangent.normalize();

  let degrees = Number(angleDeg);
  if (!Number.isFinite(degrees)) degrees = 0;
  if (mode === 'parallel') degrees = 0;
  if (mode === 'perpendicular') degrees = 90;
  degrees = Math.max(-90, Math.min(90, degrees));

  const desiredNormal = normal.clone().applyAxisAngle(referenceTangent, THREE.MathUtils.degToRad(degrees)).normalize();
  let targetTangent = currentTangent.clone().projectOnPlane(desiredNormal);
  if (targetTangent.lengthSq() < 1e-8) targetTangent = referenceTangent.clone().projectOnPlane(desiredNormal);
  if (targetTangent.lengthSq() < 1e-8) {
    targetTangent = new THREE.Vector3(0, 1, 0).projectOnPlane(desiredNormal);
    if (targetTangent.lengthSq() < 1e-8) targetTangent = new THREE.Vector3(1, 0, 0).projectOnPlane(desiredNormal);
  }
  targetTangent.normalize();

  const qAlignUp = new THREE.Quaternion().setFromUnitVectors(currentUp, desiredNormal);
  let alignedTangent = currentTangent.clone().applyQuaternion(qAlignUp).projectOnPlane(desiredNormal);
  if (alignedTangent.lengthSq() < 1e-8) alignedTangent = referenceTangent.clone().projectOnPlane(desiredNormal);
  alignedTangent.normalize();

  const cross = alignedTangent.clone().cross(targetTangent);
  const signedAngle = Math.atan2(desiredNormal.dot(cross), alignedTangent.dot(targetTangent));
  const qRoll = new THREE.Quaternion().setFromAxisAngle(desiredNormal, signedAngle);
  const finalQ = qRoll.multiply(qAlignUp).multiply(currentQ);

  const finalEuler = new THREE.Euler().setFromQuaternion(finalQ, 'XYZ');
  return {
    x: THREE.MathUtils.radToDeg(finalEuler.x),
    y: THREE.MathUtils.radToDeg(finalEuler.y),
    z: THREE.MathUtils.radToDeg(finalEuler.z)
  };
}

function applySurfaceAlignment(o, mode, angleDeg) {
  if (!o || o.kind !== 'tray' || !state.surfacePick || state.surfacePick.routeId !== o.id) {
    toast('Pick a surface for this tray first');
    return;
  }
  const beforeHistory = captureDesignState();
  const degrees = mode === 'parallel' ? 0 : mode === 'perpendicular' ? 90 : Number(angleDeg);
  if (!Number.isFinite(degrees)) {
    toast('Enter a valid surface angle');
    return;
  }
  o.rotation_deg = surfaceAlignmentRotation(o, state.surfacePick.normal, mode, degrees);
  o.surface_alignment = {
    mode: mode,
    angle_deg: Math.max(-90, Math.min(90, degrees)),
    normal: {
      x: state.surfacePick.normal.x,
      y: state.surfacePick.normal.y,
      z: state.surfacePick.normal.z
    }
  };
  recordHistory(beforeHistory);
  rebuildRoutes();
  render();
  toast('Tray aligned to selected surface');
}
function quaternionAngle(q) {
  return 2*Math.acos(Math.min(1,Math.abs(q.w)));
}

function alignObjectToSurface(reference,source) {
  if(!reference || !source || !reference.objectId || !source.objectId){
    toast('Select two project surfaces');
    return;
  }
  if(reference.objectId===source.objectId){
    toast('The two surfaces must belong to different objects');
    return;
  }

  const referenceRoot=getProjectRoot(reference.objectId);
  const sourceRoot=getProjectRoot(source.objectId);
  const sourceObject=state.objects.find(function(o){ return o.id===source.objectId; });
  if(!referenceRoot || !sourceRoot || !sourceObject){
    toast('Surface object is not available');
    return;
  }

  const beforeHistory=captureDesignState();
  referenceRoot.updateMatrixWorld(true);
  sourceRoot.updateMatrixWorld(true);

  const currentQuaternion=sourceRoot.getWorldQuaternion(new THREE.Quaternion());
  const sourceNormal=source.normal.clone().normalize();
  const referenceNormal=reference.normal.clone().normalize();

  // Align the two planes while using the smaller of the two valid normal
  // orientations. This avoids an unnecessary 180° flip when the surfaces
  // already face opposite directions.
  const alignSame=new THREE.Quaternion().setFromUnitVectors(sourceNormal,referenceNormal);
  const alignOpposite=new THREE.Quaternion().setFromUnitVectors(
    sourceNormal,referenceNormal.clone().negate()
  );
  const rotationDelta=quaternionAngle(alignOpposite)<quaternionAngle(alignSame)
    ? alignOpposite
    : alignSame;

  const finalQuaternion=rotationDelta.clone().multiply(currentQuaternion);
  sourceRoot.quaternion.copy(finalQuaternion);
  sourceRoot.updateMatrixWorld(true);

  const localPoint=new THREE.Vector3(source.localPoint.x,source.localPoint.y,source.localPoint.z);
  const sourceSurfacePoint=sourceRoot.localToWorld(localPoint);
  const translation=reference.point.clone().sub(sourceSurfacePoint);

  sourceRoot.position.add(translation);
  sourceRoot.updateMatrixWorld(true);

  if(sourceObject.kind==='cable' || sourceObject.kind==='tray'){
    const euler=new THREE.Euler().setFromQuaternion(sourceRoot.quaternion,'XYZ');
    sourceObject.rotation_deg={
      x:THREE.MathUtils.radToDeg(euler.x),
      y:THREE.MathUtils.radToDeg(euler.y),
      z:THREE.MathUtils.radToDeg(euler.z)
    };
    sourceObject.points.forEach(function(p){ p.add(translation); });
  }

  state.selected=source.objectId;
  state.surfaceAlignStart=null;
  clearSurfaceSelectionVisuals();
  recordHistory(beforeHistory);
  rebuildRoutes();
  syncMeasurements();
  rebuildMeasurements();
  render();
  toast('Second surface aligned to first surface');
}

function renderScene() {
  const box = $('sceneList');
  if (!state.objects.length) { box.innerHTML = '<div class="hint" style="padding:10px">No objects yet.</div>'; return; }
  box.innerHTML = state.objects.map(function(o){
    const qty = (o.kind === 'cable' || o.kind === 'tray') ? lengthOf(o.points).toFixed(2) + ' m' : o.format;
    return '<div class="scene-item ' + (o.id === state.selected ? 'active' : '') + '" data-id="' + esc(o.id) + '"><div><div class="scene-name">' + esc(o.name) + '</div><div class="scene-type">' + esc(String(o.kind).toUpperCase()) + '</div></div><div class="scene-type">' + esc(qty) + '</div></div>';
  }).join('');
  box.querySelectorAll('.scene-item').forEach(function(n){
    n.addEventListener('click', function(){
      state.selectedMeasurementId = null;
      state.selected = n.dataset.id;
      render();
    });
  });
}
function resizeRouteToLength(o, targetMeters) {
  const target = Number(targetMeters);
  if (!Number.isFinite(target) || target <= 0 || o.points.length < 2) return false;
  const current = lengthOf(o.points);
  if (!Number.isFinite(current) || current <= 0) return false;
  const scale = target / current;
  const anchor = o.points[0].clone();
  o.points.forEach(function(p){ p.copy(anchor.clone().add(p.clone().sub(anchor).multiplyScalar(scale))); });
  return true;
}

function renderProperties() {
  const el = $('properties');
  const o = state.objects.find(function(x){ return x.id === state.selected; });
  if (!o) {
    el.className = 'properties empty';
    el.textContent = 'Select a route or imported model.';
    return;
  }

  el.className = 'properties';

  if (o.kind === 'model') {
    el.innerHTML = '<div class="prop-row"><div class="prop-label">Name</div><input class="prop-value" id="p_name" value="' + esc(o.name) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Format</div><div class="prop-value">' + esc(o.format) + '</div></div>' +
      '<div class="prop-row"><div class="prop-label">Source</div><div class="prop-value">' + esc(o.source) + '</div></div>' +
      '<button class="small danger" id="deleteObjectBtn">Delete</button>';
    $('p_name').addEventListener('input', function(e){ o.name = e.target.value; renderScene(); });
  } else {
    const cable = o.kind === 'cable';
    const center = routeCenter(o.points);
    const rot = o.rotation_deg || { x: 0, y: 0, z: 0 };

    el.innerHTML =
      '<div class="property-group-title">Identity</div>' +
      '<div class="prop-row"><div class="prop-label">Name</div><input class="prop-value" id="p_name" value="' + esc(o.name) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Specification</div><input class="prop-value" id="p_spec" value="' + esc(o.specification) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Material</div><input class="prop-value" id="p_material" value="' + esc(o.material) + '"></div>' +
      '<div class="property-group-title">Dimensions</div>' +
      '<div class="prop-row"><div class="prop-label">' + (cable ? 'Diameter' : 'Width') + ' (mm)</div><input class="prop-value" id="p_a" type="number" value="' + (cable ? o.diameter_mm : o.width_mm) + '"></div>' +
      (cable ? '' : '<div class="prop-row"><div class="prop-label">Height (mm)</div><input class="prop-value" id="p_h" type="number" value="' + o.height_mm + '"></div>') +
      '<div class="property-group-title">Position (mm)</div>' +
      '<div class="prop-row"><div class="prop-label">X</div><input class="prop-value" id="p_pos_x" type="number" step="1" value="' + (center.x * 10).toFixed(1) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Y</div><input class="prop-value" id="p_pos_y" type="number" step="1" value="' + (center.y * 10).toFixed(1) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Z</div><input class="prop-value" id="p_pos_z" type="number" step="1" value="' + (center.z * 10).toFixed(1) + '"></div>' +
      '<div class="property-group-title">Local Rotation / Slope (deg)</div>' +
      '<div class="prop-row"><div class="prop-label">X</div><input class="prop-value" id="p_rot_x" type="number" step="0.1" value="' + Number(rot.x || 0).toFixed(1) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Y</div><input class="prop-value" id="p_rot_y" type="number" step="0.1" value="' + Number(rot.y || 0).toFixed(1) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Z</div><input class="prop-value" id="p_rot_z" type="number" step="0.1" value="' + Number(rot.z || 0).toFixed(1) + '"></div>' +
      (cable ? '' : '<div class="property-group-title">Surface Alignment</div>' +
        '<button class="small secondary" id="pickSurfaceBtn">Pick Surface</button>' +
        '<div class="property-hint">' + (state.surfacePick && state.surfacePick.routeId === o.id ? 'Surface selected. ' : 'Select a face on an imported CAD/model surface. ') + '0° = parallel, 90° = perpendicular.</div>' +
        '<div class="prop-row"><div class="prop-label">Relation</div><select class="prop-value" id="p_surface_mode">' +
          '<option value="parallel">Parallel to surface</option>' +
          '<option value="perpendicular">Perpendicular to surface</option>' +
          '<option value="angle">Custom angle</option>' +
        '</select></div>' +
        '<div class="prop-row"><div class="prop-label">Angle (deg)</div><input class="prop-value" id="p_surface_angle" type="number" min="-90" max="90" step="0.1" value="' +
          Number((o.surface_alignment && o.surface_alignment.angle_deg) || 0).toFixed(1) + '"></div>' +
        '<button class="small" id="applySurfaceAlignBtn">Apply Surface Alignment</button>') +
      '<div class="prop-row"><div class="prop-label">Length (m)</div><input class="prop-value" id="p_length" type="number" min="0.01" step="0.01" value="' + lengthOf(o.points).toFixed(2) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Elbows</div><div class="prop-value">' + elbows(o.points) + '</div></div>' +
      '<div class="property-hint">Drag the selected route in the 3D view for free 3D movement. Position fields give exact XYZ control.</div>' +
      '<button class="small danger" id="deleteObjectBtn">Delete</button>';

    bindPropertyHistoryInput('p_name', function(e){ o.name = e.target.value; renderScene(); });
    bindPropertyHistoryInput('p_spec', function(e){ o.specification = e.target.value; renderBoq(); });
    bindPropertyHistoryInput('p_material', function(e){ o.material = e.target.value; renderBoq(); });
    bindPropertyHistoryChange('p_length', function(e){
      if (!resizeRouteToLength(o, e.target.value)) return;
      rebuildRoutes();
      renderScene();
      renderBoq();
      renderProperties();
    });

    bindPropertyHistoryInput('p_a', function(e){
      if (cable) o.diameter_mm = Number(e.target.value) || 1;
      else o.width_mm = Number(e.target.value) || 1;
      rebuildRoutes(); renderBoq(); updateRouteSelectionVisuals();
    });

    if (!cable) {
      bindPropertyHistoryInput('p_h', function(e){
        o.height_mm = Number(e.target.value) || 1;
        rebuildRoutes(); renderBoq(); updateRouteSelectionVisuals();
      });

      const surfaceMode = $('p_surface_mode');
      const surfaceAngle = $('p_surface_angle');
      if (surfaceMode && surfaceAngle) {
        const alignment = o.surface_alignment || {};
        surfaceMode.value = alignment.mode === 'perpendicular' || alignment.mode === 'angle' ? alignment.mode : 'parallel';
        surfaceMode.addEventListener('change', function(e){
          const mode = e.target.value;
          if (mode === 'parallel') surfaceAngle.value = '0';
          if (mode === 'perpendicular') surfaceAngle.value = '90';
        });
      }

      $('pickSurfaceBtn').addEventListener('click', function(){
        state.surfacePickMode = true;
        status('Click a CAD/model surface');
        toast('Click the surface to align this tray');
      });

      $('applySurfaceAlignBtn').addEventListener('click', function(){
        applySurfaceAlignment(o, $('p_surface_mode').value, $('p_surface_angle').value);
      });
    }

    function updatePosition(axis, value) {
      const target = Number(value);
      if (!Number.isFinite(target)) return;
      const currentCenter = routeCenter(o.points);
      const targetScene = target / 10;
      const delta = targetScene - currentCenter[axis];
      if (Math.abs(delta) < 1e-9) return;
      o.points.forEach(function(p){ p[axis] += delta; });
      rebuildRoutes();
      renderScene();
      renderBoq();
      renderProperties();
    }

    bindPropertyHistoryChange('p_pos_x', function(e){ updatePosition('x', e.target.value); });
    bindPropertyHistoryChange('p_pos_y', function(e){ updatePosition('y', e.target.value); });
    bindPropertyHistoryChange('p_pos_z', function(e){ updatePosition('z', e.target.value); });

    function updateRotation(axis, value) {
      const target = Number(value);
      if (!Number.isFinite(target)) return;
      if (!o.rotation_deg) o.rotation_deg = { x: 0, y: 0, z: 0 };
      o.rotation_deg[axis] = target;
      rebuildRoutes();
      renderScene();
      renderBoq();
      renderProperties();
    }

    bindPropertyHistoryChange('p_rot_x', function(e){ updateRotation('x', e.target.value); });
    bindPropertyHistoryChange('p_rot_y', function(e){ updateRotation('y', e.target.value); });
    bindPropertyHistoryChange('p_rot_z', function(e){ updateRotation('z', e.target.value); });
  }

  $('deleteObjectBtn').addEventListener('click', deleteSelected);
}
function trimNumber(value, decimals) {
  const n = Number(value);
  if (!Number.isFinite(n)) return '0';
  return n.toFixed(decimals).replace(/\\.?0+$/, '');
}

function quantitySizeLabel(group) {
  if (group.kind === 'cable') {
    return 'Ø' + trimNumber(group.diameter_mm, 3) + ' mm';
  }
  return trimNumber(group.width_mm / 10, 2) + ' × ' + trimNumber(group.height_mm / 10, 2) + ' cm';
}

function buildQuantityTakeoffRows() {
  const grouped = new Map();
  state.objects
    .filter(function(o){ return o.kind === 'cable' || o.kind === 'tray'; })
    .forEach(function(o){
      const length = lengthOf(o.points);
      const elbowCount = elbows(o.points);
      const diameter = Number(o.diameter_mm);
      const width = Number(o.width_mm);
      const height = Number(o.height_mm);

      let key;
      let group;
      if (o.kind === 'cable') {
        key = 'cable|' + (Number.isFinite(diameter) ? diameter.toFixed(6) : '0');
        group = grouped.get(key);
        if (!group) {
          group = {
            kind: 'cable',
            diameter_mm: Number.isFinite(diameter) ? diameter : 0,
            length_m: 0,
            elbows: 0
          };
          grouped.set(key, group);
        }
      } else {
        key = 'tray|' +
          (Number.isFinite(width) ? width.toFixed(6) : '0') + '|' +
          (Number.isFinite(height) ? height.toFixed(6) : '0');
        group = grouped.get(key);
        if (!group) {
          group = {
            kind: 'tray',
            width_mm: Number.isFinite(width) ? width : 0,
            height_mm: Number.isFinite(height) ? height : 0,
            length_m: 0,
            elbows: 0
          };
          grouped.set(key, group);
        }
      }

      group.length_m += length;
      group.elbows += elbowCount;
    });

  return Array.from(grouped.values()).sort(function(a,b){
    if (a.kind !== b.kind) return a.kind === 'tray' ? -1 : 1;
    if (a.kind === 'tray') {
      return (a.width_mm - b.width_mm) || (a.height_mm - b.height_mm);
    }
    return a.diameter_mm - b.diameter_mm;
  });
}

function renderBoq() {
  const routes = state.objects.filter(function(o){ return o.kind === 'cable' || o.kind === 'tray'; });
  const cableM = routes.filter(function(o){ return o.kind === 'cable'; }).reduce(function(s,o){ return s + lengthOf(o.points); }, 0);
  const trayM = routes.filter(function(o){ return o.kind === 'tray'; }).reduce(function(s,o){ return s + lengthOf(o.points); }, 0);
  const elbowN = routes.reduce(function(s,o){ return s + elbows(o.points); }, 0);
  const rows = buildQuantityTakeoffRows();

  $('totalCable').textContent = cableM.toFixed(2) + ' m';
  $('totalTray').textContent = trayM.toFixed(2) + ' m';
  $('totalElbows').textContent = String(elbowN);

  $('boqTableWrap').innerHTML = rows.length
    ? '<table><thead><tr><th>Item</th><th>Size</th><th>Total Quantity</th><th>Unit</th><th>Elbows</th></tr></thead><tbody>' +
      rows.map(function(group){
        const item = group.kind === 'cable' ? 'Cable' : 'Cable Tray';
        return '<tr><td>' + item + '</td><td>' + esc(quantitySizeLabel(group)) + '</td><td>' +
          group.length_m.toFixed(2) + '</td><td>m</td><td>' + group.elbows + '</td></tr>';
      }).join('') +
      '</tbody></table>'
    : '<div class="hint" style="padding:10px">No routing quantities yet.</div>';
}
function render() { renderScene(); renderProperties(); renderBoq(); updateRouteSelectionVisuals(); }

async function importModelFile(fileUrl, displayName, format, nativeFormat, sourcePath) {
  const url = fileUrl;
  const selectedSourcePath = sourcePath || '';
  const selectedNativeFormat = nativeFormat || format || 'STL';
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
    const sourceRecord = {
      id: objectId,
      name: displayName,
      source: displayName,
      path: selectedSourcePath || '',
      native_format: selectedNativeFormat || format,
      format: format
    };
    state.sourceModels.push(sourceRecord);
    state.objects.push({
      id: objectId,
      kind: 'model',
      name: displayName,
      source: displayName,
      format: format,
      native_format: sourceRecord.native_format,
      source_path: sourceRecord.path
    });
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
  return {
    schema: 'cable-tray-project',
    schema_version: 2,
    project: state.project,
    model_sources: state.sourceModels.map(function(m){
      const root = state.modelRoots.get(m.id);
      return {
        id:m.id,
        name:m.name,
        source:m.source,
        path:m.path,
        native_format:m.native_format,
        format:m.format,
        snapshot: root ? root.toJSON() : (m.snapshot || null)
      };
    }),
    objects: state.objects
      .filter(function(o){ return o.kind !== 'model'; })
      .map(function(o){
        return {
          id:o.id,
          kind:o.kind,
          name:o.name,
          points:o.points.map(function(p){ return {x:p.x*10,y:p.y*10,z:p.z*10}; }),
          diameter_mm:o.diameter_mm,
          width_mm:o.width_mm,
          height_mm:o.height_mm,
          specification:o.specification,
          material:o.material,
          rotation_deg:{x:Number(o.rotation_deg && o.rotation_deg.x) || 0,y:Number(o.rotation_deg && o.rotation_deg.y) || 0,z:Number(o.rotation_deg && o.rotation_deg.z) || 0},
          surface_alignment:o.surface_alignment ? {
            mode:o.surface_alignment.mode || 'parallel',
            angle_deg:Number(o.surface_alignment.angle_deg) || 0,
            normal:o.surface_alignment.normal ? {
              x:Number(o.surface_alignment.normal.x) || 0,
              y:Number(o.surface_alignment.normal.y) || 0,
              z:Number(o.surface_alignment.normal.z) || 0
            } : null
          } : null
        };
      }),
    measurements: state.measurements.map(function(m){
      return {
        id:m.id,
        kind:m.kind,
        start:{x:m.start.x*10,y:m.start.y*10,z:m.start.z*10},
        end:{x:m.end.x*10,y:m.end.y*10,z:m.end.z*10},
        distance_m:Number(m.distance_m)||0,
        start_normal:m.start_normal?{x:m.start_normal.x,y:m.start_normal.y,z:m.start_normal.z}:null,
        end_normal:m.end_normal?{x:m.end_normal.x,y:m.end_normal.y,z:m.end_normal.z}:null,
        start_anchor:m.start_anchor || null,
        end_anchor:m.end_anchor || null
      };
    }),
    measurements_visible:state.measurementsVisible
  };
}
function downloadBlob(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(function(){ URL.revokeObjectURL(url); }, 1000);
}

function exportMaterial(sourceMaterial) {
  const material = Array.isArray(sourceMaterial) ? sourceMaterial[0] : sourceMaterial;
  let color = 0xb6bec7;
  if (material && material.color) {
    if (typeof material.color.getHex === "function") color = material.color.getHex();
    else if (Number.isFinite(material.color)) color = material.color;
  }

  const result = new THREE.MeshStandardMaterial({
    color: color,
    roughness: material && Number.isFinite(material.roughness) ? material.roughness : 0.55,
    metalness: material && Number.isFinite(material.metalness) ? material.metalness : 0.15,
    transparent: !!(material && material.transparent),
    opacity: material && Number.isFinite(material.opacity) ? material.opacity : 1,
    side: material && Number.isFinite(material.side) ? material.side : THREE.FrontSide
  });

  if (material && material.map) result.map = material.map.clone();
  return result;
}

function prepareExportClone(sourceRoot) {
  const clone = sourceRoot.clone(true);
  const removeNodes = [];

  clone.traverse(function(node){
    // Lines are editor overlays only; they are not part of the saved 3D model.
    if (node.isLine || node.isLineSegments || node.isPoints) {
      removeNodes.push(node);
      return;
    }

    // Keep exporter-facing userData JSON-safe. The live material objects used
    // by the editor must never be serialized into GLB extras.
    if (node.userData) {
      node.userData = { ...node.userData };
      delete node.userData.meshMaterial;
      delete node.userData.lineMaterial;
    }

    if (node.isMesh && node.material) {
      node.material = Array.isArray(node.material)
        ? node.material.map(function(m){ return exportMaterial(m); })
        : exportMaterial(node.material);
    }
  });

  removeNodes.forEach(function(node){
    if (node.parent) node.parent.remove(node);
  });

  clone.updateMatrixWorld(true);
  return clone;
}

function exportSceneRoot() {
  const exportRoot = new THREE.Group();
  exportRoot.name = state.project.name || 'Cable_Tray_Model';

  state.modelRoots.forEach(function(root){
    exportRoot.add(prepareExportClone(root));
  });
  state.routeRoots.forEach(function(root){
    const clone = prepareExportClone(root);
    applyRouteSelectionVisual(clone, false);
    exportRoot.add(clone);
  });

  exportRoot.updateMatrixWorld(true);
  return exportRoot;
}
function exportModel(format) {
  const chosen = String(format || '').toUpperCase();
  const exportRoot = exportSceneRoot();
  const baseName = slug(state.project.name || 'cable-tray-model');

  status('Exporting ' + chosen + '...');
  if (chosen === 'GLB') {
    // glTF/GLB uses meters as its unit convention.
    exportRoot.scale.setScalar(0.01);
    exportRoot.updateMatrixWorld(true);
    const exporter = new GLTFExporter();
    exporter.parse(
      exportRoot,
      function(result){
        if (!(result instanceof ArrayBuffer)) {
          status('Ready');
          toast('Export failed: GLB export did not return binary data.');
          return;
        }
        downloadBlob(new Blob([result], { type: 'model/gltf-binary' }), baseName + '.glb');
        status('Model exported');
        toast('GLB exported');
      },
      function(error){
        console.error(error);
        status('Ready');
        toast('Export failed: ' + (error && error.message ? error.message : String(error)));
      },
      { binary: true, trs: false, onlyVisible: true }
    );
    return;
  }

  if (chosen === 'OBJ') {
    // OBJ has no mandatory unit metadata; export in millimetres.
    exportRoot.scale.setScalar(10);
    exportRoot.updateMatrixWorld(true);
    const text = new OBJExporter().parse(exportRoot);
    downloadBlob(new Blob([text], { type: 'text/plain;charset=utf-8' }), baseName + '.obj');
    status('Model exported');
    toast('OBJ exported');
    return;
  }

  if (chosen === 'STL') {
    // STL has no unit metadata; export in millimetres for CAD workflows.
    exportRoot.scale.setScalar(10);
    exportRoot.updateMatrixWorld(true);
    const result = new STLExporter().parse(exportRoot, { binary: true });
    const data = result && result.buffer instanceof ArrayBuffer ? result.buffer : result;
    downloadBlob(new Blob([data], { type: 'model/stl' }), baseName + '.stl');
    status('Model exported');
    toast('STL exported');
    return;
  }

  throw new Error('Unsupported export format: ' + format);
}

$('exportModelBtn').addEventListener('click', function(){
  const format = $('exportModelFormat').value;
  try {
    exportModel(format);
  } catch (err) {
    console.error(err);
    status('Ready');
    toast('Export failed: ' + err.message);
  }
});

$('saveProjectBtn').addEventListener('click', saveProjectFile);
function loadProject(data) {
  if (!data || data.schema !== 'cable-tray-project') throw new Error('Not a Cable_tray project');

  state.modelRoots.forEach(function(root){ scene.remove(root); });
  state.modelRoots.clear();
  state.routeRoots.forEach(function(root){ if (root.parent) root.parent.remove(root); });
  state.routeRoots.clear();

  state.sourceModels = (data.model_sources || []).map(function(m){ return { ...m }; });
  state.objects = (data.objects || []).map(function(o){
    const rotation = o.rotation_deg || { x: 0, y: 0, z: 0 };
    return {
      ...o,
      rotation_deg: {
        x: Number(rotation.x) || 0,
        y: Number(rotation.y) || 0,
        z: Number(rotation.z) || 0
      },
      points:(o.points || []).map(function(p){ return new THREE.Vector3(mmToScene(p.x),mmToScene(p.y),mmToScene(p.z)); })
    };
  });

  const objectLoader = new THREE.ObjectLoader();
  let missingModels = 0;
  state.sourceModels.forEach(function(m){
    if (!m.snapshot) {
      missingModels++;
      state.objects.push({
        id:m.id, kind:'model', name:m.name, source:m.source,
        format:m.format, native_format:m.native_format, source_path:m.path || ''
      });
      return;
    }
    try {
      const root = objectLoader.parse(m.snapshot);
      root.userData = root.userData || {};
      root.userData.objectId = m.id;
      root.traverse(function(n){
        n.userData = n.userData || {};
        n.userData.objectId = m.id;
      });
      scene.add(root);
      state.modelRoots.set(m.id, root);
      state.objects.push({
        id:m.id, kind:'model', name:m.name, source:m.source,
        format:m.format, native_format:m.native_format, source_path:m.path || ''
      });
    } catch (error) {
      missingModels++;
      console.error('Model snapshot load failed', m.name, error);
    }
  });

  state.project = { ...state.project, ...(data.project || {}) };
  state.measurementsVisible = data.measurements_visible !== false;
  state.measurements = (data.measurements || []).map(function(m){
    return {
      id:m.id || id('measure'),
      kind:m.kind === 'surface' ? 'surface' : 'point',
      start:new THREE.Vector3(mmToScene(m.start.x),mmToScene(m.start.y),mmToScene(m.start.z)),
      end:new THREE.Vector3(mmToScene(m.end.x),mmToScene(m.end.y),mmToScene(m.end.z)),
      dimensionEnd:new THREE.Vector3(mmToScene(m.end.x),mmToScene(m.end.y),mmToScene(m.end.z)),
      distance_m:Number(m.distance_m)||0,
      start_normal:m.start_normal ? new THREE.Vector3(m.start_normal.x,m.start_normal.y,m.start_normal.z) : null,
      end_normal:m.end_normal ? new THREE.Vector3(m.end_normal.x,m.end_normal.y,m.end_normal.z) : null,
      start_anchor:m.start_anchor || null,
      end_anchor:m.end_anchor || null
    };
  });

  $('projectName').value = state.project.name || 'Factory Cable Routing';
  $('unitSystem').value = state.project.units || 'mm';
  state.selected = null;
  state.measureStart = null;
  state.selectedMeasurementId = null;
  state.surfaceAlignStart = null;
  resetHistory();
  rebuildRoutes();
  syncMeasurements();
  rebuildMeasurements();
  renderMeasurementsToggle();
  renderMeasurementList();
  render();

  toast(missingModels ? 'Project loaded; ' + missingModels + ' model(s) could not be restored.' : 'Project loaded');
}
$('projectFile').addEventListener('change', async function(event){
  const file = event.target.files && event.target.files[0];
  event.target.value = '';
  if (!file) return;
  try {
    const data = JSON.parse(await file.text());
    loadProject(data);
  } catch (error) {
    console.error(error);
    toast('Load Project failed: ' + (error && error.message ? error.message : String(error)));
  }
});

$('newProjectBtn').addEventListener('click', function(){
  if (!confirm('Clear the current design?')) return;
  state.modelRoots.forEach(function(root){ scene.remove(root); }); state.modelRoots.clear();
  state.sourceModels = [];
  state.objects = []; state.selected = null; resetHistory(); state.surfacePick = null; state.surfacePickMode = false;
  state.measureStart = null; state.surfaceAlignStart = null; state.selectedMeasurementId = null; state.measurements = []; clearSurfaceSelectionVisuals(); rebuildMeasurements(); renderMeasurementsToggle(); renderMeasurementList(); render(); toast('New project created');
});
$('exportBoqBtn').addEventListener('click', function(){
  const rows = [['Item','Size','Total Quantity','Unit','Elbows']];
  buildQuantityTakeoffRows().forEach(function(group){
    rows.push([
      group.kind === 'cable' ? 'Cable' : 'Cable Tray',
      quantitySizeLabel(group),
      group.length_m.toFixed(3),
      'm',
      String(group.elbows)
    ]);
  });
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
$('toggleMeasurementsBtn').addEventListener('click', toggleMeasurements);
$('clearMeasurementsBtn').addEventListener('click', function(){
  if (!state.measurements.length) return;
  if (!confirm('Delete all dimensions?')) return;
  clearAllMeasurements();
});
setTool('select'); rebuildMeasurements(); renderMeasurementsToggle(); renderMeasurementList(); render(); animate();

function animate(){
  requestAnimationFrame(animate);
  controls.update();
  syncMeasurements();
  updateMeasurementOverlay();
  renderer.render(scene,camera);
}
$('createAutoRouteBtn').addEventListener('click', createAutoRouteFromPreview);
$('resetAutoRouteBtn').addEventListener('click', resetAutoRoutePoints);
