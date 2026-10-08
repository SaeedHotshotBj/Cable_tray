import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { OBJExporter } from 'three/addons/exporters/OBJExporter.js';
import { STLExporter } from 'three/addons/exporters/STLExporter.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { computeBoundsTree, disposeBoundsTree, acceleratedRaycast } from 'three-mesh-bvh';
import { routeEngineeringNetwork } from './engineering_routing.js';

const state = {
  project: { name: 'Factory Cable Routing', units: 'mm', schema_version: 3 },
  objects: [], selected: null, tool: 'select', drawing: null,
  modelRoots: new Map(), routeRoots: new Map(), measureStart: null,
  sourceModels: [], dragging: null, skipClick: false,
  surfacePickMode: false, surfacePick: null,
  measureStart: null, measureMode: 'point', autoRoutePoints: [], autoRoutePreviewRoot: null, autoRouteClearanceMm: 100, measurements: [], measurementsVisible: true,
  selectedMeasurementId: null,
  surfaceAlignStart: null,
  clipboard: null,
  equipment: [],
  panels: [],
  engineeringFloors: [],
  engineeringFloorPoints: [],
  engineeringSettings: {
    gridStepMm: 100,
    clearanceMm: 100,
    maxBodyDistanceMm: 1500,
    namesVisible: true,
    mainMinCables: 2,
    traySideMarginMm: 25,
    standardTrayWidthsMm: [100,150,200,300,400,500,600,800,1000,1200]
  },
  lastEngineeringReport: null,
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
controls.dampingFactor = 0.085;
controls.enablePan = true;
controls.screenSpacePanning = true;
controls.zoomToCursor = false;
controls.zoomSpeed = 0.8;
controls.panSpeed = 0.85;
controls.rotateSpeed = 0.7;
controls.minDistance = 2;
controls.maxDistance = 300000;
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
const engineeringMarkerRoot = new THREE.Group();
engineeringMarkerRoot.name = 'EngineeringAnnotations';
scene.add(engineeringMarkerRoot);
state.engineeringMarkerRoot = engineeringMarkerRoot;
const engineeringAccessoryRoot = new THREE.Group();
engineeringAccessoryRoot.name = 'EngineeringTrayAccessories';
scene.add(engineeringAccessoryRoot);
state.engineeringAccessoryRoot = engineeringAccessoryRoot;
const raycaster = new THREE.Raycaster();
const engineeringCollisionRaycaster = new THREE.Raycaster();
engineeringCollisionRaycaster.firstHitOnly = true;
THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
const mouse = new THREE.Vector2();
let engineeringCollisionCache = new Map();
let engineeringBodyDistanceCache = new Map();
let engineeringCollisionMeshCache = [];

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
function elbowAngleDeg(points, index) {
  if (!points || index <= 0 || index >= points.length - 1) return 0;
  const incoming = points[index].clone().sub(points[index - 1]);
  const outgoing = points[index + 1].clone().sub(points[index]);
  if (incoming.lengthSq() < 1e-12 || outgoing.lengthSq() < 1e-12) return 0;
  incoming.normalize();
  outgoing.normalize();
  const dot = THREE.MathUtils.clamp(incoming.dot(outgoing), -1, 1);
  return THREE.MathUtils.radToDeg(Math.acos(dot));
}

function setRouteElbowAngle(route, index, targetDeg) {
  if (!route || index <= 0 || index >= route.points.length - 1) return false;

  const angle = Number(targetDeg);
  if (!Number.isFinite(angle) || angle < 0 || angle > 180) return false;

  const pivot = route.points[index].clone();
  const incoming = pivot.clone().sub(route.points[index - 1]);
  const outgoing = route.points[index + 1].clone().sub(pivot);
  if (incoming.lengthSq() < 1e-12 || outgoing.lengthSq() < 1e-12) return false;

  incoming.normalize();
  const outgoingLength = outgoing.length();
  if (outgoingLength < 1e-9) return false;
  outgoing.normalize();

  let bendAxis = outgoing.clone().sub(
    incoming.clone().multiplyScalar(outgoing.dot(incoming))
  );
  if (bendAxis.lengthSq() < 1e-12) {
    const fallback = Math.abs(incoming.y) < 0.999
      ? new THREE.Vector3(0, 1, 0)
      : new THREE.Vector3(0, 0, 1);
    bendAxis = fallback.sub(incoming.clone().multiplyScalar(fallback.dot(incoming)));
  }
  if (bendAxis.lengthSq() < 1e-12) return false;
  bendAxis.normalize();

  const targetRad = THREE.MathUtils.degToRad(angle);
  const desiredOutgoing = incoming.clone().multiplyScalar(Math.cos(targetRad))
    .add(bendAxis.clone().multiplyScalar(Math.sin(targetRad)))
    .normalize();

  const rotation = new THREE.Quaternion().setFromUnitVectors(outgoing, desiredOutgoing);

  for (let i = index + 1; i < route.points.length; i++) {
    const offset = route.points[i].clone().sub(pivot);
    offset.applyQuaternion(rotation);
    route.points[i].copy(pivot).add(offset);
  }
  return true;
}

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
    engineering: {
      equipment: JSON.parse(JSON.stringify(state.equipment)),
      panels: JSON.parse(JSON.stringify(state.panels)),
      floors: JSON.parse(JSON.stringify(state.engineeringFloors)),
      settings: JSON.parse(JSON.stringify(state.engineeringSettings))
    },
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
    const engineering = snapshot.engineering || {};
    state.equipment = JSON.parse(JSON.stringify(engineering.equipment || []));
    state.panels = JSON.parse(JSON.stringify(engineering.panels || []));
    state.engineeringFloors = JSON.parse(JSON.stringify(engineering.floors || []));
    state.engineeringFloorPoints = [];
    const restoredEngineeringSettings = { ...(engineering.settings || {}) };
    delete restoredEngineeringSettings.routingElevationMm;
    state.engineeringSettings = { ...state.engineeringSettings, ...restoredEngineeringSettings };
    renderEngineeringNamesToggle();
    state.project = { ...state.project, ...(snapshot.project || {}) };
    const selectedExists = state.objects.some(function(o){ return o.id === snapshot.selected; }) ||
      state.equipment.some(function(o){ return o.id === snapshot.selected; }) ||
      state.panels.some(function(o){ return o.id === snapshot.selected; });
    state.selected = selectedExists ? snapshot.selected : null;
    rebuildRoutes();
    rebuildEngineeringFloorsList();
    rebuildEngineeringMarkers();
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


function engineeringEntity(idValue) {
  return state.equipment.find(function(item){ return item.id === idValue; }) ||
    state.panels.find(function(item){ return item.id === idValue; }) || null;
}

function engineeringEntityKind(idValue) {
  if (state.equipment.some(function(item){ return item.id === idValue; })) return 'equipment';
  if (state.panels.some(function(item){ return item.id === idValue; })) return 'panel';
  return null;
}

function engineeringLabel(textValue, fillColor) {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 128;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.font = '700 42px Segoe UI, Arial, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = 'rgba(8,13,18,0.92)';
  ctx.fillRect(8, 8, canvas.width - 16, canvas.height - 16);
  ctx.strokeStyle = fillColor;
  ctx.lineWidth = 5;
  ctx.strokeRect(8, 8, canvas.width - 16, canvas.height - 16);
  ctx.fillStyle = '#ffffff';
  ctx.fillText(String(textValue || ''), canvas.width / 2, canvas.height / 2);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({ map:texture, transparent:true, depthTest:false });
  const sprite = new THREE.Sprite(material);
  sprite.userData.engineeringLabel = true;
  sprite.scale.set(90, 22.5, 1);
  sprite.renderOrder = 60;
  return sprite;
}

function resolveEngineeringAnchor(entity) {
  if (!entity || !entity.anchor) return null;
  const anchor = entity.anchor;
  const root = anchor.model_id ? state.modelRoots.get(anchor.model_id) : null;

  if (root && anchor.local_point) {
    const localPoint = new THREE.Vector3(
      Number(anchor.local_point.x) || 0,
      Number(anchor.local_point.y) || 0,
      Number(anchor.local_point.z) || 0
    );
    const worldPoint = root.localToWorld(localPoint);
    anchor.point = {
      x:worldPoint.x * 10,
      y:worldPoint.y * 10,
      z:worldPoint.z * 10
    };
    if (anchor.local_normal) {
      const normalMatrix = new THREE.Matrix3().getNormalMatrix(root.matrixWorld);
      const worldNormal = new THREE.Vector3(
        Number(anchor.local_normal.x) || 0,
        Number(anchor.local_normal.y) || 1,
        Number(anchor.local_normal.z) || 0
      ).applyMatrix3(normalMatrix).normalize();
      anchor.normal = {x:worldNormal.x,y:worldNormal.y,z:worldNormal.z};
    }
  }
  return anchor;
}

function syncEngineeringAnchors() {
  state.equipment.forEach(resolveEngineeringAnchor);
  state.panels.forEach(resolveEngineeringAnchor);
}

function disposeEngineeringMarkers() {
  if (!state.engineeringMarkerRoot) return;
  state.engineeringMarkerRoot.traverse(function(node){
    if (node.geometry && typeof node.geometry.dispose === 'function') node.geometry.dispose();
    const materials = Array.isArray(node.material) ? node.material : (node.material ? [node.material] : []);
    materials.forEach(function(material){
      if (material.map && typeof material.map.dispose === 'function') material.map.dispose();
      if (typeof material.dispose === 'function') material.dispose();
    });
  });
  state.engineeringMarkerRoot.clear();
}

function floorPolygonArea(points) {
  if (!Array.isArray(points) || points.length < 3) return 0;
  let area = 0;
  for (let i = 0; i < points.length; i++) {
    const a = points[i];
    const b = points[(i + 1) % points.length];
    area += Number(a.x) * Number(b.z) - Number(b.x) * Number(a.z);
  }
  return Math.abs(area) * 0.5;
}

function floorPointFromEvent(event) {
  pointerRay(event);
  const roots = Array.from(state.modelRoots.values());
  if (roots.length) {
    const hit = raycaster.intersectObjects(roots, true)[0];
    if (hit && hit.point) {
      return {x:hit.point.x * 10,y:hit.point.y * 10,z:hit.point.z * 10};
    }
  }
  const hit = raycaster.intersectObject(ground, false)[0];
  if (!hit) return null;
  return {x:hit.point.x * 10,y:hit.point.y * 10,z:hit.point.z * 10};
}

function renderEngineeringFloorsList() {
  const box = $('engineeringFloors');
  if (!box) return;
  if (!state.engineeringFloors.length) {
    box.innerHTML = '<div class="hint">No floor definitions. Auto Design will group equipment by height automatically.</div>';
    return;
  }

  box.innerHTML = state.engineeringFloors.map(function(floor, index){
    const areaM2 = Number(floor.area_m2) ||
      (Number(floor.area_mm2) > 0 ? Number(floor.area_mm2) / 1000000 : 0);
    return '<div class="engineering-floor-item">' +
      '<div><b>' + esc(floor.name || ('Floor ' + (index + 1))) + '</b>' +
      '<span>Y ' + Number(floor.base_y_mm || 0).toFixed(0) + ' mm · ' +
      areaM2.toFixed(2) + ' m²</span></div>' +
      '<button class="small danger engineering-floor-delete" data-floor-id="' +
      esc(floor.id) + '">Delete</button></div>';
  }).join('');

  box.querySelectorAll('.engineering-floor-delete').forEach(function(button){
    button.addEventListener('click', function(){
      const floorId = button.dataset.floorId;
      const before = captureDesignState();
      state.engineeringFloors = state.engineeringFloors.filter(function(item){
        return item.id !== floorId;
      });
      recordHistory(before);
      rebuildEngineeringMarkers();
      renderEngineeringFloorsList();
      toast('Floor definition deleted');
    });
  });
}

function finishEngineeringFloor() {
  const points = state.engineeringFloorPoints.map(function(point){ return {...point}; });
  if (points.length !== 4) {
    toast('Select exactly four corner points for the floor');
    return;
  }

  const yValues = points.map(function(point){ return Number(point.y) || 0; });
  const yRange = Math.max.apply(null, yValues) - Math.min.apply(null, yValues);
  if (yRange > 250) {
    toast('The four floor points must lie on the same floor level');
    return;
  }
  if (floorPolygonArea(points) <= 1) {
    toast('The four floor points must enclose a valid area');
    return;
  }

  const name = window.prompt(
    'Floor name',
    'Floor ' + String(state.engineeringFloors.length + 1)
  );
  if (name === null) {
    state.engineeringFloorPoints = [];
    autoRoutePreviewRoot.clear();
    setTool('select');
    return;
  }

  const baseY = yValues.reduce(function(sum, value){ return sum + value; }, 0) / yValues.length;
  const areaMm2 = floorPolygonArea(points);
  const before = captureDesignState();

  state.engineeringFloors.push({
    id:id('floor'),
    name:String(name || ('Floor ' + String(state.engineeringFloors.length + 1))),
    points:points,
    base_y_mm:baseY,
    area_mm2:areaMm2,
    area_m2:areaMm2 / 1000000,
    height_tolerance_mm:1000
  });

  state.engineeringFloorPoints = [];
  autoRoutePreviewRoot.clear();
  recordHistory(before);
  rebuildEngineeringMarkers();
  renderEngineeringFloorsList();
  setTool('select');
  toast('Floor definition saved');
}

function addEngineeringFloorPoint(event) {
  const point = floorPointFromEvent(event);
  if (!point) {
    toast('Click a valid point on the CAD/model floor');
    return;
  }

  state.engineeringFloorPoints.push(point);
  autoRoutePreviewRoot.clear();

  state.engineeringFloorPoints.forEach(function(item, index){
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(Math.max(2, mmToScene(45)), 16, 16),
      new THREE.MeshBasicMaterial({
        color:index === 0 ? 0xffd45a : 0x66d9ff,
        depthTest:false
      })
    );
    marker.position.set(mmToScene(item.x),mmToScene(item.y),mmToScene(item.z));
    marker.renderOrder = 70;
    autoRoutePreviewRoot.add(marker);
  });

  if (state.engineeringFloorPoints.length >= 2) {
    const points = state.engineeringFloorPoints.map(function(item){
      return new THREE.Vector3(mmToScene(item.x),mmToScene(item.y),mmToScene(item.z));
    });
    if (state.engineeringFloorPoints.length === 4) points.push(points[0].clone());
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({color:0x66d9ff,depthTest:false})
    );
    line.renderOrder = 69;
    autoRoutePreviewRoot.add(line);
  }

  status('Define Floor: ' + state.engineeringFloorPoints.length + '/4 corner points');
  if (state.engineeringFloorPoints.length === 4) finishEngineeringFloor();
}

function rebuildEngineeringFloorMarkers() {
  if (!state.engineeringMarkerRoot) return;
  state.engineeringFloors.forEach(function(floor){
    if (!Array.isArray(floor.points) || floor.points.length < 4) return;
    const linePoints = floor.points.map(function(point){
      return new THREE.Vector3(mmToScene(point.x),mmToScene(point.y),mmToScene(point.z));
    });
    linePoints.push(linePoints[0].clone());

    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(linePoints),
      new THREE.LineBasicMaterial({
        color:0x4da3ff,
        transparent:true,
        opacity:0.78,
        depthTest:false
      })
    );
    line.renderOrder = 55;
    line.userData.engineeringFloor = floor.id;
    state.engineeringMarkerRoot.add(line);

    const center = floor.points.reduce(function(sum, point){
      sum.add(new THREE.Vector3(mmToScene(point.x),mmToScene(point.y),mmToScene(point.z)));
      return sum;
    }, new THREE.Vector3()).multiplyScalar(1 / floor.points.length);

    const label = engineeringLabel(
      floor.name + '  (' + Number(floor.base_y_mm || 0).toFixed(0) + ' mm)',
      '#4da3ff'
    );
    label.position.copy(center);
    label.position.y += mmToScene(180);
    label.scale.set(110, 27.5, 1);
    label.userData.engineeringFloor = floor.id;
    state.engineeringMarkerRoot.add(label);
  });
}

function rebuildEngineeringMarkers() {
  if (!state.engineeringMarkerRoot) return;
  disposeEngineeringMarkers();
  syncEngineeringAnchors();
  rebuildEngineeringFloorMarkers();

  state.panels.forEach(function(panel){
    if (!panel.anchor || !panel.anchor.point) return;
    const p = panel.anchor.point;
    const root = new THREE.Group();
    root.userData.objectId = panel.id;
    root.userData.engineeringEntity = 'panel';
    const box = new THREE.Mesh(
      new THREE.BoxGeometry(mmToScene(120), mmToScene(120), mmToScene(120)),
      new THREE.MeshBasicMaterial({ color:0x54a9ff, transparent:true, opacity:0.9, depthTest:false })
    );
    box.userData.objectId = panel.id;
    box.userData.engineeringEntity = 'panel';
    root.add(box);
    const label = engineeringLabel(panel.name, '#54a9ff');
    label.position.y = mmToScene(140);
    label.visible = state.engineeringSettings.namesVisible !== false;
    label.userData.objectId = panel.id;
    label.userData.engineeringEntity = 'panel';
    root.add(label);
    root.position.set(mmToScene(p.x),mmToScene(p.y),mmToScene(p.z));
    engineeringMarkerRoot.add(root);
  });

  state.equipment.forEach(function(item){
    if (!item.anchor || !item.anchor.point) return;
    const p = item.anchor.point;
    const root = new THREE.Group();
    root.userData.objectId = item.id;
    root.userData.engineeringEntity = 'equipment';
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(mmToScene(55), 16, 12),
      new THREE.MeshBasicMaterial({ color:0xffa64d, transparent:true, opacity:0.92, depthTest:false })
    );
    sphere.userData.objectId = item.id;
    sphere.userData.engineeringEntity = 'equipment';
    root.add(sphere);
    const label = engineeringLabel(item.name, '#ffa64d');
    label.position.y = mmToScene(105);
    label.visible = state.engineeringSettings.namesVisible !== false;
    label.userData.objectId = item.id;
    label.userData.engineeringEntity = 'equipment';
    root.add(label);
    root.position.set(mmToScene(p.x),mmToScene(p.y),mmToScene(p.z));
    engineeringMarkerRoot.add(root);
  });
}

function pickEngineeringAnchor(event) {
  pointerRay(event);
  const roots = Array.from(state.modelRoots.values());
  if (!roots.length) return null;
  const hits = raycaster.intersectObjects(roots, true);
  const hit = hits[0];
  if (!hit || !hit.point || !hit.object) return null;

  let modelId = hit.object.userData && hit.object.userData.objectId;
  if (!modelId || !state.modelRoots.has(modelId)) return null;
  const root = state.modelRoots.get(modelId);

  let normal = new THREE.Vector3(0,1,0);
  if (hit.face) {
    normal.copy(hit.face.normal);
    normal.applyNormalMatrix(new THREE.Matrix3().getNormalMatrix(hit.object.matrixWorld)).normalize();
  }

  const localPoint = root.worldToLocal(hit.point.clone());
  const inverseNormalMatrix = new THREE.Matrix3().getNormalMatrix(root.matrixWorld).invert();
  const localNormal = normal.clone().applyMatrix3(inverseNormalMatrix).normalize();

  return {
    model_id:modelId,
    point:{x:hit.point.x * 10,y:hit.point.y * 10,z:hit.point.z * 10},
    normal:{x:normal.x,y:normal.y,z:normal.z},
    local_point:{x:localPoint.x,y:localPoint.y,z:localPoint.z},
    local_normal:{x:localNormal.x,y:localNormal.y,z:localNormal.z}
  };
}

function nextEngineeringName(prefix, collection) {
  const n = collection.length + 1;
  return prefix + '-' + String(n).padStart(2,'0');
}

function placeEngineeringEntity(tool, event) {
  const anchor = pickEngineeringAnchor(event);
  if (!anchor) {
    toast('Click a surface on an imported CAD/model.');
    return;
  }

  const beforeHistory = captureDesignState();

  if (tool === 'equipment') {
    const item = {
      id:id('equipment'),
      name:nextEngineeringName('Motor', state.equipment),
      type:'Motor',
      power_kw:0,
      current_a:0,
      voltage_v:400,
      cable_name:'4C x 6 mm²',
      cable_diameter_mm:16,
      destination_panel_id: state.panels.length ? state.panels[0].id : '',
      anchor:anchor,
      routing_result:null
    };
    state.equipment.push(item);
    state.selected = item.id;
    recordHistory(beforeHistory);
    setTool('select');
    rebuildEngineeringMarkers();
    render();
    toast(item.name + ' placed on surface');
    return;
  }

  const panel = {
    id:id('panel'),
    name:nextEngineeringName('MCC', state.panels),
    type:'MCC',
    anchor:anchor
  };
  state.panels.push(panel);
  state.selected = panel.id;
  recordHistory(beforeHistory);
  setTool('select');
  rebuildEngineeringMarkers();
  render();
  toast(panel.name + ' placed on surface');
}

function parseNumberList(value) {
  return String(value || '').split(',').map(function(v){ return Number(v.trim()); }).filter(function(v){ return Number.isFinite(v) && v > 0; });
}

function prepareEngineeringCollisionGeometry(root) {
  if (!root) return;
  root.traverse(function(node){
    if (!node.isMesh || !node.geometry) return;
    if (!node.geometry.boundsTree) node.geometry.computeBoundsTree();
    node.raycast = acceleratedRaycast;
  });
}

function rebuildEngineeringCollisionMeshCache() {
  const entries = [];

  state.modelRoots.forEach(function(root){
    root.updateMatrixWorld(true);

    root.traverse(function(node){
      if (
        !node.isMesh ||
        !node.geometry ||
        !node.geometry.boundsTree ||
        node.visible === false
      ) return;

      const worldBox = new THREE.Box3().setFromObject(node);
      if (worldBox.isEmpty()) return;

      entries.push({
        node:node,
        worldBox:worldBox,
        inverseWorld:new THREE.Matrix4().copy(node.matrixWorld).invert()
      });
    });
  });

  engineeringCollisionMeshCache = entries;
  return entries;
}

function disposeEngineeringCollisionGeometry(root) {
  engineeringCollisionMeshCache = [];
  if (!root) return;
  root.traverse(function(node){
    if (!node.isMesh || !node.geometry || !node.geometry.boundsTree) return;
    node.geometry.disposeBoundsTree();
  });
}

function engineeringPointToScene(point) {
  return new THREE.Vector3(
    Number(point.x) / 10,
    Number(point.y) / 10,
    Number(point.z) / 10
  );
}

function engineeringSegmentHitsModel(a, b, context) {
  const start = engineeringPointToScene(a);
  const end = engineeringPointToScene(b);
  const delta = end.clone().sub(start);
  const length = delta.length();
  if (length < 1e-9) return false;

  const width = Math.max(1, Number(context && context.trayWidthMm) || 100);
  const height = Math.max(1, Number(context && context.trayHeightMm) || 100);
  const clearance = Math.max(0, Number(context && context.bodyClearanceMm) || 0);

  const direction = delta.clone().multiplyScalar(1 / length);
  const requestedStartIgnoreMm = Math.max(
    0,
    Number(context && context.ignoreStartMm) || 0
  );
  const startInsetScene = Math.min(
    requestedStartIgnoreMm / 10,
    length * 0.45
  );
  const endInsetScene = Math.min(0.1, length * 0.05);
  const aSafe = start.clone().add(direction.clone().multiplyScalar(startInsetScene));
  const bSafe = end.clone().add(direction.clone().multiplyScalar(-endInsetScene));
  if (aSafe.distanceTo(bSafe) <= 1e-9) return false;

  const halfWidthScene = (width / 2 + clearance) / 10;
  const halfHeightScene = (height / 2 + clearance) / 10;

  const min = new THREE.Vector3(
    Math.min(aSafe.x, bSafe.x),
    Math.min(aSafe.y, bSafe.y),
    Math.min(aSafe.z, bSafe.z)
  );
  const max = new THREE.Vector3(
    Math.max(aSafe.x, bSafe.x),
    Math.max(aSafe.y, bSafe.y),
    Math.max(aSafe.z, bSafe.z)
  );

  const dx = Math.abs(delta.x);
  const dy = Math.abs(delta.y);
  const dz = Math.abs(delta.z);
  if (dx >= dy && dx >= dz) {
    min.y -= halfHeightScene; max.y += halfHeightScene;
    min.z -= halfWidthScene;  max.z += halfWidthScene;
  } else if (dy >= dx && dy >= dz) {
    min.x -= halfWidthScene;  max.x += halfWidthScene;
    min.z -= halfHeightScene; max.z += halfHeightScene;
  } else {
    min.x -= halfWidthScene;  max.x += halfWidthScene;
    min.y -= halfHeightScene; max.y += halfHeightScene;
  }

  const volume = new THREE.Box3(min, max);
  const meshEntries = engineeringCollisionMeshCache.length
    ? engineeringCollisionMeshCache
    : rebuildEngineeringCollisionMeshCache();

  if (!meshEntries.length) return false;

  for (let i = 0; i < meshEntries.length; i++) {
    const entry = meshEntries[i];

    // World-space AABB is a broadphase filter only. Passing this test does
    // not prove a collision; the exact BVH intersection below remains the
    // final authority.
    if (!entry.worldBox.intersectsBox(volume)) continue;

    if (entry.node.geometry.boundsTree.intersectsBox(
      volume,
      entry.inverseWorld
    )) {
      return true;
    }
  }

  return false;
}

function engineeringSegmentClear(a, b, context) {
  const width = Math.max(1, Number(context && context.trayWidthMm) || 100);
  const height = Math.max(1, Number(context && context.trayHeightMm) || 100);
  const clearance = Math.max(0, Number(context && context.bodyClearanceMm) || 0);
  const ignoreStartMm = Math.max(0, Number(context && context.ignoreStartMm) || 0);

  const dx = Math.abs(Number(b.x) - Number(a.x));
  const dy = Math.abs(Number(b.y) - Number(a.y));
  const dz = Math.abs(Number(b.z) - Number(a.z));
  if (dx < 0.001 && dy < 0.001 && dz < 0.001) return true;

  const key = [
    Math.round(Number(a.x)), Math.round(Number(a.y)), Math.round(Number(a.z)),
    Math.round(Number(b.x)), Math.round(Number(b.y)), Math.round(Number(b.z)),
    Math.round(width), Math.round(height), Math.round(clearance),
    Math.round(ignoreStartMm)
  ].join('|');

  if (engineeringCollisionCache.has(key)) return engineeringCollisionCache.get(key);

  const clear = !engineeringSegmentHitsModel(a, b, {
    trayWidthMm:width,
    trayHeightMm:height,
    bodyClearanceMm:clearance,
    ignoreStartMm
  });
  engineeringCollisionCache.set(key, clear);
  return clear;
}

function engineeringStandoffClear(base, point, context) {
  const width = Math.max(1, Number(context && context.trayWidthMm) || 100);
  const height = Math.max(1, Number(context && context.trayHeightMm) || 100);
  const clearance = Math.max(0, Number(context && context.bodyClearanceMm) || 0);
  const maxBodyDistanceMm = Number(context && context.maxBodyDistanceMm) || 0;

  // The standoff is only the short transition from the equipment/panel
  // attachment surface to the first routing point. Its straight vector may
  // be diagonal, while the actual tray routing is orthogonal. Treating this
  // diagonal transition as an axis-aligned tray volume creates false
  // collisions at curved/angled host surfaces and prevents routing entirely.
  //
  // The routing point itself is still subject to the exact body-distance
  // limits. Once the route starts from this validated point, every generated
  // orthogonal tray segment continues through the normal exact collision
  // checker.
  return engineeringPointBodyDistanceClear(point, {
    trayWidthMm:width,
    trayHeightMm:height,
    bodyClearanceMm:clearance,
    maxBodyDistanceMm:maxBodyDistanceMm
  });
}

function engineeringPointBodyDistanceClear(point, context) {
  const width = Math.max(1, Number(context && context.trayWidthMm) || 100);
  const height = Math.max(1, Number(context && context.trayHeightMm) || 100);
  const clearance = Math.max(0, Number(context && context.bodyClearanceMm) || 0);
  const maxDistance = Number(context && context.maxBodyDistanceMm);

  const key = [
    Math.round(Number(point.x) / 25),
    Math.round(Number(point.y) / 25),
    Math.round(Number(point.z) / 25),
    Math.round(width),
    Math.round(height),
    Math.round(clearance),
    Number.isFinite(maxDistance) ? Math.round(maxDistance) : 0
  ].join('|');

  if (engineeringBodyDistanceCache.has(key)) {
    return engineeringBodyDistanceCache.get(key);
  }

  const query = engineeringPointToScene(point);
  const meshEntries = engineeringCollisionMeshCache.length
    ? engineeringCollisionMeshCache
    : rebuildEngineeringCollisionMeshCache();

  let bodyDistance = Infinity;

  for (let i = 0; i < meshEntries.length; i++) {
    const entry = meshEntries[i];

    // The world-space box distance is a mathematically safe lower bound for
    // the true mesh distance. It can therefore skip BVH closest-point work
    // once another mesh is already known to be closer.
    const lowerBoundScene = entry.worldBox.distanceToPoint(query);
    if (lowerBoundScene * 10 > bodyDistance) continue;

    const localPoint = query.clone().applyMatrix4(entry.inverseWorld);
    const hit = entry.node.geometry.boundsTree.closestPointToPoint(localPoint);
    if (!hit) continue;

    let distanceScene = Number(hit.distance);
    if (hit.point) {
      const worldClosest = hit.point.clone().applyMatrix4(entry.node.matrixWorld);
      distanceScene = query.distanceTo(worldClosest);
    }

    if (Number.isFinite(distanceScene)) {
      bodyDistance = Math.min(bodyDistance, distanceScene * 10);
    }
  }

  let allowed = true;
  if (Number.isFinite(bodyDistance)) {
    const halfExtent = Math.max(width, height) / 2;
    const minCenterlineDistance = clearance + halfExtent;
    const maxCenterlineDistance = Number.isFinite(maxDistance) && maxDistance > 0
      ? maxDistance + halfExtent
      : Infinity;
    allowed =
      bodyDistance >= minCenterlineDistance - 0.5 &&
      bodyDistance <= maxCenterlineDistance + 0.5;
  }

  engineeringBodyDistanceCache.set(key, allowed);
  return allowed;
}


function collectRoutingObstacles(clearanceMm) {
  const obstacles = [];
  const unique = new Set();

  state.modelRoots.forEach(function(root){
    root.updateMatrixWorld(true);
    root.traverse(function(node){
      if (!node.isMesh || !node.geometry || node.visible === false) return;

      const box = new THREE.Box3().setFromObject(node);
      if (box.isEmpty()) return;

      const minX = box.min.x * 10;
      const maxX = box.max.x * 10;
      const minY = box.min.y * 10;
      const maxY = box.max.y * 10;
      const minZ = box.min.z * 10;
      const maxZ = box.max.z * 10;

      if (
        (maxX - minX) < 1 ||
        (maxY - minY) < 1 ||
        (maxZ - minZ) < 1
      ) return;

      const key = [
        Math.round(minX * 10),
        Math.round(maxX * 10),
        Math.round(minY * 10),
        Math.round(maxY * 10),
        Math.round(minZ * 10),
        Math.round(maxZ * 10)
      ].join('|');

      if (unique.has(key)) return;
      unique.add(key);
      obstacles.push({minX,maxX,minY,maxY,minZ,maxZ});
    });
  });

  obstacles.sort(function(a,b){
    return (
      ((b.maxX-b.minX)*(b.maxY-b.minY)*(b.maxZ-b.minZ)) -
      ((a.maxX-a.minX)*(a.maxY-a.minY)*(a.maxZ-a.minZ))
    );
  });

  const reduced = [];
  obstacles.forEach(function(candidate){
    const contained = reduced.some(function(existing){
      return candidate.minX >= existing.minX &&
        candidate.maxX <= existing.maxX &&
        candidate.minY >= existing.minY &&
        candidate.maxY <= existing.maxY &&
        candidate.minZ >= existing.minZ &&
        candidate.maxZ <= existing.maxZ;
    });

    if (!contained) reduced.push(candidate);
    if (reduced.length >= 500) return;
  });

  return reduced;
}

function removeEngineeringGeneratedRoutes() {
  state.objects = state.objects.filter(function(o){ return !o.engineering_generated; });
}

function getEngineeringModelMaxY() {
  let maxY = -Infinity;
  state.modelRoots.forEach(function(root){
    root.updateMatrixWorld(true);
    root.traverse(function(node){
      if (!node.isMesh || !node.geometry || node.visible === false) return;
      const box = new THREE.Box3().setFromObject(node);
      if (!box.isEmpty()) maxY = Math.max(maxY, box.max.y * 10);
    });
  });
  if (!Number.isFinite(maxY)) {
    const anchors = state.equipment.concat(state.panels)
      .map(function(entity){ return entity.anchor && entity.anchor.point ? Number(entity.anchor.point.y) : -Infinity; });
    maxY = Math.max.apply(null, anchors.concat([0]));
  }
  return maxY;
}

function getEngineeringRoutingBounds() {
  const bounds = new THREE.Box3();
  let hasGeometry = false;

  state.modelRoots.forEach(function(root){
    root.updateMatrixWorld(true);
    root.traverse(function(node){
      if (!node.isMesh || !node.geometry || node.visible === false) return;
      const box = new THREE.Box3().setFromObject(node);
      if (box.isEmpty()) return;
      bounds.union(box);
      hasGeometry = true;
    });
  });

  if (!hasGeometry || bounds.isEmpty()) return null;

  return {
    minX:bounds.min.x * 10,
    maxX:bounds.max.x * 10,
    minY:bounds.min.y * 10,
    maxY:bounds.max.y * 10,
    minZ:bounds.min.z * 10,
    maxZ:bounds.max.z * 10
  };
}

let engineeringOutsideRoutingPrompt = null;

function askEngineeringOutsideRouting(message) {
  if (engineeringOutsideRoutingPrompt) return engineeringOutsideRoutingPrompt;

  engineeringOutsideRoutingPrompt = new Promise(function(resolve) {
    const overlay = document.createElement('div');
    overlay.id = 'engineeringOutsideRoutingDialog';
    overlay.style.position = 'fixed';
    overlay.style.inset = '0';
    overlay.style.zIndex = '10000';
    overlay.style.display = 'flex';
    overlay.style.alignItems = 'center';
    overlay.style.justifyContent = 'center';
    overlay.style.padding = '24px';
    overlay.style.background = 'rgba(0,0,0,0.62)';

    const panel = document.createElement('div');
    panel.style.width = 'min(560px, calc(100vw - 48px))';
    panel.style.boxSizing = 'border-box';
    panel.style.padding = '22px';
    panel.style.borderRadius = '12px';
    panel.style.border = '1px solid rgba(255,255,255,0.16)';
    panel.style.background = '#111820';
    panel.style.color = '#f4f7fa';
    panel.style.boxShadow = '0 20px 60px rgba(0,0,0,0.45)';
    panel.style.fontFamily = 'inherit';

    const text = document.createElement('div');
    text.textContent = message;
    text.style.whiteSpace = 'pre-line';
    text.style.lineHeight = '1.55';
    text.style.fontSize = '14px';
    panel.appendChild(text);

    const actions = document.createElement('div');
    actions.style.display = 'flex';
    actions.style.justifyContent = 'flex-end';
    actions.style.gap = '10px';
    actions.style.marginTop = '18px';

    const noButton = document.createElement('button');
    noButton.type = 'button';
    noButton.textContent = 'No';
    noButton.style.minWidth = '88px';

    const yesButton = document.createElement('button');
    yesButton.type = 'button';
    yesButton.textContent = 'Yes';
    yesButton.style.minWidth = '88px';

    function finish(value) {
      document.removeEventListener('keydown', onKeyDown);
      if (overlay.parentNode) overlay.parentNode.removeChild(overlay);
      engineeringOutsideRoutingPrompt = null;
      resolve(value);
    }

    function onKeyDown(event) {
      if (event.key === 'Escape') {
        finish(false);
      } else if (event.key === 'Enter') {
        finish(true);
      }
    }

    noButton.addEventListener('click', function(){ finish(false); });
    yesButton.addEventListener('click', function(){ finish(true); });
    overlay.addEventListener('click', function(event){
      if (event.target === overlay) finish(false);
    });
    document.addEventListener('keydown', onKeyDown);

    actions.appendChild(noButton);
    actions.appendChild(yesButton);
    panel.appendChild(actions);
    overlay.appendChild(panel);
    document.body.appendChild(overlay);
    yesButton.focus();
  });

  return engineeringOutsideRoutingPrompt;
}

async function runEngineeringAutoDesign() {
  if (!state.equipment.length) {
    toast('Place at least one equipment/load first');
    return;
  }
  if (!state.panels.length) {
    toast('Place at least one electrical panel first');
    return;
  }

  syncEngineeringAnchors();
  const missing = state.equipment.filter(function(item){ return !item.destination_panel_id || !state.panels.some(function(panel){ return panel.id === item.destination_panel_id; }); });
  if (missing.length) {
    toast(missing[0].name + ' has no valid destination panel');
    state.selected = missing[0].id;
    render();
    return;
  }

  const routingBounds = getEngineeringRoutingBounds();
  const settings = {
    gridStepMm:Number($('routingGridStep').value) || 100,
    clearanceMm:Number.isFinite(Number($('autoTrayClearance').value)) ? Number($('autoTrayClearance').value) : 100,
    maxBodyDistanceMm:Number.isFinite(Number($('autoTrayMaxDistance').value))
      ? Number($('autoTrayMaxDistance').value)
      : 1500,
    ceilingY:routingBounds ? routingBounds.maxY : getEngineeringModelMaxY(),
    ceilingSafetyGapMm:50,
    routingBounds:routingBounds,
    fillLimitPercent:Number($('fillLimit').value) || 80,
    mainMinCables:Number($('mainTrayMinCables').value) || 2,
    trayHeightMm:Number($('defaultTrayHeight').value) || 100,
    mainCorridorTurnPenaltyRatio:100,
    turnPenaltyRatio:20,
    preferredYPenaltyRatio:3,
    structurePrimaryAxis:routingBounds
      ? (Math.abs(routingBounds.maxX - routingBounds.minX) >= Math.abs(routingBounds.maxZ - routingBounds.minZ) ? 'x' : 'z')
      : null,
    structureAxisPenaltyRatio:0.35,
    routingPaddingMm:Math.max(
      1000,
      (Number.isFinite(Number($('autoTrayMaxDistance').value)) ? Number($('autoTrayMaxDistance').value) : 1500) +
      2 * (Number($('routingGridStep').value) || 100)
    ),
    traySideMarginMm:25,
    standardTrayWidthsMm:parseNumberList($('autoTrayStandards').value),
    floorZones:state.engineeringFloors.map(function(floor){ return JSON.parse(JSON.stringify(floor)); }),
    floorLevelToleranceMm:500,
    verticalPenaltyRatio:0.02,
    verticalRangePenaltyRatio:0.25,
    reuseBonus:0.45,
    maxGridCells:120000
  };

  engineeringCollisionCache = new Map();
  engineeringBodyDistanceCache = new Map();
  engineeringCollisionMeshCache = [];
  state.modelRoots.forEach(function(root){ root.updateMatrixWorld(true); });
  rebuildEngineeringCollisionMeshCache();

  const obstacles = collectRoutingObstacles(settings.clearanceMm);
  let routingOptions = {
    ...settings,
    exactCollisionRouting:true,
    segmentClear:engineeringSegmentClear,
    standoffClear:engineeringStandoffClear,
    pointBodyDistanceClear:engineeringPointBodyDistanceClear,
    allowOutsideRouting:false
  };

  let report=routeEngineeringNetwork({
    equipment:state.equipment,
    panels:state.panels,
    obstacles:obstacles,
    options:routingOptions
  });

  const routedEquipmentIds=new Set(report.cable_plans.map(function(plan){ return plan.equipment.id; }));
  const missingInternalRoutes=state.equipment.filter(function(item){ return !routedEquipmentIds.has(item.id); });

  if(missingInternalRoutes.length){
    const names=missingInternalRoutes.slice(0,3).map(function(item){ return item.name; }).join(', ');
    const more=missingInternalRoutes.length>3?' ...':'';
    const allowOutside=await askEngineeringOutsideRouting(
      'No completely internal tray route could be found for: '+names+more+
      '.\\n\\nDo you want to allow the tray route to leave the model envelope?'
    );

    if(!allowOutside){
      state.lastEngineeringReport=report;
      status('Engineering routing stopped: internal route required');
      toast('No internal route was found. No outside route was created.');
      return;
    }

    routingOptions={...routingOptions,routingBounds:null,allowOutsideRouting:true};
    report=routeEngineeringNetwork({
      equipment:state.equipment,
      panels:state.panels,
      obstacles:obstacles,
      options:routingOptions
    });

    const reroutedIds=new Set(report.cable_plans.map(function(plan){ return plan.equipment.id; }));
    const stillMissing=state.equipment.filter(function(item){ return !reroutedIds.has(item.id); });
    if(stillMissing.length){
      state.lastEngineeringReport=report;
      status('Engineering routing completed with unresolved routes');
      toast('No valid route was found for all equipment, even with outside routing allowed.');
      return;
    }
  }

  const beforeHistory = captureDesignState();
  removeEngineeringGeneratedRoutes();

  const panelById = new Map(state.panels.map(function(panel){ return [panel.id,panel]; }));
  const equipmentById = new Map(state.equipment.map(function(item){ return [item.id,item]; }));

  // Build the exact shared-tray segment map once. This includes the common
  // panel connection tray as well as the long Main Tray. Dedicated motor
  // branches are excluded because their tray run normally carries one cable.
  function routeSegmentKey(a, b) {
    const start = {
      x:Number(a.x).toFixed(3),
      y:Number(a.y).toFixed(3),
      z:Number(a.z).toFixed(3)
    };
    const end = {
      x:Number(b.x).toFixed(3),
      y:Number(b.y).toFixed(3),
      z:Number(b.z).toFixed(3)
    };
    const axis = (
      Math.abs(Number(b.x) - Number(a.x)) >= Math.abs(Number(b.z) - Number(a.z)) &&
      Math.abs(Number(b.x) - Number(a.x)) >= Math.abs(Number(b.y) - Number(a.y))
    ) ? 'x' : (
      Math.abs(Number(b.y) - Number(a.y)) >= Math.abs(Number(b.z) - Number(a.z))
        ? 'y'
        : 'z'
    );

    if (axis === 'x') {
      return [
        'x',
        start.y,
        start.z,
        Math.min(Number(a.x), Number(b.x)).toFixed(3),
        Math.max(Number(a.x), Number(b.x)).toFixed(3)
      ].join('|');
    }
    if (axis === 'y') {
      return [
        'y',
        start.x,
        start.z,
        Math.min(Number(a.y), Number(b.y)).toFixed(3),
        Math.max(Number(a.y), Number(b.y)).toFixed(3)
      ].join('|');
    }
    return [
      'z',
      start.x,
      start.y,
      Math.min(Number(a.z), Number(b.z)).toFixed(3),
      Math.max(Number(a.z), Number(b.z)).toFixed(3)
    ].join('|');
  }

  const sharedSegmentKeysByCable = new Map();
  report.tray_runs.forEach(function(run){
    if (!Array.isArray(run.cable_ids) || run.cable_ids.length < 2) return;
    if (!Array.isArray(run.points) || run.points.length < 2) return;

    const keys = [];
    for (let i = 1; i < run.points.length; i++) {
      const start = run.points[i - 1];
      const end = run.points[i];
      keys.push(routeSegmentKey(
        {
          x:mmToScene(start.x),
          y:mmToScene(start.y),
          z:mmToScene(start.z)
        },
        {
          x:mmToScene(end.x),
          y:mmToScene(end.y),
          z:mmToScene(end.z)
        }
      ));
    }

    run.cable_ids.forEach(function(cableId){
      const set = sharedSegmentKeysByCable.get(cableId) || new Set();
      keys.forEach(function(key){ set.add(key); });
      sharedSegmentKeysByCable.set(cableId, set);
    });
  });

  report.cable_plans.forEach(function(plan, index){
    const item = equipmentById.get(plan.equipment.id);
    const panel = panelById.get(plan.panel.id);
    if (!item || !panel) return;
    createRoute('cable', plan.points, {
      pointsAreMm:true,
      name:item.name + ' - ' + (item.cable_name || plan.cable.name),
      diameter_mm:Number(item.cable_diameter_mm) || plan.cable.diameter_mm,
      specification:item.cable_name || plan.cable.name,
      material:'Copper/PVC',
      engineering_generated:true,
      engineering_equipment_id:item.id,
      engineering_panel_id:panel.id,
      engineering_network_id:plan.engineering_network_id
        ? String(plan.engineering_network_id)
        : 'network-' + panel.id,
      engineering_floor_id:plan.engineering_floor_id ? String(plan.engineering_floor_id) : null,
      engineering_floor_name:plan.engineering_floor_name || null,
      engineering_tray_width_mm:Number(plan.planning_tray_width_mm) || null,
      engineering_main_corridor_axis:plan.main_corridor_axis || settings.structurePrimaryAxis || null,
      engineering_main_corridor_y_mm:Number(plan.main_corridor_routing_y_mm),
      engineering_main_corridor_secondary_coordinate_mm:Number(plan.main_corridor_secondary_coordinate_mm),
      engineering_shared_segment_keys:Array.from(
        sharedSegmentKeysByCable.get(item.id) || []
      )
    });
  });

  report.tray_runs.forEach(function(run, index){
    const label = run.classification === 'main' ? 'Main Tray' : 'Branch Tray';
    const matchingPlan = report.cable_plans.find(function(plan){
      return run.cable_ids && run.cable_ids.indexOf(plan.equipment.id) >= 0;
    });
    const matchingPanel = matchingPlan ? panelById.get(matchingPlan.panel.id) : null;

    createRoute('tray', run.points, {
      pointsAreMm:true,
      name:label + ' ' + run.width_mm + 'x' + run.height_mm + ' #' + (index + 1),
      width_mm:run.width_mm,
      height_mm:run.height_mm,
      specification:run.width_mm + 'x' + run.height_mm + ' TRAY - ' + run.classification.toUpperCase(),
      material:'Galvanized Steel',
      engineering_generated:true,
      engineering_classification:run.classification,
      engineering_cable_ids:run.cable_ids || [],
      engineering_network_id:run.network_id
        ? String(run.network_id)
        : (matchingPlan
          ? (
            matchingPlan.engineering_network_id
              ? String(matchingPlan.engineering_network_id)
              : 'network-' + matchingPlan.panel.id
          )
          : null),
      engineering_floor_id:matchingPlan && matchingPlan.engineering_floor_id
        ? String(matchingPlan.engineering_floor_id)
        : null,
      engineering_floor_name:matchingPlan ? (matchingPlan.engineering_floor_name || null) : null,
      panel_connection:!!run.panel_connection
    });
  });

  const resultByEquipment = new Map(report.equipment_results.map(function(result){ return [result.equipment_id,result]; }));
  state.equipment.forEach(function(item){
    const result = resultByEquipment.get(item.id);
    item.routing_result = result ? { ...result } : null;
  });

  state.engineeringSettings = {
    ...state.engineeringSettings,
    gridStepMm:settings.gridStepMm,
    clearanceMm:settings.clearanceMm,
    maxBodyDistanceMm:settings.maxBodyDistanceMm,
    namesVisible:state.engineeringSettings.namesVisible !== false,
    mainMinCables:settings.mainMinCables,
    traySideMarginMm:settings.traySideMarginMm,
    standardTrayWidthsMm:settings.standardTrayWidthsMm
  };
  state.lastEngineeringReport = report;

  recordHistory(beforeHistory);
  state.selected = null;
  rebuildRoutes();
  rebuildEngineeringMarkers();
  render();
  status('Engineering routing completed');
  const warningText = report.warnings.length || report.cable_plans.some(function(plan){ return plan.warning; })
    ? ' Some routes used warnings/fallbacks.'
    : '';
  toast(
    report.cable_plans.length + ' cable route(s), ' +
    report.tray_runs.length + ' tray run(s) generated.' + warningText
  );
}

function renderEngineeringTakeoff() {
  const box = $('engineeringTakeoff');
  if (!box) return;

  const cableRoutes = state.objects.filter(function(o){ return o.kind === 'cable' && o.engineering_generated; });
  const trayRoutes = state.objects.filter(function(o){ return o.kind === 'tray' && o.engineering_generated; });

  if (!state.equipment.length && !state.panels.length) {
    box.innerHTML = '<div class="hint">Place equipment and electrical panels on CAD/model surfaces to build an automatic routing design.</div>';
    return;
  }

  const panelMap = new Map(state.panels.map(function(panel){ return [panel.id,panel]; }));
  const cableRows = cableRoutes.map(function(route){
    const item = state.equipment.find(function(eq){ return eq.id === route.engineering_equipment_id; });
    const panel = route.engineering_panel_id ? panelMap.get(route.engineering_panel_id) : null;
    return '<tr><td>' + esc(item ? item.name : route.name) + '</td><td>' +
      esc(item ? item.cable_name : route.specification) + '</td><td>' +
      esc(panel ? panel.name : '-') + '</td><td>' +
      lengthOf(route.points).toFixed(2) + ' m</td></tr>';
  }).join('');

  const trayGrouped = new Map();
  trayRoutes.forEach(function(route){
    const key = (route.engineering_classification || 'branch') + '|' + route.width_mm + '|' + route.height_mm;
    const entry = trayGrouped.get(key) || {
      classification:route.engineering_classification || 'branch',
      width_mm:route.width_mm,
      height_mm:route.height_mm,
      length_m:0
    };
    entry.length_m += lengthOf(route.points);
    trayGrouped.set(key,entry);
  });

  const trayRows = Array.from(trayGrouped.values()).sort(function(a,b){
    return a.classification.localeCompare(b.classification) || a.width_mm-b.width_mm || a.height_mm-b.height_mm;
  }).map(function(row){
    return '<tr><td>' + esc(row.classification === 'main' ? 'Main' : 'Branch') + '</td><td>' +
      esc(row.width_mm + ' × ' + row.height_mm + ' mm') + '</td><td>' +
      row.length_m.toFixed(2) + ' m</td></tr>';
  }).join('');

  const totalCable = cableRoutes.reduce(function(sum,route){ return sum + lengthOf(route.points); },0);
  const totalTray = trayRoutes.reduce(function(sum,route){ return sum + lengthOf(route.points); },0);
  const missingCount = state.equipment.filter(function(item){ return !item.destination_panel_id || !panelMap.has(item.destination_panel_id); }).length;

  box.innerHTML =
    '<div class="boq-summary">' +
      '<div><span>Loads</span><b>' + state.equipment.length + '</b></div>' +
      '<div><span>Panels</span><b>' + state.panels.length + '</b></div>' +
      '<div><span>Auto Cable</span><b>' + totalCable.toFixed(2) + ' m</b></div>' +
      '<div><span>Auto Tray</span><b>' + totalTray.toFixed(2) + ' m</b></div>' +
    '</div>' +
    (missingCount ? '<div class="property-hint">Unassigned loads: ' + missingCount + '</div>' : '') +
    (cableRows ? '<div class="property-group-title">Cable Schedule</div><div class="table-wrap"><table><thead><tr><th>Load</th><th>Cable</th><th>Panel</th><th>Length</th></tr></thead><tbody>' + cableRows + '</tbody></table></div>' : '') +
    (trayRows ? '<div class="property-group-title">Tray Schedule</div><div class="table-wrap"><table><thead><tr><th>Class</th><th>Size</th><th>Length</th></tr></thead><tbody>' + trayRows + '</tbody></table></div>' : '') +
    (!cableRows && !trayRows ? '<div class="hint">Press Auto Design Routing after assigning every load to a panel.</div>' : '');
}

function setTool(tool) {
  state.surfacePickMode = false;
  state.surfacePick = null;
  state.surfaceAlignStart = null;
  state.autoRoutePoints = [];
  autoRoutePreviewRoot.clear();
  state.tool = tool;
  state.drawing = (tool === 'cable' || tool === 'tray') ? { type: tool, points: [] } : null;
  state.measureStart = null;
  surfaceSelectionRoot.clear();
  document.querySelectorAll('.tool').forEach(function(b){ b.classList.toggle('active', b.dataset.tool === tool); });
  const autoRouteTypeRow = $('autoRouteTypeRow');
  const autoRouteOffsetAxisRow = $('autoRouteOffsetAxisRow');
  const autoRouteActions = $('autoRouteActions');
  const measureTypeRow = $('measureTypeRow');
  const engineeringTool = tool === 'equipment' || tool === 'panel' || tool === 'floor';
  if (autoRouteTypeRow) autoRouteTypeRow.classList.toggle('hidden', tool !== 'auto-route');
  if (autoRouteOffsetAxisRow) {
    autoRouteOffsetAxisRow.classList.toggle(
      'hidden',
      tool !== 'auto-route' || $('autoRouteType').value !== 'tray'
    );
  }
  if (measureTypeRow) measureTypeRow.classList.toggle('hidden', tool !== 'measure');
  if (autoRouteActions) autoRouteActions.classList.add('hidden');
  const hint = {
    select: 'Click a route to select it. Drag a selected tray or cable to move it in 3D; edit Position/Slope in Properties.',
    cable: 'Click route points. Press Enter to finish.',
    tray: 'Click route points. Press Enter to finish.',
    model: 'Use Load Model for 3D, SolidWorks, AutoCAD DWG or DXF files.',
    equipment: 'Click a CAD/model surface to place a motor, pump, fan or other electrical load.',
    panel: 'Click a CAD/model surface to place an electrical panel destination.',
    floor: 'Click four corner points around one floor in order. The fourth point saves the floor definition.',
    measure: state.measureMode === 'surface'
      ? 'Surface to Surface: click two CAD/model surfaces to measure the shortest surface distance.'
      : state.measureMode === 'edge'
        ? 'Edge Length: click directly on a visible model edge to measure its length.'
        : 'Point to Point: click two points on project geometry to measure the direct 3D distance.',
    'auto-route': 'Select Cable or Tray, click as many route points as needed, then press Enter to create the route.'
  };
  $('toolHint').textContent = hint[tool] || '';
  $('routeOverlay').classList.toggle('hidden', tool !== 'cable' && tool !== 'tray');
  status(
    tool === 'cable' || tool === 'tray'
      ? 'Drawing ' + tool + ' route'
      : tool === 'auto-route'
        ? 'Auto Route: select start point'
        : tool === 'equipment'
          ? 'Place Equipment: select a CAD/model surface'
          : tool === 'panel'
            ? 'Place Panel: select a CAD/model surface'
            : tool === 'floor'
              ? 'Define Floor: select corner 1 of 4'
              : 'Ready'
  );
}
document.querySelectorAll('.tool').forEach(function(b){ b.addEventListener('click', function(){ setTool(b.dataset.tool); }); });
$('autoRouteType').addEventListener('change', function(){
  const row = $('autoRouteOffsetAxisRow');
  if (row) row.classList.toggle('hidden', state.tool !== 'auto-route' || this.value !== 'tray');
});
$('measureType').addEventListener('change', function(){
  state.measureMode = this.value === 'surface'
    ? 'surface'
    : (this.value === 'edge' ? 'edge' : 'point');
  state.measureStart = null;
  clearSurfaceSelectionVisuals();
  status(
    state.measureMode === 'surface'
      ? 'Measure: select first surface'
      : (state.measureMode === 'edge' ? 'Measure: select an edge' : 'Measure: select first point')
  );
  toast(
    state.measureMode === 'surface'
      ? 'Measurement mode: Surface to Surface'
      : (state.measureMode === 'edge' ? 'Measurement mode: Edge Length' : 'Measurement mode: Point to Point')
  );
});
$('projectName').addEventListener('input', function(e){ state.project.name = e.target.value; });
$('unitSystem').addEventListener('change', function(e){ state.project.units = e.target.value; });
['routingGridStep','autoTrayClearance','autoTrayMaxDistance','mainTrayMinCables','autoTrayStandards'].forEach(function(idValue){
  const field = $(idValue);
  if (!field) return;
  field.addEventListener('change', function(){
    state.engineeringSettings = {
      ...state.engineeringSettings,
      gridStepMm:Number($('routingGridStep').value) || 100,
      clearanceMm:Number.isFinite(Number($('autoTrayClearance').value)) ? Number($('autoTrayClearance').value) : 100,
      maxBodyDistanceMm:Number.isFinite(Number($('autoTrayMaxDistance').value)) ? Number($('autoTrayMaxDistance').value) : 1500,
      mainMinCables:Number($('mainTrayMinCables').value) || 2,
      standardTrayWidthsMm:parseNumberList($('autoTrayStandards').value)
    };
  });
});
$('autoDesignBtn').addEventListener('click', runEngineeringAutoDesign);
$('exportEngineeringBoqBtn').addEventListener('click', exportEngineeringBoq);

function pointerRay(event) {
  const r = renderer.domElement.getBoundingClientRect();
  mouse.x = ((event.clientX - r.left) / r.width) * 2 - 1;
  mouse.y = -((event.clientY - r.top) / r.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);
}

function zoomCameraToCursor(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  if (rect.width <= 0 || rect.height <= 0) return;

  let delta = Number(event.deltaY) || 0;
  if (event.deltaMode === 1) delta *= 16;
  else if (event.deltaMode === 2) delta *= rect.height;
  if (Math.abs(delta) < 0.001) return;

  // Own the wheel event completely. This avoids fighting OrbitControls'
  // target math and keeps zoom centered on the actual mouse location.
  event.preventDefault();
  event.stopImmediatePropagation();

  const cursor = new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1
  );

  const cursorDirection = new THREE.Vector3(cursor.x, cursor.y, 0.5)
    .unproject(camera)
    .sub(camera.position)
    .normalize();
  const viewDirection = camera.getWorldDirection(new THREE.Vector3()).normalize();

  const targetOffset = controls.target.clone().sub(camera.position);
  const targetDepth = targetOffset.dot(viewDirection);
  const rayDepth = cursorDirection.dot(viewDirection);

  let focusDistance = rayDepth > 1e-6
    ? targetDepth / rayDepth
    : targetOffset.length();

  if (!Number.isFinite(focusDistance) || focusDistance <= 0) {
    focusDistance = targetOffset.length();
  }

  const focusPoint = camera.position.clone()
    .add(cursorDirection.multiplyScalar(focusDistance));

  const currentDistance = camera.position.distanceTo(controls.target);
  if (!Number.isFinite(currentDistance) || currentDistance <= 1e-9) return;

  let scale = Math.exp(delta * 0.0016);
  const minDistance = Math.max(0.01, Number(controls.minDistance) || 0.01);
  const maxDistance = Math.max(minDistance, Number(controls.maxDistance) || currentDistance * 1000);
  const minScale = minDistance / currentDistance;
  const maxScale = maxDistance / currentDistance;
  scale = THREE.MathUtils.clamp(scale, minScale, maxScale);

  const nextCamera = focusPoint.clone().add(
    camera.position.clone().sub(focusPoint).multiplyScalar(scale)
  );
  const nextTarget = focusPoint.clone().add(
    controls.target.clone().sub(focusPoint).multiplyScalar(scale)
  );

  camera.position.copy(nextCamera);
  controls.target.copy(nextTarget);
  controls.update();
}

renderer.domElement.addEventListener('wheel', zoomCameraToCursor, {
  capture: true,
  passive: false
});

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
  if (!roots.length) return null;

  const hits = [];
  roots.forEach(function(root){
    raycaster.intersectObject(root, true).forEach(function(hit){ hits.push(hit); });
  });

  hits.sort(function(a,b){ return a.distance - b.distance; });
  const hit = hits[0];
  if (!hit || !hit.point) return null;

  const point = hit.point.clone();
  let normal = new THREE.Vector3(0, 1, 0);
  if (hit.face && hit.object) {
    normal = hit.face.normal.clone();
    normal.applyNormalMatrix(new THREE.Matrix3().getNormalMatrix(hit.object.matrixWorld)).normalize();
    if (normal.lengthSq() < 1e-8) normal.set(0, 1, 0);
  }

  point.__routeSnapToModel = true;
  point.__routeNormal = normal;
  return point;
}

function detectAutoRouteOffsetAxis(points) {
  if (!points || points.length < 2) return 'z';

  const ranges = { x: 0, y: 0, z: 0 };
  ['x', 'y', 'z'].forEach(function(axis){
    let min = Infinity, max = -Infinity;
    points.forEach(function(point){
      min = Math.min(min, point[axis]);
      max = Math.max(max, point[axis]);
    });
    ranges[axis] = max - min;
  });

  const span = Math.max(ranges.x, ranges.y, ranges.z);
  if (span < 1e-9) return 'z';

  const tolerance = Math.max(mmToScene(0.5), span * 0.02);
  const nearlyConstant = ['x', 'y', 'z'].filter(function(axis){
    return ranges[axis] <= tolerance;
  });

  if (nearlyConstant.length) {
    nearlyConstant.sort(function(a,b){ return ranges[a] - ranges[b]; });
    return nearlyConstant[0];
  }

  const overall = points[points.length - 1].clone().sub(points[0]);
  if (overall.lengthSq() < 1e-12) {
    overall.set(
      ranges.x ? 1 : 0,
      ranges.y ? 1 : 0,
      ranges.z ? 1 : 0
    );
  }
  overall.normalize();

  return ['x', 'y', 'z'].sort(function(a,b){
    return Math.abs(overall[a]) - Math.abs(overall[b]);
  })[0];
}

function offsetAutoRoutePoints(points, clearanceMm, axisMode) {
  const axis = axisMode === 'x' || axisMode === 'y' || axisMode === 'z'
    ? axisMode
    : detectAutoRouteOffsetAxis(points);
  const offset = mmToScene(clearanceMm);
  const vector = new THREE.Vector3();
  vector[axis] = offset;

  return points.map(function(point){
    const routePoint = point.clone().add(vector);
    routePoint.__routeSnapToModel = true;
    routePoint.__routeOffsetAxis = axis;
    return routePoint;
  });
}

function clearAutoRoutePreview() {
  autoRoutePreviewRoot.clear();
  const lengthElement = $('autoRoutePreviewLength');
  if (lengthElement) lengthElement.textContent = '0.000 m';
  const actions = $('autoRouteActions');
  if (actions) actions.classList.toggle('hidden', !state.autoRoutePoints.length);
}

function autoRouteLength(points) {
  let length = 0;
  for (let i = 1; i < points.length; i++) {
    length += points[i - 1].distanceTo(points[i]);
  }
  return length;
}

function showAutoRoutePreview(points) {
  clearAutoRoutePreview();
  if (!points || !points.length) return;

  const markerRadius = Math.max(2, mmToScene(40));

  points.forEach(function(point, index){
    const color = index === 0 ? 0xffd45a : (index === points.length - 1 ? 0x66d9ff : 0xb7c7d9);
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(markerRadius, 16, 16),
      new THREE.MeshBasicMaterial({ color: color, depthTest: false })
    );
    marker.position.copy(point);
    marker.renderOrder = 40;
    autoRoutePreviewRoot.add(marker);
  });

  if (points.length >= 2) {
    const lineGeometry = new THREE.BufferGeometry().setFromPoints(
      points.map(function(point){ return point.clone(); })
    );
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
  }

  const lengthElement = $('autoRoutePreviewLength');
  if (lengthElement) {
    lengthElement.textContent = sceneToM(autoRouteLength(points)).toFixed(3) + ' m';
  }

  const actions = $('autoRouteActions');
  if (actions) actions.classList.remove('hidden');
}

function finishAutoRoute() {
  const points = state.autoRoutePoints || [];
  if (points.length < 2) {
    toast('Select at least two points for Auto Route');
    return;
  }

  const type = $('autoRouteType').value === 'tray' ? 'tray' : 'cable';
  let routePoints = points.map(function(point){
    const routePoint = point.clone();
    routePoint.__routeSnapToModel = true;
    return routePoint;
  });

  if (type === 'tray') {
    const axisMode = $('autoRouteOffsetAxis').value;
    const axisLabel = axisMode === 'auto' ? 'Auto' : axisMode.toUpperCase();
    const rawClearance = window.prompt(
      'Tray offset from the drawn line (mm). Positive = +' + axisLabel + ', negative = -' + axisLabel + '.',
      String(state.autoRouteClearanceMm)
    );
    if (rawClearance === null) return;

    const clearanceMm = Number(rawClearance);
    if (!Number.isFinite(clearanceMm)) {
      toast('Enter a valid numeric distance in millimeters');
      return;
    }

    state.autoRouteClearanceMm = clearanceMm;
    routePoints = offsetAutoRoutePoints(points, clearanceMm, axisMode);
  }

  const beforeHistory = captureDesignState();
  const obj = createRoute(type, routePoints);

  rebuildRoutes();
  recordHistory(beforeHistory);
  state.selected = obj.id;
  state.autoRoutePoints = [];
  clearAutoRoutePreview();
  setTool('select');
  render();
  toast(obj.name + ' created — route length: ' + sceneToM(autoRouteLength(routePoints)).toFixed(3) + ' m');
}

function cancelAutoRoute() {
  state.autoRoutePoints = [];
  clearAutoRoutePreview();
  status('Ready');
  toast('Auto Route cancelled');
}

function createRoute(type, points, options) {
  const config = options || {};
  const cable = type === 'cable';
  const dia = Number.isFinite(Number(config.diameter_mm)) ? Number(config.diameter_mm) : (Number($('defaultCableDiameter').value) || 24);
  const width = Number.isFinite(Number(config.width_mm)) ? Number(config.width_mm) : (Number($('defaultTrayWidth').value) || 300);
  const height = Number.isFinite(Number(config.height_mm)) ? Number(config.height_mm) : (Number($('defaultTrayHeight').value) || 100);
  const elevation = Number($('defaultElevation').value) || 3000;
  const p = points.map(function(v){
    if (config.pointsAreMm) {
      return new THREE.Vector3(mmToScene(v.x), mmToScene(v.y), mmToScene(v.z));
    }
    return new THREE.Vector3(v.x, v.__routeSnapToModel ? v.y : mmToScene(elevation), v.z);
  });
  const obj = {
    id: id(type), kind: type,
    name: config.name || ((cable ? 'Cable-' : 'Tray-') + (state.objects.filter(function(o){ return o.kind === type; }).length + 1)),
    points: p,
    diameter_mm: dia, width_mm: width, height_mm: height,
    specification: config.specification || (cable ? 'POWER-CABLE' : width + 'x' + height + ' TRAY'),
    material: config.material || (cable ? 'Copper/PVC' : 'Galvanized Steel'),
    rotation_deg: { x: 0, y: 0, z: 0 },
    surface_alignment: null,
    engineering_generated: !!config.engineering_generated,
    engineering_equipment_id: config.engineering_equipment_id || null,
    engineering_panel_id: config.engineering_panel_id || null,
    engineering_network_id: config.engineering_network_id || null,
    engineering_floor_id: config.engineering_floor_id || null,
    engineering_floor_name: config.engineering_floor_name || null,
    engineering_classification: config.engineering_classification || null,
    engineering_cable_ids: Array.isArray(config.engineering_cable_ids) ? config.engineering_cable_ids.slice() : [],
    engineering_shared_segment_keys: Array.isArray(config.engineering_shared_segment_keys)
      ? config.engineering_shared_segment_keys.slice()
      : []
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

function routeBendGeometry(points, index, radius) {
  if (!points || index <= 0 || index >= points.length - 1) return null;

  const prev = points[index - 1];
  const cur = points[index];
  const next = points[index + 1];
  const inVector = cur.clone().sub(prev);
  const outVector = next.clone().sub(cur);
  const inLength = inVector.length();
  const outLength = outVector.length();
  if (inLength < 1e-9 || outLength < 1e-9) return null;

  const inDir = inVector.normalize();
  const outDir = outVector.normalize();
  const turnDot = THREE.MathUtils.clamp(inDir.dot(outDir), -1, 1);
  const turnAngle = Math.acos(turnDot);
  if (turnAngle < 1e-6) {
    return {
      entry: cur.clone(),
      exit: cur.clone(),
      radius: 0,
      angle: 0
    };
  }

  const tangentFactor = Math.tan(turnAngle * 0.5);
  if (!Number.isFinite(tangentFactor) || tangentFactor < 1e-9) return null;

  const maxRadius = Math.min(
    Number(radius) || 0,
    inLength / (2 * tangentFactor),
    outLength / (2 * tangentFactor)
  );
  const localRadius = Math.max(0, maxRadius);
  const tangentLength = localRadius * tangentFactor;

  return {
    entry: cur.clone().sub(inDir.clone().multiplyScalar(tangentLength)),
    exit: cur.clone().add(outDir.clone().multiplyScalar(tangentLength)),
    radius: localRadius,
    angle: turnAngle
  };
}

function sweptRectGeometry(points, sideWidth, verticalHeight, sideOffset, verticalOffset, frameNormal) {
  if (!points || points.length < 2) return null;

  const halfSide = Math.max(0, Number(sideWidth) || 0) * 0.5;
  const halfHeight = Math.max(0, Number(verticalHeight) || 0) * 0.5;
  const vertices = [];
  const indices = [];

  function sampleTangent(index) {
    const current = points[index];
    const tangent = (index === 0
      ? points[1].clone().sub(current)
      : index === points.length - 1
        ? current.clone().sub(points[index - 1])
        : points[index + 1].clone().sub(points[index - 1])
    );
    if (tangent.lengthSq() < 1e-12) return new THREE.Vector3(1, 0, 0);
    return tangent.normalize();
  }

  for (let i = 0; i < points.length; i++) {
    const tangent = sampleTangent(i);
    let referenceUp = frameNormal && frameNormal.lengthSq() > 1e-12
      ? frameNormal.clone().normalize()
      : new THREE.Vector3(0, 1, 0);

    if (Math.abs(referenceUp.dot(tangent)) > 0.999) {
      referenceUp = Math.abs(tangent.z) < 0.999
        ? new THREE.Vector3(0, 0, 1)
        : new THREE.Vector3(1, 0, 0);
    }

    let side = referenceUp.clone().cross(tangent);
    if (side.lengthSq() < 1e-12) side = new THREE.Vector3(1, 0, 0).cross(tangent);
    side.normalize();

    let up = tangent.clone().cross(side).normalize();
    if (up.dot(referenceUp) < 0) {
      side.negate();
      up.negate();
    }

    const center = points[i].clone()
      .add(side.clone().multiplyScalar(Number(sideOffset) || 0))
      .add(up.clone().multiplyScalar(Number(verticalOffset) || 0));

    const sideVector = side.multiplyScalar(halfSide);
    const upVector = up.multiplyScalar(halfHeight);

    const ring = [
      center.clone().sub(sideVector).sub(upVector),
      center.clone().add(sideVector).sub(upVector),
      center.clone().add(sideVector).add(upVector),
      center.clone().sub(sideVector).add(upVector)
    ];
    ring.forEach(function(vertex){ vertices.push(vertex.x, vertex.y, vertex.z); });

    if (i > 0) {
      const a = (i - 1) * 4;
      const b = i * 4;
      for (let j = 0; j < 4; j++) {
        const n = (j + 1) % 4;
        indices.push(a + j, a + n, b + n);
        indices.push(a + j, b + n, b + j);
      }
    }
  }

  const last = (points.length - 1) * 4;
  indices.push(0, 1, 2, 0, 2, 3);
  indices.push(last, last + 2, last + 1, last, last + 3, last + 2);

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function straightRouteCurve(points) {
  const path = new THREE.CurvePath();
  if (!points || points.length < 2) return path;
  for (let i = 1; i < points.length; i++) {
    path.add(new THREE.LineCurve3(points[i - 1], points[i]));
  }
  return path;
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
    const bend = routeBendGeometry(points, i, radius);
    if (!bend) {
      entries[i] = points[i].clone();
      exits[i] = points[i].clone();
    } else {
      entries[i] = bend.entry;
      exits[i] = bend.exit;
    }
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

function engineeringCableLaneOffsetMm(obj) {
  if (!obj || obj.kind !== 'cable' || !obj.engineering_generated || !obj.engineering_network_id) {
    return 0;
  }

  const cables = state.objects
    .filter(function(item){
      return item.kind === 'cable' &&
        item.engineering_generated &&
        item.engineering_network_id === obj.engineering_network_id;
    })
    .slice()
    .sort(function(a,b){
      return String(a.engineering_equipment_id || a.id)
        .localeCompare(String(b.engineering_equipment_id || b.id));
    });

  if (cables.length < 2) return 0;

  const index = Math.max(0, cables.findIndex(function(item){ return item.id === obj.id; }));
  const diameters = cables.map(function(item){
    return Math.max(1, Number(item.diameter_mm) || 1);
  });

  const tray = state.objects.find(function(item){
    return item.kind === 'tray' &&
      item.engineering_generated &&
      !item.panel_connection &&
      item.engineering_network_id === obj.engineering_network_id &&
      Array.isArray(item.points) &&
      item.points.length >= 2;
  });

  const trayWidthMm = Math.max(
    Number(obj.engineering_tray_width_mm) || Number(tray && tray.width_mm) || 100,
    20
  );
  const sideMarginMm = Math.max(2, Math.min(25, trayWidthMm * 0.15));
  const usableWidth = Math.max(0, trayWidthMm - sideMarginMm * 2);
  const totalDiameter = diameters.reduce(function(sum, diameter){ return sum + diameter; }, 0);
  const totalGap = Math.max(0, usableWidth - totalDiameter);
  const gap = diameters.length > 1
    ? totalGap / (diameters.length - 1)
    : 0;

  let cursor = -(totalDiameter + gap * Math.max(0, diameters.length - 1)) * 0.5;
  for (let i = 0; i < index; i++) {
    cursor += diameters[i] + gap;
  }

  return cursor + diameters[index] * 0.5;
}

function engineeringCableSharedSegment(obj, a, b) {
  if (!obj || obj.kind !== 'cable' || !obj.engineering_generated) return false;

  const sharedKeys = Array.isArray(obj.engineering_shared_segment_keys)
    ? obj.engineering_shared_segment_keys
    : [];
  if (!sharedKeys.length) return false;

  const start = {
    x:Number(a.x).toFixed(3),
    y:Number(a.y).toFixed(3),
    z:Number(a.z).toFixed(3)
  };
  const end = {
    x:Number(b.x).toFixed(3),
    y:Number(b.y).toFixed(3),
    z:Number(b.z).toFixed(3)
  };

  const dx = Math.abs(Number(b.x) - Number(a.x));
  const dy = Math.abs(Number(b.y) - Number(a.y));
  const dz = Math.abs(Number(b.z) - Number(a.z));
  const axis = dx >= dz && dx >= dy ? 'x' : (dy >= dz ? 'y' : 'z');

  let key;
  if (axis === 'x') {
    key = [
      'x',
      start.y,
      start.z,
      Math.min(Number(a.x), Number(b.x)).toFixed(3),
      Math.max(Number(a.x), Number(b.x)).toFixed(3)
    ].join('|');
  } else if (axis === 'y') {
    key = [
      'y',
      start.x,
      start.z,
      Math.min(Number(a.y), Number(b.y)).toFixed(3),
      Math.max(Number(a.y), Number(b.y)).toFixed(3)
    ].join('|');
  } else {
    key = [
      'z',
      start.x,
      start.y,
      Math.min(Number(a.z), Number(b.z)).toFixed(3),
      Math.max(Number(a.z), Number(b.z)).toFixed(3)
    ].join('|');
  }

  return sharedKeys.indexOf(key) >= 0;
}

function engineeringCableMainSegment(obj, a, b) {
  if (!obj || obj.kind !== 'cable' || !obj.engineering_generated) return false;

  const axis = obj.engineering_main_corridor_axis === 'x' ||
    obj.engineering_main_corridor_axis === 'z'
    ? obj.engineering_main_corridor_axis
    : null;
  const routingY = Number(obj.engineering_main_corridor_y_mm);
  if (!axis || !Number.isFinite(routingY)) return false;

  if (
    Math.abs(Number(a.y) - routingY) > 0.001 ||
    Math.abs(Number(b.y) - routingY) > 0.001
  ) {
    return false;
  }

  // For the parallel-row topology, lane offsets are allowed only on the
  // actual shared Main corridor. A horizontal branch at the same elevation
  // must remain a single dedicated cable path to its own equipment.
  const secondary = Number(obj.engineering_main_corridor_secondary_coordinate_mm);
  if (Number.isFinite(secondary)) {
    const secondaryValue = axis === 'x' ? Number(a.z) : Number(a.x);
    if (Math.abs(secondaryValue - secondary) > 0.001) return false;
  }

  const dx = Math.abs(Number(b.x) - Number(a.x));
  const dz = Math.abs(Number(b.z) - Number(a.z));
  return axis === 'x'
    ? dx >= dz && dx > 0.001
    : dz >= dx && dz > 0.001;
}

function engineeringCableDisplayOffset(obj, points, index) {
  const laneOffsetMm = engineeringCableLaneOffsetMm(obj);
  if (Math.abs(laneOffsetMm) < 0.001 || !Array.isArray(points) || points.length < 2) {
    return new THREE.Vector3();
  }

  const previousShared = index > 0 &&
    engineeringCableSharedSegment(obj, points[index - 1], points[index]);
  const nextShared = index < points.length - 1 &&
    engineeringCableSharedSegment(obj, points[index], points[index + 1]);

  const laneFactor = (Number(previousShared) + Number(nextShared)) * 0.5;
  if (laneFactor <= 0) return new THREE.Vector3();

  const previousPoint = index > 0 ? points[index - 1] : null;
  const nextPoint = index < points.length - 1 ? points[index + 1] : null;
  const dx = nextPoint && previousPoint
    ? Number(nextPoint.x) - Number(previousPoint.x)
    : nextPoint
      ? Number(nextPoint.x) - Number(points[index].x)
      : Number(points[index].x) - Number(previousPoint.x);
  const dz = nextPoint && previousPoint
    ? Number(nextPoint.z) - Number(previousPoint.z)
    : nextPoint
      ? Number(nextPoint.z) - Number(points[index].z)
      : Number(points[index].z) - Number(previousPoint.z);

  const side = Math.abs(dx) >= Math.abs(dz)
    ? new THREE.Vector3(0, 0, 1)
    : new THREE.Vector3(1, 0, 0);

  return side.multiplyScalar(mmToScene(laneOffsetMm * laneFactor));
}

function routeVisual(obj) {
  const g = new THREE.Group();
  g.userData.objectId = obj.id;
  g.userData.routeVisual = true;

  const center = routeCenter(obj.points);
  const localPoints = obj.points.map(function(p, index){
    return p.clone()
      .sub(center)
      .add(engineeringCableDisplayOffset(obj, obj.points, index));
  });
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

  const isEngineeringTray = obj.kind === 'tray' && obj.engineering_generated === true;
  const curveRadius = obj.kind === 'cable'
    ? Math.max(0.8, mmToScene(obj.diameter_mm * 3))
    : Math.max(1.0, mmToScene(Math.min(obj.width_mm, obj.height_mm) * 0.8));
  const routeCurve = (isEngineeringTray || (isCable && obj.engineering_generated === true))
    ? straightRouteCurve(localPoints)
    : roundedRouteCurve(localPoints, curveRadius);

  if (obj.kind === 'cable') {
    const tube = new THREE.Mesh(
      new THREE.TubeGeometry(
        routeCurve,
        Math.max(24, localPoints.length * 20),
        Math.max(0.01, mmToScene(Math.max(0.1, Number(obj.diameter_mm) || 0.1)) / 2),
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
      const xAxis = yAxis.clone().cross(zAxis).normalize();
      const basis = new THREE.Matrix4().makeBasis(xAxis, yAxis, zAxis);
      mesh.quaternion.setFromRotationMatrix(basis);

      mesh.userData.objectId = obj.id;
      g.add(mesh);
      return mesh;
    }

    function addTraySegment(a, b) {
      const len = a.distanceTo(b);
      if (len < 0.001) return;

      const dir = b.clone().sub(a).normalize();
      const centerSeg = a.clone().add(b).multiplyScalar(0.5);

      // Keep the tray cross-section upright for ordinary horizontal runs.
      // When the segment is vertical, switch to world Z so the frame stays valid.
      let trayUp = new THREE.Vector3(0, 1, 0);
      if (Math.abs(trayUp.dot(dir)) > 0.999) trayUp.set(0, 0, 1);
      trayUp.sub(dir.clone().multiplyScalar(trayUp.dot(dir))).normalize();

      const sideDir = trayUp.clone().cross(dir).normalize();
      const depth = Math.max(len, sheet * 2);
      const edgeRadius = Math.min(sheet * 0.5, Math.max(0.05, depth * 0.03));

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

    const routeEntries = localPoints.map(function(point){ return point.clone(); });
    const routeExits = localPoints.map(function(point){ return point.clone(); });

    // Engineering Auto Routing never uses physical elbow pieces.
    // Each direction change is a direct corner between two straight tray runs.
    for (let i = 1; i < localPoints.length; i++) {
      addTraySegment(routeExits[i - 1], routeEntries[i]);
    }
  }

  const linePoints = routeCurve.getPoints(
    Math.max(isEngineeringTray ? 2 : 16, localPoints.length * (isEngineeringTray ? 2 : 16))
  );
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

function disposeEngineeringAccessoryRoot() {
  const root = state.engineeringAccessoryRoot;
  if (!root) return;
  root.traverse(function(node){
    if (node.geometry && typeof node.geometry.dispose === 'function') node.geometry.dispose();
    const materials = Array.isArray(node.material) ? node.material : (node.material ? [node.material] : []);
    materials.forEach(function(material){
      if (material.map && typeof material.map.dispose === 'function') material.map.dispose();
      if (typeof material.dispose === 'function') material.dispose();
    });
  });
  root.clear();
}

function taperedTrayPartGeometry(start, end, startWidthMm, endWidthMm, startHeightMm, endHeightMm, startSideOffsetMm, endSideOffsetMm, startVerticalOffsetMm, endVerticalOffsetMm, referenceUp) {
  const direction = end.clone().sub(start);
  if (direction.lengthSq() < 1e-10) return null;
  direction.normalize();

  let up = (referenceUp || new THREE.Vector3(0,1,0)).clone().normalize();
  if (Math.abs(up.dot(direction)) > 0.999) {
    up = Math.abs(direction.z) < 0.999 ? new THREE.Vector3(0,0,1) : new THREE.Vector3(1,0,0);
  }

  let side = up.clone().cross(direction);
  if (side.lengthSq() < 1e-12) return null;
  side.normalize();
  up = direction.clone().cross(side).normalize();

  function ring(center, widthMm, heightMm, sideOffsetMm, verticalOffsetMm) {
    const halfWidth = mmToScene(widthMm) * 0.5;
    const halfHeight = mmToScene(heightMm) * 0.5;
    const centerOffset = center.clone()
      .add(side.clone().multiplyScalar(mmToScene(sideOffsetMm || 0)))
      .add(up.clone().multiplyScalar(mmToScene(verticalOffsetMm || 0)));
    const sideVector = side.clone().multiplyScalar(halfWidth);
    const upVector = up.clone().multiplyScalar(halfHeight);
    return [
      centerOffset.clone().sub(sideVector).sub(upVector),
      centerOffset.clone().add(sideVector).sub(upVector),
      centerOffset.clone().add(sideVector).add(upVector),
      centerOffset.clone().sub(sideVector).add(upVector)
    ];
  }

  const startRing = ring(start,startWidthMm,startHeightMm,startSideOffsetMm,startVerticalOffsetMm);
  const endRing = ring(end,endWidthMm,endHeightMm,endSideOffsetMm,endVerticalOffsetMm);
  const positions = [];
  startRing.concat(endRing).forEach(function(point){ positions.push(point.x,point.y,point.z); });
  const indices = [0,1,2,0,2,3,4,6,5,4,7,6];
  for (let i=0;i<4;i++) {
    const j=(i+1)%4;
    indices.push(i,j,4+j,i,4+j,4+i);
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function addTaperedTrayTransition(root,start,end,startWidth,endWidth,height,objectId) {
  if (Math.abs(start.y - end.y) > 0.001) return;
  const direction=end.clone().sub(start).normalize();
  const referenceUp=Math.abs(direction.y)>0.999?new THREE.Vector3(0,0,1):new THREE.Vector3(0,1,0);
  const material=new THREE.MeshStandardMaterial({color:0xb6bec7,roughness:0.28,metalness:0.78});
  const sheet=Math.max(2,Math.min(startWidth,endWidth,height)*0.035);
  const lipWidth=Math.max(sheet*1.5,Math.min(Math.min(startWidth,endWidth),80)*0.045);

  function addPart(geometry) {
    if(!geometry) return;
    const mesh=new THREE.Mesh(geometry,material);
    mesh.userData.objectId=objectId||null;
    root.add(mesh);
  }

  addPart(taperedTrayPartGeometry(start,end,startWidth,endWidth,sheet,sheet,0,0,
    -height*0.5+sheet*0.5,-height*0.5+sheet*0.5,referenceUp));

  [-1,1].forEach(function(sign){
    addPart(taperedTrayPartGeometry(start,end,sheet,sheet,height,height,
      sign*(startWidth*0.5-sheet*0.5),sign*(endWidth*0.5-sheet*0.5),0,0,referenceUp));
    addPart(taperedTrayPartGeometry(start,end,lipWidth,lipWidth,Math.max(sheet,lipWidth*0.65),Math.max(sheet,lipWidth*0.65),
      sign*(startWidth*0.5+lipWidth*0.5),sign*(endWidth*0.5+lipWidth*0.5),
      height*0.5-lipWidth*0.35,height*0.5-lipWidth*0.35,referenceUp));
    addPart(taperedTrayPartGeometry(start,end,Math.max(sheet*1.8,lipWidth*0.9),Math.max(sheet*1.8,lipWidth*0.9),sheet,sheet,
      sign*(startWidth*0.5-sheet*0.5),sign*(endWidth*0.5-sheet*0.5),
      -height*0.5+sheet*1.45,-height*0.5+sheet*1.45,referenceUp));
  });
}

function endpointData(route) {
  if (!route || !Array.isArray(route.points) || route.points.length < 2) return [];
  return [
    {point:route.points[0],inward:route.points[1].clone().sub(route.points[0]).normalize()},
    {point:route.points[route.points.length-1],inward:route.points[route.points.length-2].clone().sub(route.points[route.points.length-1]).normalize()}
  ];
}

function rebuildEngineeringAccessories() {
  // Engineering Auto Routing does not create reducer/converter accessories.
  // Tray widths are kept consistent per engineering network, so no transition
  // geometry is required between branch and main runs.
  disposeEngineeringAccessoryRoot();
}


function rebuildRoutes() {
  state.routeRoots.forEach(function(root){
    if(root.parent) root.parent.remove(root);
  });
  state.routeRoots.clear();

  state.objects.filter(function(o){ return o.kind==='cable' || o.kind==='tray'; }).forEach(function(o){
    const g=routeVisual(o);
    scene.add(g);
    state.routeRoots.set(o.id,g);
  });

  rebuildEngineeringAccessories();
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

  if (state.tool === 'equipment' || state.tool === 'panel') {
    placeEngineeringEntity(state.tool, e);
    return;
  }

  if (state.tool === 'floor') {
    addEngineeringFloorPoint(e);
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
      toast('Click a valid route point in the 3D view');
      return;
    }

    state.autoRoutePoints.push(point);
    showAutoRoutePreview(state.autoRoutePoints);
    status('Auto Route: ' + state.autoRoutePoints.length + ' point(s) selected');
    toast('Point ' + state.autoRoutePoints.length + ' added — press Enter when finished');
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

    if (state.measureMode === 'edge') {
      addEdgeMeasurement(target);
      return;
    }

    const expectedKind = state.measureMode === 'surface' ? 'surface' : 'point';
    if (target.kind !== expectedKind) {
      toast(expectedKind === 'surface'
        ? 'Surface to Surface mode requires two surfaces'
        : 'Point to Point mode requires two points');
      return;
    }

    if (!state.measureStart) {
      state.measureStart = target;
      if (target.kind === 'surface') {
        clearSurfaceSelectionVisuals();
        addSurfaceSelectionVisual(target, 0xffd45a);
        toast('First surface selected — click the second surface');
      } else {
        toast('First measurement point set — click the second point');
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
    if (state.tool === 'auto-route' && state.autoRoutePoints.length) {
      cancelAutoRoute();
      e.preventDefault();
      return;
    }
    if ((state.tool === 'equipment' || state.tool === 'panel')) {
      setTool('select');
      e.preventDefault();
      return;
    }
    if (state.tool === 'floor' && state.engineeringFloorPoints.length) {
      state.engineeringFloorPoints = [];
      autoRoutePreviewRoot.clear();
      setTool('select');
      toast('Floor definition cancelled');
      e.preventDefault();
      return;
    }
    if (state.tool === 'measure' && state.measureStart) {
      state.measureStart = null;
      clearSurfaceSelectionVisuals();
      toast('Measurement selection cleared');
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

  if (key === 'enter' && state.tool === 'auto-route') {
    e.preventDefault();
    e.stopPropagation();
    if (e.stopImmediatePropagation) e.stopImmediatePropagation();
    finishAutoRoute();
    return;
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
  const engineeringId = state.selected;
  const engineeringKind = engineeringEntityKind(engineeringId);
  if (engineeringKind) {
    const beforeHistory = captureDesignState();
    if (engineeringKind === 'equipment') {
      state.equipment = state.equipment.filter(function(item){ return item.id !== engineeringId; });
    } else {
      state.panels = state.panels.filter(function(item){ return item.id !== engineeringId; });
      state.equipment.forEach(function(item){
        if (item.destination_panel_id === engineeringId) {
          item.destination_panel_id = '';
          item.routing_result = null;
        }
      });
    }
    removeEngineeringGeneratedRoutes();
    state.equipment.forEach(function(item){ item.routing_result = null; });
    state.selected = null;
    recordHistory(beforeHistory);
    rebuildRoutes();
    rebuildEngineeringMarkers();
    render();
    toast('Engineering object deleted; rerun Auto Design Routing');
    return;
  }

  const idx = state.objects.findIndex(function(o){ return o.id === state.selected; });
  if (idx < 0) return;
  const object = state.objects[idx];
  const beforeHistory = object.kind === 'cable' || object.kind === 'tray' ? captureDesignState() : null;
  const root = state.modelRoots.get(state.selected);
  if (root) {
    disposeEngineeringCollisionGeometry(root);
    scene.remove(root);
    state.modelRoots.delete(state.selected);
  }
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
    toast('No project geometry is available for measurement');
    return null;
  }

  const hits = [];
  roots.forEach(function(root){
    raycaster.intersectObject(root, true).forEach(function(hit){
      if (hit && hit.point && hit.object && hit.object.userData) hits.push(hit);
    });
  });
  hits.sort(function(a,b){ return a.distance - b.distance; });

  const hit = hits[0];
  if (!hit || !hit.point) {
    toast('Click a point or surface that belongs to the project');
    return null;
  }

  const objectId = hit.object.userData && hit.object.userData.objectId ? hit.object.userData.objectId : null;
  const objectData = objectId ? state.objects.find(function(o){ return o.id === objectId; }) : null;
  const root = objectId ? getProjectRoot(objectId) : null;
  if (!root) {
    toast('The selected geometry is not a project object');
    return null;
  }

  if (state.measureMode === 'surface') {
    if (!hit.face) {
      toast('Click a valid CAD/model surface');
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
      modelId:objectData && objectData.kind === 'model' ? objectId : null,
      objectId:objectId,
      root:root,
      localPoint:face.localPoint,
      localNormal:face.localNormal,
      localTriangle:face.localTriangle,
      object:face.object,
      faceIndex:face.faceIndex
    };
  }

  if (state.measureMode === 'edge') {
    const edge = edgeMeasurementTargetFromHit(hit, event, root);
    if (!edge) {
      toast('Click directly on a visible model edge');
      return null;
    }
    return edge;
  }

  const localPoint = root.worldToLocal(hit.point.clone());
  return {
    kind:'point',
    point:hit.point.clone(),
    normal:null,
    modelId:objectData && objectData.kind === 'model' ? objectId : null,
    objectId:objectId,
    root:root,
    localPoint:{x:localPoint.x,y:localPoint.y,z:localPoint.z},
    localNormal:null,
    localTriangle:null,
    object:hit.object,
    faceIndex:hit.faceIndex
  };
}

function screenDistanceToSegment(point, a, b) {
  const ab = b.clone().sub(a);
  const denom = ab.lengthSq();
  const t = denom > 1e-12
    ? THREE.MathUtils.clamp(point.clone().sub(a).dot(ab) / denom, 0, 1)
    : 0;
  const closest = a.clone().add(ab.multiplyScalar(t));
  return point.distanceTo(closest);
}

function edgeKey(a, b, precision) {
  const scale = precision || 1000000;
  function key(v) {
    return [
      Math.round(v.x * scale),
      Math.round(v.y * scale),
      Math.round(v.z * scale)
    ].join(',');
  }
  const ka = key(a);
  const kb = key(b);
  return ka < kb ? ka + '|' + kb : kb + '|' + ka;
}

function geometryEdgeIsExternalOrCrease(geometry, edgeA, edgeB, faceIndex, hitFace) {
  const position = geometry && geometry.attributes ? geometry.attributes.position : null;
  if (!position || position.count < 3) return false;

  const indices = geometry.index;
  const faceCount = indices ? Math.floor(indices.count / 3) : Math.floor(position.count / 3);
  if (faceCount <= 1) return true;

  const targetKey = edgeKey(edgeA, edgeB, 1000000);
  const hitNormal = hitFace && hitFace.normal ? hitFace.normal.clone().normalize() : null;
  let foundAdjacent = false;

  for (let i = 0; i < faceCount; i++) {
    if (i === faceIndex) continue;

    const ids = indices
      ? [indices.getX(i * 3), indices.getX(i * 3 + 1), indices.getX(i * 3 + 2)]
      : [i * 3, i * 3 + 1, i * 3 + 2];

    const verts = ids.map(function(id){
      return new THREE.Vector3().fromBufferAttribute(position, id);
    });

    for (let e = 0; e < 3; e++) {
      if (edgeKey(verts[e], verts[(e + 1) % 3], 1000000) !== targetKey) continue;
      foundAdjacent = true;

      if (!hitNormal) return true;
      const normal = verts[1].clone().sub(verts[0])
        .cross(verts[2].clone().sub(verts[0]));
      if (normal.lengthSq() < 1e-12) return true;
      normal.normalize();

      // A coplanar shared edge is triangle tessellation, not a physical model edge.
      if (Math.abs(normal.dot(hitNormal)) < 0.9995) return true;
      return false;
    }
  }

  return !foundAdjacent;
}

function edgeMeasurementTargetFromHit(hit, event, root) {
  if (!hit || !hit.face || !hit.object || !root) return null;

  const geometry = hit.object.geometry;
  const position = geometry && geometry.attributes ? geometry.attributes.position : null;
  if (!position) return null;

  const indices = geometry.index;
  const ids = indices
    ? [indices.getX(hit.faceIndex * 3), indices.getX(hit.faceIndex * 3 + 1), indices.getX(hit.faceIndex * 3 + 2)]
    : [hit.faceIndex * 3, hit.faceIndex * 3 + 1, hit.faceIndex * 3 + 2];

  if (ids.some(function(i){ return !Number.isInteger(i) || i < 0 || i >= position.count; })) return null;

  const worldVertices = ids.map(function(i){
    return new THREE.Vector3()
      .fromBufferAttribute(position, i)
      .applyMatrix4(hit.object.matrixWorld);
  });

  const rect = renderer.domElement.getBoundingClientRect();
  const click = new THREE.Vector2(
    event.clientX - rect.left,
    event.clientY - rect.top
  );

  const candidates = [
    {a:worldVertices[0], b:worldVertices[1]},
    {a:worldVertices[1], b:worldVertices[2]},
    {a:worldVertices[2], b:worldVertices[0]}
  ];

  candidates.forEach(function(candidate){
    const sa = candidate.a.clone().project(camera);
    const sb = candidate.b.clone().project(camera);
    candidate.screenA = new THREE.Vector2(
      (sa.x * 0.5 + 0.5) * rect.width,
      (-sa.y * 0.5 + 0.5) * rect.height
    );
    candidate.screenB = new THREE.Vector2(
      (sb.x * 0.5 + 0.5) * rect.width,
      (-sb.y * 0.5 + 0.5) * rect.height
    );
    candidate.screenDistance = screenDistanceToSegment(click, candidate.screenA, candidate.screenB);

    const inverseObject = hit.object.matrixWorld.clone().invert();
    const localA = candidate.a.clone().applyMatrix4(inverseObject);
    const localB = candidate.b.clone().applyMatrix4(inverseObject);
    candidate.valid = geometryEdgeIsExternalOrCrease(
      geometry,
      localA,
      localB,
      hit.faceIndex,
      hit.face
    );
    candidate.length = candidate.a.distanceTo(candidate.b);
  });

  candidates.sort(function(a,b){
    if (a.valid !== b.valid) return a.valid ? -1 : 1;
    return a.screenDistance - b.screenDistance;
  });

  const chosen = candidates[0];
  if (!chosen || !chosen.valid || chosen.screenDistance > 14 || chosen.length < 1e-9) return null;

  const inverseRoot = root.matrixWorld.clone().invert();
  const localA = chosen.a.clone().applyMatrix4(inverseRoot);
  const localB = chosen.b.clone().applyMatrix4(inverseRoot);

  return {
    kind:'edge',
    point:chosen.a.clone().add(chosen.b).multiplyScalar(0.5),
    normal:null,
    objectId:hit.object.userData.objectId || null,
    root:root,
    edgeStart:chosen.a.clone(),
    edgeEnd:chosen.b.clone(),
    localEdgeStart:{x:localA.x,y:localA.y,z:localA.z},
    localEdgeEnd:{x:localB.x,y:localB.y,z:localB.z}
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
    editor.innerHTML = '<div class="hint">Select a measurement to edit it. Exact target distance is available only for surface-to-surface measurements.</div>';
    return;
  }

  if (selected.kind !== 'surface') {
    editor.innerHTML = '<div class="hint">' +
      (selected.kind === 'edge'
        ? 'Edge length measurements are read directly from the selected model edge.'
        : 'Exact target distance is available only for surface-to-surface measurements.') +
      '</div>';
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
      label.title=m.kind==='surface'?'Surface distance':(m.kind==='edge'?'Edge length':'Point distance');
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

function addEdgeMeasurement(target) {
  if (!target || target.kind !== 'edge' || !target.objectId || !target.localEdgeStart || !target.localEdgeEnd) return;

  const m={
    id:id('measure'),
    kind:'edge',
    start:target.edgeStart.clone(),
    end:target.edgeEnd.clone(),
    distance_m:sceneToM(target.edgeStart.distanceTo(target.edgeEnd)),
    start_normal:null,
    end_normal:null,
    start_anchor:{
      objectId:target.objectId,
      localPoint:{x:target.localEdgeStart.x,y:target.localEdgeStart.y,z:target.localEdgeStart.z},
      localNormal:null,
      localTriangle:null
    },
    end_anchor:{
      objectId:target.objectId,
      localPoint:{x:target.localEdgeEnd.x,y:target.localEdgeEnd.y,z:target.localEdgeEnd.z},
      localNormal:null,
      localTriangle:null
    }
  };

  state.measurements.push(m);
  state.selectedMeasurementId=m.id;
  syncMeasurements();
  rebuildMeasurements();
  renderMeasurementsToggle();
  toast('Edge length: ' + formatDistance(m.distance_m));
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

function toggleEngineeringNames() {
  state.engineeringSettings = {
    ...state.engineeringSettings,
    namesVisible:!state.engineeringSettings.namesVisible
  };
  setEngineeringNamesVisible(state.engineeringSettings.namesVisible);
  renderEngineeringNamesToggle();
}

function setEngineeringNamesVisible(visible) {
  const show = visible !== false;
  engineeringMarkerRoot.traverse(function(node){
    if (node.userData && node.userData.engineeringLabel) node.visible = show;
  });
}

function renderEngineeringNamesToggle() {
  const button = $('toggleEngineeringNamesBtn');
  if (!button) return;
  const visible = state.engineeringSettings.namesVisible !== false;
  button.textContent = visible ? 'Hide Names' : 'Show Names';
  button.title = visible ? 'Hide motor and panel names' : 'Show motor and panel names';
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
  const entries = state.objects.map(function(o){ return o; })
    .concat(state.equipment.map(function(item){ return { ...item, kind:'equipment' }; }))
    .concat(state.panels.map(function(panel){ return { ...panel, kind:'panel' }; }));

  if (!entries.length) {
    box.innerHTML = '<div class="hint" style="padding:10px">No objects yet.</div>';
    return;
  }

  box.innerHTML = entries.map(function(o){
    let qty = o.format || '';
    if (o.kind === 'cable' || o.kind === 'tray') qty = lengthOf(o.points).toFixed(2) + ' m';
    if (o.kind === 'equipment') qty = o.destination_panel_id ? 'Assigned' : 'Unassigned';
    if (o.kind === 'panel') qty = 'Destination';
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
  const equipment = state.equipment.find(function(x){ return x.id === state.selected; });
  const panel = state.panels.find(function(x){ return x.id === state.selected; });

  if (equipment) {
    const result = equipment.routing_result || {};
    const panelOptions = '<option value="">Select panel</option>' +
      state.panels.map(function(item){
        return '<option value="' + esc(item.id) + '"' + (item.id === equipment.destination_panel_id ? ' selected' : '') + '>' + esc(item.name) + '</option>';
      }).join('');

    el.className = 'properties';
    el.innerHTML =
      '<div class="property-group-title">Equipment</div>' +
      '<div class="prop-row"><div class="prop-label">Name</div><input class="prop-value" id="e_name" value="' + esc(equipment.name) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Type</div><input class="prop-value" id="e_type" value="' + esc(equipment.type) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Power (kW)</div><input class="prop-value" id="e_power" type="number" min="0" step="0.1" value="' + Number(equipment.power_kw || 0) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Current (A)</div><input class="prop-value" id="e_current" type="number" min="0" step="0.1" value="' + Number(equipment.current_a || 0) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Voltage (V)</div><input class="prop-value" id="e_voltage" type="number" min="0" step="1" value="' + Number(equipment.voltage_v || 0) + '"></div>' +
      '<div class="property-group-title">Cable</div>' +
      '<div class="prop-row"><div class="prop-label">Cable Name</div><input class="prop-value" id="e_cable_name" value="' + esc(equipment.cable_name) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Cable Ø (mm)</div><input class="prop-value" id="e_cable_diameter" type="number" min="0.1" step="0.1" value="' + Number(equipment.cable_diameter_mm || 0) + '"></div>' +
      '<div class="property-group-title">Destination</div>' +
      '<div class="prop-row"><div class="prop-label">Panel</div><select class="prop-value" id="e_panel">' + panelOptions + '</select></div>' +
      '<div class="property-hint">Source is attached to the selected CAD surface. Move the CAD model and the equipment anchor follows its saved local point.</div>' +
      '<div class="property-group-title">Latest Routing Result</div>' +
      '<div class="prop-row"><div class="prop-label">Cable Length</div><div class="prop-value">' + (result.cable_length_m != null ? Number(result.cable_length_m).toFixed(2) + ' m' : '-') + '</div></div>' +
      '<div class="prop-row"><div class="prop-label">Branch Tray</div><div class="prop-value">' + (result.branch_tray_width_mm ? result.branch_tray_width_mm + ' mm' : '-') + '</div></div>' +
      '<div class="prop-row"><div class="prop-label">Main Tray</div><div class="prop-value">' + (result.main_tray_length_m != null ? Number(result.main_tray_length_m).toFixed(2) + ' m' : '-') + '</div></div>' +
      (result.route_warning ? '<div class="property-hint">Routing warning: ' + esc(result.route_warning) + '</div>' : '') +
      '<button class="small danger" id="deleteObjectBtn">Delete Equipment</button>';

    function bindEngineeringField(idValue, key, numeric) {
      const field = $(idValue);
      if (!field) return;
      field.addEventListener(numeric ? 'change' : 'input', function(e){
        equipment[key] = numeric ? Number(e.target.value) || 0 : e.target.value;
        if (key !== 'name') equipment.routing_result = null;
        rebuildEngineeringMarkers();
        renderScene();
        renderEngineeringTakeoff();
      });
    }
    bindEngineeringField('e_name','name',false);
    bindEngineeringField('e_type','type',false);
    bindEngineeringField('e_power','power_kw',true);
    bindEngineeringField('e_current','current_a',true);
    bindEngineeringField('e_voltage','voltage_v',true);
    bindEngineeringField('e_cable_name','cable_name',false);
    bindEngineeringField('e_cable_diameter','cable_diameter_mm',true);
    $('e_cable_diameter').addEventListener('change', function(e){
      const diameter = Math.max(0.1, Number(e.target.value) || 0.1);
      equipment.cable_diameter_mm = diameter;
      state.objects.forEach(function(route){
        if (
          route.kind === 'cable' &&
          route.engineering_generated === true &&
          route.engineering_equipment_id === equipment.id
        ) {
          route.diameter_mm = diameter;
        }
      });
      rebuildRoutes();
      renderEngineeringTakeoff();
    });
    $('e_panel').addEventListener('change', function(e){
      equipment.destination_panel_id = e.target.value;
      equipment.routing_result = null;
      renderScene();
      renderEngineeringTakeoff();
    });
    $('deleteObjectBtn').addEventListener('click', deleteSelected);
    return;
  }

  if (panel) {
    el.className = 'properties';
    const assigned = state.equipment.filter(function(item){ return item.destination_panel_id === panel.id; });
    el.innerHTML =
      '<div class="property-group-title">Electrical Panel</div>' +
      '<div class="prop-row"><div class="prop-label">Name</div><input class="prop-value" id="pnl_name" value="' + esc(panel.name) + '"></div>' +
      '<div class="prop-row"><div class="prop-label">Type</div><input class="prop-value" id="pnl_type" value="' + esc(panel.type) + '"></div>' +
      '<div class="property-group-title">Connection Point</div>' +
      '<div class="property-hint">Attached to a CAD/model surface. XYZ is stored in millimeters.</div>' +
      '<div class="prop-row"><div class="prop-label">X</div><div class="prop-value">' + (panel.anchor && panel.anchor.point ? panel.anchor.point.x.toFixed(1) : '-') + '</div></div>' +
      '<div class="prop-row"><div class="prop-label">Y</div><div class="prop-value">' + (panel.anchor && panel.anchor.point ? panel.anchor.point.y.toFixed(1) : '-') + '</div></div>' +
      '<div class="prop-row"><div class="prop-label">Z</div><div class="prop-value">' + (panel.anchor && panel.anchor.point ? panel.anchor.point.z.toFixed(1) : '-') + '</div></div>' +
      '<div class="property-group-title">Assigned Loads</div>' +
      '<div class="property-hint">' + (assigned.length ? assigned.map(function(item){ return esc(item.name); }).join(', ') : 'No loads assigned.') + '</div>' +
      '<button class="small danger" id="deleteObjectBtn">Delete Panel</button>';

    $('pnl_name').addEventListener('input', function(e){ panel.name = e.target.value; rebuildEngineeringMarkers(); renderScene(); });
    $('pnl_type').addEventListener('input', function(e){ panel.type = e.target.value; });
    $('deleteObjectBtn').addEventListener('click', deleteSelected);
    return;
  }

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
      (cable || o.points.length < 3 ? '' : '<div class="property-group-title">Elbow Angles</div>' +
        '<div class="property-hint">0° = straight. 90° = a right-angle bend. Changing one elbow rotates the connected downstream tray while preserving its segment lengths.</div>' +
        o.points.slice(1, -1).map(function(_, index){
          const pointIndex = index + 1;
          return '<div class="prop-row"><div class="prop-label">Elbow ' + (index + 1) + ' (deg)</div><input class="prop-value" id="p_elbow_' + pointIndex + '" type="number" min="0" max="180" step="0.1" value="' + elbowAngleDeg(o.points, pointIndex).toFixed(1) + '"></div>';
        }).join('')) +
      '<div class="prop-row"><div class="prop-label">Elbows</div><div class="prop-value">' + elbows(o.points) + '</div></div>' +
      '<div class="property-hint">Drag the selected route in the 3D view for free 3D movement. Position fields give exact XYZ control.</div>' +
      '<button class="small danger" id="deleteObjectBtn">Delete</button>';

    bindPropertyHistoryInput('p_name', function(e){ o.name = e.target.value; renderScene(); });
    bindPropertyHistoryInput('p_spec', function(e){ o.specification = e.target.value; renderBoq(); });
    bindPropertyHistoryInput('p_material', function(e){ o.material = e.target.value; renderBoq(); });

    if (!cable && o.points.length >= 3) {
      o.points.slice(1, -1).forEach(function(_, index){
        const pointIndex = index + 1;
        bindPropertyHistoryChange('p_elbow_' + pointIndex, function(e){
          const value = Number(e.target.value);
          if (!setRouteElbowAngle(o, pointIndex, value)) return;
          rebuildRoutes();
          renderScene();
          renderBoq();
          renderProperties();
        });
      });
    }
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
function render() {
  rebuildEngineeringMarkers();
  renderScene();
  renderProperties();
  renderBoq();
  renderEngineeringTakeoff();
  updateRouteSelectionVisuals();
}

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

    prepareEngineeringCollisionGeometry(root);

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
    schema_version: 3,
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
          engineering_generated:!!o.engineering_generated,
          engineering_equipment_id:o.engineering_equipment_id || null,
          engineering_panel_id:o.engineering_panel_id || null,
          engineering_network_id:o.engineering_network_id || null,
          engineering_floor_id:o.engineering_floor_id || null,
          engineering_floor_name:o.engineering_floor_name || null,
          engineering_classification:o.engineering_classification || null,
          engineering_cable_ids:Array.isArray(o.engineering_cable_ids) ? o.engineering_cable_ids.slice() : [],
          engineering_shared_segment_keys:Array.isArray(o.engineering_shared_segment_keys)
            ? o.engineering_shared_segment_keys.slice()
            : [],
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
    measurements_visible:state.measurementsVisible,
    engineering:{
      equipment:state.equipment,
      panels:state.panels,
      floors:state.engineeringFloors,
      settings:state.engineeringSettings
    }
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

  state.modelRoots.forEach(function(root){
    disposeEngineeringCollisionGeometry(root);
    scene.remove(root);
  });
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
      prepareEngineeringCollisionGeometry(root);
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
  const engineering = data.engineering || {};
  state.equipment = JSON.parse(JSON.stringify(engineering.equipment || []));
  state.panels = JSON.parse(JSON.stringify(engineering.panels || []));
  state.engineeringFloors = JSON.parse(JSON.stringify(engineering.floors || []));
  state.engineeringFloorPoints = [];
  const loadedEngineeringSettings = { ...(engineering.settings || {}) };
  delete loadedEngineeringSettings.routingElevationMm;
  state.engineeringSettings = { ...state.engineeringSettings, ...loadedEngineeringSettings };
  state.measurementsVisible = data.measurements_visible !== false;
  state.measurements = (data.measurements || []).map(function(m){
    return {
      id:m.id || id('measure'),
      kind:m.kind === 'surface' ? 'surface' : (m.kind === 'edge' ? 'edge' : 'point'),
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
  syncEngineeringSettingsInputs();
  renderEngineeringNamesToggle();
  renderEngineeringFloorsList();
  rebuildEngineeringMarkers();
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
  state.modelRoots.forEach(function(root){
    disposeEngineeringCollisionGeometry(root);
    scene.remove(root);
  });
  state.modelRoots.clear();
  state.sourceModels = [];
  state.objects = [];
  state.equipment = [];
  state.panels = [];
  state.engineeringFloors = [];
  state.engineeringFloorPoints = [];
  state.lastEngineeringReport = null;
  state.selected = null; resetHistory(); state.surfacePick = null; state.surfacePickMode = false;
  state.measureStart = null; state.surfaceAlignStart = null; state.selectedMeasurementId = null; state.measurements = []; clearSurfaceSelectionVisuals(); rebuildMeasurements(); renderMeasurementsToggle(); renderMeasurementList(); renderEngineeringFloorsList(); rebuildEngineeringMarkers(); render(); toast('New project created');
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


function exportEngineeringBoq() {
  const panelMap = new Map(state.panels.map(function(panel){ return [panel.id, panel]; }));
  const rows = [
    ['Cable Schedule'],
    ['Load','Cable','Destination Panel','Length (m)']
  ];

  state.objects
    .filter(function(o){ return o.kind === 'cable' && o.engineering_generated; })
    .map(function(route){
      const item = state.equipment.find(function(eq){ return eq.id === route.engineering_equipment_id; });
      const panel = route.engineering_panel_id ? panelMap.get(route.engineering_panel_id) : null;
      return {
        load:item ? item.name : route.name,
        cable:item ? item.cable_name : route.specification,
        panel:panel ? panel.name : '',
        length:lengthOf(route.points)
      };
    })
    .sort(function(a,b){ return a.load.localeCompare(b.load); })
    .forEach(function(row){
      rows.push([row.load,row.cable,row.panel,row.length.toFixed(3)]);
    });

  rows.push([]);
  rows.push(['Tray Schedule']);
  rows.push(['Class','Size (W×H mm)','Length (m)']);

  const trayGrouped = new Map();
  state.objects
    .filter(function(o){ return o.kind === 'tray' && o.engineering_generated; })
    .forEach(function(route){
      const classification = route.engineering_classification || 'branch';
      const key = classification + '|' + route.width_mm + '|' + route.height_mm;
      const entry = trayGrouped.get(key) || {
        classification,
        width_mm:Number(route.width_mm) || 0,
        height_mm:Number(route.height_mm) || 0,
        length_m:0
      };
      entry.length_m += lengthOf(route.points);
      trayGrouped.set(key, entry);
    });

  Array.from(trayGrouped.values())
    .sort(function(a,b){
      return a.classification.localeCompare(b.classification) ||
        a.width_mm - b.width_mm ||
        a.height_mm - b.height_mm;
    })
    .forEach(function(row){
      rows.push([
        row.classification === 'main' ? 'Main' : 'Branch',
        row.width_mm + ' × ' + row.height_mm,
        row.length_m.toFixed(3)
      ]);
    });

  const csv = rows.map(function(row){
    return row.map(function(value){
      return '"' + String(value == null ? '' : value).replace(/"/g,'""') + '"';
    }).join(',');
  }).join('\n');

  downloadBlob(
    new Blob([csv], { type:'text/csv;charset=utf-8' }),
    slug(state.project.name) + '_Engineering_BOQ.csv'
  );
  toast('Engineering BOQ exported');
}

function syncEngineeringSettingsInputs() {
  const settings = state.engineeringSettings || {};
  if ($('routingGridStep')) $('routingGridStep').value = Number(settings.gridStepMm) || 100;
  if ($('autoTrayClearance')) $('autoTrayClearance').value = Number(settings.clearanceMm) || 100;
  if ($('autoTrayMaxDistance')) $('autoTrayMaxDistance').value = Number(settings.maxBodyDistanceMm) || 1500;
  if ($('mainTrayMinCables')) $('mainTrayMinCables').value = Number(settings.mainMinCables) || 2;
  if ($('autoTrayStandards')) $('autoTrayStandards').value = (settings.standardTrayWidthsMm || []).join(',');
}

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
$('toggleEngineeringNamesBtn').addEventListener('click', toggleEngineeringNames);
$('clearMeasurementsBtn').addEventListener('click', function(){
  if (!state.measurements.length) return;
  if (!confirm('Delete all dimensions?')) return;
  clearAllMeasurements();
});
setTool('select'); renderEngineeringFloorsList(); rebuildMeasurements(); renderMeasurementsToggle(); renderEngineeringNamesToggle(); renderMeasurementList(); render(); animate();

function animate(){
  requestAnimationFrame(animate);
  controls.update();
  syncMeasurements();
  updateMeasurementOverlay();
  renderer.render(scene,camera);
}
