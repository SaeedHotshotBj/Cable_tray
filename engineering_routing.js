function clonePoint(p) {
  return { x: Number(p.x) || 0, y: Number(p.y) || 0, z: Number(p.z) || 0 };
}

function distanceXZ(a, b) {
  return Math.abs(a.x - b.x) + Math.abs(a.z - b.z);
}

function pointKey(ix, iz) {
  return ix + ',' + iz;
}

class MinHeap {
  constructor() {
    this.items = [];
  }

  push(item) {
    const items = this.items;
    items.push(item);
    let index = items.length - 1;
    while (index > 0) {
      const parent = (index - 1) >> 1;
      if (items[parent].f <= item.f) break;
      items[index] = items[parent];
      index = parent;
    }
    items[index] = item;
  }

  pop() {
    const items = this.items;
    if (!items.length) return null;
    const root = items[0];
    const last = items.pop();
    if (items.length) {
      let index = 0;
      while (true) {
        const left = index * 2 + 1;
        if (left >= items.length) break;
        const right = left + 1;
        let child = left;
        if (right < items.length && items[right].f < items[left].f) child = right;
        if (items[child].f >= last.f) break;
        items[index] = items[child];
        index = child;
      }
      items[index] = last;
    }
    return root;
  }

  get length() {
    return this.items.length;
  }
}


function manhattanDistance3D(a, b) {
  return Math.abs(a.x - b.x) + Math.abs(a.y - b.y) + Math.abs(a.z - b.z);
}

function pointKey3D(ix, iy, iz) {
  return ix + ',' + iy + ',' + iz;
}

function normalizeBounds3D(start, goal, obstacles, step, paddingMm) {
  const xs = [start.x, goal.x];
  const ys = [start.y, goal.y];
  const zs = [start.z, goal.z];

  obstacles.forEach(function(rect) {
    xs.push(rect.minX, rect.maxX);
    ys.push(rect.minY, rect.maxY);
    zs.push(rect.minZ, rect.maxZ);
  });

  const minX = Math.min.apply(Math, xs) - paddingMm;
  const maxX = Math.max.apply(Math, xs) + paddingMm;
  const minY = Math.min.apply(Math, ys) - paddingMm;
  const maxY = Math.max.apply(Math, ys) + paddingMm;
  const minZ = Math.min.apply(Math, zs) - paddingMm;
  const maxZ = Math.max.apply(Math, zs) + paddingMm;

  return {
    minIx: Math.floor(minX / step),
    maxIx: Math.ceil(maxX / step),
    minIy: Math.floor(minY / step),
    maxIy: Math.ceil(maxY / step),
    minIz: Math.floor(minZ / step),
    maxIz: Math.ceil(maxZ / step)
  };
}

function adaptGridStep3D(start, goal, obstacles, baseStep, maxCells, paddingMm) {
  let step = Math.max(50, Number(baseStep) || 250);

  for (let attempt = 0; attempt < 8; attempt++) {
    const bounds = normalizeBounds3D(start, goal, obstacles, step, paddingMm);
    const width = bounds.maxIx - bounds.minIx + 1;
    const height = bounds.maxIy - bounds.minIy + 1;
    const depth = bounds.maxIz - bounds.minIz + 1;
    if (width * height * depth <= maxCells) return { step, bounds };

    const scale = Math.cbrt((width * height * depth) / maxCells);
    step = Math.max(step + 50, Math.ceil(step * scale / 50) * 50);
  }

  return {
    step,
    bounds: normalizeBounds3D(start, goal, obstacles, step, paddingMm)
  };
}

function buildBlockedSet3D(obstacles, step, bounds, clearanceMm, useAabbObstacles) {
  const blocked = new Set();
  if (useAabbObstacles === false) return blocked;
  const clearance = Math.max(0, Number(clearanceMm) || 0);
  const effectiveClearance = clearance + step * 0.55;

  obstacles.forEach(function(rect) {
    const minX = rect.minX - effectiveClearance;
    const maxX = rect.maxX + effectiveClearance;
    const minY = rect.minY - effectiveClearance;
    const maxY = rect.maxY + effectiveClearance;
    const minZ = rect.minZ - effectiveClearance;
    const maxZ = rect.maxZ + effectiveClearance;

    const ix0 = Math.max(bounds.minIx, Math.ceil(minX / step));
    const ix1 = Math.min(bounds.maxIx, Math.floor(maxX / step));
    const iy0 = Math.max(bounds.minIy, Math.ceil(minY / step));
    const iy1 = Math.min(bounds.maxIy, Math.floor(maxY / step));
    const iz0 = Math.max(bounds.minIz, Math.ceil(minZ / step));
    const iz1 = Math.min(bounds.maxIz, Math.floor(maxZ / step));

    for (let ix = ix0; ix <= ix1; ix++) {
      for (let iy = iy0; iy <= iy1; iy++) {
        for (let iz = iz0; iz <= iz1; iz++) {
          blocked.add(pointKey3D(ix, iy, iz));
        }
      }
    }
  });

  return blocked;
}

function removeEndpointBlocks3D(blocked, startCell, goalCell) {
  blocked.delete(pointKey3D(startCell.ix, startCell.iy, startCell.iz));
  blocked.delete(pointKey3D(goalCell.ix, goalCell.iy, goalCell.iz));
}

function segmentIntersectsObstacle3D(a, b, obstacle, clearanceMm) {
  const clearance = Math.max(0, Number(clearanceMm) || 0);
  const min = {
    x:obstacle.minX - clearance,
    y:obstacle.minY - clearance,
    z:obstacle.minZ - clearance
  };
  const max = {
    x:obstacle.maxX + clearance,
    y:obstacle.maxY + clearance,
    z:obstacle.maxZ + clearance
  };

  let tMin = 0;
  let tMax = 1;

  ['x','y','z'].forEach(function(axis) {
    if (tMin > tMax) return;

    const start = a[axis];
    const delta = b[axis] - a[axis];

    if (Math.abs(delta) < 1e-9) {
      if (start < min[axis] || start > max[axis]) {
        tMin = 1;
        tMax = 0;
      }
      return;
    }

    let t1 = (min[axis] - start) / delta;
    let t2 = (max[axis] - start) / delta;
    if (t1 > t2) {
      const swap = t1;
      t1 = t2;
      t2 = swap;
    }

    tMin = Math.max(tMin, t1);
    tMax = Math.min(tMax, t2);
  });

  return tMin <= tMax && tMax >= 0 && tMin <= 1;
}

function segmentClear3D(a, b, obstacles, clearanceMm) {
  for (let i = 0; i < obstacles.length; i++) {
    if (segmentIntersectsObstacle3D(a, b, obstacles[i], clearanceMm)) return false;
  }
  return true;
}

function segmentClearForRouting(a, b, obstacles, options) {
  if (!segmentWithinRoutingBounds(a, b, options)) return false;
  if (!segmentWithinBodyDistanceForRouting(a, b, options)) return false;

  if (typeof options.segmentClear === 'function') {
    return options.segmentClear(
      a,
      b,
      {
        trayWidthMm:Number(options.routingTrayWidthMm) || Number(options.trayHeightMm) || 100,
        trayHeightMm:Number(options.trayHeightMm) || 100,
        bodyClearanceMm:Number(options.clearanceMm) || 0
      }
    );
  }

  return segmentClear3D(
    a,
    b,
    obstacles,
    Number(options.centerlineClearanceMm) || Number(options.clearanceMm) || 0
  );
}

function pointWithinBodyDistanceForRouting(point, options) {
  if (typeof options.pointBodyDistanceClear !== 'function') return true;
  return options.pointBodyDistanceClear(
    point,
    {
      trayWidthMm:Number(options.routingTrayWidthMm) || Number(options.trayHeightMm) || 100,
      trayHeightMm:Number(options.trayHeightMm) || 100,
      bodyClearanceMm:Number(options.clearanceMm) || 0,
      maxBodyDistanceMm:Number(options.maxBodyDistanceMm) || 0
    }
  ) !== false;
}

function segmentWithinBodyDistanceForRouting(a, b, options) {
  if (typeof options.pointBodyDistanceClear !== 'function') return true;
  const samples = [0, 0.25, 0.5, 0.75, 1];
  for (let i = 0; i < samples.length; i++) {
    const t = samples[i];
    const point = {
      x:a.x + (b.x - a.x) * t,
      y:a.y + (b.y - a.y) * t,
      z:a.z + (b.z - a.z) * t
    };
    if (!pointWithinBodyDistanceForRouting(point, options)) return false;
  }
  return true;
}

function bridgePath3D(from, to, obstacles, options) {
  const axes = ['x','y','z'];
  const permutations = [
    ['x','y','z'], ['x','z','y'],
    ['y','x','z'], ['y','z','x'],
    ['z','x','y'], ['z','y','x']
  ];

  let best = null;
  permutations.forEach(function(order) {
    const points = [clonePoint(from)];
    let cursor = clonePoint(from);
    let valid = true;

    order.forEach(function(axis) {
      if (!valid) return;
      const target = clonePoint(cursor);
      target[axis] = to[axis];
      if (
        Math.abs(target.x - cursor.x) < 0.001 &&
        Math.abs(target.y - cursor.y) < 0.001 &&
        Math.abs(target.z - cursor.z) < 0.001
      ) return;

      if (!segmentClearForRouting(cursor, target, obstacles, options)) {
        valid = false;
        return;
      }

      points.push(target);
      cursor = target;
    });

    if (!valid) return;

    if (!best || points.length < best.length) best = points;
  });

  return best;
}

function findBridgeCell(point, otherPoint, blocked, bounds, step, obstacles, options) {
  const center = {
    ix:Math.round(point.x / step),
    iy:Math.round(point.y / step),
    iz:Math.round(point.z / step)
  };

  const candidates = [];
  for (let dx = -1; dx <= 1; dx++) {
    for (let dy = -1; dy <= 1; dy++) {
      for (let dz = -1; dz <= 1; dz++) {
        const ix = center.ix + dx;
        const iy = center.iy + dy;
        const iz = center.iz + dz;
        if (!cellInsideBounds3D(ix, iy, iz, bounds)) continue;
        if (blocked.has(pointKey3D(ix, iy, iz))) continue;

        const target = {x:ix*step,y:iy*step,z:iz*step};
        if (!pointWithinBodyDistanceForRouting(target, options)) continue;
        const bridge = bridgePath3D(point, target, obstacles, options);
        if (!bridge) continue;

        let length = 0;
        for (let i = 1; i < bridge.length; i++) {
          length += manhattanDistance3D(bridge[i-1], bridge[i]);
        }

        candidates.push({ix,iy,iz,target,bridge,length});
      }
    }
  }

  candidates.forEach(function(candidate){
    candidate.score = candidate.length +
      manhattanDistance3D(candidate.target, otherPoint) * 0.25;
  });
  candidates.sort(function(a,b){
    return a.score - b.score || a.length - b.length;
  });
  return candidates[0] || null;
}

function cellInsideBounds3D(ix, iy, iz, bounds) {
  return ix >= bounds.minIx && ix <= bounds.maxIx &&
    iy >= bounds.minIy && iy <= bounds.maxIy &&
    iz >= bounds.minIz && iz <= bounds.maxIz;
}

function makeCell3D(ix, iy, iz, dir) {
  return { ix, iy, iz, dir };
}

function cellKey3D(cell) {
  return cell.ix + ',' + cell.iy + ',' + cell.iz + ',' + cell.dir;
}

function reconstructPath3D(cameFrom, current) {
  const result = [current];
  let key = cellKey3D(current);

  while (cameFrom.has(key)) {
    const previous = cameFrom.get(key);
    result.push(previous);
    key = cellKey3D(previous);
  }

  result.reverse();
  return result;
}

function pointInsideObstacle3D(point, obstacles, clearanceMm) {
  const clearance = Math.max(0, Number(clearanceMm) || 0);
  for (let i = 0; i < obstacles.length; i++) {
    const rect = obstacles[i];
    if (
      point.x >= rect.minX - clearance && point.x <= rect.maxX + clearance &&
      point.y >= rect.minY - clearance && point.y <= rect.maxY + clearance &&
      point.z >= rect.minZ - clearance && point.z <= rect.maxZ + clearance
    ) {
      return true;
    }
  }
  return false;
}

function normalizeRoutingBounds(bounds, options) {
  if (!bounds) return null;

  const trayWidth = Math.max(1, Number(options && options.routingTrayWidthMm) || Number(options && options.trayHeightMm) || 100);
  const trayHeight = Math.max(1, Number(options && options.trayHeightMm) || 100);
  const clearance = Math.max(0, Number(options && options.clearanceMm) || 0);
  const inset = Math.max(trayWidth, trayHeight) / 2 + clearance;

  const minX = Number(bounds.minX);
  const maxX = Number(bounds.maxX);
  const minY = Number(bounds.minY);
  const maxY = Number(bounds.maxY);
  const minZ = Number(bounds.minZ);
  const maxZ = Number(bounds.maxZ);

  if (![minX,maxX,minY,maxY,minZ,maxZ].every(Number.isFinite)) return null;

  const normalized = {
    minX:minX + inset,
    maxX:maxX - inset,
    minY:minY + inset,
    maxY:maxY - inset,
    minZ:minZ + inset,
    maxZ:maxZ - inset
  };

  if (
    normalized.minX > normalized.maxX ||
    normalized.minY > normalized.maxY ||
    normalized.minZ > normalized.maxZ
  ) {
    return null;
  }

  return normalized;
}

function pointWithinRoutingBounds(point, options) {
  const bounds = normalizeRoutingBounds(options && options.routingBounds, options || {});
  if (!bounds) return true;

  const x = Number(point && point.x);
  const y = Number(point && point.y);
  const z = Number(point && point.z);
  if (![x,y,z].every(Number.isFinite)) return false;

  const eps = 0.001;
  return (
    x >= bounds.minX - eps && x <= bounds.maxX + eps &&
    y >= bounds.minY - eps && y <= bounds.maxY + eps &&
    z >= bounds.minZ - eps && z <= bounds.maxZ + eps
  );
}

function segmentWithinRoutingBounds(a, b, options) {
  if (!pointWithinRoutingBounds(a, options)) return false;
  if (!pointWithinRoutingBounds(b, options)) return false;
  return true;
}

function resolveAnchorStandoff(anchor, obstacles, options) {
  const base = clonePoint(anchor && anchor.point ? anchor.point : {});
  const sourceNormal = clonePoint(anchor && anchor.normal ? anchor.normal : {x:0,y:1,z:0});
  const normal = new THREEVector3Shim(sourceNormal.x, sourceNormal.y, sourceNormal.z);
  if (normal.lengthSq() < 1e-12) normal.set(0, 1, 0);
  normal.normalize();

  const trayWidth = Math.max(1, Number(options.routingTrayWidthMm) || Number(options.trayHeightMm) || 100);
  const trayHeight = Math.max(1, Number(options.trayHeightMm) || 100);
  const halfExtent = Math.max(trayWidth, trayHeight) / 2;
  const clearance = Math.max(0, Number(options.clearanceMm) || 0);
  const minCenterlineDistance = clearance + halfExtent;
  const step = Math.max(10, Number(options.gridStepMm) || 250);
  const probeStep = Math.max(10, Math.min(step / 4, 50));
  const configuredMaxBodyDistance = Number(options.maxBodyDistanceMm);
  const maxCenterlineDistance = Number.isFinite(configuredMaxBodyDistance) && configuredMaxBodyDistance > 0
    ? configuredMaxBodyDistance + halfExtent
    : Math.max(2500, minCenterlineDistance + step * 10);
  const startDistance = Math.max(25, minCenterlineDistance);

  if (maxCenterlineDistance < startDistance) {
    return {
      point:{
        x:base.x + normal.x * startDistance,
        y:base.y + normal.y * startDistance,
        z:base.z + normal.z * startDistance
      },
      warning:'Maximum tray distance from body is smaller than the required minimum clearance plus tray half-size.',
      distance_mm:startDistance,
      valid:false
    };
  }

  const directions = [];
  const bounds = normalizeRoutingBounds(options.routingBounds, options);

  if (bounds) {
    const center = {
      x:(bounds.minX + bounds.maxX) / 2,
      y:(bounds.minY + bounds.maxY) / 2,
      z:(bounds.minZ + bounds.maxZ) / 2
    };
    const sx = Math.sign(center.x - base.x);
    const sy = Math.sign(center.y - base.y);
    const sz = Math.sign(center.z - base.z);

    // Probe combinations that move from an attachment toward the interior.
    // This handles anchors near an edge/corner where the surface normal alone
    // cannot clear the tray cross-section from the surrounding structure.
    [
      {x:sx,y:sy,z:sz},
      {x:sx,y:sy,z:0},
      {x:sx,y:0,z:sz},
      {x:0,y:sy,z:sz},
      {x:sx,y:0,z:0},
      {x:0,y:sy,z:0},
      {x:0,y:0,z:sz}
    ].forEach(function(vector){
      const candidate = new THREEVector3Shim(vector.x, vector.y, vector.z);
      if (candidate.lengthSq() < 1e-12) return;
      candidate.normalize();
      directions.push(candidate);
    });
  }

  // Keep the attachment normal directions as fallbacks for non-enclosed or
  // irregular models that do not provide a useful interior-vector probe.
  directions.push(normal);
  directions.push(normal.clone().multiplyScalar(-1));

  const uniqueDirections = [];
  directions.forEach(function(direction){
    const duplicate = uniqueDirections.some(function(existing){
      return (
        Math.abs(existing.x - direction.x) < 1e-6 &&
        Math.abs(existing.y - direction.y) < 1e-6 &&
        Math.abs(existing.z - direction.z) < 1e-6
      );
    });
    if (!duplicate) uniqueDirections.push(direction);
  });

  let best = null;

  for (let directionIndex = 0; directionIndex < uniqueDirections.length; directionIndex++) {
    const direction = uniqueDirections[directionIndex];
    const direction = directions[directionIndex];
    for (let distance = startDistance; distance <= maxCenterlineDistance + 0.001; distance += probeStep) {
      const clampedDistance = Math.min(distance, maxCenterlineDistance);
      const point = {
        x:base.x + direction.x * clampedDistance,
        y:base.y + direction.y * clampedDistance,
        z:base.z + direction.z * clampedDistance
      };

      // Prefer the interior of the CAD/model envelope. A surface normal may
      // point outward, so standoff probing must never select an exterior point
      // merely because it satisfies the body-distance corridor.
      if (!pointWithinRoutingBounds(point, options)) continue;

      const pointClear = typeof options.standoffClear === 'function'
        ? options.standoffClear(
          base,
          point,
          {
            trayWidthMm:trayWidth,
            trayHeightMm:trayHeight,
            bodyClearanceMm:clearance,
            maxBodyDistanceMm:Number.isFinite(configuredMaxBodyDistance) ? configuredMaxBodyDistance : 0
          }
        )
        : pointWithinBodyDistanceForRouting(point, options);

      if (pointClear) {
        if (!best || clampedDistance < best.distance || (Math.abs(clampedDistance - best.distance) < 0.001 && directionIndex === 0)) {
          best = {point, distance:clampedDistance, directionIndex};
        }
        break;
      }
      if (clampedDistance >= maxCenterlineDistance) break;
    }
  }

  if (best) return {point:best.point, warning:null, distance_mm:best.distance, valid:true};

  const fallbackDistance = Math.min(maxCenterlineDistance, Math.max(startDistance, minCenterlineDistance));
  return {
    point:{
      x:base.x + normal.x * fallbackDistance,
      y:base.y + normal.y * fallbackDistance,
      z:base.z + normal.z * fallbackDistance
    },
    warning:'Could not find a valid tray standoff within the configured body-distance corridor.',
    distance_mm:fallbackDistance,
    valid:false
  };
}
// Tiny local vector helper keeps the routing module independent of the Three.js runtime.
function THREEVector3Shim(x, y, z) {
  this.x = Number(x) || 0;
  this.y = Number(y) || 0;
  this.z = Number(z) || 0;
}
THREEVector3Shim.prototype.lengthSq = function() {
  return this.x*this.x + this.y*this.y + this.z*this.z;
};
THREEVector3Shim.prototype.set = function(x,y,z) {
  this.x=x; this.y=y; this.z=z; return this;
};
THREEVector3Shim.prototype.normalize = function() {
  const len = Math.sqrt(this.lengthSq());
  if (len > 1e-12) { this.x/=len; this.y/=len; this.z/=len; }
  return this;
};
THREEVector3Shim.prototype.clone = function() {
  return new THREEVector3Shim(this.x,this.y,this.z);
};
THREEVector3Shim.prototype.multiplyScalar = function(v) {
  this.x*=v; this.y*=v; this.z*=v; return this;
};

function findGridPath3D(start, goal, obstacles, options, reuseCells) {
  const baseStep = Math.max(50, Number(options.gridStepMm) || 250);
  const paddingMm = Math.max(baseStep * 4, Number(options.routingPaddingMm) || 1000);
  const maxCells = Math.max(10000, Number(options.maxGridCells) || 120000);
  const adapted = adaptGridStep3D(start, goal, obstacles, baseStep, maxCells, paddingMm);
  const step = adapted.step;
  const bounds = adapted.bounds;

  const blocked = buildBlockedSet3D(
    obstacles,
    step,
    bounds,
    Number(options.centerlineClearanceMm) || Number(options.clearanceMm) || 0,
    options.exactCollisionRouting !== true
  );

  const startBridge = findBridgeCell(
    start, goal, blocked, bounds, step, obstacles, options
  );
  const goalBridge = findBridgeCell(
    goal, start, blocked, bounds, step, obstacles, options
  );

  if (!startBridge || !goalBridge) {
    const directClear = segmentClearForRouting(start, goal, obstacles, options);
    if (directClear) {
      return {
        step,
        points:[
          {x:start.x,y:start.y,z:start.z},
          {x:goal.x,y:goal.y,z:goal.z}
        ],
        fallback:true,
        warning:'A routing-grid bridge was unavailable, but the validated direct route was used.'
      };
    }

    return {
      step,
      points:[],
      fallback:true,
      warning:'A collision-free orthogonal bridge to the routing grid could not be found.'
    };
  }

  const startCell = {ix:startBridge.ix,iy:startBridge.iy,iz:startBridge.iz};
  const goalCell = {ix:goalBridge.ix,iy:goalBridge.iy,iz:goalBridge.iz};
  removeEndpointBlocks3D(blocked, startCell, goalCell);

  const turnPenalty = step * Math.max(0, Number(options.routeTurnPenaltyRatio ?? options.turnPenaltyRatio) || 0.04);
  const verticalPenalty = step * Math.max(0, Number(options.verticalPenaltyRatio) || 0.02);
  const reuseBonus = Math.min(0.8, Math.max(0, Number(options.reuseBonus) || 0.45));

  const open = new MinHeap();
  const cameFrom = new Map();
  const gScore = new Map();
  const startState = makeCell3D(startCell.ix, startCell.iy, startCell.iz, -1);
  const startKey = cellKey3D(startState);
  gScore.set(startKey, 0);
  open.push({
    ix:startState.ix, iy:startState.iy, iz:startState.iz, dir:-1, g:0,
    f:manhattanDistance3D(
      {x:startCell.ix*step,y:startCell.iy*step,z:startCell.iz*step},
      {x:goalCell.ix*step,y:goalCell.iy*step,z:goalCell.iz*step}
    )
  });

  const directions = [
    {x:1,y:0,z:0,dir:0},
    {x:0,y:1,z:0,dir:1},
    {x:0,y:0,z:1,dir:2},
    {x:-1,y:0,z:0,dir:3},
    {x:0,y:-1,z:0,dir:4},
    {x:0,y:0,z:-1,dir:5}
  ];

  const closed = new Set();
  let goalState = null;
  let iterations = 0;
  const maxIterations = Math.max(20000, maxCells * 12);

  while (open.length && iterations++ < maxIterations) {
    const current = open.pop();
    const currentKey = cellKey3D(current);
    if (closed.has(currentKey)) continue;

    if (
      current.ix === goalCell.ix &&
      current.iy === goalCell.iy &&
      current.iz === goalCell.iz
    ) {
      goalState = current;
      break;
    }

    closed.add(currentKey);

    for (let i = 0; i < directions.length; i++) {
      const stepDir = directions[i];
      const nx = current.ix + stepDir.x;
      const ny = current.iy + stepDir.y;
      const nz = current.iz + stepDir.z;
      if (!cellInsideBounds3D(nx, ny, nz, bounds)) continue;

      const nextKey3d = pointKey3D(nx, ny, nz);
      if (blocked.has(nextKey3d)) continue;

      const currentPoint = {
        x:current.ix * step,
        y:current.iy * step,
        z:current.iz * step
      };
      const nextPoint = {
        x:nx * step,
        y:ny * step,
        z:nz * step
      };
      if (!segmentClearForRouting(currentPoint, nextPoint, obstacles, options)) continue;

      const next = makeCell3D(nx, ny, nz, stepDir.dir);
      const nextKey = cellKey3D(next);
      if (closed.has(nextKey)) continue;

      let moveCost = step;
      if (stepDir.dir === 1 || stepDir.dir === 4) moveCost += verticalPenalty;

      const currentWorldY = current.iy * step;
      const nextWorldY = ny * step;
      const lowY = Math.min(start.y, goal.y);
      const highY = Math.max(start.y, goal.y);
      const outsideY = nextWorldY < lowY
        ? lowY - nextWorldY
        : nextWorldY > highY
          ? nextWorldY - highY
          : 0;
      moveCost += outsideY * Math.max(
        0,
        Number(options.verticalRangePenaltyRatio) || 0
      );

      if (Number.isFinite(Number(options.preferredRoutingY))) {
        moveCost += Math.abs(nextWorldY - Number(options.preferredRoutingY)) *
          Math.max(0, Number(options.preferredYPenaltyRatio) || 0);
      }

      if (current.dir >= 0 && current.dir !== stepDir.dir) moveCost += turnPenalty;
      if (reuseCells && reuseCells.has(nextKey3d)) moveCost *= (1 - reuseBonus);

      const tentative = current.g + moveCost;
      const previousBest = gScore.get(nextKey);
      if (previousBest != null && tentative >= previousBest) continue;

      gScore.set(nextKey, tentative);
      cameFrom.set(
        nextKey,
        makeCell3D(current.ix, current.iy, current.iz, current.dir)
      );

      const heuristic = manhattanDistance3D(
        {x:nx*step,y:ny*step,z:nz*step},
        {x:goalCell.ix*step,y:goalCell.iy*step,z:goalCell.iz*step}
      );
      open.push({
        ix:nx, iy:ny, iz:nz, dir:stepDir.dir,
        g:tentative,
        f:tentative + heuristic
      });
    }
  }

  if (!goalState) {
    const directClear = segmentClearForRouting(start, goal, obstacles, options);
    if (directClear) {
      return {
        step,
        points:[
          {x:start.x,y:start.y,z:start.z},
          {x:goal.x,y:goal.y,z:goal.z}
        ],
        fallback:true,
        warning:'Grid routing was unavailable, but a validated direct route was used.'
      };
    }

    return {
      step,
      points:[],
      fallback:true,
      warning:'No collision-free route inside the configured body-distance corridor was found.'
    };
  }

  const cells = reconstructPath3D(cameFrom, goalState);
  const gridPoints = cells.map(function(cell) {
    return {
      x:cell.ix * step,
      y:cell.iy * step,
      z:cell.iz * step
    };
  });

  const points = [];
  startBridge.bridge.forEach(function(point){ points.push(point); });
  gridPoints.forEach(function(point){
    const previous = points[points.length - 1];
    if (
      Math.abs(previous.x - point.x) < 0.001 &&
      Math.abs(previous.y - point.y) < 0.001 &&
      Math.abs(previous.z - point.z) < 0.001
    ) return;
    points.push(point);
  });

  const goalBridgePoints = goalBridge.bridge;
  for (let i = goalBridgePoints.length - 2; i >= 0; i--) {
    points.push(goalBridgePoints[i]);
  }

  const compressed = [];
  points.forEach(function(point) {
    if (!compressed.length) {
      compressed.push(point);
      return;
    }

    const previous = compressed[compressed.length - 1];
    if (
      Math.abs(point.x - previous.x) < 0.001 &&
      Math.abs(point.y - previous.y) < 0.001 &&
      Math.abs(point.z - previous.z) < 0.001
    ) return;

    if (compressed.length >= 2) {
      const before = compressed[compressed.length - 2];
      if (areCollinearForward(before, previous, point)) {
        compressed[compressed.length - 1] = point;
        return;
      }
    }

    compressed.push(point);
  });

  return {step, points:compressed, fallback:false, warning:null};
}

function buildCablePoints(equipment, panel, routingStart, routingGoal, routedPoints) {
  const source = clonePoint(equipment.anchor.point);
  const target = clonePoint(panel.anchor.point);
  const points = [source];

  function pushDistinct(point) {
    const candidate = clonePoint(point);
    const previous = points[points.length - 1];
    if (
      Math.abs(candidate.x - previous.x) < 0.001 &&
      Math.abs(candidate.y - previous.y) < 0.001 &&
      Math.abs(candidate.z - previous.z) < 0.001
    ) return;
    points.push(candidate);
  }

  pushDistinct(routingStart);
  (routedPoints || []).forEach(pushDistinct);
  pushDistinct(routingGoal);
  pushDistinct(target);

  const compressed = [];
  points.forEach(function(point) {
    if (!compressed.length) {
      compressed.push(point);
      return;
    }

    const previous = compressed[compressed.length - 1];
    if (
      Math.abs(point.x - previous.x) < 0.001 &&
      Math.abs(point.y - previous.y) < 0.001 &&
      Math.abs(point.z - previous.z) < 0.001
    ) return;

    if (compressed.length >= 2) {
      const before = compressed[compressed.length - 2];
      if (areCollinearForward(before, previous, point)) {
        compressed[compressed.length - 1] = point;
        return;
      }
    }

    compressed.push(point);
  });

  return compressed;
}

function areCollinearForward(a, b, c) {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const abz = b.z - a.z;
  const bcx = c.x - b.x;
  const bcy = c.y - b.y;
  const bcz = c.z - b.z;
  const crossX = aby * bcz - abz * bcy;
  const crossY = abz * bcx - abx * bcz;
  const crossZ = abx * bcy - aby * bcx;
  const crossSq = crossX * crossX + crossY * crossY + crossZ * crossZ;
  const dot = abx * bcx + aby * bcy + abz * bcz;
  return crossSq < 1e-6 && dot >= -1e-6;
}

function segmentAxis(a, b) {
  const dx = Math.abs(b.x - a.x);
  const dy = Math.abs(b.y - a.y);
  const dz = Math.abs(b.z - a.z);
  if (dx >= dy && dx >= dz) return 'x';
  if (dy >= dx && dy >= dz) return 'y';
  return 'z';
}

function rounded(value) {
  return Math.round(Number(value) * 100) / 100;
}

function segmentRecord(a, b, cableId, cableDiameter) {
  if (Math.abs(a.x - b.x) < 0.001 && Math.abs(a.y - b.y) < 0.001 && Math.abs(a.z - b.z) < 0.001) return null;

  const axis = segmentAxis(a, b);
  const start = clonePoint(a);
  const end = clonePoint(b);
  const minPoint = {
    x:Math.min(start.x,end.x),
    y:Math.min(start.y,end.y),
    z:Math.min(start.z,end.z)
  };
  const maxPoint = {
    x:Math.max(start.x,end.x),
    y:Math.max(start.y,end.y),
    z:Math.max(start.z,end.z)
  };

  const length = Math.sqrt(
    Math.pow(end.x - start.x, 2) +
    Math.pow(end.y - start.y, 2) +
    Math.pow(end.z - start.z, 2)
  );

  let key;
  if (axis === 'x') {
    key = 'x|' + rounded(start.y) + '|' + rounded(start.z) + '|' + rounded(minPoint.x) + '|' + rounded(maxPoint.x);
  } else if (axis === 'y') {
    key = 'y|' + rounded(start.x) + '|' + rounded(start.z) + '|' + rounded(minPoint.y) + '|' + rounded(maxPoint.y);
  } else {
    key = 'z|' + rounded(start.x) + '|' + rounded(start.y) + '|' + rounded(minPoint.z) + '|' + rounded(maxPoint.z);
  }

  return {
    key,
    axis,
    start,
    end,
    minPoint,
    maxPoint,
    length,
    cableIds:new Set([cableId]),
    diameterByCable:new Map([[cableId, Number(cableDiameter) || 0]])
  };
}

function chooseTrayWidth(requiredWidthMm, standardWidths) {
  const standards = (standardWidths || [])
    .map(function(v){ return Number(v); })
    .filter(function(v){ return Number.isFinite(v) && v > 0; })
    .sort(function(a,b){ return a-b; });

  const fallback = [100,150,200,300,400,500,600,800,1000,1200];
  const sizes = standards.length ? standards : fallback;
  for (let i = 0; i < sizes.length; i++) {
    if (sizes[i] >= requiredWidthMm) return sizes[i];
  }
  return Math.ceil(requiredWidthMm / 100) * 100;
}

function buildTrayRuns(cablePlans, options, mainCorridors) {
  const segments = new Map();

  cablePlans.forEach(function(plan) {
    const points = Array.isArray(plan.branch_points)
      ? plan.branch_points
      : plan.points;
    for (let i = 1; i < points.length; i++) {
      const segment = segmentRecord(
        points[i - 1],
        points[i],
        plan.equipment.id,
        plan.cable.diameter_mm
      );
      if (!segment) continue;

      const existing = segments.get(segment.key);
      if (!existing) {
        segments.set(segment.key, segment);
      } else {
        existing.cableIds.add(plan.equipment.id);
        existing.diameterByCable.set(plan.equipment.id, Number(plan.cable.diameter_mm) || 0);
      }
    }
  });

  const fillLimit = Math.min(100, Math.max(1, Number(options.fillLimitPercent) || 80));
  const sideMargin = Math.max(0, Number(options.traySideMarginMm) || 25);
  const trayHeightMm = Math.max(25, Number(options.trayHeightMm) || 100);
  const mainMinCables = Math.max(2, Number(options.mainMinCables) || 2);

  const classified = [];
  segments.forEach(function(segment) {
    let diameterSum = 0;
    segment.diameterByCable.forEach(function(value){ diameterSum += value; });

    const requiredWidth = (diameterSum + sideMargin * 2) / (fillLimit / 100);
    const width = chooseTrayWidth(requiredWidth, options.standardTrayWidthsMm);
    const classification = segment.cableIds.size >= mainMinCables ? 'main' : 'branch';

    classified.push({
      ...segment,
      classification,
      width_mm:width,
      height_mm:trayHeightMm,
      required_width_mm:requiredWidth
    });
  });

  const grouped = new Map();

  classified.forEach(function(segment) {
    let fixed1;
    let fixed2;
    if (segment.axis === 'x') {
      fixed1 = rounded(segment.start.y);
      fixed2 = rounded(segment.start.z);
    } else if (segment.axis === 'y') {
      fixed1 = rounded(segment.start.x);
      fixed2 = rounded(segment.start.z);
    } else {
      fixed1 = rounded(segment.start.x);
      fixed2 = rounded(segment.start.y);
    }

    const cableSetKey = Array.from(segment.cableIds).sort().join(',');
    const lineKey = [
      segment.axis,
      fixed1,
      fixed2,
      segment.classification,
      segment.width_mm,
      segment.height_mm,
      cableSetKey
    ].join('|');

    const entry = grouped.get(lineKey) || {
      axis:segment.axis,
      fixed1,
      fixed2,
      classification:segment.classification,
      width_mm:segment.width_mm,
      height_mm:segment.height_mm,
      cable_ids:[],
      ranges:[]
    };

    segment.cableIds.forEach(function(id){
      if (entry.cable_ids.indexOf(id) < 0) entry.cable_ids.push(id);
    });

    let low;
    let high;
    if (segment.axis === 'x') {
      low = segment.minPoint.x; high = segment.maxPoint.x;
    } else if (segment.axis === 'y') {
      low = segment.minPoint.y; high = segment.maxPoint.y;
    } else {
      low = segment.minPoint.z; high = segment.maxPoint.z;
    }

    entry.ranges.push({low,high});
    grouped.set(lineKey, entry);
  });

  const runs = [];
  grouped.forEach(function(group) {
    group.ranges.sort(function(a,b){ return a.low - b.low; });
    let current = null;

    group.ranges.forEach(function(range) {
      if (!current || range.low > current.high + 0.1) {
        if (current) runs.push(makeTrayRun(group, current));
        current = {low:range.low, high:range.high};
      } else {
        current.high = Math.max(current.high, range.high);
      }
    });

    if (current) runs.push(makeTrayRun(group, current));
  });

  const sharedMainRuns = (mainCorridors || []).map(function(corridor){
    const points = (corridor.points || []).map(clonePoint);
    return {
      classification:'main',
      width_mm:Number(corridor.width_mm) || Number(options.trayHeightMm) || 100,
      height_mm:Number(corridor.height_mm) || Number(options.trayHeightMm) || 100,
      cable_ids:Array.isArray(corridor.cable_ids) ? corridor.cable_ids.slice() : [],
      points,
      length_m:routeLengthMeters(points)
    };
  }).filter(function(run){ return run.points.length > 1; });

  return runs.concat(sharedMainRuns);
}

function makeTrayRun(group, range) {
  const cableIds = Array.from(group.cable_ids || []);
  if (group.axis === 'x') {
    return {
      classification:group.classification,
      width_mm:group.width_mm,
      height_mm:group.height_mm,
      cable_ids:cableIds,
      points:[
        {x:range.low,y:group.fixed1,z:group.fixed2},
        {x:range.high,y:group.fixed1,z:group.fixed2}
      ],
      length_m:(range.high - range.low) / 1000
    };
  }
  if (group.axis === 'y') {
    return {
      classification:group.classification,
      width_mm:group.width_mm,
      height_mm:group.height_mm,
      cable_ids:cableIds,
      points:[
        {x:group.fixed1,y:range.low,z:group.fixed2},
        {x:group.fixed1,y:range.high,z:group.fixed2}
      ],
      length_m:(range.high - range.low) / 1000
    };
  }
  return {
    classification:group.classification,
    width_mm:group.width_mm,
    height_mm:group.height_mm,
    cable_ids:cableIds,
    points:[
      {x:group.fixed1,y:group.fixed2,z:range.low},
      {x:group.fixed1,y:group.fixed2,z:range.high}
    ],
    length_m:(range.high - range.low) / 1000
  };
}

function routeLengthMeters(points) {
  let totalMm = 0;
  for (let i = 1; i < points.length; i++) {
    totalMm += Math.sqrt(
      Math.pow(points[i].x - points[i - 1].x, 2) +
      Math.pow(points[i].y - points[i - 1].y, 2) +
      Math.pow(points[i].z - points[i - 1].z, 2)
    );
  }
  return totalMm / 1000;
}

function buildTraySummary(trayRuns) {
  const grouped = new Map();
  trayRuns.forEach(function(run) {
    const key = run.classification + '|' + run.width_mm + '|' + run.height_mm;
    const current = grouped.get(key) || {
      classification:run.classification,
      width_mm:run.width_mm,
      height_mm:run.height_mm,
      length_m:0,
      runs:0
    };
    current.length_m += run.length_m;
    current.runs += 1;
    grouped.set(key,current);
  });
  return Array.from(grouped.values()).sort(function(a,b){
    return a.classification.localeCompare(b.classification) ||
      a.width_mm - b.width_mm ||
      a.height_mm - b.height_mm;
  });
}

function buildEquipmentResults(cablePlans, trayRuns) {
  return cablePlans.map(function(plan) {
    const equipmentId = plan.equipment.id;
    let branchWidth = 0;
    let branchLength = 0;
    let mainLength = 0;

    trayRuns.forEach(function(run) {
      const carriesEquipment = Array.isArray(run.cable_ids) && run.cable_ids.indexOf(equipmentId) >= 0;
      if (!carriesEquipment) return;

      if (run.classification === 'branch') {
        branchWidth = Math.max(branchWidth, run.width_mm);
        branchLength += run.length_m;
      } else {
        mainLength += run.length_m;
      }
    });

    return {
      equipment_id:equipmentId,
      cable_length_m:routeLengthMeters(plan.points),
      branch_tray_width_mm:branchWidth || null,
      branch_tray_length_m:branchLength,
      main_tray_length_m:mainLength,
      route_warning:plan.warning || null
    };
  });
}

function polylineLengthMm(points) {
  let total = 0;
  for (let i = 1; i < points.length; i++) total += manhattanDistance3D(points[i - 1], points[i]);
  return total;
}

function countPolylineTurns(points) {
  if (!points || points.length < 3) return 0;
  let turns = 0;
  for (let i = 1; i < points.length - 1; i++) {
    if (!areCollinearForward(points[i - 1], points[i], points[i + 1])) turns++;
  }
  return turns;
}

function closestPointOnPolyline3D(point, polyline) {
  let best = null;
  if (!Array.isArray(polyline) || polyline.length === 0) return null;
  if (polyline.length === 1) return {
    point:clonePoint(polyline[0]),
    distance:manhattanDistance3D(point, polyline[0]),
    segmentIndex:0
  };

  for (let i = 1; i < polyline.length; i++) {
    const a = polyline[i - 1];
    const b = polyline[i];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const dz = b.z - a.z;
    const lengthSq = dx*dx + dy*dy + dz*dz;
    let t = 0;
    if (lengthSq > 1e-9) {
      t = ((point.x - a.x) * dx + (point.y - a.y) * dy + (point.z - a.z) * dz) / lengthSq;
      t = Math.max(0, Math.min(1, t));
    }
    const candidate = {x:a.x + dx*t, y:a.y + dy*t, z:a.z + dz*t};
    const distance = manhattanDistance3D(point, candidate);
    if (!best || distance < best.distance) best = {point:candidate, distance, segmentIndex:i - 1};
  }
  return best;
}

function corridorFromAttachmentToStart(attachment, corridor, segmentIndex) {
  const points = [clonePoint(attachment)];
  for (let i = segmentIndex; i >= 0; i--) {
    const candidate = corridor[i];
    if (manhattanDistance3D(points[points.length - 1], candidate) > 0.001) points.push(clonePoint(candidate));
  }
  return points;
}

function equipmentDensityScore(entry, allEntries, radius) {
  const r = Math.max(1, Number(radius) || 1000);
  let score = 0;
  allEntries.forEach(function(other){
    const distance = manhattanDistance3D(entry.start, other.start);
    score += Math.max(0, 1 - distance / r);
  });
  return score;
}

function approximateCorridorBranchDistance(corridor, entries) {
  let total = 0;
  entries.forEach(function(entry){
    const nearest = closestPointOnPolyline3D(entry.start, corridor);
    if (nearest) total += nearest.distance;
  });
  return total;
}

function resolveHighestValidRoutingY(points, options) {
  const ceilingY = Number(options.ceilingY);
  if (!Number.isFinite(ceilingY) || !Array.isArray(points) || !points.length) return null;

  const trayHeight = Math.max(1, Number(options.trayHeightMm) || 100);
  const clearance = Math.max(0, Number(options.clearanceMm) || 0);
  const safetyGap = Math.max(0, Number(options.ceilingSafetyGapMm) || 0);
  const routingBounds = normalizeRoutingBounds(options.routingBounds, options);
  const desiredY = routingBounds
    ? Math.min(
        ceilingY - safetyGap - trayHeight / 2 - clearance,
        routingBounds.maxY
      )
    : ceilingY - safetyGap - trayHeight / 2 - clearance;

  const step = Math.max(10, Number(options.gridStepMm) || 100);
  const probeStep = Math.max(10, Math.min(step / 2, 50));
  const lowerY = Math.min.apply(
    null,
    points.map(function(point){ return Number(point.y) || 0; }).concat([desiredY - Math.max(1000, Number(options.maxBodyDistanceMm) || 1500)])
  );

  for (let y = desiredY; y >= lowerY - 0.001; y -= probeStep) {
    let valid = true;
    for (let i = 0; i < points.length; i++) {
      const point = {
        x:Number(points[i].x) || 0,
        y,
        z:Number(points[i].z) || 0
      };
      if (!pointWithinBodyDistanceForRouting(point, options)) {
        valid = false;
        break;
      }
    }
    if (valid) return y;
  }

  return null;
}

function pointAtRoutingY(point, y) {
  return {
    x:Number(point.x) || 0,
    y:Number(y),
    z:Number(point.z) || 0
  };
}


export function routeEngineeringNetwork(inputs) {
  const equipment = Array.isArray(inputs && inputs.equipment) ? inputs.equipment : [];
  const panels = Array.isArray(inputs && inputs.panels) ? inputs.panels : [];
  const options = {
    gridStepMm:Number(inputs && inputs.options && inputs.options.gridStepMm) || 250,
    clearanceMm:Number.isFinite(Number(inputs && inputs.options && inputs.options.clearanceMm))
      ? Number(inputs.options.clearanceMm)
      : 100,
    fillLimitPercent:Number(inputs && inputs.options && inputs.options.fillLimitPercent) || 80,
    trayHeightMm:Number(inputs && inputs.options && inputs.options.trayHeightMm) || 100,
    mainMinCables:Number(inputs && inputs.options && inputs.options.mainMinCables) || 2,
    turnPenaltyRatio:Number(inputs && inputs.options && inputs.options.turnPenaltyRatio) || 0.04,
    verticalPenaltyRatio:Number(inputs && inputs.options && inputs.options.verticalPenaltyRatio) || 0.02,
    verticalRangePenaltyRatio:Number(inputs && inputs.options && inputs.options.verticalRangePenaltyRatio) || 0.25,
    reuseBonus:Number(inputs && inputs.options && inputs.options.reuseBonus) || 0.45,
    maxBodyDistanceMm:Number.isFinite(Number(inputs && inputs.options && inputs.options.maxBodyDistanceMm))
      ? Number(inputs.options.maxBodyDistanceMm)
      : 1500,
    ceilingY:Number.isFinite(Number(inputs && inputs.options && inputs.options.ceilingY))
      ? Number(inputs.options.ceilingY)
      : null,
    ceilingSafetyGapMm:Number(inputs && inputs.options && inputs.options.ceilingSafetyGapMm) || 50,
    routingBounds:inputs && inputs.options && inputs.options.routingBounds
      ? { ...inputs.options.routingBounds }
      : null,
    preferredYPenaltyRatio:Number(inputs && inputs.options && inputs.options.preferredYPenaltyRatio) || 1,
    mainCorridorBranchWeight:Number(inputs && inputs.options && inputs.options.mainCorridorBranchWeight) || 1.15,
    mainCorridorTurnPenaltyRatio:Number(inputs && inputs.options && inputs.options.mainCorridorTurnPenaltyRatio) || 0.75,
    mainCorridorCandidateLimit:Number(inputs && inputs.options && inputs.options.mainCorridorCandidateLimit) || 8,
    pointBodyDistanceClear:inputs && inputs.options && typeof inputs.options.pointBodyDistanceClear === 'function'
      ? inputs.options.pointBodyDistanceClear
      : null,
    traySideMarginMm:Number(inputs && inputs.options && inputs.options.traySideMarginMm) || 25,
    standardTrayWidthsMm:inputs && inputs.options && Array.isArray(inputs.options.standardTrayWidthsMm)
      ? inputs.options.standardTrayWidthsMm
      : [100,150,200,300,400,500,600,800,1000,1200],
    maxGridCells:Number(inputs && inputs.options && inputs.options.maxGridCells) || 120000,
    routingPaddingMm:Number(inputs && inputs.options && inputs.options.routingPaddingMm) || 1000,
    exactCollisionRouting:inputs && inputs.options && inputs.options.exactCollisionRouting === true,
    segmentClear:inputs && inputs.options && typeof inputs.options.segmentClear === 'function'
      ? inputs.options.segmentClear
      : null,
    standoffClear:inputs && inputs.options && typeof inputs.options.standoffClear === 'function'
      ? inputs.options.standoffClear
      : null
  };

  const obstacles = Array.isArray(inputs && inputs.obstacles) ? inputs.obstacles : [];
  const validPanels = new Map();
  panels.forEach(function(panel){ validPanels.set(panel.id, panel); });

  const groups = new Map();
  const warnings = [];

  equipment.forEach(function(item) {
    const panel = validPanels.get(item.destination_panel_id);
    if (!panel) {
      warnings.push(item.name + ': no valid destination panel selected.');
      return;
    }

    const group = groups.get(panel.id) || {panel, equipment:[]};
    group.equipment.push(item);
    groups.set(panel.id, group);
  });

  const cablePlans = [];
  const mainCorridors = [];

  groups.forEach(function(group) {
    let planningCableDiameterSum = 0;
    group.equipment.forEach(function(item){
      planningCableDiameterSum += Math.max(0, Number(item.cable_diameter_mm) || 0);
    });

    const planningRequiredWidth = (
      planningCableDiameterSum + options.traySideMarginMm * 2
    ) / (options.fillLimitPercent / 100);
    const planningTrayWidth = chooseTrayWidth(planningRequiredWidth, options.standardTrayWidthsMm);

    const routeOptions = {
      ...options,
      centerlineClearanceMm:options.clearanceMm + planningTrayWidth / 2,
      routingTrayWidthMm:planningTrayWidth
    };

    const panelStandoff = resolveAnchorStandoff(group.panel.anchor, obstacles, routeOptions);
    if (panelStandoff.warning) warnings.push(group.panel.name + ': ' + panelStandoff.warning);
    if (panelStandoff.valid === false) return;

    const prepared = group.equipment.map(function(item) {
      const equipmentStandoff = resolveAnchorStandoff(item.anchor, obstacles, routeOptions);
      if (equipmentStandoff.warning) warnings.push(item.name + ': ' + equipmentStandoff.warning);
      return {
        equipment:item,
        start:equipmentStandoff.point,
        startDistanceMm:equipmentStandoff.distance_mm,
        valid:equipmentStandoff.valid !== false
      };
    }).filter(function(entry){ return entry.valid; });

    if (!prepared.length) return;

    // Keep the existing Main Tray Minimum Cables setting meaningful:
    // with fewer cables than the threshold, route the cable directly to the panel
    // and classify its tray as a branch-only path.
    const mainMinCables = Math.max(2, Number(options.mainMinCables) || 2);
    if (prepared.length < mainMinCables) {
      prepared.forEach(function(entry) {
        const routingY = resolveHighestValidRoutingY(
          [entry.start, panelStandoff.point],
          routeOptions
        );

        const highPanelPoint = routingY == null
          ? clonePoint(panelStandoff.point)
          : pointAtRoutingY(panelStandoff.point, routingY);

        const highRoute = findGridPath3D(
          entry.start,
          highPanelPoint,
          obstacles,
          {
            ...routeOptions,
            preferredRoutingY:routingY
          },
          new Set()
        );

        if (!highRoute.points || !highRoute.points.length) {
          warnings.push(
            entry.equipment.name + ' → ' + group.panel.name +
            ': no collision-free branch route was found.'
          );
          return;
        }

        const panelDrop = highPanelPoint.y === panelStandoff.point.y
          ? {points:[clonePoint(panelStandoff.point),clonePoint(highPanelPoint)], warning:null, fallback:false}
          : findGridPath3D(
            panelStandoff.point,
            highPanelPoint,
            obstacles,
            {
              ...routeOptions,
              preferredRoutingY:routingY
            },
            new Set()
          );

        if (!panelDrop.points || !panelDrop.points.length) {
          warnings.push(
            entry.equipment.name + ' → ' + group.panel.name +
            ': no collision-free final drop to the panel was found.'
          );
          return;
        }

        const branchPoints = [];
        (highRoute.points || []).forEach(function(point){ branchPoints.push(clonePoint(point)); });
        for (let i = panelDrop.points.length - 2; i >= 0; i--) {
          const point = clonePoint(panelDrop.points[i]);
          const previous = branchPoints[branchPoints.length - 1];
          if (
            previous &&
            Math.abs(point.x - previous.x) < 0.001 &&
            Math.abs(point.y - previous.y) < 0.001 &&
            Math.abs(point.z - previous.z) < 0.001
          ) continue;
          branchPoints.push(point);
        }

        const points = [];
        function pushDistinct(point) {
          if (!point) return;
          const candidate = clonePoint(point);
          const previous = points[points.length - 1];
          if (
            previous &&
            Math.abs(candidate.x - previous.x) < 0.001 &&
            Math.abs(candidate.y - previous.y) < 0.001 &&
            Math.abs(candidate.z - previous.z) < 0.001
          ) return;
          points.push(candidate);
        }

        pushDistinct(entry.equipment.anchor.point);
        pushDistinct(entry.start);
        highRoute.points.forEach(pushDistinct);
        for (let i = panelDrop.points.length - 2; i >= 0; i--) {
          pushDistinct(panelDrop.points[i]);
        }
        pushDistinct(group.panel.anchor.point);

        cablePlans.push({
          equipment:entry.equipment,
          panel:group.panel,
          cable:{
            name:entry.equipment.cable_name || 'Power Cable',
            diameter_mm:Number(entry.equipment.cable_diameter_mm) || 0
          },
          routing_start:entry.start,
          routing_goal:panelStandoff.point,
          main_corridor_equipment_id:null,
          standoff_distance_mm:Math.max(entry.startDistanceMm, panelStandoff.distance_mm),
          planning_tray_width_mm:planningTrayWidth,
          branch_points:branchPoints,
          points,
          warning:highRoute.warning || panelDrop.warning || null,
          fallback:!!highRoute.fallback || !!panelDrop.fallback
        });
      });
      return;
    }
    const densityRadius = Math.max(
      Number(options.gridStepMm) * 8,
      Number(options.maxBodyDistanceMm) * 0.75,
      1000
    );
    prepared.forEach(function(entry){
      entry.densityScore = equipmentDensityScore(entry, prepared, densityRadius);
    });

    const candidateLimit = Math.min(
      prepared.length,
      Math.max(1, Math.floor(Number(options.mainCorridorCandidateLimit) || 8))
    );

    const candidates = prepared.slice().sort(function(a,b){
      return b.densityScore - a.densityScore ||
        manhattanDistance3D(a.start, panelStandoff.point) -
        manhattanDistance3D(b.start, panelStandoff.point);
    }).slice(0, candidateLimit);

    let bestCorridor = null;
    candidates.forEach(function(candidate){
      const routingY = resolveHighestValidRoutingY(
        [panelStandoff.point, candidate.start],
        routeOptions
      );
      if (routingY == null) return;

      const corridorResult = findGridPath3D(
        pointAtRoutingY(panelStandoff.point, routingY),
        pointAtRoutingY(candidate.start, routingY),
        obstacles,
        {
          ...routeOptions,
          preferredRoutingY:routingY,
          routeTurnPenaltyRatio:Math.max(
            Number(options.turnPenaltyRatio) || 20,
            Number(options.mainCorridorTurnPenaltyRatio) || 100
          )
        },
        new Set()
      );

      if (!corridorResult.points || corridorResult.points.length < 2) return;

      const branchEstimate = approximateCorridorBranchDistance(corridorResult.points, prepared);
      const corridorLength = polylineLengthMm(corridorResult.points);
      const turns = countPolylineTurns(corridorResult.points);
      const turnPriorityWeight = Math.max(
        Number(options.gridStepMm) * 100,
        1
      );
      const score =
        turns * turnPriorityWeight +
        corridorLength +
        branchEstimate * Math.max(0.1, Number(options.mainCorridorBranchWeight) || 1.15);

      if (!bestCorridor || score < bestCorridor.score) {
        bestCorridor = {
          points:corridorResult.points,
          endpoint:candidate,
          score,
          length_mm:corridorLength,
          turns,
          routingY
        };
      }
    });

    if (!bestCorridor) {
      warnings.push(
        group.panel.name + ': no collision-free main tray corridor was found inside the configured body-distance corridor.'
      );
      return;
    }

    // Successful corridor selection is informational, not a warning.

    const corridorPoints = bestCorridor.points;
    const mainRouteOptions = {
      ...routeOptions,
      preferredRoutingY:bestCorridor.routingY,
      routeTurnPenaltyRatio:Math.max(
        Number(options.turnPenaltyRatio) || 20,
        Number(options.mainCorridorTurnPenaltyRatio) || 100
      )
    };

    const panelHighPoint = corridorPoints[0];
    const panelDrop = findGridPath3D(
      panelStandoff.point,
      panelHighPoint,
      obstacles,
      mainRouteOptions,
      new Set()
    );

    if (!panelDrop.points || !panelDrop.points.length) {
      warnings.push(
        group.panel.name + ': no collision-free high-level connection to the panel was found.'
      );
      return;
    }

    const mainTrayPoints = panelDrop.points
      .slice()
      .reverse()
      .concat(corridorPoints.slice(1))
      .map(clonePoint);

    mainCorridors.push({
      panel_id:group.panel.id,
      width_mm:planningTrayWidth,
      height_mm:Number(options.trayHeightMm) || 100,
      cable_ids:prepared.map(function(entry){ return entry.equipment.id; }),
      points:mainTrayPoints
    });

    const reuseCells = new Set();
    corridorPoints.forEach(function(point){
      const adaptedStep = Math.max(50, Number(routeOptions.gridStepMm) || 250);
      reuseCells.add(pointKey3D(
        Math.round(point.x / adaptedStep),
        Math.round(point.y / adaptedStep),
        Math.round(point.z / adaptedStep)
      ));
    });

    prepared.forEach(function(entry) {
      const attachment = closestPointOnPolyline3D(entry.start, corridorPoints);
      if (!attachment) return;

      let branchResult = {
        points:[clonePoint(entry.start)],
        warning:null,
        fallback:false
      };

      if (attachment.distance > 0.5) {
        branchResult = findGridPath3D(
          entry.start,
          attachment.point,
          obstacles,
          mainRouteOptions,
          reuseCells
        );
      }

      if (!branchResult.points || !branchResult.points.length) {
        warnings.push(
          entry.equipment.name + ' → ' + group.panel.name +
          ': no collision-free branch to the main tray was found.'
        );
        return;
      }

      branchResult.points.forEach(function(point){
        const adaptedStep = branchResult.step || routeOptions.gridStepMm;
        reuseCells.add(pointKey3D(
          Math.round(point.x / adaptedStep),
          Math.round(point.y / adaptedStep),
          Math.round(point.z / adaptedStep)
        ));
      });

      const corridorToPanel = corridorFromAttachmentToStart(
        attachment.point,
        corridorPoints,
        attachment.segmentIndex
      );

      const points = [];
      function pushDistinct(point) {
        if (!point) return;
        const candidate = clonePoint(point);
        const previous = points[points.length - 1];
        if (
          previous &&
          Math.abs(candidate.x - previous.x) < 0.001 &&
          Math.abs(candidate.y - previous.y) < 0.001 &&
          Math.abs(candidate.z - previous.z) < 0.001
        ) return;
        points.push(candidate);
      }

      pushDistinct(entry.equipment.anchor.point);
      pushDistinct(entry.start);
      (branchResult.points || []).forEach(pushDistinct);
      corridorToPanel.forEach(pushDistinct);
      for (let i = panelDrop.points.length - 2; i >= 0; i--) {
        pushDistinct(panelDrop.points[i]);
      }
      pushDistinct(group.panel.anchor.point);

      const compressed = [];
      points.forEach(function(point){
        if (!compressed.length) {
          compressed.push(point);
          return;
        }
        const previous = compressed[compressed.length - 1];
        if (
          Math.abs(point.x - previous.x) < 0.001 &&
          Math.abs(point.y - previous.y) < 0.001 &&
          Math.abs(point.z - previous.z) < 0.001
        ) return;
        if (compressed.length >= 2) {
          const before = compressed[compressed.length - 2];
          if (areCollinearForward(before, previous, point)) {
            compressed[compressed.length - 1] = point;
            return;
          }
        }
        compressed.push(point);
      });

      cablePlans.push({
        equipment:entry.equipment,
        panel:group.panel,
        cable:{
          name:entry.equipment.cable_name || 'Power Cable',
          diameter_mm:Number(entry.equipment.cable_diameter_mm) || 0
        },
        routing_start:entry.start,
        routing_goal:panelStandoff.point,
        main_corridor_equipment_id:bestCorridor.endpoint.equipment.id,
        main_corridor_routing_y_mm:bestCorridor.routingY,
        standoff_distance_mm:Math.max(entry.startDistanceMm, panelStandoff.distance_mm),
        planning_tray_width_mm:planningTrayWidth,
        branch_points:(branchResult.points || []).map(clonePoint),
        points:compressed,
        warning:branchResult.warning || null,
        fallback:!!branchResult.fallback
      });
    });
  });

  const trayRuns = buildTrayRuns(cablePlans, options, mainCorridors);
  const traySummary = buildTraySummary(trayRuns);
  const equipmentResults = buildEquipmentResults(cablePlans, trayRuns);

  let totalCableLengthM = 0;
  cablePlans.forEach(function(plan){ totalCableLengthM += routeLengthMeters(plan.points); });

  let totalTrayLengthM = 0;
  trayRuns.forEach(function(run){ totalTrayLengthM += run.length_m; });

  return {
    cable_plans:cablePlans,
    tray_runs:trayRuns,
    main_corridors:mainCorridors,
    tray_summary:traySummary,
    equipment_results:equipmentResults,
    warnings,
    total_cable_length_m:totalCableLengthM,
    total_tray_length_m:totalTrayLengthM,
    options:{
      ...options,
      routing_method:'3d_clearance_astar',
      routing_priority:'minimum_turns_then_length'
    }
  };
}
