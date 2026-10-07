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

function buildBlockedSet3D(obstacles, step, bounds, clearanceMm) {
  const blocked = new Set();
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

function bridgePath3D(from, to, obstacles, clearanceMm) {
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

      if (!segmentClear3D(cursor, target, obstacles, clearanceMm)) {
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

function findBridgeCell(point, blocked, bounds, step, obstacles, clearanceMm) {
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
        const bridge = bridgePath3D(point, target, obstacles, clearanceMm);
        if (!bridge) continue;

        let length = 0;
        for (let i = 1; i < bridge.length; i++) {
          length += manhattanDistance3D(bridge[i-1], bridge[i]);
        }

        candidates.push({ix,iy,iz,target,bridge,length});
      }
    }
  }

  candidates.sort(function(a,b){ return a.length - b.length; });
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

function resolveAnchorStandoff(anchor, obstacles, options) {
  const base = clonePoint(anchor && anchor.point ? anchor.point : {});
  const sourceNormal = clonePoint(anchor && anchor.normal ? anchor.normal : {x:0,y:1,z:0});
  const normal = new THREEVector3Shim(sourceNormal.x, sourceNormal.y, sourceNormal.z);
  if (normal.lengthSq() < 1e-12) normal.set(0, 1, 0);
  normal.normalize();

  const clearance = Math.max(0, Number(options.clearanceMm) || 0);
  const step = Math.max(10, Number(options.gridStepMm) || 250);
  const probeStep = Math.max(10, Math.min(step / 4, 50));
  const startDistance = Math.max(25, clearance);
  const maxDistance = Math.max(2500, startDistance + step * 10);

  const directions = [normal, normal.clone().multiplyScalar(-1)];
  let best = null;

  for (let directionIndex = 0; directionIndex < directions.length; directionIndex++) {
    const direction = directions[directionIndex];
    for (let distance = startDistance; distance <= maxDistance; distance += probeStep) {
      const point = {
        x: base.x + direction.x * distance,
        y: base.y + direction.y * distance,
        z: base.z + direction.z * distance
      };
      if (!pointInsideObstacle3D(point, obstacles, clearance)) {
        if (!best || distance < best.distance || (distance === best.distance && directionIndex === 0)) {
          best = { point, distance, directionIndex };
        }
        break;
      }
    }
  }

  if (best) return { point:best.point, warning:null, distance_mm:best.distance };

  const fallbackDistance = maxDistance;
  return {
    point:{
      x:base.x + normal.x * fallbackDistance,
      y:base.y + normal.y * fallbackDistance,
      z:base.z + normal.z * fallbackDistance
    },
    warning:'Could not find a full clearance standoff from the selected surface; maximum standoff distance was used.',
    distance_mm:fallbackDistance
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

  const blocked = buildBlockedSet3D(obstacles, step, bounds, Number(options.clearanceMm) || 0);
  const startBridge = findBridgeCell(
    start, blocked, bounds, step, obstacles, Number(options.clearanceMm) || 0
  );
  const goalBridge = findBridgeCell(
    goal, blocked, bounds, step, obstacles, Number(options.clearanceMm) || 0
  );

  if (!startBridge || !goalBridge) {
    return {
      step,
      points:[
        {x:start.x,y:start.y,z:start.z},
        {x:goal.x,y:goal.y,z:goal.z}
      ],
      fallback:true,
      warning:'A collision-free orthogonal bridge to the routing grid could not be found.'
    };
  }

  const startCell = {ix:startBridge.ix,iy:startBridge.iy,iz:startBridge.iz};
  const goalCell = {ix:goalBridge.ix,iy:goalBridge.iy,iz:goalBridge.iz};
  removeEndpointBlocks3D(blocked, startCell, goalCell);

  const turnPenalty = step * Math.max(0, Number(options.turnPenaltyRatio) || 0.04);
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

      const next = makeCell3D(nx, ny, nz, stepDir.dir);
      const nextKey = cellKey3D(next);
      if (closed.has(nextKey)) continue;

      let moveCost = step;
      if (stepDir.dir === 1 || stepDir.dir === 4) moveCost += verticalPenalty;
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
    return {
      step,
      points:[
        {x:start.x,y:start.y,z:start.z},
        {x:goal.x,y:goal.y,z:goal.z}
      ],
      fallback:true,
      warning:'No obstacle-free 3D routing-grid path was found; a direct fallback was returned.'
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

function buildTrayRuns(cablePlans, options) {
  const segments = new Map();

  cablePlans.forEach(function(plan) {
    const points = plan.points;
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

  return runs;
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
    reuseBonus:Number(inputs && inputs.options && inputs.options.reuseBonus) || 0.45,
    traySideMarginMm:Number(inputs && inputs.options && inputs.options.traySideMarginMm) || 25,
    standardTrayWidthsMm:inputs && inputs.options && Array.isArray(inputs.options.standardTrayWidthsMm)
      ? inputs.options.standardTrayWidthsMm
      : [100,150,200,300,400,500,600,800,1000,1200],
    maxGridCells:Number(inputs && inputs.options && inputs.options.maxGridCells) || 120000,
    routingPaddingMm:Number(inputs && inputs.options && inputs.options.routingPaddingMm) || 1000
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

  groups.forEach(function(group) {
    const panelStandoff = resolveAnchorStandoff(group.panel.anchor, obstacles, options);
    if (panelStandoff.warning) warnings.push(group.panel.name + ': ' + panelStandoff.warning);

    const prepared = group.equipment.map(function(item) {
      const equipmentStandoff = resolveAnchorStandoff(item.anchor, obstacles, options);
      if (equipmentStandoff.warning) warnings.push(item.name + ': ' + equipmentStandoff.warning);

      return {
        equipment:item,
        start:equipmentStandoff.point,
        startDistanceMm:equipmentStandoff.distance_mm
      };
    });

    prepared.sort(function(a,b){
      return manhattanDistance3D(b.start, panelStandoff.point) -
        manhattanDistance3D(a.start, panelStandoff.point);
    });

    const reuseCells = new Set();

    prepared.forEach(function(entry) {
      const horizontal = findGridPath3D(
        entry.start,
        panelStandoff.point,
        obstacles,
        options,
        reuseCells
      );

      horizontal.points.forEach(function(point){
        const adaptedStep = horizontal.step || options.gridStepMm;
        reuseCells.add(pointKey3D(
          Math.round(point.x / adaptedStep),
          Math.round(point.y / adaptedStep),
          Math.round(point.z / adaptedStep)
        ));
      });

      const points = buildCablePoints(
        entry.equipment,
        group.panel,
        entry.start,
        panelStandoff.point,
        horizontal.points
      );

      if (horizontal.warning) warnings.push(
        entry.equipment.name + ' → ' + group.panel.name + ': ' + horizontal.warning
      );

      cablePlans.push({
        equipment:entry.equipment,
        panel:group.panel,
        cable:{
          name:entry.equipment.cable_name || 'Power Cable',
          diameter_mm:Number(entry.equipment.cable_diameter_mm) || 0
        },
        routing_start:entry.start,
        routing_goal:panelStandoff.point,
        standoff_distance_mm:Math.max(entry.startDistanceMm, panelStandoff.distance_mm),
        points,
        warning:horizontal.warning || null,
        fallback:!!horizontal.fallback
      });
    });
  });

  const trayRuns = buildTrayRuns(cablePlans, options);
  const traySummary = buildTraySummary(trayRuns);
  const equipmentResults = buildEquipmentResults(cablePlans, trayRuns);

  let totalCableLengthM = 0;
  cablePlans.forEach(function(plan){ totalCableLengthM += routeLengthMeters(plan.points); });

  let totalTrayLengthM = 0;
  trayRuns.forEach(function(run){ totalTrayLengthM += run.length_m; });

  return {
    cable_plans:cablePlans,
    tray_runs:trayRuns,
    tray_summary:traySummary,
    equipment_results:equipmentResults,
    warnings,
    total_cable_length_m:totalCableLengthM,
    total_tray_length_m:totalTrayLengthM,
    options:{
      ...options,
      routing_method:'3d_clearance_astar'
    }
  };
}
