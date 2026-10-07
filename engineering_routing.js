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
  const hasFixedRoutingY = Number.isFinite(Number(options.fixedRoutingY));
  const center = {
    ix:Math.round(point.x / step),
    iy:hasFixedRoutingY
      ? Math.round(Number(options.fixedRoutingY) / step)
      : Math.round(point.y / step),
    iz:Math.round(point.z / step)
  };

  const candidates = [];
  for (let dx = -1; dx <= 1; dx++) {
    for (let dy = hasFixedRoutingY ? 0 : -1; dy <= (hasFixedRoutingY ? 0 : 1); dy++) {
      for (let dz = -1; dz <= 1; dz++) {
        const ix = center.ix + dx;
        const iy = center.iy + dy;
        const iz = center.iz + dz;
        if (!cellInsideBounds3D(ix, iy, iz, bounds)) continue;
        if (blocked.has(pointKey3D(ix, iy, iz))) continue;

        const target = {x:ix*step,y:iy*step,z:iz*step};
        if (
          Number.isFinite(Number(options.fixedRoutingY)) &&
          Math.abs(target.y - Number(options.fixedRoutingY)) > 0.001
        ) continue;
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
    maxZ:maxZ - inset,
    valid:true
  };

  // An over-large clearance can make the usable interior envelope empty.
  // That means an internal route is impossible; it must not silently disable
  // the envelope and allow the router to escape outside the model.
  if (
    normalized.minX > normalized.maxX ||
    normalized.minY > normalized.maxY ||
    normalized.minZ > normalized.maxZ
  ) {
    normalized.valid = false;
  }

  return normalized;
}

function pointWithinRoutingBounds(point, options) {
  const bounds = normalizeRoutingBounds(options && options.routingBounds, options || {});
  if (!bounds) return true;
  if (bounds.valid === false) return false;

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
  const preferredStandoffDistance = Math.min(
    maxCenterlineDistance,
    Math.max(startDistance, Number(options.preferredStandoffDistanceMm) || startDistance + step)
  );

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
      {x:sx,y:0,z:0},
      {x:0,y:sy,z:0},
      {x:0,y:0,z:sz},
      {x:sx,y:sy,z:0},
      {x:sx,y:0,z:sz},
      {x:0,y:sy,z:sz},
      {x:sx,y:sy,z:sz}
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

      if (!pointClear) continue;

      const distancePreference = Math.abs(clampedDistance - preferredStandoffDistance);
      const diagonalPenalty =
        (Math.abs(direction.x) > 1e-6 ? 1 : 0) +
        (Math.abs(direction.y) > 1e-6 ? 1 : 0) +
        (Math.abs(direction.z) > 1e-6 ? 1 : 0);
      const structurePenalty = (
        options.structurePrimaryAxis &&
        ((options.structurePrimaryAxis === 'x' && Math.abs(direction.x) < 0.999) ||
         (options.structurePrimaryAxis === 'z' && Math.abs(direction.z) < 0.999))
      ) ? 1 : 0;

      if (
        !best ||
        distancePreference < best.distancePreference - 0.001 ||
        (
          Math.abs(distancePreference - best.distancePreference) < 0.001 &&
          (
            diagonalPenalty < best.diagonalPenalty ||
            (
              diagonalPenalty === best.diagonalPenalty &&
              (
                structurePenalty < best.structurePenalty ||
                (
                  structurePenalty === best.structurePenalty &&
                  clampedDistance < best.distance
                )
              )
            )
          )
        )
      ) {
        best = {
          point,
          distance:clampedDistance,
          directionIndex,
          distancePreference,
          diagonalPenalty,
          structurePenalty
        };
      }
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

function nearestNetworkPoint3D(point, networkPoints) {
  let best = null;
  (networkPoints || []).forEach(function(candidate) {
    const distance = manhattanDistance3D(point, candidate);
    if (!best || distance < best.distance) {
      best = { point:clonePoint(candidate), distance };
    }
  });
  return best;
}

function networkBounds3D(points) {
  if (!Array.isArray(points) || !points.length) return null;
  const bounds = {
    minX:Infinity, maxX:-Infinity,
    minY:Infinity, maxY:-Infinity,
    minZ:Infinity, maxZ:-Infinity
  };
  points.forEach(function(point) {
    bounds.minX = Math.min(bounds.minX, Number(point.x) || 0);
    bounds.maxX = Math.max(bounds.maxX, Number(point.x) || 0);
    bounds.minY = Math.min(bounds.minY, Number(point.y) || 0);
    bounds.maxY = Math.max(bounds.maxY, Number(point.y) || 0);
    bounds.minZ = Math.min(bounds.minZ, Number(point.z) || 0);
    bounds.maxZ = Math.max(bounds.maxZ, Number(point.z) || 0);
  });
  return bounds;
}

function manhattanDistanceToBounds3D(point, bounds) {
  if (!bounds) return 0;
  const dx = point.x < bounds.minX
    ? bounds.minX - point.x
    : (point.x > bounds.maxX ? point.x - bounds.maxX : 0);
  const dy = point.y < bounds.minY
    ? bounds.minY - point.y
    : (point.y > bounds.maxY ? point.y - bounds.maxY : 0);
  const dz = point.z < bounds.minZ
    ? bounds.minZ - point.z
    : (point.z > bounds.maxZ ? point.z - bounds.maxZ : 0);
  return dx + dy + dz;
}

function findGridPath3D(start, goal, obstacles, options, reuseCells) {
  const networkGoalPoints = Array.isArray(options && options.networkGoalPoints)
    ? options.networkGoalPoints.filter(Boolean).map(clonePoint)
    : [];
  const networkMode = networkGoalPoints.length > 0;
  if (networkMode && !goal) {
    const nearest = nearestNetworkPoint3D(start, networkGoalPoints);
    goal = nearest ? nearest.point : null;
  }
  if (!goal) {
    return {
      step:Math.max(50, Number(options.gridStepMm) || 250),
      points:[],
      fallback:true,
      warning:'No routing goal was available.'
    };
  }

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

  let goalBridge = null;
  let networkGoalKeys = new Set();
  const networkGoalMap = new Map();
  let networkBounds = null;

  if (networkMode) {
    networkBounds = networkBounds3D(networkGoalPoints);
    networkGoalPoints.forEach(function(point) {
      const ix = Math.round(point.x / step);
      const iy = Math.round(point.y / step);
      const iz = Math.round(point.z / step);
      for (let dx = -1; dx <= 1; dx++) {
        for (let dz = -1; dz <= 1; dz++) {
          const key = pointKey3D(ix + dx, iy, iz + dz);
          networkGoalKeys.add(key);
          if (!networkGoalMap.has(key)) networkGoalMap.set(key, []);
          networkGoalMap.get(key).push(point);
        }
      }
    });
  } else {
    goalBridge = findBridgeCell(
      goal, start, blocked, bounds, step, obstacles, options
    );
  }

  if (!startBridge || (!networkMode && !goalBridge)) {
    // A route can be completely valid even when the grid bridge around an
    // endpoint is unavailable (for example, when the endpoint falls between
    // grid cells). Prefer a validated orthogonal bridge before treating the
    // route as a fallback.
    if (!networkMode && !Number.isFinite(Number(options.fixedRoutingY))) {
      // Before reporting a failed A* search, retry a deterministic orthogonal
      // bridge. This removes false fallback warnings for simple valid routes
      // that do not need the full grid search.
      const orthogonalBridge = bridgePath3D(start, goal, obstacles, options);
      if (orthogonalBridge && orthogonalBridge.length >= 2) {
        return {
          step,
          points:orthogonalBridge,
          fallback:false,
          warning:null
        };
      }

      const directClear = segmentClearForRouting(start, goal, obstacles, options);
      if (directClear && (
        Math.abs(start.y - goal.y) < 0.001 &&
        Math.abs(start.z - goal.z) < 0.001 ||
        Math.abs(start.x - goal.x) < 0.001 &&
        Math.abs(start.z - goal.z) < 0.001 ||
        Math.abs(start.x - goal.x) < 0.001 &&
        Math.abs(start.y - goal.y) < 0.001
      )) {
        return {
          step,
          points:[
            {x:start.x,y:start.y,z:start.z},
            {x:goal.x,y:goal.y,z:goal.z}
          ],
          fallback:false,
          warning:null
        };
      }
    }

    return {
      step,
      points:[],
      fallback:true,
      warning:'A collision-free orthogonal bridge to the routing grid could not be found.'
    };
  }

  const startCell = {ix:startBridge.ix,iy:startBridge.iy,iz:startBridge.iz};
  const goalCell = networkMode
    ? null
    : {ix:goalBridge.ix,iy:goalBridge.iy,iz:goalBridge.iz};
  if (!networkMode) removeEndpointBlocks3D(blocked, startCell, goalCell);

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
    f:networkMode
      ? manhattanDistanceToBounds3D(
          {x:startCell.ix*step,y:startCell.iy*step,z:startCell.iz*step},
          networkBounds
        )
      : manhattanDistance3D(
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
      networkMode
        ? networkGoalKeys.has(pointKey3D(current.ix, current.iy, current.iz))
        : (
          current.ix === goalCell.ix &&
          current.iy === goalCell.iy &&
          current.iz === goalCell.iz
        )
    ) {
      goalState = current;
      break;
    }

    closed.add(currentKey);

    for (let i = 0; i < directions.length; i++) {
      const stepDir = directions[i];
      if (
        Number.isFinite(Number(options.fixedRoutingY)) &&
        (stepDir.dir === 1 || stepDir.dir === 4)
      ) continue;

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

      if (options.structurePrimaryAxis && (stepDir.dir === 0 || stepDir.dir === 2)) {
        const stepAxis = stepDir.dir === 0 ? 'x' : 'z';
        if (stepAxis !== options.structurePrimaryAxis) {
          moveCost += step * Math.max(0, Number(options.structureAxisPenaltyRatio) || 0);
        }
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

      const heuristic = networkMode
        ? manhattanDistanceToBounds3D(
            {x:nx*step,y:ny*step,z:nz*step},
            networkBounds
          )
        : manhattanDistance3D(
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
    if (!networkMode) {
      // Before reporting a failed A* search, retry a deterministic orthogonal
      // bridge. This removes false fallback warnings for simple valid routes
      // that do not need the full grid search.
      const orthogonalBridge = bridgePath3D(start, goal, obstacles, options);
      if (orthogonalBridge && orthogonalBridge.length >= 2) {
        return {
          step,
          points:orthogonalBridge,
          fallback:false,
          warning:null
        };
      }
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

  let attachmentPoint = null;
  if (networkMode) {
    const lastGridPoint = gridPoints[gridPoints.length - 1] || null;
    const lastKey = pointKey3D(goalState.ix, goalState.iy, goalState.iz);
    const mappedPoints = networkGoalMap.get(lastKey) || networkGoalPoints;
    const nearestAttachment = nearestNetworkPoint3D(
      lastGridPoint || start,
      mappedPoints
    );
    attachmentPoint = nearestAttachment ? nearestAttachment.point : null;

    if (!attachmentPoint) {
      return {
        step,
        points:[],
        fallback:true,
        warning:'The routing grid reached a network cell but could not resolve its exact tray-network attachment.'
      };
    }

    const currentPoint = points[points.length - 1];
    if (
      Math.abs(currentPoint.x - attachmentPoint.x) > 0.001 ||
      Math.abs(currentPoint.y - attachmentPoint.y) > 0.001 ||
      Math.abs(currentPoint.z - attachmentPoint.z) > 0.001
    ) {
      const attachmentBridge = bridgePath3D(
        currentPoint,
        attachmentPoint,
        obstacles,
        options
      );
      if (!attachmentBridge || attachmentBridge.length < 2) {
        return {
          step,
          points:[],
          fallback:true,
          warning:'A collision-free final connection to the existing tray network could not be found.'
        };
      }
      for (let i = 1; i < attachmentBridge.length; i++) points.push(attachmentBridge[i]);
    }
  } else {
    const goalBridgePoints = goalBridge.bridge;
    for (let i = goalBridgePoints.length - 2; i >= 0; i--) {
      points.push(goalBridgePoints[i]);
    }
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

  return {
    step,
    points:compressed,
    attachment_point:attachmentPoint,
    fallback:false,
    warning:null
  };
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

function planAxisForParallelMain(plan) {
  const axis = plan && plan.main_corridor_axis;
  return axis === 'x' || axis === 'z' ? axis : null;
}

function buildTrayRuns(cablePlans, options, mainCorridors) {
  const networkWidthByPanel = new Map();

  (cablePlans || []).forEach(function(plan) {
    const panelId = plan.panel && plan.panel.id;
    const width = Number(plan.planning_tray_width_mm);
    if (!panelId || !Number.isFinite(width) || width <= 0) return;
    networkWidthByPanel.set(
      panelId,
      Math.max(Number(networkWidthByPanel.get(panelId)) || 0, width)
    );
  });

  const segments = new Map();

  cablePlans.forEach(function(plan) {
    // The 1 m free-cable allowance applies only at the equipment end.
    // Keep the complete remaining cable path so the physical tray continues
    // through the shared network instead of stopping where branch_points end.
    const trayPoints = Array.isArray(plan.points)
      ? trimBranchTrayStart(
          plan,
          plan.points,
          Math.max(0, Number(options.trayStopBeforeEquipmentMm) || 1000)
        )
      : [];
    const points = Array.isArray(trayPoints) && trayPoints.length >= 2
      ? trayPoints
      : [];

    const mainLevelY = Number(plan.main_corridor_routing_y_mm);

    for (let i = 1; i < points.length; i++) {
      const segment = segmentRecord(
        points[i - 1],
        points[i],
        plan.equipment.id,
        plan.cable.diameter_mm
      );
      if (!segment) continue;

      segment.main_level_eligible = Number.isFinite(mainLevelY) &&
        Math.abs(segment.start.y - mainLevelY) < 0.001 &&
        Math.abs(segment.end.y - mainLevelY) < 0.001;

      const existing = segments.get(segment.key);
      if (!existing) {
        segments.set(segment.key, segment);
      } else {
        existing.cableIds.add(plan.equipment.id);
        existing.diameterByCable.set(
          plan.equipment.id,
          Number(plan.cable.diameter_mm) || 0
        );
        existing.main_level_eligible =
          existing.main_level_eligible || segment.main_level_eligible;
      }
    }
  });

  const trayHeightMm = Math.max(25, Number(options.trayHeightMm) || 100);
  const mainMinCables = Math.max(2, Number(options.mainMinCables) || 2);

  const classified = [];
  segments.forEach(function(segment) {
    const panelId = cablePlans.find(function(plan) {
      return plan.equipment &&
        segment.cableIds.has(plan.equipment.id);
    })?.panel?.id || null;

    const networkWidth = Number(networkWidthByPanel.get(panelId));
    const width = Number.isFinite(networkWidth) && networkWidth > 0
      ? networkWidth
      : chooseTrayWidth(
          Array.from(segment.diameterByCable.values()).reduce(function(sum, value) {
            return sum + (Number(value) || 0);
          }, Math.max(0, Number(options.traySideMarginMm) || 25) * 2) /
          (Math.min(100, Math.max(1, Number(options.fillLimitPercent) || 80)) / 100),
          options.standardTrayWidthsMm
        );

    const classification = (
      segment.cableIds.size >= mainMinCables &&
      segment.main_level_eligible
    ) ? 'main' : 'branch';

    classified.push({
      ...segment,
      classification,
      width_mm:width,
      height_mm:trayHeightMm,
      required_width_mm:width
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

  // One engineering network uses one physical tray width. This deliberately
  // removes the need for any reducer/transition between branch and main runs.
  return stitchTrayRuns(runs);
}

function trimBranchTrayStart(plan, branchPoints, stopDistanceMm) {
  const source = (branchPoints || []).map(clonePoint);
  if (source.length < 2) return [];

  const anchor = clonePoint(
    plan &&
    plan.equipment &&
    plan.equipment.anchor &&
    plan.equipment.anchor.point
      ? plan.equipment.anchor.point
      : source[0]
  );
  const limit = Math.max(0, Number(stopDistanceMm) || 0);
  if (limit <= 0) return source;

  const route = [anchor];
  source.forEach(function(point) {
    const previous = route[route.length - 1];
    if (
      Math.abs(previous.x - point.x) > 0.001 ||
      Math.abs(previous.y - point.y) > 0.001 ||
      Math.abs(previous.z - point.z) > 0.001
    ) {
      route.push(point);
    }
  });

  let remaining = limit;
  let accumulated = 0;

  for (let i = 1; i < route.length; i++) {
    const a = route[i - 1];
    const b = route[i];
    const length = manhattanDistance3D(a, b);
    if (length < 0.001) continue;

    if (accumulated + length >= limit - 0.001) {
      const ratio = Math.max(0, Math.min(1, (limit - accumulated) / length));
      const stopPoint = {
        x:a.x + (b.x - a.x) * ratio,
        y:a.y + (b.y - a.y) * ratio,
        z:a.z + (b.z - a.z) * ratio
      };

      const result = [stopPoint];
      for (let j = i; j < route.length; j++) {
        const point = route[j];
        const previous = result[result.length - 1];
        if (
          Math.abs(previous.x - point.x) > 0.001 ||
          Math.abs(previous.y - point.y) > 0.001 ||
          Math.abs(previous.z - point.z) > 0.001
        ) {
          result.push(clonePoint(point));
        }
      }
      return result.length >= 2 ? result : [];
    }

    accumulated += length;
  }

  // The main/branch route is shorter than the requested free cable allowance.
  // In that case no tray should be forced all the way to the equipment.
  return [];
}

function trayRunNodeKey(point) {
  return rounded(point.x) + '|' + rounded(point.y) + '|' + rounded(point.z);
}

function mergeTrayRunPath(items) {
  if (!items || !items.length) return null;

  const first = items[0];
  const cableIds = new Set();
  const points = [];

  items.forEach(function(item, index){
    (item.cable_ids || []).forEach(function(id){ cableIds.add(id); });
    (item.points || []).forEach(function(point, pointIndex){
      if (index === 0 || pointIndex > 0) points.push(clonePoint(point));
    });
  });

  return {
    classification:first.classification,
    width_mm:first.width_mm,
    height_mm:first.height_mm,
    cable_ids:Array.from(cableIds),
    points,
    length_m:routeLengthMeters(points)
  };
}

function stitchTrayRuns(runs) {
  const source = (runs || []).filter(function(run){
    return run && Array.isArray(run.points) && run.points.length >= 2;
  });
  const grouped = new Map();

  source.forEach(function(run){
    const groupKey = [
      run.classification || 'branch',
      Number(run.width_mm) || 0,
      Number(run.height_mm) || 0
    ].join('|');

    if (!grouped.has(groupKey)) grouped.set(groupKey, []);
    grouped.get(groupKey).push({
      run,
      startKey:trayRunNodeKey(run.points[0]),
      endKey:trayRunNodeKey(run.points[run.points.length - 1])
    });
  });

  const result = [];

  grouped.forEach(function(group){
    const adjacency = new Map();

    group.forEach(function(item, localIndex){
      item.localIndex = localIndex;
      [item.startKey, item.endKey].forEach(function(key){
        if (!adjacency.has(key)) adjacency.set(key, new Set());
        adjacency.get(key).add(localIndex);
      });
    });

    function degree(key) {
      const set = adjacency.get(key);
      return set ? set.size : 0;
    }

    const visited = new Set();

    function walk(startIndex, startNode) {
      const pathItems = [];
      let currentIndex = startIndex;
      let currentNode = startNode;

      while (currentIndex != null && !visited.has(currentIndex)) {
        const item = group[currentIndex];
        let oriented = item.run.points.map(clonePoint);
        if (item.endKey === currentNode) oriented.reverse();

        pathItems.push({
          ...item.run,
          points:oriented
        });
        visited.add(currentIndex);

        const nextNode = trayRunNodeKey(oriented[oriented.length - 1]);
        if (degree(nextNode) !== 2) break;

        const candidates = Array.from(adjacency.get(nextNode) || [])
          .filter(function(index){ return !visited.has(index); });
        if (!candidates.length) break;

        currentNode = nextNode;
        currentIndex = candidates[0];
      }

      return mergeTrayRunPath(pathItems);
    }

    group.forEach(function(item, localIndex){
      if (visited.has(localIndex)) return;

      const endpoints = [item.startKey, item.endKey]
        .filter(function(key){ return degree(key) !== 2; });

      if (!endpoints.length) return;

      const merged = walk(localIndex, endpoints[0]);
      if (merged && merged.points.length >= 2) result.push(merged);
    });

    group.forEach(function(item, localIndex){
      if (visited.has(localIndex)) return;
      const merged = walk(localIndex, item.startKey);
      if (merged && merged.points.length >= 2) result.push(merged);
    });
  });

  return result;
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

function approximateCorridorCoverageCount(corridor, entries, thresholdMm) {
  const threshold = Math.max(1, Number(thresholdMm) || 0);
  let count = 0;
  entries.forEach(function(entry){
    const nearest = closestPointOnPolyline3D(entry.start, corridor);
    if (nearest && nearest.distance <= threshold) count++;
  });
  return count;
}

function resolveHighestValidRoutingY(points, options) {
  const ceilingY = Number(options.ceilingY);
  if (!Number.isFinite(ceilingY) || !Array.isArray(points) || !points.length) return null;

  const trayHeight = Math.max(1, Number(options.trayHeightMm) || 100);
  const clearance = Math.max(0, Number(options.clearanceMm) || 0);
  const safetyGap = Math.max(0, Number(options.ceilingSafetyGapMm) || 0);
  const routingBounds = normalizeRoutingBounds(options.routingBounds, options);
  if (routingBounds && routingBounds.valid === false) return null;

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

function resolveCommonMainRoutingY(points, options) {
  if (!Array.isArray(points) || !points.length) return null;

  const bounds = normalizeRoutingBounds(options.routingBounds, options);
  if (!bounds || bounds.valid === false) return null;

  const step = Math.max(50, Number(options.gridStepMm) || 100);
  const trayHeight = Math.max(1, Number(options.trayHeightMm) || 100);
  const clearance = Math.max(0, Number(options.clearanceMm) || 0);
  const safetyGap = Math.max(0, Number(options.ceilingSafetyGapMm) || 0);
  const ceilingY = Number(options.ceilingY);
  if (!Number.isFinite(ceilingY)) return null;

  const desiredY = Math.min(
    ceilingY - safetyGap - trayHeight / 2 - clearance,
    bounds.maxY
  );

  const alignedStart = Math.floor(desiredY / step) * step;
  const alignedMin = Math.ceil(bounds.minY / step) * step;

  for (let y = alignedStart; y >= alignedMin - 0.001; y -= step) {
    let valid = true;

    for (let i = 0; i < points.length; i++) {
      const point = pointAtRoutingY(points[i], y);
      if (!pointWithinRoutingBounds(point, options) ||
          !pointWithinBodyDistanceForRouting(point, options)) {
        valid = false;
        break;
      }
    }

    if (valid) return y;
  }

  return null;
}


function resolveNetworkRoutingLevels(panelPoint, options, levelLimit) {
  if (!panelPoint) return [];
  const ceilingY = Number(options && options.ceilingY);
  if (!Number.isFinite(ceilingY)) return [];

  const bounds = normalizeRoutingBounds(options && options.routingBounds, options || {});
  if (bounds && bounds.valid === false) return [];

  const trayHeight = Math.max(1, Number(options.trayHeightMm) || 100);
  const clearance = Math.max(0, Number(options.clearanceMm) || 0);
  const safetyGap = Math.max(0, Number(options.ceilingSafetyGapMm) || 0);
  const step = Math.max(50, Number(options.gridStepMm) || 100);
  const desiredY = Math.min(
    ceilingY - safetyGap - trayHeight / 2 - clearance,
    bounds ? bounds.maxY : ceilingY
  );
  const alignedStart = Math.floor(desiredY / step) * step;
  const lowerY = bounds
    ? Math.ceil(bounds.minY / step) * step
    : Math.min(
        alignedStart - Math.max(1000, Number(options.maxBodyDistanceMm) || 1500),
        Number(panelPoint.y) || alignedStart
      );

  // A non-positive limit means "search every usable horizontal level".
  // The previous fixed three-level search could reject a perfectly valid
  // internal tray corridor simply because the upper structural braces occupied
  // the first three candidate elevations.
  const requestedLimit = Number(levelLimit);
  const limit = requestedLimit > 0
    ? Math.max(1, Math.floor(requestedLimit))
    : Infinity;

  const levels = [];

  for (let y = alignedStart; y >= lowerY - 0.001 && levels.length < limit; y -= step) {
    const candidate = pointAtRoutingY(panelPoint, y);
    if (!pointWithinRoutingBounds(candidate, options)) continue;
    if (!pointWithinBodyDistanceForRouting(candidate, options)) continue;
    levels.push(y);
  }

  return levels;
}

function networkNodeKey(point) {
  return [
    rounded(point.x),
    rounded(point.y),
    rounded(point.z)
  ].join('|');
}

function networkDistanceToNodes(point, nodes) {
  let best = Infinity;
  nodes.forEach(function(candidate) {
    best = Math.min(best, manhattanDistance3D(point, candidate));
  });
  return best;
}

function registerNetworkPath(tree, pathPoints, routingY) {
  const fixed = (pathPoints || [])
    .filter(function(point){
      return Math.abs(Number(point.y) - Number(routingY)) < 0.001;
    })
    .map(clonePoint);

  if (!fixed.length) return null;

  const attachmentPoint = fixed[fixed.length - 1];
  const attachmentKey = networkNodeKey(attachmentPoint);
  if (!tree.nodes.has(attachmentKey)) return null;

  let childKey = attachmentKey;
  for (let i = fixed.length - 2; i >= 0; i--) {
    const point = fixed[i];
    const key = networkNodeKey(point);
    if (!tree.nodes.has(key)) {
      tree.nodes.set(key, point);
      tree.parent.set(key, childKey);
    }
    childKey = key;
  }

  return {
    attachmentPoint,
    fixedPoints:fixed
  };
}

function traceNetworkToRoot(tree, attachmentPoint) {
  const points = [];
  let key = networkNodeKey(attachmentPoint);
  let guard = 0;

  while (key && tree.nodes.has(key) && guard++ < 100000) {
    points.push(clonePoint(tree.nodes.get(key)));
    key = tree.parent.get(key) || null;
  }

  if (!points.length || key !== null) return [];
  return points;
}

function appendDistinctPoints(target, source, startIndex) {
  for (let i = Math.max(0, Number(startIndex) || 0); i < (source || []).length; i++) {
    const point = clonePoint(source[i]);
    const previous = target[target.length - 1];
    if (
      previous &&
      Math.abs(previous.x - point.x) < 0.001 &&
      Math.abs(previous.y - point.y) < 0.001 &&
      Math.abs(previous.z - point.z) < 0.001
    ) continue;
    target.push(point);
  }
}

function makeNetworkCablePlan(entry, group, branchPath, tree, attachmentPoint, panelDrop, planningTrayWidth, options) {
  const treePath = traceNetworkToRoot(tree, attachmentPoint);
  if (!treePath.length) return null;

  const points = [];
  appendDistinctPoints(points, [entry.equipment.anchor.point]);
  appendDistinctPoints(points, [entry.start]);
  appendDistinctPoints(points, branchPath, 0);
  appendDistinctPoints(points, treePath, 1);
  for (let i = (panelDrop.points || []).length - 2; i >= 0; i--) {
    appendDistinctPoints(points, [panelDrop.points[i]]);
  }
  appendDistinctPoints(points, [group.panel.anchor.point]);

  return {
    equipment:entry.equipment,
    panel:group.panel,
    cable:{
      name:entry.equipment.cable_name || 'Power Cable',
      diameter_mm:Number(entry.equipment.cable_diameter_mm) || 0
    },
    routing_start:entry.start,
    routing_goal:attachmentPoint,
    main_corridor_equipment_id:null,
    main_corridor_routing_y_mm:Number(options.fixedRoutingY),
    standoff_distance_mm:Math.max(entry.startDistanceMm, options.panelStandoffDistanceMm || 0),
    planning_tray_width_mm:planningTrayWidth,
    branch_points:(branchPath || []).map(clonePoint),
    points,
    warning:null,
    fallback:false
  };
}

function clusterParallelAxisLevels(values, toleranceMm) {
  const source = (values || [])
    .map(function(value){ return Number(value); })
    .filter(Number.isFinite)
    .sort(function(a,b){ return a - b; });

  if (!source.length) return [];

  const tolerance = Math.max(1, Number(toleranceMm) || 1);
  const clusters = [];

  source.forEach(function(value){
    const current = clusters[clusters.length - 1];
    if (!current || Math.abs(value - current.max) > tolerance) {
      clusters.push({min:value, max:value, sum:value, count:1});
      return;
    }

    current.max = value;
    current.sum += value;
    current.count += 1;
  });

  return clusters.map(function(cluster){
    return cluster.sum / cluster.count;
  });
}

function resolveParallelMainCorridorCandidates(prepared, options) {
  if (!prepared || prepared.length < 2) return [];

  const gridStep = Math.max(50, Number(options && options.gridStepMm) || 100);
  const primaryAxis = options && (options.structurePrimaryAxis === 'x' || options.structurePrimaryAxis === 'z')
    ? options.structurePrimaryAxis
    : (() => {
        const xs = prepared.map(function(entry){ return Number(entry.start.x); });
        const zs = prepared.map(function(entry){ return Number(entry.start.z); });
        const xSpan = Math.max.apply(null, xs) - Math.min.apply(null, xs);
        const zSpan = Math.max.apply(null, zs) - Math.min.apply(null, zs);
        return xSpan >= zSpan ? 'x' : 'z';
      })();

  const primaryValues = prepared.map(function(entry){
    return Number(entry.start[primaryAxis]);
  });
  const secondaryAxis = primaryAxis === 'x' ? 'z' : 'x';
  const secondaryValues = prepared.map(function(entry){
    return Number(entry.start[secondaryAxis]);
  });

  const primaryMin = Math.min.apply(null, primaryValues);
  const primaryMax = Math.max.apply(null, primaryValues);
  const secondaryLevels = clusterParallelAxisLevels(
    secondaryValues,
    Math.max(gridStep * 1.5, 50)
  );

  const primarySpan = primaryMax - primaryMin;
  const secondarySpan = secondaryLevels.length >= 2
    ? secondaryLevels[secondaryLevels.length - 1] - secondaryLevels[0]
    : 0;

  // Only activate this special topology when the loads visibly form two or
  // more long, parallel rows. It must not replace ordinary network routing.
  if (
    secondaryLevels.length < 2 ||
    primarySpan < Math.max(gridStep * 6, secondarySpan * 1.5)
  ) {
    return [];
  }

  const candidates = [];
  for (let i = 0; i < secondaryLevels.length - 1; i++) {
    const low = secondaryLevels[i];
    const high = secondaryLevels[i + 1];
    const gap = high - low;
    if (gap < Math.max(gridStep * 2, 100)) continue;

    const secondary = (low + high) * 0.5;
    const lowerCount = secondaryValues.filter(function(value){ return value < secondary; }).length;
    const upperCount = secondaryValues.length - lowerCount;
    const balance = Math.abs(lowerCount - upperCount);
    const cableDistance = secondaryValues.reduce(function(sum, value){
      return sum + Math.abs(value - secondary);
    }, 0);

    candidates.push({
      primaryAxis,
      secondaryAxis,
      primaryMin,
      primaryMax,
      secondary,
      balance,
      cableDistance
    });
  }

  return candidates.sort(function(a,b){
    return a.balance - b.balance ||
      a.cableDistance - b.cableDistance ||
      a.secondary - b.secondary;
  }).slice(
    0,
    Math.max(1, Number(options && options.parallelMainCorridorCandidateLimit) || 6)
  );
}

function buildParallelMainCorridorNetwork(group, prepared, panelStandoff, obstacles, routeOptions, options, routingY, panelDrop) {
  const candidates = resolveParallelMainCorridorCandidates(prepared, options);
  if (!candidates.length || !panelDrop || !panelDrop.points || !panelDrop.points.length) {
    return null;
  }

  let best = null;

  candidates.forEach(function(candidate){
    const nearCoordinate = candidate.primaryAxis === 'x'
      ? {
          x:Math.min(
            candidate.primaryMax,
            Math.max(candidate.primaryMin, Number(panelStandoff.point.x))
          ),
          y:routingY,
          z:candidate.secondary
        }
      : {
          x:candidate.secondary,
          y:routingY,
          z:Math.min(
            candidate.primaryMax,
            Math.max(candidate.primaryMin, Number(panelStandoff.point.z))
          )
        };

    const farCoordinate = candidate.primaryAxis === 'x'
      ? {
          x:Math.abs(Number(panelStandoff.point.x) - candidate.primaryMin) <=
            Math.abs(Number(panelStandoff.point.x) - candidate.primaryMax)
            ? candidate.primaryMax
            : candidate.primaryMin,
          y:routingY,
          z:candidate.secondary
        }
      : {
          x:candidate.secondary,
          y:routingY,
          z:Math.abs(Number(panelStandoff.point.z) - candidate.primaryMin) <=
            Math.abs(Number(panelStandoff.point.z) - candidate.primaryMax)
            ? candidate.primaryMax
            : candidate.primaryMin
        };

    const nearPoint = clonePoint(nearCoordinate);
    const farPoint = clonePoint(farCoordinate);

    const spinePath = findGridPath3D(
      farPoint,
      nearPoint,
      obstacles,
      {
        ...routeOptions,
        fixedRoutingY:routingY,
        preferredRoutingY:routingY,
        routeTurnPenaltyRatio:Math.max(
          Number(options.turnPenaltyRatio) || 20,
          Number(options.mainCorridorTurnPenaltyRatio) || 100
        )
      },
      new Set()
    );

    if (!spinePath.points || spinePath.points.length < 2) return;

    const panelConnector = findGridPath3D(
      nearPoint,
      pointAtRoutingY(panelStandoff.point, routingY),
      obstacles,
      {
        ...routeOptions,
        fixedRoutingY:routingY,
        preferredRoutingY:routingY,
        routeTurnPenaltyRatio:Math.max(
          Number(options.turnPenaltyRatio) || 20,
          Number(options.mainCorridorTurnPenaltyRatio) || 100
        )
      },
      new Set()
    );

    if (!panelConnector.points || panelConnector.points.length < 2) return;

    const corridorPath = [];
    appendDistinctPoints(corridorPath, spinePath.points, 0);
    appendDistinctPoints(corridorPath, panelConnector.points, 1);

    const rootPoint = pointAtRoutingY(panelStandoff.point, routingY);
    const rootKey = networkNodeKey(rootPoint);
    const tree = {
      nodes:new Map([[rootKey, clonePoint(rootPoint)]]),
      parent:new Map([[rootKey, null]])
    };

    const registration = registerNetworkPath(tree, corridorPath, routingY);
    if (!registration) return;

    const pending = prepared.slice();
    const cablePlans = [];

    while (pending.length) {
      const networkPoints = Array.from(tree.nodes.values());
      let selected = null;
      const probeCount = Math.min(
        Math.max(1, Number(options.networkAttachmentCandidateLimit) || 4),
        pending.length
      );

      const rankedPending = pending.slice().sort(function(a,b){
        return networkDistanceToNodes(a.start, networkPoints) -
          networkDistanceToNodes(b.start, networkPoints) ||
          b.densityScore - a.densityScore;
      });

      for (let candidateIndex = 0; candidateIndex < probeCount; candidateIndex++) {
        const entry = rankedPending[candidateIndex];

        const branchResult = findGridPath3D(
          entry.start,
          null,
          obstacles,
          {
            ...routeOptions,
            fixedRoutingY:routingY,
            preferredRoutingY:routingY,
            routeTurnPenaltyRatio:Math.max(
              Number(options.turnPenaltyRatio) || 20,
              Number(options.mainCorridorTurnPenaltyRatio) || 100
            ),
            networkGoalPoints:networkPoints
          },
          new Set()
        );

        if (
          !branchResult.points ||
          branchResult.points.length < 1 ||
          !branchResult.attachment_point
        ) {
          continue;
        }

        const score =
          polylineLengthMm(branchResult.points) +
          countPolylineTurns(branchResult.points) *
          Math.max(1, Number(options.gridStepMm) || 100) *
          50;

        if (!selected || score < selected.score) {
          selected = {
            entry,
            branchResult,
            score
          };
        }
      }

      if (!selected) return;

      const attached = registerNetworkPath(
        tree,
        selected.branchResult.points,
        routingY
      );
      if (!attached) return;

      const plan = makeNetworkCablePlan(
        selected.entry,
        group,
        selected.branchResult.points,
        tree,
        attached.attachmentPoint,
        panelDrop,
        routeOptions.routingTrayWidthMm,
        {
          ...routeOptions,
          fixedRoutingY:routingY,
          panelStandoffDistanceMm:panelStandoff.distance_mm
        }
      );
      if (!plan) return;

      plan.parallel_main_corridor = true;
      plan.main_corridor_axis = candidate.primaryAxis;
      plan.main_corridor_secondary_coordinate_mm = candidate.secondary;
      cablePlans.push(plan);

      const removeIndex = pending.findIndex(function(entry){
        return entry.equipment.id === selected.entry.equipment.id;
      });
      if (removeIndex < 0) return;
      pending.splice(removeIndex, 1);
    }

    const totalLength = cablePlans.reduce(function(sum, plan){
      return sum + routeLengthMeters(plan.points);
    }, 0);
    const totalTurns = cablePlans.reduce(function(sum, plan){
      return sum + countPolylineTurns(plan.points);
    }, 0);
    const score =
      totalLength +
      totalTurns * Math.max(1, Number(options.gridStepMm) || 100) * 0.001;

    if (
      !best ||
      score < best.score
    ) {
      best = {
        routingY,
        panelHighPoint:pointAtRoutingY(panelStandoff.point, routingY),
        panelDrop,
        cablePlans,
        connectedCount:cablePlans.length,
        unresolved:[],
        networkNodes:Array.from(tree.nodes.values()).map(clonePoint),
        score
      };
    }
  });

  return best;
}

function buildPanelMultiTerminalNetwork(group, prepared, panelStandoff, obstacles, routeOptions, options) {
  if (!prepared.length) return null;

  prepared.forEach(function(entry) {
    entry.densityScore = equipmentDensityScore(
      entry,
      prepared,
      Math.max(
        Number(options.gridStepMm) * 8,
        Number(options.maxBodyDistanceMm) * 0.75,
        1000
      )
    );
  });

  const seedCandidates = prepared.slice().sort(function(a,b){
    return b.densityScore - a.densityScore ||
      manhattanDistance3D(a.start, panelStandoff.point) -
      manhattanDistance3D(b.start, panelStandoff.point);
  });

  const requestedLevelAttempts = Number(options.networkLevelAttempts);
  const levels = resolveNetworkRoutingLevels(
    panelStandoff.point,
    routeOptions,
    Number.isFinite(requestedLevelAttempts) && requestedLevelAttempts > 0
      ? requestedLevelAttempts
      : 0
  );
  const seedLimit = Math.min(
    seedCandidates.length,
    Math.max(1, Number(options.networkSeedAttempts) || 4)
  );
  const attachLimit = Math.max(1, Number(options.networkAttachmentCandidateLimit) || 4);

  let bestPartial = null;

  // When the equipment clearly forms parallel rows, try a dedicated shared
  // Main Tray corridor between the rows before the generic seed-and-branch
  // network. This prevents two parallel cable paths from becoming two
  // independent Main Trays when the open space between them is usable.
  for (let levelIndex = 0; levelIndex < levels.length; levelIndex++) {
    const routingY = levels[levelIndex];
    const panelHighPoint = pointAtRoutingY(panelStandoff.point, routingY);
    const panelDrop = findGridPath3D(
      panelStandoff.point,
      panelHighPoint,
      obstacles,
      {
        ...routeOptions,
        fixedRoutingY:routingY,
        preferredRoutingY:routingY,
        routeTurnPenaltyRatio:Math.max(
          Number(options.turnPenaltyRatio) || 20,
          Number(options.mainCorridorTurnPenaltyRatio) || 100
        )
      },
      new Set()
    );

    if (!panelDrop.points || panelDrop.points.length < 1) continue;

    const parallelResult = buildParallelMainCorridorNetwork(
      group,
      prepared,
      panelStandoff,
      obstacles,
      routeOptions,
      options,
      routingY,
      panelDrop
    );

    if (parallelResult && parallelResult.unresolved.length === 0) {
      return parallelResult;
    }
  }

  for (let levelIndex = 0; levelIndex < levels.length; levelIndex++) {
    const routingY = levels[levelIndex];
    const panelHighPoint = pointAtRoutingY(panelStandoff.point, routingY);

    const panelDrop = findGridPath3D(
      panelStandoff.point,
      panelHighPoint,
      obstacles,
      {
        ...routeOptions,
        fixedRoutingY:routingY,
        preferredRoutingY:routingY,
        routeTurnPenaltyRatio:Math.max(
          Number(options.turnPenaltyRatio) || 20,
          Number(options.mainCorridorTurnPenaltyRatio) || 100
        )
      },
      new Set()
    );

    if (!panelDrop.points || panelDrop.points.length < 1) continue;

    for (let seedIndex = 0; seedIndex < seedLimit; seedIndex++) {
      const seed = seedCandidates[seedIndex];

      const seedPath = findGridPath3D(
        seed.start,
        panelHighPoint,
        obstacles,
        {
          ...routeOptions,
          fixedRoutingY:routingY,
          preferredRoutingY:routingY,
          routeTurnPenaltyRatio:Math.max(
            Number(options.turnPenaltyRatio) || 20,
            Number(options.mainCorridorTurnPenaltyRatio) || 100
          )
        },
        new Set()
      );

      if (!seedPath.points || seedPath.points.length < 2) continue;

      const rootKey = networkNodeKey(panelHighPoint);
      const tree = {
        nodes:new Map([[rootKey, clonePoint(panelHighPoint)]]),
        parent:new Map([[rootKey, null]])
      };

      const seedRegistration = registerNetworkPath(
        tree,
        seedPath.points,
        routingY
      );
      if (!seedRegistration) continue;

      const cablePlans = [];
      const connected = new Set([seed.equipment.id]);

      const seedPlan = makeNetworkCablePlan(
        seed,
        group,
        seedPath.points,
        tree,
        seedRegistration.attachmentPoint,
        panelDrop,
        routeOptions.routingTrayWidthMm,
        {
          ...routeOptions,
          fixedRoutingY:routingY,
          panelStandoffDistanceMm:panelStandoff.distance_mm
        }
      );
      if (!seedPlan) continue;
      cablePlans.push(seedPlan);

      let pending = prepared.filter(function(entry){
        return !connected.has(entry.equipment.id);
      });
      let madeProgress = true;

      while (pending.length && madeProgress) {
        madeProgress = false;

        const networkPoints = Array.from(tree.nodes.values());
        pending.sort(function(a,b){
          return networkDistanceToNodes(a.start, networkPoints) -
            networkDistanceToNodes(b.start, networkPoints) ||
            b.densityScore - a.densityScore;
        });

        let selected = null;
        const probeCount = Math.min(attachLimit, pending.length);

        for (let candidateIndex = 0; candidateIndex < probeCount; candidateIndex++) {
          const candidate = pending[candidateIndex];

          const branchResult = findGridPath3D(
            candidate.start,
            null,
            obstacles,
            {
              ...routeOptions,
              fixedRoutingY:routingY,
              preferredRoutingY:routingY,
              routeTurnPenaltyRatio:Math.max(
                Number(options.turnPenaltyRatio) || 20,
                Number(options.mainCorridorTurnPenaltyRatio) || 100
              ),
              networkGoalPoints:networkPoints
            },
            new Set()
          );

          if (!branchResult.points || !branchResult.points.length || !branchResult.attachment_point) {
            continue;
          }

          const score =
            polylineLengthMm(branchResult.points) +
            countPolylineTurns(branchResult.points) *
            Math.max(1, Number(options.gridStepMm) || 100) *
            50;

          if (!selected || score < selected.score) {
            selected = {
              entry:candidate,
              branchResult,
              score
            };
          }
        }

        if (!selected) break;

        const registration = registerNetworkPath(
          tree,
          selected.branchResult.points,
          routingY
        );
        if (!registration) break;

        const plan = makeNetworkCablePlan(
          selected.entry,
          group,
          selected.branchResult.points,
          tree,
          registration.attachmentPoint,
          panelDrop,
          routeOptions.routingTrayWidthMm,
          {
            ...routeOptions,
            fixedRoutingY:routingY,
            panelStandoffDistanceMm:panelStandoff.distance_mm
          }
        );
        if (!plan) break;

        cablePlans.push(plan);
        connected.add(selected.entry.equipment.id);
        pending = pending.filter(function(entry){
          return entry.equipment.id !== selected.entry.equipment.id;
        });
        madeProgress = true;
      }

      const result = {
        routingY,
        panelHighPoint,
        panelDrop,
        cablePlans,
        connectedCount:cablePlans.length,
        unresolved:prepared.filter(function(entry){
          return !connected.has(entry.equipment.id);
        }),
        networkNodes:Array.from(tree.nodes.values()).map(clonePoint)
      };

      const partialScore =
        result.connectedCount * 100000000 -
        cablePlans.reduce(function(sum, plan){ return sum + routeLengthMeters(plan.points); }, 0);

      if (
        !bestPartial ||
        result.connectedCount > bestPartial.connectedCount ||
        (
          result.connectedCount === bestPartial.connectedCount &&
          partialScore > bestPartial.partialScore
        )
      ) {
        bestPartial = { ...result, partialScore };
      }

      if (result.unresolved.length === 0) {
        return result;
      }
    }
  }

  return bestPartial;
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
    structurePrimaryAxis:(inputs && inputs.options && ['x','z'].indexOf(inputs.options.structurePrimaryAxis) >= 0)
      ? inputs.options.structurePrimaryAxis
      : null,
    structureAxisPenaltyRatio:Number(inputs && inputs.options && inputs.options.structureAxisPenaltyRatio) || 0.35,
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
    networkLevelAttempts:Number.isFinite(Number(inputs && inputs.options && inputs.options.networkLevelAttempts))
      ? Number(inputs.options.networkLevelAttempts)
      : 0,
    networkSeedAttempts:Number(inputs && inputs.options && inputs.options.networkSeedAttempts) || 4,
    networkAttachmentCandidateLimit:Number(inputs && inputs.options && inputs.options.networkAttachmentCandidateLimit) || 4,
    parallelMainCorridorCandidateLimit:Number(inputs && inputs.options && inputs.options.parallelMainCorridorCandidateLimit) || 6,
    trayStopBeforeEquipmentMm:Number.isFinite(Number(inputs && inputs.options && inputs.options.trayStopBeforeEquipmentMm))
      ? Number(inputs.options.trayStopBeforeEquipmentMm)
      : 1000,
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
      routingTrayWidthMm:planningTrayWidth,
      preferredStandoffDistanceMm:Math.min(
        options.maxBodyDistanceMm + planningTrayWidth / 2,
        options.clearanceMm + planningTrayWidth / 2 + options.gridStepMm
      )
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
    if (prepared.length < Math.max(2, Number(options.mainMinCables) || 2)) {
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
        highRoute.points.forEach(function(point){ branchPoints.push(clonePoint(point)); });
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
        appendDistinctPoints(points, [entry.equipment.anchor.point]);
        appendDistinctPoints(points, [entry.start]);
        appendDistinctPoints(points, branchPoints, 0);
        for (let i = panelDrop.points.length - 2; i >= 0; i--) {
          appendDistinctPoints(points, [panelDrop.points[i]]);
        }
        appendDistinctPoints(points, [group.panel.anchor.point]);

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
          main_corridor_routing_y_mm:Number(routingY),
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

    const networkResult = buildPanelMultiTerminalNetwork(
      group,
      prepared,
      panelStandoff,
      obstacles,
      routeOptions,
      options
    );

    if (!networkResult) {
      warnings.push(
        group.panel.name + ': no collision-free multi-terminal tray network could be built inside the configured body-distance corridor.'
      );
      return;
    }

    networkResult.cablePlans.forEach(function(plan) {
      cablePlans.push(plan);
    });

    if (networkResult.unresolved.length) {
      networkResult.unresolved.forEach(function(entry) {
        warnings.push(
          entry.equipment.name + ' → ' + group.panel.name +
          ': no collision-free branch to the shared tray network was found.'
        );
      });
    }

    mainCorridors.push({
      panel_id:group.panel.id,
      width_mm:planningTrayWidth,
      height_mm:Number(options.trayHeightMm) || 100,
      cable_ids:prepared.map(function(entry){ return entry.equipment.id; }),
      points:[clonePoint(networkResult.panelHighPoint)],
      network_nodes:networkResult.networkNodes || [],
      panel_drop_points:(networkResult.panelDrop.points || []).map(clonePoint),
      main_level_y_mm:Number(networkResult.routingY)
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
      routing_method:'multi_terminal_obstacle_avoiding_steiner_astar',
      routing_priority:'shared_network_then_minimum_turns_then_length'
    }
  };
}
