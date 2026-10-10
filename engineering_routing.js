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

function normalizeBounds3D(start, goal, obstacles, step, paddingMm, extraPoints) {
  const xs = [start.x, goal.x];
  const ys = [start.y, goal.y];
  const zs = [start.z, goal.z];

  (extraPoints || []).forEach(function(point) {
    if (!point) return;
    const x = Number(point.x);
    const y = Number(point.y);
    const z = Number(point.z);
    if (Number.isFinite(x)) xs.push(x);
    if (Number.isFinite(y)) ys.push(y);
    if (Number.isFinite(z)) zs.push(z);
  });

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

function adaptGridStep3D(start, goal, obstacles, baseStep, maxCells, paddingMm, extraPoints) {
  let step = Math.max(50, Number(baseStep) || 250);

  for (let attempt = 0; attempt < 8; attempt++) {
    const bounds = normalizeBounds3D(start, goal, obstacles, step, paddingMm, extraPoints);
    const width = bounds.maxIx - bounds.minIx + 1;
    const height = bounds.maxIy - bounds.minIy + 1;
    const depth = bounds.maxIz - bounds.minIz + 1;
    if (width * height * depth <= maxCells) return { step, bounds };

    const scale = Math.cbrt((width * height * depth) / maxCells);
    step = Math.max(step + 50, Math.ceil(step * scale / 50) * 50);
  }

  return {
    step,
    bounds: normalizeBounds3D(start, goal, obstacles, step, paddingMm, extraPoints)
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

function segmentSegmentDistance3D(p1,q1,p2,q2) {
  const d1={x:q1.x-p1.x,y:q1.y-p1.y,z:q1.z-p1.z};
  const d2={x:q2.x-p2.x,y:q2.y-p2.y,z:q2.z-p2.z};
  const r={x:p1.x-p2.x,y:p1.y-p2.y,z:p1.z-p2.z};
  const dot=(a,b)=>a.x*b.x+a.y*b.y+a.z*b.z;
  const a=dot(d1,d1),e=dot(d2,d2),f=dot(d2,r);
  let s=0,t=0;
  if(a<=1e-9&&e<=1e-9){
    return Math.sqrt(dot(r,r));
  }
  if(a<=1e-9){
    s=0;t=Math.max(0,Math.min(1,f/e));
  }else{
    const c=dot(d1,r);
    if(e<=1e-9){
      t=0;s=Math.max(0,Math.min(1,-c/a));
    }else{
      const b=dot(d1,d2),denom=a*e-b*b;
      s=denom>1e-9?Math.max(0,Math.min(1,(b*f-c*e)/denom)):0;
      t=(b*s+f)/e;
      if(t<0){t=0;s=Math.max(0,Math.min(1,-c/a));}
      else if(t>1){t=1;s=Math.max(0,Math.min(1,(b-c)/a));}
    }
  }
  const c1={x:p1.x+d1.x*s,y:p1.y+d1.y*s,z:p1.z+d1.z*s};
  const c2={x:p2.x+d2.x*t,y:p2.y+d2.y*t,z:p2.z+d2.z*t};
  return Math.sqrt(
    Math.pow(c1.x-c2.x,2)+
    Math.pow(c1.y-c2.y,2)+
    Math.pow(c1.z-c2.z,2)
  );
}

function clearsExistingMainTrays(a,b,options) {
  const corridors=Array.isArray(options&&options.occupiedMainCorridors)?options.occupiedMainCorridors:[];
  if(!corridors.length)return true;
  const trayWidth=Math.max(1,Number(options.routingTrayWidthMm)||Number(options.trayHeightMm)||100);
  const clearance=Math.max(0,Number(options.clearanceMm)||0);
  for(let i=0;i<corridors.length;i++){
    const corridor=corridors[i];
    const points=Array.isArray(corridor.points)?corridor.points:[];
    if(points.length<2)continue;
    const oldWidth=Math.max(1,Number(corridor.width_mm)||Number(corridor.height_mm)||100);
    const separation=(trayWidth+oldWidth)/2+clearance;
    for(let j=1;j<points.length;j++){
      if(segmentSegmentDistance3D(a,b,points[j-1],points[j])<separation-0.001)return false;
    }
  }
  return true;
}

function segmentClearForRouting(a, b, obstacles, options) {
  if (!segmentWithinRoutingBounds(a, b, options)) return false;
  if (!segmentWithinBodyDistanceForRouting(a, b, options)) return false;

  const clear = typeof options.segmentClear === 'function'
    ? options.segmentClear(
        a,
        b,
        {
          trayWidthMm:Number(options.routingTrayWidthMm) || Number(options.trayHeightMm) || 100,
          trayHeightMm:Number(options.trayHeightMm) || 100,
          bodyClearanceMm:Number(options.clearanceMm) || 0
        }
      )
    : segmentClear3D(
        a,
        b,
        obstacles,
        Number(options.centerlineClearanceMm) || Number(options.clearanceMm) || 0
      );
  if(!clear)return false;
  return clearsExistingMainTrays(a,b,options);
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

  // Tray width is the horizontal cross-section and must only reduce the
  // available X/Z envelope. Tray height is the vertical cross-section and
  // must only reduce the Y envelope. Applying the full tray width to Y
  // incorrectly makes the routing envelope collapse as cable count grows.
  const horizontalInset = trayWidth / 2 + clearance;
  const verticalInset = trayHeight / 2 + clearance;

  const minX = Number(bounds.minX);
  const maxX = Number(bounds.maxX);
  const minY = Number(bounds.minY);
  const maxY = Number(bounds.maxY);
  const minZ = Number(bounds.minZ);
  const maxZ = Number(bounds.maxZ);

  if (![minX,maxX,minY,maxY,minZ,maxZ].every(Number.isFinite)) return null;

  const normalized = {
    minX:minX + horizontalInset,
    maxX:maxX - horizontalInset,
    minY:minY + verticalInset,
    maxY:maxY - verticalInset,
    minZ:minZ + horizontalInset,
    maxZ:maxZ - horizontalInset,
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

function selectNetworkGoalPoints(start, networkPoints, candidateLimit) {
  const source = (networkPoints || [])
    .filter(function(point){
      return point &&
        Number.isFinite(Number(point.x)) &&
        Number.isFinite(Number(point.y)) &&
        Number.isFinite(Number(point.z));
    });

  if (!source.length) return [];

  const limit = Math.max(4, Math.floor(Number(candidateLimit) || 24));
  if (source.length <= limit) return source.map(clonePoint);

  return source
    .map(function(point){
      return {
        point:clonePoint(point),
        distance:manhattanDistance3D(start, point)
      };
    })
    .sort(function(a,b){
      return a.distance - b.distance;
    })
    .slice(0, limit)
    .map(function(entry){ return entry.point; });
}

function findGridPath3D(start, goal, obstacles, options, reuseCells) {
  const rawNetworkGoalPoints = Array.isArray(options && options.networkGoalPoints)
    ? options.networkGoalPoints.filter(Boolean)
    : [];
  const networkMode = rawNetworkGoalPoints.length > 0;
  const networkGoalPoints = networkMode
    ? selectNetworkGoalPoints(
        start,
        rawNetworkGoalPoints,
        Number(options && options.networkGoalCandidateLimit) || 24
      )
    : [];

  if (networkMode && !networkGoalPoints.length) {
    return {
      step:Math.max(50, Number(options.gridStepMm) || 250),
      points:[],
      fallback:true,
      warning:'No usable Main Tray attachment nodes were available.'
    };
  }

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
  const adapted = adaptGridStep3D(
    start,
    goal,
    obstacles,
    baseStep,
    maxCells,
    paddingMm,
    networkMode ? networkGoalPoints : null
  );
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
  // Preserve a straight, slightly sloped Main Tray in the horizontal XZ plane.
  if (dy < 0.001 && dx > 0.001 && dz > 0.001) return 'xz';
  if (dx >= dy && dx >= dz) return 'x';
  if (dy >= dx && dy >= dz) return 'y';
  return 'z';
}

function rounded(value) {
  return Math.round(Number(value) * 100) / 100;
}

function roundedTo(value, decimalPlaces) {
  const factor = Math.pow(10, Math.max(0, Number(decimalPlaces) || 0));
  return Math.round(Number(value) * factor) / factor;
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
  } else if (axis === 'xz') {
    const slope = roundedTo((end.z - start.z) / (end.x - start.x), 5);
    const intercept = roundedTo(start.z - slope * start.x, 0);
    key = 'xz|' + rounded(start.y) + '|' + slope + '|' + intercept + '|' + rounded(minPoint.x) + '|' + rounded(maxPoint.x);
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
  const networkWidthById = new Map();

  (cablePlans || []).forEach(function(plan) {
    const networkId = plan.engineering_network_id ||
      (plan.panel && plan.panel.id ? String(plan.panel.id) : null);
    const width = Number(plan.planning_tray_width_mm);
    if (!networkId || !Number.isFinite(width) || width <= 0) return;
    networkWidthById.set(
      networkId,
      Math.max(Number(networkWidthById.get(networkId)) || 0, width)
    );
  });

  const rawSegments = [];

  cablePlans.forEach(function(plan) {
    // The 1 m free-cable allowance applies only at the equipment end.
    // Keep the complete remaining cable path so the physical tray continues
    // through the shared network instead of stopping where branch_points end.
    const configuredEquipmentStop = Number.isFinite(Number(options.trayStopBeforeEquipmentMm))
      ? Math.max(0, Number(options.trayStopBeforeEquipmentMm))
      : 1000;
    // A reference-style shared network keeps the branch tray close to the
    // equipment; retain the larger free-cable allowance for other route types.
    const equipmentStop = plan.parallel_main_corridor === true
      ? Math.min(configuredEquipmentStop, Math.max(75, Math.min(110, Number(options.trayHeightMm) || 100)))
      : configuredEquipmentStop;
    let trayPoints = Array.isArray(plan.points)
      ? trimBranchTrayStart(plan, plan.points, equipmentStop)
      : [];

    // For the dedicated parallel Main topology, draw only the physical
    // equipment branch here. The shared Main and panel branch are added once
    // from mainCorridors below; otherwise each cable duplicates the spine.
    if (plan.parallel_main_corridor === true) {
      const branchOnly = clipPolylineAtPoint(trayPoints, plan.routing_goal, 1);
      if (branchOnly.length >= 2) {
        const requestedGap = Math.max(100, Math.min(220, (Number(options.gridStepMm) || 100) * 1.5));
        const availableLength = polylineLengthMm(branchOnly);
        const gap = Math.min(requestedGap, Math.max(0, availableLength - 50));
        trayPoints = gap > 0 ? shortenPolylineEnd(branchOnly, gap) : branchOnly;
      } else {
        trayPoints = [];
      }
    }

    const points = Array.isArray(trayPoints) && trayPoints.length >= 2
      ? trayPoints
      : [];

    const mainLevelY = Number(plan.main_corridor_routing_y_mm);

    for (let i = 1; i < points.length; i++) {
      const pointA = points[i - 1];
      const pointB = points[i];

      // Physical trays stay on the horizontal routing plane. Vertical or
      // diagonal cable drops connect this plane to equipment/panels but do
      // not become 3D tray transitions.
      if (
        Math.abs(pointA.y - pointB.y) > 0.001 ||
        (
          Number.isFinite(mainLevelY) &&
          (
            Math.abs(pointA.y - mainLevelY) > 0.001 ||
            Math.abs(pointB.y - mainLevelY) > 0.001
          )
        )
      ) continue;

      const segment = segmentRecord(
        pointA,
        pointB,
        plan.equipment.id,
        plan.cable.diameter_mm
      );
      if (!segment) continue;

      segment.network_id = plan.engineering_network_id ||
        (plan.panel && plan.panel.id ? String(plan.panel.id) : null);

      segment.main_level_eligible = Number.isFinite(mainLevelY) &&
        Math.abs(segment.start.y - mainLevelY) < 0.001 &&
        Math.abs(segment.end.y - mainLevelY) < 0.001;

      segment.main_corridor_eligible =
        Array.isArray(plan.main_corridor_segment_keys) &&
        plan.main_corridor_segment_keys.indexOf(segment.key) >= 0;

      segment.parallel_main_eligible =
        plan.parallel_main_corridor === true &&
        plan.main_corridor_axis === segment.axis &&
        segment.main_level_eligible &&
        (
          !Number.isFinite(Number(plan.main_corridor_secondary_coordinate_mm)) ||
          Math.abs(
            Number(segment.start[plan.main_corridor_axis === 'x' ? 'z' : 'x']) -
            Number(plan.main_corridor_secondary_coordinate_mm)
          ) < Math.max(1, (Number(options.gridStepMm) || 100) * 0.5)
        );

      rawSegments.push(segment);
    }
  });

  // Add the complete shared Main spine even when no single cable path covers its end.
  const diameterByCableId=new Map();
  (cablePlans||[]).forEach(function(plan){if(plan&&plan.equipment)diameterByCableId.set(plan.equipment.id,Number(plan.cable&&plan.cable.diameter_mm)||0);});
  (mainCorridors||[]).forEach(function(corridor){
    const points=Array.isArray(corridor.points)?corridor.points:[],ids=Array.isArray(corridor.cable_ids)?corridor.cable_ids:[];
    if(points.length<2||!ids.length)return;
    for(let i=1;i<points.length;i++){
      const id=ids[0],segment=segmentRecord(points[i-1],points[i],id,Number(diameterByCableId.get(id))||0);if(!segment)continue;
      segment.network_id=corridor.network_id||null;
      segment.main_level_eligible=Number.isFinite(Number(corridor.main_level_y_mm))&&Math.abs(segment.start.y-Number(corridor.main_level_y_mm))<0.001&&Math.abs(segment.end.y-Number(corridor.main_level_y_mm))<0.001;
      segment.main_corridor_eligible=true;segment.parallel_main_eligible=true;
      ids.forEach(function(cableId){segment.cableIds.add(cableId);segment.diameterByCable.set(cableId,Number(diameterByCableId.get(cableId))||0);});
      rawSegments.push(segment);
    }

    const panelPoints=Array.isArray(corridor.panel_connection_points)?corridor.panel_connection_points:[];
    if(corridor.parallel_main_corridor===true&&panelPoints.length>=2){
      for(let i=1;i<panelPoints.length;i++){
        const id=ids[0],segment=segmentRecord(panelPoints[i-1],panelPoints[i],id,Number(diameterByCableId.get(id))||0);
        if(!segment)continue;
        segment.network_id=corridor.network_id||null;
        segment.main_level_eligible=Number.isFinite(Number(corridor.main_level_y_mm))&&
          Math.abs(segment.start.y-Number(corridor.main_level_y_mm))<0.001&&
          Math.abs(segment.end.y-Number(corridor.main_level_y_mm))<0.001;
        segment.main_corridor_eligible=false;
        segment.parallel_main_eligible=false;
        ids.forEach(function(cableId){segment.cableIds.add(cableId);segment.diameterByCable.set(cableId,Number(diameterByCableId.get(cableId))||0);});
        rawSegments.push(segment);
      }
    }
  });

  // A cable path may describe one shared trunk as A→C, while another path
  // describes the same geometry as A→B→C. Split collinear segments at every
  // endpoint before calculating cable membership so shared portions are
  // classified consistently instead of overlaying a Branch on the Main Tray.
  const segments = new Map();
  const collinearLines = new Map();

  rawSegments.forEach(function(segment) {
    let fixed1;
    let fixed2;
    let low;
    let high;

    let fixed3 = null;
    if (segment.axis === 'x') {
      fixed1 = rounded(segment.start.y);
      fixed2 = rounded(segment.start.z);
      low = rounded(segment.minPoint.x);
      high = rounded(segment.maxPoint.x);
    } else if (segment.axis === 'y') {
      fixed1 = rounded(segment.start.x);
      fixed2 = rounded(segment.start.z);
      low = rounded(segment.minPoint.y);
      high = rounded(segment.maxPoint.y);
    } else if (segment.axis === 'xz') {
      fixed1 = rounded(segment.start.y);
      fixed2 = roundedTo((segment.end.z - segment.start.z) / (segment.end.x - segment.start.x), 5);
      fixed3 = roundedTo(segment.start.z - fixed2 * segment.start.x, 0);
      low = rounded(segment.minPoint.x);
      high = rounded(segment.maxPoint.x);
    } else {
      fixed1 = rounded(segment.start.x);
      fixed2 = rounded(segment.start.y);
      low = rounded(segment.minPoint.z);
      high = rounded(segment.maxPoint.z);
    }
    if (high - low < 0.001) return;

    const lineKey = [
      segment.network_id || '',
      segment.axis,
      fixed1,
      fixed2,
      fixed3 == null ? '' : fixed3
    ].join('|');
    const line = collinearLines.get(lineKey) || {
      network_id:segment.network_id || null,
      axis:segment.axis,
      fixed1,
      fixed2,
      fixed3,
      ranges:[]
    };
    line.ranges.push({low,high,segment});
    collinearLines.set(lineKey, line);
  });

  collinearLines.forEach(function(line) {
    const breakpoints = Array.from(new Set(
      line.ranges.reduce(function(all, range){
        all.push(range.low, range.high);
        return all;
      }, [])
    )).sort(function(a,b){ return a-b; });

    for (let i = 1; i < breakpoints.length; i++) {
      const low = breakpoints[i - 1];
      const high = breakpoints[i];
      if (high - low < 0.001) continue;
      const middle = (low + high) / 2;
      const active = line.ranges.filter(function(range){
        return range.low <= middle + 0.001 && range.high >= middle - 0.001;
      });
      if (!active.length) continue;

      let start;
      let end;
      if (line.axis === 'x') {
        start = {x:low,y:line.fixed1,z:line.fixed2};
        end = {x:high,y:line.fixed1,z:line.fixed2};
      } else if (line.axis === 'y') {
        start = {x:line.fixed1,y:low,z:line.fixed2};
        end = {x:line.fixed1,y:high,z:line.fixed2};
      } else if (line.axis === 'xz') {
        start = {x:low,y:line.fixed1,z:line.fixed2 * low + line.fixed3};
        end = {x:high,y:line.fixed1,z:line.fixed2 * high + line.fixed3};
      } else {
        start = {x:line.fixed1,y:line.fixed2,z:low};
        end = {x:line.fixed1,y:line.fixed2,z:high};
      }

      const first = active[0].segment;
      const firstCableId = Array.from(first.cableIds)[0];
      const firstDiameter = Number(first.diameterByCable.get(firstCableId)) || 0;
      const atomic = segmentRecord(start,end,firstCableId,firstDiameter);
      if (!atomic) continue;
      atomic.network_id = line.network_id;
      atomic.main_level_eligible = false;
      atomic.main_corridor_eligible = false;
      atomic.parallel_main_eligible = false;

      active.forEach(function(range) {
        range.segment.cableIds.forEach(function(cableId){
          atomic.cableIds.add(cableId);
          const diameter = Number(range.segment.diameterByCable.get(cableId)) || 0;
          atomic.diameterByCable.set(
            cableId,
            Math.max(Number(atomic.diameterByCable.get(cableId)) || 0, diameter)
          );
        });
        atomic.main_level_eligible = atomic.main_level_eligible || range.segment.main_level_eligible;
        atomic.main_corridor_eligible = atomic.main_corridor_eligible || range.segment.main_corridor_eligible;
        atomic.parallel_main_eligible = atomic.parallel_main_eligible || range.segment.parallel_main_eligible;
      });

      const segmentMapKey = [line.network_id || '',atomic.key].join('||');
      const previous = segments.get(segmentMapKey);
      if (previous) {
        atomic.cableIds.forEach(function(cableId){ previous.cableIds.add(cableId); });
        atomic.diameterByCable.forEach(function(diameter,cableId){
          previous.diameterByCable.set(cableId,Math.max(
            Number(previous.diameterByCable.get(cableId)) || 0,
            Number(diameter) || 0
          ));
        });
        previous.main_level_eligible = previous.main_level_eligible || atomic.main_level_eligible;
        previous.main_corridor_eligible = previous.main_corridor_eligible || atomic.main_corridor_eligible;
        previous.parallel_main_eligible = previous.parallel_main_eligible || atomic.parallel_main_eligible;
      } else {
        segments.set(segmentMapKey,atomic);
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

    const parallelPlan = cablePlans.find(function(plan) {
      return plan.equipment &&
        segment.cableIds.has(plan.equipment.id) &&
        plan.parallel_main_corridor === true;
    });
    const parallelAxis = planAxisForParallelMain(parallelPlan);
    const corridorAxisEligible = !parallelAxis || segment.axis === parallelAxis;

    // Prefer the explicitly selected corridor, but also recognize a real
    // shared horizontal trunk when CAD anchor geometry prevents exact segment
    // keys from matching. Short panel stubs must never become Main Tray runs.
    const sharedMainMinimumLength = Math.max(
      Math.max(1, Number(options.gridStepMm) || 100) * 3,
      300
    );
    const sharedHorizontalTrunkEligible =
      segment.cableIds.size >= mainMinCables &&
      segment.main_level_eligible &&
      corridorAxisEligible &&
      segment.length + 0.001 >= sharedMainMinimumLength &&
      (options.structurePrimaryAxis === 'x' || options.structurePrimaryAxis === 'z'
        ? segment.axis === options.structurePrimaryAxis
        : segment.axis === 'x' || segment.axis === 'z');

    const classification = (
      segment.main_corridor_eligible ||
      (
        segment.parallel_main_eligible &&
        segment.length + 0.001 >= sharedMainMinimumLength
      ) ||
      sharedHorizontalTrunkEligible
    ) ? 'main' : 'branch';

    const networkWidthId = segment.network_id || panelId;
    const networkWidth = Number(networkWidthById.get(networkWidthId));

    // Only the actual shared Main Tray uses the full network width.
    // A branch serving one equipment item must be sized from the cables that
    // physically remain on that branch; otherwise every motor gets a full-width
    // copy of the Main Tray and the tree topology becomes visually misleading.
    const segmentRequiredWidth = (
      Array.from(segment.diameterByCable.values()).reduce(function(sum, value) {
        return sum + (Number(value) || 0);
      }, Math.max(0, Number(options.traySideMarginMm) || 25) * 2) /
      (Math.min(100, Math.max(1, Number(options.fillLimitPercent) || 80)) / 100)
    );

    const width = classification === 'main' &&
      Number.isFinite(networkWidth) &&
      networkWidth > 0
      ? networkWidth
      : chooseTrayWidth(
          segmentRequiredWidth,
          options.standardTrayWidthsMm
        );

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
    } else if (segment.axis === 'xz') {
      const slope = roundedTo((segment.end.z - segment.start.z) / (segment.end.x - segment.start.x), 5);
      const intercept = roundedTo(segment.start.z - slope * segment.start.x, 0);
      fixed1 = rounded(segment.start.y);
      fixed2 = slope + '|' + intercept;
    } else {
      fixed1 = rounded(segment.start.x);
      fixed2 = rounded(segment.start.y);
    }

    // Main Tray is one physical shared route even when the cable membership
    // changes along its length. Branch runs remain separated by cable set.
    const cableSetKey = segment.classification === 'branch'
      ? Array.from(segment.cableIds).sort().join(',')
      : '';
    const lineKey = [
      segment.network_id || '',
      segment.axis,
      fixed1,
      fixed2,
      segment.classification,
      segment.width_mm,
      segment.height_mm,
      cableSetKey
    ].join('|');

    const entry = grouped.get(lineKey) || {
      network_id:segment.network_id || null,
      axis:segment.axis,
      fixed1,
      fixed2,
      classification:segment.classification,
      width_mm:segment.width_mm,
      height_mm:segment.height_mm,
      lineSlope:segment.axis === 'xz' ? Number(String(fixed2).split('|')[0]) : null,
      lineIntercept:segment.axis === 'xz' ? Number(String(fixed2).split('|')[1]) : null,
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
    } else if (segment.axis === 'xz') {
      low = segment.minPoint.x; high = segment.maxPoint.x;
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

function clipPolylineAtPoint(points, target, toleranceMm) {
  const source = (points || []).map(clonePoint);
  if (!target || source.length < 2) return [];
  const tolerance = Math.max(0.01, Number(toleranceMm) || 1);
  function distance(a,b) {
    return Math.sqrt(
      Math.pow(a.x-b.x,2) +
      Math.pow(a.y-b.y,2) +
      Math.pow(a.z-b.z,2)
    );
  }
  const output = [];
  for (let i=0;i<source.length;i++) {
    const point = source[i];
    output.push(point);
    if (distance(point,target) <= tolerance) {
      output[output.length-1] = clonePoint(target);
      return output;
    }
    if (i+1 >= source.length) continue;
    const next = source[i+1];
    const dx=next.x-point.x,dy=next.y-point.y,dz=next.z-point.z;
    const lengthSq=dx*dx+dy*dy+dz*dz;
    if (lengthSq < 1e-9) continue;
    const t=Math.max(0,Math.min(1,((target.x-point.x)*dx+(target.y-point.y)*dy+(target.z-point.z)*dz)/lengthSq));
    const projection={x:point.x+dx*t,y:point.y+dy*t,z:point.z+dz*t};
    if (distance(projection,target) <= tolerance) {
      output.push(clonePoint(target));
      return output;
    }
  }
  return [];
}

function shortenPolylineEnd(points, distanceMm) {
  const output = (points || []).map(clonePoint);
  let remaining=Math.max(0,Number(distanceMm)||0);
  while(output.length>=2&&remaining>0.001){
    const last=output[output.length-1],previous=output[output.length-2];
    const dx=last.x-previous.x,dy=last.y-previous.y,dz=last.z-previous.z;
    const length=Math.sqrt(dx*dx+dy*dy+dz*dz);
    if(length<0.001){output.pop();continue;}
    if(length>remaining+0.001){
      const ratio=(length-remaining)/length;
      output[output.length-1]={x:previous.x+dx*ratio,y:previous.y+dy*ratio,z:previous.z+dz*ratio};
      remaining=0;
    }else{
      remaining-=length;
      output.pop();
    }
  }
  return output;
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
    network_id:first.network_id || null,
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
      run.network_id || '',
      run.classification || 'branch',
      Number(run.width_mm) || 0,
      Number(run.height_mm) || 0,
      run.classification === 'branch'
        ? Array.from(run.cable_ids || []).sort().join(',')
        : ''
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
      network_id:group.network_id || null,
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
      network_id:group.network_id || null,
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
  if (group.axis === 'xz') {
    const points = [
      {x:range.low,y:group.fixed1,z:group.lineSlope * range.low + group.lineIntercept},
      {x:range.high,y:group.fixed1,z:group.lineSlope * range.high + group.lineIntercept}
    ];
    return {
      network_id:group.network_id || null,
      classification:group.classification,
      width_mm:group.width_mm,
      height_mm:group.height_mm,
      cable_ids:cableIds,
      points,
      length_m:routeLengthMeters(points)
    };
  }
  return {
      network_id:group.network_id || null,
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
        routingBounds.maxY,
        Number.isFinite(Number(options.floorMaxY)) ? Number(options.floorMaxY) : Infinity
      )
    : Math.min(
        ceilingY - safetyGap - trayHeight / 2 - clearance,
        Number.isFinite(Number(options.floorMaxY)) ? Number(options.floorMaxY) : Infinity
      );

  const step = Math.max(10, Number(options.gridStepMm) || 100);
  const probeStep = Math.max(10, Math.min(step / 2, 50));
  const lowerY = Math.max(
    Number.isFinite(Number(options.floorMinY)) ? Number(options.floorMinY) : -Infinity,
    Math.min.apply(
      null,
      points.map(function(point){ return Number(point.y) || 0; }).concat([desiredY - Math.max(1000, Number(options.maxBodyDistanceMm) || 1500)])
    )
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
    bounds ? bounds.maxY : ceilingY,
    Number.isFinite(Number(options.floorMaxY)) ? Number(options.floorMaxY) : Infinity
  );
  const alignedStart = Math.floor(desiredY / step) * step;
  const globalLowerY = bounds
    ? Math.ceil(bounds.minY / step) * step
    : Math.min(
        alignedStart - Math.max(1000, Number(options.maxBodyDistanceMm) || 1500),
        Number(panelPoint.y) || alignedStart
      );
  const lowerY = Math.max(
    globalLowerY,
    Number.isFinite(Number(options.floorMinY))
      ? Math.ceil(Number(options.floorMinY) / step) * step
      : -Infinity
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

function makeNetworkCablePlan(entry, group, branchPath, tree, attachmentPoint, panelDrop, planningTrayWidth, options, mainCorridorSegmentKeys) {
  const treePath = traceNetworkToRoot(tree, attachmentPoint);
  if (!treePath.length) return null;

  const targetPanel = entry.panel || group.panel;
  const targetPanelDrop = entry.panelDrop || panelDrop;
  const targetPanelConnector = entry.panelConnector && entry.panelConnector.points
    ? entry.panelConnector.points
    : [];
  if (!targetPanel || !targetPanelDrop || !targetPanelDrop.points) return null;

  const points = [];
  appendDistinctPoints(points, [entry.equipment.anchor.point]);
  appendDistinctPoints(points, [entry.start]);
  appendDistinctPoints(points, branchPath, 0);
  appendDistinctPoints(points, treePath, 1);

  if (targetPanelConnector.length > 1) {
    appendDistinctPoints(points, targetPanelConnector, 1);
  }

  for (let i = (targetPanelDrop.points || []).length - 2; i >= 0; i--) {
    appendDistinctPoints(points, [targetPanelDrop.points[i]]);
  }
  appendDistinctPoints(points, [targetPanel.anchor.point]);

  return {
    equipment:entry.equipment,
    panel:targetPanel,
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
    main_corridor_segment_keys:Array.isArray(mainCorridorSegmentKeys)
      ? mainCorridorSegmentKeys.slice()
      : [],
    engineering_network_id:group.network_id || String(group.panel.id),
    engineering_floor_id:group.floor_id || null,
    engineering_floor_name:group.floor_name || null,
    engineering_floor_id:group.floor_id || null,
    engineering_floor_name:group.floor_name || null,
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
    primarySpan < gridStep * 3
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
  if (!prepared || prepared.length < 2 || !group || !panelStandoff || !panelStandoff.point) return null;
  if (!panelDrop || !Array.isArray(panelDrop.points) || !panelDrop.points.length) return null;
  const panelIds = new Set(prepared.map(function(entry){return String((entry.panel||group.panel).id);}));
  if(panelIds.size!==1||!panelIds.has(String(group.panel.id)))return null;

  const step=Math.max(50,Number(options.gridStepMm)||100);
  let activeMainSlope=0;
  let activeMainPrimaryCenter=0;
  let primaryAxis=options.structurePrimaryAxis==='x'||options.structurePrimaryAxis==='z'?options.structurePrimaryAxis:null;
  if(!primaryAxis){
    const xs=prepared.map(function(e){return Number(e.start.x);}),zs=prepared.map(function(e){return Number(e.start.z);});
    primaryAxis=Math.max.apply(Math,xs)-Math.min.apply(Math,xs)>=Math.max.apply(Math,zs)-Math.min.apply(Math,zs)?'x':'z';
  }
  let secondaryAxis=primaryAxis==='x'?'z':'x';
  let primary=prepared.map(function(e){return Number(e.start[primaryAxis]);});
  let secondaryValues=prepared.map(function(e){return Number(e.start[secondaryAxis]);});
  let primarySpan=Math.max.apply(Math,primary)-Math.min.apply(Math,primary);
  const secondarySpan=Math.max.apply(Math,secondaryValues)-Math.min.apply(Math,secondaryValues);
  let candidates=resolveParallelMainCorridorCandidates(prepared,options);
  if(primarySpan<step*3&&secondarySpan>primarySpan){
    primaryAxis=secondaryAxis;secondaryAxis=primaryAxis==='x'?'z':'x';
    primary=prepared.map(function(e){return Number(e.start[primaryAxis]);});
    secondaryValues=prepared.map(function(e){return Number(e.start[secondaryAxis]);});
    primarySpan=Math.max.apply(Math,primary)-Math.min.apply(Math,primary);candidates=[];
  }
  if(primarySpan<step*3)return null;
  if(!candidates.length){
    const avg=secondaryValues.reduce(function(a,b){return a+b;},0)/secondaryValues.length;
    const panelSecondaryForFallback=Number(panelStandoff.point[secondaryAxis]);
    candidates=[{
      secondary:Number.isFinite(panelSecondaryForFallback)
        ? (avg+panelSecondaryForFallback)/2
        : avg,
      singleRowFallback:true
    }];
  }

  // AABB checks currently inset both X and Z by half the tray width. For a
  // horizontal axis-aligned segment, tray width is across the cross-axis, not
  // along the segment; offset the bounds on the segment axis only. Mesh checks
  // remain active and validate the actual tray cross-section.
  function directionalClear(a,b,baseOptions){
    let adjusted=baseOptions;const bounds=baseOptions&&baseOptions.routingBounds;
    if(bounds){
      const next={...bounds},dx=Math.abs(Number(b.x)-Number(a.x)),dz=Math.abs(Number(b.z)-Number(a.z));
      const width=Math.max(1,Number(baseOptions.routingTrayWidthMm)||Number(baseOptions.trayHeightMm)||100);
      if(dx>0.001&&dz<0.001){next.minX-=width/2;next.maxX+=width/2;}
      else if(dz>0.001&&dx<0.001){next.minZ-=width/2;next.maxZ+=width/2;}
      else if(dx<0.001&&dz<0.001&&Math.abs(Number(b.y)-Number(a.y))>0.001){
        next.minX-=width/2;next.maxX+=width/2;next.minZ-=width/2;next.maxZ+=width/2;
      }
      adjusted={...baseOptions,routingBounds:next};
    }
    return segmentClearForRouting(a,b,obstacles,adjusted);
  }
  function mainPoint(p,s){
    const cross=Number(s)+activeMainSlope*(Number(p)-activeMainPrimaryCenter);
    return primaryAxis==='x'?{x:Number(p),y:routingY,z:cross}:{x:cross,y:routingY,z:Number(p)};
  }
  function dedupe(points){
    const out=[];(points||[]).forEach(function(p){if(!p)return;const q=clonePoint(p),v=out[out.length-1];
      if(v&&Math.abs(v.x-q.x)<0.001&&Math.abs(v.y-q.y)<0.001&&Math.abs(v.z-q.z)<0.001)return;out.push(q);});return out;
  }
  function clearPath(points,opts){for(let i=1;i<points.length;i++)if(!directionalClear(points[i-1],points[i],opts))return false;return true;}

  const panelPoint=pointAtRoutingY(panelStandoff.point,routingY),panelPrimary=Number(panelStandoff.point[primaryAxis]),panelSecondary=Number(panelStandoff.point[secondaryAxis]);
  const terminal=prepared.map(function(e){return Number(e.start[primaryAxis]);}).concat([panelPrimary]);
  const rawMin=Math.min.apply(Math,terminal),rawMax=Math.max.apply(Math,terminal);
  const loadMin=Math.min.apply(Math,prepared.map(function(e){return Number(e.start[primaryAxis]);}));
  const loadMax=Math.max.apply(Math,prepared.map(function(e){return Number(e.start[primaryAxis]);}));
  const atHigh=panelPrimary>=loadMax-step*0.5,atLow=panelPrimary<=loadMin+step*0.5;
  const overrun=Math.max(500,step),tailOptions=Array.from(new Set([overrun,Math.min(overrun,400),Math.min(overrun,300),Math.min(overrun,200),Math.min(overrun,100),0]));
  const minSec=Math.min.apply(Math,secondaryValues),maxSec=Math.max.apply(Math,secondaryValues),baseLevels=[];
  const trayHalfWidth=Math.max(1,Number(routeOptions.routingTrayWidthMm)||100)/2;
  const minimumPanelOffset=(trayHalfWidth+Math.max(0,Number(options.clearanceMm)||0)+step*0.5)/(1-0.215);
  candidates.forEach(function(c){
    const raw=Number(c.secondary);
    const panelSideDistance=panelSecondary-raw;
    const direction=panelSideDistance>=0?1:-1;
    const value=Math.abs(panelSideDistance)<minimumPanelOffset
      ? panelSecondary-direction*minimumPanelOffset
      : raw;
    if(Number.isFinite(value)&&!baseLevels.some(function(existing){return Math.abs(existing-value)<0.001;}))baseLevels.push(value);
  });
  const secondaryCandidates=[];
  const hasOccupiedMains=Array.isArray(options.occupiedMainCorridors)&&options.occupiedMainCorridors.length>0;
  const occupiedMainWidth=hasOccupiedMains
    ? Math.max.apply(null,options.occupiedMainCorridors.map(function(c){return Number(c.width_mm)||Number(c.height_mm)||100;}))
    : 0;
  const laneSpacing=hasOccupiedMains
    ? Math.max(step,Math.ceil((((Number(routeOptions.routingTrayWidthMm)||100)+occupiedMainWidth)/2+Math.max(0,Number(options.clearanceMm)||0))/step)*step)
    : step;
  const boundsForLanes=hasOccupiedMains
    ? normalizeRoutingBounds(routeOptions.routingBounds,routeOptions)
    : null;
  const secondaryMin=boundsForLanes
    ? (primaryAxis==='x'?boundsForLanes.minZ:boundsForLanes.minX)+(Number(routeOptions.routingTrayWidthMm)||100)/2
    : -Infinity;
  const secondaryMax=boundsForLanes
    ? (primaryAxis==='x'?boundsForLanes.maxZ:boundsForLanes.maxX)-(Number(routeOptions.routingTrayWidthMm)||100)/2
    : Infinity;

  // For capacity-split groups on the same floor and panel, keep each new Main
  // on the outside of an existing lane on the same side of the panel. This
  // prevents a later equipment branch from having to cross an earlier Main.
  // Only lines whose primary-axis span actually overlaps this group are used.
  if(hasOccupiedMains){
    const loadSecondaryMean=secondaryValues.reduce(function(sum,value){return sum+value;},0)/secondaryValues.length;
    const side= Math.sign(loadSecondaryMean-panelSecondary) || -1;
    const currentPrimaryMin=Math.min.apply(Math,terminal);
    const currentPrimaryMax=Math.max.apply(Math,terminal);
    const sameSideLanes=[];
    (options.occupiedMainCorridors||[]).forEach(function(corridor){
      if(String(corridor.floor_id == null ? '' : corridor.floor_id)!==String(group.floor_id == null ? '' : group.floor_id))return;
      const points=Array.isArray(corridor.points)?corridor.points:[];
      if(points.length<2)return;
      const lanePrimary=points.map(function(point){return Number(point[primaryAxis]);}).filter(Number.isFinite);
      const laneSecondary=points.map(function(point){return Number(point[secondaryAxis]);}).filter(Number.isFinite);
      if(lanePrimary.length<2||!laneSecondary.length)return;
      const overlap=Math.min(currentPrimaryMax,Math.max.apply(Math,lanePrimary))-
        Math.max(currentPrimaryMin,Math.min.apply(Math,lanePrimary));
      if(overlap<Math.max(1,step*0.5))return;
      const laneCoordinate=laneSecondary.reduce(function(sum,value){return sum+value;},0)/laneSecondary.length;
      if((laneCoordinate-panelSecondary)*side>0.001)sameSideLanes.push(laneCoordinate);
    });
    if(sameSideLanes.length){
      const outermost=side>0?Math.max.apply(Math,sameSideLanes):Math.min.apply(Math,sameSideLanes);
      const nextLane=outermost+side*laneSpacing;
      if(nextLane>=secondaryMin-0.001&&nextLane<=secondaryMax+0.001&&
          !baseLevels.some(function(value){return Math.abs(value-nextLane)<0.001;})){
        baseLevels.push(nextLane);
      }
    }
  }

  baseLevels.forEach(function(base){
    if(!secondaryCandidates.includes(base))secondaryCandidates.push(base);
    if(hasOccupiedMains){
      const extent=boundsForLanes
        ? Math.max(Math.abs(base-secondaryMin),Math.abs(secondaryMax-base))
        : Math.max(3000,Number(options.routingPaddingMm)||1000);
      const count=Math.min(10,Math.max(1,Math.ceil(extent/laneSpacing)));
      for(let n=1;n<=count;n++){
        [base-n*laneSpacing,base+n*laneSpacing].forEach(function(v){
          if(v<secondaryMin-0.001||v>secondaryMax+0.001)return;
          if(!secondaryCandidates.some(function(existing){return Math.abs(existing-v)<0.001;}))secondaryCandidates.push(v);
        });
      }
      return;
    }
    const count=Math.min(10,Math.floor(Math.max(0,maxSec-minSec)/(step*2)));
    for(let n=1;n<=count;n++)[base-n*step,base+n*step].forEach(function(v){
      if(maxSec-minSec>=step*2&&(v<=minSec+step*0.5||v>=maxSec-step*0.5))return;
      if(!secondaryCandidates.includes(v))secondaryCandidates.push(v);
    });
  });
  secondaryCandidates.sort(function(a,b){
    return Math.min.apply(Math,baseLevels.map(function(v){return Math.abs(a-v);}))-
      Math.min.apply(Math,baseLevels.map(function(v){return Math.abs(b-v);}))||a-b;
  });

  for(let si=0;si<secondaryCandidates.length;si++){
    const sec=secondaryCandidates[si];
    for(let ti=0;ti<tailOptions.length;ti++){
      const tail=tailOptions[ti];
      let start=Math.min(rawMin,Math.round(rawMin/step)*step),end=Math.max(rawMax,Math.round(rawMax/step)*step);
      if(atHigh)end=Math.max(end,Math.round(panelPrimary/step)*step+tail);
      else if(atLow)start=Math.min(start,Math.round(panelPrimary/step)*step-tail);
      const physicalInset=Math.min(Math.max(50,step*0.5),Math.max(0,(end-start)*0.2));
      let physicalStartPrimary=atHigh?start+physicalInset:start;
      let physicalEndPrimary=atLow?end-physicalInset:end;

      // Validate and build only the physical Main span. Cable-tree nodes may
      // extend to a load near the model boundary, but that logical extension
      // is not a tray segment and must not make an otherwise valid Main fail
      // the collision/bounds test. Keep the physical centerline inside the
      // usable envelope for the actual Main tray width.
      const usableMainBounds=normalizeRoutingBounds(routeOptions.routingBounds,routeOptions);
      if(usableMainBounds&&usableMainBounds.valid===false)continue;
      if(usableMainBounds){
        const primaryMin=primaryAxis==='x'?usableMainBounds.minX:usableMainBounds.minZ;
        const primaryMax=primaryAxis==='x'?usableMainBounds.maxX:usableMainBounds.maxZ;
        physicalStartPrimary=Math.max(physicalStartPrimary,primaryMin);
        physicalEndPrimary=Math.min(physicalEndPrimary,primaryMax);
      }
      if(physicalEndPrimary-physicalStartPrimary<Math.max(1,step*0.5))continue;
      const physicalSpan=Math.max(0.001,physicalEndPrimary-physicalStartPrimary);
      const requestedCrossShift=(panelSecondary-sec)*0.215;
      const maxCrossShift=Math.tan(2.57*Math.PI/180)*physicalSpan;
      const crossShift=Math.max(-maxCrossShift,Math.min(maxCrossShift,requestedCrossShift));
      if(atHigh){
        activeMainPrimaryCenter=physicalStartPrimary;
        activeMainSlope=crossShift/physicalSpan;
      }else if(atLow){
        activeMainPrimaryCenter=physicalEndPrimary;
        activeMainSlope=-crossShift/physicalSpan;
      }else{
        activeMainPrimaryCenter=(physicalStartPrimary+physicalEndPrimary)/2;
        activeMainSlope=crossShift/physicalSpan;
      }
      const coordsSet=new Set([physicalStartPrimary,physicalEndPrimary]);
      terminal.forEach(function(v){
        if(v>=physicalStartPrimary-0.001&&v<=physicalEndPrimary+0.001)coordsSet.add(v);
      });
      for(let v=Math.ceil(physicalStartPrimary/step)*step;v<physicalEndPrimary-0.001;v+=step)coordsSet.add(v);
      const coords=Array.from(coordsSet).filter(Number.isFinite).filter(function(v){
        return v>=physicalStartPrimary-0.001&&v<=physicalEndPrimary+0.001;
      }).sort(function(a,b){return a-b;});
      if(coords.length<2)continue;
      const spine=coords.map(function(v){return mainPoint(v,sec);});
      if(!clearPath(spine,routeOptions))continue;
      const panelSpine=mainPoint(panelPrimary,sec);
      if(!directionalClear(panelStandoff.point,panelPoint,routeOptions))continue;
      let panelConnectorResult={points:dedupe([panelSpine,panelPoint]),warning:null,fallback:false};
      if(!clearPath(panelConnectorResult.points,routeOptions)){
        const directionCandidates=[-1,1,-2,2];
        let detour=null;
        for(let di=0;di<directionCandidates.length;di++){
          const primaryOffset=directionCandidates[di]*Math.max(
            step,
            Math.ceil((((Number(routeOptions.routingTrayWidthMm)||100)+
              (Number(options.occupiedMainCorridors&&options.occupiedMainCorridors[0]&&options.occupiedMainCorridors[0].width_mm)||100))/2+
              Math.max(0,Number(options.clearanceMm)||0))/step)*step
          );
          const pivotA=clonePoint(panelSpine);
          const pivotB=clonePoint(panelPoint);
          if(primaryAxis==='x'){pivotA.x+=primaryOffset;pivotB.x+=primaryOffset;}
          else{pivotA.z+=primaryOffset;pivotB.z+=primaryOffset;}
          const candidate=[clonePoint(panelSpine),pivotA,pivotB,clonePoint(panelPoint)];
          if(clearPath(candidate,routeOptions)){detour=dedupe(candidate);break;}
        }
        if(detour){
          panelConnectorResult={points:detour,warning:null,fallback:false};
        }else if(!hasOccupiedMains){
          panelConnectorResult=findGridPath3D(
            panelSpine,
            panelPoint,
            obstacles,
            {
              ...routeOptions,
              fixedRoutingY:routingY,
              preferredRoutingY:routingY,
              routeTurnPenaltyRatio:Math.max(
                Number(options.turnPenaltyRatio)||20,
                Number(options.mainCorridorTurnPenaltyRatio)||100
              )
            },
            new Set()
          );
        }else{
          continue;
        }
      }
      if(!panelConnectorResult.points||panelConnectorResult.points.length<2)continue;
      const panelConnectorPath=dedupe(panelConnectorResult.points);
      const cleanDrop={points:[clonePoint(panelStandoff.point),clonePoint(panelPoint)],warning:null,fallback:false};
      prepared.forEach(function(e){
        e.panelStandoff=panelStandoff;e.panelHighPoint=clonePoint(panelPoint);
        e.panelConnector={points:[clonePoint(panelPoint),clonePoint(panelPoint)],warning:null,fallback:false};e.panelDrop=cleanDrop;
      });
      const rootKey=networkNodeKey(panelPoint),tree={nodes:new Map([[rootKey,clonePoint(panelPoint)]]),parent:new Map([[rootKey,null]])},keys=[];
      spine.forEach(function(p){const k=networkNodeKey(p);keys.push(k);tree.nodes.set(k,clonePoint(p));});
      const panelIndex=keys.indexOf(networkNodeKey(panelSpine));if(panelIndex<0)continue;
      const rootOnSpine=keys[panelIndex]===rootKey;
      for(let i=0;i<spine.length;i++){
        const k=keys[i],v=coords[i];
        if(rootOnSpine&&k===rootKey)tree.parent.set(k,null);
        else if(i===panelIndex)tree.parent.set(k,rootKey);
        else if(v<panelPrimary)tree.parent.set(k,keys[i+1]||rootKey);
        else tree.parent.set(k,keys[i-1]||rootKey);
      }
      // The logical tree remains connected through the routed panel connector,
      // even when its physical tray end is shortened to preserve the reference gap.
      for(let i=panelConnectorPath.length-2;i>=0;i--){
        const current=panelConnectorPath[i],next=panelConnectorPath[i+1];
        const currentKey=networkNodeKey(current),nextKey=networkNodeKey(next);
        tree.nodes.set(currentKey,clonePoint(current));
        tree.nodes.set(nextKey,clonePoint(next));
        tree.parent.set(currentKey,nextKey);
      }
      const plans=[];let valid=true;
      for(let i=0;i<prepared.length;i++){
        const entry=prepared[i],branchOptions=entry.branchRouteOptions||routeOptions;
        const plane=clonePoint(entry.start);plane.y=routingY;
        const rawAttachPrimary=Number(entry.start[primaryAxis]);
        const attachPrimary=Math.max(physicalStartPrimary,Math.min(physicalEndPrimary,rawAttachPrimary));
        let attach=mainPoint(attachPrimary,sec);
        let branch=dedupe([entry.start,plane,attach]);
        if(!clearPath(branch,branchOptions)){
          const horizontalOptions={
            ...branchOptions,
            fixedRoutingY:routingY,
            preferredRoutingY:routingY
          };
          const horizontalBridge=bridgePath3D(plane,attach,obstacles,horizontalOptions);
          if(horizontalBridge){
            branch=dedupe([entry.start].concat(horizontalBridge));
          }else if(!hasOccupiedMains){
            const routedBranch=findGridPath3D(
              entry.start,
              null,
              obstacles,
              {
                ...branchOptions,
                fixedRoutingY:routingY,
                preferredRoutingY:routingY,
                networkGoalPoints:spine,
                networkGoalCandidateLimit:Number(options.networkGoalCandidateLimit)||24,
                routeTurnPenaltyRatio:Math.max(
                  Number(options.turnPenaltyRatio)||20,
                  Number(options.mainCorridorTurnPenaltyRatio)||100
                )
              },
              new Set()
            );
            if(!routedBranch.points||!routedBranch.points.length||!routedBranch.attachment_point){valid=false;break;}
            branch=dedupe(routedBranch.points);
            attach=clonePoint(routedBranch.attachment_point);
          }else{
            valid=false;break;
          }
        }
        if(!tree.nodes.has(networkNodeKey(attach))){valid=false;break;}
        const plan=makeNetworkCablePlan(entry,group,branch,tree,attach,cleanDrop,routeOptions.routingTrayWidthMm,{
          ...routeOptions,fixedRoutingY:routingY,panelStandoffDistanceMm:panelStandoff.distance_mm
        });
        if(!plan){valid=false;break;}
        plan.parallel_main_corridor=true;plan.main_corridor_axis=primaryAxis;plan.main_corridor_secondary_coordinate_mm=sec;plans.push(plan);
      }
      if(!valid||plans.length!==prepared.length)continue;
      const panelConnectionLength=polylineLengthMm(panelConnectorPath);
      const panelGap=Math.min(
        Math.max(100,Math.min(220,step*1.5)),
        Math.max(0,panelConnectionLength-50)
      );
      const panelConnectionPoints=panelConnectionLength>0.001
        ? shortenPolylineEnd(panelConnectorPath.slice().reverse(),panelGap)
        : [clonePoint(panelPoint)];
      const length=plans.reduce(function(s,p){return s+routeLengthMeters(p.points);},0);
      const turns=plans.reduce(function(s,p){return s+countPolylineTurns(p.points);},0);
      return {routingY,panelHighPoint:clonePoint(panelPoint),panelDrop:cleanDrop,cablePlans:plans,connectedCount:plans.length,
        unresolved:[],networkNodes:Array.from(tree.nodes.values()).map(clonePoint),
        main_spine_points:[mainPoint(physicalStartPrimary,sec),mainPoint(physicalEndPrimary,sec)],
        panel_connection_points:panelConnectionPoints,
        parallel_main_corridor:true,main_corridor_axis:primaryAxis,
        score:length+turns*step*0.001};
    }
  }
  return null;
}


function buildGlobalMainBackboneNetwork(group, prepared, panelStandoff, obstacles, routeOptions, options, routingY, panelDrop) {
  if (!prepared || prepared.length < 2 || !panelStandoff || !panelDrop || !panelDrop.points || !panelDrop.points.length) {
    return null;
  }

  const gridStep = Math.max(50, Number(options.gridStepMm) || 100);
  const preferredAxis = options.structurePrimaryAxis === 'x' || options.structurePrimaryAxis === 'z'
    ? options.structurePrimaryAxis
    : null;
  const axisOrder = preferredAxis
    ? [preferredAxis, preferredAxis === 'x' ? 'z' : 'x']
    : ['x', 'z'];

  function uniqueSnappedValues(values) {
    const result = [];
    (values || []).forEach(function(value) {
      const numeric = Number(value);
      if (!Number.isFinite(numeric)) return;
      const snapped = Math.round(numeric / gridStep) * gridStep;
      if (!result.some(function(existing){ return Math.abs(existing - snapped) < 0.001; })) {
        result.push(snapped);
      }
    });
    return result;
  }

  function appendMainNode(tree, nodes, point) {
    const candidate = clonePoint(point);
    const previous = nodes[nodes.length - 1];
    if (
      previous &&
      Math.abs(previous.x - candidate.x) < 0.001 &&
      Math.abs(previous.y - candidate.y) < 0.001 &&
      Math.abs(previous.z - candidate.z) < 0.001
    ) {
      return;
    }

    const key = networkNodeKey(candidate);
    if (!tree.nodes.has(key)) tree.nodes.set(key, candidate);
    if (previous) tree.parent.set(key, networkNodeKey(previous));
    nodes.push(candidate);
  }

  function buildDenseMainNodes(tree, spinePath, nearPoint) {
    const fixed = (spinePath.points || [])
      .filter(function(point){
        return Math.abs(Number(point.y) - Number(routingY)) < 0.001;
      })
      .map(clonePoint);

    if (fixed.length < 2) return [];

    if (
      manhattanDistance3D(fixed[0], nearPoint) >
      manhattanDistance3D(fixed[fixed.length - 1], nearPoint)
    ) {
      fixed.reverse();
    }

    if (
      Math.abs(fixed[0].x - nearPoint.x) > 0.001 ||
      Math.abs(fixed[0].y - nearPoint.y) > 0.001 ||
      Math.abs(fixed[0].z - nearPoint.z) > 0.001
    ) {
      fixed.unshift(clonePoint(nearPoint));
    }

    const dense = [clonePoint(fixed[0])];
    const firstKey = networkNodeKey(dense[0]);
    if (!tree.nodes.has(firstKey)) tree.nodes.set(firstKey, clonePoint(dense[0]));

    for (let i = 1; i < fixed.length; i++) {
      const a = dense[dense.length - 1];
      const b = fixed[i];
      const length = manhattanDistance3D(a, b);
      const count = Math.max(1, Math.ceil(length / gridStep));

      for (let stepIndex = 1; stepIndex <= count; stepIndex++) {
        const t = stepIndex / count;
        appendMainNode(tree, dense, {
          x:a.x + (b.x - a.x) * t,
          y:routingY,
          z:a.z + (b.z - a.z) * t
        });
      }
    }

    return dense;
  }

  function candidateSecondaryCoordinates(axis) {
    const secondaryAxis = axis === 'x' ? 'z' : 'x';
    const values = prepared
      .map(function(entry){ return Number(entry.start[secondaryAxis]); })
      .filter(Number.isFinite)
      .sort(function(a,b){ return a - b; });

    if (!values.length) return [];

    function quantile(position) {
      const index = Math.max(
        0,
        Math.min(values.length - 1, Math.round(position * (values.length - 1)))
      );
      return values[index];
    }

    return uniqueSnappedValues([
      quantile(0.25),
      quantile(0.5),
      quantile(0.75),
      Number(panelStandoff.point[secondaryAxis])
    ]).filter(function(value){
      const testPoint = axis === 'x'
        ? {x:Number(prepared[0].start.x), y:routingY, z:value}
        : {x:value, y:routingY, z:Number(prepared[0].start.z)};

      return pointWithinRoutingBounds(testPoint, routeOptions) &&
        pointWithinBodyDistanceForRouting(testPoint, routeOptions);
    });
  }

  let best = null;

  for (let axisIndex = 0; axisIndex < axisOrder.length; axisIndex++) {
    const axis = axisOrder[axisIndex];
    const secondaryAxis = axis === 'x' ? 'z' : 'x';
    const primaryValues = prepared
      .map(function(entry){ return Number(entry.start[axis]); })
      .filter(Number.isFinite);

    if (!primaryValues.length) continue;

    const primaryMin = Math.min.apply(null, primaryValues);
    const primaryMax = Math.max.apply(null, primaryValues);
    if (primaryMax - primaryMin < gridStep) continue;

    const secondaryCoordinates = candidateSecondaryCoordinates(axis);

    for (let secondaryIndex = 0; secondaryIndex < secondaryCoordinates.length; secondaryIndex++) {
      const secondary = secondaryCoordinates[secondaryIndex];
      const panelPrimary = Number(panelStandoff.point[axis]);
      const nearPrimary = Math.max(primaryMin, Math.min(primaryMax, panelPrimary));
      const nearPoint = axis === 'x'
        ? {x:nearPrimary, y:routingY, z:secondary}
        : {x:secondary, y:routingY, z:nearPrimary};

      const farPrimary =
        Math.abs(panelPrimary - primaryMin) >= Math.abs(panelPrimary - primaryMax)
          ? primaryMin
          : primaryMax;
      const farPoint = axis === 'x'
        ? {x:farPrimary, y:routingY, z:secondary}
        : {x:secondary, y:routingY, z:farPrimary};

      if (
        !pointWithinRoutingBounds(nearPoint, routeOptions) ||
        !pointWithinRoutingBounds(farPoint, routeOptions)
      ) {
        continue;
      }

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

      if (!spinePath.points || spinePath.points.length < 2) continue;

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

      if (!panelConnector.points || panelConnector.points.length < 1) continue;

      const rootPoint = pointAtRoutingY(panelStandoff.point, routingY);
      const rootKey = networkNodeKey(rootPoint);
      const tree = {
        nodes:new Map([[rootKey, clonePoint(rootPoint)]]),
        parent:new Map([[rootKey, null]])
      };

      const corridorPath = [];
      appendDistinctPoints(corridorPath, spinePath.points, 0);
      appendDistinctPoints(corridorPath, panelConnector.points, 1);

      if (!registerNetworkPath(tree, corridorPath, routingY)) continue;

      const denseMainNodes = buildDenseMainNodes(tree, spinePath, nearPoint);
      if (denseMainNodes.length < 2) continue;

      const mainNetworkPoints = denseMainNodes.map(clonePoint);
      const mainCorridorSegmentKeys = new Set();

      for (let i = 1; i < denseMainNodes.length; i++) {
        const segment = segmentRecord(
          denseMainNodes[i - 1],
          denseMainNodes[i],
          prepared[0].equipment.id,
          Number(prepared[0].equipment.cable_diameter_mm) || 0
        );
        if (segment) mainCorridorSegmentKeys.add(segment.key);
      }

      const cablePlans = [];
      const connected = new Set();

      const rankedEquipment = prepared.slice().sort(function(a,b){
        return networkDistanceToNodes(a.start, mainNetworkPoints) -
          networkDistanceToNodes(b.start, mainNetworkPoints) ||
          b.densityScore - a.densityScore;
      });

      function tryDirectMainAttachment(entry) {
        const primaryAxis = axis;
        const secondaryAxis = axis === 'x' ? 'z' : 'x';
        const grid = Math.max(50, Number(options.gridStepMm) || 100);
        const primaryValue = Number(entry.start[primaryAxis]);
        if (!Number.isFinite(primaryValue)) return null;

        const candidates = [];
        mainNetworkPoints.forEach(function(point) {
          const secondaryDistance = Math.abs(
            Number(point[secondaryAxis]) - Number(secondary)
          );
          if (secondaryDistance > 0.001) return;

          const primaryDistance = Math.abs(
            Number(point[primaryAxis]) - primaryValue
          );
          candidates.push({point, distance:primaryDistance});
        });

        candidates.sort(function(a,b){ return a.distance - b.distance; });

        const branchCandidates = candidates.slice(
          0,
          Math.max(3, Number(options.networkAttachmentCandidateLimit) || 4)
        );

        let bestDirect = null;

        branchCandidates.forEach(function(candidate) {
          const target = clonePoint(candidate.point);
          const verticalPoint = {
            x:Number(entry.start.x),
            y:routingY,
            z:Number(entry.start.z)
          };
          const secondaryPoint = primaryAxis === 'x'
            ? {
                x:Number(verticalPoint.x),
                y:routingY,
                z:Number(secondary)
              }
            : {
                x:Number(secondary),
                y:routingY,
                z:Number(verticalPoint.z)
              };

          const path = [
            clonePoint(entry.start),
            verticalPoint,
            secondaryPoint,
            target
          ].filter(function(point, index, points){
            if (!index) return true;
            const previous = points[index - 1];
            return Math.abs(previous.x - point.x) > 0.001 ||
              Math.abs(previous.y - point.y) > 0.001 ||
              Math.abs(previous.z - point.z) > 0.001;
          });

          let clear = true;
          for (let i = 1; i < path.length; i++) {
            if (!segmentClearForRouting(
              path[i - 1],
              path[i],
              obstacles,
              entry.branchRouteOptions || routeOptions
            )) {
              clear = false;
              break;
            }
          }

          if (!clear) return;

          const score =
            polylineLengthMm(path) +
            countPolylineTurns(path) * grid * 50 +
            candidate.distance * 0.01;

          if (!bestDirect || score < bestDirect.score) {
            bestDirect = {
              points:path,
              attachment_point:target,
              fallback:false,
              warning:null,
              score
            };
          }
        });

        return bestDirect;
      }

      rankedEquipment.forEach(function(entry){
        let branchResult = tryDirectMainAttachment(entry);

        if (!branchResult) {
          branchResult = findGridPath3D(
            entry.start,
            null,
            obstacles,
            {
              ...(entry.branchRouteOptions || routeOptions),
              fixedRoutingY:routingY,
              preferredRoutingY:routingY,
              networkGoalPoints:mainNetworkPoints,
              networkGoalCandidateLimit:Number(options.networkGoalCandidateLimit) || 24,
              routeTurnPenaltyRatio:Math.max(
                Number(options.turnPenaltyRatio) || 20,
                Number(options.mainCorridorTurnPenaltyRatio) || 100
              )
            },
            new Set()
          );
        }

        if (!branchResult.points || branchResult.points.length < 1 || !branchResult.attachment_point) {
          return;
        }

        const plan = makeNetworkCablePlan(
          entry,
          group,
          branchResult.points,
          tree,
          branchResult.attachment_point,
          panelDrop,
          routeOptions.routingTrayWidthMm,
          {
            ...routeOptions,
            fixedRoutingY:routingY,
            panelStandoffDistanceMm:panelStandoff.distance_mm
          },
          Array.from(mainCorridorSegmentKeys)
        );

        if (!plan) return;

        plan.main_corridor_axis = axis;
        plan.main_corridor_secondary_coordinate_mm = secondary;
        cablePlans.push(plan);
        connected.add(entry.equipment.id);
      });

      const unresolved = prepared.filter(function(entry){
        return !connected.has(entry.equipment.id);
      });

      const totalLength = cablePlans.reduce(function(sum, plan){
        return sum + routeLengthMeters(plan.points);
      }, 0);
      const totalTurns = cablePlans.reduce(function(sum, plan){
        return sum + countPolylineTurns(plan.points);
      }, 0);
      const score =
        unresolved.length * 1000000000 +
        totalLength +
        totalTurns * Math.max(1, gridStep) * 0.001 +
        Math.abs(secondary - Number(panelStandoff.point[secondaryAxis])) * 0.01;

      const result = {
        routingY,
        panelHighPoint:rootPoint,
        panelDrop,
        cablePlans,
        connectedCount:cablePlans.length,
        unresolved,
        networkNodes:Array.from(tree.nodes.values()).map(clonePoint),
        score
      };

      if (
        !best ||
        result.connectedCount > best.connectedCount ||
        (
          result.connectedCount === best.connectedCount &&
          (
            result.routingY > best.routingY + 0.001 ||
            (
              Math.abs(result.routingY - best.routingY) <= 0.001 &&
              result.score < best.score
            )
          )
        )
      ) {
        best = result;
      }

      if (!unresolved.length) return result;
    }
  }

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
  const resolvedLevels = resolveNetworkRoutingLevels(
    panelStandoff.point,
    routeOptions,
    Number.isFinite(requestedLevelAttempts) && requestedLevelAttempts > 0
      ? requestedLevelAttempts
      : 0
  );
  // A floor has one Main-Tray routing elevation. Additional capacity batches
  // may use another clear XZ lane, but must not evade a blocked lane by
  // silently moving to a different Y level on the same floor.
  const sameFloorMainLevels = (routeOptions.occupiedMainCorridors || [])
    .filter(function(corridor){
      return String(corridor.floor_id == null ? '' : corridor.floor_id) ===
        String(group.floor_id == null ? '' : group.floor_id);
    })
    .map(function(corridor){ return Number(corridor.main_level_y_mm); })
    .filter(Number.isFinite);
  let levels = resolvedLevels;
  if (sameFloorMainLevels.length) {
    const levelCounts = new Map();
    sameFloorMainLevels.forEach(function(value){
      const key = rounded(value);
      levelCounts.set(key,(levelCounts.get(key)||0)+1);
    });
    const requiredLevel = Array.from(levelCounts.entries()).sort(function(a,b){
      return b[1]-a[1] || a[0]-b[0];
    })[0][0];
    levels = resolvedLevels.filter(function(value){
      return Math.abs(Number(value)-requiredLevel) <= 0.001;
    });
  }
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

    prepareFloorPanelConnections(
      group,
      prepared,
      panelStandoff,
      panelHighPoint,
      obstacles,
      routeOptions,
      routingY
    );

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

    // Existing Main corridors are physical obstacles. If this floor already
    // has a Main Tray and a clean parallel lane cannot be built at its fixed
    // routing level, do not run expensive alternate tree searches that would
    // risk overlapping the existing tray.
  // Larger groups use a global Main backbone so every equipment item gets
  // a real Main attachment node. The previous seed corridor must not become
  // the only usable Main target when the equipment count grows.
  const backboneThreshold = Math.max(5, attachLimit + 1);

  if (prepared.length >= backboneThreshold) {
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

      prepareFloorPanelConnections(
        group,
        prepared,
        panelStandoff,
        panelHighPoint,
        obstacles,
        routeOptions,
        routingY
      );

      const backboneResult = buildGlobalMainBackboneNetwork(
        group,
        prepared,
        panelStandoff,
        obstacles,
        routeOptions,
        options,
        routingY,
        panelDrop
      );

      if (!backboneResult) continue;

      if (
        !bestPartial ||
        backboneResult.connectedCount > bestPartial.connectedCount ||
        (
          backboneResult.connectedCount === bestPartial.connectedCount &&
          (
            backboneResult.routingY > bestPartial.routingY + 0.001 ||
            (
              Math.abs(backboneResult.routingY - bestPartial.routingY) <= 0.001 &&
              backboneResult.score < bestPartial.partialScore
            )
          )
        )
      ) {
        bestPartial = {
          ...backboneResult,
          partialScore:backboneResult.score
        };
      }

      if (backboneResult.unresolved.length === 0) {
        return backboneResult;
      }
    }
  }

  }

  if (Array.isArray(routeOptions.occupiedMainCorridors) &&
      routeOptions.occupiedMainCorridors.length > 0 &&
      options.allowOutsideRouting !== true) {
    // One attempt to route a distinct shared Main (including branches around
    // old trunk endpoints) is preferable to dozens of overlapping fallbacks.
    return bestPartial && bestPartial.unresolved && bestPartial.unresolved.length === 0
      ? bestPartial
      : null;
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

    prepareFloorPanelConnections(
      group,
      prepared,
      panelStandoff,
      panelHighPoint,
      obstacles,
      routeOptions,
      routingY
    );

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

      // Freeze the actual Main Tray network before any branch is registered.
      // Later motors may attach only to these Main nodes; a previous motor's
      // perpendicular branch must never become the target of another cable.
      const mainNetworkPoints = Array.from(tree.nodes.values()).map(clonePoint);
      const mainCorridorSegmentKeys = new Set();
      for (let i = 1; i < seedPath.points.length; i++) {
        const a = seedPath.points[i - 1];
        const b = seedPath.points[i];
        if (
          Math.abs(Number(a.y) - routingY) > 0.001 ||
          Math.abs(Number(b.y) - routingY) > 0.001
        ) continue;

        const segment = segmentRecord(
          a,
          b,
          seed.equipment.id,
          Number(seed.equipment.cable_diameter_mm) || 0
        );
        if (segment) mainCorridorSegmentKeys.add(segment.key);
      }

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
        },
        Array.from(mainCorridorSegmentKeys)
      );
      if (!seedPlan) continue;
      cablePlans.push(seedPlan);

      let pending = prepared.filter(function(entry){
        return !connected.has(entry.equipment.id);
      });
      let madeProgress = true;

      while (pending.length && madeProgress) {
        madeProgress = false;

        // Only the frozen Main Tray nodes are legal attachment targets.
        // Branch nodes are deliberately excluded to prevent branch-to-branch chaining.
        const networkPoints = mainNetworkPoints;
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
              ...(candidate.branchRouteOptions || routeOptions),
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
          },
          Array.from(mainCorridorSegmentKeys)
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
          (
            result.routingY > bestPartial.routingY + 0.001 ||
            (
              Math.abs(result.routingY - bestPartial.routingY) <= 0.001 &&
              partialScore > bestPartial.partialScore
            )
          )
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

function buildDirectFallbackCablePlan(entry, group, panelStandoff, obstacles, routeOptions, planningTrayWidth) {
  const targetPanel = entry.panel || group.panel;
  const targetStandoff = entry.panelStandoff || panelStandoff;
  const routingY = resolveHighestValidRoutingY(
    [entry.start, targetStandoff.point],
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
      ...(entry.branchRouteOptions || routeOptions),
      preferredRoutingY:routingY
    },
    new Set()
  );

  if (!highRoute.points || highRoute.points.length < 1) return null;

  const panelDrop = highPanelPoint.y === panelStandoff.point.y
    ? {
        points:[
          clonePoint(targetStandoff.point),
          clonePoint(highPanelPoint)
        ],
        warning:null,
        fallback:false
      }
    : findGridPath3D(
        targetStandoff.point,
        highPanelPoint,
        obstacles,
        {
          ...routeOptions,
          preferredRoutingY:routingY
        },
        new Set()
      );

  if (!panelDrop.points || panelDrop.points.length < 1) return null;

  const branchPoints = [];
  appendDistinctPoints(branchPoints, highRoute.points, 0);
  for (let i = panelDrop.points.length - 2; i >= 0; i--) {
    appendDistinctPoints(branchPoints, [panelDrop.points[i]]);
  }

  const points = [];
  appendDistinctPoints(points, [entry.equipment.anchor.point]);
  appendDistinctPoints(points, [entry.start]);
  appendDistinctPoints(points, highRoute.points, 0);
  for (let i = panelDrop.points.length - 2; i >= 0; i--) {
    appendDistinctPoints(points, [panelDrop.points[i]]);
  }
  appendDistinctPoints(points, [targetPanel.anchor.point]);

  return {
    equipment:entry.equipment,
    panel:(entry.panel || group.panel),
    cable:{
      name:entry.equipment.cable_name || 'Power Cable',
      diameter_mm:Number(entry.equipment.cable_diameter_mm) || 0
    },
    routing_start:entry.start,
    routing_goal:targetStandoff.point,
    main_corridor_equipment_id:null,
    main_corridor_routing_y_mm:Number.isFinite(Number(routingY)) ? Number(routingY) : null,
    standoff_distance_mm:Math.max(entry.startDistanceMm, targetStandoff.distance_mm),
    planning_tray_width_mm:planningTrayWidth,
    engineering_network_id:group.network_id || String(group.panel.id),
    branch_points:branchPoints,
    main_corridor_segment_keys:[],
    points,
    warning:'Shared Main Tray attachment was unavailable for this equipment; a direct branch route was used.',
    fallback:true
  };
}

function floorZoneContainsPoint(zone, point) {
  if (!zone || !Array.isArray(zone.points) || zone.points.length < 3 || !point) return false;

  const x = Number(point.x);
  const z = Number(point.z);
  if (!Number.isFinite(x) || !Number.isFinite(z)) return false;

  const polygon = zone.points.map(function(p){
    return {x:Number(p.x) || 0, z:Number(p.z) || 0};
  });

  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x;
    const zi = polygon[i].z;
    const xj = polygon[j].x;
    const zj = polygon[j].z;

    const intersects = ((zi > z) !== (zj > z)) &&
      (x < (xj - xi) * (z - zi) / ((zj - zi) || 1e-12) + xi);
    if (intersects) inside = !inside;
  }
  return inside;
}

function normalizeFloorZones(floorZones, globalBounds) {
  const source = Array.isArray(floorZones) ? floorZones : [];
  return source.map(function(zone, index){
    const points = Array.isArray(zone && zone.points)
      ? zone.points
          .filter(function(point){
            return point &&
              Number.isFinite(Number(point.x)) &&
              Number.isFinite(Number(point.y)) &&
              Number.isFinite(Number(point.z));
          })
          .map(clonePoint)
      : [];

    if (points.length < 4) return null;

    const yValues = points.map(function(point){ return point.y; });
    const baseY = Number.isFinite(Number(zone.base_y_mm))
      ? Number(zone.base_y_mm)
      : yValues.reduce(function(sum, value){ return sum + value; }, 0) / yValues.length;

    const xs = points.map(function(point){ return point.x; });
    const zs = points.map(function(point){ return point.z; });

    return {
      id:zone.id || 'floor-' + String(index + 1),
      name:zone.name || 'Floor ' + String(index + 1),
      points,
      base_y_mm:baseY,
      min_x_mm:Math.min.apply(null, xs),
      max_x_mm:Math.max.apply(null, xs),
      min_z_mm:Math.min.apply(null, zs),
      max_z_mm:Math.max.apply(null, zs),
      routing_bounds:zone.routing_bounds || null,
      source_bounds:globalBounds || null
    };
  }).filter(Boolean);
}

function clusterEquipmentFloorLevels(equipment, toleranceMm) {
  const tolerance = Math.max(50, Number(toleranceMm) || 500);
  const sorted = (equipment || [])
    .slice()
    .sort(function(a,b){
      return Number(a.anchor && a.anchor.point ? a.anchor.point.y : 0) -
        Number(b.anchor && b.anchor.point ? b.anchor.point.y : 0);
    });

  const clusters = [];
  sorted.forEach(function(item){
    const y = Number(item.anchor && item.anchor.point ? item.anchor.point.y : 0);
    const last = clusters[clusters.length - 1];
    if (!last || Math.abs(y - last.meanY) > tolerance) {
      clusters.push({
        key:'auto-floor-' + String(clusters.length + 1),
        meanY:y,
        equipmentIds:[item.id]
      });
      return;
    }

    last.equipmentIds.push(item.id);
    last.meanY = (
      last.meanY * (last.equipmentIds.length - 1) + y
    ) / last.equipmentIds.length;
  });

  return clusters;
}

function floorZoneBoundsOverlap(a, b) {
  if (!a || !b) return false;
  return a.min_x_mm <= b.max_x_mm &&
    a.max_x_mm >= b.min_x_mm &&
    a.min_z_mm <= b.max_z_mm &&
    a.max_z_mm >= b.min_z_mm;
}

function resolveEquipmentFloorZone(item, floorZones) {
  if (!item || !item.anchor || !item.anchor.point || !floorZones.length) return null;

  const y = Number(item.anchor.point.y);
  // Equipment anchors sit on equipment bodies, not necessarily on the floor
  // plane. The four-point zone defines a footprint; when floors overlap in
  // plan, choose the nearest floor elevation rather than imposing a fixed
  // 1000 mm vertical cutoff.
  const candidates = floorZones.filter(function(zone){
    return floorZoneContainsPoint(zone, item.anchor.point);
  });

  candidates.sort(function(a,b){
    return Math.abs(y - Number(a.base_y_mm)) - Math.abs(y - Number(b.base_y_mm));
  });

  return candidates[0] || null;
}

function choosePrimaryPanelForFloor(equipmentItems, panels) {
  if (!Array.isArray(panels) || !panels.length) return null;

  let cx = 0;
  let cz = 0;
  let count = 0;
  equipmentItems.forEach(function(item){
    if (!item || !item.anchor || !item.anchor.point) return;
    cx += Number(item.anchor.point.x) || 0;
    cz += Number(item.anchor.point.z) || 0;
    count++;
  });
  if (!count) return panels[0];

  cx /= count;
  cz /= count;

  return panels.slice().sort(function(a,b){
    const ap = a && a.anchor && a.anchor.point ? a.anchor.point : {};
    const bp = b && b.anchor && b.anchor.point ? b.anchor.point : {};
    const adx = (Number(ap.x) || 0) - cx;
    const adz = (Number(ap.z) || 0) - cz;
    const bdx = (Number(bp.x) || 0) - cx;
    const bdz = (Number(bp.z) || 0) - cz;
    return adx * adx + adz * adz - (bdx * bdx + bdz * bdz);
  })[0];
}

function prepareFloorPanelConnections(group, prepared, panelStandoff, panelHighPoint, obstacles, routeOptions, routingY) {
  const cache = new Map();

  prepared.forEach(function(entry){
    const panel = entry.panel || group.panel;
    if (!panel || !panel.anchor || !panel.anchor.point) return;

    const panelId = String(panel.id);
    if (!cache.has(panelId)) {
      const standoff = panel.id === group.panel.id
        ? panelStandoff
        : resolveAnchorStandoff(panel.anchor, obstacles, routeOptions);

      const highPoint = pointAtRoutingY(standoff.point, routingY);
      let connector = {
        points:[clonePoint(panelHighPoint), clonePoint(highPoint)],
        warning:null,
        fallback:false
      };

      if (
        Math.abs(panelHighPoint.x - highPoint.x) > 0.001 ||
        Math.abs(panelHighPoint.z - highPoint.z) > 0.001
      ) {
        connector = findGridPath3D(
          panelHighPoint,
          highPoint,
          obstacles,
          {
            ...routeOptions,
            fixedRoutingY:routingY,
            preferredRoutingY:routingY
          },
          new Set()
        );
      }

      const drop = Math.abs(highPoint.y - standoff.point.y) < 0.001
        ? {
            points:[clonePoint(standoff.point), clonePoint(highPoint)],
            warning:null,
            fallback:false
          }
        : findGridPath3D(
            standoff.point,
            highPoint,
            obstacles,
            {
              ...routeOptions,
              preferredRoutingY:routingY
            },
            new Set()
          );

      cache.set(panelId, {
        standoff,
        highPoint,
        connector,
        drop
      });
    }

    const connection = cache.get(panelId);
    entry.panelStandoff = connection.standoff;
    entry.panelHighPoint = connection.highPoint;
    entry.panelConnector = connection.connector;
    entry.panelDrop = connection.drop;
  });

  return cache;
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
    ceilingSafetyGapMm:Number.isFinite(Number(inputs && inputs.options && inputs.options.ceilingSafetyGapMm))
      ? Math.max(0, Number(inputs.options.ceilingSafetyGapMm))
      : 50,
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
    networkGoalCandidateLimit:Number(inputs && inputs.options && inputs.options.networkGoalCandidateLimit) || 24,
    parallelMainCorridorCandidateLimit:Number(inputs && inputs.options && inputs.options.parallelMainCorridorCandidateLimit) || 6,
    trayStopBeforeEquipmentMm:Number.isFinite(Number(inputs && inputs.options && inputs.options.trayStopBeforeEquipmentMm))
      ? Number(inputs.options.trayStopBeforeEquipmentMm)
      : 1000,
    exactCollisionRouting:inputs && inputs.options && inputs.options.exactCollisionRouting === true,
    allowOutsideRouting:inputs && inputs.options && inputs.options.allowOutsideRouting === true,
    floorZones:Array.isArray(inputs && inputs.options && inputs.options.floorZones)
      ? inputs.options.floorZones
      : [],
    floorLevelToleranceMm:Number.isFinite(Number(inputs && inputs.options && inputs.options.floorLevelToleranceMm))
      ? Number(inputs.options.floorLevelToleranceMm)
      : 1500,
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
  const normalizedFloorZones = normalizeFloorZones(
    options.floorZones,
    options.routingBounds
  );
  const autoFloorClusters = clusterEquipmentFloorLevels(
    equipment,
    options.floorLevelToleranceMm
  );
  const autoFloorByEquipmentId = new Map();
  autoFloorClusters.forEach(function(cluster){
    cluster.equipmentIds.forEach(function(id){
      autoFloorByEquipmentId.set(id, cluster);
    });
  });

  equipment.forEach(function(item) {
    const panel = validPanels.get(item.destination_panel_id);
    if (!panel) {
      warnings.push(item.name + ': no valid destination panel selected.');
      return;
    }

    const floorZone = resolveEquipmentFloorZone(item, normalizedFloorZones);
    const autoFloor = autoFloorByEquipmentId.get(item.id) || null;
    const floorKey = floorZone
      ? 'floor:' + String(floorZone.id)
      : (autoFloor ? 'auto:' + autoFloor.key : 'auto-floor-unknown');

    const group = groups.get(floorKey) || {
      panel:null,
      panels:new Map(),
      equipment:[],
      floor_zone:floorZone,
      floor_id:floorZone ? String(floorZone.id) : (autoFloor ? String(autoFloor.key) : null),
      floor_name:floorZone ? floorZone.name : (autoFloor ? 'Auto Floor' : 'Unassigned Floor')
    };

    group.equipment.push({
      ...item,
      __engineering_panel:panel.id
    });
    group.panels.set(panel.id, panel);
    groups.set(floorKey, group);
  });

  groups.forEach(function(group){
    group.panel = choosePrimaryPanelForFloor(
      group.equipment.map(function(item){ return validPanels.get(item.__engineering_panel); }),
      Array.from(group.panels.values())
    );
  });

  const groupedFloorLevels = Array.from(groups.values())
    .map(function(group){
      const values = group.equipment.map(function(item){
        return Number(item.anchor && item.anchor.point ? item.anchor.point.y : 0);
      }).filter(Number.isFinite);
      const zoneY = group.floor_zone ? Number(group.floor_zone.base_y_mm) : NaN;
      const meanY = Number.isFinite(zoneY)
        ? zoneY
        : (values.length
          ? values.reduce(function(sum, value){ return sum + value; }, 0) / values.length
          : 0);
      return {group, meanY};
    })
    .sort(function(a,b){ return a.meanY - b.meanY; });

  groupedFloorLevels.forEach(function(item, index){
    const previous = groupedFloorLevels[index - 1];
    const next = groupedFloorLevels[index + 1];
    const globalMinY = options.routingBounds && Number.isFinite(Number(options.routingBounds.minY))
      ? Number(options.routingBounds.minY)
      : -Infinity;
    const globalMaxY = options.routingBounds && Number.isFinite(Number(options.routingBounds.maxY))
      ? Number(options.routingBounds.maxY)
      : Number(options.ceilingY);

    let floorMinY = previous
      ? (previous.meanY + item.meanY) * 0.5
      : globalMinY;
    let floorMaxY = next
      ? (item.meanY + next.meanY) * 0.5
      : globalMaxY;

    if (item.group.floor_zone) {
      const zone = item.group.floor_zone;
      const higherOverlappingZones = normalizedFloorZones.filter(function(other){
        return String(other.id) !== String(zone.id) &&
          Number(other.base_y_mm) > Number(zone.base_y_mm) + 50 &&
          floorZoneBoundsOverlap(zone, other);
      }).sort(function(a,b){
        return Number(a.base_y_mm) - Number(b.base_y_mm);
      });

      // A higher overlapping floor defines the ceiling boundary for the
      // current floor. Using the midpoint between floors kept the Main Tray
      // unnecessarily low and made ceiling-gap settings ineffective.
      floorMinY = Number(zone.base_y_mm);
      floorMaxY = higherOverlappingZones.length
        ? Number(higherOverlappingZones[0].base_y_mm)
        : globalMaxY;
    }

    item.group.floor_base_y_mm = item.group.floor_zone &&
      Number.isFinite(Number(item.group.floor_zone.base_y_mm))
      ? Number(item.group.floor_zone.base_y_mm)
      : item.meanY;
    item.group.floor_min_y_mm = floorMinY;
    item.group.floor_max_y_mm = floorMaxY;
  });

  const cablePlans = [];
  const mainCorridors = [];
  const occupiedMainCorridors = [];

  const routingGroups = [];
  const configuredWidths = (options.standardTrayWidthsMm || [])
    .map(function(value){ return Number(value); })
    .filter(function(value){ return Number.isFinite(value) && value > 0; })
    .sort(function(a,b){ return a - b; });
  const maxStandardTrayWidth = configuredWidths.length
    ? configuredWidths[configuredWidths.length - 1]
    : 1200;
  const maxCableDiameterSum = Math.max(
    0,
    maxStandardTrayWidth * (Math.max(1, Math.min(100, options.fillLimitPercent)) / 100) -
    Math.max(0, options.traySideMarginMm || 25) * 2
  );

  groups.forEach(function(group) {
    const sourceEquipment = group.equipment.slice();
    const totalDiameter = sourceEquipment.reduce(function(sum, item){
      return sum + Math.max(0, Number(item.cable_diameter_mm) || 0);
    }, 0);

    const batches = [];
    if (totalDiameter <= maxCableDiameterSum + 0.001 || sourceEquipment.length <= 1) {
      batches.push({
        equipment:sourceEquipment,
        panel:group.panel,
        panels:group.panels
      });
    } else {
      const primaryAxis = options.structurePrimaryAxis === 'z' ? 'z' : 'x';
      const secondaryAxis = primaryAxis === 'x' ? 'z' : 'x';
      function sortEquipmentByLayout(items) {
        // Capacity batches should contain equipment from the same physical row
        // before moving to the next row. Sorting by the primary axis first
        // mixed opposite rows into each batch; their branch routes then crossed
        // Main lanes from earlier batches.
        return items.slice().sort(function(a,b){
          const as = Number(a.anchor && a.anchor.point ? a.anchor.point[secondaryAxis] : 0);
          const bs = Number(b.anchor && b.anchor.point ? b.anchor.point[secondaryAxis] : 0);
          if (Math.abs(as - bs) > 0.001) return as - bs;
          const av = Number(a.anchor && a.anchor.point ? a.anchor.point[primaryAxis] : 0);
          const bv = Number(b.anchor && b.anchor.point ? b.anchor.point[primaryAxis] : 0);
          return av - bv;
        });
      }
      function appendCapacityBatches(items, panel) {
        const sorted = sortEquipmentByLayout(items);
        let current = [];
        let currentDiameter = 0;
        sorted.forEach(function(item){
          const diameter = Math.max(0, Number(item.cable_diameter_mm) || 0);
          if (current.length && currentDiameter + diameter > maxCableDiameterSum + 0.001) {
            batches.push({
              equipment:current,
              panel:panel,
              panels:panel ? new Map([[panel.id,panel]]) : group.panels
            });
            current = [];
            currentDiameter = 0;
          }
          current.push(item);
          currentDiameter += diameter;
        });
        if (current.length) {
          batches.push({
            equipment:current,
            panel:panel,
            panels:panel ? new Map([[panel.id,panel]]) : group.panels
          });
        }
      }

      // Do not mix equipment for different panels into the same capacity
      // batch. Every batch must route to its actual destination panel; otherwise
      // the planner repeatedly tries to send loads from several panel locations
      // through one primary panel corridor and creates duplicate/overlapping Main trays.
      const itemsByPanel = new Map();
      sourceEquipment.forEach(function(item){
        const panelId = String(item.__engineering_panel || item.destination_panel_id || group.panel.id);
        if (!itemsByPanel.has(panelId)) itemsByPanel.set(panelId, []);
        itemsByPanel.get(panelId).push(item);
      });
      itemsByPanel.forEach(function(items, panelId){
        const destinationPanel = validPanels.get(panelId) || group.panel;
        appendCapacityBatches(items, destinationPanel);
      });
    }

    batches.forEach(function(batchRecord, batchIndex){
      const batchPanel = batchRecord.panel || group.panel;
      routingGroups.push({
        panel:batchPanel,
        panels:batchRecord.panels || group.panels,
        equipment:batchRecord.equipment,
        floor_zone:group.floor_zone,
        floor_id:group.floor_id,
        floor_name:group.floor_name,
        floor_base_y_mm:group.floor_base_y_mm,
        floor_min_y_mm:group.floor_min_y_mm,
        floor_max_y_mm:group.floor_max_y_mm,
        network_id:String(group.floor_id || batchPanel.id) + '::network-' + String(routingGroups.length + 1),
        source_batch_index:batchIndex
      });
    });
  });

  routingGroups.forEach(function(group) {
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
      ceilingY:Number.isFinite(Number(group.floor_max_y_mm))
        ? Number(group.floor_max_y_mm)
        : options.ceilingY,
      floorMinY:Number.isFinite(Number(group.floor_min_y_mm))
        ? Number(group.floor_min_y_mm)
        : null,
      floorMaxY:Number.isFinite(Number(group.floor_max_y_mm))
        ? Number(group.floor_max_y_mm)
        : null,
      centerlineClearanceMm:options.clearanceMm + planningTrayWidth / 2,
      routingTrayWidthMm:planningTrayWidth,
      occupiedMainCorridors:occupiedMainCorridors.map(function(corridor){return {
        network_id:corridor.network_id,
        floor_id:corridor.floor_id == null ? null : String(corridor.floor_id),
        points:(corridor.points||[]).map(clonePoint),
        width_mm:Number(corridor.width_mm)||100,
        height_mm:Number(corridor.height_mm)||100,
        main_level_y_mm:Number(corridor.main_level_y_mm)
      };}),
      preferredStandoffDistanceMm:Math.min(
        options.maxBodyDistanceMm + planningTrayWidth / 2,
        options.clearanceMm + planningTrayWidth / 2 + options.gridStepMm
      )
    };

    const panelStandoff = resolveAnchorStandoff(group.panel.anchor, obstacles, routeOptions);
    if (panelStandoff.warning) warnings.push(group.panel.name + ': ' + panelStandoff.warning);
    if (panelStandoff.valid === false) return;

    const prepared = group.equipment.map(function(item) {
      const cableDiameter = Math.max(0, Number(item.cable_diameter_mm) || 0);
      const branchRequiredWidth = (
        cableDiameter + options.traySideMarginMm * 2
      ) / (options.fillLimitPercent / 100);
      const branchTrayWidth = chooseTrayWidth(
        branchRequiredWidth,
        options.standardTrayWidthsMm
      );
      const branchRouteOptions = {
        ...routeOptions,
        centerlineClearanceMm:options.clearanceMm + branchTrayWidth / 2,
        routingTrayWidthMm:branchTrayWidth,
        preferredStandoffDistanceMm:Math.min(
          options.maxBodyDistanceMm + branchTrayWidth / 2,
          options.clearanceMm + branchTrayWidth / 2 + options.gridStepMm
        )
      };
      const equipmentStandoff = resolveAnchorStandoff(
        item.anchor,
        obstacles,
        branchRouteOptions
      );
      if (equipmentStandoff.warning) warnings.push(item.name + ': ' + equipmentStandoff.warning);
      return {
        equipment:item,
        panel:validPanels.get(item.__engineering_panel || item.destination_panel_id) || group.panel,
        start:equipmentStandoff.point,
        startDistanceMm:equipmentStandoff.distance_mm,
        branchTrayWidthMm:branchTrayWidth,
        branchRouteOptions,
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
            ...(entry.branchRouteOptions || routeOptions),
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
          panel:(entry.panel || group.panel),
          cable:{
            name:entry.equipment.cable_name || 'Power Cable',
            diameter_mm:Number(entry.equipment.cable_diameter_mm) || 0
          },
          routing_start:entry.start,
          routing_goal:panelStandoff.point,
          main_corridor_equipment_id:null,
          main_corridor_routing_y_mm:Number.isFinite(Number(routingY)) ? Number(routingY) : null,
          standoff_distance_mm:Math.max(entry.startDistanceMm, panelStandoff.distance_mm),
          planning_tray_width_mm:planningTrayWidth,
          engineering_network_id:group.network_id || String(group.panel.id),
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
      const hasOccupiedMain = Array.isArray(routeOptions.occupiedMainCorridors) &&
        routeOptions.occupiedMainCorridors.length > 0;
      if (hasOccupiedMain && options.allowOutsideRouting !== true) {
        warnings.push(
          group.panel.name + ': no collision-free, non-overlapping Main Tray lane was available for this group inside the model bounds.'
        );
        return;
      }

      let directFallbackCount = 0;

      prepared.forEach(function(entry) {
        const fallbackPlan = buildDirectFallbackCablePlan(
          entry,
          group,
          panelStandoff,
          obstacles,
          routeOptions,
          planningTrayWidth
        );

        if (fallbackPlan) {
          cablePlans.push(fallbackPlan);
          directFallbackCount++;
          return;
        }

        warnings.push(
          entry.equipment.name + ' → ' + group.panel.name +
          ': no collision-free route was found inside the configured body-distance corridor.'
        );
      });

      if (!directFallbackCount) {
        warnings.push(
          group.panel.name + ': no collision-free multi-terminal tray network or direct branch fallback could be built inside the configured body-distance corridor.'
        );
      }
      return;
    }

    const networkConnectedIds = new Set(
      networkResult.cablePlans.map(function(plan){ return plan.equipment.id; })
    );

    (networkResult.unresolved || []).forEach(function(entry) {
      const fallbackPlan = buildDirectFallbackCablePlan(
        entry,
        group,
        panelStandoff,
        obstacles,
        routeOptions,
        planningTrayWidth
      );

      if (fallbackPlan) {
        networkResult.cablePlans.push(fallbackPlan);
        networkConnectedIds.add(entry.equipment.id);
      }
    });

    networkResult.unresolved = prepared.filter(function(entry){
      return !networkConnectedIds.has(entry.equipment.id);
    });

    networkResult.cablePlans.forEach(function(plan) {
      cablePlans.push(plan);
    });

    if (networkResult.unresolved.length) {
      networkResult.unresolved.forEach(function(entry) {
        warnings.push(
          entry.equipment.name + ' → ' + group.panel.name +
          ': no collision-free branch to the shared tray network or direct fallback route was found.'
        );
      });
    }

    mainCorridors.push({
      panel_id:group.panel.id,
      floor_id:group.floor_id == null ? null : String(group.floor_id),
      network_id:group.network_id || String(group.panel.id),
      width_mm:planningTrayWidth,
      height_mm:Number(options.trayHeightMm) || 100,
      cable_ids:prepared.map(function(entry){ return entry.equipment.id; }),
      points:Array.isArray(networkResult.main_spine_points) ? networkResult.main_spine_points.map(clonePoint) : [],
      network_nodes:networkResult.networkNodes || [],
      panel_drop_points:(networkResult.panelDrop.points || []).map(clonePoint),
      main_level_y_mm:Number(networkResult.routingY),
      parallel_main_corridor:networkResult.parallel_main_corridor===true,
      panel_connection_points:Array.isArray(networkResult.panel_connection_points)
        ? networkResult.panel_connection_points.map(clonePoint)
        : []
    });

    // Derive occupied corridors from the actual Main Tray runs already
    // classified in the accumulated output, not just the seed backbone. A
    // multi-row network can contain more than one shared horizontal trunk.
    // Rebuild occupancy only for this network. Reprocessing every previous
    // cable plan after each batch grows quadratically with hundreds of loads.
    const latestMainCorridor=mainCorridors[mainCorridors.length-1];
    const currentNetworkMainRuns=buildTrayRuns(
      networkResult.cablePlans,
      options,
      latestMainCorridor ? [latestMainCorridor] : []
    ).filter(function(run){
      return run && run.classification==='main' &&
        Array.isArray(run.points) && run.points.length>=2;
    });
    currentNetworkMainRuns.forEach(function(run){
      const signature=[run.network_id||'',JSON.stringify(run.points),Number(run.width_mm)||100].join('|');
      const alreadyStored=occupiedMainCorridors.some(function(existing){
        return existing.signature===signature;
      });
      if(alreadyStored)return;
      occupiedMainCorridors.push({
        signature,
        network_id:run.network_id||null,
        floor_id:group.floor_id == null ? null : String(group.floor_id),
        points:run.points.map(clonePoint),
        width_mm:Number(run.width_mm)||100,
        height_mm:Number(run.height_mm)||Number(options.trayHeightMm)||100,
        main_level_y_mm:run.points.length?Number(run.points[0].y):NaN
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
      routing_method:'multi_terminal_obstacle_avoiding_steiner_astar',
      routing_priority:'shared_network_then_minimum_turns_then_length'
    }
  };
}
