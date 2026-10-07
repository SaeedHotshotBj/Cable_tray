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

function normalizeBounds(start, goal, obstacles, step, paddingMm) {
  const xs = [start.x, goal.x];
  const zs = [start.z, goal.z];

  obstacles.forEach(function(rect) {
    xs.push(rect.minX, rect.maxX);
    zs.push(rect.minZ, rect.maxZ);
  });

  let minX = Math.min.apply(Math, xs) - paddingMm;
  let maxX = Math.max.apply(Math, xs) + paddingMm;
  let minZ = Math.min.apply(Math, zs) - paddingMm;
  let maxZ = Math.max.apply(Math, zs) + paddingMm;

  let minIx = Math.floor(minX / step);
  let maxIx = Math.ceil(maxX / step);
  let minIz = Math.floor(minZ / step);
  let maxIz = Math.ceil(maxZ / step);

  return { minIx, maxIx, minIz, maxIz };
}

function adaptGridStep(start, goal, obstacles, baseStep, maxCells, paddingMm) {
  let step = Math.max(50, Number(baseStep) || 250);

  for (let attempt = 0; attempt < 8; attempt++) {
    const bounds = normalizeBounds(start, goal, obstacles, step, paddingMm);
    const width = bounds.maxIx - bounds.minIx + 1;
    const depth = bounds.maxIz - bounds.minIz + 1;
    if (width * depth <= maxCells) return { step, bounds };

    const scale = Math.sqrt((width * depth) / maxCells);
    step = Math.max(step + 50, Math.ceil(step * scale / 50) * 50);
  }

  return { step, bounds: normalizeBounds(start, goal, obstacles, step, paddingMm) };
}

function buildBlockedSet(obstacles, step, bounds, clearanceMm) {
  const blocked = new Set();
  const minIx = bounds.minIx;
  const maxIx = bounds.maxIx;
  const minIz = bounds.minIz;
  const maxIz = bounds.maxIz;
  const clearance = Math.max(0, Number(clearanceMm) || 0);

  obstacles.forEach(function(rect) {
    const minX = rect.minX - clearance;
    const maxX = rect.maxX + clearance;
    const minZ = rect.minZ - clearance;
    const maxZ = rect.maxZ + clearance;

    const ix0 = Math.max(minIx, Math.ceil(minX / step));
    const ix1 = Math.min(maxIx, Math.floor(maxX / step));
    const iz0 = Math.max(minIz, Math.ceil(minZ / step));
    const iz1 = Math.min(maxIz, Math.floor(maxZ / step));

    for (let ix = ix0; ix <= ix1; ix++) {
      for (let iz = iz0; iz <= iz1; iz++) {
        blocked.add(pointKey(ix, iz));
      }
    }
  });

  return blocked;
}

function removeEndpointBlocks(blocked, startCell, goalCell) {
  blocked.delete(pointKey(startCell.ix, startCell.iz));
  blocked.delete(pointKey(goalCell.ix, goalCell.iz));
}

function cellInsideBounds(ix, iz, bounds) {
  return ix >= bounds.minIx && ix <= bounds.maxIx && iz >= bounds.minIz && iz <= bounds.maxIz;
}

function makeCell(ix, iz, dir) {
  return { ix, iz, dir };
}

function cellKey(cell) {
  return cell.ix + ',' + cell.iz + ',' + cell.dir;
}

function reconstructPath(cameFrom, current) {
  const result = [current];
  let key = cellKey(current);

  while (cameFrom.has(key)) {
    const previous = cameFrom.get(key);
    result.push(previous);
    key = cellKey(previous);
  }

  result.reverse();
  return result;
}

function findHorizontalGridPath(start, goal, obstacles, options, reuseCells) {
  const baseStep = Math.max(50, Number(options.gridStepMm) || 250);
  const paddingMm = Math.max(baseStep * 4, Number(options.routingPaddingMm) || 1500);
  const maxCells = Math.max(2500, Number(options.maxGridCells) || 40000);
  const adapted = adaptGridStep(start, goal, obstacles, baseStep, maxCells, paddingMm);
  const step = adapted.step;
  const bounds = adapted.bounds;

  const startCell = {
    ix: Math.round(start.x / step),
    iz: Math.round(start.z / step)
  };
  const goalCell = {
    ix: Math.round(goal.x / step),
    iz: Math.round(goal.z / step)
  };

  const blocked = buildBlockedSet(obstacles, step, bounds, Number(options.clearanceMm) || 0);
  removeEndpointBlocks(blocked, startCell, goalCell);

  const turnPenalty = step * Math.max(0, Number(options.turnPenaltyRatio) || 0.04);
  const reuseBonus = Math.min(0.8, Math.max(0, Number(options.reuseBonus) || 0.45));

  const open = new MinHeap();
  const cameFrom = new Map();
  const gScore = new Map();
  const startCellState = makeCell(startCell.ix, startCell.iz, -1);
  const startKey = cellKey(startCellState);
  gScore.set(startKey, 0);
  open.push({ ix:startCellState.ix, iz:startCellState.iz, dir:-1, g:0, f:distanceXZ(
    {x:start.ix * step, z:start.iz * step},
    {x:goalCell.ix * step, z:goalCell.iz * step}
  ) });

  const directions = [
    { x: 1, z: 0, dir: 0 },
    { x: 0, z: 1, dir: 1 },
    { x: -1, z: 0, dir: 2 },
    { x: 0, z: -1, dir: 3 }
  ];

  const closed = new Set();
  let goalState = null;
  let iterations = 0;
  const maxIterations = maxCells * 8;

  while (open.length && iterations++ < maxIterations) {
    const current = open.pop();
    const currentCellKey = cellKey(current);
    if (closed.has(currentCellKey)) continue;

    if (current.ix === goalCell.ix && current.iz === goalCell.iz) {
      goalState = current;
      break;
    }

    closed.add(currentCellKey);

    for (let i = 0; i < directions.length; i++) {
      const stepDir = directions[i];
      const nx = current.ix + stepDir.x;
      const nz = current.iz + stepDir.z;
      if (!cellInsideBounds(nx, nz, bounds)) continue;

      const nextKey2d = pointKey(nx, nz);
      if (blocked.has(nextKey2d)) continue;

      const next = makeCell(nx, nz, stepDir.dir);
      const nextKey = cellKey(next);
      if (closed.has(nextKey)) continue;

      let moveCost = step;
      if (current.dir >= 0 && current.dir !== stepDir.dir) moveCost += turnPenalty;
      if (reuseCells && reuseCells.has(nextKey2d)) moveCost *= (1 - reuseBonus);

      const tentative = current.g + moveCost;
      const previousBest = gScore.get(nextKey);
      if (previousBest != null && tentative >= previousBest) continue;

      gScore.set(nextKey, tentative);
      cameFrom.set(nextKey, makeCell(current.ix, current.iz, current.dir));

      const heuristic = distanceXZ(
        {x:nx * step, z:nz * step},
        {x:goalCell.ix * step, z:goalCell.iz * step}
      );
      open.push({
        ix:nx,
        iz:nz,
        dir:stepDir.dir,
        g:tentative,
        f:tentative + heuristic
      });
    }
  }

  if (!goalState) {
    const directA = [
      {x:start.x, z:start.z},
      {x:goal.x, z:start.z},
      {x:goal.x, z:goal.z}
    ];
    const directB = [
      {x:start.x, z:start.z},
      {x:start.x, z:goal.z},
      {x:goal.x, z:goal.z}
    ];
    const lenA = distanceXZ(directA[0], directA[1]) + distanceXZ(directA[1], directA[2]);
    const lenB = distanceXZ(directB[0], directB[1]) + distanceXZ(directB[1], directB[2]);
    const fallback = lenA <= lenB ? directA : directB;
    return {
      step,
      points: fallback.map(function(p){ return {x:p.x, y:start.y, z:p.z}; }),
      fallback: true,
      warning: 'No obstacle-free routing-grid path was found; direct rectilinear fallback was used.'
    };
  }

  const cells = reconstructPath(cameFrom, goalState);
  const points = cells.map(function(cell) {
    return {
      x: cell.ix * step,
      y: start.y,
      z: cell.iz * step
    };
  });

  points[0] = { x:start.x, y:start.y, z:start.z };
  points[points.length - 1] = { x:goal.x, y:goal.y, z:goal.z };

  const compressed = [];
  points.forEach(function(point) {
    if (!compressed.length) {
      compressed.push(point);
      return;
    }
    const previous = compressed[compressed.length - 1];
    if (Math.abs(point.x - previous.x) < 0.001 && Math.abs(point.z - previous.z) < 0.001) return;

    if (compressed.length >= 2) {
      const before = compressed[compressed.length - 2];
      if (areCollinearForward(before, previous, point)) {
        compressed[compressed.length - 1] = point;
        return;
      }
    }
    compressed.push(point);
  });

  return { step, points:compressed, fallback:false, warning:null };
}

function buildCablePoints(equipment, panel, horizontalPoints, trayElevationMm) {
  const source = clonePoint(equipment.anchor.point);
  const target = clonePoint(panel.anchor.point);
  const sourceTray = { x:source.x, y:trayElevationMm, z:source.z };
  const targetTray = { x:target.x, y:trayElevationMm, z:target.z };

  const points = [];
  points.push(source);
  if (Math.abs(source.y - trayElevationMm) > 0.01) points.push(sourceTray);

  horizontalPoints.forEach(function(point, index) {
    if (!index && points.length && Math.abs(points[points.length - 1].x - point.x) < 0.001 && Math.abs(points[points.length - 1].z - point.z) < 0.001) {
      return;
    }
    points.push({x:point.x, y:trayElevationMm, z:point.z});
  });

  if (Math.abs(target.y - trayElevationMm) > 0.01) points.push(targetTray);
  points.push(target);

  const compressed = [];
  points.forEach(function(point) {
    if (!compressed.length) {
      compressed.push(point);
      return;
    }
    const previous = compressed[compressed.length - 1];
    const dx = Math.abs(point.x - previous.x);
    const dy = Math.abs(point.y - previous.y);
    const dz = Math.abs(point.z - previous.z);
    if (dx < 0.001 && dy < 0.001 && dz < 0.001) return;
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
    routingElevationMm:Number(inputs && inputs.options && inputs.options.routingElevationMm) || 3000,
    gridStepMm:Number(inputs && inputs.options && inputs.options.gridStepMm) || 250,
    clearanceMm:Number.isFinite(Number(inputs && inputs.options && inputs.options.clearanceMm))
      ? Number(inputs.options.clearanceMm)
      : 100,
    fillLimitPercent:Number(inputs && inputs.options && inputs.options.fillLimitPercent) || 80,
    trayHeightMm:Number(inputs && inputs.options && inputs.options.trayHeightMm) || 100,
    mainMinCables:Number(inputs && inputs.options && inputs.options.mainMinCables) || 2,
    turnPenaltyRatio:Number(inputs && inputs.options && inputs.options.turnPenaltyRatio) || 0.04,
    reuseBonus:Number(inputs && inputs.options && inputs.options.reuseBonus) || 0.45,
    traySideMarginMm:Number(inputs && inputs.options && inputs.options.traySideMarginMm) || 25,
    standardTrayWidthsMm:inputs && inputs.options && Array.isArray(inputs.options.standardTrayWidthsMm)
      ? inputs.options.standardTrayWidthsMm
      : [100,150,200,300,400,500,600,800,1000,1200],
    maxGridCells:Number(inputs && inputs.options && inputs.options.maxGridCells) || 40000,
    routingPaddingMm:Number(inputs && inputs.options && inputs.options.routingPaddingMm) || 1500
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
    const group = groups.get(panel.id) || { panel, equipment:[] };
    group.equipment.push(item);
    groups.set(panel.id,group);
  });

  const cablePlans = [];
  groups.forEach(function(group) {
    const ordered = group.equipment.slice().sort(function(a,b) {
      return distanceXZ(b.anchor.point, group.panel.anchor.point) -
        distanceXZ(a.anchor.point, group.panel.anchor.point);
    });

    const reuseCells = new Set();

    ordered.forEach(function(item) {
      const horizontalStart = {x:item.anchor.point.x,y:options.routingElevationMm,z:item.anchor.point.z};
      const horizontalGoal = {x:group.panel.anchor.point.x,y:options.routingElevationMm,z:group.panel.anchor.point.z};
      const horizontal = findHorizontalGridPath(
        horizontalStart,
        horizontalGoal,
        obstacles,
        options,
        reuseCells
      );

      horizontal.points.forEach(function(point) {
        reuseCells.add(pointKey(Math.round(point.x / horizontal.step), Math.round(point.z / horizontal.step)));
      });

      const points = buildCablePoints(item, group.panel, horizontal.points, options.routingElevationMm);
      cablePlans.push({
        equipment:item,
        panel:group.panel,
        cable:{
          name:item.cable_name || 'Power Cable',
          diameter_mm:Number(item.cable_diameter_mm) || 0
        },
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
    options
  };
}
