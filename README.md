# Cable_tray

Industrial 3D Cable Tray / Cable Routing Designer.

This project is being built as an engineering application for industrial plants: import 3D/CAD/BIM models, route cables and cable trays in 3D, validate routing constraints, and derive quantities / BOQ from the model.

## Current foundation

- 3D engineering viewport with orbit, pan, zoom, grid and axes.
- Browser import for GLB/GLTF, OBJ and STL.
- Parametric Cable Route and Cable Tray Route objects.
- Route drawing at a configurable elevation.
- Automatic route length and basic elbow counting.
- Basic cable/tray quantity takeoff.
- Versioned project JSON save/load.
- BOQ CSV export.
- Separation between source model import and the internal engineering model.

## Run on Windows

From CMD:

    cd /d F:Cable_tray
    py server.py

Open http://127.0.0.1:8765

The first launch loads Three.js from jsDelivr, so the browser needs internet access for the libraries. The application server itself is local.

## Architecture direction

CAD / BIM / 3D files -> Import adapters -> Internal Engineering Model -> 3D View + Routing + Clash + Rules -> Quantity / BOQ.

The routing engine must never depend directly on one vendor file format. Future adapters will target DWG/DXF, STEP/IGES/Parasolid, SolidWorks, IFC/Revit/BIM, JT/CATIA/NX and additional industrial formats.

## Engineering data rule

Routes are engineering entities, not just drawing lines. They keep identity, specification, material, route geometry, dimensions and computed quantities so BOQ is derived from the model.

## Next stages

1. Harden the internal engineering model and schema.
2. Add native CAD/BIM importer adapters.
3. Add parametric tray fittings and supports.
4. Add cable schedules, bend radius and termination metadata.
5. Add automatic routing and obstacle avoidance.
6. Add 3D clash and clearance analysis.
7. Add tray fill, separation and support rules.
8. Expand BOQ to counts, weights and costing.
9. Preserve source-object mapping.
10. Optimize for large plant models and multi-floor projects.
