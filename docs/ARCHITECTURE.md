# Cable_tray Architecture Notes

## Core principle

The stable boundary is the Internal Engineering Model (IEM). Engineering logic should not be tied directly to a source CAD file or to the renderer.

## Source adapters

An adapter reads a supported file, extracts geometry and metadata, produces render/collision representations, and preserves source identifiers for traceability.

Planned adapter families include DWG/DXF, STEP/IGES/Parasolid, SolidWorks, IFC/Revit/BIM, JT/CATIA/NX and GLTF/GLB/OBJ/STL.

## Internal entities

Project, CoordinateSystem, Levels, SourceModels, Equipment, Structures, Pipes, Trays, Cables, Supports, Fittings, Rulesets and BOQ.

Imported objects should retain id, source_file, source_object_id, name, type, transform, bounding_box, geometry_ref and properties.

## Routing

Routes should be graph/path entities so the same data can be used by manual routing, orthogonal routing, shortest-path routing, cost-aware routing, obstacle avoidance, existing-tray preference, separation constraints and clearance constraints.

## Geometry separation

Keep engineering/source geometry, collision geometry, measurement geometry and display meshes conceptually separate. This is important for large industrial models.

## Quantity engine

Route geometry produces length, fittings, supports, cable properties and tray-fill inputs. Those become quantity lines, which then feed BOQ, cost and reports. Manual quantity overrides must be explicit and auditable.
