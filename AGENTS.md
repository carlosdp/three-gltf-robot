# AGENTS.md

## Architecture Overview
- **Loader plugin**: `RobotKinematicsPlugin` hooks into `GLTFLoader` to read `EXT_robot_kinematics` from the asset root and resolve node references into live `THREE.Object3D` instances.
- **Runtime model**: `RobotKinematicsModel` owns link and joint runtime objects, exposes joint setters, and updates DOF node TRS values in place.

## Data Flow
1. `GLTFLoader` parses the asset.
2. The plugin reads `extensions.EXT_robot_kinematics` and resolves all referenced nodes.
3. Runtime objects are created and attached to `gltf.userData.robotKinematics`.
4. Calls to `setJointValue` update DOF node transforms, keeping the scene graph consistent.

## Folder Structure
- `src/index.ts` — public exports.
- `src/loader/` — GLTFLoader plugin and helpers.
- `src/runtime/` — runtime kinematics model, joint/link definitions, URDF bridge.
- `src/utils/` — small math utilities.
- `src/types/` — local type shims for external packages.
- `examples/` — Vite-based fullscreen viewer with GUI joint controls.
- `dist/` — build output (generated).

## Key Conventions
- Joint origin nodes are treated as fixed transforms, while DOF nodes are updated per joint value.
- DOF values are clamped to limits when present (override with `{ clamp: false }`).
- Multi-DOF joints are represented as ordered `dofs[]` aligned with `dofNodes[]`.

## Extending
- Add validators or conversion helpers in `src/runtime/` to keep loader logic thin.
- Keep new rendering or UI demonstrations under `examples/`.

## Documentation
- Keep `LLM.txt` up to date whenever public APIs or runtime behaviors change.
