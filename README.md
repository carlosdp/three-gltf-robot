# three-gltf-robot

GLTFLoader plugin for `EXT_robot_kinematics` with a lightweight runtime for joint control.

## Install

```bash
pnpm install
```

## Build

```bash
pnpm run build
```

## Test

```bash
pnpm run test
```

## Example

```bash
pnpm run dev
```

Open the Vite URL to view the full-screen robot viewer and joint controls.

## Usage

```ts
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { registerRobotKinematics, getRobotKinematics } from 'three-gltf-robot';

const loader = new GLTFLoader();
registerRobotKinematics(loader);

loader.load('robot.glb', (gltf) => {
  const models = getRobotKinematics(gltf);
  const model = models[0];
  model?.setJointValue('shoulder_pan', 0.3);
});
```

`RobotKinematicsModel` exposes helpers for getting links/joints, applying configurations, and setting joint DOF values.

## URDF ZIP Conversion (Browser)

Convert a ZIP containing a URDF + assets into a glTF JSON with `EXT_robot_kinematics` attached.

```ts
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { convertUrdfZipToGltf, registerRobotKinematics, getRobotKinematics } from 'three-gltf-robot';

const loader = new GLTFLoader();
registerRobotKinematics(loader);

const file = input.files?.[0];
if (file) {
  const result = await convertUrdfZipToGltf(file);
  loader.parse(result.gltfJson, '', (gltf) => {
    const models = getRobotKinematics(gltf);
    console.log(models[0]);
  });
}
```

Supported mesh types: `.glb`, `.gltf`, `.stl`, `.obj`, `.dae` plus URDF primitives (box, cylinder, sphere).
