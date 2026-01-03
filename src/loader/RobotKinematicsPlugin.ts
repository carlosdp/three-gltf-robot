import type { GLTFParser, GLTF } from 'three/examples/jsm/loaders/GLTFLoader.js';
import {
  createRobotKinematicsModels,
  RobotKinematicsModel,
} from '../runtime/RobotKinematics.js';
import type { ExtRobotKinematics } from '../runtime/types.js';

export class RobotKinematicsPlugin {
  readonly name = 'EXT_robot_kinematics';

  private readonly parser: GLTFParser;

  constructor(parser: GLTFParser) {
    this.parser = parser;
  }

  async afterRoot(gltf: GLTF): Promise<void> {
    const extension = this.parser.json.extensions?.EXT_robot_kinematics as
      | ExtRobotKinematics
      | undefined;
    if (!extension) return;

    const models = await createRobotKinematicsModels(
      extension,
      (index) => this.parser.getDependency('node', index)
    );

    gltf.userData.robotKinematics = models;
    gltf.userData.extensions = gltf.userData.extensions ?? {};
    gltf.userData.extensions.EXT_robot_kinematics = models;
  }
}

export function registerRobotKinematics(
  loader: { register: (callback: (parser: GLTFParser) => unknown) => void }
): void {
  loader.register((parser) => new RobotKinematicsPlugin(parser));
}

export function getRobotKinematics(gltf: GLTF): RobotKinematicsModel[] {
  return (gltf.userData.robotKinematics ?? []) as RobotKinematicsModel[];
}
