export type JointType =
  | 'fixed'
  | 'revolute'
  | 'continuous'
  | 'prismatic'
  | 'planar'
  | 'floating';

export type DOFMotion = 'rotation' | 'translation';

export interface JointLimit {
  lower?: number;
  upper?: number;
  velocity?: number;
  effort?: number;
}

export interface JointMimic {
  joint: number;
  multiplier?: number;
  offset?: number;
}

export interface JointDynamics {
  damping?: number;
  friction?: number;
}

export interface JointSafety {
  softLowerLimit?: number;
  softUpperLimit?: number;
  kPosition?: number;
  kVelocity?: number;
}

export interface JointCalibration {
  rising?: number;
  falling?: number;
}

export interface DOFDefinition {
  motion: DOFMotion;
  axis: [number, number, number];
  limit?: JointLimit;
  default?: number;
}

export interface LinkDefinition {
  name: string;
  node: number;
  visualNodes?: number[];
  collisionNodes?: number[];
  extras?: Record<string, unknown>;
}

export interface JointDefinition {
  name: string;
  type: JointType;
  parentLink: number;
  childLink: number;
  originNode: number;
  dofNodes: number[];
  dofs: DOFDefinition[];
  mimic?: JointMimic;
  dynamics?: JointDynamics;
  safety?: JointSafety;
  calibration?: JointCalibration;
}

export interface JointConfiguration {
  jointPositions: Array<number | number[]>;
}

export interface KinematicModelDefinition {
  name?: string;
  rootNode: number;
  links: LinkDefinition[];
  joints: JointDefinition[];
  configurations?: Record<string, JointConfiguration>;
  source?: Record<string, unknown>;
}

export interface ExtRobotKinematics {
  models: KinematicModelDefinition[];
}
