import { Object3D, Quaternion, Vector3 } from 'three';
import { axisAngleQuaternion, clamp, vec3FromArray } from '../utils/math.js';
import {
  DOFDefinition,
  ExtRobotKinematics,
  JointDefinition,
  JointType,
  KinematicModelDefinition,
  LinkDefinition
} from './types.js';

export type JointValue = number | number[];

export interface LinkRuntime {
  name: string;
  node: Object3D;
  visualNodes: Object3D[];
  collisionNodes: Object3D[];
  extras?: Record<string, unknown>;
}

export interface DOFRuntime {
  definition: DOFDefinition;
  node: Object3D;
  basePosition: Vector3;
  baseQuaternion: Quaternion;
  value: number;
}

export interface JointRuntime {
  name: string;
  type: JointType;
  parentLink: LinkRuntime;
  childLink: LinkRuntime;
  originNode: Object3D;
  dofNodes: Object3D[];
  dofs: DOFDefinition[];
  dofStates: DOFRuntime[];
  mimic?: JointDefinition['mimic'];
  dynamics?: JointDefinition['dynamics'];
  safety?: JointDefinition['safety'];
  calibration?: JointDefinition['calibration'];
  values: number[];
}

export class RobotKinematicsModel {
  readonly name: string;
  readonly root: Object3D;
  readonly links: LinkRuntime[];
  readonly joints: JointRuntime[];
  readonly configurations: KinematicModelDefinition['configurations'];
  readonly source?: Record<string, unknown>;

  private readonly jointMap = new Map<string, JointRuntime>();
  private readonly linkMap = new Map<string, LinkRuntime>();
  private readonly mimicTargets = new Map<string, JointRuntime[]>();

  constructor(
    name: string,
    root: Object3D,
    links: LinkRuntime[],
    joints: JointRuntime[],
    configurations: KinematicModelDefinition['configurations'],
    source: Record<string, unknown> | undefined
  ) {
    this.name = name;
    this.root = root;
    this.links = links;
    this.joints = joints;
    this.configurations = configurations;
    this.source = source;

    for (const link of links) {
      this.linkMap.set(link.name, link);
    }
    for (const joint of joints) {
      this.jointMap.set(joint.name, joint);
      if (joint.mimic) {
        const targetName = joints[joint.mimic.joint]?.name;
        if (targetName) {
          const list = this.mimicTargets.get(targetName) ?? [];
          list.push(joint);
          this.mimicTargets.set(targetName, list);
        }
      }
    }

  }

  getJoint(nameOrIndex: string | number): JointRuntime | undefined {
    if (typeof nameOrIndex === 'number') {
      return this.joints[nameOrIndex];
    }
    return this.jointMap.get(nameOrIndex);
  }

  getLink(nameOrIndex: string | number): LinkRuntime | undefined {
    if (typeof nameOrIndex === 'number') {
      return this.links[nameOrIndex];
    }
    return this.linkMap.get(nameOrIndex);
  }

  getJointValue(nameOrIndex: string | number): JointValue | undefined {
    const joint = this.getJoint(nameOrIndex);
    if (!joint) return undefined;
    return joint.values.length === 1 ? joint.values[0] : [...joint.values];
  }

  setJointValue(
    nameOrIndex: string | number,
    value: JointValue,
    options: { clamp?: boolean } = {}
  ): void {
    const joint = this.getJoint(nameOrIndex);
    if (!joint) return;
    const values = normalizeJointValue(joint, value);
    this.applyJointValues(joint, values, options);
    this.applyMimics(joint, options);
  }

  setJointDofValue(
    nameOrIndex: string | number,
    dofIndex: number,
    value: number,
    options: { clamp?: boolean } = {}
  ): void {
    const joint = this.getJoint(nameOrIndex);
    if (!joint) return;
    const values = joint.values.slice();
    if (dofIndex < 0 || dofIndex >= values.length) return;
    values[dofIndex] = value;
    this.applyJointValues(joint, values, options);
    this.applyMimics(joint, options);
  }

  setJointValues(
    values: Record<string, JointValue>,
    options: { clamp?: boolean } = {}
  ): void {
    for (const [jointName, jointValue] of Object.entries(values)) {
      this.setJointValue(jointName, jointValue, options);
    }
  }

  applyConfiguration(name: string, options: { clamp?: boolean } = {}): void {
    const config = this.configurations?.[name];
    if (!config) return;
    config.jointPositions.forEach((position, index) => {
      this.setJointValue(index, position, options);
    });
  }

  private applyJointValues(
    joint: JointRuntime,
    values: number[],
    options: { clamp?: boolean }
  ): void {
    const clampValues = options.clamp !== false;

    joint.values = values.map((value, index) => {
      const dof = joint.dofStates[index];
      const limit = dof?.definition.limit;
      if (!dof) return value;
      const nextValue = clampValues
        ? clamp(value, limit?.lower, limit?.upper)
        : value;
      const axis = dof.definition.axis;

      if (dof.definition.motion === 'translation') {
        const axisVec = vec3FromArray(axis);
        if (axisVec.lengthSq() > 0) axisVec.normalize();
        dof.node.position.copy(dof.basePosition).addScaledVector(axisVec, nextValue);
        dof.node.quaternion.copy(dof.baseQuaternion);
      } else {
        const delta = axisAngleQuaternion(axis, nextValue);
        dof.node.quaternion.copy(dof.baseQuaternion).multiply(delta);
        dof.node.position.copy(dof.basePosition);
      }

      return nextValue;
    });

  }

  private applyMimics(joint: JointRuntime, options: { clamp?: boolean }): void {
    const dependents = this.mimicTargets.get(joint.name);
    if (!dependents || joint.values.length === 0) return;
    const sourceValue = joint.values[0];
    for (const mimicJoint of dependents) {
      const mimic = mimicJoint.mimic;
      if (!mimic) continue;
      const multiplier = mimic.multiplier ?? 1;
      const offset = mimic.offset ?? 0;
      const value = sourceValue * multiplier + offset;
      this.applyJointValues(mimicJoint, [value], options);
    }
  }

}

export async function createRobotKinematicsModels(
  extension: ExtRobotKinematics,
  resolveNode: (index: number) => Promise<Object3D>
): Promise<RobotKinematicsModel[]> {
  const models: RobotKinematicsModel[] = [];

  for (const modelDef of extension.models) {
    const root = await resolveNode(modelDef.rootNode);
    const links = await resolveLinks(modelDef.links, resolveNode);
    const joints = await resolveJoints(modelDef.joints, links, resolveNode);

    const model = new RobotKinematicsModel(
      modelDef.name ?? 'robot',
      root,
      links,
      joints,
      modelDef.configurations,
      modelDef.source
    );

    for (const joint of joints) {
      const defaults = joint.dofs.map((dof) => dof.default ?? 0);
      model.setJointValue(joint.name, defaults, { clamp: true });
    }

    models.push(model);
  }

  return models;
}

async function resolveLinks(
  linkDefs: LinkDefinition[],
  resolveNode: (index: number) => Promise<Object3D>
): Promise<LinkRuntime[]> {
  const results: LinkRuntime[] = [];

  for (const linkDef of linkDefs) {
    const node = await resolveNode(linkDef.node);
    const visualNodes = await Promise.all(
      (linkDef.visualNodes ?? []).map((index) => resolveNode(index))
    );
    const collisionNodes = await Promise.all(
      (linkDef.collisionNodes ?? []).map((index) => resolveNode(index))
    );
    results.push({
      name: linkDef.name,
      node,
      visualNodes,
      collisionNodes,
      extras: linkDef.extras
    });
  }

  return results;
}

async function resolveJoints(
  jointDefs: JointDefinition[],
  links: LinkRuntime[],
  resolveNode: (index: number) => Promise<Object3D>
): Promise<JointRuntime[]> {
  const results: JointRuntime[] = [];

  for (const jointDef of jointDefs) {
    const parentLink = links[jointDef.parentLink];
    const childLink = links[jointDef.childLink];
    const originNode = await resolveNode(jointDef.originNode);
    const dofNodes = await Promise.all(jointDef.dofNodes.map((index) => resolveNode(index)));

    const dofStates: DOFRuntime[] = jointDef.dofs.map((definition, index) => {
      const node = dofNodes[index];
      return {
        definition,
        node,
        basePosition: node.position.clone(),
        baseQuaternion: node.quaternion.clone(),
        value: definition.default ?? 0
      };
    });

    results.push({
      name: jointDef.name,
      type: jointDef.type,
      parentLink,
      childLink,
      originNode,
      dofNodes,
      dofs: jointDef.dofs,
      dofStates,
      mimic: jointDef.mimic,
      dynamics: jointDef.dynamics,
      safety: jointDef.safety,
      calibration: jointDef.calibration,
      values: dofStates.map((dof) => dof.value)
    });
  }

  return results;
}

function normalizeJointValue(joint: JointRuntime, value: JointValue): number[] {
  if (Array.isArray(value)) {
    const values = value.slice(0, joint.dofs.length);
    while (values.length < joint.dofs.length) values.push(0);
    return values;
  }
  if (joint.dofs.length === 1) return [value];
  const values = joint.values.length ? joint.values.slice() : joint.dofs.map(() => 0);
  values[0] = value;
  return values;
}
