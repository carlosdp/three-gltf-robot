import JSZip from 'jszip';
import {
  BoxGeometry,
  Color,
  CylinderGeometry,
  Euler,
  LoadingManager,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  Object3D,
  Quaternion,
  SphereGeometry,
  Vector3
} from 'three';
import { GLTFExporter } from 'three/examples/jsm/exporters/GLTFExporter.js';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import type {
  DOFDefinition,
  ExtRobotKinematics,
  JointCalibration,
  JointDefinition,
  JointDynamics,
  JointLimit,
  JointMimic,
  JointSafety,
  JointType,
  KinematicModelDefinition,
  LinkDefinition
} from '../runtime/types.js';

type ZipInput = Blob | ArrayBuffer | Uint8Array;

type MeshGeometry =
  | { type: 'box'; size: Vector3 }
  | { type: 'cylinder'; radius: number; length: number }
  | { type: 'sphere'; radius: number }
  | { type: 'mesh'; filename: string; scale: Vector3 };

type UrdfPose = {
  xyz: Vector3;
  rpy: Vector3;
};

type UrdfMaterial = {
  color?: Color;
  opacity?: number;
};

type UrdfVisual = {
  origin: UrdfPose;
  geometry: MeshGeometry;
  material?: UrdfMaterial;
};

type UrdfLink = {
  name: string;
  visuals: UrdfVisual[];
};

type UrdfJoint = {
  name: string;
  type: JointType;
  parent: string;
  child: string;
  origin: UrdfPose;
  axis: Vector3;
  limit?: JointLimit;
  mimic?: UrdfMimic;
  dynamics?: JointDynamics;
  safety?: JointSafety;
  calibration?: JointCalibration;
};

type UrdfModel = {
  name: string;
  links: UrdfLink[];
  joints: UrdfJoint[];
  materials: Map<string, UrdfMaterial>;
};

type UrdfMimic = {
  joint: string;
  multiplier?: number;
  offset?: number;
};

const URDF_TO_GLTF = new Matrix4().set(
  0, 1, 0, 0,
  0, 0, 1, 0,
  1, 0, 0, 0,
  0, 0, 0, 1
);
const GLTF_TO_URDF = new Matrix4().copy(URDF_TO_GLTF).transpose();
const URDF_TO_GLTF_QUAT = new Quaternion().setFromRotationMatrix(URDF_TO_GLTF);
const ONE = new Vector3(1, 1, 1);

export interface UrdfZipToGltfOptions {
  urdfPath?: string;
  rootName?: string;
}

export interface UrdfZipToGltfResult {
  gltf: Record<string, unknown>;
  gltfJson: string;
  kinematics: ExtRobotKinematics;
  model: KinematicModelDefinition;
}

export async function convertUrdfZipToGltf(
  zip: ZipInput,
  options: UrdfZipToGltfOptions = {}
): Promise<UrdfZipToGltfResult> {
  const zipData = zip instanceof Uint8Array ? zip : new Uint8Array(await toArrayBuffer(zip));
  const archive = await JSZip.loadAsync(zipData);
  const files = Object.values(archive.files).filter((file) => !file.dir);
  const fileMap = new Map<string, Uint8Array>();

  for (const file of files) {
    const data = await file.async('uint8array');
    fileMap.set(normalizePath(file.name), data);
  }

  const urdfEntry = resolveUrdfEntry(fileMap, options.urdfPath);
  const urdfText = new TextDecoder().decode(fileMap.get(urdfEntry));
  const baseDir = urdfEntry.includes('/') ? urdfEntry.slice(0, urdfEntry.lastIndexOf('/')) : '';

  const model = parseUrdf(urdfText);
  const resolver = new ZipAssetResolver(fileMap, baseDir);
  const manager = new LoadingManager();
  manager.setURLModifier((url) => resolver.resolveUrl(url));

  try {
    const scene = await buildScene(model, resolver, manager, options.rootName);
    const gltf = await exportScene(scene);

    const nodeIndexByName = new Map<string, number>();
    const nodes = Array.isArray(gltf.nodes) ? gltf.nodes : [];
    nodes.forEach((node: { name?: string }, index: number) => {
      if (node.name) nodeIndexByName.set(node.name, index);
    });

  const { kinematics, definition } = buildKinematicsDefinition(model, scene, nodeIndexByName, urdfEntry);

    gltf.extensionsUsed = uniqueExtensionList(gltf.extensionsUsed, 'EXT_robot_kinematics');
    gltf.extensions = gltf.extensions ?? {};
    gltf.extensions.EXT_robot_kinematics = kinematics;

    gltf.asset = gltf.asset ?? { version: '2.0' };
    gltf.asset.generator = 'three-gltf-robot URDF zip converter';

    const gltfJson = JSON.stringify(gltf);

    return {
      gltf,
      gltfJson,
      kinematics,
      model: definition
    };
  } finally {
    resolver.dispose();
  }
}

function resolveUrdfEntry(fileMap: Map<string, Uint8Array>, requested?: string): string {
  if (requested) {
    const normalized = normalizePath(requested);
    if (fileMap.has(normalized)) return normalized;
    const direct = [...fileMap.keys()].find((key) => key.endsWith(normalized));
    if (direct) return direct;
    throw new Error(`URDF file not found in zip: ${requested}`);
  }

  const candidates = [...fileMap.keys()].filter((name) => name.toLowerCase().endsWith('.urdf'));
  if (candidates.length === 1) return candidates[0];
  if (candidates.length === 0) throw new Error('No URDF file found in zip.');
  throw new Error(`Multiple URDF files found in zip. Provide urdfPath. Found: ${candidates.join(', ')}`);
}

async function toArrayBuffer(input: Blob | ArrayBuffer): Promise<ArrayBuffer> {
  if (input instanceof ArrayBuffer) return input;
  return await input.arrayBuffer();
}

function uniqueExtensionList(list: unknown, extension: string): string[] {
  const next = new Set<string>(Array.isArray(list) ? list : []);
  next.add(extension);
  return [...next];
}

function parseUrdf(xmlText: string): UrdfModel {
  const doc = new DOMParser().parseFromString(xmlText, 'application/xml');
  const parserError = doc.querySelector('parsererror');
  if (parserError) {
    throw new Error('Failed to parse URDF XML.');
  }

  const robot = doc.querySelector('robot');
  if (!robot) throw new Error('URDF is missing a <robot> root element.');

  const materials = new Map<string, UrdfMaterial>();
  Array.from(robot.children)
    .filter((child) => child.tagName.toLowerCase() === 'material')
    .forEach((material) => {
      const name = material.getAttribute('name');
      if (!name) return;
      const colorTag = material.querySelector('color');
      const color = colorTag ? parseColor(colorTag.getAttribute('rgba')) : undefined;
      const opacity = colorTag ? parseOpacity(colorTag.getAttribute('rgba')) : undefined;
      materials.set(name, { color, opacity });
    });

  const links: UrdfLink[] = [];
  robot.querySelectorAll('link').forEach((link) => {
    const name = link.getAttribute('name');
    if (!name) return;

    const visuals: UrdfVisual[] = [];
    link.querySelectorAll('visual').forEach((visual) => {
      const origin = parseOrigin(visual.querySelector('origin'));
      const geometryTag = visual.querySelector('geometry');
      if (!geometryTag) return;
      const geometry = parseGeometry(geometryTag);
      if (!geometry) return;

      const materialTag = visual.querySelector('material');
      const material = parseMaterial(materialTag, materials);

      visuals.push({ origin, geometry, material });
    });

    links.push({ name, visuals });
  });

  const joints: UrdfJoint[] = [];
  robot.querySelectorAll('joint').forEach((joint) => {
    const name = joint.getAttribute('name');
    const type = joint.getAttribute('type') as JointType | null;
    if (!name || !type) return;
    const parent = joint.querySelector('parent')?.getAttribute('link');
    const child = joint.querySelector('child')?.getAttribute('link');
    if (!parent || !child) return;

    const origin = parseOrigin(joint.querySelector('origin'));
    const axis = parseAxis(joint.querySelector('axis'));
    const limit = parseLimit(joint.querySelector('limit'));
    const mimic = parseMimic(joint.querySelector('mimic'));
    const dynamics = parseDynamics(joint.querySelector('dynamics'));
    const safety = parseSafety(joint.querySelector('safety_controller'));
    const calibration = parseCalibration(joint.querySelector('calibration'));

    joints.push({
      name,
      type,
      parent,
      child,
      origin,
      axis,
      limit,
      mimic,
      dynamics,
      safety,
      calibration
    });
  });

  return {
    name: robot.getAttribute('name') ?? 'robot',
    links,
    joints,
    materials
  };
}

function parseMaterial(tag: Element | null, materials: Map<string, UrdfMaterial>): UrdfMaterial | undefined {
  if (!tag) return undefined;
  const name = tag.getAttribute('name');
  const colorTag = tag.querySelector('color');
  const color = colorTag ? parseColor(colorTag.getAttribute('rgba')) : undefined;
  const opacity = colorTag ? parseOpacity(colorTag.getAttribute('rgba')) : undefined;

  if (color || opacity !== undefined) {
    return { color, opacity };
  }

  if (name && materials.has(name)) {
    return materials.get(name);
  }

  return undefined;
}

function parseOrigin(origin: Element | null): UrdfPose {
  if (!origin) {
    return { xyz: new Vector3(), rpy: new Vector3() };
  }
  const xyz = parseVector(origin.getAttribute('xyz'));
  const rpy = parseVector(origin.getAttribute('rpy'));
  return { xyz, rpy };
}

function parseAxis(axis: Element | null): Vector3 {
  if (!axis) return new Vector3(1, 0, 0);
  return parseVector(axis.getAttribute('xyz'), new Vector3(1, 0, 0));
}

function parseVector(text: string | null, fallback = new Vector3()): Vector3 {
  if (!text) return fallback.clone();
  const parts = text.trim().split(/\s+/).map(Number);
  if (parts.length < 3 || parts.some((value) => Number.isNaN(value))) return fallback.clone();
  return new Vector3(parts[0], parts[1], parts[2]);
}

function parseGeometry(geometry: Element): MeshGeometry | null {
  const box = geometry.querySelector('box');
  if (box) {
    const size = parseVector(box.getAttribute('size'));
    return { type: 'box', size };
  }

  const cylinder = geometry.querySelector('cylinder');
  if (cylinder) {
    const radius = parseNumber(cylinder.getAttribute('radius'), 0.5);
    const length = parseNumber(cylinder.getAttribute('length'), 1);
    return { type: 'cylinder', radius, length };
  }

  const sphere = geometry.querySelector('sphere');
  if (sphere) {
    const radius = parseNumber(sphere.getAttribute('radius'), 0.5);
    return { type: 'sphere', radius };
  }

  const mesh = geometry.querySelector('mesh');
  if (mesh) {
    const filename = mesh.getAttribute('filename');
    if (!filename) return null;
    const scale = parseVector(mesh.getAttribute('scale'), ONE);
    return { type: 'mesh', filename, scale };
  }

  return null;
}

function parseLimit(limit: Element | null): JointLimit | undefined {
  if (!limit) return undefined;
  const lower = parseNumber(limit.getAttribute('lower'));
  const upper = parseNumber(limit.getAttribute('upper'));
  const velocity = parseNumber(limit.getAttribute('velocity'));
  const effort = parseNumber(limit.getAttribute('effort'));
  return { lower, upper, velocity, effort };
}

function parseMimic(mimic: Element | null): UrdfMimic | undefined {
  if (!mimic) return undefined;
  const joint = mimic.getAttribute('joint');
  if (!joint) return undefined;
  return {
    joint,
    multiplier: parseNumber(mimic.getAttribute('multiplier')),
    offset: parseNumber(mimic.getAttribute('offset'))
  };
}

function parseDynamics(dynamics: Element | null): JointDynamics | undefined {
  if (!dynamics) return undefined;
  const damping = parseNumber(dynamics.getAttribute('damping'));
  const friction = parseNumber(dynamics.getAttribute('friction'));
  return { damping, friction };
}

function parseSafety(safety: Element | null): JointSafety | undefined {
  if (!safety) return undefined;
  return {
    softLowerLimit: parseNumber(safety.getAttribute('soft_lower_limit')),
    softUpperLimit: parseNumber(safety.getAttribute('soft_upper_limit')),
    kPosition: parseNumber(safety.getAttribute('k_position')),
    kVelocity: parseNumber(safety.getAttribute('k_velocity'))
  };
}

function parseCalibration(calibration: Element | null): JointCalibration | undefined {
  if (!calibration) return undefined;
  return {
    rising: parseNumber(calibration.getAttribute('rising')),
    falling: parseNumber(calibration.getAttribute('falling'))
  };
}

function parseNumber(value: string | null, fallback?: number): number | undefined {
  if (value === null || value === undefined || value === '') return fallback;
  const parsed = Number(value);
  if (Number.isNaN(parsed)) return fallback;
  return parsed;
}

function parseColor(rgba: string | null): Color | undefined {
  if (!rgba) return undefined;
  const parts = rgba.trim().split(/\s+/).map(Number);
  if (parts.length < 3 || parts.some((value) => Number.isNaN(value))) return undefined;
  return new Color(parts[0], parts[1], parts[2]);
}

function parseOpacity(rgba: string | null): number | undefined {
  if (!rgba) return undefined;
  const parts = rgba.trim().split(/\s+/).map(Number);
  if (parts.length < 4 || parts.some((value) => Number.isNaN(value))) return undefined;
  return parts[3];
}

async function buildScene(
  model: UrdfModel,
  resolver: ZipAssetResolver,
  manager: LoadingManager,
  rootName?: string
): Promise<Object3D> {
  const root = new Object3D();
  root.name = rootName ?? `${model.name}_root`;

  const linkNodes = new Map<string, Object3D>();
  const visualNodesByLink = new Map<string, Object3D[]>();

  for (const link of model.links) {
    const linkNode = new Object3D();
    linkNode.name = link.name;
    linkNodes.set(link.name, linkNode);

    const visualNodes: Object3D[] = [];
    for (let i = 0; i < link.visuals.length; i += 1) {
      const visual = link.visuals[i];
      const visualNode = await buildVisualNode(visual, resolver, manager, `${link.name}_visual_${i}`);
      visualNodes.push(visualNode);
      linkNode.add(visualNode);
    }
    visualNodesByLink.set(link.name, visualNodes);
  }

  const childLinks = new Set<string>();
  const jointsWithNodes: Array<UrdfJoint & { originNode: Object3D; dofNodes: Object3D[]; dofs: DOFDefinition[] }> = [];

  for (const joint of model.joints) {
    const parentLink = linkNodes.get(joint.parent);
    const childLink = linkNodes.get(joint.child);
    if (!parentLink || !childLink) continue;

    const originNode = new Object3D();
    originNode.name = `joint_${joint.name}_origin`;
    applyPose(originNode, joint.origin);

    parentLink.add(originNode);

    const { dofNodes, dofs } = createJointDofNodes(joint, originNode);

    if (dofNodes.length > 0) {
      dofNodes[dofNodes.length - 1].add(childLink);
    } else {
      originNode.add(childLink);
    }

    childLinks.add(joint.child);
    jointsWithNodes.push({ ...joint, originNode, dofNodes, dofs });
  }

  const rootLinks = model.links.filter((link) => !childLinks.has(link.name));
  if (rootLinks.length === 0 && model.links.length > 0) {
    rootLinks.push(model.links[0]);
  }
  rootLinks.forEach((link) => {
    const node = linkNodes.get(link.name);
    if (node) root.add(node);
  });

  root.userData.robotLinks = linkNodes;
  root.userData.robotVisuals = visualNodesByLink;
  root.userData.robotJoints = jointsWithNodes;

  return root;
}

function applyPose(target: Object3D, pose: UrdfPose): void {
  const translation = convertVector(pose.xyz);
  const rotation = convertQuaternion(pose.rpy);
  target.position.copy(translation);
  target.quaternion.copy(rotation);
}

function convertQuaternion(rpy: Vector3): Quaternion {
  const euler = new Euler(rpy.x, rpy.y, rpy.z, 'XYZ');
  const urdfQuat = new Quaternion().setFromEuler(euler);
  const rotation = new Matrix4().makeRotationFromQuaternion(urdfQuat);
  const converted = new Matrix4().multiplyMatrices(URDF_TO_GLTF, rotation).multiply(GLTF_TO_URDF);
  return new Quaternion().setFromRotationMatrix(converted);
}

function convertVector(vector: Vector3): Vector3 {
  return vector.clone().applyMatrix4(URDF_TO_GLTF);
}

function createJointDofNodes(joint: UrdfJoint, originNode: Object3D): { dofNodes: Object3D[]; dofs: DOFDefinition[] } {
  const dofNodes: Object3D[] = [];
  const dofs: DOFDefinition[] = [];

  const axis = convertVector(joint.axis);
  if (axis.lengthSq() > 0) axis.normalize();
  const baseLimit = joint.type === 'continuous' ? undefined : joint.limit;

  const appendDof = (
    motion: DOFDefinition['motion'],
    dofAxis: Vector3,
    suffix: string,
    limit: JointLimit | undefined = baseLimit
  ) => {
    const node = new Object3D();
    node.name = `joint_${joint.name}_${suffix}`;
    const parent = dofNodes.length === 0 ? originNode : dofNodes[dofNodes.length - 1];
    parent.add(node);
    dofNodes.push(node);
    dofs.push({
      motion,
      axis: [dofAxis.x, dofAxis.y, dofAxis.z],
      limit
    });
  };

  switch (joint.type) {
    case 'fixed':
      break;
    case 'revolute':
    case 'continuous':
      appendDof('rotation', axis, 'dof0');
      break;
    case 'prismatic':
      appendDof('translation', axis, 'dof0');
      break;
    case 'planar': {
      const normal = axis.lengthSq() > 0 ? axis.clone().normalize() : new Vector3(0, 0, 1);
      const tangent = Math.abs(normal.dot(new Vector3(1, 0, 0))) > 0.9 ? new Vector3(0, 1, 0) : new Vector3(1, 0, 0);
      const u = new Vector3().crossVectors(normal, tangent).normalize();
      const v = new Vector3().crossVectors(normal, u).normalize();
      appendDof('translation', u, 'dof0');
      appendDof('translation', v, 'dof1');
      break;
    }
    case 'floating': {
      appendDof('translation', new Vector3(1, 0, 0), 'dof0');
      appendDof('translation', new Vector3(0, 1, 0), 'dof1');
      appendDof('translation', new Vector3(0, 0, 1), 'dof2');
      appendDof('rotation', new Vector3(1, 0, 0), 'dof3');
      appendDof('rotation', new Vector3(0, 1, 0), 'dof4');
      appendDof('rotation', new Vector3(0, 0, 1), 'dof5');
      break;
    }
    default:
      break;
  }

  return { dofNodes, dofs };
}

async function buildVisualNode(
  visual: UrdfVisual,
  resolver: ZipAssetResolver,
  manager: LoadingManager,
  name: string
): Promise<Object3D> {
  const node = new Object3D();
  node.name = name;

  applyPose(node, visual.origin);

  const geometryRoot = new Object3D();
  geometryRoot.quaternion.copy(URDF_TO_GLTF_QUAT);
  geometryRoot.scale.copy(visual.geometry.type === 'mesh' ? visual.geometry.scale : ONE);

  const geometry = await buildGeometryObject(visual.geometry, resolver, manager, visual.material);
  geometryRoot.add(geometry);
  node.add(geometryRoot);

  return node;
}

async function buildGeometryObject(
  geometry: MeshGeometry,
  resolver: ZipAssetResolver,
  manager: LoadingManager,
  material?: UrdfMaterial
): Promise<Object3D> {
  switch (geometry.type) {
    case 'box':
      return buildPrimitiveMesh(new BoxGeometry(geometry.size.x, geometry.size.y, geometry.size.z), material);
    case 'cylinder':
      return buildPrimitiveMesh(new CylinderGeometry(geometry.radius, geometry.radius, geometry.length), material);
    case 'sphere':
      return buildPrimitiveMesh(new SphereGeometry(geometry.radius), material);
    case 'mesh':
      return await loadMeshObject(geometry.filename, resolver, manager, material);
    default:
      return new Object3D();
  }
}

function buildPrimitiveMesh(geometry: BoxGeometry | CylinderGeometry | SphereGeometry, material?: UrdfMaterial): Mesh {
  const mat = createMaterial(material);
  const mesh = new Mesh(geometry, mat);
  return mesh;
}

function createMaterial(material?: UrdfMaterial): MeshStandardMaterial {
  if (!material?.color) {
    return new MeshStandardMaterial({ color: '#9aa7b5', metalness: 0.15, roughness: 0.6 });
  }
  return new MeshStandardMaterial({
    color: material.color,
    metalness: 0.2,
    roughness: 0.5,
    transparent: material.opacity !== undefined && material.opacity < 1,
    opacity: material.opacity ?? 1
  });
}

async function loadMeshObject(
  filename: string,
  resolver: ZipAssetResolver,
  manager: LoadingManager,
  material?: UrdfMaterial
): Promise<Object3D> {
  const resolved = resolver.resolvePath(filename);
  if (!resolved) {
    throw new Error(`Mesh asset not found in zip: ${filename}`);
  }

  const url = resolver.getBlobUrl(resolved);
  const extension = resolved.split('.').pop()?.toLowerCase();

  switch (extension) {
    case 'glb':
    case 'gltf':
      return await new Promise<Object3D>((resolve, reject) => {
        const loader = new GLTFLoader(manager);
        loader.load(
          url,
          (gltf) => resolve(gltf.scene),
          undefined,
          (error) => reject(error)
        );
      });
    case 'dae':
      return await new Promise<Object3D>((resolve, reject) => {
        const loader = new ColladaLoader(manager);
        loader.load(
          url,
          (result) => resolve(result.scene),
          undefined,
          (error) => reject(error)
        );
      });
    case 'obj':
      return await new Promise<Object3D>((resolve, reject) => {
        const loader = new OBJLoader(manager);
        loader.load(
          url,
          (object) => {
            if (material) applyMaterial(object, createMaterial(material));
            resolve(object);
          },
          undefined,
          (error) => reject(error)
        );
      });
    case 'stl':
      return await new Promise<Object3D>((resolve, reject) => {
        const loader = new STLLoader(manager);
        loader.load(
          url,
          (geom) => {
            const mesh = new Mesh(geom, createMaterial(material));
            resolve(mesh);
          },
          undefined,
          (error) => reject(error)
        );
      });
    default:
      throw new Error(`Unsupported mesh format: ${extension ?? 'unknown'}`);
  }
}

function applyMaterial(object: Object3D, material: MeshStandardMaterial): void {
  object.traverse((child) => {
    if (child instanceof Mesh) {
      child.material = material;
    }
  });
}

async function exportScene(scene: Object3D): Promise<Record<string, unknown>> {
  const exporter = new GLTFExporter();
  return await new Promise((resolve, reject) => {
    exporter.parse(
      scene,
      (result) => {
        if (result instanceof ArrayBuffer) {
          reject(new Error('Binary export is not supported in this helper.'));
          return;
        }
        resolve(result as Record<string, unknown>);
      },
      (error) => reject(error),
      {
        binary: false,
        embedImages: true
      }
    );
  });
}

function buildKinematicsDefinition(
  model: UrdfModel,
  root: Object3D,
  nodeIndexByName: Map<string, number>,
  urdfPath: string
): { kinematics: ExtRobotKinematics; definition: KinematicModelDefinition } {
  const linkIndexByName = new Map<string, number>();
  model.links.forEach((link, index) => linkIndexByName.set(link.name, index));

  const links: LinkDefinition[] = model.links.map((link) => {
    const nodeIndex = nodeIndexByName.get(link.name);
    if (nodeIndex === undefined) {
      throw new Error(`Link node missing from glTF export: ${link.name}`);
    }

    const visuals = (root.userData.robotVisuals?.get(link.name) ?? []) as Object3D[];
    const visualNodes = visuals
      .map((visual) => nodeIndexByName.get(visual.name))
      .filter((value): value is number => value !== undefined);

    return {
      name: link.name,
      node: nodeIndex,
      visualNodes: visualNodes.length ? visualNodes : undefined
    };
  });

  const jointsWithNodes = (root.userData.robotJoints ?? []) as Array<UrdfJoint & {
    originNode: Object3D;
    dofNodes: Object3D[];
    dofs: DOFDefinition[];
  }>;

  const joints: JointDefinition[] = jointsWithNodes.map((joint) => {
    const parentLink = linkIndexByName.get(joint.parent) ?? 0;
    const childLink = linkIndexByName.get(joint.child) ?? 0;
    const originNode = nodeIndexByName.get(joint.originNode.name);
    if (originNode === undefined) {
      throw new Error(`Origin node missing from glTF export: ${joint.originNode.name}`);
    }
    const dofNodes = joint.dofNodes.map((node) => {
      const index = nodeIndexByName.get(node.name);
      if (index === undefined) {
        throw new Error(`DOF node missing from glTF export: ${node.name}`);
      }
      return index;
    });

    return {
      name: joint.name,
      type: joint.type,
      parentLink,
      childLink,
      originNode,
      dofNodes,
      dofs: joint.dofs,
      mimic: resolveMimic(joint, model),
      dynamics: joint.dynamics,
      safety: joint.safety,
      calibration: joint.calibration
    };
  });

  const rootNodeIndex = nodeIndexByName.get(root.name) ?? 0;
  const definition: KinematicModelDefinition = {
    name: model.name,
    rootNode: rootNodeIndex,
    links,
    joints,
    source: {
      format: 'urdf',
      uri: urdfPath || `${model.name}.urdf`,
      resolvedAt: new Date().toISOString()
    }
  };

  const kinematics: ExtRobotKinematics = { models: [definition] };

  return { kinematics, definition };
}

function resolveMimic(
  joint: UrdfJoint,
  model: UrdfModel
): JointMimic | undefined {
  if (!joint.mimic) return undefined;
  const mimicName = joint.mimic.joint;
  const jointIndex = model.joints.findIndex((candidate) => candidate.name === mimicName);
  if (jointIndex < 0) return undefined;
  return {
    joint: jointIndex,
    multiplier: joint.mimic.multiplier,
    offset: joint.mimic.offset
  };
}

function normalizePath(path: string): string {
  return path.replace(/\\/g, '/').replace(/^\.\//, '').replace(/\/+/g, '/');
}

class ZipAssetResolver {
  private readonly files: Map<string, Uint8Array>;
  private readonly baseDir: string;
  private readonly blobUrls = new Map<string, string>();

  constructor(files: Map<string, Uint8Array>, baseDir: string) {
    this.files = files;
    this.baseDir = normalizePath(baseDir);
  }

  resolveUrl(url: string): string {
    const resolved = this.resolvePath(url);
    if (!resolved) return url;
    return this.getBlobUrl(resolved);
  }

  resolvePath(path: string): string | null {
    if (!path) return null;
    const normalized = normalizePath(stripUrl(path));
    const noProtocol = stripProtocol(normalized);
    const candidates = [
      noProtocol,
      normalizePath(this.baseDir ? `${this.baseDir}/${noProtocol}` : noProtocol)
    ];

    for (const candidate of candidates) {
      if (this.files.has(candidate)) return candidate;
    }

    const basename = noProtocol.split('/').pop();
    if (!basename) return null;
    const matches = [...this.files.keys()].filter((name) => name.endsWith(`/${basename}`) || name === basename);
    if (matches.length === 1) return matches[0];

    return null;
  }

  getBlobUrl(path: string): string {
    const cached = this.blobUrls.get(path);
    if (cached) return cached;
    const data = this.files.get(path);
    if (!data) throw new Error(`Missing zip entry: ${path}`);
    const blob = new Blob([data]);
    const url = URL.createObjectURL(blob);
    this.blobUrls.set(path, url);
    return url;
  }

  dispose(): void {
    for (const url of this.blobUrls.values()) {
      URL.revokeObjectURL(url);
    }
    this.blobUrls.clear();
  }
}

function stripUrl(url: string): string {
  return url.split(/[?#]/)[0];
}

function stripProtocol(url: string): string {
  if (url.startsWith('package://')) return url.slice('package://'.length);
  if (url.startsWith('file://')) return url.slice('file://'.length);
  return url;
}
