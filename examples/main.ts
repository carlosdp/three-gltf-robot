import GUI from 'lil-gui';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { convertUrdfZipToGltf, getRobotKinematics, registerRobotKinematics } from '@lib';

const canvas = document.querySelector('#c') as HTMLCanvasElement;
const statusEl = document.querySelector('#status') as HTMLDivElement;
const fileInput = document.querySelector('#zip-input') as HTMLInputElement;
const downloadButton = document.querySelector('#download-btn') as HTMLButtonElement;

const renderer = new THREE.WebGLRenderer({
  canvas,
  antialias: true,
  powerPreference: 'high-performance'
});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color('#0a1018');

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.01, 100);
camera.position.set(1.6, 1.2, 2.4);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.5, 0);
controls.enableDamping = true;

const keyLight = new THREE.DirectionalLight('#ffffff', 1.2);
keyLight.position.set(2, 4, 2);
scene.add(keyLight);

const fillLight = new THREE.DirectionalLight('#88aaff', 0.6);
fillLight.position.set(-2, 2, -1);
scene.add(fillLight);

scene.add(new THREE.AmbientLight('#334455', 0.5));
scene.add(new THREE.GridHelper(4, 20, '#223045', '#162030'));

const loader = new GLTFLoader();
registerRobotKinematics(loader);

let gui: GUI | null = null;
let activeRobot: THREE.Object3D | null = null;
let latestGlb: ArrayBuffer | null = null;
let latestGlbName = 'robot.glb';

const ROBOT_GLTF = {
  asset: {
    version: '2.0',
    generator: 'three-gltf-robot example'
  },
  extensionsUsed: ['EXT_robot_kinematics'],
  scenes: [{ nodes: [0] }],
  nodes: [
    { name: 'robot_root', children: [1] },
    { name: 'base_link', children: [2] },
    {
      name: 'joint_shoulder_origin',
      translation: [0, 0.2, 0],
      children: [3]
    },
    { name: 'joint_shoulder_dof0', children: [4] },
    { name: 'upper_link', children: [5] },
    {
      name: 'joint_elbow_origin',
      translation: [0, 0.0, 0.45],
      children: [6]
    },
    { name: 'joint_elbow_dof0', children: [7] },
    { name: 'forearm_link' }
  ],
  extensions: {
    EXT_robot_kinematics: {
      models: [
        {
          name: 'demo_robot',
          rootNode: 0,
          links: [
            { name: 'base_link', node: 1 },
            { name: 'upper_link', node: 4 },
            { name: 'forearm_link', node: 7 }
          ],
          joints: [
            {
              name: 'shoulder_pan',
              type: 'revolute',
              parentLink: 0,
              childLink: 1,
              originNode: 2,
              dofNodes: [3],
              dofs: [
                {
                  motion: 'rotation',
                  axis: [0, 1, 0],
                  limit: { lower: -1.57, upper: 1.57 },
                  default: 0
                }
              ]
            },
            {
              name: 'elbow_flex',
              type: 'revolute',
              parentLink: 1,
              childLink: 2,
              originNode: 5,
              dofNodes: [6],
              dofs: [
                {
                  motion: 'rotation',
                  axis: [1, 0, 0],
                  limit: { lower: -1.2, upper: 1.2 },
                  default: 0
                }
              ]
            }
          ],
          configurations: {
            home: { jointPositions: [0, 0] },
            wave: { jointPositions: [0.8, -0.8] }
          },
          source: {
            format: 'urdf',
            uri: 'package://demo/robot.urdf',
            resolvedAt: '2026-01-03T00:00:00Z'
          }
        }
      ]
    }
  }
};

function setStatus(text: string) {
  statusEl.textContent = text;
}

function setDownloadState(glb: ArrayBuffer | null, name?: string) {
  latestGlb = glb;
  downloadButton.disabled = !glb;
  if (name) latestGlbName = name;
}

async function loadGltfJson(gltfJson: string, options: { addDemoMeshes?: boolean } = {}) {
  return await new Promise<void>((resolve, reject) => {
    loader.parse(
      gltfJson,
      '',
      (gltf) => {
        if (activeRobot) scene.remove(activeRobot);
        activeRobot = gltf.scene;
        scene.add(gltf.scene);

        const models = getRobotKinematics(gltf);
        const model = models[0];
        if (!model) {
          setStatus('Loaded glTF without kinematics.');
          resolve();
          return;
        }

        if (options.addDemoMeshes) {
          addRobotMeshes(model);
        }
        setupGui(model);
        resolve();
      },
      (error) => reject(error)
    );
  });
}

async function loadDemoRobot() {
  setStatus('Loading demo robot...');
  try {
    await loadGltfJson(JSON.stringify(ROBOT_GLTF), { addDemoMeshes: true });
    setDownloadState(null);
    setStatus('Demo robot loaded. Upload a URDF zip to replace it.');
  } catch (error) {
    setStatus('Failed to load demo robot.');
    // eslint-disable-next-line no-console
    console.error(error);
  }
}

fileInput.addEventListener('change', async () => {
  const file = fileInput.files?.[0];
  if (!file) return;
  setStatus('Converting URDF zip...');
  setDownloadState(null);

  try {
    const result = await convertUrdfZipToGltf(file);
    await loadGltfJson(result.gltfJson);
    const glb = buildGlbFromGltfJson(result.gltfJson);
    const filename = file.name.replace(/\.zip$/i, '') || 'robot';
    setDownloadState(glb, `${filename}.glb`);
    setStatus('Converted and loaded robot.');
  } catch (error) {
    setStatus('Conversion failed. Check console.');
    // eslint-disable-next-line no-console
    console.error(error);
  }
});

downloadButton.addEventListener('click', () => {
  if (!latestGlb) return;
  const blob = new Blob([latestGlb], { type: 'model/gltf-binary' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = latestGlbName;
  link.click();
  URL.revokeObjectURL(url);
});

loadDemoRobot();

function addRobotMeshes(model: ReturnType<typeof getRobotKinematics>[number]) {
  const base = model.getLink('base_link');
  const upper = model.getLink('upper_link');
  const forearm = model.getLink('forearm_link');

  const baseMat = new THREE.MeshStandardMaterial({ color: '#4cc9f0', metalness: 0.2, roughness: 0.4 });
  const armMat = new THREE.MeshStandardMaterial({ color: '#f72585', metalness: 0.2, roughness: 0.35 });
  const foreMat = new THREE.MeshStandardMaterial({ color: '#fca311', metalness: 0.2, roughness: 0.4 });

  if (base) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.1, 0.5), baseMat);
    mesh.position.y = 0.05;
    base.node.add(mesh);
  }

  if (upper) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.15, 0.5), armMat);
    mesh.position.z = 0.25;
    upper.node.add(mesh);
  }

  if (forearm) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 0.45), foreMat);
    mesh.position.z = 0.22;
    forearm.node.add(mesh);
  }
}

function setupGui(model: ReturnType<typeof getRobotKinematics>[number]) {
  if (gui) gui.destroy();
  gui = new GUI({ width: 280 });

  const configs = Object.keys(model.configurations ?? {});
  if (configs.length > 0) {
    const configFolder = gui.addFolder('Configurations');
    for (const name of configs) {
      configFolder.add(
        {
          [name]: () => model.applyConfiguration(name)
        },
        name
      );
    }
  }

  for (const joint of model.joints) {
    const folder = gui.addFolder(joint.name);
    joint.dofs.forEach((dof, index) => {
      const limit = dof.limit;
      const min = limit?.lower ?? (dof.motion === 'rotation' ? -Math.PI : -0.5);
      const max = limit?.upper ?? (dof.motion === 'rotation' ? Math.PI : 0.5);
      const state = { value: joint.values[index] ?? 0 };
      folder
        .add(state, 'value', min, max, 0.01)
        .name(dof.motion === 'rotation' ? `rot ${index}` : `trans ${index}`)
        .onChange((next: number) => {
          model.setJointDofValue(joint.name, index, next);
        });
    });
  }
}

function onResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

function buildGlbFromGltfJson(gltfJson: string): ArrayBuffer {
  const gltf = JSON.parse(gltfJson) as {
    asset?: { version?: string };
    buffers?: Array<{ uri?: string; byteLength?: number }>;
  };

  let binData = new Uint8Array(0);
  if (gltf.buffers?.length) {
    const uri = gltf.buffers[0].uri;
    if (uri && uri.startsWith('data:')) {
      const base64Index = uri.indexOf('base64,');
      if (base64Index >= 0) {
        const base64 = uri.slice(base64Index + 'base64,'.length);
        const binary = atob(base64);
        binData = new Uint8Array(binary.length);
        for (let i = 0; i < binary.length; i += 1) {
          binData[i] = binary.charCodeAt(i);
        }
        delete gltf.buffers[0].uri;
        gltf.buffers[0].byteLength = binData.byteLength;
      }
    }
  }

  const jsonString = JSON.stringify(gltf);
  const jsonBytes = new TextEncoder().encode(jsonString);
  const jsonPadded = padTo4(jsonBytes, 0x20);
  const binPadded = padTo4(binData, 0x00);

  const hasBin = binPadded.length > 0;
  const totalLength = 12 + 8 + jsonPadded.length + (hasBin ? 8 + binPadded.length : 0);
  const glbBuffer = new ArrayBuffer(totalLength);
  const view = new DataView(glbBuffer);

  let offset = 0;
  view.setUint32(offset, 0x46546c67, true); // glTF
  offset += 4;
  view.setUint32(offset, 2, true);
  offset += 4;
  view.setUint32(offset, totalLength, true);
  offset += 4;

  view.setUint32(offset, jsonPadded.length, true);
  offset += 4;
  view.setUint32(offset, 0x4e4f534a, true); // JSON
  offset += 4;
  new Uint8Array(glbBuffer, offset, jsonPadded.length).set(jsonPadded);
  offset += jsonPadded.length;

  if (hasBin) {
    view.setUint32(offset, binPadded.length, true);
    offset += 4;
    view.setUint32(offset, 0x004e4942, true); // BIN
    offset += 4;
    new Uint8Array(glbBuffer, offset, binPadded.length).set(binPadded);
  }

  return glbBuffer;
}

function padTo4(data: Uint8Array, padValue: number): Uint8Array {
  const paddedLength = Math.ceil(data.length / 4) * 4;
  if (paddedLength === data.length) return data;
  const padded = new Uint8Array(paddedLength);
  padded.set(data);
  padded.fill(padValue, data.length);
  return padded;
}

window.addEventListener('resize', onResize);

renderer.setAnimationLoop(() => {
  controls.update();
  renderer.render(scene, camera);
});
