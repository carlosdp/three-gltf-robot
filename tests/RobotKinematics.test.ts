import { Object3D, Quaternion, Vector3 } from 'three';
import { createRobotKinematicsModels } from '../src/runtime/RobotKinematics.js';
import type { ExtRobotKinematics } from '../src/runtime/types.js';

const EPS = 1e-5;

function expectQuatClose(actual: Quaternion, expected: Quaternion) {
  expect(Math.abs(actual.x - expected.x)).toBeLessThan(EPS);
  expect(Math.abs(actual.y - expected.y)).toBeLessThan(EPS);
  expect(Math.abs(actual.z - expected.z)).toBeLessThan(EPS);
  expect(Math.abs(actual.w - expected.w)).toBeLessThan(EPS);
}

test('applies revolute joint rotation around axis', async () => {
  const nodes = [
    new Object3D(),
    new Object3D(),
    new Object3D(),
    new Object3D(),
    new Object3D()
  ];

  const extension: ExtRobotKinematics = {
    models: [
      {
        name: 'test_robot',
        rootNode: 0,
        links: [
          { name: 'base', node: 1 },
          { name: 'arm', node: 4 }
        ],
        joints: [
          {
            name: 'joint0',
            type: 'revolute',
            parentLink: 0,
            childLink: 1,
            originNode: 2,
            dofNodes: [3],
            dofs: [
              {
                motion: 'rotation',
                axis: [0, 1, 0],
                limit: { lower: -Math.PI, upper: Math.PI },
                default: 0
              }
            ]
          }
        ]
      }
    ]
  };

  const models = await createRobotKinematicsModels(extension, async (index) => nodes[index]);
  const model = models[0];
  const joint = model.joints[0];

  model.setJointValue('joint0', Math.PI / 2);

  const expected = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2);
  expectQuatClose(joint.dofNodes[0].quaternion, expected);
});
