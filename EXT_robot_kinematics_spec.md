## 1) Goals, constraints, and why this needs an extension

### Goals

1. **Represent the URDF kinematic tree** (links + joints) in a glTF asset.
2. **Preserve joint semantics**: joint type, axis, limits, dynamics parameters, mimic relationships.
3. **Make joint motion “actionable”** in a viewer/engine:

   * A runtime can set joint positions and update node transforms correctly.
   * Animations can target the correct nodes using core glTF animation paths.
4. **Keep the model viewable without the extension**: if the extension is ignored, the robot still renders in its default pose.

### Constraints from glTF core

* glTF uses a right‑handed coordinate system; **+Y up, +Z forward, -X right**.
* Units: **meters** for linear distances and **radians** for angles.
* Node hierarchies must be strict trees (no cycles).
* If a node is animated, it **must use TRS** (not `matrix`).
* TRS compose as `T * R * S`.

### URDF semantics we must preserve

From common URDF tooling documentation (urdfpy):

* Joint `axis` is specified **in the joint frame**.
* Joint `origin` is the pose of the **child link (and joint frame at zero)** relative to the parent link frame.
* Joint configuration meaning by type:

  * prismatic: translation along axis (meters)
  * revolute/continuous: rotation about axis (radians)
  * planar: x/y translation in the plane
  * floating: xyz + rpy (or a 4×4)
* Link contains visuals/collisions/inertial concepts.

### Existing related work

There is at least one vendor extension used for articulated parts (AGI_articulations) that describes articulations as stages/DOFs with min/max/initial. ([GitHub][1])
There has also been explicit community interest in a URDF-like robotics extension for glTF.
This proposal is closer to URDF’s kinematic tree semantics than “generic articulation sliders”.

---

## 2) Core design choice: encode joints as dedicated transform nodes

The single most important practical decision:

**For each URDF joint, the exported glTF MUST separate the joint’s fixed origin transform from its variable motion transform.**

Why:

* In URDF, the joint `origin` is fixed, and the joint position `q` applies an additional transform about/along an axis.
* In glTF, animating a node’s `rotation`/`translation` overwrites that channel; you don’t want to “overwrite” the baked-in origin.

### Canonical node pattern per joint (recommended, and used by the converter)

For a 1‑DOF joint:

```
parent_link_node
  └── joint_<name>_origin_node   (fixed: URDF origin)
        └── joint_<name>_dof0_node (variable: q)
              └── child_link_node  (identity local transform)
```

For multi‑DOF joints (planar/floating), chain multiple `dof` nodes:

```
... joint_origin
      └── dof0
           └── dof1
                └── dof2
                     └── child_link
```

This makes joint driving dead-simple:

* Revolute: set `dofNode.rotation = quat(axis, q)`
* Prismatic: set `dofNode.translation = axis * q`

…and keeps the robot renderable at default pose even if the extension is ignored (default `q=0` → dof nodes identity).

---

## 3) Extension data model

### Top-level placement

`EXT_robot_kinematics` lives at the **asset root**:

```jsonc
{
  "extensionsUsed": ["EXT_robot_kinematics"],
  "extensions": {
    "EXT_robot_kinematics": {
      "models": [ /* one or more kinematic models */ ]
    }
  }
}
```

Why root-level:

* URDF semantics are “global” to a robot model, not a single node/mesh.
* Allows multiple robots in one glTF file, if desired.

### Object: `KinematicModel`

A robot/assembly kinematic definition.

Proposed fields:

```jsonc
{
  "name": "my_robot",
  "rootNode": 0,                 // glTF node index acting as “robot root”
  "links": [ /* Link[] */ ],
  "joints": [ /* Joint[] */ ],
  "configurations": { /* optional named joint configs */ },
  "source": { /* optional provenance */ }
}
```

* `rootNode` is the node under which the kinematic tree’s root link lives (or a wrapper node used for coordinate conversion).

### Object: `Link`

Maps URDF link concepts to a glTF node representing the link frame.

```jsonc
{
  "name": "base_link",
  "node": 12,                    // glTF node index for the link frame
  "visualNodes": [34, 35],       // OPTIONAL: convenience pointers
  "collisionNodes": [40],        // OPTIONAL: convenience pointers
  "extras": { /* extension-defined or app-specific */ }
}
```

Notes:

* `node` is the **link frame node**, intended to have identity local transform relative to its parent DOF node.
* Visual and collision nodes are *optional indexing aids*; the scene graph already encodes them.

### Object: `Joint`

Encodes URDF joint semantics + binds them to the node chain that implements motion.

```jsonc
{
  "name": "shoulder_pan_joint",
  "type": "revolute",                 // fixed|revolute|continuous|prismatic|planar|floating
  "parentLink": 0,                    // index into links[]
  "childLink": 1,                     // index into links[]
  "originNode": 20,                   // glTF node: fixed joint origin transform
  "dofNodes": [21],                   // 0..N nodes: variable motion nodes, in application order
  "dofs": [ /* DOF[] (same length as dofNodes) */ ],
  "mimic": { /* optional */ },
  "dynamics": { /* optional */ },
  "safety": { /* optional */ },
  "calibration": { /* optional */ }
}
```

### Object: `DOF`

One scalar degree of freedom, mapped to a single node channel.

```jsonc
{
  "motion": "rotation",           // "rotation" or "translation"
  "axis": [0, 0, 1],              // vec3, in the LOCAL frame of the DOF node’s parent (joint frame)
  "limit": {
    "lower": -1.57,               // rad or m depending on motion
    "upper":  1.57,
    "velocity": 2.0,              // rad/s or m/s
    "effort": 50.0                // Nm or N
  },
  "default": 0.0                  // optional joint position default for this DOF
}
```

This lines up directly with URDF semantics:

* axis is in joint frame
* units match glTF: meters/radians

### Optional: `mimic`

URDF mimic joint relationship (for 1‑DOF joints).

```jsonc
{
  "joint": 3,           // index into joints[]
  "multiplier": 1.0,
  "offset": 0.0
}
```

### Optional: `dynamics`

URDF joint dynamics (damping/friction).

```jsonc
{
  "damping": 0.1,
  "friction": 0.2
}
```

### Optional: `configurations`

Named joint pose presets for UI / startup.

```jsonc
{
  "home": { "jointPositions": [0, 0, 0, 0, 0, 0] },
  "stowed": { "jointPositions": [0, -1.2, 1.8, 0, 0.5, 0] }
}
```

Where `jointPositions` is:

* aligned with `joints[]` (each entry may be:

  * a number for 1‑DOF joints
  * an array of numbers for multi‑DOF joints), **or**
* flattened by DOF count (you must specify the convention in the schema).
  I recommend **per-joint**, because it stays URDF-like.

### Optional: `source`

Round-trip / provenance info.

```jsonc
{
  "format": "urdf",
  "uri": "package://my_robot/urdf/robot.urdf",
  "resolvedAt": "2026-01-03T00:00:00Z"
}
```

---

## 4) Normative rules (validation requirements)

These are the “MUST/SHOULD” statements you’d put in the spec and in a JSON schema + validator checks.

### Structural

1. `models[i].links[*].node` MUST reference an existing glTF node index.
2. `models[i].joints[*].parentLink` and `childLink` MUST be valid indices into `links[]`.
3. `originNode` and all `dofNodes[]` MUST reference existing nodes.
4. For each joint:

   * `dofNodes.length === dofs.length`.
   * `dofNodes.length === 0` is allowed only for `fixed` joints.
5. Node hierarchy must remain a strict tree (already required by glTF).

### Kinematic consistency (recommended to validate)

For each joint:

* The `originNode` SHOULD be a direct child of the parent link node.
* The `dofNodes` SHOULD form a direct chain under `originNode`, ending at the child link node.
* All `dofNodes` SHOULD have identity TRS at rest pose (or at least identity on the controlled channel).

This “SHOULD” set enables predictable interactive control.

### Axis and limits

* Each DOF `axis` MUST be non-zero; SHOULD be normalized.
* If `limit.lower` and `limit.upper` exist, MUST satisfy `lower <= upper`.
* For `continuous` joints, `lower/upper` SHOULD be omitted (or explicitly null).

### Animation compatibility

If you generate glTF animations for joints:

* Any animated DOF node MUST use TRS, not `matrix`.

---

## 5) Runtime semantics: how a loader uses the extension

A loader/engine that supports the extension should expose:

* a list of joints (names, types, limits)
* a way to set/get joint positions
* optional mimic evaluation

### Pose evaluation algorithm (tree forward-kinematics)

Given desired joint DOF values `q`:

For each joint in parent-to-child order:

1. Leave `originNode` transform as-authored.
2. For each DOF `k`:

   * if `motion == rotation`:

     * set `dofNode[k].rotation = quat(axis[k], qk)`
   * if `motion == translation`:

     * set `dofNode[k].translation = axis[k] * qk`
3. Proceed down the tree; the glTF scene graph already propagates transforms.

Because glTF composes TRS as `T*R*S`, and we’re setting only the DOF node’s TRS on the intended channel, this cleanly matches robotics conventions.

### Mimic handling

When updating joints:

* If joint `J` has `mimic`, compute `qJ = multiplier * qRef + offset`, then apply to its DOF.

---

## 6) URDF → glTF conversion plan (step-by-step)

This is the exporter pipeline you’d implement.

### Step 0 — Preprocess URDF inputs

1. **Resolve Xacro** (if applicable) into plain URDF.
2. Resolve `package://` URIs to file paths.
3. Load mesh assets (STL/DAE/OBJ/etc.) referenced by URDF visuals/collisions.

### Step 1 — Parse URDF into a kinematic graph

Using a URDF parser (urdfdom, urdfpy, etc.), build:

* `linksByName`
* `jointsByName`
* directed edges parentLink → childLink (from joints)
* determine root link(s): links that are never a child

URDF is expected to be a tree; glTF also requires an acyclic node hierarchy.

### Step 2 — Decide coordinate conversion policy

glTF coordinate system is fixed: **+Y up, +Z forward, -X right**.

Most URDF/ROS models follow REP-103 (x forward, y left, z up). A practical converter should do one of:

**Option A (recommended): convert everything into glTF coordinates**

* Convert every URDF pose (xyz+rpy) and axis vector into glTF axes.
* Keep glTF `rootNode` identity.

**Option B: insert a single root “basis change” node**

* Keep URDF math in URDF coordinates, but put a fixed rotation/permutation on the top node.
* Still requires careful axis handling; usually ends up more confusing.

I recommend **Option A**.

### Step 3 — Build glTF nodes for links

For each URDF link:

1. Create a glTF node `linkNode`:

   * `name = <link name>`
   * local transform **identity** (TRS = default)
2. Store mapping `linkName → linkNodeIndex`
3. Do **not** attach meshes directly to the link node if you want clean separation; instead attach visual nodes beneath it.

This corresponds to the link concept (rigid body with visuals/collisions).

### Step 4 — Build glTF nodes for visuals (and optionally collisions)

For each link’s visual elements:

1. URDF visual has an `origin` pose relative to link frame.
2. Create a glTF node `visualNode` under the link node:

   * `translation/rotation` = converted visual origin
   * `mesh` = converted geometry
3. Append its node index into `Link.visualNodes` (optional but useful)

For collisions:

* URDF collision origin is also relative to link frame.
* You can either:

  * export collision meshes as hidden nodes (app-specific), or
  * store them for physics engines via a physics extension (future work), or
  * just keep them as `collisionNodes` metadata.

### Step 5 — Build glTF node chain per joint

For each URDF joint (parent → child):

* URDF joint origin is the pose of the child/joint frame relative to parent.
* URDF joint axis is specified in the joint frame.

Create:

1. `jointOriginNode` (fixed)

   * parent: parentLinkNode
   * TRS: converted from URDF joint origin (xyz+rpy → translation+quaternion)
2. Create N DOF nodes depending on joint type:

#### fixed

* No DOF nodes.
* Parent child link under `jointOriginNode`.

#### revolute / continuous

* Create one DOF node:

  * `joint_<name>_dof0`
  * identity transform at rest
* Child link node under DOF node

#### prismatic

* Same as revolute/continuous, but DOF is translation rather than rotation.

#### planar

URDF planar: “moves in the plane orthogonal to axis”
Implementation:

* Build **two translation DOF nodes** in sequence:

  * DOF0 translates along basis `u` (in plane)
  * DOF1 translates along basis `v` (in plane)
* Compute orthonormal basis:

  * `n = normalize(axis)` (plane normal)
  * choose a stable `t` not parallel to `n` (e.g., [1,0,0] unless too close)
  * `u = normalize(cross(n, t))`
  * `v = cross(n, u)`
* Store `u` and `v` as the DOF axes.

#### floating

URDF floating is 6DoF.
Implementation:

* Create 6 DOF nodes chained:

  1. Tx (translation along x)
  2. Ty (translation along y)
  3. Tz (translation along z)
  4. Rx (rotation about x)
  5. Ry (rotation about y)
  6. Rz (rotation about z)
* Axes are unit basis vectors of the joint frame (after conversion to glTF coords).
* Document that the rotational DOF order is X→Y→Z (roll/pitch/yaw), matching common URDF expectations.

### Step 6 — Connect the kinematic tree in glTF

Once you create joint nodes, you must **re-parent child link nodes** so the glTF node hierarchy matches the URDF kinematic tree.

Root link handling:

* If URDF has one root link: make its link node a child of the model’s `rootNode`.
* If URDF has multiple roots: create a synthetic `world` root node and attach each root link node.

### Step 7 — Populate `EXT_robot_kinematics` extension data

Create one `KinematicModel`:

* `name`: URDF robot name (or file stem)
* `rootNode`: the glTF root node you created
* `links[]`: in stable order (e.g., URDF order)
* `joints[]`: in stable order; each joint includes:

  * `type`
  * `parentLink`, `childLink`
  * `originNode`, `dofNodes[]`
  * `dofs[]` with axis + limits + default
  * `dynamics` from URDF if present (damping/friction)
  * limits (effort/velocity/lower/upper)
  * mimic if present

### Step 8 — (Optional) generate preview animations

This is not required by the extension, but it’s very useful:

* For each 1‑DOF joint with limits, generate a glTF animation that moves from lower→upper.
* Keep DOF nodes TRS-based to obey glTF animation rules.

### Step 9 — Export `.glb`

Embed:

* meshes
* textures
* extension JSON

The `.glb` will load in any viewer; the kinematics will be “extra info” unless the runtime supports the extension.

---

## 7) Minimal example snippet

A single revolute joint robot, sketching only nodes + extension:

```jsonc
{
  "asset": { "version": "2.0" },
  "extensionsUsed": ["EXT_robot_kinematics"],
  "scenes": [{ "nodes": [0] }],
  "nodes": [
    { "name": "robot_root", "children": [1] },

    { "name": "base_link", "children": [2] },

    { "name": "joint_shoulder_origin", "translation": [0, 0.1, 0], "rotation": [0,0,0,1], "children": [3] },

    { "name": "joint_shoulder_dof0", "children": [4] },

    { "name": "upper_arm_link", "children": [5] },

    { "name": "upper_arm_visual", "mesh": 0 }
  ],
  "extensions": {
    "EXT_robot_kinematics": {
      "models": [{
        "name": "my_robot",
        "rootNode": 0,
        "links": [
          { "name": "base_link", "node": 1, "visualNodes": [] },
          { "name": "upper_arm_link", "node": 4, "visualNodes": [5] }
        ],
        "joints": [{
          "name": "shoulder",
          "type": "revolute",
          "parentLink": 0,
          "childLink": 1,
          "originNode": 2,
          "dofNodes": [3],
          "dofs": [{
            "motion": "rotation",
            "axis": [0, 0, 1],
            "limit": { "lower": -1.57, "upper": 1.57, "velocity": 2.0, "effort": 50.0 },
            "default": 0.0
          }]
        }]
      }]
    }
  }
}
```

---

## 8) Specification work plan (what you’d write in the actual extension spec)

If you were to publish this as a Khronos-style extension, the “spec plan” would include:

### A) Repo structure (typical Khronos extension layout)

* `README.md` (spec text)
* `schema/EXT_robot_kinematics.schema.json`
* `examples/` (minimal and complex robot examples)
* `test/` (validation assets)
* Optional: a small reference loader & validator rules

### B) README spec sections (outline)

1. **Overview / Motivation**

   * URDF-like kinematic semantics in glTF
   * Use cases: robotics visualization, digital twins, simulation prep
2. **glTF Extension Schema**

   * Root-level `models[]`
   * Definitions for Link, Joint, DOF, Limits, Mimic, etc.
3. **Node Organization Guidance**

   * canonical joint node chain
   * why origin vs DOF separation
4. **Coordinate System**

   * all axes and transforms are in glTF coordinates and units (meters/radians)
5. **Runtime Behavior**

   * how to apply a DOF value to a DOF node
6. **Validation Rules**
7. **URDF Conversion Guidelines**
8. **Examples**

### C) JSON schema details

* Strong typing for indices and enums
* Clear constraints for `fixed` joints (0 DOFs)
* Per-DOF limit object, since multi-DOF joints exist
* Optional `configurations` shape

---

## 9) Practical notes and edge cases you should explicitly document

1. **Joint origin vs link origin**
   URDF joint origin defines child pose relative to parent at zero.
   Always bake that into `originNode` (fixed) and keep DOF nodes identity at default.

2. **Visual/collision origins**
   URDF visuals/collisions have poses relative to link frame.
   Put those on child nodes under the link node.

3. **Unit consistency**
   glTF uses meters/radians.
   So does typical URDF practice; do not rescale unless your assets are authored in different units.

4. **Cycles / closed chains**
   glTF forbids cycles in node hierarchy.
   URDF typically avoids closed loops. If you encounter loop-like constructs (from extensions), you must represent them as constraints (future extension) rather than parenting cycles.

5. **Multiple visuals per link**
   Just multiple visual nodes; `visualNodes[]` is optional metadata.

6. **Non-standard URDF tags**
   Keep them in `extras` or a `source.otherXml` bucket if you want round-trip; don’t standardize them in v1.

---

## 10) Why this design converts cleanly and stays “glTF-native”

* All motion is expressible with core glTF transforms (TRS).
* If you choose, you can generate standard glTF animations that drive the DOF nodes.
* The extension adds the missing robotics semantics: axis meaning, limits, joint typing, and the mapping between joints and transform nodes.

---

If you want, I can also write:

* a draft `EXT_robot_kinematics.schema.json` (full JSON schema),
* a small reference implementation outline (Python or TypeScript) that parses URDF and emits a glTF JSON + binary buffers,
* and a “validator checklist” for common exporter mistakes (axis normalization, wrong node parenting, forgetting origin/DOF separation, bad basis conversion).

[1]: https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Vendor/AGI_articulations/README.md?utm_source=chatgpt.com "glTF/extensions/2.0/Vendor/AGI_articulations/README.md ..."
