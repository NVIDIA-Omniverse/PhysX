# Rigid Bodies

Rigid bodies represent objects that move without deforming. Their motion comes
from gravity, collisions, and constraints such as [joints](joints.md); a
kinematic body is instead driven directly by the application. A rigid body is a
`UsdGeom.Xformable` prim with `PhysicsRigidBodyAPI` applied — that prim and all
of its descendants move as one rigid object.

This page covers authoring rigid bodies for scenes that ovphysx loads and how
their state is read and written at runtime. It builds on
[Physics Scene](physics_scene.md) and [Colliders](collision.md).

> `resetXformStack` on a rigid body prim decouples its transform from its parent.
> This is what makes it valid to nest a rigid body inside another prim's
> hierarchy (otherwise disallowed).

## Create a Rigid Body

```usda
def Xform "rigidBody" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
)
{
}
```

```python
from pxr import UsdGeom, UsdPhysics

xform = UsdGeom.Xform.Define(stage, "/World/rigidBody")
UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
```

On its own, a rigid body has no collider and will fall through everything. Add a
collider (below) so it interacts.

## Dynamic vs Kinematic

- **Dynamic** (default): the simulator moves the body under gravity,
  collisions, and constraints. The simulation **writes** its transform.
- **Kinematic**: the application controls the motion; the body pushes dynamic
  bodies but ignores forces. The simulation **reads** its transform.

```python
rb = UsdPhysics.RigidBodyAPI.Apply(prim)
rb.CreateKinematicEnabledAttr(True)   # kinematic; omit or False for dynamic
```

At runtime you drive a kinematic body's target pose through
[tensor bindings](../tutorials/tensor_bindings.md) or, in C/C++, through the raw
PhysX pointer (`PxRigidDynamic::setKinematicTarget()`) — refer to the
[PhysX Interop](../tutorials/physx_interop.md) tutorial.

## Disabling a Rigid Body

Setting `physics:rigidBodyEnabled = False` cancels the `PhysicsRigidBodyAPI`:
colliders in the sub-tree then behave as static geometry. For purely static
geometry, prefer simply not applying `PhysicsRigidBodyAPI` at all.

## Rigid Body Frames

A rigid body has several associated coordinate frames: the rigid-body-prim frame,
the center-of-mass frame (relative to the prim frame), and the collider and joint
local frames (also relative to the prim frame). Internally the simulation
proceeds with reference to the center of mass; the prim frame gives joints and
colliders a common reference, so the center of mass can be moved without
disturbing joint or collider frames.

## Colliders on a Rigid Body

A collider is associated with a rigid body by hierarchy: put the collider on the
body prim, or on a child prim that is posed relative to the body. The tested
idiom is a body `Xform` with a child geometry prim carrying
`PhysicsCollisionAPI`:

```python
from pxr import UsdGeom, UsdPhysics

body = UsdGeom.Xform.Define(stage, "/World/rigidBody")
UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())

cube = UsdGeom.Cube.Define(stage, "/World/rigidBody/cube")
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
```

Add several child geometry prims for a compound (multi-collider) body; all move
rigidly with the body. Collider types and mesh approximations are covered in
[Colliders](collision.md).

## Materials

`UsdPhysics.MaterialAPI` on a material prim defines static friction, dynamic
friction, restitution, and density. Bind it to a body (or collider) with
`UsdShade.MaterialBindingAPI` using the `"physics"` purpose; physics materials
follow the same binding-strength resolution as render materials.

```python
from pxr import UsdShade, UsdPhysics

material = UsdShade.Material.Define(stage, "/World/material")
mat_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
mat_api.CreateStaticFrictionAttr(1.0)
mat_api.CreateDynamicFrictionAttr(0.9)
mat_api.CreateRestitutionAttr(0.8)

binding = UsdShade.MaterialBindingAPI.Apply(body.GetPrim())
binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
```

### Combine Modes

A contact involves two materials. The effective friction and restitution are
computed from both using each material's **combine mode**, set on the codeless
`PhysxMaterialAPI` (`physxMaterial:frictionCombineMode`,
`physxMaterial:restitutionCombineMode`). Supported modes: `average` (default),
`min`, `multiply`, `max`. When the two materials disagree, the "largest" mode
wins in the order `average < min < multiply < max`.

```python
from pxr import Sdf

material_prim.ApplyAPI("PhysxMaterialAPI")
material_prim.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
material_prim.CreateAttribute("physxMaterial:restitutionCombineMode", Sdf.ValueTypeNames.Token).Set("multiply")
```

### Compliant Contacts

As an alternative to restitution-based rigid contact, a material can be prepared
for **compliant contact** whose normal force is an implicit spring governed by
stiffness and damping. Set a non-zero `physxMaterial:compliantContactStiffness`
(and optionally `physxMaterial:compliantContactDamping`). If both materials in a
pair are compliant, the stiffness combine mode follows the restitution combine
mode and the damping follows the damping combine mode. A material can also be
prepared as an *acceleration* spring rather than a force spring.

Shape friction and restitution can also be read/written in bulk at runtime through
the `RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION` tensor type — refer to the
[Tensor Bindings](../tutorials/tensor_bindings.md) reference.

## Mass Properties

A rigid body's mass distribution is defined by mass, center of mass, moment of
inertia, and principal axes.

### Explicit

Apply `UsdPhysics.MassAPI` and set the properties directly. No collider is
required.

```python
from pxr import UsdPhysics, Gf

mass_api = UsdPhysics.MassAPI.Apply(body.GetPrim())
mass_api.CreateMassAttr(3.0)
mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(2.0, 2.0, 2.0))
mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 3.0, 0.0))
mass_api.CreatePrincipalAxesAttr(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90.0).GetQuat()))
```

### Implicit (density x volume)

With no mass set, the simulator infers mass from collider volume and density.
Density can come from `MassAPI` or from a bound `MaterialAPI`. Combination
follows **inheritance** (a child inherits an unset attribute from its parent) and
**precedence**:

```
MassAPI:mass > MassAPI:density > MaterialAPI:density > default density
```

The same precedence applies to center of mass and moment of inertia. The default
density is 1000 kg/m^3 (water). If there is no collider and no mass, mass
defaults to 1.0.

> When mass is not authored, it is not exposed as a plain USD attribute. To read
> the computed mass/inertia (and center of mass), use the `RIGID_BODY_MASS`,
> `RIGID_BODY_INERTIA`, and `RIGID_BODY_COM_POSE` tensor types — refer to
> [Tensor Bindings](../tutorials/tensor_bindings.md). Those types are also
> writable to override mass at runtime; the `RIGID_BODY_INV_MASS` /
> `RIGID_BODY_INV_INERTIA` variants are read-only.

## Continuous Collision Detection

Enable CCD when fast-moving or thin objects tunnel (miss contacts because
simulation time advances in discrete steps). Two forms exist:

- **Sweep-based (linear) CCD** — sweeps linear motion over the timestep to catch
  missed collisions. Ignores angular motion.
- **Speculative (angular) CCD** — a cheaper, discrete approach that computes
  dynamic contact offsets from motion; catches fast angular motion and most
  linear tunneling, but can introduce ghost collisions under large linear
  motion.

Both can be enabled together (**hybrid CCD**) for a best-of-both result.

- Sweep-based CCD requires the scene attribute plus the per-body attribute:

```python
from pxr import Sdf

scene_prim.ApplyAPI("PhysxSceneAPI")
scene_prim.CreateAttribute("physxScene:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)

body_prim.ApplyAPI("PhysxRigidBodyAPI")
body_prim.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
```

- Speculative CCD needs only the per-body attribute
  `physxRigidBody:enableSpeculativeCCD`. Kinematic objects support speculative
  CCD only.

> Neither CCD mode works in Direct GPU API mode. Both go through the CPU
> pipeline and noticeably affect GPU-simulation performance.

## Velocities

Initial linear and angular velocity are authored on `PhysicsRigidBodyAPI`
(`physics:velocity`, `physics:angularVelocity`), reported at the center of mass in
the world frame. At runtime, read and write velocity in bulk through the
`RIGID_BODY_VELOCITY` tensor type; `RIGID_BODY_ACCELERATION` is read-only.

> A body that is part of an articulation only accepts authored velocities if it
> is the root link of a floating-base articulation; velocities on other
> articulation links are ignored (refer to [Articulations](articulations.md)).

## Forces and Torques

Apply forces and torques to a dynamic body with the codeless `PhysxForceAPI`.
They act at the center of mass, in either the world frame or the prim's local
frame (`physxForce:worldFrameEnabled`), and can be applied as forces/torques or
as accelerations (`physxForce:mode`).

```python
from pxr import Sdf, Gf

body_prim.ApplyAPI("PhysxForceAPI")
body_prim.CreateAttribute("physxForce:mode", Sdf.ValueTypeNames.Token).Set("force")
body_prim.CreateAttribute("physxForce:worldFrameEnabled", Sdf.ValueTypeNames.Bool).Set(True)
body_prim.CreateAttribute("physxForce:force", Sdf.ValueTypeNames.Vector3f).Set(Gf.Vec3f(0, 1000, 0))
body_prim.CreateAttribute("physxForce:torque", Sdf.ValueTypeNames.Vector3f).Set(Gf.Vec3f(0, 0, 2000))
```

By default an applied force lasts one step and is reset to zero; the
`retainAccelerations` attribute of `PhysxRigidBodyAPI` keeps it applied until
reset. For RL-style loops, prefer writing forces in bulk through the
**write-only** `RIGID_BODY_FORCE` (`[N,3]`, at COM) or `RIGID_BODY_WRENCH`
(`[N,9]`, force + torque + position) tensor types — refer to
[Tensor Bindings](../tutorials/tensor_bindings.md).

## Surface Velocity (Conveyors)

`PhysxSurfaceVelocityAPI` (codeless) simulates conveyor-like behavior by
injecting a surface linear/angular velocity, and applies to both kinematic and
dynamic bodies. Changing a surface velocity does not wake bodies resting on it,
so if you will change it from zero, disable sleeping on those bodies. A spline
variant (`PhysxSplinesSurfaceVelocityAPI`) drives motion along a `BasisCurves`
path.

## Instancing

USD supports two instancing mechanisms, both usable with rigid bodies (but not
with articulation links):

- **Scenegraph instancing** references shared **collision geometry** so many
  bodies reuse one collision archetype. Rigid-body parameters (mass, etc.) are
  not shared — set them per body. Shared collider attributes cannot be modified;
  create individual instances if you need per-body colliders.
- **Point instancing** (`UsdGeom.PointInstancer`) instances the **entire rigid
  body** from a prototype. The prototype itself is not simulated; only the
  instances are. Poses and velocities are driven through the point instancer's
  arrays (`positions`, `orientations`, `velocities`, `angularVelocities`), not
  through per-prim attributes. Point-instanced bodies cannot have joints unless
  you use `PhysxPhysicsJointInstancer`.

Point-instancer instances are not exposed as rows in rigid-body tensor bindings;
with the default empty-binding behavior, a binding that targets only the point
instancer has count zero. Read simulated instance state through the `RIGID_BODY`
object type in the ovstage [output read](../ovstage_integration.md) API. To
control instances, author the point instancer arrays through ovstage and pass
those control ordinals to `update_from_ovstage()`.
