# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Pure-Python type definitions for ovphysx.

This module contains IntEnum definitions that mirror the C enums in
ovphysx/include/ovphysx/ovphysx_types.h. It has zero native dependencies
(no ctypes, no shared library loading, no USD) and is safe to import in
any Python process regardless of USD version or native library state.

Keeping this module dependency-free is intentional: downstream consumers
like IsaacLab can import TensorType without triggering ovphysx's native
bootstrap or USD version checks.

Naming convention: strip OVPHYSX_TENSOR_ prefix and _F32 suffix from the
C enum name. This keeps names unambiguous (ARTICULATION_ vs RIGID_BODY_)
and makes the _bindings.py aliases mechanically verifiable.
"""

from enum import IntEnum


class TensorType(IntEnum):
    """Tensor type identifiers for TensorBindingsAPI.

    Values match ovphysx_tensor_type_t in ovphysx_types.h.
    IntEnum members compare equal to plain ints, so they pass directly
    to the C API without conversion.
    """

    INVALID = 0

    # -- Rigid body state (2D, read/write) --
    RIGID_BODY_POSE = 1           # [N, 7]  world-frame pose: (px,py,pz,qx,qy,qz,qw)
    RIGID_BODY_VELOCITY = 2       # [N, 6]  linear (xyz) + angular (xyz) velocity

    # -- Rigid body properties (2D, read/write) --
    RIGID_BODY_MASS = 3           # [N]     mass per body
    RIGID_BODY_INERTIA = 4        # [N, 9]  row-major 3x3 inertia tensor
    RIGID_BODY_COM_POSE = 5       # [N, 7]  center-of-mass pose in local frame

    # -- Articulation root (2D, read/write) --
    ARTICULATION_ROOT_POSE = 10                # [N, 7]  root link world-frame pose
    ARTICULATION_ROOT_VELOCITY = 11            # [N, 6]  root link linear + angular velocity

    # -- Articulation links (3D, read/write unless noted) --
    ARTICULATION_LINK_POSE = 20                # [N, L, 7]  per-link world-frame pose
    ARTICULATION_LINK_VELOCITY = 21            # [N, L, 6]  per-link linear + angular velocity
    ARTICULATION_LINK_ACCELERATION = 22        # [N, L, 6]  read-only; linear + angular acceleration

    # -- Articulation DOF state (2D, read/write) --
    ARTICULATION_DOF_POSITION = 30             # [N, D]  joint positions
    ARTICULATION_DOF_VELOCITY = 31             # [N, D]  joint velocities
    ARTICULATION_DOF_POSITION_TARGET = 32      # [N, D]  PD position targets
    ARTICULATION_DOF_VELOCITY_TARGET = 33      # [N, D]  PD velocity targets
    ARTICULATION_DOF_ACTUATION_FORCE = 34      # [N, D]  applied joint forces/torques

    # -- Articulation DOF properties (2D/3D, read/write) --
    ARTICULATION_DOF_STIFFNESS = 35            # [N, D]     PD stiffness (spring constant)
    ARTICULATION_DOF_DAMPING = 36              # [N, D]     PD damping coefficient
    ARTICULATION_DOF_LIMIT = 37                # [N, D, 2]  joint limits (lower, upper)
    ARTICULATION_DOF_MAX_VELOCITY = 38         # [N, D]     maximum joint velocity
    ARTICULATION_DOF_MAX_FORCE = 39            # [N, D]     maximum joint force/torque
    ARTICULATION_DOF_ARMATURE = 40             # [N, D]     reflected rotor inertia
    ARTICULATION_DOF_FRICTION_PROPERTIES = 41  # [N, D, 3]  (static, dynamic, viscous) friction

    # -- External wrenches (2D/3D, write-only) --
    RIGID_BODY_FORCE = 50                      # [N, 3]     force in world frame
    RIGID_BODY_WRENCH = 51                     # [N, 9]     [fx,fy,fz,tx,ty,tz,px,py,pz] row-major
    ARTICULATION_LINK_WRENCH = 52              # [N, L, 9]  per-link wrench, same layout

    # -- Articulation body properties (3D, read/write unless noted) --
    ARTICULATION_BODY_MASS = 60                # [N, L]     per-link mass
    ARTICULATION_BODY_COM_POSE = 61            # [N, L, 7]  per-link COM pose in local frame
    ARTICULATION_BODY_INERTIA = 62             # [N, L, 9]  row-major 3x3 inertia in COM frame
    ARTICULATION_BODY_INV_MASS = 63            # [N, L]     read-only; inverse mass
    ARTICULATION_BODY_INV_INERTIA = 64         # [N, L, 9]  read-only; inverse inertia

    # -- Articulation dynamics queries (2D/3D, read-only) --
    ARTICULATION_JACOBIAN = 70                 # [N, R, C]  geometric Jacobian
    ARTICULATION_MASS_MATRIX = 71              # [N, M, M]  generalized mass matrix
    ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE = 72  # [N, M]  Coriolis + centrifugal forces
    ARTICULATION_GRAVITY_FORCE = 73            # [N, M]     generalized gravity forces
    ARTICULATION_LINK_INCOMING_JOINT_FORCE = 74  # [N, L, 6]  joint reaction forces per link
    ARTICULATION_DOF_PROJECTED_JOINT_FORCE = 75  # [N, D]   read-only; joint forces projected to DOFs

    # -- Articulation fixed tendon properties (2D/3D, read/write) --
    ARTICULATION_FIXED_TENDON_STIFFNESS = 80        # [N, T]     tendon stiffness
    ARTICULATION_FIXED_TENDON_DAMPING = 81          # [N, T]     tendon damping
    ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS = 82  # [N, T]     limit spring stiffness
    ARTICULATION_FIXED_TENDON_LIMIT = 83            # [N, T, 2]  tendon length limits (lower, upper)
    ARTICULATION_FIXED_TENDON_REST_LENGTH = 84      # [N, T]     rest length
    ARTICULATION_FIXED_TENDON_OFFSET = 85           # [N, T]     length offset

    # -- Articulation spatial tendon properties (2D, read/write) --
    ARTICULATION_SPATIAL_TENDON_STIFFNESS = 90        # [N, T]   tendon stiffness
    ARTICULATION_SPATIAL_TENDON_DAMPING = 91          # [N, T]   tendon damping
    ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS = 92  # [N, T]   limit spring stiffness
    ARTICULATION_SPATIAL_TENDON_OFFSET = 93           # [N, T]   length offset

    # -- Shape-level properties (2D/3D, read/write).
    #    S = max collision shapes per body/link.  Bodies with fewer shapes
    #    have zero-padded trailing entries. --
    RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION = 100    # [N, S, 3] (static_friction, dynamic_friction, restitution)
    RIGID_BODY_CONTACT_OFFSET = 101                   # [N, S]    contact offset per shape
    RIGID_BODY_REST_OFFSET = 102                      # [N, S]    rest offset per shape

    ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION = 110  # [N, S, 3] (static_friction, dynamic_friction, restitution)
    ARTICULATION_CONTACT_OFFSET = 111                 # [N, S]    contact offset per shape
    ARTICULATION_REST_OFFSET = 112                    # [N, S]    rest offset per shape


class SceneQueryMode(IntEnum):
    """Scene query hit mode. Mirrors ovphysx_scene_query_mode_t."""

    CLOSEST = 0  # Return the single closest hit (or none)
    ANY     = 1  # Return whether any hit exists (0 or 1 result)
    ALL     = 2  # Return all hits


class SceneQueryGeometryType(IntEnum):
    """Geometry type for sweep/overlap queries. Mirrors ovphysx_scene_query_geometry_type_t."""

    SPHERE = 0  # Sphere defined by radius + center
    BOX    = 1  # Oriented box defined by half-extents + pose
    SHAPE  = 2  # Arbitrary UsdGeomGPrim identified by prim path


class PhysXType(IntEnum):
    """PhysX object type identifiers for ``ovphysx_get_physx_ptr()``.

    Values match ``ovphysx_physx_type_t`` in ``ovphysx_types.h`` (which in
    turn matches ``omni::physx::PhysXType``).  Alignment is enforced by
    static_asserts in ovphysxClone.cpp (C ↔ internal) and by
    test_types_sync.py (C ↔ Python).
    """

    SCENE          = 1   # PxScene
    MATERIAL       = 2   # PxMaterial
    SHAPE          = 3   # PxShape
    COMPOUND_SHAPE = 4   # omni::physx::PhysXCompoundShape (convex decomposition)
    ACTOR          = 5   # PxRigidDynamic / PxRigidStatic
    JOINT          = 6   # PxJoint (standalone joints only)
    CUSTOM_JOINT   = 7   # PxJoint (custom joints)
    ARTICULATION   = 8   # PxArticulationReducedCoordinate
    LINK            = 9   # PxArticulationLink
    LINK_JOINT      = 10  # PxArticulationJointReducedCoordinate
    PARTICLE_SYSTEM = 11  # PxPBDParticleSystem
    PARTICLE_SET    = 12  # PxParticleBuffer
    PHYSICS         = 31  # PxPhysics


class LogLevel(IntEnum):
    """Log level for ovphysx output. Mirrors ovphysx_log_level_t."""

    VERBOSE = 0
    INFO = 1
    WARNING = 2
    ERROR = 3
    NONE = 4


class ApiStatus(IntEnum):
    """Return codes from the ovphysx C API. Mirrors ovphysx_api_status_t."""

    SUCCESS = 0
    ERROR = 1
    TIMEOUT = 2
    NOT_IMPLEMENTED = 3
    INVALID_ARGUMENT = 4
    NOT_FOUND = 5
    BUFFER_TOO_SMALL = 6    # caller-supplied buffer is too small; check out_required_size
    DEVICE_MISMATCH = 7     # tensor device doesn't match the binding's expected device
    GPU_NOT_AVAILABLE = 8   # GPU requested but not available or CUDA init failed


class DeviceType(IntEnum):
    """Device selection for PhysX instance creation. Mirrors ovphysx_device_t."""

    AUTO = 0   # GPU preferred, CPU fallback.  AUTO=0 so zero-initialised create_args picks AUTO.
    GPU = 1    # GPU required
    CPU = 2    # CPU only


class ConfigBool(IntEnum):
    """Boolean config keys. Mirrors ovphysx_config_bool_t."""

    DISABLE_CONTACT_PROCESSING = 0
    COLLISION_CONE_CUSTOM_GEOMETRY = 1
    COLLISION_CYLINDER_CUSTOM_GEOMETRY = 2


class ConfigInt32(IntEnum):
    """Int32 config keys. Mirrors ovphysx_config_int32_t."""

    NUM_THREADS = 0
    SCENE_MULTI_GPU_MODE = 1


class ConfigFloat(IntEnum):
    """Float config keys (reserved). Mirrors ovphysx_config_float_t."""
    pass


class ConfigString(IntEnum):
    """String config keys (reserved). Mirrors ovphysx_config_string_t."""
    pass


class PhysXDeviceError(RuntimeError):
    """Raised when a PhysX instance requests a device mode that conflicts
    with the process-global mode locked by a prior ``PhysX()`` call.

    Device mode (CPU or GPU) is set once per process on the first
    ``PhysX()`` instantiation and cannot be changed.  To use a different
    device mode, run the code in a **separate subprocess**::

        import subprocess, sys
        subprocess.run([sys.executable, "my_cpu_script.py"], check=True)
    """


class BindingPrimMode(IntEnum):
    """Prim selection mode for tensor bindings.

    Unlike the other enums in this module, BindingPrimMode does not have a
    named typedef in ovphysx_types.h -- the values come from the internal
    implementation. It is not covered by test_types_sync.py.
    """

    EXISTING_ONLY = 0
    MUST_EXIST = 1
    CREATE_NEW = 2
