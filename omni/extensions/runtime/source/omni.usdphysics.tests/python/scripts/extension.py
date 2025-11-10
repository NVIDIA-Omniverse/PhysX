# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
from .. import get_usd_physics_test_interface

from .tests.physicsScene import UsdPhysicsSceneTest
from .tests.collisionGroup import UsdPhysicsCollisionGroupTest
from .tests.collision import UsdPhysicsCollisionTest
from .tests.rigidBody import UsdPhysicsRigidBodyTest
from .tests.deformableBody import UsdPhysicsDeformableBodyTest
from .tests.fixedJoint import UsdPhysicsFixedJointTest
from .tests.revoluteJoint import UsdPhysicsRevoluteJointTest
from .tests.prismaticJoint import UsdPhysicsPrismaticJointTest
from .tests.distanceJoint import UsdPhysicsDistanceJointTest
from .tests.sphericalJoint import UsdPhysicsSphericalJointTest
from .tests.d6Joint import UsdPhysicsD6JointTest
from .tests.articulation import UsdPhysicsArticulationTest
from .tests.attachment import UsdPhysicsAttachmentTest


class PhysicsSchemaTestsExtension(omni.ext.IExt):
    def on_startup(self):
        # Init interface cache
        get_usd_physics_test_interface()

    def on_shutdown(self):
        pass
