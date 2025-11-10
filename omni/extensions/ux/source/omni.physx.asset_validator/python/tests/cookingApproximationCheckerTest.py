# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.asset_validator.core.tests import ValidationRuleTestCase, IsAFailure
import carb
import carb.tokens
from pxr import Usd, UsdGeom, UsdPhysics

from ..scripts.cookingApproximationChecker import CookingApproximationChecker, CookingApproximationFallbackChecker

class CookingApproximationCheckerTestCase(ValidationRuleTestCase):

    @staticmethod
    def get_random_word(length: int) -> str:
        """Generates random ascii lowercase word of 'length' characters."""
        import random
        import string

        letters = string.ascii_lowercase + string.digits
        return "".join(random.choice(letters) for _ in range(length))
    
    async def test_cooking_approximation_convex_gpu_invalid(self):
        # Download the asset locally, so we can modify it (later)
        temp_path = carb.tokens.get_tokens_interface().resolve("${temp}/")
        temp_usd_file = temp_path
        temp_usd_file += self.get_random_word(8)
        temp_usd_file += "_CookingApproximationConvexGPUInvalid.usd"

        prim_path = "/World/thinCube"
        # create a simple thin cube mesh inside temp_usd_file without using scaling to make it thin
        # We need to specify vertices and indices to build the cube
        stage = Usd.Stage.CreateNew(temp_usd_file)
        z_height = 0.001
        x_width = 1.0
        y_width = 1.0
        vertices = [
            (-x_width/2, -y_width/2, -z_height),
            (x_width/2, -y_width/2, -z_height),
            (x_width/2, y_width/2, -z_height),
            (-x_width/2, y_width/2, -z_height),
            (-x_width/2, -y_width/2, z_height),
            (x_width/2, -y_width/2, z_height),
            (x_width/2, y_width/2, z_height),
            (-x_width/2, y_width/2, z_height),
        ]
        indices = [         
            0, 1, 2, 3,
            4, 5, 6, 7,
            0, 1, 5, 4,
            2, 3, 7, 6,
            0, 3, 7, 4,
            1, 2, 6, 5,
            0, 4, 5, 1,
            2, 6, 7, 3,
        ]
        mesh = UsdGeom.Mesh.Define(stage, prim_path)
        mesh.GetPointsAttr().Set(vertices)
        mesh.GetFaceVertexIndicesAttr().Set(indices)
        mesh.GetFaceVertexCountsAttr().Set([4,4,4,4,4,4,4,4])
        # Add Convex Approximation
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        meshCollisionApi = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        meshCollisionApi.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
        stage.Save()
        print(f"temp_usd_file={temp_usd_file}")

        # Check that issue exists
        self.assertRule(
            url=temp_usd_file,
            rule=CookingApproximationChecker,
            asserts=[
                IsAFailure(
                    f'Convex approximation for "{prim_path}" is not GPU compatible (likely the object is too thin)'
                ),
            ],
        )

        # Fix all issues with the suggestion provided by CookingApproximationChecker
        self.assertSuggestion(url=temp_usd_file, rule=CookingApproximationChecker, predicate=None)

class CookingApproximationFallbackCheckerTestCase(ValidationRuleTestCase):
   
    @staticmethod
    def get_random_word(length: int) -> str:
        """Generates random ascii lowercase word of 'length' characters."""
        import random
        import string

        letters = string.ascii_lowercase + string.digits
        return "".join(random.choice(letters) for _ in range(length))
    
    async def test_cooking_approximation_convex_fallback(self):
        # Download the asset locally, so we can modify it (later)
        temp_path = carb.tokens.get_tokens_interface().resolve("${temp}/")
        temp_usd_file = temp_path
        temp_usd_file += self.get_random_word(8)
        temp_usd_file += "_CookingApproximationConvexFallback.usd"

        prim_path = "/World/Cube"
        # create a simple cube mesh inside temp_usd_file without using scaling to make it thin
        # We need to specify vertices and indices to build the cube
        stage = Usd.Stage.CreateNew(temp_usd_file)
        z_height = 0.001
        x_width = 1.0
        y_width = 1.0
        vertices = [
            (-x_width/2, -y_width/2, -z_height),
            (x_width/2, -y_width/2, -z_height),
            (x_width/2, y_width/2, -z_height),
            (-x_width/2, y_width/2, -z_height),
            (-x_width/2, -y_width/2, z_height),
            (x_width/2, -y_width/2, z_height),
            (x_width/2, y_width/2, z_height),
            (-x_width/2, y_width/2, z_height),
        ]
        indices = [         
            0, 1, 2, 3,
            4, 5, 6, 7,
            0, 1, 5, 4,
            2, 3, 7, 6,
            0, 3, 7, 4,
            1, 2, 6, 5,
            0, 4, 5, 1,
            2, 6, 7, 3,
        ]
        mesh = UsdGeom.Mesh.Define(stage, prim_path)
        mesh.GetPointsAttr().Set(vertices)
        mesh.GetFaceVertexIndicesAttr().Set(indices)
        mesh.GetFaceVertexCountsAttr().Set([4,4,4,4,4,4,4,4])
        # Add Convex Approximation        
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        meshCollisionApi = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        meshCollisionApi.CreateApproximationAttr().Set(UsdPhysics.Tokens.none)

        # Add Rigid Body
        UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim())
        stage.Save()
        print(f"temp_usd_file={temp_usd_file}")

        # Check that issue exists
        self.assertRule(
            url=temp_usd_file,
            rule=CookingApproximationFallbackChecker,
            asserts=[
                IsAFailure(
                    f'Triangle mesh approximation for "{prim_path}" cant be used for dynamic bodies.'
                ),
            ],
        )

        # Fix all issues with the suggestion provided by CookingApproximationFallbackChecker
        self.assertSuggestion(url=temp_usd_file, rule=CookingApproximationFallbackChecker, predicate=None)
