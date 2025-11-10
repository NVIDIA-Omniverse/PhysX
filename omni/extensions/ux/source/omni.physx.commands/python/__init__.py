# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import inspect
import carb
import omni.kit.commands
from omni.kit.usd_undo import UsdLayerUndo
from omni.kit.commands import Command, execute, register_all_commands_in_module
from omni.physx import get_physx_interface
from omni.physx.scripts import physicsUtils, utils, deformableUtils, particleUtils
from omni.physx.scripts.pythonUtils import autoassign
from omni.usd.commands.usd_commands import DeletePrimsCommand
from omni.physx.bindings._physx import SETTING_VISUALIZATION_GAP
from typing import Any
from pxr import Usd, Sdf, Gf, UsdGeom, UsdPhysics, PhysxSchema
from functools import partial
import typing
import fnmatch

def local_execute_apply(lst, api, prim, api_prefix=None, multiple_api_token=None):
    sc = ApplyAPISchemaCommand(api, prim, api_prefix, multiple_api_token)
    lst.append(sc)
    return True, sc.do()


def local_execute_unapply(lst, api, prim, api_prefix=None, multiple_api_token=None):
    sc = UnapplyAPISchemaCommand(api, prim, api_prefix, multiple_api_token)
    lst.append(sc)
    return True, sc.do()


class PhysicsCommand(Command):
    """
    Base class for physics commands. Adds an execute helper to not force the user to use only keyword arguments
    """
    @classmethod
    def execute(cls, *args, **kwargs):
        sig = inspect.signature(cls.__init__)
        bound = sig.bind(0, *args, **kwargs)
        bound.arguments.pop("self")
        return execute(cls.__name__, **bound.kwargs)


class AddPhysicsSceneCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.addPhysicsScene. Adds a UsdPhysics.Scene prim with default params.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at. 
    """
    @autoassign
    def __init__(self, stage, path):
        pass

    def do(self):
        self._primPaths = utils.addPhysicsScene(self._stage, self._path)
        omni.usd.get_context().get_selection().set_selected_prim_paths([self._path], True)

    def undo(self):
        DeletePrimsCommand(self._primPaths).do()


class AddRigidBodyMaterialCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.addRigidBodyMaterial. Creates a UsdShade.Material prim if needed and adds a UsdPhysics.MaterialAPI to it.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at. 
        density: Physics material param.
        staticFriction: Physics material param.
        dynamicFriction: Physics material param.
        restitution: Physics rigid material param.
    """
    @autoassign
    def __init__(self, stage, path, density=None, staticFriction=None, dynamicFriction=None, restitution=None):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        utils.addRigidBodyMaterial(self._stage, self._path, self._density, self._staticFriction,
                                   self._dynamicFriction, self._restitution)
        omni.usd.get_context().get_selection().set_selected_prim_paths([self._path], True)

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(UsdPhysics.MaterialAPI, prim).do()


# DEPRECATED
class AddDeformableBodyMaterialCommand(PhysicsCommand):
    """
    DEPRECATED:
    Wrapper for omni.physx.deformableUtils.add_deformable_body_material.
    Creates a UsdShade.Material prim if needed and adds a PhysxSchema.PhysxDeformableBodyMaterialAPI to it.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at. 
        dampingScale: Physics material param.
        density: Physics material param.
        dynamicFriction: Physics material param.
        elasticityDamping: Physics material param.
        poissonsRatio: Physics material param.
        youngsModulus: Physics material param.
    """
    @autoassign
    def __init__(self, stage, path, dampingScale=None, density=None, dynamicFriction=None, 
                 elasticityDamping=None, poissonsRatio=None, youngsModulus=None):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        deformableUtils.add_deformable_body_material(
            stage=self._stage,
            path=self._path,
            damping_scale=self._dampingScale,
            dynamic_friction=self._dynamicFriction,
            density=self._density,
            elasticity_damping=self._elasticityDamping,
            poissons_ratio=self._poissonsRatio,
            youngs_modulus=self._youngsModulus)
        omni.usd.get_context().get_selection().set_selected_prim_paths([self._path], True)

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(PhysxSchema.PhysxDeformableBodyMaterialAPI, prim).do()


# DEPRECATED
class AddDeformableSurfaceMaterialCommand(PhysicsCommand):
    """
    DEPRECATED:
    Wrapper for omni.physx.deformableUtils.add_deformable_surface_material.
    Creates a UsdShade.Material prim if needed and adds a PhysxSchema.PhysxDeformableSurfaceMaterialAPI to it.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at.
        density: Physics material param.
        dynamicFriction: Physics material param.
        poissonsRatio: Physics material param.
        thickness: Physics material param.
        youngsModulus: Physics material param.
    """
    @autoassign
    def __init__(self, stage, path, density=None, dynamicFriction=None, poissonsRatio=None, thickness=None,
                  youngsModulus=None):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        deformableUtils.add_deformable_surface_material(
            stage=self._stage,
            path=self._path,
            dynamic_friction=self._dynamicFriction,
            density=self._density,
            poissons_ratio=self._poissonsRatio,
            thickness=self._thickness,
            youngs_modulus=self._youngsModulus,
        )
        omni.usd.get_context().get_selection().set_selected_prim_paths([self._path], True)

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(PhysxSchema.PhysxDeformableSurfaceMaterialAPI, prim).do()


class AddDeformableMaterialCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.deformableUtils.add_deformable_material. Creates a UsdShade.Material prim if needed and
    adds a UsdPhysics.DeformableMaterialAPI to it.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at. 
        density: Physics material param.
        static_friction: Physics material param.
        dynamic_friction: Physics material param.
        youngs_modulus: Physics deformable material param.
        poissons_ratio: Physics deformable material param.
    """
    @autoassign
    def __init__(self, stage, path,
        density=None, static_friction=None, dynamic_friction=None,
        youngs_modulus=None, poissons_ratio=None
    ):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        deformableUtils.add_deformable_material(self._stage, self._path,
            self._density, self._static_friction, self._dynamic_friction, self._youngs_modulus, self._poissons_ratio
        )
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(self._path)], True)

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsDeformableMaterialAPI"), prim).do()


class AddSurfaceDeformableMaterialCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.deformableUtils.add_surface_deformable_material. Creates a UsdShade.Material prim if needed and
    adds a UsdPhysics.SurfaceDeformableMaterialAPI to it.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at. 
        density: Physics material param.
        static_friction: Physics material param.
        dynamic_friction: Physics material param.
        youngs_modulus: Physics deformable material param.
        poissons_ratio: Physics deformable material param.
        surface_thickness: Physics surface deformable material param.
        surface_stretch_stiffness: Physics surface deformable material param.
        surface_shear_stiffness: Physics surface deformable material param.
        surface_bend_stiffness: Physics surface deformable material param.
    """
    @autoassign
    def __init__(self, stage, path,
        density=None, static_friction=None, dynamic_friction=None,
        youngs_modulus=None, poissons_ratio=None,
        surface_thickness=None, surface_stretch_stiffness=None, surface_shear_stiffness=None, surface_bend_stiffness=None
    ):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        deformableUtils.add_surface_deformable_material(self._stage, self._path,
            self._density, self._static_friction, self._dynamic_friction,
            self._youngs_modulus, self._poissons_ratio,
            self._surface_thickness, self._surface_stretch_stiffness, self._surface_shear_stiffness, self._surface_bend_stiffness
        )
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(self._path)], True)

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsSurfaceDeformableMaterialAPI"), prim).do()


class AddCollisionGroupCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.addCollisionGroup. Creates a UsdPhysics.CollisionGroup prim.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at. 
    """
    @autoassign
    def __init__(self, stage, path):
        pass

    def do(self):
        utils.addCollisionGroup(self._stage, self._path)
        omni.usd.get_context().get_selection().set_selected_prim_paths([self._path], True)

    def undo(self):
        DeletePrimsCommand([self._path]).do()


class AddPairFilterCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.addPairFilter. Filters out collisions between primitives using UsdPhysics.FilteredPairsAPI.

    Args:
        stage: USD stage.
        primPaths: List of paths. 
    """
    @autoassign
    def __init__(self, stage, primPaths):
        pass

    def do(self):
        utils.addPairFilter(self._stage, self._primPaths, partial(execute, "ApplyAPISchema"))

    # needs to be defined so that sub-commands undo is called
    def undo(self):
        pass


class RemovePairFilterCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.removePairFilter. Removes UsdPhysics.FilteredPairsAPI from primitives.

    Args:
        stage: USD stage.
        primPaths: List of paths. 
    """
    @autoassign
    def __init__(self, stage, primPaths):
        pass

    def do(self):
        utils.removePairFilter(self._stage, self._primPaths, partial(execute, "ApplyAPISchema"))

    # needs to be defined so that sub-commands undo is called
    def undo(self):
        pass


class AddGroundPlaneCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.physicsUtils.add_ground_plane. Adds a zero-thick plane to prevent physics-enabled prims from \
    falling to infinity. Creates an UsdGeom.Xform with a UsdGeom.Mesh and a UsdGeom.Plane child primitives.

    Args:
        stage: USD stage.
        planePath: Path for the root xform to be created at. Finds first free path.
        axis: Up axis.
        size: Halfsize of one side.
        position: Center of the plane.
        color: Display color.

    Returns:
        Path where the plane was actually created.
    """
    @autoassign
    def __init__(self, stage, planePath, axis, size, position, color):
        pass

    def do(self):
        self._actPath = physicsUtils.add_ground_plane(self._stage, self._planePath, self._axis, self._size, self._position, self._color)
        omni.usd.get_context().get_selection().set_selected_prim_paths([self._actPath], True)
        return self._actPath

    def undo(self):
        DeletePrimsCommand([self._actPath]).do()


class SetRigidBodyCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.setRigidBody. Applies UsdPhysics.RigidBodyAPI and a UsdPhysics.CollisionAPI to target prim. \
    Applies UsdPhysics.MeshCollisionAPI if it's a mesh. Collision API's are also applied to \
    the whole subtree if the target prim is an xform.

    Args:
        path: Path of the target primitive.
        approximationShape: Physics param.
        kinematic: Physics param.
    """
    @autoassign
    def __init__(self, path, approximationShape="convexHull", kinematic=False):
        self._usd_context = omni.usd.get_context()
        self._loc_commands = []

    def do(self):
        prim = self._usd_context.get_stage().GetPrimAtPath(self._path)
        utils.setRigidBody(prim, self._approximationShape, self._kinematic, partial(local_execute_apply, self._loc_commands))

    # needs to be defined so that sub-commands undo is called
    def undo(self):
        prim = self._usd_context.get_stage().GetPrimAtPath(self._path)

        for sc in self._loc_commands:
            sc.undo()

        UnapplyAPISchemaCommand(PhysxSchema.PhysxRigidBodyAPI, prim).do()
        UnapplyAPISchemaCommand(PhysxSchema.PhysxCollisionAPI, prim).do()


class SetStaticColliderCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.setStaticCollider. Applies Collision APIs (UsdPhysics.CollisionAPI,  UsdPhysics.MeshCollisionAPI) \
    to a target prim and its subtree.

    Args:
        path: Path of the target primitive.
        approximationShape: Physics param.
    """
    @autoassign
    def __init__(self, path, approximationShape="none"):
        self._usd_context = omni.usd.get_context()
        self._loc_commands = []

    def do(self):
        prim = self._usd_context.get_stage().GetPrimAtPath(self._path)
        utils.setStaticCollider(prim, self._approximationShape, partial(local_execute_apply, self._loc_commands))

    # needs to be defined so that sub-commands undo is called
    def undo(self):
        prim = self._usd_context.get_stage().GetPrimAtPath(self._path)

        for sc in self._loc_commands:
            sc.undo()

        UnapplyAPISchemaCommand(PhysxSchema.PhysxCollisionAPI, prim).do()


class RemoveRigidBodyCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.removeRigidBody.

    Args:
        path: Path of the target primitive.
    """
    @autoassign
    def __init__(self, path):
        self._usd_context = omni.usd.get_context()

    def do(self):
        prim = self._usd_context.get_stage().GetPrimAtPath(self._path)
        utils.removeRigidBody(prim, partial(execute, "UnapplyAPISchema"))

    # needs to be defined so that sub-commands will get a chance to undo
    def undo(self):
        pass


class RemoveStaticColliderCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.removeStaticCollider.

    Args:
        path: Path of the target primitive.
    """
    @autoassign
    def __init__(self, path):
        self._usd_context = omni.usd.get_context()

    def do(self):
        prim = self._usd_context.get_stage().GetPrimAtPath(self._path)
        utils.removeStaticCollider(prim, partial(execute, "UnapplyAPISchema"))

    # needs to be defined so that sub-commands undo is called
    def undo(self):
        pass


class CreateJointCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.createJoint. Connects two primitives with a physical joints.

    Args:
        stage: Path of the target primitive.
        joint_type: Fixed, Revolute, Prismatic, Spherical or Distance. If left blank a D6 Joint is used.
        from_prim: From primitive.
        to_prim: To primitive.

    Returns: 
        Joint primitive.
    """
    @autoassign
    def __init__(self, stage, joint_type, from_prim, to_prim):
        pass

    def do(self):
        prim = utils.createJoint(self._stage, self._joint_type, self._from_prim, self._to_prim)
        self._primPath = prim.GetPath().pathString
        return prim

    def undo(self):
        DeletePrimsCommand([self._primPath]).do()


class CreateJointsCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.createJoints. Connects a list of primitives with their parent or a pseudo-root with physical joints.

    Args:
        stage: Path of the target primitive.
        joint_type: Fixed, Revolute, Prismatic, Spherical or Distance. If left blank a D6 Joint is used.
        paths: A list of paths.
        join_to_parent: Connect primitives to their parents if True, otherwise to a scene pseudo-root.

    Returns: 
        Joint primitives.
    """
    @autoassign
    def __init__(self, stage, joint_type, paths, join_to_parent=False):
        pass

    def do(self):
        prims = utils.createJoints(self._stage, self._joint_type, self._paths, self._join_to_parent)
        self._primPathsList = [prim.GetPath().pathString for prim in prims]
        return prims

    def undo(self):
        DeletePrimsCommand(self._primPathsList).do()


# DEPRECATED
class AddDeformableBodyComponentCommand(Command):
    """
        DEPRECATED:
        Adds a deformable body component to a skin UsdGeom.Mesh using an optional set of TetrahedralMesh paths to define collision and simulation meshes.

        Parameters:
            skin_mesh_path:                                 Path to UsdGeom.Mesh to which PhysxDeformableBodyAPI is applied and that will be driven by the
                                                            simulation.

            collision_mesh_path:                            *Optional* path to collision PhysxSchema.TetrahedralMesh. If not provided, it is
                                                            created from the mesh at skin_mesh_path.

            simulation_mesh_path:                           *Optional* path to simulation PhysxSchema.TetrahedralMesh. If not provided, it is
                                                            created from the collision TetrahedralMesh. The simulation mesh path may be identical to
                                                            the collision mesh path.

            kinematic_enabled:                              Enables kinematic body. Kinematic bodies do not support custom collision or simulation meshes.

            voxel_resolution:                               Resolution along longest axis-aligned-bounding-box axis to create simulation
                                                            TetrahedralMesh from voxelizing collision TetrahedralMesh.

            collision_simplification:                       Boolean flag indicating if simplification should be applied to the mesh before creating a
                                                            softbody out of it.

            collision_simplification_remeshing:             Boolean flag indicating if the simplification should be based on remeshing.
                                                            Ignored if collision_simplification equals False.

            collision_simplification_remeshing_resolution:  The resolution used for remeshing. A value of 0 indicates that a heuristic is used to determine
                                                            the resolution. Ignored if collision_simplification_remeshing is False.

            collision_simplification_target_triangle_count: The target triangle count used for the simplification. A value of 0 indicates
                                                            that a heuristic based on the simulation_hexahedral_resolution is to determine the target count.
                                                            Ignored if collision_simplification equals False.

            collision_simplification_force_conforming:      Boolean flag indicating that the tretrahedralizer used to generate the collision mesh should produce
                                                            tetrahedra that conform to the triangle mesh. If False the implementation chooses the tretrahedralizer
                                                            used. 

            ...:                                            Optional USD schema attributes, please refer to USD schema documentation.
        Returns:
            True / False that indicates success of command execution
    """
    @autoassign
    def __init__(self, skin_mesh_path: Sdf.Path,
                 collision_mesh_path: Sdf.Path = Sdf.Path(),
                 simulation_mesh_path: Sdf.Path = Sdf.Path(),
                 kinematic_enabled: bool = None,
                 voxel_resolution: int = 10,
                 collision_simplification: bool = True,
                 collision_simplification_remeshing: bool = None,
                 collision_simplification_remeshing_resolution: int = 0,
                 collision_simplification_target_triangle_count: int = 0,
                 collision_simplification_force_conforming: bool = None,
                 solver_position_iteration_count: int = None,
                 vertex_velocity_damping: float = None,
                 sleep_damping: float = None,
                 sleep_threshold: float = None,
                 settling_threshold: float = None,
                 self_collision: bool = None,
                 self_collision_filter_distance: float = None):
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        # check paths and mesh input:
        if not self._is_path_and_mesh_configuration_valid():
            return False

        # automatic kinematic, if we see that the skin mesh is time varying
        # we don't check for transform varying yet, however both are checked for during parsing
        # for runtime fallback
        if self._kinematic_enabled is None:
            skin_mesh = UsdGeom.Mesh.Get(self._stage, self._skin_mesh_path)
            is_skin_mesh_time_varying = skin_mesh.GetPointsAttr().GetNumTimeSamples() > 1
            self._kinematic_enabled = is_skin_mesh_time_varying

        if self._collision_simplification_remeshing is None:
            self._collision_simplification_remeshing = not self._kinematic_enabled

        if self._collision_simplification_force_conforming is None:
            self._collision_simplification_force_conforming = self._kinematic_enabled

        # at this point we are guaranteed to have a valid path and mesh configuration and can proceed
        self._usd_undo.reserve(Sdf.Path(self._skin_mesh_path))
        self._usd_undo.reserve(Sdf.Path(self._collision_mesh_path))
        self._usd_undo.reserve(Sdf.Path(self._simulation_mesh_path))

        coll_mesh_points = None
        coll_mesh_indices = None
        if self._collision_mesh_path:
            coll_tet_mesh = PhysxSchema.TetrahedralMesh(self._stage.GetPrimAtPath(self._collision_mesh_path))
            coll_mesh_data = deformableUtils.TetMeshData()
            coll_mesh_data.from_mesh(coll_tet_mesh)
            coll_mesh_points = coll_mesh_data.points
            coll_mesh_indices = coll_mesh_data.indices

        sim_mesh_points = None
        sim_mesh_indices = None
        sim_mesh_embedding = None
        sim_mesh_resolution = self._voxel_resolution
        if self._simulation_mesh_path:
            sim_tet_mesh = PhysxSchema.TetrahedralMesh(self._stage.GetPrimAtPath(self._simulation_mesh_path))
            sim_mesh_data = deformableUtils.TetMeshData()
            sim_mesh_data.from_mesh(sim_tet_mesh)
            sim_mesh_points = sim_mesh_data.points
            sim_mesh_indices = sim_mesh_data.indices
            sim_mesh_embedding = sim_mesh_data.embedding
            #don't allow resolution if sim mesh is provided
            sim_mesh_resolution = 0
        elif self._collision_mesh_path:
            # if a simulation mesh is not provided, but a collision mesh is, generate the simulation mesh using
            # the collision mesh
            skin_prim = self._stage.GetPrimAtPath(self._skin_mesh_path)
            source_to_world = UsdGeom.Xformable(skin_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            scale = Gf.Vec3f(Gf.Transform(source_to_world).GetScale())
            sim_mesh_points, sim_mesh_indices = deformableUtils.compute_voxel_tetrahedral_mesh(
                coll_mesh_points,
                coll_mesh_indices,
                scale,
                self._voxel_resolution)

        success = deformableUtils.add_physx_deformable_body(
            self._stage,
            self._skin_mesh_path,
            coll_mesh_points,
            coll_mesh_indices,
            self._kinematic_enabled,
            self._collision_simplification,
            self._collision_simplification_remeshing,
            self._collision_simplification_remeshing_resolution,
            self._collision_simplification_target_triangle_count,
            self._collision_simplification_force_conforming,
            sim_mesh_points,
            sim_mesh_indices,
            sim_mesh_embedding,
            sim_mesh_resolution,
            solver_position_iteration_count = self._solver_position_iteration_count,
            vertex_velocity_damping = self._vertex_velocity_damping,
            sleep_damping = self._sleep_damping,
            sleep_threshold = self._sleep_threshold,
            settling_threshold = self._settling_threshold,
            self_collision = self._self_collision,
            self_collision_filter_distance = self._self_collision_filter_distance
        )

        if not success:
            carb.log_error(type(self).__name__ + ": add_physx_deformable_body failed.")
            self.undo()

        simMeshImageable = UsdGeom.Imageable.Get(self._stage, self._simulation_mesh_path)
        colMeshImageable = UsdGeom.Imageable.Get(self._stage, self._collision_mesh_path)
        if simMeshImageable:
            simMeshImageable.GetVisibilityAttr().Set("invisible")
        if colMeshImageable:
            colMeshImageable.GetVisibilityAttr().Set("invisible")

        return success

    def undo(self):
        self._usd_undo.undo()

    def _is_path_and_mesh_configuration_valid(self) -> bool:
        """
            Checks the input mesh paths and configuration for validity
        """
        if not self._skin_mesh_path:
            carb.log_error(type(self).__name__ + ": Must provide a skin_mesh_path.")
            return False
        self._skin_mesh_path = Sdf.Path(self._skin_mesh_path)

        # check that collision and simulation meshe paths have the correct type:
        if self._collision_mesh_path:
            self._collision_mesh_path = Sdf.Path(self._collision_mesh_path)
            prim = self._stage.GetPrimAtPath(self._collision_mesh_path)
            if not prim:
                carb.log_error(type(self).__name__ + ": Invalid collision mesh path.")
                return False
            if not prim.IsA(PhysxSchema.TetrahedralMesh):
                carb.log_error(type(self).__name__ + ": collision_mesh_path does not point to a PhysxSchema.TetrahedralMesh.")
                return False
            if self._kinematic_enabled:
                carb.log_error(type(self).__name__ + ": collision_mesh_path: custom collision mesh is not supported if kinematic is enabled.")
                return False

            # only need to consider sim mesh if there is a collision mesh:
            if self._simulation_mesh_path:
                self._simulation_mesh_path = Sdf.Path(self._simulation_mesh_path)
                prim = self._stage.GetPrimAtPath(self._simulation_mesh_path)
                if not prim:
                    carb.log_error(type(self).__name__ + ": Invalid simulation mesh path.")
                    return False
                if not prim.IsA(PhysxSchema.TetrahedralMesh):
                    carb.log_error(type(self).__name__ + ": simulation_mesh_path does not point to a PhysxSchema.TetrahedralMesh.")
                    return False
            if self._kinematic_enabled:
                carb.log_error(type(self).__name__ + ": simulation_mesh_path: custom simulation mesh is not supported if kinematic is enabled.")
                return False

        # check that the _skin_mesh_path points to a UsdGeom.Mesh
        prim = self._stage.GetPrimAtPath(self._skin_mesh_path)
        if not prim or not prim.IsA(UsdGeom.Mesh):
            carb.log_error(type(self).__name__ + ": skin_mesh_path does not point to a UsdGeom.Mesh.")
            return False

        return True


# DEPRECATED
class RemoveDeformableBodyComponentCommand(PhysicsCommand):
    """DEPRECATED:"""

    @autoassign
    def __init__(self, prim_path):
        self._usd_context = omni.usd.get_context()

    def do(self):
        stage = self._usd_context.get_stage()
        prim = stage.GetPrimAtPath(self._prim_path)

        execute("UnapplyAPISchema", api=PhysxSchema.PhysxDeformableBodyAPI, prim=prim)
        execute("UnapplyAPISchema", api=PhysxSchema.PhysxCollisionAPI, prim=prim)

        # custom properties
        attributes_to_remove = [x.GetName() for x in prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        for attribute in attributes_to_remove:
            RemoveAttributeCommand.execute(attribute, prim)
            RemoveRelationshipCommand.execute(attribute, prim)

        # custom attributes legacy
        attributes_to_remove = [x.GetName() for x in prim.GetAuthoredPropertiesInNamespace(["physxDeformableBody"])]
        for attribute in attributes_to_remove:
            RemoveAttributeCommand.execute(attribute, prim)

    def undo(self):
        pass


# DEPRECATED
class AddDeformableSurfaceComponentCommand(Command):
    """ DEPRECATED:
        Adds a deformable surface component to a UsdGeom.Mesh.

        Parameters:
            mesh_path:                                      Path to UsdGeom.Mesh to which PhysxDeformableSurfaceAPI is applied and that will be driven by the
                                                            simulation.

            ...:                                            Optional USD schema attributes, please refer to USD schema documentation.
        Returns:
            True / False that indicates success of command execution
    """
    @autoassign
    def __init__(self, mesh_path: Sdf.Path,
                 bending_stiffness_scale: float = None,
                 solver_position_iteration_count: int = None,
                 vertex_velocity_damping: float = None,
                 sleep_damping: float = None,
                 sleep_threshold: float = None,
                 settling_threshold: float = None,
                 self_collision: bool = None,
                 self_collision_filter_distance: float = None):
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        # check paths and mesh input:
        if not self._is_path_and_mesh_configuration_valid():
            return False
        
        # at this point we are guaranteed to have a valid path and mesh configuration and can proceed
        self._usd_undo.reserve(Sdf.Path(self._mesh_path))

        mesh = UsdGeom.Mesh.Get(self._stage, Sdf.Path(self._mesh_path))
        simulation_indices = deformableUtils.triangulate_mesh(mesh)

        success = deformableUtils.add_physx_deformable_surface(
            self._stage,
            prim_path = self._mesh_path,
            simulation_indices = simulation_indices,
            bending_stiffness_scale = self._bending_stiffness_scale,
            solver_position_iteration_count = self._solver_position_iteration_count,
            vertex_velocity_damping = self._vertex_velocity_damping,
            sleep_damping = self._sleep_damping,
            sleep_threshold = self._sleep_threshold,
            settling_threshold = self._settling_threshold,
            self_collision = self._self_collision,
            self_collision_filter_distance = self._self_collision_filter_distance
        )

        if not success:
            carb.log_error(type(self).__name__ + ": add_physx_deformable_surface failed.")
            self.undo()

        return success

    def undo(self):
        self._usd_undo.undo()

    def _is_path_and_mesh_configuration_valid(self) -> bool:
        """
            Checks the input mesh paths and configuration for validity
        """
        if not self._mesh_path:
            carb.log_error(type(self).__name__ + ": Must provide a mesh_path.")
            return False
        self._mesh_path = Sdf.Path(self._mesh_path)

        # check that the _mesh_path points to a UsdGeom.Mesh
        prim = self._stage.GetPrimAtPath(self._mesh_path)
        if not prim or not prim.IsA(UsdGeom.Mesh):
            carb.log_error(type(self).__name__ + ": mesh_path does not point to a UsdGeom.Mesh.")
            return False

        return True


# DEPRECATED
class RemoveDeformableSurfaceComponentCommand(PhysicsCommand):
    """ DEPRECATED:"""

    @autoassign
    def __init__(self, prim_path):
        self._usd_context = omni.usd.get_context()

    def do(self):
        stage = self._usd_context.get_stage()
        prim = stage.GetPrimAtPath(self._prim_path)

        execute("UnapplyAPISchema", api=PhysxSchema.PhysxDeformableSurfaceAPI, prim=prim)
        execute("UnapplyAPISchema", api=PhysxSchema.PhysxCollisionAPI, prim=prim)

        # custom properties
        attributes_to_remove = [x.GetName() for x in prim.GetAuthoredPropertiesInNamespace(["physxDeformable"])]
        for attribute in attributes_to_remove:
            RemoveAttributeCommand.execute(attribute, prim)

        # custom attributes legacy
        attributes_to_remove = [x.GetName() for x in prim.GetAuthoredPropertiesInNamespace(["physxDeformableSurface"])]
        for attribute in attributes_to_remove:
            RemoveAttributeCommand.execute(attribute, prim)

    def undo(self):
        pass


class SetVolumeDeformableBodyCommand(Command):
    """ Setup a volume deformable body based on a simulation UsdGeom.TetMesh.

        Parameters:
            prim_path:                                      Path to UsdGeom.TetMesh to which UsdPhysics.DeformableBodyAPI is
                                                            applied.
        Returns:
            True / False that indicates success of command execution
    """

    def __init__(self, prim_path: Sdf.Path):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False

        self._usd_undo.reserve(self._prim_path)

        success = deformableUtils.set_physics_volume_deformable_body(self._stage, self._prim_path)

        if not success:
            carb.log_error(type(self).__name__ + f": set_physics_volume_deformable_body failed: '{self._prim_path}'.")
            self.undo()

        return success

    def undo(self):
        self._usd_undo.undo()


class CreateAutoVolumeDeformableHierarchyCommand(Command):
    """ Setup a volume deformable body hierarchy based on a UsdGeom.Imageable and suitable prim sub-tree.

        Parameters:
            root_prim_path:                     Path to valid a UsdGeom.Imageable which cannot be a UsdGeom.Gprim.
                                                The UsdPhysics.DeformableBodyAPI is applied to this prim.
            simulation_tetmesh_path:            Path to where simulation mesh should be created or a valid UsdGeom.TetMesh.
            collision_tetmesh_path:             Path to where collision mesh should be created or a valid UsdGeom.TetMesh.
                                                CollisionAPI is applied to the collision mesh.
                                                May be identical to simulation_tetmesh_path.
            cooking_src_mesh_path:              Path to valid UsdGeom.Mesh that is used in cooking to generate the simulation
                                                and collision mesh.
                                                May be outside of root_prim_path sub-hierarchy.
            simulation_hex_mesh_enabled:        If True, simulation mesh is generated as a hexahedral mesh.
            cooking_src_simplification_enabled: If True, PhysxAutoDeformableMeshSimplificationAPI is applied.

        Returns:
            True / False that indicates success of command execution
    """

    @autoassign
    def __init__(self,
        root_prim_path: typing.Union[str, Sdf.Path],
        simulation_tetmesh_path: typing.Union[str, Sdf.Path],
        collision_tetmesh_path: typing.Union[str, Sdf.Path],
        cooking_src_mesh_path : typing.Union[str, Sdf.Path],
        simulation_hex_mesh_enabled: bool,
        cooking_src_simplification_enabled: bool
    ):
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._root_prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False
        if self._simulation_tetmesh_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid simulation_tetmesh_path.")
            return False
        if self._collision_tetmesh_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid collision_tetmesh_path.")
            return False
        if self._cooking_src_mesh_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid cooking_src_mesh_path.")
            return False

        self._usd_undo.reserve(self._root_prim_path)
        success = deformableUtils.create_auto_volume_deformable_hierarchy(self._stage,
            self._root_prim_path,
            self._simulation_tetmesh_path,
            self._collision_tetmesh_path,
            self._cooking_src_mesh_path,
            self._simulation_hex_mesh_enabled,
            self._cooking_src_simplification_enabled,
            set_visibility_with_guide_purpose = True
        )

        if not success:
            carb.log_error(type(self).__name__ + f": create_auto_volume_deformable_hierarchy failed: '{self._root_prim_path}'.")
            self.undo()

        return success

    def undo(self):
        self._usd_undo.undo()


class SetSurfaceDeformableBodyCommand(Command):
    """ Setup a surface deformable body based on a simulation UsdGeom.Mesh.

        Parameters:
            prim_path:                                      Path to UsdGeom.Mesh to which UsdPhysics.DeformableBodyAPI is
                                                            applied.
        Returns:
            True / False that indicates success of command execution
    """

    def __init__(self, prim_path: Sdf.Path):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False

        self._usd_undo.reserve(self._prim_path)

        success = deformableUtils.set_physics_surface_deformable_body(self._stage, self._prim_path)

        if not success:
            carb.log_error(type(self).__name__ + f": set_physics_surface_deformable_body failed: '{self._prim_path}'.")
            self.undo()

        return success

    def undo(self):
        self._usd_undo.undo()


class CreateAutoSurfaceDeformableHierarchyCommand(Command):
    """ Setup a surface deformable body hierarchy based on a UsdGeom.Imageable and suitable prim sub-tree.

        Parameters:
            root_prim_path:                     Path to valid a UsdGeom.Imageable which cannot be a UsdGeom.Gprim.
                                                The UsdPhysics.DeformableBodyAPI is applied to this prim.
            simulation_mesh_path:               Path to where simulation mesh should be created or a valid UsdGeom.Mesh.
                                                CollisionAPI is applied to the simulation mesh.
            cooking_src_mesh_path:              Path to valid UsdGeom.Mesh that is used in cooking to generate the
                                                simulation mesh.
                                                May be outside of root_prim_path sub-hierarchy.
            cooking_src_simplification_enabled: If True, PhysxAutoDeformableMeshSimplificationAPI is applied.

        Returns:
            True / False that indicates success of command execution
    """

    @autoassign
    def __init__(self,
        root_prim_path: typing.Union[str, Sdf.Path],
        simulation_mesh_path: typing.Union[str, Sdf.Path],
        cooking_src_mesh_path : typing.Union[str, Sdf.Path],
        cooking_src_simplification_enabled: bool
    ):
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._root_prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False
        if self._simulation_mesh_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid simulation_mesh_path.")
            return False
        if self._cooking_src_mesh_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid cooking_src_mesh_path.")
            return False

        self._usd_undo.reserve(self._root_prim_path)
        success = deformableUtils.create_auto_surface_deformable_hierarchy(self._stage,
            self._root_prim_path,
            self._simulation_mesh_path,
            self._cooking_src_mesh_path,
            self._cooking_src_simplification_enabled,
            set_visibility_with_guide_purpose = True
        )

        if not success:
            carb.log_error(type(self).__name__ + f": create_auto_surface_deformable_hierarchy failed: '{self._root_prim_path}'.")
            self.undo()

        return success

    def undo(self):
        self._usd_undo.undo()


class RemoveBaseDeformableBodyComponentCommand(PhysicsCommand):

    def __init__(self, prim_path):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False

        self._usd_undo.reserve(self._prim_path)
        deformableUtils.remove_deformable_body(self._stage, self._prim_path)

    def undo(self):
        self._usd_undo.undo()


class RemoveVolumeDeformableSimComponentCommand(PhysicsCommand):

    def __init__(self, prim_path):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False

        prim = self._stage.GetPrimAtPath(self._prim_path)
        self._usd_undo.reserve(self._prim_path)
        prim.RemoveAPI("OmniPhysicsVolumeDeformableSimAPI")
        utils.removeAPISchemaProperties("OmniPhysicsVolumeDeformableSimAPI", prim)

    def undo(self):
        self._usd_undo.undo()


class RemoveSurfaceDeformableSimComponentCommand(PhysicsCommand):

    def __init__(self, prim_path):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False

        prim = self._stage.GetPrimAtPath(self._prim_path)
        self._usd_undo.reserve(self._prim_path)
        prim.RemoveAPI("OmniPhysicsSurfaceDeformableSimAPI")
        utils.removeAPISchemaProperties("OmniPhysicsSurfaceDeformableSimAPI", prim)

    def undo(self):
        self._usd_undo.undo()


class RemoveDeformablePoseComponentCommand(PhysicsCommand):

    @staticmethod
    def _get_schema_instances(prim: Usd.Prim, schema_type_name: str):
        return {s[len(schema_type_name) + 1:] for s in prim.GetAppliedSchemas() if s.startswith(schema_type_name)}

    def __init__(self, prim_path, instance_name):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._instance_name = instance_name
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False
        else:
            prim = self._stage.GetPrimAtPath(self._prim_path)
            self._usd_undo.reserve(self._prim_path)
            if self._instance_name:
                prim.RemoveAPI("OmniPhysicsDeformablePoseAPI", self._instance_name)
                utils.removeMultipleAPISchemaProperties("OmniPhysicsDeformablePoseAPI", prim, "deformablePose", pose_instance_name)
            else:
                #pose_apis = UsdPhysics.DeformablePoseAPI.GetAll(prim)
                #unfortunately we can't access the instance name from a multi applied schema
                #in python...
                pose_instance_names = self._get_schema_instances(prim, "OmniPhysicsDeformablePoseAPI")
                for pose_instance_name in pose_instance_names:
                    prim.RemoveAPI("OmniPhysicsDeformablePoseAPI", pose_instance_name)
                    utils.removeMultipleAPISchemaProperties("OmniPhysicsDeformablePoseAPI", prim, "deformablePose", pose_instance_name)
        return True

    def undo(self):
        self._usd_undo.undo()


class RemoveAutoDeformableBodyCommand(PhysicsCommand):
    @autoassign
    def __init__(self, prim_path):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False
        else:
            self._usd_undo.reserve(self._prim_path)
            deformableUtils.remove_auto_deformable_body(self._stage, self._prim_path)
        return True

    def undo(self):
        self._usd_undo.undo()


class RemoveAutoDeformableHexahedralMeshCommand(PhysicsCommand):

    def __init__(self, prim_path: Sdf.Path):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False
        else:
            self._usd_undo.reserve(self._prim_path)
            deformableUtils.remove_auto_deformable_hexahedral_mesh(self._stage, self._prim_path)
        return True

    def undo(self):
        self._usd_undo.undo()


class AddAutoDeformableMeshSimplificationCommand(PhysicsCommand):
    """ Add an auto mesh simplification component to a prim with PhysxSchema.PhysxAutoDeformableBodyAPI, only supported for UsdGeom.Imageable.
        Parameters:
            prim_path: Path to UsdGeom.Imageable with PhysxSchema.PhysxAutoDeformableBodyAPI.
        Returns:
            True / False that indicates success of command execution
    """
    def __init__(self, prim_path: Sdf.Path):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False
        else:
            self._usd_undo.reserve(self._prim_path)
            success = deformableUtils.add_auto_deformable_mesh_simplification(self._stage, self._prim_path)
            if not success:
                carb.log_error(type(self).__name__ + f": add_auto_deformable_mesh_simplification failed: '{self._prim_path}'.")
                self.undo()
                return False
        return True

    def undo(self):
        self._usd_undo.undo()


class RemoveAutoDeformableMeshSimplificationCommand(PhysicsCommand):

    def __init__(self, prim_path: Sdf.Path):
        self._prim_path = Sdf.Path(prim_path) if prim_path is not None else None
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if self._prim_path is None:
            carb.log_error(type(self).__name__ + ": Must provide a valid prim_path.")
            return False
        else:
            self._usd_undo.reserve(self._prim_path)
            deformableUtils.remove_auto_deformable_mesh_simplification(self._stage, self._prim_path)
        return True

    def undo(self):
        self._usd_undo.undo()


class CreateAutoDeformableAttachmentCommand(Command):

    @autoassign
    def __init__(self, target_attachment_path: Sdf.Path, attachable0_path: Sdf.Path, attachable1_path: Sdf.Path):
        super().__init__()

    def do(self) -> bool:
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        self._usd_undo.reserve(self._target_attachment_path)
        return deformableUtils.create_auto_deformable_attachment(self._stage,
            target_attachment_path=self._target_attachment_path, attachable0_path=self._attachable0_path, attachable1_path=self._attachable1_path)

    def undo(self):
        self._usd_undo.undo()


class ApplyAPISchemaCommand(PhysicsCommand):
    """
    Undoable Apply API command.

    Args:
        api: API class.
        prim: Target primitive.
        api_prefix: Prefix of a multiple-apply API.
        multiple_api_token: Token of a multiple-apply API.
    """
    @autoassign
    def __init__(self, api, prim, api_prefix=None, multiple_api_token=None):
        self._prim_path = prim.GetPath()
        self._usd_context = omni.usd.get_context()

    def do(self):
        if self._multiple_api_token is not None:
            return self._api.Apply(self._prim, self._multiple_api_token)
        else:
            return self._api.Apply(self._prim)

    def undo(self):
        stage = self._usd_context.get_stage()
        prim = stage.GetPrimAtPath(self._prim_path)
        if self._multiple_api_token is not None:
            utils.removeMultipleAPISchemaProperties(self._api, prim, self._api_prefix, self._multiple_api_token)
            prim.RemoveAPI(self._api, self._multiple_api_token)
        else:
            utils.removeAPISchemaProperties(self._api, prim)
            prim.RemoveAPI(self._api)


class ApplyCodelessAPISchemaCommand(PhysicsCommand):
    """
    Undoable Apply Codeless API command.

    Args:
        api: API class.
        prim: Target primitive.
        api_prefix: Prefix of a multiple-apply API.
        multiple_api_token: Token of a multiple-apply API.
    """
    @autoassign
    def __init__(self, api, prim, api_prefix=None, multiple_api_token=None):
        self._prim_path = prim.GetPath()
        self._usd_context = omni.usd.get_context()

    def do(self):
        stage = self._usd_context.get_stage()
        prim = stage.GetPrimAtPath(self._prim_path)
        if self._multiple_api_token is not None:
            return prim.ApplyAPI(self._api, self._multiple_api_token)
        else:
            return prim.ApplyAPI(self._api)

    def undo(self):
        stage = self._usd_context.get_stage()
        prim = stage.GetPrimAtPath(self._prim_path)
        if self._multiple_api_token is not None:
            utils.removeMultipleAPISchemaProperties(self._api, prim, self._api_prefix, self._multiple_api_token)
            prim.RemoveAPI(self._api, self._multiple_api_token)
        else:
            utils.removeAPISchemaProperties(self._api, prim)
            prim.RemoveAPI(self._api)


class UnapplyAPISchemaCommand(PhysicsCommand):
    """
    Undoable Unapply API command.

    Args:
        api: API class.
        prim: Target primitive.
        api_prefix: Prefix of a multiple-apply API.
        multiple_api_token: Token of a multiple-apply API.
    """
    @autoassign
    def __init__(self, api, prim, api_prefix=None, multiple_api_token=None):
        self._schema_exists = False
        self._prim_path = prim.GetPath()
        self._usd_context = omni.usd.get_context()

    def do(self):
        if self._multiple_api_token is not None and self._api_prefix is not None:
            schemaAPI = self._api.Get(self._prim, self._multiple_api_token)
            if schemaAPI:
                self._schema_exists = True
                self._cache = utils.createMultipleAPISchemaPropertyCache(self._api, self._prim, self._api_prefix, self._multiple_api_token)
                utils.removeMultipleAPISchemaProperties(self._api, self._prim, self._api_prefix, self._multiple_api_token)
                return self._prim.RemoveAPI(self._api, self._multiple_api_token)
            return True
        else:
            schemaAPI = self._api.Get(self._prim.GetStage(), self._prim.GetPrimPath())
            if schemaAPI:
                self._schema_exists = True
                self._cache = utils.createAPISchemaPropertyCache(self._api, self._prim)
                utils.removeAPISchemaProperties(self._api, self._prim)
                return self._prim.RemoveAPI(self._api)
            return True

    def undo(self):
        stage = self._usd_context.get_stage()
        prim = stage.GetPrimAtPath(self._prim_path)

        if self._schema_exists:
            if self._multiple_api_token is not None and self._api_prefix is not None:
                api_inst = self._api.Apply(prim, self._multiple_api_token)
                utils.applyAPISchemaPropertyCache(self._cache, api_inst, self._multiple_api_token)
            else:
                api_inst = self._api.Apply(prim)
                utils.applyAPISchemaPropertyCache(self._cache, api_inst)


# M.H. TODO the API cache is not working
class UnapplyCodelessAPISchemaCommand(PhysicsCommand):
    """
    Undoable Unapply API command.

    Args:
        api: API class.
        prim: Target primitive.
        api_prefix: Prefix of a multiple-apply API.
        multiple_api_token: Token of a multiple-apply API.
    """
    @autoassign
    def __init__(self, api, prim, api_prefix=None, multiple_api_token=None):
        self._prim_path = prim.GetPath()
        self._usd_context = omni.usd.get_context()

    def do(self):
        if self._multiple_api_token is not None and self._api_prefix is not None:
            return self._prim.RemoveAPI(self._api, self._multiple_api_token)
        else:
            return self._prim.RemoveAPI(self._api)

    def undo(self):
        stage = self._usd_context.get_stage()
        prim = stage.GetPrimAtPath(self._prim_path)

        if self._multiple_api_token is not None and self._api_prefix is not None:
            prim.ApplyAPI(self._api, self._multiple_api_token)
        else:
            prim.ApplyAPI(self._api)

class RemoveAttributeCommand(PhysicsCommand):
    @autoassign
    def __init__(self, attribute, prim):
        self._val = None

    def do(self):
        attr = self._prim.GetAttribute(self._attribute)
        if attr:
            self._val = attr.Get()
            self._val_type = attr.GetTypeName()
            return self._prim.RemoveProperty(self._attribute)
        else:
            return False

    def undo(self):
        if self._val is not None:
            self._prim.CreateAttribute(self._attribute, self._val_type).Set(self._val)


class RemoveRelationshipCommand(PhysicsCommand):
    @autoassign
    def __init__(self, relationship, prim):
        self._val = None

    def do(self):
        rel = self._prim.GetRelationship(self._relationship)
        if rel:
            self._val = rel.GetTargets()
            return self._prim.RemoveProperty(self._relationship)
        else:
            return False

    def undo(self):
        if self._val is not None:
            self._prim.CreateRelationship(self._relationship).SetTargets(self._val)


class ChangeAttributeCommand(Command):
    """
    Change prim property undoable **Command**.

    Args:
        attr (attribute): Attribute to change.
        value: Value to change to.
        value: Value to undo to.
    """

    def __init__(self, attr: Usd.Attribute, widget: Any, value: Any, prev: Any):
        super().__init__()
        self._value = value
        self._prev = prev
        self._attribute = attr
        self._widget = widget

    def do(self):
        self._attribute.Set(self._value)

    def undo(self):
        self._attribute.Set(self._prev)
        self._widget.value = self._prev


class AddDistancePhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        if self._component == "PhysicsLimit:distance":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="distance")

    def undo(self):
        pass


class RemoveDistancePhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        if self._component == "PhysicsDistanceJoint":
            ret = self._usd_prim.SetTypeName("PhysicsJoint")
            if not ret:
                carb.log_error("Failed to remove distance joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))
        elif "PhysicsLimit" in self._component:
            execute("UnapplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="distance")

    def undo(self):
        if self._component == "PhysicsDistanceJoint":
            ret = self._usd_prim.SetTypeName("PhysicsDistanceJoint")
            if not ret:
                carb.log_error("Failed to put back distance joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))


class AddFixedPhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        pass

    def undo(self):
        pass


class RemoveFixedPhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        if self._component == "PhysicsFixedJoint":
            ret = self._usd_prim.SetTypeName("PhysicsJoint")
            if not ret:
                carb.log_error("Failed to remove fixed joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))

    def undo(self):
        if self._component == "PhysicsFixedJoint":
            ret = self._usd_prim.SetTypeName("PhysicsFixedJoint")
            if not ret:
                carb.log_error("Failed to put back fixed joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))


class AddRevolutePhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        stage = self._usd_prim.GetStage()
        if self._component == "PhysicsLimit:angular":
            revoluteJoint = UsdPhysics.RevoluteJoint.Get(stage, self._usd_prim.GetPrimPath())
            revoluteJoint.CreateLowerLimitAttr().Set(-90)
            revoluteJoint.CreateUpperLimitAttr().Set(90)
            (ret, physxAPI) = execute("ApplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="angular")
            PhysxSchema.PhysxLimitAPI.Apply(self._usd_prim, "angular")
        elif self._component == "PhysicsDrive:angular":
            execute("ApplyAPISchema", api=UsdPhysics.DriveAPI, prim=self._usd_prim, api_prefix="PhysicsDrive", multiple_api_token="angular")
        elif self._component =="PhysxJointAxis:angular":
            execute("ApplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token="angular")
        elif self._component == "PhysicsJointState:angular":
            execute("ApplyAPISchema", api=PhysxSchema.JointStateAPI, prim=self._usd_prim, api_prefix="state", multiple_api_token="angular")

    def undo(self):
        if self._component == "PhysicsLimit:angular":
            self._usd_prim.RemoveProperty("lowerLimit")
            self._usd_prim.RemoveProperty("upperLimit")


class RemoveRevolutePhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        if "PhysicsLimit" in self._component:
            ret = execute("RemoveAttribute", attribute="lowerLimit", prim=self._usd_prim)
            if not ret:
                carb.log_error("Failed to remove attribute a lowerLimit from prim {}".format(self._usd_prim.GetPrimPath().pathString))
            ret = execute("RemoveAttribute", attribute="upperLimit", prim=self._usd_prim)
            if not ret:
                carb.log_error("Failed to remove attribute a upperLimit from prim {}".format(self._usd_prim.GetPrimPath().pathString))
            ret = execute("UnapplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="angular")
            if not ret:
                carb.log_error("Failed to remove a PhysxSchema.PhysxLimitAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        elif "PhysicsDrive" in self._component:
            ret = execute("UnapplyAPISchema", api=UsdPhysics.DriveAPI, prim=self._usd_prim, api_prefix="PhysicsDrive", multiple_api_token="angular")
            if not ret:
                carb.log_error("Failed to remove a UsdPhysics.DriveAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        elif "PhysxJointAxis" in self._component:
            ret = execute("UnapplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token="angular")
            if not ret:
                carb.log_error("Failed to remove a PhysxJointAxisAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        elif "JointState" in self._component:
            ret = execute("UnapplyAPISchema", api=PhysxSchema.JointStateAPI, prim=self._usd_prim, api_prefix="state", multiple_api_token="angular")
            if not ret:
                carb.log_error("Failed to remove a PhysxSchema.JointStateAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        elif self._component == "PhysicsRevoluteJoint":
            ret = self._usd_prim.SetTypeName("PhysicsJoint")
            if not ret:
                carb.log_error("Failed to remove distance joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))
            else:
                execute("RemoveAttribute", attribute="axis", prim=self._usd_prim)
                execute("RemoveAttribute", attribute="lowerLimit", prim=self._usd_prim)
                execute("RemoveAttribute", attribute="upperLimit", prim=self._usd_prim)
                execute("UnapplyAPISchema", api=UsdPhysics.DriveAPI, prim=self._usd_prim, api_prefix="PhysicsDrive", multiple_api_token="angular")
                execute("UnapplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token="angular")
                execute("UnapplyAPISchema", api=PhysxSchema.JointStateAPI, prim=self._usd_prim, api_prefix="state", multiple_api_token="angular")
                execute("UnapplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="angular")

    def undo(self):
        if self._component == "PhysicsRevoluteJoint":
            ret = self._usd_prim.SetTypeName("PhysicsRevoluteJoint")
            if not ret:
                carb.log_error("Failed to put back revolute joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))


class AddPrismaticPhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        stage = self._usd_prim.GetStage()
        if self._component == "PhysicsLimit:linear":
            prismaticJoint = UsdPhysics.PrismaticJoint.Get(stage, self._usd_prim.GetPrimPath())
            prismaticJoint.CreateLowerLimitAttr().Set(-1.0)
            prismaticJoint.CreateUpperLimitAttr().Set(1.0)
            execute("ApplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="linear")
            PhysxSchema.PhysxLimitAPI.Apply(self._usd_prim, "linear")
        elif self._component == "PhysicsDrive:linear":
            execute("ApplyAPISchema", api=UsdPhysics.DriveAPI, prim=self._usd_prim, api_prefix="PhysicsDrive", multiple_api_token="linear")
        elif self._component =="PhysxJointAxis:linear":
            execute("ApplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token="linear")
        elif self._component == "PhysicsJointState:linear":
            execute("ApplyAPISchema", api=PhysxSchema.JointStateAPI, prim=self._usd_prim, api_prefix="state", multiple_api_token="linear")

    def undo(self):
        if self._component == "PhysicsLimit:linear":
            self._usd_prim.RemoveProperty("lowerLimit")
            self._usd_prim.RemoveProperty("upperLimit")


class RemovePrismaticPhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        if "PhysicsLimit" in self._component:
            ret = execute("RemoveAttribute", attribute="lowerLimit", prim=self._usd_prim)
            if not ret:
                carb.log_error("Failed to remove attribute a lowerLimit from prim {}".format(self._usd_prim.GetPrimPath().pathString))
            ret = execute("RemoveAttribute", attribute="upperLimit", prim=self._usd_prim)
            if not ret:
                carb.log_error("Failed to remove attribute a upperLimit from prim {}".format(self._usd_prim.GetPrimPath().pathString))
            ret = execute("UnapplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="linear")
            if not ret:
                carb.log_error("Failed to remove a PhysxSchema.PhysxLimitAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        elif "PhysicsDrive" in self._component:
            ret = execute("UnapplyAPISchema", api=UsdPhysics.DriveAPI, prim=self._usd_prim, api_prefix="PhysicsDrive", multiple_api_token="linear")
            if not ret:
                carb.log_error("Failed to remove a UsdPhysics.DriveAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        elif "PhysxJointAxis" in self._component:
            ret = execute("UnapplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token="linear")
            if not ret:
                carb.log_error("Failed to remove a PhysxJointAxisAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        elif "JointState" in self._component:
            ret = execute("UnapplyAPISchema", api=PhysxSchema.JointStateAPI, prim=self._usd_prim, api_prefix="state", multiple_api_token="linear")
            if not ret:
                carb.log_error("Failed to remove a PhysxSchema.JointStateAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        elif self._component == "PhysicsPrismaticJoint":
            ret = self._usd_prim.SetTypeName("PhysicsJoint")
            if not ret:
                carb.log_error("Failed to remove prismatic joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))
            else:
                execute("RemoveAttribute", attribute="axis", prim=self._usd_prim)
                execute("RemoveAttribute", attribute="lowerLimit", prim=self._usd_prim)
                execute("RemoveAttribute", attribute="upperLimit", prim=self._usd_prim)
                execute("UnapplyAPISchema", api=UsdPhysics.DriveAPI, prim=self._usd_prim, api_prefix="PhysicsDrive", multiple_api_token="linear")
                execute("UnapplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token="linear")
                execute("UnapplyAPISchema", api=PhysxSchema.JointStateAPI, prim=self._usd_prim, api_prefix="state", multiple_api_token="linear")
                execute("UnapplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="linear")

    def undo(self):
        if self._component == "PhysicsPrismaticJoint":
            ret = self._usd_prim.SetTypeName("PhysicsPrismaticJoint")
            if not ret:
                carb.log_error("Failed to put back prismatic joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))


class AddSphericalPhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        stage = self._usd_prim.GetStage()
        if self._component == "PhysicsLimit":
            sphericalJoint = UsdPhysics.SphericalJoint.Get(stage, self._usd_prim.GetPrimPath())
            sphericalJoint.CreateConeAngle0LimitAttr().Set(1.57)
            sphericalJoint.CreateConeAngle1LimitAttr().Set(1.57)
            (ret, physxAPI) = execute("ApplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="cone")
            PhysxSchema.PhysxLimitAPI.Apply(self._usd_prim, "cone")
        elif "PhysxJointAxis" in self._component:
            axis = self._component.split(":")[1]
            execute("ApplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token=axis)
    def undo(self):
        if self._component == "PhysicsLimit":
            self._usd_prim.RemoveProperty("coneAngle0Limit")
            self._usd_prim.RemoveProperty("coneAngle1Limit")


class RemoveSphericalPhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        if self._component == "PhysicsLimit":
            ret = execute("RemoveAttribute", attribute="coneAngle0Limit", prim=self._usd_prim)
            if not ret:
                carb.log_error("Failed to remove attribute a coneAngle0Limit from prim {}".format(self._usd_prim.GetPrimPath().pathString))
            ret = execute("RemoveAttribute", attribute="coneAngle1Limit", prim=self._usd_prim)
            if not ret:
                carb.log_error("Failed to remove attribute a coneAngle1Limit from prim {}".format(self._usd_prim.GetPrimPath().pathString))
            ret = execute("UnapplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="cone")
            if not ret:
                carb.log_error("Failed to remove a PhysxSchema.PhysxLimitAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        if "PhysxJointAxis" in self._component:
            split = self._component.split(":")
            tokens = jointAxisInstanceTokens if len(split) == 1 else [split[1]]
            for token in tokens:
                ret = execute("UnapplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token=token)
                if not ret:
                    carb.log_error("Failed to remove a PhysxJointAxisAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
       
        elif self._component == "PhysicsSphericalJoint":
            ret = self._usd_prim.SetTypeName("PhysicsJoint")
            if not ret:
                carb.log_error("Failed to remove spherical joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))
            else:
                execute("RemoveAttribute", attribute="axis", prim=self._usd_prim)
                execute("RemoveAttribute", attribute="coneAngle0Limit", prim=self._usd_prim)
                execute("RemoveAttribute", attribute="coneAngle1Limit", prim=self._usd_prim)
                execute("UnapplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token="cone")
                for token in jointAxisInstanceTokens:
                     execute("UnapplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token=token)


    def undo(self):
        if self._component == "PhysicsSphericalJoint":
            ret = self._usd_prim.SetTypeName("PhysicsSphericalJoint")
            if not ret:
                carb.log_error("Failed to put back spherical joint prim type {}".format(self._usd_prim.GetPrimPath().pathString))


class RemoveTendonComponentsCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def _get_schema_instances(self, schema_type_name: str):
        return {s[len(schema_type_name) + 1:] for s in self._usd_prim.GetAppliedSchemas() if s.startswith(schema_type_name)}

    def do(self):
        if self._component == "PhysxTendonAttachmentAPI":
            instances = self._get_schema_instances("PhysxTendonAttachmentAPI")
            for instance in instances:
                ret = execute("UnapplyAPISchema", api=PhysxSchema.PhysxTendonAttachmentAPI, prim=self._usd_prim, api_prefix="physxTendon", multiple_api_token=instance)
                if not ret:
                    carb.log_error("Failed to remove attribute {} from prim {}".format(instance, self._usd_prim.GetPrimPath().pathString))
        elif self._component == "PhysxTendonAttachmentRootAPI":
            instances = self._get_schema_instances("PhysxTendonAttachmentRootAPI")
            for instance in instances:
                ret = execute("UnapplyAPISchema", api=PhysxSchema.PhysxTendonAttachmentRootAPI, prim=self._usd_prim, api_prefix="physxTendon", multiple_api_token=instance)
                if not ret:
                    carb.log_error("Failed to remove attribute {} from prim {}".format(instance, self._usd_prim.GetPrimPath().pathString))
        elif self._component == "PhysxTendonAttachmentLeafAPI":
            instances = self._get_schema_instances("PhysxTendonAttachmentLeafAPI")
            for instance in instances:
                ret = execute("UnapplyAPISchema", api=PhysxSchema.PhysxTendonAttachmentLeafAPI, prim=self._usd_prim, api_prefix="physxTendon", multiple_api_token=instance)
                if not ret:
                    carb.log_error("Failed to remove attribute {} from prim {}".format(instance, self._usd_prim.GetPrimPath().pathString))
        elif self._component == "PhysxTendonAxisAPI":
            instances = self._get_schema_instances("PhysxTendonAxisAPI")
            for instance in instances:
                ret = execute("UnapplyAPISchema", api=PhysxSchema.PhysxTendonAxisAPI, prim=self._usd_prim, api_prefix="physxTendon", multiple_api_token=instance)
                if not ret:
                    carb.log_error("Failed to remove attribute {} from prim {}".format(instance, self._usd_prim.GetPrimPath().pathString))
        elif self._component == "PhysxTendonAxisRootAPI":
            instances = self._get_schema_instances("PhysxTendonAxisRootAPI")
            for instance in instances:
                ret = execute("UnapplyAPISchema", api=PhysxSchema.PhysxTendonAxisRootAPI, prim=self._usd_prim, api_prefix="physxTendon", multiple_api_token=instance)
                if not ret:
                    carb.log_error("Failed to remove attribute {} from prim {}".format(instance, self._usd_prim.GetPrimPath().pathString))

    def undo(self):
        pass


driveInstanceTokens = ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]
limitInstanceTokens = ["transX", "transY", "transZ", "rotX", "rotY", "rotZ", "distance"]
jointAxisInstanceTokens = ["rotX", "rotY", "rotZ"]

class AddD6PhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def clear_d6_components(self):
        for token in limitInstanceTokens:
            execute("UnapplyAPISchema", api=UsdPhysics.LimitAPI, prim=self._usd_prim, api_prefix="PhysicsLimit", multiple_api_token=token)
            execute("UnapplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token=token)
        for token in driveInstanceTokens:
            execute("UnapplyAPISchema", api=UsdPhysics.DriveAPI, prim=self._usd_prim, api_prefix="PhysicsDrive", multiple_api_token=token)
        for token in jointAxisInstanceTokens:
            execute("UnapplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token=token)

    def do(self):
        stage = self._usd_prim.GetStage()
        if "PhysicsLimit" in self._component:
            axis = self._component.split(":")[1]
            (ret, limitAPI) = execute("ApplyAPISchema", api=UsdPhysics.LimitAPI, prim=self._usd_prim, api_prefix="PhysicsLimit", multiple_api_token=axis)
            if "trans" in axis:
                limitAPI.CreateLowAttr(1.0)
                limitAPI.CreateHighAttr(-1.0)
            elif "distance" in axis:
                limitAPI.CreateHighAttr(1.0)
            else:
                limitAPI.CreateLowAttr(1.0)
                limitAPI.CreateHighAttr(-1.0)
            execute("ApplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token=axis)
            PhysxSchema.PhysxLimitAPI.Apply(self._usd_prim, axis)
        elif "PhysicsDrive" in self._component:
            axis = self._component.split(":")[1]
            execute("ApplyAPISchema", api=UsdPhysics.DriveAPI, prim=self._usd_prim, api_prefix="PhysicsDrive", multiple_api_token=axis)
        elif "PhysxJointAxis" in self._component:
            axis = self._component.split(":")[1]
            execute("ApplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token=axis)
        elif "PhysicsJointState" in self._component:
            axis = self._component.split(":")[1]
            execute("ApplyAPISchema", api=PhysxSchema.JointStateAPI, prim=self._usd_prim, api_prefix="state", multiple_api_token=axis)
        elif self._component == "PhysicsRevoluteJoint":
            self.clear_d6_components()
            self._usd_prim.SetTypeName("PhysicsRevoluteJoint")
            UsdPhysics.RevoluteJoint.Get(stage, self._usd_prim.GetPrimPath())
        elif self._component == "PhysicsPrismaticJoint":
            self.clear_d6_components()
            self._usd_prim.SetTypeName("PhysicsPrismaticJoint")
            UsdPhysics.PrismaticJoint.Get(stage, self._usd_prim.GetPrimPath())
        elif self._component == "PhysicsSphericalJoint":
            self.clear_d6_components()
            self._usd_prim.SetTypeName("PhysicsSphericalJoint")
            UsdPhysics.SphericalJoint.Get(stage, self._usd_prim.GetPrimPath())
        elif self._component == "PhysicsDistanceJoint":
            self.clear_d6_components()
            self._usd_prim.SetTypeName("PhysicsDistanceJoint")
        elif self._component == "PhysicsFixedJoint":
            self.clear_d6_components()
            self._usd_prim.SetTypeName("PhysicsFixedJoint")

    def undo(self):
        if self._component == "PhysicsRevoluteJoint":
            self._usd_prim.SetTypeName("PhysicsJoint")
        elif self._component == "PhysicsPrismaticJoint":
            self._usd_prim.SetTypeName("PhysicsJoint")
        elif self._component == "PhysicsSphericalJoint":
            self._usd_prim.SetTypeName("PhysicsJoint")
        elif self._component == "PhysicsDistanceJoint":
            self._usd_prim.SetTypeName("PhysicsJoint")
        elif self._component == "PhysicsFixedJoint":
            self._usd_prim.SetTypeName("PhysicsJoint")


class RemoveD6PhysicsJointComponentCommand(Command):
    def __init__(self, usd_prim: Usd.Prim, component: str):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component

    def do(self):
        if "PhysicsLimit" in self._component:
            split = self._component.split(":")
            tokens = limitInstanceTokens if len(split) == 1 else [split[1]]
            for token in tokens:
                ret = execute("UnapplyAPISchema", api=UsdPhysics.LimitAPI, prim=self._usd_prim, api_prefix="PhysicsLimit", multiple_api_token=token)
                if not ret:
                    carb.log_error("Failed to remove a UsdPhysics.LimitAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
                ret = execute("UnapplyAPISchema", api=PhysxSchema.PhysxLimitAPI, prim=self._usd_prim, api_prefix="physxLimit", multiple_api_token=token)
                if not ret:
                    carb.log_error("Failed to remove a PhysxSchema.PhysxLimitAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        if "PhysicsDrive" in self._component:
            split = self._component.split(":")
            tokens = driveInstanceTokens if len(split) == 1 else [split[1]]
            for token in tokens:
                ret = execute("UnapplyAPISchema", api=UsdPhysics.DriveAPI, prim=self._usd_prim, api_prefix="PhysicsDrive", multiple_api_token=token)
                if not ret:
                    carb.log_error("Failed to remove a UsdPhysics.DriveAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        if "PhysxJointAxis" in self._component:
            split = self._component.split(":")
            tokens = jointAxisInstanceTokens if len(split) == 1 else [split[1]]
            for token in tokens:
                ret = execute("UnapplyCodelessAPISchema", api="PhysxJointAxisAPI", prim=self._usd_prim, api_prefix="physxJointAxis", multiple_api_token=token)
                if not ret:
                    carb.log_error("Failed to remove a PhysxJointAxisAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        if "JointState" in self._component:
            split = self._component.split(":")
            tokens = driveInstanceTokens if len(split) == 1 else [split[1]]
            for token in tokens:
                ret = execute("UnapplyAPISchema", api=PhysxSchema.JointStateAPI, prim=self._usd_prim, api_prefix="state", multiple_api_token=token)
                if not ret:
                    carb.log_error("Failed to remove a PhysxSchema.JointStateAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))
        elif self._component == "ArticulationJoint":
            ret = execute("UnapplyAPISchema", api=UsdPhysics.ArticulationJointAPI, prim=self._usd_prim)
            if not ret:
                carb.log_error("Failed to remove a UsdPhysics.ArticulationJointAPI from prim {}".format(self._usd_prim.GetPrimPath().pathString))

    def undo(self):
        pass


class AddPhysicsComponentCommand(Command):
    """
    Add physics component command. See omni.physx.commands source for currently valid component names.

    Args:
        usd_prim: USD prim to apply a component to.
        str: Component name.
        multiple_api_token: Component instance name (if applicable).
    """

    def __init__(self, usd_prim: Usd.Prim, component: str, multiple_api_token: str = None):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component
        self._multiple_api_token = multiple_api_token
        self._loc_commands = []

    def do(self):
        usdPath = self._usd_prim.GetPath()
        stage = self._usd_prim.GetStage()

        if self._component == "PhysicsRigidBodyAPI":
            utils.setPhysics(self._usd_prim, False, partial(local_execute_apply, self._loc_commands))
        elif self._component == "PhysicsCollisionAPI":
            utils.setCollider(self._usd_prim, custom_execute_fn=partial(local_execute_apply, self._loc_commands))
        elif self._component == "PhysicsArticulationRootAPI":
            execute("ApplyAPISchema", api=UsdPhysics.ArticulationRootAPI, prim=self._usd_prim)
            execute("ApplyAPISchema", api=PhysxSchema.PhysxArticulationAPI, prim=self._usd_prim)
        elif self._component == "PhysxCharacterControllerAPI":
            (ret, cctAPI) = execute("ApplyAPISchema", api=PhysxSchema.PhysxCharacterControllerAPI, prim=self._usd_prim)
            slopeLimitDefault = 0.707
            cctAPI.CreateSlopeLimitAttr(slopeLimitDefault)
        elif self._component == "ContactReportAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxContactReportAPI, prim=self._usd_prim)
        elif self._component == "SurfaceVelocityAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxSurfaceVelocityAPI, prim=self._usd_prim)
        elif self._component == "PhysxSplinesSurfaceVelocityAPI":
            execute("ApplyCodelessAPISchemaCommand", api="PhysxSplinesSurfaceVelocityAPI", prim=self._usd_prim)
        elif self._component == "PhysicsFilteredPairsAPI":
            execute("ApplyAPISchema", api=UsdPhysics.FilteredPairsAPI, prim=self._usd_prim)
        elif self._component == "TriggerAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxTriggerAPI, prim=self._usd_prim)
        elif self._component == "MeshMergeCollisionAPI":
            execute("ApplyAPISchema", api=UsdPhysics.CollisionAPI, prim=self._usd_prim)
            execute("ApplyAPISchema", api=UsdPhysics.MeshCollisionAPI, prim=self._usd_prim)
            execute("ApplyAPISchema", api=PhysxSchema.PhysxMeshMergeCollisionAPI, prim=self._usd_prim)
        elif self._component == "TriggerStateAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxTriggerStateAPI, prim=self._usd_prim)
        elif self._component == "ForceAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxForceAPI, prim=self._usd_prim)
        elif self._component == "PhysicsMaterialAPI":
            execute("ApplyAPISchema", api=UsdPhysics.MaterialAPI, prim=self._usd_prim)
        # DEPRECATED
        elif self._component == "PhysxDeformableBodyMaterialAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxDeformableBodyMaterialAPI, prim=self._usd_prim)
        elif self._component == "PhysxDeformableSurfaceMaterialAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxDeformableSurfaceMaterialAPI, prim=self._usd_prim)
        #~DEPRECATED
        elif self._component == "OmniPhysicsDeformableMaterialAPI":
            execute("ApplyCodelessAPISchema", api="OmniPhysicsDeformableMaterialAPI", prim=self._usd_prim)
        elif self._component == "OmniPhysicsSurfaceDeformableMaterialAPI":
            execute("ApplyCodelessAPISchema", api="OmniPhysicsSurfaceDeformableMaterialAPI", prim=self._usd_prim)
        elif self._component == "QuasistaticAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxSceneQuasistaticAPI, prim=self._usd_prim)
        elif self._component == "PhysxResidualReportingAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxResidualReportingAPI, prim=self._usd_prim)
        elif self._component == "PhysicsMassAPI":
            execute("ApplyAPISchema", api=UsdPhysics.MassAPI, prim=self._usd_prim)
        elif self._component == "PhysxTendonAttachmentAPI" and self._multiple_api_token is not None:
            execute("ApplyAPISchema", api=PhysxSchema.PhysxTendonAttachmentAPI, prim=self._usd_prim,
                    api_prefix="physxTendon", multiple_api_token=self._multiple_api_token)
        elif self._component == "PhysxTendonAttachmentRootAPI" and self._multiple_api_token is not None:
            execute("ApplyAPISchema", api=PhysxSchema.PhysxTendonAttachmentRootAPI, prim=self._usd_prim,
                    api_prefix="physxTendon", multiple_api_token=self._multiple_api_token)
        elif self._component == "PhysxTendonAttachmentLeafAPI" and self._multiple_api_token is not None:
            execute("ApplyAPISchema", api=PhysxSchema.PhysxTendonAttachmentLeafAPI, prim=self._usd_prim,
                    api_prefix="physxTendon", multiple_api_token=self._multiple_api_token)
        elif self._component == "PhysxTendonAxisAPI" and self._multiple_api_token is not None:
            execute("ApplyAPISchema", api=PhysxSchema.PhysxTendonAxisAPI, prim=self._usd_prim,
                    api_prefix="physxTendon", multiple_api_token=self._multiple_api_token)
        elif self._component == "PhysxTendonAxisRootAPI" and self._multiple_api_token is not None:
            execute("ApplyAPISchema", api=PhysxSchema.PhysxTendonAxisRootAPI, prim=self._usd_prim,
                    api_prefix="physxTendon", multiple_api_token=self._multiple_api_token)
        # DEPRECATED
        elif self._component == "PhysxDeformableBodyAPI":
            execute("AddDeformableBodyComponent", skin_mesh_path=self._usd_prim.GetPath())
        elif self._component == "PhysxDeformableSurfaceAPI":
            execute("AddDeformableSurfaceComponent", mesh_path=self._usd_prim.GetPath())
        #~DEPRECATED
        elif self._component == "PhysxMimicJointAPI:rotX":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxMimicJointAPI, prim=self._usd_prim,
                    api_prefix=PhysxSchema.Tokens.physxMimicJoint, multiple_api_token=UsdPhysics.Tokens.rotX)
        elif self._component == "PhysxMimicJointAPI:rotY":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxMimicJointAPI, prim=self._usd_prim,
                    api_prefix=PhysxSchema.Tokens.physxMimicJoint, multiple_api_token=UsdPhysics.Tokens.rotY)
        elif self._component == "PhysxMimicJointAPI:rotZ":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxMimicJointAPI, prim=self._usd_prim,
                    api_prefix=PhysxSchema.Tokens.physxMimicJoint, multiple_api_token=UsdPhysics.Tokens.rotZ)
        elif self._component == "PhysxAutoDeformableMeshSimplificationAPI":
            execute("AddAutoDeformableMeshSimplification", prim_path=self._usd_prim.GetPath())
        elif UsdPhysics.DistanceJoint.Get(stage, usdPath):
            execute("AddDistancePhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.FixedJoint.Get(stage, usdPath):
            execute("AddFixedPhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.RevoluteJoint.Get(stage, usdPath):
            execute("AddRevolutePhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.PrismaticJoint.Get(stage, usdPath):
            execute("AddPrismaticPhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.SphericalJoint.Get(stage, usdPath):
            execute("AddSphericalPhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.Joint.Get(stage, usdPath):
            execute("AddD6PhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif self._component == "PhysxVehicleContextAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleContextAPI, prim=self._usd_prim)
        elif self._component == "PhysxVehicleSuspensionComplianceAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleSuspensionComplianceAPI, prim=self._usd_prim)
        elif self._component == "PhysxVehicleSteeringAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleSteeringAPI, prim=self._usd_prim)
        elif self._component == "PhysxVehicleAckermannSteeringAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleAckermannSteeringAPI, prim=self._usd_prim)
        elif self._component == "PhysxVehicleBrakesAPI:brakes0":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleBrakesAPI, prim=self._usd_prim,
                    api_prefix=PhysxSchema.Tokens.physxVehicleBrakes, multiple_api_token=PhysxSchema.Tokens.brakes0)
        elif self._component == "PhysxVehicleBrakesAPI:brakes1":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleBrakesAPI, prim=self._usd_prim,
                    api_prefix=PhysxSchema.Tokens.physxVehicleBrakes, multiple_api_token=PhysxSchema.Tokens.brakes1)
        elif self._component == "PhysxVehicleNonlinearCommandResponseAPI:drive":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI, prim=self._usd_prim,
                    api_prefix=PhysxSchema.Tokens.physxVehicleNCR, multiple_api_token=PhysxSchema.Tokens.drive)
        elif self._component == "PhysxVehicleNonlinearCommandResponseAPI:steer":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI, prim=self._usd_prim,
                    api_prefix=PhysxSchema.Tokens.physxVehicleNCR, multiple_api_token=PhysxSchema.Tokens.steer)
        elif self._component == "PhysxVehicleNonlinearCommandResponseAPI:brakes0":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI, prim=self._usd_prim,
                    api_prefix=PhysxSchema.Tokens.physxVehicleNCR, multiple_api_token=PhysxSchema.Tokens.brakes0)
        elif self._component == "PhysxVehicleNonlinearCommandResponseAPI:brakes1":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI, prim=self._usd_prim,
                    api_prefix=PhysxSchema.Tokens.physxVehicleNCR, multiple_api_token=PhysxSchema.Tokens.brakes1)
        # DEPRECATED
        elif self._component == "PhysxAutoAttachmentAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxAutoAttachmentAPI, prim=self._usd_prim)
        #~DEPRECATED
        elif self._component == "PhysxParticleSamplingAPI":
            execute("AddParticleSamplingCommand", prim=self._usd_prim)
        elif self._component == "PhysxParticleSetAPI":
            execute("AddParticleSetCommand", prim=self._usd_prim)
        # DEPRECATED
        elif self._component == "PhysxParticleClothAPI":
            execute("AddParticleClothComponentCommand", prim_path=self._usd_prim.GetPath())
        elif self._component == "PhysxAutoParticleClothAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxAutoParticleClothAPI, prim=self._usd_prim)
        #~DEPRECATED
        elif self._component == "PhysxParticleAnisotropyAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxParticleAnisotropyAPI, prim=self._usd_prim)
        elif self._component == "PhysxParticleSmoothingAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxParticleSmoothingAPI, prim=self._usd_prim)
        elif self._component == "PhysxParticleIsosurfaceAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxParticleIsosurfaceAPI, prim=self._usd_prim)
        elif self._component == "PhysxDiffuseParticlesAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxDiffuseParticlesAPI, prim=self._usd_prim)
        elif self._component == "PhysxPBDMaterialAPI":
            execute("ApplyAPISchema", api=PhysxSchema.PhysxPBDMaterialAPI, prim=self._usd_prim)


    def undo(self):
        for sc in self._loc_commands:
            sc.undo()


class RemovePhysicsComponentCommand(Command):
    """
    Remove physics component. See omni.physx.commands source for currently valid components.

    Args:
        usd_prim: USD prim to apply a component to.
        str: Component name.
        multiple_api_token: Component instance name (if applicable).
    """

    def __init__(self, usd_prim: Usd.Prim, component: str, multiple_api_token: str = None):
        super().__init__()
        self._usd_prim = usd_prim
        self._component = component
        self._multiple_api_token = multiple_api_token
        self._loc_commands = []

    def do(self):
        usdPath = self._usd_prim.GetPath()
        stage = self._usd_prim.GetStage()

        def remove_api(api, prim=self._usd_prim, api_prefix: str = None, multiple_api_token: str = None):
            ret = execute("UnapplyAPISchema", api=api, prim=prim, api_prefix=api_prefix, multiple_api_token=multiple_api_token)
            if not ret:
                carb.log_error(f"Failed to remove {api.__name__} from prim {prim.GetPrimPath().pathString}")

        def remove_codeless_api(api, prim=self._usd_prim, api_prefix: str = None, multiple_api_token: str = None):
            ret = execute("UnapplyCodelessAPISchema", api=api, prim=prim, api_prefix=api_prefix, multiple_api_token=multiple_api_token)
            if not ret:
                carb.log_error(f"Failed to remove {api.__name__} from prim {prim.GetPrimPath().pathString}")

        if self._component == "PhysicsRigidBodyAPI":
            utils.removePhysics(self._usd_prim, partial(local_execute_unapply, self._loc_commands))
        elif self._component == "PhysicsCollisionAPI":
            utils.removeCollider(self._usd_prim, partial(local_execute_unapply, self._loc_commands))
            # custom properties
            RemoveAttributeCommand.execute(PhysxSchema.Tokens.physxCollisionCustomGeometry, self._usd_prim)
        elif self._component == "PhysicsArticulationRootAPI":
            remove_api(UsdPhysics.ArticulationRootAPI)
            remove_api(PhysxSchema.PhysxArticulationAPI)
        elif self._component == "PhysxCharacterControllerAPI":
            remove_api(PhysxSchema.PhysxCharacterControllerAPI)
        elif self._component == "ContactReportAPI":
            remove_api(PhysxSchema.PhysxContactReportAPI)
        elif self._component == "SurfaceVelocityAPI":
            remove_api(PhysxSchema.PhysxSurfaceVelocityAPI)
        elif self._component == "PhysxSplinesSurfaceVelocityAPI":            
            remove_codeless_api("PhysxSplinesSurfaceVelocityAPI", self._usd_prim)
        elif self._component == "PhysicsFilteredPairsAPI":
            remove_api(UsdPhysics.FilteredPairsAPI, self._usd_prim)
        elif self._component == "TriggerAPI":
            remove_api(PhysxSchema.PhysxTriggerAPI, self._usd_prim)
        elif self._component == "MeshMergeCollisionAPI":
            remove_api(PhysxSchema.PhysxMeshMergeCollisionAPI, self._usd_prim)
            remove_api(UsdPhysics.CollisionAPI, self._usd_prim)
            remove_api(UsdPhysics.MeshCollisionAPI, self._usd_prim)
        elif self._component == "TriggerStateAPI":
            remove_api(PhysxSchema.PhysxTriggerStateAPI, self._usd_prim)
        elif self._component == "ForceAPI":
            remove_api(PhysxSchema.PhysxForceAPI, self._usd_prim)
        elif self._component == "PhysicsMaterialAPI":
            remove_api(UsdPhysics.MaterialAPI, self._usd_prim)
        # DEPRECATED
        elif self._component == "PhysxDeformableBodyMaterialAPI":
            remove_api(PhysxSchema.PhysxDeformableBodyMaterialAPI, self._usd_prim)
        elif self._component == "PhysxDeformableSurfaceMaterialAPI":
            remove_api(PhysxSchema.PhysxDeformableSurfaceMaterialAPI, self._usd_prim)
        #~DEPRECATED
        elif self._component == "OmniPhysicsDeformableMaterialAPI":
            remove_api(UsdPhysics.MaterialAPI, self._usd_prim)
            remove_codeless_api("OmniPhysicsDeformableMaterialAPI", self._usd_prim)
            remove_codeless_api("OmniPhysicsSurfaceDeformableMaterialAPI", self._usd_prim)
        elif self._component == "OmniPhysicsSurfaceDeformableMaterialAPI":
            remove_codeless_api("OmniPhysicsSurfaceDeformableMaterialAPI", self._usd_prim)
        elif self._component == "QuasistaticAPI":
            remove_api(PhysxSchema.PhysxSceneQuasistaticAPI, self._usd_prim)
        elif self._component == "PhysxResidualReportingAPI":
            remove_api(PhysxSchema.PhysxResidualReportingAPI, self._usd_prim)
        elif self._component == "PhysxTendonAttachmentAPI":
            if self._multiple_api_token is not None:
                remove_api(PhysxSchema.PhysxTendonAttachmentAPI, self._usd_prim, "physxTendon", self._multiple_api_token)
            else:
                execute("RemoveTendonComponents", component=self._component, usd_prim=self._usd_prim)
        elif self._component == "PhysxTendonAttachmentRootAPI":
            if self._multiple_api_token is not None:
                remove_api(PhysxSchema.PhysxTendonAttachmentRootAPI, self._usd_prim, "physxTendon", self._multiple_api_token)
            else:
                execute("RemoveTendonComponents", component=self._component, usd_prim=self._usd_prim)
        elif self._component == "PhysxTendonAttachmentLeafAPI":
            if self._multiple_api_token is not None:
                remove_api(PhysxSchema.PhysxTendonAttachmentLeafAPI, self._usd_prim, "physxTendon", self._multiple_api_token)
            else:
                execute("RemoveTendonComponents", component=self._component, usd_prim=self._usd_prim)
        elif self._component == "PhysxTendonAxisAPI":
            if self._multiple_api_token is not None:
                remove_api(PhysxSchema.PhysxTendonAxisAPI, self._usd_prim, "physxTendon", self._multiple_api_token)
            else:
                execute("RemoveTendonComponents", component=self._component, usd_prim=self._usd_prim)
        elif self._component == "PhysxTendonAxisRootAPI":
            if self._multiple_api_token is not None:
                remove_api(PhysxSchema.PhysxTendonAxisRootAPI, self._usd_prim, "physxTendon", self._multiple_api_token)
            else:
                execute("RemoveTendonComponents", component=self._component, usd_prim=self._usd_prim)
        # DEPRECATED
        elif self._component == "PhysxDeformableBodyAPI":
            execute("RemoveDeformableBodyComponent", prim_path=self._usd_prim.GetPath())
        elif self._component == "PhysxDeformableSurfaceAPI":
            execute("RemoveDeformableSurfaceComponent", prim_path=self._usd_prim.GetPath())
        #~DEPRECATED
        elif self._component == "OmniPhysicsDeformableBodyAPI":
            execute("RemoveBaseDeformableBodyComponent", prim_path=self._usd_prim.GetPath())
        elif self._component == "PhysxAutoDeformableBodyAPI":
            execute("RemoveAutoDeformableBody", prim_path=self._usd_prim.GetPath())
        elif self._component == "PhysxAutoDeformableMeshSimplificationAPI":
            execute("RemoveAutoDeformableMeshSimplification", prim_path=self._usd_prim.GetPath())
        elif self._component == "PhysxAutoDeformableHexahedralMeshAPI":
            execute("RemoveAutoDeformableHexahedralMesh", prim_path=self._usd_prim.GetPath())
        elif self._component == "OmniPhysicsVolumeDeformableSimAPI":
            execute("RemoveVolumeDeformableSimComponent", prim_path=self._usd_prim.GetPath())
        elif self._component == "OmniPhysicsSurfaceDeformableSimAPI":
            execute("RemoveSurfaceDeformableSimComponent", prim_path=self._usd_prim.GetPath())
        elif self._component == "OmniPhysicsDeformablePoseAPI":
            execute("RemoveDeformablePoseComponent", prim_path=self._usd_prim.GetPath(), instance_name=self._multiple_api_token)
        elif "PhysicsMassAPI" in self._component:
            execute("UnapplyAPISchema", api=UsdPhysics.MassAPI, prim=self._usd_prim)

        elif self._component == "PhysxMimicJointAPI:rotX":
            remove_api(PhysxSchema.PhysxMimicJointAPI, self._usd_prim, PhysxSchema.Tokens.physxMimicJoint, UsdPhysics.Tokens.rotX)
        elif self._component == "PhysxMimicJointAPI:rotY":
            remove_api(PhysxSchema.PhysxMimicJointAPI, self._usd_prim, PhysxSchema.Tokens.physxMimicJoint, UsdPhysics.Tokens.rotY)
        elif self._component == "PhysxMimicJointAPI:rotZ":
            remove_api(PhysxSchema.PhysxMimicJointAPI, self._usd_prim, PhysxSchema.Tokens.physxMimicJoint, UsdPhysics.Tokens.rotZ)

        elif UsdPhysics.DistanceJoint.Get(stage, usdPath):
            execute("RemoveDistancePhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.RevoluteJoint.Get(stage, usdPath):
            execute("RemoveRevolutePhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.PrismaticJoint.Get(stage, usdPath):
            execute("RemovePrismaticPhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.SphericalJoint.Get(stage, usdPath):
            execute("RemoveSphericalPhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.FixedJoint.Get(stage, usdPath):
            execute("RemoveFixedPhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif UsdPhysics.Joint.Get(stage, usdPath):
            execute("RemoveD6PhysicsJointComponent", component=self._component, usd_prim=self._usd_prim)
        elif self._component == "PhysxVehicleContextAPI":
            remove_api(PhysxSchema.PhysxVehicleContextAPI)
        elif self._component == "PhysxVehicleSuspensionComplianceAPI":
            remove_api(PhysxSchema.PhysxVehicleSuspensionComplianceAPI)
        elif self._component == "PhysxVehicleSteeringAPI":
            remove_api(PhysxSchema.PhysxVehicleSteeringAPI)
        elif self._component == "PhysxVehicleAckermannSteeringAPI":
            remove_api(PhysxSchema.PhysxVehicleAckermannSteeringAPI)
        elif self._component == "PhysxVehicleBrakesAPI:brakes0":
            remove_api(PhysxSchema.PhysxVehicleBrakesAPI, self._usd_prim, PhysxSchema.Tokens.physxVehicleBrakes, PhysxSchema.Tokens.brakes0)
        elif self._component == "PhysxVehicleBrakesAPI:brakes1":
            remove_api(PhysxSchema.PhysxVehicleBrakesAPI, self._usd_prim, PhysxSchema.Tokens.physxVehicleBrakes, PhysxSchema.Tokens.brakes1)
        # DEPRECATED
        elif self._component == "PhysxAutoAttachmentAPI":
            remove_api(PhysxSchema.PhysxAutoAttachmentAPI)
        #~DEPRECATED
        elif self._component == "PhysxAutoDeformableAttachmentAPI":
            remove_api(Usd.SchemaRegistry().GetTypeFromSchemaTypeName("PhysxAutoDeformableAttachmentAPI"))
        elif self._component == "PhysxVehicleNonlinearCommandResponseAPI:drive":
            remove_api(PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI, self._usd_prim, PhysxSchema.Tokens.physxVehicleNCR, PhysxSchema.Tokens.drive)
        elif self._component == "PhysxVehicleNonlinearCommandResponseAPI:steer":
            remove_api(PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI, self._usd_prim, PhysxSchema.Tokens.physxVehicleNCR, PhysxSchema.Tokens.steer)
        elif self._component == "PhysxVehicleNonlinearCommandResponseAPI:brakes0":
            remove_api(PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI, self._usd_prim, PhysxSchema.Tokens.physxVehicleNCR, PhysxSchema.Tokens.brakes0)
        elif self._component == "PhysxVehicleNonlinearCommandResponseAPI:brakes1":
            remove_api(PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI, self._usd_prim, PhysxSchema.Tokens.physxVehicleNCR, PhysxSchema.Tokens.brakes1)
       
        elif self._component == "PhysxParticleSamplingAPI":
            execute("RemoveParticleSamplingCommand", stage=self._usd_prim.GetStage(), prim=self._usd_prim)
        elif self._component == "PhysxParticleSetAPI":
            execute("RemoveParticleSetCommand", stage=self._usd_prim.GetStage(), prim=self._usd_prim)
        # DEPRECATED
        elif self._component == "PhysxParticleClothAPI":
            execute("RemoveParticleClothComponentCommand", prim_path=self._usd_prim.GetPath())
        elif self._component == "PhysxAutoParticleClothAPI":
            remove_api(PhysxSchema.PhysxAutoParticleCloth)
        #~DEPRECATED
        elif self._component == "PhysxParticleAnisotropyAPI":
            remove_api(PhysxSchema.PhysxParticleAnisotropyAPI)
        elif self._component == "PhysxParticleSmoothingAPI":
            remove_api(PhysxSchema.PhysxParticleSmoothingAPI)
        elif self._component == "PhysxParticleIsosurfaceAPI":
            remove_api(PhysxSchema.PhysxParticleIsosurfaceAPI)
        elif self._component == "PhysxDiffuseParticlesAPI":
            remove_api(PhysxSchema.PhysxDiffuseParticlesAPI)
        elif self._component == "PhysxPBDMaterialAPI":
            remove_api(PhysxSchema.PhysxPBDMaterialAPI)


    def undo(self):
        for sc in self._loc_commands:
            sc.undo()


class SetSpatialTendonAttachmentParentCommand(omni.kit.commands.Command):
    def __init__(
        self,
        child_attachment_path: str,
        parent_attachment_path: str
    ):
        self._child_path = child_attachment_path
        self._parent_path = parent_attachment_path
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())

    def do(self):
        # get parent and child prims:
        parentAttachmentPath = Sdf.Path(self._parent_path)
        childAttachmentPath = Sdf.Path(self._child_path)
        parentBodyPath = parentAttachmentPath.GetParentPath()
        parentInstanceName = parentAttachmentPath.name
        childBodyPath = childAttachmentPath.GetParentPath()
        childInstanceName = childAttachmentPath.name
        # check inputs:
        parentPrim = self._stage.GetPrimAtPath(parentBodyPath)
        if not (parentPrim and parentPrim.HasAPI(PhysxSchema.PhysxTendonAttachmentAPI)):
            carb.log_error(type(self).__name__ + ": Invalid parent prim target - must be valid prim with a PhysxTendonAttachmentAPI (derived) API applied.")
        childPrim = self._stage.GetPrimAtPath(childBodyPath)
        childAPI = PhysxSchema.PhysxTendonAttachmentAPI(childPrim, childInstanceName)
        if not (childPrim and childAPI):
            carb.log_error(type(self).__name__ + ": Invalid child prim target - must be valid prim with a PhysxTendonAttachmentAPI (derived) API applied.")

        rel = childAPI.CreateParentLinkRel()
        self._usd_undo.reserve(childBodyPath.AppendProperty(rel.GetName()))
        success = rel.SetTargets([parentBodyPath.pathString])
        attr = childAPI.CreateParentAttachmentAttr()
        self._usd_undo.reserve(childBodyPath.AppendProperty(attr.GetName()))
        success = success and attr.Set(parentInstanceName)
        if success:
            return True
        else:
            carb.log_error(type(self).__name__ + ": Error setting new parent attributes.")
            self.undo()
            return False

    def undo(self):
        self._usd_undo.undo()


class RemoveSpatialTendonAttachmentAPICommand(omni.kit.commands.Command):
    def __init__(
        self,
        attachment_path: str,
        attachment_api: str
    ):
        self._attachment_path = attachment_path
        self._attachment_api = attachment_api

    def do(self):
        # get parent and child prims:
        attachmentPath = Sdf.Path(self._attachment_path)
        bodyPrim = omni.usd.get_context().get_stage().GetPrimAtPath(attachmentPath.GetParentPath())
        instanceName = attachmentPath.name
        return omni.kit.commands.execute("RemovePhysicsComponent", usd_prim=bodyPrim,
                                         component=self._attachment_api, multiple_api_token=instanceName)

    def undo(self):
        pass  # rely on Physics command undo


# DEPRECATED
class ImportTetrahedralMeshCommand(omni.kit.commands.Command):
    """ Creates a PhysxSchema.TetrahedralMesh from a TetMesh file.  Must provide either path_without_extension
        as a non-empty string or a node_str + tet_str [ + face_str (optional)]

        Parameters:
            target_tet_mesh_path:   Target path for new PhysxSchema.TetrahedralMesh.
            target_surface_mesh_path:   Target path for the associated surface mesh (can be None)
            path_without_extension:  Base file path to import (should not have file extension),
            use an empty string to ignore
            node_str:   pass a string for the node file contents directly (optional)
            tet_str:    pass a string for the tetrahedral mesh file contents directly (optional)
            face_str:   pass a string for the surface mesh file contents directly (optional)
            suppress_errors:    primarily for running tests, when testing conditions that are supposed to fail
        Returns:
            True / False that indicates success of command execution.

    """

    @autoassign
    def __init__(
        self,
        target_tet_mesh_path: Sdf.Path,
        target_surface_mesh_path: typing.Union[Sdf.Path, None] = None,
        path_without_extension: str = "",
        node_str: str = "",
        tet_str: str = "",
        face_str: str = "",
        suppress_errors: bool = False
    ):
        self._command_name = type(self).__name__
        super().__init__()

    def show_error(self, msg):
        if not self._suppress_errors:
            carb.log_error(self._command_name + ": " + msg)

    def undo_with_error(self, msg: str = "") -> bool:
        if msg:
            self.show_error(msg)
        self.undo()
        return False

    def do(self):
        stage = omni.usd.get_context().get_stage()
        if not self.check_inputs(stage):
            return False

        self._usd_undo = None
        self._usd_undo = UsdLayerUndo(stage.GetEditTarget().GetLayer())
        self._usd_undo.reserve(self._target_tet_mesh_path)
        if self._target_surface_mesh_path:
            self._usd_undo.reserve(self._target_surface_mesh_path)

        target_tet_mesh = PhysxSchema.TetrahedralMesh.Define(stage, self._target_tet_mesh_path)
        target_surface_mesh = None
        if not target_tet_mesh:
            return self.undo_with_error("target PhysxSchemaTetrahedralMesh creation failed.")

        if self._target_surface_mesh_path:
            target_surface_mesh = UsdGeom.Mesh.Define(stage, self._target_surface_mesh_path)
            if not target_surface_mesh:
                return self.undo_with_error("target UsdGeom.Mesh creation failed.")

        tet_mesh_data, surface_mesh_data = self.import_tet_mesh()
        if not tet_mesh_data:
            return self.undo_with_error()

        if tet_mesh_data.is_valid():
            target_tet_mesh.GetPointsAttr().Set(tet_mesh_data.points)
            target_tet_mesh.GetIndicesAttr().Set(tet_mesh_data.indices)
            gap = carb.settings.get_settings().get(SETTING_VISUALIZATION_GAP)
            target_tet_mesh.GetPrim().CreateAttribute("visualizationGap", Sdf.ValueTypeNames.Float).Set(gap)

            if surface_mesh_data.is_valid():
                number_of_faces = int(len(surface_mesh_data.indices) / 3)
                face_vertex_counts = [3]*number_of_faces  # fill an array of size 'number_of_faces' with 3
                target_surface_mesh.CreateFaceVertexCountsAttr().Set(face_vertex_counts)
                target_surface_mesh.CreateFaceVertexIndicesAttr().Set(surface_mesh_data.indices)
                target_surface_mesh.CreatePointsAttr().Set(surface_mesh_data.points)
            return True

        return self.undo_with_error("tetrahedral mesh import failed")

    def undo(self):
        if self._usd_undo:
            self._usd_undo.undo()

    def is_float(self, value):
        try:
            float(value)
            return True
        except:
            return False

    def extract_numbers_from_file(self, file_lines):
        chars_to_replace = ",;\t"
        final_result = []
        for line in file_lines:
            temp_line = line.split("#")[0]  # remove comments from line
            for c in chars_to_replace:
                temp_line = temp_line.replace(c, " ")
            numbers = temp_line.split()
            line_result = []
            for num in numbers:
                if self.is_float(num):
                    line_result.append(float(num))
            if line_result:
                final_result.append(line_result)
        return final_result

    def get_file_lines(self, current_extension, file_contents):

        if not current_extension:
            return []
        if file_contents:
            return self.extract_numbers_from_file(file_contents.splitlines())
        else:
            cur_path = self._path_without_extension + current_extension

            try:
                cur_file = open(cur_path, "r")
                cur_lines = cur_file.readlines()
                return self.extract_numbers_from_file(cur_lines)
            except IOError:
                self.show_error(f"Error opening {cur_path}")
                return []

    def import_tet_mesh(self):
        tet_mesh_data = deformableUtils.TetMeshData()
        surface_mesh_data = deformableUtils.TetMeshData()  # even though this is a surface mesh, we can reuse this data structure
        tet_pass = 0
        node_pass = 1
        face_pass = 2
        has_faces = True if self._target_surface_mesh_path else False
        if not (self._path_without_extension or (self._node_str and self._tet_str)):
            self.show_error(
                "insufficient parameters provided. You must provide the path_without_extension parameter OR " +
                "the node_str and tet_str parameters."
            )

        tet_node_face = [
            self.get_file_lines(".ele", self._tet_str),
            self.get_file_lines(".node", self._node_str),
            self.get_file_lines(".face" if has_faces else "", self._face_str)
        ]

        tet_indices = []
        face_indices = []
        points = []

        main_index = 0
        x_val = 0.0
        y_val = 0.0
        z_val = 0.0
        for current_data in tet_node_face:
            row_count = 0
            for row in current_data:
                if row_count > 0:  # ignore first row (metadata)
                    col_count = 0
                    for col in row:
                        if col_count > 0:  # ignore first column (index)
                            if main_index == tet_pass:
                                tet_indices.append(int(col) - 1)  # subtract one to convert to zero-based index
                            elif main_index == face_pass:
                                if col_count < 4:  # ignore last column (boundary markers)
                                    face_indices.append(int(col) - 1)  # subtract one to convert to zero-based index
                            elif main_index == node_pass:
                                if col_count == 1:
                                    x_val = col
                                elif col_count == 2:
                                    y_val = col
                                elif col_count == 3:
                                    z_val = col
                                    points.append(carb.Float3(x_val, y_val, z_val))
                        col_count += 1
                row_count += 1
            main_index += 1

        tet_mesh_data.from_dict({
            "points": points,
            "indices": tet_indices
        })

        if not tet_mesh_data.is_valid():
            self.show_error("tetrahedral mesh generation failed.")
            return None, None

        if has_faces:
            surface_mesh_data.from_dict({
                "points": points,
                "indices": face_indices
            })
            if not surface_mesh_data.is_valid():
                self.show_error("surface mesh generation failed.")
                return None, None

        return tet_mesh_data, surface_mesh_data

    def check_inputs(self, stage) -> bool:
        if not self._target_tet_mesh_path:
            self.show_error("ImportTetrahedralMeshCommand must provide target_tet_mesh_path.")
            return False
        self._target_tet_mesh_path = Sdf.Path(self._target_tet_mesh_path)

        if stage.GetPrimAtPath(self._target_tet_mesh_path):
            self.show_error("target tet mesh path already exists.")
            return False

        if self._target_surface_mesh_path and stage.GetPrimAtPath(self._target_surface_mesh_path):
            self.show_error("target surface mesh path already exists.")
            return False

        # all passed
        return True
#~DEPRECATED

# DEPRECATED
class CreatePhysicsAttachmentCommand(Command):

    @autoassign
    def __init__(self, target_attachment_path: Sdf.Path, actor0_path: Sdf.Path, actor1_path: Sdf.Path):
        super().__init__()

    def do(self) -> bool:
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        return self._create_attachment(self._target_attachment_path, self._actor0_path, self._actor1_path)

    def undo(self):
        self._usd_undo.undo()

    def _create_attachment(self, target_attachment_path: Sdf.Path, actor0_path: Sdf.Path, actor1_path: Sdf.Path) -> bool:
        self._usd_undo.reserve(target_attachment_path)

        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(self._stage, target_attachment_path)

        if actor0_path is not None:
            attachment.GetActor0Rel().SetTargets([actor0_path])

        if actor1_path is not None:
            attachment.GetActor1Rel().SetTargets([actor1_path])

        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())

        #add custom attributes, TODO move to schema
        attachment.GetPrim().CreateRelationship("physxAutoAttachment:maskShapes", True)

        return True
#~DEPRECATED


class AddParticleSystemCommand(Command):
    """
    Wrapper for omni.physx.utils.add_physx_particle_system. Adds a
    PhysxSchema.PhysxParticleSystem.

    Args:
        stage: USD stage
        target_particle_system_path: Path of the primitive to be created at.

        API params, see schema doc:

        enabled
        simulation_owner
        contact_offset
        particle_contact_offset
        solid_rest_offset
        fluid_rest_offset
        enable_ccd
        solver_position_iterations
        max_depenetration_velocity
        wind
        max_neighborhood
        max_velocity
        global_self_collision_enabled
        non_particle_collision_enabled
    """
    @autoassign
    def __init__(self, target_particle_system_path: Sdf.Path = None,
                enabled: bool = None,
                simulation_owner: Sdf.Path = None,
                contact_offset: float = None,
                rest_offset: float = None,
                particle_contact_offset: float = None,
                solid_rest_offset: float = None,
                fluid_rest_offset: float = None,
                enable_ccd: bool = None,
                solver_position_iteration_count: int = None,
                max_depenetration_velocity: float = None,
                wind: Gf.Vec3f = None,
                max_neighborhood: int = None,
                max_velocity: float = None,
                global_self_collision_enabled: bool = None,
                non_particle_collision_enabled: bool = None,
                ):
        super().__init__()

    def do(self):
        stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(stage.GetEditTarget().GetLayer())

        self._usd_undo.reserve(self._target_particle_system_path)

        particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=self._target_particle_system_path,
            particle_system_enabled=self._enabled,
            simulation_owner=self._simulation_owner,
            contact_offset=self._contact_offset,
            particle_contact_offset=self._particle_contact_offset,
            solid_rest_offset=self._solid_rest_offset,
            fluid_rest_offset=self._fluid_rest_offset,
            enable_ccd=self._enable_ccd,
            solver_position_iterations=self._solver_position_iteration_count,
            max_depenetration_velocity=self._max_depenetration_velocity,
            wind=self._wind,
            max_neighborhood=self._max_neighborhood,
            max_velocity=self._max_velocity,
            global_self_collision_enabled=self._global_self_collision_enabled,
            non_particle_collision_enabled=self._non_particle_collision_enabled
        )

    def undo(self):
        self._usd_undo.undo()


class AddDiffuseParticlesCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.add_physx_diffuse_particles. Adds a
    PhysxSchema.PhysxDiffuseParticlesAPI to a primitive with PhysxSchema.PhysxParticleSetAPI.

    API params, see schema doc
        stage
        path
        enabled
        max_diffuse_particle_multiplier
        threshold
        lifetime
        air_drag
        bubble_drag
        buoyancy
        kinetic_energy_weight
        pressure_weight
        divergence_weight
        collision_decay
        use_accurate_velocity
    """
    @autoassign
    def __init__(self,
        stage,
        path,
        enabled=None,
        max_diffuse_particle_multiplier=None,
        threshold=None,
        lifetime=None,
        air_drag=None,
        bubble_drag=None,
        buoyancy=None,
        kinetic_energy_weight=None,
        pressure_weight=None,
        divergence_weight=None,
        collision_decay=None,
        use_accurate_velocity=None
    ):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        particleUtils.add_physx_diffuse_particles(
            stage=self._stage,
            path=self._path,
            enabled=self._enabled,
            max_diffuse_particle_multiplier=self._max_diffuse_particle_multiplier,
            threshold=self._threshold,
            lifetime=self._lifetime,
            air_drag=self._air_drag,
            bubble_drag=self._bubble_drag,
            buoyancy=self._buoyancy,
            kinetic_energy_weight=self._kinetic_energy_weight,
            pressure_weight=self._pressure_weight,
            divergence_weight=self._divergence_weight,
            collision_decay=self._collision_decay,
            use_accurate_velocity=self._use_accurate_velocity)

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(PhysxSchema.PhysxDiffuseParticlesAPI, prim).do()


class AddParticleAnisotropyCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.add_physx_particle_anisotropy. Adds a
    PhysxSchema.PhysxParticleAnisotropyAPI to a particle system.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at.

        API params, see schema doc
            enabled
            scale
            min
            max
    """
    @autoassign
    def __init__(self,
        stage,
        path,
        enabled=None,
        scale=None,
        min=None,
        max=None,
    ):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        particleUtils.add_physx_particle_anisotropy(
            stage=self._stage,
            path=self._path,
            enabled=self._enabled,
            scale=self._scale,
            min=self._min,
            max=self._max,
        )

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(PhysxSchema.PhysxParticleAnisotropyAPI, prim).do()


class AddParticleSmoothingCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.add_physx_particle_smoothing. Adds a
    PhysxSchema.PhysxParticleSmoothingAPI to a particle system.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at.

        API params, see schema doc
            enabled
            strength
    """
    @autoassign
    def __init__(self,
        stage,
        path,
        enabled=None,
        strength=None,
    ):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        particleUtils.add_physx_particle_smoothing(
            stage=self._stage,
            path=self._path,
            enabled=self._enabled,
            strength=self._strength
        )

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(PhysxSchema.PhysxParticleSmoothingAPI, prim).do()

class AddParticleIsosurfaceCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.utils.add_physx_particle_isosurface. Adds a
    PhysxSchema.PhysxParticleIsosurfaceAPI to a particle system.

    Args:
        stage: USD stage.
        path: Path of the primitive to be created at.

        API params, see schema doc
            enabled
            max_vertices
            max_triangles
            max_subgrids
            grid_spacing
            surface_distance
            grid_filtering_passes
            grid_smoothing_radius
            enable_anisotropy
            anisotropy_min
            anisotropy_max
            anisotropy_radius
            num_mesh_smoothing_passes
            num_mesh_normal_smoothing_passes

    """
    @autoassign
    def __init__(self,
        stage,
        path,
        enabled=None,
        max_vertices=None,
        max_triangles=None,
        max_subgrids=None,
        grid_spacing=None,
        surface_distance=None,
        grid_filtering_passes=None,
        grid_smoothing_radius=None,
        num_mesh_smoothing_passes=None,
        num_mesh_normal_smoothing_passes=None,
    ):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        particleUtils.add_physx_particle_isosurface(
            stage=self._stage,
            path=self._path,
            enabled=self._enabled,
            max_vertices=self._max_vertices,
            max_triangles=self._max_triangles,
            max_subgrids=self._max_subgrids,
            grid_spacing=self._grid_spacing,
            surface_distance=self._surface_distance,
            grid_filtering_passes=self._grid_filtering_passes,
            grid_smoothing_radius=self._grid_smoothing_radius,
            num_mesh_smoothing_passes=self._num_mesh_smoothing_passes,
            num_mesh_normal_smoothing_passes=self._num_mesh_normal_smoothing_passes,
        )

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(PhysxSchema.PhysxParticleIsosurfaceAPI, prim).do()


class AddParticleSamplingCommand(PhysicsCommand):
    """
    Adds the particle sampling API to a mesh and generate a particle prim

    Args:
        prim: the USD prim this command is executed on
    """
    @autoassign
    def __init__(self, prim):
        self._prim_path = prim.GetPath()

    def do(self):
        stage = omni.usd.get_context().get_stage()
        particleUtils.poisson_sample_mesh(stage, self._prim_path)

    def undo(self):
        UnapplyAPISchemaCommand(PhysxSchema.PhysxParticleSamplingAPI, self._prim).do()


class RemoveParticleSamplingCommand(PhysicsCommand):
    """
    Removes particle sampling API from a mesh.
    Will remove the particle prim that was generated using the sampler.

    Args:
        stage: the stage with the particles
        prim: the USD prim this command is executed on
    """
    @autoassign
    def __init__(self, stage, prim):
        self._usd_undo = UsdLayerUndo(stage.GetEditTarget().GetLayer())

    def do(self):
        if not self._prim:
            return

        # need to remove the CRC outside of the usd undo scope to make resampling work.
        RemoveAttributeCommand.execute("physxParticleSampling:crc", self._prim)

        self._usd_undo.reserve(self._prim.GetPath())
        if self._prim.IsA(UsdGeom.Mesh) and self._prim.HasAPI(PhysxSchema.PhysxParticleSamplingAPI):
            UnapplyAPISchemaCommand(PhysxSchema.PhysxParticleSamplingAPI, self._prim).do()

    def undo(self):
        # explicitly hide mesh again because it's does not seem to be picked up otherwise.
        img = UsdGeom.Imageable(self._prim)
        img.CreateVisibilityAttr("invisible")

        self._usd_undo.undo()


class AddParticleSetCommand(PhysicsCommand):
    """
    Adds the PhysxParticleSetAPI to UsdGeomPointBased

    Args:
        prim: the USD prim this command is executed on
    """
    @autoassign
    def __init__(self, prim):
        pass

    def do(self):
        stage = omni.usd.get_context().get_stage()

        # get a particle system
        particle_system = particleUtils.get_default_particle_system(stage)
        self._particle_system_path = particle_system.GetPath()

        particle_system_prim = stage.GetPrimAtPath(self._particle_system_path)
        if not particle_system_prim or not PhysxSchema.PhysxParticleSystem(particle_system_prim):
            carb.log_error(type(self).__name__ + ": particle system path needs to point to valid PhysxSchema.ParticleSystem.")
            return

        particleUtils.configure_particle_set(self._prim, self._particle_system_path, True, True, 0)

    def undo(self):
        UnapplyAPISchemaCommand(PhysxSchema.PhysxParticleSetAPI, self._prim).do()


class RemoveParticleSetCommand(PhysicsCommand):
    """
    Removes the PhysxParticleSetAPI

    Args:
        stage: the USD stage containing the particles
        prim: the USD prim with the API
    """
    @autoassign
    def __init__(self, stage, prim):
        self._usd_undo = UsdLayerUndo(stage.GetEditTarget().GetLayer())
        pass

    def do(self):
        if self._prim:
            self._usd_undo.reserve(self._prim.GetPath())
            UnapplyAPISchemaCommand(PhysxSchema.PhysxParticleSetAPI, self._prim).do()

    def undo(self):
        self._usd_undo.undo()


# DEPRECATED
class AddParticleClothComponentCommand(Command):
    """DEPRECATED:"""

    @autoassign
    def __init__(self, prim_path):
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        # get a particle system
        particle_system = particleUtils.get_default_particle_system(self._stage)
        self._particle_system_path = particle_system.GetPath()

        particle_system_prim = self._stage.GetPrimAtPath(self._particle_system_path)
        if not particle_system_prim or not PhysxSchema.PhysxParticleSystem(particle_system_prim):
            carb.log_error(type(self).__name__ + ": particle system path needs to point to valid PhysxSchema.ParticleSystem.")
            return

        mesh = UsdGeom.Mesh.Get(self._stage, self._prim_path)
        if mesh:
            self._usd_undo.reserve(Sdf.Path(self._prim_path))

            particleUtils.add_physx_particle_cloth(
                stage=self._stage,
                path=self._prim_path,
                dynamic_mesh_path=None,
                particle_system_path=self._particle_system_path,
                spring_stretch_stiffness=10000.0,
                spring_bend_stiffness=100.0,
                spring_shear_stiffness=100.0,
                spring_damping=0.2,
                self_collision=True,
                self_collision_filter=True,
                particle_group=0,
            )

    def undo(self):
        self._usd_undo.undo()


# DEPRECATED
class RemoveParticleClothComponentCommand(PhysicsCommand):
    """DEPRECATED:"""

    @autoassign
    def __init__(self, prim_path: Sdf.Path):
        self._usd_context = omni.usd.get_context()

    def do(self):
        stage = self._usd_context.get_stage()
        prim = stage.GetPrimAtPath(self._prim_path)

        execute("UnapplyAPISchema", api=PhysxSchema.PhysxParticleClothAPI, prim=prim)
        execute("UnapplyAPISchema", api=PhysxSchema.PhysxAutoParticleClothAPI, prim=prim)

        # custom properties
        attributes_to_remove = [x.GetName() for x in prim.GetAuthoredPropertiesInNamespace(["physxParticle"])]
        for attribute in attributes_to_remove:
            RemoveAttributeCommand.execute(attribute, prim)

    def undo(self):
        pass


class AddPBDMaterialCommand(PhysicsCommand):
    """
    Wrapper for omni.physx.particleUtils.add_pbd_particle_material. Adds a PhysxSchema.PhysxPBDMaterialAPI to a
    UsdShade.Material prim, and creates the material prim if needed.

    Args:
        stage: USD stage.
        path: Path of the material to add the API to / or create a material if needed

        Material params, see schema API doc
        friction
        particle_friction_scale
        damping
        viscosity
        vorticity_confinement
        surface_tension
        cohesion
        adhesion
        particle_adhesion_scale
        adhesion_offset_scale
        gravity_scale
        lift
        drag
        density
        cfl_coefficient
    """
    @autoassign
    def __init__(
        self,
        stage,
        path,
        friction=None,
        particle_friction_scale=None,
        damping=None,
        viscosity=None,
        vorticity_confinement=None,
        surface_tension=None,
        cohesion=None,
        adhesion=None,
        particle_adhesion_scale=None,
        adhesion_offset_scale=None,
        gravity_scale=None,
        lift=None,
        drag=None,
        density=None,
        cfl_coefficient=None,
    ):
        pass

    def do(self):
        self._del_prim = not self._stage.GetPrimAtPath(self._path).IsValid()
        particleUtils.add_pbd_particle_material(
            stage=self._stage,
            path=self._path,
            friction=self._friction,
            particle_friction_scale=self._particle_friction_scale,
            damping=self._damping,
            viscosity=self._viscosity,
            vorticity_confinement=self._vorticity_confinement,
            surface_tension=self._surface_tension,
            cohesion=self._cohesion,
            adhesion=self._adhesion,
            particle_adhesion_scale=self._particle_adhesion_scale,
            adhesion_offset_scale=self._adhesion_offset_scale,
            gravity_scale=self._gravity_scale,
            lift=self._lift,
            drag=self._drag,
            density=self._density,
            cfl_coefficient=self._cfl_coefficient,
        )

    def undo(self):
        if self._del_prim:
            DeletePrimsCommand([self._path]).do()
        else:
            prim = self._stage.GetPrimAtPath(self._path)
            UnapplyAPISchemaCommand(PhysxSchema.PhysxPBDMaterialAPI, prim).do()


class SetCustomMetadataCommand(PhysicsCommand):
    # new_values, old_values: undo/redo dictionaries; {objectPath: value}. Value None means metadata clear.
    def __init__(self, stage, metadata_name, new_values, old_values):
        self._stage = stage
        self._metadata_name = metadata_name
        self._new_values = new_values
        self._old_values = old_values

    def _process_dict(self, dict):
        if not self._stage:
            return False

        for path, value in dict.items():
            obj = self._stage.GetObjectAtPath(path)
            if obj is not None and not obj.IsHidden():
                if value is None:
                    utils.clear_custom_metadata(obj, self._metadata_name)
                else:
                    utils.set_custom_metadata(obj, self._metadata_name, value)

    def do(self):
        self._process_dict(self._new_values)

    def undo(self):
        self._process_dict(self._old_values)


class CookPhysxColliders(PhysicsCommand):
    def __init__(self, **kwargs):
        pass

    def do(self):
        # force Physx to cook everything in the scene so it get cached
        get_physx_interface().force_load_physics_from_usd()
        # then release it
        get_physx_interface().release_physics_objects()


register_all_commands_in_module(__name__)
