# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physics.tensors import acquire_tensor_api, float32, uint8, uint32
import carb


def create_simulation_view(frontend_name, stage_id=-1):
    """ Creates an entry point to the physics simulation through physics tensors.

        It allows creating a :py:class:`SimulationView` object given the frontend tensor framework and the physX-based tensorization backend.

        Args:
            frontend_name (str): The name of the frontend to use. It can be one of the following: "numpy", "torch", "tensorflow", "warp".
            stage_id (int): The stage id to use. Default is -1.

        Returns:
            SimulationView: The simulation view object created with the specified frontend and physX backend.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("torch")
    """
    tensor_api = acquire_tensor_api()
    if tensor_api is None:
        raise Exception("Failed to acquire tensor API")

    backend = tensor_api.create_simulation_view(stage_id)
    if backend is None:
        raise Exception("Failed to create simulation view backend")

    device_ordinal = backend.device_ordinal

    frontend_id = frontend_name.lower()

    if frontend_id == "numpy" or frontend_id == "np":
        if device_ordinal == -1:
            from .frontend_np import FrontendNumpy

            frontend = FrontendNumpy()
            return SimulationView(backend, frontend)
        else:
            raise Exception("The Numpy frontend cannot be used with GPU pipelines")

    elif frontend_id == "torch" or frontend_id == "pytorch":
        from .frontend_torch import FrontendTorch

        frontend = FrontendTorch(device_ordinal)
        return SimulationView(backend, frontend)

    elif frontend_id == "tensorflow" or frontend_id == "tf":
        from .frontend_tf import FrontendTensorflow

        frontend = FrontendTensorflow(device_ordinal)
        return SimulationView(backend, frontend)

    elif frontend_id == "warp" or frontend_id == "wp":
        from .frontend_warp import FrontendWarp

        frontend = FrontendWarp(device_ordinal)
        return SimulationView(backend, frontend)

    else:
        raise Exception("Unrecognized frontend name '%s'" % frontend_name)


def reset():
    """ Resets the physics simulation with physX backend.

        This will reset the underlying tensor buffers that are created based on the exisiting stage.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("torch")
              >>> sim_view.reset() # required step to stop the simulation
    """
    tensor_api = acquire_tensor_api()
    if tensor_api is None:
        raise Exception("Failed to acquire tensor API")
    tensor_api.reset()


class SimulationView:
    """ SimulationView class represents a simulation environment.

        SimulationView class binds the tensor framework used to represent data with the physics simulation backend.
        This class isn't meant to be instantiated directly, but rather created using the :py:func:`create_simulation_view` function.
        Once created, the simulation view can be used to create specific views for different types of physics objects such as rigid bodies, articulations, soft bodies, etc.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("torch")
              >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
              >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*")
    """
    def __init__(self, backend, frontend):
        """ Constructs a SimulationView object with the given backend object and frontend framework.

            Currently, the only backend tensor is PhysX tensor.
            Several frontend frameworks are available: torch, numpy, tensorflow, warp.

            Args:
                backend (object): The backend object that represents the physics simulation.
                frontend (object): The frontend tensor framework used to handle data on the python side.
        """
        self._backend = backend
        self._frontend = frontend

    @property
    def device(self):
        """ The device property returns the device used by the frontend tensor framework.

            .. note::
                The numpy frontend can only be used on cpu. For GPU data/pipeline, use torch or warp frontends.
        """
        return self._frontend.device

    @property
    def is_valid(self):
        """ The is_valid property returns whether the simulation view is valid or not.

            .. note::
                The simulation view can become invalid under certain conditions such as
                when a physX object participating in tensorization is removed from the backend.
                In such scenarios, accessing the physics data is unsafe therefore the simulation view is marked as invalid.
                The user of the class can also invalidate a SimulationView object by calling the invalidate() method manually.
        """
        return self._backend.is_valid
    
    @property
    def device_ordinal(self):
        """ The device_ordinal property returns the device ordinal, which is -1 for CPU
            and 0,1,... for GPU devices depending on the number of GPUs available and the one being used for simulation.
        """
        return self._backend.device_ordinal

    @property
    def cuda_context(self):
        """
            The cuda_context property returns the cuda context when using GPU simulation and None for CPU simulation.
        """
        return self._backend.cuda_context
    
    def invalidate(self):
        """ Marks the simulation view as invalid.

            The invalidate method marks the simulation view as invalid to prevent it from being used further by other references.
            This is needed when the topology of the stage changes and the existing views of the physics objects cannot be used anymore.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> sim_view.invalidate() # sim_view.is_valid = False
        """
        return self._backend.invalidate()
    
    def set_subspace_roots(self, pattern):
        """ Sets the environment transform based on the transforms that match the given pattern.

            In Simulations with multiple environments each environment can have its own environment transform.
            When this exists, some APIs such as :py:func:`RigidBodyView.get_transforms` will return the transforms in the local space of the environment with respect to the environment local transform.
            This API can be used to inform the simulation view about the subspace prims with local transforms. 

            Args:
                pattern (Union[str, List[str]]): The pattern to set the subspace roots. The pattern can be a wildcard pattern to match multiple objects.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> sim_view.set_subspace_roots("/envs/*") # This will set environment transforms based on transforms found in /envs/* pattern
                  >>> sim_view.set_subspace_roots("/") # This will set environment transforms based on the world transform
        """
        if not self._backend.set_subspace_roots(pattern):
            raise Exception("Failed to set subspace roots")

    def get_object_type(self, path):
        """ Gets the type of the physics object at the given path in the scene.

        The type of the object can be one of the following:
        * ObjectType.Invalid: The object type is invalid or unrecognized.
        * ObjectType.RigidBody: The object is a rigid body but not a link on an articulation.
        * ObjectType.Articulation: The object is an articulation.
        * ObjectType.ArticulationLink: The object is an articulation link but not the root link.
        * ObjectType.ArticulationRootLink: The object is an articulation root link.
        * ObjectType.ArticulationJoint: The object is an articulation joint.

        Args:
            path (str): The path of the object in the scene to check its type.

        Returns:
            ObjectType: The type of the object at the given path.

        Example:
            .. code-block:: python

                >>> import omni.physics.tensors as tensors
                >>> sim_view = tensors.create_simulation_view("torch")
                >>> obj_type = sim_view.get_object_type("/World/Franka_0")  # obj_type will be Articulation since ArticulationRootAPI is applied to the top level xform prim
                >>> obj_type = sim_view.get_object_type("/World/Franka_0/panda_link0")  # obj_type will be ArticulationRootLink
                >>> obj_type = sim_view.get_object_type("/World/Franka_0/panda_link1")  # obj_type will be ArticulationLink
                >>> obj_type = sim_view.get_object_type("/World/Franka_0/panda_link0/panda_joint1")  # obj_type will be ArticulationJoint
                >>> obj_type = sim_view.get_object_type("/World/Ant_0")  # obj_type will be Invalid since ArticulationRootAPI is not applied to the top level xform prim but to torso
                >>> obj_type = sim_view.get_object_type("/World/Ant_0/torso") # obj_type will be ArticulationRootLink since torso is the root link of the articulation, it also implies an articulation at this path
                >>> obj_type = sim_view.get_object_type("/World/Ant_0/torso/right_back_leg")  # obj_type will be ArticulationLink
                >>> obj_type = sim_view.get_object_type("/World/Ant_0/joints/front_left_foot")  # obj_type will be ArticulationJoint
                >>> obj_type = sim_view.get_object_type("/World/Cube_0")  # obj_type will be RigidBody if RigidBodyAPI is attached to the cube and it isn't part of an articulation
        """
        return self._backend.get_object_type(path)
    
    def create_articulation_view(self, pattern):
        """ Creates a view of articulations of the scene that match the given pattern, or a list of patterns.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the articulations in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                ArticulationView: The articulation view object created that match the given pattern. Each articulation view object can be used to get/set data for the articulations of that view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_articulation_view("/World/Franka_*") # This will create a view of all Franka articulations in the scene whose paths start with "/World/Franka_"
                  >>> subset_view = create_articulation_view(["/World/Franka_[0-2]", "/World/Franka_[8-9]"]) # This will create a view of Franka articulations whose paths match "/World/Franka_i" where i is in [0,1,2,8,9]
                  >>> another_subset_view = create_articulation_view(["/World/Franka_0", "/World/Franka_3", "/World/Franka_7"]) # This will create a view of Franka articulations whose paths match "/World/Franka_i" where i is in [0,3,7]
        """
        return ArticulationView(self._backend.create_articulation_view(pattern), self._frontend)

    def create_rigid_body_view(self, pattern):
        """ Creates a view of rigid bodies of the scene that match the given pattern, or a list of patterns.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the rigid bodies in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                RigidBodyView: The rigid body view object created that match the given pattern. Each rigid body view object can be used to get/set data for the rigid bodies of that view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_rigid_body_view("/World/Cube_*") # This will create a view of all rigid bodies in the scene whose paths start with "/World/Cube_"
                  >>> subset_view = create_rigid_body_view(["/World/Cube_[0-2]", "/World/Cube_[8-9]"]) # This will create a view of rigid bodies whose paths match "/World/Cube_i" where i is in [0,1,2,8,9]
                  >>> same_view = create_rigid_body_view(["/World/Cube_0", "/World/Cube_3", "/World/Cube_7"]) # This will create a view of rigid bodies whose paths match "/World/Cube_i" where i is in [0,3,7]
        """
        return RigidBodyView(self._backend.create_rigid_body_view(pattern), self._frontend)

    # DEPRECATED
    def create_soft_body_view(self, pattern):
        """ Creates a view of soft bodies of the scene that match the given pattern, or a list of patterns.

            .. warning::
                This method is deprecated and will be removed.
                See :py:func:`SimulationView.create_volume_deformable_body_view` as an alternative which is based on 
                a successor deformable body feature.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the soft bodies in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                SoftBodyView: The soft body view object created that match the given pattern. Each soft body view object can be used to get/set data for the soft bodies of that view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_soft_body_view("/World/Soft_*") # This will create a view of all soft bodies in the scene whose paths start with "/World/Soft_"
                  >>> subset_view = create_soft_body_view(["/World/Soft_[0-2]", "/World/Soft_[8-9]"]) # This will create a view of soft bodies whose paths match "/World/Soft_i" where i is in [0,1,2,8,9]
                  >>> another_subset_view = create_soft_body_view(["/World/Soft_0", "/World/Soft_3", "/World/Soft_7"]) # This will create a view of soft bodies whose paths match "/World/Soft_i" where i is in [0,3,7]
        """
        return SoftBodyView(self._backend.create_soft_body_view(pattern), self._frontend)

    # DEPRECATED
    def create_soft_body_material_view(self, pattern):
        """ Creates a view of soft body materials of the scene that match the given pattern, or a list of patterns.

            .. warning::
                This method is deprecated and will be removed.
                See :py:func:`SimulationView.create_deformable_material_view` as an alternative which is based on 
                a successor deformable body feature.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the soft body materials in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                SoftBodyMaterialView: The soft body material view object created that match the given pattern. Each soft body material view object can be used to get/set data for the soft body materials of that view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This will create a view of all soft body materials in the scene whose paths start with "/World/SoftMaterial_"
                  >>> subset_view = create_soft_body_material_view(["/World/SoftMaterial_[0-2]", "/World/SoftMaterial_[8-9]"]) # This will create a view of soft body materials whose paths match "/World/SoftMaterial_i" where i is in [0,1,2,8,9]
                  >>> another_subset_view = create_soft_body_material_view(["/World/SoftMaterial_0", "/World/SoftMaterial_3", "/World/SoftMaterial_7"]) # This will create a view of soft body materials whose paths match "/World/SoftMaterial_i" where i is in [0,3,7]
        """
        return SoftBodyMaterialView(self._backend.create_soft_body_material_view(pattern), self._frontend)

    def create_volume_deformable_body_view(self, pattern):
        """ Creates a view of volume deformable bodies of the scene that match the given pattern, or a list of patterns. Surface deformables are not supported within
            the same view (see :py:func:`SimulationView.create_surface_deformable_body_view`).

            Args:
                pattern (Union[str, List[str]]): The pattern to match the volume deformable bodies in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                DeformableBodyView: The deformable body view object created that match the given pattern. Each deformable body view object can be used to get/set data for the deformable bodies of that view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_volume_deformable_body_view("/World/VolumeDeformable_*") # This will create a view of all volume deformable bodies in the scene whose paths start with "/World/VolumeDeformable_"
                  >>> subset_view = create_volume_deformable_body_view(["/World/VolumeDeformable_[0-2]", "/World/VolumeDeformable_[8-9]"]) # This will create a view of volume deformable bodies whose paths match "/World/VolumeDeformable_i" where i is in [0,1,2,8,9]
                  >>> another_subset_view = create_volume_deformable_body_view(["/World/VolumeDeformable_0", "/World/VolumeDeformable_3", "/World/VolumeDeformable_7"]) # This will create a view of volume deformable bodies whose paths match "/World/VolumeDeformable_i" where i is in [0,3,7]
        """
        return DeformableBodyView(self._backend.create_volume_deformable_body_view(pattern), self._frontend)

    def create_surface_deformable_body_view(self, pattern):
        """ Creates a view of surface deformable bodies of the scene that match the given pattern, or a list of patterns. Volume deformables are not supported within
            the same view (see :py:func:`SimulationView.create_volume_deformable_body_view`).

            Args:
                pattern (Union[str, List[str]]): The pattern to match the volume deformable bodies in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                DeformableBodyView: The deformable body view object created that match the given pattern. Each deformable body view object can be used to get/set data for the deformable bodies of that view.
            
            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_surface_deformable_body_view("/World/SurfaceDeformable_*") # This will create a view of all surface deformable bodies in the scene whose paths start with "/World/SurfaceDeformable_"
                  >>> subset_view = create_surface_deformable_body_view(["/World/SurfaceDeformable_[0-2]", "/World/SurfaceDeformable_[8-9]"]) # This will create a view of surface deformable bodies whose paths match "/World/SurfaceDeformable_i" where i is in [0,1,2,8,9]
                  >>> another_subset_view = create_surface_deformable_body_view(["/World/SurfaceDeformable_0", "/World/SurfaceDeformable_3", "/World/SurfaceDeformable_7"]) # This will create a view of surface deformable bodies whose paths match "/World/SurfaceDeformable_i" where i is in [0,3,7]
        """
        return DeformableBodyView(self._backend.create_surface_deformable_body_view(pattern), self._frontend)

    def create_deformable_material_view(self, pattern):
        """ Creates a view of deformable materials of the scene that match the given pattern, or a list of patterns. Both materials for
            volume and surface deformables can be represented within the same view.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the volume deformable bodies in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                DeformableBodyView: The deformable body view object created that match the given pattern. Each deformable body view object can be used to get/set data for the deformable bodies of that view.
            
            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_deformable_material_view("/World/DeformableMaterial_*") # This will create a view of all deformable materials in the scene whose paths start with "/World/DeformableMaterial_"
                  >>> subset_view = create_deformable_material_view(["/World/DeformableMaterial_[0-2]", "/World/DeformableMaterial_[8-9]"]) # This will create a view of deformable materials whose paths match "/World/DeformableMaterial_i" where i is in [0,1,2,8,9]
                  >>> another_subset_view = create_deformable_material_view(["/World/DeformableMaterial_0", "/World/DeformableMaterial_3", "/World/DeformableMaterial_7"]) # This will create a view of deformable materials whose paths match "/World/DeformableMaterial_i" where i is in [0,3,7]
        """
        return DeformableMaterialView(self._backend.create_deformable_material_view(pattern), self._frontend)
    
    def create_rigid_contact_view(self, pattern, filter_patterns=[], max_contact_data_count = 0):
        """ Creates a view of rigid contacts of the scene that match the given pattern/filter_pattern, or a list of patterns and a list of lists of filter_patterns.

            .. note::
                providing filter_patterns is optional. If not used, the view can only provide the net contact forces of the sensors with all the objects in the scene.
                If filter_patterns are provided, the view can provide the pair-wise contact forces of the sensors with the filter objects as well.

            .. warning::
                There is a one-to-one relationship between the sensors and the filter objects created via this API, and because of this some care is needed to ensure this API does what you intent to do.

                * If a single pattern is provided for sensor pattern, then a list of filter patterns should be provided for filter_pattern. This list contains all the filters for each sensor. Note that the same number of filters should be provided for all sensors.
                * If a list of regular expressions is provided for sensor pattern, then a list of lists of filter patterns should be provided for filter_pattern (each list of filter patterns corresponds to the corresponding sensor pattern in the list of sensor patterns).

                In this scenario, the order of lists of filter patterns, should match the order of the sensor patterns in the sensor pattern list.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the rigid bodies in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.
                                                Note that the pattern is used to select a set of "sensors", which are the objects whose contacts are tracked.
                filter_patterns (Optional[Union[List[str], List[list[str]]]]): The list of patterns of "filter" rigid bodies in the scene. This can be a single pattern or a list of patterns. The pattern can be a list of wildcard patterns to match multiple objects.
                                                        Note that the filter pattern is used to select a set of "filter", which are the objects that the sensors can come in contact with.
                                                        The filter objects will not report their contacts directly, but only when they are in contact with the sensors their contact forces will be included in the sensor's contact data.

            Returns:
                RigidContactView: The rigid contact view object created based on the given sensor/filter patterns. Each rigid contact view object can be used to get specific contact data for the sensor rigid bodies of the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  # The following will create a view of all rigid contacts in the scene whose paths start with "/World/Cube_"
                  >>> view_1 = sim_view.create_rigid_contact_view("/World/Cube_*")
                  # The following will create a view consisting of cubes 0,1,2,8, and 9
                  >>> view_2 = sim_view.create_rigid_contact_view(["/World/Cube_[0-2]", "/World/Cube_[8-9]"])
                  # The following will create a view consisting of cubes 0,3, and 7
                  >>> view_3 = sim_view.create_rigid_contact_view(["/World/Cube_0", "/World/Cube_3", "/World/Cube_7"])
                  # The following will create a view between all cube sensors and the Ground plane. Note that providing a filter pattern that matches a single prim (as opposed to the same number of prims as the sensors) will work as an exception.
                  >>> view_4 = sim_view.create_rigid_contact_view("/World/Cube_*", ["/World/GroundPlane"])
                  # The following will create a view between all cube sensors and the Ground plane and spheres. Note again that each Cube sensor will match with one Sphere prim. All Cube sensors will match with the GroundPlane prim (single prim exception explained in the above example).
                  >>> view_5 = sim_view.create_rigid_contact_view("/World/Cube_*", ["/World/GroundPlane", "/World/Sphere_*"])
                  # The following will create a view between cubes 0,1,2,8, and 9 and the Ground plane and spheres 0,1,2,8, and 9 respectively. Note that each Cube sensor will match with one Sphere prim. All Cube sensors will match with the GroundPlane prim (single prim exception explained in the above example).
                  >>> view_5 = sim_view.create_rigid_contact_view(["/World/Cube_[0-2]", "/World/Cube_[8-9]"], [["/World/GroundPlane", "/World/Sphere_[0-2]"], ["/World/GroundPlane", "/World/Sphere_[8-9]"]])
                  # The following will create a view between cubes 0,1,2,8, and 9 and the Ground plane and spheres 0,1,2,8, and 9 as well as box 0,1,2,8 and 9 respectively. Note that each Cube sensor will match with one Sphere prim and one box prim. All Cube sensors will match with the GroundPlane prim (single prim exception explained in the above example).
                  >>> view_6 = sim_view.create_rigid_contact_view(["/World/Cube_[0-2]", "/World/Cube_[8-9]"], [["/World/GroundPlane", "/World/Sphere_[0-2]", "/World/Box_[0-2]"], ["/World/GroundPlane", "/World/Sphere_[8-9]", "/World/Box_[8-9]"]])
                  # Creating one-to-many relationship is also possible but works slightly differently.
                  # The following will create a view to track contacts between num_sensors cubes and for each cube to track contacts with num_filters spheres.
                  # Note that instead of providing regular expressions you can provide prim paths directly. Note also that for each sensor there is one list of filter of size num_filters that encapsulates all the spheres.
                  >>> view_7 = create_rigid_contact_view([f"/World/Cube_{j}" for j in range(num_sensors)], filter_patterns= [[f"/World/Sphere_{j}" for j in range(num_filters)]] * num_sensors)
        """
        return RigidContactView(self._backend.create_rigid_contact_view(pattern, filter_patterns, max_contact_data_count), self._frontend)

    def create_sdf_shape_view(self, pattern, num_points):
        """ Creates a view of SDF shapes of the scene that match the given pattern, or a list of patterns.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the SDF shapes in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.
                num_points (int): The number of points to sample the SDF shape with.

            Returns:
                SdfShapeView: The SDF shape view object created that match the given pattern. Each SDF shape view object can be used to get signed distance field properties of a shape.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_sdf_shape_view("/World/SDF_*", 1000) # This will create a view of all SDF shapes in the scene whose paths start with "/World/SDF_" with 1000 points sample per shape
                  >>> subset_view = create_sdf_shape_view(["/World/SDF_[0-2]", "/World/SDF_[8-9]"], 1000) # This will create a view of SDF shapes whose paths match "/World/SDF_i" where i is in [0,1,2,8,9] with 1000 points sample per shape
                  >>> another_subset_view = create_sdf_shape_view(["/World/SDF_0", "/World/SDF_3", "/World/SDF_7"], 1000) # This will create a view of SDF shapes whose paths match "/World/SDF_i" where i is in [0,3,7] with 1000 points sample per shape
        """
        return SdfShapeView(self._backend.create_sdf_shape_view(pattern, num_points), self._frontend)

    # DEPRECATED
    def create_particle_system_view(self, pattern):
        """ Creates a view of particle systems of the scene that match the given pattern, or a list of patterns.

            .. warning::
                This method is deprecated and will be removed.
                See :py:func:`SimulationView.create_surface_deformable_body_view` as an alternative which is based on 
                a successor deformable body feature.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the particle systems in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                ParticleSystemView: The particle system view object created that match the given pattern. Each particle system view object can be used to get/set data for the particle systems of that view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_particle_system_view("/World/Particles_*") # This will create a view of all particle systems in the scene whose paths start with "/World/Particles_"
                  >>> subset_view = create_particle_system_view(["/World/Particles_[0-2]", "/World/Particles_[8-9]"]) # This will create a view of particle systems whose paths match "/World/Particles_i" where i is in [0,1,2,8,9]
                  >>> another_subset_view = create_particle_system_view(["/World/Particles_0", "/World/Particles_3", "/World/Particles_7"]) # This will create a view of particle systems whose paths match "/World/Particles_i" where i is in [0,3,7]
        """
        return ParticleSystemView(self._backend.create_particle_system_view(pattern), self._frontend)

    # DEPRECATED
    def create_particle_cloth_view(self, pattern):
        """ Creates a view of particle cloths of the scene that match the given pattern, or a list of patterns.

            .. warning::
                This method is deprecated and will be removed.
                See :py:func:`SimulationView.create_surface_deformable_body_view` as an alternative which is based on 
                a successor deformable body feature.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the particle cloths in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                ParticleClothView: The particle cloth view object created that match the given pattern. Each particle cloth view object can be used to get/set data for the particle cloths of that view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_particle_cloth_view("/World/Cloth_*") # This will create a view of all particle cloths in the scene whose paths start with "/World/Cloth_"
                  >>> subset_view = create_particle_cloth_view(["/World/Cloth_[0-2]", "/World/Cloth_[8-9]"]) # This will create a view of particle cloths whose paths match "/World/Cloth_i" where i is in [0,1,2,8,9]
                  >>> another_subset_view = create_particle_cloth_view(["/World/Cloth_0", "/World/Cloth_3", "/World/Cloth_7"]) # This will create a view of particle cloths whose paths match "/World/Cloth_i" where i is in [0,3,7]
        """
        return ParticleClothView(self._backend.create_particle_cloth_view(pattern), self._frontend)

    # DEPRECATED
    def create_particle_material_view(self, pattern):
        """ Creates a view of particle materials of the scene that match the given pattern, or a list of patterns.

            .. warning::
                This method is deprecated and will be removed.
                See :py:func:`SimulationView.create_deformable_material_view` as an alternative which is based on 
                a successor deformable body feature.

            Args:
                pattern (Union[str, List[str]]): The pattern to match the particle materials in the scene. This can be a single pattern or a list of patterns. The pattern can be a wildcard pattern to match multiple objects.

            Returns:
                ParticleMaterialView: The particle material view object created that match the given pattern. Each particle material view object can be used to get/set data for the particle materials of that view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_particle_material_view("/World/ParticleMaterial_*") # This will create a view of all particle materials in the scene whose paths start with "/World/ParticleMaterial_"
                  >>> subset_view = create_particle_material_view(["/World/ParticleMaterial_[0-2]", "/World/ParticleMaterial_[8-9]"]) # This will create a view of particle materials whose paths match "/World/ParticleMaterial_i" where i is in [0,1,2,8,9]
                  >>> another_subset_view = create_particle_material_view(["/World/ParticleMaterial_0", "/World/ParticleMaterial_3", "/World/ParticleMaterial_7"]) # This will create a view of particle materials whose paths match "/World/ParticleMaterial_i" where i is in [0,3,7]
        """
        return ParticleMaterialView(self._backend.create_particle_material_view(pattern), self._frontend)

    def flush(self):
        carb.log_warn("DEPRECATED: flush() no longer is needed and has no effect.")
        return self._backend.flush()

    def clear_forces(self):
        """ Clears the force/torque data buffers from the physics simulation backend.
        """
        return self._backend.clear_forces()

    def enable_warnings(self, enable):
        """ Enables or disables extra warnings from all APIs.

            Args:
                enable (bool): True to enable warnings, False to disable warnings.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> sim_view.enable_warnings(False) # warnings are not sent anymore
        """
        return self._backend.enable_warnings(enable)

    def update_articulations_kinematic(self):
        """ Updates the kinematic state of all articulations in the scene.

            Depending on the concrete implementation, calling this might be necessary before taking another simulation step to update link transforms according to the latest joint states set by the user at the current time step.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> view.set_dof_positions(new_franka_dof_positions, franka_indices) # This will change the pose of all the Franka articulaitons in the view
                  >>> sim_view.update_articulations_kinematic() # For GPU, this is required to update the links kinematic
        """
        return self._backend.update_articulations_kinematic()


    def initialize_kinematic_bodies(self):
        """
            Initializes all the kinematics bodies in the scene. 
            By default kinematic bodies are not part of the simulated physics. This function performs any necessary initialization to make them report their transforms and velocities.
        """
        return self._backend.initialize_kinematic_bodies()


    def check(self):
        """ Checks all the physics views created by this object for validity.

            If any of the views are invalid, e.g. when a corresponding physics object is not found on the physics engine this will return False.

            Returns:
                bool: True if all the views are valid, False otherwise.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*")
                  >>> sim_view.check() # returns False if rigid_body_view or articulation_view is invalid, True otherwise
        """
        return self._backend.check()

    def step(self, dt):
        return self._backend.step(dt)

    def set_gravity(self, gravity):
        """ Sets the gravity of the physics scene.

            Args:
                gravity (carb.Float3): The gravity vector to set.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> sim_view.set_gravity(carb.Float3(0, -9.8, 0)) # This will set the gravity to be -9.8 in the y direction
        """
        return self._backend.set_gravity(gravity)

    def get_gravity(self):
        """ Gets the gravity of the physics scene.

            Returns:
                carb.Float3: The gravity vector of the scene or None if the gravity is not defined.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("torch")
                  >>> gravity = sim_view.get_gravity() # This will fetch the gravity vector used by the simulation
        """
        gravity = carb.Float3()
        res = self._backend.get_gravity(gravity)
        if res:
            return gravity
        else:
            return None 


class ArticulationView:
    """ ArticulationView class represents a batch of articulations.

        ArticulationView binds the concrete implementation of the physics backend with the frontend tensor framework that is used to handle data.
        This class isn't meant to be instantiated directly, but rather created using the :py:func:`SimulationView.create_articulation_view` method of the :py:class:`SimulationView` object
        which manages all the physics views, the device where the simulation is performed and data resides.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("warp")
              >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
    """
    def __init__(self, backend, frontend):
        """ Constructs an ArticulationView object with the given backend object and frontend framework.

            Currently, the only backend tensor is PhysX tensor.
            Several frontend frameworks are available: torch, numpy, tensorflow, warp.

            Args:
                backend (IArticulationView): The concrete implementation of the IArticulationView interface.
                frontend (FrontendBase): The tensor framework that is used to handle data which could be FrontendNumpy, FrontendTorch, FrontendTensorflow, or FrontendWarp.
        """
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        """
            The number of articulations in the view.
        """
        return self._backend.count

    @property
    def max_links(self):
        """
            The maximum number of links in all the view's articulations.
        """
        return self._backend.max_links

    @property
    def max_dofs(self):
        """
            The maximum number of degrees of freedom in all the view's articulations.
        """
        return self._backend.max_dofs

    @property
    def max_shapes(self):
        """
            The maximum number of shapes in all the view's articulations.
        """
        return self._backend.max_shapes

    @property
    def max_fixed_tendons(self):
        """
            The maximum number of fixed tendons in all the view's articulations.
        """
        return self._backend.max_fixed_tendons

    @property
    def max_spatial_tendons(self):
        """
            The maximum number of spatial tendons in all the view's articulations.
        """
        return self._backend.max_spatial_tendons

    @property
    def is_homogeneous(self):
        """
            The is_homogeneous property indicates whether all the articulations in the view are of the same type.
            Note that currently only homogeneous articulation views are supported. To handle inhomogeneous articulations, create separate views for each type.
        """
        return self._backend.is_homogeneous

    @property
    def shared_metatype(self):
        """
            The shared_metatype property indicates the metatype of the articulations in the view.
            The metatype features static information about articulation links and joints.
        """
        return self._backend.shared_metatype

    @property
    def jacobian_shape(self):
        """
            The jacobian_shape property indicates the dimension of the Jacobian matrix of the articulations in the view.
            The Jacobian matrix dimension depends on the number of degrees of freedom, number of links as well as floating/fixed base properties of the articulations.
            For fixed base articulations, the Jacobian matrix dimensions are max_dofs x ((numLinks - 1) * 6).
            For floating base articulations, the Jacobian matrix dimensions are (max_dofs + 6) x (numLinks * 6).
        """
        shape = self._backend.jacobian_shape
        if shape is None:
            raise Exception("Unable to obtain Jacobian shape")
        return shape

    # DEPRECATED
    @property
    def mass_matrix_shape(self):
        """
            The mass_matrix_shape property indicates the dimension of the mass matrix of the articulations in the view.
            The mass matrix dimension depends on the number of degrees of freedom of the articulations and is max_dofs x max_dofs.

            .. warning::
                This property is deprecated and will be removed in the future. Use :py:data:`ArticulationView.generalized_mass_matrix_shape` instead.
        """
        shape = self._backend.mass_matrix_shape
        if shape is None:
            raise Exception("Unable to obtain Mass Matrix shape")
        return shape

    @property
    def generalized_mass_matrix_shape(self):
        """
            The generalized_mass_matrix_shape property indicates the dimension of the generalized mass matrix of the articulations in the view.
            The mass matrix dimension depends on the number of degrees of freedom as well as floating/fixed base properties of the articulations.
            For fixed base articulations, the mass matrix dimensions are max_dofs x max_dofs.
            For floating base articulations, the mass matrix dimensions are (max_dofs + 6) x (max_dofs + 6).
        """
        shape = self._backend.generalized_mass_matrix_shape
        if shape is None:
            raise Exception("Unable to obtain Mass Matrix shape")
        return shape

    @property
    def prim_paths(self):
        """
            The prim_paths property indicates the paths to the articulations encapsulated in the view.
        """
        return self._backend.prim_paths

    @property
    def dof_paths(self):
        """
            The dof_paths property indicates the paths to the degrees of freedom (DOF) encapsulated in the view.
            For single DOF joints, the path is the same as the joint path whereas for multi-DOF joints, the path is the joint path appended with the DOF index.
        """
        return self._backend.dof_paths

    @property
    def link_paths(self):
        """
            The link_paths property indicates the paths to the links of articulations encapsulated in the view.
        """
        return self._backend.link_paths

    def get_metatype(self, arti_idx):
        """ Gets the metatype of the articulation at the given index.

            Args:
                arti_idx (int): The index of the articulation to get the metatype of.

            Note: 
                The metatype features static information about articulation links and joints as follows:
                - link_count: The total number of links in the articulation.
                - joint_count: The total number of joints in the articulation.
                - dof_count: The total number of degrees of freedom in the articulation.
                - link_names: A list of the link names in the articulation. If duplicate names are present, an incrementing number is appended to the original name.
                - link_parents: A list of each link’s parent name. This corresponds to the link_names list above.
                - joint_names: A list of the joint names in the articulation. If duplicate names are present, an incrementing number is appended to the original name.
                - dof_names: A list of the DOF (Degrees Of Freedom) names. It corresponds to the joint name and an incrementing number is appended for each DOF of spherical joints.
                - link_indices: A mapping from link names to link index.
                - link_parent_indices: A mapping from link names to the index of their parent link.
                - joint_indices: A mapping from joint names to their index.
                - dof_indices: A mapping from DOF names to their index.
                - joint_types: A list of the type of each joint.
                - joint_dof_offsets: A list indicating the start offset of each joint’s DOFs.
                - joint_dof_counts: A list indicating how many DOFs each joint has.
                - dof_types: A list of the type of each DOF.
                - fixed_base: A boolean indicating if the base link is fixed (True) or floating (False).

            Returns:
                ArticulationMetatype: The metatype of the articulation at the given index.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> metatype = articulation_view.get_metatype(1) # Get the metatype of the articulation with index 1 in the given articulation view
                  >>> fixed_base = metatype.fixed_base # Get a boolean indicating whether the articulation is fixed base or floating base
                  >>> link_count = metatype.link_count # Get the number of links in the articulation
                  >>> link_0_name = metatype.link_names[0] # Get the name of the link 0
                  >>> link_0_parent_name = metatype.link_parents[0] # Get the parent name of link 0
                  >>> link_0_idx = metatype.link_indices[link_0_name] # Get the link index of link_0_name
                  >>> link_0_parent_idx = metatype.link_parent_indices[link_0_name] # Get the parent index of link 0 by name from the link name to parent index map.
                  >>> joint_count = metatype.joint_count # Get the number of joints in the articulation
                  >>> joint_0_name = metatype.joint_names[0] # Get the name of the joint 0
                  >>> joint_0_idx = metatype.joint_indices[joint_0_name] # Get the joint index of joint_0_name
                  >>> joint_0_type = metatype.joint_types[joint_0_idx] # Get the joint type of joint_0_name
                  >>> joint_0_dof_offset = metatype.joint_dof_offsets[joint_0_idx] # Get the joint DOF offset of joint_0_name
                  >>> joint_0_dof_count = metatype.joint_dof_counts[joint_0_idx] # Get the number of DOFs of joint_0_name
                  >>> dof_count = metatype.dof_count # Get the number of DOFs in the articulation
                  >>> dof_0_name = metatype.dof_names[0] # Get the name of the DOF 0
                  >>> dof_0_idx = metatype.dof_indices[dof_0_name] # Get the DOF index of dof_0_name
                  >>> dof_0_type = metatype.dof_types[dof_0_idx] # Get the DOF type of dof_0_name
                  >>> joint_0_dof_0_type = metatype.dof_types[joint_0_dof_offset] # Get the DOF type of the first DOF of joint_0_name
        """
        return self._backend.get_metatype(arti_idx)

    def get_dof_types(self):
        """ Gets the degrees of freedom (DOF) types of the articulations in the view.

            Potential DOF types are:

            * 0 (DofType.Rotation) for rotation.
            * 1 (DofType.Translation) for translation.
            * 255 (DofType.Invalid) for invalid DOF type.

            .. note::
                The function raises an exception if the DOF types cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF types with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_types = articulation_view.get_dof_types() # Get the DOF type for all DOFs and all articulations in the view
                  >>> dof_types_np = dof_types.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
                  >>> dof_types_final = dof_types_np[0] # Takes only the results for the first articulation as all articulations in a view must be identical
        """
        if not hasattr(self, "_dof_types"):
            self._dof_types, self._dof_types_desc = self._frontend.create_tensor((self.count, self.max_dofs), uint8, -1)

        if not self._backend.get_dof_types(self._dof_types_desc):
            raise Exception("Failed to get DOF types from backend")

        return self._dof_types

    def get_dof_motions(self):
        """ Gets the degrees of freedom (DOF) motion types for all articulations in the view.

            Potential DOF motions are:

            * 0 (DofMotion.Free) when the DOF motion has no constraint.
            * 1 (DofMotion.Limited) when the DOF is limited in its movement.
            * 2 (DofMotion.Locked) when the DOF is locked.
            * 255 (DofMotion.Invalid) when the motion is invalid.

            .. note::
                The function raises an exception if the DOF motions cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF motions with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_motions = articulation_view.get_dof_motions() # Get the DOF motion for all DOFs and all articulations in the view
                  >>> dof_motions_np = dof_motions.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
                  >>> dof_motions_final = dof_motions_np[0] # Takes only the results for the first articulation as all articulations in a view must be identical
        """
        if not hasattr(self, "_dof_motions"):
            self._dof_motions, self._dof_motions_desc = self._frontend.create_tensor((self.count, self.max_dofs), uint8, -1)

        if not self._backend.get_dof_motions(self._dof_motions_desc):
            raise Exception("Failed to get DOF motions from backend")

        return self._dof_motions

    def get_dof_limits(self):
        """ Gets the degrees of freedom (DOF) upper and lower limits for all articulations in the view.

            .. note::
                The function raises an exception if the DOF limits cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]:
                  An array of DOF limits with shape (count, max_dofs, 2) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                  The first index of the last dimension is reserved for the lower limit and the second index is for the upper limit.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_limits = articulation_view.get_dof_limits() # Get the DOF limits for all DOFs and all articulations in the view
                  >>> dof_limits_np = dof_limits.numpy().reshape(articulation_view.count, articulation_view.max_dofs, 2) # Reshape the obtained array in a 3D numpy array on the host
                  >>> dof_limits_final = dof_limits_np[0] # Takes only the results for the first articulation as all articulations in a view must be identical
        """
        if not hasattr(self, "_dof_limits"):
            self._dof_limits, self._dof_limits_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs, 2), float32, -1
            )

        if not self._backend.get_dof_limits(self._dof_limits_desc):
            raise Exception("Failed to get DOF limits from backend")

        return self._dof_limits

    def get_drive_types(self):
        """ Gets the degrees of freedom (DOF) drive types for all articulations in the view.

            Potential drive types are:

            * 0 (DofDriveType.None) if no drive is present.
            * 1 (DofDriveType.Force) if a force drive is present.
            * 2 (DofDriveType.Acceleration) if an acceleration drive is present.

            .. note::
                The function raises an exception if the DOF drive types cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF drive types with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> drive_types = articulation_view.get_drive_types() # Get the drive type for all DOFs and all articulations in the view
                  >>> drive_types_np = drive_types.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_drive_types"):
            self._drive_types, self._drive_types_desc = self._frontend.create_tensor((self.count, self.max_dofs), uint8, -1)

        if not self._backend.get_drive_types(self._drive_types_desc):
            raise Exception("Failed to get drive types from backend")

        return self._drive_types

    def get_dof_stiffnesses(self):
        """ Gets the degrees of freedom (DOF) drive stiffnesses for all articulations in the view.

            .. note::
                The function raises an exception if the DOF stiffnesses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF stiffnesses with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_stiffnesses = articulation_view.get_dof_stiffnesses() # Get the DOF stiffness for all DOFs and all articulations in the view
                  >>> dof_stiffnesses_np = dof_stiffnesses.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_stiffnesses"):
            self._dof_stiffnesses, self._dof_stiffnesses_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_stiffnesses(self._dof_stiffnesses_desc):
            raise Exception("Failed to get DOF stiffnesses from backend")

        return self._dof_stiffnesses

    def get_dof_dampings(self):
        """ Gets the degrees of freedom (DOF) drive dampings for all articulations in the view.

            .. note::
                The function raises an exception if the DOF dampings cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF dampings with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_dampings = articulation_view.get_dof_dampings() # Get the DOF damping for all DOFs and all articulations in the view
                  >>> dof_dampings_np = dof_dampings.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_dampings"):
            self._dof_dampings, self._dof_dampings_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_dampings(self._dof_dampings_desc):
            raise Exception("Failed to get DOF dampings from backend")

        return self._dof_dampings

    def get_dof_max_forces(self):
        """ Gets the degrees of freedom (DOF) maximum forces applied by the drive for all articulations in the view.

            .. note::
                The function raises an exception if the DOF maximum forces cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF maximum forces with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_max_forces = articulation_view.get_dof_max_forces() # Get the DOF maximum force applied by the drive for all DOFs and all articulations in the view
                  >>> dof_max_forces_np = dof_max_forces.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_max_forces"):
            self._dof_max_forces, self._dof_max_forces_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_max_forces(self._dof_max_forces_desc):
            raise Exception("Failed to get DOF max forces from backend")

        return self._dof_max_forces

    def get_dof_drive_model_properties(self):
        """ Gets the degrees of freedom (DOF) drive model properties for all articulations in the view.

            The drive model properties tensor is composed of the following properties (provided in the order of appearance in the array):

            * speed effort gradient
            * max actuator velocity
            * velocity dependent resistance

            .. note::
                The function raises an exception if the DOF drive model properties cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF drive model properties with shape (count, max_dofs, 3) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations. The 3 elements of the last dimension are the speed effort gradient, max actuator velocity, and velocity dependent resistance respectively.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_drive_model_properties = articulation_view.get_dof_drive_model_properties() # Get the DOF drive model properties for all DOFs and all articulations in the view
                  >>> dof_drive_model_properties_np = dof_drive_model_properties.numpy().reshape(articulation_view.count, articulation_view.max_dofs, 3) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_dof_drive_model_properties"):
            self._dof_drive_model_properties, self._dof_drive_model_properties_desc = self._frontend.create_tensor((self.count, self.max_dofs, 3), float32, -1)

        if not self._backend.get_dof_drive_model_properties(self._dof_drive_model_properties_desc):
            raise Exception("Failed to get DOF drive model properties from backend")

        return self._dof_drive_model_properties

    # DEPRECATED
    def get_dof_friction_coefficients(self):
        """ Gets the degrees of freedom (DOF) friction coefficients for all articulations in the view.

            .. warning::
                This function is deprecated and will be removed in the future. Use :py:func:`ArticulationView.get_dof_friction_properties` instead.

            .. note::
                The function raises an exception if the DOF friction coefficients cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF friction coefficients with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_friction_coefficients = articulation_view.get_dof_friction_coefficients() # Get the DOF friction coefficient for all DOFs and all articulations in the view
                  >>> dof_friction_coefficients_np = dof_friction_coefficients.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_friction_coefficients"):
            self._dof_friction_coefficients, self._dof_friction_coefficients_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_friction_coefficients(self._dof_friction_coefficients_desc):
            raise Exception("Failed to get DOF friction coefficients from backend")

        return self._dof_friction_coefficients

    def get_dof_friction_properties(self):
        """ Gets the degrees of freedom (DOF) friction properties for all articulations in the view.

            The friction properties array is composed of the following properties (provided in the order of appearance in the array):

            * static friction effort
            * dynamic friction effort
            * viscous friction coefficient

            .. note::
                The function raises an exception if the friction properties cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF friction properties with shape (count, max_dofs, 3) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations. The 3 elements of the last dimension are the static friction effort, dynamic friction effort, and viscous friction coefficient respectively.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> friction_properties = articulation_view.get_dof_friction_properties() # Get the friction properties for all DOFs and all articulations in the view
                  >>> friction_properties_np = friction_properties.numpy().reshape(articulation_view.count, articulation_view.max_dofs, 3) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_friction_properties"):
            self._friction_properties, self._friction_properties_desc = self._frontend.create_tensor((self.count, self.max_dofs, 3), float32, -1)

        if not self._backend.get_dof_friction_properties(self._friction_properties_desc):
            raise Exception("Failed to get articulation friction properties from backend")

        return self._friction_properties

    def get_dof_max_velocities(self):
        """ Gets the degrees of freedom (DOF) maximum velocities for all articulations in the view.

            .. note::
                The function raises an exception if the DOF maximum velocities cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF maximum velocities with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_max_velocities = articulation_view.get_dof_max_velocities() # Get the DOF maximum velocity for all DOFs and all articulations in the view
                  >>> dof_max_velocities_np = dof_max_velocities.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_max_velocities"):
            self._dof_max_velocities, self._dof_max_velocities_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_max_velocities(self._dof_max_velocities_desc):
            raise Exception("Failed to get DOF max velocities from backend")

        return self._dof_max_velocities

    def get_dof_armatures(self):
        """ Gets the degrees of freedom (DOF) armatures for all articulations in the view.

            .. note::
                The function raises an exception if the DOF armatures cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF armatures with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_armatures = articulation_view.get_dof_armatures() # Get the DOF armature for all DOFs and all articulations in the view
                  >>> dof_armatures_np = dof_armatures.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_armatures"):
            self._dof_armatures, self._dof_armatures_desc = self._frontend.create_tensor((self.count, self.max_dofs), float32, -1)

        if not self._backend.get_dof_armatures(self._dof_armatures_desc):
            raise Exception("Failed to get DOF armatures from backend")

        return self._dof_armatures

    def set_dof_limits(self, data, indices):
        """ Sets the degrees of freedom (DOF) limits for articulations indicated by indices.

            .. note::
                * The function raises an exception if the DOF limits cannot be set in the backend.
                * The sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_limits` can be used to get the current limits and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF limits with shape (count, max_dofs, 2) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                                                                  The first index of the last dimension is reserved for the lower limit and the second index is for the upper limit.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the limits for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> dof_limits_np = numpy.zeros([articulation_view.count, articulation_view.max_dofs, 2]) # Create an array with the expected shape
                  >>> dof_limits_np[:, :, 0] = -0.5 # Modify the lower limit of all DOF and all articulation
                  >>> dof_limits_np[:, :, 1] = 0.5 # Modify the upper limit of all DOF and all articulation
                  >>> dof_limits_wp = warp.from_numpy(dof_limits_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_dof_limits(dof_limits_wp, all_indices) # Set the new DOF limits
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_limits(data_desc, indices_desc):
            raise Exception("Failed to set DOF limits in backend")

    def set_dof_stiffnesses(self, data, indices):
        """ Sets the degrees of freedom (DOF) drive stiffnesses for articulations indicated by indices.

            .. note::
                * The function raises an exception if the DOF stiffnesses cannot be set in the backend.
                * Value would be set even if no drive is present, but it would have no effect on the dynamics.
                * The sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_stiffnesses` can be used to get the current stiffnesses and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF stiffnesses with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the stiffnesses for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> dof_stiffnesses_np = numpy.zeros([articulation_view.count, articulation_view.max_dofs]) # Create an array with the expected shape
                  >>> dof_stiffnesses_np[0, 1] = 100.0 # Modify the stiffness of the drive applied to the DOF 1 of articulation 0
                  >>> dof_stiffnesses_wp = warp.from_numpy(dof_stiffnesses_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_dof_stiffnesses(dof_stiffnesses_wp, all_indices) # Set the new DOF stiffnesses
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_stiffnesses(data_desc, indices_desc):
            raise Exception("Failed to set DOF stiffnesses in backend")

    def set_dof_dampings(self, data, indices):
        """ Sets the degrees of freedom (DOF) drive dampings for articulations indicated by indices.

            .. note::
                * The function raises an exception if the DOF dampings cannot be set in the backend.
                * Value would be set even if no drive is present, but it would have no effect on the dynamics.
                * The sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_dampings` can be used to get the current dampings and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF dampings with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the dampings for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> dof_dampings_np = numpy.zeros([articulation_view.count, articulation_view.max_dofs]) # Create an array with the expected shape
                  >>> dof_dampings_np[0, 1] = 200.0 # Modify the damping of the drive applied to the DOF 1 of articulation 0
                  >>> dof_dampings_wp = warp.from_numpy(dof_dampings_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_dof_dampings(dof_dampings_wp, all_indices) # Set the new DOF dampings
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_dampings(data_desc, indices_desc):
            raise Exception("Failed to set DOF dampings in backend")

    def set_dof_max_forces(self, data, indices):
        """ Sets the degrees of freedom (DOF) maximum forces applied by the drive for articulations indicated by indices.

            .. note::
                * The function raises an exception if the DOF maximum forces cannot be set in the backend.
                * Value would be set even if no drive is present, but it would have no effect on the dynamics.
                * The sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_max_forces` can be used to get the current maximum forces and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF maximum forces with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the maximum forces for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> dof_max_forces_np = numpy.zeros([articulation_view.count, articulation_view.max_dofs]) # Create an array with the expected shape
                  >>> dof_max_forces_np[0, 1] = 10000.0 # Modify the maximum force of the drive applied to the DOF 1 of articulation 0
                  >>> dof_max_forces_wp = warp.from_numpy(dof_max_forces_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_dof_max_forces(dof_max_forces_wp, all_indices) # Set the new DOF maximum forces
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_max_forces(data_desc, indices_desc):
            raise Exception("Failed to set DOF max forces in backend")

    def set_dof_drive_model_properties(self, data, indices):
        """ Sets the degrees of freedom (DOF) drive model properties for articulations indicated by indices.

            The drive model properties tensor is composed of the following properties (provided in the order of appearance in the array):

            * speed effort gradient
            * max actuator velocity
            * velocity dependent resistance

            The ``drive model properties`` and the ``maxForce`` parameter define the **performance envelope** of a motor. This envelope acts as a static model that constrains the behavior of an articulated joint, ensuring it operates within physically achievable limits.
            The performance envelope is enforced through two key constraints:

            1. **Effort constraint**:

            .. math::

                |driveEffort| \leq maxForce - velocityDependentResistance \cdot |jointVelocity|

            2. **Velocity constraint**:

            .. math::

                |jointVelocity| \leq maxActuatorVelocity - speedEffortGradient \cdot |driveEffort|

            Importantly, ``driveEffort`` refers to the sum of internally computed drive effort (force or torque) and user-applied joint force or torque.


            .. note::
                * The function raises an exception if the DOF drive model properties cannot be set in the backend.
                * The sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_drive_model_properties` can be used to get the current drive model properties and modify them before setting them back.
                * Value would be set even if no drive is present, but it would have no effect on the dynamics.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF drive model properties with shape (count, max_dofs, 3) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations. The 3 elements of the last dimension are the speed effort gradient, max actuator velocity, and velocity dependent resistance respectively.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the drive model properties for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)    
                  >>> dof_drive_model_properties_np = numpy.zeros([articulation_view.count, articulation_view.max_dofs, 3]) # Create an array with the expected shape
                  >>> dof_drive_model_properties_np[0, 1, 0] = 100.0 # Modify the speed effort gradient of the drive applied to the DOF 1 of articulation 0
                  >>> dof_drive_model_properties_np[0, 1, 1] = 200.0 # Modify the max actuator velocity of the drive applied to the DOF 1 of articulation 0
                  >>> dof_drive_model_properties_np[0, 1, 2] = 0.1 # Modify the velocity dependent resistance of the drive applied to the DOF 1 of articulation 0
                  >>> dof_drive_model_properties_wp = warp.from_numpy(dof_drive_model_properties_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_dof_drive_model_properties(dof_drive_model_properties_wp, all_indices) # Set the new DOF drive model properties
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_drive_model_properties(data_desc, indices_desc):
            raise Exception("Failed to set DOF drive model properties in backend")

    # DEPRECATED
    def set_dof_friction_coefficients(self, data, indices):
        """ Sets the degrees of freedom (DOF) friction coefficients for articulations indicated by indices.

            .. warning::
                This function is deprecated and will be removed in the future. Use :py:func:`ArticulationView.set_dof_friction_properties` instead.

            .. note::
                * The function raises an exception if the DOF friction coefficients cannot be set in the backend.
                * The sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_friction_coefficients` can be used to get the current friction coefficients and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF friction coefficients with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the friction coefficients for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> dof_friction_coefficients_np = numpy.zeros([articulation_view.count, articulation_view.max_dofs]) # Create an array with the expected shape
                  >>> dof_friction_coefficients_np[0, 1] = 0.1 # Modify the friction coefficient of DOF 1 of articulation 0
                  >>> dof_friction_coefficients_wp = warp.from_numpy(dof_friction_coefficients_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_dof_friction_coefficients(dof_friction_coefficients_wp, all_indices) # Set the new DOF friction coefficients
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_friction_coefficients(data_desc, indices_desc):
            raise Exception("Failed to set DOF friction coefficients in backend")

    def set_dof_friction_properties(self, data, indices):
        """ Sets the degrees of freedom (DOF) friction properties for articulations indicated by indices.

            The friction properties array should have the following properties (provided in the order of appearance in the array):

            * static friction effort
            * dynamic friction effort
            * viscous friction coefficient

            .. note::
                * The function raises an exception if the friciton properties cannot be set in the backend.
                * The sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_friction_properties` can be used to get the current friction properties and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of friction properties with shape (count, max_dofs, 3) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations. The 3 elements of the last dimension are the static friction effort, dynamic friction effort, and viscous friction coefficient respectively.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the friction properties for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> friction_properties_np = numpy.zeros([articulation_view.count, articulation_view.max_dofs, 3]) # Create an array with the expected shape
                  >>> friction_properties_np[0, 1, 0] = 0.5 # Modify the static friction effort of DOF 1 of articulation 0
                  >>> friction_properties_np[0, 2, 1] = 0.5 # Modify the dynamic friction effort of DOF 2 of articulation 0
                  >>> friction_properties_np[1, 1, 2] = 0.3 # Modify the viscous friction coefficient of DOF 1 of articulation 1
                  >>> friction_properties_wp = warp.from_numpy(friction_properties_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_dof_friction_properties(friction_properties_wp, all_indices) # Set the new DOF friction properties
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_friction_properties(data_desc, indices_desc):
            raise Exception("Failed to set articulation friction properties in backend")

    def set_dof_max_velocities(self, data, indices):
        """ Sets the degrees of freedom (DOF) maximum velocities for articulations indicated by indices.

            .. note::
                * The function raises an exception if the DOF maximum velocities cannot be set in the backend.
                * The sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_max_velocities` can be used to get the current maximum velocities and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF maximum velocities with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the maximum velocities for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> dof_max_velocities_np = numpy.zeros([articulation_view.count, articulation_view.max_dofs]) # Create an array with the expected shape
                  >>> dof_max_velocities_np[:, :] = 10.0 # Modify the maximum velocity of all DOF and all articulation
                  >>> dof_max_velocities_wp = warp.from_numpy(dof_max_velocities_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_dof_max_velocities(dof_max_velocities_wp, all_indices) # Set the new DOF maximum velocities
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_max_velocities(data_desc, indices_desc):
            raise Exception("Failed to set DOF max velocities in backend")

    def set_dof_armatures(self, data, indices):
        """ Sets the degrees of freedom (DOF) armatures for articulations indicated by indices.

            .. note::
                * The function raises an exception if the DOF armatures cannot be set in the backend.
                * The sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_armatures` can be used to get the current armatures and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF armatures with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the armatures for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> dof_armatures_np = numpy.zeros([articulation_view.count, articulation_view.max_dofs]) # Create an array with the expected shape
                  >>> dof_armatures_np[0, 1] = 0.0 # Modify the armature of DOF 1 of articulation 0
                  >>> dof_armatures_wp = warp.from_numpy(dof_armatures_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_dof_armatures(dof_armatures_wp, all_indices) # Set the new DOF armatures
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_armatures(data_desc, indices_desc):
            raise Exception("Failed to set DOF armatures in backend")

    def get_link_transforms(self):
        """ Gets the link transforms for all articulations in the view.

            .. note::
                * The function raises an exception if the link transforms cannot be obtained from the backend.
                * The link transforms may not have been updated if :py:func:`SimulationView.update_articulations_kinematic` has not been called after setting the joint states.
                * The link transforms are given in the global frame of reference. If the environment has its own transform (see :py:func:`SimulationView.set_subspace_roots`), then the link transforms are given in the environment frame of reference.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of link transforms with shape (count, max_links, 7) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion of the transform.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_transforms = articulation_view.get_link_transforms() # Get the transform for all links and all articulations in the view
                  >>> link_transforms_np = link_transforms.numpy().reshape(articulation_view.count, articulation_view.max_links, 7) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_link_transforms"):
            self._link_transforms, self._link_transforms_desc = self._frontend.create_tensor(
                (self.count, self.max_links, 7), float32
            )

        if not self._backend.get_link_transforms(self._link_transforms_desc):
            raise Exception("Failed to get link tranforms from backend")

        return self._link_transforms

    def get_link_velocities(self):
        """ Gets the link velocities for all articulations in the view.

            .. note::
                * The function raises an exception if the link velocities cannot be obtained from the backend.
                * The link velocities are given in the global frame of reference, the linear component is reported with respect to the link's center of mass.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of link velocities with shape (count, max_links, 6) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 6 elements of the last dimension are the linear and angular velocities of the links.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_velocities = articulation_view.get_link_velocities() # Get the velocity for all links and all articulations in the view
                  >>> link_velocities_np = link_velocities.numpy().reshape(articulation_view.count, articulation_view.max_links, 6) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_link_velocities"):
            self._link_velocities, self._link_velocities_desc = self._frontend.create_tensor(
                (self.count, self.max_links, 6), float32
            )

        if not self._backend.get_link_velocities(self._link_velocities_desc):
            raise Exception("Failed to get link velocities from backend")

        return self._link_velocities
    
    def get_link_accelerations(self):
        """ Gets the link accelerations for all articulations in the view.

            .. note::
                * The function raises an exception if the link accelerations cannot be obtained from the backend.
                * The link accelerations are given in the global frame of reference, the linear component is reported with respect to the link's center of mass.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of link accelerations with shape (count, max_links, 6) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 6 elements of the last dimension are the linear and angular accelerations of the links.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_accelerations = articulation_view.get_link_accelerations() # Get the acceleration for all links and all articulations in the view
                  >>> link_accelerations_np = link_accelerations.numpy().reshape(articulation_view.count, articulation_view.max_links, 6) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_link_accelerations"):
            self._link_accelerations, self._link_accelerations_desc = self._frontend.create_tensor(
                (self.count, self.max_links, 6), float32
            )

        if not self._backend.get_link_accelerations(self._link_accelerations_desc):
            raise Exception("Failed to get link accelerations from backend")

        return self._link_accelerations
    
    def get_root_transforms(self):
        """ Gets the articulation root link transforms for all articulations in the view.

            .. note::
                * The function raises an exception if the root link transforms cannot be obtained from the backend.
                * The root transforms are given in the global frame of reference. If the environment has its own transform (see :py:func:`SimulationView.set_subspace_roots`), then the root transforms are given in the environment frame of reference.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of root link transforms with shape (count, 7) where count is the number of articulations in the view. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion of the transform.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> root_transforms = articulation_view.get_root_transforms() # Get the transform for all articulations root in the view
                  >>> root_transforms_np = root_transforms.numpy().reshape(articulation_view.count, 7) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_root_transforms"):
            self._root_transforms, self._root_transforms_desc = self._frontend.create_tensor((self.count, 7), float32)

        if not self._backend.get_root_transforms(self._root_transforms_desc):
            raise Exception("Failed to get root link transforms from backend")

        return self._root_transforms

    def get_root_velocities(self):
        """ Gets the articulation root link velocities for all articulations in the view.

            .. note::
                * The function raises an exception if the root link velocities cannot be obtained from the backend.
                * The root link velocities are given in the global frame of reference, the linear component is reported with respect to the root link's center of mass.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of root link velocities with shape (count, 6) where count is the number of articulations in the view. The 6 elements of the last dimension are the linear and angular velocities of the root link.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> root_velocities = articulation_view.get_root_velocities() # Get the velocity for all articulations root in the view
                  >>> root_velocities_np = root_velocities.numpy().reshape(articulation_view.count, 6) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_root_velocities"):
            self._root_velocities, self._root_velocities_desc = self._frontend.create_tensor((self.count, 6), float32)

        if not self._backend.get_root_velocities(self._root_velocities_desc):
            raise Exception("Failed to get root link transforms from backend")

        return self._root_velocities

    def set_root_transforms(self, data, indices):
        """ Sets the articulation root link transforms for articulations indicated by indices.

            .. note::
                * The function raises an exception if the root link transforms cannot be set in the backend.
                * The root transforms should be given in the global frame of reference. If the environment has its own transform (see :py:func:`SimulationView.set_subspace_roots`), then the root transforms should be given in the environment frame of reference.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of root link transforms with shape (count, 7) where count is the number of articulations in the view. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion of the transform.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the root link transforms for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> root_transforms = articulation_view.get_root_transforms() # Get initial transform for all articulations root in the view
                  >>> # simulate physics for a period of time and then reset the root transforms to their initial values
                  >>> all_indices = wp_utils.arange(articulation_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> articulation_view.set_root_transforms(root_transforms, all_indices) # Reset the root transforms
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_root_transforms(data_desc, indices_desc):
            raise Exception("Failed to set root link transforms in backend")

    def set_root_velocities(self, data, indices):
        """ Sets the articulation root link velocities for articulations indicated by indices.

            .. note::
                * The function raises an exception if the root link velocities cannot be set in the backend.
                * The root link velocities should be given in the global frame of reference, the linear component should be taken at the root link's center of mass.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of root link velocities with shape (count, 6) where count is the number of articulations in the view. The 6 elements of the last dimension are the linear and angular velocities of the root link.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the root link velocities for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> root_velocities_wp = wp.zeros(articulation_view.count * 6, dtype=wp.float32, device=device) # Create new root velocity array
                  >>> articulation_view.set_root_velocities(root_velocities_wp, all_indices) # Set the new root velocities
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_root_velocities(data_desc, indices_desc):
            raise Exception("Failed to set root link velocities in backend")

    def get_dof_positions(self):
        """ Gets the degrees of freedom (DOF) positions for all articulations in the view.

            .. note::
                * The function raises an exception if the DOF positions cannot be obtained from the backend.
                * The function returns the positions in radians for rotational DOFs. Note that this is different from the USD attributes which are in degrees for rotational DOFs.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF positions with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_positions = articulation_view.get_dof_positions() # Get the DOF position for all DOFs and all articulations in the view
                  >>> dof_positions_np = dof_positions.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_positions"):
            self._dof_positions, self._dof_positions_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_positions(self._dof_positions_desc):
            raise Exception("Failed to get DOF positions from backend")

        return self._dof_positions

    def get_dof_velocities(self):
        """ Gets the degrees of freedom (DOF) velocities for all articulations in the view.

            .. note::
                * The function raises an exception if the DOF velocities cannot be obtained from the backend.
                * The function returns the positions in radians/second for rotational DOFs. Note that this is different from the USD attributes which are in degrees/second for rotational DOFs.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF velocities with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_velocities = articulation_view.get_dof_velocities() # Get the DOF velocity for all DOFs and all articulations in the view
                  >>> dof_velocities_np = dof_velocities.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_velocities"):
            self._dof_velocities, self._dof_velocities_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_velocities(self._dof_velocities_desc):
            raise Exception("Failed to get DOF velocities from backend")

        return self._dof_velocities

    def set_dof_positions(self, data, indices):
        """ Sets the degrees of freedom (DOF) positions for articulations indicated by indices.

            Note that this API sets the instantaneous positions of the DOFs not the desired target positions.

            .. note::
                * The function raises an exception if the DOF positions cannot be set in the backend.
                * Sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_positions` can be used to get the current positions and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF positions with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the positions for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> dof_positions_wp = wp.zeros(articulation_view.count * articulation_view.max_dofs, dtype=wp.float32, device=device) # Create new DOF position array
                  >>> articulation_view.set_dof_positions(dof_positions_wp, all_indices) # Set the new DOF positions
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_positions(data_desc, indices_desc):
            raise Exception("Failed to set DOF positions in backend")

    def set_dof_velocities(self, data, indices):
        """ Sets the degrees of freedom (DOF) velocities for articulations indicated by indices.

            Note that this API sets the instantaneous velocities of the DOFs not the desired target velocities.

            .. note::
                * The function raises an exception if the DOF velocities cannot be set in the backend.
                * Sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_velocities` can be used to get the current velocities and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF velocities with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the velocities for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> dof_velocities_wp = wp.zeros(articulation_view.count * articulation_view.max_dofs, dtype=wp.float32, device=device) # Create new DOF velocity array
                  >>> articulation_view.set_dof_velocities(dof_velocities_wp, all_indices) # Set the new DOF velocities
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_velocities(data_desc, indices_desc):
            raise Exception("Failed to set DOF velocities in backend")

    def set_dof_actuation_forces(self, data, indices):
        """ Sets the degrees of freedom (DOF) actuation forces for articulations indicated by indices.

            Note that this function applies external forces/torques to all DOFs in the articulations and is independent of the implicit PD controller forces.

            .. note::
                * The function raises an exception if the DOF actuation forces cannot be set in the backend.
                * Sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_actuation_forces` can be used to get the current actuation forces and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF actuation forces with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the actuation forces for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> dof_actuation_forces_wp = wp.zeros(articulation_view.count * articulation_view.max_dofs, dtype=wp.float32, device=device) # Create new DOF actuation force array
                  >>> articulation_view.set_dof_actuation_forces(dof_actuation_forces_wp, all_indices) # Set the new DOF actuation forces
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_actuation_forces(data_desc, indices_desc):
            raise Exception("Failed to set DOF actuation forces in backend")

    def set_dof_position_targets(self, data, indices):
        """ Sets the degrees of freedom (DOF) position targets for articulations indicated by indices.

            Note that this API sets the desired target positions of the DOFs not the instantaneous positions.
            It may take multiple frames for the DOFs to reach the target positions depending on the stiffness and damping values of the controller.

            .. note::
                * The function raises an exception if the DOF position targets cannot be set in the backend.
                * Value would be set even if no drive is present, but it would have no effect on the dynamics.
                * Sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_position_targets` can be used to get the current position targets and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF position targets with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the position targets for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> dof_position_targets_wp = wp.ones(articulation_view.count * articulation_view.max_dofs, dtype=wp.float32, device=device) # Create new DOF position target array
                  >>> articulation_view.set_dof_position_targets(dof_position_targets_wp, all_indices) # Set the new DOF position targets
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_position_targets(data_desc, indices_desc):
            raise Exception("Failed to set DOF position targets in backend")

    def set_dof_velocity_targets(self, data, indices):
        """ Sets the degrees of freedom (DOF) velocity targets for articulations indicated by indices.

            Note that this API sets the desired target velocities of the DOFs not the instantaneous velocities.
            It may take multiple frames for the DOFs to reach the target velocities depending on the damping values of the controller.

            .. note::
                * The function raises an exception if the DOF velocity targets cannot be set in the backend.
                * Value would be set even if no drive is present, but it would have no effect on the dynamics.
                * Sparse setting of subset of DOFs within an articulation is not supported yet. :py:func:`ArticulationView.get_dof_velocity_targets` can be used to get the current velocity targets and modify them before setting them back.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of DOF velocity targets with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the velocity targets for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> dof_velocity_targets_wp = wp.ones(articulation_view.count * articulation_view.max_dofs, dtype=wp.float32, device=device) # Create new DOF position velocity array
                  >>> articulation_view.set_dof_velocity_targets(dof_velocity_targets_wp, all_indices) # Set the new DOF velocity targets
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dof_velocity_targets(data_desc, indices_desc):
            raise Exception("Failed to set DOF velocity targets in backend")

    def apply_forces_and_torques_at_position(self, force_data, torque_data, position_data, indices, is_global):
        """ Applies forces and torques at the specified positions in the articulations indicated by indices.

            Note that there are a few different ways this function can be used:

            * Not specifying the position_data will apply the forces at link transforms location.
            * Specifying the position_data will apply the forces at the specified positions at link transforms location.
            * Specifying the is_global as True will apply the forces and torques in the global frame of reference. If position_data is specified, its components are considered in the global frame of reference as well.
            * Specifying the is_global as False will apply the forces and torques in the local frame of reference of the link transforms. If position_data is specified, its components are considered in the local frame of reference as well.
            * Not specifying the force_data and torque_data won't have any effects, so at least one of them should be specified.

            .. note::
                The function raises an exception if the forces and torques cannot be applied in the backend.

            Args:
                force_data (Union[np.ndarray, torch.Tensor, wp.array]): An array of forces with shape (count, max_links, 3) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 3 elements of the last dimension are the x, y, z components of the force.
                torque_data (Union[np.ndarray, torch.Tensor, wp.array]): An array of torques with shape (count, max_links, 3) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 3 elements of the last dimension are the x, y, z components of the torque.
                position_data (Union[np.ndarray, torch.Tensor, wp.array]): An array of positions with shape (count, max_links, 3) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 3 elements of the last dimension are the x, y, z components of the applied position in the local/global frame of reference.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to apply the forces and torques for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.
                is_global (bool): A boolean flag to indicate if the forces, torques and positions are in the global frame of reference. If set to False, the forces, torques and positions are in the local frame of reference of the link transforms.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> forces_wp = wp.ones(articulation_view.count * articulation_view.max_links * 3, dtype=wp.float32, device=device) # Create new link force array
                  >>> articulation_view.apply_forces_and_torques_at_position(forces_wp, None, None, all_indices, True) # Apply forces to the links at the link transform considering the global frame of reference
        """
        force_data_desc = torque_data_desc = position_data_desc = None
        if(force_data is not None):
            force_data = self._frontend.as_contiguous_float32(force_data)
            force_data_desc = self._frontend.get_tensor_desc(force_data)
        if(torque_data is not None):
            torque_data = self._frontend.as_contiguous_float32(torque_data)
            torque_data_desc = self._frontend.get_tensor_desc(torque_data)
        if(position_data is not None):
            position_data = self._frontend.as_contiguous_float32(position_data)
            position_data_desc = self._frontend.get_tensor_desc(position_data)

        indices = self._frontend.as_contiguous_uint32(indices)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.apply_forces_and_torques_at_position(force_data_desc, torque_data_desc, position_data_desc, indices_desc, is_global):
            raise Exception("Failed to apply forces and torques at pos in backend")
        return True

    def get_dof_position_targets(self):
        """ Gets the degrees of freedom (DOF) position targets for all articulations in the view.

            .. note::
                * The function raises an exception if the DOF position targets cannot be obtained from the backend.
                * The function returns the positions in radians for rotational DOFs. Note that this is different from the USD attributes which are in degrees for rotational DOFs.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF position targets with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_position_targets = articulation_view.get_dof_position_targets() # Get the DOF position target for all DOFs and all articulations in the view
                  >>> dof_position_targets_np = dof_position_targets.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_position_targets"):
            self._dof_position_targets, self._dof_position_targets_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_position_targets(self._dof_position_targets_desc):
            raise Exception("Failed to get DOF position targets from backend")

        return self._dof_position_targets

    def get_dof_velocity_targets(self):
        """ Gets the degrees of freedom (DOF) velocity targets for all articulations in the view.

            .. note::
                * The function raises an exception if the DOF velocity targets cannot be obtained from the backend.
                * The function returns the velocities in radians/second for rotational DOFs. Note that this is different from the USD attributes which are in degrees/second for rotational DOFs.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF velocity targets with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_velocity_targets = articulation_view.get_dof_velocity_targets() # Get the DOF velocity target for all DOFs and all articulations in the view
                  >>> dof_velocity_targets_np = dof_velocity_targets.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_velocity_targets"):
            self._dof_velocity_targets, self._dof_velocity_targets_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_velocity_targets(self._dof_velocity_targets_desc):
            raise Exception("Failed to get DOF velocity targets from backend")

        return self._dof_velocity_targets

    def get_dof_actuation_forces(self):
        """ Gets the degrees of freedom (DOF) actuation forces for all articulations in the view.

            .. note::
                The function raises an exception if the DOF actuation forces cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF actuation forces with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_actuation_forces = articulation_view.get_dof_actuation_forces() # Get the DOF actuation force for all DOFs and all articulations in the view
                  >>> dof_actuation_forces_np = dof_actuation_forces.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_actuation_forces"):
            self._dof_actuation_forces, self._dof_actuation_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_actuation_forces(self._dof_actuation_forces_desc):
            raise Exception("Failed to get DOF actuation forces from backend")

        return self._dof_actuation_forces

    def get_dof_projected_joint_forces(self):
        """ Gets the degrees of freedom (DOF) projected joint forces for all articulations in the view.

            Note that this function projects the links incoming joint forces in the motion direction and hence is the active component of the force.
            To get the total 6D joint forces, including the active and passive components of the force, use :py:func:`ArticulationView.get_link_incoming_joint_force`.

            .. note::
                The function raises an exception if the DOF projected joint forces cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of DOF projected joint forces with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> dof_projected_joint_forces = articulation_view.get_dof_projected_joint_forces() # Get the DOF projected joint force for all DOFs and all articulations in the view
                  >>> dof_projected_joint_forces_np = dof_projected_joint_forces.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_dof_projected_joint_forces"):
            self._dof_projected_joint_forces, self._dof_projected_joint_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if not self._backend.get_dof_projected_joint_forces(self._dof_projected_joint_forces_desc):
            raise Exception("Failed to get dof projected joint forces from backend")

        return self._dof_projected_joint_forces

    def get_jacobians(self):
        """ Gets the Jacobian matrix for all articulations in the view.

            The Jacobian matrix maps the degrees of freedom (DOF) velocities to the link velocities in the global frame of reference.

            .. note::
                * The function raises an exception if the Jacobians cannot be obtained from the backend.
                * The size of the Jacobian is ((numLinks - 1) * 6) x max_dofs for fixed base articulations and (numLinks * 6) x (max_dofs + 6) for floating base articulations.
                * For floating base articulations, the root link velocity is expected to be in the global frame of reference.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of Jacobians with shape (count, jacobian_shape.numCols, jacobian_shape.numRows) where count is the number of articulations in the view and jacobian_shape.numCols and jacobian_shape.numRows are the number of columns and rows of the Jacobian matrix.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> jacobians = articulation_view.get_jacobians() # Get the Jacobian for all articulations in the view
                  >>> jacobians_np = jacobians.numpy().reshape(articulation_view.count, (articulation_view.max_links - 1) * 6, articulation_view.max_dofs) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_jacobians"):
            jshape = self.jacobian_shape
            self._jacobians, self._jacobians_desc = self._frontend.create_tensor(
                # (self.count, jshape[0], jshape[1]), float32
                (self.count, jshape[0] // 6, 6, jshape[1]),
                float32,
            )

        if not self._backend.get_jacobians(self._jacobians_desc):
            raise Exception("Failed to get Jacobians from backend")

        return self._jacobians

    # DEPRECATED
    def get_mass_matrices(self):
        """ Gets the mass matrices for all articulations in the view.

            This matrix represents the joint space inertia of the articulation and can be used to convert joint accelerations into joint forces/torques.

            .. warning::
                This function is deprecated and will be removed in the future. Use :py:func:`ArticulationView.get_generalized_mass_matrices` instead.

            .. note::
                * The function raises an exception if the mass matrices cannot be obtained from the backend.
                * For floating base articulations, the mass matrix provided does not allow to convert root accelerations into root forces/torques.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of mass matrices with shape (count, mass_matrix_shape.numCols, mass_matrix_shape.numRows) where count is the number of articulations in the view and mass_matrix_shape.numCols and mass_matrix_shape.numRows are the number of columns and rows of the mass matrix.
        """
        if not hasattr(self, "_mass_matrices"):
            shape = self.mass_matrix_shape
            self._mass_matrices, self._mass_matrices_desc = self._frontend.create_tensor(
                # (self.count, shape[0], shape[1]), float32
                (self.count, shape[0], shape[1]),
                float32,
            )

        if not self._backend.get_mass_matrices(self._mass_matrices_desc):
            raise Exception("Failed to get Mass Matrices from backend")

        return self._mass_matrices

    def get_generalized_mass_matrices(self):
        """ Gets the mass matrices for all articulations in the view.

            This matrix represents the joint space inertia of the articulation and can be used to convert joint accelerations into joint forces/torques.

            .. note::
                * The function raises an exception if the generalized mass matrices cannot be obtained from the backend.
                * The size of the mass matrix is max_dofs x max_dofs for fixed base articulations and (max_dofs + 6) x (max_dofs + 6) for floating base articulations.
                * For floating base articulations, the mass matrix can also be used to convert root accelerations into root forces/torques. In that case, the root properties are expected to be in the global frame of reference.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of mass matrices with shape (count, generalized_mass_matrix_shape.numCols, generalized_mass_matrix_shape.numRows) where count is the number of articulations in the view and generalized_mass_matrix_shape.numCols and generalized_mass_matrix_shape.numRows are the number of columns and rows of the mass matrix.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> mass_matrices = articulation_view.get_generalized_mass_matrices() # Get the mass matrix for all articulations in the view
                  >>> mass_matrices_np = mass_matrices.numpy().reshape(articulation_view.count, articulation_view.max_dofs, articulation_view.max_dofs) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_generalized_mass_matrices"):
            shape = self.generalized_mass_matrix_shape
            self._generalized_mass_matrices, self._generalized_mass_matrices_desc = self._frontend.create_tensor(
                (self.count, shape[0], shape[1]),
                float32,
            )

        if not self._backend.get_generalized_mass_matrices(self._generalized_mass_matrices_desc):
            raise Exception("Failed to get Mass Matrices from backend")

        return self._generalized_mass_matrices

    # DEPRECATED
    def get_coriolis_and_centrifugal_forces(self):
        """ Gets the Coriolis and Centrifugal forces for all articulations in the view.

            .. warning::
                This function is deprecated and will be removed in the future. Use :py:func:`ArticulationView.get_coriolis_and_centrifugal_compensation_forces` instead.

            .. note::
                * The function raises an exception if the Coriolis and Centrifugal forces cannot be obtained from the backend.
                * For floating base articulations, this API does not provide the Coriolis and Centrifugal forces acting on the root.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of Coriolis and Centrifugal forces with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
        """
        if not hasattr(self, "_coriolis_centrifugal"):
            self._coriolis_centrifugal, self._coriolis_centrifugal_desc = self._frontend.create_tensor(
                # (self.count, self.max_dofs), float32
                (self.count, self.max_dofs),
                float32,
            )

        if not self._backend.get_coriolis_and_centrifugal_forces(self._coriolis_centrifugal_desc):
            raise Exception("Failed to get Coriolis and Centrifugal forces from backend")

        return self._coriolis_centrifugal

    def get_coriolis_and_centrifugal_compensation_forces(self):
        """ Gets the Coriolis and Centrifugal compensation forces for all articulations in the view.

            .. note::
                * The function raises an exception if the Coriolis and Centrifugal compensation forces cannot be obtained from the backend.
                * For floating base articulations, the Coriolis and Centrifugal forces acting on the root is also provided. In that case, the root properties are expected to be in the global frame of reference.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of Coriolis and Centrifugal compensation forces with shape (count, max_dofs) for fixed based articulations, and (count, max_dofs + 6) for floating base articulatons where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> coriolis_and_centrifugal_compensation_forces = articulation_view.get_coriolis_and_centrifugal_compensation_forces() # Get the Coriolis and centrifugal compensation force for all articulations in the view
                  >>> coriolis_and_centrifugal_compensation_forces_np = coriolis_and_centrifugal_compensation_forces.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_coriolis_compensation_forces"):
            if self.shared_metatype.fixed_base:
                coriolisForceSize = self.max_dofs
            else:
                coriolisForceSize = self.max_dofs + 6
            self._coriolis_compensation_forces, self._coriolis_compensation_forces_desc = self._frontend.create_tensor(
                (self.count, coriolisForceSize),
                float32,
            )

        if not self._backend.get_coriolis_and_centrifugal_compensation_forces(self._coriolis_compensation_forces_desc):
            raise Exception("Failed to get Coriolis and Centrifugal compensation forces from backend")

        return self._coriolis_compensation_forces

    # DEPRECATED
    def get_generalized_gravity_forces(self):
        """ Gets the Generalized Gravity forces for all articulations in the view.

            .. warning::
                This function is deprecated and will be removed in the future. Use :py:func:`ArticulationView.get_gravity_compensation_forces` instead.

            .. note::
                * The function raises an exception if the Generalized Gravity forces cannot be obtained from the backend.
                * For floating base articulations, this API does not provide the Generalized Gravity forces acting on the root.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of Generalized Gravity forces with shape (count, max_dofs) where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.
        """
        if not hasattr(self, "_generalized_gravity"):
            self._generalized_gravity, self._generalized_gravity_desc = self._frontend.create_tensor(
                # (self.count, self.max_dofs), float32
                (self.count, self.max_dofs),
                float32,
            )

        if not self._backend.get_generalized_gravity_forces(self._generalized_gravity_desc):
            raise Exception("Failed to get Generalized Gravity forces from backend")

        return self._generalized_gravity

    def get_gravity_compensation_forces(self):
        """ Gets the Gravity compensation forces for all articulations in the view.

            .. note::
                * The function raises an exception if the Gravity compensation forces cannot be obtained from the backend.
                * For floating base articulations, the Gravity compensation forces of the root is also provided. In that case, the root properties are expected to be in the global frame of reference.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of Gravity compensation forces with shape (count, max_dofs) for fixed based articulations, and (count, max_dofs + 6) for floating base articulatons where count is the number of articulations in the view and max_dofs is the maximum number of degrees of freedom in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> gravity_compensation_forces = articulation_view.get_gravity_compensation_forces() # Get the gravity compensation force for all articulations in the view
                  >>> gravity_compensation_forces_np = gravity_compensation_forces.numpy().reshape(articulation_view.count, articulation_view.max_dofs) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_gravity_compensation_forces"):
            if self.shared_metatype.fixed_base:
                gravityForceSize = self.max_dofs
            else:
                gravityForceSize = self.max_dofs + 6
            self._gravity_compensation_forces, self._gravity_compensation_forces_desc = self._frontend.create_tensor(
                (self.count, gravityForceSize),
                float32,
            )

        if not self._backend.get_gravity_compensation_forces(self._gravity_compensation_forces_desc):
            raise Exception("Failed to get Gravity compensation forces from backend")

        return self._gravity_compensation_forces

    def get_articulation_mass_center(self, local_frame=False):
        """ Gets the center of mass for all articulations in the view.

            .. note::
                The function raises an exception if the center of mass cannot be obtained from the backend.

            Args:
                local_frame (bool): A boolean flag to indicate if the center of mass is in the local frame of reference of the articulations. If set to False, the center of mass is in the global frame of reference. Default to False.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of center of mass with shape (count, 3) where count is the number of articulations in the view. The 3 elements of the last dimension are the x, y, z components of the center of mass.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> articulation_COMs = articulation_view.get_articulation_mass_center(False) # Get the center of mass in the global frame for all articulations in the view
                  >>> articulation_COMs_np = articulation_COMs.numpy().reshape(articulation_view.count, 3) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_articulation_mass_center"):
            self._articulation_mass_center, self._articulation_mass_center_desc = self._frontend.create_tensor((self.count, 3), float32)

        if not self._backend.get_articulation_mass_center(self._articulation_mass_center_desc, local_frame):
            raise Exception("Failed to get articulation mass centers from backend")

        return self._articulation_mass_center
    
    def get_articulation_centroidal_momentum(self):
        """ Gets the centroidal momentum for all articulations in the view.

            The centroidal momentum matrix allows mapping of the articulation centroidal momentum to velocities.
            The bias force is a second-order term that allows for better estimation of the derivative of the centroidal momentum.

            .. note::
                * The function raises an exception if the centroidal momentum cannot be obtained from the backend.
                * This function is only implemented for floating-base articulations.
                * The velocity of the root link is expected to be given in the global frame of reference and the obtained centroidal momentum will also be in the global frame of reference.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]:
                  An array of centroidal momentum data with shape (count, 6, max_dofs + 7), where count is the number of articulations in the view.
                  The first max_dofs + 6 columns of the last dimension are the centroidal momentum matrix (i.e. a 6 x (max_dofs + 6) matrix for each articulation) and the last column's 6 elements are the components of the centroidal momentum bias force (i.e. a 6 x 1 vector for each articulation).

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Ant_*") # This assumes that the prims referenced by "/World/Ant_*" were already created in the stage
                  >>> temp_buffer = articulation_view.get_articulation_centroidal_momentum() # Get the centroidal momentum matrix and the bias force for all articulations in the view
                  >>> temp_buffer_np = temp_buffer.numpy().reshape(articulation_view.count, 6, articulation_view.max_dofs + 7) # Reshape the obtained array in a 3D numpy array on the host
                  >>> centroidal_momentum_matrices_np = temp_buffer[:, :, :-1] # Extract the centroidal momentum matrices from the results
                  >>> bias_forces_np = temp_buffer[:, :, -1:] # Extract the bias force from the results
        """
        if not hasattr(self, "_articulation_centroidal_momentum"):
            # 6 x (maxDofs + 6) for centroidal momentum matrix, and 6 x 1 for centroidal momentum bias force
            self._articulation_centroidal_momentum, self._articulation_centroidal_momentum_desc = self._frontend.create_tensor((self.count, 6, (self.max_dofs + 7)), float32)

        if not self._backend.get_articulation_centroidal_momentum(self._articulation_centroidal_momentum_desc):
            raise Exception("Failed to get articulation centroidal momentum from backend")

        return self._articulation_centroidal_momentum
    
    def get_link_incoming_joint_force(self):
        """ Gets the link incoming joint forces for all articulation in the view.

            In a kinematic tree, each link has a single incoming joint and this function provides the total 6D force/torque of links incoming joints.
            The total 6D force is given in the child joint frame.

            .. note::
                The function raises an exception if the incoming joint forces cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of incoming joint forces with shape (count, max_links, 6) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 6 elements of the last dimensions are the forces and torques of the link incoming joint represented in the joints' child frames.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_incoming_joint_forces = articulation_view.get_link_incoming_joint_force() # Get the incoming joint force for all links and all articulations in the view
                  >>> link_incoming_joint_forces_np = link_incoming_joint_forces.numpy().reshape(articulation_view.count, articulation_view.max_links, 6) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_link_incoming_joint_force"):
            self._link_incoming_joint_force, self._link_incoming_joint_force_desc = self._frontend.create_tensor((self.count, self.max_links, 6), float32)

        if not self._backend.get_link_incoming_joint_force(self._link_incoming_joint_force_desc):
            raise Exception("Failed to get incoming link joint forces from backend")

        return self._link_incoming_joint_force

    def get_masses(self):
        """ Gets the link masses for all articulation in the view.

            .. note::
                The function raises an exception if the masses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of masses with shape (count, max_links) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_masses = articulation_view.get_masses() # Get the mass for all links and all articulations in the view
                  >>> link_masses_np = link_masses.numpy().reshape(articulation_view.count, articulation_view.max_links) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_masses"):
            self._masses, self._masses_desc = self._frontend.create_tensor((self.count, self.max_links), float32, -1)

        if not self._backend.get_masses(self._masses_desc):
            raise Exception("Failed to get articulation masses from backend")

        return self._masses

    def get_inv_masses(self):
        """ Gets the link inverse masses for all articulation in the view.

            .. note::
                The function raises an exception if the inverse masses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of inverse masses with shape (count, max_links) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_inv_masses = articulation_view.get_inv_masses() # Get the inverse mass for all links and all articulations in the view
                  >>> link_inv_masses_np = link_inv_masses.numpy().reshape(articulation_view.count, articulation_view.max_links) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_inv_masses"):
            self._inv_masses, self._inv_masses_desc = self._frontend.create_tensor((self.count, self.max_links), float32, -1)

        if not self._backend.get_inv_masses(self._inv_masses_desc):
            raise Exception("Failed to get articulation inv masses from backend")

        return self._inv_masses

    def get_coms(self):
        """ Gets the pose of the principal-axes frame relative to the rigid-body-prim frame for articulations in the view.

            .. note::
                * The function raises an exception if the centers of mass cannot be obtained from the backend.
                * The poses are given in the rigid-body-prim frames.
                * Using set_inertias in-between a set_coms and get_coms call may update the principal axes rotation, see note in set_coms.


            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of centers of mass with shape (count, max_links, 7) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the center-of-mass translation and principal-axes frame rotation.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_coms = articulation_view.get_coms() # Get the poses of the principal-axes frames for all links and all articulations in the view
                  >>> link_coms_np = link_coms.numpy().reshape(articulation_view.count, articulation_view.max_links, 7) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_coms"):
            self._coms, self._com_desc = self._frontend.create_tensor((self.count, self.max_links, 7), float32, -1)

        if not self._backend.get_coms(self._com_desc):
            raise Exception("Failed to get articulation coms from backend")

        return self._coms

    def get_inertias(self):
        """ Gets the link inertia tensors for all articulations in the view.

            .. note::
                * The function raises an exception if the inertias cannot be obtained from the backend.
                * The inertia tensors are given at the center of mass, expressed in the rigid-body-prim frame.


            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of inertias with shape (count, max_links, 9) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 9 elements of the last dimension are the column-major components of the inertia tensors.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_inertias = articulation_view.get_inertias() # Get the inertia tensor for all links and all articulations in the view
                  >>> link_inertias_np = link_inertias.numpy().reshape(articulation_view.count, articulation_view.max_links, 9) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_inertias"):
            self._inertias, self._inertias_desc = self._frontend.create_tensor((self.count, self.max_links, 9), float32, -1)

        if not self._backend.get_inertias(self._inertias_desc):
            raise Exception("Failed to get articulation inertias from backend")

        return self._inertias

    def get_inv_inertias(self):
        """ Gets the link inverse inertia tensors for all articulations in the view.

            .. note::
                * The function raises an exception if the inverse inertias cannot be obtained from the backend.
                * The inverse inertia tensors are given in the rigid-body-prim frames and with respect to the center of mass.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of inverse inertias with shape (count, max_links, 9) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 9 elements of the last dimension are the column-major components of the inverse inertia tensors.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_inv_inertias = articulation_view.get_inv_inertias() # Get the inverse inertia tensor for all links and all articulations in the view
                  >>> link_inv_inertias_np = link_inv_inertias.numpy().reshape(articulation_view.count, articulation_view.max_links, 9) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_inv_inertias"):
            self._inv_inertias, self._inv_inertias_desc = self._frontend.create_tensor((self.count, self.max_links, 9), float32, -1)

        if not self._backend.get_inv_inertias(self._inv_inertias_desc):
            raise Exception("Failed to get articulation inv inertias from backend")

        return self._inv_inertias

    def get_disable_gravities(self):
        """ Receives whether the gravity is activated on articulation links.

            .. note::
                The function raises an exception if the disable gravities cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of disable gravities with shape (count, max_links) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> link_disable_gravities = articulation_view.get_disable_gravities() # Get an array of boolean indicating whether the gravity is activated for all links and all articulations in the view
                  >>> link_disable_gravities_np = link_disable_gravities.numpy().reshape(articulation_view.count, articulation_view.max_links) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_disable_gravities"):
            self._disable_gravities, self._disable_gravities_desc = self._frontend.create_tensor((self.count, self.max_links), uint8, -1)

        if not self._backend.get_disable_gravities(self._disable_gravities_desc):
            raise Exception("Failed to get rigid body disable gravities from backend")

        return self._disable_gravities

    def set_masses(self, data, indices):
        """ Sets the link masses for articulations indicated by indices.

            .. note::
                * The function raises an exception if the masses cannot be set in the backend.
                * The sparse setting of subset of link masses is not supported. :py:func:`ArticulationView.get_masses` can be used to get the current masses and modify them before setting the masses, or use :py:class:`RigidBodyView` functionality instead.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of masses with shape (count, max_links) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the masses for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> link_masses_np = numpy.zeros([articulation_view.count, articulation_view.max_links]) # Create an array with the expected shape
                  >>> link_masses_np[0, 1] = 7.5 # Modify the mass of link 1 of articulation 0
                  >>> link_masses_wp = warp.from_numpy(link_masses_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_masses(link_masses_wp, all_indices) # Set the new link masses
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_masses(data_desc, indices_desc):
            raise Exception("Failed to set articulation masses in backend")

    def set_coms(self, data, indices):
        """ Sets the pose of the links' principal-axes frames for articulations indicated by indices.

            .. note::
                * The function raises an exception if the centers of mass cannot be set in the backend.
                * The sparse setting of subset of link data is not supported yet. :py:func:`ArticulationView.get_coms` can be used to get the current centers of mass poses and modify them before setting the centers of mass for the subset of links, or use :py:class:`RigidBodyView` functionality instead.
                * The principal-axes frame poses are relative to and expressed in the rigid-body-prim frame of the link.
                * Using set_inertias after this call may update the principal axes rotation.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of principal-axes-frames poses with shape (count, max_links, 7) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the centers of mass for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> link_coms_np = numpy.zeros([articulation_view.count, articulation_view.max_links, 7]) # Create an array with the expected shape
                  >>> link_coms_np[0, 1, 2] = 0.3 # Modify the position in the Z direction of the center of mass of link 1 of articulation 0
                  >>> link_coms_wp = warp.from_numpy(link_coms_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_coms(link_coms_wp, all_indices) # Set the new poses
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_coms(data_desc, indices_desc):
            raise Exception("Failed to set articulation coms in backend")

    def set_inertias(self, data, indices):
        """ Sets the link inertia tensors for articulations indicated by indices.

            .. note::
                * The function raises an exception if the inertias cannot be set in the backend.
                * The sparse setting of subset of link data is not supported yet. :py:func:`ArticulationView.get_inertias` can be used to get the current inertias and modify them before setting the inertias, or use :py:class:`RigidBodyView` functionality instead.
                * The inertia tensor should be given with respect to the center of mass, expressed in the rigid-body-prim frame.
                * get_coms can be used to retrieve the principal-axes frame rotation computed from the diagonalized inertia tensor.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of inertia tensors with shape (count, max_links, 9) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations. The 9 elements of the last dimension are the column-major components of the inertia tensors.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the inertias for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> link_inertias_np = numpy.zeros([articulation_view.count, articulation_view.max_links, 9]) # Create an array with the expected shape
                  >>> link_inertias_np[0, 1, 4] = 70.0 # Modify the inertia tensor Iyy element of link 1 of articulation 0
                  >>> link_inertias_wp = warp.from_numpy(link_inertias_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_inertias(link_inertias_wp, all_indices) # Set the new link inertia tensors
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_inertias(data_desc, indices_desc):
            raise Exception("Failed to set articulation inertias in backend")

    def set_disable_gravities(self, data, indices):
        """ Sets the links gravity activation flag for articulations indicated by indices.

            .. note::
                * The function raises an exception if the disable gravities cannot be set in the backend.
                * The sparse setting of subset of link data is not supported yet. :py:func:`ArticulationView.get_disable_gravities` can be used to get the current gravity flags and modify them before setting the gravity flags, or use :py:class:`RigidBodyView` functionality instead.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of gravity activation flags (1 for disabled and 0 for enabled) with shape (count, max_links) where count is the number of articulations in the view and max_links is the maximum number of links in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the disable gravities for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> link_disable_gravities_np = numpy.ones([articulation_view.count, articulation_view.max_links]) # Create an array with the expected shape
                  >>> link_disable_gravities_np[0, 1] = False # Disable the gravity for link 1 of articulation 0
                  >>> link_disable_gravities_wp = warp.from_numpy(link_disable_gravities_np, dtype=warp.uint8, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_disable_gravities(link_disable_gravities_wp, all_indices) # Set whether the gravity is enabled or not for all links
        """
        data = self._frontend.as_contiguous_uint8(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_disable_gravities(data_desc, indices_desc):
            raise Exception("Failed to set rigid body disable gravities in backend")

    def get_material_properties(self):
        """ Gets the material properties for all shapes in the view.

            The material properties array is composed of the following properties (provided in the order of appearance in the array):

            * static friction
            * dynamic friction
            * restitution

            .. note::
                The function raises an exception if the material properties cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of material properties with shape (count, max_shapes, 3) where count is the number of articulations in the view and max_shapes is the maximum number of shapes in all the view's articulations. The 3 elements of the last dimension are the static friction, dynamic friction, and restitution respectively.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> material_properties = articulation_view.get_material_properties() # Get the material properties for all shapes in the view
                  >>> material_properties_np = material_properties.numpy().reshape(articulation_view.count, articulation_view.max_shapes, 3) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_material_properties"):
            self._material_properties, self._material_properties_desc = self._frontend.create_tensor((self.count, self.max_shapes, 3), float32, -1)

        if not self._backend.get_material_properties(self._material_properties_desc):
            raise Exception("Failed to get articulation material properties from backend")

        return self._material_properties

    def get_contact_offsets(self):
        """ Gets the contact offsets for all shapes in the view.

            .. note::
                The function raises an exception if the contact offsets cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of contact offsets with shape (count, max_shapes) where count is the number of articulations in the view and max_shapes is the maximum number of shapes in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> contact_offsets = articulation_view.get_contact_offsets() # Get the contact offset for all shapes in the view
                  >>> contact_offsets_np = contact_offsets.numpy().reshape(articulation_view.count, articulation_view.max_shapes) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_contact_offsets"):
            self._contact_offsets, self._contact_offsets_desc = self._frontend.create_tensor((self.count, self.max_shapes), float32, -1)

        if not self._backend.get_contact_offsets(self._contact_offsets_desc):
            raise Exception("Failed to get articulation contact offsets from backend")

        return self._contact_offsets

    def get_rest_offsets(self):
        """ Gets the rest offsets for all shapes in the view.

            .. note::
                The function raises an exception if the rest offsets cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of rest offsets with shape (count, max_shapes) where count is the number of articulations in the view and max_shapes is the maximum number of shapes in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> rest_offsets = articulation_view.get_rest_offsets() # Get the rest offset for all shapes in the view
                  >>> rest_offsets_np = rest_offsets.numpy().reshape(articulation_view.count, articulation_view.max_shapes) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_rest_offsets"):
            self._rest_offsets, self._rest_offsets_desc = self._frontend.create_tensor((self.count, self.max_shapes), float32, -1)

        if not self._backend.get_rest_offsets(self._rest_offsets_desc):
            raise Exception("Failed to get articulation rest offsets from backend")

        return self._rest_offsets

    def set_material_properties(self, data, indices):
        """ Sets the material properties for shapes indicated by indices.

            The material properties array should have the following properties (provided in the order of appearance in the array):

            * static friction
            * dynamic friction
            * restitution

            .. note::
                * The function raises an exception if the material properties cannot be set in the backend.
                * The sparse setting of subset of shape data is not supported yet. :py:func:`ArticulationView.get_material_properties` can be used to get the current material properties and modify them before setting the material properties for the subset of shapes.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of material properties with shape (count, max_shapes, 3) where count is the number of articulations in the view and max_shapes is the maximum number of shapes in all the view's articulations. The 3 elements of the last dimension are the static friction, dynamic friction, and restitution respectively.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the material properties for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> material_properties_np = numpy.zeros([articulation_view.count, articulation_view.max_shapes, 3]) # Create an array with the expected shape
                  >>> material_properties_np[0, 1, 0] = 0.5 # Modify the static friction of shape 1 of articulation 0
                  >>> material_properties_np[0, 2, 1] = 0.5 # Modify the dynamic friction of shape 2 of articulation 0
                  >>> material_properties_np[1, 1, 2] = 0.3 # Modify the restitution of shape 1 of articulation 1
                  >>> material_properties_wp = warp.from_numpy(material_properties_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_material_properties(material_properties_wp, all_indices) # Set the new shape material properties
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_material_properties(data_desc, indices_desc):
            raise Exception("Failed to set articulation material properties in backend")

    def set_contact_offsets(self, data, indices):
        """ Sets the contact offsets for shapes indicated by indices.

            .. note::
                * The function raises an exception if the contact offsets cannot be set in the backend.
                * The sparse setting of subset of shape data is not supported yet. :py:func:`ArticulationView.get_contact_offsets` can be used to get the current contact offsets and modify them before setting the contact offsets for the subset of shapes.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of contact offsets with shape (count, max_shapes) where count is the number of articulations in the view and max_shapes is the maximum number of shapes in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the contact offsets for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> contact_offsets_np = numpy.zeros([articulation_view.count, articulation_view.max_shapes]) # Create an array with the expected shape
                  >>> contact_offsets_np[0, 1] = 0.3 # Modify the contact offset of shape 1 of articulation 0
                  >>> contact_offsets_wp = warp.from_numpy(contact_offsets_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_contact_offsets(contact_offsets_wp, all_indices) # Set the new shape contact offsets
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_contact_offsets(data_desc, indices_desc):
            raise Exception("Failed to set articulation contact offsets in backend")

    def set_rest_offsets(self, data, indices):
        """ Sets the rest offsets for shapes indicated by indices.

            .. note::
                * The function raises an exception if the rest offsets cannot be set in the backend.
                * The sparse setting of subset of shape data is not supported yet. :py:func:`ArticulationView.get_rest_offsets` can be used to get the current rest offsets and modify them before setting the rest offsets for the subset of shapes.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of rest offsets with shape (count, max_shapes) where count is the number of articulations in the view and max_shapes is the maximum number of shapes in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the rest offsets for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)
                  >>> rest_offsets_np = numpy.zeros([articulation_view.count, articulation_view.max_shapes]) # Create an array with the expected shape
                  >>> rest_offsets_np[0, 1] = 0.3 # Modify the rest offset of shape 1 of articulation 0
                  >>> rest_offsets_wp = warp.from_numpy(rest_offsets_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> articulation_view.set_rest_offsets(rest_offsets_wp, all_indices) # Set the new shape rest offsets
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_rest_offsets(data_desc, indices_desc):
            raise Exception("Failed to set articulation rest offsets in backend")

    def get_fixed_tendon_stiffnesses(self):
        """ Gets the stiffnesses for all fixed tendons in the view.

            .. note::
                The function raises an exception if the fixed tendon stiffnesses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of stiffnesses with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> fixed_tendon_stiffnesses = articulation_view.get_fixed_tendon_stiffnesses() # Get the stiffness for all fixed tendons in the view
                  >>> fixed_tendon_stiffnesses_np = fixed_tendon_stiffnesses.numpy().reshape(articulation_view.count, articulation_view.max_fixed_tendons) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "fixed_tendon_stiffnesses"):
            self.fixed_tendon_stiffnesses, self._fixed_tendon_stiffnesses_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_stiffnesses(self._fixed_tendon_stiffnesses_desc):
            raise Exception("Failed to get articulation fixed tendon stiffnesses from backend")

        return self.fixed_tendon_stiffnesses

    def get_fixed_tendon_dampings(self):
        """ Gets the dampings for all fixed tendons in the view.

            .. note::
                The function raises an exception if the fixed tendon dampings cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of dampings with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> fixed_tendon_dampings = articulation_view.get_fixed_tendon_dampings() # Get the damping for all fixed tendons in the view
                  >>> fixed_tendon_dampings_np = fixed_tendon_dampings.numpy().reshape(articulation_view.count, articulation_view.max_fixed_tendons) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "fixed_tendon_dampings"):
            self.fixed_tendon_dampings, self._fixed_tendon_dampings_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_dampings(self._fixed_tendon_dampings_desc):
            raise Exception("Failed to get articulation fixed tendon dampings from backend")

        return self.fixed_tendon_dampings

    def get_fixed_tendon_limit_stiffnesses(self):
        """ Gets the limit stiffnesses for all fixed tendons in the view.

            .. note::
                The function raises an exception if the fixed tendon limit stiffnesses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of limit stiffnesses with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> fixed_tendon_limit_stiffnesses = articulation_view.get_fixed_tendon_limit_stiffnesses() # Get the limit stiffness for all fixed tendons in the view
                  >>> fixed_tendon_limit_stiffnesses_np = fixed_tendon_limit_stiffnesses.numpy().reshape(articulation_view.count, articulation_view.max_fixed_tendons) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "fixed_tendon_limit_stiffnesses"):
            self.fixed_tendon_limit_stiffnesses, self._fixed_tendon_limit_stiffnesses_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_limit_stiffnesses(self._fixed_tendon_limit_stiffnesses_desc):
            raise Exception("Failed to get articulation fixed tendon limit stiffnesses from backend")

        return self.fixed_tendon_limit_stiffnesses

    def get_fixed_tendon_limits(self):
        """ Gets the limits for all fixed tendons in the view.

            .. note::
                The function raises an exception if the fixed tendon limits cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of limits with shape (count, max_fixed_tendons, 2) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations. The 2 elements of the last dimension are the lower and upper limits respectively.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> fixed_tendon_limits = articulation_view.get_fixed_tendon_limits() # Get the limits for all fixed tendons in the view
                  >>> fixed_tendon_limits_np = fixed_tendon_limits.numpy().reshape(articulation_view.count, articulation_view.max_fixed_tendons, 2) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "fixed_tendon_limits"):
            self.fixed_tendon_limits, self._fixed_tendon_limits_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons, 2), float32)

        if not self._backend.get_fixed_tendon_limits(self._fixed_tendon_limits_desc):
            raise Exception("Failed to get articulation fixed tendon limits from backend")

        return self.fixed_tendon_limits

    def get_fixed_tendon_rest_lengths(self):
        """ Gets the rest lengths for all fixed tendons in the view.

            .. note::
                The function raises an exception if the fixed tendon rest lengths cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of rest lengths with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> fixed_tendon_rest_lengths = articulation_view.get_fixed_tendon_rest_lengths() # Get the rest length for all fixed tendons in the view
                  >>> fixed_tendon_rest_lengths_np = fixed_tendon_rest_lengths.numpy().reshape(articulation_view.count, articulation_view.max_fixed_tendons) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "fixed_tendon_rest_lengths"):
            self.fixed_tendon_rest_lengths, self._fixed_tendon_rest_lengths_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_rest_lengths(self._fixed_tendon_rest_lengths_desc):
            raise Exception("Failed to get articulation fixed tendon rest_lengths from backend")

        return self.fixed_tendon_rest_lengths

    def get_fixed_tendon_offsets(self):
        """ Gets the offsets for all fixed tendons in the view.

            .. note::
                The function raises an exception if the fixed tendon offsets cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of offsets with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> fixed_tendon_offsets = articulation_view.get_fixed_tendon_offsets() # Get the offset for all fixed tendons in the view
                  >>> fixed_tendon_offsets_np = fixed_tendon_offsets.numpy().reshape(articulation_view.count, articulation_view.max_fixed_tendons) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "fixed_tendon_offsets"):
            self.fixed_tendon_offsets, self._fixed_tendon_offsets_desc = self._frontend.create_tensor((self.count, self.max_fixed_tendons), float32)

        if not self._backend.get_fixed_tendon_offsets(self._fixed_tendon_offsets_desc):
            raise Exception("Failed to get articulation fixed tendon offsets from backend")

        return self.fixed_tendon_offsets

    def get_spatial_tendon_stiffnesses(self):
        """ Gets the stiffnesses for all spatial tendons in the view.

            .. note::
                The function raises an exception if the spatial tendon stiffnesses cannot be obtained from the backend.

            See :cpp:class:`PhysxSchema.PhysxTendonAttachmentRootAPI <PhysxSchemaPhysxTendonAttachmentRootAPI>`, :cpp:class:`PhysxSchema.PhysxTendonAttachmentAPI <PhysxSchemaPhysxTendonAttachmentAPI>`,
            and :cpp:class:`PhysxSchema.PhysxTendonAttachmentLeafAPI <PhysxSchemaPhysxTendonAttachmentLeafAPI>` for more details on spatial tendons.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of stiffnesses with shape (count, max_spatial_tendons) where count is the number of articulations in the view and max_spatial_tendons is the maximum number of spatial tendons in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> spatial_tendon_stiffnesses = articulation_view.get_spatial_tendon_stiffnesses() # Get the stiffness for all spatial tendons in the view
                  >>> spatial_tendon_stiffnesses_np = spatial_tendon_stiffnesses.numpy().reshape(articulation_view.count, articulation_view.max_spatial_tendons) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "spatial_tendon_stiffnesses"):
            self.spatial_tendon_stiffnesses, self._spatial_tendon_stiffnesses_desc = self._frontend.create_tensor((self.count, self.max_spatial_tendons), float32)

        if not self._backend.get_spatial_tendon_stiffnesses(self._spatial_tendon_stiffnesses_desc):
            raise Exception("Failed to get articulation spatial tendon stiffnesses from backend")

        return self.spatial_tendon_stiffnesses

    def get_spatial_tendon_dampings(self):
        """ Gets the dampings for all spatial tendons in the view.

            .. note::
                The function raises an exception if the spatial tendon dampings cannot be obtained from the backend.

            See :cpp:class:`PhysxSchema.PhysxTendonAttachmentRootAPI <PhysxSchemaPhysxTendonAttachmentRootAPI>`, :cpp:class:`PhysxSchema.PhysxTendonAttachmentAPI <PhysxSchemaPhysxTendonAttachmentAPI>`,
            and :cpp:class:`PhysxSchema.PhysxTendonAttachmentLeafAPI <PhysxSchemaPhysxTendonAttachmentLeafAPI>` for more details on spatial tendons.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of dampings with shape (count, max_spatial_tendons) where count is the number of articulations in the view and max_spatial_tendons is the maximum number of spatial tendons in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> spatial_tendon_dampings = articulation_view.get_spatial_tendon_dampings() # Get the damping for all spatial tendons in the view
                  >>> spatial_tendon_dampings_np = spatial_tendon_dampings.numpy().reshape(articulation_view.count, articulation_view.max_spatial_tendons) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "spatial_tendon_dampings"):
            self.spatial_tendon_dampings, self._spatial_tendon_dampings_desc = self._frontend.create_tensor((self.count, self.max_spatial_tendons), float32)

        if not self._backend.get_spatial_tendon_dampings(self._spatial_tendon_dampings_desc):
            raise Exception("Failed to get articulation spatial tendon dampings from backend")

        return self.spatial_tendon_dampings

    def get_spatial_tendon_limit_stiffnesses(self):
        """ Gets the limit stiffnesses for all spatial tendons in the view.

            .. note::
                The function raises an exception if the spatial tendon limit stiffnesses cannot be obtained from the backend.

            See :cpp:class:`PhysxSchema.PhysxTendonAttachmentRootAPI <PhysxSchemaPhysxTendonAttachmentRootAPI>`, :cpp:class:`PhysxSchema.PhysxTendonAttachmentAPI <PhysxSchemaPhysxTendonAttachmentAPI>`,
            and :cpp:class:`PhysxSchema.PhysxTendonAttachmentLeafAPI <PhysxSchemaPhysxTendonAttachmentLeafAPI>` for more details on spatial tendons.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of limit stiffnesses with shape (count, max_spatial_tendons) where count is the number of articulations in the view and max_spatial_tendons is the maximum number of spatial tendons in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> spatial_tendon_limit_stiffnesses = articulation_view.get_spatial_tendon_limit_stiffnesses() # Get the limit stiffness for all spatial tendons in the view
                  >>> spatial_tendon_limit_stiffnesses_np = spatial_tendon_limit_stiffnesses.numpy().reshape(articulation_view.count, articulation_view.max_spatial_tendons) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "spatial_tendon_limit_stiffnesses"):
            self.spatial_tendon_limit_stiffnesses, self._spatial_tendon_limit_stiffnesses_desc = self._frontend.create_tensor((self.count, self.max_spatial_tendons), float32)

        if not self._backend.get_spatial_tendon_limit_stiffnesses(self._spatial_tendon_limit_stiffnesses_desc):
            raise Exception("Failed to get articulation spatial tendon limit stiffnesses from backend")

        return self.spatial_tendon_limit_stiffnesses

    def get_spatial_tendon_offsets(self):
        """ Gets the offsets for all spatial tendons in the view.

            .. note::
                The function raises an exception if the spatial tendon offsets cannot be obtained from the backend.

            See :cpp:class:`PhysxSchema.PhysxTendonAttachmentRootAPI <PhysxSchemaPhysxTendonAttachmentRootAPI>`, :cpp:class:`PhysxSchema.PhysxTendonAttachmentAPI <PhysxSchemaPhysxTendonAttachmentAPI>`,
            and :cpp:class:`PhysxSchema.PhysxTendonAttachmentLeafAPI <PhysxSchemaPhysxTendonAttachmentLeafAPI>` for more details on spatial tendons.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of offsets with shape (count, max_spatial_tendons) where count is the number of articulations in the view and max_spatial_tendons is the maximum number of spatial tendons in all the view's articulations.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> spatial_tendon_offsets = articulation_view.get_spatial_tendon_offsets() # Get the offset for all spatial tendons in the view
                  >>> spatial_tendon_offsets_np = spatial_tendon_offsets.numpy().reshape(articulation_view.count, articulation_view.max_spatial_tendons) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "spatial_tendon_offsets"):
            self.spatial_tendon_offsets, self._spatial_tendon_offsets_desc = self._frontend.create_tensor((self.count, self.max_spatial_tendons), float32)

        if not self._backend.get_spatial_tendon_offsets(self._spatial_tendon_offsets_desc):
            raise Exception("Failed to get articulation spatial tendon offsets from backend")

        return self.spatial_tendon_offsets

    def set_fixed_tendon_properties(self, stiffnesses, dampings, limit_stiffnesses, limits, rest_lengths, offsets, indices):
        """ Sets the fixed tendon properties for fixed tendons indicated by indices.

            .. note::
                * The function raises an exception if the fixed tendon properties cannot be set in the backend.
                * All the input arrays should be provided for this API to set properties correctly.
                * The sparse setting of only some but not all the fixed tendon properties are not supported.
                * :py:func:`ArticulationView.get_fixed_tendon_stiffnesses`, :py:func:`ArticulationView.get_fixed_tendon_dampings`, :py:func:`ArticulationView.get_fixed_tendon_limit_stiffnesses`, :py:func:`ArticulationView.get_fixed_tendon_limits`, :py:func:`ArticulationView.get_fixed_tendon_rest_lengths`, and :py:func:`ArticulationView.get_fixed_tendon_offsets` functions can be used to get the current properties and modify them (as necessary) before setting the properties for the subset of tendons.

            Args:
                stiffnesses (Union[np.ndarray, torch.Tensor, wp.array]): An array of stiffnesses with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.
                dampings (Union[np.ndarray, torch.Tensor, wp.array]): An array of dampings with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.
                limit_stiffnesses (Union[np.ndarray, torch.Tensor, wp.array]): An array of limit stiffnesses with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.
                limits (Union[np.ndarray, torch.Tensor, wp.array]): An array of limits with shape (count, max_fixed_tendons, 2) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations. The 2 elements of the last dimension are the lower and upper limits respectively.
                rest_lengths (Union[np.ndarray, torch.Tensor, wp.array]): An array of rest lengths with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.
                offsets (Union[np.ndarray, torch.Tensor, wp.array]): An array of offsets with shape (count, max_fixed_tendons) where count is the number of articulations in the view and max_fixed_tendons is the maximum number of fixed tendons in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the fixed tendon properties for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)

                  >>> fixed_tendon_stiffnesses_np = numpy.zeros([articulation_view.count, articulation_view.max_fixed_tendons]) # Create an array with the expected shape
                  >>> fixed_tendon_dampings_np = numpy.zeros([articulation_view.count, articulation_view.max_fixed_tendons]) # Create an array with the expected shape
                  >>> fixed_tendon_limit_stiffnesses_np = numpy.zeros([articulation_view.count, articulation_view.max_fixed_tendons]) # Create an array with the expected shape
                  >>> fixed_tendon_limits_np = numpy.zeros([articulation_view.count, articulation_view.max_fixed_tendons, 2]) # Create an array with the expected shape
                  >>> fixed_tendon_rest_lengths_np = numpy.zeros([articulation_view.count, articulation_view.max_fixed_tendons]) # Create an array with the expected shape
                  >>> fixed_tendon_offsets_np = numpy.zeros([articulation_view.count, articulation_view.max_fixed_tendons]) # Create an array with the expected shape

                  >>> fixed_tendon_stiffnesses_np[0, 1] = 50.0 # Modify the stiffness of fixed tendon 1 of articulation 0
                  >>> fixed_tendon_dampings_np[1, 1] = 200.0 # Modify the damping of fixed tendon 1 of articulation 1
                  >>> fixed_tendon_limit_stiffnesses_np[2, 1] = 100.0 # Modify the limit stiffness of fixed tendon 1 of articulation 2
                  >>> fixed_tendon_limits_np[0, 0, 1] = 0.5 # Modify the upper limit of fixed tendon 0 of articulation 0
                  >>> fixed_tendon_rest_lengths_np[0, 2] -= 0.2 # Modify the rest length of fixed tendon 2 of articulation 0
                  >>> fixed_tendon_offsets_np[0, 3] += 0.1 # Modify the offset of fixed tendon 3 of articulation 0

                  >>> fixed_tendon_stiffnesses_wp = warp.from_numpy(fixed_tendon_stiffnesses_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> fixed_tendon_dampings_wp = warp.from_numpy(fixed_tendon_dampings_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> fixed_tendon_limit_stiffnesses_wp = warp.from_numpy(fixed_tendon_limit_stiffnesses_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> fixed_tendon_limits_wp = warp.from_numpy(fixed_tendon_limits_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> fixed_tendon_rest_lengths_wp = warp.from_numpy(fixed_tendon_rest_lengths_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> fixed_tendon_offsets_wp = warp.from_numpy(fixed_tendon_offsets_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API

                  >>> articulation_view.set_fixed_tendon_properties(fixed_tendon_stiffnesses_wp, fixed_tendon_dampings_wp, fixed_tendon_limit_stiffnesses_wp, fixed_tendon_limits_wp, fixed_tendon_rest_lengths_wp, fixed_tendon_offsets_wp, all_indices) # Set the new fixed tendon properties
        """
        stiffnesses = self._frontend.as_contiguous_float32(stiffnesses)
        stiffness_desc = self._frontend.get_tensor_desc(stiffnesses)
        dampings = self._frontend.as_contiguous_float32(dampings)
        damping_desc = self._frontend.get_tensor_desc(dampings)
        limit_stiffnesses = self._frontend.as_contiguous_float32(limit_stiffnesses)
        limit_stiffness_desc = self._frontend.get_tensor_desc(limit_stiffnesses)
        limits = self._frontend.as_contiguous_float32(limits)
        limits_desc = self._frontend.get_tensor_desc(limits)
        rest_lengths = self._frontend.as_contiguous_float32(rest_lengths)
        rest_length_desc = self._frontend.get_tensor_desc(rest_lengths)
        offsets = self._frontend.as_contiguous_float32(offsets)
        offset_desc = self._frontend.get_tensor_desc(offsets)

        indices = self._frontend.as_contiguous_uint32(indices)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_fixed_tendon_properties(stiffness_desc, damping_desc, limit_stiffness_desc, limits_desc, rest_length_desc, offset_desc, indices_desc):
            raise Exception("Failed to set articulation fixed tendon properties in backend")

    def set_spatial_tendon_properties(self, stiffnesses, dampings, limit_stiffnesses, offsets, indices):
        """ Sets the spatial tendon properties for spatial tendons indicated by indices.

            .. note::
                * The function raises an exception if the spatial tendon properties cannot be set in the backend.
                * All the input arrays should be provided for this API to set properties correctly.
                * The sparse setting of only some but not all the spatial tendon properties are not supported.
                * :py:func:`ArticulationView.get_spatial_tendon_stiffnesses`, :py:func:`ArticulationView.get_spatial_tendon_dampings`, :py:func:`ArticulationView.get_spatial_tendon_limit_stiffnesses`, and :py:func:`ArticulationView.get_spatial_tendon_offsets` functions can be used to get the current properties and modify them (as necessary) before setting the properties for the subset of tendons.

            See :cpp:class:`PhysxSchema.PhysxTendonAttachmentRootAPI <PhysxSchemaPhysxTendonAttachmentRootAPI>`, :cpp:class:`PhysxSchema.PhysxTendonAttachmentAPI <PhysxSchemaPhysxTendonAttachmentAPI>`,
            and :cpp:class:`PhysxSchema.PhysxTendonAttachmentLeafAPI <PhysxSchemaPhysxTendonAttachmentLeafAPI>` for more details on spatial tendons.

            Args:
                stiffnesses (Union[np.ndarray, torch.Tensor, wp.array]): An array of stiffnesses with shape (count, max_spatial_tendons) where count is the number of articulations in the view and max_spatial_tendons is the maximum number of spatial tendons in all the view's articulations.
                dampings (Union[np.ndarray, torch.Tensor, wp.array]): An array of dampings with shape (count, max_spatial_tendons) where count is the number of articulations in the view and max_spatial_tendons is the maximum number of spatial tendons in all the view's articulations.
                limit_stiffnesses (Union[np.ndarray, torch.Tensor, wp.array]): An array of limit stiffnesses with shape (count, max_spatial_tendons) where count is the number of articulations in the view and max_spatial_tendons is the maximum number of spatial tendons in all the view's articulations.
                offsets (Union[np.ndarray, torch.Tensor, wp.array]): An array of offsets with shape (count, max_spatial_tendons) where count is the number of articulations in the view and max_spatial_tendons is the maximum number of spatial tendons in all the view's articulations.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of articulations to set the spatial tendon properties for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of articulations in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(articulation_view.count)

                  >>> spatial_tendon_stiffnesses_np = numpy.zeros([articulation_view.count, articulation_view.max_spatial_tendons]) # Create an array with the expected shape
                  >>> spatial_tendon_dampings_np = numpy.zeros([articulation_view.count, articulation_view.max_spatial_tendons]) # Create an array with the expected shape
                  >>> spatial_tendon_limit_stiffnesses_np = numpy.zeros([articulation_view.count, articulation_view.max_spatial_tendons]) # Create an array with the expected shape
                  >>> spatial_tendon_offsets_np = numpy.zeros([articulation_view.count, articulation_view.max_spatial_tendons]) # Create an array with the expected shape

                  >>> spatial_tendon_stiffnesses_np[0, 1] = 50.0 # Modify the stiffness of spatial tendon 1 of articulation 0
                  >>> spatial_tendon_dampings_np[1, 1] = 200.0 # Modify the damping of spatial tendon 1 of articulation 1
                  >>> spatial_tendon_limit_stiffnesses_np[2, 1] = 100.0 # Modify the limit stiffness of spatial tendon 1 of articulation 2
                  >>> spatial_tendon_offsets_np[0, 3] = 0.1 # Modify the offset of spatial tendon 3 of articulation 0

                  >>> spatial_tendon_stiffnesses_wp = warp.from_numpy(spatial_tendon_stiffnesses_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> spatial_tendon_dampings_wp = warp.from_numpy(spatial_tendon_dampings_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> spatial_tendon_limit_stiffnesses_wp = warp.from_numpy(spatial_tendon_limit_stiffnesses_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> spatial_tendon_offsets_wp = warp.from_numpy(spatial_tendon_offsets_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API

                  >>> articulation_view.set_spatial_tendon_properties(spatial_tendon_stiffnesses_wp, spatial_tendon_dampings_wp, spatial_tendon_limit_stiffnesses_wp, spatial_tendon_offsets_wp, all_indices) # Set the new spatial tendon properties
        """
        stiffnesses = self._frontend.as_contiguous_float32(stiffnesses)
        stiffness_desc = self._frontend.get_tensor_desc(stiffnesses)
        dampings = self._frontend.as_contiguous_float32(dampings)
        damping_desc = self._frontend.get_tensor_desc(dampings)
        limit_stiffnesses = self._frontend.as_contiguous_float32(limit_stiffnesses)
        limit_stiffness_desc = self._frontend.get_tensor_desc(limit_stiffnesses)
        offsets = self._frontend.as_contiguous_float32(offsets)
        offset_desc = self._frontend.get_tensor_desc(offsets)

        indices = self._frontend.as_contiguous_uint32(indices)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_spatial_tendon_properties(stiffness_desc, damping_desc, limit_stiffness_desc, offset_desc, indices_desc):
            raise Exception("Failed to set articulation spatial tendon properties in backend")

    def check(self):
        """ Checks the validity of the underlying physics objects that the view uses.

            Returns:
                bool: True if all physics objects are valid, False otherwise.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> articulation_view = sim_view.create_articulation_view("/World/Franka_*") # This assumes that the prims referenced by "/World/Franka_*" were already created in the stage
                  >>> articulation_view.check() # returns False if articulation_view is invalid, True otherwise
        """
        return self._backend.check()


class RigidBodyView:
    """ RigidBodyView class represents a batch of rigid objects.

        RigidBodyView binds the concrete implementation of the physics backend with the frontend tensor framework that is used to handle data.
        This class isn't meant to be instantiated directly, but rather created using the :py:func:`SimulationView.create_rigid_body_view` method of the :py:class:`SimulationView` object
        which manages all the physics views, the device where the simulation is performed and data resides.

        .. note::
            A rigid object can be either an articulation link or an individual rigid body.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("warp")
              >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
    """
    def __init__(self, backend, frontend):
        """ Constructs a RigidBodyView object with the given backend object and frontend framework.

            Currently, the only backend tensor is PhysX tensor.
            Several frontend frameworks are available: torch, numpy, tensorflow, warp.

            Args:
                backend (IRigidBodyView): The concrete implementation of the IRigidBodyView interface.
                frontend (FrontendBase): The tensor framework that is used to handle data which could be FrontendNumpy, FrontendTorch, FrontendTensorflow, or FrontendWarp.
        """
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        """
            The number of rigid objects (articulation links and rigid bodies) in the view.
        """
        return self._backend.count

    @property
    def max_shapes(self):
        """
            The maximum number of shapes in all the rigid objects in the view.
        """
        return self._backend.max_shapes

    @property
    def prim_paths(self):
        """
            The USD paths for all the rigid objects in the view.
        """
        return self._backend.prim_paths

    def get_transforms(self):
        """ Gets the transforms for all rigid objects in the view.

            .. note::
                * The function raises an exception if the transforms cannot be obtained from the backend.
                * The transforms are given in the global frame of reference. If the environment has its own transform (see :py:func:`SimulationView.set_subspace_roots`), then the transforms are given in the environment frame of reference.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of transforms with shape (count, 7) where count is the number of rigid objects in the view. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion of the transform.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_transforms = rigid_body_view.get_transforms() # Get the transform for all rigid bodies in the view
                  >>> rb_transforms_np = rb_transforms.numpy().reshape(rigid_body_view.count, 7) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_transforms"):
            self._transforms, self._transforms_desc = self._frontend.create_tensor((self.count, 7), float32)

        if not self._backend.get_transforms(self._transforms_desc):
            raise Exception("Failed to get rigid body transforms from backend")

        return self._transforms

    def get_velocities(self):
        """ Gets the velocities for all rigid objects in the view.

            .. note::
                * The function raises an exception if the velocities cannot be obtained from the backend.
                * The velocities are given in the global frame of reference, the linear component is reported with respect to the rigid body's center of mass.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of velocities with shape (count, 6) where count is the number of rigid objects in the view. The 6 elements of the last dimension are the linear and angular velocities respectively.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_velocities = rigid_body_view.get_velocities() # Get the velocity for all rigid bodies in the view
                  >>> rb_velocities_np = rb_velocities.numpy().reshape(rigid_body_view.count, 6) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_velocities"):
            self._velocities, self._velocities_desc = self._frontend.create_tensor((self.count, 6), float32)

        if not self._backend.get_velocities(self._velocities_desc):
            raise Exception("Failed to get rigid body velocities from backend")

        return self._velocities

    def get_accelerations(self):
        """ Gets the accelerations for all rigid objects in the view.

            .. note::
                * The function raises an exception if the accelerations cannot be obtained from the backend.
                * The velocities are given in the global frame of reference, the linear component is reported with respect to the rigid body's center of mass.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of accelerations with shape (count, 6) where count is the number of rigid objects in the view. The 6 elements of the last dimension are the linear and angular accelerations respectively.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_accelerations = rigid_body_view.get_accelerations() # Get the acceleration for all rigid bodies in the view
                  >>> rb_accelerations_np = rb_accelerations.numpy().reshape(rigid_body_view.count, 6) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_accelerations"):
            self._accelerations, self._accelerations_desc = self._frontend.create_tensor((self.count, 6), float32)

        if not self._backend.get_accelerations(self._accelerations_desc):
            raise Exception("Failed to get rigid body accelerations from backend")

        return self._accelerations

    def set_kinematic_targets(self, data, indices):
        """ Sets the kinematic targets for rigid objects indicated by indices.

            .. note::
                * The function raises an exception if the kinematic targets cannot be set in the backend.
                * The function does not modify the tranforms of dynamic rigid bodies or articulation links (as they are simulated by the solver), and can return warning when encountering dynamic rigid bodies in the view.
                * Correctly setting the kinematic targets may require creating a view of only kinematic rigid bodies rather than using an existing view that combines both kinematic and dynamic rigid bodies.
                * The kinematic targets should be given in the global frame of reference. If the environment has its own transform (see :py:func:`SimulationView.set_subspace_roots`), then the kinematic targets should be given in the environment frame of reference.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of transforms with shape (count, 7) where count is the number of rigid objects in the view. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion of the transform.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the kinematic targets for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> kinematic_rigid_body_view = sim_view.create_rigid_body_view("/World/KinematicCube_*") # This assumes that the prims referenced by "/World/KinematicCube_*" were already created in the stage
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*")
                  >>> rb_transforms = rigid_body_view.get_transforms() # Get the transform for all rigid bodies in the view
                  >>> all_indices = wp_utils.arange(kinematic_rigid_body_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> kinematic_rigid_body_view.set_kinematic_targets(rb_transforms, all_indices) # Set the kinematic targets as the cube position
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_kinematic_targets(data_desc, indices_desc):
            raise Exception("Failed to set rigid body kinematic targets in backend")

    def set_transforms(self, data, indices):
        """ Sets the transforms for rigid objects indicated by indices.

            .. note::
                * The function raises an exception if the transforms cannot be set in the backend.
                * Note that setting only the translation or only the rotation part of the transform is not supported yet. :py:func:`RigidBodyView.get_transforms` can be used to get the current transforms and modify them before setting back the transforms for the subset of rigid objects.
                * The transforms should be given in the global frame of reference. If the environment has its own transform (see :py:func:`SimulationView.set_subspace_roots`), then the transforms should be given in the environment frame of reference.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of transforms with shape (count, 7) where count is the number of rigid objects in the view. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion of the transform.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the transforms for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_transforms = rigid_body_view.get_transforms() # Get the transform for all rigid bodies in the view
                  >>> # simulate physics for a period of time and then reset the transforms to their initial values
                  >>> all_indices = wp_utils.arange(rigid_body_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> rigid_body_view.set_transforms(rb_transforms, all_indices) # Reset the rigid body transforms
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_transforms(data_desc, indices_desc):
            raise Exception("Failed to set rigid body transforms in backend")

    def set_velocities(self, data, indices):
        """ Sets the velocities for rigid objects indicated by indices.

            .. note::
                * The function raises an exception if the velocities cannot be set in the backend.
                * Note that setting only the linear or only the angular part of the velocity is not supported yet. :py:func:`RigidBodyView.get_velocities` can be used to get the current velocities and modify them before setting back the velocities for the subset of rigid objects.
                * The velocities should be given in the global frame of reference, the linear component should be taken at the rigid body's center of mass.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of velocities with shape (count, 6) where count is the number of rigid objects in the view. The 6 elements of the last dimension are the linear and angular velocities respectively.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the velocities for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> rb_velocities_wp = wp.zeros(rigid_body_view.count * 6, dtype=wp.float32, device=device) # Create new rigid body velocity array
                  >>> rigid_body_view.set_velocities(rb_velocities_wp, all_indices) # Reset the rigid body velocities
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_velocities(data_desc, indices_desc):
            raise Exception("Failed to set rigid body velocities in backend")

    def apply_forces_and_torques_at_position(self, force_data, torque_data, position_data, indices, is_global):
        """ Applies forces and torques at the specified positions in the rigid objects indicated by indices.

            Note that there are a few different ways this function can be used:

            * Not specifying the position_data will apply the forces at link transforms location.
            * Specifying the position_data will apply the forces at the specified positions at link transforms location.
            * Specifying the is_global as True will apply the forces and torques in the global frame of reference. If position_data is specified, its components are considered in the global frame of reference as well.
            * Specifying the is_global as False will apply the forces and torques in the local frame of reference of the link transforms. If position_data is specified, its components are considered in the local frame of reference as well.
            * Not specifying the force_data and torque_data won't have any effects, so at least one of them should be specified.

            .. note::
                The function raises an exception if the forces and torques cannot be applied in the backend.

            Args:
                force_data (Union[np.ndarray, torch.Tensor, wp.array]): An array of forces with shape (count, 3) where count is the number of rigid objects in the view. The 3 elements of the last dimension are the x, y, z components of the force.
                torque_data (Union[np.ndarray, torch.Tensor, wp.array]): An array of torques with shape (count, 3) where count is the number of rigid objects in the view. The 3 elements of the last dimension are the x, y, z components of the torque.
                position_data (Union[np.ndarray, torch.Tensor, wp.array]): An array of positions with shape (count, 3) where count is the number of rigid objects in the view. The 3 elements of the last dimension are the x, y, z components of the applied position in the local/global frame of reference.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to apply the forces and torques for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.
                is_global (bool): A boolean flag to indicate if the forces, torques and positions are in the global frame of reference. If set to False, the forces, torques and positions are in the local frame of reference of the link transforms.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> forces_wp = wp.ones(rigid_body_view.count * 3, dtype=wp.float32, device=device) # Create new rigid body force array
                  >>> rigid_body_view.apply_forces_and_torques_at_position(forces_wp, None, None, all_indices, True) # Apply forces to the rigid body transform considering the global frame of reference
        """
        force_data_desc = torque_data_desc = position_data_desc = None
        if(force_data is not None):
            force_data = self._frontend.as_contiguous_float32(force_data)
            force_data_desc = self._frontend.get_tensor_desc(force_data)
        if(torque_data is not None):
            torque_data = self._frontend.as_contiguous_float32(torque_data)
            torque_data_desc = self._frontend.get_tensor_desc(torque_data)
        if(position_data is not None):
            position_data = self._frontend.as_contiguous_float32(position_data)
            position_data_desc = self._frontend.get_tensor_desc(position_data)

        indices = self._frontend.as_contiguous_uint32(indices)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.apply_forces_and_torques_at_position(force_data_desc, torque_data_desc, position_data_desc, indices_desc, is_global):
            raise Exception("Failed to apply forces and torques at pos in backend")
        return True

    def apply_forces(self, data, indices, is_global=True):
        """ Applies only forces at the link transforms of the rigid objects indicated by indices.

            .. note::
                The function raises an exception if the forces cannot be applied in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of forces with shape (count, 3) where count is the number of rigid objects in the view. The 3 elements of the last dimension are the x, y, z components of the force in either local or global frame of reference depending on the is_global flag.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to apply the forces for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.
                is_global (bool): A boolean flag to indicate if the forces are in the global frame of reference. If set to False, the forces are in the local frame of reference of the link transforms. Default is True.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> forces_wp = wp.ones(rigid_body_view.count * 3, dtype=wp.float32, device=device) # Create new rigid body force array
                  >>> rigid_body_view.apply_forces(forces_wp, all_indices, True) # Apply forces to the rigid body transform considering the global frame of reference
        """
        if not self.apply_forces_and_torques_at_position(data, None, None, indices, is_global):
            raise Exception("Failed to apply forces in backend")

    def get_masses(self):
        """ Gets the masses for all rigid objects in the view.

            .. note::
                The function raises an exception if the masses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of masses with shape (count, 1) where count is the number of rigid objects in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_masses = rigid_body_view.get_masses() # Get the mass for all rigid bodies in the view
        """
        if not hasattr(self, "_masses"):
            self._masses, self._masses_desc = self._frontend.create_tensor((self.count, 1), float32, -1)

        if not self._backend.get_masses(self._masses_desc):
            raise Exception("Failed to get rigid body masses from backend")

        return self._masses

    def get_inv_masses(self):
        """ Gets the inverse masses for all rigid objects in the view.

            .. note::
                The function raises an exception if the inverse masses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of inverse masses with shape (count, 1) where count is the number of rigid objects in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_inv_masses = rigid_body_view.get_inv_masses() # Get the inverse mass for all rigid bodies in the view
        """
        if not hasattr(self, "_inv_masses"):
            self._inv_masses, self._inv_masses_desc = self._frontend.create_tensor((self.count, 1), float32, -1)

        if not self._backend.get_inv_masses(self._inv_masses_desc):
            raise Exception("Failed to get rigid body inv masses from backend")

        return self._inv_masses

    def get_coms(self):
        """ Gets the pose of the principal-axes frame relative to the rigid-body-prim frame for all rigid objects in the view.

            .. note::
                * The function raises an exception if the centers of mass cannot be obtained from the backend.
                * The centers-of-mass translation and principal axes rotation are given in the rigid-body-prim frame.
                * The principal axes rotations returned by this function may not be consistent with the principal axes previously set by set_coms if the user calls set_inertias in between a set_coms  and get_coms call, see the note in set_coms.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of centers of mass with shape (count, 7) where count is the number of rigid objects in the view. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_coms = rigid_body_view.get_coms() # Get the transform of the center of mass for all rigid bodies in the view
                  >>> rb_coms_np = rb_coms.numpy().reshape(rigid_body_view.count, 3) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_coms"):
            self._coms, self._com_desc = self._frontend.create_tensor((self.count, 7), float32, -1)

        if not self._backend.get_coms(self._com_desc):
            raise Exception("Failed to get rigid body coms from backend")

        return self._coms

    def get_inertias(self):
        """ Gets the inertia tensors for all rigid bodies in the view.

            .. note::
                * The function raises an exception if the inertias cannot be obtained from the backend.
                * The inertia tensors are given at the center of mass, expressed in the rigid-body-prim frame.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of inertias with shape (count, 9) where count is the number of rigid objects in the view. The 9 elements of the last dimension are the column-major components of the inertia tensors.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_inertias = rigid_body_view.get_inertias() # Get the inertia tensor for all rigid bodies in the view
                  >>> rb_inertias_np = rb_inertias.numpy().reshape(rigid_body_view.count, 9) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_inertias"):
            self._inertias, self._inertias_desc = self._frontend.create_tensor((self.count, 9), float32, -1)

        if not self._backend.get_inertias(self._inertias_desc):
            raise Exception("Failed to get rigid body inertias from backend")

        return self._inertias

    def get_inv_inertias(self):
        """ Gets the inverse inertia tensors for all rigid objects in the view.

            .. note::
                * The function raises an exception if the inverse inertias cannot be obtained from the backend.
                * The inverse inertia tensors are given in the rigid-body-prim frames and with respect to the center of mass.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of inverse inertias with shape (count, 9) where count is the number of rigid objects in the view. The 9 elements of the last dimension are the column-major components of the inverse inertia tensors.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_inv_inertias = rigid_body_view.get_inv_inertias() # Get the inverse inertia tensor for all rigid bodies in the view
                  >>> rb_inv_inertias_np = rb_inv_inertias.numpy().reshape(rigid_body_view.count, 9) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_inv_inertias"):
            self._inv_inertias, self._inv_inertias_desc = self._frontend.create_tensor((self.count, 9), float32, -1)

        if not self._backend.get_inv_inertias(self._inv_inertias_desc):
            raise Exception("Failed to get rigid body inv inertias from backend")

        return self._inv_inertias

    def get_disable_gravities(self):
        """ Receives whether the gravity is activated on rigid objects.

            .. note::
                The function raises an exception if the disable gravities cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of gravity activation flags (1 for disabled and 0 for enabled) with shape (count, 1) where count is the number of rigid objects in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_disable_gravities = rigid_body_view.get_disable_gravities() # Get an array of boolean indicating whether the gravity is activated for all rigid bodies in the view
        """
        if not hasattr(self, "_disable_gravities"):
            self._disable_gravities, self._disable_gravities_desc = self._frontend.create_tensor((self.count, 1), uint8, -1)

        if not self._backend.get_disable_gravities(self._disable_gravities_desc):
            raise Exception("Failed to get rigid body disable gravities from backend")

        return self._disable_gravities

    def get_disable_simulations(self):
        """ Receives whether rigid objects are simulated or not.

            .. note::
                The function raises an exception if the disable gravities cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation flags (1 for non-simulated and 0 for simulated) with shape (count, 1) where count is the number of rigid objects in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_disable_simulations = rigid_body_view.get_disable_simulations() # Get an array of boolean indicating whether the simulation is activated for all rigid bodies in the view
        """
        if not hasattr(self, "_disable_simulations"):
            self._disable_simulations, self._disable_simulations_desc = self._frontend.create_tensor((self.count, 1), uint8, -1)

        if not self._backend.get_disable_simulations(self._disable_simulations_desc):
            raise Exception("Failed to get rigid body disable simulations from backend")

        return self._disable_simulations

    def set_masses(self, data, indices):
        """ Sets the masses for rigid objects indicated by indices.

            .. note::
                The function raises an exception if the masses cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of masses with shape (count, 1) where count is the number of rigid objects in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the masses for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count)
                  >>> rb_masses_np = numpy.zeros(rigid_body_view.count) # Create an array with the expected shape
                  >>> rb_masses_np[0] = 7.5 # Modify the mass of rigid body 0
                  >>> rb_masses_wp = warp.from_numpy(rb_masses_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> rigid_body_view.set_masses(rb_masses_wp, all_indices) # Set the new rigid bodies mass
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_masses(data_desc, indices_desc):
            raise Exception("Failed to set rigid body masses in backend")

    def set_coms(self, data, indices):
        """ Sets the pose of the principal-axes frames for rigid objects indicated by indices.

            .. note::
                * The function raises an exception if the centers of mass cannot be set in the backend.
                * The principal-axes frame poses are relative to and expressed in the rigid-body-prim frame.
                * Using set_inertias after this call may update the principal axes rotation.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of principal-axes-frames poses with shape (count, 7) where count is the number of rigid objects in the view. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the principal-axes frames for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count)
                  >>> rb_coms_np = numpy.zeros([rigid_body_view.count, 7]) # Create an array with the expected shape
                  >>> rb_coms_np[0, 2] = 0.2 # Modify the position in the Z direction of the center of mass of rigid body 0
                  >>> rb_coms_wp = warp.from_numpy(rb_coms_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> rigid_body_view.set_coms(rb_coms_wp, all_indices) # Set the new poses
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_coms(data_desc, indices_desc):
            raise Exception("Failed to set rigid body coms in backend")

    def set_inertias(self, data, indices):
        """ Sets the inertia tensors for rigid objects indicated by indices.

            .. note::
                * The function raises an exception if the inertias cannot be set in the backend.
                * A value of 0 in an element is interpreted as infinite inertia along that axis, however this is only permitted for rigid bodies and not for articulation links.
                * The inertia tensor should be given with respect to the center of mass, expressed in the rigid-body-prim frame.
                * get_coms may be used to retrieve the principal-axes frame rotation computed from the diagonalized inertia tensor.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of inertia tensors with shape (count, 9) where count is the number of rigid objects in the view. The 9 elements of the last dimension are the column-major components of the inertia tensors.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the inertias for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count)
                  >>> rb_inertias_np = numpy.zeros([rigid_body_view.count, 9]) # Create an array with the expected shape
                  >>> rb_inertias_np[0, 4] = 75.0 # Modify the inertia tensor Iyy element of rigid body 0
                  >>> rb_inertias_wp = warp.from_numpy(rb_inertias_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> rigid_body_view.set_inertias(rb_inertias_wp, all_indices) # Set the new inertia tensors
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_inertias(data_desc, indices_desc):
            raise Exception("Failed to set rigid body inertias in backend")

    def set_disable_gravities(self, data, indices):
        """ Sets the rigid objects gravity activation flag for objects indicated by indices.

            .. note::
                The function raises an exception if the disable gravities cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of gravity activation flags (1 for disabled and 0 for enabled) with shape (count, 1) where count is the number of rigid objects in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the disable gravities for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count)
                  >>> rb_disable_gravities_np = numpy.zeros(rigid_body_view.count) # Create an array with the expected shape
                  >>> rb_disable_gravities_np[:] = False # Disable the gravity for all rigid bodies
                  >>> rb_disable_gravities_wp = warp.from_numpy(rb_disable_gravities_np, dtype=warp.uint8, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> rigid_body_view.set_disable_gravities(rb_disable_gravities_wp, all_indices) # Set whether the gravity is enabled or not for all rigid bodies
        """
        data = self._frontend.as_contiguous_uint8(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_disable_gravities(data_desc, indices_desc):
            raise Exception("Failed to set rigid body disable gravities in backend")

    def set_disable_simulations(self, data, indices):
        """ Sets the rigid objects simulation activation flag for objects indicated by indices.

            .. note::
                The function raises an exception if the disable simulations cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of simulation activation flags (1 for non-simulated and 0 for simulated) with shape (count, 1) where count is the number of rigid objects in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the disable simulations for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count)
                  >>> rb_disable_simulations_np = numpy.zeros(rigid_body_view.count) # Create an array with the expected shape
                  >>> rb_disable_simulations_np[:] = False # Disable the simulation for all rigid bodies
                  >>> rb_disable_simulations_wp = warp.from_numpy(rb_disable_simulations_np, dtype=warp.uint8, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> rigid_body_view.set_disable_simulations(rb_disable_simulations_wp, all_indices) # Set whether the simulation is enabled or not for all rigid bodies
        """
        data = self._frontend.as_contiguous_uint8(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_disable_simulations(data_desc, indices_desc):
            raise Exception("Failed to set rigid body disable simulations in backend")

    def get_material_properties(self):
        """ Gets the material properties for all rigid objects in the view.

            The material properties array is composed of the following properties (provided in the order of appearance in the array):

            * static friction
            * dynamic friction
            * restitution

            .. note::
                The function raises an exception if the material properties cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of material properties with shape (count, max_shapes, 3) where count is the number of rigid objects in the view and max_shapes is the maximum number of shapes in all the rigid objects in the view. The 3 elements of the last dimension are the static friction, dynamic friction, and restitution respectively.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_material_properties = rigid_body_view.get_material_properties() # Get the material properties for all rigid bodies in the view
                  >>> rb_material_properties_np = rb_material_properties.numpy().reshape(rigid_body_view.count, rigid_body_view.max_shapes, 3) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_material_properties"):
            self._material_properties, self._material_properties_desc = self._frontend.create_tensor((self.count, self.max_shapes, 3), float32, -1)

        if not self._backend.get_material_properties(self._material_properties_desc):
            raise Exception("Failed to get rigid body material properties from backend")

        return self._material_properties

    def get_contact_offsets(self):
        """ Gets the contact offsets for all rigid objects in the view.

            .. note::
                The function raises an exception if the contact offsets cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of contact offsets with shape (count, max_shapes) where count is the number of rigid objects in the view and max_shapes is the maximum number of shapes in all the rigid objects in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_contact_offsets = rigid_body_view.get_contact_offsets() # Get the contact offsets for all rigid bodies in the view
        """
        if not hasattr(self, "_contact_offsets"):
            self._contact_offsets, self._contact_offsets_desc = self._frontend.create_tensor((self.count, self.max_shapes), float32, -1)

        if not self._backend.get_contact_offsets(self._contact_offsets_desc):
            raise Exception("Failed to get rigid body contact offsets from backend")

        return self._contact_offsets

    def get_rest_offsets(self):
        """ Gets the rest offsets for all rigid objects in the view.

            .. note::
                The function raises an exception if the rest offsets cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of rest offsets with shape (count, max_shapes) where count is the number of rigid objects in the view and max_shapes is the maximum number of shapes in all the rigid objects in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rb_rest_offsets = rigid_body_view.get_rest_offsets() # Get the rest offsets for all rigid bodies in the view
        """
        if not hasattr(self, "rest_offsets"):
            self.rest_offsets, self._rest_offsets_desc = self._frontend.create_tensor((self.count, self.max_shapes), float32, -1)

        if not self._backend.get_rest_offsets(self._rest_offsets_desc):
            raise Exception("Failed to get rigid body rest offsets from backend")

        return self.rest_offsets

    def set_material_properties(self, data, indices):
        """ Sets the material properties for rigid objects indicated by indices.

            The material properties array should have the following properties (provided in the order of appearance in the array):

            * static friction
            * dynamic friction
            * restitution

            .. note::
                The function raises an exception if the material properties cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of material properties with shape (count, max_shapes, 3) where count is the number of rigid objects in the view and max_shapes is the maximum number of shapes in all the rigid objects in the view. The 3 elements of the last dimension are the static friction, dynamic friction, and restitution respectively.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the material properties for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count)
                  >>> rb_material_properties_np = numpy.zeros([rigid_body_view.count, rigid_body_view.max_shapes, 3]) # Create an array with the expected shape
                  >>> rb_material_properties_np[:, :, 0] = 0.5 # Modify the static friction for all rigid bodies and all shapes
                  >>> rb_material_properties_np[:, :, 1] = 0.5 # Modify the dynamic friction for all rigid bodies and all shapes
                  >>> rb_material_properties_np[:, :, 2] = 0.3 # Modify the restitution for all rigid bodies and all shapes
                  >>> rb_material_properties_wp = warp.from_numpy(rb_material_properties_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> rigid_body_view.set_material_properties(rb_material_properties_wp, all_indices) # Set the material properties for all rigid bodies
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_material_properties(data_desc, indices_desc):
            raise Exception("Failed to set rigid body material properties in backend")

    def set_contact_offsets(self, data, indices):
        """ Sets the contact offsets for rigid objects indicated by indices.

            .. note::
                The function raises an exception if the contact offsets cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of contact offsets with shape (count, max_shapes) where count is the number of rigid objects in the view and max_shapes is the maximum number of shapes in all the rigid objects in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the contact offsets for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count)
                  >>> rb_contact_offsets_np = numpy.zeros(rigid_body_view.count, rigid_body_view.max_shapes) # Create an array with the expected shape
                  >>> rb_contact_offsets_np[0, 0] = 0.3 # Modify the contact offset for rigid body 0 and shape 0
                  >>> rb_contact_offsets_wp = warp.from_numpy(rb_contact_offsets_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> rigid_body_view.set_contact_offsets(rb_contact_offsets_wp, all_indices) # Set the contact offsets for all rigid bodies
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_contact_offsets(data_desc, indices_desc):
            raise Exception("Failed to set rigid body contact offsets in backend")

    def set_rest_offsets(self, data, indices):
        """ Sets the rest offsets for rigid objects indicated by indices.

            .. note::
                The function raises an exception if the rest offsets cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of rest offsets with shape (count, max_shapes) where count is the number of rigid objects in the view and max_shapes is the maximum number of shapes in all the rigid objects in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of rigid objects to set the rest offsets for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of rigid objects in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(rigid_body_view.count)
                  >>> rb_rest_offsets_np = numpy.zeros(rigid_body_view.count, rigid_body_view.max_shapes) # Create an array with the expected shape
                  >>> rb_rest_offsets_np[0, 0] = 0.3 # Modify the rest offset for rigid body 0 and shape 0
                  >>> rb_rest_offsets_wp = warp.from_numpy(rb_rest_offsets_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> rigid_body_view.set_rest_offsets(rb_rest_offsets_wp, all_indices) # Set the rest offsets for all rigid bodies
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_rest_offsets(data_desc, indices_desc):
            raise Exception("Failed to set rigid body rest offsets in backend")

    def check(self):
        """ Checks the validity of the underlying physics objects that the view uses.

            Returns:
                bool: True if all physics objects are valid, False otherwise.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_body_view = sim_view.create_rigid_body_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rigid_body_view.check() # returns False if rigid_body_view is invalid, True otherwise
        """
        return self._backend.check()

# DEPRECATED
class SoftBodyView:
    """ SoftBodyView class represents a batch of soft bodies.

        .. warning::
            This class and its functionality is deprecated and will be removed.
            See :py:class:`DeformableBodyView` as an alternative which is based on a successor deformable body feature.

        SoftBodyView binds the concrete implementation of the physics backend with the frontend tensor framework that is used to handle data.
        This class isn't meant to be instantiated directly, but rather created using the :py:func:`SimulationView.create_soft_body_view` method of the :py:class:`SimulationView` object
        which manages all the physics views, the device where the simulation is performed and data resides.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("warp")
              >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
    """
    def __init__(self, backend, frontend):
        """ Constructs a SoftBodyView object with the given backend object and frontend framework.

            Currently, the only backend tensor is PhysX tensor.
            Several frontend frameworks are available: torch, numpy, tensorflow, warp.

            Args:
                backend (ISoftBodyView): The backend object that implements the physics simulation for soft bodies.
                frontend (FrontendBase): The tensor framework that is used to handle data which could be FrontendNumpy, FrontendTorch, FrontendTensorflow, or FrontendWarp.
        """
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        """
            The number of soft bodies in the view.
        """
        return self._backend.count

    @property
    def max_elements_per_body(self):
        """
            The maximum number of elements per soft body in the view.
        """
        return self._backend.max_elements_per_body
    
    @property
    def max_vertices_per_body(self):
        """
            The maximum number of vertices per soft body in the view.
        """
        return self._backend.max_vertices_per_body

    @property
    def max_sim_elements_per_body(self):
        """
            The maximum number of simulated elements per soft body in the view.
            Simulated elements have a corresponding element in the solver while non-simulated elements are not necessarily part of the simulation mesh.
        """
        return self._backend.max_sim_elements_per_body
    
    @property
    def max_sim_vertices_per_body(self):
        """
            The maximum number of simulated vertices per soft body in the view.
            Simulated vertices have a corresponding element in the solver while non-simulated vertices are not necessarily part of the simulation mesh.
        """
        return self._backend.max_sim_vertices_per_body

    def get_element_stresses(self):
        """ Gets the collision mesh Cauchy stresses for all elements in the soft bodies in the view.

            The stresses are computed based on type of the material model being used and the deformation of the elements, where deformation depends on the deformation gradients of the elements.

            .. note::
                The function raises an exception if the element stresses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of element Cauchy stresses with shape (count, max_elements_per_body, 9) where count is the number of soft bodies in the view and max_elements_per_body is the maximum number of elements per soft body in the view. The 9 elements of the last dimension are the components of the Cauchy stress tensors.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> element_stresses = soft_body_view.get_element_stresses() # Get the Cauchy stresses for all elements of all soft bodies in the view
                  >>> element_stresses_np = element_stresses.numpy().reshape(soft_body_view.count, soft_body_view.max_elements_per_body, 9) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_element_stresses"):
            self._element_stresses, self._element_stresses_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body, 9), float32)

        if not self._backend.get_element_stresses(self._element_stresses_desc):
            raise Exception("Failed to get soft body element Cauchy stresses from backend")

        return self._element_stresses

    def get_element_deformation_gradients(self):
        """ Gets the collision mesh element-wise second-order deformation gradient tensors for the deformable bodies.

            Deformation gradient tensor is a second-order tensor that describes the deformation of a body.
            It is used to compute the stresses and strains in the body and depends on the current configuration, rotation as well as the rest configuration of the body (rest poses)

            .. note::
                The function raises an exception if the element deformation gradients cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of element deformation gradients with shape (count, max_elements_per_body, 9) where count is the number of soft bodies in the view and max_elements_per_body is the maximum number of elements per soft body in the view. The 9 elements of the last dimension are the components of the deformation gradient tensors.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> element_deformation_gradients = soft_body_view.get_element_deformation_gradients() # Get the collision mesh element-wise second-order deformation gradient tensors for all soft bodies in the view
                  >>> element_deformation_gradients_np = element_deformation_gradients.numpy().reshape(soft_body_view.count, soft_body_view.max_elements_per_body, 9) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_element_deformation_gradients"):
            self._element_deformation_gradients, self._element_deformation_gradients_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body, 9), float32)

        if not self._backend.get_element_deformation_gradients(self._element_deformation_gradients_desc):
            raise Exception("Failed to get soft body element deformation gradients from backend")

        return self._element_deformation_gradients

    def get_element_rest_poses(self):
        """ Gets the collision mesh element rest poses for the deformable bodies.

            Element-wise rest poses define the rest configuration of the elements and are used to compute the deformation gradients of the elements.

            .. note::
                The function raises an exception if the element rest poses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of element rest poses with shape (count, max_elements_per_body * 9) where count is the number of soft bodies in the view and max_elements_per_body is the maximum number of elements per soft body in the view. The 9 in the last dimension refers to the components of the rest pose tensor.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> element_rest_poses = soft_body_view.get_element_rest_poses() # Get the collision mesh element rest poses for all soft bodies in the view
                  >>> element_rest_poses_np = element_rest_poses.numpy().reshape(soft_body_view.count, soft_body_view.max_elements_per_body, 9) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_element_rest_poses"):
            self._element_rest_poses, self._element_rest_poses_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body * 9), float32)

        if not self._backend.get_element_rest_poses(self._element_rest_poses_desc):
            raise Exception("Failed to get soft body element rest poses from backend")

        return self._element_rest_poses

    def get_element_rotations(self):
        """ Gets the collision mesh element rotations for the deformable bodies.

            Element-wise rotations define the rotation of the elements and are used in computation of the deformation gradients.

            .. note::
                The function raises an exception if the element rotations cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of element rotations with shape (count, max_elements_per_body * 4) where count is the number of soft bodies in the view and max_elements_per_body is the maximum number of elements per soft body in the view. The 4 elements of the last dimension are the qx, qy, qz, qw components of the rotation quaternion.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> element_rotations = soft_body_view.get_element_rotations() # Get the collision mesh element rotations for all soft bodies in the view
                  >>> element_rotations_np = element_rotations.numpy().reshape(soft_body_view.count, soft_body_view.max_elements_per_body, 4) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_element_rotations"):
            self._element_rotations, self._element_rotations_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body * 4), float32)

        if not self._backend.get_element_rotations(self._element_rotations_desc):
            raise Exception("Failed to get soft body element rotations from backend")

        return self._element_rotations

    def get_nodal_positions(self):
        """ Gets the collision mesh nodal positions for the deformable bodies.

            .. note::
                The function raises an exception if the nodal positions cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of nodal positions with shape (count, max_vertices_per_body, 3) where count is the number of soft bodies in the view and max_vertices_per_body is the maximum number of vertices per soft body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal positions.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> nodal_positions = soft_body_view.get_nodal_positions() # Get the collision mesh nodal positions for all soft bodies in the view
                  >>> nodal_positions_np = nodal_positions.numpy().reshape(soft_body_view.count, soft_body_view.max_vertices_per_body, 3) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_nodal_positions"):
            self._nodal_positions, self._nodal_positions_desc = self._frontend.create_tensor((self.count, self.max_vertices_per_body, 3), float32)

        if not self._backend.get_nodal_positions(self._nodal_positions_desc):
            raise Exception("Failed to get soft body nodal positions from backend")

        return self._nodal_positions

    def get_element_indices(self):
        """ Gets the collision mesh element indices for the deformable bodies.

            Element indices can be used to obtain the vertices that make up the elements.

            .. note::
                The function raises an exception if the element indices cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of element indices with shape (count, max_elements_per_body, 4) where count is the number of soft bodies in the view and max_elements_per_body is the maximum number of elements per soft body in the view. The 4 elements of the last dimension are the indices of the vertices that make up the element.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> element_indices = soft_body_view.get_element_indices() # Get the collision mesh element indices for all soft bodies in the view
                  >>> element_indices_np = element_indices.numpy().reshape(soft_body_view.count, soft_body_view.max_elements_per_body, 4) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_element_indices"):
            self._element_indices, self._element_indices_desc = self._frontend.create_tensor((self.count, self.max_elements_per_body, 4), uint32)

        if not self._backend.get_element_indices(self._element_indices_desc):
            raise Exception("Failed to get soft body simulation mesh element indices from backend")

        return self._element_indices

    ### simulation mesh data
    def get_sim_element_indices(self):
        """ Gets the simulation mesh element indices for the deformable bodies.

            Element indices can be used to obtain the vertices that make up the elements.

            .. note::
                The function raises an exception if the simulation mesh element indices cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation mesh element indices with shape (count, max_sim_elements_per_body, 4) where count is the number of soft bodies in the view and max_sim_elements_per_body is the maximum number of simulated elements per soft body in the view. The 4 elements of the last dimension are the indices of the vertices that make up the element.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_element_indices = soft_body_view.get_sim_element_indices() # Get the simulation mesh element indices for all soft bodies in the view
                  >>> sim_element_indices_np = sim_element_indices.numpy().reshape(soft_body_view.count, soft_body_view.max_sim_elements_per_body, 4) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_sim_element_indices"):
            self._sim_element_indices, self._sim_element_indices_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body, 4), uint32)

        if not self._backend.get_sim_element_indices(self._sim_element_indices_desc):
            raise Exception("Failed to get soft body simulation mesh element indices from backend")

        return self._sim_element_indices

    def get_sim_element_stresses(self):
        """ Gets the simulation mesh Cauchy stresses for all elements in the soft bodies in the view.

            The stresses are computed based on type of the material model being used and the deformation of the elements, where deformation depends on the deformation gradients of the elements.

            .. note::
                The function raises an exception if the simulation mesh element stresses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation mesh element Cauchy stresses with shape (count, max_sim_elements_per_body, 9) where count is the number of soft bodies in the view and max_sim_elements_per_body is the maximum number of simulated elements per soft body in the view. The 9 elements of the last dimension are the components of the Cauchy stress tensors.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_element_stresses = soft_body_view.get_sim_element_stresses() # Get the simulation mesh Cauchy stresses for all elements of all soft bodies in the view
                  >>> sim_element_stresses_np = sim_element_stresses.numpy().reshape(soft_body_view.count, soft_body_view.max_sim_elements_per_body, 9) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_sim_element_stresses"):
            self._sim_element_stresses, self._sim_element_stresses_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body, 9), float32)

        if not self._backend.get_sim_element_stresses(self._sim_element_stresses_desc):
            raise Exception("Failed to get soft body simulation element stresses from backend")

        return self._sim_element_stresses

    def get_sim_element_deformation_gradients(self):
        """ Gets the simulation mesh element-wise second-order deformation gradient tensors for the deformable bodies.

            Deformation gradient tensor is a second-order tensor that describes the deformation of a body.
            It is used to compute the stresses and strains in the body and depends on the current configuration, rotation as well as the rest configuration of the body (rest poses)

            .. note::
                The function raises an exception if the simulation mesh element deformation gradients cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation mesh element deformation gradients with shape (count, max_sim_elements_per_body, 9) where count is the number of soft bodies in the view and max_sim_elements_per_body is the maximum number of simulated elements per soft body in the view. The 9 elements of the last dimension are the components of the deformation gradient tensors.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_element_deformation_gradients = soft_body_view.get_sim_element_deformation_gradients() # Get the simulation mesh element-wise second-order deformation gradient tensors for all soft bodies in the view
                  >>> sim_element_deformation_gradients_np = sim_element_deformation_gradients.numpy().reshape(soft_body_view.count, soft_body_view.max_sim_elements_per_body, 9) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_sim_element_deformation_gradients"):
            self._sim_element_deformation_gradients, self._sim_element_deformation_gradients_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body, 9), float32)
        if not self._backend.get_sim_element_deformation_gradients(self._sim_element_deformation_gradients_desc):
            raise Exception("Failed to get soft body simulation element deformation gradients from backend")

        return self._sim_element_deformation_gradients

    def get_sim_element_rest_poses(self):
        """ Gets the simulation mesh element rest poses for the deformable bodies.

            Element-wise rest poses define the rest configuration of the elements and are used to compute the deformation gradients of the elements.

            .. note::
                The function raises an exception if the simulation mesh element rest poses cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation mesh element rest poses with shape (count, max_sim_elements_per_body * 9) where count is the number of soft bodies in the view and max_sim_elements_per_body is the maximum number of simulated elements per soft body in the view. The 9 in the last dimension refers to the components of the rest pose tensor.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_element_rest_poses = soft_body_view.get_sim_element_rest_poses() # Get the simulation mesh element rest poses for all soft bodies in the view
                  >>> sim_element_rest_poses_np = sim_element_rest_poses.numpy().reshape(soft_body_view.count, soft_body_view.max_sim_elements_per_body, 9) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_sim_element_rest_poses"):
            self._sim_element_rest_poses, self._sim_element_rest_poses_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body * 9), float32)

        if not self._backend.get_sim_element_rest_poses(self._sim_element_rest_poses_desc):
            raise Exception("Failed to get soft body simulation element rest poses from backend")

        return self._sim_element_rest_poses

    def get_sim_element_rotations(self):
        """ Gets the simulation mesh element rotations for the deformable bodies.

            Element-wise rotations define the rotation of the elements and are used in computation of the deformation gradients.

            .. note::
                The function raises an exception if the simulation mesh element rotations cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation mesh element rotations with shape (count, max_sim_elements_per_body * 4) where count is the number of soft bodies in the view and max_sim_elements_per_body is the maximum number of simulated elements per soft body in the view. The 4 elements of the last dimension are the qx, qy, qz, qw components of the rotation quaternion.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_element_rotations = soft_body_view.get_sim_element_rotations() # Get the simulation mesh element rotations for all soft bodies in the view
                  >>> sim_element_rotations_np = sim_element_rotations.numpy().reshape(soft_body_view.count, soft_body_view.max_sim_elements_per_body, 4) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_sim_element_rotations"):
            self._sim_element_rotations, self._sim_element_rotations_desc = self._frontend.create_tensor((self.count, self.max_sim_elements_per_body * 4), float32)

        if not self._backend.get_sim_element_rotations(self._sim_element_rotations_desc):
            raise Exception("Failed to get soft body element rotations from backend")

        return self._sim_element_rotations

    def get_sim_nodal_positions(self):
        """ Gets the simulation mesh nodal positions for the deformable bodies.

            .. note::
                The function raises an exception if the simulation mesh nodal positions cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation mesh nodal positions with shape (count, max_sim_vertices_per_body, 3) where count is the number of soft bodies in the view and max_sim_vertices_per_body is the maximum number of simulated vertices per soft body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal positions

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_nodal_positions = soft_body_view.get_sim_nodal_positions() # Get the simulation mesh nodal positions for all soft bodies in the view
                  >>> sim_nodal_positions_np = sim_nodal_positions.numpy().reshape(soft_body_view.count, soft_body_view.max_sim_vertices_per_body, 3) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_sim_nodal_positions"):
            self._sim_nodal_positions, self._sim_nodal_positions_desc = self._frontend.create_tensor((self.count, self.max_sim_vertices_per_body, 3), float32)

        if not self._backend.get_sim_nodal_positions(self._sim_nodal_positions_desc):
            raise Exception("Failed to get soft body simulation mesh nodal positions from backend")

        return self._sim_nodal_positions

    def set_sim_nodal_positions(self, data, indices):
        """ Sets the simulation mesh nodal positions for the deformable bodies.

            .. note::
                The function raises an exception if the simulation mesh nodal positions cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of nodal positions with shape (count, max_sim_vertices_per_body, 3) where count is the number of soft bodies in the view and max_sim_vertices_per_body is the maximum number of simulated vertices per soft body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal positions.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of soft bodies to set the nodal positions to. The array can have any dimension (user_count, 1) as long as all indices are less than the number of soft bodies in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_nodal_positions = soft_body_view.get_sim_nodal_positions() # Get the simulation mesh nodal positions for all soft bodies in the view
                  >>> # simulate physics for a period of time and then reset the simulation mesh nodal positions to their initial values
                  >>> all_indices = wp_utils.arange(soft_body_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> soft_body_view.set_sim_nodal_positions(sim_nodal_positions, all_indices) # Reset the simulation mesh nodal positions for the soft bodies in the view
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_sim_nodal_positions(data_desc, indices_desc):
            raise Exception("Failed to set soft body simulation mesh nodal positions in backend")

    def get_sim_nodal_velocities(self):
        """ Gets the simulation mesh nodal velocities for the deformable bodies.

            .. note::
                The function raises an exception if the simulation mesh nodal velocities cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation mesh nodal velocities with shape (count, max_sim_vertices_per_body, 3) where count is the number of soft bodies in the view and max_sim_vertices_per_body is the maximum number of simulated vertices per soft body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal linear velocities.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_nodal_velocities = soft_body_view.get_sim_nodal_velocities() # Get the simulation mesh nodal velocities for all soft bodies in the view
                  >>> sim_nodal_velocities_np = sim_nodal_velocities.numpy().reshape(soft_body_view.count, soft_body_view.max_sim_vertices_per_body, 3) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_sim_nodal_velocities"):
            self._sim_nodal_velocities, self._sim_nodal_velocities_desc = self._frontend.create_tensor((self.count, self.max_sim_vertices_per_body, 3), float32)

        if not self._backend.get_sim_nodal_velocities(self._sim_nodal_velocities_desc):
            raise Exception("Failed to get soft body simulation mesh nodal velocities from backend")

        return self._sim_nodal_velocities

    def set_sim_nodal_velocities(self, data, indices):
        """ Sets the simulation mesh nodal velocities for the deformable bodies.

            .. note::
                The function raises an exception if the simulation mesh nodal velocities cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of nodal velocities with shape (count, max_sim_vertices_per_body, 3) where count is the number of soft bodies in the view and max_sim_vertices_per_body is the maximum number of simulated vertices per soft body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal linear velocities.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of soft bodies to set the nodal velocities to. The array can have any dimension (user_count, 1) as long as all indices are less than the number of soft bodies in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(soft_body_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> sim_nodal_velocities_wp = wp.zeros(soft_body_view.count * soft_body_view.max_sim_vertices_per_body * 3, dtype=wp.float32, device=device) # Create new simulation mesh nodal velocity array
                  >>> soft_body_view.set_sim_nodal_velocities(sim_nodal_velocities_wp, all_indices) # Reset the simulation mesh nodal velocities for the soft bodies in the view
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_sim_nodal_velocities(data_desc, indices_desc):
            raise Exception("Failed to set soft body simulation mesh nodal velocities in backend")

    def get_sim_kinematic_targets(self):
        """ Gets the simulation mesh kinematic targets for the deformable bodies.

            .. note::
                The function raises an exception if the simulation mesh kinematic targets cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation mesh kinematic targets with shape (count, max_sim_vertices_per_body, 4) where count is the number of soft bodies in the view and max_sim_vertices_per_body is the maximum number of simulated vertices per soft body in the view. The first 3 elements of the last dimension are the x, y, z components of the kinematic target positions and the last element activates (0 for kinematically controlled and 1 for dynamic nodes) the kinematic target.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_kinematic_targets = soft_body_view.get_sim_kinematic_targets() # Get the simulation mesh kinematic targets for all soft bodies in the view
                  >>> sim_kinematic_targets_np = sim_kinematic_targets.numpy().reshape(soft_body_view.count, soft_body_view.max_sim_vertices_per_body, 4) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_sim_kinematic_targets"):
            self._sim_kinematic_targets, self._sim_kinematic_targets_desc = self._frontend.create_tensor((self.count, self.max_sim_vertices_per_body, 4), float32)

        if not self._backend.get_sim_kinematic_targets(self._sim_kinematic_targets_desc):
            raise Exception("Failed to get soft body simulation mesh kinematic targets from backend")

        return self._sim_kinematic_targets
    
    def set_sim_kinematic_targets(self, data, indices):
        """ Sets the simulation mesh kinematic targets for the deformable bodies.

            .. note::
                The function raises an exception if the simulation mesh kinematic targets cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of kinematic targets with shape (count, max_sim_vertices_per_body, 4) where count is the number of soft bodies in the view and max_sim_vertices_per_body is the maximum number of simulated vertices per soft body in the view. The first 3 elements of the last dimension are the x, y, z components of the kinematic target positions and the last element activates (0 for kinematically controlled and 1 for dynamic nodes) the kinematic target.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of soft bodies to set the kinematic targets to. The array can have any dimension (user_count, 1) as long as all indices are less than the number of soft bodies in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> sim_kinematic_targets = soft_body_view.get_sim_kinematic_targets() # Get the simulation mesh kinematic targets for all soft bodies in the view
                  >>> # simulate physics for a period of time and then reset the simulation mesh kinematic targets to their initial values
                  >>> all_indices = wp_utils.arange(soft_body_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> soft_body_view.set_sim_kinematic_targets(sim_kinematic_targets, all_indices) # Reset the simulation mesh kinematic targets for the soft bodies in the view
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_sim_kinematic_targets(data_desc, indices_desc):
            raise Exception("Failed to set soft body simulation mesh kinematic targets in backend")

    def get_transforms(self):
        """ Gets the soft body transforms for bodies in the view.

            .. note::
                * The function raises an exception if the soft body transforms cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of soft body transforms with shape (count, 7) where count is the number of soft bodies in the view. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion of the transform.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> transforms = soft_body_view.get_transforms() # Get the transform for all soft bodies in the view
                  >>> transforms_np = transforms.numpy().reshape(soft_body_view.count, 7) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_transforms"):
            self._transforms, self._transforms_desc = self._frontend.create_tensor((self.count, 7), float32)

        if not self._backend.get_transforms(self._transforms_desc):
            raise Exception("Failed to get soft body transforms from backend")

        return self._transforms
        
    def check(self):
        """ Checks the validity of the underlying physics objects that the view uses.

            Returns:
                bool: True if all physics objects are valid, False otherwise.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_view = sim_view.create_soft_body_view("/World/Soft_*") # This assumes that the prims referenced by "/World/Soft_*" were already created in the stage
                  >>> soft_body_view.check() # returns False if soft_body_view is invalid, True otherwise
        """
        return self._backend.check()


# DEPRECATED
class SoftBodyMaterialView:
    """ SoftBodyMaterialView class represents a batch of soft body materials.

        .. warning::
            This class and its functionality is deprecated and will be removed.
            See :py:class:`DeformableMaterialView` as an alternative which is based on a successor deformable body feature.

        SoftBodyMaterialView binds the concrete implementation of the physics backend with the frontend tensor framework that is used to handle data.
        This class isn't meant to be instantiated directly, but rather created using the :py:func:`SimulationView.create_soft_body_material_view` method of the :py:class:`SimulationView` object.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("warp")
              >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
    """
    def __init__(self, backend, frontend):
        """ Constructs a SoftBodyMaterialView object with the given backend object and frontend framework.

            Currently, the only backend tensor is PhysX tensor.
            Several frontend frameworks are available: torch, numpy, tensorflow, warp

            Args:
                backend (ISoftBodyMaterialView): The backend object that implements the physics simulation for soft body materials.
                frontend (FrontendBase): The tensor framework that is used to handle data which could be FrontendNumpy, FrontendTorch, FrontendTensorflow, or FrontendWarp.
        """
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        """
            The number of soft body materials in the view
        """
        return self._backend.count

    def get_dynamic_friction(self):
        """ Gets the dynamic friction for all materials in the view.

            .. note::
                The function raises an exception if the dynamic friction cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of dynamic friction with shape (count, 1) where count is the number of soft body materials in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> sb_dynamic_friction = soft_body_material_view.get_dynamic_friction() # Get the dynamic friction for all soft body materials in the view
        """
        if not hasattr(self, "_dynamic_friction"):
            self._dynamic_friction, self._dynamic_friction_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_dynamic_friction(self._dynamic_friction_desc):
            raise Exception("Failed to get soft body material dynamic friction from backend")

        return self._dynamic_friction

    def set_dynamic_friction(self, data, indices):
        """ Sets the dynamic friction for soft body materials indicated by indices.

            .. note::
                The function raises an exception if the dynamic friction cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of dynamic friction with shape (count, 1) where count is the number of soft body materials in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of soft body materials to set the dynamic friction for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of soft body materials in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(soft_body_material_view.count)
                  >>> sb_dynamic_friction_np = numpy.zeros(soft_body_material_view.count) # Create an array with the expected shape
                  >>> sb_dynamic_friction_np[0] = 0.5 # Modify the dynamic friction for soft body material 0
                  >>> sb_dynamic_friction_wp = warp.from_numpy(sb_dynamic_friction_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> soft_body_material_view.set_dynamic_friction(sb_dynamic_friction_wp, all_indices) # Set the dynamic friction for all soft body materials
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dynamic_friction(data_desc, indices_desc):
            raise Exception("Failed to set soft body material dynamic friction in backend")

    def get_damping(self):
        """ Gets the damping for all materials in the view.

            .. note::
                The function raises an exception if the damping cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of damping with shape (count, 1) where count is the number of soft body materials in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> sb_damping = soft_body_material_view.get_damping() # Get the damping for all soft body materials in the view
        """
        if not hasattr(self, "_damping"):
            self._damping, self._damping_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_damping(self._damping_desc):
            raise Exception("Failed to get soft body material damping from backend")

        return self._damping

    def set_damping(self, data, indices):
        """ Sets the damping scale for soft body materials indicated by indices.

            .. note::
                The function raises an exception if the damping scale cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of damping scale with shape (count, 1) where count is the number of soft body materials in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of soft body materials to set the damping scale for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of soft body materials in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(soft_body_material_view.count)
                  >>> sb_damping_np = numpy.zeros(soft_body_material_view.count) # Create an array with the expected shape
                  >>> sb_damping_np[0] = 10.0 # Modify the damping for soft body material 0
                  >>> sb_damping_wp = warp.from_numpy(sb_damping_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> soft_body_material_view.set_damping(sb_damping_wp, all_indices) # Set the damping for all soft body materials
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_damping(data_desc, indices_desc):
            raise Exception("Failed to set soft body material damping scale in backend")

    def get_damping_scale(self):
        """ Gets the damping scale for all materials in the view.

            .. note::
                The function raises an exception if the damping scale cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of damping scale with shape (count, 1) where count is the number of soft body materials in the view

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> sb_damping_scale = soft_body_material_view.get_damping_scale() # Get the damping scale for all soft body materials in the view
        """
        if not hasattr(self, "_damping_scale"):
            self._damping_scale, self._damping_scale_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_damping_scale(self._damping_scale_desc):
            raise Exception("Failed to get soft body material damping scale from backend")

        return self._damping_scale

    def set_damping_scale(self, data, indices):
        """ Sets the damping scale for soft body materials indicated by indices.

            .. note::
                The function raises an exception if the damping scale cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of damping scale with shape (count, 1) where count is the number of soft body materials in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of soft body materials to set the damping scale for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of soft body materials in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(soft_body_material_view.count)
                  >>> sb_damping_scale_np = numpy.zeros(soft_body_material_view.count) # Create an array with the expected shape
                  >>> sb_damping_scale_np[0] = 0.7 # Modify the damping_scale for soft body material 0
                  >>> sb_damping_scale_wp = warp.from_numpy(sb_damping_scale_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> soft_body_material_view.set_damping_scale(sb_damping_scale_wp, all_indices) # Set the damping scale for all soft body materials
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_damping_scale(data_desc, indices_desc):
            raise Exception("Failed to set soft body material damping scale in backend")

    def get_poissons_ratio(self):
        """ Gets the Poisson's ratio for all materials in the view.

            .. note::
                The function raises an exception if the Poisson's ratio cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of Poisson's ratio with shape (count, 1) where count is the number of soft body materials in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> sb_poisson_ratio = soft_body_material_view.get_poissons_ratio() # Get the Poisson ratio for all soft body materials in the view
        """
        if not hasattr(self, "_poissons_ratio"):
            self._poissons_ratio, self._poissons_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_poissons_ratio(self._poissons_desc):
            raise Exception("Failed to get soft body material poissons ratio from backend")

        return self._poissons_ratio

    def set_poissons_ratio(self, data, indices):
        """ Sets the Poisson's ratio for soft body materials indicated by indices.

            .. note::
                The function raises an exception if the Poisson's ratio cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of Poisson's ratio with shape (count, 1) where count is the number of soft body materials in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of soft body materials to set the Poisson's ratio for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of soft body materials in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(soft_body_material_view.count)
                  >>> sb_poisson_ratio_np = numpy.zeros(soft_body_material_view.count) # Create an array with the expected shape
                  >>> sb_poisson_ratio_np[0] = 0.1 # Modify the Poisson ratio for soft body material 0
                  >>> sb_poisson_ratio_wp = warp.from_numpy(sb_poisson_ratio_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> soft_body_material_view.set_poissons_ratio(sb_poisson_ratio_wp, all_indices) # Set the Poisson ratio for all soft body materials
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_poissons_ratio(data_desc, indices_desc):
            raise Exception("Failed to set soft body material poissons ratio in backend")

    def get_youngs_modulus(self):
        """ Gets the Young's modulus for all materials in the view.

            .. note::
                The function raises an exception if the Young's modulus cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of Young's modulus with shape (count, 1) where count is the number of soft body materials in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> sb_young_modulus = soft_body_material_view.get_youngs_modulus() # Get the Young modulus for all soft body materials in the view
        """
        if not hasattr(self, "_youngs_modulus"):
            self._youngs_modulus, self._youngs_modulus_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_youngs_modulus(self._youngs_modulus_desc):
            raise Exception("Failed to get soft body material youngs modulus from backend")

        return self._youngs_modulus

    def set_youngs_modulus(self, data, indices):
        """ Sets the Young's modulus for soft body materials indicated by indices.

            .. note::
                The function raises an exception if the Young's modulus cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of Young's modulus with shape (count, 1) where count is the number of soft body materials in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of soft body materials to set the Young's modulus for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of soft body materials in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(soft_body_material_view.count)
                  >>> sb_young_modulus_np = numpy.zeros(soft_body_material_view.count) # Create an array with the expected shape
                  >>> sb_young_modulus_np[0] = 50.0 # Modify the Young modulus for soft body material 0
                  >>> sb_young_modulus_wp = warp.from_numpy(sb_young_modulus_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> soft_body_material_view.set_youngs_modulus(sb_young_modulus_wp, all_indices) # Set the Young modulus for all soft body materials
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_youngs_modulus(data_desc, indices_desc):
            raise Exception("Failed to set soft body material youngs modulus in backend")

    def check(self):
        """ Checks the validity of the underlying physics objects that the view uses.

            Returns:
                bool: True if all physics objects are valid, False otherwise.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> soft_body_material_view = sim_view.create_soft_body_material_view("/World/SoftMaterial_*") # This assumes that the prims referenced by "/World/SoftMaterial_*" were already created in the stage
                  >>> soft_body_material_view.check() # returns False if soft_body_material_view is invalid, True otherwise
        """
        return self._backend.check()


class DeformableBodyView:
    """ DeformableBodyView class represents a batch of either volume or surface deformable bodies.

        DeformableBodyView binds the concrete implementation of the physics backend with the frontend tensor framework that
        is used to handle data. This class isn't meant to be instantiated directly, but rather created using the
        :py:func:`SimulationView.create_volume_deformable_body_view` or :py:func:`SimulationView.create_surface_deformable_body_view`
        method of the :py:class:`SimulationView` object which manages all the physics views, the device where the simulation is
        performed and data resides.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("warp")
              >>> deformable_body_view = sim_view.create_volume_deformable_body_view("/World/Volume_*") # This assumes that the prims referenced by "/World/Volume_*" were already created in the stage
    """

    def __init__(self, backend, frontend):
        """ Constructs a DeformableBodyView object with the given backend object and frontend framework.

            Currently, the only backend tensor is PhysX tensor.
            Several frontend frameworks are available: torch, numpy, tensorflow, warp.

            Args:
                backend (IDeformableBodyView): The backend object that implements the physics simulation for deformable bodies.
                frontend (FrontendBase): The tensor framework that is used to handle data which could be FrontendNumpy, FrontendTorch, FrontendTensorflow, or FrontendWarp.
        """
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        """
            The number of deformable bodies in the view.
        """
        return self._backend.count

    @property
    def prim_paths(self):
        """
            The paths to the deformable bodies encapsulated in the view.

            In USD each deformable body may be represented as a hierarchy, with a root prim that has the
            DeformableBodyAPI applied to it. The prim_paths property returns the paths to the deformable body
            root prims.
        """
        return self._backend.prim_paths

    @property
    def simulation_mesh_prim_paths(self):
        """
            The paths to the simulation meshes encapsulated in the view.

            In USD each deformable body may be represented as a hierarchy, with a root prim that has the
            DeformableBodyAPI applied to it. In a hierarchical setup the simulation mesh is an immediate
            child prim with a VolumeDeformableSimAPI (tetrahedral mesh) or SurfaceDeformableSimAPI (triangle mesh)
            applied to it. The simulation_mesh_prim_paths property returns the simulation mesh paths for the
            deformable bodies encapsulated in the view. In non-hierarchical setups the paths returned by simulation_mesh_prim_paths
            are equal to the paths returned by :py:attr:`DeformableBodyView.prim_paths`.
        """
        return self._backend.simulation_mesh_prim_paths

    @property
    def collision_mesh_prim_paths(self):
        """
            The paths to the collision meshes encapsulated in the view.

            In USD each deformable body may be represented as a hierarchy, with a root prim that has the
            DeformableBodyAPI applied to it. In a hierarchical setup the collision mesh is a prim within
            the deformable body's sub-tree with a CollisionAPI applied to it. For volume deformables
            the collision mesh may be distinct from the simulation mesh. In this case, collision_mesh_prim_paths
            will return different paths compared to the paths returned by :py:attr:`DeformableBodyView.simulation_mesh_prim_paths`.
        """
        return self._backend.collision_mesh_prim_paths

    @property
    def max_simulation_elements_per_body(self):
        """
            The maximum number of elements in any simulation mesh of a deformable body in the view. 
        """
        return self._backend.max_simulation_elements_per_body

    @property
    def max_simulation_nodes_per_body(self):
        """
            The maximum number of nodes in any simulation mesh of a deformable body in the view. 
        """
        return self._backend.max_simulation_nodes_per_body

    @property
    def max_rest_nodes_per_body(self):
        """
            The maximum number of rest shape nodes in any deformable body in the view.

            The rest shape is the configuration at which each simulation element is stress/strain free.
            It is represented as a separate set of nodes and elements, allowing for a distinct topology
            from the simulation mesh, which is useful for e.g. panel based clothing.
            However, in the current implementation, the rest shape is limited to the same topology as the
            simulation mesh, meaning that max_rest_nodes_per_body currently returns the same value as
            :py:attr:`DeformableBodyView.max_simulation_nodes_per_body`.
        """
        return self._backend.max_rest_nodes_per_body

    @property
    def max_collision_elements_per_body(self):
        """
            The maximum number of elements in any collision mesh of a deformable body in the view. 
        """
        return self._backend.max_collision_elements_per_body
    
    @property
    def max_collision_nodes_per_body(self):
        """
            The maximum number of nodes in any collision mesh of a deformable body in the view. 
        """
        return self._backend.max_collision_nodes_per_body

    @property
    def num_nodes_per_element(self):
        """
            The number of nodes that are referenced per element. For volume deformables it is 4 (tetrahedron), for 
            surface deformables it is 3 (triangles).
        """
        return self._backend.num_nodes_per_element

    ### simulation mesh data

    def get_simulation_element_indices(self):
        """ Gets the simulation mesh element indices for the deformable bodies.

            Element indices can be used to obtain the nodes that make up the elements.

            .. note::
                The function raises an exception if the element indices cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation element indices with shape (count, max_simulation_elements_per_body, num_nodes_per_element) where count is the number of deformable bodies in the view and max_simulation_elements_per_body is the maximum number of elements per deformable body simulation mesh in the view. The num_nodes_per_element of the last dimension are the indices of the nodes that make up the element.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> body_view = sim_view.create_volume_deformable_body_view("/World/Volume_*") # This assumes that the prims referenced by "/World/Volume_*" were already created in the stage
                  >>> sim_indices = body_view.get_simulation_element_indices() # Get the simulation mesh element indices for all deformable bodies in the view
                  >>> sim_indices_np = sim_indices.numpy().reshape(body_view.count, body_view.max_simulation_elements_per_body, body_view.num_nodes_per_element) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_simulation_element_indices"):
            self._simulation_element_indices, self._simulation_element_indices_desc = self._frontend.create_tensor((self.count, self.max_simulation_elements_per_body, self.num_nodes_per_element), uint32)

        if not self._backend.get_simulation_element_indices(self._simulation_element_indices_desc):
            raise Exception("Failed to get deformable body simulation mesh element indices from backend")

        return self._simulation_element_indices

    def get_simulation_nodal_positions(self):
        """ Gets the simulation mesh nodal positions for the deformable bodies.

            .. note::
                The function raises an exception if the nodal positions cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of nodal positions with shape (count, max_simulation_nodes_per_body, 3) where count is the number of deformable bodies in the view and max_simulation_nodes_per_body is the maximum number of nodes per deformable body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal positions.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> body_view = sim_view.create_volume_deformable_body_view("/World/Volume_*") # This assumes that the prims referenced by "/World/Volume_*" were already created in the stage
                  >>> sim_positions = body_view.get_simulation_nodal_positions() # Get the simulation mesh nodal positions for all deformable bodies in the view
                  >>> sim_positions_np = sim_positions.numpy().reshape(body_view.count, body_view.max_simulation_nodes_per_body, 3) # Reshape the obtained array in a 3D numpy array on the host
        """
        if not hasattr(self, "_simulation_nodal_positions"):
            self._simulation_nodal_positions, self._simulation_nodal_positions_desc = self._frontend.create_tensor((self.count, self.max_simulation_nodes_per_body, 3), float32)

        if not self._backend.get_simulation_nodal_positions(self._simulation_nodal_positions_desc):
            raise Exception("Failed to get deformable body simulation mesh nodal positions from backend")

        return self._simulation_nodal_positions

    def set_simulation_nodal_positions(self, data, indices):
        """ Sets the simulation mesh nodal positions for the deformable bodies.

            .. note::
                The function raises an exception if the simulation mesh nodal positions cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of nodal positions with shape (count, max_simulation_nodes_per_body, 3) where count is the number of deformable bodies in the view and max_simulation_nodes_per_body is the maximum number of simulated nodes per deformable body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal positions.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of deformable bodies to set the nodal positions to. The array can have any dimension (user_count, 1) as long as all indices are less than the number of deformable bodies in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> body_view = sim_view.create_volume_deformable_body_view("/World/Volume_*") # This assumes that the prims referenced by "/World/Volume_*" were already created in the stage
                  >>> sim_nodal_positions = body_view.get_simulation_nodal_positions() # Get the simulation mesh nodal positions for all deformable bodies in the view
                  >>> # simulate physics for a period of time and then reset the simulation mesh nodal positions to their initial values
                  >>> all_indices = wp_utils.arange(body_view.count, device=device) # device is either cpu or cuda:0 (if using the first GPU)
                  >>> body_view.set_simulation_nodal_positions(sim_nodal_positions, all_indices) # Reset the simulation mesh nodal positions for the deformable bodies in the view
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_simulation_nodal_positions(data_desc, indices_desc):
            raise Exception("Failed to set deformable body simulation mesh nodal positions in backend")

    def get_simulation_nodal_velocities(self):
        """ Gets the simulation mesh nodal velocities for the deformable bodies.
        
            See :py:func:`DeformableBodyView.get_simulation_nodal_positions` for a usage example. 

            .. note::
                The function raises an exception if the nodal velocities cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of nodal velocities with shape (count, max_simulation_nodes_per_body, 3) where count is the number of deformable bodies in the view and max_simulation_nodes_per_body is the maximum number of nodes per deformable body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal velocities.
        """
        if not hasattr(self, "_simulation_nodal_velocities"):
            self._simulation_nodal_velocities, self._simulation_nodal_velocities_desc = self._frontend.create_tensor((self.count, self.max_simulation_nodes_per_body, 3), float32)

        if not self._backend.get_simulation_nodal_velocities(self._simulation_nodal_velocities_desc):
            raise Exception("Failed to get deformable body simulation mesh nodal velocities from backend")

        return self._simulation_nodal_velocities

    def set_simulation_nodal_velocities(self, data, indices):
        """ Sets the simulation mesh nodal velocities for the deformable bodies.
        
            See :py:func:`DeformableBodyView.set_simulation_nodal_positions` for a usage example. 

            .. note::
                The function raises an exception if the simulation mesh nodal velocities cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of nodal velocities with shape (count, max_simulation_nodes_per_body, 3) where count is the number of deformable bodies in the view and max_simulation_nodes_per_body is the maximum number of simulated nodes per deformable body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal velocities.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of deformable bodies to set the nodal velocities to. The array can have any dimension (user_count, 1) as long as all indices are less than the number of deformable bodies in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_simulation_nodal_velocities(data_desc, indices_desc):
            raise Exception("Failed to set deformable body simulation mesh nodal velocities in backend")

    def get_simulation_nodal_kinematic_targets(self):
        """ Gets the simulation mesh kinematic targets for the deformable bodies.
        
            Kinematic targets are currently only supported for volume deformable bodies.
            See :py:func:`DeformableBodyView.get_simulation_nodal_positions` for a usage example, however note that
            here each target is a 4-float-tuple. 

            .. note::
                The function raises an exception if the simulation mesh kinematic targets cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of simulation mesh kinematic targets with shape (count, max_simulation_nodes_per_body, 4) where count is the number of deformable bodies in the view and max_simulation_nodes_per_body is the maximum number of simulated nodes per deformable body in the view. The first 3 elements of the last dimension are the x, y, z components of the kinematic target positions and the last element activates (0 for kinematically controlled and 1 for dynamic nodes) the kinematic target.
        """
        if not hasattr(self, "_simulation_nodal_kinematic_targets"):
            self._simulation_nodal_kinematic_targets, self._simulation_nodal_kinematic_targets_desc = self._frontend.create_tensor((self.count, self.max_simulation_nodes_per_body, 4), float32)

        if not self._backend.get_simulation_nodal_kinematic_targets(self._simulation_nodal_kinematic_targets_desc):
            raise Exception("Failed to get deformable body simulation mesh nodal kinematic targets from backend")

        return self._simulation_nodal_kinematic_targets
    
    def set_simulation_nodal_kinematic_targets(self, data, indices):
        """ Sets the simulation mesh kinematic targets for the deformable bodies.
        
            See :py:func:`DeformableBodyView.set_simulation_nodal_positions` for a usage example, however note that here
            each target is a 4-float-tuple.

            .. note::
                The function raises an exception if the simulation mesh kinematic targets cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of kinematic targets with shape (count, max_simulation_nodes_per_body, 4) where count is the number of deformable bodies in the view and max_simulation_nodes_per_body is the maximum number of simulated nodes per deformable body in the view. The first 3 elements of the last dimension are the x, y, z components of the kinematic target positions and the last element activates (0 for kinematically controlled and 1 for dynamic nodes) the kinematic target.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of deformable bodies to set the kinematic targets to. The array can have any dimension (user_count, 1) as long as all indices are less than the number of deformable bodies in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_simulation_nodal_kinematic_targets(data_desc, indices_desc):
            raise Exception("Failed to set deformable body simulation mesh nodal kinematic targets in backend")

    ### rest shape data

    def get_rest_element_indices(self):
        """ Gets the rest shape element indices for the deformable bodies.

            The rest shape is the configuration at which each simulation element is stress/strain free.
            It is represented as a separate set of nodes and elements, allowing for a distinct topology
            from the simulation mesh, which is useful for e.g. panel based clothing.
            However, in the current implementation, the rest shape is limited to the same topology as the
            simulation mesh, meaning that get_rest_element_indices currently returns the same values as
            :py:func:`DeformableBodyView.get_simulation_element_indices`.
            See :py:func:`DeformableBodyView.get_simulation_element_indices` for a usage example.

            .. note::
                The function raises an exception if the element indices cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of rest element indices with shape (count, max_simulation_elements_per_body, num_nodes_per_element) where count is the number of deformable bodies in the view and max_simulation_elements_per_body is the maximum number of elements per deformable body simulation mesh in the view. The num_nodes_per_element of the last dimension are the indices of the nodes that make up the element.
        """
        if not hasattr(self, "_rest_element_indices"):
            self._rest_element_indices, self._rest_element_indices_desc = self._frontend.create_tensor((self.count, self.max_simulation_elements_per_body, self.num_nodes_per_element), uint32)

        if not self._backend.get_rest_element_indices(self._rest_element_indices_desc):
            raise Exception("Failed to get deformable body rest element indices from backend")

        return self._rest_element_indices

    def get_rest_nodal_positions(self):
        """ Gets the rest shape nodal positions for the deformable bodies.
        
            The rest shape is the configuration at which each simulation element is stress/strain free.
            It is represented as a separate set of nodes and elements, allowing for a distinct topology
            from the simulation mesh, which is useful for e.g. panel based clothing.
            However, in the current implementation, the rest shape is limited to the same topology as the
            simulation mesh, meaning that get_rest_nodal_positions currently returns positions of nodes which
            which align with simulation mesh node positions, but with distinct values.
            See :py:func:`DeformableBodyView.get_simulation_nodal_positions` for a usage example.

            .. note::
                The function raises an exception if the nodal positions cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of nodal positions with shape (count, max_rest_nodes_per_body, 3) where count is the number of deformable bodies in the view and max_rest_nodes_per_body is the maximum number of nodes per deformable body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal positions.
        """
        if not hasattr(self, "_rest_nodal_positions"):
            self._rest_nodal_positions, self._rest_nodal_positions_desc = self._frontend.create_tensor((self.count, self.max_rest_nodes_per_body, 3), float32)

        if not self._backend.get_rest_nodal_positions(self._rest_nodal_positions_desc):
            raise Exception("Failed to get deformable body rest nodal positions from backend")

        return self._rest_nodal_positions

    ### collision mesh data

    def get_collision_element_indices(self):
        """ Gets the collision mesh element indices for the deformable bodies.

            Element indices can be used to obtain the nodes that make up the elements.
            See :py:func:`DeformableBodyView.get_simulation_element_indices` for a usage example.

            .. note::
                The function raises an exception if the element indices cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of collision element indices with shape (count, max_collision_elements_per_body, num_nodes_per_element) where count is the number of deformable bodies in the view and max_collision_elements_per_body is the maximum number of elements per deformable body collision mesh in the view. The num_nodes_per_element of the last dimension are the indices of the nodes that make up the element.
        """
        if not hasattr(self, "_collision_element_indices"):
            self._collision_element_indices, self._collision_element_indices_desc = self._frontend.create_tensor((self.count, self.max_collision_elements_per_body, self.num_nodes_per_element), uint32)

        if not self._backend.get_collision_element_indices(self._collision_element_indices_desc):
            raise Exception("Failed to get deformable body collision mesh element indices from backend")

        return self._collision_element_indices

    def get_collision_nodal_positions(self):
        """ Gets the collision mesh nodal positions for the deformable bodies.

            See :py:func:`DeformableBodyView.get_collision_nodal_positions` for a usage example.

            .. note::
                The function raises an exception if the nodal positions cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of nodal positions with shape (count, max_collision_nodes_per_body, 3) where count is the number of deformable bodies in the view and max_collision_nodes_per_body is the maximum number of nodes per deformable body in the view. The 3 elements of the last dimension are the x, y, z components of the nodal positions.
        """
        if not hasattr(self, "_collision_nodal_positions"):
            self._collision_nodal_positions, self._collision_nodal_positions_desc = self._frontend.create_tensor((self.count, self.max_collision_nodes_per_body, 3), float32)

        if not self._backend.get_collision_nodal_positions(self._collision_nodal_positions_desc):
            raise Exception("Failed to get deformable body collision nodal positions from backend")

        return self._collision_nodal_positions

    def get_transforms(self):
        """ Gets the transforms for deformable bodies in the view.

            The transforms are reconstructed from the deformable body's simulation mesh axis aligned bounding box. The
            translation is set to center of the AABB, while the rotation always remains the identity quaternion.

            .. note::
                * The function raises an exception if the transforms cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of deformable body transforms with shape (count, 7) where count is the number of deformable bodies in the view. The 7 elements of the last dimension are the x, y, z, qx, qy, qz, qw components of the translation and rotation quaternion of the transform.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> body_view = sim_view.create_volume_deformable_body_view("/World/Volume_*") # This assumes that the prims referenced by "/World/Volume_*" were already created in the stage
                  >>> transforms = body_view.get_transforms() # Get the transform for all deformable bodies in the view
                  >>> transforms_np = transforms.numpy().reshape(body_view.count, 7) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_transforms"):
            self._transforms, self._transforms_desc = self._frontend.create_tensor((self.count, 7), float32)

        if not self._backend.get_transforms(self._transforms_desc):
            raise Exception("Failed to get deformable body transforms from backend")

        return self._transforms
        
    def check(self):
        """ Checks the validity of the underlying physics objects that the view uses.

            Returns:
                bool: True if all physics objects are valid, False otherwise.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> body_view = sim_view.create_surface_deformable_body_view("/World/Surface_*") # This assumes that the prims referenced by "/World/Surface_*" were already created in the stage
                  >>> body_view.check() # returns False if body_view is invalid, True otherwise
        """
        return self._backend.check()


class DeformableMaterialView:
    """ DeformableMaterialView class represents a batch of deformable materials.

        DeformableMaterialView binds the concrete implementation of the physics backend with the frontend tensor framework that is used
        to handle data. This class isn't meant to be instantiated directly, but rather created using the
        :py:func:`SimulationView.create_deformable_material_view` method of the :py:class:`SimulationView` object.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("warp")
              >>> deformable_material_view = sim_view.create_deformable_material_view("/World/DeformableMaterial_*") # This assumes that the prims referenced by "/World/DeformableMaterial_*" were already created in the stage
    """

    def __init__(self, backend, frontend):
        """ Constructs a DeformableMaterialView object with the given backend object and frontend framework.

            Currently, the only backend tensor is PhysX tensor.
            Several frontend frameworks are available: torch, numpy, tensorflow, warp

            Args:
                backend (IDeformableMaterialView): The backend object that implements the physics simulation for deformable materials.
                frontend (FrontendBase): The tensor framework that is used to handle data which could be FrontendNumpy, FrontendTorch, FrontendTensorflow, or FrontendWarp.
        """
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        """
            The number of deformable materials in the view
        """
        return self._backend.count

    def get_dynamic_friction(self):
        """ Gets the dynamic friction for all materials in the view.

            .. note::
                The function raises an exception if the dynamic friction cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of dynamic friction with shape (count, 1) where count is the number of deformable materials in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> deformable_material_view = sim_view.create_deformable_material_view("/World/DeformableMaterial_*") # This assumes that the prims referenced by "/World/DeformableMaterial_*" were already created in the stage
                  >>> dynamic_friction = deformable_material_view.get_dynamic_friction() # Get the dynamic friction for all deformable materials in the view
        """
        if not hasattr(self, "_dynamic_friction"):
            self._dynamic_friction, self._dynamic_friction_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_dynamic_friction(self._dynamic_friction_desc):
            raise Exception("Failed to get deformable_material dynamic friction from backend")

        return self._dynamic_friction

    def set_dynamic_friction(self, data, indices):
        """ Sets the dynamic friction for deformable materials indicated by indices.

            .. note::
                The function raises an exception if the dynamic friction cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of dynamic friction with shape (count, 1) where count is the number of deformable materials in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of deformable materials to set the dynamic friction for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of deformable materials in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> deformable_material_view = sim_view.create_deformable_material_view("/World/DeformableMaterial_*") # This assumes that the prims referenced by "/World/DeformableMaterial_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(deformable_material_view.count)
                  >>> dynamic_friction_np = numpy.zeros(deformable_material_view.count) # Create an array with the expected shape
                  >>> dynamic_friction_np[0] = 0.5 # Modify the dynamic friction for deformable material 0
                  >>> dynamic_friction_wp = warp.from_numpy(dynamic_friction_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> deformable_material_view.set_dynamic_friction(dynamic_friction_wp, all_indices) # Set the dynamic friction for all deformable materials
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_dynamic_friction(data_desc, indices_desc):
            raise Exception("Failed to set deformable_material dynamic friction in backend")

    def get_youngs_modulus(self):
        """ Gets the Young's modulus for all materials in the view.
        
            See `DeformableMaterialView.get_dynamic_friction` for an example for how to retrieve material properties.

            .. note::
                The function raises an exception if the Young's modulus cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of Young's modulus with shape (count, 1) where count is the number of deformable materials in the view.
        
            Example:
                .. code-block:: python
                    >>> import omni.physics.tensors as tensors
                    >>> sim_view = tensors.create_simulation_view("warp")
                    >>> deformable_material_view = sim_view.create_deformable_material_view("/World/DeformableMaterial_*") # This assumes that the prims referenced by "/World/DeformableMaterial_*" were already created in the stage
                    >>> youngs_modulus = deformable_material_view.get_youngs_modulus() # Get the Young's modulus for all deformable materials in the view
        """
        if not hasattr(self, "_youngs_modulus"):
            self._youngs_modulus, self._youngs_modulus_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_youngs_modulus(self._youngs_modulus_desc):
            raise Exception("Failed to get deformable_material youngs modulus from backend")

        return self._youngs_modulus

    def set_youngs_modulus(self, data, indices):
        """ Sets the Young's modulus for deformable_materials indicated by indices.
        
            See `DeformableMaterialView.set_dynamic_friction` for an example for how to update material properties.

            .. note::
                The function raises an exception if the Young's modulus cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of Young's modulus with shape (count, 1) where count is the number of deformable_materials in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of deformable_materials to set the Young's modulus for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of deformable_materials in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.

            Example:
                .. code-block:: python
                    >>> import omni.physics.tensors as tensors
                    >>> sim_view = tensors.create_simulation_view("warp")
                    >>> deformable_material_view = sim_view.create_deformable_material_view("/World/DeformableMaterial_*") # This assumes that the prims referenced by "/World/DeformableMaterial_*" were already created in the stage
                    >>> all_indices = wp_utils.arange(deformable_material_view.count)
                    >>> youngs_modulus_np = numpy.zeros(deformable_material_view.count) # Create an array with the expected shape
                    >>> youngs_modulus_np[0] = 1000000.0 # Modify the Young's modulus for deformable material 0
                    >>> youngs_modulus_wp = warp.from_numpy(youngs_modulus_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                    >>> deformable_material_view.set_youngs_modulus(youngs_modulus_wp, all_indices) # Set the Young's modulus for all deformable materials
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_youngs_modulus(data_desc, indices_desc):
            raise Exception("Failed to set deformable_material youngs modulus in backend")

    def get_poissons_ratio(self):
        """ Gets the Poisson's ratio for all materials in the view.

            .. note::
                The function raises an exception if the Poisson's ratio cannot be obtained from the backend.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of Poisson's ratio with shape (count, 1) where count is the number of deformable materials in the view.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> deformable_material_view = sim_view.create_deformable_material_view("/World/DeformableMaterial_*") # This assumes that the prims referenced by "/World/DeformableMaterial_*" were already created in the stage
                  >>> db_poisson_ratio = deformable_material_view.get_poissons_ratio() # Get the Poisson ratio for all deformable_materials in the view
        """
        if not hasattr(self, "_poissons_ratio"):
            self._poissons_ratio, self._poissons_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_poissons_ratio(self._poissons_desc):
            raise Exception("Failed to get deformable_material poissons ratio from backend")

        return self._poissons_ratio

    def set_poissons_ratio(self, data, indices):
        """ Sets the Poisson's ratio for deformable_materials indicated by indices.

            .. note::
                The function raises an exception if the Poisson's ratio cannot be set in the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of Poisson's ratio with shape (count, 1) where count is the number of deformable_materials in the view.
                indices (Union[np.ndarray, torch.Tensor, wp.array]): An array of indices of deformable_materials to set the Poisson's ratio for. The array can have any dimension (user_count, 1) as long as all indices are less than the number of deformable_materials in the view (count). Note that providing repeated indices can lead to undefined behavior as the subsitutions may be performed in parallel.


            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> deformable_material_view = sim_view.create_deformable_material_view("/World/DeformableMaterial_*") # This assumes that the prims referenced by "/World/DeformableMaterial_*" were already created in the stage
                  >>> all_indices = wp_utils.arange(deformable_material_view.count)
                  >>> db_poisson_ratio_np = numpy.zeros(deformable_material_view.count) # Create an array with the expected shape
                  >>> db_poisson_ratio_np[0] = 0.1 # Modify the Poisson ratio for deformable_material 0
                  >>> db_poisson_ratio_wp = warp.from_numpy(db_poisson_ratio_np, dtype=warp.float32, device="cpu") # Convert back to a format accepted by the tensor API
                  >>> deformable_material_view.set_poissons_ratio(db_poisson_ratio_wp, all_indices) # Set the Poisson ratio for all deformable_materials
        """
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_poissons_ratio(data_desc, indices_desc):
            raise Exception("Failed to set deformable_material poissons ratio in backend")

    def check(self):
        """ Checks the validity of the underlying physics objects that the view uses.

            Returns:
                bool: True if all physics objects are valid, False otherwise.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> deformable_material_view = sim_view.create_deformable_material_view("/World/DeformableMaterial_*") # This assumes that the prims referenced by "/World/DeformableMaterial_*" were already created in the stage
                  >>> deformable_material_view.check() # returns False if deformable_material_view is invalid, True otherwise
        """
        return self._backend.check()


class RigidContactView:
    """ RigidContactView class represents a batch of rigid contact.

        RigidContactView binds the concrete implementation of the physics backend with the frontend tensor framework that is used to handle data.
        This class isn't meant to be instantiated directly, but rather created using the :py:func:`SimulationView.create_rigid_contact_view` method of the :py:class:`SimulationView` object.

        .. note::
            RigidContactView objects can be used to access contact forces and their relavant quantities in a tensorized way.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("warp")
              >>> rigid_contact_view = sim_view.create_rigid_contact_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
    """
    def __init__(self, backend, frontend):
        """ Constructs a RigidContactView object with the given backend object and frontend framework.

            Currently, the only backend tensor is PhysX tensor.
            Several frontend frameworks are available: torch, numpy, tensorflow, warp.

            Args:
                backend (IRigidContactView): The backend object that implements the physics simulation for rigid contacts.
                frontend (FrontendBase): The tensor framework that is used to handle data which could be FrontendNumpy, FrontendTorch, FrontendTensorflow, or FrontendWarp.
        """
        self._backend = backend
        self._frontend = frontend
        self._num_components = 0

    @property
    def sensor_count(self):
        """
            The number of sensors in the view. Sensors are the objects whose contacts with other objects are tracked by this view.
        """
        return self._backend.sensor_count

    @property
    def filter_count(self):
        """
            The number of filters in the view. Filters are the objects that allows getting more detailed pair-wise contact forces with sensors.
            Providing filters is optional, but if filters are not provided, only the net contact forces of the sensor objects can be obtained with the view.
        """
        return self._backend.filter_count

    @property
    def max_contact_data_count(self):
        """
            The maximum number of contact data that can be stored in the view.
            Providing a non-zero value for this parameter allows getting detailed contact data for each sensor-filter pair.
            This is used for managing the underlying memory, so needs to be tuned for each problem depending on expected number of contacts in the environment.
        """
        return self._backend.max_contact_data_count

    @property
    def sensor_paths(self):
        """
            The USD paths of the sensor prims in the view.
        """
        return self._backend.sensor_paths

    @property
    def filter_paths(self):
        """
            The USD paths of the filter prims in the view.
        """
        return self._backend.filter_paths
    
    @property
    def sensor_names(self):
        """
            The names of the sensor prims in the view.
        """
        return self._backend.sensor_names

    @property
    def filter_names(self):
        """
            The names of the filter prims in the view.
        """
        return self._backend.filter_names
    
    def get_net_contact_forces(self, dt):
        """ Gets the net contact forces acting on the sensors in the view.

            This includes all the contact forces from all the prims interacting with the sensors, not only the filter prims.

            .. note::
                The function raises an exception if the net contact forces cannot be obtained from the backend.

            Args:
                dt (float): The time step of the simulation to use to convert impulses to forces.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of net contact forces with shape (sensor_count, 3) where sensor_count is the number of sensors in the view. The 3 elements of the last dimension are the x, y, z components of the net contact forces.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_contact_view = sim_view.create_rigid_contact_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> net_contact_forces = rigid_contact_view.get_net_contact_forces(dt) # Get the net contact forces acting on the sensors in the view
                  >>> net_contact_forces_np = net_contact_forces.numpy().reshape(rigid_contact_view.sensor_count, 3) # Reshape the obtained array in a 2D numpy array on the host
        """
        if not hasattr(self, "_net_forces"):
            self._net_forces, self._net_forces_desc = self._frontend.create_tensor(
                (self.sensor_count, 3), float32
            )
        if not self._backend.get_net_contact_forces(self._net_forces_desc, dt):
            raise Exception("Failed to get net contact forces from backend")
        else:
            return self._net_forces

    def get_contact_force_matrix(self, dt):
        """ Gets the matrix of pair-wise contact forces between sensors and filter prims.

            This only includes the contact forces between the sensors and the filter prims but not other potential contacts between the sensors and non-filter prims.

            .. note::
                * The function raises an exception if the contact force matrix cannot be obtained from the backend.
                * This function can only be used if the view is initialized with a valid filter prims.
                * There could be more than one contact point between a sensor prim and a filter prim, but this function aggregates and reports the sum of all those contact forces between a pair of sensor/filter.

            Args:
                dt (float): The time step of the simulation to use to convert impulses to forces.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]:
                  An array of contact forces with shape (sensor_count, filter_count, 3) where sensor_count is the number of sensors in the view and filter_count is the number of filter prims in the view.
                  The 3 elements of the last dimension are the x, y, z components of the contact forces between the sensors prims (in the first dimension) and the filter prims (in the second dimension).

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_contact_view = sim_view.create_rigid_contact_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> contact_force_matrix = rigid_contact_view.get_contact_force_matrix(dt) # Get the matrix of pair-wise contact forces between sensors and filter prims
                  >>> contact_force_matrix_np = contact_force_matrix.numpy().reshape(rigid_contact_view.sensor_count, rigid_contact_view.filter_count, 3) # Reshape the obtained array in a 3D numpy array on the host
        """
        if self.filter_count == 0:
            raise Exception("Contact force matrix is not available because no filters were specified")
        if not hasattr(self, "_matrix"):
            self._matrix, self._matrix_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count, 3), float32)
        if not self._backend.get_contact_force_matrix(self._matrix_desc, dt):
            raise Exception("Failed to get contact force matrix from backend")
        return self._matrix

    def get_contact_data(self, dt):
        """ Gets the detailed contact data between sensors and filter prims per patch.

            This includes the contact forces, contact points, contact normals, and separation distances.

            Note that there could be more than one contact point between a sensor prim and a filter prim. This function however, does not report the aggregated values for a pair of sensor/filter prims. This is an important distinction from the :py:func:`RigidContactView.get_contact_force_matrix` function.
            This is needed because some of the variables reported by this function (such as contact normals, points, and separation) are defined only for contact points and cannot be aggregated accross the shape without further information about the geometry of the contact.

            Note also that the structure of the data returned by this function is different from the :py:func:`RigidContactView.get_contact_force_matrix` function. This function returns contacts data in buffers that need to be further processed.
            Specifically, the last two tensors returned by this function (i.e. contact_count_buffer, and start_indices_buffer) are used to locate the set of contact data for each sensor-filter pair in the first three tensors (i.e. force_buffer, point_buffer, normal_buffer, separation_buffer).

            For each sensor-filter pair, contact data (force_buffer, point_buffer, normal_buffer, separation_buffer) for all the contact points between sensor-filter prims are within the range of start_indices_buffer[sensor, filter] to start_indices_buffer[sensor, filter] + contact_count_buffer[sensor, filter].

            .. note::
                * The function raises an exception if the contact data cannot be obtained from the backend.
                * This function can only be used if the view is initialized with a valid filter prims.
                * The function raises an exception if the view is initialized with max_contact_data_count = 0.

            Args:
                dt (float): The time step of the simulation to use to convert impulses to forces.

            Returns:
                Tuple[Union[np.ndarray, torch.Tensor, wp.array], Union[np.ndarray, torch.Tensor, wp.array], Union[np.ndarray, torch.Tensor, wp.array], Union[np.ndarray, torch.Tensor, wp.array], Union[np.ndarray, torch.Tensor, wp.array]]:
                  A tuple of five arrays of contact data:
                    * The first array is the contact forces with shape (max_contact_data_count, 1) where max_contact_data_count is the maximum number of contact data that can be stored in the view.
                    * The second array is the contact points with shape (max_contact_data_count, 3) where the 3 elements of the last dimension are the x, y, z components of the contact points.
                    * The third array is the contact normals with shape (max_contact_data_count, 3) where the 3 elements of the last dimension are the x, y, z components of the contact normals vector.
                    * The fourth array is the separation distances with shape (max_contact_data_count, 1).
                    * The fifth array is the contact count buffer with shape (sensor_count, filter_count) where sensor_count is the number of sensors in the view and filter_count is the number of filter prims in the view.
                    * The sixth array is the start indices buffer with shape (sensor_count, filter_count).

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_contact_view = sim_view.create_rigid_contact_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> forces, points, normals, distances, counts, start_indices = rigid_contact_view.get_contact_data(dt) # Get the detailed contact data between sensors and filter prims per patch

                  >>> forces_np = forces.numpy().reshape(rigid_contact_view.max_contact_data_count) # Reshape the obtained array in a 1D numpy array on the host
                  >>> points_np = points.numpy().reshape(rigid_contact_view.max_contact_data_count * 3) # Reshape the obtained array in a 1D numpy array on the host
                  >>> normals_np = normals.numpy().reshape(rigid_contact_view.max_contact_data_count * 3) # Reshape the obtained array in a 1D numpy array on the host
                  >>> distances_np = distances.numpy().reshape(rigid_contact_view.max_contact_data_count) # Reshape the obtained array in a 1D numpy array on the host
                  >>> counts_np = counts.numpy().reshape(rigid_contact_view.sensor_count, rigid_contact_view.filter_count) # Reshape the obtained array in a 2D numpy array on the host
                  >>> start_indices_np = start_indices.numpy().reshape(rigid_contact_view.sensor_count, rigid_contact_view.filter_count) # Reshape the obtained array in a 2D numpy array on the host

                  # for given prims i and j, the net pairwise contact force is the sum of individual contact forces
                  >>> start_indices_ij = start_indices_np[i, j]
                  >>> count_ij = counts_np[i, j]
                  >>> if count_ij > 0:
                  >>>     forces_ij = forces_np[start_indices_ij:start_indices_ij + count_ij] * normals_np[start_indices_ij:start_indices_ij + count_ij]
                  >>> force_aggregate = numpy.sum(forces_ij, axis=0)
        """
        if self.filter_count == 0:
            raise Exception("Contact data is not available because no filters were specified")

        if self.max_contact_data_count == 0:
            raise Exception("No contact data is available because create_rigid_contact_view is initialized with max_contact_data_count = 0")

        if not hasattr(self, "_force_buffer"):
            self._force_buffer, self._force_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 1), float32)         
        if not hasattr(self, "_point_buffer"):
            self._point_buffer, self._point_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 3), float32)
        if not hasattr(self, "_normal_buffer"):
            self._normal_buffer, self._normal_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 3), float32)
        if not hasattr(self, "_separation_buffer"):
            self._separation_buffer, self._separation_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 1), float32)  
        if not hasattr(self, "_start_indices_buffer"):
            self._start_indices_buffer, self._start_indices_buffer_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count), uint32)
        if not hasattr(self, "_contact_count_buffer"):
            self._contact_count_buffer, self._contact_count_buffer_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count), uint32)  

        if not self._backend.get_contact_data(self._force_buffer_desc, self._point_buffer_desc, self._normal_buffer_desc, 
                                                self._separation_buffer_desc, self._contact_count_buffer_desc, self._start_indices_buffer_desc, dt):
            raise Exception("Failed to get contact data from backend")

        return self._force_buffer, self._point_buffer, self._normal_buffer, self._separation_buffer, self._contact_count_buffer, self._start_indices_buffer,

    def get_friction_data(self, dt):
        """ Gets the friction data between sensors and filter prims.

            This includes the friction forces and the friction points.

            Note that there could be more than one contact point between a sensor prim and a filter prim. This function is similar to :py:func:`RigidContactView.get_contact_force_matrix`, but does not report the aggregated values for a pair of sensor/filter prims.
            This is needed because some of the variables reported by this function (such as friction points) are defined only for contact points and cannot be aggregated accross the shape without further information about the geometry of the contact.

            Note also that the structure of the data returned by this function is different from the :py:func:`RigidContactView.get_contact_force_matrix` function. This function returns contacts data in buffers that need to be further processed.
            Specifically, the last two tensors returned by this function (i.e. contact_count_buffer, and start_indices_buffer) are used to locate the set of contact data for each sensor-filter pair in the first two tensors (i.e. force_buffer, point_buffer).

            For each sensor-filter pair, contact data (force_buffer, point_buffer) for all the contact points between sensor-filter prims are within the range of start_indices_buffer[sensor, filter] to start_indices_buffer[sensor, filter] + contact_count_buffer[sensor, filter].

            .. note::
                * The function raises an exception if the friction data cannot be obtained from the backend.
                * This function can only be used if the view is initialized with a valid filter prims.
                * The function raises an exception if the view is initialized with max_contact_data_count = 0.

            Args:
                dt (float): The time step of the simulation to use to convert impulses to forces.

            Returns:
                Tuple[Union[np.ndarray, torch.Tensor, wp.array], Union[np.ndarray, torch.Tensor, wp.array], Union[np.ndarray, torch.Tensor, wp.array], Union[np.ndarray, torch.Tensor, wp.array]]:
                  A tuple of four arrays of friction data:
                    * The first array is the friction forces with shape (max_contact_data_count, 3) where max_contact_data_count is the maximum number of contact data that can be stored in the view. The 3 elements of the last dimension are the x, y, z components of the friction forces.
                    * The second array is the friction points with shape (max_contact_data_count, 3) where the 3 elements of the last dimension are the x, y, z components of the friction points.
                    * The third array is the contact count buffer with shape (sensor_count, filter_count) where sensor_count is the number of sensors in the view and filter_count is the number of filter prims in the view.
                    * The fourth array is the start indices buffer with shape (sensor_count, filter_count).

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_contact_view = sim_view.create_rigid_contact_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> frictions, points, counts, start_indices = rigid_contact_view.get_friction_data(dt) # Get the detailed friction data between sensors and filter prims

                  >>> frictions_np = frictions.numpy().reshape(rigid_contact_view.max_contact_data_count * 3) # Reshape the obtained array in a 1D numpy array on the host
                  >>> points_np = points.numpy().reshape(rigid_contact_view.max_contact_data_count * 3) # Reshape the obtained array in a 1D numpy array on the host
                  >>> counts_np = counts.numpy().reshape(rigid_contact_view.sensor_count, rigid_contact_view.filter_count) # Reshape the obtained array in a 2D numpy array on the host
                  >>> start_indices_np = start_indices.numpy().reshape(rigid_contact_view.sensor_count, rigid_contact_view.filter_count) # Reshape the obtained array in a 2D numpy array on the host

                  # for given prims i and j, the net pairwise friction force is the sum of individual friction forces
                  >>> start_indices_ij = start_indices_np[i, j]
                  >>> count_ij = counts_np[i, j]
                  >>> if count_ij > 0:
                  >>>     frictions_forces_ij = frictions_np[start_indices_ij:start_indices_ij + count_ij]
                  >>> frictions_force_aggregate = numpy.sum(frictions_forces_ij, axis=0)
        """
        if self.filter_count == 0:
            raise Exception("Friction data is not available because no filters were specified")

        if self.max_contact_data_count == 0:
            raise Exception("No contact data is available because create_rigid_contact_view is initialized with max_contact_data_count = 0")

        if not hasattr(self, "_friction_force_buffer"):
            self._friction_force_buffer, self._friction_force_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 3), float32)         
        if not hasattr(self, "_friction_point_buffer"):
            self._friction_point_buffer, self._friction_point_buffer_desc = self._frontend.create_tensor(
                (self.max_contact_data_count, 3), float32)
        if not hasattr(self, "_start_indices_buffer"):
            self._start_indices_buffer, self._start_indices_buffer_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count), uint32)
        if not hasattr(self, "_contact_count_buffer"):
            self._contact_count_buffer, self._contact_count_buffer_desc = self._frontend.create_tensor(
                (self.sensor_count, self.filter_count), uint32)  

        if not self._backend.get_friction_data(self._friction_force_buffer_desc, self._friction_point_buffer_desc, self._contact_count_buffer_desc, self._start_indices_buffer_desc, dt):
            raise Exception("Failed to get friction data from backend")

        return self._friction_force_buffer, self._friction_point_buffer, self._contact_count_buffer, self._start_indices_buffer,

    def check(self):
        """ Checks the validity of the underlying physics objects that the view uses.

            Returns:
                bool: True if all physics objects are valid, False otherwise.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> rigid_contact_view = sim_view.create_rigid_contact_view("/World/Cube_*") # This assumes that the prims referenced by "/World/Cube_*" were already created in the stage
                  >>> rigid_contact_view.check() # returns False if rigid_contact_view is invalid, True otherwise
        """
        return self._backend.check()

class SdfShapeView:
    """ SdfShapeView class represents a batch of SDF shapes.

        SdfShapeView binds the concrete implementation of the physics backend with the frontend tensor framework that is used to handle data.
        This class isn't meant to be instantiated directly, but rather created using the :py:func:`SimulationView.create_sdf_shape_view` method of the :py:class:`SimulationView` object.

        .. note::
            SdfShapeView objects can be used to access signed distance functions and their gradients in a tensorized way.

        Example:
            .. code-block:: python

              >>> import omni.physics.tensors as tensors
              >>> sim_view = tensors.create_simulation_view("warp")
              >>> sdf_shape_view = sim_view.create_sdf_shape_view("/World/SDF_*", 1000) # This assumes that the prims referenced by "/World/SDF_*" were already created in the stage
    """
    def __init__(self, backend, frontend):
        """ Constructs a SdfShapeView object with the given backend object and frontend framework.

            Currently, the only backend tensor is PhysX tensor.
            Several frontend frameworks are available: torch, numpy, tensorflow, warp.

            Args:
                backend (ISdfShapeView): The backend object that implements the physics simulation for signed distance functions.
                frontend (FrontendBase): The tensor framework that is used to handle data which could be FrontendNumpy, FrontendTorch, FrontendTensorflow, or FrontendWarp.
        """
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        """
            The number of shape with SDF information in the view.
        """
        return self._backend.count

    @property
    def max_num_points(self):
        """
            The maximum number of points that can be used to query SDF and gradients in the view.
        """
        return self._backend.max_num_points

    @property
    def object_paths(self):
        """
            The USD paths of the shapes in the view.
        """
        return self._backend.object_paths

    def get_sdf_and_gradients(self, data):
        """ Gets the signed distance functions and their gradients for the shapes in the view.

            .. note::
                The function raises an exception if the signed distance functions and their gradients cannot be obtained from the backend.

            Args:
                data (Union[np.ndarray, torch.Tensor, wp.array]): An array of points with shape (count, max_num_points, 3) where count is the number of shapes in the view and max_num_points is the maximum number of points that can be used to query SDF and gradients in the view. The 3 elements of the last dimension are the x, y, z components of the points.

            Returns:
                Union[np.ndarray, torch.Tensor, wp.array]: An array of signed distance functions and their gradients with shape (count, max_num_points, 4) where count is the number of shapes in the view and max_num_points is the maximum number of points that can be used to query SDF and gradients in the view. The first element of the last dimension is the signed distance function and the last 3 elements are the x, y, z components of the gradient.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> sdf_shape_view = sim_view.create_sdf_shape_view("/World/SDF_*", 1000) # This assumes that the prims referenced by "/World/SDF_*" were already created in the stage
                  >>> sdf_and_gradients = sdf_shape_view.get_sdf_and_gradients(dt) # Get the signed distance functions and their gradients for the shapes in the view
                  >>> sdf_and_gradients_np = sdf_and_gradients.numpy().reshape(sdf_shape_view.count, sdf_shape_view.max_num_points, 4) # Reshape the obtained array in a 3D numpy array on the host
        """
        data = self._frontend.as_contiguous_float32(data)
        data_desc = self._frontend.get_tensor_desc(data)
        if not hasattr(self, "_sdf_and_gradients"):
            self._sdf_and_gradients, self._sdf_and_gradients_desc = self._frontend.create_tensor(
                (self.count, self._backend.max_num_points, 4), float32
            )
        if not self._backend.get_sdf_and_gradients(self._sdf_and_gradients_desc, data_desc):
            raise Exception("Failed to get sdf and gradients from backend")
        else:
            return self._sdf_and_gradients

    def check(self):
        """ Checks the validity of the underlying physics objects that the view uses.

            Returns:
                bool: True if all physics objects are valid, False otherwise.

            Example:
                .. code-block:: python

                  >>> import omni.physics.tensors as tensors
                  >>> sim_view = tensors.create_simulation_view("warp")
                  >>> sdf_shape_view = sim_view.create_sdf_shape_view("/World/SDF_*", 1000) # This assumes that the prims referenced by "/World/SDF_*" were already created in the stage
                  >>> sdf_shape_view.check() # returns False if sdf_shape_view is invalid, True otherwise
        """
        return self._backend.check()

# DEPRECATED
class ParticleSystemView:
    """
        .. warning::
            This class and its functionality is deprecated and will be removed.
            See :py:class:`DeformableBodyView` as an alternative which is based on a successor deformable body feature.
    """
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    def get_solid_rest_offsets(self):
        if not hasattr(self, "_solid_rest_offsets"):
            self._solid_rest_offsets, self._solid_rest_offsets_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_solid_rest_offsets(self._solid_rest_offsets_desc):
            raise Exception("Failed to get particle system solid rest offsets from backend")

        return self._solid_rest_offsets

    def set_solid_rest_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_solid_rest_offsets(data_desc, indices_desc):
            raise Exception("Failed to set particle system solid rest offsets in backend")

    def get_fluid_rest_offsets(self):
        if not hasattr(self, "_fluid_rest_offsets"):
            self._fluid_rest_offsets, self._fluid_rest_offsets_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_fluid_rest_offsets(self._fluid_rest_offsets_desc):
            raise Exception("Failed to get particle system fluid rest offsets from backend")

        return self._fluid_rest_offsets

    def set_fluid_rest_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_fluid_rest_offsets(data_desc, indices_desc):
            raise Exception("Failed to set particle system fluid rest offsets in backend")

    def get_particle_contact_offsets(self):
        if not hasattr(self, "_particle_contact_offsets"):
            self._particle_contact_offsets, self._particle_contact_offsets_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_particle_contact_offsets(self._particle_contact_offsets_desc):
            raise Exception("Failed to get particle system particle contact offsets from backend")

        return self._particle_contact_offsets

    def set_particle_contact_offsets(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_particle_contact_offsets(data_desc, indices_desc):
            raise Exception("Failed to set particle system particle contact offsets in backend")

    def get_wind(self):
        if not hasattr(self, "_wind"):
            self._wind, self._wind_desc = self._frontend.create_tensor((self.count, 3), float32, -1)
        
        if not self._backend.get_wind(self._wind_desc):
            raise Exception("Failed to get particle system particle contact offsets from backend")

        return self._wind

    def set_wind(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_wind(data_desc, indices_desc):
            raise Exception("Failed to set particle system particle wind in backend")

    def check(self):
        return self._backend.check()


# DEPRECATED
class ParticleClothView:
    """
    .. warning::
        This class and its functionality is deprecated and will be removed.
        See :py:class:`DeformableBodyView` as an alternative which is based on a successor deformable body feature.
    """
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    @property
    def max_particles_per_cloth(self):
        return self._backend.max_particles_per_cloth

    @property
    def max_springs_per_cloth(self):
        return self._backend.max_springs_per_cloth

    def get_positions(self):
        if not hasattr(self, "_positions"):
            self._positions, self._positions_desc = self._frontend.create_tensor((self.count, self.max_particles_per_cloth * 3), float32)
        
        if not self._backend.get_positions(self._positions_desc):
            raise Exception("Failed to get particle cloth positions from backend")

        return self._positions

    def set_positions(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_positions(data_desc, indices_desc):
            raise Exception("Failed to set particle cloth positions in backend")

    def get_velocities(self):
        if not hasattr(self, "_velocities"):
            self._velocities, self._velocities_desc = self._frontend.create_tensor((self.count, self.max_particles_per_cloth * 3), float32)

        if not self._backend.get_velocities(self._velocities_desc):
            raise Exception("Failed to get particle cloth velocities from backend")

        return self._velocities

    def set_velocities(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_velocities(data_desc, indices_desc):
            raise Exception("Failed to set particle cloth velocities in backend")

    def get_masses(self):
        if not hasattr(self, "_masses"):
            self._masses, self._masses_desc = self._frontend.create_tensor((self.count, self.max_particles_per_cloth), float32)

        if not self._backend.get_masses(self._masses_desc):
            raise Exception("Failed to get particle cloth masses from backend")

        return self._masses

    def set_masses(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_masses(data_desc, indices_desc):
            raise Exception("Failed to set particle cloth masses in backend")

    def get_spring_damping(self):
        if not hasattr(self, "_spring_damping"):
            self._spring_damping, self.spring_damping_desc = self._frontend.create_tensor((self.count, self.max_springs_per_cloth), float32)

        if not self._backend.get_spring_damping(self.spring_damping_desc):
            raise Exception("Failed to get particle spring damping from backend")

        return self._spring_damping     

    def set_spring_damping(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_spring_damping(data_desc, indices_desc):
            raise Exception("Failed to set particle spring damping in backend")

    def get_spring_stiffness(self):
        if not hasattr(self, "_spring_stiffness"):
            self._spring_stiffness, self.spring_stiffness_desc = self._frontend.create_tensor((self.count, self.max_springs_per_cloth), float32)

        if not self._backend.get_spring_stiffness(self.spring_stiffness_desc):
            raise Exception("Failed to get particle spring stiffness from backend")

        return self._spring_stiffness    

    def set_spring_stiffness(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_spring_stiffness(data_desc, indices_desc):
            raise Exception("Failed to set particle spring stiffness in backend")            

    def check(self):
        return self._backend.check()


# DEPRECATED
class ParticleMaterialView:
    """
        .. warning::
            This class and its functionality is deprecated and will be removed.
            See :py:class:`DeformableMaterialView` as an alternative which is based on a successor deformable body feature.
    """
    def __init__(self, backend, frontend):
        self._backend = backend
        self._frontend = frontend

    @property
    def count(self):
        return self._backend.count

    def get_friction(self):
        if not hasattr(self, "_friction"):
            self._friction, self._friction_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_friction(self._friction_desc):
            raise Exception("Failed to get particle material friction from backend")

        return self._friction

    def set_friction(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_friction(data_desc, indices_desc):
            raise Exception("Failed to set particle material friction in backend")

    def get_damping(self):
        if not hasattr(self, "_damping"):
            self._damping, self._damping_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_damping(self._damping_desc):
            raise Exception("Failed to get particle material damping from backend")

        return self._damping

    def set_damping(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_damping(data_desc, indices_desc):
            raise Exception("Failed to set particle material damping in backend")

    def get_gravity_scale(self):
        if not hasattr(self, "_gravity_scale"):
            self._gravity_scale, self._gravity_scale_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_gravity_scale(self._gravity_scale_desc):
            raise Exception("Failed to get particle system particle material gravity scale from backend")

        return self._gravity_scale

    def set_gravity_scale(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_gravity_scale(data_desc, indices_desc):
            raise Exception("Failed to set particle system particle material gravity scale in backend")

    def get_lift(self):
        if not hasattr(self, "_lift"):
            self._lift, self._lift_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_lift(self._lift_desc):
            raise Exception("Failed to get particle material lift from backend")

        return self._lift

    def set_lift(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_lift(data_desc, indices_desc):
            raise Exception("Failed to set particle material lift in backend")

    def get_drag(self):
        if not hasattr(self, "_drag"):
            self._drag, self._drag_desc = self._frontend.create_tensor((self.count, 1), float32, -1)
        
        if not self._backend.get_drag(self._drag_desc):
            raise Exception("Failed to get particle material drag from backend")

        return self._drag

    def set_drag(self, data, indices):
        data = self._frontend.as_contiguous_float32(data)
        indices = self._frontend.as_contiguous_uint32(indices)
        data_desc = self._frontend.get_tensor_desc(data)
        indices_desc = self._frontend.get_tensor_desc(indices)

        if not self._backend.set_drag(data_desc, indices_desc):
            raise Exception("Failed to set particle material drag in backend")

    def check(self):
        return self._backend.check()
