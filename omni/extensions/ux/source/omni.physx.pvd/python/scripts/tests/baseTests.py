# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#


#from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
#import omni.physx
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase
from pathlib import Path
from omni.physxpvd.bindings import _physxPvd
import omni.usd
import carb.settings
import os
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf, Tf, UsdUtils, PhysxSchema

from omni.physxpvd.scripts.extension import PhysxPvdExtension, get_physx_pvd_interface

from omni.physxpvd.scripts.omniusd_to_physxusd.omniusd_to_physxusd import ConvertOmniPvdToPhysXUSD

from omni.physx.scripts import physicsUtils, particleUtils
from omni.physxfoundation import get_physx_foundation_interface
import time

def get_or_create_stage(path):
    try:
        stage = Usd.Stage.Open(path)
    except:
        stage = Usd.Stage.CreateNew(path)
    layer = stage.GetRootLayer();
    if layer != None:
        layer.Clear();
    return stage

def setup_env_2(stage, envID, envPos):
    for i in range(10):
        # Create a falling box
        boxPath = "/World/env_2_" + str(envID) + "/Box"
        box = physicsUtils.add_rigid_box(
            stage, 
            boxPath,
            size=Gf.Vec3d(1.0, 1.0, 1.0),
            density=1.0
        )
        # Position the box above the ground
        box.GetAttribute("xformOp:translate").Set(Gf.Vec3d(envPos[0]+i*2.0, envPos[1] + i*2.0 + 4.0, envPos[2]))

def setup_env(stage, envID, envPos):
    for i in range(10):
        # Create a falling box
        boxPath = "/World/env_1_" + str(envID) + "/Box"
        box = physicsUtils.add_rigid_box(
            stage, 
            boxPath,
            size=Gf.Vec3d(1.0, 1.0, 1.0),
            density=1.0
        )
        # Position the box above the ground
        box.GetAttribute("xformOp:translate").Set(Gf.Vec3d(envPos[0]+i*2.0, envPos[1] + i*2.0 + 4.0, envPos[2]))


def create_rope_articulation(stage,
                            envID,
                            initPos,
                            scale = 0.42,
                            nbBoxes = 15
):
    radius = 1.0 * scale
    halfHeight = 1.0 * scale
    
    pos = Gf.Vec3f(0.0, 0.0, 0.0)

    articulationPath = "/World/env_1_" + str(envID) + "/articulation"
    UsdGeom.Xform.Define(stage, articulationPath)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(articulationPath))

    parentName = articulationPath

    # Create rope
    for i in range(nbBoxes):
        # link
        linkName = articulationPath + "/articulationLink" + str(i)

        if i != 0:
            aticulatedJointName = articulationPath + "/articulatedRevoluteJoint" + str(i)

            component = UsdPhysics.RevoluteJoint.Define(stage, aticulatedJointName)
            val0 = [Sdf.Path(parentName)]
            val1 = [Sdf.Path(linkName)]

            if parentName != "":
                component.CreateBody0Rel().SetTargets(val0)
            component.CreateLocalPos0Attr().Set(Gf.Vec3f(radius + halfHeight, 0.0, 0.0))
            component.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

            component.CreateBody1Rel().SetTargets(val1)
            component.CreateLocalPos1Attr().Set(Gf.Vec3f(-(radius + halfHeight), 0.0, 0.0))
            component.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

            component.CreateAxisAttr("Z")
            component.CreateLowerLimitAttr(float(-180 / 32.0))
            component.CreateUpperLimitAttr(float(180 / 32.0))
        else:
            aticulatedJointName = articulationPath + "/rootJoint"
            component = UsdPhysics.FixedJoint.Define(stage, aticulatedJointName)

            val1 = [Sdf.Path(linkName)]
            component.CreateLocalPos0Attr().Set(initPos)
            component.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

            component.CreateBody1Rel().SetTargets(val1)
            component.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            component.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            
        capsuleGeom = UsdGeom.Capsule.Define(stage, linkName)
        capsulePrim = stage.GetPrimAtPath(linkName)
        capsuleGeom.CreateHeightAttr(halfHeight)
        capsuleGeom.CreateRadiusAttr(radius)
        capsuleGeom.CreateAxisAttr("X")
        capsuleGeom.AddTranslateOp().Set(initPos + pos)
        capsuleGeom.AddOrientOp().Set(Gf.Quatf(1.0))
        capsuleGeom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        #capsuleGeom.CreateDisplayColorAttr().Set([demo.get_primary_color()])

        UsdPhysics.CollisionAPI.Apply(capsulePrim)
        UsdPhysics.RigidBodyAPI.Apply(capsulePrim)            
        UsdPhysics.MassAPI.Apply(capsulePrim).CreateMassAttr().Set(1.0)  # Add mass of 1.0 to each link

        parentName = linkName
        pos[0] += (radius + halfHeight) * 2.0

class OmniPVDBaseTests(PhysicsKitStageAsyncTestCase):
    async def test_ovd_to_usd(self):
        filePath2 = str(Path(__file__).parent.parent.parent.parent.parent.joinpath("data/various.ovd"))
        upAxis = 1
        usdaType = 1

        # Just test the reading by passing an empty string for the output path
        self.assertTrue(get_physx_pvd_interface().ovd_to_usd(filePath2, "", upAxis, usdaType))
            
    async def test_messages(self):

        physxPvdInterface = get_physx_pvd_interface()

        messages = physxPvdInterface.get_messages()
        self.assertTrue(len(messages)==0, f"OVD messages not empty")

        ########################################################################
        # messages.ovd was created from the PhysX SDK unit test: OmniPvdPhysXTests.testRecordMessage
        # while also at the very top of the file: physics\physx\test\unittests\OmniPvd\src\OmniPvdPhysXTests.cpp
        # in the function: virtual void TearDown()
        # the if statement is set to true with the intended output OVD file
        # if (true)
		# {
		#    char binaryDumpFileName[] = "d:/ovd_dumps/messages.ovd";
        #
        # Execute the unit test like : --gtest_filter=*OmniPvdPhysXTests.testRecordMessage* for the SDKUnitTests project
        # to output the desired messages.ovd file for this OmniPhysics test
        ########################################################################

        filePathOVD = str(Path(__file__).parent.parent.parent.parent.parent.joinpath("data/messages.ovd"))
        upAxis = 1
        usdaType = 1
        # Load the OVD
        self.assertTrue(physxPvdInterface.ovd_to_usd(filePathOVD, "", upAxis, usdaType))
        messages = physxPvdInterface.get_messages()
        self.assertTrue(len(messages)>0, f"No OVD messages found")

        # Test the other path for messages to be filled
        physxPvdInterface.load_ovd(filePathOVD)
        messages = physxPvdInterface.get_messages()
        self.assertTrue(len(messages)>0, f"No OVD messages found")

    async def test_ovd_over_with_layer_creation(self):

        omni_data_path = carb.tokens.get_tokens_interface().resolve("${omni_data}")
        #print(f"omni_data_path: {omni_data_path}")

        physxPvdInterface = get_physx_pvd_interface()
        
        scene_setup_start = time.time()

        outputDir = str(Path(omni_data_path).joinpath("omnipvd_output"))
        outputDir = outputDir.replace("\\", "/")  # Normalize path separators
        if not outputDir.endswith("/"):
            outputDir += "/"  # Ensure path ends with forward slash
        os.makedirs(outputDir, exist_ok=True)  # Ensure directory exists
        #print(f"Output directory: {outputDir}")
        
        settings = carb.settings.get_settings()
        # First set the recording directory
        settings.set("/persistent/physics/omniPvdOvdRecordingDirectory", outputDir)
        # Then enable recording
        settings.set("/physics/omniPvdOutputEnabled", True)

        fileType = "usda"

        

        # Debug prints
        #print(f"Recording directory set to: {outputDir}")
        #print(f"Recording enabled: {settings.get('/physics/omniPvdOutputEnabled')}")
        #print(f"Recording directory from settings: {settings.get('/persistent/physics/omniPvdOvdRecordingDirectory')}")

        # Create a simulation stage and save it to disk
        stagePath = str(Path(outputDir).joinpath("stage_simulation." + fileType))
        stagePath = stagePath.replace("\\", "/")
        stage = get_or_create_stage(stagePath)

        # Create some metadata in the stage root layer
        stage.GetRootLayer().customLayerData= {"test": "test"}

        # Add a prim into the root layer of the stage

        

        # Create a simulation layer and save it to disk
        simulationLayerPath = str(Path(outputDir).joinpath("layer_simulation." + fileType))
        simulationLayerPath = simulationLayerPath.replace("\\", "/")
        simulationLayer = Sdf.Layer.CreateNew(simulationLayerPath)        
        # Add simulation layer to stage's layer stack
        stage.GetRootLayer().subLayerPaths.append(simulationLayerPath)

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        stage.SetEditTarget(stage.GetRootLayer())
        setup_env_2(stage, 1, Gf.Vec3d(0, 0, 0))

        # Set edit target to simulation layer
        stage.SetEditTarget(simulationLayer)

        # Create the physics scene
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)

        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())

        # The following numbers are enough for a 4k env scene
        physxSceneAPI.CreateGpuTotalAggregatePairsCapacityAttr().Set(500000)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr().Set(500000)
        physxSceneAPI.CreateGpuCollisionStackSizeAttr().Set(8 * 64 * 1024 * 1024)
        physxSceneAPI.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(500000)
        physxSceneAPI.CreateGpuMaxRigidPatchCountAttr().Set(500000)
        physxSceneAPI.CreateGpuMaxRigidContactCountAttr().Set(500000)
        
        # Create a ground plane
        ground = physicsUtils.add_ground_plane(
            stage, 
            "/World/ground",
            axis=UsdGeom.Tokens.y,
            size=1000,
            position=Gf.Vec3d(0, 0, 0),
            color=[0.5, 0.5, 0.5]
        )

        # Create envs
        side_length = 10
        for x in range(side_length):
            for z in range(side_length):
                envID = x*side_length + z + 1
                setup_env(stage, envID, Gf.Vec3d(x*30.0, 0, z*30.0))
                create_rope_articulation(stage, envID, Gf.Vec3f(x*30, 5.0, z*30))


        scene_setup_stop = time.time()

        sim_start = time.time()

        

        # Attach the stage to the context
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.Insert(stage).ToLongInt()
        omni.usd.get_context().attach_stage_with_callback(stage_id=stage_id, on_finish_fn=lambda success, message: None)
        
        

        # Attach to physics simulation
        from omni.physx import get_physx_simulation_interface
        physx = get_physx_simulation_interface()

        physx.attach_stage(stage_id)
        
        
        # Simulate for 10 steps at 60fps
        for i in range(10):
            physx.simulate(1.0/60.0, i * 1.0/60.0)
            physx.fetch_results()
            await omni.kit.app.get_app().next_update_async()

        sim_stop = time.time()
        
        physx.detach_stage()
        cache.Erase(stage)
        stage.Save()

        # detaching the stage will also close it and force the serialization of the OVD file

        recording_dir = settings.get("/persistent/physics/omniPvdOvdRecordingDirectory")
        if not recording_dir:
            self.fail("Failed to get recording directory from settings")
            
        # Debug prints for directory contents
        #print(f"Checking directory contents of: {recording_dir}")
        #print(f"Directory exists: {os.path.exists(recording_dir)}")
        #if os.path.exists(recording_dir):
        #    print(f"Directory contents: {os.listdir(recording_dir)}")
            
        # Find the most recent .ovd file in the recording directory
        import glob
        ovd_files = [f for f in glob.glob(os.path.join(recording_dir, "*.ovd")) if not f.endswith("tmp.ovd")]
        #print(f"Found OVD files: {ovd_files}")
        if not ovd_files:
            self.fail("No OVD files found in recording directory")
            
        # Get the most recent file
        inputOvdPath = max(ovd_files, key=os.path.getctime)
        #print(f"Using most recent OVD file: {inputOvdPath}")

        # Set the output stage filename
        stageFilename = "stage_over.usda"

        # Test with the recorded time range
        startTime = 0.0
        stopTime = 0.0  # Match the simulation duration

        verifyOverLayer = True

        ovd_start = time.time()

        # Call the function
        result = physxPvdInterface.ovd_to_usd_over_with_layer_creation(
            inputOvdPath, stagePath, outputDir, stageFilename, startTime, stopTime, True, verifyOverLayer)
        
        ovd_stop = time.time()

        # Verify the result
        self.assertTrue(result)

        # Verify the output files exist
        outputStagePath = Path(outputDir) / stageFilename
        layer_over_file = "layer_over." + fileType
        outputOverPath = Path(outputDir) / layer_over_file
        
        self.assertTrue(outputStagePath.exists(), f"Output stage file not found: {outputStagePath}")
        self.assertTrue(outputOverPath.exists(), f"Output over file not found: {outputOverPath}")

        # Clean up USD resources
        stage = None
        simulationLayer = None
        
        # Clean up PhysX resources
        physx = None
        
        # Disable recording
        settings.set("/physics/omniPvdOutputEnabled", False)
        
        #print(f"scene_setup_stop - scene_setup_start: {scene_setup_stop - scene_setup_start}, sim_stop - sim_start: {sim_stop - sim_start}, ovd_stop - ovd_start: {ovd_stop - ovd_start}")

        omni.usd.get_context().new_stage()

        return

        # Return from this function at this point if you would like to inspect the input and output files
        # Clean up temporary files
        if os.path.exists(stagePath):
            os.remove(stagePath)
        if os.path.exists(simulationLayerPath):
            os.remove(simulationLayerPath)
        if os.path.exists(outputStagePath):
            os.remove(outputStagePath)
        if os.path.exists(outputOverPath):
            os.remove(outputOverPath)
        if os.path.exists(inputOvdPath):
            os.remove(inputOvdPath)
        if os.path.exists(os.path.join(recording_dir, "tmp.ovd")):
            os.remove(os.path.join(recording_dir, "tmp.ovd"))


    async def test_ovd_recording_with_directory_creation(self):

        # This test is to verify that the OVD recording directory is created and that the OVD file is written to it
        physxPvdInterface = get_physx_pvd_interface()

        # Get the omni_data path
        omni_data_path = carb.tokens.get_tokens_interface().resolve("${omni_data}")
        outputDir = str(Path(omni_data_path).joinpath("omnipvd_output"))
        outputDir = outputDir.replace("\\", "/")  # Normalize path separators
        if not outputDir.endswith("/"):
            outputDir += "/"  # Ensure path ends with forward slash
        os.makedirs(outputDir, exist_ok=True)  # Ensure directory exists

        # Intentionally don't create the sub directories here,
        # instead test that the OVD file is written to the sub directories
        outputDir = str(Path(outputDir).joinpath("sub_dir_1/sub_dir_2"))

        # Set the recording directory and enable recording
        settings = carb.settings.get_settings()
        settings.set("/persistent/physics/omniPvdOvdRecordingDirectory", outputDir)
        settings.set("/physics/omniPvdOutputEnabled", True)

        # Create a simulation stage and save it to disk
        stagePath = str(Path(outputDir).joinpath("stage_simulation.usda"))
        stagePath = stagePath.replace("\\", "/")
        stage = get_or_create_stage(stagePath)

        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)

        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())

        # The following numbers are enough for a 4k env scene
        physxSceneAPI.CreateGpuTotalAggregatePairsCapacityAttr().Set(500000)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr().Set(500000)
        physxSceneAPI.CreateGpuCollisionStackSizeAttr().Set(8 * 64 * 1024 * 1024)
        physxSceneAPI.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(500000)
        physxSceneAPI.CreateGpuMaxRigidPatchCountAttr().Set(500000)
        physxSceneAPI.CreateGpuMaxRigidContactCountAttr().Set(500000)
        
        # Create a ground plane
        ground = physicsUtils.add_ground_plane(
            stage, 
            "/World/ground",
            axis=UsdGeom.Tokens.y,
            size=1000,
            position=Gf.Vec3d(0, 0, 0),
            color=[0.5, 0.5, 0.5]
        )

        # Create envs
        side_length = 4
        for x in range(side_length):
            for z in range(side_length):
                envID = x*side_length + z + 1
                setup_env(stage, envID, Gf.Vec3d(x*30.0, 0, z*30.0))
                create_rope_articulation(stage, envID, Gf.Vec3f(x*30, 5.0, z*30))


        # Attach the stage to the context
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.Insert(stage).ToLongInt()
        omni.usd.get_context().attach_stage_with_callback(stage_id=stage_id, on_finish_fn=lambda success, message: None)
        
        
        # Attach to physics simulation
        from omni.physx import get_physx_simulation_interface
        physx = get_physx_simulation_interface()

        physx.attach_stage(stage_id)
                
        for i in range(10):
            physx.simulate(1.0/60.0, i * 1.0/60.0)
            # the line below creates error spews related to a missing camera Prim
            #physx.fetch_results()
        
        physx.detach_stage()
        cache.Erase(stage)
        stage.Save()

        # detaching the stage will also close it and force the serialization of the OVD file

        recording_dir = settings.get("/persistent/physics/omniPvdOvdRecordingDirectory")
        if not recording_dir:
            self.fail("Failed to get recording directory from settings")

        if not os.path.exists(recording_dir):
            self.fail("Recording directory does not exist")
            
        # Find the most recent .ovd file in the recording directory
        import glob
        ovd_files = [f for f in glob.glob(os.path.join(recording_dir, "*.ovd")) if not f.endswith("tmp.ovd")]
        #print(f"Found OVD files: {ovd_files}")
        if not ovd_files:
            self.fail("No OVD files found in recording directory")

        # Clean up USD resources
        stage = None
        simulationLayer = None
        
        # Clean up PhysX resources
        physx = None
        
        # Disable recording
        settings.set("/physics/omniPvdOutputEnabled", False)            

    async def test_window(self):
        PhysxPvdExtension.instance._menu.show_window()

    async def test_omniusd_to_physxusd(self):

        filePath2 = str(Path(__file__).parent.parent.parent.parent.parent.joinpath("data/various_omnipvd_usd/stage.usda"))

        # Load the stage to be the active stage
        omni.usd.get_context().open_stage(filePath2)
        
        # Convert to PhysX USD, but as a stage in memory for the destination
        toPhysxUSDConverter = ConvertOmniPvdToPhysXUSD()

        outputStage = toPhysxUSDConverter.convert("", True)
        self.assertTrue(outputStage != None)
        self.assertTrue(outputStage.GetPrimAtPath("/rigiddynamic/PxActor_1_World_boxActor"))

        outputStage = None

        toPhysxUSDConverter = None

    async def test_ovd_bake_particles_and_rigid_bodies(self):
        """Test OVD baking with both particles and rigid bodies in the scene.
        Particles are only created and verified when a GPU is present."""

        omni_data_path = carb.tokens.get_tokens_interface().resolve("${omni_data}")

        physxPvdInterface = get_physx_pvd_interface()

        # Check if a GPU is available for particle simulation
        has_gpu = get_physx_foundation_interface().cuda_device_check()

        outputDir = str(Path(omni_data_path).joinpath("omnipvd_output_particles"))
        outputDir = outputDir.replace("\\", "/")
        if not outputDir.endswith("/"):
            outputDir += "/"
        os.makedirs(outputDir, exist_ok=True)

        settings = carb.settings.get_settings()
        settings.set("/persistent/physics/omniPvdOvdRecordingDirectory", outputDir)
        settings.set("/physics/omniPvdOutputEnabled", True)

        # Wait for the OmniPvd setting to take effect
        await omni.kit.app.get_app().next_update_async()

        fileType = "usda"

        # Create a simulation stage and save it to disk
        stagePath = str(Path(outputDir).joinpath("stage_particles_rigid." + fileType))
        stagePath = stagePath.replace("\\", "/")
        stage = get_or_create_stage(stagePath)

        # Create a simulation layer (following the pattern from test_ovd_over_with_layer_creation)
        simulationLayerPath = str(Path(outputDir).joinpath("layer_simulation_particles." + fileType))
        simulationLayerPath = simulationLayerPath.replace("\\", "/")
        simulationLayer = Sdf.Layer.CreateNew(simulationLayerPath)
        stage.GetRootLayer().subLayerPaths.append(simulationLayerPath)

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # Create rigid bodies on the root layer (following working test pattern)
        stage.SetEditTarget(stage.GetRootLayer())
        setup_env_2(stage, 1, Gf.Vec3d(0, 0, 0))

        # Set edit target to simulation layer for physics scene and particles
        stage.SetEditTarget(simulationLayer)

        # Create the physics scene
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)

        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        # Enable GPU dynamics only if a GPU is present (required for particle simulation)
        if has_gpu:
            physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(True)
        physxSceneAPI.CreateGpuTotalAggregatePairsCapacityAttr().Set(500000)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr().Set(500000)
        physxSceneAPI.CreateGpuCollisionStackSizeAttr().Set(8 * 64 * 1024 * 1024)
        physxSceneAPI.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(500000)
        physxSceneAPI.CreateGpuMaxRigidPatchCountAttr().Set(500000)
        physxSceneAPI.CreateGpuMaxRigidContactCountAttr().Set(500000)

        # Create a ground plane
        ground = physicsUtils.add_ground_plane(
            stage,
            "/World/ground",
            axis=UsdGeom.Tokens.y,
            size=1000,
            position=Gf.Vec3d(0, 0, 0),
            color=[0.5, 0.5, 0.5]
        )

        # Create some rigid bodies and articulations on simulation layer (like working test)
        setup_env(stage, 1, Gf.Vec3d(0, 0, 0))
        create_rope_articulation(stage, 1, Gf.Vec3f(0, 5.0, 0))

        ################################################################################
        # Create particle system and particles (only if GPU is present)
        ################################################################################
        if has_gpu:
            particle_system_path = Sdf.Path("/World/particleSystem")
            particleUtils.add_physx_particle_system(
                stage=stage,
                particle_system_path=particle_system_path,
                simulation_owner=scene.GetPath(),
                contact_offset=0.3,
                rest_offset=0.0,
                particle_contact_offset=0.2,
                solid_rest_offset=0.0,
                fluid_rest_offset=0.1,
                solver_position_iterations=4,
            )

            # Create PBD material for particles
            pbd_material_path = Sdf.Path("/World/PBDMaterial")
            particleUtils.add_pbd_particle_material(
                stage=stage,
                path=pbd_material_path,
                friction=0.1,
                viscosity=0.0,
                density=1000.0,
            )

            # Apply material to particle system
            particle_system_prim = stage.GetPrimAtPath(particle_system_path)
            physicsUtils.add_physics_material_to_prim(stage, particle_system_prim, pbd_material_path)

            # Create particle positions in a small grid
            particle_spacing = 0.2
            dim_x, dim_y, dim_z = 4, 4, 4  # Small 4x4x4 grid = 64 particles
            lower = Gf.Vec3f(-5.0, 8.0, 0.0)  # Starting position above ground

            positions_list, velocities_list = particleUtils.create_particles_grid(
                lower, particle_spacing, dim_x, dim_y, dim_z
            )
            widths_list = [particle_spacing * 0.5] * len(positions_list)

            # Create particle set
            particle_set_path = Sdf.Path("/World/particleSet")
            particleUtils.add_physx_particleset_points(
                stage=stage,
                path=particle_set_path,
                positions_list=positions_list,
                velocities_list=velocities_list,
                widths_list=widths_list,
                particle_system_path=particle_system_path,
                self_collision=True,
                fluid=True,
                particle_group=0,
                particle_mass=1.0,
                density=1000.0,
            )

        ################################################################################
        # Run simulation
        ################################################################################
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.Insert(stage).ToLongInt()
        omni.usd.get_context().attach_stage_with_callback(stage_id=stage_id, on_finish_fn=lambda success, message: None)

        from omni.physx import get_physx_simulation_interface
        physx = get_physx_simulation_interface()

        physx.attach_stage(stage_id)

        # Wait for physics to fully initialize (important for GPU particle data sync flag)
        await omni.kit.app.get_app().next_update_async()

        # Simulate for several steps
        for i in range(10):
            physx.simulate(1.0/60.0, i * 1.0/60.0)
            physx.fetch_results()
            await omni.kit.app.get_app().next_update_async()

        physx.detach_stage()
        cache.Erase(stage)
        stage.Save()

        ################################################################################
        # Test OVD baking
        ################################################################################
        recording_dir = settings.get("/persistent/physics/omniPvdOvdRecordingDirectory")
        if not recording_dir:
            self.fail("Failed to get recording directory from settings")

        # Find the most recent .ovd file
        import glob
        ovd_files = [f for f in glob.glob(os.path.join(recording_dir, "*.ovd")) if not f.endswith("tmp.ovd")]
        if not ovd_files:
            self.fail("No OVD files found in recording directory")

        inputOvdPath = max(ovd_files, key=os.path.getctime)

        # Set output stage filename
        stageFilename = "stage_particles_rigid_over.usda"

        # Test with the recorded time range
        startTime = 0.0
        stopTime = 0.0
        verifyOverLayer = True

        # Call the baking function
        result = physxPvdInterface.ovd_to_usd_over_with_layer_creation(
            inputOvdPath, stagePath, outputDir, stageFilename, startTime, stopTime, True, verifyOverLayer)

        # Verify the result
        self.assertTrue(result, "OVD baking failed")

        # Verify the output files exist
        outputStagePath = Path(outputDir) / stageFilename
        layer_over_file = "layer_over." + fileType
        outputOverPath = Path(outputDir) / layer_over_file

        self.assertTrue(outputStagePath.exists(), f"Output stage file not found: {outputStagePath}")
        self.assertTrue(outputOverPath.exists(), f"Output over file not found: {outputOverPath}")

        # Verify the output stage can be opened and contains expected prims
        overStage = Usd.Stage.Open(str(outputStagePath))
        self.assertTrue(overStage, "Failed to open output stage")

        # Verify key prims exist
        self.assertTrue(overStage.GetPrimAtPath("/World/env_1_1/Box"), "Box prim should exist")
        self.assertTrue(overStage.GetPrimAtPath("/World/env_1_1/articulation/articulationLink0"), "Articulation link should exist")

        # Print results for user
        print("\n=== Bake Test Results ===")
        print(f"Output stage: {outputStagePath}")
        print(f"OVD file: {inputOvdPath}")

        boxPrim = overStage.GetPrimAtPath("/World/env_1_1/Box")
        if boxPrim:
            transformAttr = boxPrim.GetAttribute("xformOp:transform")
            if transformAttr:
                print(f"Rigid body time samples: {len(transformAttr.GetTimeSamples())}")

        # Only verify particle baking results if GPU was available
        if has_gpu:
            self.assertTrue(overStage.GetPrimAtPath("/World/particleSet"), "Particle set prim should exist")
            particleSetPrim = overStage.GetPrimAtPath("/World/particleSet")
            if particleSetPrim:
                pointsAttr = particleSetPrim.GetAttribute("points")
                if pointsAttr:
                    print(f"Particle time samples: {len(pointsAttr.GetTimeSamples())}")
        else:
            print("GPU not available - particle baking verification skipped")
        print("=========================\n")

        overStage = None

        # Cleanup
        stage = None
        physx = None
        settings.set("/physics/omniPvdOutputEnabled", False)

        omni.usd.get_context().new_stage()
