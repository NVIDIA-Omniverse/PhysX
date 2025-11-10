# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from omni.physxpvd.scripts.extension import PhysxPvdExtension

from omni.physxpvd.scripts.omniusd_to_physxusd.omniusd_to_physxusd import ConvertOmniPvdToPhysXUSD

from omni.physx.scripts import physicsUtils
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

        self._physxPvdInterface = _physxPvd.acquire_physx_pvd_interface()

        filePath2 = str(Path(__file__).parent.parent.parent.parent.parent.joinpath("data/various.ovd"))
        print(filePath2)
        upAxis = 1
        usdaType = 1

        # Just test the reading by passing an empty string for the output path
        self.assertTrue(self._physxPvdInterface.ovd_to_usd(filePath2, "", upAxis, usdaType))
        
        self._physxPvdInterface = None
            
    async def test_messages(self):

        self._physxPvdInterface = _physxPvd.acquire_physx_pvd_interface()

        messages = self._physxPvdInterface.get_messages()
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
        self.assertTrue(self._physxPvdInterface.ovd_to_usd(filePathOVD, "", upAxis, usdaType))
        messages = self._physxPvdInterface.get_messages()
        self.assertTrue(len(messages)>0, f"No OVD messages found")

        # Test the other path for messages to be filled
        self._physxPvdInterface.load_ovd(filePathOVD)
        messages = self._physxPvdInterface.get_messages()
        self.assertTrue(len(messages)>0, f"No OVD messages found")
        
        self._physxPvdInterface = None


    async def test_ovd_over_with_layer_creation(self):

        omni_data_path = carb.tokens.get_tokens_interface().resolve("${omni_data}")
        #print(f"omni_data_path: {omni_data_path}")

        self._physxPvdInterface = _physxPvd.acquire_physx_pvd_interface()
        

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
        
        
        # Simulate for 100 steps at 60fps
        for i in range(10):
            physx.simulate(1.0/60.0, i * 1.0/60.0)
            # the line below creates error spews related to a missing camera Prim
            #physx.fetch_results()

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
        result = self._physxPvdInterface.ovd_to_usd_over_with_layer_creation(
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
        self._physxPvdInterface = None
        
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

    async def test_window(self):
        PhysxPvdExtension.instance._menu.show_window()

    async def test_omniusd_to_physxusd(self):

        filePath2 = str(Path(__file__).parent.parent.parent.parent.parent.joinpath("data/various_omnipvd_usd/stage.usda"))
        print(filePath2)

        # Load the stage to be the active stage
        omni.usd.get_context().open_stage(filePath2)
        
        # Convert to PhysX USD, but as a stage in memory for the destination
        toPhysxUSDConverter = ConvertOmniPvdToPhysXUSD()

        outputStage = toPhysxUSDConverter.convert("", True)
        self.assertTrue(outputStage != None)
        self.assertTrue(outputStage.GetPrimAtPath("/rigiddynamic/PxActor_1_World_boxActor"))

        outputStage = None

        toPhysxUSDConverter = None

