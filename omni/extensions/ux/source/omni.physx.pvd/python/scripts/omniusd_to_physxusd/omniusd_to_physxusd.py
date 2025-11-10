# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
import carb
from omni import ui
from omni.physxpvd.bindings import _physxPvd
import omni.usd
from pprint import pprint
from datetime import datetime
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf
import os, sys

class ConvertOmniPvdToPhysXUSD():

    ################################################################################
    # See if there is already a USD Stage with the path provided, if so just
    # return a reference USD Stage to it, otherwise create a new USD Stage.
    ################################################################################
    def get_or_create_stage(self,path):
        try:
            stage = Usd.Stage.Open(path)
        except:
            stage = Usd.Stage.CreateNew(path)
        layer = stage.GetRootLayer();
        if layer != None:
            layer.Clear();
        return stage

    ################################################################################
    # Initialize the PhysX Scene of the output Stage
    ################################################################################
    def init_physx_stage(self, stagePath, upAxis):
        outputStage = self.get_or_create_stage(stagePath)

        UsdGeom.SetStageUpAxis(outputStage, upAxis)

        UsdGeom.SetStageMetersPerUnit(outputStage, 0.01)
        outputScene = UsdPhysics.Scene.Define(outputStage, "/physicsScene")

        if UsdGeom.Tokens.y == upAxis:
            outputScene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        else:
            outputScene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))

        outputScene.CreateGravityMagnitudeAttr().Set(981.0)
        return outputStage

    ################################################################################
    # True if the prim is visible, else False
    ################################################################################
    def is_visible(self, pvdPrim, kitSampleFrame):
        pvdXform = UsdGeom.Xformable(pvdPrim)
        pvdVisibilityAttr = pvdXform.GetVisibilityAttr()
        visibility = pvdVisibilityAttr.Get(kitSampleFrame)
        if visibility!="invisible":
            return True
        else:
            return False
    
    ################################################################################
    # True if the parent prim is visible, else False
    ################################################################################
    def is_parent_visible(self, pvdPrim, kitSampleFrame):
        pvdParentPrim = pvdPrim.GetParent()
        return self.is_visible(pvdParentPrim,kitSampleFrame)

    ################################################################################
    # Returns the path of the output PhysX Actor
    ################################################################################
    def get_physx_actor_parent_path(self, pvdPhysXActorPrim, isStatic):
        pvdPhysXActorPrimName = str(pvdPhysXActorPrim.GetName())
        pvdPhysXActorIndexStart = pvdPhysXActorPrimName.find('_') + 1
        pvdPhysXActorIndexString = pvdPhysXActorPrimName[pvdPhysXActorIndexStart:]
        if isStatic:
            return "/rigidstatic/PxActor_" + pvdPhysXActorIndexString
        else:
            return "/rigiddynamic/PxActor_" + pvdPhysXActorIndexString

    ################################################################################
    # Returns the index of the PhysX Actor Shape as a string
    ################################################################################
    def get_physx_actor_shape_index(self, pvdPhysXActorShapePrim):
        pvdPhysXActorShapeName = str(pvdPhysXActorShapePrim.GetName())
        pvdPhysXActorShapeIndexStart = pvdPhysXActorShapeName.find('_') + 1
        return pvdPhysXActorShapeName[pvdPhysXActorShapeIndexStart:]

    def get_physx_actor_base_link_path(self, pvdArtBasePrim):
        pvdPhysXArtBasePrimName = str(pvdArtBasePrim.GetName())    
        pvdPhysXArtBaseIndexStart = pvdPhysXArtBasePrimName.find('_') + 1
        return "/articulations/PxArticulationReducedCoordinate_" + pvdPhysXArtBasePrimName[pvdPhysXArtBaseIndexStart:]

    ################################################################################
    # Returns the path of the output PhysX Actor Link
    # Should be formatted similar to : "/artbase/arb_n/arl_n"
    ################################################################################
    def get_physx_actor_parent_link_path(self, pvdPhysXActorPrim):
        ################################################################################
        # Get the arl_ index directly from the primName of the Prim
        ################################################################################
        pvdPhysXActorPrimName = str(pvdPhysXActorPrim.GetName())
        pvdPhysXActorIndexStart = pvdPhysXActorPrimName.find('_') + 1
        pvdPhysXActorIndexString = pvdPhysXActorPrimName[pvdPhysXActorIndexStart:]
        ################################################################################
        # Get the arb_ index from the "/scenes_n/artbase/arb_n " primPath
        ################################################################################
        pvdPhysXActorPrimPath = str(pvdPhysXActorPrim.GetPrimPath())
        articulationSubStringIndexStart = pvdPhysXActorPrimPath.find("PxArticulationReducedCoordinate_") + 33
        articulationSubStringIndexStop = pvdPhysXActorPrimPath.find('/', articulationSubStringIndexStart)
        articulationIndexString = pvdPhysXActorPrimPath[articulationSubStringIndexStart:articulationSubStringIndexStop]

        pvdArticulationPath = "/articulations/PxArticulationReducedCoordinate_"
        physXActorParentPath = pvdArticulationPath + articulationIndexString + "/PxActor_" + pvdPhysXActorIndexString
        #print("actoPArentPath" + physXActorParentPath)
        return physXActorParentPath

    ################################################################################
    # PhysX actor (rs/rd) is two levels above the geom, we are bb_, example below
    #   /rs_1/s_1/bb_1
    #   /rd_1/s_1/bb_1
    ################################################################################
    def get_shape_path_for_simple_geom(self, pvdPrim, isStatic):
        actorPath = self.get_physx_actor_parent_path(pvdPrim.GetParent().GetParent(), isStatic)
        shapeIndexString = self.get_physx_actor_shape_index(pvdPrim.GetParent())
        return actorPath + "/PxShape_" + shapeIndexString

    ################################################################################
    # PhysX actor (rs/rd) is three levels above the geom, we are m_, example below
    #   /rs_1/s_1/o/m_1
    #   /rd_1/s_1/o/m_1
    ################################################################################
    def get_shape_path_for_refererenced_geom(self, pvdPrim, isStatic):
        actorPath = self.get_physx_actor_parent_path(pvdPrim.GetParent().GetParent().GetParent(), isStatic)
        shapeIndexString = self.get_physx_actor_shape_index(pvdPrim.GetParent().GetParent())
        return actorPath + "/PxShape_" + shapeIndexString

    def get_shape_path_for_simple_articulation_geom(self, pvdPrim):
        parentPath = self.get_physx_actor_parent_link_path(pvdPrim.GetParent().GetParent())
        shapeIndexString = self.get_physx_actor_shape_index(pvdPrim.GetParent())
        return parentPath + "/PxShape_" + shapeIndexString

    def get_shape_path_for_referenced_articulation_geom(self, pvdPrim):
        parentPath = self.get_physx_actor_parent_link_path(pvdPrim.GetParent().GetParent().GetParent())
        shapeIndexString = self.get_physx_actor_shape_index(pvdPrim.GetParent().GetParent())
        return parentPath + "/PxShape_" + shapeIndexString

    def copy_pvd_xform_and_color_to_shape(self, outputStage, pvdPrim, pvdShapeGeom, xfCache, shapePath, shapeGeom):
        shapePrim = outputStage.GetPrimAtPath(shapePath)
        pose = xfCache.GetLocalToWorldTransform(pvdPrim)
        shapeXform = UsdGeom.Xformable(shapePrim)
        shapeXform.MakeMatrixXform().Set(pose)
        UsdPhysics.CollisionAPI.Apply(shapePrim)
        shapeGeom.CreateDisplayColorAttr().Set(pvdShapeGeom.GetDisplayColorAttr().Get())

    def make_physx_actor_xform(self, outputStage, primName, pvdPrim, xfCache, kitSampleFrame, isStatic):
        visible = self.is_visible(pvdPrim,kitSampleFrame)
        if visible:
            actorPath = self.get_physx_actor_parent_path(pvdPrim, isStatic)
            ################################################################################
            # Initialize the USD Prim
            ################################################################################
            actorGeom = UsdGeom.Xform.Define(outputStage, actorPath)
            actorPrim = outputStage.GetPrimAtPath(actorPath)
            ################################################################################
            # TODO : apply "PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"
            # Also make all the sub prims such as shapes have local relative tforms, so that
            # the actor and shapes can have their separate transforms
            ################################################################################
            if not isStatic:
                rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(actorPrim)

    def make_bbox(self, outputStage, primName, pvdPrim, xfCache, kitSampleFrame, isStatic, isArticulation):
        # should also check visibility of the grandparent, right
        visibleParent = self.is_parent_visible(pvdPrim,kitSampleFrame)
        if visibleParent:
            if isArticulation:
                shapePath = self.get_shape_path_for_simple_articulation_geom(pvdPrim)
            else:
                shapePath = self.get_shape_path_for_simple_geom(pvdPrim, isStatic)
            ################################################################################
            # Cube specific Prim initialization
            ################################################################################
            shapeGeom = UsdGeom.Cube.Define(outputStage, shapePath)
            pvdShapeGeom = UsdGeom.Cube(pvdPrim)
            shapeGeom.CreateSizeAttr().Set(pvdShapeGeom.GetSizeAttr().Get())

            self.copy_pvd_xform_and_color_to_shape(outputStage, pvdPrim, pvdShapeGeom, xfCache, shapePath, shapeGeom)

    def make_sphere(self, outputStage, primName, pvdPrim, xfCache, kitSampleFrame, isStatic, isArticulation):
        visibleParent = self.is_parent_visible(pvdPrim,kitSampleFrame)
        if visibleParent:
            if isArticulation:
                shapePath = self.get_shape_path_for_simple_articulation_geom(pvdPrim)
            else:
                shapePath = self.get_shape_path_for_simple_geom(pvdPrim, isStatic)
            ################################################################################
            # Sphere specific Prim initialization
            ################################################################################
            shapeGeom = UsdGeom.Sphere.Define(outputStage, shapePath)
            pvdShapeGeom = UsdGeom.Sphere(pvdPrim)
            shapeGeom.CreateRadiusAttr().Set(pvdShapeGeom.GetRadiusAttr().Get())

            self.copy_pvd_xform_and_color_to_shape(outputStage, pvdPrim, pvdShapeGeom, xfCache, shapePath, shapeGeom)

    def make_capsule(self, outputStage, primName, pvdPrim, xfCache, kitSampleFrame, isStatic, isArticulation):
        visibleParent = self.is_parent_visible(pvdPrim,kitSampleFrame)
        if visibleParent:
            if isArticulation:
                shapePath = self.get_shape_path_for_simple_articulation_geom(pvdPrim)
            else:
                shapePath = self.get_shape_path_for_simple_geom(pvdPrim, isStatic)
            ################################################################################
            # Capsule specific Prim initialization
            ################################################################################
            shapeGeom = UsdGeom.Capsule.Define(outputStage, shapePath)
            pvdShapeGeom = UsdGeom.Capsule(pvdPrim)
            shapeGeom.CreateHeightAttr().Set(pvdShapeGeom.GetHeightAttr().Get())
            shapeGeom.CreateRadiusAttr().Set(pvdShapeGeom.GetRadiusAttr().Get())
            shapeGeom.CreateAxisAttr().Set(pvdShapeGeom.GetAxisAttr().Get())

            self.copy_pvd_xform_and_color_to_shape(outputStage, pvdPrim, pvdShapeGeom, xfCache, shapePath, shapeGeom)

    def make_mesh(self, outputStage, primName, pvdPrim, xfCache, kitSampleFrame, isStatic, isArticulation):
        ################################################################################
        # TODO : Check for visibility
        ################################################################################
        if pvdPrim.IsA(UsdGeom.Mesh):
            if isArticulation:
                shapePath = self.get_shape_path_for_referenced_articulation_geom(pvdPrim)
            else:
                shapePath = self.get_shape_path_for_refererenced_geom(pvdPrim, isStatic)
            ################################################################################
            # Mesh specific Prim initialization
            ################################################################################
            shapeGeom = UsdGeom.Mesh.Define(outputStage, shapePath)
            pvdShapeGeom = UsdGeom.Mesh(pvdPrim)
            pvdMeshFaceVertexCount = pvdShapeGeom.GetFaceVertexCountsAttr().Get()
            pvdMeshFaceVertexIndices = pvdShapeGeom.GetFaceVertexIndicesAttr().Get()
            pvdMeshPoints = pvdShapeGeom.GetPointsAttr().Get()
            pvdMeshNormals = pvdShapeGeom.GetNormalsAttr().Get()

            shapeGeom.CreateFaceVertexCountsAttr().Set(pvdMeshFaceVertexCount)
            shapeGeom.CreateFaceVertexIndicesAttr().Set(pvdMeshFaceVertexIndices)
            shapeGeom.CreatePointsAttr().Set(pvdMeshPoints)
            shapeGeom.CreateDoubleSidedAttr().Set(False)
            shapeGeom.CreateNormalsAttr().Set(pvdMeshNormals)
            ################################################################################
            # If a mesh is dynamic, then it :
            #   is assumed to be a convex
            #   gets applied both the RigidBodyAPI as well as the MeshCollisionAPI
            ################################################################################
            if not isStatic:
                shapePrim = outputStage.GetPrimAtPath(shapePath)
                meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(shapePrim)
                meshCollisionAPI.CreateApproximationAttr("convexHull")
            ################################################################################
            # Copy display color
            ################################################################################
            self.copy_pvd_xform_and_color_to_shape(outputStage, pvdPrim, pvdShapeGeom, xfCache, shapePath, shapeGeom)

    ################################################################################
    # Creates a PhysXUSD ArticulationRootAPI, assumes the primName is "arb_"
    ################################################################################
    def make_articulation_base(self, outputStage, primName, pvdPrim, xfCache, kitSampleFrame):
        artBasePath = self.get_physx_actor_base_link_path(pvdPrim)
        UsdGeom.Xform.Define(outputStage, artBasePath)
        UsdPhysics.ArticulationRootAPI.Apply(outputStage.GetPrimAtPath(artBasePath))

    ################################################################################
    # Acts as a PhysX RigidBody Actor
    ################################################################################
    def make_articulation_link(self, outputStage, primName, pvdPrim, xfCache, kitSampleFrame):
        linkPath = self.get_physx_actor_parent_link_path(pvdPrim)
        linkXform = UsdGeom.Xform.Define(outputStage, linkPath)
        linkPrim = outputStage.GetPrimAtPath(linkPath)
        UsdPhysics.RigidBodyAPI.Apply(linkPrim)

    def make_articulation_joint(self, outputStage, primName, pvdPrim, xfCache, kitSampleFrame):
        ################################################################################
        # A joint in PvdUSD is formatted as : /scenes_1/artbase/arb_1/arl_1/arl_2/arj_1
        # but we need to extract that into something PhysXUSD understands, which doesn't
        # yet have a notion of separate scenes.
        #
        # Extract the parent, val0 from : /scenes_1/artbase/arb_1/arl_1
        #   gives: /artbase/arb_1/arl_1
        #
        # Extract the child, val1 from : /scenes_1/artbase/arb_1/arl_1/arl_2
        #   gives: /artbase/arb_1/arl_2
        #
        # Set a joint name : /artbase/arb_1/arl_2/arj_1
        #
        ################################################################################

        parentLinkPath = self.get_physx_actor_parent_link_path(pvdPrim.GetParent().GetParent())
        childLinkPath = self.get_physx_actor_parent_link_path(pvdPrim.GetParent())
        jointPath = childLinkPath + "/" + pvdPrim.GetName()

        print("jointPathPvd: " + str(pvdPrim.GetPrimPath()) + " jointNamePvd: " + primName)
        print("parentLinkPath: " + parentLinkPath + " childLinkPath: " + childLinkPath)
        print("jointPath: " + jointPath)

        # joint definition between the articulation links
        joint = UsdPhysics.RevoluteJoint.Define(outputStage, jointPath)
        val0 = [Sdf.Path(parentLinkPath)]
        val1 = [Sdf.Path(childLinkPath)]

        joint.CreateBody0Rel().SetTargets(val0)
        joint.CreateBody1Rel().SetTargets(val1)
        #joint.CreateBreakForceAttr().Set(sys.float_info.max)
        #joint.CreateBreakTorqueAttr().Set(sys.float_info.max)
        joint.CreateAxisAttr("Y")
        joint.CreateLowerLimitAttr(float(-3.14 / 32.0))
        joint.CreateUpperLimitAttr(float(3.14 / 32.0))


    def make_geom(self, outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation):
        if "PxGeomBox" in primName:
            self.make_bbox(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)
        elif "PxGeomSphere" in primName:
            self.make_sphere(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)
        elif "PxGeomCapsule" in primName:
            self.make_capsule(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)
        elif ("PxConvexMesh_" in primName):
            self.make_mesh(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)
        elif ("PxTriangleMesh_" in primName):
            self.make_mesh(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)
        elif ("PxHeightField_" in primName):
            self.make_mesh(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)
        elif "PxGeomPlane" in primName:
            self.make_mesh(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)

#        elif ("convexmesh_" in primName) and (not ("convexmesh_ref" in primName)):
#            self.make_mesh(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)
#        elif ("trianglemesh_" in primName) and (not ("trianglemesh_ref" in primName)):
#            self.make_mesh(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)
#        elif ("heightfield_" in primName) and (not ("heightfield_ref" in primName)):
#            self.make_mesh(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, isArticulation)

    ################################################################################
    # The main loop of the PVD USD frame snapshot to PhysX USD conversion
    ################################################################################
    def convert(self, outputStagePath, isInMemoryStage):
        self._usd_context = omni.usd.get_context()
        self.stage = self._usd_context.get_stage()

        upAxis = UsdGeom.GetStageUpAxis(self.stage)

        if not isInMemoryStage:
            outputStage = self.init_physx_stage(outputStagePath, upAxis)
        else:
            outputStage = Usd.Stage.CreateInMemory()
        
        ################################################################################
        # Prepare the transform cache with the current time in the UI
        ################################################################################
        xfCache = UsdGeom.XformCache()
        kitTimeline = omni.timeline.get_timeline_interface()
        kitTime = kitTimeline.get_current_time()
        kitSamplesPerTime = kitTimeline.get_time_codes_per_seconds()
        kitSampleFrame = kitTime * kitSamplesPerTime
        xfCache.SetTime(kitSampleFrame)

        ################################################################################
        # For all prims :
        #   is the prim in scene_x?  only one scene can be sampled and output
        #   is prim visible at samplingTime?
        #     primPath contains "/artbase/" -> articulation
        #       primName is "arb_x" -> articulation base
        #       primName is "arl_x" -> articulation link
        #       primName is "arj_x" -> joint
        #         jointType is "" -> root joint / fixed joint
        #         jointType is "2" -> revolute joint
        #     primPath contains "rigid" -> rigid body
        #       primPath contains "dynamic" -> dynamic
        #         primName is "bb_x" -> box
        #         primName is "sp_x" -> sphere
        #       primPath contains "static" -> static
        ################################################################################

        ################################################################################
        # Get all prims in the stage
        ################################################################################

        ################################################################################
        # TODO : 1) scene_1 below should be possible to chose, not hardcoded to the first
        #           scene in a stage.
        ################################################################################        
        for prim in self.stage.Traverse():            
            primPath = str(prim.GetPrimPath())
            primName = str(prim.GetName())
            #print("primPath(" + primPath + ") primName(" + primName +")")
            if "/PxScene_1/" in primPath:
                if "/articulations/" in primPath:
                    if primName.startswith("PxArticulationReducedCoordinate_"):
                        self.make_articulation_base(outputStage, primName, prim, xfCache, kitSampleFrame)
                    elif primName.startswith("PxActor_"):
                        self.make_articulation_link(outputStage, primName, prim, xfCache, kitSampleFrame)
                    elif primName.startswith("PxArticulationJointReducedCoordinate_"):
                        self.make_articulation_joint(outputStage, primName, prim, xfCache, kitSampleFrame)
                    else:
                        isStatic = False
                        self.make_geom(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, True)
                elif "/rigid" in primPath:
                    isStatic = True
                    if "/rigiddynamic" in primPath:
                        isStatic = False
                    if primName.startswith("PxActor_"):
                        self.make_physx_actor_xform(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic)
                    else:
                        self.make_geom(outputStage, primName, prim, xfCache, kitSampleFrame, isStatic, False)

        # Write the resulting output Stage
        if not isInMemoryStage:
            outputStage.Save()
            carb.log_info("OmniPVD PhysX USD Stage successfully saved to :" + outputStagePath);
            return None
        else:
            return outputStage;
        
