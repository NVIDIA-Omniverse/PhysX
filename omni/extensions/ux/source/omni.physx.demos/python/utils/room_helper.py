# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni
import math
from omni.physx.scripts import physicsUtils
from pxr import Usd, UsdLux, UsdGeom, Gf, UsdPhysics, UsdShade, Sdf, PhysxSchema
import omni.kit.viewport.utility as vp_utils
import omni.kit.commands
import os
import omni.physx.scripts.utils as core_utils
import carb.settings

# Boom is an optional dependency
# This code will try to get the interface, but gracefully handles it not being loaded
# Code to set up Boom data will be bypassed if this returns None
def get_boom_api():
    try:
        import omni.audio.boom.bindings._boom as boom_api
        return boom_api.acquire_interface()
    except ModuleNotFoundError:
        pass
    return None


class InstanceItem:
    def __init__(self, hasCollision, protoPath, instancePath, materialPath):
        self.hasCollision = hasCollision
        self.protoPath = protoPath
        self.instancePath = instancePath
        self.materialPath = materialPath

# convert untyped paths to scope
def get_path_as_scope(stage, path):
    pathStr = str(path)
    xpArr = pathStr.split("/")
    lenM1 = len(xpArr)-1
    finalPathRoot = ""
    for i in range(lenM1):
        if not xpArr[i]:
            continue
        finalPathRoot = finalPathRoot + "/" + xpArr[i]
        if not stage.GetPrimAtPath(finalPathRoot):
            UsdGeom.Scope.Define(stage, finalPathRoot)

class RoomInstancer:

    def get_proto_path(self, room):
        defaultPrimPath = self._stage.GetDefaultPrim().GetPath()
        if room._roomTemplateStr not in self._roomPrototypePath:
            protoPath = self._roomPrototypePath[room._roomTemplateStr] = defaultPrimPath.AppendPath("roomPrototype_" + room._roomTemplateStr)
            xform = UsdGeom.Xform.Define(self._stage, protoPath)
            imageable = UsdGeom.Imageable.Get(self._stage, protoPath)
            imageable.MakeInvisible()

        return self._roomPrototypePath[room._roomTemplateStr]

    def __init__(self, stage):
        self._stage = stage
        self._shapeList = None
        self._hasInstance = {}
        self._roomPrototypePath = {}
        self._baseCoordinate = {}

    def link_instances(self, room, instanceItem, coordinate):
        xformPath = instanceItem.instancePath
        get_path_as_scope(room._stage, xformPath)

        xform = UsdGeom.Xform.Define(self._stage, xformPath)
        xform.GetPrim().GetReferences().AddInternalReference(instanceItem.protoPath)
        xform.GetPrim().SetInstanceable(True)
        xform.AddTranslateOp().Set(coordinate)
        imageable = UsdGeom.Imageable.Get(self._stage, xformPath)
        imageable.MakeVisible()

    def has_instance(self, instanceStr):
        return instanceStr in self._hasInstance

    def end_setup(self, room, coordinate):
        # we must negate the offset in the prototype
        if not self.has_instance(room._roomTemplateStr):
            self._hasInstance[room._roomTemplateStr] = True
            self._baseCoordinate[room._roomTemplateStr] = Gf.Vec3d(0.0, 0.0, coordinate[2])

        for instanceItem in room._instanceArr:
            self.link_instances(room, instanceItem, coordinate - self._baseCoordinate[room._roomTemplateStr])


    def __del__(self):
        pass

    def on_shutdown(self):
        pass

class WallTemplate():
    def __init__(self, wallHeight = 0, windowHeight = 0, windowWidth = 0, numWindows = 0, isArch = False, wallColor = Gf.Vec3f(136.0, 191.0, 152.0)/255.0, template = ""):
        self.wallHeight = wallHeight
        self.windowHeight = windowHeight
        self.windowWidth = windowWidth
        self.numWindows = numWindows
        self.isArch = isArch
        self.wallColor = wallColor

        if template == "museumRoom":
            self.wallColor = Gf.Vec3f(224, 125, 79)*0.75/255.0
        if template == "museumHall":
            self.wallColor = Gf.Vec3f(224, 125, 79)*0.75/255.0
        if template == "museumRoomBack":
            self.wallColor = Gf.Vec3f(224, 125, 79)*0.75/255.0
            # self.wallColor = Gf.Vec3f(203, 195, 219)/255.0
            # self.wallColor = Gf.Vec3f(43, 51, 37)/255.0

class RoomTemplate():
    def __init__(self, walls = {}, roofLevel = -1, roofAxis = UsdGeom.Tokens.x, tileCol0 = Gf.Vec3f(0.1, 0.15, 0.25)*5.0, tileCol1 = Gf.Vec3f(0.05, 0.1, 0.2)*5.0, template = ""):
        # wall and pillar configuration:
        # + -------- x -
        #  1   N   0   y
        #      0       |
        #  W 3   2 E   |
        #      1       |
        #  3   S   2   +

        self.walls = walls
        self.roofLevel = roofLevel
        self.roofAxis = roofAxis
        self.tileCol0 = tileCol0
        self.tileCol1 = tileCol1

        if template == "museumRoom":
            self.roofAxis = UsdGeom.Tokens.y
            
            # self.tileCol0 = Gf.Vec3f(0.0, 0.0, 0.1)
            # self.tileCol1 = Gf.Vec3f(0.0, 0.0, 0.15)
            # self.tileCol0 = Gf.Vec3f(0.1, 0.15, 0.25)*5.0
            # self.tileCol1 = Gf.Vec3f(0.05, 0.1, 0.2)*5.0
            # self.tileCol0 = Gf.Vec3f(0.9)
            # self.tileCol1 = Gf.Vec3f(1.0)
        if template == "museumHall":
            self.roofAxis = UsdGeom.Tokens.x
            # self.tileCol0 = Gf.Vec3f(0.1)
            # self.tileCol1 = Gf.Vec3f(0.13)
            # self.tileCol0 = Gf.Vec3f(0.9)
            # self.tileCol1 = Gf.Vec3f(1.0)
            # self.tileCol0 = Gf.Vec3f(0.0, 0.0, 0.1)
            # self.tileCol1 = Gf.Vec3f(0.0, 0.0, 0.15)
            self.tileCol0 = Gf.Vec3f(11, 19, 31)/255.0
            self.tileCol1 = Gf.Vec3f(11, 19, 31)*0.5/255.0

        directions = ["N","S","E","W"]
        for i in directions:
            if i not in self.walls:
                self.walls[i] = WallTemplate(wallHeight = 0, windowHeight = 0, windowWidth = 0, numWindows = 0)

class RoomHelper():

    museumScale = 2
    roofHeightMuseum = 340 * museumScale
    museumHeight1 = 600
    museumHeight2 = museumHeight1 + roofHeightMuseum
    museumHeight3 = museumHeight1 + roofHeightMuseum * 2.0
    museumWallHeights = [museumHeight1, museumHeight2, museumHeight3]

    roomTemplates = {
        "default": RoomTemplate( walls = {
            "N": WallTemplate(wallHeight = 250, windowHeight = 125, windowWidth = 80, numWindows = 3),
            "E": WallTemplate(wallHeight = 250, windowHeight = 125, windowWidth = 80, numWindows = 3),
        }),

        "arcade": RoomTemplate( tileCol0 = Gf.Vec3f(0.9), tileCol1 = Gf.Vec3f(1.0), walls = {
            "N": WallTemplate(wallHeight = 300, windowHeight = 150, windowWidth = 150, numWindows = 3, isArch = True),
            "E": WallTemplate(wallHeight = 300, windowHeight = 150, windowWidth = 150, numWindows = 3, isArch = True),
        }),

        "arcadeInverse": RoomTemplate( tileCol0 = Gf.Vec3f(0.9), tileCol1 = Gf.Vec3f(1.0), walls = {
            "S": WallTemplate(wallHeight = 300, windowHeight = 150, windowWidth = 150, numWindows = 3, isArch = True),
            "W": WallTemplate(wallHeight = 300, windowHeight = 150, windowWidth = 150, numWindows = 3, isArch = True),
        }),

        

        "museum_room_north_east": RoomTemplate( template = "museumRoom", roofLevel = museumHeight1, walls = {
            "N": WallTemplate(template = "museumRoomBack", wallHeight = museumHeight2, windowHeight = museumScale*250, windowWidth = museumScale*80, numWindows = 3),
            "E": WallTemplate(template = "museumRoom", wallHeight = museumHeight1),
        }),
        "museum_room_north_center": RoomTemplate( template = "museumRoom", roofLevel = museumHeight1, walls = {
            "N": WallTemplate(template = "museumRoomBack", wallHeight = museumHeight2, windowHeight = museumScale*250, windowWidth = museumScale*80, numWindows = 3),
            "E": WallTemplate(template = "museumRoom", wallHeight = museumHeight1, windowHeight = 250, windowWidth = museumScale*150, numWindows = 3, isArch = True),
        }),
        "museum_room_north_west": RoomTemplate( template = "museumRoom", roofLevel = museumHeight1, walls = {
            "N": WallTemplate(template = "museumRoomBack", wallHeight = museumHeight2, windowHeight = museumScale*250, windowWidth = museumScale*80, numWindows = 3),
            "W": WallTemplate(template = "museumRoom", wallHeight = museumHeight1),
            "E": WallTemplate(template = "museumRoom", wallHeight = museumHeight1, windowHeight = 250, windowWidth = museumScale*150, numWindows = 3, isArch = True),
        }),

        "museum_hallway_east": RoomTemplate( template = "museumHall", roofLevel = museumHeight2, walls = {
            "E": WallTemplate(template = "museumHall", wallHeight = museumHeight3, windowHeight = museumScale*250, windowWidth = museumScale*200, numWindows = 1),
            "S":  WallTemplate(template = "museumHall", wallHeight = museumHeight2, windowHeight = 500, windowWidth = museumScale*400, numWindows = 1, isArch = True),
            "N":  WallTemplate(template = "museumHall", wallHeight = museumHeight2, windowHeight = 500, windowWidth = museumScale*400, numWindows = 1, isArch = True),
        }),
        "museum_hallway_center": RoomTemplate( template = "museumHall", roofLevel = museumHeight2, walls = {
            "E":  WallTemplate(template = "museumHall", wallHeight = museumHeight3, windowHeight = 1150, windowWidth = museumScale*400, numWindows = 1, isArch = True),
            "S":  WallTemplate(template = "museumHall", wallHeight = museumHeight2, windowHeight = 500, windowWidth = museumScale*400, numWindows = 1, isArch = True),
            "N":  WallTemplate(template = "museumHall", wallHeight = museumHeight2, windowHeight = 500, windowWidth = museumScale*400, numWindows = 1, isArch = True),
        }),
        "museum_hallway_west": RoomTemplate( template = "museumHall", roofLevel = museumHeight2, walls = {
            "E":  WallTemplate(template = "museumHall", wallHeight = museumHeight3, windowHeight = 1150, windowWidth = museumScale*400, numWindows = 1, isArch = True),
            "W":  WallTemplate(template = "museumHall", wallHeight = museumHeight3, windowHeight = museumScale*250, windowWidth = museumScale*200, numWindows = 1),
            "S":  WallTemplate(template = "museumHall", wallHeight = museumHeight2, windowHeight = 500, windowWidth = museumScale*400, numWindows = 1, isArch = True),
            "N":  WallTemplate(template = "museumHall", wallHeight = museumHeight2, windowHeight = 500, windowWidth = museumScale*400, numWindows = 1, isArch = True),
        }),

        "museum_room_south_east": RoomTemplate( template = "museumRoom", roofLevel = museumHeight1, walls = {
            "S": WallTemplate(template = "museumRoomBack", wallHeight = museumHeight2, windowHeight = museumScale*250, windowWidth = museumScale*80, numWindows = 3),
            "E": WallTemplate(template = "museumRoom", wallHeight = museumHeight1),
        }),
        "museum_room_south_center": RoomTemplate( template = "museumRoom", roofLevel = museumHeight1, walls = {
            "S": WallTemplate(template = "museumRoomBack", wallHeight = museumHeight2, windowHeight = museumScale*250, windowWidth = museumScale*80, numWindows = 3),
            "E": WallTemplate(template = "museumRoom", wallHeight = museumHeight1, windowHeight = 250, windowWidth = museumScale*150, numWindows = 3, isArch = True),
        }),
        "museum_room_south_west": RoomTemplate( template = "museumRoom", roofLevel = museumHeight1, walls = {
            "S": WallTemplate(template = "museumRoomBack", wallHeight = museumHeight2, windowHeight = museumScale*250, windowWidth = museumScale*80, numWindows = 3),
            "W": WallTemplate(template = "museumRoom", wallHeight = museumHeight1),
            "E": WallTemplate(template = "museumRoom", wallHeight = museumHeight1, windowHeight = 250, windowWidth = museumScale*150, numWindows = 3, isArch = True),
        })
    }

    def __init__(
        self,
        demoInstance,
        stage,
        zoom = 1.0, # zoom of 1.0 shows the full scene, smaller values are closer to the camera target
        hasTable = True,
        hasWalls = True,
        pathsToHide = [],
        floorOffset = 0.0,
        staticDynamicRestitution = None,
        enableDefaultLighting = True,
        camPitch = 0.37, # camera pitch and yaw in the normalized range of 0 to 1
        camYaw = 0.125,
        camElevation = 0.0,
        onlyUsePlane = False,
        contactRestOffset = None,
        enableCollisionAudio = True,
        roomTemplateStr = "default",
        createCamera = True,
        tableSurfaceDim = Gf.Vec2f(200.0, 100.0),
        overrideSphericalLightIntensity = None,
    ):
        try:
            self._currentDemoParams = demoInstance.demoParams
        except AttributeError:
            self._currentDemoParams = None

        self._directions = ["N","S","E","W"]

        demoInstance._room = self
        self._roomInstancer = None
        self._enableDefaultLighting = enableDefaultLighting
        self._stage = stage
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()
        self._glassMatPath = RoomHelper.get_glass_material(stage)
        self._upAxis = UsdGeom.GetStageUpAxis(stage)
        self._scaleFactor = 1.0 / (UsdGeom.GetStageMetersPerUnit(stage) * 100.0)
        self._captureUIText = False
        self._dirtyText = False
        self._text_array = []
        self._zoom = zoom
        self._camElevation = camElevation
        self._camYaw = camYaw
        self._camPitch = camPitch
        self._tableSurfaceDim = tableSurfaceDim
        self._overrideSphericalLightIntensity = overrideSphericalLightIntensity

        self._isMuseum = False
        self._roomTemplateStr = roomTemplateStr
        if self._currentDemoParams:
            self._isMuseum = self._currentDemoParams["isMuseum"]
            self._roomTemplateStr = self._currentDemoParams["roomTemplateStr"]

        if self._roomTemplateStr not in RoomHelper.roomTemplates:
            carb.log_warn(self._roomTemplateStr + " not found in room templates")
            self._roomTemplateStr = "default"
        self._roomTemplate = RoomHelper.roomTemplates[self._roomTemplateStr]

        if self._currentDemoParams:
            self._captureUIText = True
            if self._upAxis != UsdGeom.Tokens.z:
                carb.log_error("The demo museum only supports demos that use an up axis of 'Z', " + self._currentDemoParams["rootPath"] + " does not")
            if self._scaleFactor != 1.0:
                carb.log_error("The demo museum only supports demos that use 0.01 meters per unit, " + self._currentDemoParams["rootPath"] + " does not")

        self._onlyUsePlane = onlyUsePlane
        # if self._currentDemoParams:
        #     self._onlyUsePlane = True

        self._hasTable = hasTable
        self._hasWalls = hasWalls
        if contactRestOffset:
            self._contactOffset = contactRestOffset[0]
            self._restOffset = contactRestOffset[1]
        else:
            self._contactOffset = None
            self._restOffset = None
        if self._onlyUsePlane:
            self._hasTable = False
            self._hasWalls = False
        
        self._tableHeightOrig = 75
        self._tableHeight = (self._tableHeightOrig if self._hasTable else 0)
        self._floorOffset = floorOffset - self._tableHeight

        if self._upAxis == UsdGeom.Tokens.z:
            self._orientation = [0,1,2]
        if self._upAxis == UsdGeom.Tokens.y:
            self._orientation = [1,2,0]
        if self._upAxis == UsdGeom.Tokens.x:
            self._orientation = [2,1,0]

        self._createCamera = createCamera
        self._rootWorldPath = Sdf.Path(self._defaultPrimPath)

        if self._currentDemoParams:
            self._createCamera = False
            rootPos = self._currentDemoParams["rootPosition"]
            rootPos[2] = -(self._floorOffset * 2.0)
            self._rootPos = rootPos
            self._roomInstancer = self._currentDemoParams["roomInstancer"]
            self._enableDefaultLighting = False
            self._defaultPrimPath = self._defaultPrimPath.AppendPath(self._currentDemoParams["rootPath"])
            get_path_as_scope(self._stage, self._defaultPrimPath)
            xform = UsdGeom.Xform.Define(stage, self._defaultPrimPath)
            physicsUtils.set_or_add_translate_op(xform, self.orient_pos(rootPos))
            self._roomInstancer._currentPath = self._defaultPrimPath

        self._colliderAPIs = []
        self._tableColor = Gf.Vec3f(168.0, 142.0, 119.0) / 255.0
        self._tableTopPath = Sdf.Path()

        rangeVal = 16
        tileSize = self.museumScale * 20 if self._isMuseum else 20

        self._physicsMaterialPath = None

        # collision audio materials
        self._glassAudioMaterialPath = None
        self._metalMaterialPath = None
        self._plasticMaterialPath = None
        self._woodMaterialPath = None

        if self._createCamera:
            self._camPath = self._defaultPrimPath.AppendPath("Camera")
            demoInstance.demo_camera = self._camPath
            cam = UsdGeom.Camera.Define(stage, self._camPath)
            cam.CreateFocalLengthAttr().Set(16.0)
            self.set_camera_target(self._camPath, cam)

        if staticDynamicRestitution:
            self._physicsMaterialPath = self._defaultPrimPath.AppendPath("roomPhysicsMaterial")
            UsdShade.Material.Define(stage, self._physicsMaterialPath)
            material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(self._physicsMaterialPath))
            material.CreateStaticFrictionAttr().Set(staticDynamicRestitution[0])
            material.CreateDynamicFrictionAttr().Set(staticDynamicRestitution[1])
            material.CreateRestitutionAttr().Set(staticDynamicRestitution[2])

        # if Boom is loaded, set up materials for use by the collision audio system
        self._boom = get_boom_api()
        if self._boom and enableCollisionAudio:
            self._glassAudioMaterialPath = self.setup_collision_audio(
                stage, "glassPhysicsMaterial",
                {
                    "impact": {
                        "threshold": [0.3, 10],
                        "influenceRange": [1, 2],
                        "gain": [(0, 1), (1, 1)],
                        "audioAssetFirstIndex": [0, 3, 5],
                    },

                    "rolling": {
                        "threshold": [1, 10],
                        "influenceRange": [1, 2],
                        "gain": [(0, 1), (1, 1)],
                        "audioAssetFirstIndex": [0, 1, 2],
                    },

                    "sliding": {
                        "threshold": [1, 10],
                        "influenceRange": [1, 2],
                        "gain": [(0, 1), (1, 1)],
                        "audioAssetFirstIndex": [0, 1, 2],
                    }
                }
            )

            self._metalMaterialPath = self.setup_collision_audio(
                stage, "metalPhysicsMaterial",
                {
                    "impact": {
                        "threshold": [1, 10, 100],
                        "influenceRange": [0.5, 1.5, 2.5],
                        "gain": [(0, 0.5), (0.3, 0.5), (0.5, 0.5)],
                        "audioAssetFirstIndex": [0, 1, 3, 4],
                    },

                    "rolling": {
                        "threshold": [1, 10],
                        "influenceRange": [1, 2],
                        "gain": [(0, 1), (1, 1)],
                        "audioAssetFirstIndex": [0, 1, 2],
                    },

                    "sliding": {
                        "threshold": [0, 10],
                        "influenceRange": [1, 2],
                        "gain": [(0, 1), (1, 1)],
                        "audioAssetFirstIndex": [0, 1, 2],
                    }
                }
            )

            self._plasticMaterialPath = self.setup_collision_audio(
                stage, "plasticPhysicsMaterial",
                {
                    "impact": {
                        "threshold": [1, 10],
                        "influenceRange": [0.5, 1.5],
                        "gain": [(0, 1), (1, 1)],
                        "audioAssetFirstIndex": [0, 1, 2],
                    },

                    "rolling": {
                        "threshold": [1, 10],
                        "influenceRange": [1, 2],
                        "gain": [(0, 1), (1, 1)],
                        "audioAssetFirstIndex": [0, 2, 3],
                    },

                    "sliding": {
                        "threshold": [1, 10],
                        "influenceRange": [1, 2],
                        "gain": [(0, 1), (0.2, 0.3)],
                        "audioAssetFirstIndex": [0, 2, 3],
                    }
                }
            )

            self._woodMaterialPath = self.setup_collision_audio(
                stage, "woodPhysicsMaterial",
                {
                    "impact": {
                        "threshold": [0.1, 10, 100],
                        "influenceRange": [0.5, 1.25, 2.5],
                        "gain": [(0.1, 1), (0.75, 1.5), (0.25, 0.5)],
                        "audioAssetFirstIndex": [0, 3, 5, 6],
                    },

                    "rolling": {
                        "threshold": [1, 10, 50],
                        "influenceRange": [1, 2, 3],
                        "gain": [(0, 1), (1, 1), (1, 1)],
                        "audioAssetFirstIndex": [0, 2, 3, 4],
                    },

                    "sliding": {
                        "threshold": [0, 10, 50],
                        "influenceRange": [1, 2, 3],
                        "gain": [(0, 1), (1, 1), (1, 1)],
                        "audioAssetFirstIndex": [0, 2, 3, 4],
                    }
                }
            )


        targetLightPos = self.orient_pos(Gf.Vec3f(30.0, 500.0, 600.0))
        self._sphereLight, self._targetLightPos = RoomHelper.get_lights(self._stage, self._defaultPrimPath, demoInstance, self._isMuseum, self._scaleFactor, self._enableDefaultLighting, targetLightPos, self._overrideSphericalLightIntensity)

        self._windowsCreated = False
        self.create_floor(rangeVal, tileSize)
        if not self._onlyUsePlane:
            if self._hasWalls:
                self.create_walls(rangeVal, tileSize)
            if self._hasTable:
                self.create_table()
            if self._windowsCreated:
                pass

        stage.SetTimeCodesPerSecond(60.0) # todo: no longer needed?
        carb.settings.get_settings().set_bool("/app/viewport/grid/enabled", False)

        # create a guide box to determine the world transform of the origin if this demo is offset
        originBoxPath = self._defaultPrimPath.AppendPath("originGuide")
        originBox = UsdGeom.Cube.Define(stage, originBoxPath)
        originBox.CreateSizeAttr(1.0)
        originBox.AddTranslateOp().Set(Gf.Vec3f(0.0))
        originBox.CreatePurposeAttr(UsdGeom.Tokens.guide)
        self._originWorld = core_utils.get_world_position(stage, originBoxPath)

        # # gwoolery: Hide unused prims from loaded scenes and move them out of the way for now.
        # # Cannot delete them, as they are ancestral prims referenced by a USD.
        # # Best solution would be to get rid of the USD file and define only the needed prims in python, I think.
        for path in pathsToHide:
            # print(path)
            imageable = UsdGeom.Imageable.Get(stage, path)
            imageable.MakeInvisible()
            xformable = UsdGeom.Xformable.Get(stage, path)
            physicsUtils.set_or_add_translate_op(xformable, self.orient_pos(Gf.Vec3f(0.0, 0.0, -10000.0)))

        basePath = self._defaultPrimPath.AppendPath("roomScene")
        if self._roomInstancer:
            protoPath = self._roomInstancer.get_proto_path(self).AppendPath("roomScene")
        else:
            protoPath = self._rootWorldPath

        self._instanceArr = [
            InstanceItem(False, protoPath.AppendPath("renderables"), basePath.AppendPath("renderables"), None),
            InstanceItem(True, protoPath.AppendPath("colliders/floor"), basePath.AppendPath("colliders/floor"), self._plasticMaterialPath)
        ]

        if self._hasWalls:
            self._instanceArr.append(
                InstanceItem(True, protoPath.AppendPath("colliders/walls"), basePath.AppendPath("colliders/walls"), self._plasticMaterialPath)
            )

        if self._windowsCreated:
            self._instanceArr.append(
                InstanceItem(True, protoPath.AppendPath("colliders/windows"), basePath.AppendPath("colliders/windows"), self._glassAudioMaterialPath)
            )

        if self._hasTable:
            self._instanceArr.append(
                InstanceItem(True, protoPath.AppendPath("colliders/table"), basePath.AppendPath("colliders/table"), self._woodMaterialPath)
            )

        if not self._onlyUsePlane:
            if self._roomInstancer:
                self._roomInstancer.end_setup(self, Gf.Vec3d(0.0, 0.0, -self._rootPos[2]*0.5 ))            
            for instanceItem in self._instanceArr:
                self.apply_collision(instanceItem)
            
            if self._windowsCreated:
                glassPrim = self._stage.GetPrimAtPath(self._instanceArr[3].instancePath)
                glassPrim.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
                self.set_glass_material(self._instanceArr[3].instancePath)

    def set_camera_target(self, cam_path, cam, zoom_override = None):
        defaultCamTarget = Gf.Vec3f(0, 0, self._tableHeight)
        camDistance = 1500.0 * (zoom_override if zoom_override else self._zoom)
        PI_1 = math.pi
        PI_2 = math.pi * 2.0
        camVec = Gf.Vec3f(
            math.cos(PI_2*self._camYaw)*math.sin(PI_1*self._camPitch),
            math.sin(PI_2*self._camYaw)*math.sin(PI_1*self._camPitch),
            math.cos(PI_1*self._camPitch)
        )*camDistance
        elevationOffset = Gf.Vec3f(0.0, 0.0, self._camElevation)
        camPos = self.orient_pos(camVec + defaultCamTarget + elevationOffset)
        camTarget = self.orient_pos(defaultCamTarget + elevationOffset)
        
        # gwoolery: these are necessary to ensure that golden images are generated in tests.
        # The source is based on code found in ViewportCameraState, but unable to use that
        # due to failures in the aforementioned case (perhaps due to the view port not being fully initialized)
        # Other methods of setting the camera target, such as with customLayerData, also fail.
        RoomHelper.set_target_world(self._stage, cam_path, cam, Gf.Vec3d(camTarget))
        RoomHelper.set_position_world(self._stage, cam_path, cam, Gf.Vec3d(camPos))

    @staticmethod
    def get_lights(stage, defaultPrimPath, demoInstance, isMuseum, scaleFactor, createLights, targetLightPos, overrideSphericalLightDensity=None):
        lightPath = defaultPrimPath.AppendPath("SphereLightRoom")
        domeLightPath = defaultPrimPath.AppendPath("DomeLightRoom")
        
        isregistry = carb.settings.acquire_settings_interface()
        AMBIENT_LIGHT_INTENSITY = "/rtx/sceneDb/ambientLightIntensity"
        INDIRECT_DIFFUSE = "/rtx/indirectDiffuse/enabled"
        AUTO_EXPOSURE = "/rtx/post/histogram/enabled"
        AMBIENT_RAY_LENGTH = "/rtx/ambientOcclusion/rayLength"
        
        if isMuseum:
            isregistry.set_float(AMBIENT_LIGHT_INTENSITY, 0.1)
            isregistry.set_bool(INDIRECT_DIFFUSE, True)
            isregistry.set_bool(AUTO_EXPOSURE, True)
            isregistry.set_float(AMBIENT_RAY_LENGTH, 300.0 * scaleFactor)
        else:
            isregistry.set_float(AMBIENT_LIGHT_INTENSITY, 0.3)
            isregistry.set_bool(INDIRECT_DIFFUSE, False)
            isregistry.set_bool(AUTO_EXPOSURE, False)
            isregistry.set_float(AMBIENT_RAY_LENGTH, 35.0 * scaleFactor)

        scaleFactorSqrt = math.sqrt(scaleFactor)
        lightIntensity = (600000 if isMuseum else 500000) * scaleFactorSqrt # intensity is squared, must compensate in scaling

        if overrideSphericalLightDensity is not None:
            lightIntensity = overrideSphericalLightDensity

        isTest = False
        if getattr(demoInstance, "_is_test", None):
            isTest = True

        sphereLight = None
        if createLights:
            sphereLight = UsdLux.SphereLight.Define(stage, lightPath)
            sphereLight.CreateRadiusAttr((10.0 if isMuseum else 20.0) * scaleFactor)
            sphereLight.CreateIntensityAttr(lightIntensity)
            sphereLight.AddTranslateOp().Set(targetLightPos)
            sphereLight.CreateSpecularAttr().Set(0.0 if isMuseum else 1.0)

            if not isTest: # for whatever reason, a domelight creates noise in the golden images, so disable it during tests
                domeLight = UsdLux.DomeLight.Define(stage, domeLightPath)
                domeLight.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, -9999.0))
                domeLight.CreateIntensityAttr((400 if isMuseum else 150)) #  * scaleFactorSqrt
                domeLight.CreateColorAttr(Gf.Vec3f(0.5, 0.75, 1.0))

        return sphereLight, targetLightPos



    @staticmethod
    def get_glass_material(stage):
        glassPath = "/World/Looks/OmniGlass"
        if stage.GetPrimAtPath(glassPath):
            return glassPath
        mtl_created = []
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniGlass.mdl",
            mtl_name="OmniGlass",
            mtl_created_list=mtl_created,
            select_new_prim=False,
        )
        return mtl_created[0]

    def apply_collision(self, instanceItem):
        if not instanceItem.hasCollision:
            return

        colliderPrim = self._stage.GetPrimAtPath(instanceItem.instancePath)
        if self._contactOffset:
            physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(colliderPrim)
            physxCollisionAPI.CreateContactOffsetAttr(self._contactOffset)
            physxCollisionAPI.CreateRestOffsetAttr(self._restOffset)

        # store references to the collider APIs so that they can be used for things like multiple scenes
        self._colliderAPIs.append(UsdPhysics.CollisionAPI.Apply(colliderPrim))

        if self._physicsMaterialPath:
            self.set_physics_material(instanceItem.instancePath, self._physicsMaterialPath, False)
        elif instanceItem.materialPath:
            self.set_physics_material(instanceItem.instancePath, instanceItem.materialPath, True)

    def __del__(self):
        # set all attributes to None to make sure this isn't holding onto refs for anything
        # if any member needs more complex cleanup, do it before this loop
        for attr_name in list(self.__dict__.keys()):
            setattr(self, attr_name, None)

    def update_ui_text(self, text):
        self._text_array = text
        self._dirtyText = True

    def fetch_ui_text(self):
        self._dirtyText = False
        return self._text_array

    def get_hsv_color(self, hue, saturation = 1.0, luminosity = 0.5):
        hue6 = hue * 6.0
        modulo = Gf.Vec3f((hue6 + 0.0) % 6.0, (hue6 + 4.0) % 6.0, (hue6 + 2.0) % 6.0)
        absolute = Gf.Vec3f(abs(modulo[0] - 3.0), abs(modulo[1] - 3.0), abs(modulo[2] - 3.0))
        rgb = Gf.Vec3f(
            Gf.Clampf(absolute[0] - 1.0, 0.0, 1.0),
            Gf.Clampf(absolute[1] - 1.0, 0.0, 1.0),
            Gf.Clampf(absolute[2] - 1.0, 0.0, 1.0),
        )

        linter = Gf.Vec3f(1.0) * (1.0 - saturation) + rgb * saturation
        rgb = luminosity * linter
        return rgb

    def get_table_color(self):
        return self._tableColor

    def setup_collision_audio(self, stage, materialName, materialData):
        # create the collision material prim under the "Looks" folder in the stage
        materialPath = self._defaultPrimPath.AppendPath("Looks").AppendPath(materialName)
        UsdShade.Material.Define(stage, materialPath)
        materiaPrim = stage.GetPrimAtPath(materialPath)
        if not materiaPrim:
            return None

        # add physics API, the collision audio system relies on audio data living on the physics material
        physMaterial = UsdPhysics.MaterialAPI.Apply(materiaPrim)
        physMaterial.CreateStaticFrictionAttr().Set(0.5)
        physMaterial.CreateDynamicFrictionAttr().Set(0.5)
        physMaterial.CreateRestitutionAttr().Set(0.5)

        # all audio assets need to be stored here and have standardized naming
        audioBaseDir = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../audio")))
        audioBaseDir = audioBaseDir.replace("\\", "/")
        audioAssetSize = ["small", "medium", "large"]

        # set up some entries for it to cover basic physics events
        for eventType in materialData.keys():
            # auto gen the asset names based on how many there are, the event type, and the material name
            audioAssets = []
            assetBasePath = f"{audioBaseDir}/{materialName}/{eventType}"
            for size in range(len(materialData[eventType]["audioAssetFirstIndex"]) - 1):
                count = materialData[eventType]["audioAssetFirstIndex"][size + 1] - materialData[eventType]["audioAssetFirstIndex"][size]
                for idx in range(count):
                    assetPath = f"{assetBasePath}/{audioAssetSize[size]}_{(idx + 1):02}.wav"
                    audioAssets.append(assetPath)

            # call this first to set up the prim with basic collision audio data
            # this will take care of applying the API as necessary
            self._boom.add_threshold_group(materialPath.pathString, eventType)
            # then use this one to fill in all the details
            self._boom.update_data(
                materialPath.pathString, eventType, True,
                materialData[eventType]["threshold"],
                materialData[eventType]["influenceRange"],
                materialData[eventType]["gain"],
                audioAssets,
                materialData[eventType]["audioAssetFirstIndex"]
            )

        return materialPath

    def set_physics_material(self, path, mat_path, contact_reporting: bool):
        if not mat_path:
            return

        prim = self._stage.GetPrimAtPath(path)
        physicsUtils.add_physics_material_to_prim(self._stage, prim, mat_path)

        if contact_reporting:
            # needs to have the contact report API applied so Boom will get collision events
            PhysxSchema.PhysxContactReportAPI.Apply(prim)

    def set_glass_material(self, path):
        omni.kit.commands.execute(
            "BindMaterial", prim_path=path, material_path=self._glassMatPath
        )

    def get_path(self, rootPath, primPath):
        if self._roomInstancer:
            finalPathRoot = self._roomInstancer.get_proto_path(self)
        else:
            finalPathRoot = self._defaultPrimPath

        rootArr = rootPath.split("/")
        for i in rootArr:
            finalPathRoot = finalPathRoot.AppendPath(i)
            if not self._stage.GetPrimAtPath(finalPathRoot):
                UsdGeom.Scope.Define(self._stage, finalPathRoot)

        return Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, finalPathRoot.AppendPath(primPath), True))

    def create_box(self, rootPath, primPath, dimensions, position, color, positionMod = None):
        boxActorPath = self.get_path(rootPath, primPath)
        newPosition = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)

        # deep copy to avoid modifying reference
        for i in range(3):
            newPosition[i] = position[i]
            if positionMod:
                newPosition[i] *= positionMod[i]

        cubeGeom = UsdGeom.Cube.Define(self._stage, boxActorPath)
        cubePrim = self._stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(1.0)
        half_extent = 0.5
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(self.orient_pos(newPosition))
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(self.orient_dim(dimensions))
        cubeGeom.CreateDisplayColorAttr().Set([color])

        return boxActorPath

    def add_points(self, reverseWinding, pointArrDest, pointArrSource):
        if reverseWinding:
            for i in range(4):
                pointArrDest.append(pointArrSource[3-i])
        else:
            for i in range(4):
                pointArrDest.append(pointArrSource[i])


    def create_archway(
        self,
        stage,
        rootPath,
        primPath,
        color,
        centerPoint,
        axis,
        halfLength,
        halfHeight,
        halfWidth,
        innerRadius,
        outerRadius,
        resolution,
        fillOuter,
        mask = {"inner": True, "outer": True},
        positionMod = None
    ):

        newCenterPoint = self.orient_pos(centerPoint)
        if positionMod:
            for i in range(3):
                newCenterPoint[i] *= positionMod[i]

        archPath = self.get_path(rootPath, primPath)

        if resolution < 3:
            carb.log_error("Insufficient resolution for archway")
            return

        if axis == UsdGeom.Tokens.x:
            axisMod = Gf.Vec3f(1.0, 0.0, 0.0)
            tangentMod = Gf.Vec3f(0.0, 1.0, 0.0)
        elif axis == UsdGeom.Tokens.y:
            axisMod = Gf.Vec3f(0.0, 1.0, 0.0)
            tangentMod = Gf.Vec3f(1.0, 0.0, 0.0)
        else:
            carb.log_error("Invalid axis specified for creating archway")
            return

        begPoint = -axisMod * halfLength
        endPoint = axisMod * halfLength

        begInner = [None]*resolution
        begOuter = [None]*resolution
        endInner = [None]*resolution
        endOuter = [None]*resolution
        nrmInner = [None]*resolution
        nrmOuter = [None]*resolution

        for i in range(resolution):
            angle = (i * math.pi) / (resolution - 1)
            if axis == UsdGeom.Tokens.x:
                angleVec = Gf.Vec3f(0.0, math.cos(angle), math.sin(angle))
            elif axis == UsdGeom.Tokens.y:
                angleVec = Gf.Vec3f(math.cos(angle), 0.0, math.sin(angle))

            radPointInner = angleVec * innerRadius
            begInner[i] = (begPoint + radPointInner)
            endInner[i] = (endPoint + radPointInner)
            nrmInner[i] = (angleVec * -1.0)

            if fillOuter:
                if i < resolution*0.25:
                    lerpVal = i/(resolution*0.25)
                    nrmOuter[i] = -tangentMod
                    radPointOuter = tangentMod*halfWidth + Gf.Vec3f(0.0, 0.0, lerpVal*halfHeight)
                elif i < resolution*0.75:
                    lerpVal = (i - (resolution*0.25))/(resolution*0.5)
                    nrmOuter[i] = Gf.Vec3f(0.0, 0.0, 1.0)
                    radPointOuter = (lerpVal)*(-tangentMod*halfWidth) + (1.0-lerpVal)*(tangentMod*halfWidth) +  Gf.Vec3f(0.0, 0.0, halfHeight)
                else:
                    lerpVal = (i - (resolution*0.75))/(resolution*0.25)
                    if i == (resolution - 1):
                        lerpVal = 1.0
                    nrmOuter[i] = tangentMod
                    radPointOuter = -tangentMod*halfWidth + Gf.Vec3f(0.0, 0.0, (1.0-lerpVal)*halfHeight)

                begOuter[i] = (begPoint + radPointOuter)
                endOuter[i] = (endPoint + radPointOuter)
            else:
                radPointOuter = angleVec * outerRadius
                begOuter[i] = (begPoint + radPointOuter)
                endOuter[i] = (endPoint + radPointOuter)
                nrmOuter[i] = (angleVec * 1.0)

            begInner[i] = self.orient_dim(begInner[i])
            begOuter[i] = self.orient_dim(begOuter[i])
            endInner[i] = self.orient_dim(endInner[i])
            endOuter[i] = self.orient_dim(endOuter[i])
            nrmInner[i] = self.orient_dim(nrmInner[i])
            nrmOuter[i] = self.orient_dim(nrmOuter[i])
            


        points = []
        normals = []
        indices = []
        vertexCounts = []
        reverseWinding = (axis == UsdGeom.Tokens.y)

        if mask["inner"]:
            # inner arch
            for i in range(resolution - 1):
                i0 = i + 0
                i1 = i + 1
                self.add_points(reverseWinding, points,[
                    begInner[i0], # --
                    endInner[i0], # +-
                    endInner[i1], # ++
                    begInner[i1] # -+
                ])
                self.add_points(reverseWinding, normals,[
                    nrmInner[i0], # --
                    nrmInner[i0], # +-
                    nrmInner[i1], # ++
                    nrmInner[i1]  # -+
                ])


        if mask["outer"]:
            # outer arch
            for i in range(resolution - 1):
                i0 = i + 0
                i1 = i + 1
                self.add_points(reverseWinding, points,[
                    begOuter[i1], # -+
                    endOuter[i1], # ++
                    endOuter[i0], # +-
                    begOuter[i0]  # --
                ])
                self.add_points(reverseWinding, normals,[
                    nrmOuter[i1], # -+
                    nrmOuter[i1], # ++
                    nrmOuter[i0], # +-
                    nrmOuter[i0]  # --
                ])

        # arch face front
        for i in range(resolution - 1):
            i0 = i + 0
            i1 = i + 1
            self.add_points(reverseWinding, points,[
                endInner[i0], # --
                endOuter[i0], # +-
                endOuter[i1], # ++
                endInner[i1]  # -+
            ])
            self.add_points(reverseWinding, normals,[
                axisMod * 1.0, # --
                axisMod * 1.0, # +-
                axisMod * 1.0, # ++
                axisMod * 1.0  # -+
            ])

        # arch face back
        for i in range(resolution - 1):
            i0 = i + 0
            i1 = i + 1
            self.add_points(reverseWinding, points,[
                begInner[i1], # -+
                begOuter[i1], # ++
                begOuter[i0], # +-
                begInner[i0]  # --
            ])
            self.add_points(reverseWinding, normals,[
                axisMod * -1.0, # --
                axisMod * -1.0, # +-
                axisMod * -1.0, # ++
                axisMod * -1.0  # -+
            ])
            

        # arch bottom
        lastInd = resolution - 1
        normDown = Gf.Vec3f(0.0, 0.0, -1.0)
        self.add_points(reverseWinding, points,[
            begOuter[0], # -+
            endOuter[0], # ++
            endInner[0], # +-
            begInner[0]  # --
        ])
        self.add_points(reverseWinding, normals,[
            normDown, # -+
            normDown, # ++
            normDown, # +-
            normDown  # --
        ])

        self.add_points(reverseWinding, points,[
            begInner[lastInd], # --
            endInner[lastInd], # +-
            endOuter[lastInd], # ++
            begOuter[lastInd]  # -+
        ])
        self.add_points(reverseWinding, normals,[
            normDown, # --
            normDown, # +-
            normDown, # ++
            normDown  # -+
        ])


        for i in range(len(points)):
            indices.append(i)

        newRange = int(len(points)/4)
        for i in range(newRange):
            vertexCounts.append(4)

        entityArch = physicsUtils.create_mesh(stage, archPath, points, normals, indices, vertexCounts)
        entityArch.CreateDisplayColorAttr().Set([color])
        entityArch.AddTranslateOp().Set(newCenterPoint)
        entityArch.AddOrientOp().Set(Gf.Quatf(1.0))
        entityArch.AddScaleOp().Set(Gf.Vec3f(1.0))

        return archPath

    def create_checker_mesh(self, stage, path, axis, halfSize, xdim, ydim, checkerMod):

        xdimHalf = xdim / 2
        ydimHalf = ydim / 2

        count = math.floor((xdim*ydim)/2)
        vertexCount = count * 4
        upVector = self.orient_no_scale(Gf.Vec3f(0.0, 0.0, 1.0))

        normals = [upVector]*vertexCount
        indices = range(vertexCount)
        vertexCounts = [4] * count

        baseIndex = 0
        points = [None]*vertexCount # cannot instantiate unique objects here
        for y in range(ydim):
            yOffset = (y-ydimHalf)*halfSize*2.0 + halfSize
            for x in range(xdim):
                xOffset = (x-xdimHalf)*halfSize*2.0 + halfSize
                if (x + y) % 2 == checkerMod:
                    points[baseIndex + 0] = Gf.Vec3f(xOffset-halfSize, yOffset-halfSize, 0.0)
                    points[baseIndex + 1] = Gf.Vec3f(xOffset+halfSize, yOffset-halfSize, 0.0)
                    points[baseIndex + 2] = Gf.Vec3f(xOffset+halfSize, yOffset+halfSize, 0.0)
                    points[baseIndex + 3]  = Gf.Vec3f(xOffset-halfSize, yOffset+halfSize, 0.0)

                    if axis != UsdGeom.Tokens.z:
                        for i in range(4):
                            points[baseIndex + i] = self.orient_no_scale(points[baseIndex + i])

                    baseIndex += 4

        return physicsUtils.create_mesh(stage, path, points, normals, indices, vertexCounts)

    def create_plane(self, rootPath, primPath, size, position, color, collisionEnabled=False, xdim=0, ydim=0, checkerMod = 0):
        planePath = self.get_path(rootPath, primPath)

        if xdim == 0:
            entityPlane = physicsUtils.create_mesh_square_axis(self._stage, planePath, self._upAxis, size * self._scaleFactor)
        else:
            entityPlane = self.create_checker_mesh(self._stage, planePath, self._upAxis, size * self._scaleFactor, xdim, ydim, checkerMod)
        
        entityPlane.CreateDisplayColorAttr().Set([color])
        entityPlane.AddTranslateOp().Set(self.orient_pos(position))
        entityPlane.AddOrientOp().Set(Gf.Quatf(1.0))
        entityPlane.AddScaleOp().Set(Gf.Vec3f(1.0))

        # if physicsMat:
        #     self.set_physics_material(planePath, physicsMat, True)

        if collisionEnabled:
            UsdPhysics.CollisionAPI.Apply(entityPlane.GetPrim())
        else:
            entityPlane.CreatePurposeAttr().Set("render")

        planePrim = self._stage.GetPrimAtPath(planePath)
        planePrim.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)

        return planePath

    def create_table(self):
        # always create table, do not skip if instance exists because it can vary between rooms

        tableDim = Gf.Vec3f(self._tableSurfaceDim[0], self._tableSurfaceDim[1], self._tableHeight)
        tableThickness = 10.0
        legThickness = 8.0
        legOffset = 5.0
        legTotalOffset = (legOffset + legThickness*0.5)

        self._tableTopPath = self.create_box(
            "roomScene/colliders/table","tableTopActor",
            Gf.Vec3f(tableDim[0], tableDim[1], tableThickness),
            Gf.Vec3f(0.0, 0.0, tableDim[2] - tableThickness*0.5),
            self._tableColor
        )

        offsetArr = [Gf.Vec2f(-1.0,-1.0), Gf.Vec2f(-1.0,1.0), Gf.Vec2f(1.0,-1.0), Gf.Vec2f(1.0,1.0)]

        i = 0
        for offsetValue in offsetArr:
            self.create_box(
                "roomScene/colliders/table","tableLeg" + str(i) + "Actor",
                Gf.Vec3f(legThickness, legThickness, tableDim[2] - tableThickness),
                Gf.Vec3f(offsetValue[0]*(tableDim[0]*0.5 - legTotalOffset), offsetValue[1]*(tableDim[1]*0.5 - legTotalOffset), (tableDim[2] - tableThickness)*0.5 ),
                self._tableColor
            )
            i += 1

    def create_floor(self, rangeVal, tileSize):
        newRangeVal = rangeVal + 2
        halfRangeVal = newRangeVal / 2
        floorThickness = 10.0

        tileCol0 = self._roomTemplate.tileCol0
        tileCol1 = self._roomTemplate.tileCol1

        planeCol = Gf.Vec3f(150.0,200.0,255.0)/255.0
        planeSize = 50000.0
        self._roomRadius = tileSize * newRangeVal

        if self._currentDemoParams:
            planeSize = self._currentDemoParams["groundPlaneSize"]

        if self._roomInstancer:
            if self._roomInstancer.has_instance(self._roomTemplateStr):
                return

        if self._onlyUsePlane:
            self.create_plane("roomScene/colliders/floor","groundPlane", planeSize * 0.5, Gf.Vec3f(0.0), planeCol, True)
            return

        if not self._isMuseum:
            self.create_box(
                "roomScene/colliders/floor","infinitePlane",
                Gf.Vec3f(planeSize, planeSize, floorThickness),
                Gf.Vec3f(0.0, 0.0, -(floorThickness + floorThickness*0.5)),
                planeCol
            )

        if (self._isMuseum):
            newRangeVal -= 2 # prevent overlaping floors in museum

        self.create_box(
            "roomScene/colliders/floor","mainFloorActor",
            Gf.Vec3f(newRangeVal*tileSize*2.0, newRangeVal*tileSize*2.0, floorThickness),
            Gf.Vec3f(0.0, 0.0, -0.5*floorThickness),
            tileCol0
        )

        position = Gf.Vec3f(0.0, 0.0, 0.1)

        self.create_plane("roomScene/renderables","groundPlane0", tileSize, position, tileCol0, xdim = newRangeVal, ydim = newRangeVal, checkerMod = 0)
        self.create_plane("roomScene/renderables","groundPlane1", tileSize, position, tileCol1, xdim = newRangeVal, ydim = newRangeVal, checkerMod = 1)

    def orient_no_scale(self, vec):
        newVec = Gf.Vec3f(0.0)
        for i in range(3):
            newVec[i] = vec[self._orientation[i]]
        return newVec

    def orient_dim(self, vec):
        newVec = Gf.Vec3f(0.0)
        for i in range(3):
            newVec[i] = vec[self._orientation[i]] * self._scaleFactor
        return newVec

    def orient_pos(self, vec):
        newVec = Gf.Vec3f(0.0)
        for i in range(3):
            curOrientation = self._orientation[i]
            newVec[i] = vec[curOrientation]
            if curOrientation == 2:
                newVec[i] += self._floorOffset # shift everything downwards so that the table is at the origin
            newVec[i] *= self._scaleFactor
        return newVec

    def swap_xy(self, vec):
        temp = vec[0]
        vec[0] = vec[1]
        vec[1] = temp

    def create_walls(self, rangeVal, tileSize):
        if self._roomInstancer:
            if self._roomInstancer.has_instance(self._roomTemplateStr):
                return

        newRangeVal = rangeVal + 2
        
        pillarJut = tileSize*0.5
        moldingJut = tileSize*0.5
        moldingHeight = tileSize
        if self._isMuseum:
            moldingHeight *= 0.5
        roofOuterRadius = newRangeVal*tileSize - moldingJut * 2.0

        # hasCeiling = True
        innerWallHeightMax = 0
        innerWallHeightMin = 99999

        for i in range(2):
            for m in range(2):
                wallIndex = i*2 + (1-m)
                direction = self._directions[wallIndex]
                curWall = self._roomTemplate.walls[direction]
                if (curWall.wallHeight <= 0):
                    continue

                wallHeight = curWall.wallHeight
                windowHeight = curWall.windowHeight
                windowWidth = curWall.windowWidth
                numWindows = curWall.numWindows
                isArch = curWall.isArch

                wallColor = curWall.wallColor
                moldingColor = Gf.Vec3f(255.0, 255.0, 255.0)/255.0
                pillarColor = Gf.Vec3f(255.0, 255.0, 255.0)/255.0
                
                innerWallHeight = wallHeight - moldingHeight*2.0
                innerWallHeightMax = max(innerWallHeight, innerWallHeightMax)
                innerWallHeightMin = min(innerWallHeight, innerWallHeightMin)
                
                windowSupportHeight = (innerWallHeight-windowHeight)*0.5

                # windowSupportHeight = min(
                #     windowSupportHeight,
                #     (innerWallHeight - windowHeight - windowWidth * 0.5)
                # )

                # if (innerWallHeight - windowHeight - windowWidth) < windowSupportHeight:
                #     windowSupportHeight = (innerWallHeight - windowHeight - windowWidth) * 0.5
                windowSpacerWidth = (rangeVal*tileSize*2.0 - pillarJut*2.0 - numWindows*windowWidth)/(numWindows+1)
                wallThickness = tileSize
                windowFrameThickness = tileSize*0.25
                windowInnerHeight = windowHeight - windowFrameThickness * 2.0
                windowInnerWidth = windowWidth - windowFrameThickness * 2.0
                wallOfsset = rangeVal*tileSize + wallThickness*0.5
                wallDimHeight = windowSupportHeight if ((numWindows > 0) and not isArch) else innerWallHeight

                windowDim = Gf.Vec3f(windowWidth, 0.0, windowHeight)
                windowPos = Gf.Vec3f(0.0, wallOfsset, windowHeight*0.5 + windowSupportHeight + moldingHeight)
                windowSpacerDim = Gf.Vec3f(windowSpacerWidth, wallThickness, innerWallHeight if isArch else windowHeight )
                windowSpacerPos = Gf.Vec3f(0.0, wallOfsset, windowHeight*0.5 + windowSupportHeight + moldingHeight)
                wallDim = Gf.Vec3f(rangeVal*tileSize*2.0 - pillarJut*2.0, wallThickness, wallDimHeight)
                wallPosTop = Gf.Vec3f(0.0, wallOfsset, windowSupportHeight*0.5 + windowSupportHeight + windowHeight + moldingHeight)
                wallPosBottom = Gf.Vec3f(0.0, wallOfsset, windowSupportHeight*0.5 + moldingHeight)
                wallPosMid = (wallPosTop + wallPosBottom) * 0.5
                moldingDim = Gf.Vec3f(rangeVal*tileSize*2.0 - (pillarJut+moldingJut)*2.0, wallThickness + moldingJut*2.0, moldingHeight)
                moldingPosBottom = Gf.Vec3f(0.0, wallOfsset, moldingHeight*0.5)
                moldingPosTop = Gf.Vec3f(0.0, wallOfsset, moldingHeight*0.5 + innerWallHeight + moldingHeight)

                if i == 1:
                    self.swap_xy(wallDim)
                    self.swap_xy(wallPosTop)
                    self.swap_xy(wallPosBottom)
                    self.swap_xy(wallPosMid)
                    self.swap_xy(moldingDim)
                    self.swap_xy(moldingPosBottom)
                    self.swap_xy(moldingPosTop)
                    self.swap_xy(windowSpacerPos)
                    self.swap_xy(windowSpacerDim)
                    self.swap_xy(windowDim)
                    self.swap_xy(windowPos)

                if m == 0:
                    posMod = Gf.Vec3f(1.0, 1.0, 1.0)
                else:
                    if i == 0:
                        posMod = Gf.Vec3f(1.0, -1.0, 1.0)
                    else:
                        posMod = Gf.Vec3f(-1.0, 1.0, 1.0)

                if numWindows > 0:

                    # if isArch:
                    #     # windowSpacerDim[2] = windowHeight + windowSupportHeight
                    #     # windowSpacerPos[2] = windowHeight*0.5 + windowSupportHeight * 0.5 + moldingHeight
                    # else:
                    #     # windowSpacerDim[2] = windowHeight
                    #     # windowSpacerPos[2] = windowHeight*0.5 + windowSupportHeight + moldingHeight

                    for j in range(numWindows+1):
                        indexStr = str(m) + "_" + str(i) + "_" + str(j)
                        windowSpacerPos[i] = (windowSpacerWidth + windowWidth)*j + windowSpacerWidth*0.5 - wallDim[i]*0.5

                        self.create_box(
                            "roomScene/colliders/walls","windowSpacerActor" + indexStr,
                            windowSpacerDim,
                            windowSpacerPos,
                            wallColor,
                            posMod
                        )

                    for j in range(numWindows):
                        indexStr = str(m) + "_" + str(i) + "_" + str(j)
                        windowPos[i] = (windowSpacerWidth + windowWidth)*j + windowSpacerWidth + windowWidth*0.5 - wallDim[i]*0.5

                        windowDim[2] = windowHeight
                        windowDim[i] = windowWidth
                        windowDim[1-i] = wallThickness + moldingJut*2.0
                        windowDim[2] = windowFrameThickness
                        windowPos[2] = windowFrameThickness*0.5 + windowSupportHeight + moldingHeight

                        if not isArch:
                            self.create_box(
                                "roomScene/colliders/walls","windowActorBottom" + indexStr,
                                windowDim,
                                windowPos,
                                moldingColor,
                                posMod
                            )

                        windowDim[1-i] = wallThickness + moldingJut
                        windowPos[2] = windowFrameThickness*0.5 + windowFrameThickness + windowInnerHeight + windowSupportHeight + moldingHeight

                        if not isArch:
                            self.create_box(
                                "roomScene/colliders/walls","windowActorTop" + indexStr,
                                windowDim,
                                windowPos,
                                moldingColor,
                                posMod
                            )

                        if isArch:
                            windowPos[2] = windowHeight + moldingHeight
                            self.create_archway(
                                self._stage,
                                "roomScene/renderables","windowArchOuter" + indexStr,
                                wallColor,
                                windowPos,
                                UsdGeom.Tokens.y if (i == 0) else UsdGeom.Tokens.x,
                                wallThickness * 0.5, # halfLength
                                innerWallHeight - windowHeight, # halfHeight
                                windowWidth*0.5, # halfWidth
                                windowWidth*0.5,
                                windowWidth*0.5,
                                24,
                                True,
                                mask = {"inner": False, "outer": False},
                                positionMod = posMod
                            )
                            self.create_archway(
                                self._stage,
                                "roomScene/renderables","windowArchInner" + indexStr,
                                moldingColor,
                                windowPos,
                                UsdGeom.Tokens.y if (i == 0) else UsdGeom.Tokens.x,
                                wallThickness * 0.5 + moldingJut * 0.5, # halfLength
                                windowSupportHeight, # halfHeight
                                windowWidth*0.5, # halfWidth
                                windowWidth*0.5 - windowFrameThickness,
                                windowWidth*0.5,
                                24,
                                False,
                                positionMod = posMod
                            )

                        if isArch:
                            windowDim[2] = windowHeight
                            windowPos[2] = windowHeight*0.5 + moldingHeight
                        else:
                            windowDim[2] = windowInnerHeight
                            windowPos[2] = windowInnerHeight*0.5 + windowFrameThickness + windowSupportHeight + moldingHeight
                        
                        
                        windowDim[i] = moldingJut*0.5
                        windowPos[i] = (windowSpacerWidth + windowWidth)*j + windowSpacerWidth + windowFrameThickness*0.5 - wallDim[i]*0.5
                        self.create_box(
                            "roomScene/colliders/walls","windowActorR" + indexStr,
                            windowDim,
                            windowPos,
                            moldingColor,
                            posMod
                        )

                        windowPos[i] = (windowSpacerWidth + windowWidth)*j + windowSpacerWidth + windowInnerWidth + windowFrameThickness + windowFrameThickness*0.5 - wallDim[i]*0.5
                        self.create_box(
                            "roomScene/colliders/walls","windowActorL" + indexStr,
                            windowDim,
                            windowPos,
                            moldingColor,
                            posMod
                        )

                        windowDim[i] = windowInnerWidth
                        windowDim[1-i] = windowFrameThickness
                        windowDim[2] = windowInnerHeight

                        windowPos[2] = windowInnerHeight*0.5 + windowFrameThickness + windowSupportHeight + moldingHeight + windowDim[2]*0.001
                        windowPos[i] = (windowSpacerWidth + windowWidth)*j + windowSpacerWidth + windowFrameThickness + windowInnerWidth*0.5 - wallDim[i]*0.5 + windowDim[i]*0.001

                        if not isArch:
                            glassPath = self.create_box(
                                "roomScene/colliders/windows","windowGlassActor" + indexStr,
                                windowDim * 0.998,
                                windowPos,
                                Gf.Vec3f(20.0, 20.0, 30.0)/255.0,
                                posMod
                            )
                            self._windowsCreated = True
                            glassPrim = self._stage.GetPrimAtPath(glassPath)
                            glassPrim.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)

                        windowDivThickness = 2.0
                        windowDim[i] = windowDivThickness
                        windowDim[1-i] = windowFrameThickness + 2.0
                        windowDim[2] = windowInnerHeight
                        windowPos[2] = windowInnerHeight*0.5 + windowFrameThickness + windowSupportHeight + moldingHeight
                        windowPos[i] = (windowSpacerWidth + windowWidth)*j + windowSpacerWidth + windowFrameThickness + windowInnerWidth*0.5 - wallDim[i]*0.5
                        
                        if not isArch:
                            self.create_box(
                                "roomScene/colliders/walls","windowDiv0_" + indexStr,
                                windowDim,
                                windowPos,
                                moldingColor,
                                posMod
                            )
                            for k in range(2):
                                windowDim[i] = windowInnerWidth
                                windowDim[1-i] = windowFrameThickness + 2.0
                                windowDim[2] = windowDivThickness
                                windowPos[2] = windowInnerHeight*(0.33*(k+1)) + windowFrameThickness + windowSupportHeight + moldingHeight
                                windowPos[i] = (windowSpacerWidth + windowWidth)*j + windowSpacerWidth + windowFrameThickness + windowInnerWidth*0.5 - wallDim[i]*0.5
                                self.create_box(
                                    "roomScene/colliders/walls","windowDiv1_" + indexStr + "_" + str(k),
                                    windowDim,
                                    windowPos,
                                    moldingColor,
                                    posMod
                                )

                    if not isArch:
                        self.create_box(
                            "roomScene/colliders/walls","wallActorTop" + str(m) + "_" + str(i),
                            wallDim,
                            wallPosTop,
                            wallColor,
                            posMod
                        )
                        self.create_box(
                            "roomScene/colliders/walls","wallActorBottom" + str(m) + "_" + str(i),
                            wallDim,
                            wallPosBottom,
                            wallColor,
                            posMod
                        )
                else:
                    self.create_box(
                        "roomScene/colliders/walls","wallActorBottom" + str(m) + "_" + str(i),
                        wallDim,
                        wallPosMid,
                        wallColor,
                        posMod
                    )


                self.create_box(
                    "roomScene/colliders/walls","moldingActorBottom" + str(m) + "_" + str(i),
                    moldingDim,
                    moldingPosBottom,
                    moldingColor,
                    posMod
                )
                self.create_box(
                    "roomScene/colliders/walls","moldingActorTop" + str(m) + "_" + str(i),
                    moldingDim,
                    moldingPosTop,
                    moldingColor,
                    posMod
                )
            

        offsetArr = [Gf.Vec2f(-1.0,-1.0), Gf.Vec2f(1.0,-1.0), Gf.Vec2f(-1.0,1.0), Gf.Vec2f(1.0,1.0)]

        hasWallNorth = self._roomTemplate.walls["N"].wallHeight > 0
        hasWallSouth = self._roomTemplate.walls["S"].wallHeight > 0
        hasWallEast = self._roomTemplate.walls["E"].wallHeight > 0
        hasWallWest = self._roomTemplate.walls["W"].wallHeight > 0

        for i in range(4):

            if i == 0:
                hasPillar = hasWallNorth or hasWallEast
            elif i == 1:
                hasPillar = hasWallNorth or hasWallWest
            elif i == 2:
                hasPillar = hasWallSouth or hasWallEast
            elif i == 3:
                hasPillar = hasWallSouth or hasWallWest

            if not hasPillar:
                continue

            pillarOffset = (rangeVal*tileSize + wallThickness*0.5)
            moldingThickness = wallThickness + (pillarJut + moldingJut)*2.0
            self.create_box(
                "roomScene/colliders/walls","pillarActor" + str(i),
                Gf.Vec3f(wallThickness + pillarJut*2.0, wallThickness + pillarJut*2.0, innerWallHeightMin),
                Gf.Vec3f(pillarOffset*offsetArr[i][0], pillarOffset*offsetArr[i][1], innerWallHeightMin*0.5 + moldingHeight),
                pillarColor
            )

            self.create_box(
                "roomScene/colliders/walls","pillarActorMoldingBottom" + str(i),
                Gf.Vec3f(moldingThickness, moldingThickness, moldingHeight),
                Gf.Vec3f(pillarOffset*offsetArr[i][0], pillarOffset*offsetArr[i][1], moldingHeight*0.5),
                moldingColor
            )

            self.create_box(
                "roomScene/colliders/walls","pillarActorMoldingTop" + str(i),
                Gf.Vec3f(moldingThickness, moldingThickness, moldingHeight),
                Gf.Vec3f(pillarOffset*offsetArr[i][0], pillarOffset*offsetArr[i][1], moldingHeight*0.5 + innerWallHeightMin + moldingHeight),
                moldingColor
            )

            if (innerWallHeightMin != innerWallHeightMax):
                self.create_box(
                    "roomScene/colliders/walls","pillarActorUpper" + str(i),
                    Gf.Vec3f(wallThickness + pillarJut*2.0, wallThickness + pillarJut*2.0, (innerWallHeightMax-innerWallHeightMin-moldingHeight)),
                    Gf.Vec3f(pillarOffset*offsetArr[i][0], pillarOffset*offsetArr[i][1], (innerWallHeightMax-innerWallHeightMin-moldingHeight)*0.5 + moldingHeight*2.0 + innerWallHeightMin),
                    pillarColor
                )
                self.create_box(
                    "roomScene/colliders/walls","pillarActorMoldingTopUpper" + str(i),
                    Gf.Vec3f(moldingThickness, moldingThickness, moldingHeight),
                    Gf.Vec3f(pillarOffset*offsetArr[i][0], pillarOffset*offsetArr[i][1], moldingHeight*0.5 + innerWallHeightMin + (innerWallHeightMax-innerWallHeightMin) + moldingHeight),
                    moldingColor
                )

        if self._roomTemplate.roofLevel > -1:
            self.create_archway(
                self._stage,
                "roomScene/renderables","roof",
                moldingColor,
                Gf.Vec3f(0.0, 0.0, self._roomTemplate.roofLevel),
                self._roomTemplate.roofAxis,
                newRangeVal*tileSize - wallThickness * 2.0,
                newRangeVal*tileSize*2.0,
                newRangeVal*tileSize*4.0,
                newRangeVal*tileSize - wallThickness * 2.0,
                roofOuterRadius,
                36,
                False
            )


    @staticmethod
    def get_world_camera_up(stage) -> Gf.Vec3d:
        up_axis = UsdGeom.GetStageUpAxis(stage)
        if up_axis == UsdGeom.Tokens.y:
            return Gf.Vec3d(0, 1, 0)
        if up_axis == UsdGeom.Tokens.z:
            return Gf.Vec3d(0, 0, 1)
        if up_axis == UsdGeom.Tokens.x:
            return Gf.Vec3d(1, 0, 0)
        return Gf.Vec3d(0, 1, 0)

    @staticmethod
    def set_position_world(stage, cam_path, usd_camera, world_position: Gf.Vec3d):
        __time = Usd.TimeCode.Default()
        world_xform = usd_camera.ComputeLocalToWorldTransform(__time)
        parent_xform = usd_camera.ComputeParentToWorldTransform(__time)
        iparent_xform = parent_xform.GetInverse()
        initial_local_xform = world_xform * iparent_xform
        pos_in_parent = iparent_xform.Transform(world_position)
        cam_prim = usd_camera.GetPrim()
        coi_attr = cam_prim.GetAttribute('omni:kit:centerOfInterest')
        prev_local_coi = coi_attr.Get(__time)
        coi_in_parent = iparent_xform.Transform(world_xform.Transform(prev_local_coi))
        cam_up = RoomHelper.get_world_camera_up(stage)
        new_local_transform = Gf.Matrix4d(1).SetLookAt(pos_in_parent, coi_in_parent, cam_up).GetInverse()
        new_local_transform = new_local_transform.SetTranslateOnly(pos_in_parent)

        omni.kit.commands.create(
            'TransformPrimCommand',
            path=cam_path,
            new_transform_matrix=new_local_transform,
            old_transform_matrix=initial_local_xform,
            usd_context_name="",
            time_code=__time
        ).do()

        if coi_attr and prev_local_coi:
            prev_world_coi = world_xform.Transform(prev_local_coi)
            new_local_coi = (new_local_transform * parent_xform).GetInverse().Transform(prev_world_coi)
            omni.kit.commands.create(
                'ChangePropertyCommand',
                prop_path=coi_attr.GetPath(),
                value=new_local_coi,
                prev=prev_local_coi,
                timecode=__time,
                usd_context_name="",
                type_to_create_if_not_exist=Sdf.ValueTypeNames.Vector3d
            ).do()

    @staticmethod
    def set_target_world(stage, cam_path, usd_camera, world_target: Gf.Vec3d):
        __time = Usd.TimeCode.Default()
        world_xform = usd_camera.ComputeLocalToWorldTransform(__time)
        parent_xform = usd_camera.ComputeParentToWorldTransform(__time)
        iparent_xform = parent_xform.GetInverse()
        initial_local_xform = world_xform * iparent_xform

        cam_prim = usd_camera.GetPrim()
        coi_attr = cam_prim.GetAttribute('omni:kit:centerOfInterest')
        prev_local_coi = coi_attr.Get(__time)

        pos_in_parent = iparent_xform.Transform(initial_local_xform.Transform(Gf.Vec3d(0, 0, 0)))
        # Rotate camera to look at new target, leaving it where it is
        cam_up = RoomHelper.get_world_camera_up(stage)
        coi_in_parent = iparent_xform.Transform(world_target)
        new_local_transform = Gf.Matrix4d(1).SetLookAt(pos_in_parent, coi_in_parent, cam_up).GetInverse()
        new_local_coi = (new_local_transform * parent_xform).GetInverse().Transform(world_target)

        omni.kit.commands.create(
            'ChangePropertyCommand',
            prop_path=coi_attr.GetPath(),
            value=new_local_coi,
            prev=prev_local_coi,
            timecode=__time,
            usd_context_name="",
            type_to_create_if_not_exist=Sdf.ValueTypeNames.Vector3d
        ).do()

        omni.kit.commands.create(
            'TransformPrimCommand',
            path=cam_path,
            new_transform_matrix=new_local_transform,
            old_transform_matrix=initial_local_xform,
            usd_context_name="",
            time_code=__time
        ).do()
