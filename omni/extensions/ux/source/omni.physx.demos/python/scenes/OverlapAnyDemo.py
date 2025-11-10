# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb, math, omni.timeline, carb.windowing
from omni.physx.scripts.physicsUtils import *
from pxr import UsdLux, UsdGeom, UsdPhysics, Gf, Vt, PhysxSchema, UsdUtils, PhysicsSchemaTools
import omni.physxdemos as demo
from omni.physx import get_physx_scene_query_interface, get_physx_simulation_interface
from omni.debugdraw import get_debug_draw_interface
from omni.physxui import get_physicsui_instance
from .SceneQueryBaseDemo import *
from .DebugDrawHelpers import *
from .KeyboardHelpers import *
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from omni.physxui import get_input_manager


CONTROL_UP = carb.input.KeyboardInput.UP
CONTROL_DOWN = carb.input.KeyboardInput.DOWN
CONTROL_LEFT = carb.input.KeyboardInput.LEFT
CONTROL_RIGHT = carb.input.KeyboardInput.RIGHT
CONTROL_RESET = carb.input.KeyboardInput.R

def setupD6JointLimit(d6Prim, limitName, low, high):
    limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, limitName)
    limitAPI.CreateLowAttr(low)
    limitAPI.CreateHighAttr(high)

def setupShape(stage, geom, collisionShapePath, color, position, orientation, scale, enableCollisions = True):
    geom.CreateDisplayColorAttr().Set([color])
    geom.AddTranslateOp().Set(position)
    geom.AddOrientOp().Set(orientation)
    geom.AddScaleOp().Set(scale)
    if enableCollisions:
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(collisionShapePath))

def setupCubeShape(stage, collisionShapePath, color, size, position, scale, enableCollisions = True):
    cubeGeom = UsdGeom.Cube.Define(stage, collisionShapePath)
    cubeGeom.CreateSizeAttr(size)
    setupShape(stage, cubeGeom, collisionShapePath, color, position, Gf.Quatf(1.0), scale, enableCollisions)

def createStaticWall(stage, rigidWallPath, blockSizeX, blockSizeY, blockSizeZ, position, color):
    UsdGeom.Xform.Define(stage, rigidWallPath)
    setupCubeShape(stage, rigidWallPath + "/box", color, 1.0, position, Gf.Vec3f(blockSizeX*2.0, blockSizeY*2.0, blockSizeZ*2.0))

def createBlock(stage, collisionShapePath, blockSize, position, color, enableCollisions = True):
    setupCubeShape(stage, collisionShapePath, color, blockSize, position, Gf.Vec3f(1.0, 1.0, 0.75), enableCollisions)

def rotate_around_axis(x, y, z, angle):
    s = math.sin(0.5 * angle)
    return Gf.Quatf(math.cos(0.5 * angle), s * x, s * y, s * z)

def createSlantedBlock(stage, collisionShapePath, blockSize, position, color, code):
    h = blockSize*0.5*0.75
    s = blockSize*0.5
    points = [
        Gf.Vec3f(-s, -s, h), Gf.Vec3f(s, -s, h),  Gf.Vec3f(s, s, h),
        Gf.Vec3f(-s, -s, -h), Gf.Vec3f(s, -s, -h),  Gf.Vec3f(s, s, -h),
        Gf.Vec3f(-s, -s, h), Gf.Vec3f(s, -s, h),  Gf.Vec3f(s, -s, -h),  Gf.Vec3f(-s, -s, -h),
        Gf.Vec3f(s, -s, h),  Gf.Vec3f(s, s, h),   Gf.Vec3f(s, s, -h),   Gf.Vec3f(s, -s, -h),
        Gf.Vec3f(s, s, h),   Gf.Vec3f(-s, -s, h), Gf.Vec3f(-s, -s, -h), Gf.Vec3f(s, s, -h),
    ]

    convexGeom = UsdGeom.Mesh.Define(stage, collisionShapePath)
    convexGeom.CreateFaceVertexCountsAttr([3, 3, 4, 4, 4])
    convexGeom.CreateFaceVertexIndicesAttr([0,2,1, 3,4,5, 6,7,8,9, 10,11,12,13, 14,15,16,17])
    convexGeom.CreatePointsAttr(points)
    meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(stage.GetPrimAtPath(collisionShapePath))
    meshCollisionAPI.CreateApproximationAttr("convexHull")
    setupShape(stage, convexGeom, collisionShapePath, color, position, rotate_around_axis(0.0, 0.0, 1.0, int(code)*3.14159*0.5), Gf.Vec3f(1.0, 1.0, 1.0))

def createSphereLight(stage, lightPath, radius, intensity, position, color = Gf.Vec3f(1.0, 1.0, 1.0)):
    # light
    sphereLight = UsdLux.SphereLight.Define(stage, lightPath)
    sphereLight.CreateRadiusAttr(radius)
    sphereLight.CreateIntensityAttr(intensity)
    sphereLight.AddTranslateOp().Set(position)
    sphereLight.CreateColorAttr(color)

def createBoardScene(stage, defaultPrimPath, nb_x, nb_y, blockSize, data):

    sunLight = UsdLux.DistantLight.Define(stage, defaultPrimPath + "/Sun")
    sunLight.CreateIntensityAttr(5000)
    sunLight.AddTranslateOp().Set(Gf.Vec3f(-90.0, 130.0, 110.0))
    rotZYXangles = Gf.Vec3f(290, 345, 0)
    sunLight.AddRotateZYXOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(rotZYXangles)

    groundColor = Gf.Vec3f(214.0/255.0, 152.0/255.0, 103.0/255.0) * 0.75
    wallColor = Gf.Vec3f(106.0/255.0, 52.0/255.0, 26.0/255.0) * 0.75
    targetColor = Gf.Vec3f(0.05, 1.0, 0.05)
    sphereColor = Gf.Vec3f(0.025, 0.05, 1.0)

    # Create side walls
    sideWallHeight = 1.0
    halfScale = blockSize * 0.5
    createStaticWall(stage, defaultPrimPath + "/sideWall0", halfScale, 5.0, sideWallHeight, Gf.Vec3f(7.0+halfScale, 0.0, -sideWallHeight*0.5), wallColor)
    createStaticWall(stage, defaultPrimPath + "/sideWall1", halfScale, 5.0, sideWallHeight, Gf.Vec3f(-7.0-halfScale, 0.0, -sideWallHeight*0.5), wallColor)
    createStaticWall(stage, defaultPrimPath + "/sideWall2", 7.0+blockSize, halfScale, sideWallHeight, Gf.Vec3f(0.0, 5.0+halfScale, -sideWallHeight*0.5), wallColor)
    createStaticWall(stage, defaultPrimPath + "/sideWall3", 7.0+blockSize, halfScale, sideWallHeight, Gf.Vec3f(0.0, -5.0-halfScale, -sideWallHeight*0.5), wallColor)

    # Create board - a compound actor containing all the block shapes
    boardPath = defaultPrimPath + "/board"
    rigidXform = UsdGeom.Xform.Define(stage, boardPath)
    boardPrim = stage.GetPrimAtPath(boardPath)

    physicsAPI = UsdPhysics.RigidBodyAPI.Apply(boardPrim)
    massAPI = UsdPhysics.MassAPI.Apply(boardPrim)

    physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(boardPrim)
    physxAPI.CreateSleepThresholdAttr().Set(0.0)
    physxAPI.CreateStabilizationThresholdAttr().Set(0.0)

    spherePrim = 0
    sphereInitPos = Gf.Vec3f(0.0)
    offset_x = (nb_x-1)*halfScale
    offset_y = -(nb_y-1)*halfScale
    count = 0
    for y in range(0, nb_y):
        for x in range(0, nb_x):
            blockPos = Gf.Vec3f(offset_x - x*blockSize, offset_y + y*blockSize, halfScale)
            if data[count] == 'S':
                sphereInitPos = blockPos
                spherePrim = add_rigid_sphere(stage, "/marble", blockSize*0.4, sphereInitPos, Gf.Quatf(1.0), sphereColor, 0.005, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
                physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(spherePrim)
                physxAPI.CreateAngularDampingAttr().Set(0.0)
                physxAPI.CreateSleepThresholdAttr().Set(0.0)
                physxAPI.CreateStabilizationThresholdAttr().Set(0.0)
            elif data[count] == 'E':
                createBlock(stage, boardPath + "/exitBlock", blockSize, blockPos, targetColor, False)
            elif data[count] == '*':
                createBlock(stage, boardPath + "/wallBlock" + str(count), blockSize, blockPos, wallColor)
            elif data[count] == '0' or data[count] == '1' or data[count] == '2' or data[count] == '3':
                createSlantedBlock(stage, boardPath + "/wallBlock" + str(count), blockSize, blockPos, wallColor, data[count])
            if data[count] != 'X':
                createBlock(stage, boardPath + "/groundBlock" + str(count), blockSize, blockPos + Gf.Vec3f(0.0, 0.0, -blockSize*0.75), groundColor)
            else:
                createSphereLight(stage, boardPath + "/SphereLight" + str(count), blockSize*0.45, 30000, blockPos + Gf.Vec3f(0.0, 0.0, -blockSize), Gf.Vec3f(1.0, 0.05, 0.025))
            count = count + 1

    massAPI.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0))

    # Create D6 joint between origin of the board and world
    d6Joint = UsdPhysics.Joint.Define(stage, defaultPrimPath + "/d6Joint")
    d6Joint.CreateBody0Rel().SetTargets([defaultPrimPath + "/board"])
    angleLimit = 5.0
    d6Prim = d6Joint.GetPrim()
    setupD6JointLimit(d6Prim, "transX", 1.0, -1.0)
    setupD6JointLimit(d6Prim, "transY", 1.0, -1.0)
    setupD6JointLimit(d6Prim, "transZ", 1.0, -1.0)
    setupD6JointLimit(d6Prim, "rotX", -angleLimit, angleLimit)
    setupD6JointLimit(d6Prim, "rotY", -angleLimit, angleLimit)
    setupD6JointLimit(d6Prim, "rotZ", 1.0, -1.0)

    return boardPrim, spherePrim, sphereInitPos

class OverlapAnyDemo(demo.Base):
    title = "Overlap Any"
    category = demo.Categories.SCENE_QUERY
    short_description = "Demo showing 'overlap any' usage"
    description = "This example shows how to use 'overlap any' calls to emulate triggers.\n\nAn 'overlap any' query is one that returns a single result and early exits.\n\n\
There is a trigger under the board to detect failures and a green target trigger to detect success. Use arrow keys to rotate the board.\n\n\
This demo also shows how to use the 'apply_force_at_pos' API (used to rotate the board).\n\nPress play (space) to run the simulation."

    def create(self, stage):
        self.stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.defaultPrimPath, scene = demo.setup_physics_scene(self, stage, gravityMod = 0.1)
        self._debugDraw = get_debug_draw_interface()
        self.globalTime = 0.0
        self.gameTime = 0.0
        self.displayMessage = 0

        data = "\
***************************\
*         * *** **   *    *\
********* *        * * ** *\
*       * ****X ****    ***\
* *** * *     *    ***0   *\
* * * * ***** *** 21   21 *\
* * * *     *     3*1 2*0 *\
*   * *** * * ***  3*X*0 2*\
* ***X*X* * * *     X*X  **\
* *   * * * * ***X 2*X*1 3*\
* * * * * * *   * 2*0 3*1 *\
* * * * * *** *** 30 X 30 *\
*   *       *       2*1   *\
* ******* *** *X****E*X** *\
*       *     *      *    *\
* ***** ******* **** * ****\
*X                X  *   S*\
***************************"
        self.boardPrim, self.spherePrim, self.sphereInitPos = createBoardScene(stage, self.defaultPrimPath, 27, 18, 0.5, data)
        self.rbo_encoded = PhysicsSchemaTools.sdfPathToInt(self.defaultPrimPath + "/board")

        self._sub_keyboard = init_keyboard(self._on_keyboard_event)
        self._controls = [CONTROL_UP, CONTROL_DOWN, CONTROL_LEFT, CONTROL_RIGHT, CONTROL_RESET]
        self._keys = {k: False for k in self._controls}

        # use inputmanager to block conflicting actions/hotkeys
        self._actions = ["OverlapAnyDemoUp", "OverlapAnyDemoDown", "OverlapAnyDemoLeft", "OverlapAnyDemoRight", "OverlapAnyDemoReset"]
        im = get_input_manager()
        for a, c in zip(self._actions, self._controls):
            im.register_keyboard_action(a, c, 0)

        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)
        self._info_box = None

        self.autofocus = True # autofocus on the scene at first update
        self.autofocus_zoom = 0.4 # Get a bit closer

    def on_shutdown(self):
        close_keyboard(self._sub_keyboard)
        self._sub_keyboard = None
        self._timeline = None
        self._timeline_subscription = None
        if self._info_box:
            self._info_box.destroy()
            self._info_box = None
        im = get_input_manager()
        for a in self._actions:
            im.unregister_action(a)

    def reset(self, msg = 0):
        set_or_add_translate_op(UsdGeom.Xformable(self.spherePrim), self.sphereInitPos)
        set_or_add_orient_op(UsdGeom.Xformable(self.boardPrim), Gf.Quatf(1.0))
        self.globalTime = 0.0
        self.displayMessage = msg

    def process_key(self, physxIFace, physxAPI, keyCode, force, position):
        if self._keys[keyCode]:            
            get_physx_simulation_interface().apply_force_at_pos(self.stage_id, self.rbo_encoded, carb.Float3(0.0, 0.0, force), position, "Impulse")
            physxAPI.CreateAngularDampingAttr().Set(0.0)    # Disable damping while player rotates the board

    def update(self, stage, dt, viewport, physxIFace):
        if not omni.timeline.get_timeline_interface().is_playing():
            if self._info_box:
                self._info_box.destroy()
                self._info_box = None
            return

        text = ["Time: " + str(int(self.globalTime)) + " seconds"]

        if omni.timeline.get_timeline_interface().is_playing():
            self.globalTime += dt
        if self.displayMessage==1 and self.globalTime<2.0:
            text.append("FAILED! Board has been reset.")
        elif self.displayMessage==2:
            text.append("SUCCESS! Your time: " + str(int(self.gameTime)) + " seconds")
        else:
            text.append("")

        if not self._info_box:
            physicsui = get_physicsui_instance()
            if physicsui:
                self._info_box = physicsui.create_text_info_box(text, screen_coords=[0, -200], width=250)
        else:
            self._info_box.set_text(text)
        

        # Detect overlap with trigger box under the board (failure)
        origin = carb.Float3(0.0, 0.0, -2.6)        #origin = carb.Float3(0.0, 0.0, 0.0)
        extent = carb.Float3(7.0, 5.0, 1.0)         #extent = carb.Float3(7.0, 5.0, 0.5)
        #draw_box(self._debugDraw, origin, extent)  #uncomment this to visualize the trigger box under the board
        if get_physx_scene_query_interface().overlap_box(extent, origin, carb.Float4(0.0, 0.0, 0.0, 1.0), None, True):
            self.reset(1)

        # Detect overlap with green target block (success)
        xform = UsdGeom.Xformable(stage.GetPrimAtPath(self.defaultPrimPath + "/board/exitBlock"))
        tf = Gf.Transform(xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        if get_physx_scene_query_interface().overlap_sphere(0.1, carb.Float3(tf.GetTranslation()), None, False):
            self.gameTime = self.globalTime
            self.reset(2)

        # Process input keys
        physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(self.boardPrim)
        physxAPI.CreateAngularDampingAttr().Set(100.0)  # Slows down the board when no keys are pressed (easier gameplay)
        offset = 6.0
        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        impulse = 0.5 * (metersPerUnit * kilogramsPerUnit)
        self.process_key(physxIFace, physxAPI, CONTROL_LEFT, impulse, carb.Float3(-offset, 0.0, 0.0))
        self.process_key(physxIFace, physxAPI, CONTROL_RIGHT, impulse, carb.Float3(offset, 0.0, 0.0))
        self.process_key(physxIFace, physxAPI, CONTROL_UP, impulse, carb.Float3(0.0, offset, 0.0))
        self.process_key(physxIFace, physxAPI, CONTROL_DOWN, impulse, carb.Float3(0.0, -offset, 0.0))
        if self._keys[CONTROL_RESET]:
            self.reset()

        # Update camera
        xform = UsdGeom.Xformable(stage.GetPrimAtPath(self.defaultPrimPath + "/marble"))
        tf = Gf.Transform(xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        target = Gf.Vec3d(tf.GetTranslation())
        direction = Gf.Vec3f(0.0 - target[0], 10.0 - target[1], 10.0 - target[2])
        direction.Normalize()
        camPos = Gf.Vec3d(target[0]+direction[0]*5.0, target[1]+direction[1]*5.0, target[2]+direction[2]*5.0)

        camera_state = ViewportCameraState()
        camera_state.set_target_world(target, True)
        camera_state.set_position_world(camPos, True)

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.input not in self._controls:
            return True
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._keys[event.input] = True
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._keys[event.input] = False
        return True

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self.globalTime = 0.0
            self.displayMessage = 0
