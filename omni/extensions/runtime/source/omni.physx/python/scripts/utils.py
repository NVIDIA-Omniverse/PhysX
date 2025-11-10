# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import typing
from pxr import Usd, UsdGeom, UsdShade, Sdf, Tf, Gf, UsdPhysics, PhysxSchema, UsdUtils
from omni.physx import get_physx_simulation_interface, get_physxunittests_interface
from omni.physx.bindings._physx import METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES
import traceback
from functools import partial
import carb.logging

HALF_PI = 1.57079632679489662
MAX_FLOAT = 3.40282347e38

AXES_INDICES = {"X": 0, "Y": 1, "Z": 2}

COOKED_DATA_TOKENS = [
    PhysxSchema.Tokens.convexHull,
    PhysxSchema.Tokens.convexDecomposition,
    PhysxSchema.Tokens.triangleMesh,
    PhysxSchema.Tokens.clothConstaint # DEPRECATED
]

MESH_APPROXIMATIONS = {
    "none": PhysxSchema.PhysxTriangleMeshCollisionAPI,
    "convexHull": PhysxSchema.PhysxConvexHullCollisionAPI,
    "convexDecomposition": PhysxSchema.PhysxConvexDecompositionCollisionAPI,
    "meshSimplification": PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI,
    "convexMeshSimplification": PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI,
    "boundingCube": None,
    "boundingSphere": None,
    "sphereFill": PhysxSchema.PhysxSphereFillCollisionAPI,
    "sdf": PhysxSchema.PhysxSDFMeshCollisionAPI,
}


def get_TfType_compatible(schema_type_or_type_name):
    if isinstance(schema_type_or_type_name, str):
        return Usd.SchemaRegistry().GetTypeFromSchemaTypeName(schema_type_or_type_name)
    else:
        return schema_type_or_type_name


def getSchemaPrimDef(schema):
    schema_type = get_TfType_compatible(schema)
    isApi = Usd.SchemaRegistry().IsAppliedAPISchema(schema_type)
    schemaToken = Usd.SchemaRegistry().GetAPISchemaTypeName(schema_type) if isApi else Usd.SchemaRegistry().GetConcreteSchemaTypeName(schema_type)
    return Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(schemaToken) if isApi else Usd.SchemaRegistry().FindConcretePrimDefinition(schemaToken)


def _getSchemaRegistryProperties(schema):
    primDef = getSchemaPrimDef(schema)
    return [primDef.GetSchemaPropertySpec(x) for x in primDef.GetPropertyNames()]


def _getSchemaOwner(prop, schema) -> Usd.SchemaBase:
    schemaTypeName = prop.owner.name
    isApi = Usd.SchemaRegistry().IsAppliedAPISchema(schemaTypeName)
    if not isApi:
        return schema

    namespaces = [Usd, UsdPhysics, PhysxSchema]
    for namespace in namespaces:
        if hasattr(namespace, schemaTypeName):
            return getattr(namespace, schemaTypeName)

    if schemaTypeName.startswith("Physics"):
        if hasattr(UsdPhysics, schemaTypeName[7:]):
            return getattr(UsdPhysics, schemaTypeName[7:])
        elif hasattr(PhysxSchema, schemaTypeName[7:]):
            return getattr(PhysxSchema, schemaTypeName[7:])

    raise ValueError(f"Unknown schemaTypeName {schemaTypeName}")


def _getSchemaPropertyName(schema, name):
    schema_type = get_TfType_compatible(schema)
    if Usd.SchemaRegistry().IsMultipleApplyAPISchema(schema_type):
        # prop.name becomes `prefix:__INSTANCE_NAME__:name`
        tokens = name.split(":", maxsplit=2)
        if len(tokens) >= 3:
            return tokens[2]
        else:
            return name
    else:
        return name


def getSchemaPropertyNames(schema):
    return [_getSchemaPropertyName(schema, prop.name) for prop in _getSchemaRegistryProperties(schema)]


def getSchemaAttribute(schema) -> typing.Tuple[str, Usd.SchemaBase]:
    props = _getSchemaRegistryProperties(schema)
    return [(_getSchemaPropertyName(schema, prop.name), _getSchemaOwner(prop, schema)) for prop in props if issubclass(prop.__class__, Sdf.AttributeSpec)]


def getSchemaRelationship(schema) -> typing.Tuple[str, Usd.SchemaBase]:
    props = _getSchemaRegistryProperties(schema)
    return [(_getSchemaPropertyName(schema, prop.name), _getSchemaOwner(prop, schema)) for prop in props if issubclass(prop.__class__, Sdf.RelationshipSpec)]


def getDerivedSchemas(schema):
    return [dt.pythonClass for dt in schema._GetStaticTfType().GetAllDerivedTypes()]


def getAxisAlignedVector(axis, len):
    vec = Gf.Vec3f(0.0)
    vec[AXES_INDICES[axis]] = len
    return vec


def getForwardVector(up_axis):
    if up_axis == "Y":
        return getAxisAlignedVector("Z", -1)
    elif up_axis == "Z":
        return getAxisAlignedVector("X", -1)


def isDefined(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        carb.log_warn("Prim at path %s is already defined" % path)
        return True

    return False


def hasSchema(prim, schemaName):
    schemas = prim.GetAppliedSchemas()
    for s in schemas:
        if s == schemaName:
            return True
    return False


def setPhysics(prim, kinematic, custom_execute_fn=None):
    if hasSchema(prim, "PhysicsRigidBodyAPI"):
        carb.log_warn("PhysicsRigidBodyAPI is already defined")
        return

    if custom_execute_fn is None:
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(prim)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    else:
        (_, physicsAPI) = custom_execute_fn(api=UsdPhysics.RigidBodyAPI, prim=prim)
        custom_execute_fn(api=PhysxSchema.PhysxRigidBodyAPI, prim=prim)
    physicsAPI.CreateRigidBodyEnabledAttr(True)
    physicsAPI.CreateKinematicEnabledAttr(kinematic)


def setCollider(prim, approximationShape="none", custom_execute_fn=None):
    # using this attribute instead of purpose=guide, so that the volumes will be easily renderable
    if prim.GetAttribute("omni:no_collision"):
        return

    if hasSchema(prim, "CollisionAPI"):
        carb.log_warn("CollisionAPI is already defined")
        return

    def isPartOfRigidBody(currPrim):
        if currPrim.HasAPI(UsdPhysics.RigidBodyAPI):
            return True

        currPrim = currPrim.GetParent()

        if not currPrim.IsValid():
            return False

        return isPartOfRigidBody(currPrim)
    isMesh = prim.IsA(UsdGeom.Mesh)
    if isMesh and approximationShape == "none" and isPartOfRigidBody(prim):
        carb.log_warn(f"setCollider: {prim.GetPath()} is a part of a rigid body. Resetting approximation shape from none (trimesh) to convexHull")
        approximationShape = "convexHull"

    if custom_execute_fn is None:
        collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
    else:
        (_, collisionAPI) = custom_execute_fn(api=UsdPhysics.CollisionAPI, prim=prim)
        custom_execute_fn(api=PhysxSchema.PhysxCollisionAPI, prim=prim)
    collisionAPI.CreateCollisionEnabledAttr().Set(True)

    if isMesh or prim.IsInstanceable():
        api = MESH_APPROXIMATIONS.get(approximationShape, 0)  # None is a valid value
        if api == 0:
            carb.log_warn(f"setCollider: invalid approximation type {approximationShape} provided for {prim.GetPath()}. Falling back to convexHull.")
            approximationShape = "convexHull"
            api = MESH_APPROXIMATIONS[approximationShape]
        if custom_execute_fn is None:
            if api is not None:
                api.Apply(prim)
            meshcollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(prim)
        else:
            if api is not None:
                custom_execute_fn(api=api, prim=prim)
            (_, meshcollisionAPI) = custom_execute_fn(api=UsdPhysics.MeshCollisionAPI, prim=prim)

        meshcollisionAPI.CreateApproximationAttr().Set(approximationShape)


def setColliderSubtree(prim, approximationShape="none", execute_command_fn=None):
    pit = iter(Usd.PrimRange(prim))
    for p in pit:
        if p.GetMetadata("hide_in_stage_window"):
            pit.PruneChildren()
            continue
        if p.IsA(UsdGeom.Gprim) or p.IsInstanceable():
            setCollider(p, approximationShape, execute_command_fn)


def setRigidBody(prim, approximationShape, kinematic, custom_execute_fn=None):
    setPhysics(prim, kinematic, custom_execute_fn)

    if prim.IsA(UsdGeom.Xformable):
        setColliderSubtree(prim, approximationShape, custom_execute_fn)
    else:
        setCollider(prim, approximationShape, custom_execute_fn)


def setStaticCollider(prim, approximationShape="none", custom_execute_fn=None):
    setColliderSubtree(prim, approximationShape, custom_execute_fn)

def removePhysics(prim, custom_execute_fn=None):
    if custom_execute_fn is None:
        ret = prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
    else:
        ret = custom_execute_fn(api=UsdPhysics.RigidBodyAPI, prim=prim)
        custom_execute_fn(api=PhysxSchema.PhysxRigidBodyAPI, prim=prim)

    if not ret:
        carb.log_error("Failed to remove a UsdPhysics.RigidBodyAPI from prim {}".format(prim.GetPrimPath().pathString))


def removeCollider(prim, custom_execute_fn=None):
    if custom_execute_fn is None:
        ret = prim.RemoveAPI(UsdPhysics.CollisionAPI)
        prim.RemoveAPI(PhysxSchema.PhysxCollisionAPI)
        if prim.IsA(UsdGeom.Mesh):
            prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
            prim.RemoveAPI(PhysxSchema.PhysxConvexHullCollisionAPI)
            prim.RemoveAPI(PhysxSchema.PhysxConvexDecompositionCollisionAPI)
            prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI)
            prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI)
        for token in COOKED_DATA_TOKENS:
            prim.RemoveAPI(PhysxSchema.PhysxCookedDataAPI, token)
    else:
        ret = custom_execute_fn(api=UsdPhysics.CollisionAPI, prim=prim)
        custom_execute_fn(api=PhysxSchema.PhysxCollisionAPI, prim=prim)
        if prim.IsA(UsdGeom.Mesh) or prim.IsInstanceable():
            custom_execute_fn(api=UsdPhysics.MeshCollisionAPI, prim=prim)
            custom_execute_fn(api=PhysxSchema.PhysxConvexHullCollisionAPI, prim=prim)
            custom_execute_fn(api=PhysxSchema.PhysxConvexDecompositionCollisionAPI, prim=prim)
            custom_execute_fn(api=PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI, prim=prim)
            custom_execute_fn(api=PhysxSchema.PhysxTriangleMeshCollisionAPI, prim=prim)
        for token in COOKED_DATA_TOKENS:
            custom_execute_fn(api=PhysxSchema.PhysxCookedDataAPI, prim=prim, api_prefix="physxCookedData", multiple_api_token=token)

    if not ret:
        carb.log_error("Failed to remove a UsdPhysics.CollisionAPI from prim {}".format(prim.GetPrimPath().pathString))


def removeColliderSubtree(prim, custom_execute_fn=None):
    primRange = Usd.PrimRange(prim)
    for p in primRange:
        if p.IsA(UsdGeom.Gprim):
            removeCollider(p, custom_execute_fn)


def removeRigidBodySubtree(prim, custom_execute_fn=None):
    removePhysics(prim, custom_execute_fn)
    removeColliderSubtree(prim, custom_execute_fn)


def removeRigidBody(prim, custom_execute_fn=None):
    if prim.IsA(UsdGeom.Xformable):
        removeRigidBodySubtree(prim, custom_execute_fn)
    else:
        removePhysics(prim, custom_execute_fn)
        removeCollider(prim, custom_execute_fn)


def removeStaticCollider(prim, custom_execute_fn=None):
    removeColliderSubtree(prim, custom_execute_fn)


def addPhysicsScene(stage, path):
    if isDefined(stage, path):
        return []
    UsdPhysics.Scene.Define(stage, path)
    paths = []
    paths.append(path)
    return paths


def getUnitScaleFactor(stage):
    metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
    scaleFactor = 1.0 / metersPerUnit
    return scaleFactor


def ensureMaterialOnPath(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        if not prim.IsA(UsdShade.Material):
            carb.log_warn(f"AddMaterial: Prim at path {path} is already defined and not a Material")
            return False
        return True

    UsdShade.Material.Define(stage, path)
    return True


def addRigidBodyMaterial(stage, path, density=None, staticFriction=None, dynamicFriction=None, restitution=None):
    if not ensureMaterialOnPath(stage, path):
        return False

    UsdShade.Material.Define(stage, path)
    material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(path))

    if staticFriction is not None:
        material.CreateStaticFrictionAttr().Set(staticFriction)
    if dynamicFriction is not None:
        material.CreateDynamicFrictionAttr().Set(dynamicFriction)
    if restitution is not None:
        material.CreateRestitutionAttr().Set(restitution)
    if density is not None:
        material.CreateDensityAttr().Set(density)

    return True


def addCollisionGroup(stage, path):
    collisionGroup = UsdPhysics.CollisionGroup.Define(stage, Sdf.Path(path))
    collisionGroup.CreateFilteredGroupsRel()


# Doing it this way because Usd.Prim.hasAPI doesn't know about our API schemas
def hasAPI(name, prim):
    foundSchema = False
    for schema in prim.GetAppliedSchemas():
        if schema == name:
            foundSchema = True
    return foundSchema


def descendantHasAPI(name, prim):
    children = prim.GetChildren()
    if prim.HasAPI(name):
        return True
    elif children:
        if prim.HasAttribute('xformOp:reset'):  # if this prim has op:reset, ignore its descendants
            return False
        for i in children:
            result = descendantHasAPI(name, i)
            if result:
                return True
    return False


def ancestorHasAPI(name, prim):
    if prim.HasAPI(name):
        return True
    elif prim.GetParent():
        if prim.HasAttribute('xformOp:reset'):  # if this prim has op:reset, ignore its ancestors
            return False
        return ancestorHasAPI(name, prim.GetParent())
    return False


def addPlaneCollider(stage, prim_path, upAxis):
    plane = UsdGeom.Plane.Define(stage, prim_path)
    plane.CreateAxisAttr().Set(upAxis)
    plane.CreatePurposeAttr().Set("guide")
    planePrim = stage.GetPrimAtPath(prim_path)
    setCollider(planePrim)


def addPairFilter(stage, paths, custom_execute_fn=None):
    for path in paths:
        prim = stage.GetPrimAtPath(path)

        if custom_execute_fn is None:
            filteringPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(prim)
        else:
            (_, filteringPairsAPI) = custom_execute_fn(api=UsdPhysics.FilteredPairsAPI, prim=prim)
        rel = filteringPairsAPI.CreateFilteredPairsRel()

        for otherPath in paths:
            if otherPath is not path:
                rel.AddTarget(Sdf.Path(otherPath))


def removePairFilter(stage, paths, custom_execute_fn=None):
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if custom_execute_fn is None:
            UsdPhysics.FilteredPairsAPI.Apply(prim)
        else:
            custom_execute_fn(api=UsdPhysics.FilteredPairsAPI, prim=prim)


def create_unused_path(stage, base_path, path):
    if stage.GetPrimAtPath(base_path + "/" + path).IsValid():
        uniquifier = 0
        while stage.GetPrimAtPath(base_path + "/" + path + str(uniquifier)).IsValid():
            uniquifier += 1
        path = path + str(uniquifier)
    return path


def createJoint(stage, joint_type, from_prim, to_prim):
    # for single selection use to_prim
    if to_prim is None:
        to_prim = from_prim
        from_prim = None

    from_path = from_prim.GetPath().pathString if from_prim is not None and from_prim.IsValid() else ""
    to_path = to_prim.GetPath().pathString if to_prim is not None and to_prim.IsValid() else ""
    single_selection = from_path == "" or to_path == ""

    # to_path can be not writable as in case of instancing, find first writable path
    joint_base_path = to_path
    base_prim = stage.GetPrimAtPath(joint_base_path)
    while base_prim != stage.GetPseudoRoot():
        if base_prim.IsInPrototype():
            base_prim = base_prim.GetParent()
        elif base_prim.IsInstanceProxy():
            base_prim = base_prim.GetParent()
        elif base_prim.IsInstanceable():
            base_prim = base_prim.GetParent()
        else:
            break
    joint_base_path = str(base_prim.GetPrimPath())
    if joint_base_path == '/':
        joint_base_path = ''

    joint_name = "/" + create_unused_path(stage, joint_base_path, joint_type + "Joint")
    joint_path = joint_base_path + joint_name

    if joint_type == "Fixed":
        component = UsdPhysics.FixedJoint.Define(stage, joint_path)
    elif joint_type == "Revolute":
        component = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
        component.CreateAxisAttr("X")
    elif joint_type == "Prismatic":
        component = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
        component.CreateAxisAttr("X")
    elif joint_type == "Spherical":
        component = UsdPhysics.SphericalJoint.Define(stage, joint_path)
        component.CreateAxisAttr("X")
    elif joint_type == "Distance":
        component = UsdPhysics.DistanceJoint.Define(stage, joint_path)
        component.CreateMinDistanceAttr(0.0)
        component.CreateMaxDistanceAttr(0.0)
    elif joint_type == "Gear":
        component = PhysxSchema.PhysxPhysicsGearJoint.Define(stage, joint_path)
    elif joint_type == "RackAndPinion":
        component = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(stage, joint_path)
    else:
        component = UsdPhysics.Joint.Define(stage, joint_path)
        prim = component.GetPrim()
        for limit_name in ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]:
            limit_api = UsdPhysics.LimitAPI.Apply(prim, limit_name)
            limit_api.CreateLowAttr(1.0)
            limit_api.CreateHighAttr(-1.0)

    xfCache = UsdGeom.XformCache()

    if not single_selection:
        to_pose = xfCache.GetLocalToWorldTransform(to_prim)
        from_pose = xfCache.GetLocalToWorldTransform(from_prim)
        rel_pose = to_pose * from_pose.GetInverse()
        rel_pose = rel_pose.RemoveScaleShear()
        pos1 = Gf.Vec3f(rel_pose.ExtractTranslation())
        rot1 = Gf.Quatf(rel_pose.ExtractRotationQuat())

        component.CreateBody0Rel().SetTargets([Sdf.Path(from_path)])
        component.CreateBody1Rel().SetTargets([Sdf.Path(to_path)])
        component.CreateLocalPos0Attr().Set(pos1)
        component.CreateLocalRot0Attr().Set(rot1)
        component.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        component.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    else:
        to_pose = xfCache.GetLocalToWorldTransform(to_prim)
        to_pose = to_pose.RemoveScaleShear()
        pos1 = Gf.Vec3f(to_pose.ExtractTranslation())
        rot1 = Gf.Quatf(to_pose.ExtractRotationQuat())

        component.CreateBody1Rel().SetTargets([Sdf.Path(to_path)])
        component.CreateLocalPos0Attr().Set(pos1)
        component.CreateLocalRot0Attr().Set(rot1)
        component.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        component.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

    component.CreateBreakForceAttr().Set(MAX_FLOAT)
    component.CreateBreakTorqueAttr().Set(MAX_FLOAT)

    return stage.GetPrimAtPath(joint_base_path + joint_name)


def createJoints(stage, joint_type, paths, join_to_parent=False):
    new_joints = []

    for path in paths:
        to_prim = stage.GetPrimAtPath(path)
        from_prim = None

        if join_to_parent and to_prim.IsValid():
            from_prim = to_prim.GetParent()
            if from_prim == from_prim.GetStage().GetPseudoRoot():
                from_prim = None

        new_joint = createJoint(stage, joint_type, from_prim, to_prim)

        if new_joint is not None and new_joint.IsValid():
            new_joints.append(new_joint)

    return new_joints


def removeAPISchemaProperties(api, prim):
    rbProp = getSchemaPropertyNames(api)
    for prop in rbProp:
        prim.RemoveProperty(prop)


def removeMultipleAPISchemaProperties(api, prim, api_prefix, multiple_token):
    rbProp = getSchemaPropertyNames(api)
    for prop in rbProp:
        name = _getPropertyMultipleName(api_prefix, multiple_token, prop)
        prim.RemoveProperty(name)


def _getPropertyCPPName(tftoken):
    return "".join([a[0].upper() + a[1:] for a in tftoken.split(":")[-1:]])


def _getPropertyMultipleName(api_prefix, multiple_token, name):
    return api_prefix + ":" + multiple_token + ":" + name


def createAPISchemaPropertyCache(api, prim):
    return [
        [(_getPropertyCPPName(a), prim.GetAttribute(a).Get(), api_attr)
            for a, api_attr in getSchemaAttribute(api)],
        [(_getPropertyCPPName(r), prim.GetRelationship(r).GetTargets(), api_rel)
            for r, api_rel in getSchemaRelationship(api)]
    ]


def applyAPISchemaPropertyCache(cache, api_inst, multiple_api_token=None):
    for name, value, api in cache[0]:
        if multiple_api_token is not None:
            current_api_inst = api(api_inst, multiple_api_token)
        else:
            current_api_inst = api(api_inst)
        createattr_fn = getattr(current_api_inst, "Create" + name + "Attr")
        createattr_fn(value)

    for name, value, api in cache[1]:
        if multiple_api_token is not None:
            current_api_inst = api(api_inst, multiple_api_token)
        else:
            current_api_inst = api(api_inst)
        createattr_fn = getattr(current_api_inst, "Create" + name + "Rel")
        createattr_fn().SetTargets(value)


def createMultipleAPISchemaPropertyCache(api, prim, api_prefix, multiple_token):
    return [
        [(_getPropertyCPPName(a),
          prim.GetAttribute(_getPropertyMultipleName(api_prefix, multiple_token, a)).Get(),
          api_attr)
         for a, api_attr in getSchemaAttribute(api)],
        [(_getPropertyCPPName(r),
          prim.GetRelationship(_getPropertyMultipleName(api_prefix, multiple_token, r)).GetTargets(),
          api_rel)
         for r, api_rel in getSchemaRelationship(api)]
    ]


def new_memory_stage(attach_stage=True, file_to_load=""):
    if file_to_load != "":
        stage = Usd.Stage.Open(file_to_load)
    else:
        stage = Usd.Stage.CreateInMemory()
    cache = UsdUtils.StageCache.Get()
    cache.Insert(stage)
    if (attach_stage):
        stage_id = cache.GetId(stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(stage_id)
    return stage


def release_memory_stage(stage, detach_stage=True):
    if (detach_stage):
        get_physx_simulation_interface().detach_stage()
    cache = UsdUtils.StageCache.Get()
    cache.Erase(stage)


def get_spatial_tendon_parent_link(prim, instance_name):
    if not prim:
        return None

    api = PhysxSchema.PhysxTendonAttachmentAPI(prim, instance_name)
    if not api:
        return None

    targets = api.GetParentLinkRel().GetTargets()
    if len(targets) == 0:
        return None

    return targets[0]


def get_spatial_tendon_attachment_candidates(prim: Usd.Prim) -> typing.List[str]:
    """
    Returns a list of PhysxTendonAttachment(Root)API instance names on the provided prim
    that are candidates for creating a parent relationship to. Notably, this list will exclude
    leaf attachments that are not suitable targets for a parent relationship.

    Args:
        prim: the Usd.Prim to parse for suitable attachment candidates

    Returns:
        A str list of attachment instance names
    """

    # early exit:
    if not prim or not prim.HasAPI(PhysxSchema.PhysxTendonAttachmentAPI):
        return []

    # otherwise go through:
    candidates = []
    rootApi = "PhysxTendonAttachmentRootAPI:"
    attApi = "PhysxTendonAttachmentAPI:"
    for api in prim.GetPrimTypeInfo().GetAppliedAPISchemas():
        if api.startswith(rootApi):
            candidates.append(api[len(rootApi):])
        elif api.startswith(attApi):
            candidates.append(api[len(attApi):])
    return candidates


def set_physics_scene_asyncsimrender(scenePrim, val=True):
    scene = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
    sceneUpdateType = scene.GetUpdateTypeAttr().Get()
    if sceneUpdateType != PhysxSchema.Tokens.Disabled:
        if val is False:
            scene.CreateUpdateTypeAttr().Set(PhysxSchema.Tokens.Synchronous)
        else:
            scene.CreateUpdateTypeAttr().Set(PhysxSchema.Tokens.Asynchronous)


def get_basis(up_axis):
    if up_axis == "Y":
        up = Gf.Vec3d(0, 1, 0)
        forward = Gf.Vec3d(0, 0, -1)
        right = Gf.Vec3d(1, 0, 0)
    else:
        up = Gf.Vec3d(0, 0, 1)
        forward = Gf.Vec3d(-1, 0, 0)
        right = Gf.Vec3d(0, 1, 0)
    return up, forward, right


def get_schema_instances(prim, schema_type_name):
    return {s[len(schema_type_name) + 1:] for s in prim.GetAppliedSchemas() if s.startswith(schema_type_name)}


# custom metadata utilities
def has_custom_metadata(object, metadata_name):
    return object.HasCustomDataKey(metadata_name)


def get_custom_metadata(object, metadata_name):
    return object.GetCustomDataByKey(metadata_name)


def set_custom_metadata(object, metadata_name, value):
    object.SetCustomDataByKey(metadata_name, value)


def clear_custom_metadata(object, metadata_name):
    object.ClearCustomDataByKey(metadata_name)


def set_local_space_velocities(prim, value):
    set_custom_metadata(prim, METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES, value)


def clear_local_space_velocities(prim):
    clear_custom_metadata(prim, METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES)


def get_aligned_body_transform(stage, cache, joint, body0base):
    # get both bodies if available
    b0paths = joint.GetBody0Rel().GetTargets()
    b1paths = joint.GetBody1Rel().GetTargets()

    b0prim = None
    b1prim = None

    if len(b0paths):
        b0prim = stage.GetPrimAtPath(b0paths[0])
        if not b0prim.IsValid():
            b0prim = None

    if len(b1paths):
        b1prim = stage.GetPrimAtPath(b1paths[0])
        if not b1prim.IsValid():
            b1prim = None

    b0locpos = joint.GetLocalPos0Attr().Get()
    b1locpos = joint.GetLocalPos1Attr().Get()
    b0locrot = joint.GetLocalRot0Attr().Get()
    b1locrot = joint.GetLocalRot1Attr().Get()

    # switch depending on which is the base
    if body0base:
        t0prim = b0prim
        t0locpos = b0locpos
        t0locrot = b0locrot
        t1prim = b1prim
    else:
        t0prim = b1prim
        t0locpos = b1locpos
        t0locrot = b1locrot
        t1prim = b0prim

    if t0prim:
        t0world = cache.GetLocalToWorldTransform(t0prim)
    else:
        t0world = Gf.Matrix4d()
        t0world.SetIdentity()

    if t1prim:
        t1world = cache.GetLocalToWorldTransform(t1prim)
    else:
        t1world = Gf.Matrix4d()
        t1world.SetIdentity()

    t0local = Gf.Transform()
    t0local.SetRotation(Gf.Rotation(Gf.Quatd(t0locrot)))
    t0local.SetTranslation(Gf.Vec3d(t0locpos))
    t0mult = t0local * Gf.Transform(t0world)

    t1world = Gf.Transform(t1world.GetInverse())
    rel_tm = t0mult * t1world
    return rel_tm


def safe_import_tests(source_module_name, submodules_list=None):
    """
    Tries to import tests if the tests ext is present. The tests ext is expected to be loaded as follows:

    1) use when the tests need to be visible in the test runner UI and the tests ext needs to be
    explicitly loaded by another ext or is included in the kit file

    [dependencies]
    "omni.physx.tests" = {optional=true}

    2) this will include the dependency only when running tests in the separate kit process, like when run
    using the generated batch files and the tests would then likely not be visible in the test runner UI

    [[test]]
    dependencies = ["omni.physx.tests"]

    """
    try:
        from omni.physxtests.utils import loaders
        loaders.import_tests_auto(source_module_name, submodules_list, 2)
        return True
    except ImportError as e:
        # report import error except the one expected to happen when outside our repo
        if e.name != "omni.physxtests" and e.name != "omni.physxtestsvisual":
            traceback.print_exc()
            carb.log_error(f"safe_import_tests encountered an ImportError exception {e}")
        return False


class CameraTransformHelper():
    def __init__(self, xform_prim, time=Usd.TimeCode.Default(), rows=True):
        self._rows = rows
        xform = UsdGeom.Xformable(xform_prim)
        self._m = xform.ComputeLocalToWorldTransform(time)
        self._t = None
        self._r = None

    def _extract_translation(self):
        if not self._t:
            self._t = self._m.ExtractTranslation()

    def _extract_rotation(self):
        if not self._r:
            self._r = self._m.ExtractRotationMatrix()

    def _get_rot(self, index):
        self._extract_rotation()
        return self._r.GetRow(index) if self._rows else self._r.GetColumn(index)

    def get_pos(self):
        self._extract_translation()
        return self._t

    def get_right(self):
        return self._get_rot(0)

    def get_forward(self):
        return -self._get_rot(2)

    def get_up(self):
        return self._get_rot(1)


def get_world_position(stage, path):
    imageable = UsdGeom.Imageable.Get(stage, path)
    obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
    alignedBox = obb.ComputeAlignedBox()
    minCorner = alignedBox.GetMin()
    maxCorner = alignedBox.GetMax()
    return (minCorner + maxCorner)/2.0


class ExpectMessage():
    def __init__(self, test_case, expected: str | list[str], expected_result=True, expect_all=True, result_fn=None, partial_string_match=False):
        """
        Test for the presence or absence of any message in the output (use with the with operator to scope).
        Initially used to test for printout of error messages and not having your test fail when they do.
        Creating a hierarchy of ExpectMessage will not work!

        Args:
            test_case: Test case instance to call assertTrue on.
            expected: A single string or a list of strings of messages that are expected (by default test will pass if any message from the list appears, see 'expect_all')
            expected_result: True for presence test, False for absence test.
            expect_all: True if you want test for the presence or absence of all from the  list of strings is passed in 'expected'
            result_fn: Callback with the result as parameter instead of passing to assertTrue directly.
        """
        self._expected = expected
        self._expected_result = expected_result
        self._expect_all = expect_all
        self._result_fn = result_fn
        self._partial_string_match = partial_string_match
        self._logging = carb.logging.acquire_logging()
        self._test_case = test_case

    def __enter__(self):
        def fail(test_case_fail, message):
            if not message.startswith("Test failure because of "):
                test_case_fail(message)

        self._test_case.fail = partial(fail, self._test_case.fail)
        if type(self._expected) is list:
            get_physxunittests_interface().start_logger_check_for_multiple(self._expected, self._expected_result, self._expect_all, self._partial_string_match)
        else:
            get_physxunittests_interface().start_logger_check(self._expected, self._expected_result, self._partial_string_match)
        expt = ("Expecting" if self._expected_result else "Not expecting") + (" all" if self._expect_all else "") + ": "
        print(f"{expt} '{self._expected}'.")

    def __exit__(self, type, value, traceback):
        res = get_physxunittests_interface().end_logger_check()
        print(f"Expectation {'met' if res else 'not met'}.")
        if self._result_fn:
            self._result_fn(res)
        else:
            self._test_case.assertTrue(res)
