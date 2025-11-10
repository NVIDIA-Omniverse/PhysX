// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <private/omni/physics/schematests/IUsdPhysicsTests.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <carb/Framework.h>

#include <carb/BindingsPythonUtils.h>

#include <memory>
#include <string>
#include <vector>

CARB_BINDINGS("omni.usdphysicstests.python")


namespace
{

using report_callback_function = std::function<void(const py::dict& objectDesc)>;

const carb::Float3& toFloat3(const pxr::GfVec3f& vec)
{
    return (const carb::Float3&) vec;
}

const carb::Float4& toFloat4(const pxr::GfQuatf& vec)
{
    return (const carb::Float4&) vec;
}

const carb::Double4& toDouble4(const pxr::GfVec4d& vec)
{
    return (const carb::Double4&) vec;
}

std::vector<const char*> pathToStringVector(const pxr::SdfPathVector& paths)
{
    std::vector<const char*> strings;
    strings.resize(paths.size());
    for (size_t i = 0; i < strings.size(); ++i)
    {
        strings[i] = paths[i].GetText();
    }
    return strings;
}

std::vector<const char*> tokenToStringVector(const pxr::TfTokenVector& tokens)
{
    std::vector<const char*> strings;
    strings.resize(tokens.size());
    for (size_t i = 0; i < strings.size(); ++i)
    {
        strings[i] = tokens[i].GetText();
    }
    return strings;
}

class  MarshalCallbacks: public omni::physics::schema::IUsdPhysicsListener
{
public:
    void registerCallback(report_callback_function cb)
    {
        mCallback = cb;

        omni::physics::schema::IUsdPhysics* schemaParser = carb::getCachedInterface<omni::physics::schema::IUsdPhysics>();
        schemaParser->registerPhysicsListener(this);
    }

    void deregisterCallback()
    {
        omni::physics::schema::IUsdPhysics* schemaParser = carb::getCachedInterface<omni::physics::schema::IUsdPhysics>();
        schemaParser->unregisterPhysicsListener(this);

        mCallback = nullptr;
    }

    void addShapeData(const omni::physics::schema::ShapeDesc* desc, py::dict& outDict)
    {
        outDict["local_position"] = toFloat3(desc->localPos);
        outDict["local_scale"] = toFloat3(desc->localScale);
        outDict["local_rotation"] = toFloat4(desc->localRot);

        outDict["rigid_body"] = desc->rigidBody != pxr::SdfPath() ? desc->rigidBody.GetText() : "";
        outDict["material"] = !desc->materials.empty() ? desc->materials[0].GetText() : "";

        outDict["enabled"] = desc->collisionEnabled;
        outDict["simulation_owners"] = pathToStringVector(desc->simulationOwners);
    }

    void addDeformableBodyData(const omni::physics::schema::DeformableBodyDesc* desc, py::dict& outDict)
    {
        outDict["simulation_owner"] = desc->simulationOwners.empty() ? "" : desc->simulationOwners[0].GetText();
        outDict["transform_row0"] = toDouble4(desc->transform.GetRow(0));
        outDict["transform_row1"] = toDouble4(desc->transform.GetRow(1));
        outDict["transform_row2"] = toDouble4(desc->transform.GetRow(2));
        outDict["transform_row3"] = toDouble4(desc->transform.GetRow(3));
        outDict["enabled"] = desc->bodyEnabled;
        outDict["kinematic"] = desc->kinematicBody;
        outDict["start_asleep"] = desc->startsAsleep;
        outDict["sim_mesh_path"] = desc->simMeshPath.GetText();
        outDict["collision_geom_paths"] = pathToStringVector(desc->collisionGeomPaths);
        outDict["skin_geom_paths"] = pathToStringVector(desc->skinGeomPaths);
        outDict["sim_mesh_material_path"] = desc->simMeshMaterialPath.GetText();
        outDict["collision_geom_material_paths"] = pathToStringVector(desc->collisionGeomMaterialPaths);
        outDict["skin_geom_material_paths"] = pathToStringVector(desc->skinGeomMaterialPaths);
        outDict["sim_mesh_bind_pose_token"] = desc->simMeshBindPoseToken.GetText();
        outDict["collision_geom_bind_pose_tokens"] = tokenToStringVector(desc->collisionGeomBindPoseTokens);
        outDict["skin_geom_bind_pose_tokens"] = tokenToStringVector(desc->skinGeomBindPoseTokens);
        outDict["collision_geom_self_collision_filter_pose_tokens"] = tokenToStringVector(desc->collisionGeomSelfCollisionFilterPoseTokens);
    }

    void addJointData(const omni::physics::schema::JointDesc* desc, py::dict& outDict)
    {
        outDict["body0"] = desc->body0 != pxr::SdfPath() ? desc->body0.GetText() : "";
        outDict["body1"] = desc->body1 != pxr::SdfPath() ? desc->body1.GetText() : "";

        outDict["local_pose0_position"] = toFloat3(desc->localPose0Position);
        outDict["local_pose0_rotation"] = toFloat4(desc->localPose0Orientation);
        outDict["local_pose1_position"] = toFloat3(desc->localPose1Position);
        outDict["local_pose1_rotation"] = toFloat4(desc->localPose1Orientation);

        outDict["break_force"] = desc->breakForce;
        outDict["break_torque"] = desc->breakTorque;

        outDict["exclude_from_articulation"] = desc->excludeFromArticulation;
        outDict["enabled"] = desc->jointEnabled;
    }

    void addAxisData(omni::physics::schema::Axis::Enum axis, py::dict& outDict)
    {
        outDict["axis"] = axis == omni::physics::schema::Axis::eX ? "X" :
            axis == omni::physics::schema::Axis::eY ? "Y" : "Z";
    }

    void addDriveData(const omni::physics::schema::JointDrive& drive, py::dict& outDict, const std::string prefix = "")
    {
        outDict[(prefix + std::string("acceleration")).c_str()] = drive.acceleration;
        outDict[(prefix + std::string("damping")).c_str()] = drive.damping;
        outDict[(prefix + std::string("stiffness")).c_str()] = drive.stiffness;
        outDict[(prefix + std::string("force_limit")).c_str()] = drive.forceLimit;
        outDict[(prefix + std::string("target_position")).c_str()] = drive.targetPosition;
        outDict[(prefix + std::string("target_velocity")).c_str()] = drive.targetVelocity;
    }

    const char* jointAxisString(omni::physics::schema::JointAxis::Enum axis)
    {
        switch (axis)
        {
        case omni::physics::schema::JointAxis::eDistance:
            return "distance";
        case omni::physics::schema::JointAxis::eRotX:
            return "rot_x";
        case omni::physics::schema::JointAxis::eRotY:
            return "rot_y";
        case omni::physics::schema::JointAxis::eRotZ:
            return "rot_z";
        case omni::physics::schema::JointAxis::eTransX:
            return "trans_x";
        case omni::physics::schema::JointAxis::eTransY:
            return "trans_y";
        case omni::physics::schema::JointAxis::eTransZ:
            return "trans_z";
        default:
            return "";
        }
        return "";
    }

    void addAttachmentData(const omni::physics::schema::AttachmentDesc* attachmentDesc, py::dict& outDict)
    {
        outDict["enabled"] = attachmentDesc->enabled;
        outDict["src0"] = attachmentDesc->src0 != pxr::SdfPath() ? attachmentDesc->src0.GetText() : "";
        outDict["src1"] = attachmentDesc->src1 != pxr::SdfPath() ? attachmentDesc->src1.GetText() : "";
        outDict["stiffness"] = attachmentDesc->stiffness;
        outDict["damping"] = attachmentDesc->damping;
    }

    void addCollisionFilterData(const omni::physics::schema::ElementCollisionFilterDesc* desc, py::dict& outDict)
    {
        outDict["enabled"] = desc->enabled;
        outDict["src0"] = desc->src0 != pxr::SdfPath() ? desc->src0.GetText() : "";
        outDict["src1"] = desc->src1 != pxr::SdfPath() ? desc->src1.GetText() : "";
    }

    virtual void parsePrim(const pxr::UsdPrim& prim, omni::physics::schema::ObjectDesc* objectDesc, uint64_t primTypes, const pxr::TfTokenVector& appliedApis)
    {
    }

    virtual void reportObjectDesc(const pxr::SdfPath& path, const omni::physics::schema::ObjectDesc* objectDesc)
    {
        if (!objectDesc)
            return;

        py::dict outDict;
        outDict["prim_path"] = path.GetText();
        switch (objectDesc->type)
        {
        case omni::physics::schema::ObjectType::eScene:
        {
            const omni::physics::schema::SceneDesc* sceneDesc = (const omni::physics::schema::SceneDesc*) objectDesc;
            outDict["object_type"] = "scene";
            outDict["gravity"] = carb::Float3 { sceneDesc->gravityDirection[0] * sceneDesc->gravityMagnitude, sceneDesc->gravityDirection[1] * sceneDesc->gravityMagnitude,
                sceneDesc->gravityDirection[2] * sceneDesc->gravityMagnitude };
        }
        break;
        case omni::physics::schema::ObjectType::eCollisionGroup:
        {            
            outDict["object_type"] = "collisionGroup";                        
        }
        break;
        case omni::physics::schema::ObjectType::eRigidBody:
        {
            const omni::physics::schema::RigidBodyDesc* rigidBody = (const omni::physics::schema::RigidBodyDesc*) objectDesc;
            outDict["object_type"] = "rigidBody";
            outDict["simulation_owner"] = rigidBody->simulationOwners.empty() ? "" : rigidBody->simulationOwners[0].GetText();
            outDict["position"] = toFloat3(rigidBody->position);
            outDict["scale"] = toFloat3(rigidBody->scale);
            outDict["rotation"] = toFloat4(rigidBody->rotation);
            outDict["enabled"] = rigidBody->rigidBodyEnabled;
            outDict["kinematic"] = rigidBody->kinematicBody;
            outDict["start_asleep"] = rigidBody->startsAsleep;
            outDict["linear_velocity"] = toFloat3(rigidBody->linearVelocity);
            outDict["angular_velocity"] = toFloat3(rigidBody->angularVelocity);
            std::vector<const char*> colNames;
            for (const pxr::SdfPath& cols  : rigidBody->collisions)
            {
                colNames.push_back(cols.GetText());
            }
            outDict["collisions"] = colNames;
        }
        break;
        case omni::physics::schema::ObjectType::eVolumeDeformableBody: {
            const omni::physics::schema::DeformableBodyDesc* deformableBody =
                (const omni::physics::schema::DeformableBodyDesc*)objectDesc;
            outDict["object_type"] = "volumeDeformableBody";
            addDeformableBodyData(deformableBody, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eSurfaceDeformableBody: {
            const omni::physics::schema::DeformableBodyDesc* deformableBody =
                (const omni::physics::schema::DeformableBodyDesc*)objectDesc;
            outDict["object_type"] = "surfaceDeformableBody";
            addDeformableBodyData(deformableBody, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eDeformableMaterial: {
            const omni::physics::schema::DeformableMaterialDesc* deformableMaterial =
                (const omni::physics::schema::DeformableMaterialDesc*)objectDesc;
            outDict["object_type"] = "volumeDeformableMaterial";
            outDict["density"] = deformableMaterial->density;
            outDict["dynamic_friction"] = deformableMaterial->dynamicFriction;
            outDict["static_friction"] = deformableMaterial->staticFriction;
            outDict["youngs_modulus"] = deformableMaterial->youngsModulus;
            outDict["poissons_ratio"] = deformableMaterial->poissonsRatio;
        }
        break;
        case omni::physics::schema::ObjectType::eSurfaceDeformableMaterial: {
            const omni::physics::schema::SurfaceDeformableMaterialDesc* deformableMaterial =
                (const omni::physics::schema::SurfaceDeformableMaterialDesc*)objectDesc;
            outDict["object_type"] = "surfaceDeformableMaterial";
            outDict["density"] = deformableMaterial->density;
            outDict["dynamic_friction"] = deformableMaterial->dynamicFriction;
            outDict["static_friction"] = deformableMaterial->staticFriction;
            outDict["youngs_modulus"] = deformableMaterial->youngsModulus;
            outDict["poissons_ratio"] = deformableMaterial->poissonsRatio;
            outDict["surface_thickness"] = deformableMaterial->surfaceThickness;
            outDict["surface_stretch_stiffness"] = deformableMaterial->surfaceStretchStiffness;
            outDict["surface_shear_stiffness"] = deformableMaterial->surfaceShearStiffness;
            outDict["surface_bend_stiffness"] = deformableMaterial->surfaceBendStiffness;
        }
        break;
        case omni::physics::schema::ObjectType::eSphereShape:
        {
            const omni::physics::schema::SphereShapeDesc* sphereShape = (const omni::physics::schema::SphereShapeDesc*) objectDesc;
            outDict["object_type"] = "sphereShape";
            addShapeData(sphereShape, outDict);
            outDict["radius"] = sphereShape->radius;
        }
        break;
        case omni::physics::schema::ObjectType::eCubeShape:
        {
            const omni::physics::schema::CubeShapeDesc* cubeShape = (const omni::physics::schema::CubeShapeDesc*) objectDesc;
            outDict["object_type"] = "cubeShape";
            addShapeData(cubeShape, outDict);
            outDict["half_extents"] = toFloat3(cubeShape->halfExtents);
        }
        break;
        case omni::physics::schema::ObjectType::eCapsuleShape:
        {
            const omni::physics::schema::CapsuleShapeDesc* capsuleShape = (const omni::physics::schema::CapsuleShapeDesc*) objectDesc;
            outDict["object_type"] = "capsuleShape";
            addShapeData(capsuleShape, outDict);
            outDict["radius"] = capsuleShape->radius;
            outDict["half_height"] = capsuleShape->halfHeight;
        }
        break;
        case omni::physics::schema::ObjectType::eConeShape:
        {
            const omni::physics::schema::ConeShapeDesc* coneShape = (const omni::physics::schema::ConeShapeDesc*) objectDesc;
            outDict["object_type"] = "coneShape";
            addShapeData(coneShape, outDict);
            outDict["radius"] = coneShape->radius;
            outDict["half_height"] = coneShape->halfHeight;
        }
        break;
        case omni::physics::schema::ObjectType::eCylinderShape:
        {
            const omni::physics::schema::CylinderShapeDesc* cylinderShape = (const omni::physics::schema::CylinderShapeDesc*) objectDesc;
            outDict["object_type"] = "cylinderShape";
            addShapeData(cylinderShape, outDict);
            outDict["radius"] = cylinderShape->radius;
            outDict["half_height"] = cylinderShape->halfHeight;
        }
        break;
        case omni::physics::schema::ObjectType::eMeshShape:
        {
            const omni::physics::schema::MeshShapeDesc* meshShape = (const omni::physics::schema::MeshShapeDesc*) objectDesc;
            outDict["object_type"] = "meshShape";
            addShapeData(meshShape, outDict);
            outDict["scale"] = toFloat3(meshShape->meshScale);
            outDict["approximation"] = meshShape->approximation.GetText();
            outDict["double_sided"] = meshShape->doubleSided;
        }
        break;
        case omni::physics::schema::ObjectType::eJointRevolute:
        {
            const omni::physics::schema::RevoluteJointDesc* revoluteJoint = (const omni::physics::schema::RevoluteJointDesc*) objectDesc;
            outDict["object_type"] = "revoluteJoint";
            addJointData(revoluteJoint, outDict);
            addAxisData(revoluteJoint->axis, outDict);
            if (revoluteJoint->limit.enabled)
            {
                outDict["limit_enabled"] = true;
                outDict["lower"] = revoluteJoint->limit.lower;
                outDict["upper"] = revoluteJoint->limit.upper;
            }
            else
            {
                outDict["limit_enabled"] = false;
            }
            if (revoluteJoint->drive.enabled)
            {
                outDict["drive_enabled"] = true;
                addDriveData(revoluteJoint->drive, outDict, std::string("drive_angular_"));
            }
            else
            {
                outDict["drive_enabled"] = false;
            }
        }
        break;
        case omni::physics::schema::ObjectType::eJointPrismatic:
        {
            const omni::physics::schema::PrismaticJointDesc* prismaticJoint = (const omni::physics::schema::PrismaticJointDesc*) objectDesc;
            outDict["object_type"] = "prismaticJoint";
            addJointData(prismaticJoint, outDict);
            addAxisData(prismaticJoint->axis, outDict);
            if (prismaticJoint->limit.enabled)
            {
                outDict["limit_enabled"] = true;
                outDict["lower"] = prismaticJoint->limit.lower;
                outDict["upper"] = prismaticJoint->limit.upper;
            }
            else
            {
                outDict["limit_enabled"] = false;
            }
            if (prismaticJoint->drive.enabled)
            {
                outDict["drive_enabled"] = true;
                addDriveData(prismaticJoint->drive, outDict, std::string("drive_linear_"));
            }
            else
            {
                outDict["drive_enabled"] = false;
            }
        }
        break;
        case omni::physics::schema::ObjectType::eJointSpherical:
        {
            const omni::physics::schema::SphericalJointDesc* sphericalJoint = (const omni::physics::schema::SphericalJointDesc*) objectDesc;
            outDict["object_type"] = "sphericalJoint";
            addJointData(sphericalJoint, outDict);
            addAxisData(sphericalJoint->axis, outDict);
            if (sphericalJoint->limit.enabled)
            {
                outDict["limit_enabled"] = true;
                outDict["cone_angle0"] = sphericalJoint->limit.angle0;
                outDict["cone_angle1"] = sphericalJoint->limit.angle1;
            }
            else
            {
                outDict["limit_enabled"] = false;
            }
        }
        break;
        case omni::physics::schema::ObjectType::eJointDistance:
        {
            const omni::physics::schema::DistanceJointDesc* distanceJoint = (const omni::physics::schema::DistanceJointDesc*) objectDesc;
            outDict["object_type"] = "distanceJoint";
            addJointData(distanceJoint, outDict);            
            if (distanceJoint->minEnabled)
            {
                outDict["min_enabled"] = true;
                outDict["min_distance"] = distanceJoint->limit.minDist;                
            }
            else
            {
                outDict["min_enabled"] = false;
            }
            if (distanceJoint->maxEnabled)
            {
                outDict["max_enabled"] = true;
                outDict["max_distance"] = distanceJoint->limit.maxDist;
            }
            else
            {
                outDict["max_enabled"] = false;
            }
        }
        break;
        case omni::physics::schema::ObjectType::eJointFixed:
        {
            const omni::physics::schema::FixedJointDesc* fixedJoint = (const omni::physics::schema::FixedJointDesc*) objectDesc;
            outDict["object_type"] = "fixedJoint";
            addJointData(fixedJoint, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eJointD6:
        {
            const omni::physics::schema::D6JointDesc* d6Joint = (const omni::physics::schema::D6JointDesc*) objectDesc;
            outDict["object_type"] = "d6Joint";
            addJointData(d6Joint, outDict);
            for (size_t i = 0; i < d6Joint->jointLimits.size(); i++)
            {
                omni::physics::schema::JointAxis::Enum axis = d6Joint->jointLimits[i].first;
                const omni::physics::schema::JointLimit& limit = d6Joint->jointLimits[i].second;
                std::string axisString = "limit_" + std::string(jointAxisString(axis));
                std::string axisStringEnabled = axisString + "_enabled";
                std::string axisStringLow = axisString + "_lower";
                std::string axisStringHigh = axisString + "_upper";
                outDict[axisStringEnabled.c_str()] = limit.enabled;
                outDict[axisStringLow.c_str()] = limit.lower;
                outDict[axisStringHigh.c_str()] = limit.upper;
            }
            for (size_t i = 0; i < d6Joint->jointDrives.size(); i++)
            {
                omni::physics::schema::JointAxis::Enum axis = d6Joint->jointDrives[i].first;
                const omni::physics::schema::JointDrive& drive = d6Joint->jointDrives[i].second;
                std::string axisString = "drive_" + std::string(jointAxisString(axis)) + "_";
                addDriveData(drive, outDict, axisString);
            }
        }
        break;
        case omni::physics::schema::ObjectType::eArticulation:
        {
            const omni::physics::schema::ArticulationDesc* articulation = (const omni::physics::schema::ArticulationDesc*) objectDesc;
            outDict["object_type"] = "articulation";
            std::vector<const char*> rootNames;
            for (size_t i = 0; i < articulation->rootPrims.size(); i++)
            {
                rootNames.push_back(articulation->rootPrims[i].GetText());
            }
            outDict["root_prims"] = rootNames;
        }
        break;
        case omni::physics::schema::ObjectType::eAttachmentVtxVtx:
        {
            const omni::physics::schema::AttachmentDesc* attachment = (const omni::physics::schema::AttachmentDesc*)objectDesc;
            outDict["object_type"] = "vtxVtxAttachment";
            addAttachmentData(attachment, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eAttachmentVtxTri:
        {
            const omni::physics::schema::AttachmentDesc* attachment = (const omni::physics::schema::AttachmentDesc*)objectDesc;
            outDict["object_type"] = "vtxTriAttachment";
            addAttachmentData(attachment, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eAttachmentVtxTet:
        {
            const omni::physics::schema::AttachmentDesc* attachment = (const omni::physics::schema::AttachmentDesc*)objectDesc;
            outDict["object_type"] = "vtxTetAttachment";
            addAttachmentData(attachment, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eAttachmentVtxCrv:
        {
            const omni::physics::schema::AttachmentDesc* attachment = (const omni::physics::schema::AttachmentDesc*)objectDesc;
            outDict["object_type"] = "vtxCrvAttachment";
            addAttachmentData(attachment, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eAttachmentVtxXform:
        {
            const omni::physics::schema::AttachmentDesc* attachment = (const omni::physics::schema::AttachmentDesc*)objectDesc;
            outDict["object_type"] = "vtxXformAttachment";
            addAttachmentData(attachment, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eAttachmentTetXform: {
            const omni::physics::schema::AttachmentDesc* attachment =
                (const omni::physics::schema::AttachmentDesc*)objectDesc;
            outDict["object_type"] = "tetXformAttachment";
            addAttachmentData(attachment, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eAttachmentTriTri: {
            const omni::physics::schema::AttachmentDesc* attachment =
                (const omni::physics::schema::AttachmentDesc*)objectDesc;
            outDict["object_type"] = "triTriAttachment";
            addAttachmentData(attachment, outDict);
        }
        break;
        case omni::physics::schema::ObjectType::eElementCollisionFilter:
        {
            const omni::physics::schema::ElementCollisionFilterDesc* collisionFilter = (const omni::physics::schema::ElementCollisionFilterDesc*)objectDesc;
            outDict["object_type"] = "collisionFilter";
            addCollisionFilterData(collisionFilter, outDict);
        }
        break;
        default:
            break;
        }

        if (mCallback)
            carb::StdFuncUtils<decltype(mCallback)>::callPythonCodeSafe(mCallback, outDict);
    }

private:
    report_callback_function mCallback;
};
static MarshalCallbacks gMarshalCallbacks;

PYBIND11_MODULE(_usdPhysicsTests, m)
{
    using namespace carb;
    using namespace omni::physics::schema;

    m.doc() = "pybind11 omni.usdphysicstests bindings";

    defineInterfaceClass<IUsdPhysicsTests>(m, "IUsdPhysicsTests", "acquire_usd_physics_tests_interface", "release_usd_physics_tests_interface")
        .def("attach_stage", wrapInterfaceFunction(&IUsdPhysicsTests::attachStage))
        .def("deattach_stage", wrapInterfaceFunction(&IUsdPhysicsTests::deattachStage))
        .def("report_object_desc_callback",
            [](IUsdPhysicsTests* tests, report_callback_function fn) {
                if (fn)
                   gMarshalCallbacks.registerCallback(fn);
                else
                   gMarshalCallbacks.deregisterCallback();
            },
            py::arg("fn"));
}
} // namespace
