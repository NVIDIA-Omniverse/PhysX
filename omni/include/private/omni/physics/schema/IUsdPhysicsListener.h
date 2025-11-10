// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>

namespace omni
{
namespace physics
{
namespace schema
{
class DescCache;

struct ErrorCode
{
    enum Enum
    {
        eError,
        eWarning,
        eInfo
    };
};

struct ObjectType
{
    enum Enum
    {
        eUndefined,

        eScene,

        eRigidBody,

        eSphereShape,
        eCubeShape,
        eCapsuleShape,
        eCylinderShape,
        eConeShape,
        eMeshShape,
        eCustomShape,
        eSpherePointsShape,

        eJointFixed,
        eJointRevolute,
        eJointPrismatic,
        eJointSpherical,
        eJointDistance,
        eJointD6,
        eJointCustom,

        eMaterial,

        eArticulation,

        eCollisionGroup,

        eVolumeDeformableBody,
        eSurfaceDeformableBody,
        eCurvesDeformableBody,

        eDeformableMaterial,
        eSurfaceDeformableMaterial,
        eCurvesDeformableMaterial,

        eAttachmentVtxVtx,
        eAttachmentVtxTri,
        eAttachmentVtxTet,
        eAttachmentVtxCrv,
        eAttachmentVtxXform,
        eAttachmentTetXform,
        eAttachmentTriTri,
        eElementCollisionFilter
    };
};

struct Axis
{
    enum Enum
    {
        eX,
        eY,
        eZ
    };
};

struct JointAxis
{
    enum Enum
    {
        eDistance,
        eTransX,
        eTransY,
        eTransZ,
        eRotX,
        eRotY,
        eRotZ
    };
};

struct ObjectDesc
{
    ObjectDesc() : type(ObjectType::eUndefined)
    {
    }

    virtual ~ObjectDesc()
    {
    }

    ObjectType::Enum type;
    pxr::UsdPrim usdPrim;
    void* userData;
};

struct MaterialDesc : ObjectDesc
{
    MaterialDesc() : staticFriction(0.0f), dynamicFriction(0.0f), restitution(0.0f), density(-1.0f)
    {
        type = ObjectType::eMaterial;
    }

    float staticFriction;
    float dynamicFriction;
    float restitution;
    float density;
};

struct BaseMaterialDesc : ObjectDesc
{
    BaseMaterialDesc() : staticFriction(0.0f), dynamicFriction(0.0f), density(-1.0f)
    {
    }

    float staticFriction;
    float dynamicFriction;
    float density;
};

// Deformable material used for volume, surface and curve deformables.
struct DeformableMaterialDesc : BaseMaterialDesc
{
    DeformableMaterialDesc() : youngsModulus(0.0f), poissonsRatio(0.0f)
    {
        type = ObjectType::eDeformableMaterial;
    }

    float youngsModulus;
    float poissonsRatio;
};

struct SurfaceDeformableMaterialDesc : DeformableMaterialDesc
{
    SurfaceDeformableMaterialDesc()
        : surfaceThickness(0.0f), surfaceStretchStiffness(0.0f), surfaceShearStiffness(0.0f), surfaceBendStiffness(0.0f)
    {
        type = omni::physics::schema::ObjectType::eSurfaceDeformableMaterial;
    }

    float surfaceThickness;
    float surfaceStretchStiffness;
    float surfaceShearStiffness;
    float surfaceBendStiffness;
};

struct CurvesDeformableMaterialDesc : DeformableMaterialDesc
{
    CurvesDeformableMaterialDesc()
        : curveThickness(0.0f), curveStretchStiffness(0.0f), curveBendStiffness(0.0f), curveTwistStiffness(0.0f)
    {
        type = omni::physics::schema::ObjectType::eCurvesDeformableMaterial;
    }

    float curveThickness;
    float curveStretchStiffness;
    float curveBendStiffness;
    float curveTwistStiffness;
};

struct SceneDesc : ObjectDesc
{
    SceneDesc() : gravityDirection(0.0f, 0.0f, -1.0f), gravityMagnitude(9.81f)
    {
        type = ObjectType::eScene;
    }

    pxr::GfVec3f gravityDirection;
    float gravityMagnitude;
};

struct CollisionGroupDesc : ObjectDesc
{
    CollisionGroupDesc()
    {
        type = ObjectType::eCollisionGroup;
    }

    pxr::SdfPathVector filteredGroups;
    pxr::UsdCollectionMembershipQuery collisionQuery;
};

struct ShapeDesc : ObjectDesc
{
    ShapeDesc()
        : localPos(0.0f, 0.0f, 0.0f),
          localRot(1.0f, 0.0f, 0.0f, 0.0f),
          localScale(1.0f, 1.0f, 1.0f),
          collisionEnabled(true),
          masterDesc(false)
    {
    }

    pxr::UsdPrim sourceGprim;
    pxr::SdfPath rigidBody;
    pxr::GfVec3f localPos;
    pxr::GfQuatf localRot;
    pxr::GfVec3f localScale;
    pxr::SdfPathVector materials;
    pxr::SdfPathVector simulationOwners;
    pxr::SdfPathVector filteredCollisions;
    bool collisionEnabled;
    bool masterDesc;
};

struct SphereShapeDesc : ShapeDesc
{
    SphereShapeDesc(float inRadius = 0.0f) : radius(inRadius)
    {
        type = ObjectType::eSphereShape;
    }

    float radius;
};

struct CapsuleShapeDesc : ShapeDesc
{
    CapsuleShapeDesc(float inRadius = 0.0f, float half_height = 0.0f, Axis::Enum cap_axis = Axis::eX)
        : radius(inRadius), halfHeight(half_height), axis(cap_axis)
    {
        type = ObjectType::eCapsuleShape;
    }

    float radius;
    float halfHeight;
    Axis::Enum axis;
};

struct CylinderShapeDesc : ShapeDesc
{
    CylinderShapeDesc(float inRadius = 0.0f, float half_height = 0.0f, Axis::Enum cap_axis = Axis::eX)
        : radius(inRadius), halfHeight(half_height), axis(cap_axis)
    {
        type = ObjectType::eCylinderShape;
    }

    float radius;
    float halfHeight;
    Axis::Enum axis;
};

struct ConeShapeDesc : ShapeDesc
{
    ConeShapeDesc(float inRadius = 0.0f, float half_height = 0.0f, Axis::Enum cap_axis = Axis::eX)
        : radius(inRadius), halfHeight(half_height), axis(cap_axis)
    {
        type = ObjectType::eConeShape;
    }

    float radius;
    float halfHeight;
    Axis::Enum axis;
};

struct CustomShapeDesc : ShapeDesc
{
    CustomShapeDesc()
    {
        type = ObjectType::eCustomShape;
    }

    pxr::TfToken customGeometryToken;
};

struct CubeShapeDesc : ShapeDesc
{
    CubeShapeDesc(const pxr::GfVec3f& inHalfExtents) : halfExtents(inHalfExtents)
    {
        type = ObjectType::eCubeShape;
    }

    pxr::GfVec3f halfExtents;
};

struct MeshShapeDesc : ShapeDesc
{
    MeshShapeDesc() : meshScale(1.f, 1.f, 1.f), points(nullptr), indices(nullptr), faces(nullptr), doubleSided(false)
    {
        type = ObjectType::eMeshShape;
    }

    ~MeshShapeDesc()
    {
        if (points)
            delete points;
        if (indices)
            delete indices;
        if (faces)
            delete faces;
    }

    pxr::TfToken approximation;
    pxr::GfVec3f meshScale;
    pxr::VtArray<pxr::GfVec3f>* points;
    pxr::VtArray<int>* indices;
    pxr::VtArray<int>* faces;
    bool doubleSided;
};

// This struct represents a single sphere-point
// which is a position and a radius
struct SpherePoint
{
    pxr::GfVec3f center;
    float radius;
};

// This struct represents a collection of
// sphere points. Basically just an array of
// spheres which has been populated from a
// UsdGeomPoints primitive
struct SpherePointsShapeDesc : ShapeDesc
{
    SpherePointsShapeDesc(void)
    {
        type = ObjectType::eSpherePointsShape;
    }

    ~SpherePointsShapeDesc(void)
    {
    }

    std::vector<SpherePoint> spherePoints;
};

using SdfPathSet = std::set<pxr::SdfPath>;

struct RigidBodyDesc : ObjectDesc
{
    RigidBodyDesc()
        : position(0.0f, 0.0f, 0.0f),
          rotation(1.0f, 0.0f, 0.0f, 0.0f),
          scale(1.0f, 1.0f, 1.0f),
          rigidBodyEnabled(true),
          kinematicBody(false),
          startsAsleep(false),
          linearVelocity(0.0f, 0.0f, 0.0f),
          angularVelocity(0.0f, 0.0f, 0.0f)
    {
        type = ObjectType::eRigidBody;
    }

    SdfPathSet collisions;
    pxr::SdfPathVector filteredCollisions;
    pxr::SdfPathVector simulationOwners;
    pxr::GfVec3f position;
    pxr::GfQuatf rotation;
    pxr::GfVec3f scale;

    bool rigidBodyEnabled;
    bool kinematicBody;
    bool startsAsleep;
    pxr::GfVec3f linearVelocity;
    pxr::GfVec3f angularVelocity;
};

struct BodyDesc : ObjectDesc
{
    BodyDesc() : bodyEnabled(true), kinematicBody(false), startsAsleep(false)
    {
    }

    pxr::SdfPathVector filteredCollisions;
    pxr::SdfPathVector simulationOwners;
    bool bodyEnabled;
    bool kinematicBody;
    bool startsAsleep;
};

// Represets a deformable body including rest shape and
// graphical (skin) meshes. The type gets assigned during parsing:
// eDeformableVolumeBody, eDeformableSurfaceBody or eDeformableCurvesBody.
struct DeformableBodyDesc : BodyDesc
{
    DeformableBodyDesc() : transform(1.0f), mass(-1.0f)
    {
    }

    pxr::GfMatrix4d transform;
    float mass;

    pxr::SdfPath simMeshPath;
    pxr::SdfPath simMeshMaterialPath;
    pxr::TfToken simMeshBindPoseToken;

    pxr::SdfPathVector collisionGeomPaths; // all collision point based geometries
    pxr::SdfPathVector collisionGeomMaterialPaths; // same size as collisionGeomPaths
    pxr::TfTokenVector collisionGeomBindPoseTokens; // same size as collisionGeomPaths
    pxr::TfTokenVector collisionGeomSelfCollisionFilterPoseTokens; // same size as collisionGeomPaths

    pxr::SdfPathVector skinGeomPaths; // all skin point based geometries
    pxr::SdfPathVector skinGeomMaterialPaths; // same size as skinGeomPaths
    pxr::TfTokenVector skinGeomBindPoseTokens; // same size as skinGeomPaths
};

struct JointLimit
{
    JointLimit() : enabled(false), angle0(90.0), angle1(-90.0)
    {
    }

    bool enabled;
    union
    {
        float angle0;
        float lower;
        float minDist;
    };
    union
    {
        float angle1;
        float upper;
        float maxDist;
    };
};

struct JointDrive
{
    JointDrive()
        : enabled(false),
          targetPosition(0.0f),
          targetVelocity(0.0f),
          forceLimit(FLT_MAX),
          stiffness(0.0f),
          damping(0.0f),
          acceleration(false)
    {
    }

    bool enabled;
    float targetPosition;
    float targetVelocity;
    float forceLimit;
    float stiffness;
    float damping;
    bool acceleration;
};


struct ArticulationDesc : ObjectDesc
{
    ArticulationDesc()
    {
        type = ObjectType::eArticulation;
    }

    pxr::SdfPathVector rootPrims;
    pxr::SdfPathVector filteredCollisions;
    std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> articulatedJoints;
    std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> articulatedBodies;
};

using JointLimits = std::vector<std::pair<JointAxis::Enum, JointLimit>>;
using JointDrives = std::vector<std::pair<JointAxis::Enum, JointDrive>>;

struct JointDesc : public ObjectDesc
{
    JointDesc()
        : localPose0Position(0.0f, 0.0f, 0.0f),
          localPose0Orientation(1.0f, 0.0f, 0.0f, 0.0f),
          localPose1Position(0.0f, 0.0f, 0.0f),
          localPose1Orientation(1.0f, 0.0f, 0.0f, 0.0f),
          jointEnabled(true),
          breakForce(FLT_MAX), // USD default is none, which is not a float...
          breakTorque(FLT_MAX),
          excludeFromArticulation(false)
    {
    }

    pxr::SdfPath rel0;
    pxr::SdfPath rel1;
    pxr::SdfPath body0;
    pxr::SdfPath body1;
    pxr::GfVec3f localPose0Position;
    pxr::GfQuatf localPose0Orientation;
    pxr::GfVec3f localPose1Position;
    pxr::GfQuatf localPose1Orientation;
    bool jointEnabled;
    float breakForce;
    float breakTorque;
    bool excludeFromArticulation;
    bool collisionEnabled;
};

struct CustomJointDesc : public JointDesc
{
    CustomJointDesc()
    {
        type = ObjectType::eJointCustom;
    }
};

struct FixedJointDesc : public JointDesc
{
    FixedJointDesc()
    {
        type = ObjectType::eJointFixed;
    }
};

struct D6JointDesc : public JointDesc
{
    D6JointDesc()
    {
        type = ObjectType::eJointD6;
    }

    JointLimits jointLimits;
    JointDrives jointDrives;
};

struct PrismaticJointDesc : public JointDesc
{
    PrismaticJointDesc() : axis(Axis::eX)
    {
        type = ObjectType::eJointPrismatic;
    }

    Axis::Enum axis;
    JointLimit limit;
    JointDrive drive;
};

struct SphericalJointDesc : public JointDesc
{
    SphericalJointDesc() : axis(Axis::eX)
    {
        type = ObjectType::eJointSpherical;
    }

    Axis::Enum axis;
    JointLimit limit;
};

struct RevoluteJointDesc : public JointDesc
{
    RevoluteJointDesc() : axis(Axis::eX)
    {
        type = ObjectType::eJointRevolute;
    }

    Axis::Enum axis;
    JointLimit limit;
    JointDrive drive;
};

struct DistanceJointDesc : public JointDesc
{
    DistanceJointDesc() : minEnabled(false), maxEnabled(false)
    {
        type = ObjectType::eJointDistance;
    }

    bool minEnabled;
    bool maxEnabled;
    JointLimit limit;
};

struct AttachmentDesc : public ObjectDesc
{
    AttachmentDesc(ObjectType::Enum inType) : enabled(true), stiffness(0.0f), damping(0.0f)
    {
        type = inType;
    }

    bool enabled;
    pxr::SdfPath src0;
    pxr::SdfPath src1;
    float stiffness;
    float damping;
};

struct ElementCollisionFilterDesc : ObjectDesc
{
    ElementCollisionFilterDesc() : enabled(true)
    {
        type = ObjectType::eElementCollisionFilter;
    }

    bool enabled;
    pxr::SdfPath src0;
    pxr::SdfPath src1;
};

inline void getCollisionShapeLocalTransfrom(pxr::UsdGeomXformCache& xfCache,
                                            const pxr::UsdPrim& collisionPrim,
                                            const pxr::UsdPrim& bodyPrim,
                                            pxr::GfVec3f& localPosOut,
                                            pxr::GfQuatf& localRotOut,
                                            pxr::GfVec3f& localScaleOut)
{
    // compute the shape rel transform to a body and store it.
    pxr::GfVec3f localPos(0.0f);
    if (collisionPrim != bodyPrim)
    {
        bool resetXformStack;
        const pxr::GfMatrix4d mat = xfCache.ComputeRelativeTransform(collisionPrim, bodyPrim, &resetXformStack);
        pxr::GfTransform colLocalTransform(mat);

        localPos = pxr::GfVec3f(colLocalTransform.GetTranslation());
        localRotOut = pxr::GfQuatf(colLocalTransform.GetRotation().GetQuat());
        localScaleOut = pxr::GfVec3f(colLocalTransform.GetScale());
    }
    else
    {
        const pxr::GfMatrix4d mat(1.0);

        localRotOut = pxr::GfQuatf(1.0f);
        localScaleOut = pxr::GfVec3f(1.0f);
    }

    // now apply the body scale to localPos
    // physics does not support scales, so a rigid body scale has to be baked into the localPos
    const pxr::GfTransform tr(xfCache.GetLocalToWorldTransform(bodyPrim));
    const pxr::GfVec3d sc = tr.GetScale();

    for (int i = 0; i < 3; i++)
    {
        localPos[i] *= (float)sc[i];
    }

    localPosOut = localPos;
}

class IUsdPhysicsListener
{
public:
    IUsdPhysicsListener() = default;

    virtual ~IUsdPhysicsListener() = default;

    /**
     * Report physics object on given path during traversing the range
     *
     * Note that this might not return complete desc information
     * full desc is sent in reportObjectDesc function. ParsePrim cannot resolve
     * additional dependencies on objects that are yet to be parsed. These
     * are resolved after the traversal and full object desc is send through reportObjectDesc.
     *
     * @param prim USD prim that was parsed.
     * @param objectDesc The descriptor of the physics object that was found. Can be nullptr, each prim is sent
     *  so that users can parse its own data.
     */
    virtual void parsePrim(const pxr::UsdPrim& prim,
                           ObjectDesc* objectDesc,
                           uint64_t primTypes,
                           const pxr::TfTokenVector& appliedApis) = 0;

    /**
     * Report physics object
     *
     * @param path USD prim path.
     * @param objectDesc The descriptor of the physics object that was found.
     */
    virtual void reportObjectDesc(const pxr::SdfPath& path, const ObjectDesc* objectDesc) = 0;
};

} // namespace schema
} // namespace physics
} // namespace omni
