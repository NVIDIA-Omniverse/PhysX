// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
    PXR_NS::UsdPrim usdPrim;
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

    PXR_NS::GfVec3f gravityDirection;
    float gravityMagnitude;
};

struct CollisionGroupDesc : ObjectDesc
{
    CollisionGroupDesc()
    {
        type = ObjectType::eCollisionGroup;
    }

    PXR_NS::SdfPathVector filteredGroups;
    PXR_NS::UsdCollectionMembershipQuery collisionQuery;
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

    PXR_NS::UsdPrim sourceGprim;
    PXR_NS::SdfPath rigidBody;
    PXR_NS::GfVec3f localPos;
    PXR_NS::GfQuatf localRot;
    PXR_NS::GfVec3f localScale;
    PXR_NS::SdfPathVector materials;
    PXR_NS::SdfPathVector simulationOwners;
    PXR_NS::SdfPathVector filteredCollisions;
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

    PXR_NS::TfToken customGeometryToken;
};

struct CubeShapeDesc : ShapeDesc
{
    CubeShapeDesc(const PXR_NS::GfVec3f& inHalfExtents) : halfExtents(inHalfExtents)
    {
        type = ObjectType::eCubeShape;
    }

    PXR_NS::GfVec3f halfExtents;
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

    PXR_NS::TfToken approximation;
    PXR_NS::GfVec3f meshScale;
    PXR_NS::VtArray<PXR_NS::GfVec3f>* points;
    PXR_NS::VtArray<int>* indices;
    PXR_NS::VtArray<int>* faces;
    bool doubleSided;
};

// This struct represents a single sphere-point
// which is a position and a radius
struct SpherePoint
{
    PXR_NS::GfVec3f center;
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

using SdfPathSet = std::set<PXR_NS::SdfPath>;

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
    PXR_NS::SdfPathVector filteredCollisions;
    PXR_NS::SdfPathVector simulationOwners;
    PXR_NS::GfVec3f position;
    PXR_NS::GfQuatf rotation;
    PXR_NS::GfVec3f scale;

    bool rigidBodyEnabled;
    bool kinematicBody;
    bool startsAsleep;
    PXR_NS::GfVec3f linearVelocity;
    PXR_NS::GfVec3f angularVelocity;
};

struct BodyDesc : ObjectDesc
{
    BodyDesc() : bodyEnabled(true), kinematicBody(false), startsAsleep(false)
    {
    }

    PXR_NS::SdfPathVector filteredCollisions;
    PXR_NS::SdfPathVector simulationOwners;
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

    PXR_NS::GfMatrix4d transform;
    float mass;

    PXR_NS::SdfPath simMeshPath;
    PXR_NS::SdfPath simMeshMaterialPath;
    PXR_NS::TfToken simMeshBindPoseToken;
    bool simMeshLeftHandedOrientation;

    PXR_NS::SdfPathVector collisionGeomPaths; // all collision point based geometries
    PXR_NS::SdfPathVector collisionGeomMaterialPaths; // same size as collisionGeomPaths
    PXR_NS::TfTokenVector collisionGeomBindPoseTokens; // same size as collisionGeomPaths
    PXR_NS::TfTokenVector collisionGeomSelfCollisionFilterPoseTokens; // same size as collisionGeomPaths
    std::vector<bool> collisionGeomLeftHandedOrientations;

    PXR_NS::SdfPathVector skinGeomPaths; // all skin point based geometries
    PXR_NS::SdfPathVector skinGeomMaterialPaths; // same size as skinGeomPaths
    PXR_NS::TfTokenVector skinGeomBindPoseTokens; // same size as skinGeomPaths
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

    PXR_NS::SdfPathVector rootPrims;
    PXR_NS::SdfPathVector filteredCollisions;
    std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash> articulatedJoints;
    std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash> articulatedBodies;
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

    PXR_NS::SdfPath rel0;
    PXR_NS::SdfPath rel1;
    PXR_NS::SdfPath body0;
    PXR_NS::SdfPath body1;
    PXR_NS::GfVec3f localPose0Position;
    PXR_NS::GfQuatf localPose0Orientation;
    PXR_NS::GfVec3f localPose1Position;
    PXR_NS::GfQuatf localPose1Orientation;
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
    PXR_NS::SdfPath src0;
    PXR_NS::SdfPath src1;
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
    PXR_NS::SdfPath src0;
    PXR_NS::SdfPath src1;
};

inline void getCollisionShapeLocalTransfrom(PXR_NS::UsdGeomXformCache& xfCache,
                                            const PXR_NS::UsdPrim& collisionPrim,
                                            const PXR_NS::UsdPrim& bodyPrim,
                                            PXR_NS::GfVec3f& localPosOut,
                                            PXR_NS::GfQuatf& localRotOut,
                                            PXR_NS::GfVec3f& localScaleOut)
{
    // compute the shape rel transform to a body and store it.
    PXR_NS::GfVec3f localPos(0.0f);
    if (collisionPrim != bodyPrim)
    {
        bool resetXformStack;
        const PXR_NS::GfMatrix4d mat = xfCache.ComputeRelativeTransform(collisionPrim, bodyPrim, &resetXformStack);
        PXR_NS::GfTransform colLocalTransform(mat);

        localPos = PXR_NS::GfVec3f(colLocalTransform.GetTranslation());
        localRotOut = PXR_NS::GfQuatf(colLocalTransform.GetRotation().GetQuat());
        localScaleOut = PXR_NS::GfVec3f(colLocalTransform.GetScale());
    }
    else
    {
        const PXR_NS::GfMatrix4d mat(1.0);

        localRotOut = PXR_NS::GfQuatf(1.0f);
        localScaleOut = PXR_NS::GfVec3f(1.0f);
    }

    // now apply the body scale to localPos
    // physics does not support scales, so a rigid body scale has to be baked into the localPos
    const PXR_NS::GfTransform tr(xfCache.GetLocalToWorldTransform(bodyPrim));
    const PXR_NS::GfVec3d sc = tr.GetScale();

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
    virtual void parsePrim(const PXR_NS::UsdPrim& prim,
                           ObjectDesc* objectDesc,
                           uint64_t primTypes,
                           const PXR_NS::TfTokenVector& appliedApis) = 0;

    /**
     * Report physics object
     *
     * @param path USD prim path.
     * @param objectDesc The descriptor of the physics object that was found.
     */
    virtual void reportObjectDesc(const PXR_NS::SdfPath& path, const ObjectDesc* objectDesc) = 0;
};

} // namespace schema
} // namespace physics
} // namespace omni
