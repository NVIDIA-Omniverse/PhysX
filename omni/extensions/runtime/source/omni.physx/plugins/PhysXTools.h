// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include "OmniPhysX.h"
#include "PhysXDefines.h"
#include "internal/InternalScene.h"

#include "ObjectDataQuery.h"

#include "usdLoad/AttachedStage.h"
#include "usdLoad/PrimUpdate.h"

#include "utils/Pair.h"

#include <carb/logging/Log.h>
#include <private/omni/physx/PhysxUsd.h>
#include <carb/events/IEvents.h>
#include <omni/physx/IPhysx.h>


#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContext.h>

#include <common/utilities/Utilities.h>
#include <common/foundation/TypeCast.h>
#include <common/foundation/Algorithms.h>

#include <set>

namespace omni
{
namespace physx
{

inline float degToRad(const float a)
{
    return 0.01745329251994329547f * a;
}

inline float radToDeg(const float a)
{
    return 57.29577951308232286465f * a;
}

inline pxr::GfVec3f degToRad(const pxr::GfVec3f& a)
{
    return pxr::GfVec3f(0.01745329251994329547f * a);
}

inline pxr::GfVec3f radToDeg(const pxr::GfVec3f& a)
{
    return pxr::GfVec3f(57.29577951308232286465f * a);
}

inline ::physx::PxVec3 radToDeg(const ::physx::PxVec3& a)
{
    return ::physx::PxVec3(57.29577951308232286465f * a);
}

template <class Type>
__forceinline Type* getPtr(PhysXType type, omni::physx::usdparser::ObjectId id)
{
    const internal::InternalPhysXDatabase& internaldb = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (id == omni::physx::usdparser::kInvalidObjectId)
        return nullptr;
    void* ptr = internaldb.getTypedRecord(type, id);
    return reinterpret_cast<Type*>(ptr);
}

template <class Type>
__forceinline Type* getInternalPtr(PhysXType type, omni::physx::usdparser::ObjectId id)
{
    const internal::InternalPhysXDatabase& internaldb = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (id == omni::physx::usdparser::kInvalidObjectId)
        return nullptr;
    void* ptr = internaldb.getInternalTypedRecord(type, id);
    return reinterpret_cast<Type*>(ptr);
}

namespace internal
{

template <typename T>
inline void removeFilteredObject(T ptr, std::unordered_map<Pair<T>, uint32_t, PairHash>& pairSet)
{
    typedef typename std::unordered_map<Pair<T>, uint32_t, PairHash>::iterator iterator;

    iterator it = pairSet.begin();
    while (it != pairSet.end())
    {
        if (it->first.contains(ptr))
        {
            it = pairSet.erase(it);
        }
        else
        {
            it++;
        }
    }
}

template <typename T>
inline void removeFilteredObject(T ptr, std::unordered_set<Pair<T>, PairHash>& pairSet)
{
    typedef typename std::unordered_set<Pair<T>, PairHash>::iterator iterator;

    iterator it = pairSet.begin();
    while (it != pairSet.end())
    {
        if ((*it).contains(ptr))
        {
            it = pairSet.erase(it);
        }
        else
        {
            it++;
        }
    }
}

template <typename T>
inline void swapFilteredObject(T oldPtr, T newPtr, std::unordered_set<Pair<T>, PairHash>& pairSet)
{
    typedef typename std::unordered_set<Pair<T>, PairHash>::iterator iterator;

    iterator it = pairSet.begin();
    while (it != pairSet.end())
    {
        (*it).swap(oldPtr, newPtr);
        it++;
    }
}

inline uint32_t convertToCollisionGroup(const usdparser::ObjectId collisionGroupId)
{
    const uint32_t collisionGroup =
        (collisionGroupId == usdparser::kInvalidObjectId) ? 0 : uint32_t(size_t(collisionGroupId));
    return collisionGroup;
}

inline void convertCollisionGroupToPxFilterData(const uint32_t collisionGroup, ::physx::PxFilterData& filterData)
{
    filterData.word2 = collisionGroup;
    // in word1 we store pair filtering information
    // in word3 we store contact modify information
}

inline uint32_t convertCollisionGroupFromPxFilterData(const ::physx::PxFilterData& fd)
{
    return fd.word2;
}

inline void convertFilterPairToPxFilterData(const uint32_t filterPair, ::physx::PxFilterData& filterData)
{
    filterData.word1 = filterPair;
    // in word2 we store collision group
    // in word3 we store contact modify information
}

inline uint32_t convertFilterPairFromPxFilterData(const ::physx::PxFilterData& fd)
{
    return fd.word1;
}


template <typename... ValuesT>
void sendErrorEvent(carb::events::IEventStreamPtr eventStream, ErrorEvent type, ValuesT... values)
{
    ::sendErrorEvent(eventStream, static_cast<carb::events::EventType>(type), values...);
}

inline bool getParentXform(const pxr::UsdPrim& prim, pxr::UsdPrim& parentXformPrim)
{
    if (pxr::UsdGeomXformable(prim).GetResetXformStack())
    {
        return false;
    }

    pxr::UsdPrim parent = prim.GetParent();
    while (parent && !parent.IsPseudoRoot())
    {
        if (parent.IsA<pxr::UsdGeomXformable>())
        {
            parentXformPrim = parent;
            return true;
        }
        parent = parent.GetParent();
    }

    return false;
}

template <typename T>
bool getValue(const usdparser::AttachedStage& attachedStage,
              const pxr::SdfPath& path,
              const pxr::TfToken& attributeName,
              const pxr::UsdTimeCode& timeCode,
              T& retVal)
{
    auto changeSource = attachedStage.getChangeSource();

    // If we know the data source is not from USD, get it from fabric first
    if (changeSource != usdparser::ChangeSource::eUsd)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        if (omniPhysX.getFabricBatchData())
        {
            return omniPhysX.getFabricBatchData()->getCurrentData<T>(retVal);
        }

        // call fabric getValue, which will first look in StageReaderWriter for a value
        // and fall back reading from USD if no value in StageReaderWriter
        omni::fabric::IStageReaderWriter* iStageReaderWriter = omniPhysX.getIStageReaderWriter();
        if (iStageReaderWriter && timeCode == pxr::UsdTimeCode::Default())
        {
            // grab the carb interface for StageReaderWriter and use it to access the
            // (potentially NULL) current stageInProgress for the UsdStage
            const omni::fabric::UsdStageId stageId = { uint64_t(attachedStage.getStageId()) };
            auto stageInProgress = iStageReaderWriter->get(stageId);

            if (stageInProgress.id)
            {
                // Grab a pointer to in-memory representation for the attribute value, in this
                // case a pointer to a T. Will be NULL if attribute doesn't exist in fabric
                auto valueSpan = iStageReaderWriter->getAttributeRd(
                    stageInProgress, omni::fabric::asInt(path), omni::fabric::asInt(attributeName));

                T* valuePtr = (T*)valueSpan.ptr;

                if (valuePtr)
                {
                    // We have a value stored for this attribute in fabric, return it
                    retVal = *valuePtr;
                    return true;
                }
            }
        }
    }

    // If we do not know the source / the source is USD / the source is Fabric but failed to fetch,
    // fallback to USD
    pxr::UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(path);
    if (!prim)
        return false;

    const pxr::UsdProperty prop = prim.GetProperty(attributeName);

    if (!prop.IsDefined())
        return false;

    CARB_ASSERT(prop.Is<pxr::UsdAttribute>());

    const pxr::UsdAttribute& attr = (const pxr::UsdAttribute&)(prop);

    return attr.Get(&retVal, timeCode);
}

template <typename T>
bool getArrayValue(const usdparser::AttachedStage& attachedStage,
                   const pxr::SdfPath& path,
                   const pxr::TfToken& attributeName,
                   const pxr::UsdTimeCode& timeCode,
                   T& retVal)
{
    // USD part
    pxr::UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(path);
    if (!prim)
        return false;

    const pxr::UsdProperty prop = prim.GetProperty(attributeName);

    if (!prop.IsDefined())
        return false;

    CARB_ASSERT(prop.Is<pxr::UsdAttribute>());

    const pxr::UsdAttribute& attr = (const pxr::UsdAttribute&)(prop);

    return attr.Get(&retVal, timeCode);
}

inline bool getRelationshipValue(const usdparser::AttachedStage& attachedStage,
                                 const pxr::SdfPath& path,
                                 const pxr::TfToken& relName,
                                 pxr::SdfPathVector& retVal)
{
    auto changeSource = attachedStage.getChangeSource();

    // If we know the data source is from fabric, get it from fabric first
    if (changeSource != usdparser::ChangeSource::eUsd)
    {
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();

        // call fabric getValue, which will first look in StageReaderWriter for a value
        // and fall back reading from USD if no value in StageReaderWriter
        omni::fabric::IStageReaderWriter* iStageReaderWriter = omniPhysX.getIStageReaderWriter();
        if (iStageReaderWriter)
        {
            // grab the carb interface for StageReaderWriter and use it to access the
            // (potentially NULL) current stageInProgress for the UsdStage
            const omni::fabric::UsdStageId stageId = { uint64_t(omniPhysX.getStageId()) };
            auto stageInProgress = iStageReaderWriter->get(stageId);

            if (stageInProgress.id)
            {
                auto relArray = iStageReaderWriter->getArrayAttributeRd(
                    stageInProgress, omni::fabric::asInt(path), omni::fabric::asInt(relName));

                omni::fabric::Path* valuePtr = (omni::fabric::Path*)relArray.ptr;

                if (valuePtr)
                {
                    for (size_t i = 0; i < relArray.elementCount; i++)
                    {
                        // We have a value stored for this attribute in fabric, return it
                        retVal.push_back(omni::fabric::toSdfPath(valuePtr[i]));
                    }
                    return true;
                }
            }
        }
    }

    // If we do not know the source / the source is USD / the source is Fabric but failed to fetch,
    // fallback to USD
    pxr::UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(path);
    if (!prim)
        return false;

    const pxr::UsdProperty prop = prim.GetProperty(relName);

    if (!prop.IsDefined())
        return false;

    CARB_ASSERT(prop.Is<pxr::UsdRelationship>());

    const pxr::UsdRelationship& rel = (const pxr::UsdRelationship&)(prop);

    return rel.GetTargets(&retVal);
}

inline bool getFloatBounded(const usdparser::AttachedStage& attachedStage,
                            const pxr::SdfPath& path,
                            const pxr::TfToken attributeName,
                            const pxr::UsdTimeCode timeCode,
                            float& outFloat,
                            const float lowBound,
                            const float upBound)
{
    float data = 0.0f;
    const bool result = getValue<float>(attachedStage, path, attributeName, timeCode, data);

    if (data > upBound)
    {
        data = upBound;
    }
    else if (data < lowBound)
    {
        data = lowBound;
    }
    outFloat = data;
    return result;
}

inline ::physx::PxQuat fixupCapsuleQuat(omni::physx::usdparser::Axis axis)
{
    ::physx::PxQuat fixupQ(::physx::PxIdentity);
    const float hRt2 = sqrt(2.0f) / 2.0f;
    if (axis == usdparser::eZ)
    {
        fixupQ = ::physx::PxQuat(hRt2, 0.0f, -hRt2, 0.0f);
    }
    else if (axis == usdparser::eY)
    {
        fixupQ = ::physx::PxQuat(hRt2, -hRt2, 0.0f, 0.0f);
    }
    return fixupQ;
}

inline ::physx::PxQuat fixupCapsuleQuat(const pxr::TfToken& axis)
{
    ::physx::PxQuat fixupQ(::physx::PxIdentity);
    const float hRt2 = sqrt(2.0f) / 2.0f;
    if (axis == pxr::UsdPhysicsTokens.Get()->z)
    {
        fixupQ = ::physx::PxQuat(hRt2, 0.0f, -hRt2, 0.0f);
    }
    else if (axis == pxr::UsdPhysicsTokens.Get()->y)
    {
        fixupQ = ::physx::PxQuat(hRt2, -hRt2, 0.0f, 0.0f);
    }
    return fixupQ;
}

inline ::physx::PxQuat fixupConeAndCylinderQuat(omni::physx::usdparser::Axis axis)
{
    ::physx::PxQuat fixupQ(::physx::PxIdentity);
    if (axis == usdparser::eZ)
        fixupQ = ::physx::PxQuat(::physx::PxPiDivTwo, ::physx::PxVec3(0, -1, 0));
    else if (axis == usdparser::eY)
        fixupQ = ::physx::PxQuat(::physx::PxPiDivTwo, ::physx::PxVec3(0, 0, 1));

    return fixupQ;
}

inline bool getJointAndLocalPose(const omni::physx::usdparser::AttachedStage& attachedStage,
                                 const pxr::SdfPath& jointPath,
                                 const ::physx::PxRigidActor* jointActor,
                                 ::physx::PxBase*& jointOut,
                                 ::physx::PxTransform& localFrame)
{
    ::physx::PxBase* joint =
        reinterpret_cast<::physx::PxBase*>(omni::physx::getObjectDataOrID<omni::physx::ObjectDataQueryType::ePHYSX_PTR>(
            jointPath, ePTJoint, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    if (!joint)
    {
        joint = reinterpret_cast<::physx::PxBase*>(
            omni::physx::getObjectDataOrID<omni::physx::ObjectDataQueryType::ePHYSX_PTR>(
                jointPath, ePTLinkJoint, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    }

    if (joint && joint->getConcreteType() == ::physx::PxJointConcreteType::eD6)
    {
        ::physx::PxRigidActor* actor0;
        ::physx::PxRigidActor* actor1;

        ::physx::PxJoint* jointPtr = (::physx::PxJoint*)joint;
        jointPtr->getActors(actor0, actor1);
        if (jointActor == actor0)
            localFrame = jointPtr->getLocalPose(::physx::PxJointActorIndex::eACTOR0);
        if (jointActor == actor1)
            localFrame = jointPtr->getLocalPose(::physx::PxJointActorIndex::eACTOR1);
    }
    else if (joint && joint->getConcreteType() == ::physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
    {
        ::physx::PxArticulationJointReducedCoordinate* jointPtr = (::physx::PxArticulationJointReducedCoordinate*)joint;

        if (jointActor == &jointPtr->getParentArticulationLink())
            localFrame = jointPtr->getParentPose();
        else if (jointActor == &jointPtr->getChildArticulationLink())
        {
            localFrame = jointPtr->getChildPose();
        }
        else
        {
            return false;
        }
    }

    jointOut = joint;
    return true;
}

#define UNKNOWN_FACE_ID 0xffffffff

class FaceIndexResolve
{
public:
    FaceIndexResolve(::physx::PxShape* shape) : mShape(shape)
    {
    }

    uint32_t resolveFaceIndex(uint32_t faceIndex)
    {
        uint32_t retVal = 0;
        const ::physx::PxGeometry& geom = mShape->getGeometry();
        if (geom.getType() == ::physx::PxGeometryType::eTRIANGLEMESH)
        {
            if (faceIndex == UNKNOWN_FACE_ID)
                return UNKNOWN_FACE_ID;

            const ::physx::PxTriangleMeshGeometry& triGeom = static_cast<const ::physx::PxTriangleMeshGeometry&>(geom);
            const ::physx::PxTriangleMesh* mesh = triGeom.triangleMesh;
            if (mesh && faceIndex < mesh->getNbTriangles())
            {
                const uint32_t remappedIndex = mesh->getTrianglesRemap()[faceIndex];
                const uint32_t* remmapedTriangle = getMeshCache()->getTriangleMeshFaceMap(mesh);
                if (remmapedTriangle)
                    retVal = remmapedTriangle[remappedIndex];
            }
            else
            {
                return UNKNOWN_FACE_ID;
            }
        }
        return retVal;
    }

private:
    ::physx::PxShape* mShape;
};

} // namespace internal
} // namespace physx
} // namespace omni
