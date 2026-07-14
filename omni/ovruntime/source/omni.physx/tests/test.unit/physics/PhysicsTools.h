// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#define USE_PHYSX_GPU 1 // GPU Rigid Bodies

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <carb/dictionary/IDictionary.h>


namespace omni
{
namespace physx
{
struct IPhysxUnitTests;
struct IPhysxVehicle;
struct IPhysxVehicleTesting;
struct IPhysxCooking;
struct IPhysxCookingServicePrivate;
struct IPhysxCookingService;
struct IPhysxSceneQuery;
struct IPhysxPrivate;
struct IPhysxReplicator;
struct IPhysxFabric;
}
}

class ScopedFabricActivation
{
public:
    ScopedFabricActivation();
    ~ScopedFabricActivation();

    // no copying!!
    ScopedFabricActivation(const ScopedFabricActivation&) = delete;
    void operator=(const ScopedFabricActivation&) = delete;

    omni::physx::IPhysxFabric* mIPhysxFabric = nullptr;
};

class ScopedOmniPhysicsActivation
{
public:
    ScopedOmniPhysicsActivation();
    ~ScopedOmniPhysicsActivation();

    ScopedOmniPhysicsActivation(const ScopedOmniPhysicsActivation&) = delete;
    void operator=(const ScopedOmniPhysicsActivation&) = delete;
};

class ExecOnDestruction
{
public:
    ExecOnDestruction(std::function<void()> fn) : mFn(fn) {}
    ~ExecOnDestruction() { mFn(); }
private:
    std::function<void()> mFn;
};

class ErrorEventListener : public carb::events::IEventListener
{
public:
    ErrorEventListener()
        : mEnabled(true), mDict(nullptr)
    {
    }

    void onEvent(carb::events::IEvent* e) override
    {
        if (!mEnabled || !mDict)
            return;

        int eventType = int(e->type);
        if (eventType == omni::physx::ErrorEvent::ePhysxError || eventType == omni::physx::ErrorEvent::ePhysxCudaError)
        {
            const carb::dictionary::Item* item = mDict->getItem(e->payload, "errorString");            
            FAIL(mDict->getStringBuffer(item));
        }
    }

    size_t addRef() override
    {
        return ++mRefCount;
    }

    size_t release() override
    {
        // hmmm
        if (mRefCount)
        {
            --mRefCount;
        }
        return mRefCount;
    }

    size_t mRefCount = 0;

    void setEnabled(bool val)
    {
        mEnabled = val;
    }
    
    bool enabled() const
    {
        return mEnabled;
    }

    void setDict(carb::dictionary::IDictionary* val)
    {
        mDict = val;
    }

private:
    bool mEnabled;
    carb::dictionary::IDictionary* mDict;
};

class PhysicsTest
{
public:

    static PhysicsTest* getPhysicsTests();

    void release();

    omni::physx::IPhysx* acquirePhysxInterface();

    omni::physx::IPhysxUnitTests* acquirePhysxUnitTestInterface();

    omni::physx::IPhysxSimulation* acquirePhysxSimulationInterface();

    omni::physx::IPhysxCooking* acquirePhysxCookingInterface();

    omni::physx::IPhysxCookingServicePrivate* acquirePhysxCookingServicePrivateInterface();

    omni::physx::IPhysxCookingService* acquirePhysxCookingServiceInterface();

    omni::physx::IPhysxSceneQuery* acquirePhysxSceneQueryInterface();

    omni::physx::IPhysxPrivate* acquirePhysxPrivateInterface();

    omni::physx::IPhysxReplicator* acquirePhysxReplicatorInterface();

    carb::settings::ISettings* acquireSettingsInterface();

    carb::AppScoped* getApp() { return mApp; }

    std::string getDataDirectory();

    std::string getUnitTestsDataDirectory();

    void enablePVD(bool enable);

    const ErrorEventListener& getErrorEventListener() const
    {
        return mErrorListener;
    }

    ErrorEventListener& getErrorEventListener()
    {
        return mErrorListener;
    }

private:
    PhysicsTest();

    ~PhysicsTest();

    carb::AppScoped*                                mApp;
    ErrorEventListener                              mErrorListener;
    carb::events::IEventStreamPtr                   mEventStreamPtr;
    carb::events::ISubscriptionPtr                  mSubscriptionPtr;
};

inline PXR_NS::GfVec3f getPhysicsPrimPos(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& path)
{
    PXR_NS::GfVec3f pos(0.0f);
    PXR_NS::UsdPrim prim = stage->GetPrimAtPath(path);
    if (prim)
    {
        static PXR_NS::TfToken translateToken("xformOp:translate");
        PXR_NS::UsdAttribute attr = prim.GetAttribute(translateToken);
        if (attr)
        {
            // fallback to check doubles eventually
            if (!attr.Get(&pos))
            {
                PXR_NS::GfVec3d posd(0.0);
                if (attr.Get(&posd))
                {
                    pos = PXR_NS::GfVec3f(posd);
                }
            }
        }
    }
    return pos;
}

inline PXR_NS::GfQuatf getPhysicsPrimQuat(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& path)
{
    PXR_NS::GfQuatf quat(1.0f);
    PXR_NS::UsdPrim prim = stage->GetPrimAtPath(path);
    if(prim)
    {
        static PXR_NS::TfToken orientToken("xformOp:orient");
        PXR_NS::UsdAttribute attr = prim.GetAttribute(orientToken);
        if(attr)
        {
            // fallback to check doubles eventually
            if(!attr.Get(&quat))
            {
                PXR_NS::GfQuatd quatD(0.0);
                if(attr.Get(&quatD))
                {
                    quat = PXR_NS::GfQuatf(quatD);
                }
            }
        }
    }
    return quat;
}

inline void setPhysicsPrimPos(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& path, const PXR_NS::GfVec3f& pos)
{
    PXR_NS::UsdPrim prim = stage->GetPrimAtPath(path);
    if (prim)
    {
        static PXR_NS::TfToken translateToken("xformOp:translate");
        PXR_NS::UsdAttribute attr = prim.GetAttribute(translateToken);
        if (attr)
        {
            // fallback to check doubles eventually
            if (!attr.Set(pos))
            {
                PXR_NS::GfVec3d posd(pos);
                attr.Set(posd);
            }
        }
    }    
}

template<class T>
inline T* getPhysxBaseDerivedFromPathChecked(const PXR_NS::SdfPath& path, const omni::physx::PhysXType type)
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysx* physxInterface = physicsTests.acquirePhysxInterface();
    ::physx::PxBase* pxBase = reinterpret_cast<::physx::PxBase*>(physxInterface->getPhysXPtr(path, type));
    REQUIRE(pxBase);
    T* pxPtr = pxBase->is<T>();
    REQUIRE(pxPtr);
    return pxPtr;
}

// AD Todo: remove this again once we have types for particle buffers.
template<class T>
inline T* getPhysxPtrFromPathUnchecked(const PXR_NS::SdfPath& path, const omni::physx::PhysXType type)
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysx* physxInterface = physicsTests.acquirePhysxInterface();
    ::physx::PxBase* pxBase = reinterpret_cast<::physx::PxBase*>(physxInterface->getPhysXPtr(path, type));
    return reinterpret_cast<T*>(pxBase);
}

inline physx::PxScene* getPhysxSceneAtPathChecked(const PXR_NS::SdfPath& scenePath)
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysx* physxInterface = physicsTests.acquirePhysxInterface();
    ::physx::PxScene* pxScene = reinterpret_cast<::physx::PxScene*>(physxInterface->getPhysXPtr(scenePath, omni::physx::PhysXType::ePTScene));
    REQUIRE(pxScene);
    return pxScene;
}

template <typename T>
inline void compare(const T& v0, const T& v1, float epsilon)
{
    CHECK(fabsf(v0[0] - v1[0]) < epsilon);
    CHECK(fabsf(v0[1] - v1[1]) < epsilon);
    CHECK(fabsf(v0[2] - v1[2]) < epsilon);
}

inline void compare(const ::physx::PxVec3& v0, const PXR_NS::GfVec3f& v1, float epsilon)
{
    CHECK(fabsf(v0.x - v1[0]) < epsilon);
    CHECK(fabsf(v0.y - v1[1]) < epsilon);
    CHECK(fabsf(v0.z - v1[2]) < epsilon);
}


inline void compare(const carb::Float3& v0, const carb::Float3& v1, float epsilon)
{
    CHECK(fabsf(v0.x - v1.x) < epsilon);
    CHECK(fabsf(v0.y - v1.y) < epsilon);
    CHECK(fabsf(v0.z - v1.z) < epsilon);
}

inline void compare(const carb::Double3& v0, const carb::Double3& v1, float epsilon)
{
    CHECK(fabsf(v0.x - v1.x) < epsilon);
    CHECK(fabsf(v0.y - v1.y) < epsilon);
    CHECK(fabsf(v0.z - v1.z) < epsilon);
}

inline void compare(const carb::Float3& v0, const PXR_NS::GfVec3f& v1, float epsilon)
{
    CHECK(fabsf(v0.x - v1[0]) < epsilon);
    CHECK(fabsf(v0.y - v1[1]) < epsilon);
    CHECK(fabsf(v0.z - v1[2]) < epsilon);
}


inline void compare(const ::physx::PxQuat& v0, const ::physx::PxQuat& v1, float epsilon)
{
    CHECK(fabsf(v0.x - v1.x) < epsilon);
    CHECK(fabsf(v0.y - v1.y) < epsilon);
    CHECK(fabsf(v0.z - v1.z) < epsilon);
    CHECK(fabsf(v0.w - v1.w) < epsilon);
}

inline void compare(const ::physx::PxQuat& v0, const PXR_NS::GfQuatf& v1, float epsilon)
{
    CHECK(fabsf(v0.x - v1.GetImaginary()[0]) < epsilon);
    CHECK(fabsf(v0.y - v1.GetImaginary()[1]) < epsilon);
    CHECK(fabsf(v0.z - v1.GetImaginary()[2]) < epsilon);
    CHECK(fabsf(v0.w - v1.GetReal()) < epsilon);
}

inline void compare(const carb::Float4& v0, const PXR_NS::GfQuatf& v1, float epsilon)
{
    CHECK(fabsf(v0.x - v1.GetImaginary()[0]) < epsilon);
    CHECK(fabsf(v0.y - v1.GetImaginary()[1]) < epsilon);
    CHECK(fabsf(v0.z - v1.GetImaginary()[2]) < epsilon);
    CHECK(fabsf(v0.w - v1.GetReal()) < epsilon);
}

inline void compare(const PXR_NS::GfQuatf& v0, const PXR_NS::GfQuatf& v1, float epsilon)
{
    CHECK(fabsf(v0.GetImaginary()[0] - v1.GetImaginary()[0]) < epsilon);
    CHECK(fabsf(v0.GetImaginary()[1] - v1.GetImaginary()[1]) < epsilon);
    CHECK(fabsf(v0.GetImaginary()[2] - v1.GetImaginary()[2]) < epsilon);
    CHECK(fabsf(v0.GetReal() - v1.GetReal()) < epsilon);
}

inline void compare(const carb::Float4& v0, const carb::Float4& v1, float epsilon)
{
	CHECK(fabsf(v0.x - v1.x) < epsilon);
    CHECK(fabsf(v0.y - v1.y) < epsilon);
    CHECK(fabsf(v0.z - v1.z) < epsilon);
    CHECK(fabsf(v0.w - v1.w) < epsilon);
}

inline ::physx::PxQuat toPhysX(const PXR_NS::GfQuath& v)
{
    return ::physx::PxQuat(float(v.GetImaginary()[0]), float(v.GetImaginary()[1]), float(v.GetImaginary()[2]), float(v.GetReal()));
}


inline float degToRad(const float a)
{
    return 0.01745329251994329547f * a;
}

inline float radToDeg(const float a)
{
    return 57.29577951308232286465f * a;
}

inline PXR_NS::GfVec3f degToRad(const PXR_NS::GfVec3f& a)
{
    return PXR_NS::GfVec3f(0.01745329251994329547f * a);
}

inline PXR_NS::GfVec3f radToDeg(const PXR_NS::GfVec3f& a)
{
    return PXR_NS::GfVec3f(57.29577951308232286465f * a);
}

namespace PXR_INTERNAL_NS
{
std::ostream& operator<<(std::ostream& os, const UsdStageRefPtr& value);
} // namespace PXR_INTERNAL_NS

namespace physx
{
template<typename E, typename T>
std::ostream& operator<<(std::ostream& os, const physx::PxFlags<E, T>& value)
{
    os << static_cast<T>(value);
    return os;
}
}

PXR_NS::UsdGeomMesh createConcaveMesh(PXR_NS::UsdStageWeakPtr stage, PXR_NS::SdfPath path, float halfSize, float ZOffset = 0);
