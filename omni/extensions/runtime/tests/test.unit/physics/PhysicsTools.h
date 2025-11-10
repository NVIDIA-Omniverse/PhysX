// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

class ScopedPopulationActivation
{
public:
    ScopedPopulationActivation();
    ~ScopedPopulationActivation();

    ScopedPopulationActivation(const ScopedPopulationActivation&) = delete;
    void operator=(const ScopedPopulationActivation&) = delete;
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

    omni::ext::ExtensionManager* getExtensionManager()
    {
        return mManager;
    }

private:
    PhysicsTest();

    ~PhysicsTest();

    omni::ext::ExtensionManager*                    mManager;
    carb::AppScoped*                                mApp;
    ErrorEventListener                              mErrorListener;
    carb::events::IEventStreamPtr                   mEventStreamPtr;
    carb::events::ISubscriptionPtr                  mSubscriptionPtr;
};

inline pxr::GfVec3f getPhysicsPrimPos(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& path)
{
    pxr::GfVec3f pos(0.0f);
    pxr::UsdPrim prim = stage->GetPrimAtPath(path);
    if (prim)
    {
        static pxr::TfToken translateToken("xformOp:translate");
        pxr::UsdAttribute attr = prim.GetAttribute(translateToken);
        if (attr)
        {
            // fallback to check doubles eventually
            if (!attr.Get(&pos))
            {
                pxr::GfVec3d posd(0.0);
                if (attr.Get(&posd))
                {
                    pos = pxr::GfVec3f(posd);
                }
            }
        }
    }
    return pos;
}

inline pxr::GfQuatf getPhysicsPrimQuat(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& path)
{
    pxr::GfQuatf quat(1.0f);
    pxr::UsdPrim prim = stage->GetPrimAtPath(path);
    if(prim)
    {
        static pxr::TfToken orientToken("xformOp:orient");
        pxr::UsdAttribute attr = prim.GetAttribute(orientToken);
        if(attr)
        {
            // fallback to check doubles eventually
            if(!attr.Get(&quat))
            {
                pxr::GfQuatd quatD(0.0);
                if(attr.Get(&quatD))
                {
                    quat = pxr::GfQuatf(quatD);
                }
            }
        }
    }
    return quat;
}

inline void setPhysicsPrimPos(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& path, const pxr::GfVec3f& pos)
{
    pxr::UsdPrim prim = stage->GetPrimAtPath(path);
    if (prim)
    {
        static pxr::TfToken translateToken("xformOp:translate");
        pxr::UsdAttribute attr = prim.GetAttribute(translateToken);
        if (attr)
        {
            // fallback to check doubles eventually
            if (!attr.Set(pos))
            {
                pxr::GfVec3d posd(pos);
                attr.Set(posd);
            }
        }
    }    
}

template<class T>
inline T* getPhysxBaseDerivedFromPathChecked(const pxr::SdfPath& path, const omni::physx::PhysXType type)
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
inline T* getPhysxPtrFromPathUnchecked(const pxr::SdfPath& path, const omni::physx::PhysXType type)
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysx* physxInterface = physicsTests.acquirePhysxInterface();
    ::physx::PxBase* pxBase = reinterpret_cast<::physx::PxBase*>(physxInterface->getPhysXPtr(path, type));
    return reinterpret_cast<T*>(pxBase);
}

inline physx::PxScene* getPhysxSceneAtPathChecked(const pxr::SdfPath& scenePath)
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

inline void compare(const ::physx::PxVec3& v0, const pxr::GfVec3f& v1, float epsilon)
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

inline void compare(const carb::Float3& v0, const pxr::GfVec3f& v1, float epsilon)
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

inline void compare(const ::physx::PxQuat& v0, const pxr::GfQuatf& v1, float epsilon)
{
    CHECK(fabsf(v0.x - v1.GetImaginary()[0]) < epsilon);
    CHECK(fabsf(v0.y - v1.GetImaginary()[1]) < epsilon);
    CHECK(fabsf(v0.z - v1.GetImaginary()[2]) < epsilon);
    CHECK(fabsf(v0.w - v1.GetReal()) < epsilon);
}

inline void compare(const carb::Float4& v0, const pxr::GfQuatf& v1, float epsilon)
{
    CHECK(fabsf(v0.x - v1.GetImaginary()[0]) < epsilon);
    CHECK(fabsf(v0.y - v1.GetImaginary()[1]) < epsilon);
    CHECK(fabsf(v0.z - v1.GetImaginary()[2]) < epsilon);
    CHECK(fabsf(v0.w - v1.GetReal()) < epsilon);
}

inline void compare(const pxr::GfQuatf& v0, const pxr::GfQuatf& v1, float epsilon)
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

inline ::physx::PxQuat toPhysX(const pxr::GfQuath& v)
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

inline pxr::GfVec3f degToRad(const pxr::GfVec3f& a)
{
    return pxr::GfVec3f(0.01745329251994329547f * a);
}

inline pxr::GfVec3f radToDeg(const pxr::GfVec3f& a)
{
    return pxr::GfVec3f(57.29577951308232286465f * a);
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

pxr::UsdGeomMesh createConcaveMesh(pxr::UsdStageWeakPtr stage, pxr::SdfPath path, float halfSize, float ZOffset = 0);
