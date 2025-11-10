// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on

#define CARB_EXPORTS

#include <omni/convexdecomposition/ConvexDecomposition.h>

#include <carb/PluginUtils.h>
#include <carb/PluginUtils.h>
#include <carb/extras/Path.h>
#include <carb/settings/ISettings.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include "vcd.h"
#include "FM.h"
#include "ScopedTime.h"
#include "TriangulateUsdMeshPrim.h"
#define ENABLE_SPHERE_APPROX_IMPLEMENTATION 1
#include "SphereApprox.h"

#if CARB_COMPILER_MSC
#    pragma warning(disable : 4996)
#endif

using namespace carb::tasking;

// Scoped mutex lock
using lock_guard = std::lock_guard<MutexWrapper>;


#include <unordered_map>

const struct carb::PluginImplDesc kPluginImpl = { "omni.convexdecomposition.plugin", "V-HACD Convex Decomposition ",
                                                  "NVIDIA", carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, omni::convexdecomposition::ConvexDecomposition)
CARB_PLUGIN_IMPL_DEPS(carb::settings::ISettings,carb::tasking::ITasking)

namespace omni
{
namespace convexdecomposition
{

class VHACDInstance : public vcd::NotifyVoxelizedConvexDecomposition, public sphereapprox::NotifySphereApproximation
{
public:
    VHACDInstance(VHACDHANDLE handle) : mHandle(handle)
    {
    }

    virtual ~VHACDInstance(void)
    {
        releaseVHACD();
    }

    void releaseVHACD(void)
    {
        releaseTriMesh();
        if ( mSphereApprox )
        {
            if ( !mSphereApprox->isReady() )
            {
                mSphereApprox->cancel();
                while (!mSphereApprox->isReady())
                {
                    // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    mSphereApprox->wait(5);
                }
            }
            mSphereApprox->release();
            mSphereApprox = nullptr;
        }
        if (mVHACD)
        {
            if (!mVHACD->isFinished())
            {
                mVHACD->cancel();
                while (!mVHACD->isFinished())
                {
                    // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    mVHACD->wait(5);
                }
            }
            mVHACD->release();
            mVHACD = nullptr;
        }
    }

    void validateParameters(sphereapprox::SphereApprox::Parameters &params)
    {
        if ( params.mVoxelResolution < 10000 || params.mVoxelResolution > 10000000 )
        {
            CARB_LOG_WARN("Invalid voxel resolution(%d) must be between 10,000 and 10,000,000\n", params.mVoxelResolution);
            if ( params.mVoxelResolution < 10000 )
            {
                params.mVoxelResolution = 10000;
            }
            else
            {
                params.mVoxelResolution = 10000000;
            }
        }
        if (params.mErrorMetric < 0.01 || params.mErrorMetric > 25.0)
        {
            CARB_LOG_WARN("Invalid volume error metric(%0.2f) must be between 0.01 and 25\n", params.mErrorMetric);
            if (params.mErrorMetric < 0.01)
            {
                params.mErrorMetric = 0.01;
            }
            else
            {
                params.mErrorMetric = 25;
            }
        }
        if ( params.mMaxSpheres < 1 || params.mMaxSpheres > 100000 )
        {
            CARB_LOG_WARN("Invalid maximum spheres count(%d) must be between 1 and 100,000\n", params.mMaxSpheres);
            if ( params.mMaxSpheres < 1 )
            {
                params.mMaxSpheres = 1;
            }
            else
            {
                params.mMaxSpheres = 100000;
            }
        }
        if ( params.mMaxSeedCount < 8 || params.mMaxSeedCount > 100000 )
        {
            CARB_LOG_WARN("Invalid maximum seed points count(%d) must be between 1 and 100,000\n", params.mMaxSeedCount);
            if ( params.mMaxSeedCount < 8 )
            {
                params.mMaxSeedCount = 8;
            }
            else
            {
                params.mMaxSeedCount = 100000;
            }
        }
    }

    void validateParameters(vcd::Params &params)
    {
        if ( params.mVoxelResolution < 10000 || params.mVoxelResolution > 10000000 )
        {
            CARB_LOG_WARN("Invalid voxel resolution(%d) must be between 10,000 and 10,000,000\n", params.mVoxelResolution);
            if ( params.mVoxelResolution < 10000 )
            {
                params.mVoxelResolution = 10000;
            }
            else
            {
                params.mVoxelResolution = 10000000;
            }
        }
        if ( params.mMaxHullVertices < 8 || params.mMaxHullVertices > 64 )
        {
            CARB_LOG_WARN("Invalid max hull vertices(%d) must be between 8 and 64\n", params.mMaxHullVertices);
            if ( params.mMaxHullVertices < 8 )
            {
                params.mMaxHullVertices = 8;
            }
            else
            {
                params.mMaxHullVertices = 64;
            }
        }
        if ( params.mMaxConvexHulls < 1 || params.mMaxConvexHulls > 2048 )
        {
            CARB_LOG_WARN("Invalid max convex hulls(%d) must be between 1 and 2,048\n", params.mMaxConvexHulls);
            if (params.mMaxConvexHulls < 1)
            {
                params.mMaxConvexHulls = 1;
            }
            else
            {
                params.mMaxConvexHulls = 2048;
            }
        }
        if (params.mMaxDepth < 1 || params.mMaxDepth > 32)
        {
            CARB_LOG_WARN("Invalid max recursion depth(%d) must be between 1 and 32\n", params.mMaxDepth);
            if (params.mMaxDepth < 1)
            {
                params.mMaxDepth = 1;
            }
            else
            {
                params.mMaxDepth = 32;
            }
        }
        if (params.mMinVoxelSize < 1 || params.mMinVoxelSize > 8)
        {
            CARB_LOG_WARN("Invalid min voxel size(%d) must be between 1 and 8\n", params.mMinVoxelSize);
            if (params.mMinVoxelSize < 1)
            {
                params.mMinVoxelSize = 1;
            }
            else
            {
                params.mMinVoxelSize = 8;
            }
        }
        if (params.mErrorPercentage < 0.01 || params.mErrorPercentage > 25.0 )
        {
            CARB_LOG_WARN("Invalid volume error percentage(%0.2f) must be between 0.01 and 25\n", params.mErrorPercentage);
            if (params.mErrorPercentage < 0.01)
            {
                params.mErrorPercentage = 0.01;
            }
            else
            {
                params.mErrorPercentage = 25;
            }
        }
    }

    bool beginVHACD(const Parameters& p, const SimpleMesh& sourceMesh)
    {
        bool ret = false;

        if (mVHACD == nullptr)
        {
            mVHACD = vcd::VoxelizedConvexDecomposition::create(true);
            if (mVHACD)
            {
                mParameters = p;
                vcd::Params params;
                params.mMaxConvexHulls = p.maxConvexHullCount;
                params.mMaxHullVertices = p.maxHullVertices;
                params.mVoxelResolution = p.voxelResolution;
                params.mFillMode = vcd::VoxelFillMode(p.voxelFillMode);
                params.mTriangleCount = sourceMesh.triangleCount;
                params.mVertexCount = sourceMesh.vertexCount;
                params.mIndices = sourceMesh.indices;
                params.mVertices = sourceMesh.vertices;
                params.mShrinkWrap = p.shrinkWrap;
                params.mErrorPercentage = p.errorPercentage;
                validateParameters(params);
                params.mCallback = this;
                {
                    mVHACD->process(params);
                }
                ret = true;
            }
        }

        return ret;
    }

    bool isComplete(void)
    {
        bool ret = true;

        if ( mSphereApprox )
        {
            ret = mSphereApprox->isReady();
        }
        if (mVHACD)
        {
            ret = mVHACD->isFinished();
        }

        return ret;
    }

    uint32_t getConvexHullCount(void)
    {
        uint32_t ret = 0;

        if (mVHACD)
        {
            ret = mVHACD->getMergedConvexHullCount();
        }

        return ret;
    }

    bool getConvexHull(uint32_t hullIndex, SimpleMesh& hullResults)
    {
        bool ret = false;

        if (mVHACD && hullIndex < mVHACD->getMergedConvexHullCount())
        {
            const vcd::SimpleMesh *sm = mVHACD->getMergedConvexHull(hullIndex);
            const vcd::SimpleMesh *rm = mVHACD->getRootConvexHull();
            if ( sm && rm )
            {
                ret = true;
                hullResults.vertices = sm->mVertices;
                hullResults.vertexCount = sm->mVertexCount;
                hullResults.indices = sm->mIndices;
                hullResults.triangleCount = sm->mTriangleCount;
                hullResults.center.x = sm->mCenter[0];
                hullResults.center.y = sm->mCenter[1];
                hullResults.center.z = sm->mCenter[2];

                // Total number of polygons
                hullResults.polygonCount = sm->mPolygonCount;
                // Total number of polygon indices
                hullResults.polygonIndexCount = sm->mPolygonIndexCount;
                // Polygon data
                hullResults.polygons = (const PolygonData *)sm->mPolygonData;
                // Polygon indices
                hullResults.polygonIndices = sm->mPolygonIndices;

                hullResults.rootCenter.x = rm->mCenter[0];
                hullResults.rootCenter.y = rm->mCenter[1];
                hullResults.rootCenter.z = rm->mCenter[2];

                hullResults.volume = sm->mVolume;

            }
        }

        return ret;
    }

    bool cancel(void)
    {
        bool ret = true;

        if ( mSphereApprox )
        {
            mSphereApprox->cancel();
        }
        if (mVHACD)
        {
            mVHACD->cancel(); // cancel operation in progress.
        }

        return ret;
    }

    virtual void notifySphereApproximationComplete(void) final
    {
        if (mParameters.notifyCompleteCallback)
        {
            mParameters.notifyCompleteCallback(mHandle, mParameters.notifyCompleteCallbackUserPtr);
        }
    }


    // This is an optional user callback which is only called when running V-HACD asynchronously.
    // This is a callback performed to notify the user that the
    // convex decomposition background process is completed. This callback will occur from
    // a different thread so the user should take that into account.
    virtual void notifyVoxelizedConvexDecompositionComplete(void) final
    {
        if (mParameters.notifyCompleteCallback)
        {
            mParameters.notifyCompleteCallback(mHandle, mParameters.notifyCompleteCallbackUserPtr);
        }
    }


    // Profiling callbacks, not yet hooked up to Carbonite profiling system
    virtual uint32_t AllocTagId(const char* const tagName) final
    {
        uint32_t ret = 0;

        return ret;
    }

    virtual void EnterTag(uint32_t tagId) final
    {
    }

    virtual void ExitTag() final
    {
    }

    virtual void Bookmark(const char* const text) final
    {
    }

     /**
     * Run the convex decomposition synchronously
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param p  : A pointer to the parameters to use for this operation.
     * @param sourceMesh : A pointer to the source raw triangle mesh to perform the operation on.
     * @return : Returns the number of convex hulls produced.
     */
    uint32_t computeVHACD(const Parameters& p, const SimpleMesh& sourceMesh)
    {
        uint32_t ret = 0;

        if (mVHACD == nullptr)
        {
            mVHACD = vcd::VoxelizedConvexDecomposition::create(true);
        }
        if (mVHACD)
        {
            mParameters = p;
            vcd::Params params;
            params.mMaxConvexHulls = p.maxConvexHullCount;
            params.mMaxHullVertices = p.maxHullVertices;
            params.mVoxelResolution = p.voxelResolution;
            params.mFillMode = vcd::VoxelFillMode(p.voxelFillMode);
            params.mTriangleCount = sourceMesh.triangleCount;
            params.mVertexCount = sourceMesh.vertexCount;
            params.mIndices = sourceMesh.indices;
            params.mVertices = sourceMesh.vertices;
            params.mErrorPercentage = p.errorPercentage;
            params.mShrinkWrap = p.shrinkWrap;
            validateParameters(params);
            params.mCallback = this;
            {
                mVHACD->process(params);
                while ( !mVHACD->isFinished() )
                {
                    // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    mVHACD->wait(5);
                }
            }
            ret = mVHACD->getMergedConvexHullCount();
        }

        return ret;
    }

    void releaseTriMesh(void)
    {
        mTriMeshVertexCount = 0;
        mTriMeshIndices.clear();
        free(mTriMeshVertices);
        mTriMeshVertices = nullptr;
    }

    bool beginSphereApproximation(const Parameters& p, const SimpleMesh& sourceMesh)
    {
        bool ret = false;
        if (mSphereApprox == nullptr)
        {
            mSphereApprox = sphereapprox::SphereApprox::create();
            if (mSphereApprox)
            {
                mParameters = p;
                sphereapprox::SphereApprox::Parameters params;
                params.mVoxelResolution = p.voxelResolution;
                params.mFillMode = sphereapprox::FillMode(p.voxelFillMode);
                params.mErrorMetric = p.errorPercentage;
                params.mAsync = true;
                params.mMaxSpheres = p.maxSpheres;
                params.mMaxSeedCount = p.maxSeedCount;
                params.mNotifyCallback = this;
                validateParameters(params);
                ret = mSphereApprox->compute(sourceMesh.vertices,sourceMesh.vertexCount,sourceMesh.indices,sourceMesh.triangleCount,params);
            }
        }
        return ret;
    }

    const SimpleSphere *getSphereApproximation(uint32_t &sphereCount, bool reducedSpheres)
    {
        const SimpleSphere *ret = nullptr;

        if ( mSphereApprox && mSphereApprox->isReady() )
        {
            const auto sp = mSphereApprox->getSpheres(sphereCount,reducedSpheres);
            if ( sphereCount )
            {
                mSpheres.resize(sphereCount);
                for (uint32_t i=0; i<sphereCount; i++)
                {
                    SimpleSphere &ss = mSpheres[i];
                    ss.center.x = sp[i].mCenter[0];
                    ss.center.y = sp[i].mCenter[1];
                    ss.center.z = sp[i].mCenter[2];
                    ss.radius = sp[i].mRadius;
                }
                ret = &mSpheres[0];
            }
        }

        return ret;
    }

    const SimpleSphere * computeSphereApproximation(const Parameters& p, const SimpleMesh& sourceMesh, uint32_t &sphereCount, bool reducedResults)
    {
        const SimpleSphere *ret = nullptr;

        if ( beginSphereApproximation(p,sourceMesh))
        {
            while ( !mSphereApprox->isReady() )
            {
                // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                mSphereApprox->wait(5);
            }
            ret = getSphereApproximation(sphereCount,reducedResults);
        }

        return ret;
    }

    VHACDHANDLE mHandle{ 0 };
    Parameters mParameters;
    vcd::VoxelizedConvexDecomposition *mVHACD{nullptr};
    sphereapprox::SphereApprox          *mSphereApprox{nullptr};
    uint32_t                    mTriMeshVertexCount{0};
    double                      *mTriMeshVertices{nullptr};
    std::vector< uint32_t >     mTriMeshIndices;
    std::vector< SimpleSphere > mSpheres;
};



typedef std::unordered_map<VHACDHANDLE, VHACDInstance*> VHACDInstanceMap;

class VHACDFactory
{
public:
    VHACDFactory(void)
    {
    }

    virtual ~VHACDFactory(void)
    {
        for (auto& i : mInstances)
        {
            i.second->cancel();
            delete i.second;
        }
    }

    /**
     * Creates an instance of the V-HACD system.
     *
     * @return the unique handle/id for this instance.
     */
    VHACDHANDLE createVHACD(void)
    {
        VHACDInstance* inst = new VHACDInstance(mId);
        VHACDHANDLE ret = addVHACDInstance(inst);
        return ret;
    }

    /**
     * Releases a previously created V-HACD instance. Returns true if the provided handle
     * was valid and/or the instance is not currently running a background task.
     *
     * @param id The handle of a previously allocated V-HACD instance
     * @return Returns true if the V-HACD instance was safely released.
     */
    bool releaseVHACD(VHACDHANDLE id)
    {
        bool ret = false;

        ret = removeVHACDInstance(id);

        return ret;
    }

    /**
     * Begins the convex decomposition process.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param p  : A pointer to the parameters to use for this operation.
     * @param sourceMesh : A pointer to the source raw triangle mesh to perform the operation on.
     * @return : Returns true if the input parameters were valid and we have started the operation.
     */
    bool beginVHACD(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh)
    {
        bool ret = false;

        auto i = getInstance(id);
        if (i)
        {
            ret = i->beginVHACD(p, sourceMesh);
        }

        return ret;
    }

     /**
     * Run the convex decomposition synchronously
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param p  : A pointer to the parameters to use for this operation.
     * @param sourceMesh : A pointer to the source raw triangle mesh to perform the operation on.
     * @return : Returns the number of convex hulls produced.
     */
    uint32_t computeVHACD(VHACDHANDLE id,const Parameters& p, const SimpleMesh& sourceMesh)
    {
        uint32_t ret = 0;

        auto i = getInstance(id);
        if (i)
        {
            ret = i->computeVHACD(p,sourceMesh);
        }

        return ret;
    }

    /**
     * Returns true if the convex decomposition operation is completed.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * *return : Returns true if the convex decomposition operation is not currently running.
     */
    bool isComplete(VHACDHANDLE id)
    {
        bool ret = false;

        auto i = getInstance(id);
        if (i)
        {
            ret = i->isComplete();
        }

        return ret;
    }

    /**
     * Returns the number of convex hulls generated by the convex decomposition process.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @return : Returns the number of convex hulls generated
     */
    uint32_t getConvexHullCount(VHACDHANDLE id)
    {
        uint32_t ret = 0;

        auto i = getInstance(id);
        if (i)
        {
            ret = i->getConvexHullCount();
        }

        return ret;
    }

    /**
     * Retrieves the triangle mesh corresponding to the convex hull results.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param hullIndex : Which convex hull we are asking for.
     * @param hullResults : A pointer to a struct which will reflect the convex hull mesh data
     * @return : Returns true if the convex hull was successfully retrieved, false if the handle or index was invalid.
     */
    bool getConvexHull(VHACDHANDLE id, uint32_t hullIndex, SimpleMesh& hullResults)
    {
        bool ret = false;

        auto i = getInstance(id);
        if (i)
        {
            ret = i->getConvexHull(hullIndex, hullResults);
        }

        return ret;
    }

    /**
     * Cancels any convex decomposition that is in process.
     *
     * @return : Returns true if the handle was valid and the operation was successfully canceled.
     */
    bool cancelVHACD(VHACDHANDLE id)
    {
        bool ret = false;

        auto i = getInstance(id);
        if (i)
        {
            ret = i->cancel();
        }

        return ret;
    }

    bool beginSphereApproximation(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh)
    {
        bool ret = false;

        auto i = getInstance(id);
        if (i)
        {
            ret = i->beginSphereApproximation(p,sourceMesh);
        }

        return ret;
    }

    const SimpleSphere *getSphereApproximation(VHACDHANDLE id, uint32_t &sphereCount, bool reducedSpheres)
    {
        const SimpleSphere *ret = nullptr;
        auto i = getInstance(id);
        if (i)
        {
            ret = i->getSphereApproximation(sphereCount,reducedSpheres);
        }
        return ret;
    }

    const SimpleSphere * computeSphereApproximation(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh, uint32_t &sphereCount, bool reducedResults)
    {
        const SimpleSphere *ret = nullptr;
        auto i = getInstance(id);
        if (i)
        {
            ret = i->computeSphereApproximation(p,sourceMesh,sphereCount,reducedResults);
        }
        return ret;
    }

    VHACDHANDLE addVHACDInstance(VHACDInstance *i)
    {
        lock_guard _lock(mVHACDInstanceMutex);
        mId++;
        mInstances[mId] = i;
        return mId;
    }

    bool removeVHACDInstance(VHACDHANDLE h)
    {
        bool ret = false;

        VHACDInstance *inst = nullptr;
        {
            lock_guard _lock(mVHACDInstanceMutex);
            VHACDInstanceMap::iterator found = mInstances.find(h);
            if ( found != mInstances.end() )
            {
                inst = (*found).second;
                mInstances.erase(found);
            }
        }
        if ( inst )
        {
            ret = true;
            inst->cancel();
            delete inst;
        }
        return ret;
    }

    VHACDInstance *getInstance(VHACDHANDLE h)
    {
        VHACDInstance *inst = nullptr;
        lock_guard _lock(mVHACDInstanceMutex);
        VHACDInstanceMap::iterator found = mInstances.find(h);
        if (found != mInstances.end())
        {
            inst = (*found).second;
        }
        return inst;
    }

    MutexWrapper mVHACDInstanceMutex;
    std::atomic< VHACDHANDLE> mId{ 0 };
    VHACDInstanceMap mInstances;

};

VHACDFactory* gVHACDFactory{ nullptr };

/**
 * Creates an instance of the V-HACD system.
 *
 * @return the unique handle/id for this instance.
 */
VHACDHANDLE createVHACD(void)
{
    return gVHACDFactory->createVHACD();
}

/**
 * Releases a previously created V-HACD instance. Returns true if the provided handle
 * was valid and/or the instance is not currently running a background task.
 *
 * @param id The handle of a previously allocated V-HACD instance
 * @return Returns true if the V-HACD instance was safely released.
 */
bool releaseVHACD(VHACDHANDLE id)
{
    return gVHACDFactory->releaseVHACD(id);
}

/**
 * Begins the convex decomposition process.
 *
 * @param id : The handle of a previously allocated V-HACD instance
 * @param p  : A pointer to the parameters to use for this operation.
 * @param sourceMesh : A pointer to the source raw triangle mesh to perform the operation on.
 * @return : Returns true if the input parameters were valid and we have started the operation.
 */
bool beginVHACD(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh)
{
    return gVHACDFactory->beginVHACD(id, p, sourceMesh);
}

bool beginSphereApproximation(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh)
{
    return gVHACDFactory->beginSphereApproximation(id,p,sourceMesh);
}

/**
 * Returns true if the convex decomposition operation is completed.
 *
 * @param id : The handle of a previously allocated V-HACD instance
 * *return : Returns true if the convex decomposition operation is not currently running.
 */
bool isComplete(VHACDHANDLE id)
{
    return gVHACDFactory->isComplete(id);
}

/**
 * Returns the number of convex hulls generated by the convex decomposition process.
 *
 * @param id : The handle of a previously allocated V-HACD instance
 * @return : Returns the number of convex hulls generated
 */
uint32_t getConvexHullCount(VHACDHANDLE id)
{
    return gVHACDFactory->getConvexHullCount(id);
}

/**
 * Retrieves the triangle mesh corresponding to the convex hull results.
 *
 * @param id : The handle of a previously allocated V-HACD instance
 * @param hullIndex : Which convex hull we are asking for.
 * @param hullResults : A pointer to a struct which will reflect the convex hull mesh data
 * @return : Returns true if the convex hull was successfully retrieved, false if the handle or index was invalid.
 */
bool getConvexHull(VHACDHANDLE id, uint32_t hullIndex, SimpleMesh& hullResults)
{
    return gVHACDFactory->getConvexHull(id, hullIndex, hullResults);
}

const SimpleSphere *getSphereApproximation(VHACDHANDLE id,uint32_t &sphereCount,bool reducedSpheres)
{
    return gVHACDFactory->getSphereApproximation(id,sphereCount,reducedSpheres);
}
/**
 * Cancels any convex decomposition that is in process.
 *
 * @return : Returns true if the handle was valid and the operation was successfully canceled.
 */
bool cancelVHACD(VHACDHANDLE id)
{
    return gVHACDFactory->cancelVHACD(id);
}

/**
 * A method to save a 'SimpleMesh' as a wavefront OBJ file on disk. Primarily used for debugging inputs and outputs.
 *
 * @param fname : Name of file on disk to save the mesh to
 * @param sm : Pointer to a simple triangle mesh
 * @return :Returns true if the wavefront OBJ file was successfully saved, false if it was not.
 */
bool saveOBJ(const char* fname, const SimpleMesh& sm, bool flipWindingOrder)
{
    bool ret = false;

    if (fname && sm.vertexCount && sm.triangleCount && sm.vertices && sm.indices)
    {
        FILE* fph = fopen(fname, "wb");
        if (fph)
        {
            fprintf(fph, "# SimpleMesh:%s saved by the ConvexDecomposition Kit Extension.\n", fname);
            for (uint32_t i = 0; i < sm.vertexCount; i++)
            {
                const double* p = &sm.vertices[i * 3];
                fprintf(fph, "v %f %f %f\n", p[0], p[1], p[2]);
            }
            for (uint32_t i = 0; i < sm.triangleCount; i++)
            {
                const uint32_t* tri = &sm.indices[i * 3];
                uint32_t i1 = tri[0] + 1;
                uint32_t i2 = tri[1] + 1;
                uint32_t i3 = tri[2] + 1;
                if (flipWindingOrder)
                {
                    fprintf(fph, "f %d %d %d\n", i3, i2, i1);
                }
                else
                {
                    fprintf(fph, "f %d %d %d\n", i1, i2, i3);
                }
            }
            fclose(fph);
            ret = true;
        }
    }

    return ret;
}

/**
    * Run the convex decomposition synchronously
    *
    * @param id : The handle of a previously allocated V-HACD instance
    * @param p  : A pointer to the parameters to use for this operation.
    * @param sourceMesh : A pointer to the source raw triangle mesh to perform the operation on.
    * @return : Returns the number of convex hulls produced.
    */
uint32_t computeVHACD(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh)
{
    return gVHACDFactory->computeVHACD(id,p,sourceMesh);
}

const SimpleSphere * computeSphereApproximation(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh,uint32_t &sphereCount,bool reducedResults)
{
    return gVHACDFactory->computeSphereApproximation(id,p,sourceMesh,sphereCount,reducedResults);
}

bool applySphereApproximation(const char *primPath,uint32_t stageId)
{
    bool ret = false;

    pxr::UsdStageRefPtr currentStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    if ( currentStage )
    {
        auto prim = currentStage->GetPrimAtPath(pxr::SdfPath(primPath));
        if ( prim )
        {
            triangulateusd::TriangulateUSDPrim *tprim = triangulateusd::TriangulateUSDPrim::create(prim);
            if ( tprim )
            {
                uint32_t tcount = tprim->triangulate();
                if ( tcount )
                {
                    sphereapprox::SphereApprox *sa = sphereapprox::SphereApprox::create();
                    sphereapprox::SphereApprox::Parameters p;
                    uint32_t vertexCount;
                    const float *vertices = tprim->getVertices(vertexCount);
                    const uint32_t *indices = tprim->getIndices(tcount);
                    sa->compute(vertices,vertexCount,indices,tcount,p);
                    while ( !sa->isReady() )
                    {
                        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        sa->wait(5);
                    }
                    uint32_t sphereCount;
                    const sphereapprox::SphereApprox::Sphere *spheres = sa->getSpheres(sphereCount,true);
                    std::string apath = std::string(primPath) + "/sphereapprox";
                    auto aprim = pxr::UsdGeomPoints::Define(currentStage,pxr::SdfPath(apath));
                    pxr::VtArray<pxr::GfVec3f> spoints;
                    pxr::VtArray<float> widths;
                    for (uint32_t i=0; i<sphereCount; i++)
                    {
                        const auto &s = spheres[i];
                        pxr::GfVec3f p;
                        p[0] = (float)s.mCenter[0];
                        p[1] = (float)s.mCenter[1];
                        p[2] = (float)s.mCenter[2];
                        spoints.push_back(p);
                        widths.push_back(float(s.mRadius*2));
                    }

                    aprim.GetPointsAttr().Set(spoints);
                    aprim.GetWidthsAttr().Set(widths);
                    auto sprim = currentStage->GetPrimAtPath(pxr::SdfPath(apath));
                    pxr::UsdPhysicsCollisionAPI::Apply(sprim);
                    pxr::UsdPhysicsRigidBodyAPI::Apply(prim);
                    sa->release();

                    ret = true;
                }
                tprim->release();
            }
        }
    }
    return ret;
}


} // namespace convexdecomposition
} // namespace omni

CARB_EXPORT void carbOnPluginStartup()
{
    omni::convexdecomposition::gVHACDFactory = new omni::convexdecomposition::VHACDFactory;
}

CARB_EXPORT void carbOnPluginShutdown()
{
    delete omni::convexdecomposition::gVHACDFactory;
}

void fillInterface(omni::convexdecomposition::ConvexDecomposition& iface)
{
    using namespace omni::convexdecomposition;

    // clang-format off
    iface =
    {
    createVHACD,
    releaseVHACD,
    beginVHACD,
    beginSphereApproximation,
    isComplete,
    getSphereApproximation,
    getConvexHullCount,
    getConvexHull,
    cancelVHACD,
    saveOBJ,
    computeVHACD,
    computeSphereApproximation,
    applySphereApproximation
    };
    // clang-format on
}
