// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/tasking/TaskingUtils.h>

#include "MergeConvexHulls.h"
#include "SimpleMesh.h"
#include "QuickHull.h"
#include "RaycastMesh.h"
#include "FM.h"
#include "ScopedTime.h"
#include "JobSystem.h"
#include "Quantizer.h"
#include "ShrinkWrap.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <assert.h>
#include <mutex>
#include <float.h>
#include <math.h>

#ifdef _MSC_VER
#pragma warning(disable:4100 4456)
#endif

#define SHOW_TIME 0

using namespace carb::tasking;

// Scoped mutex lock
using lock_guard = std::lock_guard<MutexWrapper>;

namespace vcd
{

class MergeConvexHullsImpl;

class MergeTask
{
public:
    MergeConvexHullsImpl *mPthis;
    SimpleMesh  *mHullA;
    SimpleMesh  *mHullB;
};

void mergeTask(void *ptr);

class HullPair
{
public:
    HullPair(void) { };
    HullPair(uint32_t hullA,uint32_t hullB,double concavity) : mConcavity(concavity), mHullA(hullA), mHullB(hullB)
    {
    }

    bool operator<(const HullPair &h) const
    {
        return mConcavity > h.mConcavity ? true : false;
    }

    double      mConcavity{0};
    uint32_t    mHullA{0};
    uint32_t    mHullB{0};
};

using HullPairQueue = std::priority_queue< HullPair >;
using HullMap = std::unordered_map< uint32_t, SimpleMesh * >;

class Vec3
{
public:
    Vec3(void)
    {
    }
    Vec3(double _x,double _y,double _z) : x(_x), y(_y), z(_z)
    {
    }
    Vec3(const double *v)
    {
        x = v[0];
        y = v[1];
        z = v[2];
    }

    // The multiply operator appears to be a dot-product
    inline double operator*(const Vec3& rhs) const
    {
        return (x * rhs.x + y * rhs.y + z * rhs.z);
    }

    inline Vec3 operator-(const Vec3& rhs) const
    {
        return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    inline Vec3 operator+(const Vec3& rhs) const
    {
        return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    inline void operator+=(const Vec3& rhs) 
    {
        x+=rhs.x;
        y+=rhs.y;
        z+=rhs.z;
    }

    inline void operator*=(const double v)
    {
        x = x*v;
        y = y*v;
        z = z*v;
    }

    inline void operator/=(const double v)
    {
        x = x / v;
        y = y / v;
        z = z / v;
    }

    inline Vec3 operator^(const Vec3& rhs) const
    {
        return Vec3(y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x);
    }

    double x;
    double y;
    double z;
};

double computeVolume4(const Vec3 &a,const Vec3 &b,const Vec3 &c,const Vec3 &d)
{
#if 1
    Vec3 ad = a - d;
    Vec3 bd = b - d;
    Vec3 cd = c - d;
    Vec3 bcd = bd ^ cd;
    double dot = ad * bcd;
    return dot;
#else
    return (a - d) * ((b - d) ^ (c - d));
#endif
}

using SimpleMeshVector = std::vector< SimpleMesh *>;

class MergeConvexHullsImpl : public MergeConvexHulls
{
public:
    MergeConvexHullsImpl(JobSystem *jobSystem) : mJobSystem(jobSystem)
    {
    }

    virtual ~MergeConvexHullsImpl(void)
    {
        releaseMeshes();
    }

    // Deep copy this mesh
    virtual void addConvexHull(const SimpleMesh &sm) final
    {
        SimpleMesh *h = new SimpleMesh(sm);
        h->mVolume = getVolume(*h);
        mSourceConvexHulls.push_back(h);
    }

    void removeHull(uint32_t id)
    {
        HullMap::iterator found = mHulls.find(id);
        if ( found != mHulls.end() )
        {
            SimpleMesh *sm = (*found).second;
            delete sm;
            mHulls.erase(found);
        }
        else
        {
            printf("Fatal error!\n");
            exit(1);
        }
    }

    SimpleMesh *getHull(uint32_t id)
    {
        SimpleMesh *ret = nullptr;

        HullMap::iterator found = mHulls.find(id);
        if (found != mHulls.end())
        {
            ret = (*found).second;
        }

        return ret;
    }


    SimpleMesh *getExistingHull(uint32_t id)
    {
        SimpleMesh *ret = nullptr;

        HullMap::iterator found = mHulls.find(id);
        if ( found != mHulls.end() )
        {
            ret = (*found).second;
        }
        else
        {
            printf("Fatal error condition!\n");
            exit(1);
        }

        return ret;
    }

    void releaseMeshes(void)
    {
        for (auto &i : mHulls)
        {
            delete i.second;
        }
        mHulls.clear();
        for (auto &i : mSourceConvexHulls)
        {
            delete i;
        }
        mSourceConvexHulls.clear();
    }

    virtual uint32_t mergeConvexHulls(uint32_t maxHullCount,double _overallMeshVolume) final
    {
        mOverallMeshVolume = _overallMeshVolume;
#if SHOW_TIME
        double timeSetup = 0;
        double timeCostMatrix = 0;
        double timeFindingHull = 0;
        double timeCombinedHull = 0;
        double timeNewCost = 0;

        Timer timer;
        timer.reset();
#endif

        for (auto &i:mHulls)
        {
            delete i.second;
        }
        mHulls.clear();

        // Build the set of merged convex hulls, this will get modified during
        // the merging process
        for (auto &i:mSourceConvexHulls)
        {
            SimpleMesh *sm = new SimpleMesh(*i); // Deep copy the source mesh
            sm->mMeshId = mMeshId;
            mMeshId++;
            mHulls[sm->mMeshId] = sm;
        }

        // Build the initial cost matrix representing the concavity of every pair
        // of convex hulls
        size_t hullCount = mHulls.size();
        std::vector< SimpleMesh *> hulls;
        hulls.reserve(hullCount);
        for (auto &i:mHulls)
        {
            hulls.push_back(i.second);
        }
#if SHOW_TIME
        timeSetup+=timer.getElapsedSeconds();
        timer.reset();
#endif
        size_t csize = ((hullCount*hullCount)-hullCount)>>1;
        MergeTask *mtasks = new MergeTask[csize];
        MergeTask *task = mtasks;
        //printf("Computing cost matrix.\n");
        for (size_t p1=1; p1<hullCount; ++p1)
        {
            SimpleMesh *sm1 = hulls[p1]; // Get a pointer to the simple mesh representing this convex hull
            // Now iterate through all hulls less than the one we are comparing against
            for (size_t p2=0; p2<p1; ++p2)
            {
                SimpleMesh *sm2 = hulls[p2];   // Get the simple mesh of the second hull
                task->mHullA = sm1;
                task->mHullB = sm2;
                task->mPthis = this;
                if ( mJobSystem )
                {
                    mJobSystem->addJob(task,mergeTask);
                }
                task++;
            }
        }
        size_t taskCount = task - mtasks;
        if ( taskCount > csize )
        {
            printf("Fatal error!\n");
            exit(1);
        }
        if ( mJobSystem )
        {
            if (taskCount)
            {
                mJobSystem->startJobs();
                mJobSystem->waitForJobsToComplete();
            }
        }
        else
        {
            for (size_t i=0; i<taskCount; i++)
            {
                performTask(&mtasks[i]);
            }
        }
#if SHOW_TIME
        timeCostMatrix+=timer.getElapsedSeconds();
#endif
        //printf("Finished computing the cost matrix, now merging.\n");

        // Start with the hull cost size as we try to keep merging until we are below the maximum number
        // of convex hulls desired
        bool cancel = false;

        while ( !cancel )
        {
#if SHOW_TIME
            timer.reset();
#endif
            size_t costSize = mHulls.size();

            HullPair hp;
            bool foundPair = false;
            double bestCost=FLT_MAX;
            SimpleMesh *ch1 = nullptr;
            SimpleMesh *ch2 = nullptr;


            // We are looking for the lowest cost pair that is still valid
            while ( !foundPair )
            {
                if ( mHullPairQueue.empty() )
                {
                    break; // we exhausted the hull pair map, so we are done
                }
                else
                {
                    hp = mHullPairQueue.top();
                    mHullPairQueue.pop();
                    ch1 = getHull(hp.mHullA); // See if both hulls it refers to are still valid
                    ch2 = getHull(hp.mHullB);
                    if ( ch1 && ch2 )
                    {
                        foundPair = true;
                        bestCost = hp.mConcavity;
                    }
                }
            }
            if ( !foundPair )
            {
                break;
            }
            // If there is effectively no volume error, keep merging
            if ( bestCost < 0.00001 )
            {
            }
            else
            {
                // If our cost is below the maximum hull count, problem solved and we can return
                if ((costSize - 1) < maxHullCount)
                {
                    break; // our source hulls are less than the maximum hull count, so use this solution
                }
            }
#if SHOW_TIME
            timeFindingHull+=timer.getElapsedSeconds();
            timer.reset();
#endif
            SimpleMesh *sm1 = computeCombinedConvexHull(*ch1,*ch2);
            const double volume1 = getVolume(*sm1);  // compute the volume of this new combined hull
            sm1->mVolume = volume1;

            sm1->mMeshId = mMeshId;
            sm1->mVolume = volume1;
            mMeshId++;

            removeHull(ch1->mMeshId); // remove the two hulls we just merged
            removeHull(ch2->mMeshId);
#if SHOW_TIME
            timeCombinedHull+=timer.getElapsedSeconds();
            timer.reset();
#endif
            // Note, after removing these two hulls the cost matrix will still have references to them.
            // They will be cleaned up / garbage collected at they are encountered processing 

            // Calculate costs versus the new hull
            // So, for this new combined hull, we have to compute it's cost against all of the still valid existing hulls
            task = mtasks;
            for (auto &i:mHulls)
            {
                SimpleMesh *sm2 = i.second;
                task->mHullA = sm1;
                task->mHullB = sm2;
                task->mPthis = this;
                if ( mJobSystem )
                {
                    mJobSystem->addJob(task,mergeTask);
                }
                task++;
            }
            taskCount = task-mtasks;

            if (mJobSystem )
            {
                if ( taskCount )
                {
                    mJobSystem->startJobs();
                    mJobSystem->waitForJobsToComplete();
                }
            }
            else
            {
                for (size_t i=0; i<taskCount; i++)
                {
                    performTask(&mtasks[i]);
                }
            }

            mHulls[sm1->mMeshId] = sm1; // add the new convex hull
#if SHOW_TIME
            timeNewCost+=timer.getElapsedSeconds();
#endif
        }

        delete []mtasks;

#if SHOW_TIME
        printf("TimeSetup:          %0.4f\n", timeSetup);
        printf("TimeCostMatrix:     %0.4f\n", timeCostMatrix);
        printf("TimeFindingHull:    %0.4f\n", timeFindingHull);
        printf("TimeCombinedHull:   %0.4f\n", timeCombinedHull);
        printf("TimeNewCost:        %0.4f\n", timeNewCost);
#endif
        mMergedConvexHulls.clear();
        mMergedConvexHulls.reserve(mHulls.size());
        for (auto &i:mHulls)
        {
            SimpleMesh *sm = i.second;
            vcd::fm_computeCentroid(sm->mVertexCount,sm->mVertices,sm->mCenter);
            mMergedConvexHulls.push_back(sm);
        }
        //printf("Merged %d convex hulls down to %d results.\n", uint32_t(mSourceConvexHulls.size()), uint32_t(mMergedConvexHulls.size()));

        return uint32_t(mMergedConvexHulls.size());
    }

    double computeConcavity(double volumeSeparate,double volumeCombined,double volumeMesh)
    {
        return fabs(volumeSeparate - volumeCombined) / volumeMesh;
    }

    // Take the points in convex hull A and the points in convex hull B and generate
    // a new convex hull on the combined set of points.
    // Once completed, we create a SimpleMesh instance to hold the triangle mesh
    // and we compute an inflated AABB for it.
    SimpleMesh *computeCombinedConvexHull(const SimpleMesh &sm1,const SimpleMesh &sm2)
    {
        uint32_t vcount = sm1.mVertexCount+sm2.mVertexCount; // Total vertices from both hulls
        double *vertices = new double[vcount*3];  // Allocate memory for that many vertices
        memcpy(vertices,sm1.mVertices,sizeof(double)*3*sm1.mVertexCount); // Copy the vertices from the first hull
        double *df = vertices+(sm1.mVertexCount*3); // Get a pointer to where to write the vertices for the second hull.
        memcpy(df,sm2.mVertices,sizeof(double)*3*sm2.mVertexCount); // Copy the vertices from the second hull

        quickhull::QuickHull *qh = quickhull::QuickHull::create();
        quickhull::HullPoints hp;
        hp.mVertexCount = vcount;
        hp.mVertices = vertices;
        hp.mMaxQuantizeVertexCount = vcount;
        hp.mMaxHullVertices = vcount;
        qh->computeConvexHull(hp);

        uint32_t hvcount,htcount;
        const double *hvertices = qh->getVertices(hvcount);
        const uint32_t *hindices = qh->getIndices(htcount);

        SimpleMesh *ret = new SimpleMesh(hvcount, hvertices, htcount, hindices);

        fm_getAABB(hvcount,hvertices,sizeof(double)*3,ret->mBmin,ret->mBmax);
        fm_inflateMinMax(ret->mBmin,ret->mBmax,0.1);

        qh->release();


        // Return the convex hull as a simple mesh
        return ret;
    }

    virtual uint32_t getOriginalConvexHullCount(void) const final
    {
        return uint32_t(mSourceConvexHulls.size());
    }

    virtual uint32_t getMergeConvexHullCount(void) const final
    {
        return uint32_t(mMergedConvexHulls.size());
    }

    virtual const SimpleMesh *getOriginalConvexHull(uint32_t index) const final
    {
        const SimpleMesh *ret = nullptr;

        if ( !mSourceConvexHulls.empty() && index < mSourceConvexHulls.size() )
        {
            ret = mSourceConvexHulls[index];
        }

        return ret;
    }

    virtual const SimpleMesh *getMergedConvexHull(uint32_t index) const final
    {
        const SimpleMesh *ret = nullptr;

        if ( !mMergedConvexHulls.empty() && index < mMergedConvexHulls.size() )
        {
            ret = mMergedConvexHulls[index];
        }

        return ret;
    }

    virtual void release(void) final
    {
        delete this;
    }

    double getVolume(const SimpleMesh &sm)
    {
#if 1
        double totalVolume = 0;
        Vec3 bary(0,0,0);
        for (uint32_t i=0; i<sm.mVertexCount; i++)
        {
            Vec3 p(&sm.mVertices[i*3]);
            bary+=p;
        }
        bary/=double(sm.mVertexCount);

        for (uint32_t i=0; i<sm.mTriangleCount; i++)
        {
            uint32_t i1 = sm.mIndices[i*3+0];
            uint32_t i2 = sm.mIndices[i*3+1];
            uint32_t i3 = sm.mIndices[i*3+2];

            Vec3 ver0(&sm.mVertices[i1*3]);
            Vec3 ver1(&sm.mVertices[i2*3]);
            Vec3 ver2(&sm.mVertices[i3*3]);

            totalVolume+=computeVolume4(ver0,ver1,ver2,bary);

        }
        totalVolume = totalVolume / 6.0;
        return totalVolume;
#else
        return vcd::fm_computeMeshVolume(sm.mVertices,sm.mTriangleCount,sm.mIndices);
#endif
    }

    virtual uint32_t finalizeResults(uint32_t maxHullVertices, RaycastMesh *raycastMesh, double voxelScale,bool doShrinkWrap,double vertexScale) final
    {

        //ScopedTime st("finalizeResults");

        size_t hcount = mMergedConvexHulls.size();

        // To 'finalize' the results what we do is
        // we iterate through each convex hull.
        // For each convex hull we shrinkwrap the vertices to the source mesh if they are
        // withing the voxel scale distance provided.
        // Next, it checks to see if the number of vertices in the convex hull exceeds the
        // maximum hull vertex count specified by the user.
        // If so, we quantize the points down to the most statistically significant set
        // of points of that size.
        // Finally, it recomputes the convex hull using these shrinkwrapped and possibly reduced vertices

        ShrinkWrap *ws = ShrinkWrap::create();
        // Shrinkwrap each convex hull against the original source mesh and reduce the hull vertex count if 
        //needed
        for (size_t i=0; i<hcount; i++)
        {
            SimpleMesh *sm = mMergedConvexHulls[i];
            ws->shrinkWrap(*sm,*raycastMesh,maxHullVertices,voxelScale,doShrinkWrap);
            sm->scaleVertices(vertexScale);
        }

        ws->release();

        return uint32_t(mMergedConvexHulls.size());
    }

    void performTask(MergeTask *mt)
    {
        SimpleMesh *sm1 = mt->mHullA;
        SimpleMesh *sm2 = mt->mHullB;

        double volume1 = sm1->mVolume;
        double volume2 = sm2->mVolume;
        double concavity = FLT_MAX;
        if (fm_intersectAABB(sm1->mBmin, sm1->mBmax, sm2->mBmin, sm2->mBmax))
        {
            SimpleMesh *combined = computeCombinedConvexHull(*sm1, *sm2); // Build the combined convex hull
            double combinedVolume = getVolume(*combined); // get the combined volume
            concavity = computeConcavity(volume1 + volume2, combinedVolume, mOverallMeshVolume);
            delete combined; // done with it
        }
        else
        {
            double bmin[3];
            double bmax[3];
            fm_combineAABB(sm1->mBmin, sm1->mBmax, sm2->mBmin, sm2->mBmax, bmin, bmax);
            double combinedVolume = fm_volumeAABB(bmin, bmax);
            const double volume2 = sm2->mVolume;
            concavity = computeConcavity(volume1 + volume2, combinedVolume, mOverallMeshVolume);
        }

        HullPair hp(sm1->mMeshId, sm2->mMeshId,concavity);

        // When inserting the result into the hull pair map we need
        // to take a mutex lock so multiple tasks aren't trying to refresh this
        // container at the same time.
        {
            lock_guard _lock(mTaskMutex);
            mHullPairQueue.push(hp);
        }
    }

    // Just use the input hulls as the output without no other corrections
    virtual void finalizeHulls(double scaleVertices) final
    {
        mMergedConvexHulls.clear();
        mMergedConvexHulls.reserve(mHulls.size());
        for (auto &i : mSourceConvexHulls)
        {
            SimpleMesh *sm = new SimpleMesh(*i);
            vcd::fm_computeCentroid(sm->mVertexCount, sm->mVertices, sm->mCenter);
            sm->scaleVertices(scaleVertices);
            mMergedConvexHulls.push_back(sm);
        }
    }

    uint32_t            mMeshId{0};
    SimpleMeshVector    mSourceConvexHulls;
    SimpleMeshVector    mMergedConvexHulls;
    HullPairQueue       mHullPairQueue;
    HullMap             mHulls;
    JobSystem           *mJobSystem{nullptr};
    MutexWrapper        mTaskMutex;
    double              mOverallMeshVolume{0};
};

void mergeTask(void *ptr)
{
    MergeTask *mt = (MergeTask *)ptr;
    mt->mPthis->performTask(mt);
}

MergeConvexHulls *MergeConvexHulls::create(JobSystem *jobSystem)
{
    jobSystem = nullptr;
    auto ret = new MergeConvexHullsImpl(jobSystem);
    return static_cast< MergeConvexHulls *>(ret);
}


}
