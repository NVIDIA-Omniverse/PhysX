// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>

// This is a header only library. In only *one* of your source files you must
// declare: #define ENABLE_SPHERE_APPROX_IMPLEMENTATION 1
// prior to including 'SphereApprox.h' to make sure the implementation
// code is all generated.

namespace sphereapprox
{

/**
 * This enumeration determines how the voxels as filled to create a solid
 * object. The default should be 'FLOOD_FILL' which generally works fine
 * for closed meshes. However, if the mesh is not watertight, then using
 * RAYCAST_FILL may be preferable as it will determine if a voxel is part
 * of the interior of the source mesh by raycasting around it.
 *
 * Finally, there are some cases where you might actually want a convex
 * decomposition to treat the source mesh as being hollow. If that is the
 * case you can pass in 'SURFACE_ONLY' and then the convex decomposition
 * will converge only onto the 'skin' of the surface mesh.
 */
enum class FillMode
{
    FLOOD_FILL, // This is the default behavior, after the voxelization step it uses a flood fill to determine 'inside'
                // from 'outside'. However, meshes with holes can fail and create hollow results.
    SURFACE_ONLY, // Only consider the 'surface', will create 'skins' with hollow centers.
    RAYCAST_FILL // Uses raycasting to determine inside from outside.
};

class Volume;

class NotifySphereApproximation
{
public:
    virtual void notifySphereApproximationComplete(void) = 0;
};

/**
 * This is the public interface to the sphere approximation library.
 * You use 'create' to create an instance of it and 'release' when you are
 * finished.
 */
class SphereApprox
{
public:
    /**
     * Static method to create an instance of the SphereApprox class
     * Example: sphereapprox::SphereApprox *sa = sphereapprox::SphereApprox::create();
     * Then invoke 'release' when you are finished with it: sa->release();
     */
    static SphereApprox* create(void);
    /**
     * This class is used to return the sphere approximation results
     */
    class Sphere
    {
    public:
        double mCenter[3]; // The center of the sphere
        double mRadius{ 0 }; // The radius of the sphere
        uint32_t mId{ 0 }; // An internal unique id assigned to each sphere
    };

    /**
     * This class defines the tuning parameters which control how the
     * sphere approximation algorithm will run
     */
    class Parameters
    {
    public:
        FillMode mFillMode{ FillMode::FLOOD_FILL }; // The voxelization fill mode
        uint32_t mVoxelResolution{ 100000 }; // default voxel resolution
        uint32_t mMaxSeedCount{ 1024 }; // Maximum number of seed voxels
        double mErrorMetric{ 5 }; // Percent of exterior voxels allowed in a sphere
        uint32_t mMinStepCount{ 3 }; // do at least this many steps when growing a sphere
        uint32_t mMaxSpheres{ 64 }; // Maximum number of spheres we can produce
        uint32_t mMaxOverlapCount{ 7 }; // Maximum number of overlaps before culling this sphere
        double mCullPercentage{ 50.0 }; // Percentage to cull a sphere
        bool mAsync{ true }; // Whether or not to solve asynchronously
        uint32_t mMaxThreadCount{ 22 }; // number of threads to use
        bool mFinalize{ true }; // Default is to solve completely, only set to false for debugging
        bool mDisplayTiming{ false }; // Debug option to display timing
        NotifySphereApproximation* mNotifyCallback{ nullptr };
    };

    /**
     * This method is used to begin computing a sphere approximation of
     * a triangle mesh asynchronously unless otherwise specified.
     * This version accepts float vertex inputs.
     *
     * @param points : The input vertices in the format of X1,Y1,Z1, X2,Y2,Z2 as floats
     * @param countPoints : The number of input vertices
     * @param triangles : The triangle indices for the source triangle mesh as unsigned integers. Three indexes per
     * triangle.
     * @param countTriangles : The number of triangles in the triangle mesh
     * @param params : The tuning parameters to use
     *
     * @return : Returns true if the task could be started.
     */
    virtual bool compute(const float* const points,
                         const uint32_t countPoints,
                         const uint32_t* const triangles,
                         const uint32_t countTriangles,
                         const Parameters& params) = 0;

    /**
     * This method is used to begin computing a sphere approximation of
     * a triangle mesh asynchronously unless otherwise specified.
     * This version accepts double vertex inputs.
     *
     * @param points : The input vertices in the format of X1,Y1,Z1, X2,Y2,Z2 as doubles
     * @param countPoints : The number of input vertices
     * @param triangles : The triangle indices for the source triangle mesh as unsigned integers. Three indexes per
     * triangle.
     * @param countTriangles : The number of triangles in the triangle mesh
     * @param params : The tuning parameters to use
     *
     * @return : Returns true if the task could be started.
     */
    virtual bool compute(const double* const points,
                         const uint32_t countPoints,
                         const uint32_t* const triangles,
                         const uint32_t countTriangles,
                         const Parameters& params) = 0;

    /**
     * Invoke this method to cancel the sphere approximation early if it
     * is running asynchronously
     */
    virtual void cancel() = 0;

    /**
     * Instruct the job thread to wait for the specified number of milliseconds
     */
    virtual void wait(uint32_t ms) = 0;

    /**
     * This method returns the sphere approximation results, either sorted by
     * volume or the reduced sphere set.
     *
     * @param sphereCount : The number of spheres matching the request
     * @param reduced : If true it will return the reduced set of spheres rather than all of them.
     *
     * @return : Returns a pointer to an array of spheres matching the request
     */
    virtual const Sphere* getSpheres(uint32_t& sphereCount, bool reduced) const = 0;

    /**
     * Releases the instance of this class
     */
    virtual void release() = 0;

    /**
     * Returns true if the sphere approximation has completed in a background thread
     *
     * @return : Returns true if the approximation computation is complete
     */
    virtual bool isReady() const = 0;

    /**
     * This is a debugging option used internally for testing purposes only.
     */
    virtual bool stepSphere(void) = 0;

    /**
     * This is a debugging option used internally for testing purposes only.
     */
    virtual void setParameters(Parameters& p) = 0;

    /**
     * This is a debugging option used internally for testing purposes only.
     */
    virtual bool beginReduction(void) = 0;

    /**
     * This is a debugging option used internally for testing purposes only.
     */
    virtual bool reductionStep(void) = 0;

    /**
     * This is a debugging option used internally for testing purposes only.
     */
    virtual const Parameters& getParameters(void) const = 0;

protected:
    virtual ~SphereApprox()
    {
    }
};


} // namespace sphereapprox


#if ENABLE_SPHERE_APPROX_IMPLEMENTATION
#    include <assert.h>
#    include <math.h>
#    include <stdlib.h>
#    include <string.h>
#    include <float.h>
#    include <limits.h>

#    include <algorithm>
#    include <array>
#    include <atomic>
#    include <chrono>
#    include <condition_variable>
#    include <deque>
#    include <future>
#    include <iostream>
#    include <list>
#    include <memory>
#    include <mutex>
#    include <queue>
#    include <thread>
#    include <unordered_map>
#    include <unordered_set>
#    include <utility>
#    include <vector>
#    include <functional>


#    ifdef _MSC_VER
#        pragma warning(push)
#        pragma warning(disable : 4100 4127 4189 4244 4456 4701 4702 4996 4267)
#    endif // _MSC_VER

#    ifdef __GNUC__
#        pragma GCC diagnostic push
// Minimum set of warnings used for cleanup
// #pragma GCC diagnostic warning "-Wall"
// #pragma GCC diagnostic warning "-Wextra"
// #pragma GCC diagnostic warning "-Wpedantic"
// #pragma GCC diagnostic warning "-Wold-style-cast"
// #pragma GCC diagnostic warning "-Wnon-virtual-dtor"
// #pragma GCC diagnostic warning "-Wshadow"
#    endif // __GNUC__

#    pragma once

#    include <stdint.h>

#    include <algorithm>
#    include <array>
#    include <atomic>
#    include <cassert>
#    include <chrono>
#    include <condition_variable>
#    include <cstddef>
#    include <cstdint>
#    include <deque>
#    include <future>
#    include <list>
#    include <mutex>
#    include <queue>
#    include <random>
#    include <thread>
#    include <tuple>
#    include <type_traits>
#    include <vector>

namespace threadpool
{
class ThreadPool;
}

namespace kmeanspp
{

class KmeansPP
{
public:
    class Parameters
    {
    public:
        const float* mPoints{ nullptr }; // A point to a set of 3d data points in X/Y/Z format
        uint32_t mPointCount{ 0 }; // The total number of input points
        uint32_t mMaxPoints{ 0 }; // The maximum number of output points
        // The KmeansPP++ algorithm does a very CPU intensive task to get the
        // initial cluster values. By default it uses *all* of the input points
        // to calculate this. However, the number of input points could easily be
        // in the millions and this makes the initialization phase take an extremely
        // long time.
        // You can assign 'mmMaximumPlusPlusCount' in the following ways:
        //
        // * If it is less than or equal to the mMaxPoints it will *disable* the
        // * KmeansPP++ initialization phase and instead just sample 'mMaxPoints' number
        // * of input points as the initial cluster values.
        //
        // * If it is exactly equal to 'mPointCount' count then it will do the standard
        // * kmeanspp++ computation which considers *all* of the input points.
        //
        // * If the value is greater than 'mMaxPoints' but less than 'mPointCount' it
        // * will still run the KmeansPP++ clustering computation but on a smaller subset
        // * of data points. This is a good compromise to get the benefits of the KmeansPP++
        // * initialization while not destroying performance by considering all points.
        uint32_t mMaximumPlusPlusCount{ 0 };
        // The maximum number of iterations to perform before giving up.
        // Usually KmeansPP++ converges in far less than a 100 iterations so this
        // is kind of an emergency out if it ever failed to converge
        uint32_t mMaxIterations{ 100 };
        // The user provides a random number seed to use. The reason for this
        // is that for the purposes of testing and validation we want to be able
        // to get the same results for the same set of input data each time.
        // For your own purposes you can change the random number seed if you prefer.
        uint32_t mRandomSeed{ 0 };
        // By default we use a KdTree to speed up the distance computations.
        // The only reason to disable this is for testing purposes.
        bool mUseKdTree{ true }; // you would always want this to true unless trying to debug something
        // By default multi-threading should be enabled as it provides an order of
        // magnitude performance increase on hyperthreaded machines. The only reason
        // to disable it would be for testing and debugging purposes.
        bool mUseThreading{ true };
        // On hyperthreaded machines 22 threads seems to be a sweet spot.
        // If you want to use fewer threads, just change this value
        uint32_t mThreadCount{ 22 };
        // If the user provides a pointer to an instance of the ThreadPool class
        // then it will be used rather than creating a unique instance of one.
        threadpool::ThreadPool* mThreadPool{ nullptr };
    };

    static KmeansPP* create(void);

    virtual const float* compute(const Parameters& params, uint32_t& resultPointCount) = 0;

    virtual void release(void) = 0;

protected:
    virtual ~KmeansPP(void)
    {
    }
};

} // namespace kmeanspp


namespace threadpool
{

class ThreadPool
{
public:
    ThreadPool();
    ThreadPool(int worker);
    ~ThreadPool();
    template <typename F, typename... Args>
    auto enqueue(F&& f, Args&&... args)
#    ifndef __cpp_lib_is_invocable
        -> std::future<typename std::result_of<F(Args...)>::type>;
#    else
        -> std::future<typename std::invoke_result_t<F, Args...>>;
#    endif
private:
    std::vector<std::thread> workers;
    std::deque<std::function<void()>> tasks;
    std::mutex task_mutex;
    std::condition_variable cv;
    bool closed;
    int count;
};

ThreadPool::ThreadPool() : ThreadPool(1)
{
}

ThreadPool::ThreadPool(int worker) : closed(false), count(0)
{
    workers.reserve(worker);
    for (int i = 0; i < worker; i++)
    {
        workers.emplace_back([this] {
            std::unique_lock<std::mutex> lock(this->task_mutex);
            while (true)
            {
                while (this->tasks.empty())
                {
                    if (this->closed)
                    {
                        return;
                    }
                    this->cv.wait(lock);
                }
                auto task = this->tasks.front();
                this->tasks.pop_front();
                lock.unlock();
                task();
                lock.lock();
            }
        });
    }
}

template <typename F, typename... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args)
#    ifndef __cpp_lib_is_invocable
    -> std::future<typename std::result_of<F(Args...)>::type>
#    else
    -> std::future<typename std::invoke_result_t<F, Args...>>
#    endif
{

#    ifndef __cpp_lib_is_invocable
    using return_type = typename std::result_of<F(Args...)>::type;
#    else
    using return_type = typename std::invoke_result_t<F, Args...>;
#    endif
    auto task =
        std::make_shared<std::packaged_task<return_type()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
    auto result = task->get_future();

    {
        std::unique_lock<std::mutex> lock(task_mutex);
        if (!closed)
        {
            tasks.emplace_back([task] { (*task)(); });
            cv.notify_one();
        }
    }

    return result;
}

ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(task_mutex);
        closed = true;
    }
    cv.notify_all();
    for (auto&& worker : workers)
    {
        worker.join();
    }
}


} // namespace threadpool

namespace kdtree
{

//****
/**
 * Class for representing a point. coordinate_type must be a numeric type.
 */
template <typename coordinate_type, size_t dimensions>
class point
{
public:
    point(coordinate_type x, coordinate_type y, coordinate_type z)
    {
        mCoordinates[0] = x;
        mCoordinates[1] = y;
        mCoordinates[2] = z;
    }
    point(std::array<coordinate_type, dimensions> c) : mCoordinates(c)
    {
    }

    /**
     * Returns the coordinate in the given dimension.
     *
     * @param index dimension index (zero based)
     * @return coordinate in the given dimension
     */
    coordinate_type get(size_t index) const
    {
        return mCoordinates[index];
    }

    /**
     * Returns the distance squared from this point to another
     * point.
     *
     * @param pt another point
     * @return distance squared from this point to the other point
     */
    float distance(const point& pt) const
    {
        float dist = 0;
        for (size_t i = 0; i < dimensions; ++i)
        {
            float d = get(i) - pt.get(i);
            dist += d * d;
        }
        return dist;
    }

    void setId(uint32_t id)
    {
        mId = id;
    }
    uint32_t getId(void) const
    {
        return mId;
    }

private:
    std::array<coordinate_type, dimensions> mCoordinates;
    uint32_t mId;
};


// Templated implementation of KdTree
template <typename coordinate_type, size_t dimensions>
class KdTreeTemplate
{
public:
    typedef point<coordinate_type, dimensions> point_type;

private:
    class KdNode
    {
    public:
        KdNode(const point_type& pt) : mPoint(pt), mLeft(nullptr), mRight(nullptr)
        {
        }
        coordinate_type get(size_t index) const
        {
            return mPoint.get(index);
        }

        float distance(const point_type& pt) const
        {
            return mPoint.distance(pt);
        }

        point_type mPoint;
        KdNode* mLeft;
        KdNode* mRight;
    };

    class FindNearest
    {
    public:
        KdNode* mBest = { nullptr };
        float mBestDistance{ 0 };
        size_t mVisitCount{ 0 };
    };
    KdNode* mRoot{ nullptr };

    std::vector<KdNode> mNodes;

    class KdNodeCompare
    {
    public:
        KdNodeCompare(size_t index) : index_(index)
        {
        }

        bool operator()(const KdNode& n1, const KdNode& n2) const
        {
            return n1.mPoint.get(index_) < n2.mPoint.get(index_);
        }

        size_t index_;
    };

    KdNode* buildKdTree(size_t begin, size_t end, size_t index)
    {
        if (end <= begin)
            return nullptr;
        size_t n = begin + (end - begin) / 2;
        auto i = mNodes.begin();
        std::nth_element(i + begin, i + n, i + end, KdNodeCompare(index));
        index = (index + 1) % dimensions;
        mNodes[n].mLeft = buildKdTree(begin, n, index);
        mNodes[n].mRight = buildKdTree(n + 1, end, index);
        return &mNodes[n];
    }

    void nearest(KdNode* root, const point_type& point, size_t index, FindNearest& fn)
    {
        if (root == nullptr)
            return;
        ++fn.mVisitCount;
        float d = root->distance(point);
        if (fn.mBest == nullptr || d < fn.mBestDistance)
        {
            fn.mBestDistance = d;
            fn.mBest = root;
        }
        if (fn.mBestDistance == 0)
            return;
        float dx = root->get(index) - point.get(index);
        index = (index + 1) % dimensions;
        nearest(dx > 0 ? root->mLeft : root->mRight, point, index, fn);
        if (dx * dx >= fn.mBestDistance)
            return;
        nearest(dx > 0 ? root->mRight : root->mLeft, point, index, fn);
    }

public:
    KdTreeTemplate(const KdTreeTemplate&) = delete;
    KdTreeTemplate& operator=(const KdTreeTemplate&) = delete;
    /**
     * Constructor taking a pair of iterators. Adds each
     * point in the range [begin, end) to the tree.
     *
     * @param begin start of range
     * @param end end of range
     */
    template <typename iterator>
    KdTreeTemplate(iterator begin, iterator end) : mNodes(begin, end)
    {
        mRoot = buildKdTree(0, mNodes.size(), 0);
    }


    /**
     * Returns true if the tree is empty, false otherwise.
     */
    bool empty() const
    {
        return mNodes.empty();
    }

    /**
     * Finds the nearest point in the tree to the given point.
     * It is not valid to call this function if the tree is empty.
     *
     * @param pt a point
     * @return the nearest point in the tree to the given point
     */
    const point_type& nearest(const point_type& pt)
    {
        if (mRoot == nullptr)
            throw std::logic_error("tree is empty");
        FindNearest fn;
        fn.mBest = nullptr;
        fn.mVisitCount = 0;
        fn.mBestDistance = 0;
        nearest(mRoot, pt, 0, fn);
        return fn.mBest->mPoint;
    }
};

typedef point<float, 3> point3d;
typedef KdTreeTemplate<float, 3> tree3d;

//****

class KdPoint
{
public:
    KdPoint(void){};
    KdPoint(float _x, float _y, float _z) : x(_x), y(_y), z(_z)
    {
    }
    float x;
    float y;
    float z;
    uint32_t mId;
};

using Point3dVector = std::vector<point3d>;

class KdTree
{
public:
    KdTree(void)
    {
    }

    ~KdTree(void)
    {
        delete mTree;
    }

    void reservePoints(uint32_t pcount)
    {
        delete mTree;
        mTree = nullptr;
        mPoints.clear();
        mPoints.reserve(pcount);
    }

    // Add this point...
    void addPoint(const KdPoint& p)
    {
        point3d pp(p.x, p.y, p.z);
        pp.setId(p.mId);
        mPoints.push_back(pp);
    }

    void buildTree(void)
    {
        mTree = new tree3d(std::begin(mPoints), std::end(mPoints));
    }

    float findNearest(const KdPoint& p, KdPoint& result)
    {
        float ret = -1;

        if (mTree)
        {
            point3d pt(p.x, p.y, p.z);
            point3d n = mTree->nearest(pt);
            result.x = n.get(0);
            result.y = n.get(1);
            result.z = n.get(2);
            result.mId = n.getId();
            ret = n.distance(pt);
        }

        return ret;
    }

private:
    tree3d* mTree{ nullptr };
    Point3dVector mPoints;
};


} // namespace kdtree

namespace randpool
{
class RandPool
{
public:
    RandPool(uint32_t size, uint32_t _seed) // size of random number bool.
    {
        mRandomEngine.seed(_seed);
        mData = new uint32_t[size];
        mSize = size;
        mTop = mSize;
        for (uint32_t i = 0; i < mSize; i++)
        {
            mData[i] = i;
        }
    }

    ~RandPool(void)
    {
        delete[] mData;
    };

    // pull a number from the random number pool, will never return the
    // same number twice until the 'deck' (pool) has been exhausted.
    // Will set the shuffled flag to true if the deck/pool was exhausted
    // on this call.
    uint32_t get(bool& shuffled)
    {
        if (mTop == 0) // deck exhausted, shuffle deck.
        {
            shuffled = true;
            mTop = mSize;
        }
        else
        {
            shuffled = false;
        }
        uint32_t entry = uint32_t(mRandomEngine.operator()()) % mTop;
        mTop--;
        uint32_t ret = mData[entry]; // swap top of pool with entry
        mData[entry] = mData[mTop]; // returned
        mData[mTop] = ret;
        return ret;
    };


private:
    uint32_t* mData; // random number bool.
    uint32_t mSize; // size of random number pool.
    uint32_t mTop; // current top of the random number pool.
    std::linear_congruential_engine<uint64_t, 6364136223846793005, 1442695040888963407, UINT64_MAX> mRandomEngine;
};


} // namespace randpool

namespace kmeanspp
{

// Process 128 points per thread
#    define PARALLEL_FOR_CHUNK_SIZE 256

class ParallelFor
{
public:
    uint32_t mStartIndex{ 0 };
    uint32_t mStopIndex{ 0 };
    std::future<void> mFuture;
};

using ParallelForVector = std::vector<ParallelFor>;


class Point3
{
public:
    bool operator!=(const Point3& p) const
    {
        bool ret = false;

        if (p.x != x || p.y != y || p.z != z)
        {
            ret = true;
        }

        return ret;
    }

    float distanceSquared(const Point3& p) const
    {
        float dx = x - p.x;
        float dy = y - p.y;
        float dz = z - p.z;
        return dx * dx + dy * dy + dz * dz;
    }

    float x;
    float y;
    float z;
};

using Point3Vector = std::vector<Point3>;
using ClusterVector = std::vector<uint32_t>;
using DistanceVector = std::vector<float>;

class KmeansImpl : public KmeansPP
{
public:
    KmeansImpl(void)
    {
    }

    virtual ~KmeansImpl(void)
    {
    }

    virtual const float* compute(const Parameters& params, uint32_t& resultPointCount) final
    {
        const float* ret = nullptr;

        mParams = params;

        mK = params.mMaxPoints;
        mMeans.clear();
        mMeans.reserve(mK);
        // If the number of input points is less than or equal
        // to 'K' we just return the input points directly
        if (params.mPointCount <= mK)
        {
            mMeans.resize(params.mPointCount);
            memcpy(&mMeans[0], params.mPoints, sizeof(Point3) * params.mPointCount);
            resultPointCount = params.mPointCount;
            ret = &mMeans[0].x;
            return ret;
        }

        mClusters.resize(params.mPointCount);
        mData.resize(params.mPointCount);
        memcpy(&mData[0], params.mPoints, sizeof(float) * 3 * params.mPointCount);
        {
            initializeClusters(params);
        }
        uint32_t count = 0;
        // Resize the means container to have room for
        // a total of three copies.
        // When iterating on kmeanspp we stop if the means results are
        // the same as the previous iteration *or* the one before that.
        // To avoid memory copying we instead just cycle pointers
        size_t msize = mMeans.size();
        mMeans.resize(msize * 3);
        mCurrentMeans = &mMeans[0];
        mOldMeans = &mMeans[msize];
        mOldOldMeans = &mMeans[msize * 2];
        memcpy(mOldMeans, mCurrentMeans, sizeof(Point3) * msize);
        memcpy(mOldOldMeans, mCurrentMeans, sizeof(Point3) * msize);

        do
        {
            {
                calculateClusters(mCurrentMeans, msize, params.mUseKdTree);
            }
            // Pointer swap, the current means is now the old means.
            // The old means is now the old-old means
            // And the old old means pointer now becomes the current means pointer
            Point3* temp = mOldOldMeans;
            mOldOldMeans = mOldMeans;
            mOldMeans = mCurrentMeans;
            mCurrentMeans = temp;

            calculateMeans(mCurrentMeans, msize, mOldMeans);

            count++;
            if (sameMeans(mCurrentMeans, mOldMeans, msize))
            {
                break;
            }
            if (sameMeans(mCurrentMeans, mOldOldMeans, msize))
            {
                break;
            }
        } while (count < params.mMaxIterations);

        releaseThreadPool();

        resultPointCount = mK;
        ret = &mMeans[0].x;
        return ret;
    }

    bool nearlySameMeans(const Point3Vector& a, const Point3Vector& b)
    {
        bool ret = true;

        for (size_t i = 0; i < a.size(); i++)
        {
            double d = a[i].distanceSquared(b[i]);
            if (d > mLimitDelta)
            {
                ret = false;
            }
        }

        return ret;
    }


    bool sameMeans(const Point3* a, const Point3* b, size_t msize)
    {
        bool ret = true;

        for (size_t i = 0; i < msize; i++)
        {
            if (a[i] != b[i])
            {
                ret = false;
                break;
            }
        }

        return ret;
    }

    void calculateMeans(Point3* means, size_t msize, const Point3* oldMeans)
    {
        std::vector<uint32_t> counts;
        assert(mData.size() == mClusters.size());
        counts.resize(msize);
        memset(&counts[0], 0, sizeof(uint32_t) * msize);
        memset(means, 0, sizeof(Point3) * msize);

        for (size_t i = 0; i < mClusters.size(); i++)
        {
            uint32_t id = mClusters[i];
            assert(id < uint32_t(msize));
            auto& mean = means[id];
            counts[id]++;

            const auto& p = mData[i];
            mean.x += p.x;
            mean.y += p.y;
            mean.z += p.z;
        }
        for (size_t i = 0; i < msize; i++)
        {
            if (counts[i] == 0)
            {
                means[i] = oldMeans[i];
            }
            else
            {
                float recip = 1.0f / float(counts[i]);
                means[i].x *= recip;
                means[i].y *= recip;
                means[i].z *= recip;
            }
        }
    }

    virtual void release(void) final
    {
        delete this;
    }

    void initializeClusters(const Parameters& params)
    {
        uint32_t maxPlusPlusCount = params.mMaximumPlusPlusCount;
        if (maxPlusPlusCount < params.mMaxPoints)
        {
            maxPlusPlusCount = params.mMaxPoints;
        }
        else if (maxPlusPlusCount > params.mPointCount)
        {
            maxPlusPlusCount = params.mPointCount;
        }
        if (params.mMaxPoints == maxPlusPlusCount)
        {
            mMeans.clear();
            mMeans.resize(maxPlusPlusCount);
            randpool::RandPool rp(params.mPointCount, uint32_t(mParams.mRandomSeed));
            for (uint32_t i = 0; i < maxPlusPlusCount; i++)
            {
                bool shuffled;
                uint32_t index = rp.get(shuffled);
                const auto& p = mData[index];
                mMeans[i] = p;
            }
            return;
        }
        uint32_t dataSize = uint32_t(mData.size());
        if (maxPlusPlusCount != dataSize)
        {
            mReducedData.resize(maxPlusPlusCount);
            randpool::RandPool rp(dataSize, uint32_t(mParams.mRandomSeed));
            for (uint32_t i = 0; i < maxPlusPlusCount; i++)
            {
                bool shuffled;
                uint32_t index = rp.get(shuffled);
                mReducedData[i] = mData[index];
            }
        }
        else
        {
            mReducedData = mData;
        }
        if (getThreadPool())
        {
            uint32_t dcount = uint32_t(mReducedData.size());
            uint32_t chunkCount = (dcount + PARALLEL_FOR_CHUNK_SIZE - 1) / PARALLEL_FOR_CHUNK_SIZE;
            mParallelFor.resize(chunkCount);
            for (uint32_t i = 0; i < chunkCount; i++)
            {
                ParallelFor& p = mParallelFor[i];
                p.mStartIndex = i * PARALLEL_FOR_CHUNK_SIZE;
                p.mStopIndex = p.mStartIndex + (PARALLEL_FOR_CHUNK_SIZE - 1);
                if (p.mStopIndex >= dcount)
                {
                    p.mStopIndex = dcount - 1;
                }
            }
        }

        std::random_device rand_device;
        uint64_t seed = mParams.mRandomSeed;
        // Using a very simple PRBS generator, parameters selected according to
        // https://en.wikipedia.org/wiki/Linear_congruential_generator#Parameters_in_common_use
        std::linear_congruential_engine<uint64_t, 6364136223846793005, 1442695040888963407, UINT64_MAX> rand_engine(seed);
        // Select first mean at random from the set
        {
            std::uniform_int_distribution<size_t> uniform_generator(0, mReducedData.size() - 1);
            size_t rindex = uniform_generator(rand_engine);
            mMeans.push_back(mReducedData[rindex]);
        }
        if (params.mUseKdTree)
        {
            mKdTree = new kdtree::KdTree;
            mKdTree->reservePoints(mK);
            const auto& m = mMeans[0];
            kdtree::KdPoint p(m.x, m.y, m.z);
            p.mId = 0;
            mKdTree->addPoint(p);
            mKdTree->buildTree();
        }

        mDistances.resize(mReducedData.size());

        for (uint32_t count = 1; count < mK; ++count)
        {
            // Calculate the distance to the closest mean for each data point
            if (params.mUseKdTree)
            {
                closestDistanceKdTree();
            }
            else
            {
                closestDistance();
            }
            // Pick a random point weighted by the distance from existing means
            // TODO: This might convert floating point weights to ints, distorting the distribution for small weights
            std::discrete_distribution<size_t> generator(mDistances.begin(), mDistances.end());
            uint32_t index = (uint32_t)mMeans.size();
            mMeans.push_back(mReducedData[generator(rand_engine)]);
            if (params.mUseKdTree)
            {
                const auto& m = mMeans[index];
                kdtree::KdPoint p(m.x, m.y, m.z);
                p.mId = index;
                mKdTree->addPoint(p);
                mKdTree->buildTree();
            }
        }
        delete mKdTree;
        mKdTree = nullptr;
    }

    void closestDistance(void)
    {
        if (getThreadPool())
        {
            auto tp = getThreadPool();
            for (auto& p : mParallelFor)
            {
                ParallelFor* pf = &p;
                p.mFuture = tp->enqueue([this, pf] {
                    for (uint32_t i = pf->mStartIndex; i <= pf->mStopIndex; i++)
                    {
                        float closest = FLT_MAX;
                        for (auto& m : mMeans)
                        {
                            float distance = mReducedData[i].distanceSquared(m);
                            if (distance < closest)
                            {
                                closest = distance;
                            }
                        }
                        mDistances[i] = closest;
                    }
                });
            }
            for (auto& p : mParallelFor)
            {
                p.mFuture.get();
            }
        }
        else
        {
            uint32_t index = 0;
            for (auto& d : mReducedData)
            {
                float closest = FLT_MAX;
                for (auto& m : mMeans)
                {
                    float distance = d.distanceSquared(m);
                    if (distance < closest)
                    {
                        closest = distance;
                    }
                }
                mDistances[index] = closest;
                index++;
            }
        }
    }

    void closestDistanceKdTree(void)
    {
        if (getThreadPool())
        {
            auto tp = getThreadPool();
            for (auto& p : mParallelFor)
            {
                ParallelFor* pf = &p;
                p.mFuture = tp->enqueue([this, pf] {
                    for (uint32_t i = pf->mStartIndex; i <= pf->mStopIndex; i++)
                    {
                        const auto& d = mReducedData[i];
                        kdtree::KdPoint p(d.x, d.y, d.z), r;
                        mDistances[i] = mKdTree->findNearest(p, r);
                    }
                });
            }
            for (auto& p : mParallelFor)
            {
                p.mFuture.get();
            }
        }
        else
        {
            uint32_t index = 0;
            for (auto& d : mReducedData)
            {
                kdtree::KdPoint p(d.x, d.y, d.z), r;
                mDistances[index] = mKdTree->findNearest(p, r);
                index++;
            }
        }
    }

    void calculateClusters(const Point3* means, size_t msize, bool useKdTree)
    {
        if (useKdTree)
        {
            kdtree::KdTree kdt;
            kdt.reservePoints(msize);
            for (uint32_t i = 0; i < msize; i++)
            {
                const auto& p = means[i];
                kdtree::KdPoint kp(p.x, p.y, p.z);
                kp.mId = i;
                kdt.addPoint(kp);
            }
            kdt.buildTree();
            if (getThreadPool())
            {
                auto tp = getThreadPool();
                kdtree::KdTree* kdtp = &kdt;
                for (auto& p : mParallelFor)
                {
                    ParallelFor* pf = &p;
                    p.mFuture = tp->enqueue([this, pf, kdtp] {
                        for (uint32_t i = pf->mStartIndex; i <= pf->mStopIndex; i++)
                        {
                            const auto& p = mData[i];
                            kdtree::KdPoint kp(p.x, p.y, p.z);
                            kdtree::KdPoint result;
                            result.mId = 0;
                            kdtp->findNearest(kp, result);
                            mClusters[i] = result.mId;
                        }
                    });
                }
                for (auto& p : mParallelFor)
                {
                    p.mFuture.get();
                }
            }
            else
            {
                for (size_t i = 0; i < mData.size(); i++)
                {
                    const auto& p = mData[i];
                    kdtree::KdPoint kp(p.x, p.y, p.z);
                    kdtree::KdPoint result;
                    kdt.findNearest(kp, result);
                    assert(result.mId < msize);
                    mClusters[i] = result.mId;
                }
            }
        }
        else
        {
            if (getThreadPool())
            {
                auto tp = getThreadPool();
                for (auto& p : mParallelFor)
                {
                    ParallelFor* pf = &p;
                    p.mFuture = tp->enqueue([this, pf, means, msize] {
                        for (uint32_t i = pf->mStartIndex; i <= pf->mStopIndex; i++)
                        {
                            mClusters[i] = closestMean(means, msize, mData[i]);
                        }
                    });
                }
                for (auto& p : mParallelFor)
                {
                    p.mFuture.get();
                }
            }
            else
            {
                for (size_t i = 0; i < mData.size(); i++)
                {
                    mClusters[i] = closestMean(means, msize, mData[i]);
                }
            }
        }
    }

    uint32_t closestMean(const Point3* means, size_t msize, const Point3& p) const
    {
        uint32_t ret = 0;
        float closest = FLT_MAX;
        for (uint32_t i = 0; i < msize; i++)
        {
            float d2 = p.distanceSquared(means[i]);
            if (d2 < closest)
            {
                closest = d2;
                ret = i;
            }
        }
        return ret;
    }

    threadpool::ThreadPool* getThreadPool(void)
    {
        threadpool::ThreadPool* ret = mParams.mThreadPool ? mParams.mThreadPool : mThreadPool;

        if (!ret && mParams.mUseThreading)
        {
            ret = mThreadPool = new threadpool::ThreadPool(mParams.mThreadCount);
        }

        return ret;
    }

    void releaseThreadPool(void)
    {
        delete mThreadPool;
        mThreadPool = nullptr;
    }

    uint32_t mK{ 32 }; // Maximum number of mean values to produce
    Point3Vector mData; // Input data
    Point3Vector mReducedData; // Input data

    Point3* mCurrentMeans{ nullptr };
    Point3* mOldMeans{ nullptr };
    Point3* mOldOldMeans{ nullptr };

    DistanceVector mDistances;
    Point3Vector mMeans; // Means

    ClusterVector mClusters; // Which cluster each source data point is in
    double mLimitDelta{ 0.001f };

    kdtree::KdTree* mKdTree{ nullptr };
    threadpool::ThreadPool* mThreadPool{ nullptr };
    ParallelForVector mParallelFor;
    Parameters mParams;
};

KmeansPP* KmeansPP::create(void)
{
    auto ret = new KmeansImpl;
    return static_cast<KmeansPP*>(ret);
}


} // namespace kmeanspp


namespace sphereapprox
{

struct Vertex
{
    double mX;
    double mY;
    double mZ;

    Vertex() = default;
    Vertex(double x, double y, double z) : mX(x), mY(y), mZ(z)
    {
    }

    const double& operator[](size_t idx) const
    {
        switch (idx)
        {
        case 0:
            return mX;
        case 1:
            return mY;
        case 2:
            return mZ;
        };
        return mX;
    }
};

struct Triangle
{
    uint32_t mI0;
    uint32_t mI1;
    uint32_t mI2;

    Triangle() = default;
    Triangle(uint32_t i0, uint32_t i1, uint32_t i2) : mI0(i0), mI1(i1), mI2(i2)
    {
    }
};

template <typename T>
class Vector3
{
public:
    /*
     * Getters
     */
    T& operator[](size_t i);
    const T& operator[](size_t i) const;
    T& GetX();
    T& GetY();
    T& GetZ();
    const T& GetX() const;
    const T& GetY() const;
    const T& GetZ() const;

    /*
     * Normalize and norming
     */
    T Normalize();
    Vector3 Normalized();
    T GetNorm() const;
    T GetNormSquared() const;
    int LongestAxis() const;

    /*
     * Vector-vector operations
     */
    Vector3& operator=(const Vector3& rhs);
    Vector3& operator+=(const Vector3& rhs);
    Vector3& operator-=(const Vector3& rhs);

    Vector3 CWiseMul(const Vector3& rhs) const;
    Vector3 Cross(const Vector3& rhs) const;
    T Dot(const Vector3& rhs) const;
    Vector3 operator+(const Vector3& rhs) const;
    Vector3 operator-(const Vector3& rhs) const;

    /*
     * Vector-scalar operations
     */
    Vector3& operator-=(T a);
    Vector3& operator+=(T a);
    Vector3& operator/=(T a);
    Vector3& operator*=(T a);

    Vector3 operator*(T rhs) const;
    Vector3 operator/(T rhs) const;

    /*
     * Unary operations
     */
    Vector3 operator-() const;

    /*
     * Comparison operators
     */
    bool operator<(const Vector3& rhs) const;
    bool operator>(const Vector3& rhs) const;

    /*
     * Returns true if all elements of *this are greater than or equal to all elements of rhs, coefficient wise
     * LE is less than or equal
     */
    bool CWiseAllGE(const Vector3<T>& rhs) const;
    bool CWiseAllLE(const Vector3<T>& rhs) const;

    Vector3 CWiseMin(const Vector3& rhs) const;
    Vector3 CWiseMax(const Vector3& rhs) const;
    T MinCoeff() const;
    T MaxCoeff() const;

    T MinCoeff(uint32_t& idx) const;
    T MaxCoeff(uint32_t& idx) const;

    /*
     * Constructors
     */
    Vector3() = default;
    Vector3(T a);
    Vector3(T x, T y, T z);
    Vector3(const Vector3& rhs);
    ~Vector3() = default;

    template <typename U>
    Vector3(const Vector3<U>& rhs);

    Vector3(const sphereapprox::Vertex&);
    Vector3(const sphereapprox::Triangle&);

    operator sphereapprox::Vertex() const;

private:
    std::array<T, 3> m_data{ T(0.0) };
};

typedef sphereapprox::Vector3<double> Vect3;

struct BoundsAABB
{
    BoundsAABB() = default;
    BoundsAABB(const std::vector<sphereapprox::Vertex>& points);
    BoundsAABB(const Vect3& min, const Vect3& max);

    BoundsAABB Union(const BoundsAABB& b);

    bool Intersects(const BoundsAABB& b) const;

    double SurfaceArea() const;
    double Volume() const;

    BoundsAABB Inflate(double ratio) const;

    sphereapprox::Vect3 ClosestPoint(const sphereapprox::Vect3& p) const;

    sphereapprox::Vect3& GetMin();
    sphereapprox::Vect3& GetMax();
    const sphereapprox::Vect3& GetMin() const;
    const sphereapprox::Vect3& GetMax() const;

    sphereapprox::Vect3 GetSize() const;
    sphereapprox::Vect3 GetCenter() const;

    sphereapprox::Vect3 m_min{ double(0.0) };
    sphereapprox::Vect3 m_max{ double(0.0) };
};


/*
 * Out of line definitions
 */

template <typename T>
T clamp(const T& v, const T& lo, const T& hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/*
 * Getters
 */
template <typename T>
inline T& Vector3<T>::operator[](size_t i)
{
    return m_data[i];
}

template <typename T>
inline const T& Vector3<T>::operator[](size_t i) const
{
    return m_data[i];
}

template <typename T>
inline T& Vector3<T>::GetX()
{
    return m_data[0];
}

template <typename T>
inline T& Vector3<T>::GetY()
{
    return m_data[1];
}

template <typename T>
inline T& Vector3<T>::GetZ()
{
    return m_data[2];
}

template <typename T>
inline const T& Vector3<T>::GetX() const
{
    return m_data[0];
}

template <typename T>
inline const T& Vector3<T>::GetY() const
{
    return m_data[1];
}

template <typename T>
inline const T& Vector3<T>::GetZ() const
{
    return m_data[2];
}

/*
 * Normalize and norming
 */
template <typename T>
inline T Vector3<T>::Normalize()
{
    T n = GetNorm();
    if (n != T(0.0))
        (*this) /= n;
    return n;
}

template <typename T>
inline Vector3<T> Vector3<T>::Normalized()
{
    Vector3<T> ret = *this;
    T n = GetNorm();
    if (n != T(0.0))
        ret /= n;
    return ret;
}

template <typename T>
inline T Vector3<T>::GetNorm() const
{
    return std::sqrt(GetNormSquared());
}

template <typename T>
inline T Vector3<T>::GetNormSquared() const
{
    return this->Dot(*this);
}

template <typename T>
inline int Vector3<T>::LongestAxis() const
{
    auto it = std::max_element(m_data.begin(), m_data.end());
    return int(std::distance(m_data.begin(), it));
}

/*
 * Vector-vector operations
 */
template <typename T>
inline Vector3<T>& Vector3<T>::operator=(const Vector3<T>& rhs)
{
    GetX() = rhs.GetX();
    GetY() = rhs.GetY();
    GetZ() = rhs.GetZ();
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator+=(const Vector3<T>& rhs)
{
    GetX() += rhs.GetX();
    GetY() += rhs.GetY();
    GetZ() += rhs.GetZ();
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator-=(const Vector3<T>& rhs)
{
    GetX() -= rhs.GetX();
    GetY() -= rhs.GetY();
    GetZ() -= rhs.GetZ();
    return *this;
}

template <typename T>
inline Vector3<T> Vector3<T>::CWiseMul(const Vector3<T>& rhs) const
{
    return Vector3<T>(GetX() * rhs.GetX(), GetY() * rhs.GetY(), GetZ() * rhs.GetZ());
}

template <typename T>
inline Vector3<T> Vector3<T>::Cross(const Vector3<T>& rhs) const
{
    return Vector3<T>(GetY() * rhs.GetZ() - GetZ() * rhs.GetY(), GetZ() * rhs.GetX() - GetX() * rhs.GetZ(),
                      GetX() * rhs.GetY() - GetY() * rhs.GetX());
}

template <typename T>
inline T Vector3<T>::Dot(const Vector3<T>& rhs) const
{
    return GetX() * rhs.GetX() + GetY() * rhs.GetY() + GetZ() * rhs.GetZ();
}

template <typename T>
inline Vector3<T> Vector3<T>::operator+(const Vector3<T>& rhs) const
{
    return Vector3<T>(GetX() + rhs.GetX(), GetY() + rhs.GetY(), GetZ() + rhs.GetZ());
}

template <typename T>
inline Vector3<T> Vector3<T>::operator-(const Vector3<T>& rhs) const
{
    return Vector3<T>(GetX() - rhs.GetX(), GetY() - rhs.GetY(), GetZ() - rhs.GetZ());
}

template <typename T>
inline Vector3<T> operator*(T lhs, const Vector3<T>& rhs)
{
    return Vector3<T>(lhs * rhs.GetX(), lhs * rhs.GetY(), lhs * rhs.GetZ());
}

/*
 * Vector-scalar operations
 */
template <typename T>
inline Vector3<T>& Vector3<T>::operator-=(T a)
{
    GetX() -= a;
    GetY() -= a;
    GetZ() -= a;
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator+=(T a)
{
    GetX() += a;
    GetY() += a;
    GetZ() += a;
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator/=(T a)
{
    GetX() /= a;
    GetY() /= a;
    GetZ() /= a;
    return *this;
}

template <typename T>
inline Vector3<T>& Vector3<T>::operator*=(T a)
{
    GetX() *= a;
    GetY() *= a;
    GetZ() *= a;
    return *this;
}

template <typename T>
inline Vector3<T> Vector3<T>::operator*(T rhs) const
{
    return Vector3<T>(GetX() * rhs, GetY() * rhs, GetZ() * rhs);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator/(T rhs) const
{
    return Vector3<T>(GetX() / rhs, GetY() / rhs, GetZ() / rhs);
}

/*
 * Unary operations
 */
template <typename T>
inline Vector3<T> Vector3<T>::operator-() const
{
    return Vector3<T>(-GetX(), -GetY(), -GetZ());
}

/*
 * Comparison operators
 */
template <typename T>
inline bool Vector3<T>::operator<(const Vector3<T>& rhs) const
{
    if (GetX() == rhs.GetX())
    {
        if (GetY() == rhs.GetY())
        {
            return (GetZ() < rhs.GetZ());
        }
        return (GetY() < rhs.GetY());
    }
    return (GetX() < rhs.GetX());
}

template <typename T>
inline bool Vector3<T>::operator>(const Vector3<T>& rhs) const
{
    if (GetX() == rhs.GetX())
    {
        if (GetY() == rhs.GetY())
        {
            return (GetZ() > rhs.GetZ());
        }
        return (GetY() > rhs.GetY());
    }
    return (GetX() > rhs.GetZ());
}

template <typename T>
inline bool Vector3<T>::CWiseAllGE(const Vector3<T>& rhs) const
{
    return GetX() >= rhs.GetX() && GetY() >= rhs.GetY() && GetZ() >= rhs.GetZ();
}

template <typename T>
inline bool Vector3<T>::CWiseAllLE(const Vector3<T>& rhs) const
{
    return GetX() <= rhs.GetX() && GetY() <= rhs.GetY() && GetZ() <= rhs.GetZ();
}

template <typename T>
inline Vector3<T> Vector3<T>::CWiseMin(const Vector3<T>& rhs) const
{
    return Vector3<T>(std::min(GetX(), rhs.GetX()), std::min(GetY(), rhs.GetY()), std::min(GetZ(), rhs.GetZ()));
}

template <typename T>
inline Vector3<T> Vector3<T>::CWiseMax(const Vector3<T>& rhs) const
{
    return Vector3<T>(std::max(GetX(), rhs.GetX()), std::max(GetY(), rhs.GetY()), std::max(GetZ(), rhs.GetZ()));
}

template <typename T>
inline T Vector3<T>::MinCoeff() const
{
    return *std::min_element(m_data.begin(), m_data.end());
}

template <typename T>
inline T Vector3<T>::MaxCoeff() const
{
    return *std::max_element(m_data.begin(), m_data.end());
}

template <typename T>
inline T Vector3<T>::MinCoeff(uint32_t& idx) const
{
    auto it = std::min_element(m_data.begin(), m_data.end());
    idx = uint32_t(std::distance(m_data.begin(), it));
    return *it;
}

template <typename T>
inline T Vector3<T>::MaxCoeff(uint32_t& idx) const
{
    auto it = std::max_element(m_data.begin(), m_data.end());
    idx = uint32_t(std::distance(m_data.begin(), it));
    return *it;
}

/*
 * Constructors
 */
template <typename T>
inline Vector3<T>::Vector3(T a) : m_data{ a, a, a }
{
}

template <typename T>
inline Vector3<T>::Vector3(T x, T y, T z) : m_data{ x, y, z }
{
}

template <typename T>
inline Vector3<T>::Vector3(const Vector3& rhs) : m_data{ rhs.m_data }
{
}

template <typename T>
template <typename U>
inline Vector3<T>::Vector3(const Vector3<U>& rhs) : m_data{ T(rhs.GetX()), T(rhs.GetY()), T(rhs.GetZ()) }
{
}

template <typename T>
inline Vector3<T>::Vector3(const sphereapprox::Vertex& rhs) : Vector3<T>(rhs.mX, rhs.mY, rhs.mZ)
{
    static_assert(std::is_same<T, double>::value, "Vertex to Vector3 constructor only enabled for double");
}

template <typename T>
inline Vector3<T>::Vector3(const sphereapprox::Triangle& rhs) : Vector3<T>(rhs.mI0, rhs.mI1, rhs.mI2)
{
    static_assert(std::is_same<T, uint32_t>::value, "Triangle to Vector3 constructor only enabled for uint32_t");
}

template <typename T>
inline Vector3<T>::operator sphereapprox::Vertex() const
{
    static_assert(std::is_same<T, double>::value, "Vector3 to Vertex conversion only enable for double");
    return ::sphereapprox::Vertex(GetX(), GetY(), GetZ());
}

inline BoundsAABB::BoundsAABB(const std::vector<sphereapprox::Vertex>& points) : m_min(points[0]), m_max(points[0])
{
    for (uint32_t i = 1; i < points.size(); ++i)
    {
        const sphereapprox::Vertex& p = points[i];
        m_min = m_min.CWiseMin(p);
        m_max = m_max.CWiseMax(p);
    }
}

inline BoundsAABB::BoundsAABB(const sphereapprox::Vect3& min, const sphereapprox::Vect3& max) : m_min(min), m_max(max)
{
}

BoundsAABB BoundsAABB::Union(const BoundsAABB& b)
{
    return BoundsAABB(GetMin().CWiseMin(b.GetMin()), GetMax().CWiseMax(b.GetMax()));
}

inline bool sphereapprox::BoundsAABB::Intersects(const sphereapprox::BoundsAABB& b) const
{
    if ((GetMin().GetX() > b.GetMax().GetX()) || (b.GetMin().GetX() > GetMax().GetX()))
        return false;
    if ((GetMin().GetY() > b.GetMax().GetY()) || (b.GetMin().GetY() > GetMax().GetY()))
        return false;
    if ((GetMin().GetZ() > b.GetMax().GetZ()) || (b.GetMin().GetZ() > GetMax().GetZ()))
        return false;
    return true;
}

double BoundsAABB::SurfaceArea() const
{
    sphereapprox::Vect3 d = GetMax() - GetMin();
    return double(2.0) * (d.GetX() * d.GetY() + d.GetX() * d.GetZ() + d.GetY() * d.GetZ());
}

inline double sphereapprox::BoundsAABB::Volume() const
{
    sphereapprox::Vect3 d = GetMax() - GetMin();
    return d.GetX() * d.GetY() * d.GetZ();
}

inline BoundsAABB sphereapprox::BoundsAABB::Inflate(double ratio) const
{
    double inflate = (GetMin() - GetMax()).GetNorm() * double(0.5) * ratio;
    return BoundsAABB(GetMin() - inflate, GetMax() + inflate);
}

inline sphereapprox::Vect3 sphereapprox::BoundsAABB::ClosestPoint(const sphereapprox::Vect3& p) const
{
    return p.CWiseMax(GetMin()).CWiseMin(GetMax());
}

inline sphereapprox::Vect3& sphereapprox::BoundsAABB::GetMin()
{
    return m_min;
}

inline sphereapprox::Vect3& sphereapprox::BoundsAABB::GetMax()
{
    return m_max;
}

inline const sphereapprox::Vect3& sphereapprox::BoundsAABB::GetMin() const
{
    return m_min;
}

inline const sphereapprox::Vect3& sphereapprox::BoundsAABB::GetMax() const
{
    return m_max;
}

inline sphereapprox::Vect3 sphereapprox::BoundsAABB::GetSize() const
{
    return GetMax() - GetMin();
}

inline sphereapprox::Vect3 sphereapprox::BoundsAABB::GetCenter() const
{
    return (GetMin() + GetMax()) * double(0.5);
}

/*
 * Relies on three way comparison, which std::sort doesn't use
 */
template <class T, class dCompareKey>
void Sort(T* const array, int elements)
{
    const int batchSize = 8;
    int stack[1024][2];

    stack[0][0] = 0;
    stack[0][1] = elements - 1;
    int stackIndex = 1;
    const dCompareKey comparator;
    while (stackIndex)
    {
        stackIndex--;
        int lo = stack[stackIndex][0];
        int hi = stack[stackIndex][1];
        if ((hi - lo) > batchSize)
        {
            int mid = (lo + hi) >> 1;
            if (comparator.Compare(array[lo], array[mid]) > 0)
            {
                std::swap(array[lo], array[mid]);
            }
            if (comparator.Compare(array[mid], array[hi]) > 0)
            {
                std::swap(array[mid], array[hi]);
            }
            if (comparator.Compare(array[lo], array[mid]) > 0)
            {
                std::swap(array[lo], array[mid]);
            }
            int i = lo + 1;
            int j = hi - 1;
            const T pivot(array[mid]);
            do
            {
                while (comparator.Compare(array[i], pivot) < 0)
                {
                    i++;
                }
                while (comparator.Compare(array[j], pivot) > 0)
                {
                    j--;
                }

                if (i <= j)
                {
                    std::swap(array[i], array[j]);
                    i++;
                    j--;
                }
            } while (i <= j);

            if (i < hi)
            {
                stack[stackIndex][0] = i;
                stack[stackIndex][1] = hi;
                stackIndex++;
            }
            if (lo < j)
            {
                stack[stackIndex][0] = lo;
                stack[stackIndex][1] = j;
                stackIndex++;
            }
            assert(stackIndex < int(sizeof(stack) / (2 * sizeof(stack[0][0]))));
        }
    }

    int stride = batchSize + 1;
    if (elements < stride)
    {
        stride = elements;
    }
    for (int i = 1; i < stride; ++i)
    {
        if (comparator.Compare(array[0], array[i]) > 0)
        {
            std::swap(array[0], array[i]);
        }
    }

    for (int i = 1; i < elements; ++i)
    {
        int j = i;
        const T tmp(array[i]);
        for (; comparator.Compare(array[j - 1], tmp) > 0; --j)
        {
            assert(j > 0);
            array[j] = array[j - 1];
        }
        array[j] = tmp;
    }
}

/*
Maintaining comment due to attribution
Purpose:

TRIANGLE_AREA_3D computes the area of a triangle in 3D.

Modified:

22 April 1999

Author:

John Burkardt

Parameters:

Input, double X1, Y1, Z1, X2, Y2, Z2, X3, Y3, Z3, the (getX,getY,getZ)
coordinates of the corners of the triangle.

Output, double TRIANGLE_AREA_3D, the area of the triangle.
*/
double ComputeArea(const sphereapprox::Vect3& p1, const sphereapprox::Vect3& p2, const sphereapprox::Vect3& p3)
{
    /*
    Find the projection of (P3-P1) onto (P2-P1).
    */
    double base = (p2 - p1).GetNorm();
    /*
    The height of the triangle is the length of (P3-P1) after its
    projection onto (P2-P1) has been subtracted.
    */
    double height;
    if (base == double(0.0))
    {
        height = double(0.0);
    }
    else
    {
        double dot = (p3 - p1).Dot(p2 - p1);
        double alpha = dot / (base * base);

        sphereapprox::Vect3 a = p3 - p1 - alpha * (p2 - p1);
        height = a.GetNorm();
    }

    return double(0.5) * base * height;
}

bool ComputeCentroid(const std::vector<sphereapprox::Vertex>& points,
                     const std::vector<sphereapprox::Triangle>& indices,
                     sphereapprox::Vect3& center)

{
    bool ret = false;
    if (points.size())
    {
        center = sphereapprox::Vect3(0);

        sphereapprox::Vect3 numerator(0);
        double denominator = 0;

        for (uint32_t i = 0; i < indices.size(); i++)
        {
            uint32_t i1 = indices[i].mI0;
            uint32_t i2 = indices[i].mI1;
            uint32_t i3 = indices[i].mI2;

            const sphereapprox::Vect3& p1 = points[i1];
            const sphereapprox::Vect3& p2 = points[i2];
            const sphereapprox::Vect3& p3 = points[i3];

            // Compute the average of the sum of the three positions
            sphereapprox::Vect3 sum = (p1 + p2 + p3) / 3;

            // Compute the area of this triangle
            double area = ComputeArea(p1, p2, p3);

            numerator += (sum * area);

            denominator += area;
        }
        double recip = 1 / denominator;
        center = numerator * recip;
        ret = true;
    }
    return ret;
}

double Determinant3x3(const std::array<sphereapprox::Vect3, 3>& matrix, double& error)
{
    double det = double(0.0);
    error = double(0.0);

    double a01xa12 = matrix[0].GetY() * matrix[1].GetZ();
    double a02xa11 = matrix[0].GetZ() * matrix[1].GetY();
    error += (std::abs(a01xa12) + std::abs(a02xa11)) * std::abs(matrix[2].GetX());
    det += (a01xa12 - a02xa11) * matrix[2].GetX();

    double a00xa12 = matrix[0].GetX() * matrix[1].GetZ();
    double a02xa10 = matrix[0].GetZ() * matrix[1].GetX();
    error += (std::abs(a00xa12) + std::abs(a02xa10)) * std::abs(matrix[2].GetY());
    det -= (a00xa12 - a02xa10) * matrix[2].GetY();

    double a00xa11 = matrix[0].GetX() * matrix[1].GetY();
    double a01xa10 = matrix[0].GetY() * matrix[1].GetX();
    error += (std::abs(a00xa11) + std::abs(a01xa10)) * std::abs(matrix[2].GetZ());
    det += (a00xa11 - a01xa10) * matrix[2].GetZ();

    return det;
}

double ComputeMeshVolume(const std::vector<sphereapprox::Vertex>& vertices,
                         const std::vector<sphereapprox::Triangle>& indices)
{
    double volume = 0;
    for (uint32_t i = 0; i < indices.size(); i++)
    {
        const std::array<sphereapprox::Vect3, 3> m = { vertices[indices[i].mI0], vertices[indices[i].mI1],
                                                       vertices[indices[i].mI2] };
        double placeholder;
        volume += Determinant3x3(m, placeholder);
    }

    volume *= (double(1.0) / double(6.0));
    if (volume < 0)
        volume *= -1;
    return volume;
}


class KdTreeNode;

enum Axes
{
    X_AXIS = 0,
    Y_AXIS = 1,
    Z_AXIS = 2
};

class KdTreeFindNode
{
public:
    KdTreeFindNode() = default;

    KdTreeNode* m_node{ nullptr };
    double m_distance{ 0.0 };
};

/*
 * To minimize memory allocations while maintaining pointer stability.
 * Used in KdTreeNode and ConvexHull, as both use tree data structures that rely on pointer stability
 * Neither rely on random access or iteration
 * They just dump elements into a memory pool, then refer to pointers to the elements
 * All elements are default constructed in NodeStorage's m_nodes array
 */
template <typename T, std::size_t MaxBundleSize = 1024>
class NodeBundle
{
    struct NodeStorage
    {
        bool IsFull() const;

        T& GetNextNode();

        std::size_t m_index;
        std::array<T, MaxBundleSize> m_nodes;
    };

    std::list<NodeStorage> m_list;
    typename std::list<NodeStorage>::iterator m_head{ m_list.end() };

public:
    T& GetNextNode();

    T& GetFirstNode();

    void Clear();
};

template <typename T, std::size_t MaxBundleSize>
bool NodeBundle<T, MaxBundleSize>::NodeStorage::IsFull() const
{
    return m_index == MaxBundleSize;
}

template <typename T, std::size_t MaxBundleSize>
T& NodeBundle<T, MaxBundleSize>::NodeStorage::GetNextNode()
{
    assert(m_index < MaxBundleSize);
    T& ret = m_nodes[m_index];
    m_index++;
    return ret;
}

template <typename T, std::size_t MaxBundleSize>
T& NodeBundle<T, MaxBundleSize>::GetNextNode()
{
    /*
     * || short circuits, so doesn't dereference if m_bundle == m_bundleHead.end()
     */
    if (m_head == m_list.end() || m_head->IsFull())
    {
        m_head = m_list.emplace(m_list.end());
    }

    return m_head->GetNextNode();
}

template <typename T, std::size_t MaxBundleSize>
T& NodeBundle<T, MaxBundleSize>::GetFirstNode()
{
    assert(m_head != m_list.end());
    return m_list.front().m_nodes[0];
}

template <typename T, std::size_t MaxBundleSize>
void NodeBundle<T, MaxBundleSize>::Clear()
{
    m_list.clear();
}

class KdTree
{
public:
    KdTree() = default;

    const sphereapprox::Vertex& GetPosition(uint32_t index) const;

    uint32_t Search(const sphereapprox::Vect3& pos, double radius, uint32_t maxObjects, KdTreeFindNode* found) const;

    uint32_t Add(const sphereapprox::Vertex& v);

    KdTreeNode& GetNewNode(uint32_t index);

    uint32_t GetNearest(const sphereapprox::Vect3& pos,
                        double radius,
                        bool& _found) const; // returns the nearest possible neighbor's index.

    const std::vector<sphereapprox::Vertex>& GetVertices() const;
    std::vector<sphereapprox::Vertex>&& TakeVertices();

    uint32_t GetVCount() const;

private:
    KdTreeNode* m_root{ nullptr };
    NodeBundle<KdTreeNode> m_bundle;

    std::vector<sphereapprox::Vertex> m_vertices;
};

class KdTreeNode
{
public:
    KdTreeNode() = default;
    KdTreeNode(uint32_t index);

    void Add(KdTreeNode& node, Axes dim, const KdTree& iface);

    uint32_t GetIndex() const;

    void Search(Axes axis,
                const sphereapprox::Vect3& pos,
                double radius,
                uint32_t& count,
                uint32_t maxObjects,
                KdTreeFindNode* found,
                const KdTree& iface);

private:
    uint32_t m_index = 0;
    KdTreeNode* m_left = nullptr;
    KdTreeNode* m_right = nullptr;
};

const sphereapprox::Vertex& KdTree::GetPosition(uint32_t index) const
{
    assert(index < m_vertices.size());
    return m_vertices[index];
}

uint32_t KdTree::Search(const sphereapprox::Vect3& pos, double radius, uint32_t maxObjects, KdTreeFindNode* found) const
{
    if (!m_root)
        return 0;
    uint32_t count = 0;
    m_root->Search(X_AXIS, pos, radius, count, maxObjects, found, *this);
    return count;
}

uint32_t KdTree::Add(const sphereapprox::Vertex& v)
{
    uint32_t ret = uint32_t(m_vertices.size());
    m_vertices.emplace_back(v);
    KdTreeNode& node = GetNewNode(ret);
    if (m_root)
    {
        m_root->Add(node, X_AXIS, *this);
    }
    else
    {
        m_root = &node;
    }
    return ret;
}

KdTreeNode& KdTree::GetNewNode(uint32_t index)
{
    KdTreeNode& node = m_bundle.GetNextNode();
    node = KdTreeNode(index);
    return node;
}

uint32_t KdTree::GetNearest(const sphereapprox::Vect3& pos,
                            double radius,
                            bool& _found) const // returns the nearest possible neighbor's index.
{
    uint32_t ret = 0;

    _found = false;
    KdTreeFindNode found;
    uint32_t count = Search(pos, radius, 1, &found);
    if (count)
    {
        KdTreeNode* node = found.m_node;
        ret = node->GetIndex();
        _found = true;
    }
    return ret;
}

const std::vector<sphereapprox::Vertex>& KdTree::GetVertices() const
{
    return m_vertices;
}

std::vector<sphereapprox::Vertex>&& KdTree::TakeVertices()
{
    return std::move(m_vertices);
}

uint32_t KdTree::GetVCount() const
{
    return uint32_t(m_vertices.size());
}

KdTreeNode::KdTreeNode(uint32_t index) : m_index(index)
{
}

void KdTreeNode::Add(KdTreeNode& node, Axes dim, const KdTree& tree)
{
    Axes axis = X_AXIS;
    uint32_t idx = 0;
    switch (dim)
    {
    case X_AXIS:
        idx = 0;
        axis = Y_AXIS;
        break;
    case Y_AXIS:
        idx = 1;
        axis = Z_AXIS;
        break;
    case Z_AXIS:
        idx = 2;
        axis = X_AXIS;
        break;
    }

    const sphereapprox::Vertex& nodePosition = tree.GetPosition(node.m_index);
    const sphereapprox::Vertex& position = tree.GetPosition(m_index);
    if (nodePosition[idx] <= position[idx])
    {
        if (m_left)
            m_left->Add(node, axis, tree);
        else
            m_left = &node;
    }
    else
    {
        if (m_right)
            m_right->Add(node, axis, tree);
        else
            m_right = &node;
    }
}

uint32_t KdTreeNode::GetIndex() const
{
    return m_index;
}

void KdTreeNode::Search(Axes axis,
                        const sphereapprox::Vect3& pos,
                        double radius,
                        uint32_t& count,
                        uint32_t maxObjects,
                        KdTreeFindNode* found,
                        const KdTree& iface)
{
    const sphereapprox::Vect3 position = iface.GetPosition(m_index);

    const sphereapprox::Vect3 d = pos - position;

    KdTreeNode* search1 = 0;
    KdTreeNode* search2 = 0;

    uint32_t idx = 0;
    switch (axis)
    {
    case X_AXIS:
        idx = 0;
        axis = Y_AXIS;
        break;
    case Y_AXIS:
        idx = 1;
        axis = Z_AXIS;
        break;
    case Z_AXIS:
        idx = 2;
        axis = X_AXIS;
        break;
    }

    if (d[idx] <= 0) // JWR  if we are to the left
    {
        search1 = m_left; // JWR  then search to the left
        if (-d[idx] < radius) // JWR  if distance to the right is less than our search radius, continue on the right
                              // as well.
            search2 = m_right;
    }
    else
    {
        search1 = m_right; // JWR  ok, we go down the left tree
        if (d[idx] < radius) // JWR  if the distance from the right is less than our search radius
            search2 = m_left;
    }

    double r2 = radius * radius;
    double m = d.GetNormSquared();

    if (m < r2)
    {
        switch (count)
        {
        case 0: {
            found[count].m_node = this;
            found[count].m_distance = m;
            break;
        }
        case 1: {
            if (m < found[0].m_distance)
            {
                if (maxObjects == 1)
                {
                    found[0].m_node = this;
                    found[0].m_distance = m;
                }
                else
                {
                    found[1] = found[0];
                    found[0].m_node = this;
                    found[0].m_distance = m;
                }
            }
            else if (maxObjects > 1)
            {
                found[1].m_node = this;
                found[1].m_distance = m;
            }
            break;
        }
        default: {
            bool inserted = false;

            for (uint32_t i = 0; i < count; i++)
            {
                if (m < found[i].m_distance) // if this one is closer than a pre-existing one...
                {
                    // insertion sort...
                    uint32_t scan = count;
                    if (scan >= maxObjects)
                        scan = maxObjects - 1;
                    for (uint32_t j = scan; j > i; j--)
                    {
                        found[j] = found[j - 1];
                    }
                    found[i].m_node = this;
                    found[i].m_distance = m;
                    inserted = true;
                    break;
                }
            }

            if (!inserted && count < maxObjects)
            {
                found[count].m_node = this;
                found[count].m_distance = m;
            }
        }
        break;
        }

        count++;

        if (count > maxObjects)
        {
            count = maxObjects;
        }
    }


    if (search1)
        search1->Search(axis, pos, radius, count, maxObjects, found, iface);

    if (search2)
        search2->Search(axis, pos, radius, count, maxObjects, found, iface);
}

class VertexIndex
{
public:
    VertexIndex(double granularity, bool snapToGrid);

    sphereapprox::Vect3 SnapToGrid(sphereapprox::Vect3 p);

    uint32_t GetIndex(sphereapprox::Vect3 p, bool& newPos);

    const std::vector<sphereapprox::Vertex>& GetVertices() const;

    std::vector<sphereapprox::Vertex>&& TakeVertices();

    uint32_t GetVCount() const;

    bool SaveAsObj(const char* fname, uint32_t tcount, uint32_t* indices)
    {
        bool ret = false;

        FILE* fph = fopen(fname, "wb");
        if (fph)
        {
            ret = true;

            const std::vector<sphereapprox::Vertex>& v = GetVertices();
            for (uint32_t i = 0; i < v.size(); ++i)
            {
                fprintf(fph, "v %0.9f %0.9f %0.9f\r\n", v[i].mX, v[i].mY, v[i].mZ);
            }

            for (uint32_t i = 0; i < tcount; i++)
            {
                uint32_t i1 = *indices++;
                uint32_t i2 = *indices++;
                uint32_t i3 = *indices++;
                fprintf(fph, "f %d %d %d\r\n", i1 + 1, i2 + 1, i3 + 1);
            }
            fclose(fph);
        }

        return ret;
    }

private:
    bool m_snapToGrid : 1;
    double m_granularity;
    KdTree m_KdTree;
};

VertexIndex::VertexIndex(double granularity, bool snapToGrid) : m_snapToGrid(snapToGrid), m_granularity(granularity)
{
}

sphereapprox::Vect3 VertexIndex::SnapToGrid(sphereapprox::Vect3 p)
{
    for (int i = 0; i < 3; ++i)
    {
        double m = fmod(p[i], m_granularity);
        p[i] -= m;
    }
    return p;
}

uint32_t VertexIndex::GetIndex(sphereapprox::Vect3 p, bool& newPos)
{
    uint32_t ret;

    newPos = false;

    if (m_snapToGrid)
    {
        p = SnapToGrid(p);
    }

    bool found;
    ret = m_KdTree.GetNearest(p, m_granularity, found);
    if (!found)
    {
        newPos = true;
        ret = m_KdTree.Add(sphereapprox::Vertex(p.GetX(), p.GetY(), p.GetZ()));
    }

    return ret;
}

const std::vector<sphereapprox::Vertex>& VertexIndex::GetVertices() const
{
    return m_KdTree.GetVertices();
}

std::vector<sphereapprox::Vertex>&& VertexIndex::TakeVertices()
{
    return std::move(m_KdTree.TakeVertices());
}

uint32_t VertexIndex::GetVCount() const
{
    return m_KdTree.GetVCount();
}

/*
 * A wrapper class for 3 10 bit integers packed into a 32 bit integer
 * Layout is [PAD][X][Y][Z]
 * Pad is bits 31-30, X is 29-20, Y is 19-10, and Z is 9-0
 */
class Voxel
{
    /*
     * Specify all of them for consistency
     */
    static constexpr int VoxelBitsZStart = 0;
    static constexpr int VoxelBitsYStart = 10;
    static constexpr int VoxelBitsXStart = 20;
    static constexpr int VoxelBitMask = 0x03FF; // bits 0 through 9 inclusive
public:
    Voxel() = default;

    Voxel(uint32_t index);

    Voxel(uint32_t x, uint32_t y, uint32_t z);

    bool operator==(const Voxel& v) const;

    sphereapprox::Vector3<uint32_t> GetVoxel() const;

    uint32_t GetX() const;
    uint32_t GetY() const;
    uint32_t GetZ() const;

    uint32_t GetVoxelAddress() const;

private:
    uint32_t m_voxel{ 0 };
};

Voxel::Voxel(uint32_t index) : m_voxel(index)
{
}

Voxel::Voxel(uint32_t x, uint32_t y, uint32_t z)
    : m_voxel((x << VoxelBitsXStart) | (y << VoxelBitsYStart) | (z << VoxelBitsZStart))
{
    assert(x < 1024 && "Voxel constructed with X outside of range");
    assert(y < 1024 && "Voxel constructed with Y outside of range");
    assert(z < 1024 && "Voxel constructed with Z outside of range");
}

bool Voxel::operator==(const Voxel& v) const
{
    return m_voxel == v.m_voxel;
}

sphereapprox::Vector3<uint32_t> Voxel::GetVoxel() const
{
    return sphereapprox::Vector3<uint32_t>(GetX(), GetY(), GetZ());
}

uint32_t Voxel::GetX() const
{
    return (m_voxel >> VoxelBitsXStart) & VoxelBitMask;
}

uint32_t Voxel::GetY() const
{
    return (m_voxel >> VoxelBitsYStart) & VoxelBitMask;
}

uint32_t Voxel::GetZ() const
{
    return (m_voxel >> VoxelBitsZStart) & VoxelBitMask;
}

uint32_t Voxel::GetVoxelAddress() const
{
    return m_voxel;
}

struct SimpleMesh
{
    std::vector<sphereapprox::Vertex> m_vertices;
    std::vector<sphereapprox::Triangle> m_indices;
};

/*======================== 0-tests ========================*/
inline bool IntersectRayAABB(const sphereapprox::Vect3& start,
                             const sphereapprox::Vect3& dir,
                             const sphereapprox::BoundsAABB& bounds,
                             double& t)
{
    //! calculate candidate plane on each axis
    bool inside = true;
    sphereapprox::Vect3 ta(double(-1.0));

    //! use unrolled loops
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (start[i] < bounds.GetMin()[i])
        {
            if (dir[i] != double(0.0))
                ta[i] = (bounds.GetMin()[i] - start[i]) / dir[i];
            inside = false;
        }
        else if (start[i] > bounds.GetMax()[i])
        {
            if (dir[i] != double(0.0))
                ta[i] = (bounds.GetMax()[i] - start[i]) / dir[i];
            inside = false;
        }
    }

    //! if point inside all planes
    if (inside)
    {
        t = double(0.0);
        return true;
    }

    //! we now have t values for each of possible intersection planes
    //! find the maximum to get the intersection point
    uint32_t taxis;
    double tmax = ta.MaxCoeff(taxis);

    if (tmax < double(0.0))
        return false;

    //! check that the intersection point lies on the plane we picked
    //! we don't test the axis of closest intersection for precision reasons

    //! no eps for now
    double eps = double(0.0);

    sphereapprox::Vect3 hit = start + dir * tmax;

    if ((hit.GetX() < bounds.GetMin().GetX() - eps || hit.GetX() > bounds.GetMax().GetX() + eps) && taxis != 0)
        return false;
    if ((hit.GetY() < bounds.GetMin().GetY() - eps || hit.GetY() > bounds.GetMax().GetY() + eps) && taxis != 1)
        return false;
    if ((hit.GetZ() < bounds.GetMin().GetZ() - eps || hit.GetZ() > bounds.GetMax().GetZ() + eps) && taxis != 2)
        return false;

    //! output results
    t = tmax;

    return true;
}

// Moller and Trumbore's method
inline bool IntersectRayTriTwoSided(const sphereapprox::Vect3& p,
                                    const sphereapprox::Vect3& dir,
                                    const sphereapprox::Vect3& a,
                                    const sphereapprox::Vect3& b,
                                    const sphereapprox::Vect3& c,
                                    double& t,
                                    double& u,
                                    double& v,
                                    double& w,
                                    double& sign,
                                    sphereapprox::Vect3* normal)
{
    sphereapprox::Vect3 ab = b - a;
    sphereapprox::Vect3 ac = c - a;
    sphereapprox::Vect3 n = ab.Cross(ac);

    double d = -dir.Dot(n);
    double ood = double(1.0) / d; // No need to check for division by zero here as infinity arithmetic will save us...
    sphereapprox::Vect3 ap = p - a;

    t = ap.Dot(n) * ood;
    if (t < double(0.0))
    {
        return false;
    }

    sphereapprox::Vect3 e = -dir.Cross(ap);
    v = ac.Dot(e) * ood;
    if (v < double(0.0) || v > double(1.0)) // ...here...
    {
        return false;
    }
    w = -ab.Dot(e) * ood;
    if (w < double(0.0) || v + w > double(1.0)) // ...and here
    {
        return false;
    }

    u = double(1.0) - v - w;
    if (normal)
    {
        *normal = n;
    }

    sign = d;

    return true;
}

// RTCD 5.1.5, page 142
inline sphereapprox::Vect3 ClosestPointOnTriangle(const sphereapprox::Vect3& a,
                                                  const sphereapprox::Vect3& b,
                                                  const sphereapprox::Vect3& c,
                                                  const sphereapprox::Vect3& p,
                                                  double& v,
                                                  double& w)
{
    sphereapprox::Vect3 ab = b - a;
    sphereapprox::Vect3 ac = c - a;
    sphereapprox::Vect3 ap = p - a;

    double d1 = ab.Dot(ap);
    double d2 = ac.Dot(ap);
    if (d1 <= double(0.0) && d2 <= double(0.0))
    {
        v = double(0.0);
        w = double(0.0);
        return a;
    }

    sphereapprox::Vect3 bp = p - b;
    double d3 = ab.Dot(bp);
    double d4 = ac.Dot(bp);
    if (d3 >= double(0.0) && d4 <= d3)
    {
        v = double(1.0);
        w = double(0.0);
        return b;
    }

    double vc = d1 * d4 - d3 * d2;
    if (vc <= double(0.0) && d1 >= double(0.0) && d3 <= double(0.0))
    {
        v = d1 / (d1 - d3);
        w = double(0.0);
        return a + v * ab;
    }

    sphereapprox::Vect3 cp = p - c;
    double d5 = ab.Dot(cp);
    double d6 = ac.Dot(cp);
    if (d6 >= double(0.0) && d5 <= d6)
    {
        v = double(0.0);
        w = double(1.0);
        return c;
    }

    double vb = d5 * d2 - d1 * d6;
    if (vb <= double(0.0) && d2 >= double(0.0) && d6 <= double(0.0))
    {
        v = double(0.0);
        w = d2 / (d2 - d6);
        return a + w * ac;
    }

    double va = d3 * d6 - d5 * d4;
    if (va <= double(0.0) && (d4 - d3) >= double(0.0) && (d5 - d6) >= double(0.0))
    {
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        v = double(1.0) - w;
        return b + w * (c - b);
    }

    double denom = double(1.0) / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    return a + ab * v + ac * w;
}

class AABBTree
{
public:
    AABBTree() = default;
    AABBTree(AABBTree&&) = default;
    AABBTree& operator=(AABBTree&&) = default;

    AABBTree(const std::vector<sphereapprox::Vertex>& vertices, const std::vector<sphereapprox::Triangle>& indices);

    bool TraceRay(const sphereapprox::Vect3& start,
                  const sphereapprox::Vect3& to,
                  double& outT,
                  double& faceSign,
                  sphereapprox::Vect3& hitLocation) const;

    bool TraceRay(const sphereapprox::Vect3& start,
                  const sphereapprox::Vect3& dir,
                  uint32_t& insideCount,
                  uint32_t& outsideCount) const;

    bool TraceRay(const sphereapprox::Vect3& start,
                  const sphereapprox::Vect3& dir,
                  double& outT,
                  double& u,
                  double& v,
                  double& w,
                  double& faceSign,
                  uint32_t& faceIndex) const;

    sphereapprox::Vect3 GetCenter() const;
    sphereapprox::Vect3 GetMinExtents() const;
    sphereapprox::Vect3 GetMaxExtents() const;

    bool GetClosestPointWithinDistance(const sphereapprox::Vect3& point,
                                       double maxDistance,
                                       sphereapprox::Vect3& closestPoint) const;

private:
    struct Node
    {
        union
        {
            uint32_t m_children;
            uint32_t m_numFaces{ 0 };
        };

        uint32_t* m_faces{ nullptr };
        sphereapprox::BoundsAABB m_extents;
    };

    struct FaceSorter
    {
        FaceSorter(const std::vector<sphereapprox::Vertex>& positions,
                   const std::vector<sphereapprox::Triangle>& indices,
                   uint32_t axis);

        bool operator()(uint32_t lhs, uint32_t rhs) const;

        double GetCentroid(uint32_t face) const;

        const std::vector<sphereapprox::Vertex>& m_vertices;
        const std::vector<sphereapprox::Triangle>& m_indices;
        uint32_t m_axis;
    };

    // partition the objects and return the number of objects in the lower partition
    uint32_t PartitionMedian(Node& n, uint32_t* faces, uint32_t numFaces);
    uint32_t PartitionSAH(Node& n, uint32_t* faces, uint32_t numFaces);

    void Build();

    void BuildRecursive(uint32_t nodeIndex, uint32_t* faces, uint32_t numFaces);

    void TraceRecursive(uint32_t nodeIndex,
                        const sphereapprox::Vect3& start,
                        const sphereapprox::Vect3& dir,
                        double& outT,
                        double& u,
                        double& v,
                        double& w,
                        double& faceSign,
                        uint32_t& faceIndex) const;


    bool GetClosestPointWithinDistance(const sphereapprox::Vect3& point,
                                       const double maxDis,
                                       double& dis,
                                       double& v,
                                       double& w,
                                       uint32_t& faceIndex,
                                       sphereapprox::Vect3& closest) const;

    void GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
                                                  const sphereapprox::Vect3& point,
                                                  double& outDisSq,
                                                  double& outV,
                                                  double& outW,
                                                  uint32_t& outFaceIndex,
                                                  sphereapprox::Vect3& closest) const;

    sphereapprox::BoundsAABB CalculateFaceBounds(uint32_t* faces, uint32_t numFaces);

    // track the next free node
    uint32_t m_freeNode;

    const std::vector<sphereapprox::Vertex>* m_vertices{ nullptr };
    const std::vector<sphereapprox::Triangle>* m_indices{ nullptr };

    std::vector<uint32_t> m_faces;
    std::vector<Node> m_nodes;
    std::vector<sphereapprox::BoundsAABB> m_faceBounds;

    // stats
    uint32_t m_treeDepth{ 0 };
    uint32_t m_innerNodes{ 0 };
    uint32_t m_leafNodes{ 0 };

    uint32_t s_depth{ 0 };
};

AABBTree::FaceSorter::FaceSorter(const std::vector<sphereapprox::Vertex>& positions,
                                 const std::vector<sphereapprox::Triangle>& indices,
                                 uint32_t axis)
    : m_vertices(positions), m_indices(indices), m_axis(axis)
{
}

inline bool AABBTree::FaceSorter::operator()(uint32_t lhs, uint32_t rhs) const
{
    double a = GetCentroid(lhs);
    double b = GetCentroid(rhs);

    if (a == b)
    {
        return lhs < rhs;
    }
    else
    {
        return a < b;
    }
}

inline double AABBTree::FaceSorter::GetCentroid(uint32_t face) const
{
    const sphereapprox::Vect3& a = m_vertices[m_indices[face].mI0];
    const sphereapprox::Vect3& b = m_vertices[m_indices[face].mI1];
    const sphereapprox::Vect3& c = m_vertices[m_indices[face].mI2];

    return (a[m_axis] + b[m_axis] + c[m_axis]) / double(3.0);
}

AABBTree::AABBTree(const std::vector<sphereapprox::Vertex>& vertices, const std::vector<sphereapprox::Triangle>& indices)
    : m_vertices(&vertices), m_indices(&indices)
{
    Build();
}

bool AABBTree::TraceRay(const sphereapprox::Vect3& start,
                        const sphereapprox::Vect3& to,
                        double& outT,
                        double& faceSign,
                        sphereapprox::Vect3& hitLocation) const
{
    sphereapprox::Vect3 dir = to - start;
    double distance = dir.Normalize();
    double u, v, w;
    uint32_t faceIndex;
    bool hit = TraceRay(start, dir, outT, u, v, w, faceSign, faceIndex);
    if (hit)
    {
        hitLocation = start + dir * outT;
    }

    if (hit && outT > distance)
    {
        hit = false;
    }
    return hit;
}

bool AABBTree::TraceRay(const sphereapprox::Vect3& start,
                        const sphereapprox::Vect3& dir,
                        uint32_t& insideCount,
                        uint32_t& outsideCount) const
{
    double outT, u, v, w, faceSign;
    uint32_t faceIndex;
    bool hit = TraceRay(start, dir, outT, u, v, w, faceSign, faceIndex);
    if (hit)
    {
        if (faceSign >= 0)
        {
            insideCount++;
        }
        else
        {
            outsideCount++;
        }
    }
    return hit;
}

bool AABBTree::TraceRay(const sphereapprox::Vect3& start,
                        const sphereapprox::Vect3& dir,
                        double& outT,
                        double& u,
                        double& v,
                        double& w,
                        double& faceSign,
                        uint32_t& faceIndex) const
{
    outT = FLT_MAX;
    TraceRecursive(0, start, dir, outT, u, v, w, faceSign, faceIndex);
    return (outT != FLT_MAX);
}

sphereapprox::Vect3 AABBTree::GetCenter() const
{
    return m_nodes[0].m_extents.GetCenter();
}

sphereapprox::Vect3 AABBTree::GetMinExtents() const
{
    return m_nodes[0].m_extents.GetMin();
}

sphereapprox::Vect3 AABBTree::GetMaxExtents() const
{
    return m_nodes[0].m_extents.GetMax();
}

bool AABBTree::GetClosestPointWithinDistance(const sphereapprox::Vect3& point,
                                             double maxDistance,
                                             sphereapprox::Vect3& closestPoint) const
{
    double dis, v, w;
    uint32_t faceIndex;
    bool hit = GetClosestPointWithinDistance(point, maxDistance, dis, v, w, faceIndex, closestPoint);
    return hit;
}

// partition faces around the median face
uint32_t AABBTree::PartitionMedian(Node& n, uint32_t* faces, uint32_t numFaces)
{
    FaceSorter predicate(*m_vertices, *m_indices, n.m_extents.GetSize().LongestAxis());
    std::nth_element(faces, faces + numFaces / 2, faces + numFaces, predicate);

    return numFaces / 2;
}

// partition faces based on the surface area heuristic
uint32_t AABBTree::PartitionSAH(Node&, uint32_t* faces, uint32_t numFaces)
{
    uint32_t bestAxis = 0;
    uint32_t bestIndex = 0;
    double bestCost = FLT_MAX;

    for (uint32_t a = 0; a < 3; ++a)
    {
        // sort faces by centroids
        FaceSorter predicate(*m_vertices, *m_indices, a);
        std::sort(faces, faces + numFaces, predicate);

        // two passes over data to calculate upper and lower bounds
        std::vector<double> cumulativeLower(numFaces);
        std::vector<double> cumulativeUpper(numFaces);

        sphereapprox::BoundsAABB lower;
        sphereapprox::BoundsAABB upper;

        for (uint32_t i = 0; i < numFaces; ++i)
        {
            lower.Union(m_faceBounds[faces[i]]);
            upper.Union(m_faceBounds[faces[numFaces - i - 1]]);

            cumulativeLower[i] = lower.SurfaceArea();
            cumulativeUpper[numFaces - i - 1] = upper.SurfaceArea();
        }

        double invTotalSA = double(1.0) / cumulativeUpper[0];

        // test all split positions
        for (uint32_t i = 0; i < numFaces - 1; ++i)
        {
            double pBelow = cumulativeLower[i] * invTotalSA;
            double pAbove = cumulativeUpper[i] * invTotalSA;

            double cost = double(0.125) + (pBelow * i + pAbove * (numFaces - i));
            if (cost <= bestCost)
            {
                bestCost = cost;
                bestIndex = i;
                bestAxis = a;
            }
        }
    }

    // re-sort by best axis
    FaceSorter predicate(*m_vertices, *m_indices, bestAxis);
    std::sort(faces, faces + numFaces, predicate);

    return bestIndex + 1;
}

void AABBTree::Build()
{
    const uint32_t numFaces = uint32_t(m_indices->size());

    // build initial list of faces
    m_faces.reserve(numFaces);

    // calculate bounds of each face and store
    m_faceBounds.reserve(numFaces);

    std::vector<sphereapprox::BoundsAABB> stack;
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        sphereapprox::BoundsAABB top = CalculateFaceBounds(&i, 1);

        m_faces.push_back(i);
        m_faceBounds.push_back(top);
    }

    m_nodes.reserve(uint32_t(numFaces * double(1.5)));

    // allocate space for all the nodes
    m_freeNode = 1;

    // start building
    BuildRecursive(0, m_faces.data(), numFaces);

    assert(s_depth == 0);
}

void AABBTree::BuildRecursive(uint32_t nodeIndex, uint32_t* faces, uint32_t numFaces)
{
    const uint32_t kMaxFacesPerLeaf = 6;

    // if we've run out of nodes allocate some more
    if (nodeIndex >= m_nodes.size())
    {
        uint32_t s = std::max(uint32_t(double(1.5) * m_nodes.size()), 512U);
        m_nodes.resize(s);
    }

    // a reference to the current node, need to be careful here as this reference may become invalid if array is resized
    Node& n = m_nodes[nodeIndex];

    // track max tree depth
    ++s_depth;
    m_treeDepth = std::max(m_treeDepth, s_depth);

    n.m_extents = CalculateFaceBounds(faces, numFaces);

    // calculate bounds of faces and add node
    if (numFaces <= kMaxFacesPerLeaf)
    {
        n.m_faces = faces;
        n.m_numFaces = numFaces;

        ++m_leafNodes;
    }
    else
    {
        ++m_innerNodes;

        // face counts for each branch
        const uint32_t leftCount = PartitionMedian(n, faces, numFaces);
        // const uint32_t leftCount = PartitionSAH(n, faces, numFaces);
        const uint32_t rightCount = numFaces - leftCount;

        // alloc 2 nodes
        m_nodes[nodeIndex].m_children = m_freeNode;

        // allocate two nodes
        m_freeNode += 2;

        // split faces in half and build each side recursively
        BuildRecursive(m_nodes[nodeIndex].m_children + 0, faces, leftCount);
        BuildRecursive(m_nodes[nodeIndex].m_children + 1, faces + leftCount, rightCount);
    }

    --s_depth;
}

void AABBTree::TraceRecursive(uint32_t nodeIndex,
                              const sphereapprox::Vect3& start,
                              const sphereapprox::Vect3& dir,
                              double& outT,
                              double& outU,
                              double& outV,
                              double& outW,
                              double& faceSign,
                              uint32_t& faceIndex) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == NULL)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        double dist[2] = { FLT_MAX, FLT_MAX };

        IntersectRayAABB(start, dir, leftChild.m_extents, dist[0]);
        IntersectRayAABB(start, dir, rightChild.m_extents, dist[1]);

        uint32_t closest = 0;
        uint32_t furthest = 1;

        if (dist[1] < dist[0])
        {
            closest = 1;
            furthest = 0;
        }

        if (dist[closest] < outT)
        {
            TraceRecursive(node.m_children + closest, start, dir, outT, outU, outV, outW, faceSign, faceIndex);
        }

        if (dist[furthest] < outT)
        {
            TraceRecursive(node.m_children + furthest, start, dir, outT, outU, outV, outW, faceSign, faceIndex);
        }
    }
    else
    {
        double t, u, v, w, s;

        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i];

            const sphereapprox::Vect3& a = (*m_vertices)[(*m_indices)[indexStart].mI0];
            const sphereapprox::Vect3& b = (*m_vertices)[(*m_indices)[indexStart].mI1];
            const sphereapprox::Vect3& c = (*m_vertices)[(*m_indices)[indexStart].mI2];
            if (IntersectRayTriTwoSided(start, dir, a, b, c, t, u, v, w, s, NULL))
            {
                if (t < outT)
                {
                    outT = t;
                    outU = u;
                    outV = v;
                    outW = w;
                    faceSign = s;
                    faceIndex = node.m_faces[i];
                }
            }
        }
    }
}

bool AABBTree::GetClosestPointWithinDistance(const sphereapprox::Vect3& point,
                                             const double maxDis,
                                             double& dis,
                                             double& v,
                                             double& w,
                                             uint32_t& faceIndex,
                                             sphereapprox::Vect3& closest) const
{
    dis = maxDis;
    faceIndex = uint32_t(~0);
    double disSq = dis * dis;

    GetClosestPointWithinDistanceSqRecursive(0, point, disSq, v, w, faceIndex, closest);
    dis = sqrt(disSq);

    return (faceIndex < (~(static_cast<unsigned int>(0))));
}

void AABBTree::GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
                                                        const sphereapprox::Vect3& point,
                                                        double& outDisSq,
                                                        double& outV,
                                                        double& outW,
                                                        uint32_t& outFaceIndex,
                                                        sphereapprox::Vect3& closestPoint) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == nullptr)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        // double dist[2] = { FLT_MAX, FLT_MAX };
        sphereapprox::Vect3 lp = leftChild.m_extents.ClosestPoint(point);
        sphereapprox::Vect3 rp = rightChild.m_extents.ClosestPoint(point);


        uint32_t closest = 0;
        uint32_t furthest = 1;
        double dcSq = (point - lp).GetNormSquared();
        double dfSq = (point - rp).GetNormSquared();
        if (dfSq < dcSq)
        {
            closest = 1;
            furthest = 0;
            std::swap(dfSq, dcSq);
        }

        if (dcSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(
                node.m_children + closest, point, outDisSq, outV, outW, outFaceIndex, closestPoint);
        }

        if (dfSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(
                node.m_children + furthest, point, outDisSq, outV, outW, outFaceIndex, closestPoint);
        }
    }
    else
    {

        double v, w;
        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i];

            const sphereapprox::Vect3& a = (*m_vertices)[(*m_indices)[indexStart].mI0];
            const sphereapprox::Vect3& b = (*m_vertices)[(*m_indices)[indexStart].mI1];
            const sphereapprox::Vect3& c = (*m_vertices)[(*m_indices)[indexStart].mI2];

            sphereapprox::Vect3 cp = ClosestPointOnTriangle(a, b, c, point, v, w);
            double disSq = (cp - point).GetNormSquared();

            if (disSq < outDisSq)
            {
                closestPoint = cp;
                outDisSq = disSq;
                outV = v;
                outW = w;
                outFaceIndex = node.m_faces[i];
            }
        }
    }
}

sphereapprox::BoundsAABB AABBTree::CalculateFaceBounds(uint32_t* faces, uint32_t numFaces)
{
    sphereapprox::Vect3 minExtents(FLT_MAX);
    sphereapprox::Vect3 maxExtents(-FLT_MAX);

    // calculate face bounds
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        sphereapprox::Vect3 a = (*m_vertices)[(*m_indices)[faces[i]].mI0];
        sphereapprox::Vect3 b = (*m_vertices)[(*m_indices)[faces[i]].mI1];
        sphereapprox::Vect3 c = (*m_vertices)[(*m_indices)[faces[i]].mI2];

        minExtents = a.CWiseMin(minExtents);
        maxExtents = a.CWiseMax(maxExtents);

        minExtents = b.CWiseMin(minExtents);
        maxExtents = b.CWiseMax(maxExtents);

        minExtents = c.CWiseMin(minExtents);
        maxExtents = c.CWiseMax(maxExtents);
    }

    return sphereapprox::BoundsAABB(minExtents, maxExtents);
}

enum class VoxelValue : uint8_t
{
    PRIMITIVE_UNDEFINED = 0,
    PRIMITIVE_OUTSIDE_SURFACE_TOWALK = 1,
    PRIMITIVE_OUTSIDE_SURFACE = 2,
    PRIMITIVE_INSIDE_SURFACE = 3,
    PRIMITIVE_ON_SURFACE = 4,
    REPRESENTED = 5 // Indicates that this voxel is already represented in the output spheres
};

class Volume
{
public:
    void Voxelize(const std::vector<sphereapprox::Vertex>& points,
                  const std::vector<sphereapprox::Triangle>& triangles,
                  const size_t dim,
                  FillMode fillMode,
                  const AABBTree& aabbTree);

    void RaycastFill(const AABBTree& aabbTree);

    void SetVoxel(const size_t i, const size_t j, const size_t k, VoxelValue value);

    VoxelValue& GetVoxel(const size_t i, const size_t j, const size_t k);

    const VoxelValue& GetVoxel(const size_t i, const size_t j, const size_t k) const;

    const std::vector<Voxel>& GetSurfaceVoxels() const;
    const std::vector<Voxel>& GetInteriorVoxels() const;

    double GetScale() const;
    const sphereapprox::BoundsAABB& GetBounds() const;
    const sphereapprox::Vector3<uint32_t>& GetDimensions() const;

    sphereapprox::BoundsAABB m_bounds;
    double m_scale{ 1.0 };
    sphereapprox::Vector3<uint32_t> m_dim{ 0 };
    size_t m_numVoxelsOnSurface{ 0 };
    size_t m_numVoxelsInsideSurface{ 0 };
    size_t m_numVoxelsOutsideSurface{ 0 };
    std::vector<VoxelValue> m_data;

private:
    void MarkOutsideSurface(
        const size_t i0, const size_t j0, const size_t k0, const size_t i1, const size_t j1, const size_t k1);
    void FillOutsideSurface();

    void FillInsideSurface();

    std::vector<sphereapprox::Voxel> m_surfaceVoxels;
    std::vector<sphereapprox::Voxel> m_interiorVoxels;
};

bool PlaneBoxOverlap(const sphereapprox::Vect3& normal, const sphereapprox::Vect3& vert, const sphereapprox::Vect3& maxbox)
{
    int32_t q;
    sphereapprox::Vect3 vmin;
    sphereapprox::Vect3 vmax;
    double v;
    for (q = 0; q < 3; q++)
    {
        v = vert[q];
        if (normal[q] > double(0.0))
        {
            vmin[q] = -maxbox[q] - v;
            vmax[q] = maxbox[q] - v;
        }
        else
        {
            vmin[q] = maxbox[q] - v;
            vmax[q] = -maxbox[q] - v;
        }
    }
    if (normal.Dot(vmin) > double(0.0))
        return false;
    if (normal.Dot(vmax) >= double(0.0))
        return true;
    return false;
}

bool AxisTest(
    double a, double b, double fa, double fb, double v0, double v1, double v2, double v3, double boxHalfSize1, double boxHalfSize2)
{
    double p0 = a * v0 + b * v1;
    double p1 = a * v2 + b * v3;

    double min = std::min(p0, p1);
    double max = std::max(p0, p1);

    double rad = fa * boxHalfSize1 + fb * boxHalfSize2;
    if (min > rad || max < -rad)
    {
        return false;
    }

    return true;
}

bool TriBoxOverlap(const sphereapprox::Vect3& boxCenter,
                   const sphereapprox::Vect3& boxHalfSize,
                   const sphereapprox::Vect3& triVer0,
                   const sphereapprox::Vect3& triVer1,
                   const sphereapprox::Vect3& triVer2)
{
    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-direction) */
    /*       this gives 3x3=9 more tests */

    sphereapprox::Vect3 v0 = triVer0 - boxCenter;
    sphereapprox::Vect3 v1 = triVer1 - boxCenter;
    sphereapprox::Vect3 v2 = triVer2 - boxCenter;
    sphereapprox::Vect3 e0 = v1 - v0;
    sphereapprox::Vect3 e1 = v2 - v1;
    sphereapprox::Vect3 e2 = v0 - v2;

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */

    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */
    double fex = fabs(e0[0]);
    double fey = fabs(e0[1]);
    double fez = fabs(e0[2]);

    /*
     * These should use Get*() instead of subscript for consistency, but the function calls are long enough already
     */
    if (!AxisTest(e0[2], -e0[1], fez, fey, v0[1], v0[2], v2[1], v2[2], boxHalfSize[1], boxHalfSize[2]))
        return 0; // X01
    if (!AxisTest(-e0[2], e0[0], fez, fex, v0[0], v0[2], v2[0], v2[2], boxHalfSize[0], boxHalfSize[2]))
        return 0; // Y02
    if (!AxisTest(e0[1], -e0[0], fey, fex, v1[0], v1[1], v2[0], v2[1], boxHalfSize[0], boxHalfSize[1]))
        return 0; // Z12

    fex = fabs(e1[0]);
    fey = fabs(e1[1]);
    fez = fabs(e1[2]);

    if (!AxisTest(e1[2], -e1[1], fez, fey, v0[1], v0[2], v2[1], v2[2], boxHalfSize[1], boxHalfSize[2]))
        return 0; // X01
    if (!AxisTest(-e1[2], e1[0], fez, fex, v0[0], v0[2], v2[0], v2[2], boxHalfSize[0], boxHalfSize[2]))
        return 0; // Y02
    if (!AxisTest(e1[1], -e1[0], fey, fex, v0[0], v0[1], v1[0], v1[1], boxHalfSize[0], boxHalfSize[2]))
        return 0; // Z0

    fex = fabs(e2[0]);
    fey = fabs(e2[1]);
    fez = fabs(e2[2]);

    if (!AxisTest(e2[2], -e2[1], fez, fey, v0[1], v0[2], v1[1], v1[2], boxHalfSize[1], boxHalfSize[2]))
        return 0; // X2
    if (!AxisTest(-e2[2], e2[0], fez, fex, v0[0], v0[2], v1[0], v1[2], boxHalfSize[0], boxHalfSize[2]))
        return 0; // Y1
    if (!AxisTest(e2[1], -e2[0], fey, fex, v1[0], v1[1], v2[0], v2[1], boxHalfSize[0], boxHalfSize[1]))
        return 0; // Z12

    /* Bullet 1: */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in 0-direction */
    double min = std::min({ v0.GetX(), v1.GetX(), v2.GetX() });
    double max = std::max({ v0.GetX(), v1.GetX(), v2.GetX() });
    if (min > boxHalfSize[0] || max < -boxHalfSize[0])
        return false;

    /* test in 1-direction */
    min = std::min({ v0.GetY(), v1.GetY(), v2.GetY() });
    max = std::max({ v0.GetY(), v1.GetY(), v2.GetY() });
    if (min > boxHalfSize[1] || max < -boxHalfSize[1])
        return false;

    /* test in getZ-direction */
    min = std::min({ v0.GetZ(), v1.GetZ(), v2.GetZ() });
    max = std::max({ v0.GetZ(), v1.GetZ(), v2.GetZ() });
    if (min > boxHalfSize[2] || max < -boxHalfSize[2])
        return false;

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    sphereapprox::Vect3 normal = e0.Cross(e1);

    if (!PlaneBoxOverlap(normal, v0, boxHalfSize))
        return false;
    return true; /* box and triangle overlaps */
}

void Volume::Voxelize(const std::vector<sphereapprox::Vertex>& points,
                      const std::vector<sphereapprox::Triangle>& indices,
                      const size_t dimensions,
                      FillMode fillMode,
                      const AABBTree& aabbTree)
{
    double a = std::pow(dimensions, 0.33);
    size_t dim = a * double(1.5);
    dim = std::max(dim, size_t(32));

    if (points.size() == 0)
    {
        return;
    }

    m_bounds = BoundsAABB(points);

    sphereapprox::Vect3 d = m_bounds.GetSize();
    double r;
    // Equal comparison is important here to avoid taking the last branch when d[0] == d[1] with d[2] being the smallest
    // dimension. That would lead to dimensions in i and j to be a lot bigger than expected and make the amount of
    // voxels in the volume totally unmanageable.
    if (d[0] >= d[1] && d[0] >= d[2])
    {
        r = d[0];
        m_dim[0] = uint32_t(dim);
        m_dim[1] = uint32_t(2 + static_cast<size_t>(dim * d[1] / d[0]));
        m_dim[2] = uint32_t(2 + static_cast<size_t>(dim * d[2] / d[0]));
    }
    else if (d[1] >= d[0] && d[1] >= d[2])
    {
        r = d[1];
        m_dim[1] = uint32_t(dim);
        m_dim[0] = uint32_t(2 + static_cast<size_t>(dim * d[0] / d[1]));
        m_dim[2] = uint32_t(2 + static_cast<size_t>(dim * d[2] / d[1]));
    }
    else
    {
        r = d[2];
        m_dim[2] = uint32_t(dim);
        m_dim[0] = uint32_t(2 + static_cast<size_t>(dim * d[0] / d[2]));
        m_dim[1] = uint32_t(2 + static_cast<size_t>(dim * d[1] / d[2]));
    }

    m_scale = r / (dim - 1);
    double invScale = (dim - 1) / r;

    m_data = std::vector<VoxelValue>(m_dim[0] * m_dim[1] * m_dim[2], VoxelValue::PRIMITIVE_UNDEFINED);
    m_numVoxelsOnSurface = 0;
    m_numVoxelsInsideSurface = 0;
    m_numVoxelsOutsideSurface = 0;

    sphereapprox::Vect3 p[3];
    sphereapprox::Vect3 boxcenter;
    sphereapprox::Vect3 pt;
    const sphereapprox::Vect3 boxhalfsize(double(0.5));
    for (size_t t = 0; t < indices.size(); ++t)
    {
        size_t i0, j0, k0;
        size_t i1, j1, k1;
        sphereapprox::Vector3<uint32_t> tri = indices[t];
        for (int32_t c = 0; c < 3; ++c)
        {
            pt = points[tri[c]];

            p[c] = (pt - m_bounds.GetMin()) * invScale;

            size_t i = static_cast<size_t>(p[c][0] + double(0.5));
            size_t j = static_cast<size_t>(p[c][1] + double(0.5));
            size_t k = static_cast<size_t>(p[c][2] + double(0.5));

            assert(i < m_dim[0] && i >= 0 && j < m_dim[1] && j >= 0 && k < m_dim[2] && k >= 0);

            if (c == 0)
            {
                i0 = i1 = i;
                j0 = j1 = j;
                k0 = k1 = k;
            }
            else
            {
                i0 = std::min(i0, i);
                j0 = std::min(j0, j);
                k0 = std::min(k0, k);

                i1 = std::max(i1, i);
                j1 = std::max(j1, j);
                k1 = std::max(k1, k);
            }
        }
        if (i0 > 0)
            --i0;
        if (j0 > 0)
            --j0;
        if (k0 > 0)
            --k0;
        if (i1 < m_dim[0])
            ++i1;
        if (j1 < m_dim[1])
            ++j1;
        if (k1 < m_dim[2])
            ++k1;
        for (size_t i_id = i0; i_id < i1; ++i_id)
        {
            boxcenter[0] = uint32_t(i_id);
            for (size_t j_id = j0; j_id < j1; ++j_id)
            {
                boxcenter[1] = uint32_t(j_id);
                for (size_t k_id = k0; k_id < k1; ++k_id)
                {
                    boxcenter[2] = uint32_t(k_id);
                    bool res = TriBoxOverlap(boxcenter, boxhalfsize, p[0], p[1], p[2]);
                    VoxelValue& value = GetVoxel(i_id, j_id, k_id);
                    if (res && value == VoxelValue::PRIMITIVE_UNDEFINED)
                    {
                        value = VoxelValue::PRIMITIVE_ON_SURFACE;
                        ++m_numVoxelsOnSurface;
                        m_surfaceVoxels.emplace_back(uint32_t(i_id), uint32_t(j_id), uint32_t(k_id));
                    }
                }
            }
        }
    }

    if (fillMode == FillMode::SURFACE_ONLY)
    {
        const size_t i0_local = m_dim[0];
        const size_t j0_local = m_dim[1];
        const size_t k0_local = m_dim[2];
        for (size_t i_id = 0; i_id < i0_local; ++i_id)
        {
            for (size_t j_id = 0; j_id < j0_local; ++j_id)
            {
                for (size_t k_id = 0; k_id < k0_local; ++k_id)
                {
                    const VoxelValue& voxel = GetVoxel(i_id, j_id, k_id);
                    if (voxel != VoxelValue::PRIMITIVE_ON_SURFACE)
                    {
                        SetVoxel(i_id, j_id, k_id, VoxelValue::PRIMITIVE_OUTSIDE_SURFACE);
                    }
                }
            }
        }
    }
    else if (fillMode == FillMode::FLOOD_FILL)
    {
        /*
         * Marking the outside edges of the voxel cube to be outside surfaces to walk
         */
        MarkOutsideSurface(0, 0, 0, m_dim[0], m_dim[1], 1);
        MarkOutsideSurface(0, 0, m_dim[2] - 1, m_dim[0], m_dim[1], m_dim[2]);
        MarkOutsideSurface(0, 0, 0, m_dim[0], 1, m_dim[2]);
        MarkOutsideSurface(0, m_dim[1] - 1, 0, m_dim[0], m_dim[1], m_dim[2]);
        MarkOutsideSurface(0, 0, 0, 1, m_dim[1], m_dim[2]);
        MarkOutsideSurface(m_dim[0] - 1, 0, 0, m_dim[0], m_dim[1], m_dim[2]);
        FillOutsideSurface();
        FillInsideSurface();
    }
    else if (fillMode == FillMode::RAYCAST_FILL)
    {
        RaycastFill(aabbTree);
    }
}

void Volume::RaycastFill(const AABBTree& aabbTree)
{
    const uint32_t i0 = m_dim[0];
    const uint32_t j0 = m_dim[1];
    const uint32_t k0 = m_dim[2];

    size_t maxSize = i0 * j0 * k0;

    std::vector<Voxel> temp;
    temp.reserve(maxSize);
    uint32_t count{ 0 };
    m_numVoxelsInsideSurface = 0;
    for (uint32_t i = 0; i < i0; ++i)
    {
        for (uint32_t j = 0; j < j0; ++j)
        {
            for (uint32_t k = 0; k < k0; ++k)
            {
                VoxelValue& voxel = GetVoxel(i, j, k);
                if (voxel != VoxelValue::PRIMITIVE_ON_SURFACE)
                {
                    sphereapprox::Vect3 start = sphereapprox::Vect3(i, j, k) * m_scale + m_bounds.GetMin();

                    uint32_t insideCount = 0;
                    uint32_t outsideCount = 0;

                    sphereapprox::Vect3 directions[6] = {
                        sphereapprox::Vect3(1, 0, 0), sphereapprox::Vect3(-1, 0, 0), // this was 1, 0, 0 in the original
                                                                                     // code, but looks wrong
                        sphereapprox::Vect3(0, 1, 0), sphereapprox::Vect3(0, -1, 0), sphereapprox::Vect3(0, 0, 1),
                        sphereapprox::Vect3(0, 0, -1)
                    };

                    for (uint32_t r = 0; r < 6; r++)
                    {
                        aabbTree.TraceRay(start, directions[r], insideCount, outsideCount);
                        // Early out if we hit the outside of the mesh
                        if (outsideCount)
                        {
                            break;
                        }
                        // Early out if we accumulated 3 inside hits
                        if (insideCount >= 3)
                        {
                            break;
                        }
                    }

                    if (outsideCount == 0 && insideCount >= 3)
                    {
                        voxel = VoxelValue::PRIMITIVE_INSIDE_SURFACE;
                        temp.emplace_back(i, j, k);
                        count++;
                        m_numVoxelsInsideSurface++;
                    }
                    else
                    {
                        voxel = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE;
                    }
                }
            }
        }
    }

    if (count)
    {
        m_interiorVoxels = std::move(temp);
    }
}

void Volume::SetVoxel(const size_t i, const size_t j, const size_t k, VoxelValue value)
{
    assert(i < m_dim[0] || i >= 0);
    assert(j < m_dim[1] || j >= 0);
    assert(k < m_dim[2] || k >= 0);

    m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]] = value;
}

VoxelValue& Volume::GetVoxel(const size_t i, const size_t j, const size_t k)
{
    assert(i < m_dim[0] || i >= 0);
    assert(j < m_dim[1] || j >= 0);
    assert(k < m_dim[2] || k >= 0);
    return m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]];
}

const VoxelValue& Volume::GetVoxel(const size_t i, const size_t j, const size_t k) const
{
    assert(i < m_dim[0] || i >= 0);
    assert(j < m_dim[1] || j >= 0);
    assert(k < m_dim[2] || k >= 0);
    return m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]];
}

const std::vector<Voxel>& Volume::GetSurfaceVoxels() const
{
    return m_surfaceVoxels;
}

const std::vector<Voxel>& Volume::GetInteriorVoxels() const
{
    return m_interiorVoxels;
}

double Volume::GetScale() const
{
    return m_scale;
}

const sphereapprox::BoundsAABB& Volume::GetBounds() const
{
    return m_bounds;
}

const sphereapprox::Vector3<uint32_t>& Volume::GetDimensions() const
{
    return m_dim;
}

void Volume::MarkOutsideSurface(
    const size_t i0, const size_t j0, const size_t k0, const size_t i1, const size_t j1, const size_t k1)
{
    for (size_t i = i0; i < i1; ++i)
    {
        for (size_t j = j0; j < j1; ++j)
        {
            for (size_t k = k0; k < k1; ++k)
            {
                VoxelValue& v = GetVoxel(i, j, k);
                if (v == VoxelValue::PRIMITIVE_UNDEFINED)
                {
                    v = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
                }
            }
        }
    }
}

inline void WalkForward(int64_t start, int64_t end, VoxelValue* ptr, int64_t stride, int64_t maxDistance)
{
    for (int64_t i = start, count = 0; count < maxDistance && i < end && *ptr == VoxelValue::PRIMITIVE_UNDEFINED;
         ++i, ptr += stride, ++count)
    {
        *ptr = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
    }
}

inline void WalkBackward(int64_t start, int64_t end, VoxelValue* ptr, int64_t stride, int64_t maxDistance)
{
    for (int64_t i = start, count = 0; count < maxDistance && i >= end && *ptr == VoxelValue::PRIMITIVE_UNDEFINED;
         --i, ptr -= stride, ++count)
    {
        *ptr = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
    }
}

void Volume::FillOutsideSurface()
{
    size_t voxelsWalked = 0;
    const int64_t i0 = m_dim[0];
    const int64_t j0 = m_dim[1];
    const int64_t k0 = m_dim[2];

    // Avoid striding too far in each direction to stay in L1 cache as much as possible.
    // The cache size required for the walk is roughly (4 * walkDistance * 64) since
    // the k direction doesn't count as it's walking byte per byte directly in a cache lines.
    // ~16k is required for a walk distance of 64 in each directions.
    const size_t walkDistance = 64;

    // using the stride directly instead of calling GetVoxel for each iterations saves
    // a lot of multiplications and pipeline stalls due to data dependencies on imul.
    const size_t istride = &GetVoxel(1, 0, 0) - &GetVoxel(0, 0, 0);
    const size_t jstride = &GetVoxel(0, 1, 0) - &GetVoxel(0, 0, 0);
    const size_t kstride = &GetVoxel(0, 0, 1) - &GetVoxel(0, 0, 0);

    // It might seem counter intuitive to go over the whole voxel range multiple times
    // but since we do the run in memory order, it leaves us with far fewer cache misses
    // than a BFS algorithm and it has the additional benefit of not requiring us to
    // store and manipulate a fifo for recursion that might become huge when the number
    // of voxels is large.
    // This will outperform the BFS algorithm by several orders of magnitude in practice.
    do
    {
        voxelsWalked = 0;
        for (int64_t i = 0; i < i0; ++i)
        {
            for (int64_t j = 0; j < j0; ++j)
            {
                for (int64_t k = 0; k < k0; ++k)
                {
                    VoxelValue& voxel = GetVoxel(i, j, k);
                    if (voxel == VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK)
                    {
                        voxelsWalked++;
                        voxel = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE;

                        // walk in each direction to mark other voxel that should be walked.
                        // this will generate a 3d pattern that will help the overall
                        // algorithm converge faster while remaining cache friendly.
                        WalkForward(k + 1, k0, &voxel + kstride, kstride, walkDistance);
                        WalkBackward(k - 1, 0, &voxel - kstride, kstride, walkDistance);

                        WalkForward(j + 1, j0, &voxel + jstride, jstride, walkDistance);
                        WalkBackward(j - 1, 0, &voxel - jstride, jstride, walkDistance);

                        WalkForward(i + 1, i0, &voxel + istride, istride, walkDistance);
                        WalkBackward(i - 1, 0, &voxel - istride, istride, walkDistance);
                    }
                }
            }
        }

        m_numVoxelsOutsideSurface += voxelsWalked;
    } while (voxelsWalked != 0);
}

void Volume::FillInsideSurface()
{
    const uint32_t i0 = uint32_t(m_dim[0]);
    const uint32_t j0 = uint32_t(m_dim[1]);
    const uint32_t k0 = uint32_t(m_dim[2]);

    size_t maxSize = i0 * j0 * k0;

    std::vector<Voxel> temp;
    temp.reserve(maxSize);
    uint32_t count{ 0 };

    for (uint32_t i = 0; i < i0; ++i)
    {
        for (uint32_t j = 0; j < j0; ++j)
        {
            for (uint32_t k = 0; k < k0; ++k)
            {
                VoxelValue& v = GetVoxel(i, j, k);
                if (v == VoxelValue::PRIMITIVE_UNDEFINED)
                {
                    v = VoxelValue::PRIMITIVE_INSIDE_SURFACE;
                    temp.emplace_back(i, j, k);
                    count++;
                    ++m_numVoxelsInsideSurface;
                }
            }
        }
    }

    if (count)
    {
        m_interiorVoxels = std::move(temp);
    }
}

//********************************************************************************************************************


//********************************************************************************************************************
// Definition of the ThreadPool
//********************************************************************************************************************

class ThreadPool
{
public:
    ThreadPool();
    ThreadPool(int worker);
    ~ThreadPool();
    template <typename F, typename... Args>
    auto enqueue(F&& f, Args&&... args)
#    ifndef __cpp_lib_is_invocable
        -> std::future<typename std::result_of<F(Args...)>::type>;
#    else
        -> std::future<typename std::invoke_result_t<F, Args...>>;
#    endif
private:
    std::vector<std::thread> workers;
    std::deque<std::function<void()>> tasks;
    std::mutex task_mutex;
    std::condition_variable cv;
    bool closed;
    int count;
};

ThreadPool::ThreadPool() : ThreadPool(1)
{
}

ThreadPool::ThreadPool(int worker) : closed(false), count(0)
{
    workers.reserve(worker);
    for (int i = 0; i < worker; i++)
    {
        workers.emplace_back([this] {
            std::unique_lock<std::mutex> lock(this->task_mutex);
            while (true)
            {
                while (this->tasks.empty())
                {
                    if (this->closed)
                    {
                        return;
                    }
                    this->cv.wait(lock);
                }
                auto task = this->tasks.front();
                this->tasks.pop_front();
                lock.unlock();
                task();
                lock.lock();
            }
        });
    }
}

template <typename F, typename... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args)
#    ifndef __cpp_lib_is_invocable
    -> std::future<typename std::result_of<F(Args...)>::type>
#    else
    -> std::future<typename std::invoke_result_t<F, Args...>>
#    endif
{

#    ifndef __cpp_lib_is_invocable
    using return_type = typename std::result_of<F(Args...)>::type;
#    else
    using return_type = typename std::invoke_result_t<F, Args...>;
#    endif
    auto task =
        std::make_shared<std::packaged_task<return_type()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
    auto result = task->get_future();

    {
        std::unique_lock<std::mutex> lock(task_mutex);
        if (!closed)
        {
            tasks.emplace_back([task] { (*task)(); });
            cv.notify_one();
        }
    }

    return result;
}

ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(task_mutex);
        closed = true;
    }
    cv.notify_all();
    for (auto&& worker : workers)
    {
        worker.join();
    }
}

using RescanSet = std::unordered_set<uint32_t>;
using VoxelIndexMap = std::unordered_map<uint32_t, uint32_t>;
using SphereMap = std::unordered_map<uint32_t, SphereApprox::Sphere>;

class SphereGrow
{
public:
    SphereGrow(Volume* v,
               double scale,
               const Vect3& center,
               const Voxel& seed,
               uint32_t sphereIndex,
               const SphereApprox::Parameters& params)
        : mMeshScale(scale), mMeshCenter(center), mVolume(v), mSeed(seed), mSphereIndex(sphereIndex), mParams(params)
    {
        mVoxelScale = mVolume->GetScale();
        mVoxelScaleHalf = mVoxelScale * .05;
        mVoxelAdjust = mVolume->GetBounds().GetMin() - mVoxelScaleHalf;
    }

    inline void getPoint(const Voxel& v, Vect3& point)
    {
        point[0] = double(v.GetX()) * mVoxelScale + mVoxelScaleHalf + mVoxelAdjust.GetX();
        point[1] = double(v.GetY()) * mVoxelScale + mVoxelScaleHalf + mVoxelAdjust.GetY();
        point[2] = double(v.GetZ()) * mVoxelScale + mVoxelScaleHalf + mVoxelAdjust.GetZ();
    }

    inline void getFloatPos(const Voxel& v, float fpos[3])
    {
        // First get it out of voxel index space into voxel
        // normalized space

        double vx = double(v.GetX()) * mVoxelScale + mVoxelAdjust.GetX();
        double vy = double(v.GetY()) * mVoxelScale + mVoxelAdjust.GetY();
        double vz = double(v.GetZ()) * mVoxelScale + mVoxelAdjust.GetZ();

        // Now go from voxel normalized space back to the original mesh
        // space and convert to floats
        fpos[0] = float(vx * mMeshScale + mMeshCenter.GetX());
        fpos[1] = float(vy * mMeshScale + mMeshCenter.GetY());
        fpos[2] = float(vz * mMeshScale + mMeshCenter.GetZ());
    }


    inline void getFloatPos(const Vect3& v, float fpos[3])
    {
        fpos[0] = float(v.GetX() * mMeshScale + mMeshCenter.GetX());
        fpos[1] = float(v.GetY() * mMeshScale + mMeshCenter.GetY());
        fpos[2] = float(v.GetZ() * mMeshScale + mMeshCenter.GetZ());
    }

    void processPoint(int x, int y, int z, double radiusSquared, bool isRescan)
    {
        Voxel vox(x, y, z);
        Vect3 p;
        getPoint(vox, p);

        double fdx = p.GetX() - mVoxelCenter.GetX();
        double fdy = p.GetY() - mVoxelCenter.GetY();
        double fdz = p.GetZ() - mVoxelCenter.GetZ();

        double d2 = fdx * fdx + fdy * fdy + fdz * fdz;

        if (d2 <= radiusSquared)
        {
            auto found = mRescan.find(vox.GetVoxelAddress());
            if (found != mRescan.end())
            {
                if (isRescan)
                {
                    // Can be removed from the rescan set because it has
                    // now been consumed
                    mRescan.erase(found);
                }
                else
                {
                    return; // we already processed this point in the previous pass
                }
            }
            VoxelValue v = mVolume->GetVoxel(x, y, z);
            mNewVoxelsMap[vox.GetVoxelAddress()] = uint32_t(v);
            if (v == VoxelValue::PRIMITIVE_INSIDE_SURFACE || v == VoxelValue::PRIMITIVE_ON_SURFACE)
            {
                mVoxelCount++;
            }
            else
            {
                mEmptyCount++;
            }
        }
        else
        {
            mRescan.insert(vox.GetVoxelAddress());
        }
    }


    void processPointClipped(int x, int y, int z, double radiusSquared, bool isRescan)
    {
        if (x >= 0 && x < mDimX && y >= 0 && y < mDimY && z >= 0 && z < mDimZ)
        {
            processPoint(x, y, z, radiusSquared, isRescan);
        }
        else
        {
            mEmptyCount++;
        }
    }

    bool step(void)
    {
        if (mFinished)
            return false;

        bool ret = false;

        mNewVoxelsMap.clear();

        uint32_t voxelCount = mVoxelCount;
        uint32_t emptyCount = mEmptyCount;

        auto dim = mVolume->GetDimensions();

        mDimX = dim.GetX();
        mDimY = dim.GetY();
        mDimZ = dim.GetZ();

        mCenterX = mSeed.GetX();
        mCenterY = mSeed.GetY();
        mCenterZ = mSeed.GetZ();

        getPoint(Voxel(mCenterX, mCenterY, mCenterZ), mVoxelCenter);

        double radius = (mVoxelScale + mVoxelScaleHalf) * double(mStepCount);
        double radiusSquared = radius * radius;

        int32_t x1 = mCenterX - mStepCount;
        int32_t x2 = mCenterX + mStepCount;

        int32_t y1 = mCenterY - mStepCount;
        int32_t y2 = mCenterY + mStepCount;

        int32_t z1 = mCenterZ - mStepCount;
        int32_t z2 = mCenterZ + mStepCount;

        bool clipped = true;

        if (x1 >= 0 && x2 < mDimX && y1 >= 0 && y2 < mDimY && z1 >= 0 && z2 < mDimZ)
        {
            clipped = false;
        }
        if (mStepCount == 1)
        {
            for (int32_t x = x1; x <= x2; x++)
            {
                for (int32_t y = y1; y <= y2; y++)
                {
                    for (int32_t z = z1; z <= z2; z++)
                    {
                        if (clipped)
                        {
                            processPointClipped(x, y, z, radiusSquared, false);
                        }
                        else
                        {
                            processPoint(x, y, z, radiusSquared, false);
                        }
                    }
                }
            }
            mStepCount++;
        }
        else
        {
            // Make a copy of the set of points we need to re-process from
            // the previous frame
            RescanSet rscan = mRescan;
            for (auto i = rscan.begin(); i != rscan.end(); ++i)
            {
                Voxel vox(*i);
                if (clipped)
                {
                    processPointClipped(vox.GetX(), vox.GetY(), vox.GetZ(), radiusSquared, true);
                }
                else
                {
                    processPoint(vox.GetX(), vox.GetY(), vox.GetZ(), radiusSquared, true);
                }
            }
            // The X/Y plane
            for (int32_t x = x1; x <= x2; x++)
            {
                for (int32_t y = y1; y <= y2; y++)
                {
                    if (clipped)
                    {
                        processPointClipped(x, y, z1, radiusSquared, false);
                        processPointClipped(x, y, z2, radiusSquared, false);
                    }
                    else
                    {
                        processPoint(x, y, z1, radiusSquared, false);
                        processPoint(x, y, z2, radiusSquared, false);
                    }
                }
            }
            // The XZ plane
            for (int32_t x = x1 + 1; x <= x2 - 1; x++)
            {
                for (int32_t z = z1 + 1; z <= z2 - 1; z++)
                {
                    if (clipped)
                    {
                        processPointClipped(x, y1, z, radiusSquared, false);
                        processPointClipped(x, y2, z, radiusSquared, false);
                    }
                    else
                    {
                        processPoint(x, y1, z, radiusSquared, false);
                        processPoint(x, y2, z, radiusSquared, false);
                    }
                }
            }
            // The YZ plane
            for (int32_t y = y1 + 1; y <= y2 - 1; y++)
            {
                for (int32_t z = z1 + 1; z <= z2 - 1; z++)
                {
                    if (clipped)
                    {
                        processPointClipped(x1, y, z, radiusSquared, false);
                        processPointClipped(x2, y, z, radiusSquared, false);
                    }
                    else
                    {
                        processPoint(x1, y, z, radiusSquared, false);
                        processPoint(x2, y, z, radiusSquared, false);
                    }
                }
            }
            mStepCount++;
        }

        double err = (mEmptyCount * 100) / double(mVoxelCount + mEmptyCount);

        if (err < mParams.mErrorMetric || mStepCount < mParams.mMinStepCount)
        {
            for (auto& i : mNewVoxelsMap)
            {
                mVoxelIndexMap[i.first] = i.second;
            }
            mNewVoxelsMap.clear();
            mError = err;
            mSphereRadius = double(mStepCount) * mVoxelScale;
            mSphereCenter = mVoxelCenter;
            ret = true;
        }
        else
        {
            mFinished = true;
            mEmptyCount = emptyCount;
            mUnrepresentedCount = mVoxelCount = voxelCount;
        }

        return ret;
    }

    void fill(void)
    {
        while (step())
            ;
    }


    void recomputeRepresented(void)
    {
        mOverlapCount++;
        mCulled = true;
        // We go through out voxel index map.
        // Find any voxel that we currently own and then see if
        // it has already been represented by a previous sphere.
        // If it has, then we decrement our total unrepresented voxel
        // counter and update the
        uint32_t refreshCount = 0;
        uint32_t activeCount = 0;
        for (auto& i : mVoxelIndexMap)
        {
            if (i.second == uint32_t(VoxelValue::PRIMITIVE_ON_SURFACE) ||
                i.second == uint32_t(VoxelValue::PRIMITIVE_INSIDE_SURFACE))
            {
                Voxel vox(i.first);
                VoxelValue v = mVolume->GetVoxel(vox.GetX(), vox.GetY(), vox.GetZ());
                if (v == VoxelValue::REPRESENTED)
                {
                    mUnrepresentedCount--;
                    i.second = uint32_t(VoxelValue::REPRESENTED);
                    refreshCount++;
                }
                else
                {
                    activeCount++;
                }
            }
        }
        if (refreshCount)
        {
            VoxelIndexMap::iterator i = mVoxelIndexMap.begin();
            while (i != mVoxelIndexMap.end())
            {
                if ((*i).second == uint32_t(VoxelValue::REPRESENTED))
                {
                    i = mVoxelIndexMap.erase(i);
                }
                else
                {
                    i++;
                }
            }
        }
        mRepresentationPercentage = double(mUnrepresentedCount * 100) / double(mEmptyCount + mVoxelCount);
    }

    void markRepresented(void)
    {
        for (auto& i : mVoxelIndexMap)
        {
            if (i.second == uint32_t(VoxelValue::PRIMITIVE_ON_SURFACE) ||
                i.second == uint32_t(VoxelValue::PRIMITIVE_INSIDE_SURFACE))
            {
                Voxel vox(i.first);
                mVolume->SetVoxel(vox.GetX(), vox.GetY(), vox.GetZ(), VoxelValue::REPRESENTED);
            }
        }
    }

    double pointDistance(const Vect3& a, const Vect3& b) const
    {
        double dx = a.GetX() - b.GetX();
        double dy = a.GetY() - b.GetY();
        double dz = a.GetZ() - b.GetZ();
        double d2 = dx * dx + dy * dy * dz * dz;
        return sqrt(d2);
    }


    bool intersects(const SphereGrow& parent)
    {
        bool ret = false;

        double distance = pointDistance(mSphereCenter, parent.mSphereCenter);
        if (distance < (mSphereRadius + parent.mSphereRadius))
        {
            ret = true;
        }


        return ret;
    }

    uint32_t mOverlapCount{ 0 };
    double mRepresentationPercentage{ 0 };

    bool mCulled{ false };
    bool mFinished{ false };
    double mError{ 0 };

    double mSphereRadius{ 0 };
    Vect3 mSphereCenter{ 0 };

    double mMeshScale;
    Vect3 mMeshCenter;
    uint32_t mLastVoxelCount{ 0 };

    uint32_t mVoxelCount{ 0 };
    uint32_t mEmptyCount{ 0 };
    uint32_t mUnrepresentedCount{ 0 };

    double mVoxelScale;
    double mVoxelScaleHalf;
    Vect3 mVoxelAdjust;
    Vect3 mVoxelCenter;
    int32_t mDimX;
    int32_t mDimY;
    int32_t mDimZ;

    int32_t mCenterX;
    int32_t mCenterY;
    int32_t mCenterZ;

    uint32_t mStepCount{ 1 };
    double mRadius;
    Volume* mVolume;

    VoxelIndexMap mVoxelIndexMap; // Maps from a voxel coordinate space into a vertex index space
    VoxelIndexMap mNewVoxelsMap;
    Voxel mSeed;
    RescanSet mRescan; // Maps from a voxel coordinate space into a vertex index space
    uint32_t mSphereIndex{ 0 };
    SphereApprox::Parameters mParams;
};

using SphereGrowMap = std::unordered_map<uint32_t, SphereGrow*>;

class SphereAproxImpl : public SphereApprox
{
public:
    SphereAproxImpl(void)
    {
    }

    virtual ~SphereAproxImpl(void)
    {
        releaseResources();
    }

    virtual void cancel() final
    {
        mCanceled = true;
    }

    virtual void wait(uint32_t ms) final
    {
        if (mCompute.valid())
            mCompute.wait_for(std::chrono::milliseconds(ms));
    }

    virtual bool compute(const float* const points,
                         const uint32_t countPoints,
                         const uint32_t* const triangles,
                         const uint32_t countTriangles,
                         const Parameters& params)
    {
        double* dpoints = new double[countPoints * 3];
        for (uint32_t i = 0; i < countPoints * 3; i++)
        {
            dpoints[i] = double(points[i]);
        }
        bool ret = compute(dpoints, countPoints, triangles, countTriangles, params);
        delete[] dpoints;
        return ret;
    }

    virtual bool compute(const double* const points,
                         const uint32_t countPoints,
                         const uint32_t* const triangles,
                         const uint32_t countTriangles,
                         const Parameters& params) final
    {
        if (mIsRunning)
            return false;
        mParams = params;
        releaseResources();
        mInputPointCount = countPoints;
        mInputTriangleCount = countTriangles;
        mInputVertices = new double[countPoints * 3];
        memcpy(mInputVertices, points, sizeof(double) * 3 * countPoints);
        mInputIndices = new uint32_t[countTriangles * 3];
        memcpy(mInputIndices, triangles, sizeof(uint32_t) * 3 * countTriangles);
        if (mParams.mAsync)
        {
            mThreadPool = new threadpool::ThreadPool(mParams.mMaxThreadCount);
            mIsRunning = true;
            mCompute = mThreadPool->enqueue([this] { computeInternal(); });
        }
        else
        {
            computeInternal();
        }
        return true;
    }

    bool computeInternal(void)
    {
        // Compute the bounding box of the input mesh so we
        // can normalize it
        if (!mCanceled)
        {
            Vect3 bmin(FLT_MAX);
            Vect3 bmax(-FLT_MAX);
            for (uint32_t i = 0; i < mInputPointCount; i++)
            {
                const Vertex& p = *(const Vertex*)&mInputVertices[i * 3];
                bmin = bmin.CWiseMin(p);
                bmax = bmax.CWiseMax(p);
            }
            mMeshCenter = (bmax + bmin) * 0.5;
            Vect3 scale = bmax - bmin;
            mMeshScale = scale.MaxCoeff();
            mMeshRecipScale = mMeshScale > double(0.0) ? double(1.0) / mMeshScale : double(0.0);
        }
        // Now normalize the input mesh and re-index it to remove any degenerate
        // triangles and/or duplicate vertices
        if (!mCanceled)
        {
            uint32_t dcount = 0;
            VertexIndex vi = VertexIndex(0.001, false);
            mIndices.reserve(mInputTriangleCount * 3);
            for (uint32_t i = 0; i < mInputTriangleCount; i++)
            {
                uint32_t i1 = mInputIndices[i * 3 + 0];
                uint32_t i2 = mInputIndices[i * 3 + 1];
                uint32_t i3 = mInputIndices[i * 3 + 2];
                const Vertex& p1 = *(const Vertex*)&mInputVertices[i1 * 3];
                const Vertex& p2 = *(const Vertex*)&mInputVertices[i2 * 3];
                const Vertex& p3 = *(const Vertex*)&mInputVertices[i3 * 3];
                i1 = getIndex(vi, p1);
                i2 = getIndex(vi, p2);
                i3 = getIndex(vi, p3);
                if (i1 == i2 || i1 == i3 || i2 == i3)
                {
                    dcount++;
                }
                else
                {
                    mIndices.emplace_back(i1, i2, i3);
                }
            }
            if (dcount)
            {
                printf("Skipped %d degenerate triangles.\n", dcount);
            }
            mVertices = vi.TakeVertices();
        }
        if (!mCanceled)
        {
            mAABBTree = new AABBTree(mVertices, mIndices);
        }
        if (!mCanceled)
        {
            mVolume = new Volume;
            mVolume->Voxelize(mVertices, mIndices, mParams.mVoxelResolution, mParams.mFillMode, *mAABBTree);
            mVoxelScale = mVolume->GetScale();
            mVoxelScaleHalf = mVoxelScale * 0.5;
            mVoxelBmin = mVolume->GetBounds().GetMin();
            mVoxelBmax = mVolume->GetBounds().GetMax();
            mVoxelAdjust = mVoxelBmin - mVoxelScaleHalf;
        }
        if (!mCanceled)
        {
            computeSeedPoints(mParams.mMaxSeedCount);
        }
        if (!mCanceled)
        {
            performSphereGrow();
        }
        if (mParams.mFinalize && !mCanceled)
        {
            uint32_t count = 0;
            while (findNextSphere())
            {
                count++;
            }
            if (beginReduction())
            {
                while (reductionStep())
                    ;
            }
        }
        if (mCanceled)
        {
            releaseResources();
        }
        mIsRunning = false; // it's finished!
        if (mParams.mNotifyCallback)
        {
            mParams.mNotifyCallback->notifySphereApproximationComplete();
        }
        return 0;
    }

    virtual void release() final
    {
        delete this;
    }

    virtual bool isReady() const final
    {
        return mIsRunning ? false : true;
    }

    // Takes the input mesh vertex, normalizing it around the origin and
    // then gets the vertex index for it.
    uint32_t getIndex(VertexIndex& vi, const Vertex& p)
    {
        Vect3 pos = (Vect3(p) - mMeshCenter) * mMeshRecipScale;
        bool newPos;
        uint32_t ret = vi.GetIndex(pos, newPos);
        return ret;
    }

    void releaseResources(void)
    {
        if (mIsRunning)
        {
            cancel();
            mCompute.get();
        }
        mCanceled = false; // clear the cancel semaphore
        // Kill the thread pool instance
        delete mThreadPool;
        mThreadPool = nullptr;

        // Release the memory allocated for the backing store of the
        // input triangle mesh
        delete[] mInputIndices;
        delete[] mInputVertices;
        mInputVertices = nullptr;
        mInputIndices = nullptr;
        mInputPointCount = 0;
        mInputTriangleCount = 0;

        // Clear the containers with the cleaned up
        // triangle mesh
        mVertices.clear();
        mIndices.clear();

        // Delete the AABBTree instance
        delete mAABBTree;
        mAABBTree = nullptr;

        // Delete the voxelized representation of the mesh
        delete mVolume;
        mVolume = nullptr;

        // Delete all grow spheres instances and clear the container
        for (auto& i : mGrowSpheres)
        {
            delete i.second;
        }
        mGrowSpheres.clear();

        // Delete all grow sphere instances in the final spheres container
        for (auto& i : mFinalSpheres)
        {
            delete i.second;
        }
        mFinalSpheres.clear();

        // Clear other containers
        mSpheres.clear();
        mReducedSpheres.clear();
        mReducePoints.clear();
        mSeeds.clear();
        mCandidates.clear();
    }


    // To compute the seed points we do the following...
    // We first allow all points marked as 'inside'
    // Next we allow all surface points which have at least 10 voxel neighbors
    // We add all of these seed points to an array.
    // Next, we need to add this points to an indexed array guaranteeing
    // a safe distance. We must add the points in randomized order to avoid
    // clumping / aliasing issues
    void computeSeedPoints(uint32_t maxSeedPoints)
    {
        mSeeds.clear();
        std::vector<float> seeds;
        {
            auto& interior = mVolume->GetInteriorVoxels();
            for (auto i = interior.begin(); i != interior.end(); ++i)
            {
                seeds.push_back(float(i->GetX()));
                seeds.push_back(float(i->GetY()));
                seeds.push_back(float(i->GetZ()));
            }
        }
        {
            auto& surface = mVolume->GetSurfaceVoxels();
            for (auto i = surface.begin(); i != surface.end(); ++i)
            {
                seeds.push_back(float(i->GetX()));
                seeds.push_back(float(i->GetY()));
                seeds.push_back(float(i->GetZ()));
            }
        }
        {
            kmeanspp::KmeansPP* kpp = kmeanspp::KmeansPP::create();
            uint32_t pcount = uint32_t(seeds.size() / 3);
            uint32_t resultPointCount;
            kmeanspp::KmeansPP::Parameters kp;
            kp.mUseKdTree = true;
            kp.mUseThreading = mParams.mAsync;
            kp.mThreadPool = mThreadPool;
            kp.mPoints = &seeds[0];
            kp.mPointCount = pcount;
            kp.mMaxPoints = mParams.mMaxSeedCount;
            kp.mMaximumPlusPlusCount = kp.mMaxPoints * 4;

            const float* results = kpp->compute(kp, resultPointCount);
            const float* p = results;
            for (uint32_t i = 0; i < resultPointCount; i++)
            {
                uint32_t x = uint32_t(p[0] + 0.5f);
                uint32_t y = uint32_t(p[1] + 0.5f);
                uint32_t z = uint32_t(p[2] + 0.5f);
                Voxel v(x, y, z);
                mSeeds.push_back(v);
                p += 3;
            }
            kpp->release();
        }
    }

    void performSphereGrow(void)
    {
        mSphereIndex = 0;
        for (auto& i : mSeeds)
        {
            SphereGrow* sg = new SphereGrow(mVolume, mMeshScale, mMeshCenter, i, mSphereIndex, mParams);
            mGrowSpheres[mSphereIndex] = sg;
            sg->fill();
            mSphereIndex++;
        }
    }

    void getPoint(const Voxel& v, Vect3& point)
    {
        point[0] = double(v.GetX()) * mVoxelScale + mVoxelScaleHalf + mVoxelAdjust.GetX();
        point[1] = double(v.GetY()) * mVoxelScale + mVoxelScaleHalf + mVoxelAdjust.GetY();
        point[2] = double(v.GetZ()) * mVoxelScale + mVoxelScaleHalf + mVoxelAdjust.GetZ();
    }

    bool findNextSphere(void)
    {
        bool ret = false;

        if (!mGrowSpheres.empty())
        {
            std::vector<SphereGrow*> killList;
            SphereGrow* bestSphere = nullptr;
            uint32_t maxRepresented = 0;
            for (auto& i : mGrowSpheres)
            {
                SphereGrow* sg = i.second;
                if (sg->mOverlapCount >= mParams.mMaxOverlapCount &&
                    sg->mRepresentationPercentage < mParams.mCullPercentage)
                {
                    killList.push_back(sg);
                }
                else
                {
                    if (sg->mUnrepresentedCount > maxRepresented)
                    {
                        maxRepresented = sg->mUnrepresentedCount;
                        bestSphere = sg;
                    }
                }
            }

            for (auto& sg : killList)
            {
                SphereGrowMap::iterator found = mGrowSpheres.find(sg->mSphereIndex);
                if (found != mGrowSpheres.end())
                {
                    mGrowSpheres.erase(found);
                    delete sg;
                }
            }
            if (bestSphere)
            {
                SphereGrowMap::iterator found = mGrowSpheres.find(bestSphere->mSphereIndex);
                if (found != mGrowSpheres.end())
                {
                    mGrowSpheres.erase(found);
                }
                mFinalSpheres[bestSphere->mSphereIndex] = bestSphere;

                Sphere sp;
                sp.mCenter[0] =
                    bestSphere->mSphereCenter.GetX() * bestSphere->mMeshScale + bestSphere->mMeshCenter.GetX();
                sp.mCenter[1] =
                    bestSphere->mSphereCenter.GetY() * bestSphere->mMeshScale + bestSphere->mMeshCenter.GetY();
                sp.mCenter[2] =
                    bestSphere->mSphereCenter.GetZ() * bestSphere->mMeshScale + bestSphere->mMeshCenter.GetZ();
                sp.mRadius = bestSphere->mSphereRadius * bestSphere->mMeshScale;
                sp.mId = bestSphere->mSphereIndex;
                mSpheres.push_back(sp);

                bestSphere->markRepresented();

                // ok..once this sphere has been represented, we need to recompute
                // the number of unrepresented voxels in each other sphere remaining
                for (auto& i : mGrowSpheres)
                {
                    if (bestSphere->intersects(*i.second))
                    {
                        i.second->recomputeRepresented();
                    }
                }
                ret = true; // we successfully produced a new sphere!
            }
        }

        return ret;
    }

    virtual const Sphere* getSpheres(uint32_t& sphereCount, bool reduced) const final
    {
        const Sphere* ret = nullptr;
        sphereCount = 0;

        if (!mIsRunning)
        {
            if (reduced)
            {
                if (!mReducedSpheres.empty())
                {
                    ret = &mReducedSpheres[0];
                    sphereCount = uint32_t(mReducedSpheres.size());
                }
            }
            else if (!mSpheres.empty())
            {
                ret = &mSpheres[0];
                sphereCount = uint32_t(mSpheres.size());
            }
        }

        return ret;
    }

    virtual const Parameters& getParameters(void) const final
    {
        return mParams;
    }

    virtual bool stepSphere(void) final
    {
        bool ret = false;

        if (!mIsRunning)
        {
            ret = findNextSphere();
        }

        return ret;
    }

    // For debugging purposes only.
    virtual Volume* getVolume(void) const final
    {
        return mIsRunning ? nullptr : mVolume;
    }

    virtual bool beginReduction(void) final
    {
        bool ret = false;

        if (mSpheres.size() <= mParams.mMaxSpheres)
        {
            mReducedSpheres = mSpheres;
            return false;
        }
        ret = true;
        mReducedSpheres.clear();
        auto km = kmeanspp::KmeansPP::create();
        uint32_t scount = (uint32_t)mSpheres.size();
        std::vector<kmeanspp::Point3> points;
        for (auto& i : mSpheres)
        {
            kmeanspp::Point3 p;
            p.x = i.mCenter[0];
            p.y = i.mCenter[1];
            p.z = i.mCenter[2];
            points.push_back(p);
        }
        kmeanspp::KmeansPP::Parameters params;
        params.mUseKdTree = true;
        params.mUseThreading = mParams.mAsync;
        params.mThreadPool = mThreadPool;
        params.mMaximumPlusPlusCount = scount;
        params.mPointCount = scount;
        params.mPoints = (const float*)&points[0];
        params.mMaxPoints = mParams.mMaxSpheres;

        uint32_t resultPointCount;
        const float* results = km->compute(params, resultPointCount);
        mReducePoints.clear();
        for (uint32_t i = 0; i < resultPointCount; i++)
        {
            const float* p = &results[i * 3];
            SphereApprox::Sphere sp;
            sp.mCenter[0] = p[0];
            sp.mCenter[1] = p[1];
            sp.mCenter[2] = p[2];
            sp.mId = i;
            mReducePoints[i] = sp;
        }
        mCandidates.clear();
        for (auto& i : mSpheres)
        {
            mCandidates[i.mId] = i;
        }
        km->release();

        return ret;
    }

    // In the reduction step we look for the point which is inside
    // of a sphere with the maximum radius
    // We then consume that point and add that sphere as the result
    virtual bool reductionStep(void) final
    {
        bool ret = false;

        uint32_t bestMatchSphereId = 0;
        uint32_t bestMatchPointId = 0;
        double maxRadius = 0;
        double bestDist = FLT_MAX;
        for (double RADIUS_SCALE_FACTOR = 1.2; RADIUS_SCALE_FACTOR < 4 && maxRadius == 0; RADIUS_SCALE_FACTOR += 0.2)
        {
            for (auto& i : mReducePoints)
            {
                for (auto& j : mCandidates)
                {
                    double dx = i.second.mCenter[0] - j.second.mCenter[0];
                    double dy = i.second.mCenter[1] - j.second.mCenter[1];
                    double dz = i.second.mCenter[2] - j.second.mCenter[2];
                    double dist = sqrt(dx * dx + dy * dy + dz * dz);
                    if (dist < (j.second.mRadius * RADIUS_SCALE_FACTOR))
                    {
                        double maxRadiusMin = maxRadius * 0.8;
                        double maxRadiusMax = maxRadius * 1.2;
                        if (j.second.mRadius > maxRadiusMax)
                        {
                            maxRadius = j.second.mRadius;
                            bestMatchSphereId = j.first;
                            bestMatchPointId = i.first;
                            bestDist = dist;
                        }
                        else if (j.second.mRadius >= maxRadiusMin && j.second.mRadius <= maxRadiusMax)
                        {
                            if (dist < bestDist)
                            {
                                maxRadius = j.second.mRadius;
                                bestMatchSphereId = j.first;
                                bestMatchPointId = i.first;
                                bestDist = dist;
                            }
                        }
                    }
                }
            }
        }
        if (maxRadius > 0)
        {
            ret = true;
            SphereMap::iterator found1 = mReducePoints.find(bestMatchPointId);
            SphereMap::iterator found2 = mCandidates.find(bestMatchSphereId);
            mReducedSpheres.push_back((*found2).second);
            mReducePoints.erase(found1);
            mCandidates.erase(found2);
        }

        return ret;
    }

    virtual void setParameters(Parameters& p) final
    {
        if (!mIsRunning)
        {
            mParams = p;
        }
    }

    Vect3 mMeshCenter;
    double mMeshScale{ 0 };
    double mMeshRecipScale{ 1 };
    std::vector<Vertex> mVertices;
    std::vector<Triangle> mIndices;
    AABBTree* mAABBTree{ nullptr };
    double mVoxelScale{ 0 };
    double mVoxelScaleHalf{ 0 };
    Vect3 mVoxelAdjust;
    uint32_t mSeedIndex{ 0 };
    std::vector<Voxel> mSeeds;
    Vect3 mVoxelBmin;
    Vect3 mVoxelBmax;
    Volume* mVolume{ nullptr };
    uint32_t mSphereIndex{ 0 };
    SphereGrowMap mGrowSpheres;
    SphereGrowMap mFinalSpheres;
    std::vector<Sphere> mSpheres;
    std::vector<Sphere> mReducedSpheres;
    Parameters mParams;
    SphereMap mReducePoints;
    SphereMap mCandidates;
    std::atomic<bool> mCanceled{ false };
    std::atomic<bool> mIsRunning{ false };
    std::future<void> mCompute;
    threadpool::ThreadPool* mThreadPool{ nullptr };
    // Backing store of input geometry
    uint32_t mInputPointCount{ 0 };
    uint32_t mInputTriangleCount{ 0 };
    double* mInputVertices{ nullptr };
    uint32_t* mInputIndices{ nullptr };
};

SphereApprox* SphereApprox::create(void)
{
    auto ret = new SphereAproxImpl;
    return static_cast<SphereApprox*>(ret);
}


} // namespace sphereapprox

#    ifdef _MSC_VER
#        pragma warning(pop)
#    endif // _MSC_VER

#    ifdef __GNUC__
#        pragma GCC diagnostic pop
#    endif // __GNUC__

#endif // ENABLE_SPHERE_APROX_IMPLEMENTATION
