// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <omni/Span.h>

namespace omni
{
namespace clashdetection
{

struct Segment
{
    carb::Float3    mP0;
    carb::Float3    mP1;
};

struct DuplicateMeshData
{
    const char* mName0;         // Name/path of first mesh
    const char* mName1;         // Name/path of second mesh
    uint64_t    mMeshHash0[2];  // Hash of first colliding mesh (hash of the mesh data)
    uint64_t    mMeshHash1[2];  // Hash of second colliding mesh (hash of the mesh data)
                                // Note: for a duplicated mesh pair, mMeshHash0 should be equal to mMeshHash1.
    uint64_t    mClashHash[2];  // Hash of the clash itself (hash of the meshes' names)
    uint32_t    mNbTris;        // (Total) number of triangles in duplicated mesh
    double      mTime;          // Overlap time on timeline
    float       mMinDistance;   // Always zero for duplicated meshes
    float       mMaxLocalDepth; // Always -1.0f (undefined) for duplicated meshes

    inline void reset()
    {
        mName0 = mName1 = nullptr;
        mMeshHash0[0] = mMeshHash0[1] = 0;
        mMeshHash1[0] = mMeshHash1[1] = 0;
        mClashHash[0] = mClashHash[1] = 0;
        mNbTris = 0;
        mTime = 0.0;
        mMinDistance = 0.0f;
        mMaxLocalDepth = -1.0f;
    }
};

struct OverlapData
{
    // Constant data for all records
    const char* mName0;         // Name/path of first mesh
    const char* mName1;         // Name/path of second mesh
    uint64_t    mMeshHash0[2];  // Hash of first colliding mesh (hash of the mesh data)
    uint64_t    mMeshHash1[2];  // Hash of second colliding mesh (hash of the mesh data)
    uint64_t    mClashHash[2];  // Hash of the clash itself (hash of the meshes' names)
    uint32_t    mNbRecords;     // Number of frames the pair overlaps (always 1 for static clash detection)

    // Record-dependent data
    uint32_t    mNbTris;        // Number of overlapping triangles for current frame
    double      mTime;          // Overlap time on timeline
    float       mMinDistance;   // Minimum distance recorded between pairs of triangles
    float       mMaxLocalDepth; // Maximum local depth recorded between pairs of triangles (see setComputeMaxLocalDepth)

    inline void reset()
    {
        mName0 = mName1 = nullptr;
        mMeshHash0[0] = mMeshHash0[1] = 0;
        mMeshHash1[0] = mMeshHash1[1] = 0;
        mClashHash[0] = mClashHash[1] = 0;
        mNbRecords = mNbTris = 0;
        mTime = 0.0;
        mMinDistance = 0.0f;
        mMaxLocalDepth = -1.0f; // -1.0f for undefined / not-computed
    }
};

struct StepData
{
    const char* mName;      // Name of the step
    float       mProgress;  // Progress value between 0.0 and 1.0
    bool        mFinished;  // True for the last step

    inline void reset()
    {
        mName = nullptr;
        mProgress = 0.0f;
        mFinished = true;
    }
};

struct TimelineData
{
    double  mStartTime;             // Start time
    double  mEndTime;               // End time
    double  mCurrentTime;           // Current time
    double  mTimeCodesPerSecond;    // Time codes per second
};

struct Stats
{
    uint32_t    mNbMeshes;
    uint32_t    mNbSharedMeshes;
    uint32_t    mNbCookedMeshes;

    inline void reset()
    {
        mNbMeshes = 0;
        mNbSharedMeshes = 0;
        mNbCookedMeshes = 0;
    }
};

enum MeshIndex
{
    eMesh0, // First mesh of the pair
    eMesh1, // Second mesh of the pair
};

enum OverlapReportFlag
{
    eDefault    = 0,        // By default, return collision faces (backward compatibility with initial version)
    eFaces      = (1 << 0), // Report USD indices of colliding faces
    eOutline    = (1 << 1), // Report collision outline (segments)
};

struct OverlapReport
{
    // Collision faces (if OverlapReportFlag::eFaces is set)
    uint32_t        mNbCollidingFaces;      // Number of colliding faces
    const uint32_t* mCollidingFaces;        // Indices of colliding faces (USD indices)

    // Collision outline (if OverlapReportFlag::eOutline is set)
    uint32_t        mNbCollidingSegments;   // Number of colliding segments in outline
    const Segment*  mCollidingSegments;     // Collision outline

    inline void reset()
    {
        mNbCollidingFaces = mNbCollidingSegments = 0;
        mCollidingFaces = nullptr;
        mCollidingSegments = nullptr;
    }
};

struct IClashDetection2
{
    CARB_PLUGIN_INTERFACE("omni::physx::IClashDetection2", 1, 10)

    //==== Context management ====

    // Creates clash detection context.
    //
    // A context captures all persistent data related to current clash detection.
    //
    // \return  Clash detection context
    void*(CARB_ABI* create)();

    // Releases clash detection context.
    //
    // Releases all persistent data and deletes the context. A context should not be used anymore after release.
    //
    // \param[in]   context    Clash detection context
    void(CARB_ABI* release)(void* context);

    // Resets clash detection context.
    //
    // Releases all persistent data. A context can be reused after a reset.
    //
    // \param[in]   context    Clash detection context
    void(CARB_ABI* reset)(void* context);

    //==== Settings ====

    // Enables or disables logging.
    //
    // Logging will output some information to the console during clash detection. Mostly for debugging.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setLog)(void* context, bool flag);

    // Enables or disables coplanar hits.
    //
    // A coplanar hit is when two overlapping triangles have the same plane. This naturally happens when e.g. a flat
    // object just rests on the ground, and this built-in feature is a convenient way to ignore these results.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setCoplanar)(void* context, bool flag);

    // Selects between "any hit" or "all hits" mode.
    //
    // In "any hit" mode the system will abort a query as soon as a single overlap is found. This can be useful to get
    // a quick set of initial results, which can later be refined with the "all hits" mode.
    //
    // This is equivalent to setTriangleLimit(1).
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    // \see setTriangleLimit
    void(CARB_ABI* setAnyHit)(void* context, bool flag);

    // Selects between quantized or non-quantized BVH bounds.
    //
    // Quantized uses less memory, non-quantized is potentially faster.
    // Recommended: false
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setQuantized)(void* context, bool flag);

    // Selects between tight or coarse object bounds.
    //
    // Tight bounds are more expensive to compute but prune more pairs out of the broadphase.
    // Recommended: true
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setTightBounds)(void* context, bool flag);

    // EXPERIMENTAL. Sub-component filtering.
    //
    // Recommended: false
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setSubcomponentFiltering)(void* context, bool flag);

    // EXPERIMENTAL. Enables or disables usage of USDRT.
    //
    // Enabling USDRT can provide faster initial Stage traversals.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setUseUSDRT)(void* context, bool flag);

    // Runs clash detection with a single thread or multiple threads.
    //
    // Recommended: false
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setSingleThreaded)(void* context, bool flag);

    // Enables or disables new task manager implementation.
    //
    // Only used with setSingleThreaded(false)
    // Recommended: true
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setNewTaskManager)(void* context, bool flag);

    // Enables or disables purging permanent dynamic overlaps.
    //
    // This is only used for dynamic clash detection. Enabling this tells the system to discard pairs of dynamic objects
    // that always overlap over the tested time interval.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setPurgePermanentDynamicOverlaps)(void* context, bool flag);

    // Enables or disables purging permanent static overlaps.
    //
    // This is only used for dynamic clash detection. Enabling this tells the system to discard pairs of static objects
    // that always overlap over the tested time interval.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setPurgePermanentStaticOverlaps)(void* context, bool flag);

    // Enables or disables maximum local depth computation.
    //
    // This features enables an alternative (slower) codepath that computes the max maximum local depth between pairs of
    // overlapping triangles. The results will be reported in OverlapData::mMaxLocalDepth (and set to zero if the feature
    // is disabled).
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    // \note This value is not a true measure of penetration depth between two meshes.
    // \see setContactEpsilon OverlapData::mMaxLocalDepth
    void(CARB_ABI* setComputeMaxLocalDepth)(void* context, bool flag);

    // Sets the number of tasks used in multi-threaded mode.
    //
    // Only used with setSingleThreaded(false) and setNewTaskManager(false)
    //
    // \param[in]   context     Clash detection context
    // \param[in]   nbTasks     Number of tasks
    void(CARB_ABI* setNbTasks)(void* context, uint32_t nbTasks);

    // Sets the epsilon value used to detect objects with a similar pose.
    //
    // This is used to detect 'duplicate meshes', i.e. objects with the same topology / geometry / pose. Duplicate
    // meshes are only reported with static clash detection.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   epsilon     Epsilon value
    // \see getNbDuplicateMeshes getDuplicatedMeshesData
    void(CARB_ABI* setPoseEpsilon)(void* context, float epsilon);

    // Sets the epsilon value used to enlarge object bounds a bit.
    //
    // This is used to avoid misses due to FPU accuracy when bounds are just touching.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   epsilon     Epsilon value
    void(CARB_ABI* setBoundsEpsilon)(void* context, float epsilon);

    // Sets the epsilon value used to cull "zero-area triangles" and slivers.
    //
    // Triangles whose area is smaller than this limit will be ignored.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   epsilon     Epsilon value
    void(CARB_ABI* setAreaEpsilon)(void* context, float epsilon);

    // Sets the epsilon value used to cull "contact cases".
    //
    // This is only used with setComputeMaxLocalDepth(true).
    //
    // For each set of overlapping triangles we compute a max local depth value D, which is the maximum of all penetration
    // depths between pairs of triangles in the set. If D is below the contact epsilon, we discard the set and do not
    // report it as a clash. A "contact case" is when two meshes are just touching (e.g. a box resting on another box).
    // In these cases all the local penetration depths between pairs of touching triangles is zero, and so is D. We can
    // therefore use the contact epsilon value to filter these out.
    //
    // Use a negative value to disable this culling. In this case the maximum local depth is still computed and reported
    // in OverlapData::mMaxLocalDepth. Clients can do their own processing and culling afterwards using the data.
    //
    // Use a small positive epsilon to do the culling internally. In this case discarded pairs will not be reported,
    // saving memory.
    //
    // Note that there are false positives here, i.e. real clashes can be seen as contact cases. So use this judiciously.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   value       Epsilon value
    // \see setComputeMaxLocalDepth OverlapData::mMaxLocalDepth
    void(CARB_ABI* setContactEpsilon)(void* context, float value);

    // Sets the tolerance for clash detection queries.
    //
    // This defines whether the system detects "hard clashes" or "soft clashes".
    // Use 0.0 for hard clashes, or a positive distance value for soft clashes.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   value       Tolerance value
    void(CARB_ABI* setTolerance)(void* context, float value);

    // EXPERIMENTAL. Defines how the results are internally sorted.
    //
    // 0 = sort by number of overlapping triangles
    // 1 = sort by cost query
    // 2 = sort by local depth
    //
    // \param[in]   context     Clash detection context
    // \param[in]   mode        Sort mode
    void(CARB_ABI* setSortMode)(void* context, uint32_t mode);

    // EXPERIMENTAL. Defines the low-level overlap routine.
    //
    // 0 = regular version
    // 1 = alternative version 1
    // 2 = alternative version 2
    // 3 = alternative version 3
    //
    // \param[in]   context     Clash detection context
    // \param[in]   code        Overlap version
    void(CARB_ABI* setOverlapCode)(void* context, uint32_t code);

    // Defines the maximum number of triangles in leaf nodes of the BVH.
    //
    // Should be between 2 and 15. Larger values mean faster cooking, reduced memory usage, but potentially slower
    // queries. Recommended: 4 or 15.
    //
    // \param[in]   context         Clash detection context
    // \param[in]   trisPerLeaf     Number of tris per leaf
    void(CARB_ABI* setTrisPerLeaf)(void* context, uint32_t trisPerLeaf);

    // Defines the triangle limit for narrow-phase queries.
    //
    // Defines the maximum number of triangle overlaps the system will record before aborting a clash query.
    // Some deeply overlapping meshes may take a long time to process, and it is not always useful to compute
    // the exact set of overlapping triangles. Using a limit of e.g. 1000 triangle pairs might be enough,
    // while proving faster overall queries. This is not a global limit, this is a per-mesh-pair limit, i.e.
    // the system will record up to that number of triangle pairs for each new pair of overlapping meshes.
    //
    // Use 0 to disable the feature and ask for unlimited results.
    //
    // A triangle limit of 1 is equivalent to the "any hit" mode enabled with setAnyHit(). But this new function
    // is more flexible.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   limit       Maximum number of triangle pairs (per couple of overlapping meshes)
    // \see setAnyHit
    void(CARB_ABI* setTriangleLimit)(void* context, uint32_t limit);

    // Defines the maximum amount of broadphase / candicate pairs allowed by the system.
    //
    // Sometimes badly setup or broken stages have all objects overlapping each-other, creating a lot of candidate
    // pairs to analyze out of the "broadphase". This can take a large amount of time. This features is a safeguard
    // that limits the number of allowed pairs to a reasonable number. Once the limit is exceeded, the pipeline is
    // cancelled (see isCancelled).
    //
    // The default limit is 0xffffffff and it is not possible to go beyond that.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   limit       Maximum number of allowed broadphase pairs.
    // \see isCancelled
    void(CARB_ABI* setBroadphaseLimit)(void* context, uint32_t limit);

    // Defines whether to run "static" or "dynamic" clash detection.
    //
    // For static clash detection, use startTime = endTime = the time for which clash detection should run.
    // The 'timeCodesPerSecond' parameter will be ignored in this case, so it can be zero.
    //
    // If startTime != endTime, the system will step between startTime and endTime using the timeCodesPerSecond
    // parameter, and perform clash detection at each point inbetween.
    //
    // Note that duplicate meshes are only reported for static clash detection - and ignored with dynamic detection.
    //
    // \param[in]   context             Clash detection context
    // \param[in]   startTime           Start time
    // \param[in]   endTime             End time
    // \param[in]   timeCodesPerSecond  Time codes per second
    // \see getTimeLineData
    void(CARB_ABI* setTimes)(void* context, double startTime, double endTime, double timeCodesPerSecond);

    // TODO: merge with setScope
    // Defines whether to run clash detection on the full scene, or on selected prims.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setFullScene)(void* context, bool flag);

    // Defines whether to run the full clash detection pipeline, or just the find-duplicates part.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setOnlyFindDuplicates)(void* context, bool flag);

    // Defines whether to ignore redundant overlaps, or keep them.
    //
    // This is about redundant overlapping pairs created by duplicated meshes in the scene. For example:
    // - you have 3 objects A0, A1, B
    // - A0 and A1 are duplicates (same mesh data, same pose)
    // - both are overlapping B
    //
    // A0 and A1 will generate a DuplicateMeshData entry, but the system will still generate overlap data
    // (OverlapData) for both (A0;B) and (A1;B). This data is redundant since A0 and A1 are duplicate meshes.
    // In this case the system spends a lot of time computing "the same" clash data twice, using up twice the memory.
    // This cost increases quadratically with the amount of overlapping duplicate meshes, so it can decrease
    // performance and increase memory usage significantly when the input scene is not properly cleaned up.
    //
    // This setting tells the system to first detect these redundant overlap pairs, and only compute clash
    // data for a single one of them.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   flag        Enables or disables this setting
    void(CARB_ABI* setIgnoreRedundantOverlaps)(void* context, bool flag);

    // TODO: merge with setFullScene
    //
    // Sets the clash detection scene processing scope.
    //
    // If both primPaths1 and primPaths2 lists are empty -> process full scene.
    // If only primPaths1 list contains items -> limit processing only to primPaths1 items.
    // If primPaths1 and primPaths2 lists contain items -> process primPaths1 against primPaths2.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   stageId     The stage ID. The passed prims should belong to that stage.
    // \param[in]   paths1      List of prim paths in uint64_t format (conversion uint64_t -> const pxr::SdfPath&)
    // \param[in]   paths2      List of prim paths in uint64_t format (conversion uint64_t -> const pxr::SdfPath&)
    void(CARB_ABI* setScope)(void* context, uint32_t stageId,
                             omni::span<const uint64_t> paths1,
                             omni::span<const uint64_t> paths2);

    //==== Pipeline ====

    // Creates a clash detection pipeline using current settings.
    //
    // \param[in]   context     Clash detection context
    // \return  Number of steps in the pipeline.
    // \see getPipelineStepData runPipelineStep runPipelineAsync
    uint32_t(CARB_ABI* createPipeline)(void* context);

    //==== Pipeline synchronous run ====

    // Gets information about required pipeline step.
    //
    // \param[in]   context     Clash detection context
    // \param[out]  data        Step data
    // \param[in]   index       Required step between 0 and the number returned by createPipeline
    // \return  True if success
    // \see StepData createPipeline runPipelineStep
    bool(CARB_ABI* getPipelineStepData)(void* context, StepData& data, uint32_t index);

    // Runs required pipeline step.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   index       Required step between 0 and the number returned by createPipeline
    // \return  True if success
    // \see getPipelineStepData createPipeline
    bool(CARB_ABI* runPipelineStep)(void* context, uint32_t index);

    //==== Pipeline asynchronous run ====

    // Runs entire pipeline asynchronously.
    //
    // This function spawns threads that run the entire pipeline in the background.
    //
    // \param[in]   context     Clash detection context
    // \return  Asynchronous cookie (pipeline identifier)
    // \see createPipeline isPipelineRunning getPipelineStepDataAsync finishPipeline cancelPipeline
    void*(CARB_ABI* runPipelineAsync)(void* context);

    // Checks whether the asynchronous pipeline is still running.
    //
    // When the function returns true, the pipeline is still running and you can get the
    // current pipeline step using getPipelineStepDataAsync.
    //
    // When the function returns false, the pipeline is done and you can get the results
    // using fetchResults.
    //
    // \param[in]   async     Asynchronous pipeline cookie returned from runPipelineAsync
    // \return  True if the pipeline is still running.
    // \see createPipeline runPipelineAsync getPipelineStepDataAsync finishPipeline cancelPipeline
    bool(CARB_ABI* isPipelineRunning)(void* async);

    // Returns current state / step of asynchronous pipeline.
    //
    // \param[in]   async   Asynchronous pipeline cookie returned from runPipelineAsync
    // \param[out]  data    Step data
    // \see createPipeline runPipelineAsync isPipelineRunning finishPipeline cancelPipeline
    void(CARB_ABI* getPipelineStepDataAsync)(void* async, StepData& data);

    // Finish asynchronous pipeline and delete cookie.
    //
    // This is a blocking call until the pipeline finishes.
    // It is illegal to access results (getNbOverlaps, etc) before calling this function.
    // It is illegal to reuse passed cookie after calling this function.
    //
    // \param[in]   async   Asynchronous pipeline cookie returned from runPipelineAsync
    // \return  True if success.
    // \see createPipeline runPipelineAsync isPipelineRunning getPipelineStepDataAsync cancelPipeline
    bool(CARB_ABI* finishPipeline)(void* async);

    // Cancel asynchronous pipeline.
    //
    // Cancels pipeline. This does not delete the pipeline, you still need to call finishPipeline() after that.
    //
    // \param[in]   async   Asynchronous pipeline cookie returned from runPipelineAsync
    // \see createPipeline runPipelineAsync isPipelineRunning getPipelineStepDataAsync finishPipeline
    void(CARB_ABI* cancelPipeline)(void* async);

    //==== Results ====

    // Returns the number of duplicate meshes found during the last clash detection run.
    //
    // This is only available for static clash detection. After dynamic clash detection this will always return 0.
    //
    // \param[in]   context     Clash detection context
    // \return  Number of duplicate meshes
    // \see getDuplicatedMeshesData deleteDuplicateMeshes
    uint32_t(CARB_ABI* getNbDuplicateMeshes)(void* context);

    // Returns duplicate mesh data.
    //
    // \param[in]   context     Clash detection context
    // \param[out]  data        Duplicate mesh data
    // \param[in]   index       Index between 0 and the number returned by getNbDuplicateMeshes
    // \return  True if success
    // \see DuplicateMeshData getNbDuplicateMeshes deleteDuplicateMeshes
    bool(CARB_ABI* getDuplicatedMeshesData)(void* context, DuplicateMeshData& data, uint32_t index);

    // Optional helper. Automatically deletes some of the duplicate meshes until there are no dups anymore.
    //
    // \param[in]   context     Clash detection context
    // \see getNbDuplicateMeshes getDuplicatedMeshesData
    void(CARB_ABI* deleteDuplicateMeshes)(void* context);

    // Returns the number of overlaps found during the last clash detection run.
    //
    // \param[in]   context     Clash detection context
    // \return  Number of overlaps
    // \see getOverlapData
    uint32_t(CARB_ABI* getNbOverlaps)(void* context);

    // Returns overlap data.
    //
    // \param[in]   context     Clash detection context
    // \param[out]  data        Overlap data
    // \param[in]   index       Index between 0 and the number returned by getNbOverlaps
    // \param[in]   recordIndex Record index between 0 and OverlapData::mNbRecords
    // \return  True if success
    // \see OverlapData getNbOverlaps
    bool(CARB_ABI* getOverlapData)(void* context, OverlapData& data, uint32_t index, uint32_t recordIndex);

    //==== Overlap reports ====

    // Creates a detailed report for desired overlap.
    //
    // This returns additional information not reported by getOverlapData (e.g. indices of overlapping faces).
    //
    // \param[in]   context     Clash detection context
    // \param[out]  report      Overlap report
    // \param[in]   index       Index between 0 and the number returned by getNbOverlaps
    // \param[in]   recordIndex Record index between 0 and OverlapData::mNbRecords
    // \param[in]   meshIndex   Mesh index (first or second mesh of the pair).
    // \param[in]   flags       Combination of OverlapReportFlag flags or 0 for default behavior.
    // \return  True if success
    // \see OverlapReport MeshIndex getNbOverlaps releaseOverlapReport
    bool(CARB_ABI* createOverlapReport)(
        void* context, OverlapReport& report, uint32_t index, uint32_t recordIndex, MeshIndex meshIndex, uint32_t flags);

    // Releases a previously created overlap report.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   report      Overlap report
    // \see OverlapReport createOverlapReport
    void(CARB_ABI* releaseOverlapReport)(void* context, OverlapReport& report);

    //==== Rendering helpers ====

    // Creates an 'overlap mesh' in the session layer, to visualize the results.
    //
    // \param[in]   context     Clash detection context
    // \param[in]   index       Index between 0 and the number returned by getNbOverlaps
    // \param[in]   recordIndex Record index between 0 and OverlapData::mNbRecords
    // \param[in]   printNames  (Debug) True to print mesh names to the console
    // \param[in]   mode        Desired mode (tbd)
    // \return  True if success
    // \see releaseOverlapMeshes
    bool(CARB_ABI* createOverlapMeshes)(void* context, uint32_t index, uint32_t recordIndex, bool printNames, uint32_t mode);

    // Releases all previously created 'overlap meshes'.
    //
    // \param[in]   context     Clash detection context
    // \see createOverlapMeshes
    void(CARB_ABI* releaseOverlapMeshes)(void* context);

    //==== Misc helpers ====

    // Small helper. Retrieves data from the timeline.
    //
    // \param[out]  data    Timeline data
    void(CARB_ABI* getTimeLineData)(TimelineData& data);

    //==== Stats ====

    // Retrieves various stats after a query has returned.
    //
    // \param[in]   context     Clash detection context
    // \param[out]  stats       Stats
    void(CARB_ABI* getQueryStats)(void* context, Stats& stats);

    //==== Memory helpers ====

    // Small helper. Frees data allocated by clash detection allocator.
    //
    // \param[in]  ptr  Pointer to memory
    void(CARB_ABI* releaseMemory)(void* ptr);

    // Small helper. Returns if clash detection engine ran out of memory during its last run.
    //
    // \return  True if it did run out of memory, false otherwise.
    bool(CARB_ABI* isOutOfMemory)();

    // Small helper. Returns if clash detection engine has been cancelled, either internally or externally.
    //
    // The pipeline can get cancelled internally when reaching internal limits.
    //
    // \return  True if it was cancelled, false otherwise.
    bool(CARB_ABI* isCancelled)();
};

} // namespace clashdetection
} // namespace omni
