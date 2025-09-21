// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_NARROWPHASE_CORE_H
#define PXG_NARROWPHASE_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxPinnedArray.h"
#include "foundation/PxUserAllocated.h"
#include "task/PxTask.h"
#include "geometry/PxMeshScale.h"

#include "foundation/PxMutex.h"
#include "PxsContactManagerState.h"
#include "PxgContactManager.h"
#include "PxgPersistentContactManifold.h"
#include "PxgCudaBuffer.h"
#include "cudaNpCommon.h"

#include "PxgCudaPagedLinearAllocator.h"
#include "PxgCopyManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgGeometryManager.h"
#include "PxvNphaseImplementationContext.h"
#include "PxgHeapMemAllocator.h"
#include "PxgShapeManager.h"
#include "PxgRadixSortDesc.h"
#include "geometry/PxParticleSystemGeometry.h"
#include "PxvGeometry.h"

namespace physx
{
	class PxsTransformCache;
	struct PxsMaterialData;
	struct PxsCachedTransform;
	struct PxcDataStreamPool;
	class PxgCudaKernelWranglerManager;
	class PxgNphaseImplementationContext;
	
	class PxRenderOutput;
	class PxgGpuContext;
	struct PxTriangleMeshGeometryLL;
	class PxgParticleSystemCore;

	struct PxgFemFemContactInfo;
	struct PxgFemOtherContactInfo;

	namespace Gu
	{
		struct ConvexHullData;
		class TriangleMesh;
		class DeformableVolumeMesh;
		class BVTetrahedronMesh;
		struct HeightFieldData;
		class PersistentContactManifold;
	}

	namespace Cm
	{
		class FlushPool;
	}

	namespace IG
	{
		class IslandSim;
	}


	struct GPU_BUCKET_ID
	{
		enum Enum
		{
			eFallback = 0,
			eConvex = 1,						//manifold
			eConvexPlane = 2,					//manifold
			eConvexTrimesh = 3,					//multi-manifold
			eConvexHeightfield = 4,				//multi-manifold
			eSphereTrimesh = 5,					//multi-manifold
			eSphereHeightfield = 6,				//multi-manifold
			eTrianglePlane = 7,					//multi-manifold - technically could be single manifold but needs 6 points

			eSphere = 8,						//no manifold
			eBoxBox = 9,

			eConvexCorePlane = 10,
			eConvexCoreConvex = 11,
			eConvexCoreTrimesh = 12,
			eConvexCoreTetmesh = 13,
			eConvexCoreClothmesh = 14,

			eTriangleHeightfield = 15,			//no mainfold
			eTriangleTriangle = 16,				//no manifold
			eSoftbody = 17,						//no manifold
			eSoftbodies = 18,					//no manifold
			eSoftbodyFemCloth = 19,				//no manifold
			eSoftbodyTrimesh = 20,				//no manifold
			eSoftbodySdfTrimesh = 21,			//no manifold
			eSoftbodyHeightfield = 22,			//no manifold
			eFemClothSphere = 23,				//no manifold
			eFemClothPlane = 24,				//no manifold
			eFemClothBox = 25,					//no manifold
			eFemClothConvexes = 26,				//no manifold
			eFemClothes = 27,					//no manifold
			eFemClothTrimesh = 28,				//no manifold
			eFemClothSdfTrimesh = 29,
			eFemClothHeightfield = 30,			//no manifold
			eConvexParticle = 31,				//no manifold
			eParticlesystems = 32,				//no manifold
			eParticlesystemSoftbody = 33,		//no manifold
			eParticlesystemFemCloth = 34,		//no manifold
			eParticlesystemTrimesh = 35,		//no manifold
			eParticlesystemSdfTrimesh = 36,		//no manifold
			eParticlesystemHeightfield = 37,	//no manifold

			eCount
		};
	};

	const PxU32 BUCKET_ManifoldSize[GPU_BUCKET_ID::eCount] = {
		0,														//0
		sizeof(PxgPersistentContactManifold),					//1
		sizeof(PxgPersistentContactManifold),					//2
		sizeof(PxgPersistentContactMultiManifold),				//3
		sizeof(PxgPersistentContactMultiManifold),				//4
		sizeof(PxgPersistentContactMultiManifold),				//5
		sizeof(PxgPersistentContactMultiManifold),				//6
		sizeof(PxgPersistentContactMultiManifold),				//7
		0,														//8
		0,														//9
		0,														//10
		0,														//11
		0,														//12
		0,														//13
		0,														//14
		0,														//15
		0,														//16
		0,														//17
		0,														//18
		0,														//19
		0,														//20
		0,														//21
		0,														//22
		0,														//23
		0,														//24
		0,														//25
		0,														//26
		0,														//27
		0,														//28
		0,														//29
		0,														//30
		0,														//31
		0,														//32
		0,														//33
		0,														//34
		0,														//35
		0,														//36
		0,														//37
	};

	struct PxgContactManagers : public PxsContactManagerBase
	{
		PxgContactManagers(const PxU32 bucketId, const PxVirtualAllocator& allocator) : PxsContactManagerBase(bucketId), 
			mGpuInputContactManagers(allocator), 
			mCpuContactManagerMapping(allocator),
			mShapeInteractions(allocator),
			mRestDistances(allocator),
			mTorsionalProperties(allocator)
		{
		}

		PxPinnedArray<PxgContactManagerInput>		mGpuInputContactManagers;
		PxPinnedArray<PxsContactManager*>			mCpuContactManagerMapping;
		PxPinnedArray<const Sc::ShapeInteraction*>	mShapeInteractions;
		PxFloatArrayPinned							mRestDistances;
		PxPinnedArray<PxsTorsionalFrictionData>		mTorsionalProperties;

		void clear()
		{
			mGpuInputContactManagers.forceSize_Unsafe(0);
			mCpuContactManagerMapping.forceSize_Unsafe(0);
			mShapeInteractions.forceSize_Unsafe(0);
			mRestDistances.forceSize_Unsafe(0);
			mTorsionalProperties.forceSize_Unsafe(0);
		}

		void preallocateNewBuffers(PxU32 nbToPreallocate)
		{
			mGpuInputContactManagers.reserve(nbToPreallocate);
			mCpuContactManagerMapping.reserve(nbToPreallocate);
			mShapeInteractions.reserve(nbToPreallocate);
			mRestDistances.reserve(nbToPreallocate);
			mTorsionalProperties.reserve(nbToPreallocate);
		}

	private:
		PX_NOCOPY(PxgContactManagers)
	};

	struct PxgNewContactManagers : public PxgContactManagers
	{
		PxPinnedArray<PxsContactManagerOutput>	mGpuOutputContactManagers;

		PxgNewContactManagers(const PxU32 bucketIndex, const PxVirtualAllocator& allocator) : PxgContactManagers(bucketIndex, allocator),
			mGpuOutputContactManagers(allocator)
		{
		}

		void clear()
		{
			PxgContactManagers::clear();
			mGpuOutputContactManagers.forceSize_Unsafe(0);
		}

		void preallocateNewBuffers(PxU32 nbToPreallocate)
		{
			mGpuOutputContactManagers.reserve(nbToPreallocate);
			PxgContactManagers::preallocateNewBuffers(nbToPreallocate);
		}
	};

	struct PxgGpuContactManagers
	{
		PxgTypedCudaBuffer<PxgContactManagerInput>    mContactManagerInputData;
		PxgTypedCudaBuffer<PxsContactManagerOutput>   mContactManagerOutputData;
		PxgCudaBuffer                                 mPersistentContactManifolds;

		PxgTypedCudaBuffer<PxU32>                     mTempRunsumArray;
		PxgTypedCudaBuffer<PxU32>                     mTempRunsumArray2;
		PxgTypedCudaBuffer<PxU32>                     mBlockAccumulationArray;
		PxgTypedCudaBuffer<PxsContactManagerOutputCounts> mLostFoundPairsOutputData;
		PxgTypedCudaBuffer<PxsContactManager*>        mLostFoundPairsCms;
		PxgTypedCudaBuffer<PxsContactManager*>        mCpuContactManagerMapping;
		PxgTypedCudaBuffer<Sc::ShapeInteraction*>     mShapeInteractions;
		PxgTypedCudaBuffer<PxReal>                    mRestDistances;
		PxgTypedCudaBuffer<PxsTorsionalFrictionData>  mTorsionalProperties;
		uint2*								mLostAndTotalReportedPairsCountPinned;	
		const PxU32							mBucketIndex;
		
		PxgGpuContactManagers(const PxU32 bucketIndex, PxgHeapMemoryAllocatorManager* manager) :
			mContactManagerInputData(manager, PxsHeapStats::eNARROWPHASE), 
			mContactManagerOutputData(manager, PxsHeapStats::eNARROWPHASE),
			mPersistentContactManifolds(manager, PxsHeapStats::eNARROWPHASE), 
			mTempRunsumArray(manager, PxsHeapStats::eNARROWPHASE), 
			mTempRunsumArray2(manager, PxsHeapStats::eNARROWPHASE),
			mBlockAccumulationArray(manager, PxsHeapStats::eNARROWPHASE), 
			mLostFoundPairsOutputData(manager, PxsHeapStats::eNARROWPHASE), 
			mLostFoundPairsCms(manager, PxsHeapStats::eNARROWPHASE),
			mCpuContactManagerMapping(manager, PxsHeapStats::eNARROWPHASE),
			mShapeInteractions(manager, PxsHeapStats::eNARROWPHASE), 
			mRestDistances(manager, PxsHeapStats::eNARROWPHASE), 
			mTorsionalProperties(manager, PxsHeapStats::eNARROWPHASE),
			mLostAndTotalReportedPairsCountPinned(NULL),
			mBucketIndex(bucketIndex)
		{
			PX_ASSERT(bucketIndex < (1 << PxsContactManagerBase::MaxBucketBits));
		}

	private:
		PX_NOCOPY(PxgGpuContactManagers)
	};

	struct PxgGpuPairManagementBuffers
	{
		PxgCudaBuffer					mTempRunsumArray;
		PxgCudaBuffer					mBlockAccumulationArray;
		PxgCudaBuffer					mRemovedIndicesArray;

		PxgGpuPairManagementBuffers(PxgHeapMemoryAllocatorManager* manager): 
			mTempRunsumArray(manager, PxsHeapStats::eNARROWPHASE), mBlockAccumulationArray(manager, PxsHeapStats::eNARROWPHASE), mRemovedIndicesArray(manager, PxsHeapStats::eNARROWPHASE)
		{
		}

	private:
		PX_NOCOPY(PxgGpuPairManagementBuffers)
	};
	
	
	class PxgMirroredContactManagerPair : public PxUserAllocated
	{
	public:
		PxgMirroredContactManagerPair(const PxU32 bucketId, const PxVirtualAllocator& allocator) :
			mContactManagers(bucketId, allocator),
			mNewContactManagers(bucketId, allocator)
		{
		}

		PX_FORCE_INLINE PxU32 getNbFirstPassTests() { return mContactManagers.mCpuContactManagerMapping.size(); }
		PX_FORCE_INLINE PxU32 getNbSecondPassTests() { return mNewContactManagers.mCpuContactManagerMapping.size(); }

		PX_FORCE_INLINE PxU32 getNbPassTests() { return mContactManagers.mCpuContactManagerMapping.size() + mNewContactManagers.mCpuContactManagerMapping.size(); }

		PX_FORCE_INLINE void allocateContactManagers()
		{
			const PxU32 newPairCount = mContactManagers.mGpuInputContactManagers.size() + mNewContactManagers.mGpuInputContactManagers.size();

			if (newPairCount > mContactManagers.mGpuInputContactManagers.capacity())
			{
				PxU32 newSize = PxMax(newPairCount, mContactManagers.mGpuInputContactManagers.capacity() * 2);

				mContactManagers.mGpuInputContactManagers.reserve(newSize);

				//mContactManagers.mCpuContactManagerMapping.reserve(newSize);
			}
		}

		PxgContactManagers			mContactManagers;			//existing contact managers
		PxgNewContactManagers		mNewContactManagers;		//new contact managers

	};

	class PxgGpuContactManagerPair : public PxUserAllocated
	{
	public:
		PxgGpuContactManagerPair(const PxU32 bucketId, PxgHeapMemoryAllocatorManager* manager) :
			mContactManagers(bucketId, manager),
			mNewContactManagers(bucketId, manager)
		{
		}

		PX_FORCE_INLINE void allocateBlockAccumulationArray(const PxU32 size)
		{
			mContactManagers.mBlockAccumulationArray.allocate(sizeof(PxU32) * size, PX_FL);
			mNewContactManagers.mBlockAccumulationArray.allocate(sizeof(PxU32) * size, PX_FL);
		}

		PX_FORCE_INLINE void allocateLostAndTotalReportedPairsCount(PxsHeapMemoryAllocator* mappedMemoryAllocators)
		{
			mContactManagers.mLostAndTotalReportedPairsCountPinned = reinterpret_cast<uint2*>(mappedMemoryAllocators->allocate(sizeof(uint2), PxsHeapStats::eNARROWPHASE, PX_FL));
			mNewContactManagers.mLostAndTotalReportedPairsCountPinned = reinterpret_cast<uint2*>(mappedMemoryAllocators->allocate(sizeof(uint2), PxsHeapStats::eNARROWPHASE, PX_FL));
		}

		PX_FORCE_INLINE PxU32 getTotalLostFoundPairs()
		{
			return mContactManagers.mLostAndTotalReportedPairsCountPinned->y + mNewContactManagers.mLostAndTotalReportedPairsCountPinned->y;
		}

		PX_FORCE_INLINE PxU32 getTotalLostFoundPatches()
		{
			return mContactManagers.mLostAndTotalReportedPairsCountPinned->x + mNewContactManagers.mLostAndTotalReportedPairsCountPinned->x;
		}

		PxgGpuContactManagers			mContactManagers;
		PxgGpuContactManagers			mNewContactManagers;
	};

	struct PxsShapeCore;
	struct PxgKernelIds;
	
	class PxgGpuNarrowphaseCore : public PxUserAllocated
	{
		PX_NOCOPY(PxgGpuNarrowphaseCore)

	public:
		PxgMirroredContactManagerPair*			mContactManagers[GPU_BUCKET_ID::eCount];		//some order as BUCKET_ID. 0 is fallback and it will be NULL in this array
		PxgGpuContactManagerPair*				mGpuContactManagers[GPU_BUCKET_ID::eCount];	//some order as BUCKET_ID. 0 is fallback and it will be NULL in this array

		PxgGpuPairManagementBuffers				mPairManagementBuffers;

		PxgCudaBuffer							mGpuTransformCache;

		PxgCudaBuffer							mGpuContactDistance;

		typedef PxInt32ArrayPinned RemovedIndicesArray;

		RemovedIndicesArray*					mRemovedIndices[GPU_BUCKET_ID::eCount];

		PxBitMap										mKeepMap;

		PxPinnedArray<PxsContactManagerOutputCounts>	mLostFoundPairsOutputData;
		PxPinnedArray<PxsContactManager*>				mLostFoundPairsCms;

		PxU32									mTotalLostFoundPairs;
		PxU32									mTotalLostFoundPatches;
		PxU32									mTotalNumPairs;

		PxgPairManagementData* 					mPairManagementData[GPU_BUCKET_ID::eCount]; //mapped memory

		PxgCudaBuffer							mGpuPairManagementData;
	

		PxPinnedArray<PxgRadixSortDesc>			mRSDesc;
		PxgCudaBufferN<2>						mRadixSortDescBuf; //radix sort with rank

		PxgCudaBuffer							mTempGpuRigidIndiceBuf;
		PxgCudaBuffer							mTempGpuShapeIndiceBuf;
		PxgCudaBuffer							mRadixCountTotalBuf;

		CUdeviceptr								mContactStream;
		CUdeviceptr								mPatchStream;
		CUdeviceptr								mForceAndIndiceStream;
		
		PxgTypedCudaBuffer<PxgPatchAndContactCounters> mPatchAndContactCountersOnDevice; //device memory
		PxgPatchAndContactCounters*             mPatchAndContactCountersReadback; //host memory


		PxgShapeManager							mGpuShapesManager;
		PxgMaterialManager						mGpuMaterialManager;
		PxgFEMSoftBodyMaterialManager			mGpuFEMMaterialManager;
		PxgFEMClothMaterialManager				mGpuFEMClothMaterialManager;
		PxgPBDMaterialManager					mGpuPBDMaterialManager;

		//device memory
		PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator> mIntermStackAlloc;

		CUstream								mStream;
		CUstream								mSolverStream; //this is the stream handle belong to the solver, we can't destroy the solver stream
		PxgCudaKernelWranglerManager*			mGpuKernelWranglerManager;
		PxCudaContextManager*					mCudaContextManager;
		PxCudaContext*							mCudaContext;
		PxgHeapMemoryAllocatorManager*			mHeapMemoryManager;

		PxgCopyManager							mCopyMan;
	    PxgCopyManager							mCopyManBp;
		PxgGeometryManager						mGeometryManager;

		IG::IslandSim*							mIslandSim;

		PxgNphaseImplementationContext*			mNphaseImplContext;
		PxgGpuContext*							mGpuContext;

		PxU32									mCollisionStackSizeBytes;

		PxU32*									mMaxConvexMeshTempMemory;

		struct RefcountedRecord
		{
			PxU32 refCnt;
			PxU32 idx;
		};

		typedef PxHashMap<size_t, RefcountedRecord> RefcountedRecordsMap;
		RefcountedRecordsMap*									mShapesMap;

		RefcountedRecordsMap*									mGeometriesMap;
		RefcountedRecordsMap*									mMaterialsMap;
		RefcountedRecordsMap*									mFEMMaterialsMap;
		RefcountedRecordsMap*									mFEMClothMaterialsMap;
		RefcountedRecordsMap*									mPBDMaterialsMap;

		PxgCudaBuffer											mGpuMultiManifold;
		PxgCudaBuffer											mGpuManifold;

		CUevent													mParticleEvent;
		CUevent													mSoftbodyEvent;
		CUevent													mFemClothEvent;
		CUevent													mDirectApiDmaEvent;

#if PX_ENABLE_SIM_STATS
		PxU32													mGpuDynamicsRigidContactCountStats;
		PxU32													mGpuDynamicsRigidPatchCountStats;
		PxU32													mGpuDynamicsCollisionStackSizeStats;
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
		
		void createGpuStreamsAndEvents();
		void releaseGpuStreamsAndEvents();


		void removeLostPairsGpu(const PxU32 bucketID, const PxU16 stage5KernelID, const bool copyManifold);
		
		void appendContactManagersGpu(PxU32 nbExistingManagers, PxU32 nbNewManagers, PxgGpuContactManagers& gpuContactManagers, PxgGpuContactManagers& newGpuContactManagers, PxU32 manifoldSize);
		void drawManifold(PxgPersistentContactManifold* manifolds, PxgContactManagerInput* cmInput, PxsCachedTransform* cachedTransform, const PxU32 numTests, PxRenderOutput& renderOutput,
			const PxU32 color, const PxF32 size);
		void drawPoint(PxRenderOutput& renderOutput, PxVec3 point, const PxU32 color, const PxF32 size);
		void drawLine(PxRenderOutput& out, PxVec3 a, const PxVec3 b, const PxU32 color);

		template <typename ManagementData, typename Manifold>
			void removeLostPairsGpuInternal( ManagementData& cpuBuffer, CUdeviceptr gpuBuffer,
				PxgContactManagers& contactManagers, PxgGpuContactManagers& gpuContactManagers, PxInt32ArrayPinned& removedIndices, PxgGpuPairManagementBuffers& pairManagementBuffers,
			PxU16 stage5KernelID, const bool copyManifold = true);

			


	public:

		PxgGpuNarrowphaseCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, const PxGpuDynamicsMemoryConfig& gpuDynamicsConfig, void* contactStreamBase,
			void* patchStreamBase, void* forceAndIndiceStreamBase, IG::IslandSim* islandSim, CUstream solverStream, PxgHeapMemoryAllocatorManager* heapMemoryManager,
			PxgNphaseImplementationContext* nphaseImplContext);
		virtual ~PxgGpuNarrowphaseCore();		
		
		void testSDKSphereGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces, PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
			PxRenderOutput* renderOutput);

		void testSDKBoxBoxGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces, PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
			PxRenderOutput* renderOutput);

		void testSDKConvexConvexGjkEpaGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces, PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit, 
			PxRenderOutput* renderOutput);

		void testSDKConvexPlaneGjkEpaGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces, PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
			PxRenderOutput* renderOutput);

		void testSDKConvexCorePlaneGjkEpaGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces, PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
			PxRenderOutput* renderOutput);

		void testSDKConvexCoreConvexGjkEpaGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces, PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
			PxRenderOutput* renderOutput);

		void testSDKConvexCoreTrimeshGjkEpaGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces, PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
			PxRenderOutput* renderOutput);

		void testSDKConvexCoreTetmeshGjkEpaGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKConvexCoreClothmeshGjkEpaGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKSphereTriMeshSATGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
			PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit);

		void testSDKSphereHeightfieldGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
			PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit);

		void testSDKTriMeshPlaneGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
			PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit);

		void testSDKTriMeshHeightfieldGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
			PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit);
		
		void testSDKTriMeshTriMeshGpu(PxgGpuContactManagers& gpuManagers,const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
			PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit);

		void testSDKConvexTriMeshSATGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
			PxU32 patchBytesLimit, PxU32 contactBytesLimit,	PxU32 forceBytesLimit);

		void testSDKConvexHeightfieldGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
			PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
			PxU32 patchBytesLimit, PxU32 contactBytesLimit,	PxU32 forceBytesLimit);

		void testSDKParticleSystemGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests);

		void testSDKParticleSoftbody(PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
			PxRenderOutput* renderOutput);

		void testSDKParticleFemCloth(PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
			PxRenderOutput* renderOutput);

		void testSDKConvexParticle(PxgGpuContactManagers& gpuManagers, const PxU32 numTests);

		void testSDKParticleSdfTriMesh(PxgGpuContactManagers& gpuManagers, const PxU32 numTests);

		void testSDKParticleTriMesh(PxgGpuContactManagers& gpuManagers,const PxU32 numTests);

		void testSDKParticleHeightfield(PxgGpuContactManagers& gpuManagers, const PxU32 numTests);

		void testSDKSoftbody(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKSoftbodies(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKSoftbodyCloth(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKSoftbodyTrimesh(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKSoftbodySdfTrimesh(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKSoftbodyHeightfield(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKFemClothSphere(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKFemClothPlane(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKFemClothBox(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKFemClothConvexes(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);
		
		void testSDKFemClothCloth(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKFemClothSdfTrimesh(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKFemClothTrimesh(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);

		void testSDKFemClothHeightfield(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput);
		
		void updateFrictionPatches(PxgGpuContactManagers& gpuManagers, PxU32 count, PxU8* baseContactPatches, PxU8* baseFrictionPatches);

		void fetchNarrowPhaseResults(PxcDataStreamPool* contactStreamPool, PxcDataStreamPool* patchStreamPool, PxcDataStreamPool* forceStreamPool,
			PxsContactManagerOutput* cmOutputs, const Sc::ShapeInteraction*const* shapeInteractions, const PxReal* restDistances, const PxsTorsionalFrictionData* torsionalData, PxU32 nbFallbackPairs,
			const PxsContactManagerOutputCounts* foundPatchCounts, const PxsContactManager*const* foundPatchManagers, PxU32 nbFoundPatchManagers);

		void syncNotRigidWithNp();
		
		void acquireContext();
		void releaseContext();

		CUstream getStream();

		void registerContactManager(PxsContactManager* cm, const Sc::ShapeInteraction* shapeInteraction, PxsContactManagerOutput& output, const PxU32 bucketId);
		void unregisterContactManager(PxsContactManager* manager, const PxU32 bucketId);
		void refreshContactManager(PxsContactManager* manager, PxsContactManagerOutput* cmOutputs, PxgContactManagerInput& input, const PxU32 bucketId);

		void removeLostPairs();
		void appendContactManagers(PxsContactManagerOutput* cmOutputs, PxU32 nbFallbackPairs);

		void prepareTempContactManagers();

		template <GPU_BUCKET_ID::Enum>
		void prepareTempContactManagers();

		void prepareTempContactManagersTasks(Cm::FlushPool& flushPool, PxBaseTask* continuation);

		bool isMeshGPUCompatible(const PxTriangleMeshGeometryLL& meshData);

		bool isClothMeshGPUCompatible(const PxTriangleMeshGeometryLL& meshData); // skipping material counts

		bool isTetMeshGPUCompatible(const Gu::BVTetrahedronMesh* meshData);

		void computeRigidsToShapes();

		void prepareGpuNarrowphase(PxsTransformCache& cache, const PxReal* contactDistances, bool hasContactDistanceChanged);

		PxsContactManagerOutput* getGPUContactManagerOutputBase() { return reinterpret_cast<PxsContactManagerOutput*>(mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mContactManagerOutputData.getDevicePtr()); }

		PxReal* getGPURestDistances() { return reinterpret_cast<PxReal*>(mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mRestDistances.getDevicePtr()); }

		PxsTorsionalFrictionData* getGPUTorsionalData() { return reinterpret_cast<PxsTorsionalFrictionData*>(mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mTorsionalProperties.getDevicePtr()); }

		Sc::ShapeInteraction** getGPUShapeInteractions() { return reinterpret_cast<Sc::ShapeInteraction**>(mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mShapeInteractions.getDevicePtr()); }

		PxgContactManagers& getExistingContactManagers(GPU_BUCKET_ID::Enum type) { return mContactManagers[type]->mContactManagers; }
		PxgNewContactManagers& getNewContactManagers(GPU_BUCKET_ID::Enum type) { return mContactManagers[type]->mNewContactManagers; }

		PxgGpuContactManagers& getExistingGpuContactManagers(GPU_BUCKET_ID::Enum type) { return mGpuContactManagers[type]->mContactManagers; }
		PxgGpuContactManagers& getNewGpuContactManagers(GPU_BUCKET_ID::Enum type) { return mGpuContactManagers[type]->mNewContactManagers; }


		PxPinnedArray<PxsContactManagerOutputCounts>& getLostFoundPairsOutput() { return mLostFoundPairsOutputData; }
		PxPinnedArray<PxsContactManager*>& getLostFoundPairsCms() { return mLostFoundPairsCms; }

		PxgCudaBuffer& getTransformCache()	{ return mGpuTransformCache; }
		PxU32 getTotalNbLostFoundPairs() const { return mTotalLostFoundPairs; }
		PxU32 getTotalNbLostFoundPatches() const { return mTotalLostFoundPatches; }


		void preallocateNewBuffers(PxU32 nbNewPairs);

		void pushBuffer();

		PxU32 addHull(const Gu::ConvexHullData& hull);
		PxU32 getHullIdxByHostPtr(const Gu::ConvexHullData* hull);
		void removeGeometry(PxU32 idx);

		//schedules data copies on the near phase stream
		void uploadDataChunksToGpu();
		void waitAndResetCopyQueues();

		//schedules particle material copies on the broad phase stream,
		//because particle materials are already accessed in pre integreation
		//- we migth be able to remove this dependency
	    void uploadDataChunksToGpuBp();
	    void waitAndResetCopyQueuesBp();

		PxU32 addTriMesh(const Gu::TriangleMesh& mesh);
		PxU32 getTriMeshIdxByHostPtr(const Gu::TriangleMesh* mesh);

		PxU32 addHeightfield(const Gu::HeightFieldData& hf);
		PxU32 getHeightfieldIdxByHostPtr(const Gu::HeightFieldData* hf);

		void registerShape(const PxNodeIndex& nodeIndex, const PxsShapeCore& shapeCore, const PxU32 transformCacheID, const bool isFemCloth, PxActor* actor);
		void updateShapeMaterial(const PxsShapeCore& shapeCore);
		PxU32 getShapeIndex(const PxsShapeCore& shapeCore);
		void unregisterShape(const PxsShapeCore& shapeCore, const PxU32 transformCacheID, const bool isFemCloth);

	
		void registerAggregate(const PxU32 transformCacheID);

		void registerMaterial(const PxsMaterialCore& materialCore);
		void updateMaterial(const PxsMaterialCore& materialCore);
		void unregisterMaterial(const PxsMaterialCore& materialCore);

		void registerFEMMaterial(const PxsDeformableSurfaceMaterialCore& materialCore);
		void updateFEMMaterial(const PxsDeformableSurfaceMaterialCore& materialCore);
		void unregisterFEMMaterial(const PxsDeformableSurfaceMaterialCore& materialCore);

		void registerFEMMaterial(const PxsDeformableVolumeMaterialCore& materialCore);
		void updateFEMMaterial(const PxsDeformableVolumeMaterialCore& materialCore);
		void unregisterFEMMaterial(const PxsDeformableVolumeMaterialCore& materialCore);

		void registerParticleMaterial(const PxsPBDMaterialCore& materialCore);
		void updateParticleMaterial(const PxsPBDMaterialCore& materialCore);
		void unregisterParticleMaterial(const PxsPBDMaterialCore& materialCore);

		//direct gpu contact access  
		bool copyContactData(void* data, PxU32* numContactPairs, const PxU32 maxContactPairs, CUevent startEvent,
			CUevent finishEvent, PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces);

		bool evaluateSDFDistances(PxVec4* localGradientAndSDFConcatenated, const PxShapeGPUIndex* shapeIndices, const PxVec4* localSamplePointsConcatenated, const PxU32* samplePointCountPerShape, PxU32 nbElements, PxU32 maxPointCount, CUevent startEvent, CUevent finishEvent);

		void synchronizedStreams(CUstream artiStream);

		PxgShapeManager& getGpuShapeManager() { return mGpuShapesManager;  }

		template <class MaterialData>
		void mapMaterialIndices(PxU16* gpuMaterialIndices, const PxU16* materialIndices, PxU32 numMaterialIndices);

	private:

		void compactLostFoundPairs(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxU32* touchChangeFlags, PxsContactManagerOutput* cmOutputs);

		void softbodyOtherContactApplyCollisionToSimMeshMapping(
			PxgDevicePointer<float4> contactsd,
			PxgDevicePointer<float4> barycentricsd,
			PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd,
			PxgDevicePointer<PxU32> totalNumCountsd,
			PxgDevicePointer<PxU32> prevNumCountsd);

	public:

		void softbodyFemContactApplyCollisionToSimMeshMapping(
			PxgDevicePointer<float4> barycentrics0d,
			PxgDevicePointer<float4> barycentrics1d,
			PxgDevicePointer<PxgFemFemContactInfo> contactInfosd,
			PxgDevicePointer<PxU32> totalNumCountsd,
			PxgDevicePointer<PxU32> prevNumCountsd, bool isSelfCollision, bool isCloth);

	private:

		void adjustNpIndices(PxgNewContactManagers& newContactManagers, PxPinnedArray<PxgContactManagerInput>& itMainInputs,
			PxPinnedArray<PxsContactManager*>& itCms, PxPinnedArray<const Sc::ShapeInteraction*>& itSIs,
			PxFloatArrayPinned& itR, PxPinnedArray<PxsTorsionalFrictionData>& itTor,
			PxPinnedArray<PxgContactManagerInput>& itNewInputs,
			PxPinnedArray<PxsContactManager*>& itNewCms,
			PxPinnedArray<const Sc::ShapeInteraction*>& itNewSIs, PxFloatArrayPinned& itNewR,
			PxPinnedArray<PxsTorsionalFrictionData>& itNewTor);

		void registerContactManagerInternal(PxsContactManager* cm, const Sc::ShapeInteraction* shapeInteraction, PxgContactManagerInput* input, PxsContactManagerOutput& output, PxgNewContactManagers& newContactManagers);

		void unregisterContactManagerInternal(PxsContactManager* cm, PxInt32ArrayPinned& removedIndices, PxgNewContactManagers& newContactManagers);
	
		void refreshContactManagerInternal(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs, const Sc::ShapeInteraction** shapeInteractions, PxgContactManagerInput& input, PxgNewContactManagers& newContactManagers,
			PxInt32ArrayPinned& removedIndices);

		template <typename Manifold> 
		void prepareTempContactManagers(PxgGpuContactManagers& gpuManagers, PxgNewContactManagers& newManagers, Manifold* emptyManifold);

		void prepareTempContactManagers(PxgGpuContactManagers& gpuManagers, PxgNewContactManagers& newManagers);

		void removeLostPairsInternal(PxInt32ArrayPinned& removedIndices, PxgContactManagers& contactManagers);

		void prepareTempContactManagersInternal(PxgNewContactManagers& newManagers, Cm::FlushPool& flushPool, PxBaseTask* continuation);

		void drawContacts(PxRenderOutput& out, CUdeviceptr contactsd, CUdeviceptr normalPensd, const PxU32 numContacts);

		template <typename MaterialCore, typename MaterialData>
		PxU32 registerMaterialInternal(const MaterialCore& materialCore, RefcountedRecordsMap* materialsMap, PxgMaterialManager& materialManager);

		template <typename MaterialCore, typename MaterialData >
		void updateMaterialInternal(const MaterialCore& materialCore, RefcountedRecordsMap* materialsMap, PxgMaterialManager& materialManager);

		template <typename MaterialData>
		PxU16 mapMaterialIndex(PxU16 sdkMaterialIndex, RefcountedRecordsMap* materialsMap, PxgMaterialManager& materialManager)
		{
			return mapMaterialIndexInternal(sdkMaterialIndex, materialsMap, materialManager, sizeof(MaterialData));
		}

		PxU16 mapMaterialIndexInternal(PxU16 sdkMaterialIndex, RefcountedRecordsMap* materialsMap,
			PxgMaterialManager& materialManager, PxU32 materialDataByteSize);

		template <typename MaterialCore>
		void unregisterMaterialInternal(const MaterialCore& materialCore, RefcountedRecordsMap* materialsMap, PxgMaterialManager& materialManager);
	
		void updateContactDistance(const PxReal* contactDistances, const PxU32 numContactDistance);
	};

	template <>
	PX_FORCE_INLINE void PxgGpuNarrowphaseCore::mapMaterialIndices<PxsPBDMaterialData>(PxU16* gpuMaterialIndices,
		const PxU16* materialIndices, PxU32 numMaterialIndices)
	{
		PxgGpuNarrowphaseCore::RefcountedRecordsMap* map = mPBDMaterialsMap;
		PxgPBDMaterialManager& manager = mGpuPBDMaterialManager;
		for(PxU32 i = 0; i < numMaterialIndices; ++i)
		{
			const PxU16 gpuIndex = mapMaterialIndex<PxsPBDMaterialData>(materialIndices[i], map, manager);
			gpuMaterialIndices[i] = gpuIndex;
		}
	}
}

#endif
