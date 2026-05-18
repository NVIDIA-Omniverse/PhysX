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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_NARROWPHASE_CORE_H
#define PXG_NARROWPHASE_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxUserAllocated.h"

#include "CmPinnableArray.h"
#include "CmPinnableObject.h"
#include "PxvNphaseImplementationContext.h"
#include "PxsContactManagerState.h"

#include "PxgContactManager.h"
#include "PxgCudaBuffer.h"
#include "PxgCudaPagedLinearAllocator.h"
#include "PxgCopyManager.h"
#include "PxgGeometryManager.h"
#include "PxgShapeManager.h"
#include "PxgRadixSortDesc.h"
#include "PxgBucket.h"
#include "cudaNpCommon.h"

namespace physx
{
	class PxgCudaKernelWranglerManager;
	class PxgNphaseImplementationContext;
	class PxgGpuContext;
	struct PxgFemFemContactInfo;
	struct PxgFemOtherContactInfo;
	struct PxgPersistentContactManifold;
	class PxgParticleSystemCore;

	class PxsTransformCache;
	struct PxsMaterialData;
	struct PxsCachedTransform;

	struct PxcDataStreamPool;

	class PxBaseTask;
	class PxCudaContextManager;
	class PxRenderOutput;
	struct PxTriangleMeshGeometryLL;

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


	struct PxgContactManagers : public PxsContactManagerBase
	{
		PxgContactManagers(const PxU32 bucketId, Cm::VirtualAllocatorCallback& hostAlloc) : PxsContactManagerBase(bucketId), 
			mGpuInputContactManagers(hostAlloc, PxsHeapStats::eNARROWPHASE), 
			mCpuContactManagerMapping(hostAlloc, PxsHeapStats::eNARROWPHASE),
			mShapeInteractions(hostAlloc, PxsHeapStats::eNARROWPHASE),
			mRestDistances(hostAlloc, PxsHeapStats::eNARROWPHASE),
			mTorsionalProperties(hostAlloc, PxsHeapStats::eNARROWPHASE)
		{
		}

		Cm::PinnableArray<PxgContactManagerInput>		mGpuInputContactManagers;
		Cm::PinnableArray<PxsContactManager*>			mCpuContactManagerMapping;
		Cm::PinnableArray<const Sc::ShapeInteraction*>	mShapeInteractions;
		Cm::PinnableArray<PxReal>						mRestDistances;
		Cm::PinnableArray<PxsTorsionalFrictionData>		mTorsionalProperties;

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
		Cm::PinnableArray<PxsContactManagerOutput>	mGpuOutputContactManagers;

		PxgNewContactManagers(const PxU32 bucketIndex, Cm::VirtualAllocatorCallback& hostAlloc) : PxgContactManagers(bucketIndex, hostAlloc),
			mGpuOutputContactManagers(hostAlloc)
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
		Cm::PinnableObject<uint2>                     mLostAndTotalReportedPairsCountMapped;
		const PxU32                                   mBucketIndex;
		
		PxgGpuContactManagers(const PxU32 bucketIndex, PxgAllocatorDesc& allocDesc) :
			mContactManagerInputData(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE), 
			mContactManagerOutputData(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
			mPersistentContactManifolds(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE), 
			mTempRunsumArray(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE), 
			mTempRunsumArray2(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
			mBlockAccumulationArray(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE), 
			mLostFoundPairsOutputData(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE), 
			mLostFoundPairsCms(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
			mCpuContactManagerMapping(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
			mShapeInteractions(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE), 
			mRestDistances(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE), 
			mTorsionalProperties(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
			mLostAndTotalReportedPairsCountMapped(allocDesc.hostMappedAlloc, PxsHeapStats::eNARROWPHASE, Cm::PinnableAllocatorFallback::eDISABLED),
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

		PxgGpuPairManagementBuffers(PxgHeapMemoryAllocator& deviceAlloc): 
			mTempRunsumArray(deviceAlloc, PxsHeapStats::eNARROWPHASE),
			mBlockAccumulationArray(deviceAlloc, PxsHeapStats::eNARROWPHASE),
			mRemovedIndicesArray(deviceAlloc, PxsHeapStats::eNARROWPHASE)
		{
		}

	private:
		PX_NOCOPY(PxgGpuPairManagementBuffers)
	};
	
	
	class PxgMirroredContactManagerPair : public PxUserAllocated
	{
	public:
		PxgMirroredContactManagerPair(const PxU32 bucketId, Cm::VirtualAllocatorCallback& hostAlloc) :
			mContactManagers(bucketId, hostAlloc),
			mNewContactManagers(bucketId, hostAlloc)
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
		PxgGpuContactManagerPair(const PxU32 bucketId, PxgAllocatorDesc& allocDesc) :
			mContactManagers(bucketId, allocDesc),
			mNewContactManagers(bucketId, allocDesc)
		{
		}

		PX_FORCE_INLINE void allocateBlockAccumulationArray(const PxU32 size)
		{
			mContactManagers.mBlockAccumulationArray.allocate(sizeof(PxU32) * size, PX_FL);
			mNewContactManagers.mBlockAccumulationArray.allocate(sizeof(PxU32) * size, PX_FL);
		}

		PX_FORCE_INLINE PxU32 getTotalLostFoundPairs()
		{
			const PxU32 a = mContactManagers.mLostAndTotalReportedPairsCountMapped.isValid() ? mContactManagers.mLostAndTotalReportedPairsCountMapped.get().y : 0;
			const PxU32 b = mNewContactManagers.mLostAndTotalReportedPairsCountMapped.isValid() ? mNewContactManagers.mLostAndTotalReportedPairsCountMapped.get().y : 0;
			return a + b;
		}

		PX_FORCE_INLINE PxU32 getTotalLostFoundPatches()
		{
			const PxU32 a = mContactManagers.mLostAndTotalReportedPairsCountMapped.isValid() ? mContactManagers.mLostAndTotalReportedPairsCountMapped.get().x : 0;
			const PxU32 b = mNewContactManagers.mLostAndTotalReportedPairsCountMapped.isValid() ? mNewContactManagers.mLostAndTotalReportedPairsCountMapped.get().x : 0;
			return a + b;
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

		typedef Cm::PinnableArray<PxU32> RemovedIndicesArray;

		RemovedIndicesArray*					mRemovedIndices[GPU_BUCKET_ID::eCount];

		PxBitMap										mKeepMap;

		Cm::PinnableArray<PxsContactManagerOutputCounts>	mLostFoundPairsOutputData;
		Cm::PinnableArray<PxsContactManager*>				mLostFoundPairsCms;

		PxU32												mTotalLostFoundPairs;
		PxU32												mTotalLostFoundPatches;
		PxU32												mTotalNumPairs;

		Cm::PinnableArray<PxgPairManagementData>			mPairManagementData;
		PxgCudaBuffer										mGpuPairManagementData;
	

		Cm::PinnableArray<PxgRadixSortDesc>		mRSDesc;
		PxgCudaBufferN<2>						mRadixSortDescBuf; //radix sort with rank

		PxgCudaBuffer							mTempGpuRigidIndiceBuf;
		PxgCudaBuffer							mTempGpuShapeIndiceBuf;
		PxgCudaBuffer							mRadixCountTotalBuf;

		CUdeviceptr								mContactStream;
		CUdeviceptr								mPatchStream;
		CUdeviceptr								mForceAndIndiceStream;

		//device memory
		PxgTypedCudaBuffer<PxgPatchAndContactCounters>	mPatchAndContactCountersOnDevice;
		Cm::PinnableObject<PxgPatchAndContactCounters>	mPatchAndContactCountersReadback;

		PxgShapeManager							mGpuShapesManager;
		PxgMaterialManager						mGpuMaterialManager;
		PxgFEMSoftBodyMaterialManager			mGpuFEMMaterialManager;
		PxgFEMClothMaterialManager				mGpuFEMClothMaterialManager;
		PxgPBDMaterialManager					mGpuPBDMaterialManager;

		//device memory
		PxgCudaPagedLinearAllocator				mIntermStackAlloc;

		CUstream								mStream;
		CUstream								mSolverStream; //this is the stream handle belong to the solver, we can't destroy the solver stream
		PxgCudaKernelWranglerManager*			mGpuKernelWranglerManager;
		PxCudaContextManager*					mCudaContextManager;
		PxCudaContext*							mCudaContext;

		PxgCopyManager							mCopyMan;
	    PxgCopyManager							mCopyManBp;
		PxgGeometryManager						mGeometryManager;

		IG::IslandSim*							mIslandSim;

		PxgNphaseImplementationContext*			mNphaseImplContext;
		PxgGpuContext*							mGpuContext;

		PxU32									mCollisionStackSizeBytes;

		// needs to be mapped pinned memory
		Cm::PinnableObject<PxU32>				mMaxConvexMeshTempMemoryMapped;

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
				PxgContactManagers& contactManagers, PxgGpuContactManagers& gpuContactManagers, Cm::PinnableArray<PxU32>& removedIndices, PxgGpuPairManagementBuffers& pairManagementBuffers,
			PxU16 stage5KernelID, const bool copyManifold = true);

			


	public:

		PxgGpuNarrowphaseCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, const PxGpuDynamicsMemoryConfig& gpuDynamicsConfig, void* contactStreamBase,
			void* patchStreamBase, void* forceAndIndiceStreamBase, IG::IslandSim* islandSim, CUstream solverStream, PxgAllocatorDesc& allocDesc,
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


		Cm::PinnableArray<PxsContactManagerOutputCounts>& getLostFoundPairsOutput() { return mLostFoundPairsOutputData; }
		Cm::PinnableArray<PxsContactManager*>& getLostFoundPairsCms() { return mLostFoundPairsCms; }

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

		void adjustNpIndices(PxgNewContactManagers& newContactManagers, Cm::PinnableArray<PxgContactManagerInput>& itMainInputs,
			Cm::PinnableArray<PxsContactManager*>& itCms, Cm::PinnableArray<const Sc::ShapeInteraction*>& itSIs,
			Cm::PinnableArray<PxReal>& itR, Cm::PinnableArray<PxsTorsionalFrictionData>& itTor,
			Cm::PinnableArray<PxgContactManagerInput>& itNewInputs,
			Cm::PinnableArray<PxsContactManager*>& itNewCms,
			Cm::PinnableArray<const Sc::ShapeInteraction*>& itNewSIs, Cm::PinnableArray<PxReal>& itNewR,
			Cm::PinnableArray<PxsTorsionalFrictionData>& itNewTor);

		void registerContactManagerInternal(PxsContactManager* cm, const Sc::ShapeInteraction* shapeInteraction, PxgContactManagerInput* input, PxsContactManagerOutput& output, PxgNewContactManagers& newContactManagers);

		void unregisterContactManagerInternal(PxsContactManager* cm, Cm::PinnableArray<PxU32>& removedIndices, PxgNewContactManagers& newContactManagers);
	
		void refreshContactManagerInternal(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs, const Sc::ShapeInteraction** shapeInteractions, PxgContactManagerInput& input, PxgNewContactManagers& newContactManagers,
			Cm::PinnableArray<PxU32>& removedIndices);

		template <typename Manifold> 
		void prepareTempContactManagers(PxgGpuContactManagers& gpuManagers, PxgNewContactManagers& newManagers, Manifold* emptyManifold);

		void prepareTempContactManagers(PxgGpuContactManagers& gpuManagers, PxgNewContactManagers& newManagers);

		void removeLostPairsInternal(Cm::PinnableArray<PxU32>& removedIndices, PxgContactManagers& contactManagers);

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
