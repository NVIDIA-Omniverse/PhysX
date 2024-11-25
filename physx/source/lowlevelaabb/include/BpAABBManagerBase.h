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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef BP_AABBMANAGER_BASE_H
#define BP_AABBMANAGER_BASE_H

#include "foundation/PxAllocator.h"
#include "foundation/PxPinnedArray.h"
#include "foundation/PxBitMap.h"
#include "foundation/PxSList.h"
#include "foundation/PxBitUtils.h"
#include "BpVolumeData.h"
#include "BpBroadPhaseUpdate.h"
#include "GuBounds.h"
#include "PxFiltering.h"
#include "PxAggregate.h"
#include "foundation/PxSimpleTypes.h"

namespace physx
{
class PxcScratchAllocator;
class PxRenderOutput;
class PxBaseTask;

namespace Cm
{
	class FlushPool;
}

namespace Bp
{
	typedef PxU32 BoundsIndex;
	//typedef PxU32 ActorHandle;

	/**
	\brief Changes to the configuration of overlap pairs are reported as void* pairs.
	\note Each void* in the pair corresponds to the void* passed to AABBManager::createVolume.
	\see AABBManager::createVolume, AABBManager::getCreatedOverlaps, AABBManager::getDestroyedOverlaps
	*/
	struct AABBOverlap
	{
		PX_FORCE_INLINE AABBOverlap() {}
		PX_FORCE_INLINE AABBOverlap(void* userData0, void* userData1/*, ActorHandle pairHandle*/) : mUserData0(userData0), mUserData1(userData1)/*, mPairHandle(pairHandle)*/ 
		{
			// PT: TODO: why is this forbidden?
			PX_ASSERT(userData0 != userData1);
		}

		// PT: these will eventually be the userData pointers passed to addBounds(), i.e. Sc::ElementSim pointers in PhysX. This may not be
		// necessary at all, since in the current design the bounds indices themselves come from BP clients (they're not handles managed by the BP).
		// So there's a 1:1 mapping between bounds-indices (which are effectively edlement IDs in PhysX) and the userData pointers (Sc::ElementSim).
		// Thus we could just return bounds indices directly to users - at least in the context of PhysX, maybe the standalone BP is different.
		void*	mUserData0;
		void*	mUserData1;

		// PT: TODO: not sure what happened there but mPairUserData is not used inside the BP itself so we need to revisit this.
		/*		union
				{
					ActorHandle	mPairHandle;		//For created pairs, this is the index into the pair in the pair manager
					void*		mUserData;			//For deleted pairs, this is the user data written by the application to the pair
				};*/
		void*	mPairUserData;			//For deleted pairs, this is the user data written by the application to the pair
	};

	struct BpCacheData : public PxSListEntry
	{
		PxArray<AABBOverlap> mCreatedPairs[2];
		PxArray<AABBOverlap> mDeletedPairs[2];

		void reset()
		{
			mCreatedPairs[0].resizeUninitialized(0);
			mCreatedPairs[1].resizeUninitialized(0);
			mDeletedPairs[0].resizeUninitialized(0);
			mDeletedPairs[1].resizeUninitialized(0);
		}
	};

	typedef	PxPinnedArray<Bp::FilterGroup::Enum>	GroupsArrayPinned;
	typedef	PxPinnedArray<VolumeData>				VolumeDataArrayPinned;
	typedef	PxPinnedArray<ShapeHandle>				ShapeHandleArrayPinned;

	class BoundsArray : public PxUserAllocated
	{
												PX_NOCOPY(BoundsArray)
	public:
												BoundsArray(PxVirtualAllocator& allocator) : mBounds(allocator)	{}

		PX_FORCE_INLINE	void					initEntry(PxU32 index)
												{
													index++;	// PT: always pretend we need one more entry, to make sure reading the last used entry will be SIMD-safe.
													const PxU32 oldCapacity = mBounds.capacity();
													if (index >= oldCapacity)
													{
														const PxU32 newCapacity = PxNextPowerOfTwo(index);
														mBounds.reserve(newCapacity);
														mBounds.forceSize_Unsafe(newCapacity);
													}
												}

		PX_FORCE_INLINE void					updateBounds(const PxTransform& transform, const PxGeometry& geom, PxU32 index)
												{
													Gu::computeBounds(mBounds[index], geom, transform, 0.0f, 1.0f);
													mHasAnythingChanged = true;
												}

		PX_FORCE_INLINE	void					setBounds(const PxBounds3& bounds, PxU32 index)
												{
	//												PX_CHECK_AND_RETURN(bounds.isValid() && !bounds.isEmpty(), "BoundsArray::setBounds - illegal bounds\n");
													mBounds[index] = bounds;
													mHasAnythingChanged = true;
												}

		PX_FORCE_INLINE const PxBounds3*		begin()					const	{ return mBounds.begin();		}
		PX_FORCE_INLINE PxBounds3*				begin()							{ return mBounds.begin();		}
		PX_FORCE_INLINE PxBoundsArrayPinned&	getBounds()						{ return mBounds;				}
		PX_FORCE_INLINE	const PxBounds3&		getBounds(PxU32 index)	const	{ return mBounds[index];		}
		PX_FORCE_INLINE PxU32					size()					const	{ return mBounds.size();		}
		PX_FORCE_INLINE	bool					hasChanged()			const	{ return mHasAnythingChanged;	}
		PX_FORCE_INLINE	void					resetChangedState()				{ mHasAnythingChanged = false;	}
		PX_FORCE_INLINE	void					setChangedState()				{ mHasAnythingChanged = true;	}

						void					shiftOrigin(const PxVec3& shift)
												{
													// we shift some potential NaNs here because we don't know what's active, but should be harmless
													const PxU32 nbBounds = mBounds.size();
													for(PxU32 i=0; i<nbBounds; i++)
													{
														mBounds[i].minimum -= shift;
														mBounds[i].maximum -= shift;
													}
													mHasAnythingChanged = true;
												}
	private:
						PxBoundsArrayPinned		mBounds;
						bool					mHasAnythingChanged;
	};

	/**
	\brief A structure responsible for:
	* storing an aabb representation for each active shape in the related scene
	* managing the creation/removal of aabb representations when their related shapes are created/removed
	* updating all aabbs that require an update due to modification of shape geometry or transform
	* updating the aabb of all aggregates from the union of the aabbs of all shapes that make up each aggregate
	* computing and reporting the incremental changes to the set of overlapping aabb pairs
	*/
	class AABBManagerBase : public PxUserAllocated
	{
												PX_NOCOPY(AABBManagerBase)
	public:
												AABBManagerBase(BroadPhase& bp, BoundsArray& boundsArray, PxFloatArrayPinned& contactDistance,
																PxU32 maxNbAggregates, PxU32 maxNbShapes, PxVirtualAllocator& allocator, PxU64 contextID,
																PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode);

		virtual									~AABBManagerBase() {}

		virtual			void					destroy() = 0;

		virtual			AggregateHandle			createAggregate(BoundsIndex index, Bp::FilterGroup::Enum group, void* userData, PxU32 maxNumShapes, PxAggregateFilterHint filterHint, PxU32 envID) = 0;
		virtual			bool					destroyAggregate(BoundsIndex& index, Bp::FilterGroup::Enum& group, AggregateHandle aggregateHandle) = 0;

		virtual			bool					addBounds(BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userdata, AggregateHandle aggregateHandle, ElementType::Enum volumeType, PxU32 envID) = 0;
		virtual			bool					removeBounds(BoundsIndex index) = 0;

						void					reserveSpaceForBounds(BoundsIndex index);

		PX_FORCE_INLINE	PxIntBool				isMarkedForRemove(BoundsIndex index)	const	{ return mRemovedHandleMap.boundedTest(index);	}
//		PX_FORCE_INLINE	PxIntBool				isMarkedForAdd(BoundsIndex index)		const	{ return mAddedHandleMap.boundedTest(index);	}
		PX_FORCE_INLINE	BroadPhase*				getBroadPhase()							const	{ return &mBroadPhase;							}
		PX_FORCE_INLINE	BoundsArray&			getBoundsArray()								{ return mBoundsArray;							}
		PX_FORCE_INLINE	PxU32					getNbActiveAggregates()					const	{ return mNbAggregates;							}
		PX_FORCE_INLINE	const float*			getContactDistances()					const	{ return mContactDistance.begin();				}
		PX_FORCE_INLINE	PxBitMapPinned&			getChangedAABBMgActorHandleMap()				{ return mChangedHandleMap;						}
		PX_FORCE_INLINE void*					getUserData(const BoundsIndex index)	const	{ return (index<mVolumeData.size()) ? mVolumeData[index].getUserData() : NULL;	}

						void					setContactDistance(BoundsIndex handle, PxReal offset)
												{
													// PT: this works even for aggregated shapes, since the corresponding bit will also be set in the 'updated' map.
													mContactDistance.begin()[handle] = offset;
													setPersistentStateChanged();
													mChangedHandleMap.growAndSet(handle);
												}

						void					setBPGroup(BoundsIndex index, Bp::FilterGroup::Enum group)
												{
													PX_ASSERT((index + 1) < mVolumeData.size());
													PX_ASSERT(group != Bp::FilterGroup::eINVALID);	// PT: we use group == Bp::FilterGroup::eINVALID to mark removed/invalid entries
													mGroups[index] = group;
												}

		virtual			void					updateBPFirstPass(PxU32 numCpuTasks, Cm::FlushPool& flushPool, bool hasContactDistanceUpdated, PxBaseTask* continuation) = 0;
		virtual			void					updateBPSecondPass(PxcScratchAllocator* scratchAllocator, PxBaseTask* continuation)	= 0;

		virtual			void					postBroadPhase(PxBaseTask*, Cm::FlushPool& flushPool) = 0;
		virtual			void					reallocateChangedAABBMgActorHandleMap(const PxU32 size) = 0;

						AABBOverlap*			getCreatedOverlaps(ElementType::Enum type, PxU32& count)
												{
													PX_ASSERT(type < ElementType::eCOUNT);
													count = mCreatedOverlaps[type].size();
													return mCreatedOverlaps[type].begin();
												}

						AABBOverlap*			getDestroyedOverlaps(ElementType::Enum type, PxU32& count)
												{
													PX_ASSERT(type < ElementType::eCOUNT);
													count = mDestroyedOverlaps[type].size();
													return mDestroyedOverlaps[type].begin();
												}

						void					freeBuffers();

						struct OutOfBoundsData
						{
							PxU32	mNbOutOfBoundsObjects;
							PxU32	mNbOutOfBoundsAggregates;
							void**	mOutOfBoundsObjects;
							void**	mOutOfBoundsAggregates;
						};
		virtual			bool					getOutOfBoundsObjects(OutOfBoundsData&)		{ return false;	}
		virtual			void					clearOutOfBoundsObjects()					{}

						void					shiftOrigin(const PxVec3& shift);

		virtual			void					visualize(PxRenderOutput& out) = 0;

		virtual			void					releaseDeferredAggregateIds() = 0;
		virtual			void					setGPUStateChanged()			{}
		virtual			void					setPersistentStateChanged()		{}

#if PX_ENABLE_SIM_STATS
						PxU32					getGpuDynamicsLostFoundPairsStats()				{ return mGpuDynamicsLostFoundPairsStats; }
						PxU32					getGpuDynamicsTotalAggregatePairsStats()		{ return mGpuDynamicsTotalAggregatePairsStats; }
						PxU32					getGpuDynamicsLostFoundAggregatePairsStats()	{ return mGpuDynamicsLostFoundAggregatePairsStats; }
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif


	protected:
						void					reserveShapeSpace(PxU32 nbShapes);

		// PT: we have bitmaps here probably to quickly handle added/removed objects during same frame.
		// PT: TODO: consider replacing with plain arrays (easier to parse, already existing below, etc)
						PxBitMapPinned			mAddedHandleMap;		// PT: indexed by BoundsIndex
						PxBitMapPinned			mRemovedHandleMap;		// PT: indexed by BoundsIndex
						PxBitMapPinned			mChangedHandleMap;

		//Returns true if the bounds was pending insert, false otherwise
		PX_FORCE_INLINE bool					removeBPEntry(BoundsIndex index)	// PT: only for objects passed to the BP
												{
													if (mAddedHandleMap.test(index))		// PT: if object had been added this frame...
													{
														mAddedHandleMap.reset(index);	// PT: ...then simply revert the previous operation locally (it hasn't been passed to the BP yet).
														return true;
													}
													else
														mRemovedHandleMap.set(index);	// PT: else we need to remove it from the BP
													return false;
												}

		PX_FORCE_INLINE void					addBPEntry(BoundsIndex index)
												{
													if (mRemovedHandleMap.test(index))
														mRemovedHandleMap.reset(index);
													else
														mAddedHandleMap.set(index);
												}

		//ML: we create mGroups and mContactDistance in the AABBManager constructor. PxArray will take PxVirtualAllocator as a parameter. Therefore, if GPU BP is using,
		//we will passed a pinned host memory allocator, otherwise, we will just pass a normal allocator.
						GroupsArrayPinned		mGroups;				// NOTE: we stick Bp::FilterGroup::eINVALID in this slot to indicate that the entry is invalid (removed or never inserted.)
						PxInt32ArrayPinned		mEnvIDs;				// PT: should ideally be in the GPU class
						PxFloatArrayPinned& 	mContactDistance;
						VolumeDataArrayPinned	mVolumeData;
						BpFilter				mFilters;

		PX_FORCE_INLINE	void					initEntry(BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userData)
												{
													if ((index + 1) >= mVolumeData.size())
														reserveShapeSpace(index + 1);

													// PT: TODO: why is this needed at all? Why aren't size() and capacity() enough?
													mUsedSize = PxMax(index + 1, mUsedSize);

													PX_ASSERT(group != Bp::FilterGroup::eINVALID);	// PT: we use group == Bp::FilterGroup::eINVALID to mark removed/invalid entries
													mGroups[index] = group;
													mContactDistance.begin()[index] = contactDistance;
													mVolumeData[index].setUserData(userData);
												}

		PX_FORCE_INLINE	void					initEntry(BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userData, ElementType::Enum volumeType)
												{
													initEntry(index, contactDistance, group, userData);
													mVolumeData[index].setVolumeType(volumeType);	// PT: must be done after setUserData
												}

		PX_FORCE_INLINE	void					resetEntry(BoundsIndex index)
												{
													mGroups[index] = Bp::FilterGroup::eINVALID;
													mContactDistance.begin()[index] = 0.0f;
													mVolumeData[index].reset();

													if(index<mEnvIDs.size())
														mEnvIDs[index] = PX_INVALID_U32;
												}

		// PT: TODO: remove confusion between BoundsIndex and ShapeHandle here!
						ShapeHandleArrayPinned	mAddedHandles;
						ShapeHandleArrayPinned	mUpdatedHandles;	// PT: TODO: only on CPU
						ShapeHandleArrayPinned	mRemovedHandles;

						BroadPhase&				mBroadPhase;
						BoundsArray&			mBoundsArray;

						PxArray<AABBOverlap>	mCreatedOverlaps[ElementType::eCOUNT];
						PxArray<AABBOverlap>	mDestroyedOverlaps[ElementType::eCOUNT];

						PxU32					mUsedSize;				// highest used value + 1
						PxU32					mNbAggregates;

#if PX_ENABLE_SIM_STATS
						PxU32					mGpuDynamicsLostFoundPairsStats;
						PxU32					mGpuDynamicsTotalAggregatePairsStats;
						PxU32					mGpuDynamicsLostFoundAggregatePairsStats;
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

#if BP_USE_AGGREGATE_GROUP_TAIL
		// PT: TODO: even in the 3.4 trunk this stuff is a clumsy mess: groups are "BpHandle" suddenly passed
		// to BroadPhaseUpdateData as "ShapeHandle".
		//Free aggregate group ids.
						PxU32					mAggregateGroupTide;
				PxArray<Bp::FilterGroup::Enum>	mFreeAggregateGroups;	// PT: TODO: remove this useless array
#endif
						PxU64					mContextID;
						bool					mOriginShifted;

#if BP_USE_AGGREGATE_GROUP_TAIL
		PX_FORCE_INLINE void					releaseAggregateGroup(const Bp::FilterGroup::Enum group)
												{
													PX_ASSERT(group != Bp::FilterGroup::eINVALID);
													mFreeAggregateGroups.pushBack(group);
												}

		PX_FORCE_INLINE Bp::FilterGroup::Enum	getAggregateGroup()
												{
													PxU32 id;
													if (mFreeAggregateGroups.size())
														id = mFreeAggregateGroups.popBack();
													else
													{
														id = mAggregateGroupTide--;
														id <<= BP_FILTERING_TYPE_SHIFT_BIT;
														id |= FilterType::AGGREGATE;
													}
													const Bp::FilterGroup::Enum group = Bp::FilterGroup::Enum(id);
													PX_ASSERT(group != Bp::FilterGroup::eINVALID);
													return group;
												}
#endif
	};

} //namespace Bp
} //namespace physx

#endif //BP_AABBMANAGER_BASE_H
