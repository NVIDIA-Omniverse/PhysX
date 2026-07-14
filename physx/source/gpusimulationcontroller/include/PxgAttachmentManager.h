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

#ifndef PXG_ATTACHMENT_MANAGER_H
#define PXG_ATTACHMENT_MANAGER_H

#include "foundation/PxArray.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxSimpleTypes.h"
#include "CmPinnableArray.h"
#include "PxgSimulationCoreDesc.h"

namespace physx
{
	// Per-attachment refcount tracking, dispatched on the attachment type. Default specialization
	// is empty (zero overhead). Specialize for attachment types that need refcount-derived fields
	// (e.g. mass-splitting on PxgFEMRigidAttachment). flushTo returns true if it wrote anything,
	// so AttachmentManager can propagate the content-dirty signal.
	template <typename Attachment>
	struct AttachmentRefCount
	{
		void onActivate(const Attachment&)   {}
		void onDeactivate(const Attachment&) {}
		bool flushTo(Cm::PinnableArray<Attachment>&, const Cm::PinnableArray<PxU32>&) { return false; }
	};

	// Specialization for rigid-X attachments: per-rigid count of active attachments, used by the
	// solver for mass-splitting on the rigid response. Map is updated incrementally on activate /
	// deactivate; mDirty flips when the map content changes. flushTo is a no-op unless mDirty is
	// set, so callers can invoke it unconditionally at the DMA-up site without a separate gate.
	template <>
	struct AttachmentRefCount<PxgFEMRigidAttachment>
	{
		PxHashMap<PxU64, PxU32> mCounts;
		bool mDirty;

		AttachmentRefCount() : mDirty(false) {}

		void onActivate(const PxgFEMRigidAttachment& a)
		{
			++mCounts[a.index0];
			mDirty = true;
		}
		void onDeactivate(const PxgFEMRigidAttachment& a)
		{
			const PxHashMap<PxU64, PxU32>::Entry* entry = mCounts.find(a.index0);
			if (entry && entry->second > 0)
			{
				PxU32& c = mCounts[a.index0];
				if (--c == 0)
					mCounts.erase(a.index0);
			}
			mDirty = true;
		}
		bool flushTo(Cm::PinnableArray<PxgFEMRigidAttachment>& attachments, const Cm::PinnableArray<PxU32>& active)
		{
			if (!mDirty)
				return false;
			const PxU32 nbActive = active.size();
			for (PxU32 i = 0; i < nbActive; ++i)
			{
				PxgFEMRigidAttachment& a = attachments[active[i]];
				a.rigidBodyReferenceCount = mCounts[a.index0];
			}
			mDirty = false;
			return true;
		}
	};

	template <typename Attachment>
	class AttachmentManager
	{
	public:
		Cm::PinnableArray<Attachment>	mAttachments;
		Cm::PinnableArray<PxU32>		mActiveAttachments;
		PxHashMap<PxU32, PxU32>			mHandleToAttachmentMapping;
		PxHashMap<PxU32, PxU32>			mHandleToActiveIndex;
		PxArray<PxU32>					mHandles;
		AttachmentRefCount<Attachment>	mRefCount;

		PxU32 mBaseHandle;
		bool mAttachmentsDirty;
		bool mActiveAttachmentsDirty;

		AttachmentManager(Cm::VirtualAllocatorCallback& hostAlloc, PxU32 statType) :
			mAttachments(hostAlloc, statType),
			mActiveAttachments(hostAlloc, statType),
			mBaseHandle(0),
			mAttachmentsDirty(false),
			mActiveAttachmentsDirty(false)
		{
		}

		void addAttachment(const Attachment& attachment, const PxU32 handle)
		{
			const PxU32 size = mAttachments.size();
			mAttachments.pushBack(attachment);
			mHandles.pushBack(handle);
			mHandleToAttachmentMapping[handle] = size;
			mAttachmentsDirty = true;
		}

		bool removeAttachment(const PxU32 handle)
		{
			deactivateAttachment(handle);

			//Now remove this current handle...
			PxHashMap<PxU32, PxU32>::Entry mapping;
			bool found = mHandleToAttachmentMapping.erase(handle, mapping);
			if (found)
			{
				mAttachments.replaceWithLast(mapping.second);
				mHandles.replaceWithLast(mapping.second);
				if (mapping.second < mAttachments.size())
				{
					PxU32 newHandle = mHandles[mapping.second];
					mHandleToAttachmentMapping[newHandle] = mapping.second;
					const PxHashMap<PxU32, PxU32>::Entry* activeMapping = mHandleToActiveIndex.find(newHandle);
					if (activeMapping)
					{
						mActiveAttachments[activeMapping->second] = mapping.second;
					}
				}
				mAttachmentsDirty = true;
			}
			return found;
		}

		void activateAttachment(const PxU32 handle)
		{
			PX_ASSERT(!mHandleToActiveIndex.find(handle));
			PxU32 index = mHandleToAttachmentMapping[handle];
			mHandleToActiveIndex[handle] = mActiveAttachments.size();
			mActiveAttachments.pushBack(index);
			mRefCount.onActivate(mAttachments[index]);
			mActiveAttachmentsDirty = true;
		}

		void deactivateAttachment(const PxU32 handle)
		{
			PxHashMap<PxU32, PxU32>::Entry mapping;
			bool found = mHandleToActiveIndex.erase(handle, mapping);
			if (found)
			{
				const PxU32 attachIdx = mActiveAttachments[mapping.second];
				mRefCount.onDeactivate(mAttachments[attachIdx]);

				mActiveAttachments.replaceWithLast(mapping.second);

				if (mapping.second < mActiveAttachments.size())
				{
					PxU32 replaceHandle = mHandles[mActiveAttachments[mapping.second]];
					mHandleToActiveIndex[replaceHandle] = mapping.second;
				}

				mActiveAttachmentsDirty = true;
			}
		}

		// Write the policy's derived per-attachment fields into the staging array. Called
		// unconditionally once per frame at the DMA-up site; the policy itself decides whether
		// anything needs to happen (default policy is always a no-op; the rigid-attach
		// specialization tracks its own dirty bit on mCounts mutations). Marks the master array
		// dirty if the policy actually wrote anything, so the existing dirtyRigidAttachments
		// upload path picks up the content change.
		void flushRefCountsToAttachments()
		{
			if (mRefCount.flushTo(mAttachments, mActiveAttachments))
				mAttachmentsDirty = true;
		}
	};

} // namespace physx

#endif // PXG_ATTACHMENT_MANAGER_H
