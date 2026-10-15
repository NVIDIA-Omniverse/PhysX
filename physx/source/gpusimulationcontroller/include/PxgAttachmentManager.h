// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_ATTACHMENT_MANAGER_H
#define PXG_ATTACHMENT_MANAGER_H

#include "foundation/PxArray.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxSimpleTypes.h"
#include "CmPinnableArray.h"
#include "PxDeformableAttachment.h"
#include "PxgSimulationCoreDesc.h"

namespace physx
{
	namespace Gu
	{
		class TriangleMesh;
	}

	class PxsRigidBody;

	class PxgAttachmentManager
	{
	public:
		// One entry per attachment, kept in step by addAttachment and removeAttachment.
		Cm::PinnableArray<PxgDeformableAttachment> mAttachments;
		PxArray<PxU32> mHandles;
		PxHashMap<PxU32, PxU32> mHandleToAttachmentIndex;

		// One entry per active attachment, kept in step by activateAttachment and deactivateAttachment.
		// mActiveRigidRefCounts is the exception, rebuilt by buildRigidRefCounts.
		Cm::PinnableArray<PxU32> mActiveAttachments;
		PxHashMap<PxU32, PxU32> mHandleToActiveIndex;
		Cm::PinnableArray<PxU32> mActiveRigidRefCounts;

		PxU32 mBaseHandle;
		bool mAttachmentsDirty;
		bool mActiveAttachmentsDirty;
		bool mActiveRigidRefCountsDirty;

		PxgAttachmentManager(Cm::VirtualAllocatorCallback& hostAlloc, PxU32 statType) :
			mAttachments(hostAlloc, statType),
			mActiveAttachments(hostAlloc, statType),
			mActiveRigidRefCounts(hostAlloc, statType),
			mBaseHandle(0),
			mAttachmentsDirty(false),
			mActiveAttachmentsDirty(false),
			mActiveRigidRefCountsDirty(false)
		{
		}

		// Adds a batch of attachments between one deformable and one rigid, and returns a handle to refer to
		// the batch. The data is provided as given by the API and converted: the coordinates into the
		// rigid's own frame, and the elements into encoded pairs.

		PxU32 addVolumeRigidAttachments(const PxU32 volumeId, const PxDeformableAttachmentTargetType::Enum feature,
			const PxU32* featureIds, const PxVec4* barycentrics, PxsRigidBody* rigidBody,
			const PxNodeIndex& rigidNodeIndex, const PxTransform& frame, const PxVec4* coords, const PxU32 count,
			const bool addToActive);

		PxU32 addSurfaceRigidAttachments(const PxU32 surfaceId, const Gu::TriangleMesh& surfaceMesh,
			const PxDeformableAttachmentTargetType::Enum feature, const PxU32* featureIds, const PxVec4* barycentrics,
			PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, const PxTransform& frame, const PxVec4* coords,
			const PxU32 count, const bool addToActive);

		// Adds a batch of attachments between two deformables, and returns a handle to refer to the batch.
		// As above, the feature indices are given as the API numbers them.

		PxU32 addVolumeVolumeAttachments(const PxU32 volumeId0, const PxDeformableAttachmentTargetType::Enum feature0,
			const PxU32* featureIds0, const PxVec4* barycentrics0, const PxU32 volumeId1,
			const PxDeformableAttachmentTargetType::Enum feature1, const PxU32* featureIds1,
			const PxVec4* barycentrics1, const PxU32 count, const bool addToActive);

		PxU32 addVolumeSurfaceAttachments(const PxU32 volumeId, const PxDeformableAttachmentTargetType::Enum feature0,
			const PxU32* featureIds0, const PxVec4* barycentrics0, const PxU32 surfaceId,
			const Gu::TriangleMesh& surfaceMesh, const PxDeformableAttachmentTargetType::Enum feature1,
			const PxU32* featureIds1, const PxVec4* barycentrics1, const PxU32 count, const bool addToActive);

		PxU32 addSurfaceSurfaceAttachments(const PxU32 surfaceId0, const Gu::TriangleMesh& surfaceMesh0,
			const PxDeformableAttachmentTargetType::Enum feature0, const PxU32* featureIds0,
			const PxVec4* barycentrics0, const PxU32 surfaceId1, const Gu::TriangleMesh& surfaceMesh1,
			const PxDeformableAttachmentTargetType::Enum feature1, const PxU32* featureIds1,
			const PxVec4* barycentrics1, const PxU32 count, const bool addToActive);

		void removeAttachments(const PxU32 batchHandle);

		void activateAttachments(const PxU32 batchHandle);

		void deactivateAttachments(const PxU32 batchHandle);

		// Fills mActiveRigidRefCounts with, for each active attachment, the number of active
		// attachments sharing its rigid. The solver splits the rigid response by that count. Only
		// called where ref0 is a rigid.
		void buildRigidRefCounts();

	private:
		// One attachment at a time, which is what the batches above are made of.
		void removeAttachment(const PxU32 handle);
		void activateAttachment(const PxU32 handle);
		void deactivateAttachment(const PxU32 handle);

		PxU32 allocateAttachments(const PxU32 count, PxgDeformableAttachment*& dst);

		// What the two above share. featureRemap maps an element to the numbering the GPU mesh uses, and is
		// NULL where the two numberings agree.
		PxU32 addRigidAttachmentsInternal(const PxU32 deformableId, const PxgDeformableRef::Enum featureType,
			const PxU32* featureIds, const PxU32* featureRemap, const PxVec4* barycentrics, PxsRigidBody* rigidBody,
			const PxNodeIndex& rigidNodeIndex, const PxTransform& frame, const PxVec4* coords, const PxU32 count,
			const bool addToActive);

		// What the three above share. A remap maps an element to the numbering the GPU mesh uses, and is NULL
		// where the two numberings agree.
		PxU32 addDeformableAttachmentsInternal(const PxU32 deformableId0, const PxgDeformableRef::Enum featureType0,
			const PxU32* featureIds0, const PxU32* featureRemap0, const PxVec4* barycentrics0,
			const PxU32 deformableId1, const PxgDeformableRef::Enum featureType1, const PxU32* featureIds1,
			const PxU32* featureRemap1, const PxVec4* barycentrics1, const PxU32 count, const bool addToActive);

		// One entry per batch added at once: the handle of its first attachment maps to how many internal
		// consecutive handles the batch owns.
		PxHashMap<PxU32, PxU32> mBatchAttachmentCounts;

		PX_FORCE_INLINE PxU32 getBatchAttachmentCount(const PxU32 batchHandle) const
		{
			const PxHashMap<PxU32, PxU32>::Entry* entry = mBatchAttachmentCounts.find(batchHandle);
			PX_ASSERT(entry);
			return entry ? entry->second : 0;
		}
	};

} // namespace physx

#endif // PXG_ATTACHMENT_MANAGER_H
