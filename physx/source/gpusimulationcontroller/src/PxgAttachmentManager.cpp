// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgAttachmentManager.h"

#include "cutil_math.h"

#include "GuTriangleMesh.h"
#include "PxsRigidBody.h"

namespace physx
{
	// The GPU-side kind of an element, and the table that maps it into the numbering the GPU mesh uses.
	// Only a triangle is renumbered. A vertex and a tetrahedron use the same numbering on both sides.
	static void resolveFeature(const PxDeformableAttachmentTargetType::Enum feature, const Gu::TriangleMesh* surfaceMesh,
		PxgDeformableRef::Enum& featureType, const PxU32*& featureRemap)
	{
		featureRemap = NULL;

		switch (feature)
		{
		case PxDeformableAttachmentTargetType::eTETRAHEDRON:
			featureType = PxgDeformableRef::eTETRAHEDRON;
			break;
		case PxDeformableAttachmentTargetType::eTRIANGLE:
			featureType = PxgDeformableRef::eTRIANGLE;
			featureRemap = surfaceMesh->mGRB_faceRemapInverse;
			break;
		default:
			PX_ASSERT(feature == PxDeformableAttachmentTargetType::eVERTEX);
			featureType = PxgDeformableRef::eVERTEX;
			break;
		}
	}

	void PxgAttachmentManager::removeAttachment(const PxU32 handle)
	{
		deactivateAttachment(handle);

		//Now remove this current handle...
		PxHashMap<PxU32, PxU32>::Entry mapping;
		bool found = mHandleToAttachmentIndex.erase(handle, mapping);
		if (found)
		{
			mAttachments.replaceWithLast(mapping.second);
			mHandles.replaceWithLast(mapping.second);
			if (mapping.second < mAttachments.size())
			{
				PxU32 newHandle = mHandles[mapping.second];
				mHandleToAttachmentIndex[newHandle] = mapping.second;
				const PxHashMap<PxU32, PxU32>::Entry* activeMapping = mHandleToActiveIndex.find(newHandle);
				if (activeMapping)
				{
					mActiveAttachments[activeMapping->second] = mapping.second;
				}
			}
			mAttachmentsDirty = true;
		}
	}

	void PxgAttachmentManager::activateAttachment(const PxU32 handle)
	{
		// A batch between two deformables is added active if either actor is awake, but only one of the two
		// references it for its sleep and wake loops. That actor going from inactive to active activates
		// every batch it references, including one already active from the other actor being awake.
		if (mHandleToActiveIndex.find(handle))
			return;

		PxU32 index = mHandleToAttachmentIndex[handle];
		mHandleToActiveIndex[handle] = mActiveAttachments.size();
		mActiveAttachments.pushBack(index);
		mActiveAttachmentsDirty = true;
	}

	void PxgAttachmentManager::deactivateAttachment(const PxU32 handle)
	{
		PxHashMap<PxU32, PxU32>::Entry mapping;
		bool found = mHandleToActiveIndex.erase(handle, mapping);
		if (found)
		{
			mActiveAttachments.replaceWithLast(mapping.second);

			if (mapping.second < mActiveAttachments.size())
			{
				PxU32 replaceHandle = mHandles[mActiveAttachments[mapping.second]];
				mHandleToActiveIndex[replaceHandle] = mapping.second;
			}

			mActiveAttachmentsDirty = true;
		}
	}

	// Appends count attachments and returns the handle naming the batch. dst receives where to write them,
	// so a batch is filled in place and needs no staging of its own.
	PxU32 PxgAttachmentManager::allocateAttachments(const PxU32 count, PxgDeformableAttachment*& dst)
	{
		// An empty batch would leave mBaseHandle where it is, giving the next batch the same handle.
		PX_ASSERT(count > 0);

		const PxU32 index = mAttachments.size();

		const PxU32 batchHandle = mBaseHandle;
		mBaseHandle += count;

		mAttachments.reserve(index + count);
		mAttachments.forceSize_Unsafe(index + count);

		mHandles.reserve(index + count);
		for (PxU32 i = 0; i < count; ++i)
		{
			mHandles.pushBack(batchHandle + i);
			mHandleToAttachmentIndex[batchHandle + i] = index + i;
		}

		mBatchAttachmentCounts[batchHandle] = count;

		mAttachmentsDirty = true;

		dst = &mAttachments[index];
		return batchHandle;
	}

	PxU32 PxgAttachmentManager::addVolumeRigidAttachments(const PxU32 volumeId, const PxDeformableAttachmentTargetType::Enum feature,
		const PxU32* featureIds, const PxVec4* barycentrics, PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex,
		const PxTransform& frame, const PxVec4* coords, const PxU32 count, const bool addToActive)
	{
		PX_ASSERT(feature == PxDeformableAttachmentTargetType::eVERTEX || feature == PxDeformableAttachmentTargetType::eTETRAHEDRON);

		PxgDeformableRef::Enum featureType;
		const PxU32* featureRemap;
		resolveFeature(feature, NULL, featureType, featureRemap);

		return addRigidAttachmentsInternal(volumeId, featureType, featureIds, featureRemap, barycentrics, rigidBody,
			rigidNodeIndex, frame, coords, count, addToActive);
	}

	PxU32 PxgAttachmentManager::addSurfaceRigidAttachments(const PxU32 surfaceId, const Gu::TriangleMesh& surfaceMesh,
		const PxDeformableAttachmentTargetType::Enum feature, const PxU32* featureIds, const PxVec4* barycentrics,
		PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, const PxTransform& frame, const PxVec4* coords,
		const PxU32 count, const bool addToActive)
	{
		PX_ASSERT(feature == PxDeformableAttachmentTargetType::eVERTEX || feature == PxDeformableAttachmentTargetType::eTRIANGLE);

		PxgDeformableRef::Enum featureType;
		const PxU32* featureRemap;
		resolveFeature(feature, &surfaceMesh, featureType, featureRemap);

		return addRigidAttachmentsInternal(surfaceId, featureType, featureIds, featureRemap, barycentrics, rigidBody,
			rigidNodeIndex, frame, coords, count, addToActive);
	}

	PxU32 PxgAttachmentManager::addRigidAttachmentsInternal(const PxU32 deformableId, const PxgDeformableRef::Enum featureType,
		const PxU32* featureIds, const PxU32* featureRemap, const PxVec4* barycentrics, PxsRigidBody* rigidBody,
		const PxNodeIndex& rigidNodeIndex, const PxTransform& frame, const PxVec4* coords, const PxU32 count,
		const bool addToActive)
	{
		PX_ASSERT((featureType == PxgDeformableRef::eVERTEX) == (barycentrics == NULL));

		PxgDeformableAttachment* batch = NULL;
		const PxU32 batchHandle = allocateAttachments(count, batch);

		const PxU64 rigidRef = PxgEncodeRigidNode(rigidNodeIndex);
		// A rigid holds the attachment in its body frame, and a static has none to follow.
		const PxTransform toRigid = rigidBody ? rigidBody->getCore().getBody2Actor().getInverse() * frame : frame;

		for (PxU32 i = 0; i < count; ++i)
		{
			PxgDeformableAttachment& attachment = batch[i];

			const PxVec3 localPose = toRigid.transform(coords[i].getXYZ());
			attachment.barycentricOrPosition0 = make_float4(localPose.x, localPose.y, localPose.z, 0.0f);
			attachment.pair.ref0 = rigidRef;
			const PxU32 featureId = featureRemap ? featureRemap[featureIds[i]] : featureIds[i];
			attachment.pair.ref1 = PxgEncodeDeformableRef(deformableId, featureType, featureId);
			// Zeroed rather than left alone for a vertex: this is staged on the host and copied to the device.
			attachment.barycentric1 = barycentrics ? make_float4(barycentrics[i].x, barycentrics[i].y, barycentrics[i].z, barycentrics[i].w)
			                                       : make_float4(0.0f, 0.0f, 0.0f, 0.0f);
		}

		if (addToActive)
			activateAttachments(batchHandle);

		return batchHandle;
	}

	PxU32 PxgAttachmentManager::addVolumeVolumeAttachments(const PxU32 volumeId0,
		const PxDeformableAttachmentTargetType::Enum feature0, const PxU32* featureIds0, const PxVec4* barycentrics0,
		const PxU32 volumeId1, const PxDeformableAttachmentTargetType::Enum feature1, const PxU32* featureIds1,
		const PxVec4* barycentrics1, const PxU32 count, const bool addToActive)
	{
		PX_ASSERT(feature0 == PxDeformableAttachmentTargetType::eVERTEX || feature0 == PxDeformableAttachmentTargetType::eTETRAHEDRON);
		PX_ASSERT(feature1 == PxDeformableAttachmentTargetType::eVERTEX || feature1 == PxDeformableAttachmentTargetType::eTETRAHEDRON);

		PxgDeformableRef::Enum featureType0, featureType1;
		const PxU32 *featureRemap0, *featureRemap1;
		resolveFeature(feature0, NULL, featureType0, featureRemap0);
		resolveFeature(feature1, NULL, featureType1, featureRemap1);

		return addDeformableAttachmentsInternal(volumeId0, featureType0, featureIds0, featureRemap0, barycentrics0,
			volumeId1, featureType1, featureIds1, featureRemap1, barycentrics1, count, addToActive);
	}

	PxU32 PxgAttachmentManager::addVolumeSurfaceAttachments(const PxU32 volumeId,
		const PxDeformableAttachmentTargetType::Enum feature0, const PxU32* featureIds0, const PxVec4* barycentrics0,
		const PxU32 surfaceId, const Gu::TriangleMesh& surfaceMesh, const PxDeformableAttachmentTargetType::Enum feature1,
		const PxU32* featureIds1, const PxVec4* barycentrics1, const PxU32 count, const bool addToActive)
	{
		PX_ASSERT(feature0 == PxDeformableAttachmentTargetType::eVERTEX || feature0 == PxDeformableAttachmentTargetType::eTETRAHEDRON);
		PX_ASSERT(feature1 == PxDeformableAttachmentTargetType::eVERTEX || feature1 == PxDeformableAttachmentTargetType::eTRIANGLE);

		PxgDeformableRef::Enum featureType0, featureType1;
		const PxU32 *featureRemap0, *featureRemap1;
		resolveFeature(feature0, NULL, featureType0, featureRemap0);
		resolveFeature(feature1, &surfaceMesh, featureType1, featureRemap1);

		return addDeformableAttachmentsInternal(volumeId, featureType0, featureIds0, featureRemap0, barycentrics0,
			surfaceId, featureType1, featureIds1, featureRemap1, barycentrics1, count, addToActive);
	}

	PxU32 PxgAttachmentManager::addSurfaceSurfaceAttachments(const PxU32 surfaceId0, const Gu::TriangleMesh& surfaceMesh0,
		const PxDeformableAttachmentTargetType::Enum feature0, const PxU32* featureIds0, const PxVec4* barycentrics0,
		const PxU32 surfaceId1, const Gu::TriangleMesh& surfaceMesh1,
		const PxDeformableAttachmentTargetType::Enum feature1, const PxU32* featureIds1, const PxVec4* barycentrics1,
		const PxU32 count, const bool addToActive)
	{
		PX_ASSERT(feature0 == PxDeformableAttachmentTargetType::eVERTEX || feature0 == PxDeformableAttachmentTargetType::eTRIANGLE);
		PX_ASSERT(feature1 == PxDeformableAttachmentTargetType::eVERTEX || feature1 == PxDeformableAttachmentTargetType::eTRIANGLE);

		PxgDeformableRef::Enum featureType0, featureType1;
		const PxU32 *featureRemap0, *featureRemap1;
		resolveFeature(feature0, &surfaceMesh0, featureType0, featureRemap0);
		resolveFeature(feature1, &surfaceMesh1, featureType1, featureRemap1);

		return addDeformableAttachmentsInternal(surfaceId0, featureType0, featureIds0, featureRemap0, barycentrics0,
			surfaceId1, featureType1, featureIds1, featureRemap1, barycentrics1, count, addToActive);
	}

	PxU32 PxgAttachmentManager::addDeformableAttachmentsInternal(const PxU32 deformableId0,
		const PxgDeformableRef::Enum featureType0, const PxU32* featureIds0, const PxU32* featureRemap0,
		const PxVec4* barycentrics0, const PxU32 deformableId1, const PxgDeformableRef::Enum featureType1,
		const PxU32* featureIds1, const PxU32* featureRemap1, const PxVec4* barycentrics1, const PxU32 count,
		const bool addToActive)
	{
		PX_ASSERT((featureType0 == PxgDeformableRef::eVERTEX) == (barycentrics0 == NULL));
		PX_ASSERT((featureType1 == PxgDeformableRef::eVERTEX) == (barycentrics1 == NULL));

		PxgDeformableAttachment* batch = NULL;
		const PxU32 batchHandle = allocateAttachments(count, batch);

		for (PxU32 i = 0; i < count; ++i)
		{
			PxgDeformableAttachment& attachment = batch[i];

			const PxU32 featureId0 = featureRemap0 ? featureRemap0[featureIds0[i]] : featureIds0[i];
			const PxU32 featureId1 = featureRemap1 ? featureRemap1[featureIds1[i]] : featureIds1[i];

			attachment.pair.ref0 = PxgEncodeDeformableRef(deformableId0, featureType0, featureId0);
			attachment.pair.ref1 = PxgEncodeDeformableRef(deformableId1, featureType1, featureId1);

			// The fourth weight belongs to a tetrahedron alone. Zeroed rather than left alone for a vertex:
			// this is staged on the host and copied to the device.
			attachment.barycentricOrPosition0 = barycentrics0 ? make_float4(barycentrics0[i].x, barycentrics0[i].y, barycentrics0[i].z,
				featureType0 == PxgDeformableRef::eTETRAHEDRON ? barycentrics0[i].w : 0.0f) : make_float4(0.0f, 0.0f, 0.0f, 0.0f);
			attachment.barycentric1 = barycentrics1 ? make_float4(barycentrics1[i].x, barycentrics1[i].y, barycentrics1[i].z,
				featureType1 == PxgDeformableRef::eTETRAHEDRON ? barycentrics1[i].w : 0.0f) : make_float4(0.0f, 0.0f, 0.0f, 0.0f);
		}

		if (addToActive)
			activateAttachments(batchHandle);

		return batchHandle;
	}

	void PxgAttachmentManager::removeAttachments(const PxU32 batchHandle)
	{
		const PxU32 count = getBatchAttachmentCount(batchHandle);
		mBatchAttachmentCounts.erase(batchHandle);

		for (PxU32 i = 0; i < count; ++i)
			removeAttachment(batchHandle + i);
	}

	void PxgAttachmentManager::activateAttachments(const PxU32 batchHandle)
	{
		const PxU32 count = getBatchAttachmentCount(batchHandle);
		for (PxU32 i = 0; i < count; ++i)
			activateAttachment(batchHandle + i);
	}

	void PxgAttachmentManager::deactivateAttachments(const PxU32 batchHandle)
	{
		const PxU32 count = getBatchAttachmentCount(batchHandle);
		for (PxU32 i = 0; i < count; ++i)
			deactivateAttachment(batchHandle + i);
	}

	void PxgAttachmentManager::buildRigidRefCounts()
	{
		mActiveRigidRefCountsDirty = mAttachmentsDirty || mActiveAttachmentsDirty;
		if (!mActiveRigidRefCountsDirty)
			return;

		const PxU32 nbActive = mActiveAttachments.size();

		PxHashMap<PxU64, PxU32> counts;
		for (PxU32 i = 0; i < nbActive; ++i)
			++counts[mAttachments[mActiveAttachments[i]].pair.ref0];

		mActiveRigidRefCounts.resize(nbActive);
		for (PxU32 i = 0; i < nbActive; ++i)
			mActiveRigidRefCounts[i] = counts[mAttachments[mActiveAttachments[i]].pair.ref0];
	}

}
