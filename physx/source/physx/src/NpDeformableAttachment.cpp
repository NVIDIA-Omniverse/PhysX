// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxAllocator.h"
#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX
#include "NpCheck.h"
#include "NpScene.h"
#include "NpShape.h"
#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpArticulationLink.h"
#include "NpDeformableAttachment.h"
#include "NpDeformableSurface.h"
#include "NpDeformableVolume.h"

using namespace physx;

NpInternalAttachmentType::Enum getInternalAttachmentType(const PxDeformableAttachmentData& data, PxU32 actorIndex[2])
{
	// Make sure both actors are not NULL
	if (data.actor[0] == NULL && data.actor[1] == NULL)
		return NpInternalAttachmentType::eUNDEFINED;

	// Actor 0 is always assumed to be a deformable (deformable volume has the highest priority)
	// If Actor 0 is a null actor or rigid actor, the actors are swapped using the Actor Index
	// If Actor 0 is a deformable surface and Actor 1 is a deformable volume, the actors are swapped using the Actor Index
	actorIndex[0] = 0;
	actorIndex[1] = 1;

	if ((data.actor[0] == NULL || data.actor[0]->is<PxRigidActor>()) ||
		(data.actor[0]->is<PxDeformableSurface>() && data.actor[1] != NULL && data.actor[1]->is<PxDeformableVolume>()))
	{
		// Swap the actor index.
		PxSwap(actorIndex[0], actorIndex[1]);
	}

	// Further order according to element type, TET > TRI > VTX
	PxType actorType0 = data.actor[actorIndex[0]]->getConcreteType();
	PxType actorType1 = data.actor[actorIndex[1]] ? data.actor[actorIndex[1]]->getConcreteType() : PxConcreteType::eUNDEFINED;
	if (actorType0 == actorType1 && data.type[actorIndex[0]] < data.type[actorIndex[1]])
	{
		// Swap again to order according to first type: PxDeformableAttachmentTargetType::eTETRAHEDRON first, then eTRIANGLE, last eVERTEX.
		PxSwap(actorIndex[0], actorIndex[1]);
	}

	switch (data.actor[actorIndex[0]]->getConcreteType())
	{
		case PxConcreteType::eDEFORMABLE_SURFACE:
		{
			if (data.actor[actorIndex[1]] == NULL)
			{
				if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eWORLD)
				{
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE;
					
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eTRIANGLE)
						return NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE;
				}
			}
			else if (data.actor[actorIndex[1]]->is<PxRigidActor>())
			{
				if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eRIGID)
				{
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eSURFACE_VTX_RIGID_ACTOR;
					
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eTRIANGLE)
						return NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR;
				}
			}
			else if (data.actor[actorIndex[1]]->is<PxDeformableSurface>())
			{
				if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eTRIANGLE)
				{
					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eTRIANGLE)
						return NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI;

					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eSURFACE_TRI_SURFACE_VTX;
				}
				else if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eVERTEX)
				{
					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eSURFACE_VTX_SURFACE_VTX;
				}
			}

			break;
		}
		case PxConcreteType::eDEFORMABLE_VOLUME:
		{
			if (data.actor[actorIndex[1]] == NULL)
			{
				if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eWORLD)
				{
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eVOLUME_VTX_GLOBAL_POSE;
					
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eTETRAHEDRON)
						return NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE;
				}
			}
			else if (data.actor[actorIndex[1]]->is<PxRigidActor>())
			{
				if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eRIGID)
				{
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eVOLUME_VTX_RIGID_ACTOR;
					
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eTETRAHEDRON)
						return NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR;
				}
			}
			else if (data.actor[actorIndex[1]]->is<PxDeformableSurface>())
			{
				if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eTETRAHEDRON)
				{
					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eVOLUME_TET_SURFACE_VTX;

					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eTRIANGLE)
						return NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI;
				}
				else if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eVERTEX)
				{
					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eVOLUME_VTX_SURFACE_VTX;

					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eTRIANGLE)
						return NpInternalAttachmentType::eVOLUME_VTX_SURFACE_TRI;
				}
			}
			else if (data.actor[actorIndex[1]]->is<PxDeformableVolume>())
			{
				if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eTETRAHEDRON)
				{
					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eTETRAHEDRON)
						return NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET;

					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eVOLUME_TET_VOLUME_VTX;
				}
				else if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eVERTEX)
				{
					if (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eVERTEX)
						return NpInternalAttachmentType::eVOLUME_VTX_VOLUME_VTX;
				}
			}

			break;
		}
	}

	return NpInternalAttachmentType::eUNDEFINED;
}

bool NpDeformableAttachment::parseAttachment(const PxDeformableAttachmentData& data, AttachmentInfo& info)
{
	NpInternalAttachmentType::Enum& internalAttachmentType = info.internalAttachmentType;
	PxU32* actorIndex = info.actorIndex;

	internalAttachmentType = getInternalAttachmentType(data, actorIndex);

	switch (internalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR:
		case NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE:
		case NpInternalAttachmentType::eSURFACE_VTX_RIGID_ACTOR:
		case NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_VTX_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_VTX_GLOBAL_POSE:
		{
			{
				PX_CHECK_AND_RETURN_VAL
				(
					data.indices[actorIndex[0]].count == data.coords[actorIndex[1]].count,
					"PxDeformableAttachment: Number of attachment points are not equal for both actors.",
					NULL
				);
			}

			// Table 2 of the PxDeformableAttachmentData documentation: an eRIGID or eWORLD target is
			// specified by a pose and cartesian coordinates. Indices given for it are dropped when the
			// attachment is built, so part of what the caller supplied would do nothing.
			PX_CHECK_AND_RETURN_VAL
			(
				data.indices[actorIndex[1]].count == 0,
				"PxDeformableAttachment: No indices supported for an eRIGID or eWORLD target, which is specified by a pose and coordinates.",
				NULL
			);

			if (internalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR ||
				internalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE ||
				internalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR ||
				internalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE)
			{
				PX_CHECK_AND_RETURN_VAL
				(
					data.indices[actorIndex[0]].count == data.coords[actorIndex[0]].count,
					"PxDeformableAttachment: Number of attachment points are not equal for both actors.",
					NULL
				);
			}

			break;
		}

		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_VTX:
		case NpInternalAttachmentType::eSURFACE_VTX_SURFACE_VTX:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_VTX:
		case NpInternalAttachmentType::eVOLUME_VTX_VOLUME_VTX:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_VTX:
		case NpInternalAttachmentType::eVOLUME_VTX_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_VTX_SURFACE_VTX:
		{
			PX_CHECK_AND_RETURN_VAL
			(
				data.indices[actorIndex[0]].count == data.indices[actorIndex[1]].count,
				"PxDeformableAttachment: Number of attachment points are not equal for both actors.",
				NULL
			);

			// Table 1 of the PxDeformableAttachmentData documentation: an element target is specified by an
			// index and barycentric coordinates, a vertex target by an index on its own.
			PX_CHECK_AND_RETURN_VAL
			(
				data.coords[actorIndex[0]].count == (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eVERTEX ? 0 : data.indices[actorIndex[0]].count) &&
				data.coords[actorIndex[1]].count == (data.type[actorIndex[1]] == PxDeformableAttachmentTargetType::eVERTEX ? 0 : data.indices[actorIndex[1]].count),
				"PxDeformableAttachment: A vertex target takes no coordinates, an element target one per index.",
				NULL
			);

			break;
		}

		default:
		{
			PX_CHECK_AND_RETURN_VAL(false, "PxDeformableAttachment: No matching actor pairs found for attachment.", NULL);
		}
	}

	return true;
}

#if PX_CHECKED

// Number of elements of the requested kind on a deformable actor. A surface attaches to the triangle
// mesh it simulates and a volume to its simulation mesh.
//
// A count of zero refuses every index, which is the answer for a target type the actor has no
// elements of. PxScene::addActor rejects a deformable without a shape, and a volume without a
// simulation mesh, so a missing mesh is a state an attachment being enabled cannot be in, rather than
// one to accept unchecked.
static PxU32 getElementCount(PxDeformableBody& body, PxDeformableAttachmentTargetType::Enum type)
{
	if (body.getConcreteType() == PxConcreteType::eDEFORMABLE_VOLUME)
	{
		// An attachment addresses the simulation mesh. An element filter addresses the collision mesh.
		const PxTetrahedronMesh* mesh = static_cast<PxDeformableVolume&>(body).getSimulationMesh();
		if (mesh == NULL)
			return 0;

		if (type == PxDeformableAttachmentTargetType::eVERTEX)
			return mesh->getNbVertices();

		if (type == PxDeformableAttachmentTargetType::eTETRAHEDRON)
			return mesh->getNbTetrahedrons();

		return 0;
	}

	// A surface simulates its shape's triangle mesh directly, so it has no separate simulation mesh.
	PxShape* shape = body.getShape();
	if (shape == NULL || shape->getGeometry().getType() != PxGeometryType::eTRIANGLEMESH)
		return 0;

	const PxTriangleMesh* mesh = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry()).triangleMesh;
	if (mesh == NULL)
		return 0;

	if (type == PxDeformableAttachmentTargetType::eVERTEX)
		return mesh->getNbVertices();

	if (type == PxDeformableAttachmentTargetType::eTRIANGLE)
		return mesh->getNbTriangles();

	return 0;
}

// An index past the end of the mesh reaches the solver as an out-of-bounds element lookup. Checked
// here rather than at creation so that it does not depend on whether the mesh happened to be attached
// yet, and so that swapping in a smaller mesh is caught when the attachment is enabled again.
bool NpDeformableAttachment::checkIndicesInRange() const
{
	for (PxU32 i = 0; i < 2; ++i)
	{
		PxDeformableBody* body = mActor[i] ? mActor[i]->is<PxDeformableBody>() : NULL;
		if (body == NULL)
			continue;

		const PxU32 count = getElementCount(*body, mType[i]);

		const PxArray<PxU32>& indices = mIndices[i];
		for (PxU32 j = 0; j < indices.size(); ++j)
		{
			if (indices[j] >= count)
				return false;
		}
	}

	return true;
}

#endif // PX_CHECKED

// A vertex names no element whose corners could be weighted, so it carries no coordinates. The
// attachment manager asserts the pointer against the feature type it is given.
static const PxVec4* getBarycentrics(const PxArray<PxVec4>& coords, PxDeformableAttachmentTargetType::Enum feature)
{
	return feature == PxDeformableAttachmentTargetType::eVERTEX ? NULL : coords.begin();
}

PxTransform NpDeformableAttachment::getRigidFrame(const PxTransform& pose) const
{
	PxRigidActor* actor1 = mActor[mActorIndex[1]] == NULL ? NULL : mActor[mActorIndex[1]]->is<PxRigidActor>();

	// A static is not tracked by the solver, so its transform is folded into the frame here.
	if (actor1 && actor1->getConcreteType() == PxConcreteType::eRIGID_STATIC)
		return pose * static_cast<NpRigidStatic*>(actor1)->getGlobalPose();

	return pose;
}

void NpDeformableAttachment::addAttachment()
{
	if (!getSceneFromActors() ||  mEnabled)
		return;

	PX_CHECK_AND_RETURN(checkIndicesInRange(),
		"PxDeformableAttachment: attachment index is out of range for the deformable body's mesh. Attachment is ignored.");

	// An empty attachment is not added and its handle left invalid. It is enabled all the same, since that
	// tracks whether the attachment is in the scene rather than what it added.
	if (mIndices[mActorIndex[0]].empty())
	{
		mHandle = PX_INVALID_U32;
		mEnabled = true;
		return;
	}

	switch (mInternalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR:
		case NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE:
		case NpInternalAttachmentType::eSURFACE_VTX_RIGID_ACTOR:
		case NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_VTX_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_VTX_GLOBAL_POSE:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]] == NULL ? NULL : mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::BodyCore* core1 = mActor[mActorIndex[1]] == NULL ? NULL : getBodyCore(actor1);

			const PxU32 count = mIndices[mActorIndex[0]].size();

			const PxTransform rigidFrame = getRigidFrame(mPose[mActorIndex[1]]);

			if (mInternalAttachmentType & NpInternalAttachmentType::eVOLUME_TYPE)
			{
				const bool isTet = (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR ||
									mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE);

				mHandle = getDeformableVolumeCore(actor0)->addRigidAttachments(core1,
					isTet ? PxDeformableAttachmentTargetType::eTETRAHEDRON : PxDeformableAttachmentTargetType::eVERTEX,
					mIndices[mActorIndex[0]].begin(), isTet ? mCoords[mActorIndex[0]].begin() : NULL,
					rigidFrame, mCoords[mActorIndex[1]].begin(), count);
			}
			else
			{
				const bool isTri = (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR ||
									mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE);

				mHandle = getDeformableSurfaceCore(actor0)->addRigidAttachments(core1,
					isTri ? PxDeformableAttachmentTargetType::eTRIANGLE : PxDeformableAttachmentTargetType::eVERTEX,
					mIndices[mActorIndex[0]].begin(), isTri ? mCoords[mActorIndex[0]].begin() : NULL,
					rigidFrame, mCoords[mActorIndex[1]].begin(), count);
			}

			break;
		}

		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_VTX:
		case NpInternalAttachmentType::eSURFACE_VTX_SURFACE_VTX:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_VTX:
		case NpInternalAttachmentType::eVOLUME_VTX_VOLUME_VTX:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_VTX:
		case NpInternalAttachmentType::eVOLUME_VTX_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_VTX_SURFACE_VTX:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];
			PxActor* actor1 = mActor[mActorIndex[1]];

			const PxU32 count = mIndices[mActorIndex[0]].size();
			const PxDeformableAttachmentTargetType::Enum feature0 = mType[mActorIndex[0]];
			const PxDeformableAttachmentTargetType::Enum feature1 = mType[mActorIndex[1]];
			const PxU32* indices0 = mIndices[mActorIndex[0]].begin();
			const PxVec4* barycentrics0 = getBarycentrics(mCoords[mActorIndex[0]], feature0);
			const PxU32* indices1 = mIndices[mActorIndex[1]].begin();
			const PxVec4* barycentrics1 = getBarycentrics(mCoords[mActorIndex[1]], feature1);

			// The entry point follows the pair of actors. The feature types travel as data. Actor 0 is the
			// volume wherever there is one, so actor 1 alone separates the two volume pairs.
			if (mInternalAttachmentType & NpInternalAttachmentType::eSURFACE_TYPE)
			{
				mHandle = getDeformableSurfaceCore(actor0)->addSurfaceAttachments(getDeformableSurfaceCore(actor1),
					feature1, indices1, barycentrics1, feature0, indices0, barycentrics0, count);
			}
			else if (actor1->is<PxDeformableVolume>())
			{
				mHandle = getDeformableVolumeCore(actor0)->addVolumeAttachments(*getDeformableVolumeCore(actor1),
					feature1, indices1, barycentrics1, feature0, indices0, barycentrics0, count);
			}
			else
			{
				mHandle = getDeformableVolumeCore(actor0)->addSurfaceAttachments(*getDeformableSurfaceCore(actor1),
					feature1, indices1, barycentrics1, feature0, indices0, barycentrics0, count);
			}

			break;
		}

		default:
		{
			PX_ASSERT(0);
			break;
		}
	}

	mEnabled = true;
}

void NpDeformableAttachment::removeAttachment()
{
	if (!mEnabled)
		return;

	// An empty attachment added nothing, so there is nothing to take back.
	if (mHandle == PX_INVALID_U32)
	{
		mEnabled = false;
		return;
	}

	switch (mInternalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR:
		case NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE:
		case NpInternalAttachmentType::eSURFACE_VTX_RIGID_ACTOR:
		case NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_VTX_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_VTX_GLOBAL_POSE:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]] == NULL ? NULL : mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::BodyCore* core1 = mActor[mActorIndex[1]] == NULL ? NULL : getBodyCore(actor1);

			// Removal is keyed on the handle alone, so the triangle and the vertex attachment come off
			// the same way.
			if (mInternalAttachmentType & NpInternalAttachmentType::eSURFACE_TYPE)
			{
				getDeformableSurfaceCore(actor0)->removeRigidAttachments(core1, mHandle);
			}
			else if (mInternalAttachmentType & NpInternalAttachmentType::eVOLUME_TYPE)
			{
				getDeformableVolumeCore(actor0)->removeRigidAttachments(core1, mHandle);
			}

			break;
		}

		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_VTX:
		case NpInternalAttachmentType::eSURFACE_VTX_SURFACE_VTX:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_VTX:
		case NpInternalAttachmentType::eVOLUME_VTX_VOLUME_VTX:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_VTX:
		case NpInternalAttachmentType::eVOLUME_VTX_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_VTX_SURFACE_VTX:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];
			PxActor* actor1 = mActor[mActorIndex[1]];

			if (mInternalAttachmentType & NpInternalAttachmentType::eSURFACE_TYPE)
			{
				getDeformableSurfaceCore(actor0)->removeSurfaceAttachments(getDeformableSurfaceCore(actor1), mHandle);
			}
			else if (actor1->is<PxDeformableVolume>())
			{
				getDeformableVolumeCore(actor0)->removeVolumeAttachments(*getDeformableVolumeCore(actor1), mHandle);
			}
			else
			{
				getDeformableVolumeCore(actor0)->removeSurfaceAttachments(*getDeformableSurfaceCore(actor1), mHandle);
			}

			break;
		}

		default:
		{
			PX_ASSERT(0);
			break;
		}
	}

	mHandle = PX_INVALID_U32;
	mEnabled = false;
}

NpScene* NpDeformableAttachment::getSceneFromActors()
{
	const PxActor* actor[2] = { mActor[mActorIndex[0]], mActor[mActorIndex[1]] };

	for (PxU32 i = 0; i < 2; i++)
	{
		if (actor[i] && (actor[i]->getScene() == NULL))
			return NULL;
	}

	if (actor[0] && actor[1])
	{
		if (actor[0]->getScene() != actor[1]->getScene())
		{
			PX_CHECK_MSG(false, "PxDeformableAttachment: Actors belong to different scenes, undefined behavior expected!");

			return NULL;
		}
	}

	return static_cast<NpScene*>(actor[0]->getScene());
}

NpDeformableAttachment::NpDeformableAttachment(const PxDeformableAttachmentData& data, const AttachmentInfo& info)
	: PxDeformableAttachment(PxConcreteType::eDEFORMABLE_ATTACHMENT, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE), NpBase(NpType::eDEFORMABLE_ATTACHMENT)
{
	mInternalAttachmentType = info.internalAttachmentType;
	mEnabled = false;

	for (PxU32 i = 0; i < 2; i++)
	{
		mActor[i] = data.actor[i];
		mType[i]  = data.type[i];
		mPose[i]  = data.pose[i];

		mIndices[i].resize(data.indices[i].count);
		for (PxU32 j = 0; j < data.indices[i].count; j++)
		{
			mIndices[i][j] = data.indices[i].at(j);
		}

		mCoords[i].resize(data.coords[i].count);
		for (PxU32 j = 0; j < data.coords[i].count; j++)
		{
			mCoords[i][j] = data.coords[i].at(j);
		}

		mActorIndex[i] = info.actorIndex[i];

		// Add connector
		if (mActor[i])
			NpActor::getFromPxActor(*mActor[i]).addConnector(NpConnectorType::eAttachment, this, "PxDeformableAttachment: Attachment already added");
	}

	NpScene* s = getSceneFromActors();
	if (s)
	{
		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(s, "PxDeformableAttachment creation not allowed while simulation is running. Call will be ignored.");

		s->addToAttachmentList(*this);
	}

	setNpScene(s);
}

NpDeformableAttachment::~NpDeformableAttachment()
{
	NpFactory::getInstance().onAttachmentRelease(this);
}

void NpDeformableAttachment::release()
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableAttachment: Illegal to call release while simulation is running.");

	for (PxU32 i = 0; i < 2; i++)
	{
		// Remove connector
		if (mActor[i])
		{
			PxDeformableAttachment* buffer[1];

			if (NpActor::getFromPxActor(*mActor[i]).getConnectors(NpConnectorType::eAttachment, buffer, 1))
				NpActor::getFromPxActor(*mActor[i]).removeConnector(*mActor[i], NpConnectorType::eAttachment, this, "PxDeformableAttachment: Attachment already released");
		}
	}
	
	if (npScene)
	{
		npScene->removeFromAttachmentList(*this);
	}
	
	NpDestroyAttachment(this);
}

void NpDeformableAttachment::getActors(PxActor*& actor0, PxActor*& actor1) const
{
	NP_READ_CHECK(getNpScene());

	actor0 = mActor[0];
	actor1 = mActor[1];
}

void NpDeformableAttachment::updatePose(const PxTransform& pose)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableAttachment: Illegal to call updatePose while simulation is running.");

	switch (mInternalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR:
		case NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE:
		case NpInternalAttachmentType::eSURFACE_VTX_RIGID_ACTOR:
		case NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_VTX_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_VTX_GLOBAL_POSE:
		{
			// An empty attachment holds nothing to move, and the frame is kept all the same.
			if (mHandle == PX_INVALID_U32)
			{
				mPose[mActorIndex[1]] = pose;
				break;
			}

			PxActor* actor0 = mActor[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]] == NULL ? NULL : mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::BodyCore* core1 = mActor[mActorIndex[1]] == NULL ? NULL : getBodyCore(actor1);

			const PxU32 count = mIndices[mActorIndex[0]].size();

			const PxTransform rigidFrame = getRigidFrame(pose);

			// The pose moves what the attachment holds its coordinates in, so the batch is added again in
			// the new frame.
			if (mInternalAttachmentType & NpInternalAttachmentType::eVOLUME_TYPE)
			{
				Sc::DeformableVolumeCore* core0 = getDeformableVolumeCore(actor0);
				const bool isTet = (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR ||
									mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE);

				core0->removeRigidAttachments(core1, mHandle);

				mHandle = core0->addRigidAttachments(core1,
					isTet ? PxDeformableAttachmentTargetType::eTETRAHEDRON : PxDeformableAttachmentTargetType::eVERTEX,
					mIndices[mActorIndex[0]].begin(), isTet ? mCoords[mActorIndex[0]].begin() : NULL,
					rigidFrame, mCoords[mActorIndex[1]].begin(), count);
			}
			else
			{
				Sc::DeformableSurfaceCore* core0 = getDeformableSurfaceCore(actor0);
				const bool isTri = (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR ||
									mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE);

				core0->removeRigidAttachments(core1, mHandle);

				mHandle = core0->addRigidAttachments(core1,
					isTri ? PxDeformableAttachmentTargetType::eTRIANGLE : PxDeformableAttachmentTargetType::eVERTEX,
					mIndices[mActorIndex[0]].begin(), isTri ? mCoords[mActorIndex[0]].begin() : NULL,
					rigidFrame, mCoords[mActorIndex[1]].begin(), count);
			}

			mPose[mActorIndex[1]] = pose;

			break;
		}

		default:
		{
			PX_CHECK_AND_RETURN(false, "PxDeformableAttachment: Updating of pose is not supported for this attachment type.");
		}
	}
}

#endif
