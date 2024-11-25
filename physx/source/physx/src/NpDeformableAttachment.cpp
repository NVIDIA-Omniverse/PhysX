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

#include "PxsHeapMemoryAllocator.h"
#include "PxsMemoryManager.h"
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
						return NpInternalAttachmentType::eSURFACE_VTX_RIGID_BODY;
					
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eTRIANGLE)
						return NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY;
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
						return NpInternalAttachmentType::eVOLUME_VTX_RIGID_BODY;
					
					if (data.type[actorIndex[0]] == PxDeformableAttachmentTargetType::eTETRAHEDRON)
						return NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY;
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
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY:
		case NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE:
		case NpInternalAttachmentType::eSURFACE_VTX_RIGID_BODY:
		case NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_VTX_RIGID_BODY:
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
			
			if (internalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY ||
				internalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE ||
				internalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY ||
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
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		{
			PX_CHECK_AND_RETURN_VAL
			(
				data.indices[actorIndex[0]].count == data.coords[actorIndex[0]].count &&
				data.indices[actorIndex[1]].count == data.coords[actorIndex[1]].count &&
				data.indices[actorIndex[0]].count == data.indices[actorIndex[1]].count,
				"PxDeformableAttachment: Number of attachment points are not equal for both actors.",
				NULL
			);

			break;
		}

		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_VTX:
		case NpInternalAttachmentType::eSURFACE_VTX_SURFACE_VTX:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_VTX:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_VTX:
		case NpInternalAttachmentType::eVOLUME_VTX_VOLUME_VTX:
		case NpInternalAttachmentType::eVOLUME_VTX_SURFACE_VTX:
		default:
		{
			PX_CHECK_AND_RETURN_VAL(false, "PxDeformableAttachment: No matching actor pairs found for attachment.", NULL);
		}
	}

	return true;
}

void NpDeformableAttachment::addAttachment()
{
	if (!getSceneFromActors() ||  mEnabled)
		return;

	switch (mInternalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY:
		case NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE:
		case NpInternalAttachmentType::eSURFACE_VTX_RIGID_BODY:
		case NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_VTX_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_VTX_GLOBAL_POSE:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]] == NULL ? NULL : mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::BodyCore* core1 = mActor[mActorIndex[1]] == NULL ? NULL : getBodyCore(actor1);

			mHandles.resize(mIndices[mActorIndex[0]].size());
			for (PxU32 i = 0; i < mIndices[mActorIndex[0]].size(); i++)
			{
				PxU32 id = mIndices[mActorIndex[0]][i];

				PxVec3 actor1Pose = mCoords[mActorIndex[1]][i].getXYZ();

				if (actor1 && actor1->getConcreteType() == PxConcreteType::eRIGID_STATIC)
				{
					NpRigidStatic* stat = static_cast<NpRigidStatic*>(actor1);
					actor1Pose = stat->getGlobalPose().transform(actor1Pose);
				}

				actor1Pose = mPose[mActorIndex[1]].transform(actor1Pose);

				if (mInternalAttachmentType & NpInternalAttachmentType::eSURFACE_TYPE)
				{
					if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY ||
						mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_GLOBAL_POSE)
					{
						const PxVec4& barycentric = mCoords[mActorIndex[0]][i];
						mHandles[i] = getDeformableSurfaceCore(actor0)->addTriRigidAttachment(core1, id, barycentric, actor1Pose, NULL);
					}
					else if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_VTX_RIGID_BODY ||
							 mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE)
					{
						mHandles[i] = getDeformableSurfaceCore(actor0)->addRigidAttachment(core1, id, actor1Pose, NULL);
					}
				}
				else if (mInternalAttachmentType & NpInternalAttachmentType::eVOLUME_TYPE)
				{
					if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY ||
						mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_GLOBAL_POSE)
					{
						const PxVec4& barycentric = mCoords[mActorIndex[0]][i];
						mHandles[i] = getDeformableVolumeCore(actor0)->addTetRigidAttachment(core1, id, barycentric, actor1Pose, NULL, false);
					}
					else if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_VTX_RIGID_BODY ||
							 mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_VTX_GLOBAL_POSE)
					{
						mHandles[i] = getDeformableVolumeCore(actor0)->addRigidAttachment(core1, id, actor1Pose, NULL, false);
					}
				}
			}

			break;
		}

		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];
			PxActor* actor1 = mActor[mActorIndex[1]];

			mHandles.resize(mIndices[mActorIndex[0]].size());
			for (PxU32 i = 0; i < mIndices[mActorIndex[0]].size(); i++)
			{
				PxU32 id0 = mIndices[mActorIndex[0]][i];
				const PxVec4& barycentric0 = mCoords[mActorIndex[0]][i];

				PxU32 id1 = mIndices[mActorIndex[1]][i];
				const PxVec4& barycentric1 = mCoords[mActorIndex[1]][i];

				if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI)
				{
					mHandles[i] = getDeformableSurfaceCore(actor0)->addClothAttachment(getDeformableSurfaceCore(actor1), id1, barycentric1, id0, barycentric0);
				}
				else if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET)
				{
					mHandles[i] = getDeformableVolumeCore(actor0)->addSoftBodyAttachment(*getDeformableVolumeCore(actor1), id1, barycentric1, id0, barycentric0, NULL, 0.0f, false);
				}
				else if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI)
				{
					mHandles[i] = getDeformableVolumeCore(actor0)->addClothAttachment(*getDeformableSurfaceCore(actor1), id1, barycentric1, id0, barycentric0, NULL, 0.0f, false);
				}
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

	switch (mInternalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_VTX_RIGID_BODY:
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY:
		case NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_VTX_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_VTX_GLOBAL_POSE:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]] == NULL ? NULL : mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::BodyCore* core1 = mActor[mActorIndex[1]] == NULL ? NULL : getBodyCore(actor1);

			for (PxU32 i = 0; i < mHandles.size(); i++)
			{
				if (mInternalAttachmentType & NpInternalAttachmentType::eSURFACE_TYPE)
				{
					if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY)
					{
						getDeformableSurfaceCore(actor0)->removeTriRigidAttachment(core1, mHandles[i]);
					}
					else if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_VTX_RIGID_BODY || mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE)
					{
						getDeformableSurfaceCore(actor0)->removeRigidAttachment(core1, mHandles[i]);
					}
				}
				else if (mInternalAttachmentType & NpInternalAttachmentType::eVOLUME_TYPE)
				{
					getDeformableVolumeCore(actor0)->removeRigidAttachment(core1, mHandles[i]);
				}
			}

			break;
		}

		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];
			PxActor* actor1 = mActor[mActorIndex[1]];

			for (PxU32 i = 0; i < mHandles.size(); i++)
			{
				if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI)
				{
					getDeformableSurfaceCore(actor0)->removeClothAttachment(getDeformableSurfaceCore(actor1), mHandles[i]);
				}
				else if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET)
				{
					getDeformableVolumeCore(actor0)->removeSoftBodyAttachment(*getDeformableVolumeCore(actor1), mHandles[i]);
				}
				else if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI)
				{
					getDeformableVolumeCore(actor0)->removeClothAttachment(*getDeformableSurfaceCore(actor1), mHandles[i]);
				}
			}

			break;
		}

		default:
		{
			PX_ASSERT(0);
			break;
		}
	}

	mHandles.clear();
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
	NP_WRITE_CHECK(getNpScene());

	switch (mInternalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_VTX_RIGID_BODY:
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY:
		case NpInternalAttachmentType::eSURFACE_VTX_GLOBAL_POSE:
		case NpInternalAttachmentType::eVOLUME_VTX_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_VTX_GLOBAL_POSE:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]] == NULL ? NULL : mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::BodyCore* core1 = mActor[mActorIndex[1]] == NULL ? NULL : getBodyCore(actor1);

			for (PxU32 i = 0; i < mHandles.size(); i++)
			{
				if (mInternalAttachmentType & NpInternalAttachmentType::eSURFACE_TYPE)
				{
					if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY)
					{
						getDeformableSurfaceCore(actor0)->removeTriRigidAttachment(core1, mHandles[i]);
					}
					else
					{
						getDeformableSurfaceCore(actor0)->removeRigidAttachment(core1, mHandles[i]);
					}
				}
				else if (mInternalAttachmentType & NpInternalAttachmentType::eVOLUME_TYPE)
				{
					getDeformableVolumeCore(actor0)->removeRigidAttachment(core1, mHandles[i]);
				}
			}

			for (PxU32 i = 0; i < mHandles.size(); i++)
			{
				PxU32 id = mIndices[mActorIndex[0]][i];
				PxVec3 actor1Pose = mCoords[mActorIndex[1]][i].getXYZ();

				if (actor1 && actor1->getConcreteType() == PxConcreteType::eRIGID_STATIC)
				{
					NpRigidStatic* stat = static_cast<NpRigidStatic*>(actor1);
					actor1Pose = stat->getGlobalPose().transform(actor1Pose);
				}

				actor1Pose = pose.transform(actor1Pose);

				if (mInternalAttachmentType & NpInternalAttachmentType::eSURFACE_TYPE)
				{
					if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY)
					{
						const PxVec4& barycentric = mCoords[mActorIndex[0]][i];

						mHandles[i] = getDeformableSurfaceCore(actor0)->addTriRigidAttachment(core1, id, barycentric, actor1Pose, NULL);
					}
					else
					{
						mHandles[i] = getDeformableSurfaceCore(actor0)->addRigidAttachment(core1, id, actor1Pose, NULL);
					}
				}
				else if (mInternalAttachmentType & NpInternalAttachmentType::eVOLUME_TYPE)
				{
					if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY)
					{
						const PxVec4& barycentric = mCoords[mActorIndex[0]][i];

						mHandles[i] = getDeformableVolumeCore(actor0)->addTetRigidAttachment(core1, id, barycentric, actor1Pose, NULL, false);
					}
					else
					{
						mHandles[i] = getDeformableVolumeCore(actor0)->addRigidAttachment(core1, id, actor1Pose, NULL, false);
					}
				}
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
