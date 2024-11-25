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
#include "NpDeformableElementFilter.h"
#include "NpDeformableSurface.h"
#include "NpDeformableVolume.h"

#define PX_ELEMENT_FILTER_ALL	0x000fffff

// Assume they are the same for now. This can potentially change in the future.
PX_COMPILE_TIME_ASSERT(PX_ELEMENT_FILTER_ALL == PX_MAX_NB_DEFORMABLE_VOLUME_TET && PX_ELEMENT_FILTER_ALL == PX_MAX_NB_DEFORMABLE_SURFACE_TRI);

using namespace physx;

NpInternalAttachmentType::Enum getInternalAttachmentType(const PxDeformableElementFilterData& data, PxU32 actorIndex[2])
{
	// Make sure no actor is NULL
	// Element filtering is not supported for eWORLD attachment
	if (data.actor[0] == NULL || data.actor[1] == NULL)
		return NpInternalAttachmentType::eUNDEFINED;

	// Actor 0 is always assumed to be a deformable (deformable volume has the highest priority)
	// If Actor 0 is a rigid actor, the actors are swapped using the Actor Index
	// If Actor 0 is a deformable surface and Actor 1 is a deformable volume, the actors are swapped using the Actor Index
	actorIndex[0] = 0;
	actorIndex[1] = 1;

	if ((data.actor[0]->is<PxRigidActor>())
		|| (data.actor[0]->is<PxDeformableSurface>() && data.actor[1]->is<PxDeformableVolume>())
		)
	{
		// Swap the actor index.
		PxSwap(actorIndex[0], actorIndex[1]);
	}

	switch (data.actor[actorIndex[0]]->getConcreteType())
	{
		case PxConcreteType::eDEFORMABLE_SURFACE:
		{
			if (data.actor[actorIndex[1]]->is<PxRigidActor>())
			{
				return NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY;
			}
			else if (data.actor[actorIndex[1]]->is<PxDeformableSurface>())
			{
				return NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI;
			}
			break;
		}

		case PxConcreteType::eDEFORMABLE_VOLUME:
		{
			if (data.actor[actorIndex[1]]->is<PxRigidActor>())
			{
				return NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY;
			}
			else if (data.actor[actorIndex[1]]->is<PxDeformableSurface>())
			{
				return NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI;
			}
			else if (data.actor[actorIndex[1]]->is<PxDeformableVolume>())
			{
				return NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET;
			}

			break;
		}
	}

	return NpInternalAttachmentType::eUNDEFINED;
}

bool NpDeformableElementFilter::parseElementFilter(const PxDeformableElementFilterData& data, ElementFilterInfo& info)
{
	NpInternalAttachmentType::Enum& internalAttachmentType = info.internalAttachmentType;
	PxU32* actorIndex = info.actorIndex;

	internalAttachmentType = getInternalAttachmentType(data, actorIndex);

	if (internalAttachmentType != NpInternalAttachmentType::eUNDEFINED)
	{
		// For filtering deformable vs rigid, we allow the group count array to be empty for rigids
		PX_CHECK_AND_RETURN_VAL
		(
			(data.groupElementCounts[actorIndex[0]].count == data.groupElementCounts[actorIndex[1]].count) ||
			(data.actor[actorIndex[1]]->is<PxRigidActor>() && data.groupElementCounts[actorIndex[1]].count == 0),
			"PxDeformableElementFilter: Number of element groups are not equal for both actors.",
			NULL
		);

		// No support for wild card filtering against rigid body
		if (internalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY || internalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY)
		{
			PxTypedBoundedData<const PxU32> groupElementCounts = data.groupElementCounts[actorIndex[0]];

			for (PxU32 i = 0; i < groupElementCounts.count; i++)
			{
				if (groupElementCounts.at(i) == 0)
				{
					PX_CHECK_AND_RETURN_VAL(false, "PxDeformableElementFilter: No support for deformable group element count of 0 when filtering against a rigid body", NULL);
				}
			}
		}

		switch (internalAttachmentType)
		{
			case NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY:
			case NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY:
			case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
			case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
			case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
			{
				return true;
			}

			default:
			{
				PX_CHECK_AND_RETURN_VAL(false, "PxDeformableElementFilter: No matching actor pairs found for element filter.", NULL);
			}
		}
	}

	return false;
}

void NpDeformableElementFilter::addElementFilter()
{
	if (!getSceneFromActors() || mEnabled)
		return;

	switch (mInternalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::BodyCore* core1 = getBodyCore(actor1);

			for (PxU32 i = 0; i < mPairwiseIndices[mActorIndex[0]].size(); i++)
			{
				PxU32 id = mPairwiseIndices[mActorIndex[0]][i];

				if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY)
				{
					getDeformableVolumeCore(actor0)->addTetRigidFilter(core1, id);
				}
				else if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY)
				{
					getDeformableSurfaceCore(actor0)->addTriRigidFilter(core1, id);
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

			if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET)
			{
				getDeformableVolumeCore(actor0)->addSoftBodyFilters(*getDeformableVolumeCore(actor1), mPairwiseIndices[mActorIndex[1]].begin(), mPairwiseIndices[mActorIndex[0]].begin(), mPairwiseIndices[mActorIndex[0]].size());
			}
			else
			{
				for (PxU32 i = 0; i < mPairwiseIndices[mActorIndex[0]].size(); i++)
				{
					PxU32 id0 = mPairwiseIndices[mActorIndex[0]][i];
					PxU32 id1 = mPairwiseIndices[mActorIndex[1]][i];

					if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI)
					{
						getDeformableSurfaceCore(actor0)->addClothFilter(getDeformableSurfaceCore(actor1), id1, id0);
					}
					else if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI)
					{
						getDeformableVolumeCore(actor0)->addClothFilter(*getDeformableSurfaceCore(actor1), id1, id0);
					}
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

void NpDeformableElementFilter::removeElementFilter()
{
	if (!mEnabled)
		return;

	switch (mInternalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];
			const PxArray<PxU32>& indices = mPairwiseIndices[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::BodyCore* core1 = getBodyCore(actor1);

			for (PxU32 i = 0; i < indices.size(); i++)
			{
				if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY)
				{
					getDeformableVolumeCore(actor0)->removeTetRigidFilter(core1, indices[i]);
				}
				else if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY)
				{
					getDeformableSurfaceCore(actor0)->removeTriRigidFilter(core1, indices[i]);
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

			if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET)
			{
				getDeformableVolumeCore(actor0)->removeSoftBodyFilters(*getDeformableVolumeCore(actor1), mPairwiseIndices[mActorIndex[1]].begin(), mPairwiseIndices[mActorIndex[0]].begin(), mPairwiseIndices[mActorIndex[0]].size());
			}
			else
			{
				for (PxU32 i = 0; i < mPairwiseIndices[mActorIndex[0]].size(); i++)
				{
					PxU32 id0 = mPairwiseIndices[mActorIndex[0]][i];
					PxU32 id1 = mPairwiseIndices[mActorIndex[1]][i];

					if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI)
					{
						getDeformableSurfaceCore(actor0)->removeClothFilter(getDeformableSurfaceCore(actor1), id1, id0);
					}
					else if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI)
					{
						getDeformableVolumeCore(actor0)->removeClothFilter(*getDeformableSurfaceCore(actor1), id1, id0);
					}
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

	mEnabled = false;
}

NpScene* NpDeformableElementFilter::getSceneFromActors()
{
	const PxActor* actor[2] = { mActor[mActorIndex[0]], mActor[mActorIndex[1]] };

	if (actor[0] && actor[1])
	{
		if (actor[0]->getScene() != actor[1]->getScene())
		{
			PX_CHECK_MSG(false, "PxDeformableElementFilter: Actors belong to different scenes, undefined behavior expected!");

			return NULL;
		}

		return static_cast<NpScene*>(actor[0]->getScene());
	}

	return NULL;
}

NpDeformableElementFilter::NpDeformableElementFilter(const PxDeformableElementFilterData& data, const ElementFilterInfo& info)
	: PxDeformableElementFilter(PxConcreteType::eDEFORMABLE_ELEMENT_FILTER, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE), NpBase(NpType::eDEFORMABLE_ELEMENT_FILTER)
{
	mInternalAttachmentType = info.internalAttachmentType;
	mEnabled = false;

	// Expand groups to pairwise elements
	switch (info.internalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_BODY:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_BODY:
		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		{
			PxU32 elementStartIndex0 = 0;
			PxU32 elementStartIndex1 = 0;

			for (PxU32 i = 0; i < data.groupElementCounts[info.actorIndex[0]].count; i++)
			{
				PxU32 elementCount0 = data.groupElementCounts[info.actorIndex[0]].at(i);
				PxU32 elementCount1 = 0;

				if (data.groupElementCounts[info.actorIndex[1]].count != 0)
					elementCount1 = data.groupElementCounts[info.actorIndex[1]].at(i);

				bool isElementCountZero0 = (elementCount0 == 0);
				bool isElementCountZero1 = (elementCount1 == 0);

				for (PxU32 j = 0; j < elementCount0 + isElementCountZero0; j++)
				{
					PxU32 index0 = PX_ELEMENT_FILTER_ALL;

					if (!isElementCountZero0)
						index0 = data.groupElementIndices[info.actorIndex[0]].at(j + elementStartIndex0);

					for (PxU32 k = 0; k < elementCount1 + isElementCountZero1; k++)
					{
						PxU32 index1 = PX_ELEMENT_FILTER_ALL;

						if (!isElementCountZero1)
							index1 = data.groupElementIndices[info.actorIndex[1]].at(k + elementStartIndex1);

						mPairwiseIndices[info.actorIndex[0]].pushBack(index0);
						mPairwiseIndices[info.actorIndex[1]].pushBack(index1);
					}
				}

				elementStartIndex0 += elementCount0;
				elementStartIndex1 += elementCount1;
			}

			break;
		}

		default:
		{
			PX_ASSERT(0);
			break;
		}
	}

	for (PxU32 i = 0; i < 2; i++)
	{
		mActor[i] = data.actor[i];

		mCounts[i].resize(data.groupElementCounts[i].count);
		for (PxU32 j = 0; j < data.groupElementCounts[i].count; j++)
		{
			mCounts[i][j] = data.groupElementCounts[i].at(j);
		}

		mIndices[i].resize(data.groupElementIndices[i].count);
		for (PxU32 j = 0; j < data.groupElementIndices[i].count; j++)
		{
			mIndices[i][j] = data.groupElementIndices[i].at(j);
		}

		mActorIndex[i] = info.actorIndex[i];

		// Add connector
		if (mActor[i])
			NpActor::getFromPxActor(*mActor[i]).addConnector(NpConnectorType::eElementFilter, this, "PxDeformableElementFilter: Element filter already added");
	}

	NpScene* s = getSceneFromActors();
	if (s)
	{
		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(s, "PxDeformableElementFilter creation not allowed while simulation is running. Call will be ignored.");

		s->addToElementFilterList(*this);
	}

	setNpScene(s);
}

NpDeformableElementFilter::~NpDeformableElementFilter()
{
	NpFactory::getInstance().onElementFilterRelease(this);
}

void NpDeformableElementFilter::release()
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableElementFilter: Illegal to call release while simulation is running.");

	for (PxU32 i = 0; i < 2; i++)
	{
		// Remove connector
		if (mActor[i])
		{
			PxDeformableElementFilter* buffer[1];

			if (NpActor::getFromPxActor(*mActor[i]).getConnectors(NpConnectorType::eElementFilter, buffer, 1))
				NpActor::getFromPxActor(*mActor[i]).removeConnector(*mActor[i], NpConnectorType::eElementFilter, this, "PxDeformableElementFilter: ElementFilter already released");
		}
	}

	if (npScene)
	{
		npScene->removeFromElementFilterList(*this);
	}

	NpDestroyElementFilter(this);
}

void NpDeformableElementFilter::getActors(PxActor*& actor0, PxActor*& actor1) const
{
	NP_READ_CHECK(getNpScene());

	actor0 = mActor[0];
	actor1 = mActor[1];
}

#endif
