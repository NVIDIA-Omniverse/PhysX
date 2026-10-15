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
#include "NpDeformableElementFilter.h"
#include "NpDeformableSurface.h"
#include "NpDeformableVolume.h"

// Element index reserved as a wildcard, indicating all elements of the actor. Substituted when a
// filter group specifies no element indices, and not exposed through the API.
#define PX_ELEMENT_FILTER_ALL	0x000fffff

// The GPU element filter encoding reuses this value as its own wildcard, so the equality is load
// bearing rather than incidental. See PxgDeformableRef::ANY_FEATURE.
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
				return NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR;
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
				return NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR;
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

#if PX_CHECKED

// Whether any group of this actor references specific elements, meaning its element count is greater
// than 0. An empty group is the wildcard for all elements of the actor.
static bool isReferencingSpecificElements(const PxTypedBoundedData<const PxU32>& groupElementCounts)
{
	for (PxU32 i = 0; i < groupElementCounts.count; i++)
	{
		if (groupElementCounts.at(i) > 0)
			return true;
	}

	return false;
}

#endif // PX_CHECKED

// How many element indices are left in the array from 'start'.
static PxU32 getRemainingElements(const PxTypedBoundedData<const PxU32>& groupElementIndices, PxU32 start)
{
	return groupElementIndices.count > start ? groupElementIndices.count - start : 0;
}

bool NpDeformableElementFilter::parseElementFilter(const PxDeformableElementFilterData& data, ElementFilterInfo& info)
{
	NpInternalAttachmentType::Enum& internalAttachmentType = info.internalAttachmentType;
	PxU32* actorIndex = info.actorIndex;

	internalAttachmentType = getInternalAttachmentType(data, actorIndex);

	if (internalAttachmentType != NpInternalAttachmentType::eUNDEFINED)
	{
		// An actor with no groups at all is filtered as a whole against every group of the other actor,
		// which is always the form a rigid takes.
		PX_CHECK_AND_RETURN_VAL
		(
			(data.groupElementCounts[actorIndex[0]].count == data.groupElementCounts[actorIndex[1]].count) ||
			(data.groupElementCounts[actorIndex[0]].count == 0) || (data.groupElementCounts[actorIndex[1]].count == 0),
			"PxDeformableElementFilter: Number of element groups are not equal for both actors.",
			NULL
		);

		if (internalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR || internalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR)
		{
			// Filtering against a rigid relates to the whole actor, so the rigid has no elements for an index
			// to address. Indices given for it are dropped when the filter pairs are built, and a group count
			// above 0 would result in the expansion looking up non-existing indices.
			PX_CHECK_AND_RETURN_VAL
			(
				data.groupElementIndices[actorIndex[1]].count == 0 &&
				!isReferencingSpecificElements(data.groupElementCounts[actorIndex[1]]),
				"PxDeformableElementFilter: a rigid actor is filtered as a whole, so it supports no element indices "
				"and no element group count above 0.",
				NULL
			);

		}

		// A filter has to reference at least one element on at least one of the two actors. An empty group is
		// the wildcard for all elements of its actor, not a reference to a specific one.
		PX_CHECK_AND_RETURN_VAL
		(
			isReferencingSpecificElements(data.groupElementCounts[actorIndex[0]]) || isReferencingSpecificElements(data.groupElementCounts[actorIndex[1]]),
			"PxDeformableElementFilter: a filter has to reference at least one specific element on at least one "
			"of the two actors. To suppress all collisions between two actors, use PxShape- or PxActor-level "
			"collision filtering with a PxSimulationFilterShader instead.",
			NULL
		);

		// Likewise, every group pair has to reference at least one specific element on at least one of the
		// two actors. Where one actor supplies no groups its side of every pair is considered as the
		// wildcard. Therefore the groups of the other actor each have to reference at least one specific
		// element.
		{
			const PxU32 groupCount0 = data.groupElementCounts[actorIndex[0]].count;
			const PxU32 groupCount1 = data.groupElementCounts[actorIndex[1]].count;
			const PxU32 groupCount = PxMax(groupCount0, groupCount1);

			for (PxU32 i = 0; i < groupCount; i++)
			{
				PX_CHECK_AND_RETURN_VAL
				(
					(groupCount0 > 0 && data.groupElementCounts[actorIndex[0]].at(i) > 0) ||
					(groupCount1 > 0 && data.groupElementCounts[actorIndex[1]].at(i) > 0),
					"PxDeformableElementFilter: every group pair has to reference at least one specific element on at "
					"least one of the two actors. To suppress all collisions between two actors, use PxShape- or "
					"PxActor-level collision filtering with a PxSimulationFilterShader instead.",
					NULL
				);
			}
		}

		switch (internalAttachmentType)
		{
			case NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR:
			case NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR:
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

#if PX_CHECKED

// Number of elements an element filter addresses on a deformable actor. A surface is filtered by the
// triangles of the mesh it simulates and a volume by the tetrahedra of its collision mesh, and both
// of those are the shape's geometry, so the geometry type is the element kind.
//
// A count of zero refuses every index. PxScene::addActor rejects a deformable without a shape, so
// zero is the answer for a state a filter being enabled cannot be in, rather than one to accept
// unchecked.
static PxU32 getElementCount(PxDeformableBody& body)
{
	PxShape* shape = body.getShape();
	if (shape == NULL)
		return 0;

	const PxGeometry& geometry = shape->getGeometry();

	if (geometry.getType() == PxGeometryType::eTRIANGLEMESH)
	{
		const PxTriangleMesh* mesh = static_cast<const PxTriangleMeshGeometry&>(geometry).triangleMesh;
		return mesh ? mesh->getNbTriangles() : 0;
	}

	if (geometry.getType() == PxGeometryType::eTETRAHEDRONMESH)
	{
		const PxTetrahedronMesh* mesh = static_cast<const PxTetrahedronMeshGeometry&>(geometry).tetrahedronMesh;
		return mesh ? mesh->getNbTetrahedrons() : 0;
	}

	return 0;
}

// An index past the end of the mesh reads the mesh's element indices out of bounds when the filter
// pairs are built. Checked here rather than at creation so that it does not depend on whether the
// mesh happened to be attached yet, and so that swapping in a smaller mesh is caught when the filter
// is enabled again. The wildcard needs no exemption, since it is substituted into the pairwise
// indices for an empty group while these are the indices the caller supplied.
bool NpDeformableElementFilter::checkIndicesInRange() const
{
	for (PxU32 i = 0; i < 2; i++)
	{
		PxDeformableBody* body = mActor[i]->is<PxDeformableBody>();
		if (body == NULL)
			continue;

		const PxU32 count = getElementCount(*body);

		for (PxU32 j = 0; j < mIndices[i].size(); j++)
		{
			if (mIndices[i][j] >= count)
				return false;
		}
	}

	return true;
}

#endif // PX_CHECKED

void NpDeformableElementFilter::addElementFilter()
{
	if (!getSceneFromActors() || mEnabled)
		return;

	PX_CHECK_AND_RETURN(checkIndicesInRange(),
		"PxDeformableElementFilter: element index is out of range for the deformable body's collision mesh. Element filter is ignored.");

	switch (mInternalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::RigidCore* core1 = getRigidCore(actor1);

			PxArray<PxU32>& indices = mPairwiseIndices[mActorIndex[0]];

			if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR)
			{
				getDeformableVolumeCore(actor0)->addRigidFilters(core1, indices.begin(), indices.size());
			}
			else if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR)
			{
				getDeformableSurfaceCore(actor0)->addRigidFilters(core1, indices.begin(), indices.size());
			}

			break;
		}

		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];
			PxActor* actor1 = mActor[mActorIndex[1]];

			PxU32* indices0 = mPairwiseIndices[mActorIndex[0]].begin();
			PxU32* indices1 = mPairwiseIndices[mActorIndex[1]].begin();
			const PxU32 indicesSize = mPairwiseIndices[mActorIndex[0]].size();

			if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET)
			{
				getDeformableVolumeCore(actor0)->addVolumeFilters(*getDeformableVolumeCore(actor1), indices1, indices0, indicesSize);
			}
			else if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI)
			{
				getDeformableSurfaceCore(actor0)->addSurfaceFilters(getDeformableSurfaceCore(actor1), indices1, indices0, indicesSize);
			}
			else if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI)
			{
				getDeformableVolumeCore(actor0)->addSurfaceFilters(*getDeformableSurfaceCore(actor1), indices1, indices0, indicesSize);
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
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];
			PxArray<PxU32>& indices = mPairwiseIndices[mActorIndex[0]];

			PxRigidActor* actor1 = mActor[mActorIndex[1]]->is<PxRigidActor>();
			Sc::RigidCore* core1 = getRigidCore(actor1);

			if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR)
			{
				getDeformableVolumeCore(actor0)->removeRigidFilters(core1, indices.begin(), indices.size());
			}
			else if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR)
			{
				getDeformableSurfaceCore(actor0)->removeRigidFilters(core1, indices.begin(), indices.size());
			}

			break;
		}

		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		{
			PxActor* actor0 = mActor[mActorIndex[0]];
			PxActor* actor1 = mActor[mActorIndex[1]];

			PxU32* indices0 = mPairwiseIndices[mActorIndex[0]].begin();
			PxU32* indices1 = mPairwiseIndices[mActorIndex[1]].begin();
			const PxU32 indicesSize = mPairwiseIndices[mActorIndex[0]].size();

			if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET)
			{
				getDeformableVolumeCore(actor0)->removeVolumeFilters(*getDeformableVolumeCore(actor1), indices1, indices0, indicesSize);
			}
			else if (mInternalAttachmentType == NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI)
			{
				getDeformableSurfaceCore(actor0)->removeSurfaceFilters(getDeformableSurfaceCore(actor1), indices1, indices0, indicesSize);
			}
			else if (mInternalAttachmentType == NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI)
			{
				getDeformableVolumeCore(actor0)->removeSurfaceFilters(*getDeformableSurfaceCore(actor1), indices1, indices0, indicesSize);
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

// A connector identifies the filter rather than the slot, so a filter referencing one actor in both
// slots registers a single connector, through slot 0.
static PX_FORCE_INLINE bool ownsConnector(PxActor* const (&actor)[2], PxU32 slot)
{
	return actor[slot] != NULL && (slot == 0 || actor[1] != actor[0]);
}

NpDeformableElementFilter::NpDeformableElementFilter(const PxDeformableElementFilterData& data, const ElementFilterInfo& info)
	: PxDeformableElementFilter(PxConcreteType::eDEFORMABLE_ELEMENT_FILTER, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE), NpBase(NpType::eDEFORMABLE_ELEMENT_FILTER)
{
	mInternalAttachmentType = info.internalAttachmentType;
	mEnabled = false;

	// Expand groups to pairwise elements
	switch (info.internalAttachmentType)
	{
		case NpInternalAttachmentType::eSURFACE_TRI_RIGID_ACTOR:
		case NpInternalAttachmentType::eVOLUME_TET_RIGID_ACTOR:
		case NpInternalAttachmentType::eSURFACE_TRI_SURFACE_TRI:
		case NpInternalAttachmentType::eVOLUME_TET_VOLUME_TET:
		case NpInternalAttachmentType::eVOLUME_TET_SURFACE_TRI:
		{
			PxU32 elementStartIndex0 = 0;
			PxU32 elementStartIndex1 = 0;

			// An actor with no groups is filtered as a whole, so the other actor's groups set the count and
			// every group of it pairs with the wildcard.
			const PxU32 nbGroups = PxMax(data.groupElementCounts[info.actorIndex[0]].count,
				data.groupElementCounts[info.actorIndex[1]].count);

			for (PxU32 i = 0; i < nbGroups; i++)
			{
				PxU32 elementCount0 = 0;
				PxU32 elementCount1 = 0;

				if (data.groupElementCounts[info.actorIndex[0]].count != 0)
					elementCount0 = data.groupElementCounts[info.actorIndex[0]].at(i);

				if (data.groupElementCounts[info.actorIndex[1]].count != 0)
					elementCount1 = data.groupElementCounts[info.actorIndex[1]].at(i);

				// The element indices are not validated against the counts, and parseElementFilter's checks
				// compile out in release, so a count reaching past its index array is clamped to what is
				// there. The group falls back to the wildcard instead of reading out of bounds.
				elementCount0 = PxMin(elementCount0,
					getRemainingElements(data.groupElementIndices[info.actorIndex[0]], elementStartIndex0));
				elementCount1 = PxMin(elementCount1,
					getRemainingElements(data.groupElementIndices[info.actorIndex[1]], elementStartIndex1));

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
		if (ownsConnector(mActor, i))
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
		if (ownsConnector(mActor, i))
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
