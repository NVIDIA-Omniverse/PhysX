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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "ScNPhaseCore.h"
#include "ScShapeInteraction.h"
#include "ScTriggerInteraction.h"
#include "ScElementInteractionMarker.h"
#include "ScConstraintInteraction.h"
#include "ScConstraintSim.h"
#include "ScConstraintCore.h"
#include "ScSimStats.h"
#include "ScObjectIDTracker.h"
#include "ScSimStats.h"

#include "foundation/PxThread.h"
#include "BpBroadPhase.h"
#include "common/PxProfileZone.h"
#include "ScSoftBodyShapeSim.h"
#include "ScParticleSystemShapeSim.h"
#include "ScArticulationSim.h"

using namespace physx;
using namespace Sc;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

class Sc::FilterPairManager : public PxUserAllocated
{
	PX_NOCOPY(FilterPairManager)
public:
	FilterPairManager()
	: mPairs("FilterPairManager Array")
	, mFree(INVALID_FILTER_PAIR_INDEX)
	{}

	PxU32 acquireIndex()
	{
		PxU32 index;
		if(mFree == INVALID_FILTER_PAIR_INDEX)
		{
			index = mPairs.size();
			mPairs.pushBack(NULL);
		}
		else
		{
			index = PxU32(mFree);
			mFree = reinterpret_cast<uintptr_t>(mPairs[index]);
			mPairs[index] = NULL;
		}
		return index;
	}

	void releaseIndex(PxU32 index)
	{		
		mPairs[index] = reinterpret_cast<Sc::ElementSimInteraction*>(mFree);
		mFree = index;
	}

	void setPair(PxU32 index, Sc::ElementSimInteraction* ptr)
	{
		mPairs[index] = ptr;
	}

	Sc::ElementSimInteraction*	operator[](PxU32 index)
	{
		return mPairs[index];
	}

	PxU32 findIndex(Sc::ElementSimInteraction* ptr)
	{
		return ptr->getFilterPairIndex();
	}

private:	
	PxArray<Sc::ElementSimInteraction*> mPairs;	
	uintptr_t mFree;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE PxU32 hasTriggerFlags(PxShapeFlags flags)	{ return PxU32(flags) & PxU32(PxShapeFlag::eTRIGGER_SHAPE);	}
static void getFilterInfo_ShapeSim(PxFilterObjectAttributes& filterAttr, PxFilterData& filterData, const Sc::ShapeSim& shape)
{
	filterAttr = hasTriggerFlags(shape.getCore().getFlags()) ? PxFilterObjectFlag::eTRIGGER : PxFilterObjectFlag::Enum(0);

	BodySim* b = shape.getBodySim();
	if(b)
	{
		if(!b->isArticulationLink())
		{
			if(b->isKinematic())
				filterAttr |= PxFilterObjectFlag::eKINEMATIC;

			setFilterObjectAttributeType(filterAttr, PxFilterObjectType::eRIGID_DYNAMIC);
		}
		else
			setFilterObjectAttributeType(filterAttr, PxFilterObjectType::eARTICULATION);
	}
	else
	{
		// For softbody and particle system, the bodySim is set to null
		if (shape.getActor().isSoftBody())
			setFilterObjectAttributeType(filterAttr, PxFilterObjectType::eSOFTBODY);
		else if (shape.getActor().isParticleSystem())
			setFilterObjectAttributeType(filterAttr, PxFilterObjectType::ePARTICLESYSTEM);
		else if (shape.getActor().isHairSystem())
			setFilterObjectAttributeType(filterAttr, PxFilterObjectType::eHAIRSYSTEM);
		else
			setFilterObjectAttributeType(filterAttr, PxFilterObjectType::eRIGID_STATIC);
	}

	filterData = shape.getCore().getSimulationFilterData();
}

static PX_FORCE_INLINE void getFilterInfo(PxFilterData& fd, PxFilterObjectAttributes& fa, const ElementSim& e)
{
	getFilterInfo_ShapeSim(fa, fd, static_cast<const ShapeSim&>(e));
}

static void getFilterInfo(PxFilterData& fd0, PxFilterData& fd1, PxFilterObjectAttributes& fa0, PxFilterObjectAttributes& fa1, const ElementSim& e0, const ElementSim& e1)
{
	getFilterInfo(fd0, fa0, e0);
	getFilterInfo(fd1, fa1, e1);
}

static PX_INLINE void callPairLost(Scene& scene, const ElementSim& e0, const ElementSim& e1, PxU32 pairID, bool objVolumeRemoved)
{
	PxFilterData fd0(PxEmpty), fd1(PxEmpty);
	PxFilterObjectAttributes fa0, fa1;
	getFilterInfo(fd0, fd1, fa0, fa1, e0, e1);

	scene.getFilterCallbackFast()->pairLost(pairID, fa0, fd0, fa1, fd1, objVolumeRemoved);
}

// Filtering

static PX_INLINE void checkFilterFlags(PxFilterFlags& filterFlags)
{
	if((filterFlags & (PxFilterFlag::eKILL | PxFilterFlag::eSUPPRESS)) == (PxFilterFlag::eKILL | PxFilterFlag::eSUPPRESS))
	{
#if PX_CHECKED
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: eKILL and eSUPPRESS must not be set simultaneously. eSUPPRESS will be used.");
#endif
		filterFlags.clear(PxFilterFlag::eKILL);
	}
}

static PX_FORCE_INLINE PxPairFlags checkRbPairFlags(const ShapeSimBase& s0, const ShapeSimBase& s1, PxPairFlags pairFlags)
{
#if PX_CHECKED
	// we want to avoid to run contact generation for pairs that should not get resolved or have no contact/trigger reports
	if (!(PxU32(pairFlags) & (PxPairFlag::eSOLVE_CONTACT | ShapeInteraction::CONTACT_REPORT_EVENTS)))
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: Pair with no contact/trigger reports detected, nor is PxPairFlag::eSOLVE_CONTACT set. It is recommended to suppress/kill such pairs for performance reasons.");
	else if(!(pairFlags & (PxPairFlag::eDETECT_DISCRETE_CONTACT | PxPairFlag::eDETECT_CCD_CONTACT)))
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: Pair did not request either eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT. It is recommended to suppress/kill such pairs for performance reasons.");

	if(((s0.getFlags() & PxShapeFlag::eTRIGGER_SHAPE)!=0 || (s1.getFlags() & PxShapeFlag::eTRIGGER_SHAPE)!=0) &&
		(pairFlags & PxPairFlag::eTRIGGER_DEFAULT) && (pairFlags & PxPairFlag::eDETECT_CCD_CONTACT))
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: CCD isn't supported on Triggers yet");
#else
	PX_UNUSED(s0);
	PX_UNUSED(s1);
#endif
	return pairFlags;
}

static PX_INLINE PxPairFlags checkRbPairFlags(	const ShapeSimBase& s0, const ShapeSimBase& s1,
												const ActorSim& bs0, const ActorSim& bs1,
												PxPairFlags pairFlags, PxFilterFlags filterFlags,
												bool isNonRigid)
{
	if(filterFlags & (PxFilterFlag::eSUPPRESS | PxFilterFlag::eKILL))
		return pairFlags;

	if (bs0.isDynamicRigid() && static_cast<const BodySim&>(bs0).isKinematic() && 
		bs1.isDynamicRigid() && static_cast<const BodySim&>(bs1).isKinematic() && 
		(pairFlags & PxPairFlag::eSOLVE_CONTACT))
	{
#if PX_CHECKED
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: Resolving contacts between two kinematic objects is invalid. Contacts will not get resolved.");
#endif
		pairFlags.clear(PxPairFlag::eSOLVE_CONTACT);
	}

	if (isNonRigid && (pairFlags & PxPairFlag::eDETECT_CCD_CONTACT))
		pairFlags.clear(PxPairFlag::eDETECT_CCD_CONTACT);

	return checkRbPairFlags(s0, s1, pairFlags);
}

// PT: version specialized for ShapeSim/ShapeSim
static PX_INLINE PxPairFlags checkRbPairFlags(	const ShapeSimBase& s0, const ShapeSimBase& s1,
												bool kine0, bool kine1,
												PxPairFlags pairFlags, PxFilterFlags filterFlags,
												bool isNonRigid)
{
	if(filterFlags & (PxFilterFlag::eSUPPRESS | PxFilterFlag::eKILL))
		return pairFlags;

	if(kine0 && kine1 && (pairFlags & PxPairFlag::eSOLVE_CONTACT))
	{
#if PX_CHECKED
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: Resolving contacts between two kinematic objects is invalid. Contacts will not get resolved.");
#endif
		pairFlags.clear(PxPairFlag::eSOLVE_CONTACT);
	}

	if (pairFlags & PxPairFlag::eDETECT_CCD_CONTACT && isNonRigid)
	{
		pairFlags.clear(PxPairFlag::eDETECT_CCD_CONTACT);
	}

	return checkRbPairFlags(s0, s1, pairFlags);
}

static PX_FORCE_INLINE void fetchActorAndShape(const ElementSim& e, PxActor*& a, PxShape*& s)
{
	const ShapeSimBase& sim = static_cast<const ShapeSimBase&>(e);
	a = sim.getActor().getPxActor();

	PxActorType::Enum type = sim.getActor().getActorType();
	if (type == PxActorType::ePBD_PARTICLESYSTEM ||
		type == PxActorType::eFLIP_PARTICLESYSTEM ||
		type == PxActorType::eMPM_PARTICLESYSTEM ||
		type == PxActorType::eCUSTOM_PARTICLESYSTEM)
		s = NULL;	// Particle system does not have a valid shape so set it to null
	else
		s = sim.getPxShape();
}

static void runFilter(PxFilterInfo& filterInfo, const FilteringContext& context, const ElementSim& e0, const ElementSim& e1, PxU32 filterPairIndex, bool doCallbacks)
{
	PxFilterData fd0(PxEmpty), fd1(PxEmpty);
	PxFilterObjectAttributes fa0, fa1;
	getFilterInfo(fd0, fd1, fa0, fa1, e0, e1);

	// Run filter shader
	filterInfo.filterFlags = context.mFilterShader(fa0, fd0, fa1, fd1, filterInfo.pairFlags, context.mFilterShaderData, context.mFilterShaderDataSize);

	if(filterInfo.filterFlags & PxFilterFlag::eCALLBACK)
	{
		if(context.mFilterCallback)
		{
			if(!doCallbacks)
			{
				return;
			}
			else
			{
				if(filterPairIndex == INVALID_FILTER_PAIR_INDEX)
					filterPairIndex = context.mFilterPairManager->acquireIndex();
				// If a FilterPair is provided, then we use it, else we create a new one
				// (A FilterPair is provided in the case for a pairLost()-pairFound() sequence after refiltering)

				PxActor* a0, *a1;
				PxShape* s0, *s1;
				fetchActorAndShape(e0, a0, s0);
				fetchActorAndShape(e1, a1, s1);

				filterInfo.filterFlags = context.mFilterCallback->pairFound(filterPairIndex, fa0, fd0, a0, s0, fa1, fd1, a1, s1, filterInfo.pairFlags);
				filterInfo.filterPairIndex = filterPairIndex;
			}
		}
		else
		{
			filterInfo.filterFlags.clear(PxFilterFlag::eNOTIFY);
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: eCALLBACK set but no filter callback defined.");
		}
	}

	checkFilterFlags(filterInfo.filterFlags);

	if(filterPairIndex!=INVALID_FILTER_PAIR_INDEX && ((filterInfo.filterFlags & PxFilterFlag::eKILL) || ((filterInfo.filterFlags & PxFilterFlag::eNOTIFY) != PxFilterFlag::eNOTIFY)))
	{
		if((filterInfo.filterFlags & PxFilterFlag::eKILL) && ((filterInfo.filterFlags & PxFilterFlag::eNOTIFY) == PxFilterFlag::eNOTIFY))
			context.mFilterCallback->pairLost(filterPairIndex, fa0, fd0, fa1, fd1, false);

		if((filterInfo.filterFlags & PxFilterFlag::eNOTIFY) != PxFilterFlag::eNOTIFY)
		{
			// No notification, hence we don't need to treat it as a filter callback pair anymore.
			// Make sure that eCALLBACK gets removed as well
			filterInfo.filterFlags.clear(PxFilterFlag::eNOTIFY);
		}

		context.mFilterPairManager->releaseIndex(filterPairIndex);
		filterInfo.filterPairIndex = INVALID_FILTER_PAIR_INDEX;
	}

	// Sanity checks
	PX_ASSERT(	(filterInfo.filterFlags != PxFilterFlag::eKILL) ||
				((filterInfo.filterFlags == PxFilterFlag::eKILL) && (filterInfo.filterPairIndex == INVALID_FILTER_PAIR_INDEX)) );
	PX_ASSERT(	((filterInfo.filterFlags & PxFilterFlag::eNOTIFY) != PxFilterFlag::eNOTIFY) ||
				(((filterInfo.filterFlags & PxFilterFlag::eNOTIFY) == PxFilterFlag::eNOTIFY) && filterInfo.filterPairIndex!=INVALID_FILTER_PAIR_INDEX) );
}

// PT: version specialized for ShapeSim/ShapeSim
static PX_FORCE_INLINE void runFilterShapeSim(PxFilterInfo& filterInfo, const FilteringContext& context, const ShapeSimBase& e0, const ShapeSimBase& e1, const PxFilterObjectAttributes fa0, const PxFilterObjectAttributes fa1)
{
	// Run filter shader
	{
		const PxFilterData fd0 = e0.getCore().getSimulationFilterData();
		const PxFilterData fd1 = e1.getCore().getSimulationFilterData();
		filterInfo.filterFlags = context.mFilterShader(fa0, fd0, fa1, fd1, filterInfo.pairFlags, context.mFilterShaderData, context.mFilterShaderDataSize);
	}

	if(filterInfo.filterFlags & PxFilterFlag::eCALLBACK)
	{
		if(context.mFilterCallback)
		{
			return;
		}
		else
		{
			filterInfo.filterFlags.clear(PxFilterFlag::eNOTIFY);
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: eCALLBACK set but no filter callback defined.");
		}
	}

	checkFilterFlags(filterInfo.filterFlags);

	// Sanity checks
	PX_ASSERT(	(filterInfo.filterFlags != PxFilterFlag::eKILL) ||
				((filterInfo.filterFlags == PxFilterFlag::eKILL) && (filterInfo.filterPairIndex == INVALID_FILTER_PAIR_INDEX)) );
	PX_ASSERT(	((filterInfo.filterFlags & PxFilterFlag::eNOTIFY) != PxFilterFlag::eNOTIFY) ||
				(((filterInfo.filterFlags & PxFilterFlag::eNOTIFY) == PxFilterFlag::eNOTIFY) && filterInfo.filterPairIndex!=INVALID_FILTER_PAIR_INDEX) );
}

// helper method for some cleanup code that is used multiple times for early outs in case a rigid body collision pair gets filtered out due to some hardwired filter criteria
static PX_FORCE_INLINE PxFilterInfo filterOutRbCollisionPair(FilterPairManager* filterPairManager, PxU32 filterPairIndex, const PxFilterFlags filterFlags)
{
	if(filterPairIndex!=INVALID_FILTER_PAIR_INDEX)
		filterPairManager->releaseIndex(filterPairIndex);

	return PxFilterInfo(filterFlags);
}

PxFilterInfo Sc::filterRbCollisionPairSecondStage(const FilteringContext& context, const ShapeSimBase& s0, const ShapeSimBase& s1, const ActorSim& b0, const ActorSim& b1, PxU32 filterPairIndex, bool runCallbacks,
	bool isNonRigid)
{
	PxFilterInfo filterInfo;

	runFilter(filterInfo, context, s0, s1, filterPairIndex, runCallbacks);

	if(runCallbacks || (!(filterInfo.filterFlags & PxFilterFlag::eCALLBACK)))
		filterInfo.pairFlags = checkRbPairFlags(s0, s1, b0, b1, filterInfo.pairFlags, filterInfo.filterFlags, isNonRigid);

	return filterInfo;
}

// PT: version specialized for ShapeSim/ShapeSim
static PX_FORCE_INLINE PxFilterInfo filterRbCollisionPairSecondStage(const FilteringContext& context, const ShapeSimBase& s0, const ShapeSimBase& s1, bool kine0, bool kine1, const PxFilterObjectAttributes fa0, const PxFilterObjectAttributes fa1,
	bool isNonRigid)
{
	PxFilterInfo filterInfo;
	runFilterShapeSim(filterInfo, context, s0, s1, fa0, fa1);

	if(!(filterInfo.filterFlags & PxFilterFlag::eCALLBACK))
		filterInfo.pairFlags = checkRbPairFlags(s0, s1, kine0, kine1, filterInfo.pairFlags, filterInfo.filterFlags, isNonRigid);

	return filterInfo;
}

static bool filterArticulationLinks(const ActorSim& rbActor0, const ActorSim& rbActor1)
{
	{
		//It's the same articulation, so we can filter based on flags...
		const BodySim& bs0 = static_cast<const BodySim&>(rbActor0);
		const BodySim& bs1 = static_cast<const BodySim&>(rbActor1);

		const ArticulationSim* articulationSim0 = bs0.getArticulation();
		const ArticulationSim* articulationSim1 = bs1.getArticulation();
		if (articulationSim0 == articulationSim1)
		{
			if (articulationSim0->getCore().getArticulationFlags() & PxArticulationFlag::eDISABLE_SELF_COLLISION)
				return true;

			//check to see if one link is the parent of the other link, if so disable collision
			PxU32 linkId0 = bs0.getNodeIndex().articulationLinkId();
			PxU32 linkId1 = bs1.getNodeIndex().articulationLinkId();
			

			const Dy::ArticulationLink& link0 = articulationSim0->getLink(linkId0);
			const Dy::ArticulationLink& link1 = articulationSim1->getLink(linkId1);

			if (linkId1 < linkId0)
				return link0.parent == linkId1;
			
			return link1.parent == linkId0;
		}
	}
	
	return false;
}

static bool filterJointedBodies2(const ActorSim& actor, const ActorSim& other)
{
	if(!actor.isDynamicRigid())
		return false;

	ConstraintCore* core = actor.getScene().findConstraintCore(&actor, &other);

	if(core)
		return !(core->getFlags() & PxConstraintFlag::eCOLLISION_ENABLED);
	return false;

	/*const Sc::ActorSim* actorToMatch;
	PxU32 size;
	Interaction** interactions;

	if(actor.getActorInteractionCount() <= other.getActorInteractionCount())
	{
		size = actor.getActorInteractionCount();
		interactions = actor.getActorInteractions();
		actorToMatch = &other;
	}
	else
	{
		size = other.getActorInteractionCount();
		interactions = other.getActorInteractions();
		actorToMatch = &actor;
	}

	while(size--)
	{
		Interaction* interaction = *interactions++;
		if(interaction->getType() == InteractionType::eCONSTRAINTSHADER)
		{
			ConstraintInteraction* csi = static_cast<ConstraintInteraction*>(interaction);
			if((&csi->getActorSim0() == actorToMatch) || (&csi->getActorSim1() == actorToMatch))
				return !(csi->getConstraint()->getCore().getFlags() & PxConstraintFlag::eCOLLISION_ENABLED);
		}
	}
	return false;*/
}

static PX_FORCE_INLINE bool filterJointedBodies(const ActorSim& rbActor0, const ActorSim& rbActor1)
{
	// If the bodies of the shape pair are connected by a joint, we need to check whether this connection disables the collision.
	// Note: As an optimization, the dynamic bodies have a flag which specifies whether they have any constraints at all. That works
	//       because a constraint has at least one dynamic body and an interaction is tracked by both objects.

	if(rbActor0.isDynamicRigid())
		return filterJointedBodies2(rbActor0, rbActor1);
	return filterJointedBodies2(rbActor1, rbActor0);
}

static PX_FORCE_INLINE bool hasForceNotifEnabled(const BodySim* bs, PxRigidBodyFlag::Enum flag)
{
	if(!bs)
		return false;

	const PxsRigidCore& core = bs->getBodyCore().getCore();
	return core.mFlags.isSet(flag);
}

static PX_FORCE_INLINE bool validateSuppress(const BodySim* b0, const BodySim* b1, PxRigidBodyFlag::Enum flag)
{
	if(hasForceNotifEnabled(b0, flag))
		return false;

	if(hasForceNotifEnabled(b1, flag))
		return false;

	return true;
}

static PX_FORCE_INLINE bool filterKinematics(const BodySim* b0, const BodySim* b1, bool kine0, bool kine1,
	const PxPairFilteringMode::Enum kineKineFilteringMode, const PxPairFilteringMode::Enum staticKineFilteringMode)
{
	const bool kinematicPair = kine0 | kine1;
	if(kinematicPair)
	{
		if(staticKineFilteringMode != PxPairFilteringMode::eKEEP)
		{
			if(!b0 || !b1)
				return validateSuppress(b0, b1, PxRigidBodyFlag::eFORCE_STATIC_KINE_NOTIFICATIONS);
		}

		if(kineKineFilteringMode != PxPairFilteringMode::eKEEP)
		{
			if(kine0 && kine1)
				return validateSuppress(b0, b1, PxRigidBodyFlag::eFORCE_KINE_KINE_NOTIFICATIONS);
		}
	}
	return false;
}

static const BodySim* isKinematic(const ActorSim& actorSim, bool& kine)
{
	if(actorSim.isDynamicRigid())
	{
		const BodySim* bs = static_cast<const BodySim*>(&actorSim);
		kine = bs->isKinematic();
		return bs;
	}
	else
	{
		kine = false;
		return NULL;
	}
}

static PxFilterInfo filterRbCollisionPair(const FilteringContext& context, const ShapeSimBase& s0, const ShapeSimBase& s1, PxU32 filterPairIndex, bool& isTriggerPair, bool runCallbacks)
{
	const ActorSim& b0 = s0.getActor();
	const ActorSim& b1 = s1.getActor();

	const PxU32 trigger0 = s0.getFlags() & PxShapeFlag::eTRIGGER_SHAPE;
	const PxU32 trigger1 = s1.getFlags() & PxShapeFlag::eTRIGGER_SHAPE;
	isTriggerPair = (trigger0 | trigger1)!=0;

	bool isNonRigid = false;

	if(isTriggerPair)
	{
		if(trigger0 && trigger1)	// trigger-trigger pairs are not supported
			return filterOutRbCollisionPair(context.mFilterPairManager, filterPairIndex, PxFilterFlag::eKILL);
	}
	else
	{
		bool kine0, kine1;
		const BodySim* bs0 = isKinematic(b0, kine0);
		const BodySim* bs1 = isKinematic(b1, kine1);

		isNonRigid = b0.isNonRigid() || b1.isNonRigid();

		if(!isNonRigid && filterKinematics(bs0, bs1, kine0, kine1, context.mKineKineFilteringMode, context.mStaticKineFilteringMode))
			return filterOutRbCollisionPair(context.mFilterPairManager, filterPairIndex, PxFilterFlag::eSUPPRESS);

		if(filterJointedBodies(b0, b1))
			return filterOutRbCollisionPair(context.mFilterPairManager, filterPairIndex, PxFilterFlag::eSUPPRESS);

		if((b0.getActorType() == PxActorType::eARTICULATION_LINK) && (b1.getActorType() == PxActorType::eARTICULATION_LINK))
		{
			if(filterArticulationLinks(b0, b1))
				return filterOutRbCollisionPair(context.mFilterPairManager, filterPairIndex, PxFilterFlag::eKILL);
		}
	}
	return filterRbCollisionPairSecondStage(context, s0, s1, b0, b1, filterPairIndex, runCallbacks, isNonRigid);
}

// PT: indexed by PxActorType
static const PxU32 gTypeData[] = {
	PxFilterObjectType::eRIGID_STATIC<<1,
	(PxFilterObjectType::eRIGID_DYNAMIC<<1)|1,
	(PxFilterObjectType::eARTICULATION<<1)|1,
	(PxFilterObjectType::eSOFTBODY<<1)|1,
	(PxFilterObjectType::eFEMCLOTH << 1) | 1,
	(PxFilterObjectType::ePARTICLESYSTEM<<1)|1, //PBD
	(PxFilterObjectType::ePARTICLESYSTEM<<1)|1, //FLIP
	(PxFilterObjectType::ePARTICLESYSTEM<<1)|1, //MPM
	(PxFilterObjectType::ePARTICLESYSTEM<<1)|1, //Custom
	(PxFilterObjectType::eHAIRSYSTEM<<1)|1,
};

static PX_FORCE_INLINE bool isParticleSystem(const PxActorType::Enum actorType)
{
	return actorType == PxActorType::ePBD_PARTICLESYSTEM || actorType == PxActorType::eFLIP_PARTICLESYSTEM
		|| actorType == PxActorType::eMPM_PARTICLESYSTEM || actorType == PxActorType::eCUSTOM_PARTICLESYSTEM;
}

// PT: version specialized for ShapeSim/ShapeSim (no triggers)
static PX_FORCE_INLINE PxFilterInfo filterRbCollisionPair(const FilteringContext& context, const ShapeSimBase& s0, const ShapeSimBase& s1)
{
	const ActorSim& rbActor0 = s0.getActor();
	const PxActorType::Enum actorType0 = rbActor0.getActorType();
	const PxU32 typeData0 = gTypeData[actorType0];
	PxFilterObjectAttributes filterAttr0 = typeData0>>1;
	bool kine0 = false;
	bool isNonRigid = false;
	const BodySim* bs0 = NULL;
	if (rbActor0.isDynamicRigid())
	{
		bs0 = static_cast<const BodySim*>(&rbActor0);
		kine0 = bs0->isKinematic();
		filterAttr0 |= kine0 ? PxFilterObjectFlag::eKINEMATIC : 0;
	}
	else if (rbActor0.isNonRigid())
	{
		isNonRigid = true;
	}

	const ActorSim& rbActor1 = s1.getActor();
	const PxActorType::Enum actorType1 = rbActor1.getActorType();
	const PxU32 typeData1 = gTypeData[actorType1];
	PxFilterObjectAttributes filterAttr1 = typeData1 >> 1;
	bool kine1 = false;
	const BodySim* bs1 = NULL;
	if (rbActor1.isDynamicRigid())
	{
		bs1 = static_cast<const BodySim*>(&rbActor1);
		kine1 = bs1->isKinematic();
		filterAttr0 |= kine1 ? PxFilterObjectFlag::eKINEMATIC : 0;
	}
	else if (rbActor1.isNonRigid())
	{
		isNonRigid = true;
	}

	PX_ASSERT(!(s0.getFlags() & PxShapeFlag::eTRIGGER_SHAPE));
	PX_ASSERT(!(s1.getFlags() & PxShapeFlag::eTRIGGER_SHAPE));

	if (!isNonRigid && filterKinematics(bs0, bs1, kine0, kine1, context.mKineKineFilteringMode, context.mStaticKineFilteringMode))
		return PxFilterInfo(PxFilterFlag::eSUPPRESS);

	if(filterJointedBodies(rbActor0, rbActor1))
		return PxFilterInfo(PxFilterFlag::eSUPPRESS);

	if(isParticleSystem(actorType0) && isParticleSystem(actorType1))
		return PxFilterInfo(PxFilterFlag::eKILL);

	if(actorType0 == PxActorType::eHAIRSYSTEM && actorType1 == PxActorType::eHAIRSYSTEM)
		return PxFilterInfo(PxFilterFlag::eKILL);


	if ((actorType0 == PxActorType::eARTICULATION_LINK) ^ (actorType1 == PxActorType::eARTICULATION_LINK))
	{
		if(actorType0 == PxActorType::eARTICULATION_LINK)
		{
			const BodySim& b0 = static_cast<const BodySim&>(rbActor0);
			const PxU8 kinematicLink = b0.getLowLevelBody().mCore->kinematicLink;
			const bool isStaticOrKinematic = (actorType1 == PxActorType::eRIGID_STATIC) || kine1;
			if (kinematicLink && isStaticOrKinematic)
				return PxFilterInfo(PxFilterFlag::eSUPPRESS);
		}
		
		if (actorType1 == PxActorType::eARTICULATION_LINK)
		{
			const BodySim& b1 = static_cast<const BodySim&>(rbActor1);
			const PxU8 kinematicLink = b1.getLowLevelBody().mCore->kinematicLink;
			const bool isStaticOrKinematic = (actorType0 == PxActorType::eRIGID_STATIC) || kine0;
			if (kinematicLink && isStaticOrKinematic)
				return PxFilterInfo(PxFilterFlag::eSUPPRESS);
		}	
	}

	if((actorType0 == PxActorType::eARTICULATION_LINK) && (actorType1 == PxActorType::eARTICULATION_LINK))
	{
		const BodySim& b0 = static_cast<const BodySim&>(rbActor0);
		const BodySim& b1 = static_cast<const BodySim&>(rbActor1);
		const PxU8 kinematicLink0 = b0.getLowLevelBody().mCore->kinematicLink;
		const PxU8 kinematicLink1 = b1.getLowLevelBody().mCore->kinematicLink;
		if (kinematicLink0 && kinematicLink1)
			return PxFilterInfo(PxFilterFlag::eSUPPRESS);

		if(filterArticulationLinks(rbActor0, rbActor1))
			return PxFilterInfo(PxFilterFlag::eKILL);
	}

	return filterRbCollisionPairSecondStage(context, s0, s1, kine0, kine1, filterAttr0, filterAttr1, isNonRigid);
}

void NPhaseCore::runOverlapFilters(	PxU32 nbToProcess, const Bp::AABBOverlap* PX_RESTRICT pairs, PxFilterInfo* PX_RESTRICT filterInfo,
									PxU32& nbToKeep_, PxU32& nbToSuppress_, PxU32& nbToCallback_, PxU32* PX_RESTRICT keepMap, PxU32* PX_RESTRICT callbackMap)
{
	PxU32 nbToKeep = 0;
	PxU32 nbToSuppress = 0;
	PxU32 nbToCallback = 0;

	const FilteringContext context(mOwnerScene, mFilterPairManager);

	for(PxU32 i=0; i<nbToProcess; i++)
	{
		const Bp::AABBOverlap& pair = pairs[i];

		ElementSim* e0 = reinterpret_cast<ElementSim*>(pair.mUserData0);
		ElementSim* e1 = reinterpret_cast<ElementSim*>(pair.mUserData1);

		PX_ASSERT(e0);
		PX_ASSERT(e1);
		PX_ASSERT(!findInteraction(e0, e1));

		ShapeSimBase* s0 = static_cast<ShapeSimBase*>(e0);
		ShapeSimBase* s1 = static_cast<ShapeSimBase*>(e1);
		PX_ASSERT(&s0->getActor() != &s1->getActor());	// No actor internal interactions

		filterInfo[i] = filterRbCollisionPair(context, *s0, *s1);

		const PxFilterFlags filterFlags = filterInfo[i].filterFlags;

		if(!(filterFlags & PxFilterFlag::eKILL))
		{
			if(filterFlags & PxFilterFlag::eCALLBACK)
			{
				nbToCallback++;
				callbackMap[i / 32] |= (1 << (i & 31));
			}
			else
			{
				if(!(filterFlags & PxFilterFlag::eSUPPRESS))
					nbToKeep++;
				else
					nbToSuppress++;
				keepMap[i / 32] |= (1 << (i & 31));
			}
		}
	}

	nbToKeep_ = nbToKeep;
	nbToSuppress_ = nbToSuppress;
	nbToCallback_ = nbToCallback;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

NPhaseCore::NPhaseCore(Scene& scene, const PxSceneDesc& sceneDesc) :
	mOwnerScene									(scene),
	mContactReportActorPairSet					("contactReportPairSet"),
	mPersistentContactEventPairList				("persistentContactEventPairs"),
	mNextFramePersistentContactEventPairIndex	(0),
	mForceThresholdContactEventPairList			("forceThresholdContactEventPairs"),
	mContactReportBuffer						(sceneDesc.contactReportStreamBufferSize, (sceneDesc.flags & PxSceneFlag::eDISABLE_CONTACT_REPORT_BUFFER_RESIZE)),
	mActorPairPool								("actorPairPool"),
	mActorPairReportPool						("actorPairReportPool"),
	mShapeInteractionPool						(PxAllocatorTraits<ShapeInteraction>::Type("shapeInteractionPool"), 4096),
	mTriggerInteractionPool						("triggerInteractionPool"),
	mActorPairContactReportDataPool				("actorPairContactReportPool"),
	mInteractionMarkerPool						("interactionMarkerPool")
	,mMergeProcessedTriggerInteractions			(scene.getContextId(), this, "ScNPhaseCore.mergeProcessedTriggerInteractions")
	,mTmpTriggerProcessingBlock					(NULL)
	,mTriggerPairsToDeactivateCount				(0)
{
	mFilterPairManager = PX_NEW(FilterPairManager);
}

NPhaseCore::~NPhaseCore()
{
	// Clear pending actor pairs (waiting on contact report callback)
	clearContactReportActorPairs(false);
	PX_DELETE(mFilterPairManager);
}

PxU32 NPhaseCore::getDefaultContactReportStreamBufferSize() const
{
	return mContactReportBuffer.getDefaultBufferSize();
}

ElementSimInteraction* NPhaseCore::findInteraction(ElementSim* _element0, ElementSim* _element1)
{
	const PxHashMap<ElementSimKey, ElementSimInteraction*>::Entry* pair = mElementSimMap.find(ElementSimKey(_element0, _element1));
	return pair ? pair->second : NULL;
}

void NPhaseCore::onTriggerOverlapCreated(const Bp::AABBOverlap* PX_RESTRICT pairs, PxU32 pairCount)
{
	for(PxU32 i=0; i<pairCount; i++)
	{
		ElementSim* volume0 = reinterpret_cast<ElementSim*>(pairs[i].mUserData0);
		ElementSim* volume1 = reinterpret_cast<ElementSim*>(pairs[i].mUserData1);
		PX_ASSERT(!findInteraction(volume0, volume1));

		ShapeSimBase* shapeHi = static_cast<ShapeSimBase*>(volume1);
		ShapeSimBase* shapeLo = static_cast<ShapeSimBase*>(volume0);

		// No actor internal interactions
		PX_ASSERT(&shapeHi->getActor() != &shapeLo->getActor());

		// PT: this case is only for triggers these days
		PX_ASSERT((shapeLo->getFlags() & PxShapeFlag::eTRIGGER_SHAPE) || (shapeHi->getFlags() & PxShapeFlag::eTRIGGER_SHAPE));

		createTriggerElementInteraction(*shapeHi, *shapeLo);
	}
}

void NPhaseCore::reserveInteraction(PxU32 nbNewInteractions)
{
	if ((mElementSimMap.size() + nbNewInteractions) > mElementSimMap.capacity())
	{
		PX_PROFILE_ZONE("Reserve", 0);
		PxU32 newSize = PxMax(mElementSimMap.size() + nbNewInteractions, mElementSimMap.capacity());
		mElementSimMap.reserve(newSize);
	}
}

void NPhaseCore::registerInteraction(ElementSimInteraction* interaction)
{
	mElementSimMap.insert(ElementSimKey(&interaction->getElement0(), &interaction->getElement1()), interaction);
}

void NPhaseCore::unregisterInteraction(ElementSimInteraction* interaction)
{
	mElementSimMap.erase(ElementSimKey(&interaction->getElement0(), &interaction->getElement1()));
}

ElementSimInteraction* NPhaseCore::onOverlapRemovedStage1(ElementSim* volume0, ElementSim* volume1)
{
	return findInteraction(volume0, volume1);
}

void NPhaseCore::onOverlapRemoved(ElementSim* volume0, ElementSim* volume1, const PxU32 ccdPass, void* elemSim, PxsContactManagerOutputIterator& outputs)
{
	ElementSim* elementHi = volume1;
	ElementSim* elementLo = volume0;
	// No actor internal interactions
	PX_ASSERT(&elementHi->getActor() != &elementLo->getActor());

	// PT: TODO: get rid of 'findInteraction', cf US10491
	ElementSimInteraction* interaction = elemSim ? reinterpret_cast<ElementSimInteraction*>(elemSim) : findInteraction(elementHi, elementLo);
	// MS: The check below is necessary since at the moment LowLevel broadphase still tracks
	//     killed pairs and hence reports lost overlaps
	if(interaction)
	{
		PxU32 flags = PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH);
		PX_ASSERT(interaction->isElementInteraction());
		releaseElementPair(static_cast<ElementSimInteraction*>(interaction), flags, NULL, ccdPass, true, outputs);
	}
}

// MS: TODO: optimize this for the actor release case?
void NPhaseCore::onVolumeRemoved(ElementSim* volume, PxU32 flags, PxsContactManagerOutputIterator& outputs)
{
	const PxU32 ccdPass = 0;

	flags |= PairReleaseFlag::eRUN_LOST_TOUCH_LOGIC;

	// Release interactions
	// IMPORTANT: Iterate from the back of the list to the front as we release interactions which
	//            triggers a replace with last
	ElementSim::ElementInteractionReverseIterator iter = volume->getElemInteractionsReverse();
	ElementSimInteraction* interaction = iter.getNext();
	while(interaction)
	{
		PX_ASSERT(	(interaction->getType() == InteractionType::eMARKER) ||
					(interaction->getType() == InteractionType::eOVERLAP) ||
					(interaction->getType() == InteractionType::eTRIGGER) );

		releaseElementPair(interaction, flags, volume, ccdPass, true, outputs);

		interaction = iter.getNext();
	}
}

ElementSimInteraction* NPhaseCore::createRbElementInteraction(const PxFilterInfo& finfo, ShapeSimBase& s0, ShapeSimBase& s1, PxsContactManager* contactManager, ShapeInteraction* shapeInteraction,
	ElementInteractionMarker* interactionMarker, bool isTriggerPair)
{
	ElementSimInteraction* pair = NULL;

	if((finfo.filterFlags & PxFilterFlag::eSUPPRESS) == false)
	{
		if(!isTriggerPair)
		{
			PX_ASSERT(contactManager);
			PX_ASSERT(shapeInteraction);
			pair = createShapeInteraction(s0, s1, finfo.pairFlags, contactManager, shapeInteraction);
		}
		else
		{
			pair = createTriggerInteraction(s0, s1, finfo.pairFlags);
		}
	}
	else
		pair = createElementInteractionMarker(s0, s1, interactionMarker);

	if(finfo.filterPairIndex != INVALID_FILTER_PAIR_INDEX)
	{
		// Mark the pair as a filter callback pair
		pair->raiseInteractionFlag(InteractionFlag::eIS_FILTER_PAIR);

		// Filter callback pair: Set the link to the interaction
		mFilterPairManager->setPair(finfo.filterPairIndex, pair);
		pair->setFilterPairIndex(finfo.filterPairIndex);
	}

	return pair;
}

ElementSimInteraction* NPhaseCore::createTriggerElementInteraction(ShapeSimBase& s0, ShapeSimBase& s1)
{
	PX_ASSERT((s0.getFlags() & PxShapeFlag::eTRIGGER_SHAPE) || (s1.getFlags() & PxShapeFlag::eTRIGGER_SHAPE));

	const FilteringContext context(mOwnerScene, mFilterPairManager);

	bool isTriggerPair;
	const PxFilterInfo finfo = filterRbCollisionPair(context, s0, s1, INVALID_FILTER_PAIR_INDEX, isTriggerPair, false);
	PX_ASSERT(isTriggerPair);

	if(finfo.filterFlags & PxFilterFlag::eKILL)
	{
		PX_ASSERT(finfo.filterPairIndex == INVALID_FILTER_PAIR_INDEX);  // No filter callback pair info for killed pairs
		return NULL;
	}

	return createRbElementInteraction(finfo, s0, s1, NULL, NULL, NULL, isTriggerPair);
}

void NPhaseCore::managerNewTouch(ShapeInteraction& interaction)
{
	//(1) if the pair hasn't already been assigned, look it up!

	ActorPair* actorPair = interaction.getActorPair();

	if(!actorPair)
	{
		ShapeSim& s0 = static_cast<ShapeSim&>(interaction.getElement0());
		ShapeSim& s1 = static_cast<ShapeSim&>(interaction.getElement1());
		actorPair = findActorPair(&s0, &s1, interaction.isReportPair());
		actorPair->incRefCount(); //It's being referenced by a new pair...
		interaction.setActorPair(*actorPair);
	}	
}

ShapeInteraction* NPhaseCore::createShapeInteraction(ShapeSimBase& s0, ShapeSimBase& s1, PxPairFlags pairFlags, PxsContactManager* contactManager, ShapeInteraction* shapeInteraction)
{
	ShapeSimBase* _s0 = &s0;
	ShapeSimBase* _s1 = &s1;

	/*
	This tries to ensure that if one of the bodies is static or kinematic, it will be body B
	There is a further optimization to force all pairs that share the same bodies to have
	the same body ordering.  This reduces the number of required partitions in the parallel solver.
	Sorting rules are:
	If bodyA is static, swap
	If bodyA is rigidDynamic and bodyB is articulation, swap
	If bodyA is in an earlier BP group than bodyB, swap
	*/
	{
		ActorSim& rs0 = s0.getActor();
		ActorSim& rs1 = s1.getActor();

		const PxActorType::Enum actorType0 = rs0.getActorType();
		const PxActorType::Enum actorType1 = rs1.getActorType();

		bool articulationLinkSwap = false;
		if (actorType0 == PxActorType::eARTICULATION_LINK && actorType1 == PxActorType::eARTICULATION_LINK)
		{
			BodySim& bodySim0 = static_cast<BodySim&>(rs0);

			const PxU8 kinematicLink0 = bodySim0.getLowLevelBody().mCore->kinematicLink;

			if (kinematicLink0)
			{
				articulationLinkSwap = true;
			}
		}

		bool actorAKinematic = actorType0 == PxActorType::eRIGID_DYNAMIC && static_cast<BodySim&>(rs0).isKinematic();
		bool actorBKinematic = actorType1 == PxActorType::eRIGID_DYNAMIC && static_cast<BodySim&>(rs1).isKinematic();


		if(	 actorType0 == PxActorType::eRIGID_STATIC
			 || (actorType1 == PxActorType::eRIGID_DYNAMIC && actorType0 == PxActorType::eARTICULATION_LINK)
			 || articulationLinkSwap
			 || (isParticleSystem(actorType0) && actorType1 != PxActorType::eRIGID_STATIC)
			 || ((actorType0 == PxActorType::eRIGID_DYNAMIC && actorType1 == PxActorType::eRIGID_DYNAMIC) && actorAKinematic)
			 || (actorType0 == actorType1 && rs0.getActorID() < rs1.getActorID() && !actorBKinematic))
			PxSwap(_s0, _s1);
	}

	ShapeInteraction* si = shapeInteraction ? shapeInteraction : mShapeInteractionPool.allocate();
	PX_PLACEMENT_NEW(si, ShapeInteraction)(*_s0, *_s1, pairFlags, contactManager);

	PX_ASSERT(si->mReportPairIndex == INVALID_REPORT_PAIR_ID);

	return si;
}

TriggerInteraction* NPhaseCore::createTriggerInteraction(ShapeSimBase& s0, ShapeSimBase& s1, PxPairFlags triggerFlags)
{
	ShapeSimBase* triggerShape;
	ShapeSimBase* otherShape;

	if(s1.getFlags() & PxShapeFlag::eTRIGGER_SHAPE)
	{
		triggerShape = &s1;
		otherShape = &s0;
	}
	else
	{
		triggerShape = &s0;
		otherShape = &s1;
	}
	TriggerInteraction* pair = mTriggerInteractionPool.construct(*triggerShape, *otherShape);
	pair->setTriggerFlags(triggerFlags);
	return pair;
}

ElementInteractionMarker* NPhaseCore::createElementInteractionMarker(ElementSim& e0, ElementSim& e1, ElementInteractionMarker* interactionMarker)
{
	ElementInteractionMarker* pair = interactionMarker ? interactionMarker : mInteractionMarkerPool.allocate();
	PX_PLACEMENT_NEW(pair, ElementInteractionMarker)(e0, e1, interactionMarker != NULL);
	return pair;
}

ElementSimInteraction* NPhaseCore::refilterInteraction(ElementSimInteraction* pair, const PxFilterInfo* filterInfo, bool removeFromDirtyList, PxsContactManagerOutputIterator& outputs)
{
	const InteractionType::Enum oldType = pair->getType();

	switch (oldType)
	{
		case InteractionType::eTRIGGER:
		case InteractionType::eMARKER:
		case InteractionType::eOVERLAP:
			{
				ShapeSimBase& s0 = static_cast<ShapeSimBase&>(pair->getElement0());
				ShapeSimBase& s1 = static_cast<ShapeSimBase&>(pair->getElement1());

				PxFilterInfo finfo;
				if(filterInfo)
				{
					// The filter changes are provided by an outside source (the user filter callback)

					finfo = *filterInfo;
					PX_ASSERT(finfo.filterPairIndex!=INVALID_FILTER_PAIR_INDEX);

					if((finfo.filterFlags & PxFilterFlag::eKILL) &&
						((finfo.filterFlags & PxFilterFlag::eNOTIFY) == PxFilterFlag::eNOTIFY) )
					{
						callPairLost(mOwnerScene, pair->getElement0(), pair->getElement1(), finfo.filterPairIndex, false);
						mFilterPairManager->releaseIndex(finfo.filterPairIndex);
						finfo.filterPairIndex = INVALID_FILTER_PAIR_INDEX;
					}

					ActorSim& bs0 = s0.getActor();
					ActorSim& bs1 = s1.getActor();
					finfo.pairFlags = checkRbPairFlags(s0, s1, bs0, bs1, finfo.pairFlags, finfo.filterFlags, s0.getActor().isNonRigid() || s1.getActor().isNonRigid());
				}
				else
				{
					PxU32 filterPairIndex =  INVALID_FILTER_PAIR_INDEX;
					if(pair->readInteractionFlag(InteractionFlag::eIS_FILTER_PAIR))
					{
						filterPairIndex = mFilterPairManager->findIndex(pair);
						PX_ASSERT(filterPairIndex!=INVALID_FILTER_PAIR_INDEX);

						callPairLost(mOwnerScene, pair->getElement0(), pair->getElement1(), filterPairIndex, false);
					}

					const FilteringContext context(mOwnerScene, mFilterPairManager);

					bool isTriggerPair;
					finfo = filterRbCollisionPair(context, s0, s1, filterPairIndex, isTriggerPair, true);
					PX_UNUSED(isTriggerPair);
				}

				if(pair->readInteractionFlag(InteractionFlag::eIS_FILTER_PAIR) &&
					((finfo.filterFlags & PxFilterFlag::eNOTIFY) != PxFilterFlag::eNOTIFY) )
				{
					// The pair was a filter callback pair but not any longer
					pair->clearInteractionFlag(InteractionFlag::eIS_FILTER_PAIR);

					if(finfo.filterPairIndex!=INVALID_FILTER_PAIR_INDEX)
					{
						mFilterPairManager->releaseIndex(finfo.filterPairIndex);
						finfo.filterPairIndex = INVALID_FILTER_PAIR_INDEX;
					}
				}

				struct Local
				{
					static InteractionType::Enum getRbElementInteractionType(const ShapeSimBase* primitive0, const ShapeSimBase* primitive1, PxFilterFlags filterFlag)
					{
						if(filterFlag & PxFilterFlag::eKILL)
							return InteractionType::eINVALID;

						if(filterFlag & PxFilterFlag::eSUPPRESS)
							return InteractionType::eMARKER;

						if(primitive0->getFlags() & PxShapeFlag::eTRIGGER_SHAPE
						|| primitive1->getFlags() & PxShapeFlag::eTRIGGER_SHAPE)
							return InteractionType::eTRIGGER;

						PX_ASSERT(	(primitive0->getGeometryType() != PxGeometryType::eTRIANGLEMESH) ||
									(primitive1->getGeometryType() != PxGeometryType::eTRIANGLEMESH));

						return InteractionType::eOVERLAP;
					}
				};

				const InteractionType::Enum newType = Local::getRbElementInteractionType(&s0, &s1, finfo.filterFlags);
				if(pair->getType() != newType)  //Only convert interaction type if the type has changed
				{
					return convert(pair, newType, finfo, removeFromDirtyList, outputs);
				}
				else
				{
					//The pair flags might have changed, we need to forward the new ones
					if(oldType == InteractionType::eOVERLAP)
					{
						ShapeInteraction* si = static_cast<ShapeInteraction*>(pair);

						const PxU32 newPairFlags = finfo.pairFlags;
						const PxU32 oldPairFlags = si->getPairFlags();
						PX_ASSERT((newPairFlags & ShapeInteraction::PAIR_FLAGS_MASK) == newPairFlags);
						PX_ASSERT((oldPairFlags & ShapeInteraction::PAIR_FLAGS_MASK) == oldPairFlags);

						if(newPairFlags != oldPairFlags)
						{
							if(!(oldPairFlags & ShapeInteraction::CONTACT_REPORT_EVENTS) && (newPairFlags & ShapeInteraction::CONTACT_REPORT_EVENTS) && (si->getActorPair() == NULL || !si->getActorPair()->isReportPair()))
							{
								// for this actor pair there was no shape pair that requested contact reports but now there is one
								// -> all the existing shape pairs need to get re-adjusted to point to an ActorPairReport instance instead.
								ActorPair* actorPair = findActorPair(&s0, &s1, PxIntTrue);
								if (si->getActorPair() == NULL)
								{
									actorPair->incRefCount();
									si->setActorPair(*actorPair);
								}
							}

							if(si->readFlag(ShapeInteraction::IN_PERSISTENT_EVENT_LIST) && (!(newPairFlags & PxPairFlag::eNOTIFY_TOUCH_PERSISTS)))
							{
								// the new report pair flags don't require persistent checks anymore -> remove from persistent list
								// Note: The pair might get added to the force threshold list later
								if(si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST))
									removeFromPersistentContactEventPairs(si);
								else
									si->clearFlag(ShapeInteraction::WAS_IN_PERSISTENT_EVENT_LIST);
							}

							if(newPairFlags & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS)
							{
								PX_ASSERT((si->mReportPairIndex == INVALID_REPORT_PAIR_ID) || (!si->readFlag(ShapeInteraction::WAS_IN_PERSISTENT_EVENT_LIST)));

								if(si->mReportPairIndex == INVALID_REPORT_PAIR_ID && si->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
								{
									PX_ASSERT(!si->readFlag(ShapeInteraction::WAS_IN_PERSISTENT_EVENT_LIST));  // sanity check: an active pair should never have this flag set

									if(si->hasTouch())
										addToForceThresholdContactEventPairs(si);
								}
							}
							else if((oldPairFlags & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS))
							{
								// no force threshold events needed any longer -> clear flags
								si->clearFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_FLAGS);

								if(si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST))
									removeFromForceThresholdContactEventPairs(si);
							}
						}
						si->setPairFlags(finfo.pairFlags);
					}
					else if(oldType == InteractionType::eTRIGGER)
						static_cast<TriggerInteraction*>(pair)->setTriggerFlags(finfo.pairFlags);

					return pair;
				}
			}
			case InteractionType::eCONSTRAINTSHADER:
			case InteractionType::eARTICULATION:
			case InteractionType::eTRACKED_IN_SCENE_COUNT:
			case InteractionType::eINVALID:
			PX_ASSERT(0);
			break;
	}
	return NULL;
}

ActorPair* NPhaseCore::findActorPair(ShapeSimBase* s0, ShapeSimBase* s1, PxIntBool isReportPair)
{
	PX_ASSERT(!(s0->getFlags() & PxShapeFlag::eTRIGGER_SHAPE)
		   && !(s1->getFlags() & PxShapeFlag::eTRIGGER_SHAPE));
	// This method is only for the case where a ShapeInteraction is going to be created.
	// Else we might create an ActorPair that does not get referenced and causes a mem leak.
	
	BodyPairKey key;

	ActorSim* aLess = &s0->getActor();
	ActorSim* aMore = &s1->getActor();

	if(aLess->getActorID() > aMore->getActorID())
		PxSwap(aLess, aMore);

	key.mSim0 = aLess->getActorID();
	key.mSim1 = aMore->getActorID();

	ActorPair*& actorPair = mActorPairMap[key];
	
	if(actorPair == NULL)
	{
		if(!isReportPair)
			actorPair = mActorPairPool.construct();
		else
			actorPair = mActorPairReportPool.construct(s0->getActor(), s1->getActor());
	}

	PxIntBool actorPairHasReports = actorPair->isReportPair();

	if(!isReportPair || actorPairHasReports)
		return actorPair;
	else
	{
		PxU32 size = aLess->getActorInteractionCount();
		Interaction** interactions = aLess->getActorInteractions();
		
		ActorPairReport* actorPairReport = mActorPairReportPool.construct(s0->getActor(), s1->getActor());
		actorPairReport->convert(*actorPair);

		while(size--)
		{
			Interaction* interaction = *interactions++;
			if((&interaction->getActorSim0() == aMore) || (&interaction->getActorSim1() == aMore))
			{
				PX_ASSERT(((&interaction->getActorSim0() == aLess) || (&interaction->getActorSim1() == aLess)));

				if(interaction->getType() == InteractionType::eOVERLAP)
				{
					ShapeInteraction* si = static_cast<ShapeInteraction*>(interaction);
					if(si->getActorPair() != NULL)
						si->setActorPair(*actorPairReport);
				}
			}
		}
		actorPair = actorPairReport;
	}
	return actorPair;
}

PX_FORCE_INLINE void NPhaseCore::destroyActorPairReport(ActorPairReport& aPair)
{
	PX_ASSERT(aPair.isReportPair());
	
	aPair.releaseContactReportData(*this);
	mActorPairReportPool.destroy(&aPair);
}

ElementSimInteraction* NPhaseCore::convert(ElementSimInteraction* pair, InteractionType::Enum newType, PxFilterInfo& filterInfo, bool removeFromDirtyList,
	PxsContactManagerOutputIterator& outputs)
{
	PX_ASSERT(newType != pair->getType());

	ElementSim& elementA = pair->getElement0();
	ElementSim& elementB = pair->getElement1();

	// Wake up the actors of the pair
	if((pair->getActorSim0().getActorType() == PxActorType::eRIGID_DYNAMIC) && !(static_cast<BodySim&>(pair->getActorSim0()).isActive()))
		pair->getActorSim0().internalWakeUp();
	if((pair->getActorSim1().getActorType() == PxActorType::eRIGID_DYNAMIC) && !(static_cast<BodySim&>(pair->getActorSim1()).isActive()))
		pair->getActorSim1().internalWakeUp();

	// Since the FilterPair struct might have been re-used in the newly created interaction, we need to clear
	// the filter pair marker of the old interaction to avoid that the FilterPair gets deleted by the releaseElementPair()
	// call that follows.
	pair->clearInteractionFlag(InteractionFlag::eIS_FILTER_PAIR);

	// PT: we need to unregister the old interaction *before* creating the new one, because Sc::NPhaseCore::registerInteraction will use
	// ElementSim pointers which are the same for both. Since "releaseElementPair" will call the unregister function from
	// the element's dtor, we don't need to do it explicitly here. Just release the object.
	releaseElementPair(pair, PairReleaseFlag::eWAKE_ON_LOST_TOUCH | PairReleaseFlag::eRUN_LOST_TOUCH_LOGIC, NULL, 0, removeFromDirtyList, outputs);

	ElementSimInteraction* result = NULL;
	switch(newType)
	{
		case InteractionType::eINVALID:
			// This means the pair should get killed
			break;
		case InteractionType::eMARKER:
			{
			result = createElementInteractionMarker(elementA, elementB, NULL);
			break;
			}
		case InteractionType::eOVERLAP:
			{
			result = createShapeInteraction(static_cast<ShapeSim&>(elementA), static_cast<ShapeSim&>(elementB), filterInfo.pairFlags, NULL, NULL);
			break;
			}
		case InteractionType::eTRIGGER:
			{
			result = createTriggerInteraction(static_cast<ShapeSim&>(elementA), static_cast<ShapeSim&>(elementB), filterInfo.pairFlags);
			break;
			}
		case InteractionType::eCONSTRAINTSHADER:
		case InteractionType::eARTICULATION:
		case InteractionType::eTRACKED_IN_SCENE_COUNT:
			PX_ASSERT(0);
			break;
	};

	if(filterInfo.filterPairIndex != INVALID_FILTER_PAIR_INDEX)
	{
		PX_ASSERT(result);
		// If a filter callback pair is going to get killed, then the FilterPair struct should already have
		// been deleted.

		// Mark the new interaction as a filter callback pair
		result->raiseInteractionFlag(InteractionFlag::eIS_FILTER_PAIR);

		mFilterPairManager->setPair(filterInfo.filterPairIndex, result);
		result->setFilterPairIndex(filterInfo.filterPairIndex);
	}
	return result;
}

namespace physx
{
namespace Sc
{
static bool findTriggerContacts(TriggerInteraction* tri, bool toBeDeleted, bool volumeRemoved,
								PxTriggerPair& triggerPair, TriggerPairExtraData& triggerPairExtra,
								SimStats::TriggerPairCountsNonVolatile& triggerPairStats,
								const PxsTransformCache& transformCache)
{
	ShapeSimBase& s0 = tri->getTriggerShape();
	ShapeSimBase& s1 = tri->getOtherShape();

	const PxPairFlags pairFlags = tri->getTriggerFlags();
	PxPairFlags pairEvent;

	bool overlap;
	PxU8 testForRemovedShapes = 0;
	if(toBeDeleted)
	{
		// The trigger interaction is to lie down in its tomb, hence we know that the overlap is gone.
		// What remains is to check whether the interaction was deleted because of a shape removal in
		// which case we need to later check for removed shapes.

		overlap = false;

		if(volumeRemoved)
		{
			// Note: only the first removed volume can be detected when the trigger interaction is deleted but at a later point the second volume might get removed too.
			testForRemovedShapes = TriggerPairFlag::eTEST_FOR_REMOVED_SHAPES;
		}
	}
	else
	{
#if PX_ENABLE_SIM_STATS
		PX_ASSERT(s0.getGeometryType() < PxGeometryType::eCONVEXMESH+1);  // The first has to be the trigger shape
		triggerPairStats[s0.getGeometryType()][s1.getGeometryType()]++;
#else
		PX_UNUSED(triggerPairStats);
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

		ShapeSimBase* primitive0 = &s0;
		ShapeSimBase* primitive1 = &s1;

		PX_ASSERT(primitive0->getFlags() & PxShapeFlag::eTRIGGER_SHAPE
			   || primitive1->getFlags() & PxShapeFlag::eTRIGGER_SHAPE);

		// Reorder them if needed
		if(primitive0->getGeometryType() > primitive1->getGeometryType())
			PxSwap(primitive0, primitive1);

		const Gu::GeomOverlapFunc overlapFunc =
			Gu::getOverlapFuncTable()[primitive0->getGeometryType()][primitive1->getGeometryType()];

		const PxU32 elementID0 = primitive0->getElementID();
		const PxU32 elementID1 = primitive1->getElementID();

		const PxTransform& globalPose0 = transformCache.getTransformCache(elementID0).transform;

		const PxTransform& globalPose1 = transformCache.getTransformCache(elementID1).transform;

		PX_ASSERT(overlapFunc);
		overlap = overlapFunc(	primitive0->getCore().getGeometry(), globalPose0,
								primitive1->getCore().getGeometry(), globalPose1,
								&tri->getTriggerCache(), UNUSED_OVERLAP_THREAD_CONTEXT);
	}

	const bool hadOverlap = tri->lastFrameHadContacts();
	if(hadOverlap)
	{
		if(!overlap)
			pairEvent = PxPairFlag::eNOTIFY_TOUCH_LOST;
	}
	else
	{
		if(overlap)
			pairEvent = PxPairFlag::eNOTIFY_TOUCH_FOUND;
	}
	tri->updateLastFrameHadContacts(overlap);

	const PxPairFlags triggeredFlags = pairEvent & pairFlags;
	if(triggeredFlags)
	{
		triggerPair.triggerShape = s0.getPxShape();
		triggerPair.otherShape = s1.getPxShape();
		triggerPair.status = PxPairFlag::Enum(PxU32(pairEvent));
		triggerPair.flags = PxTriggerPairFlags(testForRemovedShapes);

		const ActorCore& actorCore0 = s0.getActor().getActorCore();
		const ActorCore& actorCore1 = s1.getActor().getActorCore();

#if PX_SUPPORT_GPU_PHYSX
		if (actorCore0.getActorCoreType() == PxActorType::eSOFTBODY)
			triggerPair.triggerActor = static_cast<const SoftBodyCore&>(actorCore0).getPxActor();
		else
#endif
			triggerPair.triggerActor = static_cast<const RigidCore&>(actorCore0).getPxActor();

#if PX_SUPPORT_GPU_PHYSX
		if (actorCore0.getActorCoreType() == PxActorType::eSOFTBODY)
			triggerPair.otherActor = static_cast<const SoftBodyCore&>(actorCore1).getPxActor();
		else
#endif
			triggerPair.otherActor = static_cast<const RigidCore&>(actorCore1).getPxActor();
		

		triggerPairExtra = TriggerPairExtraData(s0.getElementID(), s1.getElementID(),
							actorCore0.getOwnerClient(), actorCore1.getOwnerClient());
		return true;
	}
	return false;
}

class TriggerContactTask : public Cm::Task
{
private:
	TriggerContactTask& operator = (const TriggerContactTask&);

public:
	TriggerContactTask(Interaction* const* triggerPairs, PxU32 triggerPairCount, PxMutex& lock,
		TriggerInteraction** pairsToDeactivate, volatile PxI32& pairsToDeactivateCount,
		Scene& scene, PxsTransformCache& transformCache):
		Cm::Task(scene.getContextId()),
		mTriggerPairs(triggerPairs),
		mTriggerPairCount(triggerPairCount),
		mLock(lock),
		mPairsToDeactivate(pairsToDeactivate),
		mPairsToDeactivateCount(pairsToDeactivateCount),
		mScene(scene),
		mTransformCache(transformCache)
	{
	}

	virtual void runInternal()
	{
		SimStats::TriggerPairCountsNonVolatile triggerPairStats;
#if PX_ENABLE_SIM_STATS
		PxMemZero(&triggerPairStats, sizeof(SimStats::TriggerPairCountsNonVolatile));
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
		PxTriggerPair triggerPair[sTriggerPairsPerTask];
		TriggerPairExtraData triggerPairExtra[sTriggerPairsPerTask];
		PxU32 triggerReportItemCount = 0;

		TriggerInteraction* deactivatePairs[sTriggerPairsPerTask];
		PxI32 deactivatePairCount = 0;


		for(PxU32 i=0; i < mTriggerPairCount; i++)
		{
			TriggerInteraction* tri = static_cast<TriggerInteraction*>(mTriggerPairs[i]);

			PX_ASSERT(tri->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
			
			if(findTriggerContacts(tri, false, false, triggerPair[triggerReportItemCount], 
				triggerPairExtra[triggerReportItemCount], triggerPairStats, mTransformCache))
				triggerReportItemCount++;

			if(!(tri->readFlag(TriggerInteraction::PROCESS_THIS_FRAME)))
			{
				// active trigger pairs for which overlap tests were not forced should remain in the active list
				// to catch transitions between overlap and no overlap
				continue;
			}
			else
			{
				tri->clearFlag(TriggerInteraction::PROCESS_THIS_FRAME);

				// explicitly scheduled overlap test is done (after object creation, teleport, ...). Check if trigger pair should remain active or not.

				if(!tri->onActivate_(0))
				{
					PX_ASSERT(tri->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
					// Why is the assert enough?
					// Once an explicit overlap test is scheduled, the interaction can not get deactivated anymore until it got processed.

					tri->clearInteractionFlag(InteractionFlag::eIS_ACTIVE);
					deactivatePairs[deactivatePairCount] = tri;
					deactivatePairCount++;
				}
			}
		}

		if(triggerReportItemCount)
		{
			PxTriggerPair* triggerPairBuffer;
			TriggerPairExtraData* triggerPairExtraBuffer;

			{
				PxMutex::ScopedLock lock(mLock);

				mScene.reserveTriggerReportBufferSpace(triggerReportItemCount, triggerPairBuffer, triggerPairExtraBuffer);

				PxMemCopy(triggerPairBuffer, triggerPair, sizeof(PxTriggerPair) * triggerReportItemCount);
				PxMemCopy(triggerPairExtraBuffer, triggerPairExtra, sizeof(TriggerPairExtraData) * triggerReportItemCount);
			}
		}

		if(deactivatePairCount)
		{
			PxI32 newSize = PxAtomicAdd(&mPairsToDeactivateCount, deactivatePairCount);
			PxMemCopy(mPairsToDeactivate + newSize - deactivatePairCount, deactivatePairs, sizeof(TriggerInteraction*) * deactivatePairCount);
		}

#if PX_ENABLE_SIM_STATS
		SimStats& simStats = mScene.getStatsInternal();
		for(PxU32 i=0; i < PxGeometryType::eCONVEXMESH+1; i++)
		{
			for(PxU32 j=0; j < PxGeometryType::eGEOMETRY_COUNT; j++)
			{
				if(triggerPairStats[i][j] != 0)
					PxAtomicAdd(&simStats.numTriggerPairs[i][j], triggerPairStats[i][j]);
			}
		}
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	}

	virtual const char* getName() const
	{
		return "ScNPhaseCore.triggerInteractionWork";
	}

public:
	static const PxU32 sTriggerPairsPerTask = 64;

private:
	Interaction* const* mTriggerPairs;
	const PxU32 mTriggerPairCount;
	PxMutex& mLock;
	TriggerInteraction** mPairsToDeactivate;
	volatile PxI32& mPairsToDeactivateCount;
	Scene& mScene;
	PxsTransformCache& mTransformCache;
};

}  // namespace Sc
}  // namespace physx

void NPhaseCore::processTriggerInteractions(PxBaseTask* continuation)
{
	PX_ASSERT(!mTmpTriggerProcessingBlock);
	PX_ASSERT(mTriggerPairsToDeactivateCount == 0);

	Scene& scene = mOwnerScene;

	// Triggers
	Interaction** triggerInteractions = mOwnerScene.getActiveInteractions(InteractionType::eTRIGGER);
	const PxU32 pairCount = mOwnerScene.getNbActiveInteractions(InteractionType::eTRIGGER);

	if(pairCount > 0)
	{
		const PxU32 taskCountWithoutRemainder = pairCount / TriggerContactTask::sTriggerPairsPerTask;
		const PxU32 maxTaskCount = taskCountWithoutRemainder + 1;
		const PxU32 pairPtrSize = pairCount * sizeof(TriggerInteraction*);
		const PxU32 memBlockSize = pairPtrSize + (maxTaskCount * sizeof(TriggerContactTask));
		void* triggerProcessingBlock = scene.getLowLevelContext()->getScratchAllocator().alloc(memBlockSize, true);
		if(triggerProcessingBlock)
		{
			const bool hasMultipleThreads = scene.getTaskManager().getCpuDispatcher()->getWorkerCount() > 1;
			const bool moreThanOneBatch = pairCount > TriggerContactTask::sTriggerPairsPerTask;
			const bool scheduleTasks = hasMultipleThreads && moreThanOneBatch;
			// when running on a single thread, the task system seems to cause the main overhead (locking and atomic operations
			// seemed less of an issue). Hence, the tasks get run directly in that case. Same if there is only one batch.

			mTmpTriggerProcessingBlock = triggerProcessingBlock;  // note: gets released in the continuation task
			if(scheduleTasks)
				mMergeProcessedTriggerInteractions.setContinuation(continuation);

			TriggerInteraction** triggerPairsToDeactivateWriteBack = reinterpret_cast<TriggerInteraction**>(triggerProcessingBlock);
			TriggerContactTask* triggerContactTaskBuffer = reinterpret_cast<TriggerContactTask*>(reinterpret_cast<PxU8*>(triggerProcessingBlock) + pairPtrSize);
			PxsTransformCache& transformCache = mOwnerScene.getLowLevelContext()->getTransformCache();

			PxU32 remainder = pairCount;
			while(remainder)
			{
				const PxU32 nb = remainder > TriggerContactTask::sTriggerPairsPerTask ? TriggerContactTask::sTriggerPairsPerTask : remainder;
				remainder -= nb;

				TriggerContactTask* task = triggerContactTaskBuffer;
				task = PX_PLACEMENT_NEW(task, TriggerContactTask(	triggerInteractions, nb, mTriggerWriteBackLock,
																	triggerPairsToDeactivateWriteBack, mTriggerPairsToDeactivateCount, scene, transformCache));
				if(scheduleTasks)
				{
					task->setContinuation(&mMergeProcessedTriggerInteractions);
					task->removeReference();
				}
				else
					task->runInternal();

				triggerContactTaskBuffer++;
				triggerInteractions += nb;
			}

			if(scheduleTasks)
				mMergeProcessedTriggerInteractions.removeReference();
			else
				mMergeProcessedTriggerInteractions.runInternal();
		}
		else
		{
			outputError<PxErrorCode::eOUT_OF_MEMORY>(__LINE__, "Temporary memory for trigger pair processing could not be allocated. Trigger overlap tests will not take place.");
		}
	}
}

void NPhaseCore::mergeProcessedTriggerInteractions(PxBaseTask*)
{
	if(mTmpTriggerProcessingBlock)
	{
		// deactivate pairs that do not need trigger checks any longer (until woken up again)
		TriggerInteraction** triggerPairsToDeactivate = reinterpret_cast<TriggerInteraction**>(mTmpTriggerProcessingBlock);
		for(PxI32 i=0; i < mTriggerPairsToDeactivateCount; i++)
		{
			mOwnerScene.notifyInteractionDeactivated(triggerPairsToDeactivate[i]);
		}
		mTriggerPairsToDeactivateCount = 0;

		mOwnerScene.getLowLevelContext()->getScratchAllocator().free(mTmpTriggerProcessingBlock);
		mTmpTriggerProcessingBlock = NULL;
	}
}

void NPhaseCore::visualize(PxRenderOutput& renderOut, PxsContactManagerOutputIterator& outputs)
{
	// PT: put common reads here to avoid doing them for each interaction

	const PxReal scale = mOwnerScene.getVisualizationScale();
	if(scale == 0.0f)
		return;

	const PxReal param_contactForce = mOwnerScene.getVisualizationParameter(PxVisualizationParameter::eCONTACT_FORCE);
	const PxReal param_contactNormal = mOwnerScene.getVisualizationParameter(PxVisualizationParameter::eCONTACT_NORMAL);
	const PxReal param_contactError = mOwnerScene.getVisualizationParameter(PxVisualizationParameter::eCONTACT_ERROR);
	const PxReal param_contactPoint = mOwnerScene.getVisualizationParameter(PxVisualizationParameter::eCONTACT_POINT);

	if(param_contactForce==0.0f && param_contactNormal==0.0f && param_contactError==0.0f && param_contactPoint==0.0f)
		return;

	Interaction** interactions = mOwnerScene.getActiveInteractions(InteractionType::eOVERLAP);
	PxU32 nbActiveInteractions = mOwnerScene.getNbActiveInteractions(InteractionType::eOVERLAP);
	while(nbActiveInteractions--)
		static_cast<ShapeInteraction*>(*interactions++)->visualize(	renderOut, outputs,
																	scale, param_contactForce, param_contactNormal, param_contactError, param_contactPoint);
}

#ifdef REMOVED
class ProcessPersistentContactTask : public Cm::Task
{
	Sc::NPhaseCore& mCore;
	ContactReportBuffer& mBuffer;
	PxMutex& mMutex;
	ShapeInteraction*const* mPersistentEventPairs;
	PxU32 mNbPersistentEventPairs;
	PxsContactManagerOutputIterator mOutputs;
	PX_NOCOPY(ProcessPersistentContactTask)
public:

	ProcessPersistentContactTask(Sc::NPhaseCore& core, ContactReportBuffer& buffer, PxMutex& mutex, ShapeInteraction*const* persistentEventPairs,
		PxU32 nbPersistentEventPairs, PxsContactManagerOutputIterator& outputs) : Cm::Task(0), mCore(core), mBuffer(buffer), mMutex(mutex),
		mPersistentEventPairs(persistentEventPairs), mNbPersistentEventPairs(nbPersistentEventPairs), mOutputs(outputs)
	{
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("ProcessPersistentContactTask", mCore.getScene().getContextId());
		PxU32 size = mNbPersistentEventPairs;
		ShapeInteraction*const* persistentEventPairs = mPersistentEventPairs;
		while (size--)
		{
			ShapeInteraction* pair = *persistentEventPairs++;
			if (size)
			{
				if (size > 1)
				{
					if (size > 2)
					{
						ShapeInteraction* nextPair = *(persistentEventPairs + 2);
						prefetchLine(nextPair);
					}

					ShapeInteraction* nextPair = *(persistentEventPairs + 1);
					ActorPair* aPair = nextPair->getActorPair();
					prefetchLine(aPair);

					prefetchLine(&nextPair->getShape0());
					prefetchLine(&nextPair->getShape1());
				}
				ShapeInteraction* nextPair = *(persistentEventPairs);
				prefetchLine(&nextPair->getShape0().getActor());
				prefetchLine(&nextPair->getShape1().getActor());
			}

			PX_ASSERT(pair->hasTouch());
			PX_ASSERT(pair->isReportPair());

			const PxU32 pairFlags = pair->getPairFlags();
			if ((pairFlags & PxU32(PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eDETECT_DISCRETE_CONTACT)) == PxU32(PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eDETECT_DISCRETE_CONTACT))
			{
				// do not process the pair if only eDETECT_CCD_CONTACT is enabled because at this point CCD did not run yet. Plus the current CCD implementation can not reliably provide eNOTIFY_TOUCH_PERSISTS events
				// for performance reasons.
				//KS - filter based on edge activity!

				const ActorSim& bodySim0 = pair->getShape0().getActor();
				const ActorSim& bodySim1 = pair->getShape1().getActor();
			
				if (bodySim0.isActive() || (!bodySim1.isStaticRigid() && bodySim1.isActive()))
					pair->processUserNotificationAsync(PxPairFlag::eNOTIFY_TOUCH_PERSISTS, 0, false, 0, false, mOutputs/*, &alloc*/);
			}
		}
	}

	virtual const char* getName() const
	{
		return "ScNPhaseCore.ProcessPersistentContactTask";
	}

};
#endif

void NPhaseCore::processPersistentContactEvents(PxsContactManagerOutputIterator& outputs, PxBaseTask* continuation)
{
	PX_UNUSED(continuation);
	PX_UNUSED(outputs);
	PX_PROFILE_ZONE("Sc::NPhaseCore::processPersistentContactEvents", mOwnerScene.getContextId());
	
	// Go through ShapeInteractions which requested persistent contact event reports. This is necessary since there are no low level events for persistent contact.
	ShapeInteraction*const* persistentEventPairs = getCurrentPersistentContactEventPairs();
	PxU32 size = getCurrentPersistentContactEventPairCount();
	while (size--)
	{
		ShapeInteraction* pair = *persistentEventPairs++;
		if (size)
		{
			ShapeInteraction* nextPair = *persistentEventPairs;
			PxPrefetchLine(nextPair);
		}

		ActorPair* aPair = pair->getActorPair();
		PxPrefetchLine(aPair);

		PX_ASSERT(pair->hasTouch());
		PX_ASSERT(pair->isReportPair());

		const PxU32 pairFlags = pair->getPairFlags();
		if ((pairFlags & PxU32(PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eDETECT_DISCRETE_CONTACT)) == PxU32(PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eDETECT_DISCRETE_CONTACT))
		{
			// do not process the pair if only eDETECT_CCD_CONTACT is enabled because at this point CCD did not run yet. Plus the current CCD implementation can not reliably provide eNOTIFY_TOUCH_PERSISTS events
			// for performance reasons.
			//KS - filter based on edge activity!

			const ActorSim& actorSim0= pair->getShape0().getActor();
			const ActorSim& actorSim1 = pair->getShape1().getActor();

			if (actorSim0.isActive() || ((!actorSim1.isStaticRigid()) && actorSim1.isActive()))
				pair->processUserNotification(PxPairFlag::eNOTIFY_TOUCH_PERSISTS, 0, false, 0, false, outputs);
		}
	}
}

void NPhaseCore::fireCustomFilteringCallbacks(PxsContactManagerOutputIterator& outputs)
{
	PX_PROFILE_ZONE("Sim.fireCustomFilteringCallbacks", mOwnerScene.getContextId());

	PxSimulationFilterCallback* callback = mOwnerScene.getFilterCallbackFast();

	if(callback)
	{
		// Ask user for pair filter status changes
		PxU32 pairID;
		PxFilterFlags filterFlags;
		PxPairFlags pairFlags;
		while(callback->statusChange(pairID, pairFlags, filterFlags))
		{
			ElementSimInteraction* ei = (*mFilterPairManager)[pairID];

			PX_ASSERT(ei);
			// Check if the user tries to update a pair even though he deleted it earlier in the same frame

			checkFilterFlags(filterFlags);

			PX_ASSERT(ei->readInteractionFlag(InteractionFlag::eIS_FILTER_PAIR));

			PxFilterInfo finfo;
			finfo.filterFlags = filterFlags;
			finfo.pairFlags = pairFlags;
			finfo.filterPairIndex = pairID;

			ElementSimInteraction* refInt = refilterInteraction(ei, &finfo, true, outputs);

			// this gets called at the end of the simulation -> there should be no dirty interactions around
			PX_ASSERT(!refInt->readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST));
			PX_ASSERT(!refInt->getDirtyFlags());

			if((refInt == ei) && (refInt->getType() == InteractionType::eOVERLAP))  // No interaction conversion happened, the pairFlags were just updated
				static_cast<ShapeInteraction*>(refInt)->updateState(InteractionDirtyFlag::eFILTER_STATE);
		}
	}
}

void NPhaseCore::addToDirtyInteractionList(Interaction* pair)
{
	mDirtyInteractions.insert(pair);
}

void NPhaseCore::removeFromDirtyInteractionList(Interaction* pair)
{
	PX_ASSERT(mDirtyInteractions.contains(pair));

	mDirtyInteractions.erase(pair);
}

void NPhaseCore::updateDirtyInteractions(PxsContactManagerOutputIterator& outputs)
{
	// The sleeping SIs will be updated on activation
	// clow: Sleeping SIs are not awaken for visualization updates
	const bool dirtyDominance = mOwnerScene.readFlag(SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_DOMINANCE);
	const bool dirtyVisualization = mOwnerScene.readFlag(SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_VISUALIZATION);
	if(dirtyDominance || dirtyVisualization)
	{
		// Update all interactions.

		const PxU8 mask = PxTo8((dirtyDominance ? InteractionDirtyFlag::eDOMINANCE : 0) | (dirtyVisualization ? InteractionDirtyFlag::eVISUALIZATION : 0));

		Interaction** it = mOwnerScene.getInteractions(InteractionType::eOVERLAP);
		PxU32 size = mOwnerScene.getNbInteractions(InteractionType::eOVERLAP);
		while(size--)
		{
			Interaction* pair = *it++;

			PX_ASSERT(pair->getType() == InteractionType::eOVERLAP);

			if(!pair->readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST))
			{
				PX_ASSERT(!pair->getDirtyFlags());
				static_cast<ShapeInteraction*>(pair)->updateState(mask);
			}
			else
				pair->setDirty(mask);  // the pair will get processed further below anyway, so just mark the flags dirty
		}
	}

	// Update all interactions in the dirty list
	const PxU32 dirtyItcCount = mDirtyInteractions.size();
	Interaction* const* dirtyInteractions = mDirtyInteractions.getEntries();
	for(PxU32 i = 0; i < dirtyItcCount; i++)
	{
		Interaction* refInt = dirtyInteractions[i];
		Interaction* interaction = refInt;

		if(interaction->isElementInteraction() && interaction->needsRefiltering())
		{
			ElementSimInteraction* pair = static_cast<ElementSimInteraction*>(interaction);

			refInt = refilterInteraction(pair, NULL, false, outputs);
		}

		if(interaction == refInt)  // Refiltering might convert the pair to another type and kill the old one. In that case we don't want to update the new pair since it has been updated on creation.
		{
			const InteractionType::Enum iType = interaction->getType();
			if (iType == InteractionType::eOVERLAP)
				static_cast<ShapeInteraction*>(interaction)->updateState(0);
			else if (iType == InteractionType::eCONSTRAINTSHADER)
				static_cast<ConstraintInteraction*>(interaction)->updateState();

			interaction->setClean(false);  // false because the dirty interactions list gets cleard further below
		}
	}

	mDirtyInteractions.clear();
}

void NPhaseCore::releaseElementPair(ElementSimInteraction* pair, PxU32 flags, ElementSim* removedElement, const PxU32 ccdPass, bool removeFromDirtyList,
	PxsContactManagerOutputIterator& outputs)
{
	pair->setClean(removeFromDirtyList);  // Removes the pair from the dirty interaction list etc.

	if(pair->readInteractionFlag(InteractionFlag::eIS_FILTER_PAIR))
	{
		// Check if this is a filter callback pair
		const PxU32 filterPairIndex = mFilterPairManager->findIndex(pair);
		PX_ASSERT(filterPairIndex!=INVALID_FILTER_PAIR_INDEX);

		callPairLost(mOwnerScene, pair->getElement0(), pair->getElement1(), filterPairIndex, (removedElement != NULL));

		mFilterPairManager->releaseIndex(filterPairIndex);
	}

	switch(pair->getType())
	{
		case InteractionType::eTRIGGER:
			{
				PxsTransformCache& transformCache = mOwnerScene.getLowLevelContext()->getTransformCache();
				TriggerInteraction* tri = static_cast<TriggerInteraction*>(pair);
				PxTriggerPair triggerPair;
				TriggerPairExtraData triggerPairExtra;
				if (findTriggerContacts(tri, true, (removedElement != NULL),
										triggerPair, triggerPairExtra, 
										const_cast<SimStats::TriggerPairCountsNonVolatile&>(mOwnerScene.getStatsInternal().numTriggerPairs), 
										transformCache))
										// cast away volatile-ness (this is fine since the method does not run in parallel)
				{
					mOwnerScene.getTriggerBufferAPI().pushBack(triggerPair);
					mOwnerScene.getTriggerBufferExtraData().pushBack(triggerPairExtra);
				}
				mTriggerInteractionPool.destroy(tri);
			}
			break;
		case InteractionType::eMARKER:
			{
				ElementInteractionMarker* interactionMarker = static_cast<ElementInteractionMarker*>(pair);
				mInteractionMarkerPool.destroy(interactionMarker);
			}
			break;
		case InteractionType::eOVERLAP:
			{
				ShapeInteraction* si = static_cast<ShapeInteraction*>(pair);
				if(flags & PairReleaseFlag::eRUN_LOST_TOUCH_LOGIC)
					lostTouchReports(si, flags, removedElement, ccdPass, outputs);

				mShapeInteractionPool.destroy(si);
			}
			break;
		case InteractionType::eCONSTRAINTSHADER:
		case InteractionType::eARTICULATION:
		case InteractionType::eTRACKED_IN_SCENE_COUNT:
		case InteractionType::eINVALID:
			PX_ASSERT(0);
			return;
	}
}

void NPhaseCore::lostTouchReports(ShapeInteraction* si, PxU32 flags, ElementSim* removedElement, PxU32 ccdPass, 
	PxsContactManagerOutputIterator& outputs)
{
	if(si->hasTouch())
	{
		if(si->isReportPair())
			si->sendLostTouchReport((removedElement != NULL), ccdPass, outputs);

		si->adjustCountersOnLostTouch();
	}

	ActorPair* aPair = si->getActorPair();
	if(aPair && aPair->decRefCount() == 0)
	{
		RigidSim* sim0 = static_cast<RigidSim*>(&si->getActorSim0());
		RigidSim* sim1 = static_cast<RigidSim*>(&si->getActorSim1());

		BodyPairKey pair;

		if(sim0->getActorID() > sim1->getActorID())
			PxSwap(sim0, sim1);

		pair.mSim0 = sim0->getActorID();
		pair.mSim1 = sim1->getActorID();

		mActorPairMap.erase(pair);

		if(!aPair->isReportPair())
		{
			mActorPairPool.destroy(aPair);
		}
		else
		{
			ActorPairReport& apr = ActorPairReport::cast(*aPair);
			destroyActorPairReport(apr);
		}
	}
	si->clearActorPair();

	if(si->hasTouch() || (!si->hasKnownTouchState()))
	{
		ActorSim& b0 = si->getShape0().getActor();
		ActorSim& b1 = si->getShape1().getActor();

		if(flags & PairReleaseFlag::eWAKE_ON_LOST_TOUCH)
		{
			// we rely on shape pair ordering here, where the first body is never static
			// (see createShapeInteraction())
			PX_ASSERT(!b0.isStaticRigid());

			if (removedElement == NULL)
			{
				if (b1.isStaticRigid())  // no check for b0 being static, see assert further above
				{
					// given wake-on-lost-touch has been requested:
					// if one is static, we wake up the other immediately

					b0.internalWakeUp();
				}
				else if(!si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
				{
					mOwnerScene.addToLostTouchList(b0, b1);
				}
			}
			else
			{
				// given wake-on-lost-touch has been requested:
				// if an element (broadphase volume) has been removed, we wake the other actor up

				PX_ASSERT((removedElement == &si->getShape0()) || (removedElement == &si->getShape1()));

				if (&si->getShape0() == removedElement)
				{
					if (!b1.isStaticRigid())
						b1.internalWakeUp();
				}
				else
					b0.internalWakeUp();   // no check for b0 being non-static, see assert further above
			}
		}
	}
}

void NPhaseCore::clearContactReportActorPairs(bool shrinkToZero)
{
	for(PxU32 i=0; i < mContactReportActorPairSet.size(); i++)
	{
		//TODO: prefetch?
		ActorPairReport* aPair = mContactReportActorPairSet[i];
		const PxU32 refCount = aPair->getRefCount();
		PX_ASSERT(aPair->isInContactReportActorPairSet());
		PX_ASSERT(refCount > 0);
		aPair->decRefCount();  // Reference held by contact callback
		if(refCount > 1)
		{
			aPair->clearInContactReportActorPairSet();
		}
		else
		{
			BodyPairKey pair;
			PxU32 actorAID = aPair->getActorAID();
			PxU32 actorBID = aPair->getActorBID();
			pair.mSim0 = PxMin(actorAID, actorBID);
			pair.mSim1 = PxMax(actorAID, actorBID);

			mActorPairMap.erase(pair);
			destroyActorPairReport(*aPair);
		}
	}

	if(!shrinkToZero)
		mContactReportActorPairSet.clear();
	else
		mContactReportActorPairSet.reset();
}

void NPhaseCore::addToPersistentContactEventPairs(ShapeInteraction* si)
{
	// Pairs which request events which do not get triggered by the sdk and thus need to be tested actively every frame.
	PX_ASSERT(si->getPairFlags() & (PxPairFlag::eNOTIFY_TOUCH_PERSISTS | ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS));
	PX_ASSERT(si->mReportPairIndex == INVALID_REPORT_PAIR_ID);
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(si->hasTouch()); // only pairs which can from now on lose or keep contact should be in this list

	si->raiseFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST);
	if(mPersistentContactEventPairList.size() == mNextFramePersistentContactEventPairIndex)
	{
		si->mReportPairIndex = mPersistentContactEventPairList.size();
		mPersistentContactEventPairList.pushBack(si);
	}
	else
	{
		//swap with first entry that will be active next frame
		ShapeInteraction* firstDelayedSi = mPersistentContactEventPairList[mNextFramePersistentContactEventPairIndex];
		firstDelayedSi->mReportPairIndex = mPersistentContactEventPairList.size();
		mPersistentContactEventPairList.pushBack(firstDelayedSi);
		si->mReportPairIndex = mNextFramePersistentContactEventPairIndex;
		mPersistentContactEventPairList[mNextFramePersistentContactEventPairIndex] = si;
	}

	mNextFramePersistentContactEventPairIndex++;
}

void NPhaseCore::addToPersistentContactEventPairsDelayed(ShapeInteraction* si)
{
	// Pairs which request events which do not get triggered by the sdk and thus need to be tested actively every frame.
	PX_ASSERT(si->getPairFlags() & (PxPairFlag::eNOTIFY_TOUCH_PERSISTS | ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS));
	PX_ASSERT(si->mReportPairIndex == INVALID_REPORT_PAIR_ID);
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(si->hasTouch()); // only pairs which can from now on lose or keep contact should be in this list

	si->raiseFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST);
	si->mReportPairIndex = mPersistentContactEventPairList.size();
	mPersistentContactEventPairList.pushBack(si);
}

void NPhaseCore::removeFromPersistentContactEventPairs(ShapeInteraction* si)
{
	PX_ASSERT(si->getPairFlags() & (PxPairFlag::eNOTIFY_TOUCH_PERSISTS | ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS));
	PX_ASSERT(si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(si->hasTouch()); // only pairs which could lose or keep contact should be in this list

	PxU32 index = si->mReportPairIndex;
	PX_ASSERT(index != INVALID_REPORT_PAIR_ID);

	if(index < mNextFramePersistentContactEventPairIndex)
	{
		const PxU32 replaceIdx = mNextFramePersistentContactEventPairIndex - 1;

		if((mNextFramePersistentContactEventPairIndex < mPersistentContactEventPairList.size()) && (index != replaceIdx))
		{
			// keep next frame persistent pairs at the back of the list
			ShapeInteraction* tmp = mPersistentContactEventPairList[replaceIdx];
			mPersistentContactEventPairList[index] = tmp;
			tmp->mReportPairIndex = index;
			index = replaceIdx;
		}

		mNextFramePersistentContactEventPairIndex--;
	}

	si->clearFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST);
	si->mReportPairIndex = INVALID_REPORT_PAIR_ID;
	mPersistentContactEventPairList.replaceWithLast(index);
	if(index < mPersistentContactEventPairList.size()) // Only adjust the index if the removed SIP was not at the end of the list
		mPersistentContactEventPairList[index]->mReportPairIndex = index;
}

void NPhaseCore::addToForceThresholdContactEventPairs(ShapeInteraction* si)
{
	PX_ASSERT(si->getPairFlags() & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS);
	PX_ASSERT(si->mReportPairIndex == INVALID_REPORT_PAIR_ID);
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(si->hasTouch());

	si->raiseFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST);
	si->mReportPairIndex = mForceThresholdContactEventPairList.size();
	mForceThresholdContactEventPairList.pushBack(si);
}

void NPhaseCore::removeFromForceThresholdContactEventPairs(ShapeInteraction* si)
{
	PX_ASSERT(si->getPairFlags() & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS);
	PX_ASSERT(si->readFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST));
	PX_ASSERT(!si->readFlag(ShapeInteraction::IS_IN_PERSISTENT_EVENT_LIST));
	PX_ASSERT(si->hasTouch());

	const PxU32 index = si->mReportPairIndex;
	PX_ASSERT(index != INVALID_REPORT_PAIR_ID);

	si->clearFlag(ShapeInteraction::IS_IN_FORCE_THRESHOLD_EVENT_LIST);
	si->mReportPairIndex = INVALID_REPORT_PAIR_ID;
	mForceThresholdContactEventPairList.replaceWithLast(index);
	if(index < mForceThresholdContactEventPairList.size()) // Only adjust the index if the removed SIP was not at the end of the list
		mForceThresholdContactEventPairList[index]->mReportPairIndex = index;
}

PxU8* NPhaseCore::reserveContactReportPairData(PxU32 pairCount, PxU32 extraDataSize, PxU32& bufferIndex,
	ContactReportAllocationManager* alloc)
{
	extraDataSize = ContactStreamManager::computeExtraDataBlockSize(extraDataSize);
	return alloc ? alloc->allocate(extraDataSize + (pairCount * sizeof(ContactShapePair)), bufferIndex) : mContactReportBuffer.allocateNotThreadSafe(extraDataSize + (pairCount * sizeof(ContactShapePair)), bufferIndex);
}

PxU8* NPhaseCore::resizeContactReportPairData(PxU32 pairCount, PxU32 extraDataSize, ContactStreamManager& csm)
{
	PX_ASSERT((pairCount > csm.maxPairCount) || (extraDataSize > csm.getMaxExtraDataSize()));
	PX_ASSERT((csm.currentPairCount == csm.maxPairCount) || (extraDataSize > csm.getMaxExtraDataSize()));
	PX_ASSERT(extraDataSize >= csm.getMaxExtraDataSize()); // we do not support stealing memory from the extra data part when the memory for pair info runs out

	PxU32 bufferIndex;
	PxPrefetch(mContactReportBuffer.getData(csm.bufferIndex));

	extraDataSize = ContactStreamManager::computeExtraDataBlockSize(extraDataSize);
	PxU8* stream = mContactReportBuffer.reallocateNotThreadSafe(extraDataSize + (pairCount * sizeof(ContactShapePair)), bufferIndex, 16, csm.bufferIndex);
	PxU8* oldStream = mContactReportBuffer.getData(csm.bufferIndex);
	if(stream)
	{
		const PxU32 maxExtraDataSize = csm.getMaxExtraDataSize();
		if(csm.bufferIndex != bufferIndex)
		{
			if(extraDataSize <= maxExtraDataSize)
				PxMemCopy(stream, oldStream, maxExtraDataSize + (csm.currentPairCount * sizeof(ContactShapePair)));
			else
			{
				PxMemCopy(stream, oldStream, csm.extraDataSize);
				PxMemCopy(stream + extraDataSize, oldStream + maxExtraDataSize, csm.currentPairCount * sizeof(ContactShapePair));
			}
			csm.bufferIndex = bufferIndex;
		}
		else if(extraDataSize > maxExtraDataSize)
			PxMemMove(stream + extraDataSize, oldStream + maxExtraDataSize, csm.currentPairCount * sizeof(ContactShapePair));

		if(pairCount > csm.maxPairCount)
			csm.maxPairCount = PxTo16(pairCount);
		if(extraDataSize > maxExtraDataSize)
			csm.setMaxExtraDataSize(extraDataSize);
	}
	return stream;
}

ActorPairContactReportData* NPhaseCore::createActorPairContactReportData()
{
	PxMutex::ScopedLock lock(mReportAllocLock);
	return mActorPairContactReportDataPool.construct();
}

void NPhaseCore::releaseActorPairContactReportData(ActorPairContactReportData* data)
{
	mActorPairContactReportDataPool.destroy(data);
}

