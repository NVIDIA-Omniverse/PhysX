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

#include "ScFiltering.h"
#include "ScShapeInteraction.h"
#include "ScTriggerInteraction.h"
#include "ScConstraintCore.h"
#include "ScArticulationSim.h"

using namespace physx;
using namespace Sc;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE PxU64 getPairID(const ShapeSimBase& s0, const ShapeSimBase& s1)
{
	PxU64 id0 = PxU64(s0.getElementID());
	PxU64 id1 = PxU64(s1.getElementID());
	if(id1<id0)
		PxSwap(id0, id1);
	const PxU64 pairID = (id0<<32)|id1;
	return pairID;
}

///////////////////////////////////////////////////////////////////////////////

template<const bool supportTriggers>
static PxFilterObjectAttributes getFilterObjectAttributes(const ShapeSimBase& shape)
{
	const ActorSim& actorSim = shape.getActor();

	PxFilterObjectAttributes filterAttr = actorSim.getFilterAttributes();

	if(supportTriggers && (shape.getCore().getFlags() & PxShapeFlag::eTRIGGER_SHAPE))
		filterAttr |= PxFilterObjectFlag::eTRIGGER;

#if PX_DEBUG
	BodySim* b = shape.getBodySim();
	if(b)
	{
		if(!b->isArticulationLink())
		{
			if(b->isKinematic())
				PX_ASSERT(filterAttr & PxFilterObjectFlag::eKINEMATIC);

			PX_ASSERT(PxGetFilterObjectType(filterAttr)==PxFilterObjectType::eRIGID_DYNAMIC);
		}
		else
		{
			PX_ASSERT(PxGetFilterObjectType(filterAttr)==PxFilterObjectType::eARTICULATION);
		}
	}
	else
	{
	#if PX_SUPPORT_GPU_PHYSX
		// For softbody and particle system, the bodySim is set to null
		if(actorSim.isSoftBody())
		{
			PX_ASSERT(PxGetFilterObjectType(filterAttr)==PxFilterObjectType::eSOFTBODY);
		}
		else if(actorSim.isParticleSystem())
		{
			PX_ASSERT(PxGetFilterObjectType(filterAttr)==PxFilterObjectType::ePARTICLESYSTEM);
		}
		else if(actorSim.isFEMCloth())
		{
			PX_ASSERT(PxGetFilterObjectType(filterAttr)==PxFilterObjectType::eFEMCLOTH);
		}
		else if(actorSim.isHairSystem())
		{
			PX_ASSERT(PxGetFilterObjectType(filterAttr)==PxFilterObjectType::eHAIRSYSTEM);
		}
		else
	#endif
		{
			PX_ASSERT(PxGetFilterObjectType(filterAttr)==PxFilterObjectType::eRIGID_STATIC);
		}
	}
#endif
	return filterAttr;
}

///////////////////////////////////////////////////////////////////////////////

// PT: checks that the kill & suppress flags are not both set, disable kill flag if they are.
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

///////////////////////////////////////////////////////////////////////////////

static PX_INLINE PxPairFlags checkRbPairFlags(	const ShapeSimBase& s0, const ShapeSimBase& s1, bool isKinePair,
												PxPairFlags pairFlags, PxFilterFlags filterFlags, bool isNonRigid)
{
	if(filterFlags & (PxFilterFlag::eSUPPRESS | PxFilterFlag::eKILL))
		return pairFlags;

	if(isKinePair && (pairFlags & PxPairFlag::eSOLVE_CONTACT))
	{
#if PX_CHECKED
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: Resolving contacts between two kinematic objects is invalid. Contacts will not get resolved.");
#endif
		pairFlags.clear(PxPairFlag::eSOLVE_CONTACT);
	}

	if(isNonRigid && (pairFlags & PxPairFlag::eDETECT_CCD_CONTACT))
		pairFlags.clear(PxPairFlag::eDETECT_CCD_CONTACT);

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

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE bool createFilterInfo(FilterInfo& filterInfo, const PxFilterFlags filterFlags)
{
	filterInfo = FilterInfo(filterFlags);
	return true;
}

static void filterRbCollisionPairSecondStage(FilterInfo& filterInfo, const FilteringContext& context, const ShapeSimBase& s0, const ShapeSimBase& s1, bool isKinePair,
											const PxFilterObjectAttributes fa0, const PxFilterObjectAttributes fa1, bool runCallbacks, bool isNonRigid)
{
	// Run filter shader
	const PxFilterData& fd0 = s0.getCore().getSimulationFilterData();
	const PxFilterData& fd1 = s1.getCore().getSimulationFilterData();
	filterInfo.filterFlags = context.mFilterShader(fa0, fd0, fa1, fd1, filterInfo.pairFlags, context.mFilterShaderData, context.mFilterShaderDataSize);

	if(filterInfo.filterFlags & PxFilterFlag::eCALLBACK)
	{
		if(context.mFilterCallback)
		{
			if(!runCallbacks)
			{
				return;
			}
			else
			{
				// If a FilterPair is provided, then we use it, else we create a new one
				// (A FilterPair is provided in the case for a pairLost()-pairFound() sequence after refiltering)

				struct Local
				{
					static PX_FORCE_INLINE PxShape* fetchActorAndShape(const ShapeSimBase& sim, const PxFilterObjectAttributes fa, PxActor*& a)
					{
						a = sim.getActor().getPxActor();

#if PX_SUPPORT_GPU_PHYSX
						if(PxGetFilterObjectType(fa)==PxFilterObjectType::ePARTICLESYSTEM)
							return NULL;	// Particle system does not have a valid shape so set it to null
#endif
						PX_UNUSED(fa);
						return sim.getPxShape();
					}
				};

				PxActor* a0, *a1;
				PxShape* shape0 = Local::fetchActorAndShape(s0, fa0, a0);
				PxShape* shape1 = Local::fetchActorAndShape(s1, fa1, a1);

				filterInfo.filterFlags = context.mFilterCallback->pairFound(getPairID(s0, s1), fa0, fd0, a0, shape0, fa1, fd1, a1, shape1, filterInfo.pairFlags);
				filterInfo.hasPairID = true;
			}
		}
		else
		{
			filterInfo.filterFlags.clear(PxFilterFlag::eNOTIFY);
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Filtering: eCALLBACK set but no filter callback defined.");
		}
	}

	checkFilterFlags(filterInfo.filterFlags);

	const bool hasNotify = (filterInfo.filterFlags & PxFilterFlag::eNOTIFY) == PxFilterFlag::eNOTIFY;
	const bool hasKill = filterInfo.filterFlags & PxFilterFlag::eKILL;

	{
		if(filterInfo.hasPairID && (hasKill || !hasNotify))
		{
			if(hasKill && hasNotify)
				context.mFilterCallback->pairLost(getPairID(s0, s1), fa0, fd0, fa1, fd1, false);
			if(!hasNotify)
			{
				// No notification, hence we don't need to treat it as a filter callback pair anymore.
				// Make sure that eCALLBACK gets removed as well
				filterInfo.filterFlags.clear(PxFilterFlag::eNOTIFY);
			}

			filterInfo.hasPairID = false;
		}
	}

	// Sanity checks
	PX_ASSERT((!hasKill) || (hasKill && (!filterInfo.hasPairID)));
	PX_ASSERT((!hasNotify) || (hasNotify && filterInfo.hasPairID));

	if(runCallbacks || (!(filterInfo.filterFlags & PxFilterFlag::eCALLBACK)))
		filterInfo.pairFlags = checkRbPairFlags(s0, s1, isKinePair, filterInfo.pairFlags, filterInfo.filterFlags, isNonRigid);
}

static bool filterArticulationLinks(const BodySim* bs0, const BodySim* bs1)
{
	//It's the same articulation, so we can filter based on flags...
	const ArticulationSim* articulationSim0 = bs0->getArticulation();
	const ArticulationSim* articulationSim1 = bs1->getArticulation();
	if(articulationSim0 == articulationSim1)
	{
		if(articulationSim0->getCore().getArticulationFlags() & PxArticulationFlag::eDISABLE_SELF_COLLISION)
			return true;

		//check to see if one link is the parent of the other link, if so disable collision
		const PxU32 linkId0 = bs0->getNodeIndex().articulationLinkId();
		const PxU32 linkId1 = bs1->getNodeIndex().articulationLinkId();

		if(linkId1 < linkId0)
			return articulationSim0->getLink(linkId0).parent == linkId1;
		else
			return articulationSim1->getLink(linkId1).parent == linkId0;
	}
	
	return false;
}

static PX_FORCE_INLINE bool filterJointedBodies(const ActorSim& rbActor0, const ActorSim& rbActor1)
{
	// If the bodies of the shape pair are connected by a joint, we need to check whether this connection disables the collision.
	// Note: As an optimization, the dynamic bodies have a flag which specifies whether they have any constraints at all. That works
	//       because a constraint has at least one dynamic body and an interaction is tracked by both objects.

	// PT: the BF_HAS_CONSTRAINTS flag is only raised on dynamic actors in the BodySim class, but it's not raised on static actors.
	// Thus the only reliable way to use the flag (without casting to BodySim etc) is when both actors don't have the flag set, in
	// which case we're sure we're not dealing with a jointed pair.
	if(!rbActor0.readInternalFlag(ActorSim::BF_HAS_CONSTRAINTS) && !rbActor1.readInternalFlag(ActorSim::BF_HAS_CONSTRAINTS))
		return false;

	ConstraintCore* core = rbActor0.getScene().findConstraintCore(&rbActor0, &rbActor1);
	return core ? !(core->getFlags() & PxConstraintFlag::eCOLLISION_ENABLED) : false;
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
	PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode)
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

template<const bool runAllTests>
static bool filterRbCollisionPairShared(	FilterInfo& filterInfo, bool& isNonRigid, bool& isKinePair,
											const FilteringContext& context,
											const ShapeSimBase& s0, const ShapeSimBase& s1,
											const PxFilterObjectAttributes filterAttr0, const PxFilterObjectAttributes filterAttr1)
{
	const bool kine0 = PxFilterObjectIsKinematic(filterAttr0);
	const bool kine1 = PxFilterObjectIsKinematic(filterAttr1);

	const ActorSim& rbActor0 = s0.getActor();
	const BodySim* bs0 = NULL;
	if(filterAttr0 & PxFilterObjectFlagEx::eRIGID_DYNAMIC)
		bs0 = static_cast<const BodySim*>(&rbActor0);
	else if(filterAttr0 & PxFilterObjectFlagEx::eNON_RIGID)
		isNonRigid = true;

	const ActorSim& rbActor1 = s1.getActor();
	const BodySim* bs1 = NULL;
	if(filterAttr1 & PxFilterObjectFlagEx::eRIGID_DYNAMIC)
		bs1 = static_cast<const BodySim*>(&rbActor1);
	else if(filterAttr1 & PxFilterObjectFlagEx::eNON_RIGID)
		isNonRigid = true;

	if(!isNonRigid && filterKinematics(bs0, bs1, kine0, kine1, context.mKineKineFilteringMode, context.mStaticKineFilteringMode))
		return createFilterInfo(filterInfo, PxFilterFlag::eSUPPRESS);

	if(filterJointedBodies(rbActor0, rbActor1))
		return createFilterInfo(filterInfo, PxFilterFlag::eSUPPRESS);

	const PxFilterObjectType::Enum filterType0 = PxGetFilterObjectType(filterAttr0);
	const PxFilterObjectType::Enum filterType1 = PxGetFilterObjectType(filterAttr1);

	// PT: For unknown reasons the filtering code was not the same for triggers/refiltered pairs and for regular "shape sim" pairs
	// out of the BP. The tests on "runAllTests" below capture that. I did not change what the code
	// was doing, although it might very well be wrong - we might want to run all these tests in both codepaths.

	if(runAllTests)
	{
#if PX_SUPPORT_GPU_PHYSX
		if(filterType0==PxFilterObjectType::ePARTICLESYSTEM && filterType1==PxFilterObjectType::ePARTICLESYSTEM)
			return createFilterInfo(filterInfo, PxFilterFlag::eKILL);

		if(filterType0==PxFilterObjectType::eHAIRSYSTEM  && filterType1==PxFilterObjectType::eHAIRSYSTEM )
			return createFilterInfo(filterInfo, PxFilterFlag::eKILL);
#endif
	}

	const bool link0 = filterType0==PxFilterObjectType::eARTICULATION;
	const bool link1 = filterType1==PxFilterObjectType::eARTICULATION;

	if(runAllTests)
	{
		if(link0 ^ link1)
		{
			if(link0)
			{
				const PxU8 fixedBaseLink = bs0->getLowLevelBody().mCore->fixedBaseLink;
				const bool isStaticOrKinematic = (filterType1 == PxFilterObjectType::eRIGID_STATIC) || kine1;
				if(fixedBaseLink && isStaticOrKinematic)
					return createFilterInfo(filterInfo, PxFilterFlag::eSUPPRESS);
			}
		
			if(link1)
			{
				const PxU8 fixedBaseLink = bs1->getLowLevelBody().mCore->fixedBaseLink;
				const bool isStaticOrKinematic = (filterType0 == PxFilterObjectType::eRIGID_STATIC) || kine0;
				if(fixedBaseLink && isStaticOrKinematic)
					return createFilterInfo(filterInfo, PxFilterFlag::eSUPPRESS);
			}	
		}
	}

	if(link0 && link1)
	{
		if(runAllTests)
		{
			const PxU8 fixedBaseLink0 = bs0->getLowLevelBody().mCore->fixedBaseLink;
			const PxU8 fixedBaseLink1 = bs1->getLowLevelBody().mCore->fixedBaseLink;

			if(fixedBaseLink0 && fixedBaseLink1)
				return createFilterInfo(filterInfo, PxFilterFlag::eSUPPRESS);
		}

		if(filterArticulationLinks(bs0, bs1))
			return createFilterInfo(filterInfo, PxFilterFlag::eKILL);
	}
	isKinePair = kine0 && kine1;
	return false;
}

static void filterRbCollisionPair(FilterInfo& filterInfo, const FilteringContext& context, const ShapeSimBase& s0, const ShapeSimBase& s1, bool& isTriggerPair, bool runCallbacks)
{
	const PxFilterObjectAttributes filterAttr0 = getFilterObjectAttributes<true>(s0);
	const PxFilterObjectAttributes filterAttr1 = getFilterObjectAttributes<true>(s1);

	const bool trigger0 = PxFilterObjectIsTrigger(filterAttr0);
	const bool trigger1 = PxFilterObjectIsTrigger(filterAttr1);
	isTriggerPair = trigger0 || trigger1;

	bool isNonRigid = false;
	bool isKinePair = false;

	if(isTriggerPair)
	{
		if(trigger0 && trigger1)	// trigger-trigger pairs are not supported
		{
			createFilterInfo(filterInfo, PxFilterFlag::eKILL);
			return;
		}

		// PT: I think we need to do this here to properly handle kinematic triggers.
		const bool kine0 = PxFilterObjectIsKinematic(filterAttr0);
		const bool kine1 = PxFilterObjectIsKinematic(filterAttr1);
		isKinePair = kine0 && kine1;
	}
	else
	{
		if(filterRbCollisionPairShared<false>(filterInfo, isNonRigid, isKinePair, context, s0, s1, filterAttr0, filterAttr1))
			return;
	}

	filterRbCollisionPairSecondStage(filterInfo, context, s0, s1, isKinePair, filterAttr0, filterAttr1, runCallbacks, isNonRigid);
}

static PX_FORCE_INLINE void filterRbCollisionPairAllTests(FilterInfo& filterInfo, const FilteringContext& context, const ShapeSimBase& s0, const ShapeSimBase& s1)
{
	PX_ASSERT(!(s0.getFlags() & PxShapeFlag::eTRIGGER_SHAPE));
	PX_ASSERT(!(s1.getFlags() & PxShapeFlag::eTRIGGER_SHAPE));

	const PxFilterObjectAttributes filterAttr0 = getFilterObjectAttributes<false>(s0);
	const PxFilterObjectAttributes filterAttr1 = getFilterObjectAttributes<false>(s1);

	bool isNonRigid = false;
	bool isKinePair = false;

	if(filterRbCollisionPairShared<true>(filterInfo, isNonRigid, isKinePair, context, s0, s1, filterAttr0, filterAttr1))
		return;

	filterRbCollisionPairSecondStage(filterInfo, context, s0, s1, isKinePair, filterAttr0, filterAttr1, true, isNonRigid);
}

static PX_FORCE_INLINE bool testElementSimPointers(const ElementSim* e0, const ElementSim* e1)
{
	PX_ASSERT(e0);
	PX_ASSERT(e1);

	// PT: a bit of defensive coding added for OM-74224 / PX-3571. In theory this should not be needed, as the broadphase is not
	// supposed to return null pointers here. But there seems to be an issue somewhere, most probably in the GPU BP kernels,
	// and this is an attempt at preventing a crash. We could/should remove this eventually.
	// ### DEFENSIVE
	if(!e0 || !e1)
		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "NPhaseCore::runOverlapFilters: found null elements!");
	return true;
}

static PX_FORCE_INLINE bool testShapeSimCorePointers(const ShapeSimBase* s0, const ShapeSimBase* s1)
{
	bool isValid0 = s0->isPxsCoreValid();
	bool isValid1 = s1->isPxsCoreValid();
	PX_ASSERT(isValid0);
	PX_ASSERT(isValid1);

	// GW: further defensive coding added for OM-111249 / PX-4478.
	// This is only a temporary / immediate solution to mitigate crashes
	// Still need to root-cause what is causing null pointers here
	//
	// AD: TODO what are we doing about this now that there is a fix? Can we "deprecate" this test?
	//
	// ### DEFENSIVE
	if(!isValid0 || !isValid1)
		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "NPhaseCore::runOverlapFilters: found null PxsShapeCore pointers!");
	return true;
}

// PT: called from OverlapFilterTask
void NPhaseCore::runOverlapFilters(	PxU32 nbToProcess, const Bp::AABBOverlap* PX_RESTRICT pairs, FilterInfo* PX_RESTRICT filterInfo,
									PxU32& nbToKeep_, PxU32& nbToSuppress_, PxU32* PX_RESTRICT keepMap
)
{
	PxU32 nbToKeep = 0;
	PxU32 nbToSuppress = 0;

	const FilteringContext context(mOwnerScene);

	for(PxU32 i=0; i<nbToProcess; i++)
	{
		const Bp::AABBOverlap& pair = pairs[i];

		const ElementSim* e0 = reinterpret_cast<const ElementSim*>(pair.mUserData0);
		const ElementSim* e1 = reinterpret_cast<const ElementSim*>(pair.mUserData1);

		if(!testElementSimPointers(e0, e1))
			continue;

		PX_ASSERT(!findInteraction(e0, e1));

		const ShapeSimBase* s0 = static_cast<const ShapeSimBase*>(e0);
		const ShapeSimBase* s1 = static_cast<const ShapeSimBase*>(e1);

		if(!testShapeSimCorePointers(s0, s1))
			continue;
		
		PX_ASSERT(&s0->getActor() != &s1->getActor());	// No actor internal interactions

		filterInfo[i].filterFlags = PxFilterFlags(0);
		filterInfo[i].pairFlags = PxPairFlags(0);
		filterInfo[i].hasPairID = false;
		filterRbCollisionPairAllTests(filterInfo[i], context, *s0, *s1);

		const PxFilterFlags filterFlags = filterInfo[i].filterFlags;

		if(!(filterFlags & PxFilterFlag::eKILL))
		{
			if(!(filterFlags & PxFilterFlag::eSUPPRESS))
				nbToKeep++;
			else
				nbToSuppress++;
			keepMap[i / 32] |= (1 << (i & 31));
		}
	}

	nbToKeep_ = nbToKeep;
	nbToSuppress_ = nbToSuppress;
}

ElementSimInteraction* NPhaseCore::createTriggerElementInteraction(ShapeSimBase& s0, ShapeSimBase& s1)
{
	PX_ASSERT((s0.getFlags() & PxShapeFlag::eTRIGGER_SHAPE) || (s1.getFlags() & PxShapeFlag::eTRIGGER_SHAPE));

	const FilteringContext context(mOwnerScene);

	bool isTriggerPair;
	FilterInfo filterInfo;
	filterRbCollisionPair(filterInfo, context, s0, s1, isTriggerPair, false);
	PX_ASSERT(isTriggerPair);

	if(filterInfo.filterFlags & PxFilterFlag::eKILL)
	{
		PX_ASSERT(!filterInfo.hasPairID);	 // No filter callback pair info for killed pairs
		return NULL;
	}

	return createRbElementInteraction(filterInfo, s0, s1, NULL, NULL, NULL, isTriggerPair);
}

void NPhaseCore::onTriggerOverlapCreated(const Bp::AABBOverlap* PX_RESTRICT pairs, PxU32 pairCount)
{
	for(PxU32 i=0; i<pairCount; i++)
	{
		ElementSim* volume0 = reinterpret_cast<ElementSim*>(pairs[i].mUserData0);
		ElementSim* volume1 = reinterpret_cast<ElementSim*>(pairs[i].mUserData1);

		if(!testElementSimPointers(volume0, volume1))
			continue;

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

void NPhaseCore::callPairLost(const ShapeSimBase& s0, const ShapeSimBase& s1, bool objVolumeRemoved)
{
	const PxFilterObjectAttributes fa0 = getFilterObjectAttributes<true>(s0);
	const PxFilterObjectAttributes fa1 = getFilterObjectAttributes<true>(s1);

	const PxFilterData& fd0 = s0.getCore().getSimulationFilterData();
	const PxFilterData& fd1 = s1.getCore().getSimulationFilterData();

	mOwnerScene.getFilterCallbackFast()->pairLost(getPairID(s0, s1), fa0, fd0, fa1, fd1, objVolumeRemoved);
}

ElementSimInteraction* NPhaseCore::refilterInteraction(ElementSimInteraction* pair, const FilterInfo* filterInfo, bool removeFromDirtyList, PxsContactManagerOutputIterator& outputs)
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

				FilterInfo finfo;
				if(filterInfo)
				{
					// The filter changes are provided by an outside source (the user filter callback)

					finfo = *filterInfo;
					PX_ASSERT(finfo.hasPairID);

					if((finfo.filterFlags & PxFilterFlag::eKILL) &&
						((finfo.filterFlags & PxFilterFlag::eNOTIFY) == PxFilterFlag::eNOTIFY) )
					{
						callPairLost(s0, s1, false);
						finfo.hasPairID = false;
					}

					ActorSim& bs0 = s0.getActor();
					ActorSim& bs1 = s1.getActor();

					const bool isKinePair = PxFilterObjectIsKinematic(bs0.getFilterAttributes()) && PxFilterObjectIsKinematic(bs1.getFilterAttributes());
					finfo.pairFlags = checkRbPairFlags(s0, s1, isKinePair, finfo.pairFlags, finfo.filterFlags, s0.getActor().isNonRigid() || s1.getActor().isNonRigid());
				}
				else
				{
					if(pair->readInteractionFlag(InteractionFlag::eIS_FILTER_PAIR))
						callPairLost(s0, s1, false);

					const FilteringContext context(mOwnerScene);

					bool isTriggerPair;
					filterRbCollisionPair(finfo, context, s0, s1, isTriggerPair, true);
					PX_UNUSED(isTriggerPair);
				}

				if(pair->readInteractionFlag(InteractionFlag::eIS_FILTER_PAIR) &&
					((finfo.filterFlags & PxFilterFlag::eNOTIFY) != PxFilterFlag::eNOTIFY) )
				{
					// The pair was a filter callback pair but not any longer
					pair->clearInteractionFlag(InteractionFlag::eIS_FILTER_PAIR);

					finfo.hasPairID = false;
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

void NPhaseCore::fireCustomFilteringCallbacks(PxsContactManagerOutputIterator& outputs)
{
	PX_PROFILE_ZONE("Sim.fireCustomFilteringCallbacks", mOwnerScene.getContextId());

	PxSimulationFilterCallback* callback = mOwnerScene.getFilterCallbackFast();

	if(callback)
	{
		// Ask user for pair filter status changes
		PxU64 pairID;
		PxFilterFlags filterFlags;
		PxPairFlags pairFlags;
		while(callback->statusChange(pairID, pairFlags, filterFlags))
		{
			const PxU32 id0 = PxU32(pairID);
			const PxU32 id1 = PxU32(pairID>>32);
			const PxHashMap<ElementSimKey, ElementSimInteraction*>::Entry* pair = mElementSimMap.find(ElementSimKey(id0, id1));
			ElementSimInteraction* ei = pair ? pair->second : NULL;
			PX_ASSERT(ei);
			// Check if the user tries to update a pair even though he deleted it earlier in the same frame

			checkFilterFlags(filterFlags);

			PX_ASSERT(ei->readInteractionFlag(InteractionFlag::eIS_FILTER_PAIR));

			FilterInfo finfo;
			finfo.filterFlags = filterFlags;
			finfo.pairFlags = pairFlags;
			finfo.hasPairID = true;
			ElementSimInteraction* refInt = refilterInteraction(ei, &finfo, true, outputs);

			// this gets called at the end of the simulation -> there should be no dirty interactions around
			PX_ASSERT(!refInt->readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST));
			PX_ASSERT(!refInt->getDirtyFlags());

			if((refInt == ei) && (refInt->getType() == InteractionType::eOVERLAP))  // No interaction conversion happened, the pairFlags were just updated
				static_cast<ShapeInteraction*>(refInt)->updateState(InteractionDirtyFlag::eFILTER_STATE);
		}
	}
}

