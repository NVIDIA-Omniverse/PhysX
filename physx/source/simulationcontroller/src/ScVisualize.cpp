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

#include "ScScene.h"
#include "ScNPhaseCore.h"
#include "ScShapeInteraction.h"
#include "ScConstraintSim.h"
#include "ScConstraintCore.h"
#include "CmVisualization.h"

using namespace physx;
using namespace Sc;

// PT: Sc-level visualization code has been moved to this dedicated file (like we did in NpDebugViz.cpp)

static void visualize(const ConstraintSim& sim, Cm::ConstraintImmediateVisualizer& viz, PxU32 flags, const PxTransform& idt)
{
	ConstraintCore& core = sim.getCore();
	if(!(core.getFlags() & PxConstraintFlag::eVISUALIZATION))
		return;

	const Dy::Constraint& llc = sim.getLowLevelConstraint();

	PxsRigidBody* b0 = llc.body0;
	PxsRigidBody* b1 = llc.body1;
	
	const PxTransform& t0 = b0 ? b0->getPose() : idt;
	const PxTransform& t1 = b1 ? b1->getPose() : idt;

	core.getVisualize()(viz, llc.constantBlock, t0, t1, flags);
}

void Sc::ShapeInteraction::visualize(PxRenderOutput& out, PxsContactManagerOutputIterator& outputs,
									float scale, float param_contactForce, float param_contactNormal, float param_contactError, float param_contactPoint)
{
	if(mManager)  // sleeping pairs have no contact points -> do not visualize
	{
		Sc::ActorSim* actorSim0 = &getShape0().getActor();
		Sc::ActorSim* actorSim1 = &getShape1().getActor();
		if(!actorSim0->isNonRigid() && !actorSim1->isNonRigid())
		{
			PxU32 offset;
			PxU32 nextOffset = 0;
			do
			{
				const void* contactPatches;
				const void* contactPoints;
				PxU32 contactDataSize;
				PxU32 contactPointCount;
				PxU32 contactPatchCount;
				const PxReal* impulses;

				offset = nextOffset;
				nextOffset = getContactPointData(contactPatches, contactPoints, contactDataSize, contactPointCount, contactPatchCount, impulses, offset, outputs);

				const PxU32* faceIndices = reinterpret_cast<const PxU32*>(impulses + contactPointCount);
				PxContactStreamIterator iter(reinterpret_cast<const PxU8*>(contactPatches), reinterpret_cast<const PxU8*>(contactPoints), faceIndices, contactPatchCount, contactPointCount);

				PxU32 i = 0;
				while(iter.hasNextPatch())
				{
					iter.nextPatch();
					while(iter.hasNextContact())
					{
						iter.nextContact();

						if((param_contactForce != 0.0f) && impulses)
						{
							out << PxU32(PxDebugColor::eARGB_RED);
							out.outputSegment(iter.getContactPoint(), iter.getContactPoint() + iter.getContactNormal() * (scale * param_contactForce * impulses[i]));
						}
						else if(param_contactNormal != 0.0f)
						{
							out << PxU32(PxDebugColor::eARGB_BLUE);
							out.outputSegment(iter.getContactPoint(), iter.getContactPoint() + iter.getContactNormal() * (scale * param_contactNormal));
						}
						else if(param_contactError != 0.0f)
						{
							out << PxU32(PxDebugColor::eARGB_YELLOW);
							out.outputSegment(iter.getContactPoint(), iter.getContactPoint() + iter.getContactNormal() * PxAbs(scale * param_contactError * PxMin(0.f, iter.getSeparation())));
						}

						if(param_contactPoint != 0.0f)
						{
							const PxReal s = scale * 0.1f;
							const PxVec3& point = iter.getContactPoint();

							//if (0) //temp debug to see identical contacts
							//	point.x += scale * 0.01f * (contactPointCount - i + 1);

							out << PxU32(PxDebugColor::eARGB_RED);
							out.outputSegment(point + PxVec3(-s, 0, 0), point + PxVec3(s, 0, 0));
							out.outputSegment(point + PxVec3(0, -s, 0), point + PxVec3(0, s, 0));
							out.outputSegment(point + PxVec3(0, 0, -s), point + PxVec3(0, 0, s));
						}
						i++;
					}
				}
			} while (nextOffset != offset);
		}
	}
}

// Render objects before simulation starts
void Sc::Scene::visualizeStartStep()
{
	PX_PROFILE_ZONE("Sim.visualizeStartStep", mContextId);

	// Update from visualization parameters
	if(mVisualizationParameterChanged)
	{
		mVisualizationParameterChanged = false;

		// Update SIPs if visualization is enabled
		if(	getVisualizationParameter(PxVisualizationParameter::eCONTACT_POINT) || getVisualizationParameter(PxVisualizationParameter::eCONTACT_NORMAL) || 
			getVisualizationParameter(PxVisualizationParameter::eCONTACT_ERROR) || getVisualizationParameter(PxVisualizationParameter::eCONTACT_FORCE))
			mInternalFlags |= SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_VISUALIZATION;
	}

#if PX_ENABLE_DEBUG_VISUALIZATION
	const PxReal scale = getVisualizationScale();
	if(scale==0.0f)
	{
		// make sure visualization inside simulate was skipped
		PX_ASSERT(getRenderBuffer().empty()); 
		return; // early out if visualization scale is 0
	}

	PxRenderOutput out(getRenderBuffer());

	if(getVisualizationParameter(PxVisualizationParameter::eCOLLISION_COMPOUNDS))
		mAABBManager->visualize(out);

	// Visualize joints
	{
		const float frameScale = scale * getVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES);
		const float limitScale = scale * getVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS);
		if(frameScale!=0.0f || limitScale!=0.0f)
		{
			Cm::ConstraintImmediateVisualizer viz(frameScale, limitScale, out);

			PxU32 flags = 0;
			if(frameScale!=0.0f)
				flags |= PxConstraintVisualizationFlag::eLOCAL_FRAMES;
			if(limitScale!=0.0f)
				flags |= PxConstraintVisualizationFlag::eLIMITS;

			const PxTransform idt(PxIdentity);

			Sc::ConstraintCore*const * constraints = mConstraints.getEntries();
			for(PxU32 i=0, size = mConstraints.size();i<size; i++)
			{
				ConstraintSim* sim = constraints[i]->getSim();
				if(sim)
					visualize(*sim, viz, flags, idt);
			}
		}
	}

	{
		// PT: put common reads here to avoid doing them for each interaction
		const PxReal param_contactForce = getVisualizationParameter(PxVisualizationParameter::eCONTACT_FORCE);
		const PxReal param_contactNormal = getVisualizationParameter(PxVisualizationParameter::eCONTACT_NORMAL);
		const PxReal param_contactError = getVisualizationParameter(PxVisualizationParameter::eCONTACT_ERROR);
		const PxReal param_contactPoint = getVisualizationParameter(PxVisualizationParameter::eCONTACT_POINT);

		if(param_contactForce!=0.0f || param_contactNormal!=0.0f || param_contactError!=0.0f || param_contactPoint!=0.0f)
		{
			PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

			ElementSimInteraction** interactions = getActiveInteractions(InteractionType::eOVERLAP);
			PxU32 nbActiveInteractions = getNbActiveInteractions(InteractionType::eOVERLAP);
			while(nbActiveInteractions--)
				static_cast<ShapeInteraction*>(*interactions++)->visualize(	out, outputs,
																			scale, param_contactForce, param_contactNormal, param_contactError, param_contactPoint);
		}
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif
}
