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

#ifndef SC_ACTOR_SIM_H
#define SC_ACTOR_SIM_H

#include "foundation/PxUserAllocated.h"
#include "CmPtrTable.h"
#include "CmUtils.h"
#include "PxActor.h"
#include "ScInteractionFlags.h"
#include "ScActorCore.h"
#include "PxsSimpleIslandManager.h"
#include "PxFiltering.h"

namespace physx
{

class PxActor;

namespace Sc
{

#define SC_NOT_IN_SCENE_INDEX		0xffffffff  // the body is not in the scene yet
#define SC_NOT_IN_ACTIVE_LIST_INDEX	0xfffffffe  // the body is in the scene but not in the active list

	struct PxFilterObjectFlagEx : PxFilterObjectFlag
	{
		enum Enum
		{
			eRIGID_STATIC		= eNEXT_FREE,
			eRIGID_DYNAMIC		= eNEXT_FREE<<1,
			eNON_RIGID			= eNEXT_FREE<<2,
			eSOFTBODY			= eNEXT_FREE<<3,
			eFEMCLOTH			= eNEXT_FREE<<4,
			ePARTICLESYSTEM		= eNEXT_FREE<<5,
			eHAIRSYSTEM			= eNEXT_FREE<<6,

			eLAST				= eHAIRSYSTEM
		};
	};

	static const PxReal ScInternalWakeCounterResetValue = 20.0f*0.02f;

	class Interaction;
	class ElementSim;
	class Scene;

	class ShapeManager : public PxUserAllocated
	{
	public:

		ShapeManager() {}
		~ShapeManager() {}

		PX_FORCE_INLINE	PxU32				getNbElements()		const	
											{ 
												return mShapes.getCount(); 
											}
		PX_FORCE_INLINE	ElementSim**		getElements()				
											{ 
												return reinterpret_cast<ElementSim**>(mShapes.getPtrs());
											}
		PX_FORCE_INLINE	ElementSim*const*	getElements()		const	
											{ 
												return reinterpret_cast<ElementSim*const*>(mShapes.getPtrs()); 
											}
		//					void			onElementAttach(ElementSim& element);
							void			onElementDetach(ElementSim& element);

		Cm::PtrTable	mShapes;
	};

	class ActorSim : public ShapeManager
	{
		friend class Scene;  // the scene is allowed to set the scene array index
		friend class Interaction;
		PX_NOCOPY(ActorSim)

	public:

		enum InternalFlags
		{
			//BF_DISABLE_GRAVITY			= 1 << 0,	// Don't apply the scene's gravity

			BF_HAS_STATIC_TOUCH				= 1 << 1,	// Set when a body is part of an island with static contacts. Needed to be able to recalculate adaptive force if this changes
			BF_KINEMATIC_MOVED				= 1 << 2,	// Set when the kinematic was moved

			BF_ON_DEATHROW					= 1 << 3,	// Set when the body is destroyed

			BF_IS_IN_SLEEP_LIST				= 1 << 4,	// Set when the body is added to the list of bodies which were put to sleep
			BF_IS_IN_WAKEUP_LIST			= 1 << 5,	// Set when the body is added to the list of bodies which were woken up
			BF_SLEEP_NOTIFY					= 1 << 6,	// A sleep notification should be sent for this body (and not a wakeup event, even if the body is part of the woken list as well)
			BF_WAKEUP_NOTIFY				= 1 << 7,	// A wake up notification should be sent for this body (and not a sleep event, even if the body is part of the sleep list as well)

			BF_HAS_CONSTRAINTS				= 1 << 8,	// Set if the body has one or more constraints
			BF_KINEMATIC_SETTLING			= 1 << 9,	// Set when the body was moved kinematically last frame
			BF_KINEMATIC_SETTLING_2			= 1 << 10,
			BF_KINEMATIC_MOVE_FLAGS			= BF_KINEMATIC_MOVED | BF_KINEMATIC_SETTLING | BF_KINEMATIC_SETTLING_2, //Used to clear kinematic masks in 1 call
			BF_KINEMATIC_SURFACE_VELOCITY	= 1 << 11,	//Set when the application calls setKinematicVelocity. Actor remains awake until application calls clearKinematicVelocity. 
			BF_IS_COMPOUND_RIGID			= 1 << 12,	// Set when the body is a compound actor, we dont want to set the sq bounds

											// PT: WARNING: flags stored on 16-bits now.
		};

											ActorSim(Scene&, ActorCore&);
		virtual								~ActorSim();

		// Get the scene the actor resides in
		PX_FORCE_INLINE	Scene&				getScene()					const	{ return mScene; }

		// Get the number of interactions connected to the actor
		PX_FORCE_INLINE	PxU32				getActorInteractionCount()	const	{ return mInteractions.size(); }

		// Get an iterator to the interactions connected to the actor
		PX_FORCE_INLINE	Interaction**		getActorInteractions()		const	{ return mInteractions.begin();	}

		// Get the type ID of the actor
		PX_FORCE_INLINE	PxActorType::Enum	getActorType()				const	{ return mCore.getActorCoreType();	}

		// Returns true if the actor is a dynamic rigid body (including articulation links)
		PX_FORCE_INLINE	PxU16				isDynamicRigid()			const	{ return mFilterFlags & PxFilterObjectFlagEx::eRIGID_DYNAMIC;	}
		PX_FORCE_INLINE	PxU16				isSoftBody()				const	{ return mFilterFlags & PxFilterObjectFlagEx::eSOFTBODY;		}
		PX_FORCE_INLINE	PxU16				isFEMCloth()				const   { return mFilterFlags & PxFilterObjectFlagEx::eFEMCLOTH;		}
		PX_FORCE_INLINE PxU16				isParticleSystem()			const	{ return mFilterFlags & PxFilterObjectFlagEx::ePARTICLESYSTEM;	}
		PX_FORCE_INLINE	PxU16				isHairSystem()				const	{ return mFilterFlags & PxFilterObjectFlagEx::eHAIRSYSTEM;		}
		PX_FORCE_INLINE PxU16				isNonRigid()				const	{ return mFilterFlags & PxFilterObjectFlagEx::eNON_RIGID;		}
		PX_FORCE_INLINE	PxU16				isStaticRigid()				const   { return mFilterFlags & PxFilterObjectFlagEx::eRIGID_STATIC;	}

		virtual			void				postActorFlagChange(PxU32, PxU32) {}

						void				setActorsInteractionsDirty(InteractionDirtyFlag::Enum flag, const ActorSim* other, PxU8 interactionFlag);

		PX_FORCE_INLINE	ActorCore&			getActorCore()							const	{ return mCore;							}

		PX_FORCE_INLINE	bool				isActive()								const	{ return (mActiveListIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);	}

		PX_FORCE_INLINE PxU32				getActiveListIndex()					const	{ return mActiveListIndex;				}  // if the body is active, the index is smaller than SC_NOT_IN_ACTIVE_LIST_INDEX
		PX_FORCE_INLINE void				setActiveListIndex(PxU32 index)					{ mActiveListIndex = index;				}

		PX_FORCE_INLINE PxU32				getActiveCompoundListIndex()			const	{ return mActiveCompoundListIndex;		}  // if the body is active and is compound, the index is smaller than SC_NOT_IN_ACTIVE_LIST_INDEX
		PX_FORCE_INLINE void				setActiveCompoundListIndex(PxU32 index)			{ mActiveCompoundListIndex = index;		}

		PX_FORCE_INLINE PxNodeIndex			getNodeIndex()							const	{ return mNodeIndex;					}

		PX_FORCE_INLINE PxU32				getActorID()							const	{ return mId;							}

		PX_FORCE_INLINE	PxU16				getInternalFlag()						const	{ return mInternalFlags;				}
		PX_FORCE_INLINE PxU16				readInternalFlag(InternalFlags flag)	const	{ return PxU16(mInternalFlags & flag);	}
		PX_FORCE_INLINE void				raiseInternalFlag(InternalFlags flag)			{ mInternalFlags |= flag;				}
		PX_FORCE_INLINE void				clearInternalFlag(InternalFlags flag)			{ mInternalFlags &= ~flag;				}

		PX_FORCE_INLINE	PxFilterObjectAttributes	getFilterAttributes()			const	{ return PxFilterObjectAttributes(mFilterFlags);	}

		virtual			PxActor*			getPxActor() const = 0;

		//This can all be removed and functionality can be subsumed by the island system, removing the need for this 
		//virtual call and any associated work
		virtual			void				registerCountedInteraction() {}
		virtual			void				unregisterCountedInteraction() {}
		virtual			PxU32				getNumCountedInteractions()	const { return 0;  }

		virtual			void				internalWakeUp(PxReal wakeCounterValue = ScInternalWakeCounterResetValue) { PX_UNUSED(wakeCounterValue); }

	private:
		//These are called from interaction creation/destruction
						void				registerInteractionInActor(Interaction* interaction);
						void				unregisterInteractionFromActor(Interaction* interaction);
						void				reallocInteractions(Sc::Interaction**& mem, PxU32& capacity, PxU32 size, PxU32 requiredMinCapacity);
	protected:
		// dsequeira: interaction arrays are a major cause of small allocations, so we don't want to delegate them to the heap allocator
		// it's not clear this inline array is really needed, we should take it out and see whether the cache perf is worse

		static const PxU32 INLINE_INTERACTION_CAPACITY = 4;
						Interaction*		mInlineInteractionMem[INLINE_INTERACTION_CAPACITY];

		Cm::OwnedArray<Sc::Interaction*, Sc::ActorSim, PxU32, &Sc::ActorSim::reallocInteractions>
											mInteractions;

						Scene&				mScene;
						ActorCore&			mCore;
		// Sleeping
						PxU32				mActiveListIndex;			// Used by Scene to track active bodies
						PxU32				mActiveCompoundListIndex;	// Used by Scene to track active compound bodies

		// Island manager
						PxNodeIndex			mNodeIndex;

						PxU32				mId;	// PT: ID provided by Sc::Scene::mActorIDTracker

						PxU16				mInternalFlags;
						PxU16				mFilterFlags;	// PT: PxFilterObjectAttributes. Capturing the type information in local flags here is redundant
															// but avoids reading the Core memory from the Sim object, and is also faster to test multiple types at once.
	};

} // namespace Sc

}

#endif
