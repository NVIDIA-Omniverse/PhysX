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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SC_PHYSICS_H
#define SC_PHYSICS_H

#include "PxPhysics.h"
#include "PxScene.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxBasicTemplates.h"
#include "PxActor.h"

namespace physx
{

class PxMaterial;
class PxTolerancesScale;
struct PxvOffsetTable;

#if PX_SUPPORT_GPU_PHYSX
class PxPhysXGpu;
#endif

namespace Sc
{
	class Scene;
	class StaticCore;
	class RigidCore;
	class BodyCore;
	class ArticulationCore;
	class ArticulationJointCore;
	class ConstraintCore;
	class ShapeCore;

	struct OffsetTable
	{
		PX_FORCE_INLINE OffsetTable() {}

		PX_FORCE_INLINE PxShape*					convertScShape2Px(ShapeCore* sc)						const	{ return PxPointerOffset<PxShape*>(sc, scShape2Px);							}
		PX_FORCE_INLINE const PxShape*				convertScShape2Px(const ShapeCore* sc)					const	{ return PxPointerOffset<const PxShape*>(sc, scShape2Px);						}

		PX_FORCE_INLINE PxConstraint*				convertScConstraint2Px(ConstraintCore* sc)				const	{ return PxPointerOffset<PxConstraint*>(sc, scConstraint2Px);					}
		PX_FORCE_INLINE const PxConstraint*			convertScConstraint2Px(const ConstraintCore* sc)		const	{ return PxPointerOffset<const PxConstraint*>(sc, scConstraint2Px);			}

		PX_FORCE_INLINE PxArticulationReducedCoordinate*			convertScArticulation2Px(ArticulationCore* sc) const
		{
			return PxPointerOffset<PxArticulationReducedCoordinate*>(sc, scArticulationRC2Px);
		}

		PX_FORCE_INLINE const PxArticulationReducedCoordinate*	convertScArticulation2Px(const ArticulationCore* sc)	const
		{
			return PxPointerOffset<const PxArticulationReducedCoordinate*>(sc, scArticulationRC2Px);
		}

		PX_FORCE_INLINE PxArticulationJointReducedCoordinate*	convertScArticulationJoint2Px(ArticulationJointCore* sc) const
		{
			return PxPointerOffset<PxArticulationJointReducedCoordinate*>(sc, scArticulationJointRC2Px);
		}

		PX_FORCE_INLINE const PxArticulationJointReducedCoordinate* convertScArticulationJoint2Px(const ArticulationJointCore* sc)	const
		{		
			return PxPointerOffset<const PxArticulationJointReducedCoordinate*>(sc, scArticulationJointRC2Px);
		}

		ptrdiff_t	scRigidStatic2PxActor;
		ptrdiff_t 	scRigidDynamic2PxActor;
		ptrdiff_t 	scArticulationLink2PxActor;
		ptrdiff_t 	scSoftBody2PxActor;
		ptrdiff_t 	scPBDParticleSystem2PxActor;
		ptrdiff_t 	scFLIPParticleSystem2PxActor;
		ptrdiff_t 	scMPMParticleSystem2PxActor;
		ptrdiff_t 	scHairSystem2PxActor;
		ptrdiff_t 	scShape2Px;
		ptrdiff_t 	scArticulationRC2Px;
		ptrdiff_t 	scArticulationJointRC2Px;
		ptrdiff_t 	scConstraint2Px;

		ptrdiff_t	scCore2PxActor[PxActorType::eACTOR_COUNT];
	};
	extern OffsetTable gOffsetTable;

	class Physics : public PxUserAllocated
	{
	public:
		PX_FORCE_INLINE static Physics&				getInstance()						{ return *mInstance; }

													Physics(const PxTolerancesScale&, const PxvOffsetTable& pxvOffsetTable);
													~Physics();

		PX_FORCE_INLINE	const PxTolerancesScale&	getTolerancesScale()		const	{ return mScale;	}

	private:
						PxTolerancesScale			mScale;
		static			Physics*					mInstance;

	public:
		static			const PxReal				sWakeCounterOnCreation;
	};

} // namespace Sc

}

#endif
