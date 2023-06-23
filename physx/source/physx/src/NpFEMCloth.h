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

#ifndef PX_PHYSICS_NP_FEMCLOTH
#define PX_PHYSICS_NP_FEMCLOTH

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "PxFEMCloth.h"
#endif
#include "ScFEMClothCore.h"
#include "NpActorTemplate.h"

namespace physx
{
	class NpShape;

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	class NpFEMCloth : public NpActorTemplate<PxFEMCloth>
	{
	public:
		NpFEMCloth(PxCudaContextManager& cudaContextManager);
		NpFEMCloth(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager);

		virtual				 			~NpFEMCloth() {}

		void exportData(PxSerializationContext& /*context*/) const {}

		//external API
		virtual				 			PxActorType::Enum	getType() const { return PxActorType::eFEMCLOTH; }

		virtual	PxBounds3	 			getWorldBounds(float inflation = 1.01f) const;

		virtual	void					setFEMClothFlag(PxFEMClothFlag::Enum flag, bool val);
		virtual	void					setFEMClothFlags(PxFEMClothFlags flags);
		virtual	PxFEMClothFlags	  		getFEMClothFlag() const;

#if 0 // disabled until future use.
		virtual void					setDrag(const PxReal drag);
		virtual PxReal					getDrag() const;

		virtual void					setLift(const PxReal lift);
		virtual PxReal					getLift() const;

		virtual	void					setWind(const PxVec3& wind);
		virtual	PxVec3					getWind() const;

		virtual	void					setAirDensity(const PxReal wind);
		virtual	PxReal					getAirDensity() const;

		virtual	void					setBendingActivationAngle(const PxReal angle);
		virtual	PxReal					getBendingActivationAngle() const;
#endif

		virtual	void					setParameter(const PxFEMParameters& paramters);
		virtual PxFEMParameters			getParameter() const;

		virtual	void					setBendingScales(const PxReal* const bendingScale, PxU32 nbElements);
	    virtual const PxReal*			getBendingScales() const;
	    virtual PxU32					getNbBendingScales() const;

		virtual	void					setMaxVelocity(const PxReal v);
		virtual	PxReal					getMaxVelocity() const;

		virtual	void					setMaxDepenetrationVelocity(const PxReal v);
		virtual	PxReal					getMaxDepenetrationVelocity() const;

		virtual	void					setNbCollisionPairUpdatesPerTimestep(const PxU32 frequency);
		virtual	PxU32					getNbCollisionPairUpdatesPerTimestep() const;

		virtual	void					setNbCollisionSubsteps(const PxU32 frequency);
		virtual	PxU32					getNbCollisionSubsteps() const;

		virtual PxVec4*					getPositionInvMassBufferD();
		virtual PxVec4*					getVelocityBufferD();
		virtual PxVec4*					getRestPositionBufferD();
		
		virtual void					markDirty(PxFEMClothDataFlags flags);

		virtual	void					addRigidFilter(PxRigidActor* actor, PxU32 vertId);
		virtual	void					removeRigidFilter(PxRigidActor* actor, PxU32 vertId);

		virtual	PxU32					addRigidAttachment(PxRigidActor* actor, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
		virtual	void					removeRigidAttachment(PxRigidActor* actor, PxU32 handle);

		virtual	void					addTriRigidFilter(PxRigidActor* actor, PxU32 triangleId);
		virtual	void					removeTriRigidFilter(PxRigidActor* actor, PxU32 triangleId);

		virtual	PxU32					addTriRigidAttachment(PxRigidActor* actor, PxU32 triangleId, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
		virtual	void					removeTriRigidAttachment(PxRigidActor* actor, PxU32 handle);

		virtual	void					addClothFilter(PxFEMCloth* otherCloth, PxU32 otherTriIdx, PxU32 triIdx);
		virtual	void					removeClothFilter(PxFEMCloth* otherCloth, PxU32 otherTriIdx, PxU32 triIdx);

		virtual	PxU32					addClothAttachment(PxFEMCloth* otherCloth, PxU32 otherTriIdx, const PxVec4& otherTriBarycentric, PxU32 triIdx, const PxVec4& triBarycentric);
		virtual	void					removeClothAttachment(PxFEMCloth* otherCloth, PxU32 handle);

		virtual	PxCudaContextManager*	getCudaContextManager() const;
		virtual	void					setCudaContextManager(PxCudaContextManager*);

		virtual	void					setSolverIterationCounts(PxU32 minPositionIters);

		virtual	void					getSolverIterationCounts(PxU32& minPositionIters) const;

		virtual PxShape*				getShape();

		virtual	bool					attachShape(PxShape& shape);

		virtual void					detachShape();

		virtual void					release();

		PX_FORCE_INLINE	const Sc::FEMClothCore&	getCore() const { return mCore; }
		PX_FORCE_INLINE	Sc::FEMClothCore&		getCore() { return mCore; }

		static PX_FORCE_INLINE size_t			getCoreOffset() { return PX_OFFSET_OF_RT(NpFEMCloth, mCore); }

		virtual	bool					isSleeping() const;


		// Debug name
		void							setName(const char*);
		const char*						getName() const;


		void							updateMaterials();

	private:

		NpShape*						mShape;
		Sc::FEMClothCore				mCore;
		PxCudaContextManager*			mCudaContextManager;
	};
#endif
}
#endif

#endif
