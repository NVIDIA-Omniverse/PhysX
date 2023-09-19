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

#ifndef NP_SOFTBODY_H
#define NP_SOFTBODY_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "PxSoftBody.h"
#include "ScSoftBodyCore.h"
#include "NpActorTemplate.h"
#include "GuTetrahedronMesh.h"

namespace physx
{
	class NpScene;
	class NpShape;

	class NpSoftBody : public NpActorTemplate<PxSoftBody>
	{
	public:
										NpSoftBody(PxCudaContextManager& cudaContextManager);
										NpSoftBody(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager);
		
		virtual							~NpSoftBody() {}

		void                            exportData(PxSerializationContext& /*context*/) const{}
	
		//external API
		virtual PxActorType::Enum       getType() const { return PxActorType::eSOFTBODY; }

		virtual PxBounds3               getWorldBounds(float inflation = 1.01f) const;
		virtual PxU32                   getGpuSoftBodyIndex();
		
		virtual void                    setSoftBodyFlag(PxSoftBodyFlag::Enum flag, bool val);
		virtual void                    setSoftBodyFlags(PxSoftBodyFlags flags);
		virtual PxSoftBodyFlags         getSoftBodyFlag() const;

		virtual void                    setParameter(PxFEMParameters paramters);
		virtual PxFEMParameters         getParameter() const;

		virtual PxVec4*                 getPositionInvMassBufferD();
		virtual PxVec4*                 getRestPositionBufferD();
		virtual PxVec4*                 getSimPositionInvMassBufferD();
		virtual PxVec4*                 getSimVelocityBufferD();

		virtual void                    markDirty(PxSoftBodyDataFlags flags);

		virtual void                    setKinematicTargetBufferD(const PxVec4* positions, PxSoftBodyFlags flags);

		virtual PxCudaContextManager*   getCudaContextManager() const;

		virtual	void					setWakeCounter(PxReal wakeCounterValue);
		virtual	PxReal					getWakeCounter() const;

		virtual	bool					isSleeping() const;

		virtual	void					setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters);

		virtual	void					getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const;

		virtual PxShape*				getShape();
		virtual PxTetrahedronMesh*		getCollisionMesh();
		virtual const PxTetrahedronMesh*	getCollisionMesh() const;
		virtual PxTetrahedronMesh*		getSimulationMesh() { return mSimulationMesh; }
		virtual const PxTetrahedronMesh*	getSimulationMesh() const { return mSimulationMesh; }
		virtual PxSoftBodyAuxData*		getSoftBodyAuxData() { return mSoftBodyAuxData; }
		virtual const PxSoftBodyAuxData*	getSoftBodyAuxData() const { return mSoftBodyAuxData; }

		virtual	bool					attachShape(PxShape& shape);
		virtual bool					attachSimulationMesh(PxTetrahedronMesh& simulationMesh, PxSoftBodyAuxData& softBodyAuxData);

		virtual void					detachShape();
		virtual void					detachSimulationMesh();

		virtual void					release();

		PX_FORCE_INLINE	const Sc::SoftBodyCore&	getCore()	const	{ return mCore; }
		PX_FORCE_INLINE	Sc::SoftBodyCore&		getCore()			{ return mCore; }
		static PX_FORCE_INLINE size_t			getCoreOffset()		{ return PX_OFFSET_OF_RT(NpSoftBody, mCore); }

		virtual		void				addParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId);
		virtual		void				removeParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId);

		virtual		PxU32				addParticleAttachment(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId, const PxVec4& barycentric);
		virtual		void				removeParticleAttachment(PxPBDParticleSystem* particlesystem, PxU32 handle);

		virtual		void				addRigidFilter(PxRigidActor* actor, PxU32 vertId);
		virtual		void				removeRigidFilter(PxRigidActor* actor, PxU32 vertId);

		virtual		PxU32				addRigidAttachment(PxRigidActor* actor, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
		virtual		void				removeRigidAttachment(PxRigidActor* actor, PxU32 handle);

		virtual		void				addTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx);
		virtual		void				removeTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx);

		virtual		PxU32				addTetRigidAttachment(PxRigidActor* actor, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, 
										PxConeLimitedConstraint* constraint);
		
		virtual		void				addSoftBodyFilter(PxSoftBody* softbody0, PxU32 tetIdx0, PxU32 tetIdx1);
		virtual		void				removeSoftBodyFilter(PxSoftBody* softbody0, PxU32 tetIdx0, PxU32 tetIdx1);
		virtual		void				addSoftBodyFilters(PxSoftBody* softbody0, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);
		virtual		void				removeSoftBodyFilters(PxSoftBody* softbody0, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);

		virtual		PxU32				addSoftBodyAttachment(PxSoftBody* softbody0, PxU32 tetIdx0, const PxVec4& tetBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
										PxConeLimitedConstraint* constraint, PxReal constraintOffset);
		virtual		void				removeSoftBodyAttachment(PxSoftBody* softbody0, PxU32 handle);

		virtual		void				addClothFilter(PxFEMCloth* cloth, PxU32 triIdx, PxU32 tetIdx);
		virtual		void				removeClothFilter(PxFEMCloth* cloth, PxU32 triIdx, PxU32 tetIdx);

		virtual		void				addVertClothFilter(PxFEMCloth* cloth, PxU32 vertIdx, PxU32 tetIdx);
		virtual		void				removeVertClothFilter(PxFEMCloth* cloth, PxU32 vertIdx, PxU32 tetIdx);

		virtual		PxU32				addClothAttachment(PxFEMCloth* cloth, PxU32 triIdx, const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric, 
										PxConeLimitedConstraint* constraint, PxReal constraintOffset);

		virtual		void				removeClothAttachment(PxFEMCloth* cloth, PxU32 handle);

		// Debug name
		void							setName(const char*);
		const char*						getName() const;

		void		updateMaterials();

	private:

		NpShape*					mShape; //soft body should just have one shape. The geometry type should be tetrahedron mesh
		Gu::TetrahedronMesh*		mSimulationMesh;
		Gu::SoftBodyAuxData*		mSoftBodyAuxData;
		Sc::SoftBodyCore			mCore;
		PxCudaContextManager*		mCudaContextManager;
	};
}
#endif
#endif
