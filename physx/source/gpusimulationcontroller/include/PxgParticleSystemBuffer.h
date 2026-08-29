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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_PARTICLE_SYSTEM_BUFFER_H
#define PXG_PARTICLE_SYSTEM_BUFFER_H

#include "foundation/PxUserAllocated.h"
#include "PxParticleBuffer.h"
#include "PxNodeIndex.h"

#include "CmPinnableArray.h"
#include "PxsParticleBuffer.h"

#include "PxgCudaBuffer.h"

namespace physx
{

class PxCudaContextManager;
struct PxgAllocatorDesc;
struct PxgParticleContactInfo;
struct PxgParticleSimBuffer;
struct PxgParticleDiffuseSimBuffer;
struct PxgPBDParticleMaterialDerived;
struct PxgParticleCollisionHeader;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class BufferClass>
class PxgParticleBufferBase : public BufferClass, public PxUserAllocated
{
public:
	PxgParticleBufferBase(PxU32 maxNumParticles, PxCudaContextManager& contextManager);
	virtual ~PxgParticleBufferBase();

	//PxsParticleBuffer
	virtual PxVec4* getPositionInvMassesD() const PX_OVERRIDE PX_FINAL { return mPositionInvMassesD; }
	virtual PxVec4* getVelocitiesD() const PX_OVERRIDE PX_FINAL { return mVelocitiesD; }
	virtual PxU32* getPhasesD() const PX_OVERRIDE PX_FINAL { return mPhasesD; }
	virtual PxVec4* getPositionInvMassesH() const PX_OVERRIDE PX_FINAL { return mPositionInvMassesH; }
	virtual PxVec4* getVelocitiesH() const PX_OVERRIDE PX_FINAL { return mVelocitiesH; }
	virtual PxU32* getPhasesH() const PX_OVERRIDE PX_FINAL { return mPhasesH; }
	virtual void setNbActiveParticles(PxU32 nbActiveParticles) PX_OVERRIDE;
	virtual PxU32 getNbActiveParticles() const PX_OVERRIDE PX_FINAL { return mNumActiveParticles; }
	virtual PxU32 getMaxParticles() const PX_OVERRIDE PX_FINAL { return mMaxNumParticles; }
	virtual PxU32 getFlatListStartIndex() const PX_OVERRIDE PX_FINAL { return mFlatListStartIndex; }
	virtual void raiseFlags(PxParticleBufferFlag::Enum flags) PX_OVERRIDE PX_FINAL { mBufferFlags |= flags; }
	virtual PxU32 getUniqueId() const PX_OVERRIDE PX_FINAL { return mUniqueId; }
	virtual void allocHostBuffers() PX_OVERRIDE PX_FINAL;
	//~PxsParticleBuffer

	void setFlatListStartIndex(PxU32 flatListStartIndex) { mFlatListStartIndex = flatListStartIndex; }
	void copyToHost(CUstream stream);

	PxU32 mBufferFlags;

public:
	PxCudaContextManager& mContextManager;

	PxVec4* mPositionInvMassesD;
	PxVec4* mVelocitiesD;
	PxU32* mPhasesD;
	PxVec4* mPositionInvMassesH;
	PxVec4* mVelocitiesH;
	PxU32* mPhasesH;
	PxU32 mNumActiveParticles;
	PxU32 mMaxNumParticles;
	PxU32 mFlatListStartIndex;
	PxU32 mUniqueId;
};

class PxgParticleBuffer : public PxgParticleBufferBase<PxsParticleBuffer>
{
public:
	PxgParticleBuffer(PxU32 maxNumParticles, PxCudaContextManager& contextManager);
	virtual ~PxgParticleBuffer() {}

	//PxsParticleBuffer
	virtual void release() PX_OVERRIDE PX_FINAL { PX_DELETE_THIS; }
	//~PxsParticleBuffer

	void copyToHost(CUstream stream);
};

class PxgParticleAndDiffuseBuffer : public PxgParticleBufferBase<PxsParticleAndDiffuseBuffer>
{
public:
	PxgParticleAndDiffuseBuffer(PxU32 maxNumParticles, PxU32 maxNumDiffuseParticles, PxCudaContextManager& contextManager);
	virtual ~PxgParticleAndDiffuseBuffer();
	
	//PxsParticleAndDiffuseBuffer
	virtual void release() PX_OVERRIDE PX_FINAL { PX_DELETE_THIS; }
	virtual PxVec4* getDiffusePositionLifeTimeD() const PX_OVERRIDE PX_FINAL { return mDiffusePositionsLifeTimeD; }
	virtual PxVec4* getDiffuseVelocitiesD() const PX_OVERRIDE PX_FINAL { return mDiffuseVelocitiesD; }
	virtual PxU32 getNbActiveDiffuseParticles() const PX_OVERRIDE PX_FINAL
	{
		return mNumActiveDiffuseParticlesH ? *mNumActiveDiffuseParticlesH : 0;
	}
	virtual void setMaxActiveDiffuseParticles(PxU32 maxActiveDiffuseParticles) PX_OVERRIDE PX_FINAL;
	virtual PxU32 getMaxDiffuseParticles() const PX_OVERRIDE PX_FINAL { return mMaxNumDiffuseParticles; }
	virtual void setDiffuseParticleParams(const PxDiffuseParticleParams& params) PX_OVERRIDE PX_FINAL;
	virtual const PxDiffuseParticleParams& getDiffuseParticleParams() const PX_OVERRIDE PX_FINAL { return mParams; }
	//~PxsParticleAndDiffuseBuffer

	void copyToHost(CUstream stream);

public:
	PxDiffuseParticleParams	mParams;
	PxVec4* mDiffusePositionsLifeTimeD;
	PxVec4* mDiffuseVelocitiesD;
	PxI32* mNumDiffuseParticlesD;
	PxU32 mMaxNumDiffuseParticles;
	PxU32 mMaxActiveDiffuseParticles;
	PxI32* mNumActiveDiffuseParticlesH; //mapped pinned memory
};

class PxgParticleSystemBuffer
{
public:

	PxgParticleSystemBuffer(PxgAllocatorDesc& allocDesc);

	PxgTypedCudaBuffer<PxVec4>			originalPosition_mass;			//we should be able to get rid of this buffer
	PxgTypedCudaBuffer<PxU32>			grid_particle_hash;
	PxgTypedCudaBuffer<PxU32>			grid_particle_index;
	PxgTypedCudaBuffer<PxVec4>			sorted_position_mass;
	PxgTypedCudaBuffer<PxVec4>			sorted_velocity;
	PxgTypedCudaBuffer<PxVec4>			accumDeltaV;
	PxgTypedCudaBuffer<PxVec4>			sortedDeltaP;
	PxgTypedCudaBuffer<PxU32>			cell_start;
	PxgTypedCudaBuffer<PxU32>			cell_end;
	PxgTypedCudaBuffer<PxgParticleCollisionHeader> collision_headers;
	PxgTypedCudaBuffer<PxU32>			collision_counts;
	PxgTypedCudaBuffer<PxU32>			collision_index;
	PxgTypedCudaBuffer<float2>			collision_impulses;

	PxgTypedCudaBuffer<PxU32>			phases;
	PxgTypedCudaBuffer<float4>			unsortedpositions;
	PxgTypedCudaBuffer<float4>			unsortedvelocities;
	PxgTypedCudaBuffer<float4>			restArray;
	PxgTypedCudaBuffer<float4>			normal;

	PxgTypedCudaBuffer<PxReal>			density;
	PxgTypedCudaBuffer<PxReal>			staticDensity;
	PxgTypedCudaBuffer<PxVec4>			surfaceNormal;
	
	PxgTypedCudaBuffer<PxVec4>			delta;
	PxgTypedCudaBuffer<PxVec4>			curl;

	PxgTypedCudaBuffer<float4>			sorted_originalPosition_mass;
	PxgTypedCudaBuffer<PxU32>			sortedPhaseArray;

	PxgTypedCudaBuffer<PxgParticleContactInfo> particleOneWayContacts;
	PxgTypedCudaBuffer<float2>			particleOneWayForces;
	PxgTypedCudaBuffer<PxNodeIndex>		particleOneWayContactsNodeIndices;
	PxgTypedCudaBuffer<float4>			particleOneWayContactsSurfaceVelocities;
	PxgTypedCudaBuffer<PxU32>			particleOneWayContactCount;

	PxgTypedCudaBuffer<PxU32>			reverseLookup;

	PxgTypedCudaBuffer<PxU16>			phase_group_to_material_handle;
	PxgTypedCudaBuffer<PxgPBDParticleMaterialDerived> derivedPBDMaterialProperties;

	PxgTypedCudaBuffer<PxgParticleSimBuffer> user_particle_buffer; //PxgPBDParticleBuffer
	PxgTypedCudaBuffer<PxU32>			user_particle_buffer_runsum; //PxU32*
	PxgTypedCudaBuffer<PxU32>			user_particle_buffer_sorted_unique_ids; //PxU32*
	PxgTypedCudaBuffer<PxU32>			user_particle_buffer_runsum_sorted_unique_ids_original_index; //PxU32*
	PxgTypedCudaBuffer<PxgParticleDiffuseSimBuffer> user_diffuse_buffer;

	Cm::PinnableArray<PxgParticleSimBuffer>			mHostParticleBuffers;
	Cm::PinnableArray<PxgParticleDiffuseSimBuffer>	mHostDiffuseBuffers;

	Cm::PinnableArray<PxU32>						mParticleBufferRunSum;

	Cm::PinnableArray<PxU32>						mParticleBufferSortedUniqueIds;
	Cm::PinnableArray<PxU32>						mParticleBufferSortedUniqueIdsOriginalIndex;
	Cm::PinnableArray<PxReal>						mRandomTable;
	Cm::PinnableArray<PxU16>						mHostPhaseGroupToMaterialHandle;
};

class PxgParticleSystemDiffuseBuffer
{
public:
	PxgParticleSystemDiffuseBuffer(PxgAllocatorDesc& allocDesc);

	// Diffuse particle data
	PxgTypedCudaBuffer<PxVec4>                      diffuse_positions;
	PxgTypedCudaBuffer<PxVec4>                      diffuse_velocities;
	PxgTypedCudaBuffer<PxVec4>                      diffuse_potentials;
	PxgTypedCudaBuffer<PxU32>                       diffuse_cell_start;
	PxgTypedCudaBuffer<PxU32>                       diffuse_cell_end;
	PxgTypedCudaBuffer<PxU32>                       diffuse_grid_particle_hash;
	PxgTypedCudaBuffer<PxU32>                       diffuse_sorted_to_unsorted_mapping;
	PxgTypedCudaBuffer<PxU32>                       diffuse_unsorted_to_sorted_mapping;
	PxgTypedCudaBuffer<PxVec4>                      diffuse_origin_pos_life_time;
	PxgTypedCudaBuffer<PxVec4>                      diffuse_sorted_pos_life_time;
	PxgTypedCudaBuffer<PxVec4>                      diffuse_sorted_origin_pos_life_time;
	PxgTypedCudaBuffer<PxVec4>                      diffuse_sorted_vel;
	PxgTypedCudaBuffer<PxgParticleContactInfo>      diffuse_one_way_contacts;
	PxgTypedCudaBuffer<PxReal>                      diffuse_one_way_forces;
	PxgTypedCudaBuffer<PxNodeIndex>                 diffuse_one_way_contacts_node_indices;
	PxgTypedCudaBuffer<PxU32>                       diffuse_one_way_contact_count;
	PxgTypedCudaBuffer<PxU32>                       diffuse_particle_count;
};

}

#endif
