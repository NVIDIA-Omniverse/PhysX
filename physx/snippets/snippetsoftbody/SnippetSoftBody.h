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

#ifndef PHYSX_SNIPPET_SOFTBODY_H
#define PHYSX_SNIPPET_SOFTBODY_H

#include "PxPhysicsAPI.h"
#include <vector>

class SoftBody
{
public:
	SoftBody(physx::PxSoftBody* softBody, physx::PxBuffer* positionInvMass) :
		mSoftBody(softBody),
		mPositionInvMass(positionInvMass)
	{ }

	~SoftBody()
	{
	}

	void release()
	{
		if (mSoftBody)
			mSoftBody->release();
		if (mPositionInvMass)
			mPositionInvMass->release();
	}


	void copyDeformedVerticesFromGPU()
	{	
		mSoftBody->readData(physx::PxSoftBodyData::ePOSITION_INVMASS, *mPositionInvMass);

		physx::PxTetrahedronMesh* mTetMesh = mSoftBody->getCollisionMesh();
		physx::PxVec4* positionsInvMass = static_cast<physx::PxVec4*>(mPositionInvMass->map());
		
		mPositionsInvMass.resize(mTetMesh->getNbVertices());
		memcpy(mPositionsInvMass.begin(), positionsInvMass, mTetMesh->getNbVertices() * sizeof(physx::PxVec4));
		
		mPositionInvMass->unmap();	
	}

	physx::PxArray<physx::PxVec4> mPositionsInvMass;
	physx::PxSoftBody* mSoftBody;
	physx::PxBuffer* mPositionInvMass;
};

#endif
