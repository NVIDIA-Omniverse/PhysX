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

#ifndef PX_ATTACHMENT_H
#define PX_ATTACHMENT_H

#include "PxConeLimitedConstraint.h"
#include "PxFiltering.h"
#include "PxNodeIndex.h"
#include "foundation/PxVec4.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Struct to specify attachment between a particle/vertex and a rigid
\deprecated Particle-cloth, -rigids, -attachments and -volumes have been deprecated.
*/
struct PX_DEPRECATED PxParticleRigidAttachment : public PxParticleRigidFilterPair
{
	PxParticleRigidAttachment() {}

	PxParticleRigidAttachment(const PxConeLimitedConstraint& coneLimitedConstraint, const PxVec4& localPose0):
		PxParticleRigidFilterPair(PxNodeIndex().getInd(), PxNodeIndex().getInd()),
		mLocalPose0(localPose0), 
		mConeLimitParams(coneLimitedConstraint) 
	{
	}

	PX_ALIGN(16, PxVec4 mLocalPose0); //!< local pose in body frame - except for statics, these are using world positions.
	PxConeLimitParams mConeLimitParams; //!< Parameters to specify cone constraints
};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
