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

#ifndef GU_PRUNER_PAYLOAD_H
#define GU_PRUNER_PAYLOAD_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/Px.h"

namespace physx
{
	class PxBounds3;

namespace Gu
{
	// PT: anonymous payload structure used by the pruners. This is similar in spirit to a userData pointer.
	struct PrunerPayload
	{
		size_t data[2];	// Enough space for two arbitrary pointers

		PX_FORCE_INLINE	bool operator == (const PrunerPayload& other) const
		{
			return (data[0] == other.data[0]) && (data[1] == other.data[1]);
		}
	};

	// PT: pointers to internal data associated with a pruner payload. The lifetime of these pointers
	// is usually limited and they should be used immediately after retrieval.
	struct PrunerPayloadData
	{
		PxBounds3*		mBounds;	// Pointer to internal bounds.
		PxTransform*	mTransform;	// Pointer to internal transform, or NULL.
	};

	// PT: called for each removed payload. Gives users a chance to cleanup their data
	// structures without duplicating the pruner-data to payload mapping on their side.
	struct PrunerPayloadRemovalCallback
	{
						PrunerPayloadRemovalCallback()		{}
		virtual			~PrunerPayloadRemovalCallback()		{}

		virtual void	invoke(PxU32 nbRemoved, const PrunerPayload* removed) = 0;
	};
}
}

#endif
