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

#ifndef SQ_PRUNER_DATA_H
#define SQ_PRUNER_DATA_H


#include "SqTypedef.h"

// PT: SQ-API LEVEL 2 (Level 1 = SqPruner.h)
// PT: this file is part of a "high-level" set of files within Sq. The SqPruner API doesn't rely on them.
// PT: this should really be at Np level but moving it to Sq allows us to share it.

namespace physx
{
namespace Sq
{
	struct PruningIndex
	{
		enum Enum
		{
			eSTATIC		= 0,	// PT: must match PX_SCENE_PRUNER_STATIC
			eDYNAMIC	= 1,	// PT: must match PX_SCENE_PRUNER_DYNAMIC

			eCOUNT		= 2
		};
	};

	PX_FORCE_INLINE PrunerData createPrunerData(PxU32 index, Gu::PrunerHandle h)	{ return PrunerData((h << 1) | index);	}
	PX_FORCE_INLINE PxU32 getPrunerIndex(PrunerData data)							{ return PxU32(data & 1);				}
	PX_FORCE_INLINE Gu::PrunerHandle getPrunerHandle(PrunerData data)				{ return Gu::PrunerHandle(data >> 1);	}
}
}

#endif
