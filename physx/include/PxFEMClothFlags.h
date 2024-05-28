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

#ifndef PX_PHYSICS_FEM_CLOTH_FLAGS_H
#define PX_PHYSICS_FEM_CLOTH_FLAGS_H

#include "foundation/PxFlags.h"
#include "foundation/PxSimpleTypes.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Identifies input and output buffers for PxFEMCloth.
*/
struct PxFEMClothDataFlag
{
	enum Enum
	{
		eNONE = 0,
		ePOSITION_INVMASS = 1 << 0,
		eVELOCITY = 1 << 1,
		eREST_POSITION = 1 << 2,
		eALL = ePOSITION_INVMASS | eVELOCITY | eREST_POSITION
	};
};

typedef PxFlags<PxFEMClothDataFlag::Enum, PxU32> PxFEMClothDataFlags;

struct PxFEMClothFlag
{
	enum Enum
	{
		eDISABLE_SELF_COLLISION = 1 << 0,
		eUSE_ANISOTROPIC_CLOTH = 1 << 1,         // 0: use isotropic model, 1: use anistropic model
		eENABLE_FLATTENING = 1 << 2				 // 0: query rest bending angle from rest shape, 1: use zero rest bending angle
	};
};

typedef PxFlags<PxFEMClothFlag::Enum, PxU32> PxFEMClothFlags;

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
