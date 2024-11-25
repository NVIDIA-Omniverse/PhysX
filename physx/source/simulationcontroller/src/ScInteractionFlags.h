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

#ifndef SC_INTERACTION_FLAGS_H
#define SC_INTERACTION_FLAGS_H


namespace physx
{

namespace Sc
{
	struct InteractionFlag	// PT: TODO: use PxFlags
	{
		enum Enum
		{
			eRB_ELEMENT			= (1 << 0), // Interactions between rigid body shapes
			eCONSTRAINT			= (1 << 1),
			eFILTERABLE			= (1 << 2), // Interactions that go through the filter code
			eIN_DIRTY_LIST		= (1 << 3),	// The interaction is in the dirty list
			eIS_FILTER_PAIR		= (1 << 4),	// The interaction is tracked by the filter callback mechanism
			eIS_ACTIVE			= (1 << 5)
		};
	};

	struct InteractionDirtyFlag
	{
		enum Enum
		{
			eFILTER_STATE		= (1 << 0), // All changes filtering related
			eBODY_KINEMATIC		= (1 << 1) | eFILTER_STATE,  // A transition between dynamic and kinematic (and vice versa) require a refiltering
			eDOMINANCE			= (1 << 2),
			eREST_OFFSET		= (1 << 3),
			eVISUALIZATION		= (1 << 4)
		};
	};


} // namespace Sc


} // namespace physx


#endif

