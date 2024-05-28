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

#include "DyAllocator.h"

namespace physx
{
namespace Dy
{
// PT: the various error messages we used here look the same but they are slightly different. We could unify this really.

const char* gConstraintDataErrorMsg_Null_Joints =
	"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
	"Either accept joints detaching/exploding or increase buffer size allocated for constraint prep by increasing PxSceneDesc::maxNbContactDataBlocks.";

const char* gConstraintDataErrorMsg_TooLarge_Joints =
	"Attempting to allocate more than 16K of constraint data. "
	"Either accept joints detaching/exploding or simplify constraints.";

const char* gConstraintDataErrorMsg_Null_Contacts =
	"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
	"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.";

const char* gConstraintDataErrorMsg_TooLarge_Contacts =
	"Attempting to allocate more than 16K of contact data for a single contact pair in constraint prep. "
	"Either accept dropped contacts or simplify collision geometry.";

const char* gFrictionDataErrorMsg_TooLarge =
	"Attempting to allocate more than 16K of friction data for a single contact pair in constraint prep. "
	"Either accept dropped contacts or simplify collision geometry.";
}
}
