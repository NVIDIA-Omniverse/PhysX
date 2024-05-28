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

#ifndef EXT_OMNI_PVD_REGISTRATION_DATA_H
#define EXT_OMNI_PVD_REGISTRATION_DATA_H


#if PX_SUPPORT_OMNI_PVD

#include "../pvdruntime/include/OmniPvdWriter.h"

#include "extensions/PxD6Joint.h"  // for PxD6Motion
#include "extensions/PxDistanceJoint.h"  // for PxDistanceJointFlags
#include "extensions/PxPrismaticJoint.h"  // for PxPrismaticJointFlags
#include "extensions/PxRevoluteJoint.h"  // for PxRevoluteJointFlags
#include "extensions/PxSphericalJoint.h"  // for PxSphericalJointFlags
#include "extensions/PxCustomGeometryExt.h"  // for PxCustomGeometryExtBaseConvexCallbacks etc.


namespace physx
{

class PxContactJoint;
class PxFixedJoint;
class PxGearJoint;
class PxRackAndPinionJoint;


namespace Ext
{

struct OmniPvdPxExtensionsRegistrationData
{
	void registerData(OmniPvdWriter&);

	// auto-generate members and setter methods from object definition file
#include "omnipvd/CmOmniPvdAutoGenCreateRegistrationStruct.h"
#include "OmniPvdPxExtensionsTypes.h"
#include "omnipvd/CmOmniPvdAutoGenClearDefines.h"

};

}
}

#endif  // PX_SUPPORT_OMNI_PVD

#endif  // EXT_OMNI_PVD_REGISTRATION_DATA_H
