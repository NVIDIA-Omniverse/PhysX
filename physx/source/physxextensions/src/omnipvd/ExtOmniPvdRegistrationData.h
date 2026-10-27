// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
