// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXC_NP_CONTACT_PREP_SHARED_H
#define PXC_NP_CONTACT_PREP_SHARED_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
class PxcNpThreadContext;
struct PxsMaterialInfo;
class PxsMaterialManager;
class PxsConstraintBlockManager;
class PxcConstraintBlockStream;
struct PxContactPoint;
struct PxcDataStreamPool;

PX_FORCE_INLINE PxU32 computeAlignedSize(PxU32 size)
{
	return (size + 0xf) & 0xfffffff0;
}

static const PxReal PXC_SAME_NORMAL = 0.999f; //Around 6 degrees

PxU32 writeCompressedContact(const PxContactPoint* const PX_RESTRICT contactPoints, const PxU32 numContactPoints, PxcNpThreadContext* threadContext,
								PxU16& writtenContactCount, PxU8*& outContactPatches, PxU8*& outContactPoints, PxU16& compressedContactSize, PxReal*& contactForces, PxU32 contactForceByteSize,
								PxU8*& outFrictionPatches, PxcDataStreamPool* frictionPatchesStreamPool,
								const PxsMaterialManager* materialManager, bool hasModifiableContacts, bool forceNoResponse, const PxsMaterialInfo* PX_RESTRICT pMaterial, PxU8& numPatches,
								PxU32 additionalHeaderSize = 0, PxsConstraintBlockManager* manager = NULL, PxcConstraintBlockStream* blockStream = NULL, bool insertAveragePoint = false,
								PxcDataStreamPool* pool = NULL, PxcDataStreamPool* patchStreamPool = NULL, PxcDataStreamPool* forcePool = NULL, const bool isMeshType = false);

}

#endif
