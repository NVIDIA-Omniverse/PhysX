// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuContactMethodImpl.h"

using namespace physx;

bool Gu::contactCustomGeometryGeometry(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	const PxCustomGeometry& customGeom = checkedCast<PxCustomGeometry>(shape0);
	const PxGeometry& otherGeom = shape1;

	customGeom.callbacks->generateContacts(customGeom, otherGeom, transform0, transform1,
											params.mContactDistance, params.mMeshContactMargin, params.mToleranceLength,
											contactBuffer);
	return true;
}

bool Gu::contactGeometryCustomGeometry(GU_CONTACT_METHOD_ARGS)
{
	bool res = contactCustomGeometryGeometry(shape1, shape0, transform1, transform0, params, cache, contactBuffer, renderOutput);

	for (PxU32 i = 0; i < contactBuffer.count; ++i)
		contactBuffer.contacts[i].normal = -contactBuffer.contacts[i].normal;

	return res;
}
