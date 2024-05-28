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

#include "geomutils/PxContactBuffer.h"
#include "GuVecBox.h"
#include "GuBounds.h"
#include "GuContactMethodImpl.h"
#include "GuPCMShapeConvex.h"
#include "GuPCMContactGen.h"

using namespace physx;
using namespace aos;
using namespace Gu;

static bool pcmContactCustomGeometryGeometry(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	const PxCustomGeometry& customGeom = checkedCast<PxCustomGeometry>(shape0);
	const PxGeometry& otherGeom = shape1;

	float breakingThreshold = 0.01f * params.mToleranceLength;
	bool usePCM = customGeom.callbacks->usePersistentContactManifold(customGeom, breakingThreshold);
	if (otherGeom.getType() == PxGeometryType::eCUSTOM)
	{
		float breakingThreshold1 = breakingThreshold;
		if (checkedCast<PxCustomGeometry>(shape1).callbacks->usePersistentContactManifold(otherGeom, breakingThreshold1))
		{
			breakingThreshold = PxMin(breakingThreshold, breakingThreshold1);
			usePCM = true;
		}
	}
	else if (otherGeom.getType() > PxGeometryType::eCONVEXMESH)
	{
		usePCM = true;
	}

	if (usePCM)
	{
		MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

		const PxTransformV transf0 = loadTransformA(transform0), transf1 = loadTransformA(transform1);
		const PxTransformV curRTrans = transf1.transformInv(transf0);
		const FloatV cos5 = FLoad(0.9962f); // Cos of 5 degrees
		if (multiManifold.invalidate(curRTrans, FLoad(breakingThreshold)))
		{
			customGeom.callbacks->generateContacts(customGeom, otherGeom, transform0, transform1,
				params.mContactDistance, params.mMeshContactMargin, params.mToleranceLength,
				contactBuffer);

			multiManifold.initialize();
			multiManifold.setRelativeTransform(curRTrans);

			for (PxU32 ci = 0; ci < contactBuffer.count; ++ci)
			{
				const PxContactPoint& c = contactBuffer.contacts[ci];
				const Vec3V cP = V3LoadU(c.point);
				const Vec3V cN = V3LoadU(c.normal);
				const FloatV cD = FLoad(c.separation);

				SinglePersistentContactManifold* manifold = NULL;
				for (PxU8 mi = 0; mi < multiManifold.mNumManifolds; ++mi)
				{
					const Vec3V mN = multiManifold.mManifolds[mi].getWorldNormal(transf1);
					const BoolV cmp = FIsGrtr(V3Dot(cN, mN), cos5);
					if (BAllEqTTTT(cmp))
					{
						manifold = &multiManifold.mManifolds[mi];
						break;
					}
				}

				if (!manifold)
				{
					manifold = multiManifold.getEmptyManifold();
					if (manifold) multiManifold.mNumManifolds++;
				}

				if (manifold)
				{
					SinglePersistentContactManifold& m = *manifold;
					MeshPersistentContact pc;
					pc.mLocalPointA = transf0.transformInv(cP);
					pc.mLocalPointB = transf1.transformInv(cP);
					pc.mLocalNormalPen = V4SetW(transf1.rotateInv(cN), cD);
					pc.mFaceIndex = c.internalFaceIndex1;
					m.mContactPoints[m.mNumContacts++] = pc;
					m.mNumContacts = SinglePersistentContactManifold::reduceContacts(m.mContactPoints, m.mNumContacts);
				}
			}

			contactBuffer.count = 0;
		}

#if PCM_LOW_LEVEL_DEBUG
		multiManifold.drawManifold(*renderOutput, transf0, transf1);
#endif
		return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, transf1);
	}

	return customGeom.callbacks->generateContacts(customGeom, otherGeom, transform0, transform1,
		params.mContactDistance, params.mMeshContactMargin, params.mToleranceLength,
		contactBuffer);
}

bool Gu::pcmContactGeometryCustomGeometry(GU_CONTACT_METHOD_ARGS)
{
	bool res = pcmContactCustomGeometryGeometry(shape1, shape0, transform1, transform0, params, cache, contactBuffer, renderOutput);

	for (PxU32 i = 0; i < contactBuffer.count; ++i)
		contactBuffer.contacts[i].normal = -contactBuffer.contacts[i].normal;

	return res;
}

