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

#ifndef DY_CONTACT_PREP_SHARED_H
#define DY_CONTACT_PREP_SHARED_H
     
#include "foundation/PxPreprocessor.h"
#include "foundation/PxVecMath.h"
#include "PxMaterial.h"
#include "DyContactPrep.h"
#include "DyCorrelationBuffer.h"
#include "DyArticulationContactPrep.h"
#include "PxsContactManager.h"
#include "PxsContactManagerState.h"
#include "PxcNpContactPrepShared.h"

namespace physx
{
namespace Dy
{
template<class PxSolverContactDescT>
PX_FORCE_INLINE Sc::ShapeInteraction* getInteraction(const PxSolverContactDescT& desc)
{
	return reinterpret_cast<Sc::ShapeInteraction*>(desc.shapeInteraction);
}

PX_FORCE_INLINE bool pointsAreClose(const PxTransform& body1ToBody0,
									const PxVec3& localAnchor0, const PxVec3& localAnchor1,
									const PxVec3& axis, float correlDist)
{
	const PxVec3 body0PatchPoint1 = body1ToBody0.transform(localAnchor1);

	return PxAbs((localAnchor0 - body0PatchPoint1).dot(axis))<correlDist;
}

PX_FORCE_INLINE bool isSeparated(const FrictionPatch& patch, const PxTransform& body1ToBody0, const PxReal correlationDistance)
{
	PX_ASSERT(patch.anchorCount <= 2);
	for(PxU32 a = 0; a < patch.anchorCount; ++a)
	{
		if(!pointsAreClose(body1ToBody0, patch.body0Anchors[a], patch.body1Anchors[a], patch.body0Normal, correlationDistance))
			return true;
	}
	return false;
}

inline bool getFrictionPatches(CorrelationBuffer& c,
						const PxU8* frictionCookie,
						PxU32 frictionPatchCount,
						const PxTransform& bodyFrame0,
						const PxTransform& bodyFrame1,
						PxReal correlationDistance)
{
	if(frictionCookie == NULL || frictionPatchCount == 0)
		return true;

	//KS - this is now DMA'd inside the shader so we don't need to immediate DMA it here
	const FrictionPatch* patches = reinterpret_cast<const FrictionPatch*>(frictionCookie);

	//Try working out relative transforms! TODO - can we compute this lazily for the first friction patch
	bool evaluated = false;
	PxTransform body1ToBody0;

	while(frictionPatchCount--)
	{
		PxPrefetchLine(patches,128);
		const FrictionPatch& patch = *patches++;
		PX_ASSERT (patch.broken == 0 || patch.broken == 1);
		if(!patch.broken)
		{
			// if the eDISABLE_STRONG_FRICTION flag is there we need to blow away the previous frame's friction correlation, so
			// that we can associate each friction anchor with a target velocity. So we lose strong friction.
			if(patch.anchorCount != 0 && !(patch.materialFlags & PxMaterialFlag::eDISABLE_STRONG_FRICTION))
			{
				PX_ASSERT(patch.anchorCount <= 2);
			
				if(!evaluated)
				{
					body1ToBody0 = bodyFrame0.transformInv(bodyFrame1);
					evaluated = true;
				}

				if(patch.body0Normal.dot(body1ToBody0.rotate(patch.body1Normal)) > PXC_SAME_NORMAL)
				{
					if(!isSeparated(patch, body1ToBody0, correlationDistance))
					{
						if(c.frictionPatchCount == CorrelationBuffer::MAX_FRICTION_PATCHES)
							return false;
						{
							c.contactID[c.frictionPatchCount][0] = 0xffff;
							c.contactID[c.frictionPatchCount][1] = 0xffff;
							//Rotate the contact normal into world space
							c.frictionPatchWorldNormal[c.frictionPatchCount] = bodyFrame0.rotate(patch.body0Normal);
							c.frictionPatchContactCounts[c.frictionPatchCount] = 0;
							c.patchBounds[c.frictionPatchCount].setEmpty();
							c.correlationListHeads[c.frictionPatchCount] = CorrelationBuffer::LIST_END;
							PxMemCopy(&c.frictionPatches[c.frictionPatchCount++], &patch, sizeof(FrictionPatch));
						}
					}
				}
			}
		}
	}
	return true;
}

PX_FORCE_INLINE PxU32 extractContacts(PxContactBuffer& buffer, PxsContactManagerOutput& npOutput, bool& hasMaxImpulse, bool& hasTargetVelocity,
							 PxReal& invMassScale0, PxReal& invMassScale1, PxReal& invInertiaScale0, PxReal& invInertiaScale1, PxReal defaultMaxImpulse)
{
	PxContactStreamIterator iter(npOutput.contactPatches, npOutput.contactPoints, npOutput.getInternalFaceIndice(), npOutput.nbPatches, npOutput.nbContacts);

	PxU32 numContacts = buffer.count, origContactCount = buffer.count;
	if(!iter.forceNoResponse)
	{
		invMassScale0 = iter.getInvMassScale0();
		invMassScale1 = iter.getInvMassScale1();
		invInertiaScale0 = iter.getInvInertiaScale0();
		invInertiaScale1 = iter.getInvInertiaScale1();
		hasMaxImpulse = (iter.patch->internalFlags & PxContactPatch::eHAS_MAX_IMPULSE) != 0;
		hasTargetVelocity = (iter.patch->internalFlags & PxContactPatch::eHAS_TARGET_VELOCITY) != 0;

		while(iter.hasNextPatch())
		{
			iter.nextPatch();
			while(iter.hasNextContact())
			{
				iter.nextContact();
				PxPrefetchLine(iter.contact, 128);
				PxPrefetchLine(&buffer.contacts[numContacts], 128);
				PxReal maxImpulse = hasMaxImpulse ? iter.getMaxImpulse() : defaultMaxImpulse;
				if(maxImpulse != 0.f)
				{
					PX_ASSERT(numContacts < PxContactBuffer::MAX_CONTACTS);
					buffer.contacts[numContacts].normal = iter.getContactNormal();
					PX_ASSERT(PxAbs(buffer.contacts[numContacts].normal.magnitude() - 1) < 1e-3f);
					buffer.contacts[numContacts].point = iter.getContactPoint();
					buffer.contacts[numContacts].separation = iter.getSeparation();
					//KS - we use the face indices to cache the material indices and flags - avoids bloating the PxContact structure
					PX_ASSERT(iter.getMaterialFlags() <= PX_MAX_U8);
					buffer.contacts[numContacts].materialFlags = PxU8(iter.getMaterialFlags());
					buffer.contacts[numContacts].maxImpulse = maxImpulse;
					buffer.contacts[numContacts].staticFriction = iter.getStaticFriction();
					buffer.contacts[numContacts].dynamicFriction = iter.getDynamicFriction();
					buffer.contacts[numContacts].restitution = iter.getRestitution();
					buffer.contacts[numContacts].damping = iter.getDamping();
					const PxVec3& targetVel = iter.getTargetVel();
					buffer.contacts[numContacts].targetVel = targetVel;
					++numContacts;
				}
			}
		}
	}
	const PxU32 contactCount = numContacts - origContactCount;
	buffer.count = numContacts;
	return contactCount;
}

struct CorrelationListIterator
{
	CorrelationBuffer& buffer;
	PxU32 currPatch;
	PxU32 currContact;

	CorrelationListIterator(CorrelationBuffer& correlationBuffer, PxU32 startPatch) : buffer(correlationBuffer)
	{
		//We need to force us to advance the correlation buffer to the first available contact (if one exists)
		PxU32 newPatch = startPatch, newContact = 0;

		while(newPatch != CorrelationBuffer::LIST_END && newContact == buffer.contactPatches[newPatch].count)
		{
			newPatch = buffer.contactPatches[newPatch].next;
			newContact = 0;
		}

		currPatch = newPatch;
		currContact = newContact;
	}

	//Returns true if it has another contact pre-loaded. Returns false otherwise
	PX_FORCE_INLINE bool hasNextContact()	const
	{
		return (currPatch != CorrelationBuffer::LIST_END && currContact < buffer.contactPatches[currPatch].count);
	}

	inline void nextContact(PxU32& patch, PxU32& contact)
	{
		PX_ASSERT(currPatch != CorrelationBuffer::LIST_END);
		PX_ASSERT(currContact < buffer.contactPatches[currPatch].count);

		patch = currPatch;
		contact = currContact;
		PxU32 newPatch = currPatch, newContact = currContact + 1;

		while(newPatch != CorrelationBuffer::LIST_END && newContact == buffer.contactPatches[newPatch].count)
		{
			newPatch = buffer.contactPatches[newPatch].next;
			newContact = 0;
		}

		currPatch = newPatch;
		currContact = newContact;
	}

private:
	CorrelationListIterator& operator=(const CorrelationListIterator&);

};

namespace {

// Decides whether or not the damper should be turned on. We don't want damping if the contact
// is not expected to be closed this step because the damper can produce repulsive forces
// even before the contact is closed.
PX_FORCE_INLINE FloatV computeCompliantDamping(const BoolVArg isSeparated, const BoolVArg collidingWithVrel,
                                               const FloatVArg damping)
{
	const FloatV zero = FZero();
	const FloatV dampingIfEnabled = FSel(BAndNot(isSeparated, collidingWithVrel), zero, damping);
	return dampingIfEnabled;
}

PX_FORCE_INLINE void computeCompliantContactCoefficients(
    const FloatVArg dt, const FloatVArg restitution, const FloatVArg damping, const FloatVArg recipResponse,
    const FloatVArg unitResponse, const FloatVArg penetration, const FloatVArg targetVelocity,
    const BoolVArg accelerationSpring, const BoolVArg isSeparated, const BoolVArg collidingWithVrel,
    FloatVArg velMultiplier, FloatVArg impulseMultiplier, FloatVArg unbiasedErr, FloatVArg biasedErr)
{
	// negative restitution interpreted as spring stiffness for compliant contact
	const FloatV nrdt = FMul(dt, restitution); // -dt*stiffness

	const FloatV one = FOne();
	const FloatV massIfAccelElseOne = FSel(accelerationSpring, recipResponse, one);
	const FloatV dampingIfEnabled = computeCompliantDamping(isSeparated, collidingWithVrel, damping);

	const FloatV a = FMul(dt, FSub(dampingIfEnabled, nrdt));                  // a = dt * (damping + dt*stiffness)
	const FloatV b = FMul(FNeg(FMul(nrdt, penetration)), massIfAccelElseOne); // b = dt * stiffness * penetration
	const FloatV x = FRecip(FScaleAdd(a, FSel(accelerationSpring, one, unitResponse), one));
	const FloatV scaledBias = FMul(x, b);
	// FloatV scaledBias = FSel(isSeparated, FNeg(invStepDt), FDiv(FMul(nrdt, FMul(x, unitResponse)), velMultiplier));

	velMultiplier = FMul(FMul(x, a), massIfAccelElseOne);
	impulseMultiplier = FSub(one, x);
	unbiasedErr = biasedErr = FScaleAdd(targetVelocity, velMultiplier, FNeg(scaledBias));
}

PX_FORCE_INLINE void computeCompliantContactCoefficientsTGS(const FloatVArg dt, const FloatVArg restitution,
                                                            const FloatVArg damping, const FloatVArg recipResponse,
                                                            const FloatVArg unitResponse,
                                                            const BoolVArg accelerationSpring,
                                                            const BoolVArg isSeparated, const BoolVArg collidingWithVrel,
                                                            FloatVArg velMultiplier, FloatVArg biasCoeff)
{
	const FloatV nrdt = FMul(dt, restitution);      // -dt * stiffness
	const FloatV dampingIfEnabled = computeCompliantDamping(isSeparated, collidingWithVrel, damping);
	const FloatV a = FMul(dt, FSub(dampingIfEnabled, nrdt)); // a = dt * (damping + dt * stiffness)

	const FloatV one = FOne();
	const FloatV massIfAccelElseOne = FSel(accelerationSpring, recipResponse, one);
	const FloatV oneIfAccelElseR = FSel(accelerationSpring, one, unitResponse);

	const FloatV x = FRecip(FScaleAdd(a, oneIfAccelElseR, one));

	velMultiplier = FMul(FMul(x, a), massIfAccelElseOne);
	// biasCoeff = FSel(isSeparated, FNeg(invStepDt), FDiv(FMul(nrdt, FMul(x, unitResponse)), velMultiplier));
	// biasCoeff includes the unit response s.t. velDeltaFromPosError = separation*biasCoeff
	biasCoeff = FMul(nrdt, FMul(x, oneIfAccelElseR));
}

} // anonymous namespace

// PGS rigid-rigid or rigid-static normal contact prepping code
PX_FORCE_INLINE void constructContactConstraint(const Mat33V& invSqrtInertia0, const Mat33V& invSqrtInertia1, const FloatVArg invMassNorLenSq0, 
	const FloatVArg invMassNorLenSq1, const FloatVArg angD0, const FloatVArg angD1, const Vec3VArg bodyFrame0p, const Vec3VArg bodyFrame1p,
	const Vec3VArg normal, const FloatVArg norVel, const VecCrossV& norCross, const Vec3VArg angVel0, const Vec3VArg angVel1,
	const FloatVArg invDt, const FloatVArg invDtp8, const FloatVArg dt, const FloatVArg restDistance, const FloatVArg maxPenBias, const FloatVArg restitution,
	const FloatVArg bounceThreshold, const PxContactPoint& contact, SolverContactPoint& solverContact,
	const FloatVArg ccdMaxSeparation, const Vec3VArg solverOffsetSlop, const FloatVArg damping, const BoolVArg accelerationSpring)
{
	const FloatV zero = FZero();
	const Vec3V point = V3LoadA(contact.point);
	const FloatV separation = FLoad(contact.separation);

	const FloatV cTargetVel = V3Dot(normal, V3LoadA(contact.targetVel));

	const Vec3V ra = V3Sub(point, bodyFrame0p);
	const Vec3V rb = V3Sub(point, bodyFrame1p);

	/*ra = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(ra)), V3Zero(), ra);
	rb = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rb)), V3Zero(), rb);*/

	Vec3V raXn = V3Cross(ra, norCross);
	Vec3V rbXn = V3Cross(rb, norCross);

	FloatV vRelAng = FSub(V3Dot(raXn, angVel0), V3Dot(rbXn, angVel1));

	const Vec3V slop = V3Scale(solverOffsetSlop, FMax(FSel(FIsEq(norVel, zero), FMax(), FDiv(vRelAng, norVel)), FOne()));

	raXn = V3Sel(V3IsGrtr(slop, V3Abs(raXn)), V3Zero(), raXn);
	rbXn = V3Sel(V3IsGrtr(slop, V3Abs(rbXn)), V3Zero(), rbXn);

	vRelAng = FSub(V3Dot(raXn, angVel0), V3Dot(rbXn, angVel1));

	const FloatV vrel = FAdd(norVel, vRelAng);

	const Vec3V raXnSqrtInertia = M33MulV3(invSqrtInertia0, raXn);
	const Vec3V rbXnSqrtInertia = M33MulV3(invSqrtInertia1, rbXn);				

	const FloatV resp0 = FAdd(invMassNorLenSq0, FMul(V3Dot(raXnSqrtInertia, raXnSqrtInertia), angD0));
	const FloatV resp1 = FSub(FMul(V3Dot(rbXnSqrtInertia, rbXnSqrtInertia), angD1), invMassNorLenSq1);

	const FloatV unitResponse = FAdd(resp0, resp1);

	const FloatV penetration = FSub(separation, restDistance);
	const FloatV penetrationInvDt = FMul(penetration, invDt);
	const BoolV isSeparated = FIsGrtrOrEq(penetration, zero);

	const BoolV collidingWithVrel = FIsGrtr(FNeg(vrel), penetrationInvDt); // true if pen + dt*vrel < 0
	const BoolV isGreater2 = BAnd(BAnd(FIsGrtr(restitution, zero), FIsGrtr(bounceThreshold, vrel)), collidingWithVrel);

	FloatV targetVelocity = FAdd(cTargetVel, FSel(isGreater2, FMul(FNeg(vrel), restitution), zero));

	//Note - we add on the initial target velocity
	targetVelocity = FSub(targetVelocity, vrel);

	const FloatV recipResponse = FSel(FIsGrtr(unitResponse, zero), FRecip(unitResponse), zero);

	FloatV biasedErr, unbiasedErr;
	FloatV velMultiplier, impulseMultiplier;

	if (FAllGrtr(zero, restitution))
	{
		computeCompliantContactCoefficients(dt, restitution, damping, recipResponse, unitResponse, penetration,
		                                    targetVelocity, accelerationSpring, isSeparated, collidingWithVrel,
		                                    velMultiplier, impulseMultiplier, unbiasedErr, biasedErr);
	}
	else
	{
		velMultiplier = recipResponse;

		// Divide bias term by dt and additionally scale it down if it's in penetration
		// Do not scale it if it's not in penetration, otherwise we falsely act on contacts that are still
		// sufficiently far away.
		const FloatV penetrationInvDtScaled = FSel(isSeparated, penetrationInvDt, FMul(penetration, invDtp8));

		FloatV scaledBias = FMul(velMultiplier, FMax(maxPenBias, penetrationInvDtScaled));

		const BoolV ccdSeparationCondition = FIsGrtrOrEq(ccdMaxSeparation, penetration);

		scaledBias = FSel(BAnd(ccdSeparationCondition, isGreater2), zero, scaledBias);

		impulseMultiplier = FLoad(1.0f);

		biasedErr = FScaleAdd(targetVelocity, velMultiplier, FNeg(scaledBias));
		unbiasedErr = FScaleAdd(targetVelocity, velMultiplier, FSel(isGreater2, zero, FNeg(FMax(scaledBias, zero))));
	}

	//const FloatV unbiasedErr = FScaleAdd(targetVelocity, velMultiplier, FNeg(FMax(scaledBias, zero)));

	FStore(biasedErr, &solverContact.biasedErr);
	FStore(unbiasedErr, &solverContact.unbiasedErr);
	FStore(impulseMultiplier, &solverContact.impulseMultiplier);

	solverContact.raXn_velMultiplierW = V4SetW(Vec4V_From_Vec3V(raXnSqrtInertia), velMultiplier);
	solverContact.rbXn_maxImpulseW = V4SetW(Vec4V_From_Vec3V(rbXnSqrtInertia), FLoad(contact.maxImpulse));
}

}
}

#endif
