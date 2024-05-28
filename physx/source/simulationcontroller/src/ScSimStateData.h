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

#ifndef SC_SIM_STATE_DATA_H
#define SC_SIM_STATE_DATA_H

#include "foundation/PxMemory.h"
#include "ScBodyCore.h"

namespace physx
{
namespace Sc
{
	struct KinematicTransform
	{
		PxTransform		targetPose;		// The body will move to this pose over the superstep following this getting set.
		PxU8			targetValid;	// User set a kinematic target.
		PxU8			pad[2];
		PxU8			type;
	};

	struct Kinematic : public KinematicTransform
	{
		// The following members buffer the original body data to restore them when switching back to dynamic body
		// (for kinematics the corresponding LowLevel properties are set to predefined values)
		PxVec3			backupInverseInertia;			// The inverse of the body space inertia tensor
		PxReal			backupInvMass;					// The inverse of the body mass
		PxReal			backupLinearDamping;			// The velocity is scaled by (1.0f - this * dt) inside integrateVelocity() every substep.
		PxReal			backupAngularDamping;
		PxReal			backupMaxAngVelSq;				// The angular velocity's magnitude is clamped to this maximum value.
		PxReal			backupMaxLinVelSq;				// The angular velocity's magnitude is clamped to this maximum value	
	};
	PX_COMPILE_TIME_ASSERT(0 == (sizeof(Kinematic) & 0x0f));

	enum VelocityModFlags
	{
		VMF_GRAVITY_DIRTY	= (1 << 0),
		VMF_ACC_DIRTY		= (1 << 1),
		VMF_VEL_DIRTY		= (1 << 2)
	};

	// Important: Struct is reset in setForcesToDefaults.
	struct VelocityMod
	{
		PxVec3	linearPerSec;		// A request to change the linear velocity by this much each second. The velocity is changed by this * dt inside integrateVelocity().
		PxU8	pad0[4];
		PxVec3	angularPerSec;
		PxU8	pad1[3];
		PxU8	type;
		PxVec3	linearPerStep;		// A request to change the linear velocity by this much the next step. The velocity is changed inside updateForces().
		PxU32	pad2;
		PxVec3	angularPerStep;
		PxU32	pad3;

		PX_FORCE_INLINE	void					clear()													{ linearPerSec = angularPerSec = linearPerStep = angularPerStep = PxVec3(0.0f); }

		PX_FORCE_INLINE	void					clearPerStep()											{ linearPerStep = angularPerStep = PxVec3(0.0f); }

		PX_FORCE_INLINE const PxVec3&			getLinearVelModPerSec()							const	{ return linearPerSec;				}
		PX_FORCE_INLINE void					accumulateLinearVelModPerSec(const PxVec3& v)			{ linearPerSec += v;				}
		PX_FORCE_INLINE void					setLinearVelModPerSec(const PxVec3& v)					{ linearPerSec = v; }
		PX_FORCE_INLINE void					clearLinearVelModPerSec()								{ linearPerSec = PxVec3(0.0f);		}

		PX_FORCE_INLINE const PxVec3&			getLinearVelModPerStep()						const	{ return linearPerStep;				}
		PX_FORCE_INLINE void					accumulateLinearVelModPerStep(const PxVec3& v)			{ linearPerStep += v;				}
		PX_FORCE_INLINE void					clearLinearVelModPerStep()								{ linearPerStep = PxVec3(0.0f);		}

		PX_FORCE_INLINE const PxVec3&			getAngularVelModPerSec()						const	{ return angularPerSec;				}
		PX_FORCE_INLINE void					accumulateAngularVelModPerSec(const PxVec3& v)			{ angularPerSec += v;				}
		PX_FORCE_INLINE void					setAngularVelModPerSec(const PxVec3& v)					{ angularPerSec = v; }
		PX_FORCE_INLINE void					clearAngularVelModPerSec()								{ angularPerSec = PxVec3(0.0f);		}

		PX_FORCE_INLINE const PxVec3&			getAngularVelModPerStep()						const	{ return angularPerStep;			}
		PX_FORCE_INLINE void					accumulateAngularVelModPerStep(const PxVec3& v)			{ angularPerStep += v;				}
		PX_FORCE_INLINE void					clearAngularVelModPerStep()								{ angularPerStep = PxVec3(0.0f);	}
	};
	PX_COMPILE_TIME_ASSERT(sizeof(VelocityMod) == sizeof(Kinematic));


	// Structure to store data either for kinematics (target pose etc.) or for dynamics (vel and accel changes).
	// note: we do not delete this object for kinematics even if no target is set.
	struct SimStateData : public PxUserAllocated	// TODO: may want to optimize the allocation of this further.
	{
		PxU8 data[sizeof(Kinematic)];

		enum Enum
		{
			eVelMod=0,
			eKine
		};

		SimStateData(){}
		SimStateData(const PxU8 type)
		{
			PxMemZero(data, sizeof(Kinematic));
			Kinematic* kine = reinterpret_cast<Kinematic*>(data);
			kine->type = type;
		}

		PX_FORCE_INLINE PxU32 getType() const { const Kinematic* kine = reinterpret_cast<const Kinematic*>(data); return kine->type;}
		PX_FORCE_INLINE bool isKine() const {return eKine == getType();}
		PX_FORCE_INLINE bool isVelMod() const {return eVelMod == getType();}

		PX_FORCE_INLINE Kinematic* getKinematicData() { Kinematic* kine = reinterpret_cast<Kinematic*>(data); PX_ASSERT(eKine == kine->type);  return kine;}
		PX_FORCE_INLINE VelocityMod* getVelocityModData() { VelocityMod* velmod = reinterpret_cast<VelocityMod*>(data); PX_ASSERT(eVelMod == velmod->type); return velmod;}
		PX_FORCE_INLINE const Kinematic* getKinematicData() const { const Kinematic* kine = reinterpret_cast<const Kinematic*>(data); PX_ASSERT(eKine == kine->type);  return kine;}
		PX_FORCE_INLINE const VelocityMod* getVelocityModData() const { const VelocityMod* velmod = reinterpret_cast<const VelocityMod*>(data); PX_ASSERT(eVelMod == velmod->type); return velmod;}
	};

	PX_FORCE_INLINE void simStateBackupAndClearBodyProperties(SimStateData* simStateData, PxsBodyCore& core)
	{
		PX_ASSERT(simStateData && simStateData->isKine());
		Kinematic* kine = simStateData->getKinematicData();

		kine->backupLinearDamping = core.linearDamping;
		kine->backupAngularDamping = core.angularDamping;
		kine->backupInverseInertia = core.inverseInertia;
		kine->backupInvMass = core.inverseMass;
		kine->backupMaxAngVelSq = core.maxAngularVelocitySq;
		kine->backupMaxLinVelSq = core.maxLinearVelocitySq;

		core.inverseMass = 0.0f;
		core.inverseInertia = PxVec3(0.0f);
		core.linearDamping = 0.0f;
		core.angularDamping = 0.0f;
		core.maxAngularVelocitySq = PX_MAX_REAL;
		core.maxLinearVelocitySq = PX_MAX_REAL;
	}
		
	PX_FORCE_INLINE void simStateRestoreBodyProperties(const SimStateData* simStateData, PxsBodyCore& core)
	{
		PX_ASSERT(simStateData && simStateData->isKine());
		const Kinematic* kine = simStateData->getKinematicData();
		core.inverseMass = kine->backupInvMass;
		core.inverseInertia = kine->backupInverseInertia;
		core.linearDamping = kine->backupLinearDamping;
		core.angularDamping = kine->backupAngularDamping;
		core.maxAngularVelocitySq = kine->backupMaxAngVelSq;
		core.maxLinearVelocitySq = kine->backupMaxLinVelSq;
	}

	PX_FORCE_INLINE void simStateClearVelMod(SimStateData* simStateData)
	{
		if (simStateData && simStateData->isVelMod())
		{
			VelocityMod* velmod = simStateData->getVelocityModData();
			velmod->clear();
		}
	}

	PX_FORCE_INLINE bool simStateGetKinematicTarget(const SimStateData* simStateData, PxTransform& p)
	{
		if (simStateData && simStateData->isKine() && simStateData->getKinematicData()->targetValid)
		{
			p = simStateData->getKinematicData()->targetPose;
			return true;
		}
		else
			return false;
	}

	PX_FORCE_INLINE bool simStateGetHasValidKinematicTarget(const SimStateData* simStateData)
	{
		PX_ASSERT(!simStateData || simStateData->isKine());
		return simStateData && simStateData->isKine() && simStateData->getKinematicData()->targetValid;
	}

	PX_FORCE_INLINE void simStateSetKinematicTarget(SimStateData* simStateData, const PxTransform& p)
	{
		PX_ASSERT(simStateData && simStateData->isKine());
		// setting the kinematic target is only allowed if the body is part of a scene, at which point the
		// mSimStateData buffer must exist
		Kinematic* kine = simStateData->getKinematicData();
		kine->targetPose = p;
		kine->targetValid = 1;
	}

	PX_FORCE_INLINE void simStateInvalidateKinematicTarget(SimStateData* simStateData)
	{
		PX_ASSERT(simStateData && simStateData->isKine());
		simStateData->getKinematicData()->targetValid = 0;
	}
} // namespace Sc

}  // namespace physx

#endif

