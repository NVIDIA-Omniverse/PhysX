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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SC_BODY_CORE_H
#define SC_BODY_CORE_H

#include "foundation/PxTransform.h"
#include "ScRigidCore.h"
#include "PxRigidDynamic.h"
#include "PxvDynamics.h"
#include "PxvConfig.h"

namespace physx
{
namespace Sc
{
	class BodySim;

	class BodyCore : public RigidCore
	{
	public:
// PX_SERIALIZATION
											BodyCore(const PxEMPTY) : RigidCore(PxEmpty), mCore(PxEmpty) {}
			static			void			getBinaryMetaData(PxOutputStream& stream);
							void			restoreDynamicData();

//~PX_SERIALIZATION
											BodyCore(PxActorType::Enum type, const PxTransform& bodyPose);
											~BodyCore();

		//---------------------------------------------------------------------------------
		// External API
		//---------------------------------------------------------------------------------
		PX_FORCE_INLINE	const PxTransform&	getBody2World()				const	{ return mCore.body2World;			}
						void				setBody2World(const PxTransform& p);

						void				setCMassLocalPose(const PxTransform& body2Actor);

		PX_FORCE_INLINE	const PxVec3&		getLinearVelocity()			const	{ return mCore.linearVelocity;		}
						void				setLinearVelocity(const PxVec3& v, bool skipBodySimUpdate=false);
	
		PX_FORCE_INLINE	const PxVec3&		getAngularVelocity()		const	{ return mCore.angularVelocity;		}
						void				setAngularVelocity(const PxVec3& v, bool skipBodySimUpdate=false);
	
		PX_FORCE_INLINE	PxReal				getCfmScale()				const { return mCore.cfmScale; }
						void				setCfmScale(PxReal d);
		
		PX_FORCE_INLINE	void				updateVelocities(const PxVec3& linearVelModPerStep, const PxVec3& angularVelModPerStep)
											{
												mCore.linearVelocity += linearVelModPerStep;
												mCore.angularVelocity += angularVelModPerStep;
											}

		PX_FORCE_INLINE	const PxTransform&	getBody2Actor()				const	{ return mCore.getBody2Actor();			}
						void				setBody2Actor(const PxTransform& p);

						void				addSpatialAcceleration(const PxVec3* linAcc, const PxVec3* angAcc);
						void				setSpatialAcceleration(const PxVec3* linAcc, const PxVec3* angAcc);
						void				clearSpatialAcceleration(bool force, bool torque);
						void				addSpatialVelocity(const PxVec3* linVelDelta, const PxVec3* angVelDelta);
						void				clearSpatialVelocity(bool force, bool torque);

		PX_FORCE_INLINE PxReal				getMaxPenetrationBias() const		{ return mCore.maxPenBias; }
		PX_FORCE_INLINE void				setMaxPenetrationBias(PxReal p)		{ mCore.maxPenBias = p; }

						PxReal				getInverseMass()			const;
						void				setInverseMass(PxReal m);
						const PxVec3&		getInverseInertia()			const;
						void				setInverseInertia(const PxVec3& i);

						PxReal				getLinearDamping()			const;
						void				setLinearDamping(PxReal d);

						PxReal				getAngularDamping()			const;
						void				setAngularDamping(PxReal d);

		PX_FORCE_INLINE	PxRigidBodyFlags	getFlags()					const	{ return mCore.mFlags;		}
						void				setFlags(PxRigidBodyFlags f);

		PX_FORCE_INLINE	PxRigidDynamicLockFlags	getRigidDynamicLockFlags()					const	{ return mCore.lockFlags; }

		PX_FORCE_INLINE void				setRigidDynamicLockFlags(PxRigidDynamicLockFlags flags) { mCore.lockFlags = flags; }

		PX_FORCE_INLINE	PxReal				getSleepThreshold()	const	{ return mCore.sleepThreshold;	}
						void				setSleepThreshold(PxReal t);		

		PX_FORCE_INLINE	PxReal				getFreezeThreshold()	const	{ return mCore.freezeThreshold;	}
						void				setFreezeThreshold(PxReal t);

		PX_FORCE_INLINE PxReal				getMaxContactImpulse() const	{ return mCore.maxContactImpulse;  }
						void				setMaxContactImpulse(PxReal m);

		PX_FORCE_INLINE PxReal				getOffsetSlop() const	{ return mCore.offsetSlop;  }
						void				setOffsetSlop(PxReal slop);

						PxNodeIndex			getInternalIslandNodeIndex() const;

		PX_FORCE_INLINE PxReal				getWakeCounter() const { return mCore.wakeCounter; }
						void				setWakeCounter(PxReal wakeCounter, bool forceWakeUp=false);

						bool				isSleeping() const;
		PX_FORCE_INLINE	void				wakeUp(PxReal wakeCounter)			{ setWakeCounter(wakeCounter, true);	}
						void				putToSleep();

						PxReal				getMaxAngVelSq() const;
						void				setMaxAngVelSq(PxReal v);

						PxReal				getMaxLinVelSq() const;
						void				setMaxLinVelSq(PxReal v);

		PX_FORCE_INLINE	PxU16				getSolverIterationCounts()	const	{ return mCore.solverIterationCounts;	}
						void				setSolverIterationCounts(PxU16 c);

						bool				getKinematicTarget(PxTransform& p) const;
						bool				getHasValidKinematicTarget() const;
						void				setKinematicTarget(const PxTransform& p, PxReal wakeCounter);
						void				invalidateKinematicTarget();

		PX_FORCE_INLINE	PxReal				getContactReportThreshold()	const	{ return mCore.contactReportThreshold;	}
						void				setContactReportThreshold(PxReal t)	{ mCore.contactReportThreshold = t;		}

						void				onOriginShift(const PxVec3& shift);

		//---------------------------------------------------------------------------------
		// Internal API
		//---------------------------------------------------------------------------------

		PX_FORCE_INLINE void				setLinearVelocityInternal(const PxVec3& v)	{ mCore.linearVelocity = v; }
		PX_FORCE_INLINE void				setAngularVelocityInternal(const PxVec3& v)	{ mCore.angularVelocity = v; }
		PX_FORCE_INLINE	void				setWakeCounterFromSim(PxReal c)		{ mCore.wakeCounter = c;					}

						BodySim*			getSim() const;

		PX_FORCE_INLINE	PxsBodyCore&		getCore()							{ return mCore;						}
		PX_FORCE_INLINE	const PxsBodyCore&	getCore()			const			{ return mCore;						}
		static PX_FORCE_INLINE size_t		getCoreOffset()						{ return PX_OFFSET_OF_RT(BodyCore, mCore);	}

		PX_FORCE_INLINE	PxReal				getCCDAdvanceCoefficient() const	{ return mCore.ccdAdvanceCoefficient;	}
		PX_FORCE_INLINE	void				setCCDAdvanceCoefficient(PxReal c)	{ mCore.ccdAdvanceCoefficient = c;		}

						void				onRemoveKinematicFromScene();

						PxIntBool			isFrozen()							const;

		static PX_FORCE_INLINE BodyCore&	getCore(PxsBodyCore& core)
		{ 
			return *reinterpret_cast<BodyCore*>(reinterpret_cast<PxU8*>(&core) - getCoreOffset());
		}

						void				setFixedBaseLink(bool value);
	private:
						PX_ALIGN_PREFIX(16) PxsBodyCore mCore PX_ALIGN_SUFFIX(16);
	};

} // namespace Sc

}

#endif
