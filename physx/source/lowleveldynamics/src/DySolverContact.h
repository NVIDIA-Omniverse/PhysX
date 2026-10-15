// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SOLVER_CONTACT_H
#define DY_SOLVER_CONTACT_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "PxPhysXConfig.h"
#include "foundation/PxVecMath.h"

namespace physx
{

using namespace aos;

namespace Sc
{
	class ShapeInteraction;
}
/**
\brief A header to represent a friction patch for the solver.
*/

namespace Dy
{

struct SolverContactHeader
{
	enum DySolverContactFlags
	{
		eHAS_FORCE_THRESHOLDS = 0x1
	};

	PxU8	type;					//Note: mType should be first as the solver expects a type in the first byte.
	PxU8	flags;	
	PxU8	numNormalConstr;
	PxU8	numFrictionConstr;					//4

	PxReal	angDom0;							//8
	PxReal	angDom1;							//12
	PxReal	invMass0;							//16

	Vec4V   staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;		//32
	//KS - minAppliedImpulseForFrictionW is non-zero only for articulations. This is a workaround for a case in articulations where
	//the impulse is propagated such that many links do not apply friction because their normal forces were corrected by the solver in a previous
	//link. This results in some links sliding unnaturally. This occurs with prismatic or revolute joints where the impulse propagation one one link
	//resolves the normal constraint on all links
	Vec4V	normal_minAppliedImpulseForFrictionW;							//48 
																			
	PxReal	invMass1;														//52
	PxU32	broken;															//56
	PxU8*	frictionBrokenWritebackByte;									//60	64
	Sc::ShapeInteraction* shapeInteraction;									//64	72
#if PX_P64_FAMILY
	PxU32	pad[2];															//64	80
#endif // PX_X64

	PX_FORCE_INLINE void setStaticFriction(const FloatV f)	{ staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}
	PX_FORCE_INLINE void setDynamicFriction(const FloatV f)	{ staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}
	PX_FORCE_INLINE void setDominance0(const FloatV f)		{ staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}
	PX_FORCE_INLINE void setDominance1(const FloatV f)		{ staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}

	PX_FORCE_INLINE FloatV getStaticFriction() const		{ return V4GetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
	PX_FORCE_INLINE FloatV getDynamicFriction() const		{ return V4GetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
	PX_FORCE_INLINE FloatV getDominance0() const			{ return V4GetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
	PX_FORCE_INLINE FloatV getDominance1() const			{ return V4GetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}

/*	PX_FORCE_INLINE void setStaticFriction(PxF32 f)			{ V4WriteX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}
	PX_FORCE_INLINE void setDynamicFriction(PxF32 f)		{ V4WriteY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}
	PX_FORCE_INLINE void setDominance0(PxF32 f)				{ V4WriteZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}
	PX_FORCE_INLINE void setDominance1(PxF32 f)				{ V4WriteW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}

	PX_FORCE_INLINE PxF32 getStaticFrictionPxF32() const	{ return V4ReadX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
	PX_FORCE_INLINE PxF32 getDynamicFrictionPxF32() const	{ return V4ReadY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
	PX_FORCE_INLINE PxF32 getDominance0PxF32() const		{ return V4ReadZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
	PX_FORCE_INLINE PxF32 getDominance1PxF32() const		{ return V4ReadW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}*/
}; 

#if !PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactHeader) == 64);
#else
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactHeader) == 80);
#endif

/**
\brief A single rigid body contact point for the solver.
*/
struct SolverContactPoint
{
	Vec4V raXn_velMultiplierW;
	Vec4V rbXn_maxImpulseW;

	PxF32 biasedErr;
	PxF32 unbiasedErr;
	PxF32 impulseMultiplier;
	PxU32 pad;

	PX_FORCE_INLINE FloatV getVelMultiplier() const			{ return V4GetW(raXn_velMultiplierW);	}
	PX_FORCE_INLINE FloatV getImpulseMultiplier() const		{ return FLoad(impulseMultiplier);		}

	PX_FORCE_INLINE FloatV getBiasedErr() const				{ return FLoad(biasedErr);				}
	PX_FORCE_INLINE FloatV getMaxImpulse() const			{ return V4GetW(rbXn_maxImpulseW);		}
}; 

PX_COMPILE_TIME_ASSERT(sizeof(SolverContactPoint) == 48);

/**
\brief A single extended articulation contact point for the solver.
*/
struct SolverContactPointExt : public SolverContactPoint
{
	Vec3V linDeltaVA;
	Vec3V angDeltaVA;
	Vec3V linDeltaVB;
	Vec3V angDeltaVB;
};

PX_COMPILE_TIME_ASSERT(sizeof(SolverContactPointExt) == 112);

/**
\brief A single friction constraint for the solver.
*/
struct SolverContactFriction
{
	// PT: appliedForce / velMultiplier / bias used to be packed into the W lanes of the three vectors, which
	// cost a V4GetW shuffle per read and a V4SetW insert per write. They now live in what was padding, so the
	// struct is still 64 bytes but the scalars are plain loads/stores.
	Vec3V normal;			//16
	Vec3V raXn;				//32
	Vec3V rbXn;				//48
	PxReal targetVel;		//52
	PxReal appliedForce;	//56	used to be stored in W of previous Vecs
	PxReal velMultiplier;	//60	used to be stored in W of previous Vecs
	PxReal bias;			//64	used to be stored in W of previous Vecs

	PX_FORCE_INLINE void setAppliedForce(const FloatV f)	{ FStore(f, &appliedForce);	}
	PX_FORCE_INLINE void setBias(const FloatV f)			{ FStore(f, &bias);			}

	PX_FORCE_INLINE Vec3V getNormal() const			{ return normal;				}
	PX_FORCE_INLINE FloatV getAppliedForce() const	{ return FLoad(appliedForce);	}
}; 

PX_COMPILE_TIME_ASSERT(sizeof(SolverContactFriction) == 64);

/**
\brief A single extended articulation friction constraint for the solver.
*/
struct SolverContactFrictionExt : public SolverContactFriction
{
	Vec3V linDeltaVA;
	Vec3V angDeltaVA;
	Vec3V linDeltaVB;
	Vec3V angDeltaVB;
};
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactFrictionExt) == 128);

}

}

#endif
