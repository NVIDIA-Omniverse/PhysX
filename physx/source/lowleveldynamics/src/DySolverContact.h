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

#ifndef DY_SOLVER_CONTACT_H
#define DY_SOLVER_CONTACT_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "PxvConfig.h"
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

	PX_FORCE_INLINE void setStaticFriction(PxF32 f)			{ V4WriteX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}
	PX_FORCE_INLINE void setDynamicFriction(PxF32 f)		{ V4WriteY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}
	PX_FORCE_INLINE void setDominance0(PxF32 f)				{ V4WriteZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}
	PX_FORCE_INLINE void setDominance1(PxF32 f)				{ V4WriteW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, f);	}

	PX_FORCE_INLINE PxF32 getStaticFrictionPxF32() const	{ return V4ReadX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
	PX_FORCE_INLINE PxF32 getDynamicFrictionPxF32() const	{ return V4ReadY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
	PX_FORCE_INLINE PxF32 getDominance0PxF32() const		{ return V4ReadZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
	PX_FORCE_INLINE PxF32 getDominance1PxF32() const		{ return V4ReadW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W);	}
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
	// PT: TODO: there's room for 3 floats in the padding bytes so we could just stick appliedForce / velMultiplier / bias there
	// and avoid doing all the data packing / unpacking for these members...
	Vec4V normalXYZ_appliedForceW;		//16
	Vec4V raXnXYZ_velMultiplierW;		//32
	Vec4V rbXnXYZ_biasW;				//48
	PxReal targetVel;					//52
	PxU32 mPad[3];						//64

	PX_FORCE_INLINE void setAppliedForce(const FloatV f)	{ normalXYZ_appliedForceW = V4SetW(normalXYZ_appliedForceW,f);	}
	PX_FORCE_INLINE void setBias(const FloatV f)			{ rbXnXYZ_biasW = V4SetW(rbXnXYZ_biasW,f);						}

	PX_FORCE_INLINE Vec3V getNormal() const { return Vec3V_From_Vec4V(normalXYZ_appliedForceW); }
	PX_FORCE_INLINE FloatV getAppliedForce() const { return V4GetW(normalXYZ_appliedForceW); }
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
