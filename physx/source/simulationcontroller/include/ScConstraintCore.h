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

#ifndef SC_CONSTRAINT_CORE_H
#define SC_CONSTRAINT_CORE_H

#include "PxConstraint.h"

namespace physx
{
namespace Sc
{
	class ConstraintSim;
	class RigidCore;

	class ConstraintCore
	{
	public:
// PX_SERIALIZATION
											ConstraintCore(const PxEMPTY) : mFlags(PxEmpty), mConnector(NULL), mSim(NULL), mResidual()	{}
	PX_FORCE_INLINE	void					setConstraintFunctions(PxConstraintConnector& n, const PxConstraintShaderTable& shaders)
											{ 
												mConnector = &n;	
												mSolverPrep = shaders.solverPrep;
												mVisualize = shaders.visualize;
											}
		static		void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
											ConstraintCore(PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize);
											~ConstraintCore()	{}

					void					setBodies(RigidCore* r0v, RigidCore* r1v);

					PxConstraint*			getPxConstraint();
					const PxConstraint*		getPxConstraint()								const;
	PX_FORCE_INLINE	PxConstraintConnector*	getPxConnector()								const	{ return mConnector;				}

	PX_FORCE_INLINE	PxConstraintFlags		getFlags()										const	{ return mFlags;					}
					void					setFlags(PxConstraintFlags flags);

					void					getForce(PxVec3& force, PxVec3& torque)			const;

					void					setBreakForce(PxReal linear, PxReal angular);
	PX_FORCE_INLINE	void					getBreakForce(PxReal& linear, PxReal& angular)	const
											{
												linear = mLinearBreakForce;
												angular = mAngularBreakForce;
											}

					void					setMinResponseThreshold(PxReal threshold);
	PX_FORCE_INLINE	PxReal					getMinResponseThreshold()						const	{ return mMinResponseThreshold;		}

					void					breakApart();

	PX_FORCE_INLINE	PxConstraintVisualize	getVisualize()									const	{ return mVisualize;				}
	PX_FORCE_INLINE	PxConstraintSolverPrep	getSolverPrep()									const	{ return mSolverPrep;				}
	PX_FORCE_INLINE	PxU32					getConstantBlockSize()							const	{ return mDataSize;					}

	PX_FORCE_INLINE	void					setSim(ConstraintSim* sim)
											{
												PX_ASSERT((sim==0) ^ (mSim == 0));
												mSim = sim;
											}
	PX_FORCE_INLINE	ConstraintSim*			getSim()										const	{ return mSim;						}

	PX_FORCE_INLINE	bool					isDirty()										const	{ return mIsDirty ? true : false;	}
	PX_FORCE_INLINE	void					setDirty()												{ mIsDirty = 1;						}
	PX_FORCE_INLINE	void					clearDirty()											{ mIsDirty = 0;						}

	PX_FORCE_INLINE	PxConstraintResidual	getSolverResidual()								const	{ return mResidual;					}
	PX_FORCE_INLINE	void					setSolverResidual(const PxConstraintResidual& residual)	{ mResidual = residual; }

	private:
					PxConstraintFlags		mFlags;
					//In order to support O(1) insert/remove mIsDirty really wants to be an index into NpScene's dirty joint array
					PxU8					mIsDirty;
					PxU8					mPadding;

					PxVec3					mAppliedForce;
					PxVec3					mAppliedTorque;

					PxConstraintConnector*	mConnector;
					PxConstraintSolverPrep	mSolverPrep;
					PxConstraintVisualize	mVisualize;
					PxU32					mDataSize;
					PxReal					mLinearBreakForce;
					PxReal					mAngularBreakForce;
					PxReal					mMinResponseThreshold;

					ConstraintSim*			mSim;
					PxConstraintResidual	mResidual;
	};

} // namespace Sc

}

#endif
