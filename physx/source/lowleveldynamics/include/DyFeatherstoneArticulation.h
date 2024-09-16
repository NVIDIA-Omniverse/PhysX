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

#ifndef DY_FEATHERSTONE_ARTICULATION_H
#define DY_FEATHERSTONE_ARTICULATION_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVecMath.h"
#include "CmUtils.h"
#include "DyVArticulation.h"
#include "DyFeatherstoneArticulationUtils.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "solver/PxSolverDefs.h"
#include "DyArticulationTendon.h"
#include "DyCpuGpuArticulation.h"
#include "CmSpatialVector.h"
#include "DyResidualAccumulator.h"

#ifndef FEATHERSTONE_DEBUG
#define FEATHERSTONE_DEBUG 0
#endif

#define DY_STATIC_CONTACTS_IN_INTERNAL_SOLVER true

namespace physx
{

class PxContactJoint;
class PxcConstraintBlockStream;
class PxcScratchAllocator;
class PxsConstraintBlockManager;
struct SolverConstraint1DExtStep;
struct PxSolverConstraintPrepDesc;
struct PxSolverBody;
struct PxSolverBodyData;
class PxConstraintAllocator;
class PxsContactManagerOutputIterator;

struct PxSolverConstraintDesc;
	
namespace Dy
{
//#if PX_VC 
//#pragma warning(push)   
//#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
//#endif
	
	class ArticulationLinkData;
	struct ArticulationMimicJointCore;
	struct SpatialSubspaceMatrix;
	struct SolverConstraint1DExt;
	struct SolverConstraint1DStep;

	class FeatherstoneArticulation;
	struct SpatialMatrix;
	struct SpatialTransform;
	struct Constraint;
	class ThreadContext;

	struct ArticulationInternalTendonConstraint
	{
		Cm::UnAlignedSpatialVector row0;			//24	24
		Cm::UnAlignedSpatialVector row1;			//24	48

		Cm::UnAlignedSpatialVector deltaVB;			//24	72

		PxU32	linkID0;							//4		74
		PxU32	linkID1;							//4		78
		PxReal	accumulatedLength;					//4		82	//accumulate distance for spatial tendon, accumulate joint pose for fixed tendon
		PxReal	biasCoefficient;					//4		94
		PxReal	velMultiplier;						//4		98
		PxReal	impulseMultiplier;					//4		102
		PxReal	appliedForce;						//4		106
		PxReal	recipResponse;						//4		110
		PxReal	deltaVA;							//4		114
		PxReal	limitBiasCoefficient;
		PxReal	limitImpulseMultiplier;
		PxReal	limitAppliedForce;
		PxReal	restDistance;
		PxReal	lowLimit;
		PxReal	highLimit;
		PxReal	velImpulseMultiplier;
		PxReal	limitVelImpulseMultiplier;
	};

	struct ArticulationInternalConstraintBase
	{
		//Common/shared directional info between, frictions and drives
		Cm::UnAlignedSpatialVector row0;		//24	24
		Cm::UnAlignedSpatialVector row1;		//24	48
		Cm::UnAlignedSpatialVector deltaVA;		//24	72	
		Cm::UnAlignedSpatialVector deltaVB;		//24	96

		//Response information
		PxReal recipResponse;					//4		100
		PxReal response;						//4		104
	};

	struct ArticulationInternalLimit
	{
		// Initial error for high and low limits. Negative means limit is violated.
		PxReal errorLow;
		PxReal errorHigh;

		// Impulses are updated during solver iterations.
		PxReal lowImpulse; // always >= 0 (repulsive only)
		PxReal highImpulse; // always <= 0 (repulsive only)
	};

	struct ArticulationInternalConstraint : public ArticulationInternalConstraintBase
	{	
		ArticulationImplicitDriveDesc implicitDriveDesc;
		PxReal driveMaxForce;					
		PxReal driveForce;						

		PxReal frictionForceCoefficient;
		PxReal frictionMaxForce;				
		PxReal frictionForce;

		bool isLinearConstraint;

		PxReal padding[3];

		void setImplicitDriveDesc(const ArticulationImplicitDriveDesc& driveDesc)
		{
			implicitDriveDesc = driveDesc;
		}
		const ArticulationImplicitDriveDesc& getImplicitDriveDesc() const
		{
			return implicitDriveDesc;
		}	
	};
	PX_COMPILE_TIME_ASSERT(0 == (sizeof(ArticulationInternalConstraint) & 0x0f));

	struct ArticulationInternalMimicJoint
	{
		PxU32 linkA;
		PxU32 dofA;
		
		PxU32 linkB;
		PxU32 dofB;

		PxReal gearRatio;
		PxReal offset;

		//Cache effectiveInertia = 1/[J * M^-1 * J^T]  =  1/[rAA + gearRatio*(rAB + rBA) + gearRatio*gearRatio*rBB]
		//Impulse = [1, gearRatio]^T * [-b + J*v] / [J * M^-1 * J^T]
		//Impulse = [1, gearRatio]^T * [-b + J*v] * effectiveInertia;
		PxReal effectiveInertia;
	};


	struct PX_ALIGN_PREFIX(16) JointSpaceSpatialZ
	{
		PxReal mVals [6][4];

		PxReal dot(Cm::SpatialVectorF& v, PxU32 id)
		{
			return v.top.x * mVals[0][id] + v.top.y * mVals[1][id] + v.top.z * mVals[2][id]
				+ v.bottom.x * mVals[3][id] + v.bottom.y * mVals[4][id] + v.bottom.z * mVals[5][id];
		}
	} 
	PX_ALIGN_SUFFIX(16);

	class ArticulationData
	{
	public:

		ArticulationData() :  
			mPathToRootElements(NULL), mNumPathToRootElements(0), mLinksData(NULL), mJointData(NULL),
			mSpatialTendons(NULL), mNumSpatialTendons(0), mNumTotalAttachments(0),
			mFixedTendons(NULL), mNumFixedTendons(0), 
			mMimicJoints(NULL), mNbMimicJoints(0), mDt(0.f),mDofs(0xffffffff),
			mDataDirty(true)
		{
			mRootPreMotionVelocity = Cm::SpatialVectorF::Zero();
		}

		~ArticulationData();

		PX_FORCE_INLINE	void						init();
						void						resizeLinkData(const PxU32 linkCount);
						void						resizeJointData(const PxU32 dofs);
		
		PX_FORCE_INLINE PxReal*						getJointAccelerations()									{ return mJointAcceleration.begin();		}
		PX_FORCE_INLINE const PxReal*				getJointAccelerations() const							{ return mJointAcceleration.begin();		}
		PX_FORCE_INLINE PxReal*						getJointVelocities()									{ return mJointVelocity.begin();			}
		PX_FORCE_INLINE const PxReal*				getJointVelocities() const								{ return mJointVelocity.begin();			}
		PX_FORCE_INLINE PxReal*						getJointNewVelocities()									{ return mJointNewVelocity.begin();			}
		PX_FORCE_INLINE const PxReal*				getJointNewVelocities() const							{ return mJointNewVelocity.begin();			}
		PX_FORCE_INLINE PxReal*						getJointPositions()										{ return mJointPosition.begin();			}
		PX_FORCE_INLINE const PxReal*				getJointPositions() const								{ return mJointPosition.begin();			}
		PX_FORCE_INLINE PxReal*						getJointForces()										{ return mJointForce.begin();				}
		PX_FORCE_INLINE const PxReal*				getJointForces() const									{ return mJointForce.begin();				}
		PX_FORCE_INLINE PxReal*						getJointTargetPositions()								{ return mJointTargetPositions.begin();		}
		PX_FORCE_INLINE const PxReal*				getJointTargetPositions() const							{ return mJointTargetPositions.begin();		}
		PX_FORCE_INLINE PxReal*						getJointTargetVelocities()								{ return mJointTargetVelocities.begin();	}
		PX_FORCE_INLINE const PxReal*				getJointTargetVelocities() const						{ return mJointTargetVelocities.begin();	}

		PX_FORCE_INLINE ArticulationInternalConstraint&			getInternalConstraint(const PxU32 dofId)		{ return mInternalConstraints[dofId];	}
		PX_FORCE_INLINE const ArticulationInternalConstraint&	getInternalConstraint(const PxU32 dofId) const	{ return mInternalConstraints[dofId];	}
		
		PX_FORCE_INLINE Cm::SpatialVectorF*			getMotionVelocities()									{ return mMotionVelocities.begin();			}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getMotionAccelerations()								{ return mMotionAccelerations.begin();		}
		PX_FORCE_INLINE const Cm::SpatialVectorF*	getMotionAccelerations() const							{ return mMotionAccelerations.begin(); }
		PX_FORCE_INLINE		  Cm::SpatialVectorF*	getLinkIncomingJointForces()							{ return mLinkIncomingJointForces.begin(); }
		PX_FORCE_INLINE Cm::SpatialVectorF*			getCorioliseVectors()									{ return mCorioliseVectors.begin();			}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getSpatialZAVectors()									{ return mZAForces.begin();					}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getSpatialZAInternalVectors()							{ return mZAInternalForces.begin();			}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getTransmittedForces()									{ return mJointTransmittedForce.begin();	}

		PX_FORCE_INLINE Cm::SpatialVectorF*			getPosIterMotionVelocities()							{ return mPosIterMotionVelocities.begin();	}
		PX_FORCE_INLINE const Cm::SpatialVectorF*	getPosIterMotionVelocities() const						{ return mPosIterMotionVelocities.begin();	}
		PX_FORCE_INLINE PxReal*						getPosIterJointVelocities()								{ return mPosIterJointVelocities.begin();	}
	
		PX_FORCE_INLINE Cm::SpatialVectorF&			getPosIterMotionVelocity(const PxU32 index)				{ return mPosIterMotionVelocities[index];	}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getMotionVelocity(const PxU32 index)			const	{ return mMotionVelocities[index];			}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getMotionAcceleration(const PxU32 index)		const	{ return mMotionAccelerations[index];		}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getCorioliseVector(const PxU32 index)			const	{ return mCorioliseVectors[index];			}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getSpatialZAVector(const PxU32 index)			const	{ return mZAForces[index];					}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getTransmittedForce(const PxU32 index)			const	{ return mJointTransmittedForce[index];		}

		PX_FORCE_INLINE Cm::SpatialVectorF&			getMotionVelocity(const PxU32 index)					{ return mMotionVelocities[index];			}
		PX_FORCE_INLINE Cm::SpatialVectorF&			getMotionAcceleration(const PxU32 index)				{ return mMotionAccelerations[index];		}
		PX_FORCE_INLINE Cm::SpatialVectorF&			getCorioliseVector(const PxU32 index)					{ return mCorioliseVectors[index];			}
		PX_FORCE_INLINE Cm::SpatialVectorF&			getSpatialZAVector(const PxU32 index)					{ return mZAForces[index];					}
		PX_FORCE_INLINE Cm::SpatialVectorF&			getTransmittedForce(const PxU32 index)					{ return mJointTransmittedForce[index];		}

		//PX_FORCE_INLINE Dy::SpatialMatrix*			getTempSpatialMatrix() { mTempSpatialMatrix.begin(); }

		PX_FORCE_INLINE PxTransform&				getPreTransform(const PxU32 index)						{ return mPreTransform[index];				}
		PX_FORCE_INLINE const PxTransform&			getPreTransform(const PxU32 index)				const	{ return mPreTransform[index];				}
//		PX_FORCE_INLINE void						setPreTransform(const PxU32 index, const PxTransform& t){ mPreTransform[index] = t;					}
		PX_FORCE_INLINE PxTransform*				getPreTransform()										{ return mPreTransform.begin();				}

		PX_FORCE_INLINE const Cm::SpatialVectorF&	getDeltaMotionVector(const PxU32 index) const			{ return mDeltaMotionVector[index];			}
		PX_FORCE_INLINE void						setDeltaMotionVector(const PxU32 index, const Cm::SpatialVectorF& vec) { mDeltaMotionVector[index] = vec; }
		PX_FORCE_INLINE Cm::SpatialVectorF*			getDeltaMotionVector()									{ return mDeltaMotionVector.begin();		}

		PX_FORCE_INLINE ArticulationLink*			getLinks()										const	{ return mLinks;							}
		PX_FORCE_INLINE PxU32						getLinkCount()									const	{ return mLinkCount;						}
		PX_FORCE_INLINE ArticulationLink&			getLink(PxU32 index)							const	{ return mLinks[index];						}
		
		PX_FORCE_INLINE ArticulationSpatialTendon**	getSpatialTendons()								const	{ return mSpatialTendons;					}
		PX_FORCE_INLINE PxU32						getSpatialTendonCount()							const	{ return mNumSpatialTendons;				}
		PX_FORCE_INLINE ArticulationSpatialTendon*	getSpatialTendon(PxU32 index)					const	{ return mSpatialTendons[index];			}

		PX_FORCE_INLINE ArticulationFixedTendon**	getFixedTendons()								const	{ return mFixedTendons;						}
		PX_FORCE_INLINE PxU32						getFixedTendonCount()							const	{ return mNumFixedTendons;					}
		PX_FORCE_INLINE ArticulationFixedTendon*	getFixedTendon(PxU32 index)						const	{ return mFixedTendons[index];				}

		PX_FORCE_INLINE	ArticulationMimicJointCore** getMimicJointCores()							const	{ return mMimicJoints;						}
		PX_FORCE_INLINE PxU32						getMimicJointCount()							const	{ return mNbMimicJoints;					}

		PX_FORCE_INLINE ArticulationLinkData*		getLinkData()									const	{ return mLinksData;						}
						ArticulationLinkData&		getLinkData(PxU32 index)						const;

		PX_FORCE_INLINE ArticulationJointCoreData*	getJointData()									const	{ return mJointData;						}
		PX_FORCE_INLINE ArticulationJointCoreData&	getJointData(PxU32 index)						const	{ return mJointData[index];					}

		// PT: PX-1399
		PX_FORCE_INLINE PxArticulationFlags			getArticulationFlags()							const	{ return *mFlags;							}

		PX_FORCE_INLINE Cm::SpatialVector*			getExternalAccelerations()								{ return mExternalAcceleration;				}

		PX_FORCE_INLINE Cm::SpatialVector&			getExternalAcceleration(const PxU32 linkID)				{ return mExternalAcceleration[linkID];		}
		PX_FORCE_INLINE const Cm::SpatialVector&	getExternalAcceleration(const PxU32 linkID)		const	{ return mExternalAcceleration[linkID]; 	}

		PX_FORCE_INLINE PxReal						getDt()											const	{ return mDt;								}
		PX_FORCE_INLINE void						setDt(const PxReal dt)									{ mDt = dt;									}

		PX_FORCE_INLINE bool						getDataDirty()									const	{ return mDataDirty;						}
		PX_FORCE_INLINE void						setDataDirty(const bool dirty)							{ mDataDirty = dirty;						}

		PX_FORCE_INLINE	PxU32						getDofs()										const	{ return mDofs;								}
		PX_FORCE_INLINE	void						setDofs(const PxU32 dof)								{ mDofs = dof;								}

		PX_FORCE_INLINE FeatherstoneArticulation*	getArticulation()										{ return mArticulation;						}
		PX_FORCE_INLINE void						setArticulation(FeatherstoneArticulation* articulation)	{ mArticulation = articulation;				}
		
		PX_FORCE_INLINE const SpatialMatrix&		getBaseInvSpatialArticulatedInertiaW()			const	{ return mBaseInvSpatialArticulatedInertiaW; }
		PX_FORCE_INLINE		  SpatialMatrix&		getBaseInvSpatialArticulatedInertiaW()					{ return mBaseInvSpatialArticulatedInertiaW; }

		PX_FORCE_INLINE PxTransform*				getAccumulatedPoses()									{ return mAccumulatedPoses.begin();			}
		PX_FORCE_INLINE const PxTransform*			getAccumulatedPoses()							const	{ return mAccumulatedPoses.begin();			}

		PX_FORCE_INLINE const Cm::SpatialVectorF&	getRootDeferredZ()								const	{ return mRootDeferredZ;					}
		PX_FORCE_INLINE	Cm::SpatialVectorF&			getRootDeferredZ()										{ return mRootDeferredZ;					}

		PX_FORCE_INLINE const	SpatialMatrix*		getWorldSpatialArticulatedInertia()				const	{ return mWorldSpatialArticulatedInertia.begin(); }
		PX_FORCE_INLINE			SpatialMatrix*		getWorldSpatialArticulatedInertia()						{ return mWorldSpatialArticulatedInertia.begin(); }

		PX_FORCE_INLINE const	Cm::UnAlignedSpatialVector* getWorldMotionMatrix()					const	{ return mWorldMotionMatrix.begin(); }
		PX_FORCE_INLINE			Cm::UnAlignedSpatialVector* getWorldMotionMatrix()							{ return mWorldMotionMatrix.begin(); }

		PX_FORCE_INLINE const	Cm::UnAlignedSpatialVector* getMotionMatrix()						const	{ return mMotionMatrix.begin(); }
		PX_FORCE_INLINE			Cm::UnAlignedSpatialVector* getMotionMatrix()								{ return mMotionMatrix.begin(); }

		PX_FORCE_INLINE const	Cm::SpatialVectorF* getIsW()										const	{ return mIsW.begin(); }
		PX_FORCE_INLINE			Cm::SpatialVectorF* getIsW()												{ return mIsW.begin(); }

		PX_FORCE_INLINE const	PxVec3* getRw()														const	{ return mRw.begin(); }
		PX_FORCE_INLINE			PxVec3* getRw()																{ return mRw.begin(); }

		PX_FORCE_INLINE const	PxReal* getMinusStZExt()											const	{ return qstZIc.begin(); }
		PX_FORCE_INLINE			PxReal* getMinusStZExt()													{ return qstZIc.begin(); }

		PX_FORCE_INLINE const	PxReal* getQstZIc()													const	{ return qstZIc.begin(); }
		PX_FORCE_INLINE			PxReal* getQstZIc()															{ return qstZIc.begin(); }

		PX_FORCE_INLINE	const	PxReal* getQStZIntIc()												const	{ return qstZIntIc.begin();}
		PX_FORCE_INLINE			PxReal* getQStZIntIc()														{ return qstZIntIc.begin();}

		PX_FORCE_INLINE const	InvStIs* getInvStIS()												const	{ return mInvStIs.begin(); }
		PX_FORCE_INLINE			InvStIs* getInvStIS()														{ return mInvStIs.begin(); }

		PX_FORCE_INLINE const	Cm::SpatialVectorF* getISInvStIS()									const	{ return mISInvStIS.begin(); }
		PX_FORCE_INLINE			Cm::SpatialVectorF* getISInvStIS()											{ return mISInvStIS.begin(); }

		PX_FORCE_INLINE			TestImpulseResponse* getImpulseResponseMatrixWorld()						{ return mResponseMatrixW.begin(); }

		PX_FORCE_INLINE const	TestImpulseResponse* getImpulseResponseMatrixWorld()				const	{ return mResponseMatrixW.begin(); }

		PX_FORCE_INLINE const SpatialMatrix& getWorldSpatialArticulatedInertia(const PxU32 linkID) const { return mWorldSpatialArticulatedInertia[linkID]; }
		
		PX_FORCE_INLINE const InvStIs& getInvStIs(const PxU32 linkID) const { return mInvStIs[linkID]; }

		PX_FORCE_INLINE const Cm::UnAlignedSpatialVector& getMotionMatrix(const PxU32 dofId) const { return mMotionMatrix[dofId]; }
		PX_FORCE_INLINE const Cm::UnAlignedSpatialVector& getWorldMotionMatrix(const PxU32 dofId) const { return mWorldMotionMatrix[dofId]; }

		PX_FORCE_INLINE		  Cm::UnAlignedSpatialVector& getJointAxis(const PxU32 dofId)			{ return mJointAxis[dofId]; }
		PX_FORCE_INLINE const Cm::UnAlignedSpatialVector& getJointAxis(const PxU32 dofId) const		{ return mJointAxis[dofId]; }

		PX_FORCE_INLINE const PxVec3& getRw(const PxU32 linkID) const								{ return mRw[linkID]; }

		PX_FORCE_INLINE const Cm::SpatialVectorF& getIsW(const PxU32 dofId) const { return mIsW[dofId]; }

		PX_FORCE_INLINE const Cm::SpatialVectorF& getWorldIsInvD(const PxU32 dofId) const { return mISInvStIS[dofId]; }
		PX_FORCE_INLINE PxReal* getDeferredQstZ() { return mDeferredQstZ.begin(); }

		PX_FORCE_INLINE void setRootPreMotionVelocity(const Cm::UnAlignedSpatialVector& vel) { mRootPreMotionVelocity.top = vel.top; mRootPreMotionVelocity.bottom = vel.bottom; }

		PX_FORCE_INLINE PxU32*	getPathToRootElements() const { return mPathToRootElements; }
		PX_FORCE_INLINE PxU32	getPathToRootElementCount() const { return mNumPathToRootElements; }

		PX_FORCE_INLINE void incrementSolverSpatialDeltaVel(const PxU32 linkID, const Cm::SpatialVectorF& deltaV) {mSolverLinkSpatialDeltaVels[linkID] += deltaV;}

	private:
		Cm::SpatialVectorF							mRootPreMotionVelocity;
		Cm::SpatialVectorF							mRootDeferredZ;
		PxArray<PxReal>								mJointAcceleration;		//	joint acceleration
		PxArray<PxReal>								mJointInternalAcceleration;	//joint internal force acceleration
		PxArray<PxReal>								mJointVelocity;			//	joint velocity
		PxArray<PxReal>								mJointNewVelocity;		//	joint velocity due to contacts
		PxArray<PxReal>								mJointPosition;			//	joint position
		PxArray<PxReal>								mJointForce;			//	joint force
		PxArray<PxReal>								mJointTargetPositions;  //	joint target positions
		PxArray<PxReal>								mJointTargetVelocities; //	joint target velocities
	
		PxArray<PxReal>											mPosIterJointVelocities;	//joint delta velocity after postion iternation before velocity iteration
		PxArray<Cm::SpatialVectorF>								mPosIterMotionVelocities;	//link motion velocites after position iteration before velocity iteration
		PxArray<Cm::SpatialVectorF>								mMotionVelocities;			//link motion velocites
		PxArray<Cm::SpatialVectorF>								mSolverLinkSpatialDeltaVels;	//link DeltaVels arising from solver
		PxArray<Cm::SpatialVectorF>								mSolverLinkSpatialImpulses;	//link impulses arising from solver.
		PxArray<Cm::SpatialVectorF>								mMotionAccelerations;	//link motion accelerations
		PxArray<Cm::SpatialVectorF>								mLinkIncomingJointForces;
		PxArray<Cm::SpatialVectorF>								mMotionAccelerationsInternal;	//link motion accelerations
		PxArray<Cm::SpatialVectorF>								mCorioliseVectors;		//link coriolise vector
		PxArray<Cm::SpatialVectorF>								mZAInternalForces;		//link internal spatial forces
		PxArray<Cm::SpatialVectorF>								mZAForces;				//link spatial zero acceleration force/ spatial articulated force
		PxArray<Cm::SpatialVectorF>								mJointTransmittedForce; 
		PxArray<ArticulationInternalConstraint>					mInternalConstraints;
		PxArray<ArticulationInternalLimit>						mInternalLimits;
		PxArray<ArticulationInternalTendonConstraint>			mInternalSpatialTendonConstraints;
		PxArray<ArticulationInternalTendonConstraint>			mInternalFixedTendonConstraints;
	    PxArray<ArticulationInternalMimicJoint>					mInternalMimicJoints;

		PxArray<PxReal>											mDeferredQstZ;

		PxArray<Cm::SpatialVectorF>				mDeltaMotionVector; //this is for TGS solver
		PxArray<PxTransform>					mPreTransform; //this is the previous transform list for links
		PxArray<TestImpulseResponse>			mResponseMatrixW;
		PxArray<SpatialMatrix>					mWorldSpatialArticulatedInertia;
		PxArray<PxMat33>						mWorldIsolatedSpatialArticulatedInertia;
		PxArray<PxReal>							mMasses;
		PxArray<InvStIs>						mInvStIs;
		PxArray<Cm::SpatialVectorF>				mIsW;
		PxArray<PxReal>							qstZIc;//jointForce - stZIc
		PxArray<PxReal>							qstZIntIc;
		PxArray<Cm::UnAlignedSpatialVector>		mJointAxis;
		PxArray<Cm::UnAlignedSpatialVector>		mMotionMatrix;
		PxArray<Cm::UnAlignedSpatialVector>		mWorldMotionMatrix;
		PxArray<Cm::SpatialVectorF>				mISInvStIS;
		PxArray<PxVec3>							mRw;

		PxArray<PxU32>							mNbStatic1DConstraints;
		PxArray<PxU32>							mNbStaticContactConstraints;

		PxArray<PxU32>							mStatic1DConstraintStartIndex;
		PxArray<PxU32>							mStaticContactConstraintStartIndex;

		PxArray<PxQuat>							mRelativeQuat;

		ArticulationLink*						mLinks;
		PxU32									mLinkCount;
		PxU32*									mPathToRootElements;
		PxU32									mNumPathToRootElements;
		ArticulationLinkData*					mLinksData;
		ArticulationJointCoreData*				mJointData;
		ArticulationSpatialTendon**				mSpatialTendons;
		PxU32									mNumSpatialTendons;
		PxU32									mNumTotalAttachments;
		ArticulationFixedTendon**				mFixedTendons;
		PxU32									mNumFixedTendons;
		ArticulationMimicJointCore**			mMimicJoints;
		PxU32									mNbMimicJoints;
		PxReal									mDt;
		PxU32									mDofs;
		const PxArticulationFlags*				mFlags;	// PT: PX-1399
		Cm::SpatialVector*						mExternalAcceleration;
		bool									mDataDirty; //this means we need to call commonInit()
		bool									mJointDirty; //this means joint delta velocity has been changed by contacts so we need to update joint velocity/joint acceleration 
		FeatherstoneArticulation*				mArticulation;

		PxArray<PxTransform>					mAccumulatedPoses;
		PxArray<PxQuat>							mDeltaQ;

		SpatialMatrix							mBaseInvSpatialArticulatedInertiaW;

		PxReal									mInvSumMass;
		PxVec3									mCOM;

		friend class FeatherstoneArticulation;
	};

	void ArticulationData::init()
	{
		//zero delta motion vector for TGS solver
		PxMemZero(getDeltaMotionVector(), sizeof(Cm::SpatialVectorF) * mLinkCount);
		PxMemZero(getPosIterMotionVelocities(), sizeof(Cm::SpatialVectorF) * mLinkCount);
		mJointDirty = false;
	}

	struct ScratchData
	{
	public:
		ScratchData()
		{
			motionVelocities = NULL;
			motionAccelerations = NULL;
			coriolisVectors = NULL;
			spatialZAVectors = NULL;
			externalAccels = NULL;
			compositeSpatialInertias = NULL;

			jointVelocities = NULL;
			jointAccelerations = NULL;
			jointForces = NULL;
			jointPositions = NULL;
			jointFrictionForces = NULL;
		}

		Cm::SpatialVectorF* motionVelocities;
		Cm::SpatialVectorF* motionAccelerations;
		Cm::SpatialVectorF* coriolisVectors;
		Cm::SpatialVectorF* spatialZAVectors;
		Cm::SpatialVector*	externalAccels;
		Dy::SpatialMatrix*	compositeSpatialInertias;

		PxReal*				jointVelocities;
		PxReal*				jointAccelerations;
		PxReal*				jointForces;
		PxReal*				jointPositions;
		PxReal*				jointFrictionForces;
	};

	struct InternalConstraintSolverData
	{
		const PxReal dt;
		const PxReal invDt;
		const PxReal elapsedTime;
		const PxReal erp;
		const bool isVelIter;
		const bool isTGS;
		PxU32 dofId;
		PxU32 complexId;
		PxU32 limitId;
		PxU32 articId;

		InternalConstraintSolverData(const PxReal dt_, const PxReal invDt_, const PxReal elapsedTime_,
			const PxReal erp_, bool velocityIteration_, bool isTGS_) :
			dt(dt_), invDt(invDt_), elapsedTime(elapsedTime_),
			erp(erp_), isVelIter(velocityIteration_),
			isTGS(isTGS_), dofId(0), complexId(0), limitId(0)
		{
		}

		PX_NOCOPY(InternalConstraintSolverData)
	};

	struct FixedTendonSolveData
	{
		ArticulationLink* links;
		ArticulationTendonJoint* tendonJoints;
		PxReal rootVel;
		PxReal rootImp;
		PxReal erp;
		PxReal error;
		PxReal limitError;
	};

#if PX_VC 
#pragma warning(push)   
#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	//Articulation dirty flag - used to tag which properties of the articulation are dirty. Used only to transfer selected data to the GPU...
	struct ArticulationDirtyFlag
	{
		enum Enum
		{
			eDIRTY_JOINTS = 1 << 0,
			eDIRTY_POSITIONS = 1 << 1,
			eDIRTY_VELOCITIES = 1 << 2,
			eDIRTY_FORCES = 1 << 3,
			eDIRTY_ROOT_TRANSFORM = 1 << 4,
			eDIRTY_ROOT_VELOCITIES = 1 << 5,
			eDIRTY_LINKS = 1 << 6,
			eIN_DIRTY_LIST = 1 << 7,
			eDIRTY_WAKECOUNTER = 1 << 8,
			eDIRTY_EXT_ACCEL = 1 << 9,
			eDIRTY_LINK_FORCE = 1 << 10,
			eDIRTY_LINK_TORQUE = 1 << 11,
			eDIRTY_JOINT_TARGET_VEL = 1 << 12,
			eDIRTY_JOINT_TARGET_POS = 1 << 13,
			eDIRTY_SPATIAL_TENDON = 1 << 14,
			eDIRTY_SPATIAL_TENDON_ATTACHMENT = 1 << 15,
			eDIRTY_FIXED_TENDON = 1 << 16,
			eDIRTY_FIXED_TENDON_JOINT = 1 << 17,
			eDIRTY_MIMIC_JOINT = 1 << 18,
			eDIRTY_VELOCITY_LIMITS = 1 <<19,
			eDIRTY_USER_FLAGS =  1 << 20,
			eNEEDS_KINEMATIC_UPDATE = 1 << 21,
			eALL = (1<<22)-1 
		};
	};

	PX_INLINE PX_CUDA_CALLABLE void computeArticJacobianAxes(PxVec3 row[3], const PxQuat& qa, const PxQuat& qb)
	{
		// Compute jacobian matrix for (qa* qb)  [[* means conjugate in this expr]]
		// d/dt (qa* qb) = 1/2 L(qa*) R(qb) (omega_b - omega_a)
		// result is L(qa*) R(qb), where L(q) and R(q) are left/right q multiply matrix

		const PxReal wa = qa.w, wb = qb.w;
		const PxVec3 va(qa.x, qa.y, qa.z), vb(qb.x, qb.y, qb.z);

		const PxVec3 c = vb*wa + va*wb;
		const PxReal d0 = wa*wb;
		const PxReal d1 = va.dot(vb);
		const PxReal d = d0 - d1;

		row[0] = (va * vb.x + vb * va.x + PxVec3(d, c.z, -c.y)) * 0.5f;
		row[1] = (va * vb.y + vb * va.y + PxVec3(-c.z, d, c.x)) * 0.5f;
		row[2] = (va * vb.z + vb * va.z + PxVec3(c.y, -c.x, d)) * 0.5f;

		if ((d0 + d1) != 0.0f)  // check if relative rotation is 180 degrees which can lead to singular matrix
			return;
		else
		{
			row[0].x += PX_EPS_F32;
			row[1].y += PX_EPS_F32;
			row[2].z += PX_EPS_F32;
		}
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE float compAng(PxReal swingYZ, PxReal swingW)
	{
		return 4.0f * PxAtan2(swingYZ, 1.0f + swingW);	// tan (t/2) = sin(t)/(1+cos t), so this is the quarter angle
	}


	PX_ALIGN_PREFIX(64)
	class FeatherstoneArticulation
	{
		PX_NOCOPY(FeatherstoneArticulation)
	public:
		// public interface

		explicit FeatherstoneArticulation(void*);
		~FeatherstoneArticulation();

		// get data sizes for allocation at higher levels
		void		getDataSizes(PxU32 linkCount, PxU32& solverDataSize, PxU32& totalSize, PxU32& scratchSize);

		bool		resize(const PxU32 linkCount);

		void		assignTendons(const PxU32 /*nbTendons*/, Dy::ArticulationSpatialTendon** /*tendons*/);

		void		assignTendons(const PxU32 /*nbTendons*/, Dy::ArticulationFixedTendon** /*tendons*/);

		void		assignMimicJoints(const PxU32 nbMimicJoints, Dy::ArticulationMimicJointCore** mimicJoints);

		PxU32		getDofs()	const;

		PxU32		getDof(const PxU32 linkID);

		bool		applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, bool& shouldWake);

		void		copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, const bool isGpuSimEnabled);

		static	PxU32	getCacheDataSize(PxU32 totalDofs, PxU32 linkCount);

		static	PxArticulationCache*	createCache(PxU32 totalDofs, PxU32 linkCount);

		void		packJointData(const PxReal* maximum, PxReal* reduced);

		void		unpackJointData(const PxReal* reduced, PxReal* maximum);

		void		initializeCommonData();

		//gravity as input, joint force as output
		void		getGeneralizedGravityForce(const PxVec3& gravity, PxArticulationCache& cache);

		//joint velocity as input, generalised force(coriolis and centrigugal force) as output
		void		getCoriolisAndCentrifugalForce(PxArticulationCache& cache);

		//external force as input, joint force as output
		void		getGeneralizedExternalForce(PxArticulationCache& /*cache*/);

		//joint force as input, joint acceleration as output
		void		getJointAcceleration(const PxVec3& gravity, PxArticulationCache& cache);

		//joint acceleration as input, joint force as out
		void		getJointForce(PxArticulationCache& cache);

		void		getDenseJacobian(PxArticulationCache& cache, PxU32 & nRows, PxU32 & nCols);

		//These two functions are for closed loop system
		void				getKMatrix(ArticulationJointCore* loopJoint, const PxU32 parentIndex, const PxU32 childIndex, PxArticulationCache& cache);

		void		getCoefficientMatrix(const PxReal dt, const PxU32 linkID, const PxContactJoint* contactJoints, const PxU32 nbContacts, PxArticulationCache& cache);

		void		getCoefficientMatrixWithLoopJoints(ArticulationLoopConstraint* lConstraints, const PxU32 nbJoints, PxArticulationCache& cache);

		bool		getLambda(ArticulationLoopConstraint* lConstraints, const PxU32 nbJoints, PxArticulationCache& cache, PxArticulationCache& rollBackCache, 
			const PxReal* jointTorque, const PxVec3& gravity, const PxU32 maxIter, const PxReal invLengthScale);

		void		getGeneralizedMassMatrix(PxArticulationCache& cache);

		void		getGeneralizedMassMatrixCRB(PxArticulationCache& cache);

		bool storeStaticConstraint(const PxSolverConstraintDesc& desc);

		bool		willStoreStaticConstraint() { return DY_STATIC_CONTACTS_IN_INTERNAL_SOLVER; }

		void		setRootLinearVelocity(const PxVec3& velocity);
		void		setRootAngularVelocity(const PxVec3& velocity);
		void		teleportRootLink();

		void		getImpulseResponse(
			PxU32 linkID,
			const Cm::SpatialVector& impulse,
			Cm::SpatialVector& deltaV) const;

		void	getImpulseResponse(
			PxU32 linkID,
			const Cm::SpatialVectorV& impulse,
			Cm::SpatialVectorV& deltaV) const;

		void		getImpulseSelfResponse(
			PxU32 linkID0,
			PxU32 linkID1,
			const Cm::SpatialVector& impulse0,
			const Cm::SpatialVector& impulse1,
			Cm::SpatialVector& deltaV0,
			Cm::SpatialVector& deltaV1) const;

		Cm::SpatialVectorV getLinkVelocity(const PxU32 linkID) const;

		Cm::SpatialVector getLinkScalarVelocity(const PxU32 linkID) const;

		Cm::SpatialVectorV getLinkMotionVector(const PxU32 linkID) const;

		//this is called by island gen to determine whether the articulation should be awake or sleep
		Cm::SpatialVector getMotionVelocity(const PxU32 linkID) const;

		Cm::SpatialVector getMotionAcceleration(const PxU32 linkID, const bool isGpuSimEnabled) const;

		PxReal getLinkMaxPenBias(const PxU32 linkID) const;

		PxReal getCfm(const PxU32 linkID) const;

		static PxU32 computeUnconstrainedVelocities(
			const ArticulationSolverDesc& desc,
			PxReal dt,
			PxU32& acCount,
			const PxVec3& gravity, 
			PxReal invLengthScale);

		static void computeUnconstrainedVelocitiesTGS(
			const ArticulationSolverDesc& desc,
			PxReal dt, const PxVec3& gravity,
			PxReal invLengthScale, bool externalForcesEveryTgsIterationEnabled);

		static PxU32 setupSolverConstraintsTGS(const ArticulationSolverDesc& articDesc,
			PxReal dt,
			PxReal invDt,
			PxReal totalDt);

		static void saveVelocity(FeatherstoneArticulation* articulation, Cm::SpatialVectorF* deltaV);

		static void saveVelocityTGS(FeatherstoneArticulation* articulation, PxReal invDtF32);

		static void updateBodies(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV, PxReal dt);

		static void updateBodiesTGS(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV, PxReal dt);

		static void updateBodies(FeatherstoneArticulation* articulation, Cm::SpatialVectorF* tempDeltaV, PxReal dt, bool integrateJointPosition);

		static void recordDeltaMotion(const ArticulationSolverDesc& desc, const PxReal dt, Cm::SpatialVectorF* deltaV, const PxReal totalInvDt);

		static void deltaMotionToMotionVelocity(const ArticulationSolverDesc& desc, PxReal invDt);

		static void applyTgsSubstepForces(const ArticulationSolverDesc& desc, const PxReal stepDt,
								Cm::SpatialVectorF* scratchExtForcesArticulatedYW);

		/**
		\brief Apply a spatial impulse to a specified link and optionally a joint impulse to the link's inbound joint.
		The impulses will be propagated from the specified link to the root.
		The function will add to the accumulated impulses {Q - S^T * Z} of each encountered joint as well as to the accumulated root link rootDeferredZ.
		\param[in] linkID is the index of the link that will have a spatial impulse applied to it. The joint impulse
		will be applied to the inbound joint of linkID.
		\param[in] linear is the linear part of the spatial impulse to apply to linkID.
		\param[in] angular is the angular part of the spatial impulse to apply to linkID.
		\param[in] jointImpulse is an array describing the joint impulse to apply to each dof of the inbound joint of linkID. 
		\note Set jointImpulse to NULL if the intention is to apply zero joint impulse.
		\note The effect of the link and joint impulse on  the spatial velocity and joint speed associated with any link may be queried by 
		calling pxcFsGetVelocity().  This applies the latest state of {Q - S^T * Z} to the pre-solver link spatial velocities and joint speeds.
		*/
		void pxcFsApplyImpulse(PxU32 linkID, aos::Vec3V linear, aos::Vec3V angular, const PxReal* jointImpulse = 0);
	
		/**
		\brief Apply spatial impulses to two specified links and and optionally joint impulses to their inbound joints.
		\note This function has the same outcome as calling pxcFsApplyImpulse(linkID, linear, angular, jointImpulse) and
		pxcFsApplyImpulse(linkID2, linear2, angular2, jointImpulse2) in sequence.
		\see pxcFsApplyImpulse for details.
		*/
		void pxcFsApplyImpulses(
			PxU32 linkID, const aos::Vec3V& linear, const aos::Vec3V& angular, const PxReal* jointImpulse,
			PxU32 linkID2, const aos::Vec3V& linear2, const aos::Vec3V& angular2, const PxReal* jointImpulse2);

		/**
		\brief Query the spatial velocity of a link by propagating the deferred root deltaZ from root to link 
		and applying the deferred QMinusSTZ for each dof encountered. The joint dof speeds of the inbound joint
		of the link are optionally computed. The computed spatial impulse of the link is the pre-solver
		link velocity incremented by a delta that arises from the application of the deferred impulses.
		\param[in] linkID is the target link.  The propagation from root to link will terminate at the specified link.
		\param[in] jointDofSpeeds are the pre-solver joint dof speeds incremented by a delta that arises from the 
		application of the deferred impulses.
		\note jointDofSpeeds may be NULL.
		\note The link and joint velocities are not changed by this query operation.
		\return The spatial velocity that is equal to the pre-solver spatial velocity of the specified link incremented by 
		a delta that arises from the application of the deferred root and link impulses.
		*/
		Cm::SpatialVectorV pxcFsGetVelocity(const PxU32 linkID, PxReal* jointDofSpeeds = NULL) const;

		void pxcFsGetVelocities(PxU32 linkID, PxU32 linkID1, Cm::SpatialVectorV& v0, Cm::SpatialVectorV& v1) const;

		Cm::SpatialVectorV pxcFsGetVelocityTGS(PxU32 linkID);

		const PxQuat& getDeltaQ(PxU32 linkID) const;

		/**
		/brief Propagate an acceleration (or velocity) from a parent link to a child link.
		This function assumes no pre-existing knowledge of Q_i - s_i^T * Z_i^A.  If this is  known
		it is recommended to use propagateAccelerationW() instead.
		\param[in] parentToChild is the vector from parent link to child link such that childLinkPos == parentLinkPos + parentToChild
		\param[in] parentLinkDeltaV is the parent link deltaV expressed in the world frame.
		\param[in] spatialInertia is the articulated spatial inertia of the child link.
		\param[in] Z is the impulse to apply to the child link. 
		\param[in] joinDofImpulses is an optional of array joint impulses to apply to each dof of the inbound joint of the child link.
		\param[in] motionMatrixW is the Mirtich equivalent of S_i of the child link's inbound joint.
		\param[in] dofCount is the number of dofs on the child links' inbound joint.
		\param[out] jointVelocity is incremented with the change arising from the propagated velocity and applied impulse. 
	    \note joinDofImpulses may be NULL if the intention is that zero joint impulse should be propagated.
		\note jointVelocity may be NULL.
		\return The spatial velocity of the child link.
		\note See Mirtich p121 and equations for propagating forces/applying accelerations and
			p141 for propagating velocities/applying impulses.
		*/
		static Cm::SpatialVectorF propagateVelocityW(
			const PxVec3& parentToChild, const Cm::SpatialVectorF& parentLinkDeltaV,
			const Dy::SpatialMatrix& spatialInertia, const Cm::SpatialVectorF& Z,
			const PxReal* joinDofImpulses, const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrixW, const PxU32 dofCount, 
			PxReal* jointVelocity);



		bool applyCacheToDest(ArticulationData& data, PxArticulationCache& cache,
			PxReal* jVelocities, PxReal* jPosition, PxReal* jointForce,
			PxReal* jTargetPositions, PxReal* jTargetVelocities,
			const PxArticulationCacheFlags flag, bool& shouldWake);

		PX_FORCE_INLINE	ArticulationData&		getArticulationData()			{ return mArticulationData;	}
		PX_FORCE_INLINE	const ArticulationData&	getArticulationData()	const	{ return mArticulationData;	}

		PX_FORCE_INLINE void setGpuDirtyFlag(ArticulationDirtyFlag::Enum flag)
		{
			mGPUDirtyFlags |= flag;
		}
		//void	setGpuRemapId(const PxU32 id) { mGpuRemapId = id; }
		//PxU32	getGpuRemapId() { return mGpuRemapId; }

		static PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVectorF translateSpatialVector(const PxVec3& offset, const Cm::SpatialVectorF& vec)
		{
			return Cm::SpatialVectorF(vec.top, vec.bottom + offset.cross(vec.top));
		}

		static PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::UnAlignedSpatialVector translateSpatialVector(const PxVec3& offset, const Cm::UnAlignedSpatialVector& vec)
		{
			return Cm::UnAlignedSpatialVector(vec.top, vec.bottom + offset.cross(vec.top));
		}

		static PX_FORCE_INLINE PxMat33 constructSkewSymmetricMatrix(const PxVec3 r)
		{
			return PxMat33(PxVec3(0.0f, r.z, -r.y),
				PxVec3(-r.z, 0.0f, r.x),
				PxVec3(r.y, -r.x, 0.0f));
		}

		bool raiseGPUDirtyFlag(ArticulationDirtyFlag::Enum flag)
		{
			bool nothingRaised = !(mGPUDirtyFlags);
			mGPUDirtyFlags |= flag;
			return nothingRaised;
		}

		void clearGPUDirtyFlags()
		{
			mGPUDirtyFlags = 0;
		}

	public:
		void constraintPrep(ArticulationLoopConstraint* lConstraints, PxU32 nbJoints,
			PxSolverConstraintPrepDesc& prepDesc, PxSolverBody& sBody,
			PxSolverBodyData& sBodyData, PxSolverConstraintDesc* desc, PxConstraintAllocator& allocator);

		void updateArticulation(const PxVec3& gravity, PxReal invLengthScale, bool externalForcesEveryTgsIterationEnabled);

		void computeUnconstrainedVelocitiesInternal(
			const PxVec3& gravity, PxReal invLengthScale, bool externalForcesEveryTgsIterationEnabled = false);

		//copy joint data from fromJointData to toJointData
		void copyJointData(const ArticulationData& data, PxReal* toJointData, const PxReal* fromJointData);

		PxU32 computeDofs();
		//this function calculates motion subspace matrix(s) for all tree joint
		template<bool immediateMode = false>
		void jcalc(ArticulationData& data);

		//this function calculates loop joint constraint subspace matrix(s) and active force
		//subspace matrix
		void jcalcLoopJointSubspace(ArticulationJointCore* joint, ArticulationJointCoreData& jointDatum, SpatialSubspaceMatrix& T,
			const Cm::UnAlignedSpatialVector* jointAxis);

		void computeSpatialInertia(ArticulationData& data);

		//compute zero acceleration force
		void computeZ(const ArticulationData& data, const PxVec3& gravity, ScratchData& scratchData);
		void computeZD(const ArticulationData& data, const PxVec3& gravity, ScratchData& scratchData);

		void solveInternalConstraints(const PxReal dt, const PxReal invDt,
			bool velocityIteration, bool isTGS, const PxReal elapsedTime, const PxReal biasCoefficient, bool residualReportingActive);

		void solveInternalJointConstraints(const PxReal dt, const PxReal invDt,
			bool velocityIteration, bool isTGS, const PxReal elapsedTime, const PxReal biasCoefficient, bool residualReportingActive);

	private:
		Cm::SpatialVectorF solveInternalJointConstraintRecursive(InternalConstraintSolverData& data, const PxU32 linkID,
			const Cm::SpatialVectorF& parentDeltaV, const bool isTGS, const bool isVelIter, const bool residualReportingActive);
	public:
		void solveInternalSpatialTendonConstraints(bool isTGS);

		void solveInternalFixedTendonConstraints(bool isTGS);

		void setupInternalMimicJointConstraints();
		void solveInternalMimicJointConstraints(const PxReal dt, const PxReal invDt, const bool velocityIteration, const bool isTGS, const PxReal biasCoefficient);

		void writebackInternalConstraints(bool isTGS);

		void concludeInternalConstraints(bool isTGS);

		//compute coriolis force
		void computeC(ArticulationData& data, ScratchData& scratchData);

		//compute relative transform child to parent
		/**
		\brief	a) copy the latest link pose to a handy array
				b) update the link separation vectors using the latest link poses.
				c) update the motion matrices in the world frame using the latest link poses.	
		\param[in] links is an array of articulation links that contain the latest link poses.
		\param[in] linkCount is the number of links in the articulation
		\param[in] jointCoreDatas is an array of joint descriptions 
		\param[in] jointDofMotionMatrices is an array of motion matrices in the joint frame.
		\param[out] linkAccumulatedPoses is an array used to store the latest link poses taken from ArticulationLink::PxsBodyCore. 
		\param[out] linkRws is an array of link separations.
		\param[out] jointDofmotionMatricesW is an array of motion matrices in the world frame.
		*/
		static void computeRelativeTransformC2P(
			const ArticulationLink* links, const PxU32 linkCount, const ArticulationJointCoreData* jointCoreDatas,
			const Cm::UnAlignedSpatialVector* jointDofMotionMatrices,
			PxTransform* linkAccumulatedPoses, PxVec3* linkRws, Cm::UnAlignedSpatialVector* jointDofmotionMatricesW);

		//compute relative transform child to base
		void computeRelativeTransformC2B(ArticulationData& data);

		void computeLinkVelocities(ArticulationData& data, ScratchData& scratchData);

	
		/**
		/brief Prepare links for the next timestep.
		\param[in] dt is the timestep of the current simulation step that will advance sim from t to t+dt.
		\param[in] invLengthScale is the reciprocal of the lengthscale used by the simulation.
		\param[in] gravity is the gravitational acceleration to apply to all links.
		\param[in] fixBase determines whether the root link is to be fixed to the world (true) or will move freely (false).
		\param[in] linkCount is the total number of links in the articulation
		\param[in] linkAccumulatedPosesW is the pose of each link, specified in the world frame.
		\param[in,out] linkExternalAccelsW is the external acceleration to apply to each link, specified in the world frame. Only modified if externalForcesEveryTgsIterationEnabled is true.
		\param[in] linkRsW is the vector from each parent link to each child link, specified in the world frame.
		\param[in] jointDofMotionMatricesW is the motion matrix of each dof, specified in the world frame.
		\param[in] jointCoreData is the ArticulationJointCoreData instance of each link in the articulation.
		\param[in] externalForcesEveryTgsIterationEnabled if true, skip the application of external forces in this function and instead prepare change linkExternalAccelsW from acceleration to impulses
		\param[in,out] linkData is the ArticulationLinkData instance of each link in the articulation.
		\param[in,out] links is the ArticulationLink instance of each link in the articulation.
		\param[in,out] linkMotionAccelerationsW is the acceleration of each link, specified in the world frame.
		\param[out] linkMotionVelocitiesW is velocity of each link computed from the parent link velocity and joint velocity of the inbound joint. Specified in the world frame.
		\param[out] linkZAForcesExtW is the computed spatial zero acceleration force of each link, accounting for only external forces applied to the links.  Specified in the world frame.
		\param[out] linkZAForcesIntW is the computed spatial zero acceleration force of each link, accounting for only internal forces applied to the links.  Specified in the world frame.
		\param[out] linkCoriolisVectorsW is the computed coriolis vector of each link.   Specified in the world frame.
		\param[out] linkIsolatedArticulatedInertiasW is the inertia tensor (I) for the trivial sub-chain of each link. Specified in the world frame.
		\param[out] linkMasses is the mass of each link. 
		\param[out] linkSpatialArticulatedInertiasW is the spatial matrix containing the inertia tensor I and the mass matrix M for the trivial sub-chain of each link.  Specified in the world frame.
		\param[in,out] jointDofVelocities is the velocity of each degree of freedom. Will be updated in case joint velocity limits are violated (see note below).
		\param[out] rootPreMotionVelocityW is assigned the spatial velocity of the root link.		
		\param[out] comW is the centre of mass of the assembly of links, specified in the world frame.
		\param[out] invSumMass is the reciprocal of the total mass of all links.
		\note invLengthScale should have value 1/100 for centimetres scale and 1/1 for metres scale.
		\note If fixBase is true, the root link is assigned zero velocity.  If false, the root link inherits the velocity of the associated body core.
		\note If fixBase is true, the root link is assigned zero acceleration. If false, the acceleration is propagated from the previous simulation step. The acceleration 
		of all other links is left unchanged.
		\note If fix base is true, the root link is assigned a zero coriolis vector.
		\note ArticulationLinkData::maxPenBias of each link inherits the value of the associated PxsBodyCore::maxPenBias.
		\note ArticulationLink::cfm of each link is assigned the value PxsBodyCore::cfmScale*invLengthScale, except for the root link. The root link is 
		assigned a value of 0 if it is fixed to the world ie fixBase == true.
		\note The spatial zero acceleration force accounts for the external acceleration; the damping force arising from the velocity and from the velocity 
		that will accumulate from the external acceleration; the scaling force that will bring velocity back to the maximum allowed velocity if velocity exceeds the maximum allowed.
		\note If the velocity of any degree of freedom exceeds the maximum velocity of the associated joint, the velocity of each degree of freedom will be scaled by a common factor so that none exceeds the maximum.
		*/
		static void computeLinkStates(
			const PxF32 dt, const PxReal invLengthScale, const PxVec3& gravity,
			const bool fixBase,
			const PxU32 linkCount,
			const PxTransform* linkAccumulatedPosesW, Cm::SpatialVector* linkExternalAccelsW, const PxVec3* linkRsW, const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW,
			const  Dy::ArticulationJointCoreData* jointCoreData, bool externalForcesEveryTgsIterationEnabled,
			Dy::ArticulationLinkData *linkData, Dy::ArticulationLink* links, 
			Cm::SpatialVectorF* linkMotionAccelerationsW, Cm::SpatialVectorF* linkMotionVelocitiesW, 
			Cm::SpatialVectorF* linkZAForcesExtW, Cm::SpatialVectorF* linkZAForcesIntW, Cm::SpatialVectorF* linkCoriolisVectorsW, 
			PxMat33* linkIsolatedArticulatedInertiasW, PxF32* linkMasses, Dy::SpatialMatrix* linkSpatialArticulatedInertiasW, 
			PxReal* jointDofVelocities,
			Cm::SpatialVectorF& rootPreMotionVelocityW, PxVec3& comW, PxF32& invSumMass);

		/**
		\brief Propagate articulated z.a. spatial force and articulated spatial inertia from parent link to child link.
		Repeated calls to computePropagateSpatialInertia_ZA_ZIc allow z.a. spatial force and articulated spatial inertia 
		to be propagated from tip to root. 
		The computation proceeds by considering a link/joint pair composed of a child link and its
		incoming joint. 
		The articulated z.a. spatial force is split into an internal and external part.  Gravity is added to the external
		part, while user-applied external joint forces are added to the internal part.  
		\note Reference maths can be found in Eq 4.29 in Mirtich thesis.
		\note Mirtich works in the joint frame while every quantity here is in the world frame.
		\note linkArticulatedInertia has equivalent I_i^A in Mirtich
		\note jointMotionMatrix has equivalent s_i and its transpose is s_i^T.
		\param[in] jointType is the type of joint
		\param[in] nbJointDofs is the number of dofs supported by the joint.
		\param[in] jointMotionMatricesW is an array of motion matrices with one entry per dof.
		\param[in] jointISW is a cached term linkArticulatedInertia*jointDofMotionMatrix with one entry per dof.
		\param[in] jointTargetArmatures is an array of armature values with one entry per dof.
		\param[in] jointExternalForces is an array of user-applied external forces applied to the joint with one entry per dof. Can be NULL, in which case zero forces are assumed.
		\param[in] linkArticulatedInertiaW is the articulated inertia of the link.
		\param[in] linkZExtW is the external articulated z.a. force of the link.
		\param[in] linkZIntIcW is the sum of the internal z.a force of the link and linkArticulatedInertia*coriolisForce
		\param[out] linkInvStISW will be computed as 1/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix]
		\param[out]	jointDofISInvStISW will be computed as linkArticulatedInertia*jointMotionMatrix^T/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix]
		\param[out] jointDofMinusStZExtW will be computed as [-jointMotionMatrix^T * ZExt]
		\param[out] jointDofQStZIntIcW will be computed as [jointForce - jointMotionMatrix^T *ZIntIc]
		\param[out] deltaZAExtParent is a term that is to be translated to parent link and added to the ZExt value of the parent link.
		\param[out] deltaZAIntIcParent is a term that is to be translated to parent link and added to the ZInt value of the parent link.
		\return A term to be translated to parent link and added to the articulated inertia of the parent.
		*/
		static SpatialMatrix computePropagateSpatialInertia_ZA_ZIc
			(const PxArticulationJointType::Enum jointType, const PxU8 nbJointDofs,
			 const Cm::UnAlignedSpatialVector* jointMotionMatricesW, const Cm::SpatialVectorF* jointISW, 	
			 const PxReal* jointTargetArmatures, const PxReal* jointExternalForces, 
			 const SpatialMatrix& linkArticulatedInertiaW, 
			 const Cm::SpatialVectorF& linkZExtW, const Cm::SpatialVectorF& linkZIntIcW, 
			 InvStIs& linkInvStISW, Cm::SpatialVectorF* jointDofISInvStISW, 
			 PxReal* jointDofMinusStZExtW, PxReal* jointDofQStZIntIcW,
			 Cm::SpatialVectorF& deltaZAExtParent, Cm::SpatialVectorF& deltaZAIntIcParent);

		/**
		\brief Propagate articulated z.a. spatial force and articulated spatial inertia from child link to parent link. 
		Repeated calls to computePropagateSpatialInertia_ZA_ZIc allow z.a. spatial force and articulated spatial inertia 
		to be propagated from tip to root. 
		The computation proceeds by considering a link/joint pair composed of a child link and its incoming joint.
		\note Reference maths can be found in Eq 4.29 in Mirtich thesis.
		\note Mirtich works in the joint frame while every quantity here is in the world frame.
		\note linkArticulatedInertia has equivalent I_i^A in Mirtich
		\note jointMotionMatrix has equivalent s_i
		\param[in] jointType is the type of joint
		\param[in] nbJointDofs is the number of dofs supported by the joint.
		\param[in] jointMotionMatrices is an array of motion matrices with one entry per dof.
		\param[in] jointIs is a cached term linkArticulatedInertia*jointDofMotionMatrix with one entry per dof.
		\param[in] jointTargetArmatures is an array of armature values with one entry per dof.
		\param[in] jointExternalForces is an array of user-applied external forces applied to the joint with one entry per dof.			
		\param[in] linkArticulatedInertia is the articulated inertia of the link.
		\param[in] ZIc is the sum of the z.a force of the link and linkArticulatedInertia*coriolisForce.
		\param[out] invStIs will be computed as 1/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix]
		\param[out]	isInvD will be computed as linkArticulatedInertia*jointMotionMatrix^T/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix]
		\param[out] qstZIc will be computed as [jointForce - jointMotionMatrix^T *ZIc]/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix]
		\param[out] deltaZParent is a term that is to be translated to parent link and added to the articulated z.a force of the parent link.
		\return A term to be translated to parent link and added to the articulated inertia of the parent.
		*/
		static SpatialMatrix computePropagateSpatialInertia_ZA_ZIc_NonSeparated
			(const PxArticulationJointType::Enum jointType, const PxU8 nbJointDofs, 
			 const Cm::UnAlignedSpatialVector* jointMotionMatrices, const Cm::SpatialVectorF* jointIs, 
			 const PxReal* jointTargetArmatures, const PxReal* jointExternalForces, 
			 const SpatialMatrix& linkArticulatedInertia, 
 			 const Cm::SpatialVectorF& ZIc, 
			 InvStIs& invStIs, Cm::SpatialVectorF* isInvD, 
			 PxReal* qstZIc,
			 Cm::SpatialVectorF& deltaZParent);

		/*
		\brief Propagate articulated spatial inertia (but not the  articulated z.a. spatial force) from child link to parent link. 
		Repeated calls to computePropagateSpatialInertia allow the articulated spatial inertia 
		to be propagated from tip to root. 
		The computation proceeds by considering a link/joint pair composed of a child link and its
		incoming joint.
		\param[in] jointType is the type of joint
		\param[in] nbJointDofs is the number of dofs supported by the joint.
		\param[in] linkArticulatedInertia is the articulated inertia of the link.
		\param[in] jointMotionMatrices is an array of motion matrices with one entry per dof.
		\param[in] jointIs is a cached term linkArticulatedInertia*jointDofMotionMatrix with one entry per dof.
		\param[out] invStIs will be computed as 1/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix]
		\param[out]	isInvD will be computed as linkArticulatedInertia*jointMotionMatrix^T/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix]
		\return A term to be translated to parent link and added to the articulated inertia of the parent.
		*/
		static SpatialMatrix computePropagateSpatialInertia(
			const PxArticulationJointType::Enum jointType, const PxU8 nbDofs,
			const SpatialMatrix& linkArticulatedInertia, const Cm::UnAlignedSpatialVector* jointMotionMatrices,
			const Cm::SpatialVectorF* jointIs, 
			InvStIs& invStIs, Cm::SpatialVectorF* isInvD);

		static void transformInertia(const SpatialTransform& sTod, SpatialMatrix& inertia);

		static void translateInertia(const PxMat33& offset, SpatialMatrix& inertia);

		static PxMat33 translateInertia(const PxMat33& inertia, const PxReal mass, const PxVec3& t);

		/*
		\brief Propagate articulated spatial inertia and articulated z.a. spatial force from tip to root.
		\param[in] links is an array of articulation links with size denoted by linkCount.
		\param[in] linkCount is the number of articulation links. 
		\param[in] linkRsW is an array of link separations in the world frame with one entry per link.
		\param[in] jointData is an array of joint descriptions with one entry per joint.
		\param[in] jointDofMotionMatricesW ins an array of motion matrices in the world frame with one entry per dof.
		\param[in] linkCoriolisVectorsW is an array fo coriolis terms with one entry per link. 
		\param[in] jointDofForces is an array of joint-space forces/torques to be applied to joints with one entry per dof. Can be NULL, in which case zero force is assumed for all joints.
		\param[out] jointDofIsW is a cached term linkArticulatedInertia*jointDofMotionMatrix to be computed with one entry per dof.
		\param[out] linkInvStIsW will be computed as 1/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix] with one entry per link.
		\param[out] jointDofISInvStIS will be computed as linkArticulatedInertia*jointMotionMatrix^T/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix] with one entry per dof.
		\param[out] joIntDofMinusStZExtW will be computed as [-jointMotionMatrix^T * ZExt] with one entry per dof.
		\param[out] jointDofQStZIntIcW will be computed as [jointForce - jointMotionMatrix^T *ZIntIc] with one entry per dof.
		\param[in,out] linkZAExtForcsW is the articulated z.a spatial force of each link arising from external forces.
		\param[in,out] linkZAIntForcesW is the articulated z.a spatial force of each link arising from internal forces.
		\param[in,out] linkSpatialArticulatedInertiaW is the articulated spatial inertia of each link.
		\param[out] baseInvSpatialArticulatedInertiaW is the inverse of the articulated spatial inertia of the root link.
		*/
		static void computeArticulatedSpatialInertiaAndZ
			(const ArticulationLink* links, const PxU32 linkCount, const PxVec3* linkRsW,
			 const ArticulationJointCoreData* jointData,
			 const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW,
			 const Cm::SpatialVectorF* linkCoriolisVectorsW, const PxReal* jointDofForces,
			 Cm::SpatialVectorF* jointDofIsW, InvStIs* linkInvStIsW, Cm::SpatialVectorF* jointDofISInvStIS, PxReal* joIntDofMinusStZExtW, PxReal* jointDofQStZIntIcW, 
			 Cm::SpatialVectorF* linkZAExtForcesW, Cm::SpatialVectorF* linkZAIntForcesW, SpatialMatrix* linkSpatialArticulatedInertiaW, 
			 SpatialMatrix& baseInvSpatialArticulatedInertiaW);

		void computeArticulatedSpatialInertiaAndZ_NonSeparated(ArticulationData& data, ScratchData& scratchData);

		void computeArticulatedSpatialInertia(ArticulationData& data);

		/*
		\brief Compute the response matrix of each link of an articulation.
		\param[in] articulationFlags describes whether the articulation has a fixed base.
		\param[in] linkCount is the number of links in the articulation.
		\param[in] jointData is an array of joint descriptions with one entry per joint.
		\param[in] baseInvSpatialArticulatedInertiaW is the inverse of the articulated spatial inertia of the root link.
		\param[in] linkRsW is an array of link separations (childPos - parentPos) in the world frame with one entry per link.
		\param[in] jointDofMotionMatricesW is an array of motion matrices with one entry per dof.
		\param[in] jointDofISW is a cached term linkArticulatedInertia*jointDofMotionMatrix to be computed with one entry per dof.
		\param[in] linkInvStISW will be computed as 1/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix] with one entry per link.
		\param[in] jointDofIsInvDW will be computed as linkArticulatedInertia*jointMotionMatrix^T/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix] with one entry per dof.
		\param[out] links is an array of articulation links with one entry per link.  The cfm value of each link will be updated.
		\param[out] linkResponsesW if an array of link responses with one entry per link.
		*/
		static void computeArticulatedResponseMatrix
			(const PxArticulationFlags& articulationFlags, const PxU32 linkCount, 
			 const ArticulationJointCoreData* jointData,
			 const SpatialMatrix& baseInvArticulatedInertiaW, 
			 const PxVec3* linkRsW, const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW,
			 const Cm::SpatialVectorF* jointDofISW, const InvStIs* linkInvStISW, const Cm::SpatialVectorF* jointDofIsInvDW, 
			 ArticulationLink* links, TestImpulseResponse* linkResponsesW);

		void computeArticulatedSpatialZ(ArticulationData& data, ScratchData& scratchData);


		/**
		\brief Compute the joint acceleration
		\note Reference maths found in Eq 4.27 of Mirtich thesis.
		\param[in] pMotionAcceleration is the acceleration of the parent link already transformed into the (child) link frame.
		\param[in] jointDofISW is an array of cached terms linkArticulatedInertia*jointDofMotionMatrix with one entry per dof.
		\param[in] linkInvStISW is a cached term equivalent to 1/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix]
		\param[in] jointDofQStZIcW is an array of cached terms equivlaent to 
					[jointExternalForce - jointDofMotionMatrix^T * (zeroAccelSpatialForce + spatialInertia*coriolisForce] with one entry per dof.
		\param[out] jointAcceleration is an array of output joint dof accelerations equivalent to Eq 4.27 in Mirtich thesis.
		*/
		static void computeJointAccelerationW(const PxU8 nbJointDofs,
			const Cm::SpatialVectorF& pMotionAcceleration, const Cm::SpatialVectorF* jointDofISW, const InvStIs& linkInvStISW,
			const PxReal* jointDofQStZIcW, PxReal* jointAcceleration);

		//compute joint acceleration, joint velocity and link acceleration, velocity based
		//on spatial force and spatial articulated inertia tensor

		/**
		\brief Compute joint and link accelerations. 
		Accelerations are computed by iterating from root to tip using the formulae in Mirtich Figure 4.8 to compute first 
		the joint dof acceleration and then the link acceleration. 
		The accelerations are used to forward integrate the link and joint velocities.
		This function may be used to determine either the effect of external forces only or the effect of the external and internal forces combined.
		The function may not be used to determine the effect of internal forces.  For internal forces only use computeLinkInternalAcceleration().
		If external forces only are to be considered then set doIC to false to avoid adding the Coriolis vector to the link acceleration.  This is important
		because Coriolis forces are accounted as part of the update arising from internal forces.
		\param[in] doIC determines whether the link Coriolis force is added to the link acceleration. 
			Set to false if considering external forces only and true if considering the combination of internal and external forces. 
		\param[in] dt is the timestep used to accumulate joint/link velocities from joint/link accelerations. 
        \param[in] fixBase describes whether the root of the articulation is fixed or free to rotate and translate.
        \param[in] links is an array of articulation links with one entry for each link.
        \param[in] linkCount is the number of links in the articulation.
        \param[in] jointDatas is an array of joint descriptions with one entry per joint.
        \param[in] linkSpatialZAForcesW is an array of spatial z.a. forces arising from the forces acting on each link with one entry for each link.
			linkSpatialZAForces will either be internal z.a forces or the sum of internal and external forces.
        \param[in] linkCoriolisForcesW is an array of coriolis forces with one entry for each link.
        \param[in] linkRsW is an array of link separations with one entry for each link.
        \param[in] jointDofMotionMatricesW is an array of motion matrices with one entry per joint dof.
        \param[in] baseInvSpatialArticulatedInertiaW is the inverse of the articulated spatial inertia of the root link. 
   		\param[in] linkInvStISW is  1/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix] with one entry per link.
        \param[in] jointDofISW linkArticulatedInertia*jointMotionMatrix^T/ with one entry per joint dof.
        \param[in] jointDofQStZIcW has one of two forms: 
				a) [-jointDofMotionMatrix^T * linkSpatialZAForceExternal] 
				b) [jointDofForce - jointDofMotionMatrix^T*(linkSpatialZAForceTotal + linkSpatialInertia*linkCoriolisForce)]
				with one entry for each joint dof.			
		\param[out] linkMotionAccelerationsW is an array of link accelerations with one entry per link. The link accelerations are computed
				using the formula in Figure 4.8 of the Mirtich thesis.
		\param[in,out] linkMotionVelocitiesW is an array of link velocities that are forward integrated by dt using the link accelerations.
		\param[out]  jointDofAccelerations is an array of joint dof accelerations with one entry per link. The joint dof accelerations are computed
				using the formula in Figure 4.8 of the Mirtich thesis.
		\param[in,out] jointDofVelocities is an array of joint dof velocities that are forward integrated by dt using the joint dof accelerations.
		\param[out] jointDofNewVelocities is another array of joint dof velocities that are forward integrated by dt using the joint dof accelerations.
		\note If doIC is false then linkSpatialZAForces must be the external z.a. forces and jointDofQstZics must be [-jointDofMotionMatrix^T * linkSpatialZAForceExternal] 
		\note If doIC is true then  linkSpatialZAForces must be the internal z.a. forces and jointDofQstZics must be [jointDofForce - jointDofMotionMatrix^T*(linkSpatialZAForceTotal + linkSpatialInertia*linkCoriolisForce)]
		*/
		static void computeLinkAcceleration
			(const bool doIC, const PxReal dt,
			 const bool fixBase,
			 const ArticulationLink* links, const PxU32 linkCount, const ArticulationJointCoreData* jointDatas,
			 const Cm::SpatialVectorF* linkSpatialZAForcesW, const Cm::SpatialVectorF* linkCoriolisForcesW, const PxVec3* linkRsW, 
			 const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW,
			 const SpatialMatrix& baseInvSpatialArticulatedInertiaW,
			 const InvStIs* linkInvStISW, 
			 const Cm::SpatialVectorF* jointDofISW, const PxReal* jointDofQStZIcW,
			 Cm::SpatialVectorF* linkMotionAccelerationsW, Cm::SpatialVectorF* linkMotionVelocitiesW, 
			 PxReal* jointDofAccelerations, PxReal* jointDofVelocities, PxReal* jointDofNewVelocities);

		/**
		\brief Compute joint and link accelerations arising from internal z.a. forces.
		Accelerations are computed by iterating from root to tip using the formulae in Mirtich Figure 4.8 to compute first 
		the joint dof acceleration and then the link acceleration. 
		The accelerations are used to forward integrate the link and joint velocities.
		The resulting link velocities are rescaled if any link violates the maximum allowed linear or angular velocity.
		\param[in] dt is the timestep used to accumulate joint/link velocities from joint/link accelerations. 
        \param[in] fixBase describes whether the root of the articulation is fixed or free to rotate and translate.
		\param[in] comW is the centre of mass of the ensemble of links in the articulation. com is used only to enforce the max linear and angular velocity.
		\param[in] invSumMass is the inverse of the mass sum of the ensemble of links in the articulation. invSumMass is used only to enforce the max linear and angular velocity.
		\param[in] linkMaxLinearVelocity is the maximum allowed linear velocity of any link. The link linear velocities are rescaled to ensure none breaches the limit.
		\param[in] linkMaxAngularVelocity is the maximum allowed angular velocity of any link. The link angular velocities are rescaled to ensure none breaches the limit.
		\param[in] linkIsolatedSpatialArticulatedInertiasW is an array of link inertias.  The link inertias are used only to enforce the max linear and angular velocity.
        \param[in] baseInvSpatialArticulatedInertiaW is the inverse of the articulated spatial inertia of the root link. 
        \param[in] links is an array of articulation links with one entry for each link. 
        \param[in] linkCount is the number of links in the articulation.
		\param[in] linkMasses is an array of link masses with one entry per link.
		\param[in] linkRsW is an array of link separations with one entry per link.
		\param[in] linkAccumulatedPosesW is an array of link poses with one entry per link.
		\param[in] linkSpatialZAIntForcesW is an array of spatial z.a. forces arising from internal forces only with one netry per link.
		\param[in] linkCoriolisVectorsW is an array of link Coriolis forces with one entry per link.
		\param[in] jointDatas is an array of joint descriptions with one entry per joint.
		\param[in] jointDofMotionMatricesW is an array of motion matrices with one entry per dof.
   		\param[in] linkInvStISW is  1/[jointMotionMatrix^T * linkArticulatedInertia * jointMotionMatrix] with one entry per link.
        \param[in] jointDofISW linkArticulatedInertia*jointMotionMatrix^T with one entry per joint dof.
        \param[in] jointDoQStZIntIcW has form: [jointDofForce - jointDofMotionMatrix^T*(linkSpatialZAForceInternal + linkSpatialInertia*linkCoriolisForce)]
				with one entry for each joint dof.	
		\param[in,out] linkMotionAccelerationsW accumulates with the computed acceleration arising from internal forces.
		\param[out] linkMotionAccelerationIntsW is the computed acceleration arising from internal forces.
		\param[in,out] jointDofVelocities is an array of joint dof velocities that are forward integrated by dt using the joint dof accelerations arising from internal forces.
		\param[out] jointDofNewVelocities is another array of joint dof velocities that are forward integrated by dt using the joint dof accelerations arising from internal forces.
		\note computeLinkInternalAcceleration must be called after computeLinkAcceleration to allow the effect of internal forces to be accumulated on top of external forces.
		*/
		static void computeLinkInternalAcceleration
			(const PxReal dt,
			 const bool fixBase,
			 const PxVec3& comW, const PxReal invSumMass, const PxReal linkMaxLinearVelocity, const PxReal linkMaxAngularVelocity, const PxMat33* linkIsolatedSpatialArticulatedInertiasW, 
			 const SpatialMatrix& baseInvSpatialArticulatedInertiaW,	
			 const ArticulationLink* links, const PxU32 linkCount, 
			 const PxReal* linkMasses, const PxVec3* linkRsW, const PxTransform* linkAccumulatedPosesW,
			 const Cm::SpatialVectorF* linkSpatialZAIntForcesW, const Cm::SpatialVectorF* linkCoriolisVectorsW,
			 const ArticulationJointCoreData* jointDatas, const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW,
			 const InvStIs* linkInvStISW, const Cm::SpatialVectorF* jointDofISW, const PxReal* jointDoQStZIntIcW,
			 Cm::SpatialVectorF* linkMotionAccelerationsW, Cm::SpatialVectorF* linkMotionAccelerationIntsW, Cm::SpatialVectorF* linkMotionVelocitiesW, 
			 PxReal* jointDofAccelerations, PxReal* jointDofInternalAccelerations, PxReal* jointDofVelocities, PxReal* jointDofNewVelocities);

		/**
		\brief For each link compute the incoming joint force in the joint frame.
		\param[in] linkCount is the number of links in the articulation
		\param[in] linkZAForcesExtW are the external spatial zero acceleration forces in the world frame with one entry per link.
		\param[in] linkZAForcesIntW are the internal spatial zero acceleration forces in the world frame with one entry per link.
		\param[in] linkMotionAccelerationsW are the spatial accelerations ion the world framewith one entry per link.
		\param[in] linkSpatialInertiasW are the spatial articulated inertias in the world frame with one entry per link.
		\param[out] linkIncomingJointForces are the incoming joint forces specified in the joint frame that are applied to each link.
		*/
		static void computeLinkIncomingJointForce(
				const PxU32 linkCount,						
				const Cm::SpatialVectorF* linkZAForcesExtW,	const Cm::SpatialVectorF* linkZAForcesIntW,
				const Cm::SpatialVectorF* linkMotionAccelerationsW, const SpatialMatrix* linkSpatialInertiasW,
				Cm::SpatialVectorF* linkIncomingJointForces);

		//void computeTempLinkAcceleration(ArticulationData& data, ScratchData& scratchData);
		void computeJointTransmittedFrictionForce(ArticulationData& data, ScratchData& scratchData);

		static Cm::SpatialVectorF getDeltaVWithDeltaJV(const bool fixBase, const PxU32 linkID,
			const ArticulationData& data, Cm::SpatialVectorF* Z,
			PxReal* jointVelocities);

		//impulse need to be in the linkID space
		static void getZ(const PxU32 linkID, const ArticulationData& data, 
			Cm::SpatialVectorF* Z, const Cm::SpatialVectorF& impulse);

		//This method use in impulse self response. The input impulse is in the link space
		static Cm::SpatialVectorF getImpulseResponseW(
			const PxU32 linkID,
			const ArticulationData& data,
			const Cm::SpatialVectorF& impulse);

		//This method use in impulse self response. The input impulse is in the link space
		static Cm::SpatialVectorF getImpulseResponseWithJ(
			const PxU32 linkID,
			const bool fixBase,
			const ArticulationData& data,
			Cm::SpatialVectorF* Z,
			const Cm::SpatialVectorF& impulse,
			PxReal* jointVelocities);

		void getImpulseSelfResponseInv(const bool fixBase, 
			PxU32 linkID0,
			PxU32 linkID1,
			Cm::SpatialVectorF* Z,
			const Cm::SpatialVector& impulse0,
			const Cm::SpatialVector& impulse1,
			Cm::SpatialVector& deltaV0,
			Cm::SpatialVector& deltaV1,
			PxReal* jointVelocities);

		void getImpulseResponseSlowInv(Dy::ArticulationLink* links,
			const ArticulationData& data,
			PxU32 linkID0_,
			const Cm::SpatialVector& impulse0,
			Cm::SpatialVector& deltaV0,
			PxU32 linkID1_,
			const Cm::SpatialVector& impulse1,
			Cm::SpatialVector& deltaV1,
			PxReal* jointVelocities,
			Cm::SpatialVectorF* Z);

		Cm::SpatialVectorF getImpulseResponseInv(const bool fixBase, 
			const PxU32 linkID, Cm::SpatialVectorF* Z, 
			const Cm::SpatialVector& impulse,
			PxReal* jointVelocites);

		void inverseDynamic(ArticulationData& data, const PxVec3& gravity,
			ScratchData& scratchData, bool computeCoriolis);

		void inverseDynamicFloatingBase(ArticulationData& data, const PxVec3& gravity,
			ScratchData& scratchData, bool computeCoriolis);

		//compute link body force with motion velocity and acceleration
		void computeZAForceInv(ArticulationData& data, ScratchData& scratchData);
		void initCompositeSpatialInertia(ArticulationData& data, Dy::SpatialMatrix* compositeSpatialInertia);
		void computeCompositeSpatialInertiaAndZAForceInv(ArticulationData& data, ScratchData& scratchData);

		void computeRelativeGeneralizedForceInv(ArticulationData& data, ScratchData& scratchData);

		//compute link acceleration due to external forces, applied external accelerations and Coriolis force
		void computeLinkAccelerationInv(ArticulationData& data, ScratchData& scratchData);

		void computeGeneralizedForceInv(ArticulationData& data, ScratchData& scratchData);

		void calculateMassMatrixColInv(ScratchData& scratchData);

		void calculateHFixBase(PxArticulationCache& cache);

		void calculateHFloatingBase(PxArticulationCache& cache);

		//joint limits
		static void enforcePrismaticLimits(PxReal& jPosition, ArticulationJointCore* joint);


		public:

			PX_FORCE_INLINE	void addBody()
			{
				mAcceleration.pushBack(Cm::SpatialVector(PxVec3(0.f), PxVec3(0.f)));
				mUpdateSolverData = true;
			}

			PX_FORCE_INLINE	void removeBody()
			{
				mUpdateSolverData = true;
			}

			PX_FORCE_INLINE	bool					updateSolverData()									{ return mUpdateSolverData;		}

			PX_FORCE_INLINE PxU32					getMaxDepth()								const	{ return mMaxDepth;				}
			PX_FORCE_INLINE void					setMaxDepth(const PxU32	depth)						{ mMaxDepth = depth;			}

			// solver methods
			PX_FORCE_INLINE PxU32					getBodyCount()								const	{ return mSolverDesc.linkCount;	}
			PX_FORCE_INLINE void					getSolverDesc(ArticulationSolverDesc& d)	const	{ d = mSolverDesc;				}
			PX_FORCE_INLINE ArticulationSolverDesc& getSolverDesc()										{ return mSolverDesc;			}

			PX_FORCE_INLINE ArticulationCore*		getCore()											{ return mSolverDesc.core;		}
			PX_FORCE_INLINE PxU16					getIterationCounts()						const	{ return mSolverDesc.core->solverIterationCounts;	}

			PX_FORCE_INLINE void*					getUserData()								const	{ return mUserData;				}

			PX_FORCE_INLINE void					setDyContext(Dy::Context* context)					{ mContext = context;			}

			void	setupLinks(PxU32 nbLinks, Dy::ArticulationLink* links);
			void	allocatePathToRootElements(const PxU32 totalPathToRootElements);
			void	initPathToRoot();
			
		static void getImpulseSelfResponse(ArticulationLink* links,
			ArticulationData& data,
			PxU32 linkID0,
			const Cm::SpatialVectorV& impulse0,
			Cm::SpatialVectorV& deltaV0,
			PxU32 linkID1,
			const Cm::SpatialVectorV& impulse1,
			Cm::SpatialVectorV& deltaV1);

		static void getImpulseResponseSlow(Dy::ArticulationLink* links,
			ArticulationData& data,
			PxU32 linkID0_,
			const Cm::SpatialVector& impulse0,
			Cm::SpatialVector& deltaV0,
			PxU32 linkID1_,
			const Cm::SpatialVector& impulse1,
			Cm::SpatialVector& deltaV1);

		PxU32 setupSolverConstraints(
			ArticulationLink* links,
			const PxU32 linkCount,
			const bool fixBase,
			ArticulationData& data,
			PxU32& acCount);

		void setupInternalConstraints(
			ArticulationLink* links,
			const PxU32 linkCount,
			const bool fixBase,
			ArticulationData& data,
			PxReal stepDt,
			PxReal dt,
			PxReal invDt,
			bool isTGSSolver);

		void setupInternalConstraintsRecursive(
			ArticulationLink* links,
			const PxU32 linkCount,
			const bool fixBase,
			ArticulationData& data,
			const PxReal stepDt,
			const PxReal dt,
			const PxReal invDt,
			const bool isTGSSolver,
			const PxU32 linkID,
			const PxReal maxForceScale);

		void setupInternalSpatialTendonConstraintsRecursive(
			ArticulationLink* links,
			ArticulationAttachment* attachments,
			const PxU32 attachmentCount,
			const PxVec3& parentAttachmentPoint,
			const bool fixBase,
			ArticulationData& data,
			const PxReal stepDt,
			const bool isTGSSolver,
			const PxU32 attachmentID,
			const PxReal stiffness,
			const PxReal damping,
			const PxReal limitStiffness,
			const PxReal err,
			const PxU32 startLink,
			const PxVec3& startAxis,
			const PxVec3& startRaXn);


		void setupInternalFixedTendonConstraintsRecursive(
			ArticulationLink* links,
			ArticulationTendonJoint* tendonJoints,
			const bool fixBase,
			ArticulationData& data,
			const PxReal stepDt,
			const bool isTGSSolver,
			const PxU32 tendonJointID,
			const PxReal stiffness,
			const PxReal damping,
			const PxReal limitStiffness,
			const PxU32 startLink,
			const PxVec3& startAxis,
			const PxVec3& startRaXn);


		void updateSpatialTendonConstraintsRecursive(ArticulationAttachment* attachments, ArticulationData& data, const PxU32 attachmentID, const PxReal accumErr,
			const PxVec3& parentAttachmentPoint);

		//void updateFixedTendonConstraintsRecursive(ArticulationLink* links, ArticulationTendonJoint* tendonJoint, ArticulationData& data, const PxU32 tendonJointID, const PxReal accumErr);

		PxVec3 calculateFixedTendonVelocityAndPositionRecursive(FixedTendonSolveData& solveData,
			const Cm::SpatialVectorF& parentV, const Cm::SpatialVectorF& parentDeltaV, const PxU32 tendonJointID);

		Cm::SpatialVectorF solveFixedTendonConstraintsRecursive(FixedTendonSolveData& solveData,
			const PxU32 tendonJointID);

		void prepareStaticConstraints(const PxReal dt, const PxReal invDt, PxsContactManagerOutputIterator& outputs,
			Dy::ThreadContext& threadContext, PxReal correlationDist, PxReal bounceThreshold, PxReal frictionOffsetThreshold,
			PxReal ccdMaxSeparation, PxSolverBodyData* solverBodyData, PxsConstraintBlockManager& blockManager,
			Dy::ConstraintWriteback* constraintWritebackPool);

		void prepareStaticConstraintsTGS(const PxReal stepDt, const PxReal totalDt, const PxReal invStepDt, const PxReal invTotalDt, 
			PxsContactManagerOutputIterator& outputs, Dy::ThreadContext& threadContext, PxReal correlationDist, PxReal bounceThreshold, 
			PxReal frictionOffsetThreshold, PxTGSSolverBodyData* solverBodyData, 
			PxTGSSolverBodyTxInertia* txInertia, PxsConstraintBlockManager& blockManager, Dy::ConstraintWriteback* constraintWritebackPool,
			const PxReal biasCoefficient, const PxReal lengthScale);


		//integration
		void propagateLinksDown(ArticulationData& data, const PxReal* jointVelocities, PxReal* jointPositions,
			Cm::SpatialVectorF* motionVelocities);

		void updateJointProperties(
			const PxReal* jointNewVelocities,
			PxReal* jointVelocities,
			PxReal* jointAccelerations);

		void computeAndEnforceJointPositions(ArticulationData& data);

		//update link position based on joint position provided by the cache
		void teleportLinks(ArticulationData& data);

		void computeLinkVelocities(ArticulationData& data);

		PxU8* allocateScratchSpatialData(PxcScratchAllocator* allocator,
			const PxU32 linkCount, ScratchData& scratchData, bool fallBackToHeap = false);

		//This method calculate the velocity change from parent to child using parent current motion velocity
		PxTransform propagateTransform(const PxU32 linkID, ArticulationLink* links, ArticulationJointCoreData& jointDatum,
			Cm::SpatialVectorF* motionVelocities, const PxReal dt, const PxTransform& pBody2World, const PxTransform& currentTransform,
			PxReal* jointVelocity, PxReal* jointPosition, const Cm::UnAlignedSpatialVector* motionMatrix,
			const Cm::UnAlignedSpatialVector* worldMotionMatrix);

		static void updateRootBody(const Cm::SpatialVectorF& motionVelocity,
			const PxTransform& preTransform, ArticulationData& data, const PxReal dt);

		//These variables are used in the constraint partition
		PxU16							maxSolverFrictionProgress;
		PxU16							maxSolverNormalProgress;
		PxU32							solverProgress;
		PxU16							mArticulationIndex;
		PxU8							numTotalConstraints;
		
		void*							mUserData;
		Dy::Context*					mContext;
		ArticulationSolverDesc			mSolverDesc;

		PxArray<Cm::SpatialVector>		mAcceleration;		// supplied by Sc-layer to feed into articulations

		bool							mUpdateSolverData;
		PxU32							mMaxDepth;

		ArticulationData				mArticulationData;

		PxArray<PxSolverConstraintDesc> mStaticContactConstraints;
		PxArray<PxSolverConstraintDesc> mStatic1DConstraints;
		PxU32							mGPUDirtyFlags;
		bool							mJcalcDirty;

		Dy::ErrorAccumulator			mInternalErrorAccumulatorVelIter;
		Dy::ErrorAccumulator			mContactErrorAccumulatorVelIter;
	
		Dy::ErrorAccumulator			mInternalErrorAccumulatorPosIter;
		Dy::ErrorAccumulator			mContactErrorAccumulatorPosIter;

} PX_ALIGN_SUFFIX(64);

#if PX_VC 
#pragma warning(pop) 
#endif

} //namespace Dy

}

#endif
