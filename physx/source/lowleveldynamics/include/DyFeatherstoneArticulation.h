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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
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
#include "CmSpatialVector.h"

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
		PxReal	accumulatedLength;					//4		82	//accumulate distance for spatial tendon, accumualate joint pose for fixed tendon
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
		//Initial error
		PxReal errorLow;						//4		4
		PxReal errorHigh;						//4		8
		PxReal lowImpulse;						//4		12		changed
		PxReal highImpulse;						//4		16		changed
	};


	struct ArticulationInternalConstraint : public ArticulationInternalConstraintBase
	{	
		//Joint spring drive info
		PxReal driveTargetVel;					//4		128
		PxReal driveInitialBias;				//4		132
		PxReal driveBiasCoefficient;			//4		132
		PxReal driveVelMultiplier;				//4		140
		PxReal driveImpulseMultiplier;			//4		148
		PxReal maxDriveForce;					//4		152
		PxReal driveForce;						//4		156

		PxReal maxFrictionForce;				//4		160
		PxReal frictionForce;					//4		164
		PxReal frictionForceCoefficient;		//4		168

		bool isLinearConstraint;				//1		169
		PxU8 padding[7];						//11	176
	};

	//linkID can be PxU32. However, each thread is going to read 16 bytes so we just keep ArticulationSensor 16 byte align.
	//if not, newArticulationsLaunch kernel will fail to read the sensor data correctly
	struct ArticulationSensor
	{
		PxTransform					mRelativePose;	//28 28
		PxU16						mLinkID;		//02 30
		PxU16						mFlags;			//02 32
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
			mPathToRootElements(NULL), mNumPathToRootElements(0), mLinksData(NULL), mJointData(NULL), mJointTranData(NULL),
			mSpatialTendons(NULL), mNumSpatialTendons(0), mNumTotalAttachments(0),
			mFixedTendons(NULL), mNumFixedTendons(0), mSensors(NULL), mSensorForces(NULL),
			mNbSensors(0), mDt(0.f), mDofs(0xffffffff),
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
		PX_FORCE_INLINE PxReal*						getJointConstraintForces()								{ return mJointConstraintForces.begin();	}
		PX_FORCE_INLINE const PxReal*				getJointConstraintForces() const						{ return mJointConstraintForces.begin();	}
		//PX_FORCE_INLINE PxReal*					getJointFrictionForces() { return mJointFrictionForce.begin(); }

		PX_FORCE_INLINE ArticulationInternalConstraint&			getInternalConstraint(const PxU32 dofId)		{ return mInternalConstraints[dofId];	}
		PX_FORCE_INLINE const ArticulationInternalConstraint&	getInternalConstraint(const PxU32 dofId) const	{ return mInternalConstraints[dofId];	}
		
		PX_FORCE_INLINE Cm::SpatialVectorF*			getMotionVelocities()									{ return mMotionVelocities.begin();			}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getMotionAccelerations()								{ return mMotionAccelerations.begin();		}
		PX_FORCE_INLINE const Cm::SpatialVectorF*	getMotionAccelerations() const							{ return mMotionAccelerations.begin(); }
		PX_FORCE_INLINE Cm::SpatialVectorF*			getCorioliseVectors()									{ return mCorioliseVectors.begin();			}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getSpatialZAVectors()									{ return mZAForces.begin();					}
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

		PX_FORCE_INLINE ArticulationSensor**		getSensors()									const	{ return mSensors;							}
		PX_FORCE_INLINE PxU32						getSensorCount()								const	{ return mNbSensors;						}


		PX_FORCE_INLINE ArticulationLinkData*		getLinkData()									const	{ return mLinksData;						}
						ArticulationLinkData&		getLinkData(PxU32 index)						const;

		PX_FORCE_INLINE ArticulationJointCoreData*	getJointData()									const	{ return mJointData;						}
		PX_FORCE_INLINE ArticulationJointCoreData&	getJointData(PxU32 index)						const	{ return mJointData[index];					}

		PX_FORCE_INLINE ArticulationJointTargetData*	getJointTranData()									const { return mJointTranData; }
		PX_FORCE_INLINE	ArticulationJointTargetData&	getJointTranData(PxU32 index)						const { return mJointTranData[index]; }
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

		PX_FORCE_INLINE PxTransform*				getAccumulatedPoses()									{ return mAccumulatedPoses.begin();			}
		PX_FORCE_INLINE const PxTransform*			getAccumulatedPoses()							const	{ return mAccumulatedPoses.begin();			}

		PX_FORCE_INLINE Cm::SpatialVectorF*			getJointSpaceJacobians()								{ return mJointSpaceJacobians.begin();		}
		PX_FORCE_INLINE const Cm::SpatialVectorF*	getJointSpaceJacobians()						const	{ return mJointSpaceJacobians.begin();		}
		
		PX_FORCE_INLINE JointSpaceSpatialZ*			getJointSpaceDeltaV()									{ return mJointSpaceDeltaVMatrix.begin();	}
		PX_FORCE_INLINE const JointSpaceSpatialZ*	getJointSpaceDeltaV()							const	{ return mJointSpaceDeltaVMatrix.begin();	}

		PX_FORCE_INLINE Cm::SpatialVectorF*			getJointSpaceResponse()									{ return mJointSpaceResponseMatrix.begin(); }
		PX_FORCE_INLINE const Cm::SpatialVectorF*	getJointSpaceResponse()							const	{ return mJointSpaceResponseMatrix.begin(); }

		PX_FORCE_INLINE SpatialImpulseResponseMatrix* getRootResponseMatrix()								{ return mRootResponseMatrix.begin();		}
		PX_FORCE_INLINE const SpatialImpulseResponseMatrix* getRootResponseMatrix()					const	{ return mRootResponseMatrix.begin();		}
		
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getRootDeferredZ()								const	{ return mRootDeferredZ;					}
		PX_FORCE_INLINE	Cm::SpatialVectorF&			getRootDeferredZ()										{ return mRootDeferredZ;					}

		

		PX_FORCE_INLINE SpatialImpulseResponseMatrix* getImpulseResponseMatrixWorld() { return mResponseMatrixW.begin(); }

		PX_FORCE_INLINE const SpatialImpulseResponseMatrix* getImpulseResponseMatrixWorld() const { return mResponseMatrixW.begin(); }

		PX_FORCE_INLINE const SpatialMatrix& getWorldSpatialArticulatedInertia(const PxU32 linkID) const { return mWorldSpatialArticulatedInertia[linkID]; }
		

		PX_FORCE_INLINE const InvStIs& getInvStIs(const PxU32 linkID) const { return mInvStIs[linkID]; }

		PX_FORCE_INLINE const Cm::UnAlignedSpatialVector& getMotionMatrix(const PxU32 dofId) const { return mMotionMatrix[dofId]; }
		PX_FORCE_INLINE const Cm::UnAlignedSpatialVector& getWorldMotionMatrix(const PxU32 dofId) const { return mWorldMotionMatrix[dofId]; }

		PX_FORCE_INLINE		  Cm::UnAlignedSpatialVector& getJointAxis(const PxU32 dofId)			{ return mJointAxis[dofId]; }
		PX_FORCE_INLINE const Cm::UnAlignedSpatialVector& getJointAxis(const PxU32 dofId) const		{ return mJointAxis[dofId]; }

		PX_FORCE_INLINE const PxVec3& getRw(const PxU32 linkID) const								{ return mRw[linkID]; }

		PX_FORCE_INLINE const Cm::SpatialVectorF& getIsW(const PxU32 dofId) const { return mIsW[dofId]; }

		PX_FORCE_INLINE const Cm::SpatialVectorF& getWorldIsInvD(const PxU32 dofId) const { return mIsInvDW[dofId]; }
		PX_FORCE_INLINE PxReal* getDeferredQstZ() { return mDeferredQstZ.begin(); }

		PX_FORCE_INLINE PxReal* getQstZic() { return qstZIc.begin(); }

		PX_FORCE_INLINE Cm::SpatialVectorF& getSolverSpatialForce(const PxU32 linkID) { return mSolverSpatialForces[linkID]; }
		PX_FORCE_INLINE PxSpatialForce* getSensorForces() { return mSensorForces; }
		PX_FORCE_INLINE void setRootPreMotionVelocity(const Cm::UnAlignedSpatialVector& vel) { mRootPreMotionVelocity.top = vel.top; mRootPreMotionVelocity.bottom = vel.bottom; }

		PX_FORCE_INLINE PxU32*	getPathToRootElements() const { return mPathToRootElements; }
		PX_FORCE_INLINE PxU32	getPathToRootElementCount() const { return mNumPathToRootElements; }

	private:
		Cm::SpatialVectorF							mRootPreMotionVelocity;
		Cm::SpatialVectorF							mRootDeferredZ;
		PxArray<PxReal>								mJointAcceleration;		//	joint acceleration
		PxArray<PxReal>								mJointInternalAcceleration;	//joint internal force acceleration
		PxArray<PxReal>								mJointVelocity;			//	joint velocity
		PxArray<PxReal>								mJointNewVelocity;		//	joint velocity due to contacts
		PxArray<PxReal>								mJointPosition;			//	joint position
		PxArray<PxReal>								mJointForce;			//	joint force
		//Ps::Array<PxReal>							mJointFrictionForce;	//	joint friction force
	
		PxArray<PxReal>											mPosIterJointVelocities;	//joint delta velocity after postion iternation before velocity iteration
		PxArray<Cm::SpatialVectorF>								mPosIterMotionVelocities;	//link motion velocites after position iteration before velocity iteration
		PxArray<Cm::SpatialVectorF>								mMotionVelocities;		//link motion velocites
		PxArray<Cm::SpatialVectorF>								mSolverSpatialForces;
		PxArray<Cm::SpatialVectorF>								mMotionAccelerations;	//link motion accelerations
		PxArray<Cm::SpatialVectorF>								mMotionAccelerationsInternal;	//link motion accelerations
		PxArray<Cm::SpatialVectorF>								mCorioliseVectors;		//link coriolise vector
		PxArray<Cm::SpatialVectorF>								mZAInternalForces;		//link internal spatial forces
		PxArray<Cm::SpatialVectorF>								mZAForces;				//link spatial zero acceleration force/ spatial articulated force
		PxArray<Cm::SpatialVectorF>								mJointTransmittedForce; 
		PxArray<ArticulationInternalConstraint>					mInternalConstraints;
		PxArray<ArticulationInternalLimit>						mInternalLimits;
		PxArray<ArticulationInternalTendonConstraint>			mInternalSpatialTendonConstraints;
		PxArray<ArticulationInternalTendonConstraint>			mInternalFixedTendonConstraints;
		

		PxArray<PxReal>											mDeferredQstZ;

		PxArray<PxReal>											mJointConstraintForces;

		PxArray<Cm::SpatialVectorF>				mDeltaMotionVector; //this is for TGS solver
		PxArray<PxTransform>					mPreTransform; //this is the previous transform list for links
		PxArray<SpatialImpulseResponseMatrix>	mResponseMatrixW;
		PxArray<Cm::SpatialVectorF>				mJointSpaceJacobians;
		PxArray<JointSpaceSpatialZ>				mJointSpaceDeltaVMatrix;
		PxArray<Cm::SpatialVectorF>				mJointSpaceResponseMatrix;
		PxArray<Cm::SpatialVectorF>				mPropagationAccelerator;
		PxArray<SpatialImpulseResponseMatrix>	mRootResponseMatrix;
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
		PxArray<Cm::SpatialVectorF>				mIsInvDW;
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
		ArticulationJointTargetData*			mJointTranData;
		ArticulationSpatialTendon**				mSpatialTendons;
		PxU32									mNumSpatialTendons;
		PxU32									mNumTotalAttachments;
		ArticulationFixedTendon**				mFixedTendons;
		PxU32									mNumFixedTendons;
		ArticulationSensor**					mSensors;
		PxSpatialForce*							mSensorForces;
		PxU32									mNbSensors;
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
		Cm::SpatialVectorF* impulses;
		Cm::SpatialVectorF* deltaV;
		const bool velocityIteration;
		const bool isTGS;
		PxU32 dofId;
		PxU32 complexId;
		PxU32 limitId;
		PxU32 articId;

		InternalConstraintSolverData(const PxReal dt_, const PxReal invDt_, const PxReal elapsedTime_,
			const PxReal erp_, Cm::SpatialVectorF* impulses_, Cm::SpatialVectorF* deltaV_,
			bool velocityIteration_, bool isTGS_) : dt(dt_), invDt(invDt_), elapsedTime(elapsedTime_),
			erp(erp_), impulses(impulses_), deltaV(deltaV_), velocityIteration(velocityIteration_),
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
			eDIRTY_ACCELERATIONS = 1 << 3,
			eDIRTY_FORCES = 1 << 4,
			eDIRTY_ROOT_TRANSFORM = 1 << 5,
			eDIRTY_ROOT_VELOCITIES = 1 << 6,
			eDIRTY_LINKS = 1 << 7,
			eIN_DIRTY_LIST = 1 << 8,
			eDIRTY_WAKECOUNTER = 1 << 9,
			eDIRTY_EXT_ACCEL = 1 << 10,
			eDIRTY_LINK_FORCE = 1 << 11,
			eDIRTY_LINK_TORQUE = 1 << 12,
			eDIRTY_JOINT_TARGET_VEL = 1 << 13,
			eDIRTY_JOINT_TARGET_POS = 1 << 14,
			ePENDING_INSERTION = 1 << 15,
			eDIRTY_SPATIAL_TENDON = 1 << 16,
			eDIRTY_SPATIAL_TENDON_ATTACHMENT = 1 << 17,
			eDIRTY_FIXED_TENDON = 1 << 18,
			eDIRTY_FIXED_TENDON_JOINT = 1 << 19,
			eDIRTY_SENSOR = 1 << 20,
			eDIRTY_VELOCITY_LIMITS = 1 << 21,
			eDIRTY_DOFS = (eDIRTY_POSITIONS | eDIRTY_VELOCITIES | eDIRTY_ACCELERATIONS | eDIRTY_FORCES),
			eALL = (1<<21)-1 
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

		void		assignSensors(const PxU32 nbSensors, Dy::ArticulationSensor** sensors, PxSpatialForce* sensorForces);

		PxU32		getDofs();

		PxU32		getDof(const PxU32 linkID);

		bool		applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, bool& shouldWake);

		void		copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag);

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
			Cm::SpatialVectorF* Z,
			const Cm::SpatialVector& impulse,
			Cm::SpatialVector& deltaV) const;

		void	getImpulseResponse(
			PxU32 linkID,
			Cm::SpatialVectorV* /*Z*/,
			const Cm::SpatialVectorV& impulse,
			Cm::SpatialVectorV& deltaV) const;

		void		getImpulseSelfResponse(
			PxU32 linkID0,
			PxU32 linkID1,
			Cm::SpatialVectorF* Z,
			const Cm::SpatialVector& impulse0,
			const Cm::SpatialVector& impulse1,
			Cm::SpatialVector& deltaV0,
			Cm::SpatialVector& deltaV1) const;

		Cm::SpatialVectorV getLinkVelocity(const PxU32 linkID) const;

		Cm::SpatialVector getLinkScalarVelocity(const PxU32 linkID) const;

		Cm::SpatialVectorV getLinkMotionVector(const PxU32 linkID) const;

		//this is called by island gen to determine whether the articulation should be awake or sleep
		Cm::SpatialVector getMotionVelocity(const PxU32 linkID) const;

		Cm::SpatialVector getMotionAcceleration(const PxU32 linkID) const;

		void fillIndexType(const PxU32 linkId, PxU8& indexType);

		PxReal getLinkMaxPenBias(const PxU32 linkID) const;

		PxReal getCfm(const PxU32 linkID) const;

		static PxU32 computeUnconstrainedVelocities(
			const ArticulationSolverDesc& desc,
			PxReal dt,
			PxU32& acCount,
			const PxVec3& gravity, 
			Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV, const PxReal invLengthScale);

		static void computeUnconstrainedVelocitiesTGS(
			const ArticulationSolverDesc& desc,
			PxReal dt, const PxVec3& gravity,
			PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV,
			const PxReal invLengthScale);

		static PxU32 setupSolverConstraintsTGS(const ArticulationSolverDesc& articDesc,
			PxReal dt,
			PxReal invDt,
			PxReal totalDt,
			const PxReal biasCoefficient,
			PxU32& acCount,
			Cm::SpatialVectorF* Z);

		static void saveVelocity(const ArticulationSolverDesc& d, Cm::SpatialVectorF* deltaV);

		static void saveVelocityTGS(const ArticulationSolverDesc& d, PxReal invDtF32);

		static void updateBodies(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV, PxReal dt);

		static void updateBodiesTGS(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV, PxReal dt);

		static void updateBodies(FeatherstoneArticulation* articulation, Cm::SpatialVectorF* tempDeltaV, PxReal dt, bool integrateJointPosition);

		static void recordDeltaMotion(const ArticulationSolverDesc& desc, const PxReal dt, Cm::SpatialVectorF* deltaV, const PxReal totalInvDt);

		static void deltaMotionToMotionVelocity(const ArticulationSolverDesc& desc, PxReal invDt);

		void pxcFsApplyImpulse(PxU32 linkID, aos::Vec3V linear,
			aos::Vec3V angular, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

		void pxcFsApplyImpulses(PxU32 linkID, const aos::Vec3V& linear,
			const aos::Vec3V& angular, PxU32 linkID2, const aos::Vec3V& linear2,
			const aos::Vec3V& angular2, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

		void pxcFsApplyImpulses(Cm::SpatialVectorF* Z);

		Cm::SpatialVectorV pxcFsGetVelocity(PxU32 linkID);

		void pxcFsGetVelocities(PxU32 linkID, PxU32 linkID1, Cm::SpatialVectorV& v0, Cm::SpatialVectorV& v1);

		Cm::SpatialVectorV pxcFsGetVelocityTGS(PxU32 linkID);

		const PxTransform& getCurrentTransform(PxU32 linkID) const;

		const PxQuat& getDeltaQ(PxU32 linkID) const;

		//Applies a set of N impulses, all in local space and updates the links' motion and joint velocities
		void applyImpulses(Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);
		void getDeltaV(Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

		//This method calculate the velocity change due to collision/constraint impulse, record joint velocity and acceleration
		static Cm::SpatialVectorF propagateVelocityW(const PxVec3& c2p, const Dy::SpatialMatrix& spatialInertia,
			const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::SpatialVectorF& Z,
			PxReal* jointVelocity, const Cm::SpatialVectorF& hDeltaV, const PxU32 dofCount);

		static Cm::SpatialVectorF propagateAccelerationW(const PxVec3& c2p,
			const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrix,
			PxReal* jointVelocity, const Cm::SpatialVectorF& pAcceleration, const PxU32 dofCount, const Cm::SpatialVectorF* IsW, PxReal* qstZIc);

		static void propagateAccelerationW(const PxVec3& c2p,
			const InvStIs& invStIs,	PxReal* jointVelocity, const Cm::SpatialVectorF& pAcceleration, 
			const PxU32 dofCount, const Cm::SpatialVectorF* IsW);

		static Cm::SpatialVectorF propagateAccelerationW(const PxVec3& c2p,
			const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrix,
			const Cm::SpatialVectorF& pAcceleration, const PxU32 dofCount, const Cm::SpatialVectorF* IsW, PxReal* qstZIc);
		
		static Cm::SpatialVectorF propagateAccelerationW(const PxVec3& c2p,
			const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrix,
			PxReal* jointVelocity, const Cm::SpatialVectorF& pAcceleration, Cm::SpatialVectorF& Z, const PxU32 dofCount, const Cm::SpatialVectorF* IsW);

		//This method calculate the velocity change due to collision/constraint impulse
		static Cm::SpatialVectorF propagateVelocityTestImpulseW(const PxVec3& c2p, const Dy::SpatialMatrix& spatialInertia, const InvStIs& invStIs,
			const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::SpatialVectorF& Z, const Cm::SpatialVectorF& hDeltaV,
			const PxU32 dofCount);


		////This method calculate zero acceration impulse due to test/actual impluse
		//static Cm::SpatialVectorF propagateImpulse(const IsInvD& isInvD, const SpatialTransform& childToParent, 
		//	const SpatialSubspaceMatrix& motionMatrix, const Cm::SpatialVectorF& Z);

		static Cm::SpatialVectorF propagateImpulseW(const Cm::SpatialVectorF* isInvD, const PxVec3& childToParent,
			const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::SpatialVectorF& Z, const PxU32 dofCount);

		static Cm::SpatialVectorF propagateImpulseW(const Cm::SpatialVectorF* isInvD, const PxVec3& childToParent,
			const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::SpatialVectorF& Z, const PxU32 dofCount, PxReal* qstZ);

		bool applyCacheToDest(ArticulationData& data, PxArticulationCache& cache,
			PxReal* jVelocities, PxReal* jAcceleration, PxReal* jPosition, PxReal* jointForce,
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
		void constraintPrep(ArticulationLoopConstraint* lConstraints, const PxU32 nbJoints,
			Cm::SpatialVectorF* Z, PxSolverConstraintPrepDesc& prepDesc, PxSolverBody& sBody,
			PxSolverBodyData& sBodyData, PxSolverConstraintDesc* desc, PxConstraintAllocator& allocator);

		void updateArticulation(ScratchData& scratchData,
			const PxVec3& gravity,
			Cm::SpatialVectorF* Z,
			Cm::SpatialVectorF* DeltaV,
			const PxReal invLengthScale);

		void computeUnconstrainedVelocitiesInternal(
			const PxVec3& gravity,
			Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV, const PxReal invLengthScale);

		//copy joint data from fromJointData to toJointData
		void copyJointData(ArticulationData& data, PxReal* toJointData, const PxReal* fromJointData);

		PxU32 computeDofs();
		//this function calculates motion subspace matrix(s) for all tree joint
		void jcalc(ArticulationData& data, bool forceUpdate = false);

		//this function calculates loop joint constraint subspace matrix(s) and active force
		//subspace matrix
		void jcalcLoopJointSubspace(ArticulationJointCore* joint, ArticulationJointCoreData& jointDatum, SpatialSubspaceMatrix& T,
			const Cm::UnAlignedSpatialVector* jointAxis);

		void computeSpatialInertia(ArticulationData& data);

		//compute zero acceleration force
		void computeZ(const ArticulationData& data, const PxVec3& gravity, ScratchData& scratchData);
		void computeZD(const ArticulationData& data, const PxVec3& gravity, ScratchData& scratchData);

		void solveInternalConstraints(const PxReal dt, const PxReal invDt, Cm::SpatialVectorF* impulses, Cm::SpatialVectorF* DeltaV,
			bool velocityIteration, bool isTGS, const PxReal elapsedTime, const PxReal biasCoefficient);


		void solveInternalJointConstraints(const PxReal dt, const PxReal invDt, Cm::SpatialVectorF* impulses, Cm::SpatialVectorF* DeltaV,
			bool velocityIteration, bool isTGS, const PxReal elapsedTime, const PxReal biasCoefficient);

		Cm::SpatialVectorF solveInternalJointConstraintRecursive(InternalConstraintSolverData& data, const PxU32 linkID,
			const Cm::SpatialVectorF& parentDeltaV);

		void solveInternalSpatialTendonConstraints(bool isTGS);

		void solveInternalFixedTendonConstraints(bool isTGS);

		void writebackInternalConstraints(bool isTGS);

		void concludeInternalConstraints(bool isTGS);

		//compute coriolis force
		void computeC(ArticulationData& data, ScratchData& scratchData);

		//compute relative transform child to parent
		void computeRelativeTransformC2P(ArticulationData& data);
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
		\param[in] accumulatedPoses is the pose of each link, specified in the world frame.
		\param[in] externalAccels is the external acceleration to apply to each link, specified in the world frame.
		\param[in] rws is the vector from each parent link to each child link, specified in the world frame.
		\param[in] worldMotionMatrices is the motion matrix of each link, specified in the world frame.
		\param[in] jointCoreData is the ArticulationJointCoreData instance of each link in the articulation.
		\param[in,out] linkData is the ArticulationLinkData instance of each link in the articulation.
		\param[in,out] links is the ArticulationLink instance of each link in the articulation.
		\param[in,out] motionAccelerations is the acceleration of each link, specified in the world frame.
		\param[out] motionVelocities is velocity of each link computed from the parent link velocity and joint velocity of the inbound joint. Specified in the world frame.
		\param[out] spatialZAForces is the computed spatial zero acceleration force of each link, accounting for only external forces applied to the links.  Specified in the world frame.
		\param[out] spatialZAInternal is the computed spatial zero acceleration force of each link, accounting for only internal forces applied to the links.  Specified in the world frame.
		\param[out] coriolisVectors is the computed coriolis vector of each link.   Specified in the world frame.
		\param[out] worldIsolatedSpatialArticulatedInertias is the inertia tensor (I) for the trivial sub-chain of each link. Specified in the world frame.
		\param[out] linkMasses is the mass of each link. 
		\param[out] worldSpatialArticulatedInertias is the spatial matrix containing the inertia tensor I and the mass matrix M for the trivial sub-chain of each link.  Specified in the world frame.
		\param[out] jointDofCount is the number of degrees of freedom for the entire articulation.
		\param[in,out] jointVelocities is the velocity of each degree of freedom.
		\param[out] rootPreMotionVelocity is assigned the spatial velocity of the root link.		
		\param[out] com is the centre of mass of the assembly of links, specified in the world frame.
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
		\note If the velocity of any degree of freedom exceeds the maximum velocity of the associated joint, the velocity of each degree of freedom will be scaled so that none exceeds the maximum.
		*/
		static void computeLinkStates(
			const PxF32 dt, const PxReal invLengthScale, const PxVec3& gravity,
			const bool fixBase,
			const PxU32 linkCount,
			const PxTransform* accumulatedPoses, const Cm::SpatialVector* externalAccels,  const PxVec3* rws, const Cm::UnAlignedSpatialVector* worldMotionMatrices,
			const  Dy::ArticulationJointCoreData* jointCoreData,
			Dy::ArticulationLinkData *linkData, Dy::ArticulationLink* links, Cm::SpatialVectorF* motionAccelerations, 
			Cm::SpatialVectorF* motionVelocities, Cm::SpatialVectorF* spatialZAForces, Cm::SpatialVectorF* spatialZAInternal, Cm::SpatialVectorF* coriolisVectors, 
			PxMat33* worldIsolatedSpatialArticulatedInertias, PxF32* linkMasses, Dy::SpatialMatrix* worldSpatialArticulatedInertias, 
			const PxU32 jointDofCount,
			PxReal* jointVelocities,
			Cm::SpatialVectorF& rootPreMotionVelocity, PxVec3& com, PxF32& invSumMass);

		void initLinks(ArticulationData& data, const PxVec3& gravity,
			ScratchData& scratchData, Cm::SpatialVectorF* tZ, Cm::SpatialVectorF* tDeltaV);

		void computeIs(ArticulationJointCoreData& jointDatum, ArticulationJointTargetData& jointTarget, const PxU32 linkID);
		static SpatialMatrix computePropagateSpatialInertia_ZA_ZIc(const PxU8 jointType, const ArticulationJointTargetData& jointTarget, const ArticulationJointCoreData& jointDatum,
			const SpatialMatrix& articulatedInertia, const Cm::SpatialVectorF* linkIs, InvStIs& invStIs, Cm::SpatialVectorF* isInvD, const Cm::UnAlignedSpatialVector* motionMatrix,
			const PxReal* jF, const Cm::SpatialVectorF& Z, const Cm::SpatialVectorF& ZIntIc, Cm::SpatialVectorF& ZA, Cm::SpatialVectorF& ZInt, PxReal* qstZ,
			PxReal* qstZIntIc);

		static SpatialMatrix computePropagateSpatialInertia_ZA_ZIc_NonSeparated(const PxU8 jointType, const ArticulationJointTargetData& jointTarget, const ArticulationJointCoreData& jointDatum,
			const SpatialMatrix& articulatedInertia, const Cm::SpatialVectorF* linkIs, InvStIs& invStIs, Cm::SpatialVectorF* isInvD, const Cm::UnAlignedSpatialVector* motionMatrix,
			const PxReal* jF, const Cm::SpatialVectorF& Z, Cm::SpatialVectorF& ZA, PxReal* qstZIc);

		static SpatialMatrix computePropagateSpatialInertia(const PxU8 jointType, ArticulationJointCoreData& jointDatum,
			const SpatialMatrix& articulatedInertia, const Cm::SpatialVectorF* linkIs, InvStIs& invStIs, Cm::SpatialVectorF* isInvD,
			const Cm::UnAlignedSpatialVector* motionMatrix);

		static void transformInertia(const SpatialTransform& sTod, SpatialMatrix& inertia);

		static void translateInertia(const PxMat33& offset, SpatialMatrix& inertia);

		static PxMat33 translateInertia(const PxMat33& inertia, const PxReal mass, const PxVec3& t);

		void computeArticulatedSpatialInertiaAndZ(ArticulationData& data, ScratchData& scratchData);
		void computeArticulatedSpatialInertiaAndZ_NonSeparated(ArticulationData& data, ScratchData& scratchData);

		void computeArticulatedSpatialInertia(ArticulationData& data);

		void computeArticulatedResponseMatrix(ArticulationData& data);

		void computeJointSpaceJacobians(ArticulationData& data);

		void computeArticulatedSpatialZ(ArticulationData& data, ScratchData& scratchData);

		/*void computeJointAcceleration(ArticulationLinkData& linkDatum, ArticulationJointCoreData& jointDatum, 
			const Cm::SpatialVectorF& pMotionAcceleration, PxReal* jointAcceleration, const PxU32 linkID);*/

		void computeJointAccelerationW(ArticulationJointCoreData& jointDatum,
			const Cm::SpatialVectorF& pMotionAcceleration, PxReal* jointAcceleration, const Cm::SpatialVectorF* IsW, const PxU32 linkID,
			const PxReal* qstZIc);

		//compute joint acceleration, joint velocity and link acceleration, velocity based
		//on spatial force and spatial articulated inertia tensor
		void computeLinkAcceleration(ArticulationData& data, ScratchData& scratchData, bool doIC);

		void computeLinkInternalAcceleration(ArticulationData& data, ScratchData& scratchData);

		//void computeTempLinkAcceleration(ArticulationData& data, ScratchData& scratchData);
		void computeJointTransmittedFrictionForce(ArticulationData& data, ScratchData& scratchData,
			Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV);

		static Cm::SpatialVectorF getDeltaVWithDeltaJV(const bool fixBase, const PxU32 linkID,
			const ArticulationData& data, Cm::SpatialVectorF* Z,
			PxReal* jointVelocities);

		static Cm::SpatialVectorF getDeltaV(const bool fixBase, const PxU32 linkID,
			const ArticulationData& data, Cm::SpatialVectorF* Z);

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

		//provided joint velocity and joint acceleartion, compute link acceleration
		void computeLinkAccelerationInv(ArticulationData& data, ScratchData& scratchData);

		void computeGeneralizedForceInv(ArticulationData& data, ScratchData& scratchData);

		void calculateMassMatrixColInv(ScratchData& scratchData);

		void calculateHFixBase(PxArticulationCache& cache);

		void calculateHFloatingBase(PxArticulationCache& cache);

		//joint limits
		void enforcePrismaticLimits(PxReal& jPosition, ArticulationJointCore* joint);


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
			Cm::SpatialVectorF* Z,
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
			Cm::SpatialVector& deltaV1,
			Cm::SpatialVectorF* Z);

		PxU32 setupSolverConstraints(
			ArticulationLink* links,
			const PxU32 linkCount,
			const bool fixBase,
			ArticulationData& data,
			Cm::SpatialVectorF* Z,
			PxU32& acCount);

		void setupInternalConstraints(
			ArticulationLink* links,
			const PxU32 linkCount,
			const bool fixBase,
			ArticulationData& data,
			Cm::SpatialVectorF* Z,
			PxReal stepDt,
			PxReal dt,
			PxReal invDt,
			PxReal erp,
			bool isTGSSolver);

		void setupInternalConstraintsRecursive(
			ArticulationLink* links,
			const PxU32 linkCount,
			const bool fixBase,
			ArticulationData& data,
			Cm::SpatialVectorF* Z,
			const PxReal stepDt,
			const PxReal dt,
			const PxReal invDt,
			const PxReal erp,
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
			Cm::SpatialVectorF* Z,
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
			Cm::SpatialVectorF* Z,
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
		void propagateLinksDown(ArticulationData& data, PxReal* jointVelocities, PxReal* jointPositions,
			Cm::SpatialVectorF* motionVelocities);

		void updateJointProperties(
			PxReal* jointNewVelocities,
			PxReal* jointVelocities,
			PxReal* jointAccelerations);

		void recomputeAccelerations(const PxReal dt); 
		Cm::SpatialVector recomputeAcceleration(const PxU32 linkID, const PxReal dt) const;

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
							

	} PX_ALIGN_SUFFIX(64);

#if PX_VC 
#pragma warning(pop) 
#endif

	void PxvRegisterArticulationsReducedCoordinate();


} //namespace Dy

}

#endif
