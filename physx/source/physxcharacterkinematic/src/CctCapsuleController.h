// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CCT_CAPSULE_CONTROLLER
#define CCT_CAPSULE_CONTROLLER

/* Exclude from documentation */
/** \cond */

#include "characterkinematic/PxCapsuleController.h"

#include "CctController.h"

namespace physx
{
class PxPhysics;

namespace Cct
{
	class CapsuleController : public PxCapsuleController, public Controller
	{
	public:
													CapsuleController(const PxControllerDesc& desc, PxPhysics& sdk, PxScene* scene);
		virtual										~CapsuleController();

		// Controller
		virtual	PxF32								getHalfHeightInternal()				const	PX_OVERRIDE	PX_FINAL		{ return mRadius+mHeight*0.5f;			}
		virtual	bool								getWorldBox(PxExtendedBounds3& box) const	PX_OVERRIDE	PX_FINAL;
		virtual	PxController*						getPxController()							PX_OVERRIDE	PX_FINAL		{ return this;							}
		//~Controller

		// PxController
		virtual	PxControllerShapeType::Enum			getType()							const	PX_OVERRIDE	PX_FINAL		{ return mType;							}
		virtual void								release()									PX_OVERRIDE	PX_FINAL		{ releaseInternal();					}
		virtual	PxControllerCollisionFlags			move(const PxVec3& disp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles)	PX_OVERRIDE	PX_FINAL;
		virtual	bool								setPosition(const PxExtendedVec3& position)	PX_OVERRIDE	PX_FINAL		{ return setPos(position);				}
		virtual	const PxExtendedVec3&				getPosition()						const	PX_OVERRIDE	PX_FINAL		{ return mPosition;						}
		virtual	bool								setFootPosition(const PxExtendedVec3& position)	PX_OVERRIDE	PX_FINAL;
		virtual	PxExtendedVec3						getFootPosition()					const	PX_OVERRIDE	PX_FINAL;
		virtual	PxRigidDynamic*						getActor()							const	PX_OVERRIDE	PX_FINAL		{ return mKineActor;					}
		virtual	void								setStepOffset(const float offset)			PX_OVERRIDE	PX_FINAL		{ if(offset>=0.0f)
																																mUserParams.mStepOffset = offset;	}
		virtual	PxF32								getStepOffset()						const	PX_OVERRIDE	PX_FINAL		{ return mUserParams.mStepOffset;		}
		virtual	void								setNonWalkableMode(PxControllerNonWalkableMode::Enum flag)	PX_OVERRIDE	PX_FINAL	{ mUserParams.mNonWalkableMode = flag;	}
		virtual	PxControllerNonWalkableMode::Enum	getNonWalkableMode()				const	PX_OVERRIDE	PX_FINAL		{ return mUserParams.mNonWalkableMode;	}
		virtual PxF32								getContactOffset()					const	PX_OVERRIDE	PX_FINAL		{ return mUserParams.mContactOffset;	}
		virtual	void								setContactOffset(PxF32 offset)				PX_OVERRIDE	PX_FINAL		{ if(offset>0.0f)
																																mUserParams.mContactOffset = offset;}
		virtual PxVec3								getUpDirection()					const	PX_OVERRIDE	PX_FINAL		{ return mUserParams.mUpDirection;		}
		virtual	void								setUpDirection(const PxVec3& up)			PX_OVERRIDE	PX_FINAL		{ setUpDirectionInternal(up);			}
		virtual PxF32								getSlopeLimit()						const	PX_OVERRIDE	PX_FINAL		{ return mUserParams.mSlopeLimit;		}
		virtual void								setSlopeLimit(PxF32 slopeLimit)				PX_OVERRIDE	PX_FINAL		{ if(slopeLimit>0.0f)
																																mUserParams.mSlopeLimit = slopeLimit;}
		virtual	void								invalidateCache()							PX_OVERRIDE	PX_FINAL;
		virtual	PxScene*							getScene()									PX_OVERRIDE	PX_FINAL		{ return mScene;						}
		virtual	void*								getUserData()						const	PX_OVERRIDE	PX_FINAL		{ return mUserData;						}
		virtual	void								setUserData(void* userData)					PX_OVERRIDE	PX_FINAL		{ mUserData = userData;					}
		virtual	void								getState(PxControllerState& state)	const	PX_OVERRIDE	PX_FINAL		{ return getInternalState(state);		}
		virtual	void								getStats(PxControllerStats& stats)	const	PX_OVERRIDE	PX_FINAL		{ return getInternalStats(stats);		}
		virtual	void								resize(PxReal height)						PX_OVERRIDE	PX_FINAL;
		//~PxController

		// PxCapsuleController
		virtual	PxF32								getRadius()							const	PX_OVERRIDE	PX_FINAL		{ return mRadius;						}
		virtual	PxF32								getHeight()							const	PX_OVERRIDE	PX_FINAL		{ return mHeight;						}
		virtual	PxCapsuleClimbingMode::Enum			getClimbingMode()					const	PX_OVERRIDE	PX_FINAL;
		virtual	bool								setRadius(PxF32 radius)						PX_OVERRIDE	PX_FINAL;
		virtual	bool								setHeight(PxF32 height)						PX_OVERRIDE	PX_FINAL;
		virtual	bool								setClimbingMode(PxCapsuleClimbingMode::Enum)	PX_OVERRIDE	PX_FINAL;
		//~ PxCapsuleController

				void								getCapsule(PxExtendedCapsule& capsule)	const;

				PxF32								mRadius;
				PxF32								mHeight;
				PxCapsuleClimbingMode::Enum			mClimbingMode;
	};

} // namespace Cct

}

/** \endcond */
#endif
