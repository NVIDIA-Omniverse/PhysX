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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_D6_JOINT_LIMIT_H
#define PXG_D6_JOINT_LIMIT_H

#include "common/PxTolerancesScale.h"
#include "PxgD6Joint.h"

namespace physx
{
	// PT: why aren't these shared with the CPU ?

	class PxgJointLimitParameters
	{
	public:
		
		PxReal restitution;
		PxReal bounceThreshold;
		PxReal stiffness;
		PxReal damping;

		PX_CUDA_CALLABLE PxgJointLimitParameters()
		: restitution(0)
		, bounceThreshold(0)
		, stiffness(0)
		, damping(0)
		{
		}	

		/**
		\brief Returns true if the current settings are valid.

		\return true if the current settings are valid
		*/
		PX_INLINE bool isValid() const
		{
			return	PxIsFinite(restitution) && restitution >= 0 && restitution <= 1 && 
					PxIsFinite(stiffness) && stiffness >= 0 && 
					PxIsFinite(damping) && damping >= 0 &&
					PxIsFinite(bounceThreshold) && bounceThreshold >= 0;
		}

		PX_CUDA_CALLABLE PX_INLINE bool isSoft() const
		{
			return damping>0 || stiffness>0;
		}
	};


	class PxgJointLinearLimit : public PxgJointLimitParameters
	{

	public:
		
		PxReal value;

		PX_CUDA_CALLABLE PxgJointLinearLimit(PxReal extent) : value(extent)
		{
		}

		PX_CUDA_CALLABLE PxgJointLinearLimit(PxReal extent, const PxgSpring& spring) : value(extent)
		{
			stiffness = spring.stiffness;
			damping = spring.damping;
		}

		PX_CUDA_CALLABLE PxgJointLinearLimit() {}

		PX_INLINE bool isValid() const
		{
			return PxgJointLimitParameters::isValid() &&
				   PxIsFinite(value) && 
				   value > 0;
		}
	};

	class PxgJointLinearLimitPair : public PxgJointLimitParameters
	{
	public:

		PX_CUDA_CALLABLE PxgJointLinearLimitPair() {}
		/**
		\brief the range of the limit. The upper limit must be no lower than the
		lower limit, and if they are equal the limited degree of freedom will be treated as locked.

		<b>Range:</b> See the joint on which the limit is used for details<br>
		<b>Default:</b> lower = -PX_MAX_F32/3, upper = PX_MAX_F32/3
		*/
		PxReal upper, lower;

		/**
		\brief Construct a linear hard limit pair. The lower distance value must be less than the upper distance value.

		\param[in] scale		A PxTolerancesScale struct. Should be the same as used when creating the PxPhysics object.
		\param[in] lowerLimit	The lower distance of the limit
		\param[in] upperLimit	The upper distance of the limit

		\see PxJointLimitParameters PxTolerancesScale
		*/
		PX_CUDA_CALLABLE PxgJointLinearLimitPair(const PxTolerancesScale& scale, PxReal lowerLimit, PxReal upperLimit) :
			upper(upperLimit),
			lower(lowerLimit)
		{
			bounceThreshold = 2.0f*scale.length;
		}

		/**
		\brief construct a linear soft limit pair

		\param[in] lowerLimit	The lower distance of the limit
		\param[in] upperLimit	The upper distance of the limit
		\param[in] spring		The stiffness and damping parameters of the limit spring

		\see PxJointLimitParameters PxTolerancesScale
		*/
		PX_CUDA_CALLABLE PxgJointLinearLimitPair(PxReal lowerLimit, PxReal upperLimit, const PxgSpring& spring) :
			upper(upperLimit),
			lower(lowerLimit)
		{
			stiffness = spring.stiffness;
			damping = spring.damping;
		}

		/**
		\brief Returns true if the limit is valid.

		\return true if the current settings are valid
		*/
		PX_INLINE bool isValid() const
		{
			return PxgJointLimitParameters::isValid() &&
				PxIsFinite(upper) && PxIsFinite(lower) && upper >= lower &&
				PxIsFinite(upper - lower);
		}
	};


	class PxgJointAngularLimitPair : public PxgJointLimitParameters
	{
	
	public:
		
		PxReal upper, lower;

		PX_CUDA_CALLABLE PxgJointAngularLimitPair(PxReal lowerLimit, PxReal upperLimit)
		: upper(upperLimit)
		, lower(lowerLimit)
		{
			bounceThreshold = 0.5f;
		}


		PX_CUDA_CALLABLE PxgJointAngularLimitPair(PxReal lowerLimit, PxReal upperLimit, const PxgSpring& spring)
		: upper(upperLimit)
		, lower(lowerLimit)
		{
			stiffness = spring.stiffness;
			damping = spring.damping;
		}

		PX_CUDA_CALLABLE PxgJointAngularLimitPair(){}


		/**
		\brief Returns true if the limit is valid.

		\return true if the current settings are valid
		*/
		PX_INLINE bool isValid() const
		{
			return PxgJointLimitParameters::isValid() &&
				   PxIsFinite(upper) && PxIsFinite(lower) && upper >= lower;
		}
	};

	class PxgJointLimitCone : public PxgJointLimitParameters
	{
	
	public:
		
		PxReal yAngle;
		PxReal zAngle;

		PX_CUDA_CALLABLE PxgJointLimitCone(PxReal yLimitAngle, PxReal zLimitAngle):
		  yAngle(yLimitAngle),
		  zAngle(zLimitAngle)
		  {
				bounceThreshold = 0.5f;
		  }

		PX_CUDA_CALLABLE PxgJointLimitCone(PxReal yLimitAngle, PxReal zLimitAngle, const PxgSpring& spring):
		  yAngle(yLimitAngle),
		  zAngle(zLimitAngle)
		  {
			  stiffness = spring.stiffness;
			  damping = spring.damping;
		  }

		PX_CUDA_CALLABLE PxgJointLimitCone(){}

		/**
		\brief Returns true if the limit is valid.

		\return true if the current settings are valid
		*/
		PX_INLINE bool isValid() const
		{
			return PxgJointLimitParameters::isValid() &&
				   PxIsFinite(yAngle) && yAngle>0 && yAngle<PxPi && 
				   PxIsFinite(zAngle) && zAngle>0 && zAngle<PxPi;
		}
	};

	class PxgJointLimitPyramid : public PxgJointLimitParameters
	{
	public:
		/**
		\brief the minimum angle from the Y axis of the constraint frame.

		<b>Unit:</b> Angular: Radians
		<b>Range:</b> Angular: (-PI,PI)<br>
		<b>Default:</b> -PI/2
		*/
		PxReal yAngleMin;

		/**
		\brief the maximum angle from the Y axis of the constraint frame.

		<b>Unit:</b> Angular: Radians
		<b>Range:</b> Angular: (-PI,PI)<br>
		<b>Default:</b> PI/2
		*/
		PxReal yAngleMax;

		/**
		\brief the minimum angle from the Z-axis of the constraint frame.

		<b>Unit:</b> Angular: Radians
		<b>Range:</b> Angular: (-PI,PI)<br>
		<b>Default:</b> -PI/2
		*/
		PxReal zAngleMin;

		/**
		\brief the maximum angle from the Z-axis of the constraint frame.

		<b>Unit:</b> Angular: Radians
		<b>Range:</b> Angular: (-PI,PI)<br>
		<b>Default:</b> PI/2
		*/
		PxReal zAngleMax;

		PX_CUDA_CALLABLE PxgJointLimitPyramid() {}

		/**
		\brief Construct a pyramid hard limit.

		\param[in] yLimitAngleMin	The minimum limit angle from the Y-axis of the constraint frame
		\param[in] yLimitAngleMax	The maximum limit angle from the Y-axis of the constraint frame
		\param[in] zLimitAngleMin	The minimum limit angle from the Z-axis of the constraint frame
		\param[in] zLimitAngleMax	The maximum limit angle from the Z-axis of the constraint frame

		\see PxJointLimitParameters
		*/
		PX_CUDA_CALLABLE PxgJointLimitPyramid(PxReal yLimitAngleMin, PxReal yLimitAngleMax, PxReal zLimitAngleMin, PxReal zLimitAngleMax) :
			yAngleMin(yLimitAngleMin),
			yAngleMax(yLimitAngleMax),
			zAngleMin(zLimitAngleMin),
			zAngleMax(zLimitAngleMax)
		{
			bounceThreshold = 0.5f;
		}

		/**
		\brief Construct a pyramid soft limit.

		\param[in] yLimitAngleMin	The minimum limit angle from the Y-axis of the constraint frame
		\param[in] yLimitAngleMax	The maximum limit angle from the Y-axis of the constraint frame
		\param[in] zLimitAngleMin	The minimum limit angle from the Z-axis of the constraint frame
		\param[in] zLimitAngleMax	The maximum limit angle from the Z-axis of the constraint frame
		\param[in] spring			The stiffness and damping of the limit spring

		\see PxJointLimitParameters
		*/
		PX_CUDA_CALLABLE PxgJointLimitPyramid(PxReal yLimitAngleMin, PxReal yLimitAngleMax, PxReal zLimitAngleMin, PxReal zLimitAngleMax, const PxgSpring& spring) :
			yAngleMin(yLimitAngleMin),
			yAngleMax(yLimitAngleMax),
			zAngleMin(zLimitAngleMin),
			zAngleMax(zLimitAngleMax)
		{
			stiffness = spring.stiffness;
			damping = spring.damping;
		}

		/**
		\brief Returns true if the limit is valid.

		\return true if the current settings are valid
		*/
		PX_INLINE bool isValid() const
		{
			return PxgJointLimitParameters::isValid() &&
				PxIsFinite(yAngleMin) && yAngleMin>-PxPi && yAngleMin<PxPi &&
				PxIsFinite(yAngleMax) && yAngleMax>-PxPi && yAngleMax<PxPi &&
				PxIsFinite(zAngleMin) && zAngleMin>-PxPi && zAngleMin<PxPi &&
				PxIsFinite(zAngleMax) && zAngleMax>-PxPi && zAngleMax<PxPi &&
				yAngleMax >= yAngleMin && zAngleMax >= zAngleMin;
		}
	};


}

#endif