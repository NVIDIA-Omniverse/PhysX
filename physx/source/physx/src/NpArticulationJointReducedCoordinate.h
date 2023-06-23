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

#ifndef NP_ARTICULATION_JOINT_RC_H
#define NP_ARTICULATION_JOINT_RC_H

#include "PxArticulationJointReducedCoordinate.h"
#include "ScArticulationJointCore.h"
#include "NpArticulationLink.h"
#include "NpBase.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
	#include "common/PxRenderOutput.h"
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

namespace physx
{
	class NpScene;
	class NpArticulationLink;

	class NpArticulationJointReducedCoordinate : public PxArticulationJointReducedCoordinate, public NpBase
	{
	public:
		// PX_SERIALIZATION
													NpArticulationJointReducedCoordinate(PxBaseFlags baseFlags)
														: PxArticulationJointReducedCoordinate(baseFlags), NpBase(PxEmpty), mCore(PxEmpty) {}
					void							preExportDataReset() { mCore.preExportDataReset(); }

		virtual		void							resolveReferences(PxDeserializationContext& context);
		static		NpArticulationJointReducedCoordinate* createObject(PxU8*& address, PxDeserializationContext& context);
		static		void							getBinaryMetaData(PxOutputStream& stream);
					void							exportExtraData(PxSerializationContext&) {}
					void							importExtraData(PxDeserializationContext&) {}
		virtual		void							requiresObjects(PxProcessPxBaseCallback&) {}
		virtual		bool							isSubordinate()  const { return true; }
		//~PX_SERIALIZATION
													NpArticulationJointReducedCoordinate(NpArticulationLink& parent, const PxTransform& parentFrame, NpArticulationLink& child, const PxTransform& childFrame);
		virtual										~NpArticulationJointReducedCoordinate();

		//---------------------------------------------------------------------------------
		// PxArticulationJoint implementation
		//---------------------------------------------------------------------------------

		virtual		void							setJointType(PxArticulationJointType::Enum jointType);
		virtual		PxArticulationJointType::Enum	getJointType() const;

		virtual		void							setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion);
		virtual		PxArticulationMotion::Enum		getMotion(PxArticulationAxis::Enum axis) const;

		virtual		void							setFrictionCoefficient(const PxReal coefficient);
		virtual		PxReal							getFrictionCoefficient() const;

		virtual		void							setMaxJointVelocity(const PxReal maxJointV);
		virtual		PxReal							getMaxJointVelocity() const;

		virtual		void							setLimitParams(PxArticulationAxis::Enum axis, const PxArticulationLimit& pair);
		virtual		PxArticulationLimit				getLimitParams(PxArticulationAxis::Enum axis) const;

		virtual		void							setDriveParams(PxArticulationAxis::Enum axis, const PxArticulationDrive& drive);
		virtual		PxArticulationDrive				getDriveParams(PxArticulationAxis::Enum axis) const;

		virtual		void							setDriveTarget(PxArticulationAxis::Enum axis, const PxReal target, bool autowake = true);
		virtual		PxReal							getDriveTarget(PxArticulationAxis::Enum axis) const;

		virtual		void							setDriveVelocity(PxArticulationAxis::Enum axis, const PxReal targetVel, bool autowake = true);
		virtual		PxReal							getDriveVelocity(PxArticulationAxis::Enum axis) const;

		virtual		void							setArmature(PxArticulationAxis::Enum axis, const PxReal armature);
		virtual		PxReal							getArmature(PxArticulationAxis::Enum axis) const;

		virtual	void								setJointPosition(PxArticulationAxis::Enum axis, const PxReal jointPos);
		virtual	PxReal								getJointPosition(PxArticulationAxis::Enum axis) const;

		virtual	void								setJointVelocity(PxArticulationAxis::Enum axis, const PxReal jointVel);
		virtual	PxReal								getJointVelocity(PxArticulationAxis::Enum axis) const;

		void										release();


		PX_FORCE_INLINE	Sc::ArticulationJointCore&	getCore()		{ return mCore; }
		static PX_FORCE_INLINE size_t				getCoreOffset()	{ return PX_OFFSET_OF_RT(NpArticulationJointReducedCoordinate, mCore); }

		PX_INLINE void						scSetParentPose(const PxTransform& v)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setParentPose(v);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetChildPose(const PxTransform& v)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setChildPose(v);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetJointType(PxArticulationJointType::Enum v)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setJointType(v);
			UPDATE_PVD_PROPERTY
		}
		PX_INLINE void						scSetFrictionCoefficient(const PxReal v)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setFrictionCoefficient(v);
			UPDATE_PVD_PROPERTY
		}
		PX_INLINE void						scSetMaxJointVelocity(const PxReal v)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setMaxJointVelocity(v);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetLimit(PxArticulationAxis::Enum axis, const PxArticulationLimit& pair)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setLimit(axis, pair);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetDrive(PxArticulationAxis::Enum axis, const PxArticulationDrive& drive)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setDrive(axis, drive);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetDriveTarget(PxArticulationAxis::Enum axis, PxReal targetP)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setTargetP(axis, targetP);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetDriveVelocity(PxArticulationAxis::Enum axis, PxReal targetP)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setTargetV(axis, targetP);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetArmature(PxArticulationAxis::Enum axis, PxReal armature)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setArmature(axis, armature);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setMotion(axis, motion);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetJointPosition(PxArticulationAxis::Enum axis, const PxReal jointPos)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setJointPosition(axis, jointPos);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE void						scSetJointVelocity(PxArticulationAxis::Enum axis, const PxReal jointVel)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setJointVelocity(axis, jointVel);
			UPDATE_PVD_PROPERTY
		}

		virtual     PxArticulationLink&				getParentArticulationLink() const { return *mParent; }
		virtual     PxArticulationLink&				getChildArticulationLink() const { return *mChild; }

		virtual		PxTransform						getParentPose() const;
		virtual		void							setParentPose(const PxTransform& t);

		virtual		PxTransform						getChildPose() const;
		virtual		void							setChildPose(const PxTransform& t); 


		PX_INLINE	const NpArticulationLink&		getParent() const { return *mParent; }
		PX_INLINE	NpArticulationLink&				getParent() { return *mParent; }

		PX_INLINE	const NpArticulationLink&		getChild() const { return *mChild; }
		PX_INLINE	NpArticulationLink&				getChild() { return *mChild; }

		Sc::ArticulationJointCore					mCore;
		NpArticulationLink*							mParent;
		NpArticulationLink*							mChild;
#if PX_CHECKED
	private:
					bool							isValidMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion);
#endif
	};
}

#endif
