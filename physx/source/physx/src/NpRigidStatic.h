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

#ifndef NP_RIGID_STATIC_H
#define NP_RIGID_STATIC_H

#include "common/PxMetaData.h"
#include "PxRigidStatic.h"
#include "NpRigidActorTemplate.h"
#include "ScStaticCore.h"

namespace physx
{
typedef NpRigidActorTemplate<PxRigidStatic> NpRigidStaticT;

class NpRigidStatic : public NpRigidStaticT
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION
											NpRigidStatic(PxBaseFlags baseFlags) : NpRigidStaticT(baseFlags), mCore(PxEmpty) {}
					void					preExportDataReset() { NpRigidStaticT::preExportDataReset(); }
	virtual			void					requiresObjects(PxProcessPxBaseCallback& c);
	static			NpRigidStatic*			createObject(PxU8*& address, PxDeserializationContext& context);
	static			void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

											NpRigidStatic(const PxTransform& pose);
	virtual									~NpRigidStatic();

	// PxActor
	virtual			void					release()	PX_OVERRIDE;
	virtual			PxActorType::Enum		getType() const PX_OVERRIDE	{ return PxActorType::eRIGID_STATIC; }
	//~PxActor

	// PxRigidActor
	virtual			void 					setGlobalPose(const PxTransform& pose, bool wake)	PX_OVERRIDE;

	virtual			PxTransform				getGlobalPose() const	PX_OVERRIDE;
	
	//~PxRigidActor


	// PT: I think these come from NpRigidActorTemplate
	// PT: TODO: drop them eventually, they all re-route to NpActor now
	virtual			void					switchToNoSim()	PX_OVERRIDE;
	virtual			void					switchFromNoSim()	PX_OVERRIDE;

#if PX_CHECKED
					bool					checkConstraintValidity() const;
#endif

	PX_FORCE_INLINE	const Sc::StaticCore&	getCore()				const	{ return mCore;	}
	PX_FORCE_INLINE	Sc::StaticCore&			getCore()						{ return mCore;	}

	static PX_FORCE_INLINE size_t			getCoreOffset()					{ return PX_OFFSET_OF_RT(NpRigidStatic, mCore);			}
	static PX_FORCE_INLINE size_t			getNpShapeManagerOffset()		{ return PX_OFFSET_OF_RT(NpRigidStatic, mShapeManager);	}

#if PX_ENABLE_DEBUG_VISUALIZATION
					void					visualize(PxRenderOutput& out, NpScene& scene, float scale)	const;
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

private:
					Sc::StaticCore			mCore;
};

}

#endif
