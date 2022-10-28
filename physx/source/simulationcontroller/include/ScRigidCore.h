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

#ifndef SC_RIGID_CORE_H
#define SC_RIGID_CORE_H

#include "ScActorCore.h"
#include "ScPhysics.h"
#include "PxvDynamics.h"
#include "PxShape.h"

namespace physx
{
namespace Sc
{
	class RigidSim;
	class ShapeCore;

	struct ShapeChangeNotifyFlag
	{
		enum Enum
		{
			eGEOMETRY			= 1<<0,
			eMATERIAL			= 1<<1,
			eSHAPE2BODY			= 1<<2,
			eFILTERDATA			= 1<<3,
			eCONTACTOFFSET		= 1<<4,
			eRESTOFFSET			= 1<<5,
			eRESET_FILTERING	= 1<<6
		};
	};
	typedef PxFlags<ShapeChangeNotifyFlag::Enum, PxU32> ShapeChangeNotifyFlags;
	PX_FLAGS_OPERATORS(ShapeChangeNotifyFlag::Enum,PxU32)

	class RigidCore : public ActorCore
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================
	public:

	PX_FORCE_INLINE	PxActor*	getPxActor()	const
								{
									return PxPointerOffset<PxActor*>(const_cast<RigidCore*>(this), gOffsetTable.scCore2PxActor[getActorCoreType()]);
								}

					void		addShapeToScene(ShapeCore& shape);
					void		removeShapeFromScene(ShapeCore& shape, bool wakeOnLostTouch);
					void		onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags);
					void		onShapeFlagsChange(ShapeCore& shape, PxShapeFlags oldShapeFlags);
					void		unregisterShapeFromNphase(ShapeCore& shapeCore);
					void		registerShapeInNphase(ShapeCore& shapeCore);

					RigidSim*	getSim() const;

					PxU32		getRigidID() const;
	static			void		getBinaryMetaData(PxOutputStream& stream);
	protected:
								RigidCore(const PxEMPTY) :	ActorCore(PxEmpty)	{}
								RigidCore(PxActorType::Enum type);
								~RigidCore();
	};

} // namespace Sc

}

#endif
