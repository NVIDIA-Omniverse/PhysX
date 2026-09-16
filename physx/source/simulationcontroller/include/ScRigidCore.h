// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
			eSHAPE2BODY			= 1<<1,
			eFILTERDATA			= 1<<2,
			eCONTACTOFFSET		= 1<<3,
			eRESTOFFSET			= 1<<4,
			eRESET_FILTERING	= 1<<5
		};
	};
	typedef PxFlags<ShapeChangeNotifyFlag::Enum, PxU32> ShapeChangeNotifyFlags;
	PX_FLAGS_OPERATORS(ShapeChangeNotifyFlag::Enum,PxU32)

	class RigidCore : public ActorCore
	{
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
	protected:
								RigidCore(const PxEMPTY) :	ActorCore(PxEmpty)	{}
								RigidCore(PxActorType::Enum type);
								~RigidCore();
	};

} // namespace Sc

}

#endif
