// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_ISLAND_MANAGER_H
#define DY_ISLAND_MANAGER_H

// PT: low-level-dynamics client helper code for using the low-level island sim

#include "PxsIslandSim.h"

namespace physx
{
	class PxsRigidBody;

	namespace Dy
	{
		class FeatherstoneArticulation;
#if PX_SUPPORT_GPU_PHYSX
		class DeformableSurface;
		class DeformableVolume;
		class ParticleSystem;
#endif
	}

	template <typename T>
	struct IGNodeTraits
	{
		enum {TypeID = IG::Node::eTYPE_COUNT };
	};
	template <typename T> struct IGNodeTraits<const T> { enum { TypeID = IGNodeTraits<T>::TypeID }; };

	template <> struct IGNodeTraits<PxsRigidBody>					{ enum { TypeID = IG::Node::eRIGID_BODY_TYPE };			};
	template <> struct IGNodeTraits<Dy::FeatherstoneArticulation>	{ enum { TypeID = IG::Node::eARTICULATION_TYPE };		};

#if PX_SUPPORT_GPU_PHYSX
	template <> struct IGNodeTraits<Dy::DeformableSurface>			{ enum { TypeID = IG::Node::eDEFORMABLE_SURFACE_TYPE };	};
	template <> struct IGNodeTraits<Dy::DeformableVolume>			{ enum { TypeID = IG::Node::eDEFORMABLE_VOLUME_TYPE };	};
	template <> struct IGNodeTraits<Dy::ParticleSystem>				{ enum { TypeID = IG::Node::ePARTICLESYSTEM_TYPE };		};
#endif

	template<class T>
	PX_FORCE_INLINE T* getObjectFromIG(const IG::Node& node)
	{
		PX_ASSERT(PxU32(node.mType) == PxU32(IGNodeTraits<T>::TypeID));
		return reinterpret_cast<T*>(node.mObject);
	}

	PX_FORCE_INLINE PxsRigidBody* getRigidBodyFromIG(const IG::IslandSim& islandSim, PxNodeIndex nodeIndex)
	{
		return reinterpret_cast<PxsRigidBody*>(islandSim.getObject(nodeIndex, IG::Node::eRIGID_BODY_TYPE));
	}

	PX_FORCE_INLINE Dy::FeatherstoneArticulation* getArticulationFromIG(const IG::IslandSim& islandSim, PxNodeIndex nodeIndex)
	{
		return reinterpret_cast<Dy::FeatherstoneArticulation*>(islandSim.getObject(nodeIndex, IG::Node::eARTICULATION_TYPE));
	}
}

#endif