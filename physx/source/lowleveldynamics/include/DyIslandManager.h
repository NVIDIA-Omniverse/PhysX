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
	template <> struct IGNodeTraits<Dy::DeformableSurface>			{ enum { TypeID = IG::Node::eDEFORMABLE_SURFACE_TYPE };	};
	template <> struct IGNodeTraits<Dy::DeformableVolume>			{ enum { TypeID = IG::Node::eDEFORMABLE_VOLUME_TYPE };	};
	template <> struct IGNodeTraits<Dy::ParticleSystem>				{ enum { TypeID = IG::Node::ePARTICLESYSTEM_TYPE };		};

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
