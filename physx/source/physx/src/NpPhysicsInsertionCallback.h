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

#ifndef NP_PHYSICS_INSERTION_CALLBACK_H
#define NP_PHYSICS_INSERTION_CALLBACK_H

#include "common/PxInsertionCallback.h"
#include "GuTriangleMesh.h"
#include "GuHeightField.h"
#include "GuConvexMesh.h"
#include "NpFactory.h"
#include "GuTetrahedronMesh.h"

namespace physx
{
	class NpPhysicsInsertionCallback : public PxInsertionCallback
	{
	public:
		NpPhysicsInsertionCallback() {}

		virtual PxBase* buildObjectFromData(PxConcreteType::Enum type, void* data)
		{
			if(type == PxConcreteType::eTRIANGLE_MESH_BVH33 || type == PxConcreteType::eTRIANGLE_MESH_BVH34)
				return NpFactory::getInstance().createTriangleMesh(data);

			if (type == PxConcreteType::eCONVEX_MESH)
				return NpFactory::getInstance().createConvexMesh(data);

			if (type == PxConcreteType::eHEIGHTFIELD)
				return NpFactory::getInstance().createHeightField(data);

			if (type == PxConcreteType::eBVH)
				return NpFactory::getInstance().createBVH(data);

			if (type == PxConcreteType::eTETRAHEDRON_MESH)
				return NpFactory::getInstance().createTetrahedronMesh(data);

			if (type == PxConcreteType::eDEFORMABLE_VOLUME_MESH)
				return NpFactory::getInstance().createDeformableVolumeMesh(data);

			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Inserting object failed: "
				"Object type not supported for buildObjectFromData.");

			return NULL;
		}

	};

}

#endif
