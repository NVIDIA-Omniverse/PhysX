// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

		virtual PxBase* buildObjectFromData(PxConcreteType::Enum type, void* data) PX_OVERRIDE
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
