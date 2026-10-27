// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_MATERIAL_SHARED_H
#define PXS_MATERIAL_SHARED_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
	#define	MATERIAL_INVALID_HANDLE	0xffff

	class PxOutputStream;

	template<class MaterialDataT, class PxMaterialT>
	class MaterialCoreT : public MaterialDataT
	{
		public:
						MaterialCoreT(const MaterialDataT& desc) : MaterialDataT(desc), mMaterial(NULL), mMaterialIndex(MATERIAL_INVALID_HANDLE)	{}
						MaterialCoreT() : mMaterial(NULL), mMaterialIndex(MATERIAL_INVALID_HANDLE)													{}

						MaterialCoreT(const PxEMPTY) : MaterialDataT(PxEmpty)	{}
						~MaterialCoreT()										{}

		PxMaterialT*	mMaterial;		// PT: TODO: eventually this could just be a base PxBaseMaterial class instead of a templated param
		PxU16			mMaterialIndex; //handle assign by the handle manager
	};

} //namespace phyxs

#endif
