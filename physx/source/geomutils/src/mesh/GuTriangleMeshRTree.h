// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_TRIANGLEMESH_RTREE_H
#define GU_TRIANGLEMESH_RTREE_H

#include "GuTriangleMesh.h"

namespace physx
{
namespace Gu
{
class MeshFactory;

#if PX_VC
#pragma warning(push)
#pragma warning(disable: 4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif

class RTreeTriangleMesh : public TriangleMesh
{
	public:
						virtual const char*				getConcreteTypeName()	const PX_OVERRIDE { return "PxBVH33TriangleMesh"; }
// PX_SERIALIZATION
														RTreeTriangleMesh(PxBaseFlags baseFlags) : TriangleMesh(baseFlags), mRTree(PxEmpty) {}
	PX_PHYSX_COMMON_API	virtual void					exportExtraData(PxSerializationContext& ctx)	PX_OVERRIDE;
								void					importExtraData(PxDeserializationContext&);
	PX_PHYSX_COMMON_API	static	TriangleMesh*			createObject(PxU8*& address, PxDeserializationContext& context);
//~PX_SERIALIZATION
														RTreeTriangleMesh(MeshFactory* factory, TriangleMeshData& data);
						virtual							~RTreeTriangleMesh(){}

						virtual	PxMeshMidPhase::Enum	getMidphaseID()			const PX_OVERRIDE { return PxMeshMidPhase::eBVH33; }

						virtual PxVec3*					getVerticesForModification() PX_OVERRIDE;
						virtual PxBounds3				refitBVH() PX_OVERRIDE;

	PX_FORCE_INLINE				const Gu::RTree&		getRTree()				const	{ return mRTree; }
	private:
								Gu::RTree				mRTree;								
};

#if PX_VC
#pragma warning(pop)
#endif

} // namespace Gu

}

#endif
