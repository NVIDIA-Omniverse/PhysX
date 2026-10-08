// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_TRIANGLEMESH_BV4_H
#define GU_TRIANGLEMESH_BV4_H

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

class BV4TriangleMesh : public TriangleMesh
{
	public:
						virtual const char*				getConcreteTypeName()	const PX_OVERRIDE { return "PxBVH34TriangleMesh"; }
// PX_SERIALIZATION
														BV4TriangleMesh(PxBaseFlags baseFlags) : TriangleMesh(baseFlags), mMeshInterface(PxEmpty), mBV4Tree(PxEmpty)	{}
	PX_PHYSX_COMMON_API	virtual void					exportExtraData(PxSerializationContext& ctx)	PX_OVERRIDE;
								void					importExtraData(PxDeserializationContext&);
	PX_PHYSX_COMMON_API	static	TriangleMesh*			createObject(PxU8*& address, PxDeserializationContext& context);
//~PX_SERIALIZATION
														BV4TriangleMesh(MeshFactory* factory, TriangleMeshData& data);
						virtual							~BV4TriangleMesh(){}

						virtual	PxMeshMidPhase::Enum	getMidphaseID()			const PX_OVERRIDE { return PxMeshMidPhase::eBVH34;	}

						virtual PxVec3*					getVerticesForModification() PX_OVERRIDE;
						virtual PxBounds3				refitBVH() PX_OVERRIDE;

	PX_FORCE_INLINE				const Gu::BV4Tree&		getBV4Tree()			const	{ return mBV4Tree;				}
	private:
								Gu::SourceMesh			mMeshInterface;
								Gu::BV4Tree				mBV4Tree;
};

#if PX_VC
#pragma warning(pop)
#endif

} // namespace Gu

}

#endif
