// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_REPX_CORE_SERIALIZER_H
#define SN_REPX_CORE_SERIALIZER_H
#include "foundation/PxSimpleTypes.h"
#include "SnRepXSerializerImpl.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class XmlReader;
	class XmlMemoryAllocator;
	class XmlWriter;
	class MemoryBuffer;
				
	struct PX_DEPRECATED PxMaterialRepXSerializer : RepXSerializerImpl<PxMaterial>
	{
		PxMaterialRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxMaterial>( inCallback ) {}
		virtual PxMaterial* allocateObject( PxRepXInstantiationArgs& ) PX_OVERRIDE;
	};

	struct PX_DEPRECATED PxShapeRepXSerializer : public RepXSerializerImpl<PxShape>
	{
		PxShapeRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxShape>( inCallback ) {}
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* ) PX_OVERRIDE;
		virtual PxShape* allocateObject( PxRepXInstantiationArgs& ) PX_OVERRIDE { return NULL; }
	};
	
	struct PX_DEPRECATED PxBVH33TriangleMeshRepXSerializer  : public RepXSerializerImpl<PxBVH33TriangleMesh>
	{
		PxBVH33TriangleMeshRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxBVH33TriangleMesh>( inCallback ) {}
		virtual void objectToFileImpl( const PxBVH33TriangleMesh*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs& ) PX_OVERRIDE;
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* ) PX_OVERRIDE;
		virtual PxBVH33TriangleMesh* allocateObject( PxRepXInstantiationArgs&  ) PX_OVERRIDE { return NULL; }
	};
	struct PX_DEPRECATED PxBVH34TriangleMeshRepXSerializer  : public RepXSerializerImpl<PxBVH34TriangleMesh>
	{
		PxBVH34TriangleMeshRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxBVH34TriangleMesh>( inCallback ) {}
		virtual void objectToFileImpl( const PxBVH34TriangleMesh*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs& ) PX_OVERRIDE;
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* ) PX_OVERRIDE;
		virtual PxBVH34TriangleMesh* allocateObject( PxRepXInstantiationArgs&  ) PX_OVERRIDE { return NULL; }
	};

	struct PX_DEPRECATED PxHeightFieldRepXSerializer : public RepXSerializerImpl<PxHeightField>
	{
		PxHeightFieldRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxHeightField>( inCallback ) {}
		virtual void objectToFileImpl( const PxHeightField*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs& ) PX_OVERRIDE;
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* ) PX_OVERRIDE;
		virtual PxHeightField* allocateObject( PxRepXInstantiationArgs& ) PX_OVERRIDE { return NULL; }
	};
	
	struct PX_DEPRECATED PxConvexMeshRepXSerializer  : public RepXSerializerImpl<PxConvexMesh>
	{
		PxConvexMeshRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxConvexMesh>( inCallback ) {}
		virtual void objectToFileImpl( const PxConvexMesh*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs& ) PX_OVERRIDE;
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* ) PX_OVERRIDE;
		virtual PxConvexMesh* allocateObject( PxRepXInstantiationArgs& ) PX_OVERRIDE { return NULL; }
	};

	struct PX_DEPRECATED PxRigidStaticRepXSerializer : public RepXSerializerImpl<PxRigidStatic>
	{
		PxRigidStaticRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxRigidStatic>( inCallback ) {}
		virtual PxRigidStatic* allocateObject( PxRepXInstantiationArgs& ) PX_OVERRIDE;
	};

	struct PX_DEPRECATED PxRigidDynamicRepXSerializer : public RepXSerializerImpl<PxRigidDynamic>
	{
		PxRigidDynamicRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxRigidDynamic>( inCallback ) {}
		virtual PxRigidDynamic* allocateObject( PxRepXInstantiationArgs& ) PX_OVERRIDE;
	};
	
	
	struct PX_DEPRECATED PxArticulationReducedCoordinateRepXSerializer : public RepXSerializerImpl<PxArticulationReducedCoordinate>
	{
		PxArticulationReducedCoordinateRepXSerializer(PxAllocatorCallback& inCallback) : RepXSerializerImpl<PxArticulationReducedCoordinate>(inCallback) {}
		virtual void objectToFileImpl(const PxArticulationReducedCoordinate*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs&) PX_OVERRIDE;
		virtual PxArticulationReducedCoordinate* allocateObject(PxRepXInstantiationArgs&) PX_OVERRIDE;
	};
	
	struct PX_DEPRECATED PxAggregateRepXSerializer :  public RepXSerializerImpl<PxAggregate>
	{
		PxAggregateRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxAggregate>( inCallback ) {}
		virtual void objectToFileImpl( const PxAggregate*, PxCollection*, XmlWriter& , MemoryBuffer&, PxRepXInstantiationArgs& ) PX_OVERRIDE;
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* ) PX_OVERRIDE;
		virtual PxAggregate* allocateObject( PxRepXInstantiationArgs& ) PX_OVERRIDE { return NULL; }
	};


#if !PX_DOXYGEN
} // namespace physx
#endif

#endif

