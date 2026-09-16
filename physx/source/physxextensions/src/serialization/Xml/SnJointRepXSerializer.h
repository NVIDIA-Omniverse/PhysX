// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_JOINT_REPX_SERIALIZER_H
#define SN_JOINT_REPX_SERIALIZER_H

#include "extensions/PxRepXSimpleType.h"
#include "SnRepXSerializerImpl.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
	
	class XmlReader;
	class XmlMemoryAllocator;
	class XmlWriter;
	class MemoryBuffer;
	
	template<typename TJointType>
	struct PX_DEPRECATED PxJointRepXSerializer : public RepXSerializerImpl<TJointType>
	{
		PxJointRepXSerializer(PxAllocatorCallback& inAllocator) : RepXSerializerImpl<TJointType>(inAllocator) {}
		virtual PxRepXObject fileToObject(XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection) PX_OVERRIDE;
		virtual void objectToFileImpl(const TJointType* inObj, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs&) PX_OVERRIDE;
		virtual TJointType* allocateObject(PxRepXInstantiationArgs&) PX_OVERRIDE { return NULL; }
	};

#if PX_SUPPORT_EXTERN_TEMPLATE
	// explicit template instantiations declarations
	extern template struct PX_DEPRECATED PxJointRepXSerializer<PxD6Joint>;
	extern template struct PX_DEPRECATED PxJointRepXSerializer<PxDistanceJoint>;
	extern template struct PX_DEPRECATED PxJointRepXSerializer<PxFixedJoint>;
	extern template struct PX_DEPRECATED PxJointRepXSerializer<PxPrismaticJoint>;
	extern template struct PX_DEPRECATED PxJointRepXSerializer<PxRevoluteJoint>;
	extern template struct PX_DEPRECATED PxJointRepXSerializer<PxSphericalJoint>;
#endif

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
