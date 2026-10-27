// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_SERIALIZATION_REGISTRY_H
#define SN_SERIALIZATION_REGISTRY_H

#include "extensions/PxSerialization.h"
#include "extensions/PxRepXSerializer.h"

#include "foundation/PxUserAllocated.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"


namespace physx
{

namespace Cm { class Collection; }

namespace Sn {
	
	class SerializationRegistry : public PxSerializationRegistry, public PxUserAllocated
	{
	public:
		SerializationRegistry(PxPhysics& physics);					
		virtual						~SerializationRegistry();

		virtual void				release() PX_OVERRIDE { PX_DELETE_THIS;  }
		
		PxPhysics&			        getPhysics() const			{ return mPhysics; }
		
		//binary
		virtual	void				registerSerializer(PxType type, PxSerializer& serializer)	PX_OVERRIDE;
		virtual	PxSerializer*		unregisterSerializer(PxType type)	PX_OVERRIDE;
		virtual	const PxSerializer*	getSerializer(PxType type) const	PX_OVERRIDE;
				const char*			getSerializerName(PxU32 index) const;
				PxType				getSerializerType(PxU32 index) const;
				PxU32				getNbSerializers() const	{ return mSerializers.size(); }
		//repx
		virtual	void				registerRepXSerializer(PxType type, PxRepXSerializer& serializer)	PX_OVERRIDE;
		virtual	PxRepXSerializer*	getRepXSerializer(const char* typeName) const	PX_OVERRIDE;
		virtual	PxRepXSerializer*	unregisterRepXSerializer(PxType type)	PX_OVERRIDE;
	
	protected:
		SerializationRegistry &operator=(const SerializationRegistry &);
	private:
		typedef PxCoalescedHashMap<PxType, PxSerializer*>		SerializerMap;
		typedef PxHashMap<PxType, PxRepXSerializer*>	        RepXSerializerMap;

		PxPhysics&										mPhysics;
		SerializerMap									mSerializers;
		RepXSerializerMap								mRepXSerializers;
	};

	void  sortCollection(Cm::Collection& collection, SerializationRegistry& sr, bool isRepx);
} // Sn

} // physx



#endif

