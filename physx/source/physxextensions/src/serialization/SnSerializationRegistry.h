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

		virtual void				release(){ PX_DELETE_THIS;  }
		
		PxPhysics&			        getPhysics() const			{ return mPhysics; }
		
		//binary
		void						registerSerializer(PxType type, PxSerializer& serializer);
		PxSerializer*               unregisterSerializer(PxType type);
		void						registerBinaryMetaDataCallback(PxBinaryMetaDataCallback callback);	
		void						getBinaryMetaData(PxOutputStream& stream) const;
		const PxSerializer*			getSerializer(PxType type) const;
		const char*			        getSerializerName(PxU32 index) const;
		PxType                      getSerializerType(PxU32 index) const;
		PxU32                       getNbSerializers() const { return mSerializers.size(); } 
		//repx
		void						registerRepXSerializer(PxType type, PxRepXSerializer& serializer);
		PxRepXSerializer*			getRepXSerializer(const char* typeName) const;
		PxRepXSerializer*           unregisterRepXSerializer(PxType type);
	
	protected:
		SerializationRegistry &operator=(const SerializationRegistry &);
	private:
		typedef PxCoalescedHashMap<PxType, PxSerializer*>		SerializerMap;
		typedef PxHashMap<PxType, PxRepXSerializer*>	        RepXSerializerMap;

		PxPhysics&										mPhysics;
		SerializerMap									mSerializers;
		RepXSerializerMap								mRepXSerializers;
		PxArray<PxBinaryMetaDataCallback>				mMetaDataCallbacks;	
	};

	void  sortCollection(Cm::Collection& collection, SerializationRegistry& sr, bool isRepx);
} // Sn

} // physx



#endif

