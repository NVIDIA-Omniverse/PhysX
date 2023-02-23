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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#if PX_SUPPORT_OMNI_PVD

#include "OmniPvdPxExtensionsSampler.h"
#include "../pvdruntime/include/OmniPvdWriter.h"
#include "extensions/PxExtensionsAPI.h"
#include <stdio.h>

using namespace physx;

#define UNNECESSARY_SCENE_HANDLE 1

//array to hold PVD type to size conversion table.  Should be large enough for all basic PVD types.
static uint32_t sizeOfOmniPvdTypes[32];

//create a class for each SDK type and attribute -- the primary thing is to allocate handle storage, the rest is just fluff.
#define OMNI_PVD_FAKE_CLASS(c, classT, classStr) OmniPvdClassHandle OmniPvdPxExtensionsSampler::classHandle_##c;
#define OMNI_PVD_CLASS(c, classT) OmniPvdClassHandle OmniPvdPxExtensionsSampler::classHandle_##c;
#define OMNI_PVD_ENUM(c, classT) OMNI_PVD_CLASS(c, classT)
#define OMNI_PVD_CLASS_DERIVED(c, classT, baseClass) OMNI_PVD_CLASS(c, classT)
#define OMNI_PVD_ATTRIBUTE_SET(c, a, classT, attrT) OmniPvdAttributeHandle OmniPvdPxExtensionsSampler::attributeHandle_##c##_##a;
//enum values don't need to save their handle, since they are const/immutable:
#define OMNI_PVD_ENUM_VALUE(c, a, v)	
#define OMNI_PVD_ATTRIBUTE(c, a, classT, attrT, t, n) OmniPvdAttributeHandle OmniPvdPxExtensionsSampler::attributeHandle_##c##_##a;
#define OMNI_PVD_ATTRIBUTE_FLAG(c, a, classT, attrT, enumClass) OmniPvdAttributeHandle OmniPvdPxExtensionsSampler::attributeHandle_##c##_##a;

#include "OmniPvdPxExtensionsTypes.h"

#undef OMNI_PVD_ENUM
#undef OMNI_PVD_ENUM_VALUE
#undef OMNI_PVD_FAKE_CLASS
#undef OMNI_PVD_CLASS
#undef OMNI_PVD_CLASS_DERIVED
#undef OMNI_PVD_ATTRIBUTE
#undef OMNI_PVD_ATTRIBUTE_SET
#undef OMNI_PVD_ATTRIBUTE_FLAG

void OmniPvdPxExtensionsSampler::registerClasses()
{
	if (mWriter)
	{
//register all SDK classes and attributes:
#define OMNI_PVD_FAKE_CLASS(c, classT, classStr) OmniPvdPxExtensionsSampler::classHandle_##c = mWriter->registerClass(#classStr);
#define OMNI_PVD_CLASS(c, classT) OmniPvdPxExtensionsSampler::classHandle_##c = mWriter->registerClass(#classT);
#define OMNI_PVD_ENUM(c, classT) OMNI_PVD_CLASS(c, classT)
#define OMNI_PVD_CLASS_DERIVED(c, classT, baseClass) OmniPvdPxExtensionsSampler::classHandle_##c = mWriter->registerClass(#classT, OmniPvdPxExtensionsSampler::classHandle_##baseClass);
#define OMNI_PVD_ENUM_VALUE(c, a, v)				mWriter->registerEnumValue(OmniPvdPxExtensionsSampler::classHandle_##c, #a, v);
#define OMNI_PVD_ATTRIBUTE_SET(c, a, classT, attrT)	OmniPvdPxExtensionsSampler::attributeHandle_##c##_##a = mWriter->registerSetAttribute(OmniPvdPxExtensionsSampler::classHandle_##c, #a, OmniPvdDataTypeEnum::eOBJECT_HANDLE);
#define OMNI_PVD_ATTRIBUTE(c, a, classT, attrT, t, n) PX_ASSERT((n == 0) || (sizeof(attrT) == sizeOfOmniPvdTypes[t] * n)); OmniPvdPxExtensionsSampler::attributeHandle_##c##_##a = mWriter->registerAttribute(OmniPvdPxExtensionsSampler::classHandle_##c, #a, t, n);
#define OMNI_PVD_ATTRIBUTE_FLAG(c, a, classT, attrT, enumClass) OmniPvdPxExtensionsSampler::attributeHandle_##c##_##a = mWriter->registerFlagsAttribute(OmniPvdPxExtensionsSampler::classHandle_##c, OmniPvdPxExtensionsSampler::classHandle_##enumClass, #a);

#include "OmniPvdPxExtensionsTypes.h"

#undef OMNI_PVD_ENUM
#undef OMNI_PVD_ENUM_VALUE
#undef OMNI_PVD_FAKE_CLASS
#undef OMNI_PVD_CLASS
#undef OMNI_PVD_CLASS_DERIVED
#undef OMNI_PVD_ATTRIBUTE
#undef OMNI_PVD_ATTRIBUTE_SET
#undef OMNI_PVD_ATTRIBUTE_FLAG
	}
}

//instance any templates that are not used in this compilation unit so that code gets generated anyways

#define OMNI_PVD_FAKE_CLASS(c, classT, classStr) \
template void OmniPvdPxExtensionsSampler::createObject <classT>(OmniPvdClassHandle, classT const &);\
template void OmniPvdPxExtensionsSampler::destroyObject<classT>(classT const &);

#define OMNI_PVD_CLASS(c, classT) \
template void OmniPvdPxExtensionsSampler::createObject <classT>(OmniPvdClassHandle, classT const &);\
template void OmniPvdPxExtensionsSampler::destroyObject<classT>(classT const &);

#define OMNI_PVD_CLASS_DERIVED(c, classT, baseClass) OMNI_PVD_CLASS(c, classT)

#define OMNI_PVD_ATTRIBUTE_SET(c, a, classT, attrT) \
template void OmniPvdPxExtensionsSampler::addToSet<classT, attrT>(OmniPvdAttributeHandle, classT const & , attrT const & );\
template void OmniPvdPxExtensionsSampler::removeFromSet<classT, attrT>(OmniPvdAttributeHandle, classT const & , attrT const & );

#define OMNI_PVD_ATTRIBUTE(c, a, classT, attrT, t, n) \
template void OmniPvdPxExtensionsSampler::setAttribute<classT, attrT>(OmniPvdAttributeHandle, const classT&, attrT const &); \
template void OmniPvdPxExtensionsSampler::setAttributeBytes<classT, attrT>(OmniPvdAttributeHandle, const classT&, attrT const *, unsigned);

#define OMNI_PVD_ATTRIBUTE_FLAG(c, a, classT, attrT, enumClass) \
template void OmniPvdPxExtensionsSampler::setAttribute<classT, attrT>(OmniPvdAttributeHandle, classT const &, attrT const &);

#define OMNI_PVD_ENUM(c, enumT)
#define OMNI_PVD_ENUM_VALUE(c, a, v)

#include "OmniPvdPxExtensionsTypes.h"

#undef OMNI_PVD_ENUM
#undef OMNI_PVD_ENUM_VALUE
#undef OMNI_PVD_FAKE_CLASS
#undef OMNI_PVD_CLASS
#undef OMNI_PVD_CLASS_DERIVED
#undef OMNI_PVD_ATTRIBUTE
#undef OMNI_PVD_ATTRIBUTE_SET
//end instance templates

OmniPvdPxExtensionsSampler::OmniPvdPxExtensionsSampler() : mWriter(NULL)
{

	//TODO: this could be done better
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eINT8] = 1;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eINT16] = 2;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eINT32] = 4;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eINT64] = 8;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eUINT8] = 1;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eUINT16] = 2;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eUINT32] = 4;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eUINT64] = 8;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eFLOAT32] = 4;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eFLOAT64] = 8;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eSTRING] = 1;
	sizeOfOmniPvdTypes[OmniPvdDataTypeEnum::eOBJECT_HANDLE] = sizeof(uint64_t);
}

OmniPvdPxExtensionsSampler::~OmniPvdPxExtensionsSampler()
{
}

void OmniPvdPxExtensionsSampler::setOmniPvdWriter(OmniPvdWriter* omniPvdWriter)
{
	mWriter = omniPvdWriter;
}

//generic PVD API: 

template <typename ClassType> void OmniPvdPxExtensionsSampler::createObject(OmniPvdClassHandle ch, ClassType const & objectId)
{
	PX_ASSERT(mWriter);
	mWriter->createObject(UNNECESSARY_SCENE_HANDLE, ch, OmniPvdObjectHandle(&objectId), NULL);
}

template <typename ClassType> void OmniPvdPxExtensionsSampler::destroyObject(ClassType const & objectId)
{
	PX_ASSERT(mWriter);
	mWriter->destroyObject(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId));
}

template <typename ClassType, typename AttributeType> void OmniPvdPxExtensionsSampler::setAttribute(OmniPvdAttributeHandle ah, const ClassType & objectId, const AttributeType & value)
{
	PX_ASSERT(mWriter);
	mWriter->setAttributeShallow(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId), ah, (const unsigned char*)&value, sizeof(AttributeType));
}

template <typename ClassType, typename AttributeType> void OmniPvdPxExtensionsSampler::setAttributeBytes(OmniPvdAttributeHandle ah, ClassType const & objectId, const AttributeType * value, unsigned nBytes)
{
	PX_ASSERT(mWriter);
	mWriter->setAttributeShallow(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId), ah, (const unsigned char*)value, nBytes);
}

template <typename ClassType, typename AttributeType> void OmniPvdPxExtensionsSampler::addToSet(OmniPvdAttributeHandle ah, ClassType const & objectId, AttributeType const & value)
{
	PX_ASSERT(mWriter);
	const AttributeType * atp = &value;
	mWriter->addToSetAttributeShallow(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId), ah, (const unsigned char*)&atp, sizeof(atp));
}

template <typename ClassType, typename AttributeType> void OmniPvdPxExtensionsSampler::removeFromSet(OmniPvdAttributeHandle ah, ClassType const & objectId, AttributeType const & value)
{
	PX_ASSERT(mWriter);
	const AttributeType * atp = &value;
	mWriter->removeFromSetAttributeShallow(UNNECESSARY_SCENE_HANDLE, OmniPvdObjectHandle(&objectId), ah, (const unsigned char*)&atp, sizeof(atp) );
}

///////////////////////////////////////////////////////////////////////////////

static OmniPvdPxExtensionsSampler* gOmniPvdPxExtensionsSampler = NULL;

bool OmniPvdPxExtensionsSampler::createInstance()
{
	gOmniPvdPxExtensionsSampler = PX_NEW(OmniPvdPxExtensionsSampler)();
	return gOmniPvdPxExtensionsSampler != NULL;
}

OmniPvdPxExtensionsSampler* OmniPvdPxExtensionsSampler::getInstance()
{
	return gOmniPvdPxExtensionsSampler;
}

void OmniPvdPxExtensionsSampler::destroyInstance()
{
	PX_DELETE(gOmniPvdPxExtensionsSampler);
}

#endif
