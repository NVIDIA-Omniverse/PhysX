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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef OMNI_PVD_PX_SAMPLER_H
#define OMNI_PVD_PX_SAMPLER_H

/*
The below helper macros are to be used in SDK code to send information to PVD as unintrusively as possible.

Principles:
* Data created or changed by the user should be sent as soon as it has been written to an SDK class, ideally right at the end of the API create() or set() call.
* Objects should ideally be destroyed from their destructors (since release() calls might just decrement).
* Data written by the SDK should be sent at the end of the simulate frame.
* Ideally use Px-pointers as object handles.  Beware that multiple inheritance can result in different pointer values so its best to cast to the Px-type pointer / reference explicitly.
	Even if the code works by passing in different equivalent pointer types, this will generate unnecessary duplicate template code.
*/
#if PX_SUPPORT_OMNI_PVD
// You can use this in conditional statements to check for a connection
	#define OMNI_PVD_ACTIVE				(::OmniPvdPxSampler::getInstance() != NULL)
// Create object reference o of PVD type c.  Example: 	OMNI_PVD_CREATE(scene, static_cast<PxScene &>(*npScene));
	#define OMNI_PVD_CREATE(c, o)		if (::OmniPvdPxSampler::getInstance() != NULL)	{::OmniPvdPxSampler::getInstance()->createObject(OmniPvdPxSampler::classHandle_##c, o); }
// Destroy object reference o of PVD type c.  Example: OMNI_PVD_DESTROY(scene, static_cast<PxScene &>(*npScene));
	#define OMNI_PVD_DESTROY(c, o)		if (::OmniPvdPxSampler::getInstance() != NULL)	{::OmniPvdPxSampler::getInstance()->destroyObject(o); }
// Set PVD attribute a of object reference o of PVD type c to value v.  v is passed as reference to value; PVD object handles are passed as reference to POINTER here!
// Example: OMNI_PVD_SET(actor, isdynamic, a, false)  
	#define OMNI_PVD_SET(c, a, o, v)	if (::OmniPvdPxSampler::getInstance() != NULL)	{::OmniPvdPxSampler::getInstance()->setAttribute(OmniPvdPxSampler::attributeHandle_##c##_##a, o, v); }
// Same as set, but for variable length attributes like vertex buffers.  pv is the address of the data, and n is the size in bytes.
	#define OMNI_PVD_SETB(c, a, o, pv, n)if (::OmniPvdPxSampler::getInstance() != NULL)	{::OmniPvdPxSampler::getInstance()->setAttributeBytes(OmniPvdPxSampler::attributeHandle_##c##_##a, o, pv, n); }
// Same as set, but for attribute sets of unique things like an array of references.  v is passed as a REFERENCE.
	#define OMNI_PVD_ADD(c, a, o, v)	if (::OmniPvdPxSampler::getInstance() != NULL)	{::OmniPvdPxSampler::getInstance()->addToSet(OmniPvdPxSampler::attributeHandle_##c##_##a, o, v); }
// TO remove a member handle from the set.
	#define OMNI_PVD_REMOVE(c, a, o, v)	if (::OmniPvdPxSampler::getInstance() != NULL)	{::OmniPvdPxSampler::getInstance()->removeFromSet(OmniPvdPxSampler::attributeHandle_##c##_##a, o, v); }

#else
	#define OMNI_PVD_ACTIVE				(false)
	#define OMNI_PVD_CREATE(c, p)
	#define OMNI_PVD_DESTROY(c, p)
	#define OMNI_PVD_SET(c, a, p, v)
	#define OMNI_PVD_SETB(c, a, p, v, n)
	#define OMNI_PVD_ADD(c, a, p, v)
	#define OMNI_PVD_REMOVE(c, a, p, v)
#endif


#if PX_SUPPORT_OMNI_PVD
#include "omnipvd/PxOmniPvd.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxMutex.h"
#include "OmniPvdChunkAlloc.h"
#include "foundation/PxUserAllocated.h"

#include "../../../pvdruntime/include/OmniPvdDefines.h"

namespace physx
{
	class PxScene;
	class PxBase;
	class NpActor;
	class NpScene;
	class NpShape;
	class NpShapeManager;
	class PxActor;
	class PxGeometryHolder;
	class NpOmniPvd;
	class PxGeometry;
	class PxJoint;
	class PxShape;
	class PxMaterial;
}

extern void streamActorName(physx::PxActor & a, const char* name);
extern void streamShapeMaterials(physx::PxShape* shapePtr, physx::PxMaterial** mats, physx::PxU32 nbrMaterials);

enum OmniPvdSharedMeshEnum {
	eOmniPvdTriMesh     = 0,
	eOmniPvdConvexMesh  = 1,
	eOmniPvdHeightField = 2,
};

class OmniPvdLoader;
class OmniPvdWriter;
class OmniPvdPxSampler;
class OmniPvdWriteStream;
class OmniPvdActorMod;
class OmniPvdShapeMod;
class OmniPvdPxScene;

class OmniPvdPxSampler : public physx::PxUserAllocated
{
public:
	OmniPvdPxSampler();
	~OmniPvdPxSampler();
	//enables sampling: 
	void startSampling();
	bool isSampling();
	//sets destination: 
	void setOmniPvdWriter(OmniPvdWriter* omniPvdWriter);	

	// writes all contacts to the stream
	void streamSceneContacts(physx::NpScene* scene);

	// call at the end of a simulation step: 
	void sampleScene(physx::NpScene* scene);
	//// convenience function to handle different kinds of geometry correctly, including capturing meshes:
	//void createGeometry(const physx::PxGeometry & g);
	//void destroyGeometry(const physx::PxGeometry & g);

	//simplified generic API to be used via simple macros above: 
	template <typename ClassType> void createObject(OmniPvdClassHandle, ClassType const & objectId);
	template <typename ClassType> void destroyObject(ClassType const & objectId);
	template <typename ClassType, typename AttributeType> void setAttribute(OmniPvdAttributeHandle, ClassType const & objectId,  AttributeType const & value);
	template <typename ClassType, typename AttributeType> void setAttributeBytes(OmniPvdAttributeHandle, ClassType const & objectId, AttributeType const * value, unsigned nBytes);
	template <typename ClassType, typename AttributeType> void addToSet(OmniPvdAttributeHandle, ClassType const & objectId, AttributeType const & value);
	template <typename ClassType, typename AttributeType> void removeFromSet(OmniPvdAttributeHandle, ClassType const & objectId, AttributeType const & value);

	//handles for all SDK classes and attributes

#define OMNI_PVD_FAKE_CLASS(c, classT, classStr) static OmniPvdClassHandle classHandle_##c;
#define OMNI_PVD_CLASS(c, classT) static OmniPvdClassHandle classHandle_##c;
#define OMNI_PVD_CLASS_DERIVED(c, classT, baseClass) OMNI_PVD_CLASS(c, classT)
#define OMNI_PVD_ENUM(c, classT) OMNI_PVD_CLASS(c, classT)
#define OMNI_PVD_ENUM_VALUE(c, a, v)	
#define OMNI_PVD_ATTRIBUTE(c, a, classT, attrT, t, n) static OmniPvdAttributeHandle attributeHandle_##c##_##a;
#define OMNI_PVD_ATTRIBUTE_SET(c, a, classT, attrT) static OmniPvdAttributeHandle attributeHandle_##c##_##a;
#define OMNI_PVD_ATTRIBUTE_FLAG(c, a, classT, attrT, enumClass) static OmniPvdAttributeHandle attributeHandle_##c##_##a;


#include "OmniPvdTypes.h"	//SDK classes and attributes declared here
#undef OMNI_PVD_ENUM
#undef OMNI_PVD_ENUM_VALUE
#undef OMNI_PVD_FAKE_CLASS
#undef OMNI_PVD_CLASS
#undef OMNI_PVD_CLASS_DERIVED
#undef OMNI_PVD_ATTRIBUTE
#undef OMNI_PVD_ATTRIBUTE_SET
#undef OMNI_PVD_ATTRIBUTE_FLAG

	static OmniPvdPxSampler* getInstance();

	void onObjectAdd(const physx::PxBase* object);
	void onObjectRemove(const physx::PxBase* object);


private:
	OmniPvdPxScene* getSampledScene(physx::NpScene* scene);
};

#endif

#endif
