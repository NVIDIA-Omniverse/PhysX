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

#include "GuMidphaseInterface.h"
#include "GuTetrahedronMesh.h"
#include "GuBox.h"

using namespace physx;
using namespace Gu;

void TetrahedronMesh::onRefCountZero()
{
	if (mMeshFactory)
	{
		::onRefCountZero(this, mMeshFactory, false, "PxTetrahedronMesh::release: double deletion detected!");
	}
}

void SoftBodyMesh::onRefCountZero()
{
	if (mMeshFactory)
	{
		::onRefCountZero(this, mMeshFactory, false, "PxSoftBodyMesh::release: double deletion detected!");
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SoftBodyAuxData::SoftBodyAuxData(SoftBodySimulationData& d, SoftBodyCollisionData& c, CollisionMeshMappingData& e)
	: PxSoftBodyAuxData(PxType(PxConcreteType::eSOFT_BODY_STATE), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
	, mGridModelInvMass(d.mGridModelInvMass)
	, mGridModelTetraRestPoses(d.mGridModelTetraRestPoses)
	, mGridModelOrderedTetrahedrons(d.mGridModelOrderedTetrahedrons)
	, mGMNbPartitions(d.mGridModelNbPartitions)
	, mGMMaxMaxTetsPerPartitions(d.mGridModelMaxTetsPerPartitions)
	, mGMRemapOutputSize(d.mGMRemapOutputSize)
	, mGMRemapOutputCP(d.mGMRemapOutputCP)
	, mGMAccumulatedPartitionsCP(d.mGMAccumulatedPartitionsCP)
	, mGMAccumulatedCopiesCP(d.mGMAccumulatedCopiesCP)
	, mCollisionAccumulatedTetrahedronsRef(e.mCollisionAccumulatedTetrahedronsRef)
	, mCollisionTetrahedronsReferences(e.mCollisionTetrahedronsReferences)
	, mCollisionNbTetrahedronsReferences(e.mCollisionNbTetrahedronsReferences)
	, mCollisionSurfaceVertsHint(e.mCollisionSurfaceVertsHint)
	, mCollisionSurfaceVertToTetRemap(e.mCollisionSurfaceVertToTetRemap)
	, mVertsBarycentricInGridModel(e.mVertsBarycentricInGridModel)
	, mVertsRemapInGridModel(e.mVertsRemapInGridModel)
	, mTetsRemapColToSim(e.mTetsRemapColToSim)
	, mTetsRemapSize(e.mTetsRemapSize)
	, mTetsAccumulatedRemapColToSim(e.mTetsAccumulatedRemapColToSim)

	, mGMPullIndices(d.mGMPullIndices)
	, mTetraRestPoses(c.mTetraRestPoses)
	, mNumTetsPerElement(d.mNumTetsPerElement)

{
	// this constructor takes ownership of memory from the data object
	d.mGridModelInvMass = 0;
	d.mGridModelTetraRestPoses = 0;
	d.mGridModelOrderedTetrahedrons = 0;
	d.mGMRemapOutputCP = 0;
	d.mGMAccumulatedPartitionsCP = 0;
	d.mGMAccumulatedCopiesCP = 0;
	e.mCollisionAccumulatedTetrahedronsRef = 0;
	e.mCollisionTetrahedronsReferences = 0;
	e.mCollisionSurfaceVertsHint = 0;
	e.mCollisionSurfaceVertToTetRemap = 0;
	d.mGMPullIndices = 0;

	e.mVertsBarycentricInGridModel = 0;
	e.mVertsRemapInGridModel = 0;
	e.mTetsRemapColToSim = 0;
	e.mTetsAccumulatedRemapColToSim = 0;
	
	c.mTetraRestPoses = 0;
}

SoftBodyAuxData::~SoftBodyAuxData()
{
	PX_FREE(mGridModelInvMass);

	PX_FREE(mGridModelTetraRestPoses);
	PX_FREE(mGridModelOrderedTetrahedrons);
	PX_FREE(mGMRemapOutputCP);
	PX_FREE(mGMAccumulatedPartitionsCP);
	PX_FREE(mGMAccumulatedCopiesCP);
	PX_FREE(mCollisionAccumulatedTetrahedronsRef);
	PX_FREE(mCollisionTetrahedronsReferences);
	PX_FREE(mCollisionSurfaceVertsHint);
	PX_FREE(mCollisionSurfaceVertToTetRemap);

	PX_FREE(mVertsBarycentricInGridModel);
	PX_FREE(mVertsRemapInGridModel);
	PX_FREE(mTetsRemapColToSim);
	PX_FREE(mTetsAccumulatedRemapColToSim);

	PX_FREE(mGMPullIndices); 
	PX_FREE(mTetraRestPoses);
}

TetrahedronMesh::TetrahedronMesh(PxU32 nbVertices, PxVec3* vertices, PxU32 nbTetrahedrons, void* tetrahedrons, PxU8 flags, PxBounds3 aabb, PxReal geomEpsilon)
	: PxTetrahedronMesh(PxType(PxConcreteType::eTETRAHEDRON_MESH), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
	, mNbVertices(nbVertices)
	, mVertices(vertices)
	, mNbTetrahedrons(nbTetrahedrons)
	, mTetrahedrons(tetrahedrons)
	, mFlags(flags)
	, mMaterialIndices(NULL)
	, mAABB(aabb)
	, mGeomEpsilon(geomEpsilon)
{
}

TetrahedronMesh::TetrahedronMesh(TetrahedronMeshData& mesh)
	: PxTetrahedronMesh(PxType(PxConcreteType::eTETRAHEDRON_MESH), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
	, mNbVertices(mesh.mNbVertices)
	, mVertices(mesh.mVertices)
	, mNbTetrahedrons(mesh.mNbTetrahedrons)
	, mTetrahedrons(mesh.mTetrahedrons)
	, mFlags(mesh.mFlags)
	, mMaterialIndices(mesh.mMaterialIndices)
	, mAABB(mesh.mAABB)
	, mGeomEpsilon(mesh.mGeomEpsilon)
{
	// this constructor takes ownership of memory from the data object	
	mesh.mVertices = 0;
	mesh.mTetrahedrons = 0;	
	mesh.mMaterialIndices = 0;
}

TetrahedronMesh::TetrahedronMesh(MeshFactory* meshFactory, TetrahedronMeshData& mesh) 
	: PxTetrahedronMesh(PxType(PxConcreteType::eTETRAHEDRON_MESH), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)	
	, mNbVertices(mesh.mNbVertices)
	, mVertices(mesh.mVertices)
	, mNbTetrahedrons(mesh.mNbTetrahedrons)
	, mTetrahedrons(mesh.mTetrahedrons)
	, mFlags(mesh.mFlags)
	, mMaterialIndices(mesh.mMaterialIndices)
	, mAABB(mesh.mAABB)
	, mGeomEpsilon(mesh.mGeomEpsilon)	
	, mMeshFactory(meshFactory)
{
	// this constructor takes ownership of memory from the data object	
	mesh.mVertices = 0;
	mesh.mTetrahedrons = 0;
	mesh.mMaterialIndices = 0;
}

TetrahedronMesh::~TetrahedronMesh()
{
	PX_FREE(mTetrahedrons);
	PX_FREE(mVertices);
	PX_FREE(mMaterialIndices);
}

BVTetrahedronMesh::BVTetrahedronMesh(TetrahedronMeshData& mesh, SoftBodyCollisionData& d, MeshFactory* factory) : TetrahedronMesh(mesh)
, mFaceRemap(d.mFaceRemap)
, mGRB_tetraIndices(d.mGRB_primIndices)
, mGRB_tetraSurfaceHint(d.mGRB_tetraSurfaceHint)
, mGRB_faceRemap(d.mGRB_faceRemap)
, mGRB_faceRemapInverse(d.mGRB_faceRemapInverse)
, mGRB_BV32Tree(d.mGRB_BV32Tree)
{
	mMeshFactory = factory;

	bool has16BitIndices = (mesh.mFlags & PxMeshFlag::e16_BIT_INDICES) ? true : false;

	mMeshInterface4.mVerts = mVertices; // mesh.mVertices;
	mMeshInterface4.mNbVerts = mesh.mNbVertices;
	mMeshInterface4.mNbTetrahedrons = mesh.mNbTetrahedrons;
	mMeshInterface4.mTetrahedrons16 = has16BitIndices ? reinterpret_cast<IndTetrahedron16*>(mTetrahedrons/*mesh.mTetrahedrons*/) : NULL;
	mMeshInterface4.mTetrahedrons32 = has16BitIndices ? NULL : reinterpret_cast<IndTetrahedron32*>(mTetrahedrons/*mesh.mTetrahedrons*/);

	mBV4Tree = d.mBV4Tree;
	mBV4Tree.mMeshInterface = &mMeshInterface4;

	if (mGRB_BV32Tree)
	{
		mMeshInterface32.mVerts = mVertices; //  mesh.mVertices;
		mMeshInterface32.mNbVerts = mesh.mNbVertices;
		mMeshInterface32.mNbTetrahedrons = mesh.mNbTetrahedrons;
		mMeshInterface32.mTetrahedrons16 = has16BitIndices ? reinterpret_cast<IndTetrahedron16*>(d.mGRB_primIndices) : NULL;
		mMeshInterface32.mTetrahedrons32 = has16BitIndices ? NULL : reinterpret_cast<IndTetrahedron32*>(d.mGRB_primIndices);

		mGRB_BV32Tree->mMeshInterface = &mMeshInterface32;
	}

	// this constructor takes ownership of memory from the data object
	d.mGRB_tetraSurfaceHint = NULL;
	d.mFaceRemap = NULL;

	d.mGRB_primIndices = NULL;
	d.mGRB_faceRemap = NULL;
	d.mGRB_faceRemapInverse = NULL;
	d.mGRB_BV32Tree = NULL;

	mesh.mVertices = NULL;
	mesh.mTetrahedrons = NULL;
}

SoftBodyMesh::SoftBodyMesh(MeshFactory* factory, SoftBodyMeshData& d)
	: PxSoftBodyMesh(PxType(PxConcreteType::eSOFTBODY_MESH), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
	, mMeshFactory(factory)
{
	mSoftBodyAuxData = PX_NEW(SoftBodyAuxData)(d.mSimulationData, d.mCollisionData, d.mMappingData);
	mCollisionMesh = PX_NEW(BVTetrahedronMesh)(d.mCollisionMesh, d.mCollisionData, factory);
	mSimulationMesh = PX_NEW(TetrahedronMesh)(factory, d.mSimulationMesh);
}

SoftBodyMesh::~SoftBodyMesh()
{	
	if (getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		PX_DELETE(mSoftBodyAuxData);
		PX_DELETE(mCollisionMesh);
		PX_DELETE(mSimulationMesh);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: used to be automatic but making it manual saves bytes in the internal mesh

void SoftBodyMesh::exportExtraData(PxSerializationContext& stream)
{
	//PX_DEFINE_DYNAMIC_ARRAY(TriangleMesh, mVertices, PxField::eVEC3, mNbVertices, Ps::PxFieldFlag::eSERIALIZE),
	if (getCollisionMeshFast()->mVertices)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(getCollisionMeshFast()->mVertices, getCollisionMeshFast()->mNbVertices * sizeof(PxVec3));
	}

	/*if (mSurfaceTriangles)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mSurfaceTriangles, mNbTriangles * 3 * sizeof(PxU32));
	}*/

	if (getCollisionMeshFast()->mTetrahedrons)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(getCollisionMeshFast()->mTetrahedrons, getCollisionMeshFast()->mNbTetrahedrons * 4 * sizeof(PxU32));
	}

	/*if (mTetSurfaceHint)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mTetSurfaceHint, mNbTetrahedrons * sizeof(PxU8));
	}*/

	if (getCollisionMeshFast()->mMaterialIndices)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(getCollisionMeshFast()->mMaterialIndices, getCollisionMeshFast()->mNbTetrahedrons * sizeof(PxU16));
	}

	if (getCollisionMeshFast()->mFaceRemap)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(getCollisionMeshFast()->mFaceRemap, getCollisionMeshFast()->mNbTetrahedrons * sizeof(PxU32));
	}
}

void SoftBodyMesh::importExtraData(PxDeserializationContext& context)
{
	// PT: vertices are followed by indices, so it will be safe to V4Load vertices from a deserialized binary file
	if (getCollisionMeshFast()->mVertices)
		getCollisionMeshFast()->mVertices = context.readExtraData<PxVec3, PX_SERIAL_ALIGN>(getCollisionMeshFast()->mNbVertices);

	if (getCollisionMeshFast()->mTetrahedrons)
		getCollisionMeshFast()->mTetrahedrons = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(4 * getCollisionMeshFast()->mNbTetrahedrons);

	if (getCollisionMeshFast()->mFaceRemap)
		getCollisionMeshFast()->mFaceRemap = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(getCollisionMeshFast()->mNbTetrahedrons);
		
	getCollisionMeshFast()->mGRB_tetraIndices = NULL;
	getCollisionMeshFast()->mGRB_tetraSurfaceHint = NULL;
	getCollisionMeshFast()->mGRB_faceRemap = NULL;
	getCollisionMeshFast()->mGRB_faceRemapInverse = NULL;
	getCollisionMeshFast()->mGRB_BV32Tree = NULL;
}

void SoftBodyMesh::release()
{
	Cm::RefCountable_decRefCount(*this);
}

//PxVec3* TetrahedronMesh::getVerticesForModification()
//{
//	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxSoftBodyMesh::getVerticesForModification() is not currently supported.");
//
//	return NULL;
//}

//PxBounds3 BVTetrahedronMesh::refitBVH()
//{
//	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxSoftBodyMesh::refitBVH() is not currently supported.");
//
//	return PxBounds3(mAABB.getMin(), mAABB.getMax());
//}
