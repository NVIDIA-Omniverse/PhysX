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

#include "NpPruningStructure.h"
#include "GuAABBTree.h"
#include "GuAABBTreeNode.h"
#include "NpBounds.h"

#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpShape.h"

#include "GuBounds.h"

#include "CmTransformUtils.h"
#include "CmUtils.h"

#include "SqPrunerData.h"

using namespace physx;
using namespace Sq;
using namespace Gu;

//////////////////////////////////////////////////////////////////////////

#define PS_NB_OBJECTS_PER_NODE	4

//////////////////////////////////////////////////////////////////////////
PruningStructure::PruningStructure(PxBaseFlags baseFlags)
	: PxPruningStructure(baseFlags)
{
}

//////////////////////////////////////////////////////////////////////////
PruningStructure::PruningStructure()
	: PxPruningStructure(PxConcreteType::ePRUNING_STRUCTURE, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mNbActors(0), mActors(0), mValid(true)
{
	for(PxU32 i=0; i<2; i++)
		mData[i].init();
}

//////////////////////////////////////////////////////////////////////////
PruningStructure::~PruningStructure()
{	
	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		for(PxU32 i=0; i<2; i++)
		{
			PX_FREE(mData[i].mAABBTreeIndices);
			PX_FREE(mData[i].mAABBTreeNodes);
		}

		PX_FREE(mActors);
	}
}

//////////////////////////////////////////////////////////////////////////
void PruningStructure::release()
{
	// if we release the pruning structure we set the pruner structure to NUUL
	for (PxU32 i = 0; i < mNbActors; i++)
	{		
		PX_ASSERT(mActors[i]);			

		PxType type = mActors[i]->getConcreteType();
		if (type == PxConcreteType::eRIGID_STATIC)
			static_cast<NpRigidStatic*>(mActors[i])->getShapeManager().setPruningStructure(NULL);
		else if (type == PxConcreteType::eRIGID_DYNAMIC)
			static_cast<NpRigidDynamic*>(mActors[i])->getShapeManager().setPruningStructure(NULL);
	}

	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		PX_DELETE_THIS;
	else
		this->~PruningStructure();
}

template <typename ActorType>
static void getShapeBounds(PxRigidActor* actor, bool dynamic, PxBounds3* bounds, PxU32& numShapes)
{
	PruningIndex::Enum treeStructure = dynamic ? PruningIndex::eDYNAMIC : PruningIndex::eSTATIC;
	ActorType& a = *static_cast<ActorType*>(actor);
	const PxU32 nbShapes = a.getNbShapes();
	for (PxU32 iShape = 0; iShape < nbShapes; iShape++)
	{
		NpShape* shape = a.getShapeManager().getShapes()[iShape];
		if (shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
		{
			(gComputeBoundsTable[treeStructure])(*bounds, *shape, a);
			bounds++;
			numShapes++;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
bool PruningStructure::build(PxRigidActor*const* actors, PxU32 nbActors)
{
	PX_ASSERT(actors);
	PX_ASSERT(nbActors > 0);
	
	PxU32 numShapes[2] = { 0, 0 };	
	// parse the actors first to get the shapes size
	for (PxU32 actorsDone = 0; actorsDone < nbActors; actorsDone++)
	{
		if (actorsDone + 1 < nbActors)
			PxPrefetch(actors[actorsDone + 1], sizeof(NpRigidDynamic));	// worst case: PxRigidStatic is smaller

		PxType type = actors[actorsDone]->getConcreteType();
		const PxRigidActor& actor = *(actors[actorsDone]);

		NpScene* scene = NpActor::getFromPxActor(actor).getNpScene();
		if(scene)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PrunerStructure::build: Actor already assigned to a scene!");
			return false;
		}

		const PxU32 nbShapes = actor.getNbShapes();
		bool hasQueryShape = false;
		for (PxU32 iShape = 0; iShape < nbShapes; iShape++)
		{
			PxShape* shape;
			actor.getShapes(&shape, 1, iShape);
			if(shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
			{
				hasQueryShape = true;
				if (type == PxConcreteType::eRIGID_STATIC)
					numShapes[PruningIndex::eSTATIC]++;
				else
					numShapes[PruningIndex::eDYNAMIC]++;
			}
		}

		// each provided actor must have a query shape
		if(!hasQueryShape)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PrunerStructure::build: Provided actor has no scene query shape!");
			return false;
		}

		if (type == PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* rs = static_cast<NpRigidStatic*>(actors[actorsDone]);
			if(rs->getShapeManager().getPruningStructure())
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PrunerStructure::build: Provided actor has already a pruning structure!");
				return false;
			}			
			rs->getShapeManager().setPruningStructure(this);
		}
		else if (type == PxConcreteType::eRIGID_DYNAMIC)
		{
			NpRigidDynamic* rd = static_cast<NpRigidDynamic*>(actors[actorsDone]);			
			if (rd->getShapeManager().getPruningStructure())
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PrunerStructure::build: Provided actor has already a pruning structure!");
				return false;
			}
			rd->getShapeManager().setPruningStructure(this);
		}
		else 
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PrunerStructure::build: Provided actor is not a rigid actor!");
			return false;
		}
	}
	
	AABBTreeBounds bounds[2];

	for (PxU32 i = 0; i < 2; i++)
	{
		if(numShapes[i])
		{
			bounds[i].init(numShapes[i]);
		}
	}

	// now I go again and gather bounds and payload
	numShapes[PruningIndex::eSTATIC] = 0;
	numShapes[PruningIndex::eDYNAMIC] = 0;
	for (PxU32 actorsDone = 0; actorsDone < nbActors; actorsDone++)
	{
		PxType type = actors[actorsDone]->getConcreteType();
		if (type == PxConcreteType::eRIGID_STATIC)
		{
			getShapeBounds<NpRigidStatic>(actors[actorsDone], false, &bounds[PruningIndex::eSTATIC].getBounds()[numShapes[PruningIndex::eSTATIC]], numShapes[PruningIndex::eSTATIC]);
		}
		else if (type == PxConcreteType::eRIGID_DYNAMIC)
		{
			getShapeBounds<NpRigidDynamic>(actors[actorsDone], true, &bounds[PruningIndex::eDYNAMIC].getBounds()[numShapes[PruningIndex::eDYNAMIC]], numShapes[PruningIndex::eDYNAMIC]);
		}
	}
	
	AABBTree aabbTrees[2];
	for (PxU32 i = 0; i < 2; i++)
	{
		mData[i].mNbObjects = numShapes[i];
		if (numShapes[i])
		{
			// create the AABB tree
			NodeAllocator nodeAllocator;
			bool status = aabbTrees[i].build(AABBTreeBuildParams(PS_NB_OBJECTS_PER_NODE, numShapes[i], &bounds[i]), nodeAllocator);

			PX_UNUSED(status);
			PX_ASSERT(status);

			// store the tree nodes
			mData[i].mNbNodes = aabbTrees[i].getNbNodes();
			mData[i].mAABBTreeNodes = PX_ALLOCATE(BVHNode, mData[i].mNbNodes, "BVHNode");
			PxMemCopy(mData[i].mAABBTreeNodes, aabbTrees[i].getNodes(), sizeof(BVHNode)*mData[i].mNbNodes);
			mData[i].mAABBTreeIndices = PX_ALLOCATE(PxU32, mData[i].mNbObjects, "PxU32");
			PxMemCopy(mData[i].mAABBTreeIndices, aabbTrees[i].getIndices(), sizeof(PxU32)*mData[i].mNbObjects);

			// discard the data
			bounds[i].release();
		}		
	}

	// store the actors for verification and serialization
	mNbActors = nbActors;
	mActors = PX_ALLOCATE(PxActor*, mNbActors, "PxActor*");
	PxMemCopy(mActors, actors, sizeof(PxActor*)*mNbActors);

	return true;
}

//////////////////////////////////////////////////////////////////////////

PruningStructure* PruningStructure::createObject(PxU8*& address, PxDeserializationContext& context)
{
	PruningStructure* obj = PX_PLACEMENT_NEW(address, PruningStructure(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(PruningStructure);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::resolveReferences(PxDeserializationContext& context)
{
	if (!isValid())
		return;

	for (PxU32 i = 0; i < mNbActors; i++)
		context.translatePxBase(mActors[i]);
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::requiresObjects(PxProcessPxBaseCallback& c)
{
	if (!isValid())		
		return;
	
	for (PxU32 i = 0; i < mNbActors; i++)
		c.process(*mActors[i]);
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::exportExtraData(PxSerializationContext& stream)
{
	if (!isValid())
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, PX_FL, "PrunerStructure::exportExtraData: Pruning structure is invalid!");
		return;
	}

	for (PxU32 i = 0; i < 2; i++)
	{
		if (mData[i].mAABBTreeNodes)
		{
			// store nodes
			stream.alignData(PX_SERIAL_ALIGN);
			stream.writeData(mData[i].mAABBTreeNodes, mData[i].mNbNodes * sizeof(BVHNode));
		}

		if(mData[i].mAABBTreeIndices)
		{
			// store indices
			stream.alignData(PX_SERIAL_ALIGN);
			stream.writeData(mData[i].mAABBTreeIndices, mData[i].mNbObjects * sizeof(PxU32));
		}
	}

	if(mActors)
	{
		// store actor pointers
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mActors, mNbActors * sizeof(PxActor*));
	}
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::importExtraData(PxDeserializationContext& context)
{
	if (!isValid())
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, PX_FL, "PrunerStructure::importExtraData: Pruning structure is invalid!");
		return;
	}

	for (PxU32 i = 0; i < 2; i++)
	{
		if (mData[i].mAABBTreeNodes)
			mData[i].mAABBTreeNodes = context.readExtraData<BVHNode, PX_SERIAL_ALIGN>(mData[i].mNbNodes);
		if(mData[i].mAABBTreeIndices)
			mData[i].mAABBTreeIndices = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(mData[i].mNbObjects);
	}

	if (mActors)
	{
		// read actor pointers
		mActors = context.readExtraData<PxActor*, PX_SERIAL_ALIGN>(mNbActors);
	}
}

//////////////////////////////////////////////////////////////////////////

PxU32 PruningStructure::getNbRigidActors()	const
{
	return mNbActors;
}

const void* PruningStructure::getStaticMergeData()	const
{
	return &mData[PruningIndex::eSTATIC];
}

const void* PruningStructure::getDynamicMergeData()		const
{
	return &mData[PruningIndex::eDYNAMIC];
}

PxU32 PruningStructure::getRigidActors(PxRigidActor** userBuffer, PxU32 bufferSize, PxU32 startIndex/* =0 */) const
{	
	if(!isValid())
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, PX_FL, "PrunerStructure::getRigidActors: Pruning structure is invalid!");
		return 0;
	}

	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mActors, mNbActors);
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::invalidate(PxActor* actor)
{
	PX_ASSERT(actor);

	// remove actor from the actor list to avoid mem corruption
	// this slow, but should be called only with error msg send to user about invalid behavior
	for (PxU32 i = 0; i < mNbActors; i++)
	{
		if(mActors[i] == actor)
		{
			// set pruning structure to NULL and remove the actor from the list
			PxType type = mActors[i]->getConcreteType();
			if (type == PxConcreteType::eRIGID_STATIC)
				static_cast<NpRigidStatic*>(mActors[i])->getShapeManager().setPruningStructure(NULL);
			else if (type == PxConcreteType::eRIGID_DYNAMIC)
				static_cast<NpRigidDynamic*>(mActors[i])->getShapeManager().setPruningStructure(NULL);

			mActors[i] = mActors[mNbActors--];
			break;
		}		
	}

	mValid = false;
}

