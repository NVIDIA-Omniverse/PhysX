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

#include "GuHeightField.h"
#include "GuMeshFactory.h"
#include "CmSerialize.h"
#include "foundation/PxBitMap.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HeightField::HeightField(MeshFactory* factory)
: PxHeightField(PxConcreteType::eHEIGHTFIELD, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
, mSampleStride	(0)
, mNbSamples	(0)
, mMinHeight	(0.0f)
, mMaxHeight	(0.0f)
, mModifyCount	(0)
, mMeshFactory	(factory)
{
	mData.format				= PxHeightFieldFormat::eS16_TM;
	mData.rows					= 0;
	mData.columns				= 0;
	mData.convexEdgeThreshold	= 0;
	mData.flags					= PxHeightFieldFlags();
	mData.samples				= NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HeightField::HeightField(MeshFactory* factory, HeightFieldData& data)
: PxHeightField(PxConcreteType::eHEIGHTFIELD, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
, mSampleStride	(0)
, mNbSamples	(0)
, mMinHeight	(0.0f)
, mMaxHeight	(0.0f)
, mModifyCount	(0)
, mMeshFactory	(factory)
{
	mData = data;
	data.samples = NULL; // set to null so that we don't release the memory
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HeightField::~HeightField()
{
	releaseMemory();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HeightField::onRefCountZero()
{
	::onRefCountZero(this, mMeshFactory, false, "PxHeightField::release: double deletion detected!");
}

void HeightField::exportExtraData(PxSerializationContext& stream)
{
	// PT: warning, order matters for the converter. Needs to export the base stuff first
	const PxU32 size = mData.rows * mData.columns * sizeof(PxHeightFieldSample);
	stream.alignData(PX_SERIAL_ALIGN);	// PT: generic align within the generic allocator
	stream.writeData(mData.samples, size);
}

void HeightField::importExtraData(PxDeserializationContext& context)
{
	mData.samples = context.readExtraData<PxHeightFieldSample, PX_SERIAL_ALIGN>(mData.rows * mData.columns);
}

HeightField* HeightField::createObject(PxU8*& address, PxDeserializationContext& context)
{
	HeightField* obj = PX_PLACEMENT_NEW(address, HeightField(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(HeightField);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void HeightField::release()
{
	RefCountable_decRefCount(*this);
}

void HeightField::acquireReference()
{
	RefCountable_incRefCount(*this);
}

PxU32 HeightField::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool HeightField::modifySamples(PxI32 startCol, PxI32 startRow, const PxHeightFieldDesc& desc, bool shrinkBounds)
{
	const PxU32 nbCols = getNbColumns();
	const PxU32 nbRows = getNbRows();
	PX_CHECK_AND_RETURN_NULL(desc.format == mData.format, "Gu::HeightField::modifySamples: desc.format mismatch");
	//PX_CHECK_AND_RETURN_NULL(startCol + desc.nbColumns <= nbCols,
	//	"Gu::HeightField::modifySamples: startCol + nbColumns out of range");
	//PX_CHECK_AND_RETURN_NULL(startRow + desc.nbRows <= nbRows,
	//	"Gu::HeightField::modifySamples: startRow + nbRows out of range");
	//PX_CHECK_AND_RETURN_NULL(desc.samples.stride == mSampleStride, "Gu::HeightField::modifySamples: desc.samples.stride mismatch");

	// by default bounds don't shrink since the whole point of this function is to avoid modifying the whole HF
	// unless shrinkBounds is specified. then the bounds will be fully recomputed later
	PxReal minHeight = mMinHeight;
	PxReal maxHeight = mMaxHeight;
	PxU32 hiRow = PxMin(PxU32(PxMax(0, startRow + PxI32(desc.nbRows))), nbRows);
	PxU32 hiCol = PxMin(PxU32(PxMax(0, startCol + PxI32(desc.nbColumns))), nbCols);
	for (PxU32 row = PxU32(PxMax(startRow, 0)); row < hiRow; row++)
	{
		for (PxU32 col = PxU32(PxMax(startCol, 0)); col < hiCol; col++)
		{
			const PxU32 vertexIndex = col + row*nbCols;
			PxHeightFieldSample* targetSample = &mData.samples[vertexIndex];

			// update target sample from source sample
			const PxHeightFieldSample& sourceSample =
				(reinterpret_cast<const PxHeightFieldSample*>(desc.samples.data))[col - startCol + (row - startRow) * desc.nbColumns];
			*targetSample = sourceSample;

			if(isCollisionVertexPreca(vertexIndex, row, col, PxHeightFieldMaterial::eHOLE))
				targetSample->materialIndex1.setBit();
			else
				targetSample->materialIndex1.clearBit();

			// grow (but not shrink) the height extents
			const PxReal h = getHeight(vertexIndex);
			minHeight = physx::intrinsics::selectMin(h, minHeight);
			maxHeight = physx::intrinsics::selectMax(h, maxHeight);
		}
	}

	if (shrinkBounds)
	{
		// do a full recompute on vertical bounds to allow shrinking
		minHeight = PX_MAX_REAL;
		maxHeight = -PX_MAX_REAL;
		// have to recompute the min&max from scratch...
		for (PxU32 vertexIndex = 0; vertexIndex < nbRows * nbCols; vertexIndex ++)
		{
				// update height extents
				const PxReal h = getHeight(vertexIndex);
				minHeight = physx::intrinsics::selectMin(h, minHeight);
				maxHeight = physx::intrinsics::selectMax(h, maxHeight);
		}
	}
	mMinHeight = minHeight;
	mMaxHeight = maxHeight;

	// update local space aabb
	CenterExtents& bounds = mData.mAABB;
	bounds.mCenter.y = (maxHeight + minHeight)*0.5f;
	bounds.mExtents.y = (maxHeight - minHeight)*0.5f;

	mModifyCount++;

	return true;
}

bool HeightField::load(PxInputStream& stream)
{
	// release old memory
	releaseMemory();

	// Import header
	PxU32 version;
	bool endian;
	if(!readHeader('H', 'F', 'H', 'F', version, endian, stream))
		return false;

	// load mData
	mData.rows = readDword(endian, stream);
	mData.columns = readDword(endian, stream);
	if(version>=2)
	{
		mData.rowLimit = readDword(endian, stream);
		mData.colLimit = readDword(endian, stream);
		mData.nbColumns = readDword(endian, stream);
	}
	else
	{
		mData.rowLimit = PxU32(readFloat(endian, stream));
		mData.colLimit = PxU32(readFloat(endian, stream));
		mData.nbColumns = PxU32(readFloat(endian, stream));
	}
	const float thickness = readFloat(endian, stream);
	PX_UNUSED(thickness);
	mData.convexEdgeThreshold = readFloat(endian, stream);

	PxU16 flags = readWord(endian, stream);
	mData.flags = PxHeightFieldFlags(flags);

	PxU32 format = readDword(endian, stream);
	mData.format = PxHeightFieldFormat::Enum(format);

	PxBounds3 minMaxBounds;
	minMaxBounds.minimum.x = readFloat(endian, stream);
	minMaxBounds.minimum.y = readFloat(endian, stream);
	minMaxBounds.minimum.z = readFloat(endian, stream);
	minMaxBounds.maximum.x = readFloat(endian, stream);
	minMaxBounds.maximum.y = readFloat(endian, stream);
	minMaxBounds.maximum.z = readFloat(endian, stream);
	mData.mAABB = CenterExtents(minMaxBounds);

	mSampleStride = readDword(endian, stream);
	mNbSamples = readDword(endian, stream);
	mMinHeight = readFloat(endian, stream);
	mMaxHeight = readFloat(endian, stream);

	// allocate height samples
	mData.samples = NULL;
	const PxU32 nbVerts = mData.rows * mData.columns;
	if (nbVerts > 0) 
	{
		mData.samples = PX_ALLOCATE(PxHeightFieldSample, nbVerts, "PxHeightFieldSample");
		if (mData.samples == NULL)
			return PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "Gu::HeightField::load: PX_ALLOC failed!");

		stream.read(mData.samples, mNbSamples*sizeof(PxHeightFieldSample));
		if (endian)
			for(PxU32 i = 0; i < mNbSamples; i++)
			{
				PxHeightFieldSample& s = mData.samples[i];
				PX_ASSERT(sizeof(PxU16) == sizeof(s.height));
				flip(s.height);
			}
	}

	return true;
}

bool HeightField::loadFromDesc(const PxHeightFieldDesc& desc)
{
	// verify descriptor	
	PX_CHECK_AND_RETURN_NULL(desc.isValid(), "Gu::HeightField::loadFromDesc: desc.isValid() failed!");
	
	// release old memory
	releaseMemory();

	// copy trivial data
	mData.format				= desc.format;
	mData.rows					= desc.nbRows;
	mData.columns				= desc.nbColumns;
	mData.convexEdgeThreshold	= desc.convexEdgeThreshold;
	mData.flags					= desc.flags;
	mSampleStride				= desc.samples.stride;

	mData.rowLimit				= mData.rows - 2;
	mData.colLimit				= mData.columns - 2;
	mData.nbColumns				= desc.nbColumns;

	// allocate and copy height samples
	// compute extents too
	mData.samples = NULL;
	const PxU32 nbVerts = desc.nbRows * desc.nbColumns;
	mMinHeight = PX_MAX_REAL;
	mMaxHeight = -PX_MAX_REAL;

	if(nbVerts > 0) 
	{
		mData.samples = PX_ALLOCATE(PxHeightFieldSample, nbVerts, "PxHeightFieldSample");
		if(!mData.samples)
			return PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "Gu::HeightField::load: PX_ALLOC failed!");

		const PxU8* PX_RESTRICT src = reinterpret_cast<const PxU8*>(desc.samples.data);
		PxHeightFieldSample* PX_RESTRICT dst = mData.samples;
		PxI16 minHeight = PX_MAX_I16;
		PxI16 maxHeight = PX_MIN_I16;
		for(PxU32 i=0;i<nbVerts;i++)
		{			
			const PxHeightFieldSample& sample = *reinterpret_cast<const PxHeightFieldSample*>(src);
			*dst++ = sample;
			const PxI16 height = sample.height;
			minHeight = height < minHeight ? height : minHeight;
			maxHeight = height > maxHeight ? height : maxHeight;
			src += desc.samples.stride;	
		}
		mMinHeight = PxReal(minHeight);
		mMaxHeight = PxReal(maxHeight);
	}

	PX_ASSERT(mMaxHeight >= mMinHeight);

	parseTrianglesForCollisionVertices(PxHeightFieldMaterial::eHOLE);

// PT: "mNbSamples" only used by binary converter
	mNbSamples	= mData.rows * mData.columns;

	//Compute local space aabb.
	PxBounds3 bounds;
	bounds.minimum.y = getMinHeight();
	bounds.maximum.y = getMaxHeight();

	bounds.minimum.x = 0;
	bounds.maximum.x = PxReal(getNbRowsFast() - 1);
	bounds.minimum.z = 0;
	bounds.maximum.z = PxReal(getNbColumnsFast() - 1);
	mData.mAABB=bounds;

	return true;
}

bool HeightField::save(PxOutputStream& stream, bool endian)
{
	// write header
	if(!writeHeader('H', 'F', 'H', 'F', PX_HEIGHTFIELD_VERSION, endian, stream))
		return false;

	const Gu::HeightFieldData& hfData = getData();

	// write mData members
	writeDword(hfData.rows, endian, stream);
	writeDword(hfData.columns, endian, stream);
	writeDword(hfData.rowLimit, endian, stream);
	writeDword(hfData.colLimit, endian, stream);
	writeDword(hfData.nbColumns, endian, stream);
	writeFloat(0.0f, endian, stream);	// thickness
	writeFloat(hfData.convexEdgeThreshold, endian, stream);
	writeWord(hfData.flags, endian, stream);
	writeDword(hfData.format, endian, stream);

	writeFloat(hfData.mAABB.getMin(0), endian, stream);
	writeFloat(hfData.mAABB.getMin(1), endian, stream);
	writeFloat(hfData.mAABB.getMin(2), endian, stream);
	writeFloat(hfData.mAABB.getMax(0), endian, stream);
	writeFloat(hfData.mAABB.getMax(1), endian, stream);
	writeFloat(hfData.mAABB.getMax(2), endian, stream);

	// write this-> members
	writeDword(mSampleStride, endian, stream);
	writeDword(mNbSamples, endian, stream);
	writeFloat(mMinHeight, endian, stream);
	writeFloat(mMaxHeight, endian, stream);

	// write samples
	for(PxU32 i=0; i<mNbSamples; i++)
	{
		const PxHeightFieldSample& s = hfData.samples[i];
		writeWord(PxU16(s.height), endian, stream);
		stream.write(&s.materialIndex0, sizeof(s.materialIndex0));
		stream.write(&s.materialIndex1, sizeof(s.materialIndex1));
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxU32 HeightField::saveCells(void* destBuffer, PxU32 destBufferSize) const
{
	PxU32 n = mData.columns * mData.rows * sizeof(PxHeightFieldSample);
	if (n > destBufferSize) n = destBufferSize;
	PxMemCopy(destBuffer, mData.samples, n);

	return n;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HeightField::releaseMemory()
{
	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		PX_FREE(mData.samples);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace
{
	struct EdgeData
	{
		PxU32	edgeIndex;
		PxU32	cell;
		PxU32	row;
		PxU32	column;
	};
}

// PT: TODO: use those faster functions everywhere
static PxU32 getVertexEdgeIndices(const HeightField& heightfield, PxU32 vertexIndex, PxU32 row, PxU32 column, EdgeData edgeIndices[8])
{
	const PxU32 nbColumns = heightfield.getData().columns;
	const PxU32 nbRows = heightfield.getData().rows;
	PX_ASSERT((vertexIndex / nbColumns)==row);
	PX_ASSERT((vertexIndex % nbColumns)==column);

	PxU32 count = 0;
	
	if (row > 0) 
	{
//		edgeIndices[count++] = 3 * (vertexIndex - nbColumns) + 2;
		const PxU32 cell = vertexIndex - nbColumns;
		edgeIndices[count].edgeIndex	= 3 * cell + 2;
		edgeIndices[count].cell			= cell;
		edgeIndices[count].row			= row-1;
		edgeIndices[count].column		= column;
		count++;
	}
	
	if (column < nbColumns-1)
	{
		if (row > 0)
		{
			if (!heightfield.isZerothVertexShared(vertexIndex - nbColumns))
			{
//				edgeIndices[count++] = 3 * (vertexIndex - nbColumns) + 1;
				const PxU32 cell = vertexIndex - nbColumns;
				edgeIndices[count].edgeIndex	= 3 * cell + 1;
				edgeIndices[count].cell			= cell;
				edgeIndices[count].row			= row-1;
				edgeIndices[count].column		= column;
				count++;
			}
		}
//		edgeIndices[count++] = 3 * vertexIndex;
		edgeIndices[count].edgeIndex	= 3 * vertexIndex;
		edgeIndices[count].cell			= vertexIndex;
		edgeIndices[count].row			= row;
		edgeIndices[count].column		= column;
		count++;

		if (row < nbRows - 1)
		{
			if (heightfield.isZerothVertexShared(vertexIndex))
			{
//				edgeIndices[count++] = 3 * vertexIndex + 1;
				edgeIndices[count].edgeIndex	= 3 * vertexIndex + 1;
				edgeIndices[count].cell			= vertexIndex;
				edgeIndices[count].row			= row;
				edgeIndices[count].column		= column;
				count++;
			}
		}
	}

	if (row < nbRows - 1)
	{
//		edgeIndices[count++] = 3 * vertexIndex + 2;
		edgeIndices[count].edgeIndex	= 3 * vertexIndex + 2;
		edgeIndices[count].cell			= vertexIndex;
		edgeIndices[count].row			= row;
		edgeIndices[count].column		= column;
		count++;
	}

	if (column > 0)
	{
		if (row < nbRows - 1)
		{
			if (!heightfield.isZerothVertexShared(vertexIndex - 1))
			{
//				edgeIndices[count++] = 3 * (vertexIndex - 1) + 1;
				const PxU32 cell = vertexIndex - 1;
				edgeIndices[count].edgeIndex	= 3 * cell + 1;
				edgeIndices[count].cell			= cell;
				edgeIndices[count].row			= row;
				edgeIndices[count].column		= column-1;
				count++;
			}
		}
//		edgeIndices[count++] = 3 * (vertexIndex - 1);
		const PxU32 cell = vertexIndex - 1;
		edgeIndices[count].edgeIndex	= 3 * cell;
		edgeIndices[count].cell			= cell;
		edgeIndices[count].row			= row;
		edgeIndices[count].column		= column-1;
		count++;
		if (row > 0)
		{
			if (heightfield.isZerothVertexShared(vertexIndex - nbColumns - 1))
			{
//				edgeIndices[count++] = 3 * (vertexIndex - nbColumns - 1) + 1;
				const PxU32 cell1 = vertexIndex - nbColumns - 1;
				edgeIndices[count].edgeIndex	= 3 * cell1 + 1;
				edgeIndices[count].cell			= cell1;
				edgeIndices[count].row			= row-1;
				edgeIndices[count].column		= column-1;
				count++;
			}
		}
	}
	return count;
}

static PxU32 getEdgeTriangleIndices(const HeightField& heightfield, const EdgeData& edgeData, PxU32* PX_RESTRICT triangleIndices)
{
	const PxU32 nbColumns = heightfield.getData().columns;
	const PxU32 nbRows = heightfield.getData().rows;

	const PxU32 edgeIndex	= edgeData.edgeIndex;
	const PxU32 cell		= edgeData.cell;
	const PxU32 row			= edgeData.row;
	const PxU32 column		= edgeData.column;
	PX_ASSERT(cell==edgeIndex / 3);
	PX_ASSERT(row==cell / nbColumns);
	PX_ASSERT(column==cell % nbColumns);
	PxU32 count = 0;
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (column < nbColumns - 1)
			{
				if (row > 0)
				{
					if (heightfield.isZerothVertexShared(cell - nbColumns))
						triangleIndices[count++] = ((cell - nbColumns) << 1);
					else 
						triangleIndices[count++] = ((cell - nbColumns) << 1) + 1;
				}
				if (row < nbRows - 1)
				{
					if (heightfield.isZerothVertexShared(cell))
						triangleIndices[count++] = (cell << 1) + 1;
					else 
						triangleIndices[count++] = cell << 1;
				}
			}
			break;
		case 1:
			if ((row < nbRows - 1) && (column < nbColumns - 1))
			{
				triangleIndices[count++] = cell << 1;
				triangleIndices[count++] = (cell << 1) + 1;
			}
			break;
		case 2:
			if (row < nbRows - 1)
			{
				if (column > 0)
				{
					triangleIndices[count++] = ((cell - 1) << 1) + 1;
				}
				if (column < nbColumns - 1)
				{
					triangleIndices[count++] = cell << 1;
				}
			}
			break;
	}

	return count;
}

PX_FORCE_INLINE PxU32 anyHole(PxU32 doubleMatIndex, PxU16 holeMaterialIndex)
{
	return PxU32((doubleMatIndex & 0xFFFF) == holeMaterialIndex) | (PxU32(doubleMatIndex >> 16) == holeMaterialIndex);
}

void HeightField::parseTrianglesForCollisionVertices(PxU16 holeMaterialIndex)
{	
	const PxU32 nbColumns = getNbColumnsFast();
	const PxU32 nbRows = getNbRowsFast();

	PxBitMap rowHoles[2];
	rowHoles[0].resizeAndClear(nbColumns + 1);
	rowHoles[1].resizeAndClear(nbColumns + 1);

	for (PxU32 iCol = 0; iCol < nbColumns; iCol++)
	{
		if (anyHole(getMaterialIndex01(iCol), holeMaterialIndex))
		{
			rowHoles[0].set(iCol);
			rowHoles[0].set(iCol + 1);
		}
		PxU32 vertIndex = iCol;
		if(isCollisionVertexPreca(vertIndex, 0, iCol, holeMaterialIndex))
			mData.samples[vertIndex].materialIndex1.setBit();
		else
			mData.samples[vertIndex].materialIndex1.clearBit();
	}

	PxU32 nextRow = 1, currentRow = 0;
	for (PxU32 iRow = 1; iRow < nbRows; iRow++)
	{
		PxU32 rowOffset = iRow*nbColumns;
		for (PxU32 iCol = 0; iCol < nbColumns; iCol++)
		{
			const PxU32 vertIndex = rowOffset + iCol; // column index plus current row offset (vertex/cell index)
			if(anyHole(getMaterialIndex01(vertIndex), holeMaterialIndex))
			{
				rowHoles[currentRow].set(iCol);
				rowHoles[currentRow].set(iCol + 1);
				rowHoles[nextRow].set(iCol);
				rowHoles[nextRow].set(iCol + 1);
			}

			if ((iCol == 0) || (iCol == nbColumns - 1) || (iRow == nbRows - 1) || rowHoles[currentRow].test(iCol))
			{
				if(isCollisionVertexPreca(vertIndex, iRow, iCol, holeMaterialIndex))
					mData.samples[vertIndex].materialIndex1.setBit();
				else
					mData.samples[vertIndex].materialIndex1.clearBit();
			} else
			{
				if (isConvexVertex(vertIndex, iRow, iCol))
					mData.samples[vertIndex].materialIndex1.setBit();
			}
		}

		rowHoles[currentRow].clear();

		// swap prevRow and prevPrevRow
		nextRow ^= 1; currentRow ^= 1;
	}
}

bool HeightField::isSolidVertex(PxU32 vertexIndex, PxU32 row, PxU32 column, PxU16 holeMaterialIndex, bool& nbSolid) const
{
	// check if solid and boundary
	// retrieve edge indices for current vertexIndex
	EdgeData edgeIndices[8];
	const PxU32 edgeCount = ::getVertexEdgeIndices(*this, vertexIndex, row, column, edgeIndices);

	PxU32 faceCounts[8];
	PxU32 faceIndices[2 * 8];
	PxU32* dst = faceIndices;
	for (PxU32 i = 0; i < edgeCount; i++)
	{
		faceCounts[i] = ::getEdgeTriangleIndices(*this, edgeIndices[i], dst);
		dst += 2;
	}
	
	nbSolid = false;
	const PxU32* currentfaceIndices = faceIndices; // parallel array of pairs of face indices per edge index
	for (PxU32 i = 0; i < edgeCount; i++)
	{
		if (faceCounts[i] > 1)
		{
			const PxU16& material0 = getTriangleMaterial(currentfaceIndices[0]);
			const PxU16& material1 = getTriangleMaterial(currentfaceIndices[1]);
			// ptchernev TODO: this is a bit arbitrary
			if (material0 != holeMaterialIndex)
			{
				nbSolid = true;
				if (material1 == holeMaterialIndex)
					return true; // edge between solid and hole => return true
			}
			if (material1 != holeMaterialIndex)
			{
				nbSolid = true;
				if (material0 == holeMaterialIndex)
					return true; // edge between hole and solid => return true
			}
		}
		else
		{
			if (getTriangleMaterial(currentfaceIndices[0]) != holeMaterialIndex)
				return true;
		}
		currentfaceIndices += 2; // 2 face indices per edge
	}
	return false;
}

bool HeightField::isCollisionVertexPreca(PxU32 vertexIndex, PxU32 row, PxU32 column, PxU16 holeMaterialIndex) const
{
#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(isValidVertex(vertexIndex));
#endif
	PX_ASSERT((vertexIndex / getNbColumnsFast()) == row);
	PX_ASSERT((vertexIndex % getNbColumnsFast()) == column);

	// check boundary conditions - boundary edges shouldn't produce collision with eNO_BOUNDARY_EDGES flag
	if(mData.flags & PxHeightFieldFlag::eNO_BOUNDARY_EDGES) 
		if ((row == 0) || (column == 0) || (row >= mData.rows-1) || (column >= mData.columns-1))
			return false;

	bool nbSolid;
	if(isSolidVertex(vertexIndex, row, column, holeMaterialIndex, nbSolid))
		return true;

	// return true if it is boundary or solid and convex
	return (nbSolid && isConvexVertex(vertexIndex, row, column));
}

// AP: this naming is confusing and inconsistent with return value. the function appears to compute vertex coord rather than cell coords
// it would most likely be better to stay in cell coords instead, since fractional vertex coords just do not make any sense
PxU32 HeightField::computeCellCoordinates(PxReal x, PxReal z, PxReal& fracX, PxReal& fracZ) const
{
	namespace i = physx::intrinsics;

	x = i::selectMax(x, 0.0f);
	z = i::selectMax(z, 0.0f);
#if 0 // validation code for scaled clamping epsilon computation
	for (PxReal ii = 1.0f; ii < 100000.0f; ii+=1.0f)
	{
		PX_UNUSED(ii);
		PX_ASSERT(PxFloor(ii+(1-1e-7f*ii)) == ii);
	}
#endif
	const PxF32 epsx = 1.0f - PxAbs(x+1.0f) * 1e-6f; // epsilon needs to scale with values of x,z...
	const PxF32 epsz = 1.0f - PxAbs(z+1.0f) * 1e-6f;
	PxF32 x1 = i::selectMin(x, float(mData.rowLimit)+epsx);
	PxF32 z1 = i::selectMin(z, float(mData.colLimit)+epsz);
	x = PxFloor(x1);
	fracX = x1 - x;
	z = PxFloor(z1);
	fracZ = z1 - z;
	PX_ASSERT(x >= 0.0f && x < PxF32(mData.rows));
	PX_ASSERT(z >= 0.0f && z < PxF32(mData.columns));

	const PxU32 vertexIndex = PxU32(x) * mData.nbColumns + PxU32(z);
	PX_ASSERT(vertexIndex < mData.rows*mData.columns);

	return vertexIndex;
}

