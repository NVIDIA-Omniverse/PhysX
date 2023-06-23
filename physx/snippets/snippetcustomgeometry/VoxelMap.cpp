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

#include "collision/PxCollisionDefs.h"
#include "PxImmediateMode.h"
#include "VoxelMap.h"
#include "common/PxRenderOutput.h"
#include "geomutils/PxContactBuffer.h"

using namespace physx;

void VoxelMap::setDimensions(int x, int y, int z)
{
	m_dimensions[0] = x; m_dimensions[1] = y; m_dimensions[2] = z;
	m_packs.resize(size_t(pacsX()) * size_t(pacsY()) * size_t(pacsZ()), 0U);
}

int VoxelMap::dimX() const
{
	return m_dimensions[0];
}

int VoxelMap::dimY() const
{
	return m_dimensions[1];
}

int VoxelMap::dimZ() const
{
	return m_dimensions[2];
}

int VoxelMap::pacsX() const
{
	return (dimX() + 3) / 4;
}

int VoxelMap::pacsY() const
{
	return (dimY() + 3) / 4;
}

int VoxelMap::pacsZ() const
{
	return (dimZ() + 3) / 4;
}

PxVec3 VoxelMap::extents() const
{
	return PxVec3(dimX() * voxelSizeX(), dimY() * voxelSizeY(), dimZ() * voxelSizeZ());
}

void VoxelMap::setVoxelSize(float x, float y, float z)
{
	m_voxelSize = PxVec3(x, y, z);
}

const PxVec3& VoxelMap::voxelSize() const
{
	return m_voxelSize;
}

float VoxelMap::voxelSizeX() const
{
	return m_voxelSize.x;
}

float VoxelMap::voxelSizeY() const
{
	return m_voxelSize.y;
}

float VoxelMap::voxelSizeZ() const
{
	return m_voxelSize.z;
}

void VoxelMap::setVoxel(int x, int y, int z, bool yes)
{
	if (x < 0 || x >= int(dimX()) || y < 0 || y >= int(dimY()) || z < 0 || z >= int(dimZ()))
		return;

	int px = x / 4, py = y / 4, pz = z / 4;
	int bx = x & 3, by = y & 3, bz = z & 3;
	PxU64& p = m_packs[px + py * size_t(pacsX()) + pz * size_t(pacsX()) * size_t(pacsY())];
	if (yes) p |= (PxU64(1) << (bx + by * 4 + bz * 16));
	else p &= ~(PxU64(1) << (bx + by * 4 + bz * 16));
}

bool VoxelMap::voxel(int x, int y, int z) const
{
	if (x < 0 || x >= int(dimX()) || y < 0 || y >= int(dimY()) || z < 0 || z >= int(dimZ()))
		return false;

	int px = x / 4, py = y / 4, pz = z / 4;
	int bx = x & 3, by = y & 3, bz = z & 3;
	PxU64 p = m_packs[px + py * size_t(pacsX()) + pz * size_t(pacsX()) * size_t(pacsY())];
	return (p & (PxU64(1) << (bx + by * 4 + bz * 16))) != 0;
}

void VoxelMap::clearVoxels()
{
	memset(&m_packs[0], 0, m_packs.size() * sizeof(PxU64));
}

void VoxelMap::setFloorVoxels(int layers)
{
	for (int x = 0; x < dimX(); ++x)
		for (int y = 0; y < layers; ++y)
			for (int z = 0; z < dimZ(); ++z)
				setVoxel(x, y, z);
}

void VoxelMap::setWaveVoxels()
{
	PxVec3 ext = extents();
	for (int x = 0; x < dimX(); ++x)
		for (int y = 0; y < dimY(); ++y)
			for (int z = 0; z < dimZ(); ++z)
			{
				PxVec3 pos = voxelPos(x, y, z);
				float a = sqrtf((pos.x / ext.x) * (pos.x / ext.x) + (pos.z / ext.z) * (pos.z / ext.z));
				if (a * 0.5f > pos.y / ext.y + 0.4f)
					setVoxel(x, y, z);
			}
}

void VoxelMap::voxelize(const PxGeometry& geom, const PxTransform& pose, bool add)
{
	PxBounds3 bounds; PxGeometryQuery::computeGeomBounds(bounds, geom, pose);
	int sx, sy, sz, ex, ey, ez;
	getVoxelRegion(bounds, sx, sy, sz, ex, ey, ez);
	for (int x = sx; x <= ex; ++x)
		for (int y = sy; y <= ey; ++y)
			for (int z = sz; z <= ez; ++z)
				if (voxel(x, y, z))
				{
					if (!add && PxGeometryQuery::pointDistance(voxelPos(x, y, z), geom, pose) == 0)
						setVoxel(x, y, z, false);
				}
				else
				{
					if (add && PxGeometryQuery::pointDistance(voxelPos(x, y, z), geom, pose) == 0)
						setVoxel(x, y, z);
				}

}

PxVec3 VoxelMap::voxelPos(int x, int y, int z) const
{
	return PxVec3((x + 0.5f) * voxelSizeX(), (y + 0.5f) * voxelSizeY(), (z + 0.5f) * voxelSizeZ()) - extents() * 0.5f;
}

void VoxelMap::pointCoords(const PxVec3& p, int& x, int& y, int& z) const
{
	PxVec3 l = p + extents() * 0.5f, s = voxelSize();
	x = int(PxFloor(l.x / s.x));
	y = int(PxFloor(l.y / s.y));
	z = int(PxFloor(l.z / s.z));
}

void VoxelMap::getVoxelRegion(const PxBounds3& b, int& sx, int& sy, int& sz, int& ex, int& ey, int& ez) const
{
	pointCoords(b.minimum, sx, sy, sz);
	pointCoords(b.maximum, ex, ey, ez);
}

// physx::PxCustomGeometry::Callbacks overrides

IMPLEMENT_CUSTOM_GEOMETRY_TYPE(VoxelMap)

PxBounds3 VoxelMap::getLocalBounds(const PxGeometry&) const
{
	return PxBounds3::centerExtents(PxVec3(0), extents());
}

bool VoxelMap::generateContacts(const PxGeometry& /*geom0*/, const PxGeometry& geom1, const PxTransform& pose0, const PxTransform& pose1,
	const PxReal contactDistance, const PxReal meshContactMargin, const PxReal toleranceLength,
	PxContactBuffer& contactBuffer) const
{
	PxBoxGeometry voxelGeom(voxelSize() * 0.5f);
	PxGeometry* pGeom0 = &voxelGeom;

	const PxGeometry* pGeom1 = &geom1;
	PxTransform pose1in0 = pose0.transformInv(pose1);
	PxBounds3 bounds1; PxGeometryQuery::computeGeomBounds(bounds1, geom1, pose1in0, contactDistance);

	struct ContactRecorder : immediate::PxContactRecorder
	{
		PxContactBuffer* contactBuffer;
		ContactRecorder(PxContactBuffer& _contactBuffer) : contactBuffer(&_contactBuffer) {}
		virtual bool recordContacts(const PxContactPoint* contactPoints, PxU32 nbContacts, PxU32 /*index*/)
		{
			for (PxU32 i = 0; i < nbContacts; ++i)
				contactBuffer->contact(contactPoints[i]);
			return true;
		}
	}
	contactRecorder(contactBuffer);

	PxCache contactCache;

	struct ContactCacheAllocator : PxCacheAllocator
	{
		PxU8 buffer[1024];
		ContactCacheAllocator() { memset(buffer, 0, sizeof(buffer)); }
		virtual PxU8* allocateCacheData(const PxU32 /*byteSize*/) { return reinterpret_cast<PxU8*>(size_t(buffer + 0xf) & ~0xf); }
	}
	contactCacheAllocator;

	int sx, sy, sz, ex, ey, ez;
	getVoxelRegion(bounds1, sx, sy, sz, ex, ey, ez);
	for (int x = sx; x <= ex; ++x)
		for (int y = sy; y <= ey; ++y)
			for (int z = sz; z <= ez; ++z)
				if (voxel(x, y, z))
				{
					PxTransform p0 = pose0.transform(PxTransform(voxelPos(x, y, z)));
					immediate::PxGenerateContacts(&pGeom0, &pGeom1, &p0, &pose1, &contactCache, 1, contactRecorder,
						contactDistance, meshContactMargin, toleranceLength, contactCacheAllocator);
				}

	return true;
}

PxU32 VoxelMap::raycast(const PxVec3& origin, const PxVec3& unitDir, const PxGeometry& /*geom*/, const PxTransform& pose,
	PxReal maxDist, PxHitFlags hitFlags, PxU32 maxHits, PxGeomRaycastHit* rayHits, PxU32 stride, PxRaycastThreadContext*) const
{
	PxVec3 p = pose.transformInv(origin);
	PxVec3 n = pose.rotateInv(unitDir);
	PxVec3 s = voxelSize() * 0.5f;
	int x, y, z; pointCoords(p, x, y, z);
	int hitCount = 0;
	PxU8* hitBuffer = reinterpret_cast<PxU8*>(rayHits);
	float currDist = 0;
	PxVec3 hitN(0);

	while (currDist < maxDist)
	{
		PxVec3 v = voxelPos(x, y, z);
		if (voxel(x, y, z))
		{
			PxGeomRaycastHit& h = *reinterpret_cast<PxGeomRaycastHit*>(hitBuffer + hitCount * stride);
			h.distance = currDist;
			if (hitFlags.isSet(PxHitFlag::ePOSITION))
				h.position = origin + unitDir * currDist;
			if (hitFlags.isSet(PxHitFlag::eNORMAL))
				h.normal = hitN;
			if (hitFlags.isSet(PxHitFlag::eFACE_INDEX))
				h.faceIndex = (x) | (y << 10) | (z << 20);
			hitCount += 1;
		}
		if (hitCount == int(maxHits))
			break;
		float step = FLT_MAX;
		int dx = 0, dy = 0, dz = 0;
		if (n.x > FLT_EPSILON)
		{
			float d = (v.x + s.x - p.x) / n.x;
			if (d < step) { step = d; dx = 1; dy = 0; dz = 0; }
		}
		if (n.x < -FLT_EPSILON)
		{
			float d = (v.x - s.x - p.x) / n.x;
			if (d < step) { step = d; dx = -1; dy = 0; dz = 0; }
		}
		if (n.y > FLT_EPSILON)
		{
			float d = (v.y + s.y - p.y) / n.y;
			if (d < step) { step = d; dx = 0; dy = 1; dz = 0; }
		}
		if (n.y < -FLT_EPSILON)
		{
			float d = (v.y - s.y - p.y) / n.y;
			if (d < step) { step = d; dx = 0; dy = -1; dz = 0; }
		}
		if (n.z > FLT_EPSILON)
		{
			float d = (v.z + s.z - p.z) / n.z;
			if (d < step) { step = d; dx = 0; dy = 0; dz = 1; }
		}
		if (n.z < -FLT_EPSILON)
		{
			float d = (v.z - s.z - p.z) / n.z;
			if (d < step) { step = d; dx = 0; dy = 0; dz = -1; }
		}
		x += dx; y += dy; z += dz;
		hitN = PxVec3(float(-dx), float(-dy), float(-dz));
		currDist = step;
	}

	return hitCount;
}

bool VoxelMap::overlap(const PxGeometry& /*geom0*/, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1, PxOverlapThreadContext*) const
{
	PxBoxGeometry voxelGeom(voxelSize() * 0.5f);

	PxTransform pose1in0 = pose0.transformInv(pose1);
	PxBounds3 bounds1; PxGeometryQuery::computeGeomBounds(bounds1, geom1, pose1in0);

	int sx, sy, sz, ex, ey, ez;
	getVoxelRegion(bounds1, sx, sy, sz, ex, ey, ez);
	for (int x = sx; x <= ex; ++x)
		for (int y = sy; y <= ey; ++y)
			for (int z = sz; z <= ez; ++z)
				if (voxel(x, y, z))
				{
					PxTransform p0 = pose0.transform(PxTransform(voxelPos(x, y, z)));
					if (PxGeometryQuery::overlap(voxelGeom, p0, geom1, pose1, PxGeometryQueryFlags(0)))
						return true;
				}

	return false;
}

bool VoxelMap::sweep(const PxVec3& unitDir, const PxReal maxDist,
	const PxGeometry& /*geom0*/, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1,
	PxGeomSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation, PxSweepThreadContext*) const
{
	PxBoxGeometry voxelGeom(voxelSize() * 0.5f);

	PxTransform pose1in0 = pose0.transformInv(pose1);
	PxBounds3 b; PxGeometryQuery::computeGeomBounds(b, geom1, pose1in0, 0, 1.0f, PxGeometryQueryFlags(0));
	PxVec3 n = pose0.rotateInv(unitDir);
	PxVec3 s = voxelSize();

	int sx, sy, sz, ex, ey, ez;
	getVoxelRegion(b, sx, sy, sz, ex, ey, ez);
	int sx1, sy1, sz1, ex1, ey1, ez1;
	sx1 = sy1 = sz1 = -1; ex1 = ey1 = ez1 = 0;
	float currDist = 0;
	sweepHit.distance = FLT_MAX;

	while (currDist < maxDist && currDist < sweepHit.distance)
	{
		for (int x = sx; x <= ex; ++x)
			for (int y = sy; y <= ey; ++y)
				for (int z = sz; z <= ez; ++z)
					if (voxel(x, y, z))
					{
						if (x >= sx1 && x <= ex1 && y >= sy1 && y <= ey1 && z >= sz1 && z <= ez1)
							continue;

						PxGeomSweepHit hit;
						PxTransform p0 = pose0.transform(PxTransform(voxelPos(x, y, z)));
						if (PxGeometryQuery::sweep(unitDir, maxDist, geom1, pose1, voxelGeom, p0, hit, hitFlags, inflation, PxGeometryQueryFlags(0)))
							if (hit.distance < sweepHit.distance)
								sweepHit = hit;
					}

		PxVec3 mi = b.minimum, ma = b.maximum;
		PxVec3 bs = voxelPos(sx, sy, sz) - s, be = voxelPos(ex, ey, ez) + s;
		float dist = FLT_MAX;
		if (n.x > FLT_EPSILON)
		{
			float d = (be.x - ma.x) / n.x;
			if (d < dist) dist = d;
		}
		if (n.x < -FLT_EPSILON)
		{
			float d = (bs.x - mi.x) / n.x;
			if (d < dist) dist = d;
		}
		if (n.y > FLT_EPSILON)
		{
			float d = (be.y - ma.y) / n.y;
			if (d < dist) dist = d;
		}
		if (n.y < -FLT_EPSILON)
		{
			float d = (bs.y - mi.y) / n.y;
			if (d < dist) dist = d;
		}
		if (n.z > FLT_EPSILON)
		{
			float d = (be.z - ma.z) / n.z;
			if (d < dist) dist = d;
		}
		if (n.z < -FLT_EPSILON)
		{
			float d = (bs.z - mi.z) / n.z;
			if (d < dist) dist = d;
		}
		sx1 = sx; sy1 = sy; sz1 = sz; ex1 = ex; ey1 = ey; ez1 = ez;
		PxBounds3 b1 = b; b1.minimum += n * dist; b1.maximum += n * dist;
		getVoxelRegion(b1, sx, sy, sz, ex, ey, ez);
		currDist = dist;
	}

	return sweepHit.distance < FLT_MAX;
}

void VoxelMap::visualize(const physx::PxGeometry& /*geom*/, physx::PxRenderOutput& render, const physx::PxTransform& transform, const physx::PxBounds3& /*bound*/) const
{
	PxVec3 extents = voxelSize() * 0.5f;

	render << transform;

	for (int x = 0; x < dimX(); ++x)
		for (int y = 0; y < dimY(); ++y)
			for (int z = 0; z < dimZ(); ++z)
				if (voxel(x, y, z))
				{
					if (voxel(x + 1, y, z) &&
						voxel(x - 1, y, z) &&
						voxel(x, y + 1, z) &&
						voxel(x, y - 1, z) &&
						voxel(x, y, z + 1) &&
						voxel(x, y, z - 1))
						continue;

					PxVec3 pos = voxelPos(x, y, z);

					PxBounds3 bounds(pos - extents, pos + extents);

					physx::PxDebugBox box(bounds, true);
					render << box;
				}
}
