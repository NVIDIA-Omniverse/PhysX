// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef VOXEL_MAP_H
#define VOXEL_MAP_H

#include "PxPhysicsAPI.h"

/*
VoxelMap inherits physx::PxCustomGeometry::Callbacks interface and provides
implementations for 5 callback functions:

- getLocalBounds
- generateContacts
- raycast
- overlap
- sweep

It should be passed to PxCustomGeometry constructor.

*/
struct VoxelMap : physx::PxCustomGeometry::Callbacks, physx::PxUserAllocated
{
	void setDimensions(int x, int y, int z);
	int dimX() const;
	int dimY() const;
	int dimZ() const;

	void setVoxelSize(float x, float y, float z);
	const physx::PxVec3& voxelSize() const;
	float voxelSizeX() const;
	float voxelSizeY() const;
	float voxelSizeZ() const;

	physx::PxVec3 extents() const;

	void setVoxel(int x, int y, int z, bool yes = true);

	bool voxel(int x, int y, int z) const;

	void clearVoxels();

	void setFloorVoxels(int layers);

	void setWaveVoxels();

	void voxelize(const physx::PxGeometry& geom, const physx::PxTransform& pose, bool add = true);

	physx::PxVec3 voxelPos(int x, int y, int z) const;

	void pointCoords(const physx::PxVec3& p, int& x, int& y, int& z) const;

	void getVoxelRegion(const physx::PxBounds3& b, int& sx, int& sy, int& sz, int& ex, int& ey, int& ez) const;

	// physx::PxCustomGeometry::Callbacks overrides

	DECLARE_CUSTOM_GEOMETRY_TYPE
	virtual physx::PxBounds3 getLocalBounds(const physx::PxGeometry&) const;
	virtual bool generateContacts(const physx::PxGeometry& geom0, const physx::PxGeometry& geom1, const physx::PxTransform& pose0, const physx::PxTransform& pose1,
		const physx::PxReal contactDistance, const physx::PxReal meshContactMargin, const physx::PxReal toleranceLength,
		physx::PxContactBuffer& contactBuffer) const;
	virtual physx::PxU32 raycast(const physx::PxVec3& origin, const physx::PxVec3& unitDir, const physx::PxGeometry& geom, const physx::PxTransform& pose,
		physx::PxReal maxDist, physx::PxHitFlags hitFlags, physx::PxU32 maxHits, physx::PxGeomRaycastHit* rayHits, physx::PxU32 stride, physx::PxRaycastThreadContext*) const;
	virtual bool overlap(const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1, physx::PxOverlapThreadContext*) const;
	virtual bool sweep(const physx::PxVec3& unitDir, const physx::PxReal maxDist,
		const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1,
		physx::PxGeomSweepHit& sweepHit, physx::PxHitFlags hitFlags, const physx::PxReal inflation, physx::PxSweepThreadContext*) const;
	virtual void visualize(const physx::PxGeometry&, physx::PxRenderOutput&, const physx::PxTransform&, const physx::PxBounds3&) const;
	virtual void computeMassProperties(const physx::PxGeometry&, physx::PxMassProperties&) const {}
	virtual bool usePersistentContactManifold(const physx::PxGeometry&, physx::PxReal&) const { return true; }

private:

	int pacsX() const;
	int pacsY() const;
	int pacsZ() const;

	int m_dimensions[3];
	physx::PxVec3 m_voxelSize;
	physx::PxArray<physx::PxU64> m_packs;
};

#endif
