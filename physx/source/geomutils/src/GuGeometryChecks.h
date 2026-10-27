// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_GEOMETRY_CHECKS_H
#define GU_GEOMETRY_CHECKS_H

#include "geometry/PxBoxGeometry.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxPlaneGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxParticleSystemGeometry.h"
#include "geometry/PxTetrahedronMeshGeometry.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxHeightFieldGeometry.h"
#include "geometry/PxCustomGeometry.h"
#include "geometry/PxConvexCoreGeometry.h"

namespace physx
{

	// We sometimes overload capsule code for spheres, so every sphere should have 
	// valid capsule data (height = 0). This is preferable to a typedef so that we
	// can maintain traits separately for a sphere, but some care is required to deal
	// with the fact that when a reference to a capsule is extracted, it may have its
	// type field set to eSPHERE

	template <typename T>
	struct PxcGeometryTraits
	{
		enum {TypeID = PxGeometryType::eINVALID };
	};
	template <typename T> struct PxcGeometryTraits<const T> { enum { TypeID = PxcGeometryTraits<T>::TypeID }; };

	template <> struct PxcGeometryTraits<PxBoxGeometry>					{ enum { TypeID = PxGeometryType::eBOX }; };
	template <> struct PxcGeometryTraits<PxSphereGeometry>				{ enum { TypeID = PxGeometryType::eSPHERE }; };
	template <> struct PxcGeometryTraits<PxCapsuleGeometry>				{ enum { TypeID = PxGeometryType::eCAPSULE }; };
	template <> struct PxcGeometryTraits<PxPlaneGeometry>				{ enum { TypeID = PxGeometryType::ePLANE }; };
	template <> struct PxcGeometryTraits<PxParticleSystemGeometry>		{ enum { TypeID = PxGeometryType::ePARTICLESYSTEM}; };
	template <> struct PxcGeometryTraits<PxConvexCoreGeometry>			{ enum { TypeID = PxGeometryType::eCONVEXCORE }; };
	template <> struct PxcGeometryTraits<PxConvexMeshGeometry>			{ enum { TypeID = PxGeometryType::eCONVEXMESH }; };
	template <> struct PxcGeometryTraits<PxTriangleMeshGeometry>		{ enum { TypeID = PxGeometryType::eTRIANGLEMESH }; };
	template <> struct PxcGeometryTraits<PxTetrahedronMeshGeometry>		{ enum { TypeID = PxGeometryType::eTETRAHEDRONMESH }; };
	template <> struct PxcGeometryTraits<PxHeightFieldGeometry>			{ enum { TypeID = PxGeometryType::eHEIGHTFIELD }; };
	template <> struct PxcGeometryTraits<PxCustomGeometry>				{ enum { TypeID = PxGeometryType::eCUSTOM }; };

	template<class T> PX_CUDA_CALLABLE PX_FORCE_INLINE void checkType(const PxGeometry& geometry)
	{
		PX_ASSERT(PxU32(geometry.getType()) == PxU32(PxcGeometryTraits<T>::TypeID));
		PX_UNUSED(geometry);
	}

	template<> PX_CUDA_CALLABLE PX_FORCE_INLINE void checkType<PxCapsuleGeometry>(const PxGeometry& geometry)
	{
		PX_ASSERT(geometry.getType() == PxGeometryType::eCAPSULE || geometry.getType() == PxGeometryType::eSPHERE);
		PX_UNUSED(geometry);
	}

	template<> PX_CUDA_CALLABLE PX_FORCE_INLINE void checkType<const PxCapsuleGeometry>(const PxGeometry& geometry)
	{
		PX_ASSERT(geometry.getType()== PxGeometryType::eCAPSULE || geometry.getType() == PxGeometryType::eSPHERE);
		PX_UNUSED(geometry);
	}
}

#if !PX_CUDA_COMPILER
// the shape structure relies on punning capsules and spheres 
PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(physx::PxCapsuleGeometry, radius) == PX_OFFSET_OF(physx::PxSphereGeometry, radius));
#endif

#endif
