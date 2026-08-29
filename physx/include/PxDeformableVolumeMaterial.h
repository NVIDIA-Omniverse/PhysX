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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_DEFORMABLE_VOLUME_MATERIAL_H
#define PX_DEFORMABLE_VOLUME_MATERIAL_H

#include "PxDeformableMaterial.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxDeformableVolumeMaterialModel
{
	enum Enum
	{
		eCO_ROTATIONAL,   //!< Default model. Well suited for high stiffness. Does need tetrahedra with good shapes (no extreme slivers) in the rest pose.
		eNEO_HOOKEAN      //!< Well suited for lower stiffness. Robust to any tetrahedron shape.
	};
};

class PxScene;
/**
\brief Material class to represent a set of deformable volume material properties.

\see PxPhysics.createDeformableVolumeMaterial
*/
class PxDeformableVolumeMaterial : public PxDeformableMaterial
{
public:

	/**
	\brief Sets the material model.

	\param[in] model The material model

	\see getMaterialModel
	*/
	virtual		void	setMaterialModel(PxDeformableVolumeMaterialModel::Enum model) = 0;
	
	/**
	\brief Retrieves the material model.
	\return The material model.

	\see setMaterialModel()
	*/
	virtual		PxDeformableVolumeMaterialModel::Enum getMaterialModel() const = 0;

	/**
	\brief Gets the concrete type name.
	\return The name of the concrete type.
	*/
	virtual		const char*		getConcreteTypeName() const	PX_OVERRIDE	PX_FINAL	{ return "PxDeformableVolumeMaterial"; }

protected:
	PX_INLINE			PxDeformableVolumeMaterial(PxType concreteType, PxBaseFlags baseFlags) : PxDeformableMaterial(concreteType, baseFlags) {}
	PX_INLINE			PxDeformableVolumeMaterial(PxBaseFlags baseFlags) : PxDeformableMaterial(baseFlags) {}
	virtual				~PxDeformableVolumeMaterial() {}
	virtual		bool	isKindOf(const char* name) const PX_OVERRIDE { PX_IS_KIND_OF(name, "PxDeformableVolumeMaterial", PxDeformableMaterial); }
};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif // PX_DEFORMABLE_VOLUME_MATERIAL_H
