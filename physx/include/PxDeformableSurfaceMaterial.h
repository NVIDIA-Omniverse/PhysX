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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_DEFORMABLE_SURFACE_MATERIAL_H
#define PX_DEFORMABLE_SURFACE_MATERIAL_H

#include "PxDeformableMaterial.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Material class to represent surface deformable material properties.

\see PxPhysics.createDeformableSurfaceMaterial
*/
class PxDeformableSurfaceMaterial : public PxDeformableMaterial
{
public:

	/**
	\brief Sets material thickness

	<b>Default:</b> 0.001
	\param[in] thickness Material thickness.

	\see getThickness()
	*/
	virtual		void			setThickness(PxReal thickness) = 0;

	/**
	\brief Retrieves the material thickness.
	
	<b>Default:</b> 0.001
	\return thickness.
	\see setThickness()
	*/
	virtual		PxReal			getThickness() const = 0;

	/**
	\brief Sets material bending stiffness

	<b>Default:</b> 0.0
	\param[in] bendingStiffness Material bending stiffness.
	\see getBendingStiffness()
	*/
	virtual		void			setBendingStiffness(PxReal bendingStiffness) = 0;

	/**
	\brief Retrieves the material bending stiffness.

	\return bendingStiffness.
	\see setBendingStiffness()
	*/
	virtual		PxReal			getBendingStiffness() const = 0;

	/**
	\brief Sets material bending damping

	\param[in] bendingDamping Material bending damping.

	\see getBendingDamping()
	*/
	virtual		void			setBendingDamping(PxReal bendingDamping) = 0;

	/**
	\brief Retrieves the material bending damping.
	\return bending damping.

	\see setBendingDamping()
	*/
	virtual		PxReal			getBendingDamping() const = 0;


	/**
	\brief Gets the concrete type name.
	\return The name of the concrete type.
	*/
	virtual		const char*		getConcreteTypeName() const	PX_OVERRIDE	PX_FINAL	{ return "PxDeformableSurfaceMaterial"; }

protected:
	PX_INLINE					PxDeformableSurfaceMaterial(PxType concreteType, PxBaseFlags baseFlags) : PxDeformableMaterial(concreteType, baseFlags) {}
	PX_INLINE					PxDeformableSurfaceMaterial(PxBaseFlags baseFlags) : PxDeformableMaterial(baseFlags) {}
	virtual						~PxDeformableSurfaceMaterial() {}
	virtual		bool			isKindOf(const char* name) const { PX_IS_KIND_OF(name, "PxDeformableSurfaceMaterial", PxDeformableMaterial); }
};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif // PX_DEFORMABLE_SURFACE_MATERIAL_H
