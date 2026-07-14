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

#ifndef SC_SHAPE_CORE_H
#define SC_SHAPE_CORE_H

#include "foundation/PxUtilities.h"
#include "PxvGeometry.h"
#include "PxFiltering.h"
#include "PxShape.h"

namespace physx
{
class PxShape;	// PT: TODO: fw decl of higher-level class isn't great

namespace Sc
{
	class ShapeSim;

	class ShapeCore : public PxsShapeCore
	{
	public:
// PX_SERIALIZATION
													ShapeCore(const PxEMPTY);
						void						exportExtraData(PxSerializationContext& stream);
						void						importExtraData(PxDeserializationContext& context);
						void						resolveReferences(PxDeserializationContext& context);
						void						resolveMaterialReference(PxU32 materialTableIndex, PxU16 materialIndex);
//~PX_SERIALIZATION
													ShapeCore(	const PxGeometry& geometry, PxShapeFlags shapeFlags,
																const PxU16* materialIndices, PxU16 materialCount, bool isExclusive,
																PxShapeCoreFlag::Enum coreFlags = PxShapeCoreFlag::Enum(0));

													~ShapeCore();

		PX_FORCE_INLINE	PxGeometryType::Enum		getGeometryType()							const	{ return mGeometry.getType();				}
						PxShape*					getPxShape();
						const PxShape*				getPxShape()								const;

		PX_FORCE_INLINE	const GeometryUnion&		getGeometryUnion()							const	{ return mGeometry;							}
		PX_FORCE_INLINE	const PxGeometry&			getGeometry()								const	{ return mGeometry.getGeometry();			}
						void						setGeometry(const PxGeometry& geom);

						PxU16						getNbMaterialIndices()						const;
						const PxU16*				getMaterialIndices()						const;
						void						setMaterialIndices(const PxU16* materialIndices, PxU16 materialIndexCount);

		PX_FORCE_INLINE	const PxTransform&			getShape2Actor()							const	{ return getTransform();					}
		PX_FORCE_INLINE	void						setShape2Actor(const PxTransform& s2b)				{ setTransform(s2b);						}

		PX_FORCE_INLINE	const PxFilterData&			getSimulationFilterData()					const	{ return mSimulationFilterData;				}
		PX_FORCE_INLINE	void						setSimulationFilterData(const PxFilterData& data)	{ mSimulationFilterData = data;				}

		PX_FORCE_INLINE	PxReal						getContactOffset()							const	{ return mContactOffset;					}
						void						setContactOffset(PxReal offset);

		PX_FORCE_INLINE	PxReal						getRestOffset()								const	{ return mRestOffset;						}
		PX_FORCE_INLINE	void						setRestOffset(PxReal offset)						{ mRestOffset = offset;						}

		PX_FORCE_INLINE	PxReal						getTorsionalPatchRadius()					const	{ return mTorsionalRadius;					}
		PX_FORCE_INLINE	void						setTorsionalPatchRadius(PxReal tpr)					{ mTorsionalRadius = tpr;					}

		PX_FORCE_INLINE PxReal						getMinTorsionalPatchRadius()				const	{return mMinTorsionalPatchRadius;			}
		PX_FORCE_INLINE	void						setMinTorsionalPatchRadius(PxReal radius)			{ mMinTorsionalPatchRadius = radius;		}

		PX_FORCE_INLINE	PxShapeFlags				getFlags()									const	{ return PxShapeFlags(mShapeFlags);			}
		PX_FORCE_INLINE	void						setFlags(PxShapeFlags f)							{ mShapeFlags = f;							}

		PX_FORCE_INLINE ShapeSim*					getExclusiveSim() const			
													{
														return mExclusiveSim;
													}

		PX_FORCE_INLINE void						setExclusiveSim(ShapeSim* sim)	
													{
														if(!sim || mShapeCoreFlags.isSet(PxShapeCoreFlag::eIS_EXCLUSIVE))
															mExclusiveSim = sim;
													}

#if PX_WINDOWS_FAMILY	// PT: to avoid "error: offset of on non-standard-layout type" on Linux
	protected:
#endif
						PxFilterData				mSimulationFilterData;	// Simulation filter data
						ShapeSim*					mExclusiveSim;   //only set if shape is exclusive
#if PX_WINDOWS_FAMILY	// PT: to avoid "error: offset of on non-standard-layout type" on Linux
	public:
#endif
						const char*					mName;		// PT: moved here from NpShape to fill padding bytes
	};

} // namespace Sc


}

#endif
