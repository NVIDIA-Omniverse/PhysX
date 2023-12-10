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

#ifndef SC_SHAPESIM_BASE_H
#define SC_SHAPESIM_BASE_H

#include "ScElementSim.h"
#include "ScShapeCore.h"
#include "ScRigidSim.h"
#include "PxsShapeSim.h"

namespace physx
{
	namespace Sc
	{
		PX_FORCE_INLINE PxU32 isBroadPhase(PxShapeFlags flags) { return PxU32(flags) & PxU32(PxShapeFlag::eTRIGGER_SHAPE | PxShapeFlag::eSIMULATION_SHAPE); }

		class ShapeCore;

		// PT: TODO: ShapeSimBase is bonkers:
		//	PxU32			ElementSim::mElementID
		//	PxU32			ElementSim::mShapeArrayIndex;
		//	IG::NodeIndex	mLLShape::mBodySimIndex;		*** GPU only
		//	PxU32			mLLShape::mElementIndex;		*** GPU only, looks like a copy of ElementSim::mElementID
		//	PxU32			mLLShape::mShapeIndex;			*** GPU only, looks like a copy of ElementSim::mElementID
		//	PxU32			ShapeSimBase::mId;
		//	PxU32			ShapeSimBase::mSqBoundsId;
		// => do we really need 7 different IDs per shape?

		class ShapeSimBase : public ElementSim
		{
			PX_NOCOPY(ShapeSimBase)
		public:
													ShapeSimBase(ActorSim& owner, const ShapeCore* core) :
														ElementSim	(owner),
														mSqBoundsId	(PX_INVALID_U32),
														mPrunerIndex(PX_INVALID_U32)
																							{	setCore(core);	}
													~ShapeSimBase()							{					}

			PX_FORCE_INLINE void					setCore(const ShapeCore* core);
			PX_FORCE_INLINE const ShapeCore&		getCore()						const;
	        PX_FORCE_INLINE bool                    isPxsCoreValid()                const   { return mLLShape.mShapeCore != NULL; }

			PX_INLINE		PxGeometryType::Enum	getGeometryType()				const	{ return getCore().getGeometryType();	}

			// This is just for getting a reference for the user, so we cast away const-ness

			PX_INLINE		PxShape*				getPxShape()					const	{ return const_cast<PxShape*>(getCore().getPxShape()); }

			PX_FORCE_INLINE	PxReal					getRestOffset()					const	{ return getCore().getRestOffset();					}
			PX_FORCE_INLINE	PxReal					getTorsionalPatchRadius()		const	{ return getCore().getTorsionalPatchRadius();		}
			PX_FORCE_INLINE	PxReal					getMinTorsionalPatchRadius()	const	{ return getCore().getMinTorsionalPatchRadius();	}
			PX_FORCE_INLINE	PxU32					getFlags()						const	{ return getCore().getFlags();						}
			PX_FORCE_INLINE	PxReal					getContactOffset()				const	{ return getCore().getContactOffset();				}

			PX_FORCE_INLINE	PxU32					getTransformCacheID()			const	{ return getElementID();	}

			PX_FORCE_INLINE PxU32					getSqBoundsId()					const	{ return mSqBoundsId;		}
			PX_FORCE_INLINE void					setSqBoundsId(PxU32 id)					{ mSqBoundsId = id;			}

			PX_FORCE_INLINE PxU32					getSqPrunerIndex()				const	{ return mPrunerIndex;		}
			PX_FORCE_INLINE void					setSqPrunerIndex(PxU32 index)			{ mPrunerIndex = index;		}

			PX_FORCE_INLINE PxsShapeSim&			getLLShapeSim()							{ return mLLShape;			}

							void					onFilterDataChange();
							void					onRestOffsetChange();
							void					onFlagChange(PxShapeFlags oldFlags);
							void					onResetFiltering();
							void					onVolumeOrTransformChange();
							void					onMaterialChange();  // remove when material properties are gone from PxcNpWorkUnit
							void					onContactOffsetChange();
							void					markBoundsForUpdate();
							void					reinsertBroadPhase();
							void					removeFromBroadPhase(bool wakeOnLostTouch);
							void					getAbsPoseAligned(PxTransform* PX_RESTRICT globalPose)	const;

			PX_FORCE_INLINE	RigidSim&				getRbSim()				const { return static_cast<RigidSim&>(getActor()); }
							BodySim*				getBodySim()			const;

							PxsRigidCore&			getPxsRigidCore()		const;

							void					createSqBounds();
							void					destroySqBounds();

							void					updateCached(PxU32 transformCacheFlags, PxBitMapPinned* shapeChangedMap);
							void					updateCached(PxsTransformCache& transformCache, Bp::BoundsArray& boundsArray);
							void					updateBPGroup();
		protected:

			PX_FORCE_INLINE	void					internalAddToBroadPhase();
			PX_FORCE_INLINE	bool					internalRemoveFromBroadPhase(bool wakeOnLostTouch = true);
							void					initSubsystemsDependingOnElementID();
							
							PxsShapeSim				mLLShape;
							PxU32					mSqBoundsId;
							PxU32					mPrunerIndex;
		};

#if PX_P64_FAMILY
		// PT: to compensate for the padding I removed in PxsShapeSim
		PX_COMPILE_TIME_ASSERT((sizeof(ShapeSimBase) - sizeof(PxsShapeSim))>=12);
#else
		//	PX_COMPILE_TIME_ASSERT(32==sizeof(Sc::ShapeSim)); // after removing bounds from shapes
		//	PX_COMPILE_TIME_ASSERT((sizeof(Sc::ShapeSim) % 16) == 0); // aligned mem bounds are better for prefetching
#endif

		PX_FORCE_INLINE void ShapeSimBase::setCore(const ShapeCore* core)
		{
			mLLShape.mShapeCore = core ? const_cast<PxsShapeCore*>(&core->getCore()) : NULL;
		}
		PX_FORCE_INLINE const ShapeCore& ShapeSimBase::getCore() const
		{
			return Sc::ShapeCore::getCore(*mLLShape.mShapeCore);
		}

	} // namespace Sc
}

#endif
