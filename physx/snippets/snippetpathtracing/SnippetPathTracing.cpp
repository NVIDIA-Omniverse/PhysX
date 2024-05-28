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

// ****************************************************************************
// This snippet illustrates how to use a standalone PxBVH for path-tracing.
//
// This is an advanced snippet that demonstrates how to use PxBVH in a more
// realistic/complex case than SnippetStandaloneBVH. It also reuses some
// multithreading code from SnippetMultiThreading, so you should get familiar
// with these two snippets first.
//
// This snippet also illustrates some micro-optimizations (see defines below)
// that don't make much difference in a regular game/setup, but can save
// milliseconds when running a lot of raycasts.
//
// The path-tracing code itself is a mashup of various implementations found
// online, most notably:
// http://aras-p.info/blog/2018/03/28/Daily-Pathtracer-Part-0-Intro/
// https://blog.demofox.org/2020/05/25/casual-shadertoy-path-tracing-1-basic-camera-diffuse-emissive/
//
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "foundation/PxArray.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxFPU.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetCamera.h"
	#include "../snippetrender/SnippetRender.h"
#endif

using namespace physx;

//#define PRINT_TIMINGS
//#define STATS

// PT: flags controlling micro optimizations
#define OPTIM_SKIP_INTERNAL_SIMD_GUARD	1	// Take care of SSE control register directly in the snippet, skip internal version
#define OPTIM_BAKE_MESH_SCALE			1	// Bake scale in mesh vertices (queries against scaled meshes are a bit slower)
#define OPTIM_SHADOW_RAY_CODEPATH		1	// Use dedicated codepath for shadow rays (early exit + skip pos/normal hit computations)

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static int						gFrameIndex = 0;
#if OPTIM_SKIP_INTERNAL_SIMD_GUARD
	static const PxGeometryQueryFlags	gQueryFlags = PxGeometryQueryFlag::Enum(0);
#else
	static const PxGeometryQueryFlags	gQueryFlags = PxGeometryQueryFlag::eDEFAULT;
#endif

namespace
{
	enum MaterialType
	{
		Lambert,
		Metal,
		Dielectric,
	};

	struct CustomMaterial
	{
		MaterialType		mType;
		PxVec3				mAlbedo;
		PxVec3				mEmissive;
		float				mRoughness;
		float				mRI;
	};

	struct CustomObject
	{
		PxGeometryHolder	mGeom;
		PxTransform			mPose;
		CustomMaterial		mMaterial;
	};

	struct CustomHit : PxGeomRaycastHit
	{
		protected:
		const CustomObject*					mObject;
		public:
		PX_FORCE_INLINE	void				setObject(const CustomObject* obj)	{ mObject = obj;						}
		PX_FORCE_INLINE	const CustomObject*	getObject()					const	{ return mObject;						}
		PX_FORCE_INLINE	MaterialType		getType()					const	{ return mObject->mMaterial.mType;		}
		PX_FORCE_INLINE	PxVec3				getAlbedo()					const	{ return mObject->mMaterial.mAlbedo;	}
		PX_FORCE_INLINE	PxVec3				getEmissive()				const	{ return mObject->mMaterial.mEmissive;	}
		PX_FORCE_INLINE	float				getRoughness()				const	{ return mObject->mMaterial.mRoughness;	}
		PX_FORCE_INLINE	float				getRI()						const	{ return mObject->mMaterial.mRI;		}
	};

	struct Ray
	{
		PxVec3	mPos;
		PxVec3	mDir;
	};

	// PT: helper to avoid recomputing the same things for each ray
	class RayProvider
	{
		public:
											RayProvider(float screenWidth, float screenHeight, float fov, const PxVec3& camDir);

			PX_FORCE_INLINE	PxVec3			computeWorldRayF(float xs, float ys)	const;

							PxMat33			mInvView;
							const PxVec3	mCamDir;
							float			mWidth;
							float			mHeight;
							float			mOneOverWidth;
							float			mOneOverHeight;
							float			mHTan;
							float			mVTan;

			PX_NOCOPY(RayProvider)
	};

RayProvider::RayProvider(float screenWidth, float screenHeight, float fov, const PxVec3& camDir) :
	mCamDir			(camDir),
	mWidth			(screenWidth*0.5f),
	mHeight			(screenHeight*0.5f),
	mOneOverWidth	(2.0f/screenWidth),
	mOneOverHeight	(2.0f/screenHeight)
{
	const float HTan = tanf(0.25f * fabsf(PxDegToRad(fov * 2.0f)));
	mHTan	= HTan;
	mVTan	= HTan*(screenWidth/screenHeight);

	PxVec3 right, up;
	PxComputeBasisVectors(camDir, right, up);

	mInvView = PxMat33(-right, up, mCamDir);
}

PX_FORCE_INLINE	PxVec3 RayProvider::computeWorldRayF(float xs, float ys) const
{
	const float u = (xs - mWidth)*mOneOverWidth;
	const float v = -(ys - mHeight)*mOneOverHeight;

	const PxVec3 CamRay(mVTan*u, mHTan*v, 1.0f);

	return mInvView.transform(CamRay).getNormalized();
}

	class CustomScene
	{
		public:
								CustomScene();
								~CustomScene();

			void				release();
			void				addGeom(const PxGeometry& geom, const PxTransform& pose, const PxVec3& albedo, const PxVec3& emissive, MaterialType type=Lambert, float roughness=0.0f, float ri=0.0f);
			void				createBVH();
			void				render()	const;
			PxVec3				trace(const PxVec3& camPos, const PxVec3& dir, PxU32& rngState)											const;
			PxVec3				trace2(const Ray& ray, PxU32& rngState, PxU32 depth, bool doMaterialE = true)							const;

			bool				raycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, CustomHit& hit)						const;
#if OPTIM_SHADOW_RAY_CODEPATH
			bool				shadowRay(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const CustomObject* filtered)		const;
#endif
			bool				scatter(const Ray& r, const CustomHit& hit, PxVec3& attenuation, Ray& scattered, PxVec3& outLightE, PxU32& state)	const;
			PxU32				traceKernel(PxVec3* dest, const PxVec3& camPos, const RayProvider& rp, float fScreenWidth, float fScreenHeight, PxU32 seed, PxU32 offseti, PxU32 offsetj)	const;

			PxArray<CustomObject>	mObjects;
			PxArray<PxU32>		mEmissiveObjects;	// Indices of emissive objects
			PxBVH*				mBVH;
	};

CustomScene::CustomScene() : mBVH(NULL)
{
}

CustomScene::~CustomScene()
{
}

void CustomScene::release()
{
	PX_RELEASE(mBVH);
	mObjects.reset();
	PX_DELETE_THIS;
}

void CustomScene::addGeom(const PxGeometry& geom, const PxTransform& pose, const PxVec3& albedo, const PxVec3& emissive, MaterialType type, float roughness, float ri)
{
	const PxU32 id = mObjects.size();

	CustomObject obj;
	obj.mGeom.storeAny(geom);
	obj.mPose = pose;
	obj.mMaterial.mType = type;
	obj.mMaterial.mAlbedo = albedo;
	obj.mMaterial.mEmissive = emissive;
	obj.mMaterial.mRoughness = roughness;
	obj.mMaterial.mRI = ri;
	mObjects.pushBack(obj);

	if(!emissive.isZero())
		mEmissiveObjects.pushBack(id);
}

void CustomScene::createBVH()
{
	const PxU32 nbObjects = mObjects.size();
	PxBounds3* bounds = new PxBounds3[nbObjects];
	for(PxU32 i=0;i<nbObjects;i++)
	{
		const CustomObject& obj = mObjects[i];
		PxGeometryQuery::computeGeomBounds(bounds[i], obj.mGeom.any(), obj.mPose);
	}

	PxBVHDesc bvhDesc;
	bvhDesc.bounds.count = nbObjects;
	bvhDesc.bounds.data = bounds;
	bvhDesc.bounds.stride = sizeof(PxBounds3);
	mBVH = PxCreateBVH(bvhDesc);
	delete [] bounds;
}

#ifdef STATS
	static PxU32 gTotalNbRays = 0;
#endif

bool CustomScene::raycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, CustomHit& hit) const
{
#ifdef STATS
	gTotalNbRays++;
#endif
	if(!mBVH)
		return false;

	struct LocalCB : PxBVH::RaycastCallback
	{
		LocalCB(const CustomScene& scene, const PxVec3& origin_, const PxVec3& dir, CustomHit& hit_) :
			mScene	(scene),
			mHit	(hit_),
			mOrigin	(origin_),
			mDir	(dir),
			mStatus	(false)
		{
		}

		virtual bool reportHit(PxU32 boundsIndex, PxReal& distance)
		{
			const CustomObject& obj = mScene.mObjects[boundsIndex];
			if(PxGeometryQuery::raycast(mOrigin, mDir, obj.mGeom.any(), obj.mPose, distance, PxHitFlag::eDEFAULT, 1, &mLocalHit, sizeof(PxGeomRaycastHit), gQueryFlags))
			{
				// We need to discard internal hits for refracted rays
				if(mLocalHit.distance==0.0f)
					return true;

				if(mLocalHit.distance<distance)
				{
					distance = mLocalHit.distance;
					static_cast<PxGeomRaycastHit&>(mHit) = mLocalHit;
					mHit.setObject(&obj);
					mStatus = true;
				}
			}
			return true;
		}

		const CustomScene&	mScene;
		CustomHit&			mHit;
		PxGeomRaycastHit	mLocalHit;
		const PxVec3&		mOrigin;
		const PxVec3&		mDir;
		bool				mStatus;

		LocalCB& operator=(const LocalCB&)
		{
			PX_ASSERT(0);
			return *this;
		}
	};

	LocalCB CB(*this, origin, unitDir, hit);
	mBVH->raycast(origin, unitDir, maxDist, CB, gQueryFlags);
	return CB.mStatus;
}

#if OPTIM_SHADOW_RAY_CODEPATH
bool CustomScene::shadowRay(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const CustomObject* filtered) const
{
#ifdef STATS
	gTotalNbRays++;
#endif

	if(!mBVH)
		return false;

	struct LocalCB : PxBVH::RaycastCallback
	{
		LocalCB(const CustomScene& scene, const PxVec3& origin_, const PxVec3& dir, const CustomObject* filtered_) :
			mScene		(scene),
			mFiltered	(filtered_),
			mOrigin		(origin_),
			mDir		(dir),
			mStatus		(false)
		{
		}

		virtual bool reportHit(PxU32 boundsIndex, PxReal& distance)
		{
			const CustomObject& obj = mScene.mObjects[boundsIndex];

			if(&obj==mFiltered)
				return true;

			// PT: we don't need the hit position/normal for shadow rays, so we tell PhysX it can skip computing them.
			// We also use eMESH_ANY to tell the system not to look for the closest hit on triangle meshes.
			if(PxGeometryQuery::raycast(mOrigin, mDir, obj.mGeom.any(), obj.mPose, distance, PxHitFlag::eMESH_ANY, 1, &mLocalHit, sizeof(PxGeomRaycastHit), gQueryFlags))
			{
				mStatus = true;
				return false;
			}
			return true;
		}

		const CustomScene&	mScene;
		const CustomObject*	mFiltered;
		PxGeomRaycastHit	mLocalHit;
		const PxVec3&		mOrigin;
		const PxVec3&		mDir;
		bool				mStatus;

		LocalCB& operator=(const LocalCB&)
		{
			PX_ASSERT(0);
			return *this;
		}
	};

	LocalCB CB(*this, origin, unitDir, filtered);
	mBVH->raycast(origin, unitDir, maxDist, CB, gQueryFlags);
	return CB.mStatus;
}
#endif

struct TracerThread
{
	TracerThread() : mScene(NULL), mRndState(42), mActive(true)
	{
	}

	SnippetUtils::Sync*		mWorkReadySyncHandle;
	SnippetUtils::Thread*	mThreadHandle;
	SnippetUtils::Sync*		mWorkDoneSyncHandle;

	const CustomScene*		mScene;
	const RayProvider*		mRayProvider;
	PxVec3*					mDest;
	PxVec3					mCamPos;
	float					mScreenWidth;
	float					mScreenHeight;
	PxU32					mOffsetX;
	PxU32					mOffsetY;
	PxU32					mRndState;
	bool					mActive;

	void					Setup(const CustomScene* scene, const RayProvider* rp, PxVec3* dest, const PxVec3& camPos, float width, float height, PxU32 offsetX, PxU32 offsetY)
	{
		mScene = scene;
		mRayProvider = rp;
		mDest = dest;
		mCamPos = camPos;
		mScreenWidth = width;
		mScreenHeight = height;
		mOffsetX = offsetX;
		mOffsetY = offsetY;
	}

	void					Run()
	{
		if(mActive)
			mRndState = mScene->traceKernel(mDest, mCamPos, *mRayProvider, mScreenWidth, mScreenHeight, mRndState, mOffsetX, mOffsetY);
	}
};
const PxU32		gNumThreads = 16;
TracerThread	gThreads[gNumThreads];

static PX_FORCE_INLINE PxU32 wang_hash(PxU32& seed)
{
    seed = PxU32(seed ^ PxU32(61)) ^ PxU32(seed >> PxU32(16));
    seed *= PxU32(9);
    seed = seed ^ (seed >> 4);
    seed *= PxU32(0x27d4eb2d);
    seed = seed ^ (seed >> 15);
    return seed;
}
 
/*static PX_FORCE_INLINE PxU32 XorShift32(PxU32& state)
{
    PxU32 x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 15;
    state = x;
    return x;
}*/

static const float gInvValue = float(1.0 / 4294967296.0);
//static const float gInvValue2 = float(1.0 / 16777216.0);

static PX_FORCE_INLINE float RandomFloat01(PxU32& state)
{
//    return float(wang_hash(state)) / 4294967296.0;
	return float(wang_hash(state)) * gInvValue;
//	return (XorShift32(state) & 0xFFFFFF) / 16777216.0f;
//	return (XorShift32(state) & 0xFFFFFF) * gInvValue2;
}
 
/*static PxVec3 RandomInUnitDisk(PxU32& state)
{
    PxVec3 p;
    do
    {
        p = 2.0f * PxVec3(RandomFloat01(state), RandomFloat01(state), 0.0f) - PxVec3(1.0f, 1.0f, 0.0f);
    } while (p.dot(p) >= 1.0f);
    return p;
}*/

static PxVec3 RandomInUnitSphere(PxU32& state)
{
    PxVec3 p;
    do {
        p = 2.0f * PxVec3(RandomFloat01(state), RandomFloat01(state), RandomFloat01(state)) - PxVec3(1.0f);
    } while (p.dot(p) >= 1.0f);
    return p;
}

static PX_FORCE_INLINE PxVec3 RandomUnitVector(PxU32& state)
{
    const float z = RandomFloat01(state) * 2.0f - 1.0f;
    const float a = RandomFloat01(state) * PxTwoPi;
    const float r = sqrtf(1.0f - z * z);
    const float x = r * cosf(a);
    const float y = r * sinf(a);
    return PxVec3(x, y, z);
}

static PX_FORCE_INLINE PxVec3 sky(const PxVec3& dir)
{
	const float t = 0.5f * (dir.y + 1.0f);
	const PxVec3 white(1.0f);
	const PxVec3 blue(0.5f, 0.7f, 1.0f);
	return white*(1.0f-t) + blue*t;
}

static PX_FORCE_INLINE PxVec3 mul(const PxVec3& v0, const PxVec3& v1)
{
	return PxVec3(v0.x*v1.x, v0.y*v1.y, v0.z*v1.z);
}

#ifdef RENDER_SNIPPET
static PX_FORCE_INLINE PxVec3 div(const PxVec3& v0, const PxVec3& v1)
{
	const float x = v1.x!=0.0f ? v0.x/v1.x : 0.0f;
	const float y = v1.y!=0.0f ? v0.y/v1.y : 0.0f;
	const float z = v1.z!=0.0f ? v0.z/v1.z : 0.0f;
	return PxVec3(x, y, z);
//	return PxVec3(v0.x/v1.x, v0.y/v1.y, v0.z/v1.z);
}

/*static PX_FORCE_INLINE float saturate(float a)
{
	return (a < 0.0f) ? 0.0f : (a > 1.0f) ? 1.0f : a;
}*/

static PX_FORCE_INLINE PxVec3 ACESTonemap(const PxVec3& inColor)
{
	const float a = 2.51f;
	const float b = 0.03f;
	const float c = 2.43f;
	const float d = 0.59f;
	const float e = 0.14f;
	const PxVec3 col = div((mul(inColor, (PxVec3(b) + inColor * a))) , (mul(inColor, (inColor * c + PxVec3(d)) + PxVec3(e))));
//	return PxVec3(saturate(col.x), saturate(col.y), saturate(col.z));
	return col;
}
#endif

/*static PX_FORCE_INLINE PxVec3 LessThan(const PxVec3& f, float value)
{
    return PxVec3(
        (f.x < value) ? 1.0f : 0.0f,
        (f.y < value) ? 1.0f : 0.0f,
        (f.z < value) ? 1.0f : 0.0f);
}*/

/*PX_FORCE_INLINE float sqLength(const PxVec3& v)
{
	return v.dot(v);
}*/

#ifdef RENDER_SNIPPET
static PX_FORCE_INLINE PxVec3 max3(const PxVec3& v, float e)
{
    return PxVec3(PxMax(v.x, e), PxMax(v.y, e), PxMax(v.z, e));
}

static PX_FORCE_INLINE PxVec3 pow(const PxVec3& v, float e)
{
    return PxVec3(powf(v.x, e), powf(v.y, e), powf(v.z, e));
}
#endif

/*static PX_FORCE_INLINE PxVec3 pow(const PxVec3& v, const PxVec3& e)
{
    return PxVec3(powf(v.x, e.x), powf(v.y, e.y), powf(v.z, e.z));
}

static PX_FORCE_INLINE PxVec3 mix(const PxVec3& x, const PxVec3& y, const PxVec3& a)
{
	const PxVec3 b = PxVec3(1.0f) - a;
	return mul(x,b) + mul(y,a);
}*/

/*static PX_FORCE_INLINE PxVec3 mix(const PxVec3& x, const PxVec3& y, float a)
{
	const float b = 1.0f - a;
	return (x*b) + (y*a);
}*/

static PX_FORCE_INLINE PxVec3 reflect(const PxVec3& I, const PxVec3& N)
{
	return I - 2.0f * N.dot(I) * N;
}

/*static PX_FORCE_INLINE PxVec3 LinearToSRGB(const PxVec3& rgb)
{
    return mix(
        pow(rgb, PxVec3(1.0f / 2.4f)) * 1.055f - PxVec3(0.055f),
        rgb * 12.92f,
        LessThan(rgb, 0.0031308f)
    );
}*/

#ifdef RENDER_SNIPPET
static PX_FORCE_INLINE PxVec3 LinearToSRGB2(const PxVec3 rgb)
{
//    rgb = max(rgb, float3(0, 0, 0));
    return max3(1.055f * pow(rgb, 0.416666667f) - PxVec3(0.055f), 0.0f);
}
#endif

/*static PX_FORCE_INLINE PxVec3 SRGBToLinear(const PxVec3& rgb2)
{
//    rgb = clamp(rgb, 0.0f, 1.0f);
    PxVec3 rgb = rgb2;
	if(rgb.x>255.0f)
		rgb.x = 255.0f;
	if(rgb.y>255.0f)
		rgb.y = 255.0f;
	if(rgb.z>255.0f)
		rgb.z = 255.0f;
     
    return mix(
        pow(((rgb + PxVec3(0.055f)) / 1.055f), PxVec3(2.4f)),
        rgb / 12.92f,
        LessThan(rgb, 0.04045f)
    );
}*/

static PxVec3* gAccumPixels = NULL;
static PxVec3* gCurrentPixels = NULL;
#ifdef RENDER_SNIPPET
	static GLubyte* gCurrentTexture = NULL;
	static GLuint gTexID = 0;
#endif

static const PxU32 RAYTRACING_RENDER_WIDTH = 512;
static const PxU32 RAYTRACING_RENDER_HEIGHT = 512;
static bool gUseMultipleThreads = true;
static bool gShowAllThreadRenders = false;
static bool gToneMapping = true;
static bool gLinearToSRGB = false;
static float gSkyCoeff = 0.4f;
static float gExposure = 0.5f;
static const float gRayPosNormalNudge = 0.01f;
static const PxU32 gMaxNbBounces = 8;
static const PxU32 gNbSamplesPerPixel = 1;
static const float gOneOverNbSamples = 1.0f / float(gNbSamplesPerPixel);
#define DO_LIGHT_SAMPLING	1
#define ANTIALIASING

static void resetRender()
{
	gFrameIndex = 0;
	if(gAccumPixels)
		PxMemZero(gAccumPixels, RAYTRACING_RENDER_WIDTH*RAYTRACING_RENDER_HEIGHT*sizeof(PxVec3));
}

static PX_FORCE_INLINE bool refract(PxVec3 v, PxVec3 n, float nint, PxVec3& outRefracted)
{
	const float dt = v.dot(n);
	const float discr = 1.0f - nint*nint*(1.0f-dt*dt);
	if(discr > 0.0f)
	{
		outRefracted = nint * (v - n*dt) - n*sqrtf(discr);
		return true;
	}
	return false;
}

static PX_FORCE_INLINE float schlick(float cosine, float ri)
{
	float r0 = (1.0f-ri) / (1.0f+ri);
	r0 = r0*r0;
	return r0 + (1.0f-r0)*powf(1.0f-cosine, 5.0f);
}

bool CustomScene::scatter(const Ray& r, const CustomHit& hit, PxVec3& attenuation, Ray& scattered, PxVec3& outLightE, PxU32& state) const
{
	outLightE = PxVec3(0.0f);

	const MaterialType type = hit.getType();
	if(type == Lambert)
	{
		// random point on unit sphere that is tangent to the hit point
		attenuation = hit.getAlbedo();

		scattered.mPos = hit.position + hit.normal * gRayPosNormalNudge;
		scattered.mDir = (hit.normal + RandomUnitVector(state)).getNormalized();

#if DO_LIGHT_SAMPLING
		const PxU32 nbLights = mEmissiveObjects.size();
		// sample lights
		for(PxU32 j=0; j<nbLights; j++)
		{
			const int i = mEmissiveObjects[j];
			const CustomObject& smat = mObjects[i];
			if(hit.getObject() == &smat)
				continue; // skip self
			PX_ASSERT(smat.mGeom.getType()==PxGeometryType::eSPHERE);
			const PxSphereGeometry& sphereGeom = smat.mGeom.sphere();
			const PxVec3& sphereCenter = smat.mPose.p;

			// create a random direction towards sphere
			// coord system for sampling: sw, su, sv
			PxVec3 delta = sphereCenter - hit.position;
			const float maxDist2 = delta.magnitudeSquared();
			const float maxDist = PxSqrt(maxDist2);
			PxVec3 sw = delta/maxDist;
			PxVec3 su;
			if(fabsf(sw.x)>0.01f)
 				su = PxVec3(0,1,0).cross(sw);
			else
 				su = PxVec3(1,0,0).cross(sw);
			su.normalize();
			PxVec3 sv = sw.cross(su);
			// sample sphere by solid angle
			const float tmp = 1.0f - sphereGeom.radius*sphereGeom.radius / maxDist2;
			const float cosAMax = tmp>0.0f ? sqrtf(tmp) : 0.0f;
			const float eps1 = RandomFloat01(state), eps2 = RandomFloat01(state);
			const float cosA = 1.0f - eps1 + eps1 * cosAMax;
			const float sinA = sqrtf(1.0f - cosA*cosA);
			const float phi = PxTwoPi * eps2;
			const PxVec3 l = su * (cosf(phi) * sinA) + sv * (sinf(phi) * sinA) + sw * cosA;
			//l = normalize(l); // NOTE(fg): This is already normalized, by construction.

			// shoot shadow ray
#if OPTIM_SHADOW_RAY_CODEPATH
			if(!shadowRay(scattered.mPos, l, maxDist, &smat))
#else
			CustomHit lightHit;
			if(raycast(scattered.mPos, l, 5000.0f, lightHit) && lightHit.getObject()==&smat)
#endif
			{
				const float omega = PxTwoPi * (1.0f - cosAMax);
				const PxVec3 nl = hit.normal.dot(r.mDir) < 0.0f ? hit.normal : -hit.normal;
				outLightE += (mul(hit.getAlbedo(), smat.mMaterial.mEmissive)) * (PxMax(0.0f, l.dot(nl)) * omega / PxPi);
			}
		}
#endif
		return true;
	}
	else if(type == Metal)
	{
		const PxVec3 refl = reflect(r.mDir, hit.normal);
		// reflected ray, and random inside of sphere based on roughness
		const float roughness = hit.getRoughness();
		scattered.mPos = hit.position + hit.normal * gRayPosNormalNudge;
		scattered.mDir = (refl + roughness*RandomInUnitSphere(state)).getNormalized();
		attenuation = hit.getAlbedo();
		return scattered.mDir.dot(hit.normal) > 0.0f;
	}
	else if(type == Dielectric)
	{
		attenuation = PxVec3(1.0f);

		const PxVec3& rdir = r.mDir;
		const float matri = hit.getRI();

		PxVec3 outwardN;
		float nint;
		float cosine;
		if(rdir.dot(hit.normal) > 0)
		{
			outwardN = -hit.normal;
			nint = matri;
			cosine = matri * rdir.dot(hit.normal);
		}
		else
		{
			outwardN = hit.normal;
			nint = 1.0f / matri;
			cosine = -rdir.dot(hit.normal);
		}

		float reflProb;
		PxVec3 refr(1.0f);
		if(refract(rdir, outwardN, nint, refr))
			reflProb = schlick(cosine, matri);
		else
			reflProb = 1.0f;

		if(RandomFloat01(state) < reflProb)
		{
			const PxVec3 refl = reflect(rdir, hit.normal);
			scattered.mPos = hit.position + hit.normal * gRayPosNormalNudge;
			scattered.mDir = refl.getNormalized();
		}
		else
		{
			scattered.mPos = hit.position - hit.normal * gRayPosNormalNudge;
			scattered.mDir = refr.getNormalized();
		}
	}
	else
	{
		attenuation = PxVec3(1.0f, 0.0f, 1.0f);
		return false;
	}
	return true;
}

PxVec3 CustomScene::trace2(const Ray& ray, PxU32& rngState, PxU32 depth, bool doMaterialE)	const
{
	CustomHit hit;
	if(raycast(ray.mPos, ray.mDir, 5000.0f, hit))
	{
		Ray scattered;
		PxVec3 attenuation;
		PxVec3 lightE;
		PxVec3 matE = hit.getEmissive();
		if(depth < gMaxNbBounces && scatter(ray, hit, attenuation, scattered, lightE, rngState))
		{
#if DO_LIGHT_SAMPLING
			if(!doMaterialE)
				matE = PxVec3(0.0f); // don't add material emission if told so
			// for Lambert materials, we just did explicit light (emissive) sampling and already
			// for their contribution, so if next ray bounce hits the light again, don't add
			// emission
			doMaterialE = hit.getType() != Lambert;
#endif
			return matE + lightE + mul(attenuation, trace2(scattered, rngState, depth+1, doMaterialE));
		}
		else
			return matE;
	}
	else return sky(ray.mDir)*gSkyCoeff;
}

PxU32 CustomScene::traceKernel(PxVec3* dest, const PxVec3& camPos, const RayProvider& rp, float fScreenWidth, float fScreenHeight, PxU32 seed, PxU32 offseti, PxU32 offsetj) const
{
#if OPTIM_SKIP_INTERNAL_SIMD_GUARD
	PX_SIMD_GUARD
#endif

	PxU32 rngState = seed;

	for(PxU32 jj=0;jj<RAYTRACING_RENDER_HEIGHT;jj+=4)
	{
		const PxU32 j = jj + offsetj;

#ifndef ANTIALIASING
		const PxU32 yi = PxU32(fScreenHeight*float(j));
#endif
		for(PxU32 ii=0;ii<RAYTRACING_RENDER_WIDTH;ii+=4)
		{
			const PxU32 i = ii + offseti;

			PxVec3 color(0.0f);
			for(PxU32 k=0;k<gNbSamplesPerPixel;k++)
			{
#ifdef ANTIALIASING
				const float radius = 1.0f;
				const float jitterX = radius*(RandomFloat01(rngState) - 0.5f);
				const float jitterY = radius*(RandomFloat01(rngState) - 0.5f);
				const float xf = fScreenWidth*(float(i)+jitterX);
				const float yf = fScreenHeight*(float(j)+jitterY);
				const PxVec3 dir = rp.computeWorldRayF(xf, yf);
#else
				const PxU32 xi = PxU32(fScreenWidth*float(i));
				const PxVec3 dir = rp.computeWorldRayF(float(xi), float(yi));
#endif
				Ray r;
				r.mPos = camPos;
				r.mDir = dir;
				color += trace2(r, rngState, 0);
			}
			*dest++ = color * gOneOverNbSamples;
		}

		dest += (RAYTRACING_RENDER_HEIGHT/4)*3;
	}
	return rngState;
}

void CustomScene::render() const
{
#if OPTIM_SKIP_INTERNAL_SIMD_GUARD
	PX_SIMD_GUARD
#endif

#ifdef RENDER_SNIPPET
	if(0)
	{
		const PxVec3 color(1.0f, 0.5f, 0.25f);
		const PxU32 nbObjects = mObjects.size();
		for(PxU32 i=0;i<nbObjects;i++)
		{
			const CustomObject& obj = mObjects[i];
			Snippets::renderGeoms(1, &obj.mGeom, &obj.mPose, false, color);
		}
	}

	const PxU32 screenWidth = Snippets::getScreenWidth();
	const PxU32 screenHeight = Snippets::getScreenHeight();
	Snippets::Camera* sCamera = Snippets::getCamera();
	const PxVec3 camPos = sCamera->getEye();
	const PxVec3 camDir = sCamera->getDir();

	static PxVec3 cachedPos(0.0f);
	static PxVec3 cachedDir(0.0f);
	if(cachedPos!=camPos || cachedDir!=camDir)
	{
		cachedPos=camPos;
		cachedDir=camDir;
		resetRender();
	}

	const PxU32 textureWidth = RAYTRACING_RENDER_WIDTH;
	const PxU32 textureHeight = RAYTRACING_RENDER_HEIGHT;

	if(!gAccumPixels)
	{
		gAccumPixels = new PxVec3[textureWidth*textureHeight];
		PxMemZero(gAccumPixels, textureWidth*textureHeight*sizeof(PxVec3));
	}

	const float fScreenWidth = float(screenWidth)/float(RAYTRACING_RENDER_WIDTH);
	const float fScreenHeight = float(screenHeight)/float(RAYTRACING_RENDER_HEIGHT);

	static PxU32 rngState = 42;

	if(!gCurrentPixels)
	{
		gCurrentPixels = new PxVec3[textureWidth*textureHeight];
		PxMemZero(gCurrentPixels, textureWidth*textureHeight*sizeof(PxVec3));
	}

#ifdef PRINT_TIMINGS
	const DWORD tgt = timeGetTime();
	const DWORD64 time0 = __rdtsc();
#endif
	const RayProvider rp(float(screenWidth), float(screenHeight), 60.0f, camDir);

	PxVec3* buffer = gCurrentPixels;

	if(gUseMultipleThreads)
	{
		PxU32 index = 0;
		for(PxU32 j=0;j<4;j++)
		{
			for(PxU32 i=0;i<4;i++)
			{
				PxU32 offset = (RAYTRACING_RENDER_WIDTH/4)*i;
				offset += (RAYTRACING_RENDER_HEIGHT/4)*j*RAYTRACING_RENDER_WIDTH;
				gThreads[index++].Setup(this, &rp, buffer + offset, camPos, fScreenWidth, fScreenHeight, i, j);
			}
		}

		{
			for (PxU32 i=0; i<gNumThreads; i++)
				SnippetUtils::syncSet(gThreads[i].mWorkReadySyncHandle);

			for (PxU32 i=0; i<gNumThreads; i++)
			{
				SnippetUtils::syncWait(gThreads[i].mWorkDoneSyncHandle);
				SnippetUtils::syncReset(gThreads[i].mWorkDoneSyncHandle);
			}
		}
	}
	else
	{
		for(PxU32 j=0;j<RAYTRACING_RENDER_HEIGHT;j++)
		{
#ifndef ANTIALIASING
			const PxU32 yi = PxU32(fScreenHeight*float(j));
#endif
			for(PxU32 i=0;i<RAYTRACING_RENDER_WIDTH;i++)
			{
				PxVec3 color(0.0f);
				for(PxU32 k=0;k<gNbSamplesPerPixel;k++)
				{
#ifdef ANTIALIASING
					const float radius = 1.0f;
					const float jitterX = radius*(RandomFloat01(rngState) - 0.5f);
					const float jitterY = radius*(RandomFloat01(rngState) - 0.5f);
					const float xf = fScreenWidth*(float(i)+jitterX);
					const float yf = fScreenHeight*(float(j)+jitterY);
					const PxVec3 dir = rp.computeWorldRayF(xf, yf);
#else
					const PxU32 xi = PxU32(fScreenWidth*float(i));
					const PxVec3 dir = rp.computeWorldRayF(float(xi), float(yi));
#endif
					Ray r;
					r.mPos = camPos;
					r.mDir = dir;
					color += trace2(r, rngState, 0);
				}
				*buffer++ = color * gOneOverNbSamples;
			}
		}
	}

#ifdef PRINT_TIMINGS
	const DWORD64 time1 = __rdtsc();
#endif
	gFrameIndex++;
	const float coeff = 1.0f/float(gFrameIndex);
	const float coeff2 = 1.0f-coeff;

	if(gUseMultipleThreads && !gShowAllThreadRenders)
	{
		for(PxU32 j=0;j<4;j++)
		{
			for(PxU32 i=0;i<4;i++)
			{
				PxU32 offset = (RAYTRACING_RENDER_WIDTH/4)*i;
				offset += (RAYTRACING_RENDER_HEIGHT/4)*j*RAYTRACING_RENDER_WIDTH;

				const PxVec3* src = gCurrentPixels + offset;
				PxVec3* dst = gAccumPixels + i + j*RAYTRACING_RENDER_WIDTH;

				for(PxU32 jj=0;jj<RAYTRACING_RENDER_HEIGHT/4;jj++)
				{
					for(PxU32 ii=0;ii<RAYTRACING_RENDER_WIDTH/4;ii++)
					{
						dst[ii*4] = dst[ii*4]*coeff2 + src[ii]*coeff;
					}
					src += RAYTRACING_RENDER_WIDTH;
					dst += RAYTRACING_RENDER_WIDTH*4;
				}
			}
		}
	}
	else
	{
		for(PxU32 i=0;i<textureWidth*textureHeight;i++)
		{
			gAccumPixels[i] = gAccumPixels[i]*coeff2 + gCurrentPixels[i]*coeff;
		}
	}

	if(!gCurrentTexture)
		gCurrentTexture = new GLubyte[textureWidth*textureHeight*4];

	if(1)
	{
		for(PxU32 i=0;i<textureWidth*textureHeight;i++)
		{
			PxVec3 col = gAccumPixels[i];

			// apply exposure (how long the shutter is open)
			col *= gExposure;

			if(gToneMapping)
			{
				// convert unbounded HDR color range to SDR color range
				col = ACESTonemap(col);
			}

			if(gLinearToSRGB)
				col = LinearToSRGB2(col);

			col *= 255.99f;

			if(col.x>255.0f)
				col.x = 255.0f;
			if(col.y>255.0f)
				col.y = 255.0f;
			if(col.z>255.0f)
				col.z = 255.0f;
		
			gCurrentTexture[i*4+0] = GLubyte(col.x);
			gCurrentTexture[i*4+1] = GLubyte(col.y);
			gCurrentTexture[i*4+2] = GLubyte(col.z);
			gCurrentTexture[i*4+3] = 255;
		}
	}

	if(1)
	{
		if(!gTexID)
			gTexID = Snippets::CreateTexture(textureWidth, textureHeight, gCurrentTexture, false);
		else
			Snippets::UpdateTexture(gTexID, textureWidth, textureHeight, gCurrentTexture, false);
//		Snippets::DisplayTexture(gTexID, RAYTRACING_RENDER_WIDTH, 10);
		Snippets::DisplayTexture(gTexID, 0, 0);
	}

#ifdef PRINT_TIMINGS
	const DWORD64 time2 = __rdtsc();
	const DWORD tgt2 = timeGetTime();
	if(1)
	{
		printf("Render: %d\n", int(time1-time0)/1024);
		printf("End   : %d\n", int(time2-time1)/1024);
		printf("Total : %d\n", int(time2-time0)/1024);
		printf("Total : %d ms\n\n", tgt2 - tgt);
	}
#endif
#ifdef STATS
	printf("Total #rays: %d\n", gTotalNbRays);
	gTotalNbRays = 0;
#endif
#endif
}


}

static PxConvexMesh* createConvexMesh(const PxVec3* verts, const PxU32 numVerts, const PxCookingParams& params)
{
	PxConvexMeshDesc convexDesc;
	convexDesc.points.count		= numVerts;
	convexDesc.points.stride	= sizeof(PxVec3);
	convexDesc.points.data		= verts;
	convexDesc.flags			= PxConvexFlag::eCOMPUTE_CONVEX;
	return PxCreateConvexMesh(params, convexDesc);
}

static PxConvexMesh* createCylinderMesh(const PxF32 width, const PxF32 radius, const PxCookingParams& params)
{
	PxVec3 points[2*16];
	for(PxU32 i = 0; i < 16; i++)
	{
		const PxF32 cosTheta = PxCos(i*PxPi*2.0f/16.0f);
		const PxF32 sinTheta = PxSin(i*PxPi*2.0f/16.0f);
		const PxF32 y = radius*cosTheta;
		const PxF32 z = radius*sinTheta;
		points[2*i+0] = PxVec3(-width/2.0f, y, z);
		points[2*i+1] = PxVec3(+width/2.0f, y, z);
	}
	return createConvexMesh(points, 32, params);
}

static PxConvexMesh* gConvexMesh = NULL;
static PxTriangleMesh* gTriangleMesh = NULL;

static CustomScene* gScene = NULL;

static const float gMeshScaleValue = 1.5f;

enum SceneIndex
{
	SCENE_FIRST		= 0,

	SCENE_ARAS		= 0,
	SCENE_BUNNY		= 1,
	SCENE_SPHERES	= 2,

	SCENE_COUNT
};

static PxU32 gSceneIndex = SCENE_BUNNY;

static void initScene()
{
	const PxVec3 red(1.0f, 0.25f, 0.2f);
	const PxVec3 green(0.5f, 1.0f, 0.4f);
	const PxVec3 blue(0.1f, 0.5f, 1.0f);
	const PxVec3 grey(0.5f);
	const PxVec3 noEmissive(0.0f);
	const PxVec3 smallEmissive(0.0f);
//	const PxVec3 bigEmissive(0.7f);
	const PxVec3 bigEmissive(70.0f);
	const PxVec3 color(1.0f, 0.5f, 0.25f);

#if OPTIM_BAKE_MESH_SCALE
	// PT: queries against scaled meshes are slightly slower, so we pre-scale the vertices
	const PxMeshScale meshScale(1.0f);
#else
	const PxMeshScale meshScale(gMeshScaleValue);
#endif

	gScene = new CustomScene;

	gSkyCoeff = 0.4f;
	gExposure = 0.5f;

	if(gSceneIndex==SCENE_ARAS)
	{
		gScene->addGeom(PxSphereGeometry(100.0f),	PxTransform(PxVec3(0,-100.5,-1)), PxVec3(0.8f, 0.8f, 0.8f), noEmissive);
		gScene->addGeom(PxSphereGeometry(0.5f),		PxTransform(PxVec3(2,0,-1)), PxVec3(0.8f, 0.4f, 0.4f), noEmissive);
		gScene->addGeom(PxSphereGeometry(0.5f),		PxTransform(PxVec3(0,0,-1)), PxVec3(0.4f, 0.8f, 0.4f), noEmissive);
		gScene->addGeom(PxSphereGeometry(0.5f),		PxTransform(PxVec3(-2,0,-1)), PxVec3(0.4f, 0.4f, 0.8f), noEmissive, Metal);
		gScene->addGeom(PxSphereGeometry(0.5f),		PxTransform(PxVec3(2,0,1)), PxVec3(0.4f, 0.8f, 0.4f), noEmissive, Metal);
		gScene->addGeom(PxSphereGeometry(0.5f),		PxTransform(PxVec3(0,0,1)), PxVec3(0.4f, 0.8f, 0.4f), noEmissive, Metal, 0.2f);
		gScene->addGeom(PxSphereGeometry(0.5f),		PxTransform(PxVec3(-2,0,1)), PxVec3(0.4f, 0.8f, 0.4f), noEmissive, Metal, 0.6f);
		gScene->addGeom(PxSphereGeometry(0.5f),		PxTransform(PxVec3(0.5f,1,0.5f)), PxVec3(0.4f, 0.4f, 0.4f), noEmissive, Dielectric, 0.0f, 1.5f);
		gScene->addGeom(PxSphereGeometry(0.3f),		PxTransform(PxVec3(-1.5f,1.5f,0.f)), PxVec3(0.8f, 0.6f, 0.2f), PxVec3(30,25,15));
	}
	else if(gSceneIndex==SCENE_BUNNY)
	{
		gScene->addGeom(PxSphereGeometry(1.0f),								PxTransform(PxVec3(0.0f, 8.0f, 0.0f)), grey, PxVec3(70,65,55));
//		gScene->addGeom(PxSphereGeometry(1.0f),								PxTransform(PxVec3(0.0f, 8.0f, 0.0f)), grey, PxVec3(7.0f,6.5f,5.5f));
//		gScene->addGeom(PxSphereGeometry(1.0f),								PxTransform(PxVec3(4.0f, 4.5f, 4.0f)), grey, bigEmissive);
//		gScene->addGeom(PxSphereGeometry(0.1f),								PxTransform(PxVec3(4.0f, 4.5f, 4.0f)), grey, bigEmissive);

		gScene->addGeom(PxBoxGeometry(PxVec3(15.0f, 0.01f, 15.0f)),			PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), grey, smallEmissive);

		gScene->addGeom(PxBoxGeometry(PxVec3(1.0f, 2.0f, 0.5f)),			PxTransform(PxVec3(0.0f, 2.0f, -4.0f)), color, smallEmissive);
		gScene->addGeom(PxSphereGeometry(1.5f),								PxTransform(PxVec3(4.0f, 1.55f, -4.0f)), grey, smallEmissive, Metal, 0.0f);
		gScene->addGeom(PxSphereGeometry(1.5f),								PxTransform(PxVec3(4.0f, 1.55f, 0.0f)), grey, smallEmissive, Metal, 0.1f);
		gScene->addGeom(PxSphereGeometry(1.5f),								PxTransform(PxVec3(4.0f, 1.55f, 4.0f)), grey, smallEmissive, Metal, 0.2f);
//		gScene->addGeom(PxCapsuleGeometry(1.0f, 1.0f),						PxTransform(PxVec3(-4.0f, 1.05f, 0.0f)), blue, smallEmissive, Dielectric, 0.0f, 1.2f);
		gScene->addGeom(PxCapsuleGeometry(1.0f, 1.0f),						PxTransform(PxVec3(-4.0f, 1.05f, 0.0f)), blue, smallEmissive);
		gScene->addGeom(PxConvexMeshGeometry(gConvexMesh),					PxTransform(PxVec3(0.0f, 1.05f, 4.0f)), green, smallEmissive);
		gScene->addGeom(PxTriangleMeshGeometry(gTriangleMesh, meshScale),	PxTransform(PxVec3(0.0f, 0.70f*gMeshScaleValue, 0.0f)), red, smallEmissive);
//		gScene->addGeom(PxBoxGeometry(PxVec3(1.0f)),						PxTransform(PxVec3(0.0f, 1.0f, 0.0f)), color, smallEmissive);
	}
	else if(gSceneIndex==SCENE_SPHERES)
	{
		gScene->addGeom(PxBoxGeometry(PxVec3(150.0f, 0.0001f, 150.0f)),			PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), grey, smallEmissive);
		gScene->addGeom(PxSphereGeometry(10.0f),								PxTransform(PxVec3(0.0f, 40.0f, 0.0f)), grey, PxVec3(100.0f));
		const PxU32 nb = 16;
		PxU32 state = 42;
//		const PxVec3 base(1.0f, 0.5f, 0.25f);
		const PxVec3 base(1.0f, 0.5f, 0.2f);
		for(PxU32 i=0;i<nb;i++)
		{
			const float CoeffX = float(i) - float(nb/2);
			for(PxU32 j=0;j<nb;j++)
			{
				const float CoeffZ = float(j) - float(nb/2);

				const float x = CoeffX * 4.0f;
				const float z = CoeffZ * 4.0f;

				PxVec3 rndColor(RandomFloat01(state), RandomFloat01(state), RandomFloat01(state));
				rndColor += base;
				rndColor *= 0.5f;

//				gScene->addGeom(PxSphereGeometry(1.0f), PxTransform(PxVec3(x, 1.0f, z)), rndColor, smallEmissive, Metal, 0.25f);
				gScene->addGeom(PxSphereGeometry(1.0f), PxTransform(PxVec3(x, 1.0f, z)), rndColor, smallEmissive, Metal, 0.0f);
//				gScene->addGeom(PxBoxGeometry(PxVec3(1.0f)), PxTransform(PxVec3(x, 1.0f, z)), rndColor, smallEmissive, Metal, 0.0f);
//				gScene->addGeom(PxTriangleMeshGeometry(gTriangleMesh, meshScale),	PxTransform(PxVec3(x, 0.75f*gMeshScaleValue, z)), rndColor, smallEmissive);
			}
		}
	}

	gScene->createBVH();
}

static void releaseScene()
{
	PX_RELEASE(gScene);
}

void renderScene()
{
	if(gScene)
		gScene->render();
}

static void threadExecute(void* data)
{
	TracerThread* tracerThread = static_cast<TracerThread*>(data);

	for(;;)
	{
		SnippetUtils::syncWait(tracerThread->mWorkReadySyncHandle);
		SnippetUtils::syncReset(tracerThread->mWorkReadySyncHandle);

		if (SnippetUtils::threadQuitIsSignalled(tracerThread->mThreadHandle))
			break;

		tracerThread->Run();
		SnippetUtils::syncSet(tracerThread->mWorkDoneSyncHandle);
	}

	SnippetUtils::threadQuit(tracerThread->mThreadHandle);
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	const PxTolerancesScale scale;
	PxCookingParams params(scale);
	params.midphaseDesc.setToDefault(PxMeshMidPhase::eBVH34);
//	params.midphaseDesc.mBVH34Desc.quantized = false;
	// We don't need the extra data structures for just doing raycasts vs the mesh
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	gConvexMesh = createCylinderMesh(3.0f, 1.0f, params);

	{
		PxTriangleMeshDesc meshDesc;
		meshDesc.points.count		= SnippetUtils::Bunny_getNbVerts();
		meshDesc.points.stride		= sizeof(PxVec3);
		meshDesc.points.data		= SnippetUtils::Bunny_getVerts();
		meshDesc.triangles.count	= SnippetUtils::Bunny_getNbFaces();
		meshDesc.triangles.stride	= sizeof(int)*3;
		meshDesc.triangles.data		= SnippetUtils::Bunny_getFaces();

#if OPTIM_BAKE_MESH_SCALE
		PxVec3* verts = const_cast<PxVec3*>(SnippetUtils::Bunny_getVerts());
		for(PxU32 i=0;i<meshDesc.points.count;i++)
			verts[i] *= gMeshScaleValue;
#endif

		gTriangleMesh = PxCreateTriangleMesh(params, meshDesc);
	}

	initScene();

#ifdef RENDER_SNIPPET
	Snippets::enableVSync(false);
#endif
	for (PxU32 i=0; i<gNumThreads; i++)
	{
		gThreads[i].mWorkReadySyncHandle = SnippetUtils::syncCreate();
		gThreads[i].mWorkDoneSyncHandle = SnippetUtils::syncCreate();
		gThreads[i].mThreadHandle = SnippetUtils::threadCreate(threadExecute, &gThreads[i]);
	}
}

void stepPhysics(bool /*interactive*/)
{
}

void cleanupPhysics(bool /*interactive*/)
{
	for (PxU32 i=0; i<gNumThreads; i++)
	{
		SnippetUtils::threadSignalQuit(gThreads[i].mThreadHandle);
		SnippetUtils::syncSet(gThreads[i].mWorkReadySyncHandle);
	}

	for (PxU32 i=0; i<gNumThreads; i++)
	{
		SnippetUtils::threadWaitForQuit(gThreads[i].mThreadHandle);
		SnippetUtils::threadRelease(gThreads[i].mThreadHandle);
		SnippetUtils::syncRelease(gThreads[i].mWorkReadySyncHandle);
	}

	for (PxU32 i=0; i<gNumThreads; i++)
		SnippetUtils::syncRelease(gThreads[i].mWorkDoneSyncHandle);

	releaseScene();

#ifdef RENDER_SNIPPET
	Snippets::ReleaseTexture(gTexID);
#endif
	PX_RELEASE(gConvexMesh);
	PX_RELEASE(gFoundation);

#ifdef RENDER_SNIPPET
	if(gCurrentTexture)
	{
		delete [] gCurrentTexture;
		gCurrentTexture = NULL;
	}
#endif
	if(gCurrentPixels)
	{
		delete [] gCurrentPixels;
		gCurrentPixels = NULL;
	}

	if(gAccumPixels)
	{
		delete [] gAccumPixels;
		gAccumPixels = NULL;
	}

	printf("SnippetPathTracing done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	if(key == 1)
	{
		gSceneIndex++;
		if(gSceneIndex==SCENE_COUNT)
			gSceneIndex = SCENE_FIRST;
		releaseScene();
		initScene();
		resetRender();
	}
	else if(key == 2)
	{
		if(gSceneIndex)
			gSceneIndex--;
		else
			gSceneIndex = SCENE_COUNT-1;
		releaseScene();
		initScene();
		resetRender();
	}
	else if(key == 3)
	{
		gSkyCoeff += 0.1f;
		printf("Sky coeff: %f\n", double(gSkyCoeff));
		resetRender();
	}
	else if(key == 4)
	{
		gSkyCoeff -= 0.1f;
		if(gSkyCoeff<0.0f)
			gSkyCoeff=0.0f;
		printf("Sky coeff: %f\n", double(gSkyCoeff));
		resetRender();
	}
	else if(key == 5)
	{
		gExposure += 0.1f;
		printf("Exposure: %f\n", double(gExposure));
		resetRender();
	}
	else if(key == 6)
	{
		gExposure -= 0.1f;
		if(gExposure<0.0f)
			gExposure=0.0f;
		printf("Exposure: %f\n", double(gExposure));
		resetRender();
	}
	else if(key == 'r' || key == 'R')
	{
		resetRender();
	}
	else if(key == 'g' || key == 'G')
	{
		gShowAllThreadRenders = !gShowAllThreadRenders;
		printf("Debug multithread: %d\n", gShowAllThreadRenders);
	}
	else if(key == 'm' || key == 'M')
	{
		gUseMultipleThreads = !gUseMultipleThreads;
		printf("Multithreading: %d\n", gUseMultipleThreads);
	}
	else if(key == 't' || key == 'T')
	{
		gToneMapping = !gToneMapping;
		printf("Tone mapping: %d\n", gToneMapping);
	}
	else if(key == 'l' || key == 'L')
	{
		gLinearToSRGB = !gLinearToSRGB;
		printf("Linear-to-SRGB: %d\n", gLinearToSRGB);
	}
}

int snippetMain(int, const char*const*)
{
	printf("PxBVH PathTracing snippet. Use these keys:\n");
	printf(" F1/F2 - change scene\n");
	printf(" F3/F4 - change sky color intensity\n");
	printf(" F5/F6 - change exposure value\n");
	printf(" m     - multithreading on/off\n");
	printf(" t     - tone mapping on/off\n");
	printf(" l     - linear to SRGB/off\n");
	printf(" r     - reset scene\n");
	printf(" g     - debug multithreading on/off\n");
	printf("\n");

#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}




