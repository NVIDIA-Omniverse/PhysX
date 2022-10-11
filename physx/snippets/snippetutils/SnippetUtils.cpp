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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

#include "SnippetUtils.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxMat33.h"
#include "foundation/PxQuat.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxMutex.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxString.h"
#include "foundation/PxSync.h"
#include "foundation/PxThread.h"
#include "foundation/PxTime.h"

#include "extensions/PxDefaultAllocator.h"

#if PX_XBOX_SERIES_X
#pragma warning(disable : 4305)
#endif

namespace physx
{

namespace
{
PxDefaultAllocator gUtilAllocator;

struct UtilAllocator // since we're allocating internal classes here, make sure we align properly
{
	void* allocate(size_t size,const char* file,  PxU32 line) 	{ return gUtilAllocator.allocate(size, NULL, file, int(line));		}
	void deallocate(void* ptr)									{ gUtilAllocator.deallocate(ptr);								}
};
}

namespace SnippetUtils
{
	void setRotX(PxMat33& m, PxReal angle)
	{
		m = PxMat33(PxIdentity);

		const PxReal cos = cosf(angle);
		const PxReal sin = sinf(angle);

		m[1][1] = m[2][2] = cos;
		m[1][2] = sin;
		m[2][1] = -sin;
	}

	void setRotY(PxMat33& m, PxReal angle)
	{
		m = PxMat33(PxIdentity);

		const PxReal cos = cosf(angle);
		const PxReal sin = sinf(angle);

		m[0][0] = m[2][2] = cos;
		m[0][2] = -sin;
		m[2][0] = sin;
	}

	void setRotZ(PxMat33& m, PxReal angle)
	{
		m = PxMat33(PxIdentity);

		const PxReal cos = cosf(angle);
		const PxReal sin = sinf(angle);

		m[0][0] = m[1][1] = cos;
		m[0][1] = sin;
		m[1][0] = -sin;
	}

	PxQuat getRotXQuat(float angle)
	{
		PxMat33 m;
		setRotX(m, angle);
		return PxQuat(m);
	}

	PxQuat getRotYQuat(float angle)
	{
		PxMat33 m;
		setRotY(m, angle);
		return PxQuat(m);
	}

	PxQuat getRotZQuat(float angle)
	{
		PxMat33 m;
		setRotZ(m, angle);
		return PxQuat(m);
	}

	PxVec3 BasicRandom::unitRandomPt()
	{
		PxVec3 v;
		do
		{
			v.x = randomFloat();
			v.y = randomFloat();
			v.z = randomFloat();
		}
		while(v.normalize()<1e-6f);
		return v;
	}

	PxQuat BasicRandom::unitRandomQuat()
	{
		PxQuat v;
		do
		{
			v.x = randomFloat();
			v.y = randomFloat();
			v.z = randomFloat();
			v.w = randomFloat();
		}
		while(v.normalize()<1e-6f);

		return v;
	}

	void BasicRandom::unitRandomPt(PxVec3& v)
	{
		v = unitRandomPt();
	}
	void BasicRandom::unitRandomQuat(PxQuat& v)
	{
		v = unitRandomQuat();
	}

	PxI32 atomicIncrement(volatile PxI32* val)
	{
		return PxAtomicIncrement(val);
	}

	PxI32 atomicDecrement(volatile PxI32* val)
	{
		return PxAtomicDecrement(val);
	}

	//******************************************************************************//

	PxU32 getNbPhysicalCores()
	{
		return PxThread::getNbPhysicalCores();
	}

	//******************************************************************************//

	PxU32 getThreadId()
	{
		return static_cast<PxU32>(PxThread::getId());
	}

	//******************************************************************************//

	PxU64 getCurrentTimeCounterValue()
	{
		return PxTime::getCurrentCounterValue();
	}

	PxReal getElapsedTimeInMilliseconds(const PxU64 elapsedTime)
	{
		return PxTime::getCounterFrequency().toTensOfNanos(elapsedTime)/(100.0f * 1000.0f);
	}

	PxReal getElapsedTimeInMicroSeconds(const PxU64 elapsedTime)
	{
		return PxTime::getCounterFrequency().toTensOfNanos(elapsedTime)/(100.0f);
	}

	//******************************************************************************//

	struct Sync: public PxSyncT<UtilAllocator> {};

	Sync* syncCreate()
	{
		return new(gUtilAllocator.allocate(sizeof(Sync), 0, 0, 0)) Sync();
	}

	void syncWait(Sync* sync)
	{
		sync->wait();
	}

	void syncSet(Sync* sync)
	{
		sync->set();
	}

	void syncReset(Sync* sync)
	{
		sync->reset();
	}
	
	void syncRelease(Sync* sync)
	{
		sync->~Sync();
		gUtilAllocator.deallocate(sync);
	}

	//******************************************************************************//

	struct Thread: public PxThreadT<UtilAllocator>
	{
		Thread(ThreadEntryPoint entryPoint, void* data): 
			PxThreadT<UtilAllocator>(),
			mEntryPoint(entryPoint),
			mData(data)
		{
		}

		virtual void execute(void)											
		{ 
			mEntryPoint(mData);
		}

		ThreadEntryPoint mEntryPoint;
		void* mData;
	};

	Thread* threadCreate(ThreadEntryPoint entryPoint, void* data)
	{
		Thread* createThread = static_cast<Thread*>(gUtilAllocator.allocate(sizeof(Thread), 0, 0, 0));
		PX_PLACEMENT_NEW(createThread, Thread(entryPoint, data));
		createThread->start();
		return createThread;
	}

	void threadQuit(Thread* thread)
	{
		thread->quit();
	}

	void threadSignalQuit(Thread* thread)
	{
		thread->signalQuit();
	}

	bool threadWaitForQuit(Thread* thread)
	{
		return thread->waitForQuit();
	}

	bool threadQuitIsSignalled(Thread* thread)
	{
		return thread->quitIsSignalled();
	}

	void threadRelease(Thread* thread)
	{
		thread->~Thread();
		gUtilAllocator.deallocate(thread);
	}

	//******************************************************************************//

	struct Mutex: public PxMutexT<UtilAllocator> {};

	Mutex* mutexCreate()
	{
		return new(gUtilAllocator.allocate(sizeof(Mutex), 0, 0, 0)) Mutex();
	}

	void mutexLock(Mutex* mutex)
	{
		mutex->lock();
	}

	void mutexUnlock(Mutex* mutex)
	{
		mutex->unlock();
	}

	void mutexRelease(Mutex* mutex)
	{
		mutex->~Mutex();
		gUtilAllocator.deallocate(mutex);
	}


} // namespace physXUtils
} // namespace physx


using namespace physx;

#define BUNNY_NBVERTICES	(453)
#define BUNNY_NBFACES		(902)

static float gBunnyVertices[BUNNY_NBVERTICES][3]={
	{-0.334392,0.133007,0.062259},
	{-0.350189,0.150354,-0.147769},
	{-0.234201,0.343811,-0.174307},
	{-0.200259,0.285207,0.093749},
	{0.003520,0.475208,-0.159365},
	{0.001856,0.419203,0.098582},
	{-0.252802,0.093666,0.237538},
	{-0.162901,0.237984,0.206905},
	{0.000865,0.318141,0.235370},
	{-0.414624,0.164083,-0.278254},
	{-0.262213,0.357334,-0.293246},
	{0.004628,0.482694,-0.338626},
	{-0.402162,0.133528,-0.443247},
	{-0.243781,0.324275,-0.436763},
	{0.005293,0.437592,-0.458332},
	{-0.339884,-0.041150,-0.668211},
	{-0.248382,0.255825,-0.627493},
	{0.006261,0.376103,-0.631506},
	{-0.216201,-0.126776,-0.886936},
	{-0.171075,0.011544,-0.881386},
	{-0.181074,0.098223,-0.814779},
	{-0.119891,0.218786,-0.760153},
	{-0.078895,0.276780,-0.739281},
	{0.006801,0.310959,-0.735661},
	{-0.168842,0.102387,-0.920381},
	{-0.104072,0.177278,-0.952530},
	{-0.129704,0.211848,-0.836678},
	{-0.099875,0.310931,-0.799381},
	{0.007237,0.361687,-0.794439},
	{-0.077913,0.258753,-0.921640},
	{0.007957,0.282241,-0.931680},
	{-0.252222,-0.550401,-0.557810},
	{-0.267633,-0.603419,-0.655209},
	{-0.446838,-0.118517,-0.466159},
	{-0.459488,-0.093017,-0.311341},
	{-0.370645,-0.100108,-0.159454},
	{-0.371984,-0.091991,-0.011044},
	{-0.328945,-0.098269,0.088659},
	{-0.282452,-0.018862,0.311501},
	{-0.352403,-0.131341,0.144902},
	{-0.364126,-0.200299,0.202388},
	{-0.283965,-0.231869,0.023668},
	{-0.298943,-0.155218,0.369716},
	{-0.293787,-0.121856,0.419097},
	{-0.290163,-0.290797,0.107824},
	{-0.264165,-0.272849,0.036347},
	{-0.228567,-0.372573,0.290309},
	{-0.190431,-0.286997,0.421917},
	{-0.191039,-0.240973,0.507118},
	{-0.287272,-0.276431,-0.065444},
	{-0.295675,-0.280818,-0.174200},
	{-0.399537,-0.313131,-0.376167},
	{-0.392666,-0.488581,-0.427494},
	{-0.331669,-0.570185,-0.466054},
	{-0.282290,-0.618140,-0.589220},
	{-0.374238,-0.594882,-0.323298},
	{-0.381071,-0.629723,-0.350777},
	{-0.382112,-0.624060,-0.221577},
	{-0.272701,-0.566522,0.259157},
	{-0.256702,-0.663406,0.286079},
	{-0.280948,-0.428359,0.055790},
	{-0.184974,-0.508894,0.326265},
	{-0.279971,-0.526918,0.395319},
	{-0.282599,-0.663393,0.412411},
	{-0.188329,-0.475093,0.417954},
	{-0.263384,-0.663396,0.466604},
	{-0.209063,-0.663393,0.509344},
	{-0.002044,-0.319624,0.553078},
	{-0.001266,-0.371260,0.413296},
	{-0.219753,-0.339762,-0.040921},
	{-0.256986,-0.282511,-0.006349},
	{-0.271706,-0.260881,0.001764},
	{-0.091191,-0.419184,-0.045912},
	{-0.114944,-0.429752,-0.124739},
	{-0.113970,-0.382987,-0.188540},
	{-0.243012,-0.464942,-0.242850},
	{-0.314815,-0.505402,-0.324768},
	{0.002774,-0.437526,-0.262766},
	{-0.072625,-0.417748,-0.221440},
	{-0.160112,-0.476932,-0.293450},
	{0.003859,-0.453425,-0.443916},
	{-0.120363,-0.581567,-0.438689},
	{-0.091499,-0.584191,-0.294511},
	{-0.116469,-0.599861,-0.188308},
	{-0.208032,-0.513640,-0.134649},
	{-0.235749,-0.610017,-0.040939},
	{-0.344916,-0.622487,-0.085380},
	{-0.336401,-0.531864,-0.212298},
	{0.001961,-0.459550,-0.135547},
	{-0.058296,-0.430536,-0.043440},
	{0.001378,-0.449511,-0.037762},
	{-0.130135,-0.510222,0.079144},
	{0.000142,-0.477549,0.157064},
	{-0.114284,-0.453206,0.304397},
	{-0.000592,-0.443558,0.285401},
	{-0.056215,-0.663402,0.326073},
	{-0.026248,-0.568010,0.273318},
	{-0.049261,-0.531064,0.389854},
	{-0.127096,-0.663398,0.479316},
	{-0.058384,-0.663401,0.372891},
	{-0.303961,0.054199,0.625921},
	{-0.268594,0.193403,0.502766},
	{-0.277159,0.126123,0.443289},
	{-0.287605,-0.005722,0.531844},
	{-0.231396,-0.121289,0.587387},
	{-0.253475,-0.081797,0.756541},
	{-0.195164,-0.137969,0.728011},
	{-0.167673,-0.156573,0.609388},
	{-0.145917,-0.169029,0.697600},
	{-0.077776,-0.214247,0.622586},
	{-0.076873,-0.214971,0.696301},
	{-0.002341,-0.233135,0.622859},
	{-0.002730,-0.213526,0.691267},
	{-0.003136,-0.192628,0.762731},
	{-0.056136,-0.201222,0.763806},
	{-0.114589,-0.166192,0.770723},
	{-0.155145,-0.129632,0.791738},
	{-0.183611,-0.058705,0.847012},
	{-0.165562,0.001980,0.833386},
	{-0.220084,0.019914,0.768935},
	{-0.255730,0.090306,0.670782},
	{-0.255594,0.113833,0.663389},
	{-0.226380,0.212655,0.617740},
	{-0.003367,-0.195342,0.799680},
	{-0.029743,-0.210508,0.827180},
	{-0.003818,-0.194783,0.873636},
	{-0.004116,-0.157907,0.931268},
	{-0.031280,-0.184555,0.889476},
	{-0.059885,-0.184448,0.841330},
	{-0.135333,-0.164332,0.878200},
	{-0.085574,-0.170948,0.925547},
	{-0.163833,-0.094170,0.897114},
	{-0.138444,-0.104250,0.945975},
	{-0.083497,-0.084934,0.979607},
	{-0.004433,-0.146642,0.985872},
	{-0.150715,0.032650,0.884111},
	{-0.135892,-0.035520,0.945455},
	{-0.070612,0.036849,0.975733},
	{-0.004458,-0.042526,1.015670},
	{-0.004249,0.046042,1.003240},
	{-0.086969,0.133224,0.947633},
	{-0.003873,0.161605,0.970499},
	{-0.125544,0.140012,0.917678},
	{-0.125651,0.250246,0.857602},
	{-0.003127,0.284070,0.878870},
	{-0.159174,0.125726,0.888878},
	{-0.183807,0.196970,0.844480},
	{-0.159890,0.291736,0.732480},
	{-0.199495,0.207230,0.779864},
	{-0.206182,0.164608,0.693257},
	{-0.186315,0.160689,0.817193},
	{-0.192827,0.166706,0.782271},
	{-0.175112,0.110008,0.860621},
	{-0.161022,0.057420,0.855111},
	{-0.172319,0.036155,0.816189},
	{-0.190318,0.064083,0.760605},
	{-0.195072,0.129179,0.731104},
	{-0.203126,0.410287,0.680536},
	{-0.216677,0.309274,0.642272},
	{-0.241515,0.311485,0.587832},
	{-0.002209,0.366663,0.749413},
	{-0.088230,0.396265,0.678635},
	{-0.170147,0.109517,0.840784},
	{-0.160521,0.067766,0.830650},
	{-0.181546,0.139805,0.812146},
	{-0.180495,0.148568,0.776087},
	{-0.180255,0.129125,0.744192},
	{-0.186298,0.078308,0.769352},
	{-0.167622,0.060539,0.806675},
	{-0.189876,0.102760,0.802582},
	{-0.108340,0.455446,0.657174},
	{-0.241585,0.527592,0.669296},
	{-0.265676,0.513366,0.634594},
	{-0.203073,0.478550,0.581526},
	{-0.266772,0.642330,0.602061},
	{-0.216961,0.564846,0.535435},
	{-0.202210,0.525495,0.475944},
	{-0.193888,0.467925,0.520606},
	{-0.265837,0.757267,0.500933},
	{-0.240306,0.653440,0.463215},
	{-0.309239,0.776868,0.304726},
	{-0.271009,0.683094,0.382018},
	{-0.312111,0.671099,0.286687},
	{-0.268791,0.624342,0.377231},
	{-0.302457,0.533996,0.360289},
	{-0.263656,0.529310,0.412564},
	{-0.282311,0.415167,0.447666},
	{-0.239201,0.442096,0.495604},
	{-0.220043,0.569026,0.445877},
	{-0.001263,0.395631,0.602029},
	{-0.057345,0.442535,0.572224},
	{-0.088927,0.506333,0.529106},
	{-0.125738,0.535076,0.612913},
	{-0.126251,0.577170,0.483159},
	{-0.149594,0.611520,0.557731},
	{-0.163188,0.660791,0.491080},
	{-0.172482,0.663387,0.415416},
	{-0.160464,0.591710,0.370659},
	{-0.156445,0.536396,0.378302},
	{-0.136496,0.444358,0.425226},
	{-0.095564,0.373768,0.473659},
	{-0.104146,0.315912,0.498104},
	{-0.000496,0.384194,0.473817},
	{-0.000183,0.297770,0.401486},
	{-0.129042,0.270145,0.434495},
	{0.000100,0.272963,0.349138},
	{-0.113060,0.236984,0.385554},
	{0.007260,0.016311,-0.883396},
	{0.007865,0.122104,-0.956137},
	{-0.032842,0.115282,-0.953252},
	{-0.089115,0.108449,-0.950317},
	{-0.047440,0.014729,-0.882756},
	{-0.104458,0.013137,-0.882070},
	{-0.086439,-0.584866,-0.608343},
	{-0.115026,-0.662605,-0.436732},
	{-0.071683,-0.665372,-0.606385},
	{-0.257884,-0.665381,-0.658052},
	{-0.272542,-0.665381,-0.592063},
	{-0.371322,-0.665382,-0.353620},
	{-0.372362,-0.665381,-0.224420},
	{-0.335166,-0.665380,-0.078623},
	{-0.225999,-0.665375,-0.038981},
	{-0.106719,-0.665374,-0.186351},
	{-0.081749,-0.665372,-0.292554},
	{0.006943,-0.091505,-0.858354},
	{0.006117,-0.280985,-0.769967},
	{0.004495,-0.502360,-0.559799},
	{-0.198638,-0.302135,-0.845816},
	{-0.237395,-0.542544,-0.587188},
	{-0.270001,-0.279489,-0.669861},
	{-0.134547,-0.119852,-0.959004},
	{-0.052088,-0.122463,-0.944549},
	{-0.124463,-0.293508,-0.899566},
	{-0.047616,-0.289643,-0.879292},
	{-0.168595,-0.529132,-0.654931},
	{-0.099793,-0.515719,-0.645873},
	{-0.186168,-0.605282,-0.724690},
	{-0.112970,-0.583097,-0.707469},
	{-0.108152,-0.665375,-0.700408},
	{-0.183019,-0.665378,-0.717630},
	{-0.349529,-0.334459,-0.511985},
	{-0.141182,-0.437705,-0.798194},
	{-0.212670,-0.448725,-0.737447},
	{-0.261111,-0.414945,-0.613835},
	{-0.077364,-0.431480,-0.778113},
	{0.005174,-0.425277,-0.651592},
	{0.089236,-0.431732,-0.777093},
	{0.271006,-0.415749,-0.610577},
	{0.223981,-0.449384,-0.734774},
	{0.153275,-0.438150,-0.796391},
	{0.358414,-0.335529,-0.507649},
	{0.193434,-0.665946,-0.715325},
	{0.118363,-0.665717,-0.699021},
	{0.123515,-0.583454,-0.706020},
	{0.196851,-0.605860,-0.722345},
	{0.109788,-0.516035,-0.644590},
	{0.178656,-0.529656,-0.652804},
	{0.061157,-0.289807,-0.878626},
	{0.138234,-0.293905,-0.897958},
	{0.066933,-0.122643,-0.943820},
	{0.149571,-0.120281,-0.957264},
	{0.280989,-0.280321,-0.666487},
	{0.246581,-0.543275,-0.584224},
	{0.211720,-0.302754,-0.843303},
	{0.086966,-0.665627,-0.291520},
	{0.110634,-0.665702,-0.185021},
	{0.228099,-0.666061,-0.036201},
	{0.337743,-0.666396,-0.074503},
	{0.376722,-0.666513,-0.219833},
	{0.377265,-0.666513,-0.349036},
	{0.281411,-0.666217,-0.588670},
	{0.267564,-0.666174,-0.654834},
	{0.080745,-0.665602,-0.605452},
	{0.122016,-0.662963,-0.435280},
	{0.095767,-0.585141,-0.607228},
	{0.118944,0.012799,-0.880702},
	{0.061944,0.014564,-0.882086},
	{0.104725,0.108156,-0.949130},
	{0.048513,0.115159,-0.952753},
	{0.112696,0.236643,0.386937},
	{0.128177,0.269757,0.436071},
	{0.102643,0.315600,0.499370},
	{0.094535,0.373481,0.474824},
	{0.136270,0.443946,0.426895},
	{0.157071,0.535923,0.380222},
	{0.161350,0.591224,0.372630},
	{0.173035,0.662865,0.417531},
	{0.162808,0.660299,0.493077},
	{0.148250,0.611070,0.559555},
	{0.125719,0.576790,0.484702},
	{0.123489,0.534699,0.614440},
	{0.087621,0.506066,0.530188},
	{0.055321,0.442365,0.572915},
	{0.219936,0.568361,0.448571},
	{0.238099,0.441375,0.498528},
	{0.281711,0.414315,0.451121},
	{0.263833,0.528513,0.415794},
	{0.303284,0.533081,0.363998},
	{0.269687,0.623528,0.380528},
	{0.314255,0.670153,0.290524},
	{0.272023,0.682273,0.385343},
	{0.311480,0.775931,0.308527},
	{0.240239,0.652714,0.466159},
	{0.265619,0.756464,0.504187},
	{0.192562,0.467341,0.522972},
	{0.201605,0.524885,0.478417},
	{0.215743,0.564193,0.538084},
	{0.264969,0.641527,0.605317},
	{0.201031,0.477940,0.584002},
	{0.263086,0.512567,0.637832},
	{0.238615,0.526867,0.672237},
	{0.105309,0.455123,0.658482},
	{0.183993,0.102195,0.804872},
	{0.161563,0.060042,0.808692},
	{0.180748,0.077754,0.771600},
	{0.175168,0.128588,0.746368},
	{0.175075,0.148030,0.778264},
	{0.175658,0.139265,0.814333},
	{0.154191,0.067291,0.832578},
	{0.163818,0.109013,0.842830},
	{0.084760,0.396004,0.679695},
	{0.238888,0.310760,0.590775},
	{0.213380,0.308625,0.644905},
	{0.199666,0.409678,0.683003},
	{0.190143,0.128597,0.733463},
	{0.184833,0.063516,0.762902},
	{0.166070,0.035644,0.818261},
	{0.154361,0.056943,0.857042},
	{0.168542,0.109489,0.862725},
	{0.187387,0.166131,0.784599},
	{0.180428,0.160135,0.819438},
	{0.201823,0.163991,0.695756},
	{0.194206,0.206635,0.782275},
	{0.155438,0.291260,0.734412},
	{0.177696,0.196424,0.846693},
	{0.152305,0.125256,0.890786},
	{0.119546,0.249876,0.859104},
	{0.118369,0.139643,0.919173},
	{0.079410,0.132973,0.948652},
	{0.062419,0.036648,0.976547},
	{0.127847,-0.035919,0.947070},
	{0.143624,0.032206,0.885913},
	{0.074888,-0.085173,0.980577},
	{0.130184,-0.104656,0.947620},
	{0.156201,-0.094653,0.899074},
	{0.077366,-0.171194,0.926545},
	{0.127722,-0.164729,0.879810},
	{0.052670,-0.184618,0.842019},
	{0.023477,-0.184638,0.889811},
	{0.022626,-0.210587,0.827500},
	{0.223089,0.211976,0.620493},
	{0.251444,0.113067,0.666494},
	{0.251419,0.089540,0.673887},
	{0.214360,0.019258,0.771595},
	{0.158999,0.001490,0.835374},
	{0.176696,-0.059249,0.849218},
	{0.148696,-0.130091,0.793599},
	{0.108290,-0.166528,0.772088},
	{0.049820,-0.201382,0.764454},
	{0.071341,-0.215195,0.697209},
	{0.073148,-0.214475,0.623510},
	{0.140502,-0.169461,0.699354},
	{0.163374,-0.157073,0.611416},
	{0.189466,-0.138550,0.730366},
	{0.247593,-0.082554,0.759610},
	{0.227468,-0.121982,0.590197},
	{0.284702,-0.006586,0.535347},
	{0.275741,0.125287,0.446676},
	{0.266650,0.192594,0.506044},
	{0.300086,0.053287,0.629620},
	{0.055450,-0.663935,0.375065},
	{0.122854,-0.664138,0.482323},
	{0.046520,-0.531571,0.391918},
	{0.024824,-0.568450,0.275106},
	{0.053855,-0.663931,0.328224},
	{0.112829,-0.453549,0.305788},
	{0.131265,-0.510617,0.080746},
	{0.061174,-0.430716,-0.042710},
	{0.341019,-0.532887,-0.208150},
	{0.347705,-0.623533,-0.081139},
	{0.238040,-0.610732,-0.038037},
	{0.211764,-0.514274,-0.132078},
	{0.120605,-0.600219,-0.186856},
	{0.096985,-0.584476,-0.293357},
	{0.127621,-0.581941,-0.437170},
	{0.165902,-0.477425,-0.291453},
	{0.077720,-0.417975,-0.220519},
	{0.320892,-0.506363,-0.320874},
	{0.248214,-0.465684,-0.239842},
	{0.118764,-0.383338,-0.187114},
	{0.118816,-0.430106,-0.123307},
	{0.094131,-0.419464,-0.044777},
	{0.274526,-0.261706,0.005110},
	{0.259842,-0.283292,-0.003185},
	{0.222861,-0.340431,-0.038210},
	{0.204445,-0.664380,0.513353},
	{0.259286,-0.664547,0.471281},
	{0.185402,-0.476020,0.421718},
	{0.279163,-0.664604,0.417328},
	{0.277157,-0.528122,0.400208},
	{0.183069,-0.509812,0.329995},
	{0.282599,-0.429210,0.059242},
	{0.254816,-0.664541,0.290687},
	{0.271436,-0.567707,0.263966},
	{0.386561,-0.625221,-0.216870},
	{0.387086,-0.630883,-0.346073},
	{0.380021,-0.596021,-0.318679},
	{0.291269,-0.619007,-0.585707},
	{0.339280,-0.571198,-0.461946},
	{0.400045,-0.489778,-0.422640},
	{0.406817,-0.314349,-0.371230},
	{0.300588,-0.281718,-0.170549},
	{0.290866,-0.277304,-0.061905},
	{0.187735,-0.241545,0.509437},
	{0.188032,-0.287569,0.424234},
	{0.227520,-0.373262,0.293102},
	{0.266526,-0.273650,0.039597},
	{0.291592,-0.291676,0.111386},
	{0.291914,-0.122741,0.422683},
	{0.297574,-0.156119,0.373368},
	{0.286603,-0.232731,0.027162},
	{0.364663,-0.201399,0.206850},
	{0.353855,-0.132408,0.149228},
	{0.282208,-0.019715,0.314960},
	{0.331187,-0.099266,0.092701},
	{0.375463,-0.093120,-0.006467},
	{0.375917,-0.101236,-0.154882},
	{0.466635,-0.094416,-0.305669},
	{0.455805,-0.119881,-0.460632},
	{0.277465,-0.604242,-0.651871},
	{0.261022,-0.551176,-0.554667},
	{0.093627,0.258494,-0.920589},
	{0.114248,0.310608,-0.798070},
	{0.144232,0.211434,-0.835001},
	{0.119916,0.176940,-0.951159},
	{0.184061,0.101854,-0.918220},
	{0.092431,0.276521,-0.738231},
	{0.133504,0.218403,-0.758602},
	{0.194987,0.097655,-0.812476},
	{0.185542,0.011005,-0.879202},
	{0.230315,-0.127450,-0.884202},
	{0.260471,0.255056,-0.624378},
	{0.351567,-0.042194,-0.663976},
	{0.253742,0.323524,-0.433716},
	{0.411612,0.132299,-0.438264},
	{0.270513,0.356530,-0.289984},
	{0.422146,0.162819,-0.273130},
	{0.164724,0.237490,0.208912},
	{0.253806,0.092900,0.240640},
	{0.203608,0.284597,0.096223},
	{0.241006,0.343093,-0.171396},
	{0.356076,0.149288,-0.143443},
	{0.337656,0.131992,0.066374}
};

static unsigned int gBunnyTriangles[BUNNY_NBFACES][3]={
	{126,134,133},
	{342,138,134},
	{133,134,138},
	{126,342,134},
	{312,316,317},
	{169,163,162},
	{312,317,319},
	{312,319,318},
	{169,162,164},
	{169,168,163},
	{312,314,315},
	{169,164,165},
	{169,167,168},
	{312,315,316},
	{312,313,314},
	{169,165,166},
	{169,166,167},
	{312,318,313},
	{308,304,305},
	{308,305,306},
	{179,181,188},
	{177,173,175},
	{177,175,176},
	{302,293,300},
	{322,294,304},
	{188,176,175},
	{188,175,179},
	{158,177,187},
	{305,293,302},
	{305,302,306},
	{322,304,308},
	{188,181,183},
	{158,173,177},
	{293,298,300},
	{304,294,296},
	{304,296,305},
	{185,176,188},
	{185,188,183},
	{187,177,176},
	{187,176,185},
	{305,296,298},
	{305,298,293},
	{436,432, 28},
	{436, 28, 23},
	{434,278,431},
	{ 30,208,209},
	{ 30,209, 29},
	{ 19, 20, 24},
	{208,207,211},
	{208,211,209},
	{ 19,210,212},
	{433,434,431},
	{433,431,432},
	{433,432,436},
	{436,437,433},
	{277,275,276},
	{277,276,278},
	{209,210, 25},
	{ 21, 26, 24},
	{ 21, 24, 20},
	{ 25, 26, 27},
	{ 25, 27, 29},
	{435,439,277},
	{439,275,277},
	{432,431, 30},
	{432, 30, 28},
	{433,437,438},
	{433,438,435},
	{434,277,278},
	{ 24, 25,210},
	{ 24, 26, 25},
	{ 29, 27, 28},
	{ 29, 28, 30},
	{ 19, 24,210},
	{208, 30,431},
	{208,431,278},
	{435,434,433},
	{435,277,434},
	{ 25, 29,209},
	{ 27, 22, 23},
	{ 27, 23, 28},
	{ 26, 22, 27},
	{ 26, 21, 22},
	{212,210,209},
	{212,209,211},
	{207,208,278},
	{207,278,276},
	{439,435,438},
	{ 12,  9, 10},
	{ 12, 10, 13},
	{  2,  3,  5},
	{  2,  5,  4},
	{ 16, 13, 14},
	{ 16, 14, 17},
	{ 22, 21, 16},
	{ 13, 10, 11},
	{ 13, 11, 14},
	{  1,  0,  3},
	{  1,  3,  2},
	{ 15, 12, 16},
	{ 19, 18, 15},
	{ 19, 15, 16},
	{ 19, 16, 20},
	{  9,  1,  2},
	{  9,  2, 10},
	{  3,  7,  8},
	{  3,  8,  5},
	{ 16, 17, 23},
	{ 16, 23, 22},
	{ 21, 20, 16},
	{ 10,  2,  4},
	{ 10,  4, 11},
	{  0,  6,  7},
	{  0,  7,  3},
	{ 12, 13, 16},
	{451,446,445},
	{451,445,450},
	{442,440,439},
	{442,439,438},
	{442,438,441},
	{421,420,422},
	{412,411,426},
	{412,426,425},
	{408,405,407},
	{413, 67, 68},
	{413, 68,414},
	{391,390,412},
	{ 80,384,386},
	{404,406,378},
	{390,391,377},
	{390,377, 88},
	{400,415,375},
	{398,396,395},
	{398,395,371},
	{398,371,370},
	{112,359,358},
	{112,358,113},
	{351,352,369},
	{125,349,348},
	{345,343,342},
	{342,340,339},
	{341,335,337},
	{328,341,327},
	{331,323,333},
	{331,322,323},
	{327,318,319},
	{327,319,328},
	{315,314,324},
	{302,300,301},
	{302,301,303},
	{320,311,292},
	{285,284,289},
	{310,307,288},
	{310,288,290},
	{321,350,281},
	{321,281,282},
	{423,448,367},
	{272,273,384},
	{272,384,274},
	{264,265,382},
	{264,382,383},
	{440,442,261},
	{440,261,263},
	{252,253,254},
	{252,254,251},
	{262,256,249},
	{262,249,248},
	{228,243,242},
	{228, 31,243},
	{213,215,238},
	{213,238,237},
	{ 19,212,230},
	{224,225,233},
	{224,233,231},
	{217,218, 56},
	{217, 56, 54},
	{217,216,239},
	{217,239,238},
	{217,238,215},
	{218,217,215},
	{218,215,214},
	{  6,102,206},
	{186,199,200},
	{197,182,180},
	{170,171,157},
	{201,200,189},
	{170,190,191},
	{170,191,192},
	{175,174,178},
	{175,178,179},
	{168,167,155},
	{122,149,158},
	{122,158,159},
	{135,153,154},
	{135,154,118},
	{143,140,141},
	{143,141,144},
	{132,133,136},
	{130,126,133},
	{124,125,127},
	{122,101,100},
	{122,100,121},
	{110,108,107},
	{110,107,109},
	{ 98, 99, 97},
	{ 98, 97, 64},
	{ 98, 64, 66},
	{ 87, 55, 57},
	{ 83, 82, 79},
	{ 83, 79, 84},
	{ 78, 74, 50},
	{ 49, 71, 41},
	{ 49, 41, 37},
	{ 49, 37, 36},
	{ 58, 44, 60},
	{ 60, 59, 58},
	{ 51, 34, 33},
	{ 39, 40, 42},
	{ 39, 42, 38},
	{243,240, 33},
	{243, 33,229},
	{ 39, 38,  6},
	{ 44, 46, 40},
	{ 55, 56, 57},
	{ 64, 62, 65},
	{ 64, 65, 66},
	{ 41, 71, 45},
	{ 75, 50, 51},
	{ 81, 79, 82},
	{ 77, 88, 73},
	{ 93, 92, 94},
	{ 68, 47, 46},
	{ 96, 97, 99},
	{ 96, 99, 95},
	{110,109,111},
	{111,112,110},
	{114,113,123},
	{114,123,124},
	{132,131,129},
	{133,137,136},
	{135,142,145},
	{145,152,135},
	{149,147,157},
	{157,158,149},
	{164,150,151},
	{153,163,168},
	{153,168,154},
	{185,183,182},
	{185,182,184},
	{161,189,190},
	{200,199,191},
	{200,191,190},
	{180,178,195},
	{180,195,196},
	{102,101,204},
	{102,204,206},
	{ 43, 48,104},
	{ 43,104,103},
	{216,217, 54},
	{216, 54, 32},
	{207,224,231},
	{230,212,211},
	{230,211,231},
	{227,232,241},
	{227,241,242},
	{235,234,241},
	{235,241,244},
	{430,248,247},
	{272,274,253},
	{272,253,252},
	{439,260,275},
	{225,224,259},
	{225,259,257},
	{269,270,407},
	{269,407,405},
	{270,269,273},
	{270,273,272},
	{273,269,268},
	{273,268,267},
	{273,267,266},
	{273,266,265},
	{273,265,264},
	{448,279,367},
	{281,350,368},
	{285,286,301},
	{290,323,310},
	{290,311,323},
	{282,281,189},
	{292,311,290},
	{292,290,291},
	{307,306,302},
	{307,302,303},
	{316,315,324},
	{316,324,329},
	{331,351,350},
	{330,334,335},
	{330,335,328},
	{341,337,338},
	{344,355,354},
	{346,345,348},
	{346,348,347},
	{364,369,352},
	{364,352,353},
	{365,363,361},
	{365,361,362},
	{376,401,402},
	{373,372,397},
	{373,397,400},
	{376, 92,377},
	{381,378,387},
	{381,387,385},
	{386, 77, 80},
	{390,389,412},
	{416,417,401},
	{403,417,415},
	{408,429,430},
	{419,423,418},
	{427,428,444},
	{427,444,446},
	{437,436,441},
	{450,445, 11},
	{450, 11,  4},
	{447,449,  5},
	{447,  5,  8},
	{441,438,437},
	{425,426,451},
	{425,451,452},
	{417,421,415},
	{408,407,429},
	{399,403,400},
	{399,400,397},
	{394,393,416},
	{389,411,412},
	{386,383,385},
	{408,387,378},
	{408,378,406},
	{377,391,376},
	{ 94,375,415},
	{372,373,374},
	{372,374,370},
	{359,111,360},
	{359,112,111},
	{113,358,349},
	{113,349,123},
	{346,343,345},
	{343,340,342},
	{338,336,144},
	{338,144,141},
	{327,341,354},
	{327,354,326},
	{331,350,321},
	{331,321,322},
	{314,313,326},
	{314,326,325},
	{300,298,299},
	{300,299,301},
	{288,287,289},
	{189,292,282},
	{287,288,303},
	{284,285,297},
	{368,280,281},
	{448,447,279},
	{274,226,255},
	{267,268,404},
	{267,404,379},
	{429,262,430},
	{439,440,260},
	{257,258,249},
	{257,249,246},
	{430,262,248},
	{234,228,242},
	{234,242,241},
	{237,238,239},
	{237,239,236},
	{ 15, 18,227},
	{ 15,227,229},
	{222,223, 82},
	{222, 82, 83},
	{214,215,213},
	{214,213, 81},
	{ 38,102,  6},
	{122,159,200},
	{122,200,201},
	{174,171,192},
	{174,192,194},
	{197,193,198},
	{190,170,161},
	{181,179,178},
	{181,178,180},
	{166,156,155},
	{163,153,152},
	{163,152,162},
	{120,156,149},
	{120,149,121},
	{152,153,135},
	{140,143,142},
	{135,131,132},
	{135,132,136},
	{130,129,128},
	{130,128,127},
	{100,105,119},
	{100,119,120},
	{106,104,107},
	{106,107,108},
	{ 91, 95, 59},
	{ 93, 94, 68},
	{ 91, 89, 92},
	{ 76, 53, 55},
	{ 76, 55, 87},
	{ 81, 78, 79},
	{ 74, 73, 49},
	{ 69, 60, 45},
	{ 58, 62, 64},
	{ 58, 64, 61},
	{ 53, 31, 32},
	{ 32, 54, 53},
	{ 42, 43, 38},
	{ 35, 36,  0},
	{ 35,  0,  1},
	{ 34, 35,  1},
	{ 34,  1,  9},
	{ 44, 40, 41},
	{ 44, 41, 45},
	{ 33,240, 51},
	{ 63, 62, 58},
	{ 63, 58, 59},
	{ 45, 71, 70},
	{ 76, 75, 51},
	{ 76, 51, 52},
	{ 86, 85, 84},
	{ 86, 84, 87},
	{ 89, 72, 73},
	{ 89, 73, 88},
	{ 91, 92, 96},
	{ 91, 96, 95},
	{ 72, 91, 60},
	{ 72, 60, 69},
	{104,106,105},
	{119,105,117},
	{119,117,118},
	{124,127,128},
	{117,116,129},
	{117,129,131},
	{118,117,131},
	{135,140,142},
	{146,150,152},
	{146,152,145},
	{149,122,121},
	{166,165,151},
	{166,151,156},
	{158,172,173},
	{161,160,189},
	{199,198,193},
	{199,193,191},
	{204,201,202},
	{178,174,194},
	{200,159,186},
	{109, 48, 67},
	{ 48,107,104},
	{216, 32,236},
	{216,236,239},
	{223,214, 81},
	{223, 81, 82},
	{ 33, 12, 15},
	{ 32,228,234},
	{ 32,234,236},
	{240, 31, 52},
	{256,255,246},
	{256,246,249},
	{258,263,248},
	{258,248,249},
	{275,260,259},
	{275,259,276},
	{207,276,259},
	{270,271,429},
	{270,429,407},
	{413,418,366},
	{413,366,365},
	{368,367,279},
	{368,279,280},
	{303,301,286},
	{303,286,287},
	{283,282,292},
	{283,292,291},
	{320,292,189},
	{298,296,297},
	{298,297,299},
	{318,327,326},
	{318,326,313},
	{329,330,317},
	{336,333,320},
	{326,354,353},
	{334,332,333},
	{334,333,336},
	{342,339,139},
	{342,139,138},
	{345,342,126},
	{347,357,356},
	{369,368,351},
	{363,356,357},
	{363,357,361},
	{366,367,368},
	{366,368,369},
	{375,373,400},
	{ 92, 90,377},
	{409,387,408},
	{386,385,387},
	{386,387,388},
	{412,394,391},
	{396,398,399},
	{408,406,405},
	{415,421,419},
	{415,419,414},
	{425,452,448},
	{425,448,424},
	{444,441,443},
	{448,452,449},
	{448,449,447},
	{446,444,443},
	{446,443,445},
	{250,247,261},
	{250,261,428},
	{421,422,423},
	{421,423,419},
	{427,410,250},
	{417,403,401},
	{403,402,401},
	{420,392,412},
	{420,412,425},
	{420,425,424},
	{386,411,389},
	{383,382,381},
	{383,381,385},
	{378,379,404},
	{372,371,395},
	{372,395,397},
	{371,372,370},
	{361,359,360},
	{361,360,362},
	{368,350,351},
	{349,347,348},
	{356,355,344},
	{356,344,346},
	{344,341,340},
	{344,340,343},
	{338,337,336},
	{328,335,341},
	{324,352,351},
	{324,351,331},
	{320,144,336},
	{314,325,324},
	{322,308,309},
	{310,309,307},
	{287,286,289},
	{203,280,279},
	{203,279,205},
	{297,295,283},
	{297,283,284},
	{447,205,279},
	{274,384, 80},
	{274, 80,226},
	{266,267,379},
	{266,379,380},
	{225,257,246},
	{225,246,245},
	{256,254,253},
	{256,253,255},
	{430,247,250},
	{226,235,244},
	{226,244,245},
	{232,233,244},
	{232,244,241},
	{230, 18, 19},
	{ 32, 31,228},
	{219,220, 86},
	{219, 86, 57},
	{226,213,235},
	{206,  7,  6},
	{122,201,101},
	{201,204,101},
	{180,196,197},
	{170,192,171},
	{200,190,189},
	{194,193,195},
	{183,181,180},
	{183,180,182},
	{155,154,168},
	{149,156,151},
	{149,151,148},
	{155,156,120},
	{145,142,143},
	{145,143,146},
	{136,137,140},
	{133,132,130},
	{128,129,116},
	{100,120,121},
	{110,112,113},
	{110,113,114},
	{ 66, 65, 63},
	{ 66, 63, 99},
	{ 66, 99, 98},
	{ 96, 46, 61},
	{ 89, 88, 90},
	{ 86, 87, 57},
	{ 80, 78, 81},
	{ 72, 69, 49},
	{ 67, 48, 47},
	{ 67, 47, 68},
	{ 56, 55, 53},
	{ 50, 49, 36},
	{ 50, 36, 35},
	{ 40, 39, 41},
	{242,243,229},
	{242,229,227},
	{  6, 37, 39},
	{ 42, 47, 48},
	{ 42, 48, 43},
	{ 61, 46, 44},
	{ 45, 70, 69},
	{ 69, 70, 71},
	{ 69, 71, 49},
	{ 74, 78, 77},
	{ 83, 84, 85},
	{ 73, 74, 77},
	{ 93, 96, 92},
	{ 68, 46, 93},
	{ 95, 99, 63},
	{ 95, 63, 59},
	{115,108,110},
	{115,110,114},
	{125,126,127},
	{129,130,132},
	{137,133,138},
	{137,138,139},
	{148,146,143},
	{148,143,147},
	{119,118,154},
	{161,147,143},
	{165,164,151},
	{158,157,171},
	{158,171,172},
	{159,158,187},
	{159,187,186},
	{194,192,191},
	{194,191,193},
	{189,202,201},
	{182,197,184},
	{205,  8,  7},
	{ 48,109,107},
	{218,219, 57},
	{218, 57, 56},
	{207,231,211},
	{232,230,231},
	{232,231,233},
	{ 53, 52, 31},
	{388,411,386},
	{409,430,250},
	{262,429,254},
	{262,254,256},
	{442,444,428},
	{273,264,383},
	{273,383,384},
	{429,271,251},
	{429,251,254},
	{413,365,362},
	{ 67,413,360},
	{282,283,295},
	{285,301,299},
	{202,281,280},
	{284,283,291},
	{284,291,289},
	{320,189,160},
	{308,306,307},
	{307,309,308},
	{319,317,330},
	{319,330,328},
	{353,352,324},
	{332,331,333},
	{340,341,338},
	{354,341,344},
	{349,358,357},
	{349,357,347},
	{364,355,356},
	{364,356,363},
	{364,365,366},
	{364,366,369},
	{374,376,402},
	{375, 92,373},
	{ 77,389,390},
	{382,380,381},
	{389, 77,386},
	{393,394,412},
	{393,412,392},
	{401,394,416},
	{415,400,403},
	{411,410,427},
	{411,427,426},
	{422,420,424},
	{247,248,263},
	{247,263,261},
	{445,443, 14},
	{445, 14, 11},
	{449,450,  4},
	{449,  4,  5},
	{443,441, 17},
	{443, 17, 14},
	{436, 23, 17},
	{436, 17,441},
	{424,448,422},
	{448,423,422},
	{414,419,418},
	{414,418,413},
	{406,404,405},
	{399,397,395},
	{399,395,396},
	{420,416,392},
	{388,410,411},
	{386,384,383},
	{390, 88, 77},
	{375, 94, 92},
	{415,414, 68},
	{415, 68, 94},
	{370,374,402},
	{370,402,398},
	{361,357,358},
	{361,358,359},
	{125,348,126},
	{346,344,343},
	{340,338,339},
	{337,335,334},
	{337,334,336},
	{325,353,324},
	{324,331,332},
	{324,332,329},
	{323,322,309},
	{323,309,310},
	{294,295,297},
	{294,297,296},
	{289,286,285},
	{202,280,203},
	{288,307,303},
	{282,295,321},
	{ 67,360,111},
	{418,423,367},
	{418,367,366},
	{272,252,251},
	{272,251,271},
	{272,271,270},
	{255,253,274},
	{265,266,380},
	{265,380,382},
	{442,428,261},
	{440,263,258},
	{440,258,260},
	{409,250,410},
	{255,226,245},
	{255,245,246},
	{ 31,240,243},
	{236,234,235},
	{236,235,237},
	{233,225,245},
	{233,245,244},
	{220,221, 85},
	{220, 85, 86},
	{ 81,213,226},
	{ 81,226, 80},
	{  7,206,205},
	{186,184,198},
	{186,198,199},
	{204,203,205},
	{204,205,206},
	{195,193,196},
	{171,174,172},
	{173,174,175},
	{173,172,174},
	{155,167,166},
	{160,161,143},
	{160,143,144},
	{119,154,155},
	{148,151,150},
	{148,150,146},
	{140,137,139},
	{140,139,141},
	{127,126,130},
	{114,124,128},
	{114,128,115},
	{117,105,106},
	{117,106,116},
	{104,105,100},
	{104,100,103},
	{ 59, 60, 91},
	{ 97, 96, 61},
	{ 97, 61, 64},
	{ 91, 72, 89},
	{ 87, 84, 79},
	{ 87, 79, 76},
	{ 78, 80, 77},
	{ 49, 50, 74},
	{ 60, 44, 45},
	{ 61, 44, 58},
	{ 51, 50, 35},
	{ 51, 35, 34},
	{ 39, 37, 41},
	{ 33, 34,  9},
	{ 33,  9, 12},
	{  0, 36, 37},
	{  0, 37,  6},
	{ 40, 46, 47},
	{ 40, 47, 42},
	{ 53, 54, 56},
	{ 65, 62, 63},
	{ 72, 49, 73},
	{ 79, 78, 75},
	{ 79, 75, 76},
	{ 52, 53, 76},
	{ 92, 89, 90},
	{ 96, 93, 46},
	{102,103,100},
	{102,100,101},
	{116,106,108},
	{116,108,115},
	{123,125,124},
	{116,115,128},
	{118,131,135},
	{140,135,136},
	{148,147,149},
	{120,119,155},
	{164,162,152},
	{164,152,150},
	{157,147,161},
	{157,161,170},
	{186,187,185},
	{186,185,184},
	{193,197,196},
	{202,203,204},
	{194,195,178},
	{198,184,197},
	{ 67,111,109},
	{ 38, 43,103},
	{ 38,103,102},
	{214,223,222},
	{214,222,221},
	{214,221,220},
	{214,220,219},
	{214,219,218},
	{213,237,235},
	{221,222, 83},
	{221, 83, 85},
	{ 15,229, 33},
	{227, 18,230},
	{227,230,232},
	{ 52, 51,240},
	{ 75, 78, 50},
	{408,430,409},
	{260,258,257},
	{260,257,259},
	{224,207,259},
	{268,269,405},
	{268,405,404},
	{413,362,360},
	{447,  8,205},
	{299,297,285},
	{189,281,202},
	{290,288,289},
	{290,289,291},
	{322,321,295},
	{322,295,294},
	{333,323,311},
	{333,311,320},
	{317,316,329},
	{320,160,144},
	{353,325,326},
	{329,332,334},
	{329,334,330},
	{339,338,141},
	{339,141,139},
	{348,345,126},
	{347,356,346},
	{123,349,125},
	{364,353,354},
	{364,354,355},
	{365,364,363},
	{376,391,394},
	{376,394,401},
	{ 92,376,374},
	{ 92,374,373},
	{377, 90, 88},
	{380,379,378},
	{380,378,381},
	{388,387,409},
	{388,409,410},
	{416,393,392},
	{399,398,402},
	{399,402,403},
	{250,428,427},
	{421,417,416},
	{421,416,420},
	{426,427,446},
	{426,446,451},
	{444,442,441},
	{452,451,450},
	{452,450,449}
};

PxU32 SnippetUtils::Bunny_getNbVerts()
{
	return BUNNY_NBVERTICES;
}

PxU32 SnippetUtils::Bunny_getNbFaces()
{
	return BUNNY_NBFACES;
}

const PxVec3* SnippetUtils::Bunny_getVerts()
{
	return reinterpret_cast<const PxVec3*>(&gBunnyVertices[0][0]);
}

const PxU32* SnippetUtils::Bunny_getFaces()
{
	return &gBunnyTriangles[0][0];
}

