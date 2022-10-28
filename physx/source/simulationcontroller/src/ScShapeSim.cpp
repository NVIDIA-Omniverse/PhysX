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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "ScBodySim.h"
#include "ScStaticSim.h"
#include "ScScene.h"
#include "ScElementSimInteraction.h"
#include "ScShapeInteraction.h"
#include "ScTriggerInteraction.h"
#include "ScSimStats.h"
#include "ScObjectIDTracker.h"
#include "GuHeightFieldUtil.h"
#include "GuTriangleMesh.h"
#include "GuConvexMeshData.h"
#include "GuHeightField.h"
#include "PxsContext.h"
#include "PxsTransformCache.h"
#include "CmTransformUtils.h"
#include "GuBounds.h"
#include "PxsRigidBody.h"
#include "ScSqBoundsManager.h"
#include "PxsSimulationController.h"
#include "common/PxProfileZone.h"
#include "ScArticulationSim.h"

using namespace physx;
using namespace Gu;
using namespace Sc;

void resetElementID(Scene& scene, ShapeSimBase& shapeSim);

ShapeSim::ShapeSim(RigidSim& owner, ShapeCore& core) :
	ShapeSimBase(owner, &core)
{
	// sizeof(ShapeSim) = 40 bytes

	initSubsystemsDependingOnElementID();

	core.setSim(this);
}

ShapeSim::~ShapeSim()
{
	const_cast<ShapeCore*>(&getCore())->clearSim();
	Scene& scScene = getScene();
	resetElementID(scScene, *this);
}

