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

#include "geometry/PxGeometry.h"
#include "PxcContactMethodImpl.h"

using namespace physx;

#define ARGS	shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput

static bool PxcInvalidContactPair			(GU_CONTACT_METHOD_ARGS_UNUSED)	{ return false;	}

// PT: IMPORTANT: do NOT remove the indirection! Using the Gu functions directly in the table produces massive perf problems.
static bool PxcContactSphereSphere				(GU_CONTACT_METHOD_ARGS)	{ return contactSphereSphere(ARGS);					}
static bool PxcContactSphereCapsule				(GU_CONTACT_METHOD_ARGS)	{ return contactSphereCapsule(ARGS);				}
static bool PxcContactSphereBox					(GU_CONTACT_METHOD_ARGS)	{ return contactSphereBox(ARGS);					}
static bool PxcContactSpherePlane				(GU_CONTACT_METHOD_ARGS)	{ return contactSpherePlane(ARGS);					}
static bool PxcContactSphereConvex				(GU_CONTACT_METHOD_ARGS)	{ return contactCapsuleConvex(ARGS);				}
static bool PxcContactSphereMesh				(GU_CONTACT_METHOD_ARGS)	{ return contactSphereMesh(ARGS);					}
static bool PxcContactSphereHeightField			(GU_CONTACT_METHOD_ARGS)	{ return contactSphereHeightfield(ARGS);			}
static bool PxcContactPlaneBox					(GU_CONTACT_METHOD_ARGS)	{ return contactPlaneBox(ARGS);						}
static bool PxcContactPlaneCapsule				(GU_CONTACT_METHOD_ARGS)	{ return contactPlaneCapsule(ARGS);					}
static bool PxcContactPlaneConvex				(GU_CONTACT_METHOD_ARGS)	{ return contactPlaneConvex(ARGS);					}
static bool PxcContactCapsuleCapsule			(GU_CONTACT_METHOD_ARGS)	{ return contactCapsuleCapsule(ARGS);				}
static bool PxcContactCapsuleBox				(GU_CONTACT_METHOD_ARGS)	{ return contactCapsuleBox(ARGS);					}
static bool PxcContactCapsuleConvex				(GU_CONTACT_METHOD_ARGS)	{ return contactCapsuleConvex(ARGS);				}
static bool PxcContactCapsuleMesh				(GU_CONTACT_METHOD_ARGS)	{ return contactCapsuleMesh(ARGS);					}
static bool PxcContactCapsuleHeightField		(GU_CONTACT_METHOD_ARGS)	{ return contactCapsuleHeightfield(ARGS);			}
static bool PxcContactBoxBox					(GU_CONTACT_METHOD_ARGS)	{ return contactBoxBox(ARGS);						}
static bool PxcContactBoxConvex					(GU_CONTACT_METHOD_ARGS)	{ return contactBoxConvex(ARGS);					}
static bool PxcContactBoxMesh					(GU_CONTACT_METHOD_ARGS)	{ return contactBoxMesh(ARGS);						}
static bool PxcContactBoxHeightField			(GU_CONTACT_METHOD_ARGS)	{ return contactBoxHeightfield(ARGS);				}
static bool PxcContactConvexConvex				(GU_CONTACT_METHOD_ARGS)	{ return contactConvexConvex(ARGS);					}
static bool PxcContactConvexMesh				(GU_CONTACT_METHOD_ARGS)	{ return contactConvexMesh(ARGS);					}
static bool PxcContactConvexHeightField			(GU_CONTACT_METHOD_ARGS)	{ return contactConvexHeightfield(ARGS);			}
static bool PxcContactGeometryCustomGeometry	(GU_CONTACT_METHOD_ARGS)	{ return contactGeometryCustomGeometry(ARGS);		}

static bool PxcPCMContactSphereSphere			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactSphereSphere(ARGS);				}
static bool PxcPCMContactSpherePlane			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactSpherePlane(ARGS);				}
static bool PxcPCMContactSphereBox				(GU_CONTACT_METHOD_ARGS)	{ return pcmContactSphereBox(ARGS);					}
static bool PxcPCMContactSphereCapsule			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactSphereCapsule(ARGS);				}
static bool PxcPCMContactSphereConvex			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactSphereConvex(ARGS);				}
static bool PxcPCMContactSphereMesh				(GU_CONTACT_METHOD_ARGS)	{ return pcmContactSphereMesh(ARGS);				}
static bool PxcPCMContactSphereHeightField		(GU_CONTACT_METHOD_ARGS)	{ return pcmContactSphereHeightField(ARGS);			}
static bool PxcPCMContactPlaneCapsule			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactPlaneCapsule(ARGS);				}
static bool PxcPCMContactPlaneBox				(GU_CONTACT_METHOD_ARGS)	{ return pcmContactPlaneBox(ARGS);					}
static bool PxcPCMContactPlaneConvex			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactPlaneConvex(ARGS);				}
static bool PxcPCMContactCapsuleCapsule			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactCapsuleCapsule(ARGS);			}
static bool PxcPCMContactCapsuleBox				(GU_CONTACT_METHOD_ARGS)	{ return pcmContactCapsuleBox(ARGS);				}
static bool PxcPCMContactCapsuleConvex			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactCapsuleConvex(ARGS);				}
static bool PxcPCMContactCapsuleMesh			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactCapsuleMesh(ARGS);				}
static bool PxcPCMContactCapsuleHeightField		(GU_CONTACT_METHOD_ARGS)	{ return pcmContactCapsuleHeightField(ARGS);		}
static bool PxcPCMContactBoxBox					(GU_CONTACT_METHOD_ARGS)	{ return pcmContactBoxBox(ARGS);					}
static bool PxcPCMContactBoxConvex				(GU_CONTACT_METHOD_ARGS)	{ return pcmContactBoxConvex(ARGS);					}
static bool PxcPCMContactBoxMesh				(GU_CONTACT_METHOD_ARGS)	{ return pcmContactBoxMesh(ARGS);					}
static bool PxcPCMContactBoxHeightField			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactBoxHeightField(ARGS);			}
static bool PxcPCMContactConvexConvex			(GU_CONTACT_METHOD_ARGS)	{ return pcmContactConvexConvex(ARGS);				}
static bool PxcPCMContactConvexMesh				(GU_CONTACT_METHOD_ARGS)	{ return pcmContactConvexMesh(ARGS);				}
static bool PxcPCMContactConvexHeightField		(GU_CONTACT_METHOD_ARGS)	{ return pcmContactConvexHeightField(ARGS);			}
static bool PxcPCMContactGeometryCustomGeometry	(GU_CONTACT_METHOD_ARGS)	{ return pcmContactGeometryCustomGeometry(ARGS);	}

#undef ARGS

namespace physx
{
//Table of contact methods for different shape-type combinations
PxcContactMethod g_ContactMethodTable[][PxGeometryType::eGEOMETRY_COUNT] = 
{
	//PxGeometryType::eSPHERE
	{
		PxcContactSphereSphere,			//PxGeometryType::eSPHERE
		PxcContactSpherePlane,			//PxGeometryType::ePLANE
		PxcContactSphereCapsule,		//PxGeometryType::eCAPSULE
		PxcContactSphereBox,			//PxGeometryType::eBOX
		PxcContactSphereConvex,			//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,			//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,			//PxGeometryType::eTETRAHEDRONMESH
		PxcContactSphereMesh,			//PxGeometryType::eTRIANGLEMESH
		PxcContactSphereHeightField,	//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePLANE
	{
		0,								//PxGeometryType::eSPHERE
		PxcInvalidContactPair,			//PxGeometryType::ePLANE
		PxcContactPlaneCapsule,			//PxGeometryType::eCAPSULE
		PxcContactPlaneBox,				//PxGeometryType::eBOX
		PxcContactPlaneConvex,			//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,			//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,			//PxGeometryType::eTETRAHEDRONMESH
		PxcInvalidContactPair,			//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,			//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCAPSULE
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		PxcContactCapsuleCapsule,		//PxGeometryType::eCAPSULE
		PxcContactCapsuleBox,			//PxGeometryType::eBOX
		PxcContactCapsuleConvex,		//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,			//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,			//PxGeometryType::eTETRAHEDRONMESH
		PxcContactCapsuleMesh,			//PxGeometryType::eTRIANGLEMESH
		PxcContactCapsuleHeightField,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eBOX
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		PxcContactBoxBox,				//PxGeometryType::eBOX
		PxcContactBoxConvex,			//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,			//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,			//PxGeometryType::eTETRAHEDRONMESH
		PxcContactBoxMesh,				//PxGeometryType::eTRIANGLEMESH
		PxcContactBoxHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		PxcContactConvexConvex,			//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,			//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,			//PxGeometryType::eTETRAHEDRONMESH
		PxcContactConvexMesh,			//PxGeometryType::eTRIANGLEMESH
		PxcContactConvexHeightField,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePARTICLESYSTEM
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,			//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,			//PxGeometryType::eTETRAHEDRONMESH
		PxcInvalidContactPair,			//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,			//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,			//PxGeometryType::eHAIRSYSTEM
		PxcInvalidContactPair,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTETRAHEDRONMESH
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,			//PxGeometryType::eTETRAHEDRONMESH
		PxcInvalidContactPair,			//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,			//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,			//PxGeometryType::eHAIRSYSTEM
		PxcInvalidContactPair,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		0,									//PxGeometryType::eTETRAHEDRONMESH
		PxcInvalidContactPair,				//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,				//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		0,									//PxGeometryType::eTETRAHEDRONMESH
		0,									//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,				//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHAIRSYSTEM
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		0,									//PxGeometryType::eTETRAHEDRONMESH
		0,									//PxGeometryType::eTRIANGLEMESH
		0,									//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCUSTOM
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		0,									//PxGeometryType::eTETRAHEDRONMESH
		0,									//PxGeometryType::eTRIANGLEMESH
		0,									//PxGeometryType::eHEIGHTFIELD
		0,									//PxGeometryType::eHAIRSYSTEM
		PxcContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},
};
PX_COMPILE_TIME_ASSERT(sizeof(g_ContactMethodTable) / sizeof(g_ContactMethodTable[0]) == PxGeometryType::eGEOMETRY_COUNT);

//Table of contact methods for different shape-type combinations
PxcContactMethod g_PCMContactMethodTable[][PxGeometryType::eGEOMETRY_COUNT] = 
{
	//PxGeometryType::eSPHERE
	{
		PxcPCMContactSphereSphere,				//PxGeometryType::eSPHERE
		PxcPCMContactSpherePlane,				//PxGeometryType::ePLANE
		PxcPCMContactSphereCapsule,				//PxGeometryType::eCAPSULE
		PxcPCMContactSphereBox,					//PxGeometryType::eBOX
		PxcPCMContactSphereConvex,				//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,					//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,					//PxGeometryType::eTETRAHEDRONMESH
		PxcPCMContactSphereMesh,				//PxGeometryType::eTRIANGLEMESH
		PxcPCMContactSphereHeightField,			//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,					//PxGeometryType::eHAIRSYSTEM
		PxcPCMContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePLANE
	{
		0,									//PxGeometryType::eSPHERE
		PxcInvalidContactPair,				//PxGeometryType::ePLANE
		PxcPCMContactPlaneCapsule,			//PxGeometryType::eCAPSULE
		PxcPCMContactPlaneBox,				//PxGeometryType::eBOX  
		PxcPCMContactPlaneConvex,			//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,				//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,				//PxGeometryType::eTETRAHEDRONMESH
		PxcInvalidContactPair,				//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,				//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcPCMContactGeometryCustomGeometry,//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCAPSULE
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		PxcPCMContactCapsuleCapsule,			//PxGeometryType::eCAPSULE
		PxcPCMContactCapsuleBox,				//PxGeometryType::eBOX
		PxcPCMContactCapsuleConvex,				//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,					//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,					//PxGeometryType::eTETRAHEDRONMESH
		PxcPCMContactCapsuleMesh,				//PxGeometryType::eTRIANGLEMESH	
		PxcPCMContactCapsuleHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,					//PxGeometryType::eHAIRSYSTEM
		PxcPCMContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eBOX
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		0,										//PxGeometryType::eCAPSULE
		PxcPCMContactBoxBox,					//PxGeometryType::eBOX
		PxcPCMContactBoxConvex,					//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,					//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,					//PxGeometryType::eTETRAHEDRONMESH
		PxcPCMContactBoxMesh,					//PxGeometryType::eTRIANGLEMESH
		PxcPCMContactBoxHeightField,			//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,					//PxGeometryType::eHAIRSYSTEM
		PxcPCMContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		0,										//PxGeometryType::eCAPSULE
		0,										//PxGeometryType::eBOX
		PxcPCMContactConvexConvex,				//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,					//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,					//PxGeometryType::eTETRAHEDRONMESH
		PxcPCMContactConvexMesh,				//PxGeometryType::eTRIANGLEMESH
		PxcPCMContactConvexHeightField,			//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,					//PxGeometryType::eHAIRSYSTEM
		PxcPCMContactGeometryCustomGeometry,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePARTICLESYSTEM
	{
		0,						//PxGeometryType::eSPHERE
		0,						//PxGeometryType::ePLANE
		0,						//PxGeometryType::eCAPSULE
		0,						//PxGeometryType::eBOX
		0,						//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,	//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,	//PxGeometryType::eTETRAHEDRONMESH
		PxcInvalidContactPair,	//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,	//PxGeometryType::eHAIRSYSTEM
		PxcInvalidContactPair,	//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTETRAHEDRONMESH
	{
		0,							//PxGeometryType::eSPHERE
		0,							//PxGeometryType::ePLANE
		0,							//PxGeometryType::eCAPSULE
		0,							//PxGeometryType::eBOX
		0,							//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,		//PxGeometryType::ePARTICLESYSTEM
		PxcInvalidContactPair,		//PxGeometryType::eTETRAHEDRONMESH
		PxcInvalidContactPair,		//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcInvalidContactPair,		//PxGeometryType::eHAIRSYSTEM
		PxcInvalidContactPair,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		0,									//PxGeometryType::eTETRAHEDRONMESH
		PxcInvalidContactPair,				//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,				//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcPCMContactGeometryCustomGeometry,//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		0,									//PxGeometryType::eTETRAHEDRONMESH
		0,									//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,				//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcPCMContactGeometryCustomGeometry,//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHAIRSYSTEM
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		0,									//PxGeometryType::eTETRAHEDRONMESH
		0,									//PxGeometryType::eTRIANGLEMESH
		0,									//PxGeometryType::eHEIGHTFIELD
		PxcInvalidContactPair,				//PxGeometryType::eHAIRSYSTEM
		PxcInvalidContactPair,				//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCUSTOM
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		0,									//PxGeometryType::eTETRAHEDRONMESH
		0,									//PxGeometryType::eTRIANGLEMESH
		0,									//PxGeometryType::eHEIGHTFIELD
		0,									//PxGeometryType::eHAIRSYSTEM
		PxcPCMContactGeometryCustomGeometry,//PxGeometryType::eCUSTOM
	},
};
PX_COMPILE_TIME_ASSERT(sizeof(g_PCMContactMethodTable) / sizeof(g_PCMContactMethodTable[0]) == PxGeometryType::eGEOMETRY_COUNT);

}
