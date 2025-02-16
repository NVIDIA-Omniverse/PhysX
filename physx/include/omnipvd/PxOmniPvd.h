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

#ifndef PX_OMNI_PVD_H
#define PX_OMNI_PVD_H

#include "PxPhysXConfig.h"

class OmniPvdWriter;
class OmniPvdFileWriteStream;

// The OVD integration version:
// 
//   Major version indicates breaking changes in how PhysX SDK objects
//   are streamed using the OmniPVD API, or if certain attributes changed
//   name/type or set size, or was removed, hence subtractive changes.
// 
//   Minor version version indicates non-breaking changes such as the
//   addition of a class or attribute on top of those already existing,
//   hence additive changes.
#define PX_PHYSICS_OVD_INTEGRATION_VERSION_MAJOR 1
#define PX_PHYSICS_OVD_INTEGRATION_VERSION_MINOR 8

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxFoundation;

class PxOmniPvd
{
public:
	class ScopedExclusiveWriter
	{
	  public:
		PX_FORCE_INLINE ScopedExclusiveWriter(PxOmniPvd* omniPvd)
		{			
			mOmniPvd = omniPvd;
			mWriter = NULL;
			if (mOmniPvd) {			
				mWriter = mOmniPvd->acquireExclusiveWriterAccess();
			}
		}

		PX_FORCE_INLINE ~ScopedExclusiveWriter()
		{
			if (mOmniPvd && mWriter) {
				mOmniPvd->releaseExclusiveWriterAccess();
			}
		}

		PX_FORCE_INLINE OmniPvdWriter* operator-> ()
		{
			return mWriter;
		}
		
		PX_FORCE_INLINE OmniPvdWriter* getWriter()
		{
			return mWriter;
		}
	private:
		OmniPvdWriter* mWriter;
		PxOmniPvd* mOmniPvd;
	};

	virtual ~PxOmniPvd()
	{
	}
	/**
	\brief Get the OmniPvd writer.
	
	Gets an instance of the OmniPvd writer. The writer access will not be thread safe since the OmniPVD API is not thread safe itself. Writing concurrently and simultaneously using the OmniPVD API is undefined.
	
	For thread safe exlcusive access use the mechanism acquireExclusiveWriterAccess/releaseExclusiveWriterAccess.

	\return OmniPvdWriter instance on succes, NULL otherwise.
	*/
	virtual OmniPvdWriter* getWriter() = 0;
	
	/**
	\brief Acquires an exclusive writer access.
	
	This call blocks until exclusive access to the writer can be acquired. Once access has been granted, it is guaranteed that no other caller can access the writer through this method until releaseExclusiveWriterAccess() has been called.
	
	This allows to safely write PVD data in environments with concurrent processing workflows.

	\return OmniPvdWriter instance on succes, NULL otherwise.
	*/
	virtual OmniPvdWriter* acquireExclusiveWriterAccess() = 0;

	/**
	\brief Releases the exclusive writer access
	
	Releases the access to the writer that was previously acquired using acquireExclusiveWriterAccess.

	*/
	virtual void releaseExclusiveWriterAccess() = 0;

	/**
	\brief Gets an instance to the OmniPvd file write stream
	
	\return OmniPvdFileWriteStream instance on succes, NULL otherwise.
	*/
	virtual OmniPvdFileWriteStream* getFileWriteStream() = 0;
	
	/**
	\brief Starts the OmniPvd sampling

	\return True if sampling started correctly, false if not.
	*/
	virtual bool startSampling() = 0;

	/**
	\brief Releases the PxOmniPvd object

	*/
	virtual void release() = 0;

};
#if !PX_DOXYGEN
} // namespace physx
#endif
/**
\brief Creates an instance of the OmniPvd object

Creates an instance of the OmniPvd class. There may be only one instance of this class per process. Calling this method after an instance
has been created already will return the same instance over and over.

\param foundation Foundation instance (see PxFoundation)

\return PxOmniPvd instance on succes, NULL otherwise.

*/
PX_C_EXPORT PX_PHYSX_CORE_API physx::PxOmniPvd* PX_CALL_CONV PxCreateOmniPvd(physx::PxFoundation& foundation);


#endif
