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

#ifndef PX_OMNI_PVD_H
#define PX_OMNI_PVD_H

#include "PxPhysXConfig.h"

class OmniPvdWriter;
class OmniPvdFileWriteStream;
class OmniPvdSocketWriteStream;

// The OVD integration version:
// 
//   Major version indicates breaking changes in how PhysX SDK objects
//   are streamed using the OmniPVD API, or if certain attributes changed
//   name/type or set size, or was removed, hence subtractive changes.
// 
//   Minor version version indicates non-breaking changes such as the
//   addition of a class or attribute on top of those already existing,
//   hence additive changes.
#define PX_PHYSICS_OVD_INTEGRATION_VERSION_MAJOR 3
#define PX_PHYSICS_OVD_INTEGRATION_VERSION_MINOR 1

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxFoundation;
class PxOmniPvd;

/**
\brief Callback that lets a module add its own objects to the recording started by startSampling().

A module (such as PhysXExtensions) implements this callback and registers it with
PxOmniPvd::addEventCallback(). startSampling() calls onStartSampling() after it has recorded
the core PhysX objects, so the module can record its own objects onto the bound write stream.
*/
class PxOmniPvdEventCallback
{
public:
	/**
	\brief Called by startSampling() after the core PhysX objects have been recorded.

	The module records its own objects onto the bound write stream here, so they can refer to
	the core objects already recorded.

	\param omniPvd The PxOmniPvd taking the snapshot (the same instance the callback was registered with).
	*/
	virtual void onStartSampling(PxOmniPvd& omniPvd) = 0;
	virtual ~PxOmniPvdEventCallback() {}
};

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
	\brief Creates an OmniPvd TCP socket write stream, for live streaming.

	The returned stream can be bound to an OmniPvdWriter using OmniPvdWriter::setWriteStream(). The
	caller is responsible for releasing the stream via releaseSocketWriteStream().

	\param address The IP-address of the listening reader to connect to.
	\param port TCP port of the listening reader to connect to.
	\param sendTimeout Maximum time in milliseconds to block on a single send. This stops a stalled reader from blocking the simulation thread indefinitely.

	\return OmniPvdSocketWriteStream instance on succes, NULL otherwise.

	\see releaseSocketWriteStream()
	*/
	virtual OmniPvdSocketWriteStream* createSocketWriteStream(const char* address, physx::PxU16 port, physx::PxU32 sendTimeout = 3000) = 0;

	/**
	\brief Releases a socket write stream previously created with createSocketWriteStream.

	The caller owns the stream's lifetime. Keep the stream alive until PhysX is done writing
	to it: release the PxScene / PxPhysics that drive the recording before releasing the
	stream (if a recording is still active their teardown emits object-remove commands to the
	bound stream), and do not release the stream while it could still be written to.

	\param stream The socket write stream to release.

	\see createSocketWriteStream()
	*/
	virtual void releaseSocketWriteStream(OmniPvdSocketWriteStream& stream) = 0;

	/**
	\brief Starts recording to the bound write stream.

	First records the current state of the whole scene, then records every change from this
	point on. Because the current state is recorded first, the result is complete on its own
	whether the scene is empty or has been running for a while. This is what makes a late attach
	work: you can bind a stream after the simulation has been running and the recording still
	contains the full scene.

	Bind the write stream first with OmniPvdWriter::setWriteStream(), then call startSampling().
	To take a fresh recording later, call stopSampling(), bind a stream with setWriteStream()
	(the same stream may be re-bound, which makes the next recording self-contained), then call
	startSampling() again. Calling startSampling() while already sampling is not allowed: it does
	not start a new recording and returns false with an error message. Pair each startSampling()
	with a stopSampling().

	\note Recording the current state reads the whole scene, so do not call this while the scene
	is stepping. Call it before PxScene::simulate()/collide() or after fetchResults()/fetchCollision(),
	or while holding the scene write lock.

	\return True if recording started. False if recording was already on, if no writer is
	available, or if a write failed while recording the current state.

	\see OmniPvdWriter::setWriteStream()
	\see PxOmniPvd::addEventCallback()
	\see PxOmniPvd::stopSampling()
	\see PxOmniPvd::isSampling()
	*/
	virtual bool startSampling() = 0;

	/**
	\brief Stops recording.

	After this call, no further object or per-frame data is written. The write stream is not
	flushed or closed; you own the stream and decide when to flush or close it. Call stopSampling()
	before starting another recording: a later startSampling() then records the current state again
	onto the (re-)bound stream. It is not a resume.

	\return True if sampling was stopped, false if it was not sampling.

	\see PxOmniPvd::startSampling()
	*/
	virtual bool stopSampling() = 0;

	/**
	\brief Whether recording is currently on.

	True between a successful startSampling() and the next stopSampling().

	\return True if sampling, false otherwise.

	\see PxOmniPvd::startSampling()
	\see PxOmniPvd::stopSampling()
	*/
	virtual bool isSampling() const = 0;

	/**
	\brief Registers a callback that contributes its objects to the recording.

	A module (such as PhysXExtensions) implements PxOmniPvdEventCallback to record its own
	objects when startSampling() records the current state. startSampling() calls each
	registered callback's onStartSampling() after it has recorded the core PhysX objects.
	Registering the same callback twice has no effect.

	\note addEventCallback() and removeEventCallback() are not thread-safe; call them from a
	single thread (for example at setup), not concurrently with each other or with sampling.

	\param callback The callback to register.

	\see PxOmniPvd::removeEventCallback()
	*/
	virtual void addEventCallback(PxOmniPvdEventCallback& callback) = 0;

	/**
	\brief Unregisters a previously registered callback.

	Has no effect if the callback was not registered.

	\note Not thread-safe; see addEventCallback().

	\param callback The callback to unregister.

	\see PxOmniPvd::addEventCallback()
	*/
	virtual void removeEventCallback(PxOmniPvdEventCallback& callback) = 0;

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
