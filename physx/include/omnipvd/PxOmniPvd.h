// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_OMNI_PVD_H
#define PX_OMNI_PVD_H

#include "PxPhysXConfig.h"
#include "foundation/PxSimpleTypes.h"

class OmniPvdWriter;

// The OVD integration version:
// 
//   Major version indicates breaking changes in how PhysX SDK objects
//   are streamed using the OmniPVD API, or if certain attributes changed
//   name/type or set size, or was removed, hence subtractive changes.
// 
//   Minor version indicates additive, non-breaking changes, including new
//   classes or attributes and newly recorded data for existing attributes.
#define PX_PHYSICS_OVD_INTEGRATION_VERSION_MAJOR 3
#define PX_PHYSICS_OVD_INTEGRATION_VERSION_MINOR 5

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxFoundation;
class PxOmniPvd;
class PxBase;
class PxScene;

/**
\brief Selects the objects and data recorded by one call to startSampling().

The descriptor selects scenes, individual objects, or both. Primary simulation state is
always recorded for the selection. recordSecondaryData enables the remaining supported data.
The default descriptor records all objects and all supported data.

startSampling() copies the descriptor and its lists. The selection lasts until stopSampling().
To change it locally, stop and start again with a new descriptor. An accepted receiver request
can replace it through processCaptureRequests(). Calling startSampling() without a
descriptor after stopping resets the selection and records all objects and all supported data.

\see PxOmniPvd::startSampling() PxOmniPvd::validateFilterDesc()
*/
struct PxOmniPvdFilterDesc
{
	/**
	\brief Enables secondary data in addition to the primary simulation state.

	Primary data includes names and relationships, poses, bounds, joint positions,
	linear/angular/joint velocities and accelerations, particle positions, velocities and
	phases, deformable vertex state, shapes, meshes and materials.

	Secondary data includes contact points, normals, separation, impulses and friction,
	applied force/torque, incoming joint solver force/torque, actor/scene/joint configuration,
	drive targets and all diffuse particle data, including positions/lifetimes, velocities
	and parameters. A contact pair is included when either actor is selected.

	Rigid-body force/torque fields record the supplied addForce()/addTorque()/setForceAndTorque()
	arguments for eFORCE, eACCELERATION, eIMPULSE and eVELOCITY_CHANGE. The schema does not record
	the mode separately. clearForce()/clearTorque() record zero. Force/torque applied before
	startSampling() is not reconstructed, except in PxSceneFlag::eENABLE_DIRECT_GPU_API scenes,
	where the snapshot records force/torque retained on the device.
	CPU applyCache(PxArticulationCacheFlag::eFORCE) and particle/deformable applied-force arrays
	are not recorded.

	This setting applies to SDK recording. Data written directly through OmniPvdWriter is
	controlled by its writer, including data contributed by PxOmniPvdEventCallback implementations.

	<b>Default:</b> true
	\see PxRigidBody::addForce() PxRigidBody::addTorque() PxRigidBody::setForceAndTorque()
	PxRigidBody::clearForce() PxRigidBody::clearTorque()
	PxRigidDynamicGPUAPIWriteType::eFORCE PxRigidDynamicGPUAPIWriteType::eTORQUE
	PxArticulationGPUAPIWriteType::eJOINT_FORCE PxArticulationGPUAPIWriteType::eLINK_FORCE
	PxArticulationGPUAPIWriteType::eLINK_TORQUE PxArticulationGPUAPIReadType::eJOINT_FORCE
	PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE
	*/
	bool recordSecondaryData;

	/**
	\brief Scenes whose objects are selected for recording.

	All objects in these scenes are selected, including objects added later. The selected set
	is the union of the scene and object lists. When both lists are empty, all SDK objects are
	selected, including objects outside scenes. Duplicate entries have no additional effect.

	Every entry must belong to the PxPhysics associated with this PxOmniPvd.
	NULL is valid when nbScenes is zero.

	<b>Default:</b> NULL
	*/
	const PxScene* const* scenes;

	/** \brief Number of entries in scenes. <b>Default:</b> zero */
	PxU32 nbScenes;

	/**
	\brief Individual objects selected for recording in addition to the selected scenes.

	Supported objects are rigid statics/dynamics, whole articulations, deformable surfaces
	and volumes, PBD particle systems, and particle/diffuse buffers. Articulation links and
	joints, aggregates, constraints, extension joints, shapes, materials and meshes cannot
	be selected directly. Select their owning actors or whole articulations instead.

	Selected objects include their shapes, meshes, materials, articulation links/joints,
	connected constraints/extension joints, and a selected particle system's buffers.
	A selected particle buffer includes its owning system's identity and particle materials
	without selecting sibling buffers. An aggregate containing a selected actor is recorded
	with only its selected members.

	Rigid actors connected to selected actors by a recorded joint, constraint or contact pair
	are recorded with their identity, scene, shapes and current pose/velocity state. In
	PxSceneFlag::eENABLE_DIRECT_GPU_API scenes, a rigid dynamic or articulation link that has
	already been simulated and becomes connected between simulation steps gets its pose and
	velocity at the next fetchResults(). Their other joints and contact pairs are not selected.
	A contact pair whose bodies fall asleep while touching keeps the connected actor recorded,
	although the scene's contact arrays leave the pair out until the bodies wake.
	When no recorded connection requires an actor, its recording ends at the next scene
	recording update. Reconnecting records the actor's current state again.
	A connected actor that leaves its scene stops updating at once. It stays in the recording
	outside every scene and aggregate until a new connection records it again, it joins a
	selected scene, or it is released.
	An actor connected only by contacts that come and go, such as the ground under a bouncing
	body, is recorded only in the frames in which it touches a selected actor. With
	recordSecondaryData, selecting an actor that many others touch, such as a ground, records
	every actor that touches it.

	Every entry must belong to the associated PxPhysics. Releasing a selected object removes
	it from the selection. Another object reusing its address is not selected.
	NULL is valid when nbObjects is zero.

	<b>Default:</b> NULL
	\see PxOmniPvd::isObjectRecorded()
	*/
	const PxBase* const* objects;

	/** \brief Number of entries in objects. <b>Default:</b> zero */
	PxU32 nbObjects;

	/** \brief Initializes an all-object, all-data filter. */
	PX_INLINE PxOmniPvdFilterDesc() : recordSecondaryData(true), scenes(NULL), nbScenes(0), objects(NULL), nbObjects(0) {}

	/**
	\brief Checks that each nonempty array and its entries are nonnull.
	\return True if the array/count pairs and entries are valid.
	\see PxOmniPvd::validateFilterDesc()
	*/
	PX_INLINE bool isValid() const
	{
		if ((nbScenes && !scenes) || (nbObjects && !objects)) return false;
		for (PxU32 i = 0; i < nbScenes; ++i) if (!scenes[i]) return false;
		for (PxU32 i = 0; i < nbObjects; ++i) if (!objects[i]) return false;
		return true;
	}
};

/**
\brief Lets the application accept, restrict or deny capture requests sent by an OmniPVD receiver.

PxOmniPvd::processCaptureRequests() calls filterCaptureRequest() after every handle of a request has
been matched to a live, supported object of the associated PxPhysics, and before the recording changes.
Requests naming an unknown or unsupported handle, or the handle of an object released on the same connection while this PxPhysics was recording or had an active discovery
session, are rejected before this call. While any scene is between simulate() or collide() and
the completion of its matching fetchResults() or fetchResultsFinish(), a request is acknowledged
as busy before this call. Catalog refresh requests do not reach this call. While a
startCaptureDiscovery() session is active, its scope is applied after this call: a result naming
a scene or object outside the scope is denied, and a result with both arrays empty selects what
remains of the scope, or is denied if nothing remains.

\see PxOmniPvd::processCaptureRequests() PxOmniPvdFilterDesc
*/
class PxOmniPvdCaptureFilter
{
public:
	/**
	\brief Decides whether a receiver request is recorded, optionally with a reduced selection.

	The application can replace the arrays or remove objects it does not own. Both arrays empty
	selects every object, or what remains of the discovery scope while a discovery session is active. Replacement arrays must remain valid until processCaptureRequests()
	returns. The callback runs without holding the writer lock.

	\param[in,out] desc Requested filter. On return, the filter to record.
	\return True to apply the resulting filter after validation. False or an invalid filter leaves
	the current recording unchanged.

	\see PxOmniPvd::processCaptureRequests()
	*/
	virtual bool filterCaptureRequest(PxOmniPvdFilterDesc& desc) = 0;
	virtual ~PxOmniPvdCaptureFilter() {}
};

/**
\brief Callback that lets a module add its own objects to the recording started by startSampling().

A module (such as PhysXExtensions) implements this callback and registers it with
PxOmniPvd::addEventCallback(). startSampling() calls onStartSampling() after it has recorded
the core PhysX objects, so the module can record its own objects onto the bound write stream.
The callback receives the active filter so the module can apply the same data policy.
Direct OmniPvdWriter calls are not filtered by PxOmniPvd.
A module can use PxOmniPvd::isObjectRecorded() to query the active SDK object selection.
PhysXExtensions applies the SDK selection and data policy to its built-in joints and geometry.

processCaptureRequests() calls onCaptureReset() before an accepted receiver request replaces the recording.
startCaptureDiscovery() also calls it when retiring a stopped recording on the same stream and connection.
*/
class PxOmniPvdEventCallback
{
public:
	/**
	\brief Called by startSampling() after the core PhysX objects have been recorded.

	The module records its own objects onto the bound write stream here, so they can refer to
	the core objects already recorded. The default implementation calls onStartSampling(PxOmniPvd&).

	\param[in] omniPvd The PxOmniPvd taking the snapshot (the same instance the callback was registered with).
	\param[in] filter The copied active selection descriptor, or NULL for an unrestricted start without
	a filter. An active discovery scope is expanded before this callback, including when
	startSampling(NULL) is used. NULL selects all objects and all supported data. The descriptor and its lists are valid
	only for the duration of this callback. Copy any data needed after the callback returns.
	*/
	virtual void onStartSampling(PxOmniPvd& omniPvd, const PxOmniPvdFilterDesc* filter)
	{
		PX_UNUSED(filter);
		onStartSampling(omniPvd);
	}

	/**
	\brief Single-argument form of onStartSampling(), called by its default implementation.

	\deprecated Override onStartSampling(PxOmniPvd&, const PxOmniPvdFilterDesc*) instead.

	\param[in] omniPvd The PxOmniPvd taking the snapshot.
	*/
	PX_DEPRECATED virtual void onStartSampling(PxOmniPvd& omniPvd)
	{
		PX_UNUSED(omniPvd);
	}

	/**
	\brief Prepares module data when a shape or constraint changes its recorded state during a recording.

	Use isObjectRecorded() to test the current selection. For a shape, this runs before the SDK
	creates or updates its geometry and attributes. Emit module resources needed by that geometry.
	The shape and its new resources may not have been recorded yet.

	For a constraint, this runs after the SDK has recorded the PxConstraint itself. In a selective
	recording, it also runs when a recorded constraint is added to a scene or
	PxConstraint::setActors() keeps it selected. When PxConstraint::setActors() takes a recorded
	constraint out of the selection, this runs before the SDK removes the constraint from the
	recording, and isObjectRecorded() returns false. It can run more than once for the same change,
	so module code must tolerate repeated calls. Inside PxPhysics::createConstraint(), it runs before
	PxConstraintConnector::connectToConstraint() and before createConstraint() returns, so the
	connector's owner may not have stored the constraint pointer yet. It is not called when the
	constraint is released. Retire module data for a released constraint elsewhere, for example in
	PxConstraintConnector::onConstraintRelease().

	The callback runs with the writer locked and may write through getWriter(). It must not
	start/stop recording, alter the callback registry or modify simulation objects.
	It is not called for the initial snapshot.

	\param[in] omniPvd The recording instance.
	\param[in] object The shape or constraint whose recorded state changed.
	*/
	virtual void onObjectCapture(PxOmniPvd& omniPvd, const PxBase& object)
	{
		PX_UNUSED(omniPvd);
		PX_UNUSED(object);
	}
	virtual ~PxOmniPvdEventCallback() {}

	/**
	\brief Called by PxOmniPvd::processCaptureRequests() before an accepted receiver request replaces the recording.

	Destroy the module's recorded objects here. processCaptureRequests() then destroys the core
	objects, records the current state of the new selection and calls onStartSampling() so the
	module can record its objects again. The writer lock is not held. Use
	PxOmniPvd::ScopedExclusiveWriter for writer commands.
	startCaptureDiscovery() also calls this after stopSampling() when the same stream and connection
	still contain the previous recording. It then retires the core objects and writes only the catalog;
	onStartSampling() is deferred until a recording is started.

	\param[in] omniPvd The PxOmniPvd whose recording is being replaced.
	\param[in] filter Filter of the replacement recording, after PxOmniPvdCaptureFilter::filterCaptureRequest()
	and any active discovery scope have restricted it, or the new discovery scope when called by
	startCaptureDiscovery(). The descriptor and its arrays are valid only
	during this call. Copy any data needed after the callback returns.

	\see PxOmniPvd::processCaptureRequests() PxOmniPvd::startCaptureDiscovery() onStartSampling()
	*/
	virtual void onCaptureReset(PxOmniPvd& omniPvd, const PxOmniPvdFilterDesc& filter) { (void)omniPvd; (void)filter; }
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
	The returned writer is owned by this PxOmniPvd instance. Do not pass it to destroyOmniPvdWriter().
	
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
	\brief Starts recording to the bound write stream.

	First records the current state of the selected objects, then records their changes from this
	point on. Because the current state is recorded first, the newly written versioned segment
	contains the state needed to decode it from its starting boundary whether the scene is empty
	or has been running for a while. This is what makes a late attach work: you can bind a stream
	after the simulation has been running and the new segment contains the selected current state.

	Bind the write stream first with OmniPvdWriter::setWriteStream(), then call startSampling().
	To take a fresh recording later, call stopSampling(), bind a stream with setWriteStream(), then
	call startSampling() again. Rebinding resets the writer session but does not itself reset the
	transport: an already-open stream appends a versioned segment at its current position, so retain
	that boundary and position a fresh reader there. For one standalone recording decodable from
	byte zero, use a new or explicitly reset transport, or close and reopen a file writer so its
	truncating reopen policy applies. Calling startSampling() while already sampling is not allowed:
	it does not start a new recording and returns false with an error message. Call stopSampling()
	before starting another recording. For final teardown, sampling may remain active through
	PxPhysics::release() so object-removal notifications are recorded; releasing the associated
	PxPhysics implicitly ends sampling.
	Open a new TCP connection before recording a re-created PxPhysics instance. Otherwise a receiver
	request naming an object released with the earlier instance can select a new object allocated
	at the same address.

	\note The caller must synchronize this call with every scene. No scene may be between
	simulate() or collide() and the completion of its matching fetchResults() or fetchResultsFinish().
	Scene simulation, result fetching (including fetchResultsParticleSystem()), SDK object creation,
	release and scene membership changes must not run concurrently with this call. The scene-state
	checks report misuse but do not provide synchronization against another thread starting these operations.

	\param[in] filter Optional filter copied for this recording. NULL resets any previous selection and
	records all objects and all supported data, including after stopSampling(). While capture discovery is active, NULL records only the discovery scope.
	\return True if recording started. False if no PxPhysics is associated with this PxOmniPvd, the filter
	is invalid or lies outside an active discovery scope, recording was already on, any scene is between simulate() or collide() and the completion
	of its fetchResults() or fetchResultsFinish(), no writer is available, a stream write or GPU readback
	failed while recording the current state, or a registered callback stopped sampling.

	\see OmniPvdWriter::setWriteStream()
	\see PxOmniPvd::addEventCallback()
	\see PxOmniPvd::stopSampling()
	\see PxOmniPvd::isSampling()
	*/
	virtual bool startSampling(const PxOmniPvdFilterDesc* filter = NULL) = 0;

	/**
	\brief Starts an identity-only discovery session on the bound stream.

	Emits names, types, handles and ownership for eligible objects without geometry, motion,
	contacts or GPU readback. isSampling() remains false until an accepted selection request or a
	successful local startSampling() call starts selected recording. processCaptureRequests()
	processes selections and catalog refreshes in this state. stopSampling() ends both discovery
	and recording.

	The scope is copied and restricts catalog rows and subsequent remote selections. NULL or
	empty lists permit all objects. Releasing every scene and object listed in the scope never
	widens it. The scope uses only the scene and object lists, not recordSecondaryData. Each
	selection supplies its own data policy.
	A local startSampling() while waiting is also restricted to this scope.
	Names are display labels, not stable selectors. Handles are valid only for this connection
	and producer run. stopSampling() does not destroy the catalog rows. A later startCaptureDiscovery()
	on the same stream and connection destroys them before it writes its own catalog.
	If the stopped recording used that stream and connection, its core objects and registered modules'
	objects are retired through onCaptureReset() before the discovery catalog is written.

	The caller provides the same external synchronization as startSampling() and
	processCaptureRequests(). No scene may be between simulate() or collide() and the completion
	of its matching fetchResults() or fetchResultsFinish().
	\param[in] scope Optional maximum discovery and remote-selection scope.
	\return True when the initial catalog was written. False if recording or discovery is already
	active, the scope is invalid, any scene is between simulate() or collide() and the completion
	of its matching fetchResults() or fetchResultsFinish(), no write stream is bound or it cannot
	be opened, or writing the catalog failed.
	\see processCaptureRequests() hasPendingCaptureRequest() stopSampling()
	*/
	virtual bool startCaptureDiscovery(const PxOmniPvdFilterDesc* scope = NULL) = 0;

	/**
	\brief Whether the identity catalog session is active, including after selection starts recording.
	\return True from successful startCaptureDiscovery() until stopSampling() or PxPhysics release.
	\see startCaptureDiscovery() isSampling() stopSampling()
	*/
	virtual bool isCaptureDiscoveryActive() const = 0;

	/**
	\brief Checks a filter before it is passed to startSampling().

	Checks the array/count pairs, supported object types and that every listed scene and object
	belongs to this PxPhysics. While isCaptureDiscoveryActive() is true, every listed scene and
	object must also lie within the discovery scope, as startSampling() requires. If the scope
	lists scenes or objects, a filter with both lists empty fails once none of them can be selected.
	Returns false if a scene is between simulate() or collide() and the
	completion of its matching fetchResults() or fetchResultsFinish().
	Validation does not write to the stream or perform GPU transfers.

	\note The caller must provide the same synchronization as for startSampling(), including against
	concurrent fetchResultsParticleSystem() calls. Validation neither locks the scenes nor reserves
	the validated state. The result can change before startSampling() is called.

	\param[in] filter Filter to validate.
	\return True if the filter passes validation under the caller's synchronization, false otherwise.
	\see PxOmniPvdFilterDesc::isValid() startSampling()
	*/
	virtual bool validateFilterDesc(const PxOmniPvdFilterDesc& filter) const = 0;

	/**
	\brief Tests whether the active selection includes an object or requires it as a resource, constraint or joint.

	Includes objects selected by the scene/object lists and their required resources, constraints and joints.
	Returns false for actors recorded only to describe a connection to selected objects, and for the
	particle system recorded only as the owner of a selected particle buffer. Such actors can have
	recorded state even though this query returns false. The object must belong to the associated PxPhysics.
	Without a filter, every supported SDK object is included.

	\note This call does not acquire exclusive writer access, and the selection state is synchronized
	internally. It may run while another thread holds exclusive writer access or releases selected
	objects of a different scene. As for other read calls, the queried object and the objects it refers
	to (for example its scene, articulation, aggregate members, joint or constraint actors, or particle
	system) must not be modified or released concurrently.

	\param[in] object Object to test.
	\return True if the active selection includes the object or requires it as a resource, constraint or
	joint. False otherwise, including when sampling is stopped.
	\see startSampling() stopSampling()
	*/
	virtual bool isObjectRecorded(const PxBase& object) const = 0;

	/**
	\brief Stops recording and any identity discovery session.

	After this call, no further object or per-frame data is written. The write stream is not
	flushed or closed; you own the stream and decide when to flush or close it. Call stopSampling()
	before starting another recording: a later startSampling() then records the current state again
	onto the (re-)bound stream. It is not a resume. Calling stopSampling() is optional during final
	teardown: leaving sampling active through PxPhysics::release() records object-removal
	notifications, and releasing the associated PxPhysics implicitly ends sampling. It is safe to
	call stopSampling() after that PxPhysics has been released; the call returns false because its
	associated sampler state is no longer available.
	Stopping an associated recording discards a completed receiver request that has not been processed, without an acknowledgement.
	Requests the stream has not yet delivered are unaffected.

	\note Call at an application safe point without concurrent SDK object creation or release.
	Capture-owned particle host mirrors are kept after stopping. The next recording snapshot releases
	the mirrors of particle buffers the new recording does not include, and the diffuse particle mirrors
	when the new recording's recordSecondaryData is false. The remaining mirrors are freed when the particle
	buffer is released, including when PxPhysics is released.

	\return True if the sampling state was set to false, false if no associated sampler state was
	available.

	\see PxOmniPvd::startSampling()
	*/
	virtual bool stopSampling() = 0;

	/**
	\brief Whether recording is currently on.

	True after a successful startSampling() until stopSampling() is called or the associated
	PxPhysics is released.

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

	\param[in] callback The callback to register.

	\see PxOmniPvd::removeEventCallback()
	*/
	virtual void addEventCallback(PxOmniPvdEventCallback& callback) = 0;

	/**
	\brief Unregisters a previously registered callback.

	Has no effect if the callback was not registered.

	\note Not thread-safe; see addEventCallback().

	\param[in] callback The callback to unregister.

	\see PxOmniPvd::addEventCallback()
	*/
	virtual void removeEventCallback(PxOmniPvdEventCallback& callback) = 0;

	/**
	\brief Polls and processes at most one receiver request on the recording's write stream.

	No scene may be between simulate() or collide() and the completion of its matching fetchResults()
	or fetchResultsFinish(). Otherwise the request is acknowledged as busy. The caller must
	synchronize this call with simulation, result fetching (including fetchResultsParticleSystem()),
	SDK object creation, release and scene membership changes. Validation does not lock or reserve
	the scenes and cannot prevent another thread from starting simulation.
	Object handles received from the stream are matched to live SDK objects of the types supported
	by PxOmniPvdFilterDesc before the optional PxOmniPvdCaptureFilter runs.
	An accepted selection request destroys the recorded objects, then records the current state of the new
	selection on the same stream. Acknowledgements are PxOmniPvdMetaData.captureRequestId,
	captureRequestStatus and captureEpoch attributes. Discovery refreshes preserve the recording
	and epoch; they are denied unless startCaptureDiscovery() enabled the catalog session.

	\param[in] filter Optional filter that restricts or denies requests that passed the SDK checks.
	NULL applies every request that passes those checks.
	\return True for no request or an acknowledged request. False if inactive or if the transport
	or recording of the current state failed.

	\see PxOmniPvdCaptureFilter hasPendingCaptureRequest()
	*/
	virtual bool processCaptureRequests(PxOmniPvdCaptureFilter* filter = NULL) = 0;

	/**
	\brief Performs a bounded nonblocking poll and retains a completed receiver request.

	Does not enumerate SDK objects or change the recording filter. Concurrent calls to this method are
	serialized by the OmniPVD writer lock; serialize calls with recording lifecycle changes and
	processCaptureRequests(). A request remains buffered until processCaptureRequests() runs at an
	SDK safe point. Stopping an associated recording or starting a new one discards a buffered request
	without acknowledging it; requests the stream has not yet delivered are unaffected. Orderly
	receiver half-close ends control while telemetry continues.
	\return True when a buffered request or terminal stream/control failure needs safe-point processing.
	False when inactive or no processing is needed, including a healthy stream without control support.

	\see processCaptureRequests()
	*/
	virtual bool hasPendingCaptureRequest() = 0;

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

\param[in] foundation Foundation instance (see PxFoundation)

\return PxOmniPvd instance on succes, NULL otherwise.

*/
PX_C_EXPORT PX_PHYSX_CORE_API physx::PxOmniPvd* PX_CALL_CONV PxCreateOmniPvd(physx::PxFoundation& foundation);


#endif
