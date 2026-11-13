// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_CAPTURE_REQUEST_H
#define OMNI_PVD_CAPTURE_REQUEST_H

#include "OmniPvdDefines.h"
#include <stddef.h>

/** \brief Result of a nonblocking producer-side capture request poll. */
struct OmniPvdCaptureRequestResult
{
	enum Enum { eNONE, eREQUEST, eERROR };
};

/**
 * \brief Receiver-to-producer selection or catalog request using the OmniPVD stream.
 *
 * For selection requests, scenes and objects form a union. When both lists are empty, the
 * request selects every object that the producer permits. Selected objects always include their primary data
 * (poses, velocities and required geometry/materials). recordSecondaryData also includes parameters, forces, contacts,
 * articulation joint solver reactions and module telemetry.
 * Objects that can be selected follow PxOmniPvdFilterDesc: rigid actors, whole articulations, deformables,
 * particle systems and particle buffers. Links, joints, aggregates and resources cannot be selected directly.
 * Requests never confer authority over an object: the producer validates every handle against
 * live objects, and the application's PxOmniPvdCaptureFilter can deny or restrict the request. Maximum total selector count is 65536.
 *
 * requestCapture() borrows arrays only during its call. pollCaptureRequest() returns arrays owned
 * by the stream, valid until the next poll or close. A successful send means pending, not applied.
 * requestCaptureCatalog() requests current eligible handles without changing the selection.
 * pollCaptureRequest() identifies these requests with catalogOnly and empty selector lists.
 * PhysX acknowledges through ordinary PxOmniPvdMetaData.captureRequestId/captureRequestStatus/captureEpoch
 * attributes: 1 applied, 2 invalid, 3 busy, 4 denied. Read those through OmniPvdReader.
 * Handles are process-local addresses, valid only for the current producer run and connection.
 * Never persist them as saved selections or reuse them after reconnecting or restarting the producer.
 * Names are not unique SDK identifiers; this protocol does not resolve names or USD paths.
 * Resolve saved application identifiers on the producer for each new run.
 * Handles of SDK objects destroyed while PhysX is recording or a PxOmniPvd::startCaptureDiscovery()
 * session is active are retired for the connection, even if a later object reuses the address.
 * Releases while neither is active are not retired.
 * Open a new connection and discover fresh handles to select such replacements.
 * Retirement history does not survive re-creating PxPhysics on the same connection;
 * open a new connection after re-creating PxPhysics.
 */
struct OmniPvdCaptureRequest
{
	OmniPvdCaptureRequest() : requestId(0), recordSecondaryData(true), catalogOnly(false), scenes(NULL), nbScenes(0), objects(NULL), nbObjects(0) {}
	uint64_t requestId; //!< Nonzero and monotonically increasing within one TCP connection.
	bool recordSecondaryData; //!< True includes secondary data. Ignored for catalog requests.
	bool catalogOnly; //!< Decoded catalog request; send with requestCaptureCatalog(), not requestCapture().
	const OmniPvdObjectHandle* scenes;
	uint32_t nbScenes;
	const OmniPvdObjectHandle* objects;
	uint32_t nbObjects;
};

#endif
