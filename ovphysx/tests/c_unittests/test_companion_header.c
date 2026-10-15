// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-COMPANION-001
// @covers AC-1

#include <ovphysx_companion/companion.h>
#include <ovphysx_companion/preview.h>
#include <ovphysx_companion/mesh.h>
#include <ovphysx_companion/native.h>
#include <ovphysx_companion/cooking.h>
#include <ovphysx_companion/cct.h>
#include <ovphysx_companion/debug.h>
#include <ovphysx_companion/ovd.h>
#include <ovphysx_companion/events.h>
#include <ovphysx_companion/inspection.h>
#include <ovphysx_companion/interaction.h>
#include <ovphysx_companion/particles.h>
#include <ovphysx_companion/queries.h>
#include <ovphysx_companion/mass.h>
#include <ovphysx_companion/configuration.h>
#include <ovphysx_companion/vehicle.h>

_Static_assert(sizeof(ovphysx_companion_status_t) == 4, "Status must be uint32");
_Static_assert(sizeof(ovphysx_companion_session_t) == 8, "Session must be uint64");
_Static_assert(offsetof(ovphysx_companion_api_t, capabilities) == 8, "Table header layout changed");

/* Compiled as C11 without linking the runtime or including its private headers. */
ovphysx_companion_status_t companion_c_table_call(const ovphysx_companion_api_t* api,
    ovphysx_companion_session_t session, ovphysx_companion_identity_t* output)
{
    return api->get_identity(session, output);
}
