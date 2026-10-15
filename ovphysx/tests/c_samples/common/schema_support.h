// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/* Application-side discovery for the separately installed physx-usd-schemas
 * wheel. Native applications may unpack the wheel without installing Python.
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2 AC-3
 */
#ifndef OVPHYSX_SAMPLE_SCHEMA_SUPPORT_H
#define OVPHYSX_SAMPLE_SCHEMA_SUPPORT_H

#include <stdio.h>
#include <stdlib.h>

#define PHYSX_SAMPLE_SCHEMA_PATH_SIZE 4096

typedef struct physx_sample_schema_paths_t
{
    char resources[2][PHYSX_SAMPLE_SCHEMA_PATH_SIZE];
    size_t lengths[2];
} physx_sample_schema_paths_t;

/* root is the extracted provider wheel directory, containing physx_usd_schemas/. */
static inline int physx_sample_schema_paths(
    const char* root, physx_sample_schema_paths_t* paths, char* error, size_t error_size)
{
    const char* modules[] = { "PhysxSchema", "OmniUsdPhysicsDeformableSchema" };
    const char* files[] = { "plugInfo.json", "generatedSchema.usda" };
    char filename[PHYSX_SAMPLE_SCHEMA_PATH_SIZE];
    int count;
    size_t module, file;
    if (!root || !root[0] || !paths)
    {
        snprintf(error, error_size, "Install physx-usd-schemas"
                 " separately and set PHYSX_USD_SCHEMAS_ROOT to its extracted wheel directory.");
        return 0;
    }
    for (module = 0; module < 2; ++module)
    {
        count = snprintf(paths->resources[module], sizeof(paths->resources[module]),
                         "%s/physx_usd_schemas/%s/resources", root, modules[module]);
        if (count < 0 || (size_t)count >= sizeof(paths->resources[module]))
        {
            snprintf(error, error_size, "PhysX schema provider path is too long: %s", root);
            return 0;
        }
        paths->lengths[module] = (size_t)count;
        for (file = 0; file < 2; ++file)
        {
            FILE* resource;
            count = snprintf(filename, sizeof(filename), "%s/%s", paths->resources[module], files[file]);
            if (count < 0 || (size_t)count >= sizeof(filename))
            {
                snprintf(error, error_size, "PhysX schema resource path is too long: %s", root);
                return 0;
            }
            resource = fopen(filename, "rb");
            if (!resource)
            {
                snprintf(error, error_size, "Incomplete PhysX schema provider: missing %s/resources/%s; reinstall "
                         "physx-usd-schemas.", modules[module], files[file]);
                return 0;
            }
            fclose(resource);
        }
    }
    if (error_size)
        error[0] = '\0';
    return 1;
}

#endif
