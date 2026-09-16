// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// NOTE: This file is included in the documentation via literalinclude.
// The tutorial marker comments below define the included range.

// [tutorial-start]
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "ovstage_sample.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

// Count files matching *_rec.ovd in the given directory.
static int count_ovd_files(const fs::path& dir) {
    int count = 0;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        const auto name = entry.path().filename().string();
        if (name.size() > 8 && name.substr(name.size() - 8) == "_rec.ovd")
            ++count;
    }
    return count;
}

static int run(void)
{
    // Create a fresh output directory for this invocation in the caller-owned working directory.
    std::error_code ec;
    const fs::path working_dir = fs::current_path(ec);
    if (ec) {
        const std::string reason = ec.message();
        fprintf(stderr, "Failed to resolve the working directory: %s\n", reason.c_str());
        return 1;
    }

    const std::chrono::steady_clock::rep unique_suffix =
        std::chrono::steady_clock::now().time_since_epoch().count();
    fs::path output_dir =
        working_dir / ("ovphysx_pvd_cpp_sample_" + std::to_string(unique_suffix));
    if (!fs::create_directory(output_dir, ec)) {
        const std::string reason = ec ? ec.message() : "directory already exists";
        fprintf(stderr, "Failed to create directory '%s': %s\n",
                output_dir.string().c_str(), reason.c_str());
        return 1;
    }
    std::string dir_str = output_dir.string();
    printf("OmniPVD recording directory: %s\n", dir_str.c_str());

    // Configure OmniPVD recording via typed config entries. Both have to be set
    // before instance creation, because the recording pipeline initializes
    // during physics engine startup.
    ovphysx_config_entry_t config[] = {
        ovphysx_config_entry_omnipvd_ovd_recording_directory(ovphysx_cstr(dir_str.c_str())),
        ovphysx_config_entry_omnipvd_output_enabled(true),
    };

    ovphysx_create_args create_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    create_args.config_entries = config;
    create_args.config_entry_count = 2;
    ovphysx_handle_t handle = 0;

    ovphysx_result_t r = ovphysx_initialize();
    if (r.status != OVPHYSX_API_SUCCESS) {
        ovphysx_string_t err = ovphysx_get_last_error();
        fprintf(stderr, "Failed to initialize ovphysx: %.*s\n",
                (int)(err.ptr ? err.length : 0), err.ptr ? err.ptr : "");
        return 1;
    }

    r = ovphysx_create_instance(&create_args, &handle);
    if (r.status != OVPHYSX_API_SUCCESS) {
        ovphysx_string_t err = ovphysx_get_last_error();
        fprintf(stderr, "Failed to create PhysX instance: %.*s\n",
                (int)(err.ptr ? err.length : 0), err.ptr ? err.ptr : "");
        ovphysx_shutdown();
        return 1;
    }

    ovphysx_sample_stage_attachment_t stage_attachment = {};
    if (!ovphysx_sample_attach_usd_with_ovstage(
            handle, OVPHYSX_TEST_DATA "/simple_physics_scene.usda", &stage_attachment)) {
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    // Run simulation steps. OmniPVD records each frame.
    const float dt = 1.0f / 60.0f;
    const int n_steps = 10;
    printf("Running %d simulation steps...\n", n_steps);

    for (int i = 0; i < n_steps; i++) {
        ovphysx_result_t step_r = ovphysx_step_sync(handle, dt);
        if (step_r.status != OVPHYSX_API_SUCCESS) {
            ovphysx_string_t err = ovphysx_get_last_error();
            fprintf(stderr, "Step %d failed: %.*s\n", i,
                    (int)(err.ptr ? err.length : 0), err.ptr ? err.ptr : "");
            ovphysx_sample_destroy_stage(handle, &stage_attachment);
            ovphysx_destroy_instance(handle);
            ovphysx_shutdown();
            return 1;
        }
    }

    printf("Simulation complete.\n");

    // Destroying the instance finalizes the recording. tmp.ovd is renamed to a
    // timestamped *_rec.ovd file.
    ovphysx_sample_destroy_stage(handle, &stage_attachment);
    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();

    // At least one OVD file has to exist. A non-zero return lets CI catch regressions.
    int ovd_count = count_ovd_files(output_dir);
    if (ovd_count > 0) {
        printf("Recorded %d OVD file(s) in %s\n", ovd_count, dir_str.c_str());
        printf("Runtime cleanup complete; recording retained for inspection\n");
        return 0;
    }

    fprintf(stderr, "FAIL: No OVD files found in %s\n", dir_str.c_str());
    return 1;
}

int main(void) {
    int rc = run();
    return rc;
}
// [tutorial-end]
