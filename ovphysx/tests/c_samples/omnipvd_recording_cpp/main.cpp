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

// Count files matching *.ovd in the given directory.
static int count_ovd_files(const fs::path& dir) {
    int count = 0;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        const auto name = entry.path().filename().string();
        if (name.size() > 4 && name.substr(name.size() - 4) == ".ovd")
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

    ovphysx_instance_handle_t handle = 0;

    ovphysx_result_t r = ovphysx_initialize();
    if (r.status != OVPHYSX_API_SUCCESS) {
        ovphysx_string_t err = ovphysx_get_last_error();
        fprintf(stderr, "Failed to initialize ovphysx: %.*s\n",
                (int)(err.ptr ? err.length : 0), err.ptr ? err.ptr : "");
        return 1;
    }

    r = ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE, ovphysx_config_value_bool(true));
    if (r.status != OVPHYSX_API_SUCCESS) {
        const ovphysx_string_t err = ovphysx_get_last_error();
        fprintf(stderr, "Failed to enable OVD recording capability: %.*s\n",
                static_cast<int>(err.length), err.ptr ? err.ptr : "");
        ovphysx_shutdown();
        return 1;
    }

    r = ovphysx_create_instance(&handle);
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

    const std::string file_path = (output_dir / "capture.ovd").string();
    ovphysx_omnipvd_destination_t destination = {};
    destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_FILE;
    destination.file_path = ovphysx_cstr(file_path.c_str());
    r = ovphysx_ovd_start_recording(handle, &destination);
    if (r.status != OVPHYSX_API_SUCCESS) {
        const ovphysx_string_t err = ovphysx_get_last_error();
        fprintf(stderr, "Failed to start OVD recording: %.*s\n", static_cast<int>(err.length), err.ptr);
        ovphysx_sample_destroy_stage(handle, &stage_attachment);
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

    r = ovphysx_ovd_stop_recording(handle);
    const bool stopped = r.status == OVPHYSX_API_SUCCESS;
    // Finalize before releasing the borrowed stage.
    ovphysx_sample_destroy_stage(handle, &stage_attachment);
    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();

    // At least one OVD file has to exist. A non-zero return lets CI catch regressions.
    int ovd_count = count_ovd_files(output_dir);
    if (stopped && ovd_count > 0) {
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
