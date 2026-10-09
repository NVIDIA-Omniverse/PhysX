// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
// All rights reserved. SPDX-License-Identifier: Apache-2.0
// @implements REQ-CAPI-IMPORT-MEMORY-REPRO-001
#include <ovphysx/ovphysx.h>

#include <ovstage/ovstage.h>
#include <ovx/path_dictionary/path_dictionary.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <malloc.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

void Physics(ovphysx_result_t result)
{
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_string_t error = ovphysx_get_last_error();
        throw std::runtime_error(std::string(error.ptr ? error.ptr : "", error.length));
    }
}
int Env(const char* name, int fallback)
{
    const char* value = std::getenv(name);
    return value ? std::stoi(value) : fallback;
}
void Sample(const char* phase, int iteration)
{
    std::ifstream statm("/proc/self/statm");
    size_t pages = 0, resident = 0;
    statm >> pages >> resident;
    const struct mallinfo2 heap = mallinfo2();
    std::cout << "MEMORY {\"phase\":\"" << phase << "\",\"iteration\":" << iteration
              << ",\"rss_bytes\":" << resident * sysconf(_SC_PAGESIZE) << ",\"allocated_bytes\":" << heap.uordblks
              << ",\"mmap_bytes\":" << heap.hblkhd << "}" << std::endl;
}
void Check(bool ok, const char* message)
{
    if (!ok)
        throw std::runtime_error(message);
}
void Complete(ovstage_instance_t* stage, ovstage_enqueue_result_t op)
{
    Check(op.status == OVSTAGE_OK, "enqueue failed");
    ovstage_op_wait_result_t result{};
    Check(ovstage_wait_op(stage, op.op_index, 30'000'000'000ULL, &result) == OVSTAGE_OK, "wait failed");
    Check(result.error_op_id_count == 0, "operation failed");
    Check(ovstage_release_op(stage, op.op_index) == OVSTAGE_OK, "release failed");
}
int main(int argc, char** argv)
try
{
    Check(argc == 2, "expected scene path");
    int iterations = Env("PROBE_ITERATIONS", 2000);
    int bodies = Env("PROBE_BODIES", 4);
    bool write = Env("PROBE_WRITE", 1), drain = Env("PROBE_DRAIN", 1);
    bool alternate = Env("PROBE_ALTERNATE", 1), mass = Env("PROBE_MASS", 0);
    Check(iterations >= 0 && bodies > 0 && bodies <= 4, "invalid parameters");
    Sample("before_create", 0);
    Physics(ovphysx_set_cpu_mode(true));
    Physics(ovphysx_initialize());
    ovphysx_handle_t physics = 0;
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    Physics(ovphysx_create_instance(&args, &physics));
    Check(ovstage_initialize(nullptr) == OVSTAGE_OK, "stage initialize failed");
    ovphysx_string_t schemas{};
    Physics(ovphysx_get_codeless_schema_root(&schemas));
    ovx_string_t schema_path{ schemas.ptr, schemas.length };
    Check(ovstage_population_register_usd_schemas(&schema_path, 1) == OVSTAGE_OK, "schema registration failed");
    ovstage_instance_t* stage = nullptr;
    ovstage_instance_desc_t desc{};
    desc.name = "control-drain-repro";
    Check(ovstage_create_instance(&desc, &stage) == OVSTAGE_OK, "stage create failed");
    std::string scene = argv[1];
    ovstage_population_enqueue_result_t population = ovstage_population_open_usd_from_file(
        stage, { scene.data(), scene.size() }, 1, 0.0, OVSTAGE_POPULATION_DOMAIN_ALL);
    Check(population.status == OVSTAGE_OK, "population enqueue failed");
    ovstage_population_op_wait_result_t populated{};
    Check(ovstage_population_wait_op(stage, population.op_index, 30'000'000'000ULL, &populated) == OVSTAGE_OK &&
              populated.error_op_id_count == 0,
          "population failed");
    ovstage_write_floor_desc_t initial{};
    initial.ordinal = 1;
    initial.scope = OVSTAGE_SCOPE_ALL;
    Complete(stage, ovstage_advance_write_floor(stage, &initial));
    Physics(ovphysx_attach_ovstage(physics, stage, 1));
    if (Env("PROBE_WARMUP", 1))
        Physics(ovphysx_warmup(physics));
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    std::vector<std::string> names;
    for (int i = 0; i < bodies; ++i)
        names.push_back("/World/Box_" + std::to_string(i));
    std::vector<ovx_string_t> paths;
    for (std::string& name : names)
        paths.push_back({ name.data(), name.size() });
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    Check(dict->vtable->create_path_list_from_strings(dict->context, paths.data(), paths.size(), &list).status ==
              OVX_API_SUCCESS,
          "path list failed");
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    Check(ovstage_query_from_path_list(stage, list, &query) == OVSTAGE_OK, "query failed");
    dict->vtable->release_path_list_reference(dict->context, list);
    std::vector<double> translations(bodies * 3);
    std::vector<float> masses(bodies);
    Sample("ready", 0);
    for (int i = 0; i < iterations; ++i)
    {
        const ovstage_ordinal_t ordinal = static_cast<uint64_t>(i) + 2;
        const double delta = alternate ? (i % 2) * 0.1 : 0.0;
        for (int row = 0; row < bodies; ++row)
        {
            translations[row * 3] = row * 2.0;
            translations[row * 3 + 2] = 5.0 + delta;
            masses[row] = 1.0f + static_cast<float>(delta);
        }
        if (write)
        {
            int64_t shape[] = { bodies };
            DLTensor tensor{};
            tensor.data = mass ? static_cast<void*>(masses.data()) : translations.data();
            tensor.device = { kDLCPU, 0 };
            tensor.ndim = 1;
            tensor.dtype = mass ? DLDataType{ kDLFloat, 32, 1 } : DLDataType{ kDLFloat, 64, 3 };
            tensor.shape = shape;
            ovstage_write_data_t data{};
            data.tensors = &tensor;
            data.tensor_count = 1;
            data.count = bodies;
            std::string name = mass ? "physics:mass" : "xformOp:translate";
            ovx_string_or_token_t attr{};
            attr.string = { name.data(), name.size() };
            Complete(stage, ovstage_write_attribute(stage, query, attr, ordinal, data, OVSTAGE_PRIM_MODE_UPSERT));
        }
        ovstage_write_floor_desc_t floor{};
        floor.ordinal = ordinal;
        floor.scope = OVSTAGE_SCOPE_ALL;
        Complete(stage, ovstage_advance_write_floor(stage, &floor));
        if (drain)
        {
            ovstage_ordinal_range_t range{};
            range.has_start_ordinal = true;
            range.start_ordinal = range.end_ordinal = ordinal;
            Physics(ovphysx_update_from_ovstage(physics, range));
        }
        if ((i + 1) % 400 == 0 || i + 1 == iterations)
            Sample("running", i + 1);
    }
    Complete(stage, ovstage_release_query(stage, query));
    Sample("after_query_release", iterations);
    Physics(ovphysx_detach_ovstage(physics));
    Physics(ovphysx_destroy_instance(physics));
    Sample("after_physics_destroy", iterations);
    Check(ovstage_destroy_instance(stage) == OVSTAGE_OK, "stage destroy failed");
    Sample("after_stage_destroy", iterations);
    Check(ovstage_shutdown() == OVSTAGE_OK, "stage shutdown failed");
    Physics(ovphysx_shutdown());
    Sample("after_shutdown", iterations);
    malloc_trim(0);
    Sample("after_trim", iterations);
    return 0;
}
catch (const std::exception& error)
{
    std::cerr << error.what() << std::endl;
    return 1;
}
