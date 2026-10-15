// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

_Static_assert(sizeof(ovphysx_instance_handle_t) == sizeof(uint64_t),
               "instance handle ABI size");

ovphysx_result_t enable_ovd_recording_capability(void)
{
    return ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true));
}
