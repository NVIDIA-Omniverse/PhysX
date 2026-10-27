-- SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- copy files into exts folder
repo_build.prebuild_copy
{
    { "generatedSchema.usda",  root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/share/usd/plugins/OmniUsdPhysicsDeformableSchema/resources" },
    { "schema.usda",  root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/share/usd/plugins/OmniUsdPhysicsDeformableSchema/resources" },
    { "plugInfo.json",  root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/share/usd/plugins/OmniUsdPhysicsDeformableSchema/resources" },
}
