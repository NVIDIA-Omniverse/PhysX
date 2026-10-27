-- SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

group "schema"
    dofile ("source/physxSchema/premake5.lua")
    dofile ("source/physxSchemaAddition/premake5.lua")
    dofile ("source/physicsSchemaTools/premake5.lua")
    dofile ("source/omniUsdPhysicsDeformableSchema/premake5.lua")
