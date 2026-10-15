// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-INSPECTION-001
// @covers AC-1 AC-2 AC-4
#pragma once
#include <omni/physx/NativeInspection.h>
#include "internal/InternalPhysXDatabase.h"
#include "usdLoad/AttachedStage.h"
namespace omni::physx::native
{
// Private helpers: caller already holds exact-owner runtime admission. Never
// acquire it recursively and never retain a returned record after releasing it.
ObjectReference objectReference(const usdparser::AttachedStage&,uint64_t object);
const internal::InternalDatabase::Record* objectRecord(const usdparser::AttachedStage&,const ObjectReference&);
bool matchesObjectKind(PhysXType type,ObjectKind kind);
}
