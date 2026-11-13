// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <PvdDomInspection.h>
#include <OmniPvdLibraryFunctions.h>
#ifdef TEST_PHYSX_PACKAGE
#include <PxPhysicsAPI.h>
#endif

int main(int argc, char** argv)
{
#ifdef TEST_PHYSX_PACKAGE
    physx::PxDefaultAllocator allocator;
    physx::PxDefaultErrorCallback errors;
    physx::PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errors);
    if (!foundation) return 1;
    physx::PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale());
    if (!physics) { foundation->release(); return 1; }
    const bool validScale = physics->getTolerancesScale().isValid();
    physics->release(); foundation->release();
    if (!validScale) return 1;
#endif
    PvdDomInspect::Document document;
    PvdDomInspect::Error error;
    if (argc > 1)
    {
        if (!document.load(argv[1], PvdDomInspect::LoadLimits(), error) || !document.dom()) return 1;
    }
    else if (document.load("nonexistent-consumer-input.ovd", PvdDomInspect::LoadLimits(), error)) return 1;
    const auto result = PvdDomInspect::compareLinearVelocity(document, document, PvdDomInspect::ComparisonOptions());
    const std::string json = PvdDomInspect::evidenceToJson(result, {}, {});
    return result.status == PvdDomInspect::ComparisonStatus::Incomparable && !json.empty() ? 0 : 1;
}
