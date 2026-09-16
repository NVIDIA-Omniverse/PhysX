// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

// Concrete USD deformable auto-attachment author (ADR-0027): authors the Attachment /
// ElementCollisionFilter child prims through the omni::physics::usd authoring family and the
// deformable schema tokens. Lives inside the test-only USD library so it may name pxr types
// freely; the pxr-free AttachmentAuthoringBridge.cpp reaches it only through
// parse::IUsdAttachmentAuthor, handing over resolved string paths (rebuilt into SdfPath here)
// so this side never needs the active source.

#include <omni/physics/usd/UsdAttachmentAuthor.h>

#include <omni/physics/usd/UsdDeformableAttachmentWrite.h>
#include <omniUsdPhysicsDeformableSchema/tokens.h>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

#include <memory>

namespace omni::physics::usd
{

namespace
{

const PXR_NS::UsdStageWeakPtr& stageAt(const void* handleStorage)
{
    return *reinterpret_cast<const PXR_NS::UsdStageWeakPtr*>(handleStorage);
}

// autoAttachmentPath/childName -> `<root>/<childName>`, mirroring the USD arm's childPath():
// empty root (unresolvable key) yields an empty path and the authoring call below no-ops.
PXR_NS::SdfPath childPath(const char* autoAttachmentPath, const char* childName)
{
    const PXR_NS::SdfPath root(autoAttachmentPath ? autoAttachmentPath : "");
    return root.IsEmpty() ? root : root.AppendElementString(childName);
}

const PXR_NS::TfToken& primTypeToken(parse::UsdAttachmentPrimKind kind)
{
    switch (kind)
    {
    case parse::UsdAttachmentPrimKind::eVtxVtx:
        return PXR_NS::OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsVtxVtxAttachment;
    case parse::UsdAttachmentPrimKind::eVtxTri:
        return PXR_NS::OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsVtxTriAttachment;
    case parse::UsdAttachmentPrimKind::eVtxTet:
        return PXR_NS::OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsVtxTetAttachment;
    case parse::UsdAttachmentPrimKind::eVtxXform:
    default:
        return PXR_NS::OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsVtxXformAttachment;
    }
}

class UsdAttachmentAuthor final : public parse::IUsdAttachmentAuthor
{
public:
    bool canAuthor(const void* handleStorage) const override
    {
        return bool(stageAt(handleStorage));
    }

    void removeAttachmentsAndFilters(const void* handleStorage, const char* autoAttachmentPath) override
    {
        omni::physics::usd::removeAttachmentsAndFilters(stageAt(handleStorage),
                                                        PXR_NS::SdfPath(autoAttachmentPath ? autoAttachmentPath : ""));
    }

    void disableAttachmentsAndFilters(const void* handleStorage, const char* autoAttachmentPath) override
    {
        omni::physics::usd::disableAttachmentsAndFilters(stageAt(handleStorage),
                                                         PXR_NS::SdfPath(autoAttachmentPath ? autoAttachmentPath : ""));
    }

    void defineAttachmentPrim(const void* handleStorage,
                              const char* autoAttachmentPath,
                              const char* childName,
                              parse::UsdAttachmentPrimKind kind,
                              const char* body0Path,
                              const char* body1Path) override
    {
        omni::physics::usd::defineAttachmentPrim(stageAt(handleStorage),
                                                 childPath(autoAttachmentPath, childName), primTypeToken(kind),
                                                 PXR_NS::SdfPath(body0Path ? body0Path : ""),
                                                 PXR_NS::SdfPath(body1Path ? body1Path : ""));
    }

    void defineElementCollisionFilterPrim(const void* handleStorage,
                                          const char* autoAttachmentPath,
                                          const char* childName,
                                          const char* body0Path,
                                          const char* body1Path) override
    {
        omni::physics::usd::defineElementCollisionFilterPrim(stageAt(handleStorage),
                                                             childPath(autoAttachmentPath, childName),
                                                             PXR_NS::SdfPath(body0Path ? body0Path : ""),
                                                             PXR_NS::SdfPath(body1Path ? body1Path : ""));
    }
};

} // namespace

std::unique_ptr<omni::physics::parse::IUsdAttachmentAuthor> makeUsdAttachmentAuthor()
{
    return std::make_unique<UsdAttachmentAuthor>();
}

} // namespace omni::physics::usd
