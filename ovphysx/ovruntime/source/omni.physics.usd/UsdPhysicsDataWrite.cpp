// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-WRITE-CORE-001
 * @covers AC-3 AC-4 AC-5 AC-8
 *
 * @implements REQ-WRITE-TRANSFORM-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-WRITE-DATA-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-WRITE-ARRAY-001
 * @covers AC-3 AC-4 AC-5
 */
#include <cstring>
#include "UsdPhysicsDataWrite.h"

#include "UsdSource.h"
#include "UsdXformHelpers.h"

#include <foundation/PxVec3.h>
#include <foundation/PxQuat.h>

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdGeom/xformOp.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>
#include <pxr/usd/usdPhysics/tokens.h>
#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/matrix3d.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/quath.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3h.h>
#include <pxr/base/vt/array.h>

#include <carb/logging/Log.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace PXR_NS;

namespace omni::physics::usd
{

namespace
{
GfQuatf toQuatf(const ::physx::PxQuat& q)
{
    // PxQuat stores (x, y, z, w); GfQuatf is (real=w, imaginary=(x, y, z)).
    return GfQuatf(q.w, GfVec3f(q.x, q.y, q.z));
}

// Read quaternion element `i` from a source column and return it as a `GfQuatf`
// (Gf real-first). The sink's quaternion contract is uniform: source elements
// are in PxQuat order (xyzw, real-last) — the engine-native layout — at the
// view's precision; the USD/Gf real-first layout is produced here, not by the
// caller. This is the array/scatter analogue of `toQuatf` for `writeTransforms`.
GfQuatf readQuatXYZW(const uint8_t* base, size_t i, size_t stride, bool half)
{
    const uint8_t* p = base + i * stride;
    if (half)
    {
        const GfHalf* h = reinterpret_cast<const GfHalf*>(p);
        return GfQuatf(float(h[3]), GfVec3f(float(h[0]), float(h[1]), float(h[2])));
    }
    const float* f = reinterpret_cast<const float*>(p);
    return GfQuatf(f[3], GfVec3f(f[0], f[1], f[2]));
}

// Compose a TRS matrix (scale, then rotate, then translate) for the fallback /
// single-transform-op paths.
GfMatrix4d composeMatrix(const ::physx::PxVec3& p, const ::physx::PxQuat& q, const ::physx::PxVec3* s)
{
    GfTransform tr;
    if (s)
        tr.SetScale(GfVec3d(s->x, s->y, s->z));
    tr.SetRotation(GfRotation(GfQuatd(q.w, GfVec3d(q.x, q.y, q.z))));
    tr.SetTranslation(GfVec3d(p.x, p.y, p.z));
    return tr.GetMatrix();
}

// Scatter a decomposed pose onto an existing xform-op stack, matching the
// long-standing behavior of InternalScene::setPrimXformOps(prim, transform):
// honor authored translate / orient / scale ops at their authored precision
// (writing the quaternion directly, no matrix round-trip), and fall back to a
// single matrix op when the stack can't represent the pose. Scale is written
// only when `scale` is non-null and a scale op exists.
void scatterTransform(UsdPrim& prim, const ::physx::PxVec3& p, const ::physx::PxQuat& q, const ::physx::PxVec3* s)
{
    UsdGeomXformable primXform(prim);
    bool resetXformStack = false;
    bool translateSet = false;
    bool orientSet = false;

    const std::vector<UsdGeomXformOp> xformOps = primXform.GetOrderedXformOps(&resetXformStack);
    for (const UsdGeomXformOp& op : xformOps)
    {
        const UsdGeomXformOp::Type opType = op.GetOpType();
        const UsdGeomXformOp::Precision opPrecision = op.GetPrecision();

        if (opType == UsdGeomXformOp::TypeTransform)
        {
            op.Set(composeMatrix(p, q, s));
            return;
        }
        else if (opType == UsdGeomXformOp::TypeTranslate && !translateSet)
        {
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(GfVec3f(p.x, p.y, p.z));
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfVec3d(p.x, p.y, p.z));
            translateSet = true;
        }
        else if (opType == UsdGeomXformOp::TypeScale && s)
        {
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(GfVec3f(s->x, s->y, s->z));
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfVec3d(s->x, s->y, s->z));
        }
        else if (opType == UsdGeomXformOp::TypeOrient && !orientSet)
        {
            const GfQuatf quat = toQuatf(q);
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(quat);
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfQuatd(quat));
            else if (opPrecision == UsdGeomXformOp::PrecisionHalf)
                op.Set(GfQuath(quat));
            orientSet = true;
        }
        else if (opType == UsdGeomXformOp::TypeRotateZYX && !orientSet)
        {
            const GfRotation rot(toQuatf(q));
            const GfVec3d angles = rot.Decompose(GfVec3d::XAxis(), GfVec3d::YAxis(), GfVec3d::ZAxis());
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(GfVec3f(float(angles[0]), float(angles[1]), float(angles[2])));
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfVec3d(angles[0], angles[1], angles[2]));
            orientSet = true;
        }
    }

    if (!translateSet || !orientSet)
    {
        primXform.ClearXformOpOrder();
        UsdGeomXformOp xform = primXform.MakeMatrixXform();
        primXform.SetResetXformStack(resetXformStack);
        if (xform)
            xform.Set(composeMatrix(p, q, s));
    }
}

// Author a local pose onto an XformCommonAPI-compatible prim WITHOUT rewriting
// its xform-op stack: decompose the orientation into Euler angles in the prim's
// authored rotation order and write via SetXformVectors (scale/pivot preserved
// from the prim).
bool authorViaXformCommonAPI(const UsdPrim& prim, const GfVec3d& translation, const GfQuatf& orientation)
{
    UsdGeomXformCommonAPI xformAPI(prim);
    if (!xformAPI)
    {
        CARB_LOG_WARN_ONCE("XformCommonAPI not available for prim %s during update.", prim.GetPrimPath().GetText());
        return false;
    }

    const GfQuatd inputQuat(orientation);
    GfMatrix3d rotMatrix = GfMatrix3d(inputQuat);

    UsdGeomXformCommonAPI::RotationOrder rotOrder;
    GfVec3f currentRotation, currentScale, currentPivot;
    GfVec3d currentTranslation;
    xformAPI.GetXformVectors(
        &currentTranslation, &currentRotation, &currentScale, &currentPivot, &rotOrder, UsdTimeCode::Default());

    // Transpose the matrix (USD stores matrices in transposed form)
    rotMatrix = rotMatrix.GetTranspose();

    // Extract Euler angles from rotation matrix (intrinsic rotations)
    GfVec3f eulerAngles;
    double r00 = rotMatrix[0][0], r01 = rotMatrix[0][1], r02 = rotMatrix[0][2];
    double r10 = rotMatrix[1][0], r11 = rotMatrix[1][1], r12 = rotMatrix[1][2];
    double r20 = rotMatrix[2][0], r21 = rotMatrix[2][1], r22 = rotMatrix[2][2];

    switch (rotOrder)
    {
    case UsdGeomXformCommonAPI::RotationOrderXYZ:
    {
        double y = asin(std::max(-1.0, std::min(1.0, -r20)));
        if (std::abs(r20) < 0.99999)
        {
            double x = atan2(r21, r22);
            double z = atan2(r10, r00);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        else
        {
            double z = atan2(-r01, r11);
            eulerAngles[0] = 0.0f;
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        break;
    }
    case UsdGeomXformCommonAPI::RotationOrderXZY:
    {
        double z = asin(std::max(-1.0, std::min(1.0, r10)));
        if (std::abs(r10) < 0.99999)
        {
            double x = atan2(-r12, r11);
            double y = atan2(-r20, r00);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        else
        {
            double y = atan2(r02, r22);
            eulerAngles[0] = 0.0f;
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        break;
    }
    case UsdGeomXformCommonAPI::RotationOrderYXZ:
    {
        double x = asin(std::max(-1.0, std::min(1.0, r21)));
        if (std::abs(r21) < 0.99999)
        {
            double y = atan2(-r20, r22);
            double z = atan2(-r01, r11);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        else
        {
            double z = atan2(r10, r00);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = 0.0f;
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        break;
    }
    case UsdGeomXformCommonAPI::RotationOrderYZX:
    {
        double z = asin(std::max(-1.0, std::min(1.0, -r01)));
        if (std::abs(r01) < 0.99999)
        {
            double y = atan2(r02, r00);
            double x = atan2(r21, r11);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        else
        {
            double x = atan2(-r12, r22);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = 0.0f;
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        break;
    }
    case UsdGeomXformCommonAPI::RotationOrderZXY:
    {
        double x = asin(std::max(-1.0, std::min(1.0, -r12)));
        if (std::abs(r12) < 0.99999)
        {
            double z = atan2(r10, r11);
            double y = atan2(r02, r22);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        else
        {
            double y = atan2(-r20, r00);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = 0.0f;
        }
        break;
    }
    case UsdGeomXformCommonAPI::RotationOrderZYX:
    {
        double y = asin(std::max(-1.0, std::min(1.0, r02)));
        if (std::abs(r02) < 0.99999)
        {
            double z = atan2(-r01, r00);
            double x = atan2(-r12, r22);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = static_cast<float>(z * 180.0 / M_PI);
        }
        else
        {
            double x = atan2(r21, r11);
            eulerAngles[0] = static_cast<float>(x * 180.0 / M_PI);
            eulerAngles[1] = static_cast<float>(y * 180.0 / M_PI);
            eulerAngles[2] = 0.0f;
        }
        break;
    }
    default:
        eulerAngles[0] = 0.0f;
        eulerAngles[1] = 0.0f;
        eulerAngles[2] = 0.0f;
        break;
    }

    // Scale/pivot are preserved from the prim (physics does not author scale).
    return xformAPI.SetXformVectors(
        translation, eulerAngles, currentScale, currentPivot, rotOrder, UsdTimeCode::Default());
}
} // namespace

UsdPhysicsDataWrite::UsdPhysicsDataWrite(UsdStageWeakPtr stage, const omni::physics::usd::UsdSource* source)
    : mStage(stage), mSource(source)
{
}

UsdPhysicsDataWrite::~UsdPhysicsDataWrite() = default;

UsdPrim UsdPhysicsDataWrite::primFor(omni::physics::parse::ObjectKey key) const
{
    if (!mSource || !mStage || !key.valid())
        return UsdPrim{};
    const SdfPath path = mSource->pathFor(key);
    if (path.IsEmpty())
        return UsdPrim{};
    return mStage->GetPrimAtPath(path);
}

UsdPrim UsdPhysicsDataWrite::usdPrimForWrite(omni::physics::parse::ObjectKey key) const
{
    return primFor(key);
}

TfToken UsdPhysicsDataWrite::tfTokenFor(omni::physics::parse::TokenId id) const
{
    return mSource ? mSource->tfTokenFor(id) : TfToken{};
}

void UsdPhysicsDataWrite::prepareTransformWrite(const omni::physics::parse::ObjectKey* keys,
                                                size_t count,
                                                bool* outEligible)
{
    if (!keys)
        return;
    for (size_t i = 0; i < count; ++i)
    {
        const bool eligible = prepareTransformWriteOne(keys[i]);
        if (outEligible)
            outEligible[i] = eligible;
    }
}

void UsdPhysicsDataWrite::releaseTransformWrite(const omni::physics::parse::ObjectKey* keys, size_t count)
{
    if (!keys)
        return;
    std::lock_guard<carb::tasking::MutexWrapper> lock(mTransformStateMutex);
    for (size_t i = 0; i < count; ++i)
        mTransformState.erase(keys[i]);
}

bool UsdPhysicsDataWrite::prepareTransformWriteOne(omni::physics::parse::ObjectKey key)
{
    UsdPrim prim = primFor(key);
    if (!prim)
        return false;

    TransformWriteState state;

    // When the XformCommonAPI write mode is enabled and the prim's stack is
    // compatible, leave the stack as-is and author via SetXformVectors later
    // (preserving the prim's rotation-order representation). Otherwise normalize
    // to scale/orient/translate.
    if (mUpdateUsingXformCommonAPI && UsdGeomXformCommonAPI(prim))
    {
        state.useXformCommonAPI = true;
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(mTransformStateMutex);
            mTransformState[key] = state;
        }
        return true;
    }

    // Normalize the xform-op stack to scale/orient/translate, capturing any
    // ops that couldn't be folded in as a pre/post residual transform. This is
    // the USD-specific setup InternalActor used to run inline.
    GfMatrix4d preMatrix(1.0), postMatrix(1.0);
    bool preValid = false, postValid = false;
    if (!omni::physics::usd::setupTransformOpsAsScaleOrientTranslate(prim, &preMatrix, &preValid, &postMatrix, &postValid))
        return false;

    if (preValid || postValid)
    {
        state.hasExtraTransform = true;
        state.extraTransformPreMultiply = preValid;
        // setupTransformOpsAsScaleOrientTranslate writes the residual into
        // preMatrix/postMatrix; the per-frame apply uses its inverse (matching
        // InternalActor's `mExtraTransfInv = mExtraTransfInv.GetInverse()`).
        state.extraTransformInverse = (preValid ? preMatrix : postMatrix).GetInverse();
    }
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mTransformStateMutex);
        mTransformState[key] = state;
    }
    return true;
}

void UsdPhysicsDataWrite::beginWrite()
{
    // Re-read parent frames each batch so a moving (e.g. nested/animated) parent
    // isn't served a stale world transform.
    mXformCache.Clear();
    if (!mChangeBlock)
        mChangeBlock = std::make_unique<SdfChangeBlock>();
}

void UsdPhysicsDataWrite::endWrite()
{
    mChangeBlock.reset();
}

void UsdPhysicsDataWrite::writeTransforms(const omni::physics::parse::ObjectKey* keys,
                                          size_t count,
                                          const omni::physics::parse::DataWriteView& positions,
                                          const omni::physics::parse::DataWriteView& orientations,
                                          const omni::physics::parse::DataWriteView& scales)
{
    if (!keys || !positions.data || !orientations.data)
        return;

    // The USD backend authors host-side; a device-resident column would need a
    // device->host copy that no backend produces yet. Reject (loudly, once) so a
    // future GPU path is a deliberate addition, not a silent misread.
    if (positions.device >= 0 || orientations.device >= 0 || (scales.data && scales.device >= 0))
    {
        CARB_LOG_WARN_ONCE("IPhysicsDataWrite (USD backend): device-resident transform columns "
                           "are not supported yet; skipping writeTransforms batch.");
        return;
    }

    // Columns are packed (or strided) buffers at host precision: positions/scales
    // are vec3 (PxVec3 layout), orientations are quaternion xyzw (PxQuat layout).
    const size_t posStride = positions.stride ? positions.stride : sizeof(::physx::PxVec3);
    const size_t orientStride = orientations.stride ? orientations.stride : sizeof(::physx::PxQuat);
    const size_t scaleStride = scales.stride ? scales.stride : sizeof(::physx::PxVec3);
    const auto* posBase = static_cast<const uint8_t*>(positions.data);
    const auto* orientBase = static_cast<const uint8_t*>(orientations.data);
    const auto* scaleBase = static_cast<const uint8_t*>(scales.data);
    const bool hasScales = scales.data != nullptr;
    auto posAt = [&](size_t i) { return *reinterpret_cast<const ::physx::PxVec3*>(posBase + i * posStride); };
    auto orientAt = [&](size_t i) { return *reinterpret_cast<const ::physx::PxQuat*>(orientBase + i * orientStride); };
    auto scaleAt = [&](size_t i) { return *reinterpret_cast<const ::physx::PxVec3*>(scaleBase + i * scaleStride); };

    // Resolve prims once, then author in order of increasing prim-path depth so
    // an ancestor body is authored before any descendant body reads its
    // (freshly authored) world transform for the world->local conversion. The
    // per-batch xform cache (cleared in beginWrite) then computes that parent
    // frame from the just-authored ops. For non-nested bodies the order is
    // irrelevant (their parent frame is static), so this is safe in general.
    std::vector<UsdPrim> prims(count);
    std::vector<size_t> order(count);
    for (size_t i = 0; i < count; ++i)
    {
        prims[i] = primFor(keys[i]);
        order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return prims[a].GetPath().GetPathElementCount() < prims[b].GetPath().GetPathElementCount();
    });

    for (size_t oi = 0; oi < count; ++oi)
    {
        const size_t i = order[oi];
        UsdPrim& prim = prims[i];
        if (!prim)
            continue;

        ::physx::PxVec3 scaleVal;
        const ::physx::PxVec3* scale = nullptr;
        if (hasScales)
        {
            scaleVal = scaleAt(i);
            scale = &scaleVal;
        }

        const TransformWriteState* st = nullptr;
        if (auto stateIt = mTransformState.find(keys[i]); stateIt != mTransformState.end())
            st = &stateIt->second;

        // Resolve the object's nearest xformable ancestor each frame (handles
        // moving/reparented parents without any dirty-tracking on the engine side).
        UsdPrim parentXform;
        const bool hasParent = omni::physics::usd::getParentXform(prim, parentXform);
        const bool hasExtra = st && st->hasExtraTransform;
        const bool useCommonAPI = st && st->useXformCommonAPI;

        // Common case: object is root-equivalent (world == local) with no
        // residual offset and standard ops -> author the world pose directly,
        // no matrix round-trip (matches the old root-body fast path).
        if (!hasParent && !hasExtra && !useCommonAPI)
        {
            scatterTransform(prim, posAt(i), orientAt(i), scale);
            continue;
        }

        // Otherwise build a matrix, convert WORLD -> object-local using the
        // parent frame, fold in the residual extra-transform, and author
        // the decomposed local pose.
        GfMatrix4d mat = composeMatrix(posAt(i), orientAt(i), scale);
        if (hasParent)
            mat = mat * mXformCache.GetLocalToWorldTransform(parentXform).GetInverse();
        if (hasExtra)
            mat = st->extraTransformPreMultiply ? (mat * st->extraTransformInverse) :
                                                  (st->extraTransformInverse * mat);

        const GfTransform tr(mat);
        const GfVec3f p(tr.GetTranslation());
        const GfQuatf q(tr.GetRotation().GetQuat());

        if (useCommonAPI)
        {
            // Author onto the prim's XformCommonAPI stack (preserving its
            // rotation order) instead of scale/orient/translate ops.
            authorViaXformCommonAPI(prim, GfVec3d(p), q);
            continue;
        }

        const ::physx::PxVec3 cp{ p[0], p[1], p[2] };
        const GfVec3f qi = q.GetImaginary();
        const ::physx::PxQuat cq{ qi[0], qi[1], qi[2], q.GetReal() };
        scatterTransform(prim, cp, cq, scale);
    }
}

void UsdPhysicsDataWrite::writeData(const omni::physics::parse::ObjectKey* keys,
                                    size_t count,
                                    omni::physics::parse::TokenId attr,
                                    const omni::physics::parse::DataWriteView& data)
{
    using omni::physics::parse::DataType;

    if (!keys || !data.data)
        return;
    if (data.device >= 0)
    {
        CARB_LOG_WARN_ONCE("IPhysicsDataWrite (USD backend): device-resident writeData columns "
                           "are not supported yet; skipping write.");
        return;
    }

    const TfToken attrToken = tfTokenFor(attr);
    const auto* base = static_cast<const uint8_t*>(data.data);
    const bool half = data.type == DataType::e16Bit;
    const size_t quatStride = data.stride ? data.stride : (half ? 4 * sizeof(GfHalf) : 4 * sizeof(float));

    // Scatter: element i of the column to attribute `attr` on keys[i]. Element
    // shape comes from the destination attribute, precision from the view. Non-
    // quat elements (xyz / scalar) share the Px and Gf layout and are copied
    // through the matching Gf pointer; quaternions arrive in PxQuat order (xyzw)
    // and are reordered to Gf real-first here (uniform with writeTransforms).
    size_t written = 0;
    bool sawObject = false;
    for (size_t i = 0; i < count; ++i)
    {
        UsdPrim prim = primFor(keys[i]);
        if (!prim)
            continue;
        sawObject = true;
        UsdAttribute usdAttr = prim.GetAttribute(attrToken);
        if (!usdAttr)
            continue;

        auto setElem = [&](auto sample)
        {
            using ElemT = decltype(sample);
            const size_t stride = data.stride ? data.stride : sizeof(ElemT);
            usdAttr.Set(*reinterpret_cast<const ElemT*>(base + i * stride));
        };

        const SdfValueTypeName tn = usdAttr.GetTypeName();
        if (tn == SdfValueTypeNames->Quath || tn == SdfValueTypeNames->Quatf || tn == SdfValueTypeNames->Quatd)
        {
            const GfQuatf q = readQuatXYZW(base, i, quatStride, half);
            if (tn == SdfValueTypeNames->Quath)
                usdAttr.Set(GfQuath(q));
            else if (tn == SdfValueTypeNames->Quatd)
                usdAttr.Set(GfQuatd(q));
            else
                usdAttr.Set(q);
        }
        else if (tn == SdfValueTypeNames->Float || tn == SdfValueTypeNames->Half || tn == SdfValueTypeNames->Double)
        {
            setElem(float());
        }
        else // vec3 family (vector3f/point3f/normal3f/color3f/float3 ...)
        {
            if (half)
                setElem(GfVec3h());
            else
                setElem(GfVec3f());
        }
        ++written;
    }

    // If at least one target object existed but none carried the attribute, the
    // token was almost certainly mis-interned (wrong name / wrong source). Surface
    // it once rather than letting the whole batch vanish silently.
    if (sawObject && written == 0 && count > 0)
    {
        CARB_LOG_WARN_ONCE("IPhysicsDataWrite (USD backend): writeData wrote nothing for attribute '%s' "
                           "across %zu objects — none had the attribute (mis-interned token?).",
                           attrToken.GetText(), count);
    }
}

void UsdPhysicsDataWrite::writeArray(omni::physics::parse::ObjectKey key,
                                     omni::physics::parse::TokenId attr,
                                     const omni::physics::parse::DataWriteView& data)
{
    using omni::physics::parse::DataType;

    // The USD backend authors host-side; a device-resident column cannot be read
    // here. Guard before any host dereference of data.data (matches writeData/writeTransforms).
    if (data.device >= 0)
    {
        CARB_LOG_WARN_ONCE("IPhysicsDataWrite (USD backend): device-resident writeArray columns "
                           "are not supported; skipping write.");
        return;
    }

    UsdPrim prim = primFor(key);
    if (!prim)
        return;
    UsdAttribute usdAttr = prim.GetAttribute(tfTokenFor(attr));
    if (!usdAttr)
        return;
    // A null payload is only valid for an empty (count == 0) write — the
    // "author an empty array" reset the cooking write-back uses to clear
    // velocities. A null payload with a non-zero count is a caller bug; skip it.
    if (!data.data && data.count != 0)
        return;

    const auto* base = static_cast<const uint8_t*>(data.data);
    const size_t n = data.count;

    // Non-quat elements are laid out identically in Px and Gf (xyz / scalar), so
    // copy element-by-element through the matching Gf pointer at the view's
    // precision (no reorder). Quaternions are the exception: the source is in
    // PxQuat order (xyzw) and is reordered to Gf real-first here (see below).
    auto fill = [&](auto& vtArray, auto sample)
    {
        using ElemT = decltype(sample);
        const size_t stride = data.stride ? data.stride : sizeof(ElemT);
        for (size_t i = 0; i < n; ++i)
            vtArray[i] = *reinterpret_cast<const ElemT*>(base + i * stride);
    };

    // Element shape comes from the destination attribute; precision from the view.
    const SdfValueTypeName tn = usdAttr.GetTypeName();
    const bool half = data.type == DataType::e16Bit;

    if (tn == SdfValueTypeNames->QuathArray || tn == SdfValueTypeNames->QuatfArray ||
        tn == SdfValueTypeNames->QuatdArray)
    {
        // Source quats are xyzw (PxQuat order); reorder to Gf real-first and emit
        // at the destination attribute's precision.
        const size_t stride = data.stride ? data.stride : (half ? 4 * sizeof(GfHalf) : 4 * sizeof(float));
        if (tn == SdfValueTypeNames->QuathArray)
        {
            VtQuathArray arr(n);
            for (size_t i = 0; i < n; ++i)
                arr[i] = GfQuath(readQuatXYZW(base, i, stride, half));
            usdAttr.Set(arr);
        }
        else if (tn == SdfValueTypeNames->QuatdArray)
        {
            VtQuatdArray arr(n);
            for (size_t i = 0; i < n; ++i)
                arr[i] = GfQuatd(readQuatXYZW(base, i, stride, half));
            usdAttr.Set(arr);
        }
        else
        {
            VtQuatfArray arr(n);
            for (size_t i = 0; i < n; ++i)
                arr[i] = readQuatXYZW(base, i, stride, half);
            usdAttr.Set(arr);
        }
    }
    else if (tn == SdfValueTypeNames->FloatArray || tn == SdfValueTypeNames->HalfArray ||
             tn == SdfValueTypeNames->DoubleArray)
    {
        VtFloatArray arr(n);
        fill(arr, float());
        usdAttr.Set(arr);
    }
    else if (tn == SdfValueTypeNames->Int4Array)
    {
        // Integer connectivity (e.g. UsdGeomTetMesh tetVertexIndices). Source
        // elements are 4 contiguous int32 (GfVec4i layout), copied verbatim.
        VtVec4iArray arr(n);
        fill(arr, GfVec4i());
        usdAttr.Set(arr);
    }
    else if (tn == SdfValueTypeNames->Int3Array)
    {
        // Integer connectivity (e.g. UsdGeomTetMesh surfaceFaceVertexIndices).
        VtVec3iArray arr(n);
        fill(arr, GfVec3i());
        usdAttr.Set(arr);
    }
    else if (tn == SdfValueTypeNames->IntArray)
    {
        // Flat integer arrays (e.g. UsdGeomMesh faceVertexCounts /
        // faceVertexIndices written by the surface-deformable cook).
        VtIntArray arr(n);
        fill(arr, int());
        usdAttr.Set(arr);
    }
    else // vec3 family (point3f/vector3f/normal3f/color3f/float3 ...)
    {
        if (half)
        {
            VtVec3hArray arr(n);
            fill(arr, GfVec3h());
            usdAttr.Set(arr);
        }
        else
        {
            VtVec3fArray arr(n);
            fill(arr, GfVec3f());
            usdAttr.Set(arr);
        }
    }
}

void UsdPhysicsDataWrite::writeUIntAttribute(omni::physics::parse::ObjectKey key,
                                             const TfToken& attr,
                                             uint32_t value)
{
    UsdPrim prim = primFor(key);
    if (!prim)
        return;
    // CreateAttribute is idempotent (returns the existing attribute when present),
    // so this both creates-with-type and sets, matching the cooking marker write.
    prim.CreateAttribute(attr, SdfValueTypeNames->UInt).Set(value);
}

void UsdPhysicsDataWrite::writeUCharArrayAttribute(omni::physics::parse::ObjectKey key,
                                                   const TfToken& attr,
                                                   const uint8_t* data,
                                                   size_t count)
{
    UsdPrim prim = primFor(key);
    if (!prim)
        return;
    // Mirrors the legacy storeMeshKey byte layout (sizeof(MeshKey) raw bytes as a
    // UCharArray, created if absent) so loadMeshKey round-trips unchanged.
    UsdAttribute usdAttr = prim.GetAttribute(attr);
    if (!usdAttr.HasAuthoredValue())
        usdAttr = prim.CreateAttribute(attr, SdfValueTypeNames->UCharArray);
    VtArray<uchar> vtData(count);
    if (data && count)
        std::memcpy(vtData.data(), data, count);
    usdAttr.Set(vtData);
}

void UsdPhysicsDataWrite::removeAttribute(omni::physics::parse::ObjectKey key, const TfToken& attr)
{
    UsdPrim prim = primFor(key);
    if (!prim)
        return;
    if (prim.HasAttribute(attr))
        prim.RemoveProperty(attr);
}

} // namespace omni::physics::usd
