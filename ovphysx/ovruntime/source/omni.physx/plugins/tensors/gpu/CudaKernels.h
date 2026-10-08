// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-6
 *
 * @implements REQ-READ-INVDYN-001
 * @covers AC-10
 */

#pragma once

#include "tensors/PhysicsTypes.h"
#include "tensors/gpu/CudaCommon.h"
#include "tensors/gpu/GpuSimulationData.h"

// Included rather than forward-declared: the gather takes a DofScalePolicy by value. Both headers are
// dependency-free (cstdint plus a host/device macro) so the CUDA side can include them.
#include "tensors/ArticulationDofOvStageRecord.h"
#include "tensors/ArticulationLinkOvStageRecord.h"

namespace omni
{
namespace physx
{
namespace tensors
{
struct GpuArticulationRootRecord;
struct GpuArticulationDofRecord;
struct ArticulationTendonOvStageRecord;
struct GpuArticulationLinkRecord;
struct GpuArticulationFixedTendonRecord;
struct GpuArticulationSpatialTendonRecord;
struct GpuRigidContactFilterIdPair;
struct GpuRigidBodyRecord;
struct GpuPointInstancerRecord;
struct GpuPointSetReadRecord;
struct PointSetTransform;
struct SingleAllocPolicy;

//
// articulations
//
::physx::PxU32 fillArtiDirtyIndices(SingleAllocPolicy& policy,
                                    ::physx::PxU32* indicesRet,
                                    const ::physx::PxU32* allIndicies,
                                    const ArticulationGpuFlags* artiDirtyFlags,
                                    const ArticulationGpuFlag::Enum flag,
                                    const ::physx::PxU32 numArtis);

::physx::PxU32 fillArtiTransforms(SingleAllocPolicy& policy,
                                  ::physx::PxTransform* transformDev,
                                  ::physx::PxU32* indicesRet,
                                  const ::physx::PxU32* allIndicies,
                                  const ArticulationGpuFlags* artiDirtyFlags,
                                  const ArticulationGpuFlag::Enum flag,
                                  const ::physx::PxU32 numArtis);

::physx::PxU32 fillArtiVelocities(SingleAllocPolicy& policy,
                                  ::physx::PxVec3* artLinVelDev,
                                  ::physx::PxVec3* artAngVelDev,
                                  ::physx::PxU32* indicesRet,
                                  const ::physx::PxU32* allIndicies,
                                  const ArticulationGpuFlags* artiDirtyFlags,
                                  const ArticulationGpuFlag::Enum flag,
                                  const ::physx::PxU32 numArtis);

::physx::PxU32 fillArtiFT(SingleAllocPolicy& policy,
                          ::physx::PxVec3* FT,
                          ::physx::PxU32* indicesRet,
                          const ::physx::PxU32* allIndicies,
                          const ArticulationGpuFlags* artiDirtyFlags,
                          ArticulationGpuFlags* artiLinksDirtyFlags,
                          const ArticulationGpuFlag::Enum flag,
                          const ::physx::PxU32 numArtis,
                          const ::physx::PxU32 maxLinks);

bool fetchArtiRootTransforms(TensorTransform* dst,
                             const ::physx::PxTransform* src,
                             const ::physx::PxU32 numArti,
                             const GpuArticulationRootRecord* rootRecords);

bool submitArtiRootTransforms(::physx::PxTransform* dst,
                              const TensorTransform* src,
                              const ::physx::PxU32* srcIndices,
                              ::physx::PxU32* dirtyArtiGpuIndices,
                              const ::physx::PxU32 numIndices,
                              const ::physx::PxU32 numArti,
                              const GpuArticulationRootRecord* rootRecords);

bool fetchArtiRootVelocities(TensorVelAcc* dst,
                             const ::physx::PxVec3* srcLin,
                             const ::physx::PxVec3* srcAng,
                             const ::physx::PxU32 numArti);

bool fetchArtiRootPoseColumnOvStage(float* dst, const ::physx::PxTransform* src, ::physx::PxU32 count, bool orientation);

bool fetchArtiRootVelocityColumnOvStage(float* dst, const ::physx::PxVec3* src, ::physx::PxU32 count);

// In-place subspace-origin subtraction over a whole world-COM column. Distinct from the
// point-instancer reframe (InstancerReframe.h), which is a full affine composition with rotation.
bool applySubspaceOriginArtiMassCentersOvStage(::physx::PxVec3* dst,
                                               ::physx::PxU32 count,
                                               const GpuArticulationRootRecord* rootRecords);

bool submitArtiRootVelocities(::physx::PxVec3* dstLin,
                              ::physx::PxVec3* dstAng,
                              const TensorVelAcc* src,
                              const ::physx::PxU32* srcIndices,
                              ::physx::PxU32* dirtyArtiGpuIndices,
                              const ::physx::PxU32 numIndices,
                              const ::physx::PxU32 numArti,
                              const GpuArticulationRootRecord* rootRecords);

bool fetchArtiLinkTransforms(TensorTransform* dst,
                             const ::physx::PxTransform* src,
                             const ::physx::PxU32 numLinks,
                             const ::physx::PxU32 maxLinks,
                             const ::physx::PxU32 simMaxLinks,
                             const GpuArticulationLinkRecord* linkRecords);

// fetches link velocities or accelerations from src
bool fetchArtiLinkVelocitiesAccelerations(TensorVelAcc* dst,
                                          const ::physx::PxVec3* linSrc,
                                          const ::physx::PxVec3* angSrc,
                                          const ::physx::PxU32 numLinks,
                                          const ::physx::PxU32 maxLinks,
                                          const ::physx::PxU32 simMaxLinks);

bool fetchArtiMassMatrices(float* dst,
                           const float* src,
                           const ::physx::PxU32 numElements,
                           const ::physx::PxU32 massMatrixSize,
                           const ::physx::PxU32 simMassMatrixSize,
                           const ::physx::PxU32 generalizedCoords,
                           const ::physx::PxU32 rootDofs,
                           const ::physx::PxU32 dofRecordBase,
                           const GpuArticulationDofRecord* dofRecords,
                           bool applyBodyOrderSign);

// Cohort gather for the coriolis / gravity columns. `dofRecordBase` is the record offset of the
// cohort's FIRST row: all its rows share a metatype, so they share the per-dof sign.
bool fetchArtiGeneralizedForceColumnOvStage(float* dst,
                                            const float* src,
                                            const ::physx::PxU32 count,
                                            const ::physx::PxU32 width,
                                            const ::physx::PxU32 rootDofs,
                                            const ::physx::PxU32 simMaxDofs,
                                            const ::physx::PxU32 dofRecordBase,
                                            const GpuArticulationDofRecord* dofRecords);

bool fetchArtiDofAttributeGravityAndCoriolis(float* dst,
                                             const float* src,
                                             const ::physx::PxU32 numDofs,
                                             const ::physx::PxU32 maxDofs,
                                             const ::physx::PxU32 simMaxDofs,
                                             const GpuArticulationDofRecord* dofRecords,
                                             const bool hasRootDofs);

bool fetchArtiCentroidalMomentumMatrices(float* dst,
                                         const float* src,
                                         const ::physx::PxU32 numElem,
                                         const ::physx::PxU32 maxDofs,
                                         const ::physx::PxU32 cenMomBlockSize,
                                         const ::physx::PxU32 simCenMomBlockSize,
                                         const ::physx::PxU32 startSimBiasForceBlock,
                                         const ::physx::PxU32 dofRecordBase,
                                         const GpuArticulationDofRecord* dofRecords,
                                         bool applyBodyOrderSign);

bool fetchArtiJacobian(float* dst,
                       const float* src,
                       const ::physx::PxU32 numElements,
                       const ::physx::PxU32 jacobianSize,
                       const ::physx::PxU32 simJacobianSize,
                       const ::physx::PxU32 jacobianCols,
                       const ::physx::PxU32 rootDofs,
                       const ::physx::PxU32 dofRecordBase,
                       const GpuArticulationDofRecord* dofRecords,
                       bool applyBodyOrderSign);

bool fetchArtiDofAttribute(float* dst,
                           const float* src,
                           const ::physx::PxU32 numDofs,
                           const ::physx::PxU32 maxDofs,
                           const ::physx::PxU32 simMaxDofs,
                           const GpuArticulationDofRecord* dofRecords);

// ovstage joint-state gather:
//   dst[i] = dofScaleFor(recs[i], policy) * src[recs[i].viewArtiIdx * simMaxDofs + recs[i].physxDofIdx]
//
// `policy` selects which of the record's axis facts the attribute folds. An argument rather than a
// record field because one read shares the record list across every attribute.
bool fetchArtiDofAttributeOvStage(float* dst,
                                  const float* src,
                                  const ::physx::PxU32 numOutputs,
                                  const ::physx::PxU32 simMaxDofs,
                                  const DofScalePolicy policy,
                                  const ArticulationDofOvStageRecord* records);

// ovstage per-LINK gather for a fixed-width vector attribute:
//   dst[i*comp + c] = src[(recs[i].viewArtiIdx * maxLinks + recs[i].physxLinkIdx) * comp + c]
//
// `src` is a DENSE [count x maxLinks x comp] row from one of the link-parallel fetches, so `maxLinks`
// is the view's stride, not the scene's. No policy: the frame is resolved in the dense pass.
bool fetchArtiLinkVectorOvStage(float* dst,
                                const float* src,
                                const ::physx::PxU32 numOutputs,
                                const ::physx::PxU32 maxLinks,
                                const ::physx::PxU32 comp,
                                const ArticulationLinkOvStageRecord* records);

// ovstage rigid/link pose column, fused: PhysX-layout scratch -> the packed destination column in
// ONE pass. Resolves rigid-dynamic vs articulation-link source (de-padding the [numArti x
// simMaxLinks] link block), subtracts the subspace origin, and writes only the requested component
// -- position (3 floats) or orientation (4) -- so no [N,7] intermediate is materialized.
//
// `outRecordIdx` gives output slot i its record: null means identity (slot i -> record i), a list
// means an arbitrary subset/reorder of a larger view. `numOutputs` is the destination row count.
bool fetchRbPoseColumnOvStage(float* dst,
                              const ::physx::PxTransform* actorData,
                              const ::physx::PxTransform* linkTransforms,
                              const GpuRigidBodyRecord* records,
                              const ::physx::PxU32* outRecordIdx,
                              const ::physx::PxU32 numOutputs,
                              const ::physx::PxU32 simMaxLinks,
                              const bool wantOrientation);

// ovstage rigid/link velocity column, fused. Same record/indirection contract; the caller passes
// the linear or angular source pair, so the component selector is the pointers themselves.
bool fetchRbVelocityColumnOvStage(float* dst,
                                  const ::physx::PxVec3* rdData,
                                  const ::physx::PxVec3* linkData,
                                  const GpuRigidBodyRecord* records,
                                  const ::physx::PxU32* outRecordIdx,
                                  const ::physx::PxU32 numOutputs,
                                  const ::physx::PxU32 simMaxLinks);

// ovstage joint-DOF write (ADR-0012): the exact inverse of fetchArtiDofAttributeOvStage.
//
// THE SCALE IS DIVIDED, NOT MULTIPLIED, and that is the whole risk in this function. The record's
// `scale` folds the unit convention (rad->deg on angular axes) and the sign into one multiply on the
// read; the write has to undo both. Multiplying here instead would reproduce the 57.295x
// joint-velocity error the unit-conversion contract exists to prevent (ADR-0001 section 7.5): descriptors
// carry engine-native units, and the boundary converts -- once, in each direction.
//
// Writes into the scene's DOF scratch in PhysX layout, which the caller has already filled with the
// articulations' current values. That read-modify-write is not optional: PhysX writes whole
// articulation DOF blocks, so a DOF this call does not touch has to already hold its current value
// or it would be zeroed.
bool submitArtiDofAttributeOvStage(float* dofScalars,
                                   const float* src,
                                   const ::physx::PxU32 numOutputs,
                                   const ::physx::PxU32 simMaxDofs,
                                   const ArticulationDofOvStageRecord* records,
                                   // Which folds the scatter undoes. Not derivable from the write
                                   // flag: the four state/target columns are eAngularSigned and the
                                   // actuation force is eSigned, yet all five are JOINT_* writes.
                                   DofScalePolicy policy);

// ovstage rigid-body write, PACKED form (ADR-0012). The pair below replaces a flags-and-compaction
// scatter, and the reason is architectural rather than performance: fillRdTransforms computes its
// element count on the HOST, from a thrust::copy_if iterator difference, so any path built on it
// must block. The read carries no host block at all (ADR-0008 W2 / Decision 7), and a write that
// blocks cannot join that contract. Building the packed pair directly makes the count numOutputs --
// known on the host before the kernel launches -- so nothing has to be read back.
//
// It also takes the write OUT of the shared scene buffers: nothing here touches mRdPoseDev or the
// velocity scratch, so a read gather and a write scatter over one scene no longer alias, and the
// SharedDeviceBuffer ordering that would otherwise be needed between them does not arise.
//
// PhysX pairs the two arrays by position: the data for the body whose GPU index sits at slot x must
// sit at slot x. Both are therefore built from the same row list in one pass.

// Packed GPU-index list for the rows this write covers: dstIdx[i] is the DirectGPU index of the
// body at output slot i.
//
// A record with no GPU row (tensorRdIdx == sentinel: a disabled body, or an articulation link) has
// no index to write and would poison the whole call, so the CALLER must ensure the row list carries
// none. That is guaranteed by construction -- the ovstage rigid enumeration yields only rigid
// dynamics, and refreshRdGpuIndices fails the op if any lacks a GPU row -- and is checked on the
// host where the list is built, because a kernel cannot report it without a synchronize.
bool submitRbPackedIndicesOvStage(::physx::PxRigidDynamicGPUIndex* dstIdx,
                                 const ::physx::PxRigidDynamicGPUIndex* rdGpuIndices,
                                 const GpuRigidBodyRecord* records,
                                 const ::physx::PxU32* outRecordIdx,
                                 const ::physx::PxU32 numOutputs);

// Overlay one pose component onto a packed transform block the caller has already filled with the
// bodies' current poses. The half not being written survives, which is the whole point: PhysX has a
// single eGLOBAL_POSE write and a session carries one attribute.
//
// The subspace origin is ADDED here where fetchRbPoseColumnOvStage subtracts it, so a column read,
// edited and written back round-trips to the value it started from.
bool submitRbPackedPoseOvStage(::physx::PxTransform* packedPose,
                                const float* src,
                                const GpuRigidBodyRecord* records,
                                const ::physx::PxU32* outRecordIdx,
                                const ::physx::PxU32 numOutputs,
                                const bool wantOrientation);

// ovstage LINK force/wrench write (ADR-0012). The articulation counterpart of
// submitRbWrenchOvStage: PhysX addresses a link's force as (articulation, link) rather than as a
// rigid-dynamic row, so the destination slot is `tensorArtiIdx * simMaxLinks + linkIdx` and the data
// block covers the WHOLE view.
//
// `comps` is 3 for a plain force and 9 for a wrench; the torque-about-COM conversion runs only in the
// 9 case, using the link's own pose and centre of mass.
bool submitLinkWrenchOvStage(::physx::PxVec3* linkForces,
                              ::physx::PxVec3* linkTorques,
                              const float* src,
                              const ::physx::PxTransform* linkTransforms,
                              const ::physx::PxVec3* comsByRecord,
                              const GpuRigidBodyRecord* records,
                              const ::physx::PxU32* rows,
                              ::physx::PxU32 numOutputs,
                              ::physx::PxU32 simMaxLinks,
                              ::physx::PxU32 comps);

// ovstage WRENCH write (ADR-0012). Turns one [N,9] column -- force(3), torque(3), world
// application point(3) -- into the two dense PxVec3 arrays PhysX's eFORCE / eTORQUE write types take.
//
// The conversion is the whole reason `wrench` is not just `force`: a load applied AWAY from the
// centre of mass also produces a torque about it. Same expression the tensor binding's applyForces
// helper uses, so the two APIs agree:
//
//     comWorld = bodyPose.transform(comLocal)
//     torque   = srcTorque + (point - comWorld) x force
//
// `poses` and `coms` are indexed by OUTPUT slot i (poses were read packed through the same index
// list), while `coms` is indexed by the body's record row -- hence `rows`.
bool submitRbWrenchOvStage(::physx::PxVec3* outForces,
                            ::physx::PxVec3* outTorques,
                            const float* src,
                            const ::physx::PxTransform* poses,
                            const ::physx::PxVec3* comsByRecord,
                            const ::physx::PxU32* rows,
                            ::physx::PxU32 numOutputs);

// ovstage articulation-ROOT write (ADR-0012). Overlay one component onto the view-wide root block
// the caller has already filled with the articulations' current root state, indexing it through the
// ovstage row list: output i addresses view articulation `rows[i]`.
//
// Read-modify-write is not optional here for the same reason it is not for DOFs, but for a second
// reason on top: PhysX has ONE eROOT_GLOBAL_POSE write covering position and orientation together,
// and a session carries one attribute -- so the half not being written has to already be in the
// block. And the write covers the WHOLE view (a subset write would need a compacted index list whose
// length comes off the host), so every articulation the query did not match has to hold its current
// value too.
//
// The subspace origin is ADDED here where fetchArtiRootTransforms subtracts it, so a root pose read,
// edited and written back round-trips to the value it started from.
bool submitArtiRootPoseOvStage(::physx::PxTransform* rootBlock,
                                const float* src,
                                const ::physx::PxU32* rows,
                                const GpuArticulationRootRecord* rootRecords,
                                ::physx::PxU32 numOutputs,
                                bool wantOrientation);

// The velocity counterpart. No origin: a velocity is frame-invariant under a translation, which is
// why the root velocity read applies none either. Linear and angular are separate PhysX write types,
// so each has its own block and neither has to preserve the other.
bool submitArtiRootVelocityOvStage(::physx::PxVec3* velBlock,
                                    const float* src,
                                    const ::physx::PxU32* rows,
                                    ::physx::PxU32 numOutputs);

// ovstage tendon-property WRITE (ADR-0012): the inverse of fetchArtiTendonPropertyOvStage.
//
// NO SCALE, and that is a decision rather than an omission. A fixed tendon's length is a weighted
// sum of joint positions, so its rest length, limits and offset are in generalized-coordinate units
// and its stiffness is force per those units -- the unit question does arise here. It is already
// answered upstream: updateTendonAxisSingleGearing folds rad2deg into the gearing coefficient, and
// everything after that is passthrough. The READ applies no scale for that reason, so the write
// applies none either; converting here would break the round trip.
//
// Writes into the scene's tendon block in PhysX layout, which the caller has already filled with the
// articulations' current tendon structs. That read-modify-write is not optional: PhysX writes whole
// tendon structs, so a property this call does not address must already hold its current value.
bool submitArtiTendonPropertyOvStage(float* tendonStructs,
                                     const float* src,
                                     ::physx::PxU32 numOutputs,
                                     ::physx::PxU32 simMaxTendons,
                                     ::physx::PxU32 structFloats,
                                     ::physx::PxU32 fieldOffset,
                                     ::physx::PxU32 comp,
                                     const ArticulationTendonOvStageRecord* records);


bool submitArtiDofAttribute(float* dst,
                            const float* src,
                            const ::physx::PxU32* srcArtiIndices,
                            ::physx::PxU32* dirtyArtiGpuIndices,
                            const ::physx::PxU32 numDofs,
                            const ::physx::PxU32 maxDofs,
                            const ::physx::PxU32 simMaxDofs,
                            const ::physx::PxU32 maxArtis,
                            const GpuArticulationDofRecord* dofRecords);

// ovstage tendon gather: one kernel for every tendon attribute, on both tendon kinds.
//
// PxGpuSpatialTendonData and PxGpuFixedTendonData are plain float structs, so an attribute is just
// a float offset and a component count into a fixed stride -- there is nothing per-attribute left
// for a kernel of its own to do. `structFloats` is 4 for spatial tendons and 8 for fixed ones.
//
//   dst[i*comp + c] = src[(recs[i].viewArtiIdx * simMaxTendons + recs[i].tendonIdx) * structFloats
//                         + fieldOffset + c]
bool fetchArtiTendonPropertyOvStage(float* dst,
                                    const float* src,
                                    const ::physx::PxU32 numOutputs,
                                    const ::physx::PxU32 simMaxTendons,
                                    const ::physx::PxU32 structFloats,
                                    const ::physx::PxU32 fieldOffset,
                                    const ::physx::PxU32 comp,
                                    const ArticulationTendonOvStageRecord* records);

// ovstage point-instancer columns: instancer-local pose (reframed on the device) and world-frame
// velocity, both scattered into per-instancer destination sub-ranges named by `offsets`.
bool fetchInstancerPoseColumnOvStage(float* dst,
                                     const ::physx::PxTransform* actorData,
                                     const GpuRigidBodyRecord* rbRecords,
                                     const GpuPointInstancerRecord* records,
                                     const InstancerAffine* instancerInverses,
                                     const ::physx::PxU32* offsets,
                                     ::physx::PxU32 numInstances,
                                     bool wantOrientation);

bool fetchInstancerVelocityColumnOvStage(float* dst,
                                         const ::physx::PxVec3* rdData,
                                         const GpuRigidBodyRecord* rbRecords,
                                         const GpuPointInstancerRecord* records,
                                         const ::physx::PxU32* offsets,
                                         ::physx::PxU32 numInstances);

// The ovstage point-instancer WRITE (ADR-0012). One instancer per call, because the write
// hands each instancer's array to the caller as its own group and commits them independently.
//
// A pose write is a READ-MODIFY-WRITE here, exactly as it is on the host path and for the same
// reason: a session carries one attribute, but a pose needs both halves, so the half the column does
// not supply is recovered from the body's current world pose (reframed to local), substituted, and
// composed back out. That is why this takes the forward AND inverse transforms.
//
// `records` points at THIS instancer's range and `numOutputs` is its length, so output slot i is
// simply thread i -- the view groups its records by instancer, which removes both a per-instance
// filter and a per-commit slot-map upload. Holes never appear in a range: an instance with no live
// body is dropped when the records are built.
bool submitInstancerPoseColumnOvStage(::physx::PxTransform* packedPose,
                                      const float* src,
                                      const ::physx::PxTransform* actorData,
                                      const GpuRigidBodyRecord* rbRecords,
                                      const GpuPointInstancerRecord* records,
                                      const InstancerAffine* instancerInverses,
                                      const InstancerAffine* instancerForwards,
                                      ::physx::PxU32 numOutputs,
                                      ::physx::PxU32 instancerIdx,
                                      bool wantOrientation);

// The velocity counterpart. No reframe in either direction -- a velocity is published in world space,
// so the instancer transform does not apply to it.
bool submitInstancerVelocityColumnOvStage(::physx::PxVec3* packedVel,
                                          const float* src,
                                          const GpuPointInstancerRecord* records,
                                          ::physx::PxU32 numOutputs);

bool fetchFixedTendonStiffness(float* dst,
                               const ::physx::PxGpuFixedTendonData* tendonProperties,
                               const ::physx::PxU32 numTendons,
                               const ::physx::PxU32 maxTendons,
                               const ::physx::PxU32 simMaxTendons);

bool fetchFixedTendonDamping(float* dst,
                             const ::physx::PxGpuFixedTendonData* tendonProperties,
                             const ::physx::PxU32 numTendons,
                             const ::physx::PxU32 maxTendons,
                             const ::physx::PxU32 simMaxTendons);

bool fetchFixedTendonLimitStiffness(float* dst,
                                    const ::physx::PxGpuFixedTendonData* tendonProperties,
                                    const ::physx::PxU32 numTendons,
                                    const ::physx::PxU32 maxTendons,
                                    const ::physx::PxU32 simMaxTendons);

bool fetchFixedTendonLimits(float* dst,
                            const ::physx::PxGpuFixedTendonData* tendonProperties,
                            const ::physx::PxU32 numTendons,
                            const ::physx::PxU32 maxTendons,
                            const ::physx::PxU32 simMaxTendons);

bool fetchFixedTendonRestLength(float* dst,
                                const ::physx::PxGpuFixedTendonData* tendonProperties,
                                const ::physx::PxU32 numTendons,
                                const ::physx::PxU32 maxTendons,
                                const ::physx::PxU32 simMaxTendons);

bool fetchFixedTendonOffset(float* dst,
                            const ::physx::PxGpuFixedTendonData* tendonProperties,
                            const ::physx::PxU32 numTendons,
                            const ::physx::PxU32 maxTendons,
                            const ::physx::PxU32 simMaxTendons);

bool fetchSpatialTendonStiffness(float* dst,
                                 const ::physx::PxGpuSpatialTendonData* tendonProperties,
                                 const ::physx::PxU32 numTendons,
                                 const ::physx::PxU32 maxTendons,
                                 const ::physx::PxU32 simMaxTendons);

bool fetchSpatialTendonDamping(float* dst,
                               const ::physx::PxGpuSpatialTendonData* tendonProperties,
                               const ::physx::PxU32 numTendons,
                               const ::physx::PxU32 maxTendons,
                               const ::physx::PxU32 simMaxTendons);

bool fetchSpatialTendonLimitStiffness(float* dst,
                                      const ::physx::PxGpuSpatialTendonData* tendonProperties,
                                      const ::physx::PxU32 numTendons,
                                      const ::physx::PxU32 maxTendons,
                                      const ::physx::PxU32 simMaxTendons);

bool fetchSpatialTendonOffset(float* dst,
                              const ::physx::PxGpuSpatialTendonData* tendonProperties,
                              const ::physx::PxU32 numTendons,
                              const ::physx::PxU32 maxTendons,
                              const ::physx::PxU32 simMaxTendons);

bool submitArtiFixedTendonProperties(::physx::PxGpuFixedTendonData* dst,
                                     const float* stiffnesses,
                                     const float* dampings,
                                     const float* limitStiffnesses,
                                     const float* limits,
                                     const float* restLengths,
                                     const float* offsets,
                                     const ::physx::PxU32* srcIndices,
                                     ::physx::PxU32* dirtyArtiGpuIndices,
                                     const ::physx::PxU32 numIndices,
                                     const ::physx::PxU32 maxTendons,
                                     const ::physx::PxU32 simMaxTendons,
                                     const GpuArticulationFixedTendonRecord* tensorRecords);

bool submitArtiSpatialTendonProperties(::physx::PxGpuSpatialTendonData* dst,
                                       const float* stiffnesses,
                                       const float* dampings,
                                       const float* limitStiffnesses,
                                       const float* offsets,
                                       const ::physx::PxU32* srcIndices,
                                       ::physx::PxU32* dirtyArtiGpuIndices,
                                        const ::physx::PxU32 numIndices,
                                       const ::physx::PxU32 maxTendons,
                                       const ::physx::PxU32 simMaxTendons,
                                       const GpuArticulationSpatialTendonRecord* tensorRecords);

bool fetchArtiLinkIncomingJointForce(PhysxGpuSpatialForces* dst,
                                     const PhysxGpuSpatialForces* src,
                                     const ::physx::PxTransform* linkTransforms,
                                     const ::physx::PxU32 numLinks,
                                     const ::physx::PxU32 maxLinks,
                                     const ::physx::PxU32 simMaxLinks,
                                     const GpuArticulationLinkRecord* linkRecords);

bool fetchDofProjectionForce(float* dst,
                             const PhysxGpuSpatialForces* src,
                             const ::physx::PxTransform* linkTransforms,
                             const ::physx::PxU32 numLinks,
                             const ::physx::PxU32 maxLinks,
                             const ::physx::PxU32 simMaxLinks,
                             const GpuArticulationLinkRecord* linkRecords);

bool submitArtiLinkForces(::physx::PxVec3* dstLinkForces,
                          ::physx::PxVec3* dstLinkTorques,
                          ::physx::PxU32* dirtyArtiGpuIndices,
                          const ::physx::PxTransform* linkTransforms,
                          const ::physx::PxVec3* actorAndLinksComs,
                          const ::physx::PxVec3* srcForces,
                          const ::physx::PxVec3* srcTorques,
                          const ::physx::PxVec3* srcPositions,
                          const ::physx::PxU32* srcArtiIndices,
                          const ::physx::PxU32 numArtIndices,
                          const ::physx::PxU32 numLinks,
                          const ::physx::PxU32 simMaxLinks,
                          const GpuArticulationLinkRecord* linkRecords,
                          const bool isGlobal,
                          const bool submitForces,
                          const bool submitTorques,
                          const bool applyAtPosition);

//
// rigid bodies
//
::physx::PxU32 fillRdTransforms(SingleAllocPolicy& policy,
                                ::physx::PxTransform* rdTransformDev,
                                ::physx::PxU32* indicesRet,
                                const ::physx::PxU32* allRdIndices,
                                const ActorGpuFlags* rdDirtyFlags,
                                const ActorGpuFlag::Enum flag,
                                const ::physx::PxU32 numRds);
::physx::PxU32 fillRdVelocities(SingleAllocPolicy& policy,
                                ::physx::PxVec3* rdLinVelDev,
                                ::physx::PxVec3* rdAngVelDev,
                                ::physx::PxU32* indicesRet,
                                const ::physx::PxU32* allRdIndices,
                                const ActorGpuFlags* rdDirtyFlags,
                                const ActorGpuFlag::Enum flag,
                                const ::physx::PxU32 numRds);

::physx::PxU32 fillRdFT(SingleAllocPolicy& policy,
                        ::physx::PxVec3* FT,
                        ::physx::PxU32* indicesRet,
                        const ::physx::PxU32* allRdIndices,
                        const ActorGpuFlags* rdDirtyFlags,
                        const ActorGpuFlag::Enum flag,
                        const ::physx::PxU32 numRds);

void exclusiveScan(::physx::PxU32* dstCounts, ::physx::PxU32* dstStartIndices, const ::physx::PxU32 numElem);

bool fetchRbTransforms(TensorTransform* dst,
                       const ::physx::PxTransform* actorData,
                       const ::physx::PxTransform* linkTransforms,
                       const ::physx::PxU32 numBodies,
                       const ::physx::PxU32 simMaxLinks,
                       const GpuRigidBodyRecord* rbRecords);

bool submitRbTransforms(::physx::PxTransform* dstActorData,
                        ::physx::PxTransform* dstRootTransforms,
                        ActorGpuFlags* rdDirtyFlags,
                        ArticulationGpuFlags* artiDirtyFlags,
                        const TensorTransform* src, // transforms in RigidBodyView
                        const ::physx::PxU32* srcRbIndices, // indices in RigidBodyView
                        const ::physx::PxU32 numRbIndices,
                        const ::physx::PxU32 numBodies,
                        const GpuRigidBodyRecord* rbRecords);


bool fetchRbVelAcc(TensorVelAcc* dst,
                   const ::physx::PxVec3* actorLinVelAcc,
                   const ::physx::PxVec3* actorAngVelAcc,
                   const ::physx::PxVec3* linkLinVelAcc,
                   const ::physx::PxVec3* linkAngVelAcc,
                   const ::physx::PxU32 numBodies,
                   const ::physx::PxU32 maxArtLinks,
                   const GpuRigidBodyRecord* rbRecords);

bool submitRbVelocities(::physx::PxVec3* actorLinVel,
                        ::physx::PxVec3* actorAngVel,
                        ::physx::PxVec3* linkLinVel,
                        ::physx::PxVec3* linkAngVel,
                        ActorGpuFlags* rdDirtyFlags,
                        ArticulationGpuFlags* artiDirtyFlags,
                        const TensorVelAcc* src, // velocities in RigidBodyView
                        const ::physx::PxU32* srcRbIndices, // indices in RigidBodyView
                        const ::physx::PxU32 numRbIndices,
                        const ::physx::PxU32 numRbs,
                        const GpuRigidBodyRecord* rbRecords);


bool submitRbForces(::physx::PxVec3* dstActorForces, // forces in physx rd staging buffer
                    ::physx::PxVec3* dstActorTorques, // torques in physx rd staging buffer
                    ::physx::PxVec3* dstLinkForces, // forces in physx link staging buffer
                    ::physx::PxVec3* dstLinkTorques, // forces in physx link staging buffer
                    ActorGpuFlags* rdDirtyFlags,
                    ArticulationGpuFlags* artiDirtyFlags,
                    ArticulationGpuFlags* artiLinksDirtyFlags,
                    const ::physx::PxTransform* linkTransforms,
                    const ::physx::PxTransform* actorData,
                    const ::physx::PxVec3* actorAndLinksComs,
                    const ::physx::PxVec3* srcForces, // forces in RigidBodyView
                    const ::physx::PxVec3* srcTorques, // torques in RigidBodyView
                    const ::physx::PxVec3* srcPositions, // positions to apply forces in RigidBodyView
                    const ::physx::PxU32* srcRbIndices, // indices in RigidBodyView
                    const ::physx::PxU32 numRbIndices,
                    const ::physx::PxU32 simMaxLinks,
                    const GpuRigidBodyRecord* rbRecords,
                    const bool isGlobal,
                    const bool submitForces,
                    const bool submitTorques,
                    const bool applyAtPosition);

// Rigid contacts
//

bool fetchNetRigidContactForces(::physx::PxVec3* netForces,
                                const ::physx::PxGpuContactPair* contactPairs,
                                ::physx::PxU32 numContactPairs,
                                ::physx::PxU32 maxLinks,
                                float timeStepInv,
                                const ::physx::PxU32* nodeIdx2ArtiGpuIdx,
                                const ::physx::PxU32* rdContactIndices,
                                const ::physx::PxU32* linkContactIndices);

bool fetchRigidContactForceMatrix(::physx::PxVec3* forceMatrix,
                                  const ::physx::PxGpuContactPair* contactPairs,
                                  ::physx::PxU32 numContactPairs,
                                  ::physx::PxU32 numFilters,
                                  ::physx::PxU32 maxLinks,
                                  float timeStepInv,
                                  const ::physx::PxU32* nodeIdx2ArtiGpuIdx,
                                  const ::physx::PxU32* rdContactIndices,
                                  const ::physx::PxU32* linkContactIndices,
                                  const GpuRigidContactFilterIdPair* filterLookup);

bool fetchRigidContactCount(::physx::PxU32* countMatrix,
                            const ::physx::PxGpuContactPair* contactPairs,
                            ::physx::PxU32 numContactPairs,
                            ::physx::PxU32 numFilters,
                            ::physx::PxU32 maxLinks,
                            const ::physx::PxU32* nodeIdx2ArtiGpuIdx,
                            const ::physx::PxU32* rdContactIndices,
                            const ::physx::PxU32* linkContactIndices,
                            const GpuRigidContactFilterIdPair* filterLookup);

bool fetchRigidContactData(::physx::PxReal* forceBuffer,
                           ::physx::PxVec3* pointBuffer,
                           ::physx::PxVec3* normalBuffer,
                           ::physx::PxReal* separationBuffer,
                           ::physx::PxU32* countMatrix,
                           ::physx::PxU32* startIndicesMatrix,
                           const ::physx::PxGpuContactPair* contactPairs,
                           ::physx::PxU32 numContactPairs,
                           ::physx::PxU32 numFilters,
                           ::physx::PxU32 numDataPoints,
                           ::physx::PxU32 maxLinks,
                           float timeStepInv,
                           const ::physx::PxU32* nodeIdx2ArtiGpuIdx,
                           const ::physx::PxU32* rdContactIndices,
                           const ::physx::PxU32* linkContactIndices,
                           const GpuRigidContactFilterIdPair* filterLookup);

bool fetchRigidFrictionData(::physx::PxVec3* forceBuffer,
                            ::physx::PxVec3* pointBuffer,
                            ::physx::PxU32* countMatrix,
                            ::physx::PxU32* startIndicesMatrix,
                            const ::physx::PxGpuContactPair* contactPairs,
                            ::physx::PxU32 numContactPairs,
                            ::physx::PxU32 numFilters,
                            ::physx::PxU32 numDataPoints,
                            ::physx::PxU32 maxLinks,
                            float timeStepInv,
                            const ::physx::PxU32* nodeIdx2ArtiGpuIdx,
                            const ::physx::PxU32* rdContactIndices,
                            const ::physx::PxU32* linkContactIndices,
                            const GpuRigidContactFilterIdPair* filterLookup);

bool fetchFrictionCount(::physx::PxU32* countMatrix,
                        const ::physx::PxGpuContactPair* contactPairs,
                        ::physx::PxU32 numContactPairs,
                        ::physx::PxU32 numFilters,
                        ::physx::PxU32 maxLinks,
                        const ::physx::PxU32* nodeIdx2ArtiGpuIdx,
                        const ::physx::PxU32* rdContactIndices,
                        const ::physx::PxU32* linkContactIndices,
                        const GpuRigidContactFilterIdPair* filterLookup);

// Raw contact data (no filter matching - returns all contacts for sensors)
bool fetchRawRigidContactCount(::physx::PxU32* countBuffer,
                               const ::physx::PxGpuContactPair* contactPairs,
                               ::physx::PxU32 numContactPairs,
                               ::physx::PxU32 maxLinks,
                               const ::physx::PxU32* nodeIdx2ArtiGpuIdx,
                               const ::physx::PxU32* rdContactIndices,
                               const ::physx::PxU32* linkContactIndices);

bool fetchRawRigidContactData(::physx::PxReal* forceBuffer,
                              ::physx::PxVec3* pointBuffer,
                              ::physx::PxVec3* normalBuffer,
                              ::physx::PxReal* separationBuffer,
                              uint64_t* actorIdBuffer,
                              ::physx::PxU32* countBuffer,
                              ::physx::PxU32* startIndicesBuffer,
                              const ::physx::PxGpuContactPair* contactPairs,
                              ::physx::PxU32 numContactPairs,
                              ::physx::PxU32 numDataPoints,
                              ::physx::PxU32 maxLinks,
                              float timeStepInv,
                              const ::physx::PxU32* nodeIdx2ArtiGpuIdx,
                              const ::physx::PxU32* rdContactIndices,
                              const ::physx::PxU32* linkContactIndices,
                              const GpuActorPathIdPair* actorPathLookup,
                              ::physx::PxU32 numActorPathPairs);

// Clamp per-sensor counts and start indices so start[i] + count[i] <= cap.
// Call after fetchRawRigidContactData to correct the
// atomically-incremented counts that may exceed the written prefix.
bool clampContactLayout(::physx::PxU32* counts, ::physx::PxU32* startIndices,
                        ::physx::PxU32 numSensors, ::physx::PxU32 cap);

// Interleave contiguous per-sensor counts/start indices into a caller-facing
// (numSensors, 2) layout tensor: column 0 count, column 1 start index.
bool packSensorLayout(::physx::PxU32* dstLayout, const ::physx::PxU32* counts,
                      const ::physx::PxU32* startIndices, ::physx::PxU32 numSensors);

bool submitSdfQueryPoints(::physx::PxVec4* dst,
                          const ::physx::PxVec3* src,
                          const GpuSdfShapeRecord* sdfRecords,
                          const ::physx::PxU32 numIndices,
                          const ::physx::PxU32 maxPointsPerShape);

// deformable bodies

bool fetchDeformableBodyUInt4Data(::physx::PxU32* dst,
                                  const GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const ::physx::PxU32 numBodies,
                                  const ::physx::PxU32 dstMaxElementsPerBody);

bool fetchDeformableBodyUInt3Data(::physx::PxU32* dst,
                                  const GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const ::physx::PxU32 numBodies,
                                  const ::physx::PxU32 dstMaxElementsPerBody);

bool fetchDeformableBodyVec3Data(::physx::PxVec3* dst,
                                 const GpuDeformableBodyRecord* bodyRecords,
                                 const DeformableBodyData::Enum dataFlag,
                                 const ::physx::PxU32 numBodies,
                                 const ::physx::PxU32 dstMaxElementsPerBody);

bool fetchDeformableBodyVec4Data(::physx::PxVec4* dst,
                                 const GpuDeformableBodyRecord* bodyRecords,
                                 const DeformableBodyData::Enum dataFlag,
                                 const ::physx::PxU32 numBodies,
                                 const ::physx::PxU32 dstMaxElementsPerBody);

bool submitDeformableBodyVec3Data(const ::physx::PxVec3* src,
                                  const ::physx::PxU32* indices,
                                  GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const ::physx::PxU32 numBodies,
                                  const ::physx::PxU32 srcMaxElementsPerBody);

bool submitDeformableBodyVec4Data(const ::physx::PxVec4* src,
                                  const ::physx::PxU32* indices,
                                  GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const ::physx::PxU32 numBodies,
                                  const ::physx::PxU32 srcMaxElementsPerBody);


//
// mask -> indices compaction
//
// Converts a uint8 mask [N] into a compact indices array [K] where K = number of nonzero mask elements.
// Uses Thrust's copy_if for GPU-friendly parallel stream compaction.
// This runs on the CUDA stream associated with the provided execution policy (currently the default stream).
// Returns true on success and writes K (number of selected indices) to outK.
bool compactMaskToIndices(SingleAllocPolicy& policy,
                          ::physx::PxU32* indicesOut,
                          const uint8_t* maskDev,
                          ::physx::PxU32 N,
                          ::physx::PxU32& outK);

// ovstage deformable sim-mesh columns. ONE launch covers every body in the read: each thread owns a
// vertex, finds its body through `records`, and writes straight into that body's slice of the
// destination column. `transforms` is the per-body world-to-sim-mesh matrix, applied only when
// `reframe` is set: points are published sim-mesh-local, velocities in world frame.
//
// The source is PhysX's own PxVec4 buffer and the destination is a packed PxVec3-shaped column, so
// the vec4->vec3 compaction happens here rather than in a second pass.
bool fetchPointSetColumnOvStage(float* dst,
                                const GpuPointSetReadRecord* records,
                                const PointSetTransform* transforms,
                                ::physx::PxU32 numSets,
                                ::physx::PxU32 maxPointsPerSet,
                                bool reframe);

// The ovstage POINT-SET write (ADR-0012): scatter one body's caller column into PhysX's
// own device buffer. The inverse of fetchPointSetColumnOvStage, but per BODY rather than rectangular
// over (set, point): a write session commits one group at a time, so there is no ragged set of them
// to launch across and no idle threads to trade away.
//
// `dst` is PhysX's buffer, whose fourth lane is INVERSE MASS on a position column. It is read back
// and re-stored per element rather than written: the caller's column is vec3, and a four-lane store
// would set every vertex's mass to infinite -- which the read cannot show, because it drops that
// lane, and which surfaces only as a body that has quietly stopped responding to force.
//
// `reframe` selects what the column MEANS, mirroring the gather: points arrive sim-mesh-local and
// are carried to world by `localToWorld`, velocities are world already and are stored as they came.
bool submitPointSetColumnOvStage(::physx::PxVec4* dst,
                                 const float* src,
                                 const PointSetTransform& localToWorld,
                                 ::physx::PxU32 numPoints,
                                 bool reframe);

} // namespace tensors
} // namespace physx
} // namespace omni
