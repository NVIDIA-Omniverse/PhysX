// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../PhysicsTypes.h"
#include "CudaCommon.h"
#include "GpuSimulationData.h"

namespace omni
{
namespace physx
{
namespace tensors
{
struct GpuArticulationRootRecord;
struct GpuArticulationDofRecord;
struct GpuArticulationLinkRecord;
struct GpuArticulationFixedTendonRecord;
struct GpuArticulationSpatialTendonRecord;
struct GpuRigidContactFilterIdPair;
struct GpuRigidBodyRecord;
struct GpuParticleClothRecord;
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
                           const ::physx::PxU32 simMassMatrixSize);

bool fetchArtiDofAttributeGravityAndCoriolis(float* dst,
                                             const float* src,
                                             const ::physx::PxU32 numDofs,
                                             const ::physx::PxU32 maxDofs,
                                             const ::physx::PxU32 simMaxDofs,
                                             const GpuArticulationDofRecord* dofRecords,
                                             const bool rootDofs);

bool fetchArtiCentroidalMomentumMatrices(float* dst,
                                         const float* src,
                                         const ::physx::PxU32 numElem,
                                         const ::physx::PxU32 maxDofs,
                                         const ::physx::PxU32 simMaxDofs,
                                         const ::physx::PxU32 cenMomBlockSize,
                                         const ::physx::PxU32 simCenMomBlockSize,
                                         const ::physx::PxU32 startSimBiasForceBlock);

bool fetchArtiJacobian(float* dst,
                       const float* src,
                       const ::physx::PxU32 numElements,
                       const ::physx::PxU32 jacobianSize,
                       const ::physx::PxU32 simJacobianSize);

bool fetchArtiDofAttribute(float* dst,
                           const float* src,
                           const ::physx::PxU32 numDofs,
                           const ::physx::PxU32 maxDofs,
                           const ::physx::PxU32 simMaxDofs,
                           const GpuArticulationDofRecord* dofRecords);

bool submitArtiDofAttribute(float* dst,
                            const float* src,
                            const ::physx::PxU32* srcArtiIndices,
                            ::physx::PxU32* dirtyArtiGpuIndices,
                            const ::physx::PxU32 numDofs,
                            const ::physx::PxU32 maxDofs,
                            const ::physx::PxU32 simMaxDofs,
                            const ::physx::PxU32 maxArtis,
                            const GpuArticulationDofRecord* dofRecords);

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

// bool submitRbTransforms(::physx::PxGpuBodyData* dstActorData,
//                         ::physx::PxTransform* dstRootTransforms,
//                         ActorGpuFlags* rdDirtyFlags,
//                         ArticulationGpuFlags* artiDirtyFlags,
//                         const TensorTransform* src, // transforms in RigidBodyView
//                         const ::physx::PxU32* srcRbIndices, // indices in RigidBodyView
//                         const ::physx::PxU32 numRbIndices,
//                         const GpuRigidBodyRecord* rbRecords);

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
//
// Particle Cloth
//

::physx::PxU32 getPcDirtyIndices(SingleAllocPolicy& policy,
                                 ::physx::PxU32* indicesRet,
                                 const ::physx::PxParticleBufferFlags* pcDirtyFlags,
                                 const ::physx::PxU32 numParticleCloths);

bool fetchParticleClothPositions(::physx::PxVec3* dst,
                                 const GpuParticleClothRecord* pcRecords,
                                 const ::physx::PxU32 numCloths,
                                 const ::physx::PxU32 maxParticlesPerBuffer);

bool submitParticleClothPositions(const ::physx::PxVec3* src,
                                  const ::physx::PxU32* indices,
                                  const ::physx::PxU32 numIndices,
                                  ::physx::PxParticleBufferFlags* clothDirtyFlags,
                                  const GpuParticleClothRecord* pcRecords,
                                  const ::physx::PxU32 maxParticlesPerBuffer);

bool fetchParticleClothVelocities(::physx::PxVec3* dst,
                                  const GpuParticleClothRecord* pcRecords,
                                  const ::physx::PxU32 numCloths,
                                  const ::physx::PxU32 maxParticlesPerBuffer);

bool submitParticleClothVelocities(const ::physx::PxVec3* src,
                                   const ::physx::PxU32* indices,
                                   const ::physx::PxU32 numIndices,
                                   ::physx::PxParticleBufferFlags* clothDirtyFlags,
                                   const GpuParticleClothRecord* pcRecords,
                                   const ::physx::PxU32 maxParticlesPerBuffer);

bool fetchParticleClothMasses(float* dst,
                              const GpuParticleClothRecord* pcRecords,
                              const ::physx::PxU32 numCloths,
                              const ::physx::PxU32 maxParticlesPerCloth);

bool submitParticleClothMasses(const float* src,
                               const ::physx::PxU32* indices,
                               const ::physx::PxU32 numIndices,
                               ::physx::PxParticleBufferFlags* clothDirtyFlags,
                               const GpuParticleClothRecord* pcRecords,
                               const ::physx::PxU32 maxParticlesPerBuffer);

bool fetchParticleClothSpringDamping(float* dst,
                                     const GpuParticleClothRecord* pxRecords,
                                     const ::physx::PxU32 numCloths,
                                     const ::physx::PxU32 maxSpringsPerCloth);

bool submitParticleClothSpringDamping(const float* src,
                                      const ::physx::PxU32* indices,
                                      const ::physx::PxU32 numIndices,
                                      const GpuParticleClothRecord* pcRecords,
                                      const ::physx::PxU32 maxSpringsPerCloth);

bool fetchParticleClothSpringStiffness(float* dst,
                                       const GpuParticleClothRecord* pxRecords,
                                       const ::physx::PxU32 numCloths,
                                       const ::physx::PxU32 maxSpringsPerCloth);

bool submitParticleClothSpringStiffness(const float* src,
                                        const ::physx::PxU32* indices,
                                        const ::physx::PxU32 numIndices,
                                        const GpuParticleClothRecord* pcRecords,
                                        const ::physx::PxU32 maxSpringsPerCloth);

bool submitSdfQueryPoints(::physx::PxVec4* dst,
                          const ::physx::PxVec3* src,
                          const GpuSdfShapeRecord* sdfRecords,
                          const ::physx::PxU32 numIndices,
                          const ::physx::PxU32 maxPointsPerShape);

// soft bodies
bool fetchSbElementRestPoses(::physx::PxMat33* dst,
                             GpuSoftBodyRecord* mSbRecordsDev,
                             const ::physx::PxU32 numSbs,
                             const ::physx::PxU32 maxElementsPerBody,
                             bool simulationMesh);

bool fetchSbElementRotations(::physx::PxQuat* dst,
                             GpuSoftBodyRecord* mSbRecordsDev,
                             const ::physx::PxU32 numSbs,
                             const ::physx::PxU32 maxElementsPerBody,
                             bool simulationMesh);

bool fetchSbElementIndices(::physx::PxU32* dst,
                           GpuSoftBodyRecord* mSbRecordsDev,
                           const ::physx::PxU32 numSbs,
                           const ::physx::PxU32 maxElementsPerBody,
                           bool simulationMesh);

bool computeSbElementStresses(::physx::PxMat33* dst,
                              GpuSoftBodyRecord* mSbRecords,
                              ::physx::PxVec4* materialProperties,
                              ::physx::PxFEMSoftBodyMaterialModel::Enum* materialModels,
                              const ::physx::PxU32 numSbs,
                              const ::physx::PxU32 maxElementsPerBody,
                              bool simMesh);

bool computeSbElementDeformationGradients(::physx::PxMat33* dst,
                                          GpuSoftBodyRecord* mSbRecordsDev,
                                          const ::physx::PxU32 numSbs,
                                          const ::physx::PxU32 maxElementsPerBody,
                                          bool simMesh);

bool fetchSbNodalValues(::physx::PxVec3* dst,
                        GpuSoftBodyRecord* mSbRecordsDev,
                        const ::physx::PxU32 numSbs,
                        const ::physx::PxU32 maxVerticesPerBody,
                        bool simulationMesh,
                        bool transform);

bool submitSbNodalValues(const ::physx::PxVec3* src,
                         const ::physx::PxU32* indices,
                         GpuSoftBodyRecord* mSbRecordsDev,
                         const ::physx::PxU32 numSbs,
                         const ::physx::PxU32 maxElementsPerBody,
                         bool transform);

bool fetchSbKinematicTargets(::physx::PxVec4* dst,
                             GpuSoftBodyRecord* mSbRecordsDev,
                             const ::physx::PxU32 numSbs,
                             const ::physx::PxU32 maxVerticesPerBody,
                             bool transform);

bool submitSbKinematicTargets(const ::physx::PxVec4* src,
                              const ::physx::PxU32* indices,
                              GpuSoftBodyRecord* mSbRecordsDev,
                              const ::physx::PxU32 numSbs,
                              const ::physx::PxU32 maxElementsPerBody,
                              bool transform);

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


} // namespace tensors
} // namespace physx
} // namespace omni
