// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <internal/Internal.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
}

// scene
bool updateGravityMagnitude(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            const PXR_NS::TfToken&,
                            const PXR_NS::UsdTimeCode&);
bool updateGravityDirection(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            const PXR_NS::TfToken&,
                            const PXR_NS::UsdTimeCode&);
bool updateTimeStepsPerSecond(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateSceneUpdateType(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const PXR_NS::TfToken&,
                           const PXR_NS::UsdTimeCode&);
bool updateQuasistaticEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateQuasistaticCollection(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&);

// body
bool updateBodyEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       const PXR_NS::TfToken&,
                       const PXR_NS::UsdTimeCode&);
bool updateBodyDensity(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       const PXR_NS::TfToken&,
                       const PXR_NS::UsdTimeCode&);
bool updateBodyLinearVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateBodyAngularVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const PXR_NS::TfToken&,
                               const PXR_NS::UsdTimeCode&);
bool updateBodyLinearDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const PXR_NS::TfToken&,
                             const PXR_NS::UsdTimeCode&);
bool updateBodyAngularDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateBodyMaxLinearVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&);
bool updateBodyMaxAngularVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId objectId,
                                  const PXR_NS::TfToken&,
                                  const PXR_NS::UsdTimeCode&);
bool updateBodySleepThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateBodyStabilizationThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateBodyMaxDepenetrationVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateBodyContactSlopCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const PXR_NS::TfToken& property,
                                      const PXR_NS::UsdTimeCode&);
bool updateBodyMaxContactImpulse(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&);
bool updateBodySolverPositionIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);
bool updateBodySolverVelocityIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);
bool updateBodyEnableKinematics(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateBodyEnableCCD(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const PXR_NS::TfToken&,
                         const PXR_NS::UsdTimeCode&);
bool updateBodyEnableSpeculativeCCD(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    const PXR_NS::TfToken&,
                                    const PXR_NS::UsdTimeCode&);
bool updateBodyRetainAccelerations(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateBodyGyroscopicForces(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateBodyDisableGravity(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateBodyLockedPosAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const PXR_NS::TfToken&,
                             const PXR_NS::UsdTimeCode&);
bool updateBodyLockedRotAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const PXR_NS::TfToken&,
                             const PXR_NS::UsdTimeCode&);
bool updateBodyTransformStack(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateBodyWorldForce(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          const PXR_NS::TfToken&,
                          const PXR_NS::UsdTimeCode&);
bool updateBodyWorldTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const PXR_NS::TfToken&,
                           const PXR_NS::UsdTimeCode&);
bool updateBodyCfmScale(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const PXR_NS::TfToken&,
                        const PXR_NS::UsdTimeCode&);
bool updateBodySolveContacts(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const PXR_NS::TfToken&,
                             const PXR_NS::UsdTimeCode&);
bool updateBodySimulationOwner(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const PXR_NS::TfToken&,
                               const PXR_NS::UsdTimeCode&);

bool updatePhysxContactReportThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);

bool updateBodyInstancedPositions(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId objectId,
                                  const PXR_NS::TfToken&,
                                  const PXR_NS::UsdTimeCode&);
bool updateBodyInstancedOrientations(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     const PXR_NS::TfToken&,
                                     const PXR_NS::UsdTimeCode&);
bool updateBodyInstancedVelocities(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateBodyInstancedAngularVelocities(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId objectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);

bool updateBodySurfaceVelocityEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateBodySurfaceLinearVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     const PXR_NS::TfToken&,
                                     const PXR_NS::UsdTimeCode&);
bool updateBodySurfaceAngularVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateBodySurfaceVelocityLocalSpace(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId objectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);
bool updateBodySplineSurfaceVelocityMagnitude(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId objectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);
bool updateBodySplineSurfaceVelocityEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              const PXR_NS::TfToken&,
                                              const PXR_NS::UsdTimeCode&);

    // force
bool updatePhysxForceEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const PXR_NS::TfToken&,
                             const PXR_NS::UsdTimeCode&);
bool updatePhysxForceWorldFrameEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updatePhysxForce(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      const PXR_NS::TfToken&,
                      const PXR_NS::UsdTimeCode&);
bool updatePhysxTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       const PXR_NS::TfToken&,
                       const PXR_NS::UsdTimeCode&);
bool updatePhysxForceMode(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          const PXR_NS::TfToken&,
                          const PXR_NS::UsdTimeCode&);

// shape
bool updateShapeEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const PXR_NS::TfToken&,
                        const PXR_NS::UsdTimeCode&);
bool updateShapeDensity(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const PXR_NS::TfToken&,
                        const PXR_NS::UsdTimeCode&);
bool updateShapeContactOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateShapeRestOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const PXR_NS::TfToken&,
                           const PXR_NS::UsdTimeCode&);
bool updateShapeTorsionalPatchRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     const PXR_NS::TfToken&,
                                     const PXR_NS::UsdTimeCode&);
bool updateShapeMinTorsionalPatchRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);

// material
bool updateMaterialDynamicFriction(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateMaterialStaticFriction(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId objectId,
                                  const PXR_NS::TfToken&,
                                  const PXR_NS::UsdTimeCode&);
bool updateMaterialRestitution(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const PXR_NS::TfToken&,
                               const PXR_NS::UsdTimeCode&);
bool updateMaterialFrictionCombineMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updateMaterialRestitutionCombineMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId objectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);
bool updateMaterialDampingCombineMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateCompliantMaterial(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const PXR_NS::TfToken& property,
                             const PXR_NS::UsdTimeCode&);

// joint
bool updateDriveTargetPosition(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const PXR_NS::TfToken&,
                               const PXR_NS::UsdTimeCode&);
bool updateDriveTargetVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const PXR_NS::TfToken&,
                               const PXR_NS::UsdTimeCode&);
bool updateDriveMaxForce(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const PXR_NS::TfToken&,
                         const PXR_NS::UsdTimeCode&);
bool updateDriveMaxActuatorVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const PXR_NS::TfToken&,
                        const PXR_NS::UsdTimeCode&);
bool updateDriveVelocityDependentResistance(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const PXR_NS::TfToken&,
                        const PXR_NS::UsdTimeCode&);
bool updateDriveSpeedEffortGradient(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const PXR_NS::TfToken&,
                        const PXR_NS::UsdTimeCode&);
bool updateDriveDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const PXR_NS::TfToken&,
                        const PXR_NS::UsdTimeCode&);
bool updateDriveStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          const PXR_NS::TfToken&,
                          const PXR_NS::UsdTimeCode&);
bool updateDriveType(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateLimitHigh(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateLimitLow(omni::physx::usdparser::AttachedStage& attachedStage,
                    omni::physx::usdparser::ObjectId objectId,
                    const PXR_NS::TfToken&,
                    const PXR_NS::UsdTimeCode&);
bool updateJointStatePosition(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateJointStateVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateEnableCollision(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const PXR_NS::TfToken&,
                           const PXR_NS::UsdTimeCode&);
bool updateBreakForce(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      const PXR_NS::TfToken&,
                      const PXR_NS::UsdTimeCode&);
bool updateBreakTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       const PXR_NS::TfToken&,
                       const PXR_NS::UsdTimeCode&);
bool updateLocalPos0(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateLocalPos1(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateLocalRot0(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateLocalRot1(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateGearRatio(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateGearHinge0(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      const PXR_NS::TfToken&,
                      const PXR_NS::UsdTimeCode&);
bool updateGearHinge1(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      const PXR_NS::TfToken&,
                      const PXR_NS::UsdTimeCode&);
bool updateRackPinionRatio(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const PXR_NS::TfToken&,
                           const PXR_NS::UsdTimeCode&);
bool updateRackHinge(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateRackPrismatic(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const PXR_NS::TfToken&,
                         const PXR_NS::UsdTimeCode&);
bool updateArmature(omni::physx::usdparser::AttachedStage& attachedStage,
                    omni::physx::usdparser::ObjectId objectId,
                    const PXR_NS::TfToken&,
                    const PXR_NS::UsdTimeCode&);
bool updateArmaturePerAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const PXR_NS::TfToken&,
                        const PXR_NS::UsdTimeCode&);
bool updateLimitBounceThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateLimitDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const PXR_NS::TfToken&,
                        const PXR_NS::UsdTimeCode&);
bool updateLimitRestitution(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            const PXR_NS::TfToken&,
                            const PXR_NS::UsdTimeCode&);
bool updateLimitStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          const PXR_NS::TfToken&,
                          const PXR_NS::UsdTimeCode&);
bool updateDistanceJointSpringDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateDistanceJointSpringStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateDistanceJointSpringEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);

// articulation
bool updateArticulationFixBase(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const PXR_NS::TfToken&,
                               const PXR_NS::UsdTimeCode&);
bool updateArticulationSolverPositionIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                                    omni::physx::usdparser::ObjectId objectId,
                                                    const PXR_NS::TfToken&,
                                                    const PXR_NS::UsdTimeCode&);
bool updateArticulationSolverVelocityIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                                    omni::physx::usdparser::ObjectId objectId,
                                                    const PXR_NS::TfToken&,
                                                    const PXR_NS::UsdTimeCode&);
bool updateArticulationSleepThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateArticulationStabilizationThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              const PXR_NS::TfToken&,
                                              const PXR_NS::UsdTimeCode&);

// articulation link
bool updateArticulationMaxJointVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateArticulationMaxJointVelocityPerAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);
bool updateArticulationFrictionCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId objectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);
bool updateArticulationStaticFrictionEffort(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);
bool updateArticulationDynamicFrictionEffort(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId objectId,
                                                const PXR_NS::TfToken&,
                                                const PXR_NS::UsdTimeCode&);
bool updateArticulationViscousFrictionCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);

// collision group
inline bool updateCollisionGroup(omni::physx::usdparser::AttachedStage& /*attachedStage*/,
                                 omni::physx::usdparser::ObjectId /*objectId*/,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&)
{
    return true;
}

// filtered pairs
bool updateFilteredPairs(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const PXR_NS::TfToken&,
                         const PXR_NS::UsdTimeCode&);

// cct
bool updateCctSlopeLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const PXR_NS::TfToken&,
                         const PXR_NS::UsdTimeCode&);
bool updateCctHeight(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateCctRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateCctContactOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            const PXR_NS::TfToken&,
                            const PXR_NS::UsdTimeCode&);
bool updateCctStepOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const PXR_NS::TfToken&,
                         const PXR_NS::UsdTimeCode&);
bool updateCctUpAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const PXR_NS::TfToken&,
                     const PXR_NS::UsdTimeCode&);
bool updateCctNonWalkableMode(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateCctClimbingMode(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const PXR_NS::TfToken&,
                           const PXR_NS::UsdTimeCode&);

// particle system
bool updateParticleSystemAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updatePBDMaterialAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateParticleDensity(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const PXR_NS::TfToken&,
                           const PXR_NS::UsdTimeCode&);

bool updateParticleSmoothingEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId objectId,
                                             const PXR_NS::TfToken& property,
                                             const PXR_NS::UsdTimeCode&);
bool updateParticleAnisotropyEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              const PXR_NS::TfToken& property,
                                              const PXR_NS::UsdTimeCode&);
bool updateParticleIsosurfaceEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              const PXR_NS::TfToken& property,
                                              const PXR_NS::UsdTimeCode&);
bool updateParticleIsosurfaceAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       const PXR_NS::TfToken& property,
                                       const PXR_NS::UsdTimeCode&);

// partice set
bool updateParticleSetEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateParticleSetSelfCollision(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    const PXR_NS::TfToken&,
                                    const PXR_NS::UsdTimeCode&);
bool updateParticleSetFluid(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            const PXR_NS::TfToken&,
                            const PXR_NS::UsdTimeCode&);
bool updateParticleSetParticleGroup(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    const PXR_NS::TfToken&,
                                    const PXR_NS::UsdTimeCode&);

bool updateParticlePositions(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const PXR_NS::TfToken&,
                             const PXR_NS::UsdTimeCode&);
bool updateParticleSimPositions(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateParticleVelocities(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);

bool updateDiffuseParticlesEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const PXR_NS::TfToken& property,
                                            const PXR_NS::UsdTimeCode&);
bool updateDiffuseParticlesAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     const PXR_NS::TfToken& property,
                                     const PXR_NS::UsdTimeCode&);

// vehicle
bool updateVehicleContextUpdateMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    const PXR_NS::TfToken&,
                                    const PXR_NS::UsdTimeCode&);
bool updateVehicleContextVerticalAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleContextLongitudinalAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);

bool updateVehicleEngineMomentOfInertia(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleEnginePeakTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateVehicleEngineMaxRotationSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleEngineIdleRotationSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);
bool updateVehicleEngineTorqueCurve(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    const PXR_NS::TfToken&,
                                    const PXR_NS::UsdTimeCode&);
bool updateVehicleEngineDampingRateFullThrottle(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId,
                                                const PXR_NS::TfToken&,
                                                const PXR_NS::UsdTimeCode&);
bool updateVehicleEngineDampingRateZeroThrottleClutchEngaged(omni::physx::usdparser::AttachedStage& attachedStage,
                                                             omni::physx::usdparser::ObjectId,
                                                             const PXR_NS::TfToken&,
                                                             const PXR_NS::UsdTimeCode&);
bool updateVehicleEngineDampingRateZeroThrottleClutchDisengaged(omni::physx::usdparser::AttachedStage& attachedStage,
                                                                omni::physx::usdparser::ObjectId,
                                                                const PXR_NS::TfToken&,
                                                                const PXR_NS::UsdTimeCode&);

bool updateVehicleTireFrictionTableFrictionValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId,
                                                  const PXR_NS::TfToken&,
                                                  const PXR_NS::UsdTimeCode&);
bool updateVehicleTireFrictionTableGroundMaterials(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const PXR_NS::TfToken&,
                                                   const PXR_NS::UsdTimeCode&);
bool updateVehicleTireFrictionTableDefaultFrictionValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                        omni::physx::usdparser::ObjectId,
                                                        const PXR_NS::TfToken&,
                                                        const PXR_NS::UsdTimeCode&);

bool updateVehicleSuspensionSpringStrength(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionSpringDamperRate(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const PXR_NS::TfToken&,
                                             const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionMaxCompression(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionMaxDroop(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     const PXR_NS::TfToken&,
                                     const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionTravelDistance(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionSprungMass(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionCamberAtRest(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionCamberAtMaxCompression(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const PXR_NS::TfToken&,
                                                   const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionCamberAtMaxDroop(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const PXR_NS::TfToken&,
                                             const PXR_NS::UsdTimeCode&);

bool updateVehicleTireLatStiffX(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateVehicleTireLatStiffY(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateVehicleTireLateralStiffnessGraph(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);
bool updateVehicleTireLongStiffPerGrav(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updateVehicleTireLongitudinalStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);
bool updateVehicleTireCamberStiffPerGrav(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleTireCamberStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleTireFrictionVsSlip(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     const PXR_NS::TfToken&,
                                     const PXR_NS::UsdTimeCode&);
bool updateVehicleTireFrictionTableRel(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updateVehicleTireRestLoad(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId,
                               const PXR_NS::TfToken&,
                               const PXR_NS::UsdTimeCode&);

bool updateVehicleWheelRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelWidth(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId,
                             const PXR_NS::TfToken&,
                             const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelMass(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId,
                            const PXR_NS::TfToken&,
                            const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelMomentOfInertia(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelDampingRate(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelMaxBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelMaxHandBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelMaxSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     const PXR_NS::TfToken&,
                                     const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelToeAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);

bool updateVehicleWheelAttachmentIndex(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentWheel(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentTire(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspension(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspensionTravelDirection(omni::physx::usdparser::AttachedStage& attachedStage,
                                                           omni::physx::usdparser::ObjectId,
                                                           const PXR_NS::TfToken&,
                                                           const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspensionForceAppPointOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                                               omni::physx::usdparser::ObjectId,
                                                               const PXR_NS::TfToken&,
                                                               const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentWheelCenterOfMassOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         const PXR_NS::TfToken&,
                                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentTireForceAppPointOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         const PXR_NS::TfToken&,
                                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspensionFramePosition(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         const PXR_NS::TfToken&,
                                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspensionFrameOrientation(omni::physx::usdparser::AttachedStage& attachedStage,
                                                            omni::physx::usdparser::ObjectId,
                                                            const PXR_NS::TfToken&,
                                                            const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentWheelFramePosition(omni::physx::usdparser::AttachedStage& attachedStage,
                                                    omni::physx::usdparser::ObjectId,
                                                    const PXR_NS::TfToken&,
                                                    const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentWheelFrameOrientation(omni::physx::usdparser::AttachedStage& attachedStage,
                                                       omni::physx::usdparser::ObjectId,
                                                       const PXR_NS::TfToken&,
                                                       const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentDriven(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelAttachmentCollisionGroup(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId,
                                                const PXR_NS::TfToken&,
                                                const PXR_NS::UsdTimeCode&);

bool updateVehicleSuspensionComplWheelToeAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                               omni::physx::usdparser::ObjectId,
                                               const PXR_NS::TfToken&,
                                               const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionComplWheelCamberAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId,
                                                  const PXR_NS::TfToken&,
                                                  const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionComplSuspForceAppPoint(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const PXR_NS::TfToken&,
                                                   const PXR_NS::UsdTimeCode&);
bool updateVehicleSuspensionComplTireForceAppPoint(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const PXR_NS::TfToken&,
                                                   const PXR_NS::UsdTimeCode&);

bool updateVehicleEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId,
                          const PXR_NS::TfToken&,
                          const PXR_NS::UsdTimeCode&);
bool updateVehicleLimitSuspensionExpansionVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const PXR_NS::TfToken&,
                                                   const PXR_NS::UsdTimeCode&);
bool updateVehicleMinPassiveLongslipDenom(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);
bool updateVehicleMinActiveLongslipDenom(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleMinLateralSlipDenom(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleLongitudinalStickyTireThresholdSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                                       omni::physx::usdparser::ObjectId,
                                                       const PXR_NS::TfToken&,
                                                       const PXR_NS::UsdTimeCode&);
bool updateVehicleLongitudinalStickyTireThresholdTime(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId,
                                                      const PXR_NS::TfToken&,
                                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleLongitudinalStickyTireDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId,
                                                const PXR_NS::TfToken&,
                                                const PXR_NS::UsdTimeCode&);
bool updateVehicleLateralStickyTireThresholdSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId,
                                                  const PXR_NS::TfToken&,
                                                  const PXR_NS::UsdTimeCode&);
bool updateVehicleLateralStickyTireThresholdTime(omni::physx::usdparser::AttachedStage& attachedStage,
                                                 omni::physx::usdparser::ObjectId,
                                                 const PXR_NS::TfToken&,
                                                 const PXR_NS::UsdTimeCode&);
bool updateVehicleLateralStickyTireDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);

bool updateVehicleControllerAccelerator(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleControllerBrake0(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateVehicleControllerBrake1(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateVehicleControllerBrake(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  const PXR_NS::TfToken&,
                                  const PXR_NS::UsdTimeCode&);
bool updateVehicleControllerHandbrake(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleControllerSteer(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  const PXR_NS::TfToken&,
                                  const PXR_NS::UsdTimeCode&);
bool updateVehicleControllerSteerLeft(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleControllerSteerRight(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updateVehicleControllerTargetGear(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);

bool updateVehicleTankControllerThrust0(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleTankControllerThrust1(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);

bool updateVehicleDriveBasicPeakTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);

bool updateVehicleWheelControllerDriveTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const PXR_NS::TfToken&,
                                             const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelControllerBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const PXR_NS::TfToken&,
                                             const PXR_NS::UsdTimeCode&);
bool updateVehicleWheelControllerSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);

bool updateVehicleMultiWheelDifferentialWheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                               omni::physx::usdparser::ObjectId,
                                               const PXR_NS::TfToken&,
                                               const PXR_NS::UsdTimeCode&);
bool updateVehicleMultiWheelDifferentialTorqueRatios(omni::physx::usdparser::AttachedStage& attachedStage,
                                                     omni::physx::usdparser::ObjectId,
                                                     const PXR_NS::TfToken&,
                                                     const PXR_NS::UsdTimeCode&);
bool updateVehicleMultiWheelDifferentialAverageWheelSpeedRatios(omni::physx::usdparser::AttachedStage& attachedStage,
                                                                omni::physx::usdparser::ObjectId,
                                                                const PXR_NS::TfToken&,
                                                                const PXR_NS::UsdTimeCode&);

bool updateVehicleTankDifferentialNumberOfWheelsPerTrack(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         const PXR_NS::TfToken&,
                                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleTankDifferentialThrustIndexPerTrack(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId,
                                                      const PXR_NS::TfToken&,
                                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleTankDifferentialTrackToWheelIndices(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId,
                                                      const PXR_NS::TfToken&,
                                                      const PXR_NS::UsdTimeCode&);
bool updateVehicleTankDifferentialWheelIndicesInTrackOrder(omni::physx::usdparser::AttachedStage& attachedStage,
                                                           omni::physx::usdparser::ObjectId,
                                                           const PXR_NS::TfToken&,
                                                           const PXR_NS::UsdTimeCode&);

bool updateVehicleBrakes0Wheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateVehicleBrakes1Wheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateVehicleBrakes0MaxBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleBrakes1MaxBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleBrakes0TorqueMultipliers(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);
bool updateVehicleBrakes1TorqueMultipliers(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);

bool updateVehicleSteeringWheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&);
bool updateVehicleSteeringMaxSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleSteeringAngleMultipliers(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);

bool updateVehicleAckermannSteeringWheel0(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);
bool updateVehicleAckermannSteeringWheel1(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);
bool updateVehicleAckermannSteeringMaxSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                                 omni::physx::usdparser::ObjectId,
                                                 const PXR_NS::TfToken&,
                                                 const PXR_NS::UsdTimeCode&);
bool updateVehicleAckermannSteeringWheelBase(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const PXR_NS::TfToken&,
                                             const PXR_NS::UsdTimeCode&);
bool updateVehicleAckermannSteeringTrackWidth(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId,
                                              const PXR_NS::TfToken&,
                                              const PXR_NS::UsdTimeCode&);
bool updateVehicleAckermannSteeringStrength(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);

bool updateVehicleNCRDriveCommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleNCRSteerCommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleNCRBrakes0CommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);
bool updateVehicleNCRBrakes1CommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);

bool updateVehicleNCRDriveSpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                        omni::physx::usdparser::ObjectId,
                                                        const PXR_NS::TfToken&,
                                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleNCRSteerSpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                        omni::physx::usdparser::ObjectId,
                                                        const PXR_NS::TfToken&,
                                                        const PXR_NS::UsdTimeCode&);
bool updateVehicleNCRBrakes0SpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                          omni::physx::usdparser::ObjectId,
                                                          const PXR_NS::TfToken&,
                                                          const PXR_NS::UsdTimeCode&);
bool updateVehicleNCRBrakes1SpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                          omni::physx::usdparser::ObjectId,
                                                          const PXR_NS::TfToken&,
                                                          const PXR_NS::UsdTimeCode&);

bool updateVehicleNCRDriveSpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleNCRSteerSpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);
bool updateVehicleNCRBrakes0SpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);
bool updateVehicleNCRBrakes1SpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const PXR_NS::TfToken&,
                                           const PXR_NS::UsdTimeCode&);

// spatial tendons
bool updateSpatialTendonStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  const PXR_NS::TfToken&,
                                  const PXR_NS::UsdTimeCode&);
bool updateSpatialTendonDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateSpatialTendonLimitStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const PXR_NS::TfToken&,
                                       const PXR_NS::UsdTimeCode&);
bool updateSpatialTendonOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId,
                               const PXR_NS::TfToken&,
                               const PXR_NS::UsdTimeCode&);
bool updateSpatialTendonEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateTendonAttachmentGearing(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateTendonAttachmentLocalPos(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    const PXR_NS::TfToken&,
                                    const PXR_NS::UsdTimeCode&);
bool updateTendonAttachmentLeafRestLength(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const PXR_NS::TfToken&,
                                          const PXR_NS::UsdTimeCode&);
bool updateTendonAttachmentLeafLowLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const PXR_NS::TfToken&,
                                        const PXR_NS::UsdTimeCode&);
bool updateTendonAttachmentLeafHighLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);

// fixed tendons
bool updateFixedTendonStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateFixedTendonLimitStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     const PXR_NS::TfToken&,
                                     const PXR_NS::UsdTimeCode&);
bool updateFixedTendonDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateFixedTendonOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId,
                             const PXR_NS::TfToken&,
                             const PXR_NS::UsdTimeCode&);
bool updateFixedTendonRestLength(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&);
bool updateFixedTendonLowLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId,
                               const PXR_NS::TfToken&,
                               const PXR_NS::UsdTimeCode&);
bool updateFixedTendonHighLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateFixedTendonEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId,
                              const PXR_NS::TfToken&,
                              const PXR_NS::UsdTimeCode&);
bool updateTendonAxisSingleGearing(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateTendonAxisSingleForceCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdTimeCode&);

// deformables
bool updateDeformableBody(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId,
                          const PXR_NS::TfToken&,
                          const PXR_NS::UsdTimeCode&);
bool updateDeformableContactOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const PXR_NS::TfToken&,
                                   const PXR_NS::UsdTimeCode&);
bool updateDeformableRestOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateDeformableMaterial(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const PXR_NS::TfToken& property,
                              const PXR_NS::UsdTimeCode&);

// mimic joints
bool updateMimicJointGearing(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId,
                             const PXR_NS::TfToken&,
                             const PXR_NS::UsdTimeCode&);
bool updateMimicJointOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId,
                            const PXR_NS::TfToken&,
                            const PXR_NS::UsdTimeCode&);
bool updateMimicJointNaturalFrequency(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);
bool updateMimicJointDampingRatio(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  const PXR_NS::TfToken&,
                                  const PXR_NS::UsdTimeCode&);

bool updateNewtonMimicJointCoef1(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&);
bool updateNewtonMimicJointCoef0(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&);

// note: the other mimic joint properties trigger a structural change, so a reparse, thus no change methods for those

// Newton schema runtime change tracking. Each wrapper applies the Newton value only when the
// corresponding PhysX attribute is not authored on the prim (PhysX > Newton priority).
bool updateNewtonTimeStepsPerSecond(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    const PXR_NS::TfToken&,
                                    const PXR_NS::UsdTimeCode&);
bool updateNewtonGravityEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const PXR_NS::TfToken&,
                                const PXR_NS::UsdTimeCode&);
bool updateNewtonShapeContactMargin(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    const PXR_NS::TfToken&,
                                    const PXR_NS::UsdTimeCode&);
bool updateNewtonShapeContactGap(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&);
bool updateNewtonDeformableContactMargin(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdTimeCode&);
bool updateNewtonDeformableContactGap(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const PXR_NS::TfToken&,
                                      const PXR_NS::UsdTimeCode&);

} // namespace physx
} // namespace omni
