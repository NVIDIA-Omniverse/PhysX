// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
                            const pxr::TfToken&,
                            const pxr::UsdTimeCode&);
bool updateGravityDirection(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            const pxr::TfToken&,
                            const pxr::UsdTimeCode&);
bool updateTimeStepsPerSecond(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateSceneUpdateType(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const pxr::TfToken&,
                           const pxr::UsdTimeCode&);
bool updateQuasistaticEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateQuasistaticCollection(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 const pxr::TfToken&,
                                 const pxr::UsdTimeCode&);

// body
bool updateBodyEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       const pxr::TfToken&,
                       const pxr::UsdTimeCode&);
bool updateBodyDensity(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       const pxr::TfToken&,
                       const pxr::UsdTimeCode&);
bool updateBodyLinearVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateBodyAngularVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const pxr::TfToken&,
                               const pxr::UsdTimeCode&);
bool updateBodyLinearDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const pxr::TfToken&,
                             const pxr::UsdTimeCode&);
bool updateBodyAngularDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateBodyMaxLinearVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 const pxr::TfToken&,
                                 const pxr::UsdTimeCode&);
bool updateBodyMaxAngularVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId objectId,
                                  const pxr::TfToken&,
                                  const pxr::UsdTimeCode&);
bool updateBodySleepThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateBodyStabilizationThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateBodyMaxDepenetrationVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateBodyContactSlopCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const pxr::TfToken& property,
                                      const pxr::UsdTimeCode&);
bool updateBodyMaxContactImpulse(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 const pxr::TfToken&,
                                 const pxr::UsdTimeCode&);
bool updateBodySolverPositionIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);
bool updateBodySolverVelocityIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);
bool updateBodyEnableKinematics(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateBodyEnableCCD(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const pxr::TfToken&,
                         const pxr::UsdTimeCode&);
bool updateBodyEnableSpeculativeCCD(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    const pxr::TfToken&,
                                    const pxr::UsdTimeCode&);
bool updateBodyRetainAccelerations(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateBodyGyroscopicForces(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateBodyDisableGravity(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateBodyLockedPosAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const pxr::TfToken&,
                             const pxr::UsdTimeCode&);
bool updateBodyLockedRotAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const pxr::TfToken&,
                             const pxr::UsdTimeCode&);
bool updateBodyTransformStack(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateBodyWorldForce(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          const pxr::TfToken&,
                          const pxr::UsdTimeCode&);
bool updateBodyWorldTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const pxr::TfToken&,
                           const pxr::UsdTimeCode&);
bool updateBodyCfmScale(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const pxr::TfToken&,
                        const pxr::UsdTimeCode&);
bool updateBodySolveContacts(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const pxr::TfToken&,
                             const pxr::UsdTimeCode&);
bool updateBodySimulationOwner(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const pxr::TfToken&,
                               const pxr::UsdTimeCode&);

bool updatePhysxContactReportThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);

bool updateBodyInstancedPositions(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId objectId,
                                  const pxr::TfToken&,
                                  const pxr::UsdTimeCode&);
bool updateBodyInstancedOrientations(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     const pxr::TfToken&,
                                     const pxr::UsdTimeCode&);
bool updateBodyInstancedVelocities(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateBodyInstancedAngularVelocities(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId objectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);

bool updateBodySurfaceVelocityEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateBodySurfaceLinearVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     const pxr::TfToken&,
                                     const pxr::UsdTimeCode&);
bool updateBodySurfaceAngularVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateBodySurfaceVelocityLocalSpace(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId objectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);
bool updateBodySplineSurfaceVelocityMagnitude(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId objectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);
bool updateBodySplineSurfaceVelocityEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              const pxr::TfToken&,
                                              const pxr::UsdTimeCode&);

    // force
bool updatePhysxForceEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const pxr::TfToken&,
                             const pxr::UsdTimeCode&);
bool updatePhysxForceWorldFrameEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updatePhysxForce(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      const pxr::TfToken&,
                      const pxr::UsdTimeCode&);
bool updatePhysxTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       const pxr::TfToken&,
                       const pxr::UsdTimeCode&);
bool updatePhysxForceMode(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          const pxr::TfToken&,
                          const pxr::UsdTimeCode&);

// shape
bool updateShapeEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const pxr::TfToken&,
                        const pxr::UsdTimeCode&);
bool updateShapeDensity(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const pxr::TfToken&,
                        const pxr::UsdTimeCode&);
bool updateShapeContactOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateShapeRestOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const pxr::TfToken&,
                           const pxr::UsdTimeCode&);
bool updateShapeTorsionalPatchRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     const pxr::TfToken&,
                                     const pxr::UsdTimeCode&);
bool updateShapeMinTorsionalPatchRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);

// material
bool updateMaterialDynamicFriction(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateMaterialStaticFriction(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId objectId,
                                  const pxr::TfToken&,
                                  const pxr::UsdTimeCode&);
bool updateMaterialRestitution(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const pxr::TfToken&,
                               const pxr::UsdTimeCode&);
bool updateMaterialFrictionCombineMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updateMaterialRestitutionCombineMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId objectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);
bool updateMaterialDampingCombineMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateCompliantMaterial(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const pxr::TfToken& property,
                             const pxr::UsdTimeCode&);

// joint
bool updateDriveTargetPosition(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const pxr::TfToken&,
                               const pxr::UsdTimeCode&);
bool updateDriveTargetVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const pxr::TfToken&,
                               const pxr::UsdTimeCode&);
bool updateDriveMaxForce(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const pxr::TfToken&,
                         const pxr::UsdTimeCode&);
bool updateDriveMaxActuatorVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const pxr::TfToken&,
                        const pxr::UsdTimeCode&);
bool updateDriveVelocityDependentResistance(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const pxr::TfToken&,
                        const pxr::UsdTimeCode&);
bool updateDriveSpeedEffortGradient(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const pxr::TfToken&,
                        const pxr::UsdTimeCode&);
bool updateDriveDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const pxr::TfToken&,
                        const pxr::UsdTimeCode&);
bool updateDriveStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          const pxr::TfToken&,
                          const pxr::UsdTimeCode&);
bool updateDriveType(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateLimitHigh(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateLimitLow(omni::physx::usdparser::AttachedStage& attachedStage,
                    omni::physx::usdparser::ObjectId objectId,
                    const pxr::TfToken&,
                    const pxr::UsdTimeCode&);
bool updateJointStatePosition(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateJointStateVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateEnableCollision(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const pxr::TfToken&,
                           const pxr::UsdTimeCode&);
bool updateBreakForce(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      const pxr::TfToken&,
                      const pxr::UsdTimeCode&);
bool updateBreakTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       const pxr::TfToken&,
                       const pxr::UsdTimeCode&);
bool updateLocalPos0(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateLocalPos1(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateLocalRot0(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateLocalRot1(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateGearRatio(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateGearHinge0(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      const pxr::TfToken&,
                      const pxr::UsdTimeCode&);
bool updateGearHinge1(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      const pxr::TfToken&,
                      const pxr::UsdTimeCode&);
bool updateRackPinionRatio(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const pxr::TfToken&,
                           const pxr::UsdTimeCode&);
bool updateRackHinge(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateRackPrismatic(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const pxr::TfToken&,
                         const pxr::UsdTimeCode&);
bool updateArmature(omni::physx::usdparser::AttachedStage& attachedStage,
                    omni::physx::usdparser::ObjectId objectId,
                    const pxr::TfToken&,
                    const pxr::UsdTimeCode&);
bool updateArmaturePerAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const pxr::TfToken&,
                        const pxr::UsdTimeCode&);
bool updateLimitBounceThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateLimitDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        const pxr::TfToken&,
                        const pxr::UsdTimeCode&);
bool updateLimitRestitution(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            const pxr::TfToken&,
                            const pxr::UsdTimeCode&);
bool updateLimitStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          const pxr::TfToken&,
                          const pxr::UsdTimeCode&);
bool updateDistanceJointSpringDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateDistanceJointSpringStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateDistanceJointSpringEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);

// articulation
bool updateArticulationFixBase(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               const pxr::TfToken&,
                               const pxr::UsdTimeCode&);
bool updateArticulationSolverPositionIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                                    omni::physx::usdparser::ObjectId objectId,
                                                    const pxr::TfToken&,
                                                    const pxr::UsdTimeCode&);
bool updateArticulationSolverVelocityIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                                    omni::physx::usdparser::ObjectId objectId,
                                                    const pxr::TfToken&,
                                                    const pxr::UsdTimeCode&);
bool updateArticulationSleepThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateArticulationStabilizationThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              const pxr::TfToken&,
                                              const pxr::UsdTimeCode&);

// articulation link
bool updateArticulationMaxJointVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateArticulationMaxJointVelocityPerAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);
bool updateArticulationFrictionCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId objectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);
bool updateArticulationStaticFrictionEffort(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);
bool updateArticulationDynamicFrictionEffort(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId objectId,
                                                const pxr::TfToken&,
                                                const pxr::UsdTimeCode&);
bool updateArticulationViscousFrictionCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);

// collision group
inline bool updateCollisionGroup(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 const pxr::TfToken&,
                                 const pxr::UsdTimeCode&)
{
    return true;
}

// filtered pairs
bool updateFilteredPairs(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const pxr::TfToken&,
                         const pxr::UsdTimeCode&);

// cct
bool updateCctSlopeLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const pxr::TfToken&,
                         const pxr::UsdTimeCode&);
bool updateCctHeight(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateCctRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateCctContactOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            const pxr::TfToken&,
                            const pxr::UsdTimeCode&);
bool updateCctStepOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         const pxr::TfToken&,
                         const pxr::UsdTimeCode&);
bool updateCctUpAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     const pxr::TfToken&,
                     const pxr::UsdTimeCode&);
bool updateCctNonWalkableMode(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateCctClimbingMode(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const pxr::TfToken&,
                           const pxr::UsdTimeCode&);

// particle system
bool updateParticleSystemAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updatePBDMaterialAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateParticleDensity(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           const pxr::TfToken&,
                           const pxr::UsdTimeCode&);

bool updateParticleSmoothingEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId objectId,
                                             const pxr::TfToken& property,
                                             const pxr::UsdTimeCode&);
bool updateParticleAnisotropyEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              const pxr::TfToken& property,
                                              const pxr::UsdTimeCode&);
bool updateParticleIsosurfaceEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              const pxr::TfToken& property,
                                              const pxr::UsdTimeCode&);
bool updateParticleIsosurfaceAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       const pxr::TfToken& property,
                                       const pxr::UsdTimeCode&);

// partice set
bool updateParticleSetEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateParticleSetSelfCollision(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    const pxr::TfToken&,
                                    const pxr::UsdTimeCode&);
bool updateParticleSetFluid(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            const pxr::TfToken&,
                            const pxr::UsdTimeCode&);
bool updateParticleSetParticleGroup(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    const pxr::TfToken&,
                                    const pxr::UsdTimeCode&);

bool updateParticlePositions(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             const pxr::TfToken&,
                             const pxr::UsdTimeCode&);
bool updateParticleSimPositions(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateParticleVelocities(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);

bool updateDiffuseParticlesEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            const pxr::TfToken& property,
                                            const pxr::UsdTimeCode&);
bool updateDiffuseParticlesAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     const pxr::TfToken& property,
                                     const pxr::UsdTimeCode&);

// DEPRECATED particle cloth
bool updateParticleClothEnabledDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId objectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);
bool updateParticleClothSelfCollisionDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId objectId,
                                                const pxr::TfToken&,
                                                const pxr::UsdTimeCode&);
bool updateParticleClothSelfCollisionFilterDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId objectId,
                                                      const pxr::TfToken&,
                                                      const pxr::UsdTimeCode&);
bool updateParticleClothParticleGroupDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId objectId,
                                                const pxr::TfToken&,
                                                const pxr::UsdTimeCode&);
bool updateParticleClothPressureDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId objectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);
bool updateParticleClothPointsDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId objectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);
bool updateParticleClothVelocitiesDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId objectId,
                                             const pxr::TfToken&,
                                             const pxr::UsdTimeCode&);
bool warnParticleClothNoRuntimeCookingDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                                 omni::physx::usdparser::ObjectId objectId,
                                                 const pxr::TfToken&,
                                                 const pxr::UsdTimeCode&);

// vehicle
bool updateVehicleContextUpdateMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    const pxr::TfToken&,
                                    const pxr::UsdTimeCode&);
bool updateVehicleContextVerticalAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateVehicleContextLongitudinalAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);

bool updateVehicleEngineMomentOfInertia(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateVehicleEnginePeakTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateVehicleEngineMaxRotationSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);
bool updateVehicleEngineIdleRotationSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);
bool updateVehicleEngineTorqueCurve(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    const pxr::TfToken&,
                                    const pxr::UsdTimeCode&);
bool updateVehicleEngineDampingRateFullThrottle(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId,
                                                const pxr::TfToken&,
                                                const pxr::UsdTimeCode&);
bool updateVehicleEngineDampingRateZeroThrottleClutchEngaged(omni::physx::usdparser::AttachedStage& attachedStage,
                                                             omni::physx::usdparser::ObjectId,
                                                             const pxr::TfToken&,
                                                             const pxr::UsdTimeCode&);
bool updateVehicleEngineDampingRateZeroThrottleClutchDisengaged(omni::physx::usdparser::AttachedStage& attachedStage,
                                                                omni::physx::usdparser::ObjectId,
                                                                const pxr::TfToken&,
                                                                const pxr::UsdTimeCode&);

bool updateVehicleTireFrictionTableFrictionValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId,
                                                  const pxr::TfToken&,
                                                  const pxr::UsdTimeCode&);
bool updateVehicleTireFrictionTableGroundMaterials(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const pxr::TfToken&,
                                                   const pxr::UsdTimeCode&);
bool updateVehicleTireFrictionTableDefaultFrictionValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                        omni::physx::usdparser::ObjectId,
                                                        const pxr::TfToken&,
                                                        const pxr::UsdTimeCode&);

bool updateVehicleSuspensionSpringStrength(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);
bool updateVehicleSuspensionSpringDamperRate(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const pxr::TfToken&,
                                             const pxr::UsdTimeCode&);
bool updateVehicleSuspensionMaxCompression(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);
bool updateVehicleSuspensionMaxDroop(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     const pxr::TfToken&,
                                     const pxr::UsdTimeCode&);
bool updateVehicleSuspensionTravelDistance(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);
bool updateVehicleSuspensionSprungMass(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updateVehicleSuspensionCamberAtRest(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);
bool updateVehicleSuspensionCamberAtMaxCompression(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const pxr::TfToken&,
                                                   const pxr::UsdTimeCode&);
bool updateVehicleSuspensionCamberAtMaxDroop(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const pxr::TfToken&,
                                             const pxr::UsdTimeCode&);

bool updateVehicleTireLatStiffX(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateVehicleTireLatStiffY(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateVehicleTireLateralStiffnessGraph(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);
bool updateVehicleTireLongStiffPerGrav(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updateVehicleTireLongitudinalStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);
bool updateVehicleTireCamberStiffPerGrav(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);
bool updateVehicleTireCamberStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateVehicleTireFrictionVsSlip(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     const pxr::TfToken&,
                                     const pxr::UsdTimeCode&);
bool updateVehicleTireFrictionTableRel(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updateVehicleTireRestLoad(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId,
                               const pxr::TfToken&,
                               const pxr::UsdTimeCode&);

bool updateVehicleWheelRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateVehicleWheelWidth(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId,
                             const pxr::TfToken&,
                             const pxr::UsdTimeCode&);
bool updateVehicleWheelMass(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId,
                            const pxr::TfToken&,
                            const pxr::UsdTimeCode&);
bool updateVehicleWheelMomentOfInertia(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updateVehicleWheelDampingRate(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateVehicleWheelMaxBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateVehicleWheelMaxHandBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);
bool updateVehicleWheelMaxSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     const pxr::TfToken&,
                                     const pxr::UsdTimeCode&);
bool updateVehicleWheelToeAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);

bool updateVehicleWheelAttachmentIndex(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentWheel(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentTire(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspension(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspensionTravelDirection(omni::physx::usdparser::AttachedStage& attachedStage,
                                                           omni::physx::usdparser::ObjectId,
                                                           const pxr::TfToken&,
                                                           const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspensionForceAppPointOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                                               omni::physx::usdparser::ObjectId,
                                                               const pxr::TfToken&,
                                                               const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentWheelCenterOfMassOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         const pxr::TfToken&,
                                                         const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentTireForceAppPointOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         const pxr::TfToken&,
                                                         const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspensionFramePosition(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         const pxr::TfToken&,
                                                         const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentSuspensionFrameOrientation(omni::physx::usdparser::AttachedStage& attachedStage,
                                                            omni::physx::usdparser::ObjectId,
                                                            const pxr::TfToken&,
                                                            const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentWheelFramePosition(omni::physx::usdparser::AttachedStage& attachedStage,
                                                    omni::physx::usdparser::ObjectId,
                                                    const pxr::TfToken&,
                                                    const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentWheelFrameOrientation(omni::physx::usdparser::AttachedStage& attachedStage,
                                                       omni::physx::usdparser::ObjectId,
                                                       const pxr::TfToken&,
                                                       const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentDriven(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateVehicleWheelAttachmentCollisionGroup(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId,
                                                const pxr::TfToken&,
                                                const pxr::UsdTimeCode&);

bool updateVehicleSuspensionComplWheelToeAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                               omni::physx::usdparser::ObjectId,
                                               const pxr::TfToken&,
                                               const pxr::UsdTimeCode&);
bool updateVehicleSuspensionComplWheelCamberAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId,
                                                  const pxr::TfToken&,
                                                  const pxr::UsdTimeCode&);
bool updateVehicleSuspensionComplSuspForceAppPoint(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const pxr::TfToken&,
                                                   const pxr::UsdTimeCode&);
bool updateVehicleSuspensionComplTireForceAppPoint(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const pxr::TfToken&,
                                                   const pxr::UsdTimeCode&);

bool updateVehicleEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId,
                          const pxr::TfToken&,
                          const pxr::UsdTimeCode&);
bool updateVehicleLimitSuspensionExpansionVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   const pxr::TfToken&,
                                                   const pxr::UsdTimeCode&);
bool updateVehicleMinPassiveLongslipDenom(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);
bool updateVehicleMinActiveLongslipDenom(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);
bool updateVehicleMinLateralSlipDenom(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateVehicleLongitudinalStickyTireThresholdSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                                       omni::physx::usdparser::ObjectId,
                                                       const pxr::TfToken&,
                                                       const pxr::UsdTimeCode&);
bool updateVehicleLongitudinalStickyTireThresholdTime(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId,
                                                      const pxr::TfToken&,
                                                      const pxr::UsdTimeCode&);
bool updateVehicleLongitudinalStickyTireDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId,
                                                const pxr::TfToken&,
                                                const pxr::UsdTimeCode&);
bool updateVehicleLateralStickyTireThresholdSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId,
                                                  const pxr::TfToken&,
                                                  const pxr::UsdTimeCode&);
bool updateVehicleLateralStickyTireThresholdTime(omni::physx::usdparser::AttachedStage& attachedStage,
                                                 omni::physx::usdparser::ObjectId,
                                                 const pxr::TfToken&,
                                                 const pxr::UsdTimeCode&);
bool updateVehicleLateralStickyTireDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);

bool updateVehicleControllerAccelerator(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateVehicleControllerBrake0(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateVehicleControllerBrake1(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateVehicleControllerBrake(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  const pxr::TfToken&,
                                  const pxr::UsdTimeCode&);
bool updateVehicleControllerHandbrake(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateVehicleControllerSteer(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  const pxr::TfToken&,
                                  const pxr::UsdTimeCode&);
bool updateVehicleControllerSteerLeft(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateVehicleControllerSteerRight(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updateVehicleControllerTargetGear(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);

bool updateVehicleTankControllerThrust0(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateVehicleTankControllerThrust1(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);

bool updateVehicleDriveBasicPeakTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);

bool updateVehicleWheelControllerDriveTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const pxr::TfToken&,
                                             const pxr::UsdTimeCode&);
bool updateVehicleWheelControllerBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const pxr::TfToken&,
                                             const pxr::UsdTimeCode&);
bool updateVehicleWheelControllerSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);

bool updateVehicleMultiWheelDifferentialWheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                               omni::physx::usdparser::ObjectId,
                                               const pxr::TfToken&,
                                               const pxr::UsdTimeCode&);
bool updateVehicleMultiWheelDifferentialTorqueRatios(omni::physx::usdparser::AttachedStage& attachedStage,
                                                     omni::physx::usdparser::ObjectId,
                                                     const pxr::TfToken&,
                                                     const pxr::UsdTimeCode&);
bool updateVehicleMultiWheelDifferentialAverageWheelSpeedRatios(omni::physx::usdparser::AttachedStage& attachedStage,
                                                                omni::physx::usdparser::ObjectId,
                                                                const pxr::TfToken&,
                                                                const pxr::UsdTimeCode&);

bool updateVehicleTankDifferentialNumberOfWheelsPerTrack(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         const pxr::TfToken&,
                                                         const pxr::UsdTimeCode&);
bool updateVehicleTankDifferentialThrustIndexPerTrack(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId,
                                                      const pxr::TfToken&,
                                                      const pxr::UsdTimeCode&);
bool updateVehicleTankDifferentialTrackToWheelIndices(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId,
                                                      const pxr::TfToken&,
                                                      const pxr::UsdTimeCode&);
bool updateVehicleTankDifferentialWheelIndicesInTrackOrder(omni::physx::usdparser::AttachedStage& attachedStage,
                                                           omni::physx::usdparser::ObjectId,
                                                           const pxr::TfToken&,
                                                           const pxr::UsdTimeCode&);

bool updateVehicleBrakes0Wheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateVehicleBrakes1Wheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateVehicleBrakes0MaxBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateVehicleBrakes1MaxBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateVehicleBrakes0TorqueMultipliers(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);
bool updateVehicleBrakes1TorqueMultipliers(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);

bool updateVehicleSteeringWheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 const pxr::TfToken&,
                                 const pxr::UsdTimeCode&);
bool updateVehicleSteeringMaxSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateVehicleSteeringAngleMultipliers(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);

bool updateVehicleAckermannSteeringWheel0(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);
bool updateVehicleAckermannSteeringWheel1(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);
bool updateVehicleAckermannSteeringMaxSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                                 omni::physx::usdparser::ObjectId,
                                                 const pxr::TfToken&,
                                                 const pxr::UsdTimeCode&);
bool updateVehicleAckermannSteeringWheelBase(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             const pxr::TfToken&,
                                             const pxr::UsdTimeCode&);
bool updateVehicleAckermannSteeringTrackWidth(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId,
                                              const pxr::TfToken&,
                                              const pxr::UsdTimeCode&);
bool updateVehicleAckermannSteeringStrength(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);

bool updateVehicleNCRDriveCommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateVehicleNCRSteerCommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateVehicleNCRBrakes0CommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);
bool updateVehicleNCRBrakes1CommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);

bool updateVehicleNCRDriveSpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                        omni::physx::usdparser::ObjectId,
                                                        const pxr::TfToken&,
                                                        const pxr::UsdTimeCode&);
bool updateVehicleNCRSteerSpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                        omni::physx::usdparser::ObjectId,
                                                        const pxr::TfToken&,
                                                        const pxr::UsdTimeCode&);
bool updateVehicleNCRBrakes0SpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                          omni::physx::usdparser::ObjectId,
                                                          const pxr::TfToken&,
                                                          const pxr::UsdTimeCode&);
bool updateVehicleNCRBrakes1SpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                          omni::physx::usdparser::ObjectId,
                                                          const pxr::TfToken&,
                                                          const pxr::UsdTimeCode&);

bool updateVehicleNCRDriveSpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);
bool updateVehicleNCRSteerSpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);
bool updateVehicleNCRBrakes0SpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);
bool updateVehicleNCRBrakes1SpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           const pxr::TfToken&,
                                           const pxr::UsdTimeCode&);

// spatial tendons
bool updateSpatialTendonStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  const pxr::TfToken&,
                                  const pxr::UsdTimeCode&);
bool updateSpatialTendonDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateSpatialTendonLimitStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       const pxr::TfToken&,
                                       const pxr::UsdTimeCode&);
bool updateSpatialTendonOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId,
                               const pxr::TfToken&,
                               const pxr::UsdTimeCode&);
bool updateSpatialTendonEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateTendonAttachmentGearing(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateTendonAttachmentLocalPos(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    const pxr::TfToken&,
                                    const pxr::UsdTimeCode&);
bool updateTendonAttachmentLeafRestLength(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          const pxr::TfToken&,
                                          const pxr::UsdTimeCode&);
bool updateTendonAttachmentLeafLowLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        const pxr::TfToken&,
                                        const pxr::UsdTimeCode&);
bool updateTendonAttachmentLeafHighLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         const pxr::TfToken&,
                                         const pxr::UsdTimeCode&);

// fixed tendons
bool updateFixedTendonStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateFixedTendonLimitStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     const pxr::TfToken&,
                                     const pxr::UsdTimeCode&);
bool updateFixedTendonDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateFixedTendonOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId,
                             const pxr::TfToken&,
                             const pxr::UsdTimeCode&);
bool updateFixedTendonRestLength(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 const pxr::TfToken&,
                                 const pxr::UsdTimeCode&);
bool updateFixedTendonLowLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId,
                               const pxr::TfToken&,
                               const pxr::UsdTimeCode&);
bool updateFixedTendonHighLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateFixedTendonEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId,
                              const pxr::TfToken&,
                              const pxr::UsdTimeCode&);
bool updateTendonAxisSingleGearing(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateTendonAxisSingleForceCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);

// DEPRECATED deformables
bool updateDeformableBodyMaterialDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            const pxr::TfToken&,
                                            const pxr::UsdTimeCode&);
bool updateDeformableBodyDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    const pxr::TfToken& property,
                                    const pxr::UsdTimeCode& timeCode);
bool warnDeformableBodyNoRuntimeCookingDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId objectId,
                                                  const pxr::TfToken& property,
                                                  const pxr::UsdTimeCode& timeCode);
bool updateDeformableSurfaceMaterialDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                               omni::physx::usdparser::ObjectId,
                                               const pxr::TfToken&,
                                               const pxr::UsdTimeCode&);
bool updateDeformableSurfaceDeprecated(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       const pxr::TfToken& property,
                                       const pxr::UsdTimeCode& timeCode);

// deformables
bool updateDeformableBody(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId,
                          const pxr::TfToken&,
                          const pxr::UsdTimeCode&);
bool updateDeformableContactOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   const pxr::TfToken&,
                                   const pxr::UsdTimeCode&);
bool updateDeformableRestOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                const pxr::TfToken&,
                                const pxr::UsdTimeCode&);
bool updateDeformableMaterial(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              const pxr::TfToken& property,
                              const pxr::UsdTimeCode&);

// mimic joints
bool updateMimicJointGearing(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId,
                             const pxr::TfToken&,
                             const pxr::UsdTimeCode&);
bool updateMimicJointOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId,
                            const pxr::TfToken&,
                            const pxr::UsdTimeCode&);
bool updateMimicJointNaturalFrequency(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      const pxr::TfToken&,
                                      const pxr::UsdTimeCode&);
bool updateMimicJointDampingRatio(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  const pxr::TfToken&,
                                  const pxr::UsdTimeCode&);

// note: the other mimic joint properties trigger a structural change, so a reparse, thus no change methods for those

} // namespace physx
} // namespace omni
