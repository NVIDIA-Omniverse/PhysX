// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CORE-003
 * @covers AC-2
 */

#pragma once

#include <omni/physics/parse/IPhysicsSource.h>

#include <internal/Internal.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
}

// Did the user actually author one of the PhysxCollisionAPI offsets
// (physxCollision:contactOffset / :restOffset), as opposed to inheriting the
// schema fallback? The schema default for both is NEGATIVE inf, an "unset"
// sentinel, so the resolved value answers this without a separate authored bit.
//
// Use this rather than hasAuthoredAttribute in the Newton fallback handlers: a
// resolved-value backend answers hasAuthoredAttribute `true` for everything it
// publishes (ADR-0020), which made every Newton contactMargin/contactGap
// fallback silently inert on ovstage. Defined in PhysXCollisionPropertiesUpdate.cpp.
bool isPhysxOffsetAuthored(const omni::physx::usdparser::AttachedStage& attachedStage,
                           const omni::physics::parse::ObjectKey& key,
                           omni::physics::parse::TokenId attr);

// scene
bool updateGravityMagnitude(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateGravityDirection(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateTimeStepsPerSecond(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateSceneUpdateType(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);
bool updateQuasistaticEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateQuasistaticCollection(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);

// body
bool updateBodyEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       omni::physics::parse::TokenId,
                       omni::physics::parse::ReadTime);
bool updateBodyDensity(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       omni::physics::parse::TokenId,
                       omni::physics::parse::ReadTime);
bool updateBodyLinearVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyAngularVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateBodyLinearDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateBodyAngularDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyMaxLinearVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateBodyMaxAngularVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId objectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateBodySleepThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyStabilizationThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateBodyMaxDepenetrationVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateBodyContactSlopCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId property,
                                      omni::physics::parse::ReadTime);
bool updateBodyMaxContactImpulse(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId objectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateBodySolverPositionIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateBodySolverVelocityIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateBodyEnableKinematics(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateBodyEnableCCD(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateBodyEnableSpeculativeCCD(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateBodyRetainAccelerations(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateBodyGyroscopicForces(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateBodyDisableGravity(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyLockedPosAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateBodyLockedRotAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateBodyTransformStack(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyCfmScale(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateBodySolveContacts(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateBodySimulationOwner(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);

bool updatePhysxContactReportThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);

bool updateBodyInstancedPositions(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId objectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateBodyInstancedOrientations(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateBodyInstancedVelocities(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateBodyInstancedAngularVelocities(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId objectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);

bool updateBodySurfaceVelocityEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateBodySurfaceLinearVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateBodySurfaceAngularVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateBodySurfaceVelocityLocalSpace(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId objectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateBodySplineSurfaceVelocityMagnitude(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId objectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateBodySplineSurfaceVelocityEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              omni::physics::parse::TokenId,
                                              omni::physics::parse::ReadTime);

    // force
bool updatePhysxForceEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updatePhysxForceWorldFrameEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updatePhysxForce(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      omni::physics::parse::TokenId,
                      omni::physics::parse::ReadTime);
bool updatePhysxTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       omni::physics::parse::TokenId,
                       omni::physics::parse::ReadTime);
bool updatePhysxForceMode(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);

// shape
bool updateShapeEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateShapeDensity(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateShapeContactOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateShapeRestOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);
bool updateShapeTorsionalPatchRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateShapeMinTorsionalPatchRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);

// material
bool updateMaterialDynamicFriction(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateMaterialStaticFriction(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId objectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateMaterialRestitution(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateMaterialFrictionCombineMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateMaterialRestitutionCombineMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId objectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateMaterialDampingCombineMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateCompliantMaterial(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId property,
                             omni::physics::parse::ReadTime);

// joint
bool updateDriveTargetPosition(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateDriveTargetVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateDriveMaxForce(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateDriveMaxActuatorVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateDriveVelocityDependentResistance(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateDriveSpeedEffortGradient(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateDriveDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateDriveStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);
bool updateDriveType(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLimitHigh(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLimitLow(omni::physx::usdparser::AttachedStage& attachedStage,
                    omni::physx::usdparser::ObjectId objectId,
                    omni::physics::parse::TokenId,
                    omni::physics::parse::ReadTime);
bool updateJointStatePosition(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateJointStateVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateEnableCollision(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);
bool updateBreakForce(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      omni::physics::parse::TokenId,
                      omni::physics::parse::ReadTime);
bool updateBreakTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                       omni::physx::usdparser::ObjectId objectId,
                       omni::physics::parse::TokenId,
                       omni::physics::parse::ReadTime);
bool updateLocalPos0(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLocalPos1(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLocalRot0(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLocalRot1(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateGearRatio(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateGearHinge0(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      omni::physics::parse::TokenId,
                      omni::physics::parse::ReadTime);
bool updateGearHinge1(omni::physx::usdparser::AttachedStage& attachedStage,
                      omni::physx::usdparser::ObjectId objectId,
                      omni::physics::parse::TokenId,
                      omni::physics::parse::ReadTime);
bool updateRackPinionRatio(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);
bool updateRackHinge(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateRackPrismatic(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateArmature(omni::physx::usdparser::AttachedStage& attachedStage,
                    omni::physx::usdparser::ObjectId objectId,
                    omni::physics::parse::TokenId,
                    omni::physics::parse::ReadTime);
bool updateArmaturePerAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateLimitBounceThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateLimitDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                        omni::physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateLimitRestitution(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateLimitStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId objectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);
bool updateDistanceJointSpringDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateDistanceJointSpringStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateDistanceJointSpringEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);

// articulation
bool updateArticulationFixBase(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateArticulationSolverPositionIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                                    omni::physx::usdparser::ObjectId objectId,
                                                    omni::physics::parse::TokenId,
                                                    omni::physics::parse::ReadTime);
bool updateArticulationSolverVelocityIterationCount(omni::physx::usdparser::AttachedStage& attachedStage,
                                                    omni::physx::usdparser::ObjectId objectId,
                                                    omni::physics::parse::TokenId,
                                                    omni::physics::parse::ReadTime);
bool updateArticulationSleepThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateArticulationStabilizationThreshold(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              omni::physics::parse::TokenId,
                                              omni::physics::parse::ReadTime);

// articulation link
bool updateArticulationMaxJointVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId objectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateArticulationMaxJointVelocityPerAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateArticulationFrictionCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId objectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateArticulationStaticFrictionEffort(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateArticulationDynamicFrictionEffort(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId objectId,
                                                omni::physics::parse::TokenId,
                                                omni::physics::parse::ReadTime);
bool updateArticulationViscousFrictionCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);

// collision group
inline bool updateCollisionGroup(omni::physx::usdparser::AttachedStage& /*attachedStage*/,
                                 omni::physx::usdparser::ObjectId /*objectId*/,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime)
{
    return true;
}

// filtered pairs
bool updateFilteredPairs(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);

// cct
bool updateCctSlopeLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateCctHeight(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateCctRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateCctContactOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateCctStepOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                         omni::physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateCctUpAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                     omni::physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateCctNonWalkableMode(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateCctClimbingMode(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);

// particle system
bool updateParticleSystemAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updatePBDMaterialAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateParticleDensity(omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);

bool updateParticleSmoothingEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId objectId,
                                             omni::physics::parse::TokenId property,
                                             omni::physics::parse::ReadTime);
bool updateParticleAnisotropyEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              omni::physics::parse::TokenId property,
                                              omni::physics::parse::ReadTime);
bool updateParticleIsosurfaceEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId objectId,
                                              omni::physics::parse::TokenId property,
                                              omni::physics::parse::ReadTime);
bool updateParticleIsosurfaceAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId objectId,
                                       omni::physics::parse::TokenId property,
                                       omni::physics::parse::ReadTime);

// partice set
bool updateParticleSetEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateParticleSetSelfCollision(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateParticleSetFluid(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateParticleSetParticleGroup(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId objectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);

bool updateParticlePositions(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateParticleSimPositions(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateParticleVelocities(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);

bool updateDiffuseParticlesEnabledAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId property,
                                            omni::physics::parse::ReadTime);
bool updateDiffuseParticlesAttribute(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId objectId,
                                     omni::physics::parse::TokenId property,
                                     omni::physics::parse::ReadTime);

// vehicle
bool updateVehicleContextUpdateMode(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateVehicleContextVerticalAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleContextLongitudinalAxis(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);

bool updateVehicleEngineMomentOfInertia(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleEnginePeakTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateVehicleEngineMaxRotationSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleEngineIdleRotationSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleEngineTorqueCurve(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateVehicleEngineDampingRateFullThrottle(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId,
                                                omni::physics::parse::TokenId,
                                                omni::physics::parse::ReadTime);
bool updateVehicleEngineDampingRateZeroThrottleClutchEngaged(omni::physx::usdparser::AttachedStage& attachedStage,
                                                             omni::physx::usdparser::ObjectId,
                                                             omni::physics::parse::TokenId,
                                                             omni::physics::parse::ReadTime);
bool updateVehicleEngineDampingRateZeroThrottleClutchDisengaged(omni::physx::usdparser::AttachedStage& attachedStage,
                                                                omni::physx::usdparser::ObjectId,
                                                                omni::physics::parse::TokenId,
                                                                omni::physics::parse::ReadTime);

bool updateVehicleTireFrictionTableFrictionValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId,
                                                  omni::physics::parse::TokenId,
                                                  omni::physics::parse::ReadTime);
bool updateVehicleTireFrictionTableGroundMaterials(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);
bool updateVehicleTireFrictionTableDefaultFrictionValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                        omni::physx::usdparser::ObjectId,
                                                        omni::physics::parse::TokenId,
                                                        omni::physics::parse::ReadTime);

bool updateVehicleSuspensionSpringStrength(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleSuspensionSpringDamperRate(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);
bool updateVehicleSuspensionMaxCompression(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleSuspensionMaxDroop(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateVehicleSuspensionTravelDistance(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleSuspensionSprungMass(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleSuspensionCamberAtRest(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleSuspensionCamberAtMaxCompression(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);
bool updateVehicleSuspensionCamberAtMaxDroop(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);

bool updateVehicleTireLatStiffX(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateVehicleTireLatStiffY(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateVehicleTireLateralStiffnessGraph(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateVehicleTireLongStiffPerGrav(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleTireLongitudinalStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateVehicleTireCamberStiffPerGrav(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleTireCamberStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleTireFrictionVsSlip(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateVehicleTireFrictionTableRel(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleTireRestLoad(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);

bool updateVehicleWheelRadius(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateVehicleWheelWidth(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateVehicleWheelMass(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateVehicleWheelMomentOfInertia(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleWheelDampingRate(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateVehicleWheelMaxBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleWheelMaxHandBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleWheelMaxSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateVehicleWheelToeAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);

bool updateVehicleWheelAttachmentIndex(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentWheel(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentTire(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspension(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspensionTravelDirection(omni::physx::usdparser::AttachedStage& attachedStage,
                                                           omni::physx::usdparser::ObjectId,
                                                           omni::physics::parse::TokenId,
                                                           omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspensionForceAppPointOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                                               omni::physx::usdparser::ObjectId,
                                                               omni::physics::parse::TokenId,
                                                               omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentWheelCenterOfMassOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         omni::physics::parse::TokenId,
                                                         omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentTireForceAppPointOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         omni::physics::parse::TokenId,
                                                         omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspensionFramePosition(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         omni::physics::parse::TokenId,
                                                         omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspensionFrameOrientation(omni::physx::usdparser::AttachedStage& attachedStage,
                                                            omni::physx::usdparser::ObjectId,
                                                            omni::physics::parse::TokenId,
                                                            omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentWheelFramePosition(omni::physx::usdparser::AttachedStage& attachedStage,
                                                    omni::physx::usdparser::ObjectId,
                                                    omni::physics::parse::TokenId,
                                                    omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentWheelFrameOrientation(omni::physx::usdparser::AttachedStage& attachedStage,
                                                       omni::physx::usdparser::ObjectId,
                                                       omni::physics::parse::TokenId,
                                                       omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentDriven(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentCollisionGroup(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId,
                                                omni::physics::parse::TokenId,
                                                omni::physics::parse::ReadTime);

bool updateVehicleSuspensionComplWheelToeAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                               omni::physx::usdparser::ObjectId,
                                               omni::physics::parse::TokenId,
                                               omni::physics::parse::ReadTime);
bool updateVehicleSuspensionComplWheelCamberAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId,
                                                  omni::physics::parse::TokenId,
                                                  omni::physics::parse::ReadTime);
bool updateVehicleSuspensionComplSuspForceAppPoint(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);
bool updateVehicleSuspensionComplTireForceAppPoint(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);

bool updateVehicleEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);
bool updateVehicleLimitSuspensionExpansionVelocity(omni::physx::usdparser::AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);
bool updateVehicleMinPassiveLongslipDenom(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleMinActiveLongslipDenom(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleMinLateralSlipDenom(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleLongitudinalStickyTireThresholdSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                                       omni::physx::usdparser::ObjectId,
                                                       omni::physics::parse::TokenId,
                                                       omni::physics::parse::ReadTime);
bool updateVehicleLongitudinalStickyTireThresholdTime(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId,
                                                      omni::physics::parse::TokenId,
                                                      omni::physics::parse::ReadTime);
bool updateVehicleLongitudinalStickyTireDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                                omni::physx::usdparser::ObjectId,
                                                omni::physics::parse::TokenId,
                                                omni::physics::parse::ReadTime);
bool updateVehicleLateralStickyTireThresholdSpeed(omni::physx::usdparser::AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId,
                                                  omni::physics::parse::TokenId,
                                                  omni::physics::parse::ReadTime);
bool updateVehicleLateralStickyTireThresholdTime(omni::physx::usdparser::AttachedStage& attachedStage,
                                                 omni::physx::usdparser::ObjectId,
                                                 omni::physics::parse::TokenId,
                                                 omni::physics::parse::ReadTime);
bool updateVehicleLateralStickyTireDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);

bool updateVehicleControllerAccelerator(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleControllerBrake0(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateVehicleControllerBrake1(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateVehicleControllerBrake(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateVehicleControllerHandbrake(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleControllerSteer(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateVehicleControllerSteerLeft(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleControllerSteerRight(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleControllerTargetGear(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);

bool updateVehicleTankControllerThrust0(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleTankControllerThrust1(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);

bool updateVehicleDriveBasicPeakTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);

bool updateVehicleWheelControllerDriveTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);
bool updateVehicleWheelControllerBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);
bool updateVehicleWheelControllerSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);

bool updateVehicleMultiWheelDifferentialWheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                               omni::physx::usdparser::ObjectId,
                                               omni::physics::parse::TokenId,
                                               omni::physics::parse::ReadTime);
bool updateVehicleMultiWheelDifferentialTorqueRatios(omni::physx::usdparser::AttachedStage& attachedStage,
                                                     omni::physx::usdparser::ObjectId,
                                                     omni::physics::parse::TokenId,
                                                     omni::physics::parse::ReadTime);
bool updateVehicleMultiWheelDifferentialAverageWheelSpeedRatios(omni::physx::usdparser::AttachedStage& attachedStage,
                                                                omni::physx::usdparser::ObjectId,
                                                                omni::physics::parse::TokenId,
                                                                omni::physics::parse::ReadTime);

bool updateVehicleTankDifferentialNumberOfWheelsPerTrack(omni::physx::usdparser::AttachedStage& attachedStage,
                                                         omni::physx::usdparser::ObjectId,
                                                         omni::physics::parse::TokenId,
                                                         omni::physics::parse::ReadTime);
bool updateVehicleTankDifferentialThrustIndexPerTrack(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId,
                                                      omni::physics::parse::TokenId,
                                                      omni::physics::parse::ReadTime);
bool updateVehicleTankDifferentialTrackToWheelIndices(omni::physx::usdparser::AttachedStage& attachedStage,
                                                      omni::physx::usdparser::ObjectId,
                                                      omni::physics::parse::TokenId,
                                                      omni::physics::parse::ReadTime);
bool updateVehicleTankDifferentialWheelIndicesInTrackOrder(omni::physx::usdparser::AttachedStage& attachedStage,
                                                           omni::physx::usdparser::ObjectId,
                                                           omni::physics::parse::TokenId,
                                                           omni::physics::parse::ReadTime);

bool updateVehicleBrakes0Wheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateVehicleBrakes1Wheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateVehicleBrakes0MaxBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleBrakes1MaxBrakeTorque(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleBrakes0TorqueMultipliers(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleBrakes1TorqueMultipliers(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);

bool updateVehicleSteeringWheels(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateVehicleSteeringMaxSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleSteeringAngleMultipliers(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);

bool updateVehicleAckermannSteeringWheel0(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringWheel1(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringMaxSteerAngle(omni::physx::usdparser::AttachedStage& attachedStage,
                                                 omni::physx::usdparser::ObjectId,
                                                 omni::physics::parse::TokenId,
                                                 omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringWheelBase(omni::physx::usdparser::AttachedStage& attachedStage,
                                             omni::physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringTrackWidth(omni::physx::usdparser::AttachedStage& attachedStage,
                                              omni::physx::usdparser::ObjectId,
                                              omni::physics::parse::TokenId,
                                              omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringStrength(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);

bool updateVehicleNCRDriveCommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleNCRSteerCommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes0CommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes1CommandValues(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);

bool updateVehicleNCRDriveSpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                        omni::physx::usdparser::ObjectId,
                                                        omni::physics::parse::TokenId,
                                                        omni::physics::parse::ReadTime);
bool updateVehicleNCRSteerSpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                        omni::physx::usdparser::ObjectId,
                                                        omni::physics::parse::TokenId,
                                                        omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes0SpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                          omni::physx::usdparser::ObjectId,
                                                          omni::physics::parse::TokenId,
                                                          omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes1SpeedResponsesPerCommandValue(omni::physx::usdparser::AttachedStage& attachedStage,
                                                          omni::physx::usdparser::ObjectId,
                                                          omni::physics::parse::TokenId,
                                                          omni::physics::parse::ReadTime);

bool updateVehicleNCRDriveSpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleNCRSteerSpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes0SpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes1SpeedResponses(omni::physx::usdparser::AttachedStage& attachedStage,
                                           omni::physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);

// spatial tendons
bool updateSpatialTendonStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateSpatialTendonDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateSpatialTendonLimitStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                       omni::physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateSpatialTendonOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateSpatialTendonEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateTendonAttachmentGearing(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateTendonAttachmentLocalPos(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateTendonAttachmentLeafRestLength(omni::physx::usdparser::AttachedStage& attachedStage,
                                          omni::physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateTendonAttachmentLeafLowLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                        omni::physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateTendonAttachmentLeafHighLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);

// fixed tendons
bool updateFixedTendonStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateFixedTendonLimitStiffness(omni::physx::usdparser::AttachedStage& attachedStage,
                                     omni::physx::usdparser::ObjectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateFixedTendonDamping(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateFixedTendonOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateFixedTendonRestLength(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateFixedTendonLowLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                               omni::physx::usdparser::ObjectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateFixedTendonHighLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateFixedTendonEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateTendonAxisSingleGearing(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateTendonAxisSingleForceCoefficient(omni::physx::usdparser::AttachedStage& attachedStage,
                                            omni::physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);

// deformables
bool updateDeformableBody(omni::physx::usdparser::AttachedStage& attachedStage,
                          omni::physx::usdparser::ObjectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);
bool updateDeformableContactOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                   omni::physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateDeformableRestOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateDeformableMaterial(omni::physx::usdparser::AttachedStage& attachedStage,
                              omni::physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId property,
                              omni::physics::parse::ReadTime);

// mimic joints
bool updateMimicJointGearing(omni::physx::usdparser::AttachedStage& attachedStage,
                             omni::physx::usdparser::ObjectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateMimicJointOffset(omni::physx::usdparser::AttachedStage& attachedStage,
                            omni::physx::usdparser::ObjectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateMimicJointNaturalFrequency(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateMimicJointDampingRatio(omni::physx::usdparser::AttachedStage& attachedStage,
                                  omni::physx::usdparser::ObjectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);

bool updateNewtonMimicJointCoef1(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateNewtonMimicJointCoef0(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);

// note: the other mimic joint properties trigger a structural change, so a reparse, thus no change methods for those

// Newton schema runtime change tracking. Each wrapper applies the Newton value only when the
// corresponding PhysX attribute is not authored on the prim (PhysX > Newton priority).
bool updateNewtonTimeStepsPerSecond(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateNewtonGravityEnabled(omni::physx::usdparser::AttachedStage& attachedStage,
                                omni::physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateNewtonShapeContactMargin(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateNewtonShapeContactGap(omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateNewtonDeformableContactMargin(omni::physx::usdparser::AttachedStage& attachedStage,
                                         omni::physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateNewtonDeformableContactGap(omni::physx::usdparser::AttachedStage& attachedStage,
                                      omni::physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateNewtonJointVelocityLimit(omni::physx::usdparser::AttachedStage& attachedStage,
                                    omni::physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);

} // namespace physx
} // namespace omni
