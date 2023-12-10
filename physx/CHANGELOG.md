# v5.3.1-105.1

## General

### Changed

* PxgDynamicsMemoryConfig::tempBufferCapacity will now be interpreted as a user-provided initial size, and will resize automatically if more memory is needed.

### Fixed

* A bug that led to phantom collisions for convex-heightfield interactions on GPU has been fixed.
* A bug that caused velocity and impulse updates of the GPU articulation solver (PGS and TGS) not to be propagated to subsequent iterations, causing slower convergence and potentially unstable collision responses between rigids and articulations.
* Fixed binary serialization for GPU enabled triangle meshes and meshes with SDF support.
* Several bugs with GPU aggregates have been fixed that could have led to missed and phantom collisions. The issues were mostly related to additions/removals of aggregate contents.
* Gpu accelerated SDF cooking is now deterministic.
* SDF snippet shows how to optionally store cooked data into files.
* Small improvements to SDF collisions, especially when objects with wildly different size collide.
* Creating objects from a PxInputData containing invalid data could lead to a confusing (and incorrect) error message about a "double deletion". This has been fixed.
* Bugs in island management related to actors other than rigid bodies.
* A bug that could lead to a crash when calling the PxTetMaker::validateTriangleMesh function with a mesh referencing more vertices than passed into the function. That defect is now reported as eTRIANGLE_INDEX_OUT_OF_RANGE. 
* A crash bug that appeared when releasing actors with externally-provided forces and torques has been fixed. [Issue #211](https://github.com/NVIDIA-Omniverse/PhysX/issues/211)
* A bug that caused a memory corruption in the GPU solver when using D6 joints with rigid bodies and articulations has been fixed.

## Rigid Body

### Added

* The extraction of an isosurface from a SDF can now use multiple CPU cores.

### Fixed

* A crash happening when using contact report thresholds with point-friction (PxFrictionType::eONE_DIRECTIONAL / PxFrictionType::eTWO_DIRECTIONAL) has been fixed.
* A "fear of the wireframe" issue in Sphere vs TriangleMesh collision when simulating on GPU is fixed.

## Articulations

### Fixed

* Articulation joint velocity limits are respected when articulation joint drives are configured to push past the limit.
* Spherical articulation joints could sometimes flip their position by 2 pi causing problems with joint limits. This has been fixed.

## Joints

### Fixed

* The PxConstraintFlag::eENABLE_EXTENDED_LIMITS flag now works properly for D6 based revolute joints when the GPU pipeline with the TGS solver is active.

## Character controller

### Fixed

* You can now only create one PxCreateControllerManager per PxScene. This avoids filtering-related issues when multiple controller managers are created for the same PxScene.

## Particles

### Added

* PxParticleSystem::getParticleMaterials() to query materials that have been registered with phases.

### Fixed

* PxParticleSystem::getNbParticleMaterials() always returned 1, instead of the materials referenced by phases.
* Particle - Convex Shape collisions failing with spread out particles.
* Particle phase references to PxPBDMaterial were broken when releasing (an unreferenced) PxPBDMaterial.

## Pvd

### Added

* A way to get a thread safe OmniPvd writer from the PxOmniPvd interface through using acquireExclusiveWriterAccess() and releaseExclusiveWriterAccess().

### Fixed

* OmniPVD no longer breaks when running and recording multiple scenes in parallel.
* Corrected mirroring of the inbountJoinDOF attribute of PxArticulationLink

## Extensions

### Fixed

* A bug in custom cone/cylinder collision with triangle meshes. There was a gap between a cone/cylinder and a mesh, noticeable for centimeter-scale shapes. Note that the last position argument of e.g.: PxCustomGeometryExt::CylinderCallbacks::useSubstituteGeometry was removed from the API.


# v5.3.0-105.1

## Supported Platforms

### Runtime

* Linux (tested on Ubuntu 20.04)
* Microsoft Windows 10 or later (GPU acceleration: display driver and GPU supporting CUDA 11 / CUDA ARCH 3.0)

### Development

* Microsoft Windows 10 or later
* Microsoft Visual Studio 2017, 2019, 2022

## General

### Changed

* The method PxLineStripSkinning::evaluateInterpolatedVertices changed the transform argument from `PxReal*` to ` PxMat44*` to be more explicit about the underlying data that is expected.
* The apply* and copy* functions in PxScene changed their event arguments from `void*` to `CUevent` to fix misunderstandings about the type of those arguments. This also fixes a bug where pointers to events where passed but not dereferenced when recording/awaiting them.
* The TGS solver on CPU and GPU now computes the number of position and velocity iteration according to the requested numbers by the actors in each island, matching the behavior of the PGS solver. Previously TGS velocity iterations in excess of 4 were silently converted to position iterations. To preserve the old behavior any actor requesting more than 4 velocity iterations should convert excess velocity iteration counts to position iteration counts, e.g., formerly 10 position and 10 velocity iterations should become 16 position and 4 velocity iterations.
* The `acquire()` and `release()` functions in `PxCudaContextManager` that manage the PhysX CUDA context now use push/pop semantics. This fixes bug that led to a wrong context being bound after `release()` when sharing an existing CUDA context with PhysX.
* Calling `setCMassLocalPose()` on a rigid body when using the direct-GPU API is now allowed. Note that calling `updateArticulationsKinematic()` after updating CMassLocalPose but before the next call to `simulate()` will still use the old CMassLocalPose.

### Fixed

* A memory leak has been fixed in the actor pairs management code.
* A race condition was fixed that led to nondeterministic contact reports in some scenarios.
* Fix FEM cloth attachment filtering bug
* Fix FEM narrow phase collision crash
* Sphere-Trianglemesh collision bug is fixed
* A bug that led to aggregated shapes being processed as part of the regular broadphase when changing transforms using the direct-GPU API has been fixed.
* A bug that led to missed collisions and phantom collisions when changing transforms using the direct-GPU API has been fixed.
* A bug that led to incorrect and nondeterministic behaviour for convex-trianglemesh, convex-heightfield, sphere-trianglemesh, capsule-trianglemesh, sphere-heightfield and capsule-heightfield collisions on GPU has been fixed.
* A bug that led to contact target velocities spilling over from one contact to other contacts in the same solver batch.
* A bug that led to incorrect and nondeterministic behaviour for trianglemesh-trianglemesh collisions on GPU has been fixed.
* A bug that led to incorrect materials being used for convex-convex collisions on GPU has been fixed.

### Removed

* Context creation for CUDA/Graphics interoperability has been deprecated. interopMode has been removed from PxCudaContextManagerDesc.
* PxSceneFlag::eFORCE_READBACK has been removed. There is no replacement.
* PxSceneFlag::eSUPPRESS_READBACK was deprecated and has been removed. Use PxSceneFlag::eENABLE_DIRECT_GPU_API instead.

## Rigid Body

### Added

* Possibility to use the GPU to cook an SDF making the process a lot faster.
* Option to launch CUDA kernels synchronously when creating the CUDA Context Manager. This option is required to accurately determine the correct kernel that returns a CUDA error.

### Fixed

* The torsional patch radius parameter (see PxShape::setTorsionalPatchRadius()) was potentially ignored when running the simulation on GPU.
* Potential race condition related to activating/deactivating trigger pairs.
* A small misalignment of SDFs with the triangle mesh.
* A small error in the gradient calculation of SDFs.
* A sphere could tunnel through the edge between two triangles in a triangle mesh.
* Race condition in SDF computation cuda kernel is fixed.
* Fixed invalid access problem when selecting the SDF contact handler.

### Deprecated

* PxFrictionType::eONE_DIRECTIONAL has been deprecated and will be removed in the future. Please use ePATCH or eTWO_DIRECTIONAL instead.

## Articulations

### Changed

* `PxScene::copyArticulationData()` and `PxScene::applyArticulationData()` do not allow reading write-only and writing read-only data anymore. Read/write properties are specified in the API doc of `PxArticulationGpuDataType`.

### Fixed

* A bug that led to wrong joint targets being set when using the direct-GPU API has been fixed.
* A bug that led to link constraint-force-mixing scale not being included in collision constraints when using GPU dynamics has been fixed.
* Articulation drive did not produce the same force magnitude for drives with velocity biases that were equally positive and negative.  This was true of the CPU and GPU solver pipelines. This has been fixed.
* Articulation drive produced unphysically large forces when run in combination with PxSolverType::eTGS and non-zero velocity iteration count. This was true of the CPU and GPU solver pipelines. This has been fixed by no longer updating joint drive force during velocity iterations with PxSolverType::eTGS. The expectation is that there are sufficient position iterations such that the drive force that accumulated over the position iterations is an accurate force. This avoids numerical discrepancies arising from the difference in effective simulation timestep employed by the position and velocity iterations.  This discrepancy was particularly acute with a large number of velocity iterations.
* Articulation drive suffered from an unphysical damping term with all combinations of PxSolverType::eTGS/PxSolverType::ePGS/PxSceneFlag::eENABLE_GPU_DYNAMICS.  This has been fixed.
* Potential crashes due to reading uninitialized memory were fixed.
* The function PxArticulationReducedCoordinate::setMaxCOMAngularVelocity() had no effect if called after the 1st sim step with PxSceneFlag::eENABLE_GPU_DYNAMICS raised. This has been fixed.
* The function PxArticulationReducedCoordinate::setMaxCOMLinearVelocity() had no effect if called after the 1st sim step with PxSceneFlag::eENABLE_GPU_DYNAMICS raised. This has been fixed.
* Raising or lowering PxArticulationFlag::eFIX_BASE had no effect if modified after the 1st sim step with PxSceneFlag::eENABLE_GPU_DYNAMICS raised. This has been fixed.
* The root link acceleration was reported as {0} even when the root link was not fixed. This affected GPU only.  The fix has been applied to PxArticulationReducedCoordinate::copyInternalStateToCache(),  PxArticulationReducedCoordinate::getLinkAcceleration() and PxScene::copyArticulationData().
* Only half the expected friction force was applied in certain scenarios when using PxSolverType::eTGS, PxFrictionType::ePATCH, PxMaterialFlag::eIMPROVED_PATCH_FRICTION and running on CPU.

### Deprecated 

* The functions PxArticulationReducedCoordinate::setMaxCOMLinearVelocity(), PxArticulationReducedCoordinate::getMaxCOMLinearVelocity(), PxArticulationReducedCoordinate::setMaxCOMAngularVelocity(), PxArticulationReducedCoordinate::getMaxCOMAngularVelocity() have all been marked as deprecated and will be removed in a future release.

## Joints

### Deprecated 

* Px1DConstraintFlag::eDRIVE_ROW has been marked as deprecated and will be removed in a later release.  It has been renamed to Px1DConstraintFlag::eDEPRECATED_DRIVE_ROW to signal the intention to remove this flag in a later release.

## Vehicles2

### Added

* A new snippet that shows an example of using a custom tire model has been added (see SnippetVehicle2CustomTire).

### Changed

* The snippet SnippetVehicle2Customization has been renamed to SnippetVehicle2CustomSuspension.
* PxVehicleCommandNonLinearResponseParams::nbSpeedRenponsesPerCommandValue was misspelled and now renamed to nbSpeedResponsesPerCommandValue.
* More parameters get recorded by OmniPVD. As a consequence, PxVehiclePVDComponent and some other PVD related vehicle APIs changed.
* It is now legal to set entries in PxVehicleTankDriveDifferentialParams::nbWheelsPerTrack to 0 or 1.
* The APIs of some methods use more specific input parameters now to decrease dependency on certain data structures. See the migration guide for more details. This applies to the methods: PxVehicleTireDirsUpdate(), PxVehicleTireCamberAnglesUpdate() and PxVehicleTireGripUpdate().

### Fixed

* Nonlinear command responses were broken for negative steer command values. Now they are treated symmetrically as intended.
* PxVehiclePhysXActorDestroy() triggered a warning if the articulation link was not a leaf link.

### Removed

* PxVehicleTankDriveDifferentialParams::nbWheelsInTracks has been removed. The entries in ::nbWheelsPerTrack can be summed up to compute that value instead.

## Cooking

### Added

* PxTriangleMeshCookingResult::eEMPTY_MESH has been added. This cooking result is output when the mesh cleaning process removes all the triangles of a mesh.
* PxCookingParams::meshAreaMinLimit has been added. This is used in the mesh cleaning process to remove triangles whose area is too small.
* PxCookingParams::meshEdgeLengthMaxLimit has been added.

### Changed

* The requirements for convex meshes being GPU compatible have been tightened. Overly oblong meshes are now rejected by the cooking with an error message. Collision
detection will fall back to CPU for these meshes.

### Fixed
* Fixed out of memory crash when cooking a convex hull of a very high resolution mesh.

## Soft Body

### Added

* Support for voxel meshes with 5 tetrahedra per voxel to counteract anisotropy in the mesh.

### Changed:

* Defaults of PxConeLimitedConstraint::mLowLimit, mHighLimit have been changed to -1.0 indicating no limit.
* Soft body sleep damping is improved to minimize an effect that looks like a soft body would lower its stiffness before it goes to sleep.

### Fixed
* Overflow of the soft body contact buffer will result in a warning.

## Extensions

### Added

* CCD support for PxCustomGeometryExt::CylinderCallbacks and PxCustomGeometryExt::ConeCallbacks.

### Changed

* PxCustomGeometryExt::CylinderCallbacks and PxCustomGeometryExt::ConeCallbacks classes have their public member variables (height, radius, axis and margin) replaced with setter and getter member functions.

## Pvd

### Fixed

* Better coverage in OVD of attribute mirroring for : PxActor, PxRigidActor, PxRigidBody, PxRigidStatic and PxRigidDynamic, specifically for initial values, user set functions and post simulation updates.

# v5.2.0 & v5.2.1

## Supported Platforms

### Runtime

* Linux (tested on Ubuntu 20.04)
* Microsoft Windows 10 or later (GPU acceleration: display driver and GPU supporting CUDA 11 / CUDA ARCH 3.0)

### Development

* Microsoft Windows 10 or later
* Microsoft Visual Studio 2017, 2019, 2022

## General

* PhysX GPU binaries built with CUDA toolkit 11.8.
	* Added compute capability 8.9 (Ada) and 9.0 (Hopper)
	* Removed compute capability 5.0 (Maxwell)
	* Enabled multi-threaded cuda kernel compilation
	
### Changed:

* INVALID_FILTER_PAIR_INDEX has been moved out of the public API. It was incorrectly exposed to users.
* The API for the filter callback changed slightly. The pairID parameter in PxSimulationFilterCallback::pairFound(), PxSimulationFilterCallback::pairLost() and PxSimulationFilterCallback::statusChange() is now a PxU64 instead of a PxU32. The filter callback will now be called from multiple threads.
* Minimum required Windows OS version was changed from Windows XP to Windows 7
* Replaced all calls to `select` with calls to `poll` in the socket implementations
* PxSceneFlag::eENABLE_DIRECT_GPU_API and its predecessor PxSceneFlag::eSUPPRESS_READBACK are now immutable.
* CmakeModules is no longer an external dependency. It's now included in PhysX source.

### Fixed

* A bug that produced duplicate broadphase pairs and led to rapidly increasing GPU memory consumption was fixed.
* An immediate mode bug has been fixed. It was happening in the contact generation code, using persistent contacts, and could produce jittering.
* An indexing error was corrected that caused the CPU PGS solver with point friction to skip velocity and impulse writebacks in some scenarios.
* A thread synchronization issue was addressed that may have caused nondeterministic behavior in the CPU PGS solver.
* A crash that appeared when overflowing the maximum amount of aggregate pairs in the GPU broadphase has been fixed.
* A bug that generated broadphase pairs for removed shapes has been fixed.
* A crash that occurred when using excessively small contact buffers and/or patch buffers on the GPU. Now, contacts that don't fit into the buffer are dropped, and an error is reported.
* A bug when running generate_projects.bat if one of the parent directories contain a space.

### Deprecated

* PxSceneFlag::eFORCE_READBACK is deprecated. There is no replacement.
* PxSceneFlag::eSUPPRESS_READBACK is deprecated. The replacement is PxSceneFlag::eENABLE_DIRECT_GPU_API.

### Removed

* PxBVHStructure has been removed. Use PxBVH.
* PxBVHStructureDesc has been removed. Use PxBVHDesc.
* PxPhysics::getBVHStructures() has been removed. Use PxPhysics::getBVHs()
* PxGetAssertHandler() and PxSetAssertHandler() have been removed.
* PxMassModificationProps has been removed. Use PxConstraintInvMassScale instead.
* PxRegisterImmediateArticulations, PxRegisterArticulationsReducedCoordinate, PxRegisterHeightFields, PxCreateBasePhysics have been removed. Articulations and heightfields are now always enabled.
* PxBuffer has been removed. There is no replacement. The soft body interface now exposes direct access to GPU buffers.
* GpuExtension library has been removed.
* The deprecated PxPhysicsInsertionCallback has been removed. Please use PxInsertionCallback instead.
* The deprecated PxTaskType::Enum entries TT_CPU, TT_NOT_PRESENT and TT_COMPLETED have been removed. These entries were replaced with eCPU, eNOT_PRESENT and eCOMPLETED.
* These deprecated immediate-mode types have been removed: PxFeatherstoneArticulationJointData, PxFeatherstoneArticulationLinkData, PxFeatherstoneArticulationData, PxMutableLinkData, PxLinkData.
* The deprecated PxPhysics::createBVHStructure() and PxPhysics::getNbBVHStructures() functions have been removed.
* A deprecated PxPhysics::createAggregate() function has been removed.
* Deprecated passthrough functions in PxShape such as `getGeometryType()` and the specialized `get<geomType>Geometry()` were removed. Calls to these functions should be replaced by the accessing the underlying geometry directly with `getGeometry()`.
* Context creation for CUDA/Graphics interoperability has been deprecated. interopMode has been removed from PxCudaContextManagerDesc.
* No more Support for Microsoft Visual Studio 2013 and 2015.
* All 32 bits presets are removed.

## Rigid Body

### Fixed

* A crash involving static or kinematic aggregates used in combination with PxPairFilteringMode::eKILL has been fixed.
* PxRigidDynamicLockFlags (especially the linear lock flags) did not work properly with PGS. This has been fixed.
* A rare bug involving GPU aggregates, in which newly created actors could freely move through existing actors, has been fixed.
* A crash with invalid setups, where multiple materials were set on a shape referencing a triangle mesh that had no per-triangle material indices, has been fixed. Additionally this invalid setup will now trigger an error message.
* The convex-vs-mesh PCM contact generation is now more robust (CPU/GPU). Some jittering cases have been fixed.
* The capsule-vs-mesh PCM contact generation is now more robust (GPU). Some jittering cases have been fixed.
* A bug that produced jittering when contact modification was used has been fixed. It happened mainly for primitives-vs-mesh contacts, in cases where multiple contact patches were involved.
* A bug in GPU box-box contact generation that caused memory overflows and nondeterministic behavior as a result.
* A bug in the constraint solver that was using wrong indices to solve friction constraints.
* A crash in the GPU broadphase with empty aggregates has been fixed.
* Limited the maximum amount of memory that the SDF debug visualizer uses to avoid out-of-memory errors on high-resolution SDFs.
* A sufficiently large number of contacts is now generated for a dynamic object with an SDF collider lying on a single, large static triangle represented by a triangle mesh collider.
* Improved robustness to imperfect input when cooking SDFs; for example, duplicate triangles with opposite winding now produce a correct SDF.
* Fixed a rare case where the SDF contact generation algorithm could get stuck on SDF singularities and produce incorrect contacts.
* Resolved a crash that occurred when a sphere primitive collider comes into contact with an SDF collider.

## Articulations

### Added

* A new feature that computes and reports the force applied by a joint to a child link has been added.  The reported force is in the joint's child frame.  A more detailed specification of the reported force may be found in the doxy for the newly added array PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCES. The force may be queried on CPU using the newly added flag PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCES in conjunction with the newly added array PxArticulationCache::linkIncomingJointForce and the pre-existing function PxArticulationReducedCoordinate::copyInternalStateToCache(). The equivalent with the direct GPU API is to use the newly added enumeration PxArticulationGpuDataType::eLINK_INCOMING_JOINT_FORCE in conjunction with the pre-existing function PxScene::copyArticulationData().

### Fixed

* A rare bug involving articulations in aggregates has been fixed. An internal counter was not correctly updated, and could lead to internal asserts and potential crashes.
* Setting root link transforms or joint positions with the PxScene GPU API could result in incorrect collision behavior if an articulation contained links with no shapes.
* Incorrect values for link acceleration were reported with PxSceneFlag::eENABLE_GPU_DYNAMICS raised and lowered and with PxSceneFlag::eSUPPRESS_READBACK raised and lowered. This affected PxArticulationReducedCoordinate::getLinkAcceleration(), PxArticulationCache::linkAcceleration and PxScene::copyArticulationData().  This bug has been fixed.
* Incorrect values for joint acceleration were reported when PxSceneFlag::eENABLE_GPU_DYNAMICS was raised. This affected PxArticulationGpuDataType::eJOINT_ACCELERATION/PxScene::copyArticulationData() with PxSceneFlag::eSUPPRESS_READBACK raised and PxArticulationCacheFlag::eACCELERATION/PxArticulationReducedCoordinate::copyInternalStateToCache()/PxArticulationCache::jointAcceleration with PxSceneFlag::eSUPPRESS_READBACK lowered. This bug has been fixed.
* Setting a target velocity on a modifiable contact of an articulation link was not working properly. This has been fixed.
* A crash that appeared when adding an articulation was added and removed from a scene without running a sim step in-between has been fixed.
* Articulation links with extremely large mass in excess of approximately 1e+7 mass units had a tendency to fall through the ground due to internal threshold guards.  This affected all solver types running on CPU. This has been fixed.
* A floating-point precision issue resulting in slower-than-expected moving prismatic joints under certain simulation conditions with the TGS solver on GPU has been fixed.

### Deprecated

* PxArticulationSensor has been deprecated. Along with this, PxArticulationCache::sensorForces, PxArticulationCacheFlag::eSENSOR_FORCES, PxArticulationGpuDataType::eSENSOR_FORCE, and PxArticulationSensorFlag have also been deprecated.
* PxArticulationCache::jointSolverForces has been deprecated. The replacement is PxArticulationCache::linkIncomingJointForces. Along with this, the PxArticulationFlag::eCOMPUTE_JOINT_FORCES is also deprecated.

### Removed

* Deprecated PxArticulationJointReducedCoordinate::setLimit and PxArticulationJointReducedCoordinate::getLimit were removed. Use PxArticulationJointReducedCoordinate::setLimitParams and PxArticulationJointReducedCoordinate::getLimitParams instead.
* Deprecated PxArticulationJointReducedCoordinate::setDrive and PxArticulationJointReducedCoordinate::getDrive were removed. Use PxArticulationJointReducedCoordinate::setDriveParams and PxArticulationJointReducedCoordinate::getDriveParams instead.

### Changed

* Debug visualization of articulation links (body mass axes) will now show their sleeping state (similar to rigid bodies).

## Joints

### Changed

* The debug visualization color of active limits has changed from red to yellow.

### Removed

* Joint projection has been removed.
* The joint's "contact distance" parameter has been removed. The limits are now always active.

### Fixed

* The D6 joint's twist drive was using the wrong actor's axis (B instead of A). This has been fixed, and it could affect joint setups in existing scenes. To fix this in existing content it might be enough to flip the joint frames of involved actors, but this may not be possible depending on which other features (joint limits, etc) have been setup for the same joint. In the worst case it might be necessary to re-tune these joints.
* D6 joints configured to act as fixed joints (i.e. all motions locked) between static actors or world and a floating articulation base link did not constrain the rotational degrees of freedom.

### Deprecated

* PxContactJoint, PxJacobianRow and PxContactJointCreate() have all been marked as deprecated and will be removed in a later release.

## Scene queries

### Removed

* Deprecated PxBVH::raycast(), PxBVH::sweep() and PxBVH::overlap() functions have been removed. Use the new versions with callbacks.
* A deprecated PxQueryFilterCallback::postFilter() function has been removed. Use the similar function with a different signature.
* The deprecated PxGeometryQuery::getWorldBounds() function has been removed. Please use PxGeometryQuery::computeGeomBounds() instead.
* A deprecated PxGeometryQuery::raycast() function has been removed. Please use the other function with the same name but a different signature.

### Fixed

* Overlap queries could still return a non-zero number of hits when all hits got filtered in the post-filter callback (e.g. using PxQueryHitType::eNONE). This has been fixed.

### Added

* PxScene::evaluateSDFDistances() to evaluate sdf distances and gradients at given sample points for a batch of shapes

## Cooking

### Removed

* The deprecated PxCooking class has been removed. Use the standalone "immediate cooking" functions instead (e.g. PxCookBVH, PxCreateBVH...)
* PxCooking::cookBVHStructure() has been removed. Use PxCooking::cookBVH()
* PxCooking::createBVHStructure() has been removed. Use PxCooking::createBVH()

### Deprecated

* PxConvexFlag::eGPU_COMPATIBLE has been deprecated. Set PxCookingParams::buildGPUData to true to cook convex meshes that need to be GPU compatible.

### Fixed

* When convex hull cooking hit the polygon limit, the coplanar faces merge step was not run.

## Serialization

### Changed:

* Version mismatch checks etc. when deserializing binary data are now applied in profile and release build configurations too. PxSerialization::createCollectionFromBinary() will return NULL if the checks fail and error messages will get sent.

### Fixed:

* When deserializing materials (any PxBaseMaterial-derived), their userData member was wrongly re-initialized to zero, overwriting the serialized value.

## Pvd

### Changed:

* The OmniPVD API has been reworked to be more consistent and provide less room for confusion. Among the various changes are:
  * The "Set" attribute type has been renamed to "UniqueList". As a consequence, the OmniPvdWriter methods registerSetAttribute, addToSetAttribute.., removeFromSetAttribute.. have been renamed to registerUniqueListAttribute, addToUniqueListAttribute, removeFromUniqueListAttribute. The enum values eOmniPvdRegisterSetAttribute, eOmniPvdAddToSetAttribute, eOmniPvdRemoveFromSetAttribute have been renamed to eREGISTER_UNIQUE_LIST_ATTRIBUTE, eADD_TO_UNIQUE_LIST_ATTRIBUTE, eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE.
  * OmniPvdCommandEnum has been renamed to OmniPvdCommand and all enum values have been renamed too.
  * OmniPvdDataTypeEnum has been renamed to OmniPvdDataType
  * OmniPvdAttributeDataType has been removed and its usage in the API methods replaced with OmniPvdDataType::Enum
  * The OmniPvdWriter methods setAttributeShallow, addToSetAttributeShallow, removeFromSetAttributeShallow have been renamed to setAttribute, addToUniqueListAttribute, removeFromUniqueListAttribute and can be dinstinguished from the more generic versions with the same name by the argument list.
  * The order of the handleDepth and attributeHandles params has flipped in the OmniPvdWriter methods setAttribute, addToUniqueListAttribute, removeFromUniqueListAttribute (formerly called addToSetAttribute, removeFromSetAttribute) methods.
  * The order of the enumClassHandle and attributeName parameters has flipped in the OmniPvdWriter::registerFlagsAttribute() method.
  * OmniPvdReader::getCommandType() has been removed. The information is already available as part of the OmniPvdReader::getNextCommand() method.
  * The parameters in OmniPvdReader::startReading() have turned from pointers to references.
  * The stream parameters in OmniPvdWriter::setWriteStream() and OmniPvdReader::setReadStream() have turned from pointers to references.
  * The input parameters for the following functions have turned from pointers to references: destroyOmniPvdReaderFp, destroyOmniPvdWriterFp, destroyOmniPvdFileReadStreamFp, destroyOmniPvdFileWriteStreamFp, destroyOmniPvdMemoryStreamFp
  * Various input parameters in the methods of OmniPvdWriter, OmniPvdReadStream, OmniPvdWriteStream have changed from const to non-const.
  * The returned data pointers of the methods getClassName(), getAttributeName(), getObjectName(), getAttributeDataPointer() of OmniPvdReader are now const.
* The OmniPVD snippet now aborts in release build configuration since PVD is not available in release.
* Unit tests can now export single or series of automatically time stamped files, when the omnipvdfile command line parameter is set to a directory
* Crash bugs in the contact reports fixed
* OmniPVD now uses the OmniPVD API derived class support to stream debug data
  * Removes attribute duplication for shared base classes
  * Removed ambiguity about which class a certain object is part of
  * No need to have a class type attribute in streamed debug objects, the class is the type
  * As a consequece greatly simplifies object and class handling in the OmniPVD extension

## Vehicles2

### Fixed:

* When using sweeps for vehicle wheel vs. ground collision detection, PxVehiclePhysXRoadGeometryQueryUpdate() wrongly treated the case of an exactly zero hit distance (wheel touching ground with suspension being exactly at max compression) as no hit.
* Undesired sleep/wake cycle for vehicles that are not moving while engines are revving. Applying throttle will keep vehicles awake now.
* Negative suspension jounce (and an assert) could result in certain scenarios where PxVehicleSuspensionStateCalculationParams::limitSuspensionExpansionVelocity was set to true and gravity was pointing in the opposite direction of the suspension travel direction.
* Various PVD related bugs.
* If the wheel IDs in PxVehicleAxleDescription::wheelIdsInAxleOrder were shuffled, the wrong road geometry velocity information was used to compute the tire slip speeds.
* When driving backwards, the thresholdForwardSpeedForWheelAngleIntegration parameter (see PxVehicleSimulationContext) was ignored.

### Changed:

* PxVehiclePhysXRoadGeometryQueryParams has been adjusted to allow for wheel specific filter data. As a consequence, the method PxVehiclePhysXRoadGeometryQueryUpdate() has been adjusted too. See the migration guide for more details.
* Only the engine drivetrain or direct drivetrain properties are recorded in PVD now (and not both for the same vehicle).
* All the methods in PxVehiclePvdFunctions.h and PxVehiclePvdHelpers.h have been adjusted to use references to OmniPvdWriter, PxVehiclePvdObjectHandles or PxVehiclePvdAttributeHandles objects instead of pointers to make it even clearer that these parameters are required.
* The PxVehiclePvdAttributeHandles parameter of the PxVehiclePvdObjectRelease() method has been removed.
* The PxVehiclePvdAttributeHandles and OmniPvdWriter parameter of the PxVehiclePvdObjectCreate() method have been removed.
* The OmniPvdWriter parameter of the PxVehiclePvdAttributesRelease() method has been removed.


## Soft Body

### Added:

* Kinematic soft body feature
  * PxSoftBodyFlag::eKINEMATIC and PxSoftBodyFlag::ePARTIALLY_KINEMATIC.
  * PxSoftBody::setKinematicTargetBufferD function to set kinematic targets based on a device buffer.
  * PxConfigureSoftBodyKinematicTarget function to prepare kinematic targets for PxSoftBody::setKinematicTargetBufferD.
  * PxSoftBodyExt::relaxSoftBodyMesh function to repose a tetrahedral mesh from one configuration into another.
  * Optional outputVertexToInputTriangle, removeDisconnectedPatches parameters for PxTetMaker::simplifyTriangleMesh.
  * PxTetrahedronMeshExt::createPointsToTetrahedronMap function to associate points to their closest tetrahedon.
  * A snippet that shows how to set up and use a kinematic soft body.
* constraintOffset parameter to PxSoftBody::addSoftBodyAttachment and PxSoftBody::addClothAttachment to specify an offset of the PxConeLimitedConstraint with respect to the PxConeLimitedConstraint::mAxis. 

### Removed:

* PxBuffer has been removed. Writing and reading the softbody simulation state is now done directly in GPU buffers. For examples, see SnippetSoftBody.

### Deprecated:

* PxSoftBodyExt::commit() has been deprecated. The replacement is PxSoftBodyExt::copyToDevice().

### Changed:

* Renamed PxFEMSoftBodyMaterialModel::CO_ROTATIONAL, NEO_HOOKEAN to PxFEMSoftBodyMaterialModel::eCO_ROTATIONAL, eNEO_HOOKEAN.
* PxSoftBodyDataFlag has been renamed to PxSoftBodyGpuDataFlag.
* Default constructor of PxConeLimitedConstraint initializes PxConeLimitedConstraint::mAngle to -1.0 since 0.0f now indicates a 0.0 cone opening angle.
* Soft body flags used to copy stress computation were changed. The stress computation can now be performed via the intermediate data that can be copied from the internal buffers.

### Fixed:

* A bug that resulted in changes to PxSoftBodyFlags not being picked up has been fixed.
* Fixed a case where particles colliding with soft bodies could lead to a crash.
* Fixed a corner case where the mass of an internal node could become negative.
* An index bug when rendering tetrahedra in the snippets.

## Particles

### Changed:

* Renamed PxParticleRigidAttachment::mParams to mConeLimitParams
* Added PxParticleRigidAttachment constructor to initialize with PxConeLimitedConstraint and localPose0.
* Added PxConeLimitParams constructor to initialize with PxConeLimitedConstraint.
* Added PxParticleRigidFilterPair constructor to initialize with rigid node ID and particle ID.

### Fixed:

* A crash when using a large number of particles has been fixed.
* A bug that resulted in changes to PxParticleFlag not being picked up has been fixed.

# v5.1.3

## General

### Added:

*  Support for Microsoft Visual Studio 2022 for Windows builds.

### Fixed

* Changing the materials of a shape did not work when using GPU dynamics.
* Fixed exclusive shape binary serialization.

### Deprecated

* RepX/Xml serialization has been deprecated.

## Rigid Body

### Fixed

* A rare bug involving GPU aggregates, in which newly created actors could freely move through existing actors, has been fixed.

## Joints

### Fixed

* The D6 joint's twist drive was using the wrong actor's axis (B instead of A). This has been fixed, and it could affect joint setups in existing scenes. To fix this in existing content it might be enough to flip the joint frames of involved actors, but this may not be possible depending on which other features (joint limits, etc) have been setup for the same joint. In the worst case it might be necessary to re-tune these joints.

## Soft Body

### Fixed

* Rendering for tetmeshes in snippets had some tet faces inverted. This has been fixed.
* The voxel tetmesher won't crash anymore when called with zero elements as input.
* A bug in collision computation between a soft body and a scaled triangle mesh has been fixed.

## Particles

### Fixed

* The Poisson Sampler will not cause number overflows and crashes anymore when called with parameters that lead to too many samples.
* PxParticleClothCooker fixes to support shearing and bending constraints. This will change the behavior of newly cooked cloth assets.

## Pvd

### Fixed

* Fixed a potential crash bug when contact points are recorded through OmniPVD.


# v5.1.2

## General

### Fixed:

* Binary serialization of materials' userData.
* Fixed precision issue in index computation in Gu::HeightField::computeCellCoordinates [Issue #52](https://github.com/NVIDIA-Omniverse/PhysX/issues/52)
* Performance for SnippetCustomGeometry is now much better, particularly on Linux
* Compiler errors on Linux - [Issue #25](https://github.com/NVIDIA-Omniverse/PhysX/issues/25)

## Cooking

### Fixed

* A bug that generated non-GPU compatible convex meshes even though GPU compatibility was requested.


# v5.1.1

## General

### Changed:

* Be aware that due to reorganization of some virtual functions in the public interface the binary data layout has changed from v5.1.0. Linking code that includes the headers of v5.1.1 against binaries that have been built with an older version will likely cause problems.

### Added:

* Support for spatial and fixed tendon serialization.

### Fixed:

* Binary serialization of articulations had a bug, which got fixed.
* Includes [PR #8: Download bootstrap packages using TLS](https://github.com/NVIDIA-Omniverse/PhysX/pull/8/)

## Rigid Body

### Fixed

* A crash when colliding an SDF mesh against a sphere

## Particle Systems

### Fixed

* Particle systems now support is<> type conversion.

### Removed

* The PxParticlePhase class has been removed. It was unused.

## Vehicles2

### Changed:

* SnippetVehicle2Multithreading is now using custom profiling code to provide timings in release builds too.


# v5.1.0

## Supported Platforms

### Runtime

* Linux (tested on Ubuntu 20.04)
* Microsoft Windows 10 or later (GPU acceleration: display driver and GPU supporting CUDA 11 / CUDA ARCH 3.0)

### Development

* Microsoft Windows 10 or later
* Microsoft Visual Studio 2017, 2019

## Overview

* New SDF collisions!
* New custom geometry!
* New custom scene query system!
* New GJK queries API!
* New soft bodies!
* New mesh-vs-mesh overlap queries!
* New Vehicle SDK with customizable components and functionality!
* New gyroscopic forces!
* New gear joint and rack-and-pinion joint!

## General

### Added:

* A new function PxSetMutexProtocol() has been added exclusively for Linux OS.  This function affects the way in which shdfnd::Mutex sets flags that affect OS strategies to avoid thread priority inversion.  The behavior was hard-coded to PTHREAD_PRIO_INHERIT but now can be set to any of PTHREAD_PRIO_INHERIT, PTHREAD_PRIO_PROTECT, PTHREAD_PRIO_NONE.  A choice of PTHREAD_PRIO_NONE can lead to significant performance improvements with large thread counts but requires care to avoid priority inversion, a phenomena that occurs when a low priority thread holds a lock contended by higher priority threads.
* A flag PxVisualizationParameter::eSIMULATION_MESH has been added to render the simulation mesh instead of the collision mesh for tetmeshes.
* A flag PxVisualizationParameter::eSDF has been added to render the SDF of a mesh instead of the collision mesh for triangle meshes with SDFs.
* PxPhysics has new functions related to the creation and retrieval of tetrahedral meshes.

### Deprecated:

* PxPhysicsInsertionCallback is deprecated. Please use PxInsertionCallback instead.
* The PxFlags::set() function has been removed. Please now use PxFlags::raise() to set a single flag, or operator= to set all flags.
* The enum values of PxTaskType have been renamed for consistency reasons. See the corresponding API documentation for details.
* The PxRegisterHeightFields, PxRegisterArticulationsReducedCoordinate and PxCreateBasePhysics functions are deprecated.
* Binary data conversion and binary meta data have been deprecated.
    * PxBinaryConverter
    * PxConverterReportMode
    * PxGetPhysicsBinaryMetaData()
    * PxSerialization::serializeCollectionToBinaryDeterministic()
    * PxSerialization::dumpBinaryMetaData()
    * PxSerialization::createBinaryConverter()
    * PxBinaryMetaDataCallback
    * PxSerializationRegistry::registerBinaryMetaDataCallback()

### Fixed:

* PxPhysics::getFoundation() and PxScene::getPhysics() did not return the correct references in scenarios where two or more dynamic libraries are built with static PhysX libraries. In such a scenario, PxPhysics or PxScene objects from dynamic library A would return the wrong references when queried inside dynamic library B.
* Collision edges (PxVisualizationParameter::eCOLLISION_EDGES) were not properly rendered when PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE is used. Using this flag means that all edges are active, but none of them were rendered. The correct thing to do is to render all of them.
* PxActorFlag::eVISUALIZATION was not properly used. Shapes of actors whose visualization flag is disabled will now get skipped as well.
* Removed duplicate closing cross-thread event for Basic.narrowPhase event.
* Replaced all custom offsetof expressions that appear to dereference a null pointer with the PX_OFFSET_OF_RT macro - implementing Github PR 396.
* Debug visualization of face normals was incorrect for triangle meshes with negative scales. This has been fixed.

### Removed:

* Scene double buffering has been completely removed.  This means it is now illegal to call API write commands that make changes to the scene or to any actor or shape in a scene while the scene is simulating; that is, in-between PxScene::simulate() and PxScene::fetchResults().  Examples include PxRigidDynamic::setLinearVelocity() and PxScene::addActor(). Another example is PxShape::setLocalPose() for any shape attached to an actor that is in a scene currently being simulated.  Removal of scene double buffering has similar consequences for API read commands: it is now illegal to read any property that will be modified during a simulation step.  Examples include PxRigidActor::getGlobalPose() and PxConstraint::getForce(). Split simulation is slightly less restrictive in that some reads are allowed during PxScene::collide() and some writes allowed after PxScene::fetchCollision() but before PxScene::advance().  Examples include PxRigidActor::getWorldBounds() and PxArticulation::setWakeCounter(). However, it is important to note that the rules that apply to PxScene::simulate() apply equally to PxScene::advance().  In all build configs, any corresponding illegal API read or write will result in an error being issued to PxErrorStream and the illegal API call immediately returning without executing the function. A final comment is that API read operations in event callbacks remain legal.
* PxVisualizationParameter::eDEPRECATED_COLLISION_PAIRS has been removed.
* PxBroadPhaseCaps::maxNbObjects has been removed. It was unused.
* PxSceneFlag::eADAPTIVE_FORCE has been removed.
* The semi-advanced PhysX "Samples" are no longer provided. The "Snippets" continue to provide simple example code to illustrate how to use certain PhysX features. The physics demos in NVIDIA Omniverse offer more advanced samples now.
* The deprecated PxScene::setFrictionType() method has been removed. Simply set the desired friction type in PxSceneDesc.

### Changed:

* The Foundation types PxVec2, PxVec3, PxVec4, PxQuat, PxMat33, PxMat34, PxMat44 and PxTransform now have higher-accuracy implementations that use double instead of float. These are not currently used directly in the PhysX SDK but can be used by clients of the SDK if needed.
* The previous snippet SnippetRaycastCCD has been replaced with SnippetCCD. This snippet illustrates how to use different types of CCD methods, including regular, raycast and speculative CCD.
* PxDefaultCpuDispatcherCreate() has been modified to support different strategies to combat wasteful thread usage when there is no work to perform.
* The PxSimulationEventCallback functions onTrigger(), onContact() and onConstraintBreak() have slightly different behavior in that api queries to the physx actors referenced by the callbacks now return the state of the actor after the simulate step rather than the state of the actor at the detection event. At the risk of a performance penalty, the flags PxPairFlag::ePRE_SOLVER_VELOCITY and PxPairFlag::eCONTACT_EVENT_POSE may be used to retrieve the poses and velocities of the actors prior to the simulation step in the implemented onContact() function. These poses and velocities represent the state of the actors when the contact was first detected during the simulation step.
* PxCapsuleGeometry with halfHeight=0.0 are now legal.
* PxNodeIndex is now a 64-bit index, with the upper 32-bits representing the rigid body/actor ID and the lower 31-bits representing the articulation link ID and 1 bit to indicate if this is an articulation link or a rigid body. However, due to GPU memory constraints, an articulation can only support a maximum of 65536 links.
* Various PxScene::addXXX() functions now return a bool status (previously void) to detect errors more easily.
* TGS solver is now the default, PGS can still be used by setting the scene flags accordingly.
* The PxScene::addActor(), ::addActors(), ::addAggregate(), addCollection(), ::resetFiltering(), ::simulate(), ::advance(), ::collide() methods now return a boolean to denote success or failure.
* Several immediate-mode structs have been renamed from FeatherStoneArticulation to ArticulationRC (the last letters are an acronym for reduced-coordinate)

## Rigid body

### Added:

* A new flag PxConstraintFlag::eALWAYS_UPDATE has been added for constraints that should always be updated, i.e. the corresponding PxConstraintConnector::prepareData() function is called each frame automatically.
* A new flag PxConstraintFlag::eDISABLE_CONSTRAINT has been added. The solver prep functions are not called when this flag is set, effectively disabling the constraint.
* A userData parameter has been added to PxAggregate.
* A userData parameter has been added to PxConstraint.
* The PxPhysics::createAggregate() function has a new parameter. A deprecated wrapper for the previous function signature has been added.
* A new flag PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES has been added. This introduces gyroscopic forces to rigid bodies to simulate effects like the Dzhanibekov Effect. 
* A new class PxCustomGeometry has been added. User can now create custom collision shapes by implementing a set of callback functions.
* Two pre-made custom geometry implementations added in PxCustomGeometryExt extension - Cylinder and Cone.
* A new set of API functions PxGjkQuery have been added. This is intended for spatial queries on custom shapes represented by their GJK Support Mapping.
* PxMeshGeometryFlag::eTIGHT_BOUNDS has been added. This is similar to PxConvexMeshGeometryFlag::eTIGHT_BOUNDS, but for triangle meshes.
* A new broadphase has been added (PxBroadPhaseType::ePABP).
* A standalone broadphase interface has been added (see PxCreateBroadPhase and PxCreateAABBManager).
* A compliant contact model has been added. Users can now customize spring-stiffness and damping for a soft contact response.
* Triangle mesh colliders are now supported on dynamic rigid bodies if a SDF (Signed Distance Field) gets generated during cooking.
* PxSceneDesc::frictionCorrelationDistance allows to configure the distance for merging contact points into a single anchor point.
* PxSceneDesc::contactPairSlabSize can be used to define the size of the contact pool slabs.

### Removed:
* PxSceneDesc::solverOffsetSlop has been removed and can now be set per rigid body (see PxRigidBody::setContactSlopCoefficient()).

### Changed:

* PxShape::getGeometry() now returns a PxGeometry reference instead of a PxGeometryHolder. See the migration guide to 5.1 for details. The PxShape::getGeometryType() and PxShape::getXXXGeometry() functions have been deprecated as a result.
* PxMaterialFlag::eIMPROVED_PATCH_FRICTION is now enabled by default.
* PxRigidBody::setLinearVelocity() was removed and replaced with PxRigidDynamic::setLinearVelocity()
* PxRigidBody::setAngularVelocity() was removed and replaced with PxRigidDynamic::setAngularVelocity()
* PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD and PxRigidBodyFlag::eENABLE_CCD can now be enabled at the same time on a given body. In this hybrid CCD mode the linear part of the motion is handled by sweeps, and the angular part of the motion is handled by speculative contacts.

### Fixed:

* Removing a shape from a sleeping dynamic rigid actor woke the actor up if it was touching a shape of a static rigid actor.
* Removing a shape from a dynamic rigid actor, did not necessarily wake up touching actors even though wakeOnLostTouch was set to true in PxRigidActor::detachShape().
* A performance problem in PxScene::getActors() has been fixed.
* Missing support for maxContactForces in the TGS solver has been added.
* A rare crash due to reading past the boundaries of a memory pool has been fixed.
* Deformable meshes using the BVH34 midphase structure did not handle collision edge flags correctly. This has been fixed.
* Collision edge flags were sometimes incorrect for negative-scaled meshes, giving birth to invalid or missed contacts. This has been fixed.
* The sphere-vs-mesh PCM contact generation had a bug that sometimes made the sphere go through the mesh due to a missed vertex contact. This has been fixed.
* Performance and stability issues when simulating convexes colliding against many triangles in complex PxTriangleMesh geometries has been improved.
* Attempting to apply a force to a kinematic rigid body will no longer lead to a crash in profile or release builds.
* Triangle mesh negative scale support for GPU code path.
* Switching a constrained dynamic body to kinematic no longer triggers an assert in debug mode.

## Joints

### Added:

* New PxGearJoint and PxRackAndPinionJoint have been added.

### Fixed:

* PxJoint::setActors() had a bug when called at runtime, that left an internal structure referencing the previous actors. As a result PxConstraintFlag::eCOLLISION_ENABLED was not properly handled between jointed actors.

### Deprecated:

* PxJointLimitParameters::contactDistance is deprecated.
* Joint projection is deprecated.

## Scene queries

### Removed:

* Batched query support has been removed from the PhysX SDK. The deprecated structs/classes/callbacks PxBatchQuery, PxBatchQueryDesc, PxBatchQueryResult, PxRaycastQueryResult, PxSweepQueryResult, PxOverlapQueryResult, PxBatchQueryPreFilterShader and PxBatchQueryPostFilterShader have all been removed from the SDK. The deprecated function PxScene::createBatchQuery has also been removed. In place of PxBatchQuery, a new class PxBatchQueryExt has been added to the extensions library. This acts as a wrapper for the functions PxScene::raycast(), PxScene::sweep() and PxScene::overlap() and aims to preserve the core functionality of PxBatchQuery. PxBatchQueryExt instances are instantiated with the function PxCreateBatchQueryExt(). 

### Added:

* PxSceneDesc can now take an optional PxSceneQuerySystem parameter. If defined, all PxScene scene-query related calls will be re-routed to this interface. This allows users to potentially customize the implementation of all scene-query operations. An external implementation of this interface is available in the PxExtensions library, and can be created with the PxCreateExternalSceneQuerySystem function.
* It is possible to further customize the scene-query system, e.g. using more than the two built-in pruning structures in PhysX. See PxCreateCustomSceneQuerySystem and SnippetMultiPruners.
* The function PxCreateBatchQueryExt() has been added to the extension library. The purpose of this function is to instantiate a new class PxBatchQueryExt. This class acts as a replacement for the PxBatchQuery class of previous releases which has now been removed. PxBatchQueryExt allows queries to be added to a queue and then executed on command.
* The flag PxQueryFlag::eBATCH_QUERY_LEGACY_BEHAVIOUR has been added to support PxBatchQueryExt and/or any other user replacement for PxBatchQuery. When this flag is raised, the PhysX SDK ignores an internal filter equation and guarantees that the PxQueryHitType returned by the corresponding PxQueryFilterCallback instance is used directly without any other logic being applied.
* The function PxBatchQueryStatus::getStatus() has been added to the extensions library to determine if an overflow occurred during the execution of a batch with PxBatchQueryExt::execute(). Overflows occur when the touch buffer is insufficiently large to return all touches for all queries.
* The function PxScene::overlap() now has an optional PxQueryCache pointer as function argument. This follows the pattern of the complementary raycast() and sweep() functions of the PxScene class.
* The function PxGeometryQuery::pointDistance() now supports meshes when the PxMeshMidPhase::eBVH34 data structure is used. It has a new parameter to return the closest triangle index for meshes.
* SnippetPointDistanceQuery, SnippetGeometryQuery, SnippetStandaloneBVH and SnippetPathTracing have been added.
* The PxScene::raycast(), PxScene::overlap() and PxScene::sweep() functions have a new PxGeometryQueryFlags parameter.
* The PxGeometryQuery::raycast(), PxGeometryQuery::overlap(), PxGeometryQuery::sweep(), PxGeometryQuery::computePenetration(), PxGeometryQuery::pointDistance() functions have a new PxGeometryQueryFlags parameter.
* The PxMeshQuery::findOverlapTriangleMesh(), PxMeshQuery::findOverlapHeightField() and PxMeshQuery::sweep() functions have a new PxGeometryQueryFlags parameter.
* PxBVH now has a culling function (PxBVH::cull()) that can be used to implement view-frustum culling. See SnippetFrustumQuery for an example.
* PxBVH now has refit functions (PxBVH::refit(), PxBVH::partialRefit()) that can be used for dynamic trees. See SnippetStandaloneBVH for an example.
* PxBVH now has a generic traversal function (PxBVH::traverse()) that can be used for arbitrary purposes, like e.g. debug-visualizing the tree bounds. See SnippetStandaloneBVH for an example.
* There is a new PxFindOverlap function to find overlaps between two PxBVH objects.
* The PxRigidActorExt::createBVHFromActor() helper function has been added.
* PxSceneDesc::dynamicTreeSecondaryPruner has been added. The new PxDynamicTreeSecondaryPruner enum lets users choose which implementation to use in dynamic trees.
* PxSceneDesc::staticBVHBuildStrategy and PxSceneDesc::dynamicBVHBuildStrategy have been added. This lets users control the build strategy of the static & dynamic pruning structures.
* PxSceneDesc::staticNbObjectsPerNode and PxSceneDesc::dynamicNbObjectsPerNode have been added. This lets users control the number of objects per node for the static & dynamic pruning structures.
* PxHitFlag::eANY_HIT has been added. It is similar to the previous PxHitFlag::eMESH_ANY flag, but this time for any geometry that contains multiple primitives - for example a PxCustomGeometry.
* PxGeometryQuery::raycast, PxGeometryQuery::overlap and PxGeometryQuery::sweep now take an optional context parameter that is passed to the low-level functions, and in particular to the PxCustomGeometry callbacks.
* PxGeometryQuery::overlap now supports triangle mesh vs triangle mesh.
* A new PxMeshQuery::findOverlapTriangleMesh function has been added to compute triangle overlaps between two triangle meshes.		

### Fixed:

* The build code for BVH34 trees had an issue that could produce degenerate trees, leading to rare performance problems and even stack overflow during traversal. This has been fixed, but it made the build code slightly slower, which could be a problem for users cooking at runtime. Since the problem was rare, the previous/faster build code has been kept, available in PxBVH34BuildStrategy::eFAST. It is not enabled by default.
* The scene query system was sometimes incorrectly updated for PxBVH structures. This has been fixed.
* A crash has been fixed when doing a query against an empty scene while using PxPruningStructureType::eNONE for the dynamic structure.
* The BVH34 codepath had a bug in the raycast-vs-mesh-with-multiple-hits case, where returned hits could be further away than defined max hit distance. This has been fixed.
* A rare crash involving the compound pruner and the PxActorFlag::eDISABLE_SIMULATION flag has been fixed.
* Fixed a rare scene-query issue happening with stabilization enabled (the SQ structures could miss updates, leading to incorrect SQ results).
* In rare cases, PxTriangleMesh::refitBVH() could return an incorrect bounding box when using PxMeshMidPhase::eBVH34. This has been fixed.
* In very rare cases, sweep tests using the eMTD flag and exactly touching a shape (impact distance == 0) could return an incorrect impact normal. This has been fixed.
* The capsule-vs-heightfield overlap query (PxGeometryQuery::overlap) was not reporting hits in some cases. This has been fixed.
* The convex-vs-heightfield overlap query (PxGeometryQuery::overlap) had a bug when using scaled convexes. This has been fixed.
* The sphere-vs-mesh and capsule-vs-mesh sweeps sometimes returned slightly incorrect impact distances (especially with long sweeps), which resulted in swept shapes penetrating the meshes when moved to the impact positions. This has been fixed.
* In rare cases the sphere-vs-mesh and capsule-vs-mesh sweeps could miss triangles entirely. This has been fixed.

### Changed:

* The PxQueryHit struct does not contain touched actor & shape pointers anymore. They have been moved higher up to the PxRaycastHit, PxOverlapHit and PxSweepHit structs. Explicit padding has also been dropped for these classes.
* The PxQueryFilterCallback::postfilter() function has changed. The hit actor and hit shape are now passed as extra arguments to the function.
* PxGeometryQuery::raycast() now operates on PxGeomRaycastHit structures and takes an extra stride parameter. Similarly PxGeometryQuery::sweep() now uses a PxGeomSweepHit structure.

### Deprecated:

* PxGeometryQuery::getWorldBounds() has been deprecated. Please use PxGeometryQuery::computeGeomBounds() instead.
* PxHitFlag::eMESH_ANY has been deprecated. Please use PxHitFlag::eANY_HIT instead.
* PxBVHStructure has been renamed to PxBVH. PxBVHStructureDesc has been renamed to PxBVHDesc.
* The PxBVHStructure scene query functions have changed. The old API is deprecated, a new API has been added.

## Character controller

### Added:

* A PxClientID parameter has been added to PxControllerDesc, to let users setup the owner client before the kinematic actor is added to the scene.

### Fixed:

* The vertical displacement vector in the down pass was sometimes incorrect (larger than it should have been). This has been fixed.
* Releasing an articulation link while a character was standing on it produced a crash. This has been fixed.

## Vehicles2

### Added:

* The Vehicle SDK has been refactored into a completely new form to allow rapid customization and prototyping. SnippetVehicle2Customization, SnippetVehicle2DirectDrive, SnippetVehicle2FourWheelDrive etc. demonstrate use of the new Vehicle SDK. The public API for the new Vehicle SDK may be found under "physxRoot/include/vehicle2". All functions, structs and classes in the new Vehicle SDK are inside the physx::vehicle2 namespace. A migration guide may be found in the PhysX 5.1 SDK Guide under the subsection "Migrating From PhysX SDK 4.0 to 5.1/Vehicles".

## Vehicles

### Deprecated:

* All structs, classes and functions of the old Vehicle SDK have been marked as deprecated and will be removed in a later release.

### Changed:

* Concurrent calls to PxVehicleUpdateSingleVehicleAndStoreTelemetryData() are now permitted if the additional parameter vehicleConcurrentUpdates is used. 
* The functions PxVehicleSuspensionSweeps() and PxVehicleSuspensionRaycasts() have been modified to accommodate the removal of PxBatchQuery and the addition of PxBatchQueryExt. The arguments of both functions have been modified with a PxBatchQueryExt pointer directly replacing a PxBatchQuery pointer. New functionality in PxBachQueryExt has allowed PxRaycastQueryResult/PxSweepQueryResult to be removed from the argument list of PxVehicleSuspensionRaycasts()/PxVehicleSuspensionSweeps(). This change much simplifies calls to PxVehicleSuspensionRaycasts() and PxVehicleSuspensionSweeps() and requires less user management of the various arrays involved.

### Added:

* PxVehicleWheelsDynData::getConstraints() and PxVehicleWheelsDynData::getNbConstraints() have been added to potentially have vehicles use immediate mode for solving the vehicle rigid body constraints.
* New method PxVehicleGraph::getRawData() to extract raw telemetry data.
* New PxVehicleSteerFilter class, used in PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs & PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs to smooth the vehicle's steering angle when going from air to ground.
* A new function PxVehicleWheelsDynData::setTireContacts() has been added to the PhysX Vehicle SDK. This function allows users to directly set tire contact plane and friction for all tires on the vehicle as an alternative to using PxVehicleSuspensionSweeps() or PxVehicleSuspensionRaycasts().
* A new function PxVehicleDrivableSurfaceToTireFrictionPairs::getSurfaceType(const PxMateria& surfaceMaterial), which returns the surface type associated with a PxMaterial instance.
* A new function PxVehicleDrivableSurfaceToTireFrictionPairs::getTypePairFriction(const PxMaterial& surfaceMaterial, const PxU32 tireType), which returns the friction value associated with a specified combination of PxMaterial instance and tire type.
* New complementary functions PxVehicleDrivableSurfaceToTireFrictionPairs::serializeToBinary() and PxVehicleDrivableSurfaceToTireFrictionPairs::deserializeFromBinary(), which allow  friction tables to be serialized and deserialized.
* A new structure PxVehicleContext has been introduced to allow, for example, to have a set of common settings for vehicles in scene A and a different set for vehicles in scene B. PxVehicleUpdates() is one of the methods where there is now the option to use this new structure.
* The flag PxVehicleWheelsSimFlag::eDISABLE_SPRUNG_MASS_SUM_CHECK has been introduced to allow setting suspension sprung mass values that do not sum up to the mass of the vehicle rigid body.

### Fixed:

* Vehicle wheel suspension sweeps used the wrong scale for the sweep geometry if the wheel shapes used PxConvexMeshGeometry with scale factors other than 1.

## Cooking

### Deprecated:

* The PxCooking object is deprecated. Please use the new standalone cooking functions instead (in the cooking library) or the low-level cooking functions (in the GeomUtils library).
* PxCooking::cookBVHStructure() is deprecated. Please use PxCookBVH() instead.

### Added:

* Added PxBVH34BuildStrategy enum to PxBVH34MidphaseDesc. Users can now select a SAH-based build strategy for BVH34 trees.
* BVH34 trees can now be quantized or not depending on PxBVH34MidphaseDesc::quantized.
* Added remeshing and mesh simplification to preprocess meshes such that they can be used for softbody simulation. New API functions are PxTetMaker::simplifyTriangleMesh()/remeshTriangleMesh()/createTreeBasedTetrahedralMesh()
* Added low-level cooking functions (GeomUtils), that can be used to cook objects without using the cooking library. See SnippetStandaloneQuerySystem for an example. The cooking library is still here though for backward compatibility.
* Triangle mesh cooking supports the generation of a SDF (Signed Distance Field) to allow triangle mesh colliders on dynamic actors.

### Fixed:

* Serialization of uncompressed BVH34 trees was broken (both for regular cooked files and binary serialization). This has been fixed.
* PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE was not taken into account when PxCookingParams::buildTriangleAdjacencies was used.

### Changed:

* PxMeshMidPhase::eBVH34 is now the default midphase structure. PxMeshMidPhase::eBVH33 has been deprecated.

## Pvd

### Added:

* Adds the OmniPVD API - an object oriented serialization and deserialization library for debug data. The project is called PvdRuntime.
  * Adds the OmniPVDWriter exposing serialization through OmniPVDWriteStream, OmniPVDFileWriteStream.
  * Adds the OmniPVDReader exposing de-serialization through OmniPVReadStream, OmniPVDFileReadStream.
* Adds an OmniPVD API user in the PhysX SDK, exposed through PxOmniPVD, which streams PhysX debug data into an OmniPVD API write stream.
  * PxOmniPVD is now an optional argument to the PxCreatePhysics() function, allowing for the parallel inclusion of both PVD and OVD debug streams.
  * Adds PxPhysics::getOmniPvd() which returns the PxOmnipVD instance used in the call to PxCreatePhysics()
  * Exports : Rigid Static, Rigid Dynamic, Joints, Articulations, Contacts and more into the OmniPVD stream.
* Possibility to serialize and save out, as well as read and parse OVD files (through the usage of OmniPVD API), which constain an OmniPVD command stream of PhysX debug data.
* Comes with a viewer in Omniverse Create, in the form of an extension called omni.physx.pvd also known as the OmniPVD extension.
  * Adds the ability to import and inspect OVD files using the USD interface

### Changed:

* The file source/pvd/include/PxPvdRenderBuffer.h has been removed along with the structs it declared: pvdsdk::PvdDebugPoint, pvdsdk::PvdDebugLine, pvdsdk::PvdDebugTriangle and pvdsdk::PvdDebugText.  Usage of these structs may be directly replaced with PxDebugPoint, PxDebugLine, PxDebugTriangle and PxDebugText, which are all declared in common/PxRenderBuffer.h. 

## Articulations

### Deprecated:

* PxArticulationJointReducedCoordinate::setLimit()/getLimit()/setDrive()/getDrive() functions are deprecated. New API has been added 

### Added:

* Armature was added to the articulation joints. This adds additional inertia to the joint degrees of freedom.
* PxArticulationJointReducedCoordinate::dofScale was added. This scales the projection of a child's inertia onto the joint degrees of freedom, resulting in an implicit resistance to motion around the joint. This can improve simulation stability.
* Sensors were added to PxArticulationReducedCoordinate to report forces and torques acting on links in articulations.
* Fixed tendon support was added to articulations.
* Spatial tendon support was added to articulations.
* PxArticulationJointReducedCoordinate::setJointPosition/PxArticulationJointReducedCoordinate::getJointPosition was added.
* PxArticulationJointReducedCoordinate::setJointVelocity/PxArticulationJointReducedCoordinate::getJointVelocity was added.
* PxArticulationReducedCoordinate::getRootGlobalPose() was added.
* PxArticulationReducedCoordinate::setRootLinearVelocity()/getRootLinearVelocity() was added.
* PxArticulationReducedCoordinate::setRootAngularVelocity()/getRootAngularVelocity() was added.
* PxArticulationReducedCoordinate::setMaxCOMLinearVelocity()/getMaxCOMLinearVelocity() was added.
* PxArticulationReducedCoordinate::setMaxCOMAngularVelocity()/getMaxCOMAngularVelocity() was added.
* PxArticulationReducedCoordinate::setStabilizationThreshold()/getStabilizationThreshold() was added.
* PxArticulationReducedCoordinate::isSleeping()/wakeUp()/putToSleep() was added.
* PxArticulationReducedCoordinate::setSleepThreshold()/getSleepThreshold() was added.
* PxArticulationReducedCoordinate::setSolverIterationCounts()/getSolverIterationCounts() was added.
* PxArticulationReducedCoordinate::setWakeCounter()/getWakeCounter() was added.
* PxArticulationReducedCoordinate::updateKinematic() and corresponding PxArticulationKinematicFlags were added. The method allows updating link states after changing joint and root state through the respective PxArticulationReducedCoordinate and PxArticulationJointReducedCoordinate API calls.
* PxArticulationLink::setCfmScale()/getCfmScale() was added.
* Articulations in a GPU simulation may be updated/read directly from/to user-provided device buffers, see notes in GPU Rigid Bodies->Added.
* PxArticulationReducedCoordinate::setLimitParams/getLimitParams/setDriveParams()/getDriveParams() was added.
* Articulation system supports up to 65536 links per articulation. 
* PxScene::computeGeneralizedMassMatrices() was added for batch computation of articulations' mass matrices on GPU.

### Changed:

* It is no longer possible to change the articulation structure while it is in a scene. However, the articulation retains its state through removal and re-adding to the scene, even when its configuration changes, so the application can remove the articulation from the scene, change its structure and re-add it to the scene in a straightforward way. Specifically, the following is no longer possible when an articulation is in a scene:
  * Adding or removing links, tendons, or sensors 
  * Changing sensor flags and local poses 
  * Changing joint type or motion configuration 
* PxArticulationReducedCoordinate::getCoefficientMatrixSize returns element size (i.e. number of PxReals) instead of Byte size, and returns 0xFFFFFFFF instead of 0 in case the articulation is not in a scene.
* Removed PxArticulationReducedCoordinate::releaseCache function and introduced a release method with the PxArticulationCache. 
* Removed PxArticulationReducedCoordinate::releaseSpatialTendon function and introduced a release method with PxArticulationTendon.
* Removed PxArticulationReducedCoordinate::releaseFixedTendon function and introduced a release method with PxArticulationTendon.
* Removed PxArticulationSpatialTendon::releaseAttachment function and introduced a release method with PxArticulationAttachment.
* Removed PxArticulationFixedTendon::releaseTendonJoint function and introduced a release method with PxArticulationTendonJoint.
* Replaced PxArticulationFixedTendon::teleportRootLink function with PxArticulationReducedCoordinate::setRootGlobalPose.
* Both PxArticulationReducedCoordinate::getDofs and PxArticulationReducedCoordinate::getCacheDataSize return 0xFFFFFFFF instead of 0 in case the articulation is not in a scene.

### Fixed:

* Velocity drives on prismatic joints now consistent with rigid body prismatic joints.
* Numerical integration has been improved to better conserve momentum.

### Removed:

* The maximal coordinate articulations have been removed with the equivalent functionality exposed through the newer reduced coordinate articulations.
* It is no longer possible to set a global pose on an articulation link.
* It is no longer possible to set the linear velocity on an articulation link.
* It is no longer possible to set the angular velocity on an articulation link.
* PxArticulationReducedCoordinate::getLinkVelocity. Use PxArticulationLink::getLinearVelocity/getAngularVelocity or PxArticulationCache to read link velocities.

## GPU Rigid Bodies

### Added:

* Support for GPU-accelerated aggregate broad phase collision detection has been added.
* PxSceneFlag::eSUPPRESS_READBACK flag was added. This suppresses state readback from GPU to the CPU (e.g. rigid body transforms, velocities, articulation state), which significantly improves performance. However, in order to access or set state on rigid bodies/articulations, the application must use the new copy/apply GPU API to access this state, providing device buffers to read from/write to.
* PxSceneFlag::eFORCE_READBACK flag was added to force GPU readback of articulation data even if PxSceneFlag::eSUPPRESS_READBACK is set.
* PxScene::copyArticulationData was added to copy the state of a set of articulations from the internal buffers inside PhysX to a user-provided device buffer.
* PxScene::applyArticulationData was added to apply the state of a set of articulations from a user-provided device buffer to the internal buffers inside PhysX.
* PxScene::copyActorData was added to copy the PxRigidDynamic/PxRigidActor data from the internal buffers inside PhysX to a user-provided device buffer.
* PxScene::applyActorData was added to apply the state of a set of PxRigidDynamic/PxRigidActor from a user-provided device buffer to the internal buffers inside PhysX.
* PxScene::copySoftBodyData was added to copy the state of a set of soft bodies from the internal buffers inside PhysX to a user-provided device buffer.
* PxScene::applySoftBodyData was added to apply the state of a set of soft bodies from a user-provided device buffer to the internal buffers inside PhysX.
* PxScene::copyContactData was added to copy the contact data from the internal buffers inside PhysX to a user-provided device buffer.

### Changed:

* Reworked PxgDynamicsMemoryConfig to simplify configuring GPU memory usage. This change can also significantly reduce GPU memory usage compared to PhysX 4.

### Fixed:

* Speculative CCD support was added to GPU rigid bodies.

## Particle System

### Added:

* A PBD (Position Based Dynamics) particle system capable of simulating fluids and granular materials
* Interacts with all other supported actors (rigid bodies, soft bodies etc).
* User buffer architecture to store particles. It simplifies adding and removing particles at runtime and eliminates the need to specify a maximal number of particles when setting up a particle system.
* Supports multiple materials. Each particle can reference its own or a shared material.

## Softbodies

### Added:

* FEM (Finite Element Method) based softbodies.
* Interact with all other supported actors (rigid bodies, particles etc).
* Generation of tetmeshes to create a softbody out of a triangle mesh. Different kinds of tetmeshes are supported to match different use cases (conforming and voxel based tetmeshes).
* Per-tetrahedra materials support.
* Attachment support including soft body vs soft body and soft body vs rigid body.


# v4.1.2
April 2021

## General
    
### Added:
        
* Added SnippetTriggers to show how to emulate triggers using regular non-trigger shapes. This supports trigger-trigger notifications and CCD.
* Added Android 64 bits target:
            
  * Added build preset for Android arm64-v8a architecture.
  * Using ANDROID ABI as part of the Android output folder to avoid name collisions between 32 and 64 bit binaries.
  * Ignoring strict-aliasing warning on Android.
  * Fixed compilation error on Android debug armv8: Not inlining computeDriveInertia function to fix "conditional branch out of range" error.
            
* Added support to build iOS with dynamic libraries:
            
  * The changes are copied from macOS makefiles, now iOS makefiles are in line with macOS ones.
  * Update toolchain cmake file to only generate 64 bits target on iOS (as its preset suggests because it's called "ios64").
            
* Added support to build Android with dynamic libraries.
            
  * The changes are copied from iOS makefiles, now Android makefiles are in line with iOS ones.
            
* Modified cmake files of PhysXCharacterKinematic and PhysXExtension projects for Mac/iOS/Android so they add the suffix "_static" like the other platforms.
        
### Fixed
        
* Some profile zones did not properly setup the "context" parameter. This has been fixed.
* Removed duplicate closing cross-thread event for Basic.narrowPhase event.
* Fixed buffer over-read in CmPool.h
* Replaced all custom offsetof expressions that seem to dereference a null pointer with the PX_OFFSET_OF_RT macro.
* Replaced run-time assert on sizeof(PxHeightFieldSample::height) with compile-time assert in physx::Gu::HeightFieldUtil constructor.
* Various minor issues (doc typos, const correctness, compilation warnings, etc) reported on GitHub have been fixed.
        
    

##  Rigid body 
    
### Changed:
        
* PxScene::setFrictionType() has been marked as deprecated due to its strong limitations. Simply set the desired friction type in PxSceneDesc.
* It is now legal to set the number of velocity iterations to zero. In some difficult configurations involving large mass ratios, the TGS solver's convergence can be negatively impacted by velocity iterations.
        
### Fixed
        
* The PxContactSet::ignore() function was not working properly and has been fixed. This may have caused issues in PxVehicleModifyWheelContacts.
* The debug visualization code could crash when using PxVisualizationParameter::eCOLLISION_COMPOUNDS. This has been fixed.
* Fixed a crash in the reduced-coordinates articulation system when the application removes the articulation from the scene and reinserts it back into the scene.
* Improved stacking quality with TGS solver simulating stacks of articulations.
* Fixed TGS solver stability issue constraining a rigid body to a kinematic actor. 
* Fixed typo with ang dominance in CPU block solver.
* Fixed rare crash destroying a contact manager during CCD.
* Fixed buffer over-write when simulating a convex mesh with more than 64 vertices in a single face.
        
### Added
        
* PxRigidBodyFlag::eFORCE_KINE_KINE_NOTIFICATIONS and PxRigidBodyFlag::eFORCE_STATIC_KINE_NOTIFICATIONS have been added.
        
    

## Cooking
    
### Fixed:
        
* The number of bytes allocated for vertices by the convex hull builder was incorrect. This has been fixed.
        
    

## Serialization
    
### Fixed:
        
* A performance problem in PxBinaryConverter when converting large collections has been fixed.
        
    

## Vehicles
    
### Added:
        
* PxVehicleWheelsDynData::getConstraints() and PxVehicleWheelsDynData::getNbConstraints() have been added to potentially have vehicles use immediate mode for solving the vehicle rigid body constraints.
* New method PxVehicleGraph::getRawData() to extract raw telemetry data.
* An inflation parameter has been added to PxVehicleSuspensionSweeps.
* New flags PxVehicleWheelsSimFlag::eDISABLE_INTERNAL_CYLINDER_PLANE_INTERSECTION_TEST and PxVehicleWheelsSimFlag::eDISABLE_SUSPENSION_FORCE_PROJECTION have been added.
        
### Changed:
        
* Concurrent calls to PxVehicleUpdateSingleVehicleAndStoreTelemetryData() are now permitted if the additional parameter vehicleConcurrentUpdates is used. 
        
### Fixed:
        
* A null pointer dereference bug has been fixed. The bug occurred if the vehicle's rigid body actor was asleep and the vehicle relied on cached tire contact planes rather than the results of a fresh suspension query.
        
    

## Character controller
    
### Fixed:
        
* The prefilter & postfilters callback were called all the time, ignoring the PxQueryFlag::ePREFILTER and PxQueryFlag::ePOSTFILTER flags. This has been fixed.
        
    

## Scene queries
    
### Fixed:
        
* The BVH34 codepath had a bug in the raycast-vs-mesh-with-multiple-hits case, where returned hits could be further away than defined max hit distance. This has been fixed.
        
    



# v4.1.1
August 2019

## General
    
### Added:
        
* Support for Visual Studio 2019 has been added, cmake 3.14 is required.
        
### Changed:
        
* Android binary output directory name contains Android ABI string.
        
    

## Vehicles
    
### Added:
        
* PxVehicleWheelsSimFlags and corresponding set/get methods have been added to PxVehicleWheelsSimData. The flag eLIMIT_SUSPENSION_EXPANSION_VELOCITY can be used to avoid suspension forces being applied if the suspension can not expand fast enough to push the wheel onto the ground in a simulation step. This helps to reduce artifacts like the vehicle sticking to the ground if extreme damping ratios are chosen.
        
### Fixed:
        
* The PxVehicleDrive::setToRestState() was not clearing all cached data, which could sometimes make vehicles misbehave after calls to this function.
        
    

## Cooking
    
### Added:
        
* Added error message when not at least four valid vertices exist after vertices cleanup.
        
    

## Serialization
    
### Fixed:
        
* Binary serialization of kinematic rigid dynamic actors was failing unless they were part of a scene.
        
    

##  Rigid body 
    
### Fixed
        
* Out of shared memory failure with GPU articulations.
* Inconsistent results when setting joint drive targets with GPU articulations compared to CPU articulations.
* Assert when simulating a scene with > 64k rigid bodies and joints.
* Error in PxActor::getConnectors() method when there are multiple connector types.
* Setting joint positions on articulations did not update world-space link poses and velocities.
* Improved TGS articulation joint drive solver.
* Improved robustness of articulation spherical joints.
* Joint forces/positions/velocities set through the PxArticulationCache are correctly applied when using GPU articulations.
* Fixed rare crash in MBP when the system contains out-of-bounds objects.
* Fixed a crash in the reduced-coordinates articulation system when the application removes the articulation from the scene and reinserts it back into the scene.
        
    



# v4.1.0
March 2019

## Overview
    
### Immediate mode support for reduced-coordinates articulations and the temporal Gauss Seidel solver.
### GPU acceleration for reduced-coordinates articulations.
    

## General
    
### Added:
        
* Added support for UWP, note that cmake 3.13.4 is required for uwp arm64.
        
### Fixed:
        
* PhysXGpu DLLs are now standalone, so they will now work with both static and dynamic PhysX libraries.
* PhysX delay loading code is disabled for the static library configuration.
        
### Changed:
        
* Removed PxGpuDispatcher class. Instead of querying the GPU dispatcher with PxCudaContextManager::getGpuDispatcher() and providing it to the PxScene with PxSceneDesc::gpuDispatcher, please provide the CUDA context manager directly using PxSceneDesc::cudaContextManager.
* PxCreateCudaContextManager does have an additional parameter PxProfilerCallback, that is required in order to get profiling events from the GPU dll.
* FastXml project is now compiled as OBJECT on win platforms and is linked into PhysXExtensions library.
* Removed PxArticulationBase::getType(), PxArticulationBase::eReducedCoordinate, PxArticulationBase::eMaximumCoordinate and added PxConcreteType::eARTICULATION_REDUCED_COORDINATE, PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE.
        
    

## Rigid Bodies
    
### Added:
        
* Immediate mode API for reduced-coordinates articulations.
* Immediate mode API for the temporal Gauss Seidel (TGS) solver .
* Compute dense Jacobian matrix for the reduced-coordinates articulations.
* GPU acceleration for reduced-coordinates articulations with PGS solver.
* GPU acceleration for reduced-coordinates articulations with TGS solver (experimental).
        
### Changed:
        
* PxSimulationStatistics::nbDynamicBodies does not include kinematics any longer. Instead they are covered in new nbKinematicBodies counter.
        
### Fixed:
        
* Fixed speculative CCD optimization with sleeping bodies.
* Fixed the overlap termination condition in the GJK code for sphere primitive. 
* Fixed a bug in the face selection algorithm for paper thin box overlapped with capsule. 
* Fixed a contact recycling issue with PCM contact gen. 
* Fixed an issue with articulation when removing and adding links to the articulation. 
        
    

## Serialization
    
### Added:
        
* PxSerialization::serializeCollectionToBinaryDeterministic, convenience function to post-process binary output with PxBinaryConverter for determinism. For achieving determinism, the checked build needs to be used.
* Support for binary and xml serialization for PxArticulationReducedCoordinate.
        
### Fixed:
        
* PxBinaryConverter can now produce deterministic output, independent of the runtime environment the objects have been serialized in. For achieving determinism, the checked build needs to be used for serializing collections.
        
### Changed:
        
* PX_BINARY_SERIAL_VERSION has been changed to a global unique identifier string. PX_PHYSICS_VERSION is no longer part of binary data versioning.
        
    



# v4.0.0.25635910
January 2019

## General
    
### Fixed:
        
* Fixed issue in PxBinaryConverter::convert that could corrupt platform re-targeting of convex meshes with more than 127 vertices.
* GenerateProject scripts should now also work when not called from PhysX directory.
* GenerateProject script will now create correct compiler/ directory on Linux based systems.
* Removed /Wall from MSVC compilers.
* Fixed CMake install, added missing cudacontextmanager files.
* Fixed binary serialization of actors in aggregates without serialization of the containing aggregate.
        
### Removed:
        
* CharacterKinematic API export/import macros have been removed.
        
### Added:
        
* Support for Linux samples has been added.
* PxConfig.h include file will be generated during generate projects script. Including this file in your project will ensure that required defines (like PX_PHYSX_STATIC_LIB) are set.
        
### Changed:
        
* PX_FOUNDATION_API was moved to PhysX and uses PX_PHYSX_STATIC_LIB define as the rest of the SDK.
* PxAssertHandler moved from PxShared to PhysX and marked as deprecated.
* PxShared does use PX_SHARED_ASSERT instead of PX_ASSERT which is used just in the PhysX SDK and uses PxAssertHandler.
        
    



# v4.0
December 2018

## Supported Platforms

<table cellspacing=3>
    <tr>
        <th width="24" />
        <th align="left">Runtime</th>
        <th width="18" />
        <th align="left">Development</th>
    </tr>
    <tr>
        <td />
        <td>Apple iOS (tested on 12.1)</td>
        <td />
        <td>Xcode (tested with 10.1)</td>
    </tr>
    <tr>
        <td />
        <td>Apple macOS (tested on 10.13)</td>
        <td />
        <td>Xcode (tested with 10.1)</td>
    </tr>
    <tr>
        <td />
        <td>Google Android ARM (tested with API Level 19 - KITKAT)</td>
        <td />
        <td>NDK r13b</td>
    </tr>
    <tr>
        <td />
        <td>Linux (tested on Ubuntu 16.04), GPU acceleration: display driver and GPU supporting CUDA 10 / CUDA ARCH 3.0</td>
        <td />
        <td>Clang (tested with 3.8)</td>
    </tr>
    <tr>
        <td />
        <td>Microsoft Windows, GPU acceleration: display driver and GPU supporting CUDA 10 / CUDA ARCH 3.0</td>
        <td />
        <td>Microsoft Visual Studio 2013, 2015, 2017</td>
    </tr>
    <tr>
        <td />
        <td>Microsoft XBox One*</td>
        <td />
        <td></td>
    </tr>
    <tr>
        <td />
        <td>Nintendo Switch*</td>
        <td />
        <td></td>
    </tr>
    <tr>
        <td />
        <td>Sony Playstation 4*</td>
        <td />
        <td></td>
    </tr>
</table>

\* Console code subject to platform NDA not available on GitHub.  Developers licensed by respective platform owners please contact NVIDIA for access.

## Changes and Resolved Issues


<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>



## General
    
### Added:
        
* New Temporal Gauss Seidel (TGS) solver offering a new level of simulation accuracy.
* New Reduced Coordinate Articulation feature with no relative positional error and realistic actuation.
* New automatic multi-broadphase (ABP) providing better out of the box performance for many use cases.
* New BVH structure supporting better performance for actors with many shapes.
        
### Removed:
        
* PhysX Particle feature.
* PhysX Cloth feature.
* The deprecated active transforms feature has been removed. Please use active actors instead.
* The deprecated multi client behavior feature has been removed.
* The deprecated legacy heightfields have been removed.
        
### Changed:
        
* The PhysX SDK build system is now based on CMake generated build configuration files. For more details, please refer to the PhysX SDK 4.0 Migration Guide.
* The Linux build has been changed to produce static as opposed to shared libraries. The compiler was switched from GCC to Clang.
* The PxShared library contains functionality shared beyond the PhysX SDK. It has been streamlined to a minimal set of headers. The PxFoundation singleton has been moved back to the PhysX SDK, as well as the task manager, CUDA context manager and PhysX Visual Debugger (PVD) functionality.
* PhysXDelayLoadHook and PhysXGpuLoadHook have been simplified, and PxFoundationDelayLoadHook has been removed.
        
    

## Rigid Bodies
    
### Added:
        
* A new broadphase implementation has been added. See PxBroadPhaseType::eABP for details. This is now the default broadphase implementation.
* TGS: A new rigid body solver, which can produce improved convergence compared to the default rigid body solver.
* Optional torsional friction model to simulate rotational friction when there is just a single point of contact.
* A flag to enable friction constraints to be processed every frame has been added
* PxContactJoint added to represent contacts in inverse dynamics. Not intended for use in simulation.
* A missing PxShape::getReferenceCount() function has been added.
* A new flag has been added to PxMaterial to improve friction accuracy. The flag is disabled by default to maintain legacy behavior.
        
### Removed:
        
* PxVisualizationParameter::eDEPRECATED_BODY_JOINT_GROUPS has been removed.
* PxSceneDesc::maxNbObjectsPerRegion has been removed.
* PxRigidActor::createShape() has been removed. Please use PxPhysics::createShape() or PxRigidActorExt::createExclusiveShape() instead
* The deprecated mass parameter in PxTolerancesScale has been removed.
* PxSceneFlag::eDEPRECATED_TRIGGER_TRIGGER_REPORTS has been removed.
        
### Changed:
        
* Aggregates can now contain more than 128 actors.
* Switching a kinematic object to dynamic does not automatically wake up the object anymore. Explicit calls to PxRigidDynamic::wakeUp() are now needed.
* Switching a kinematic object to dynamic re-inserts the object into the broadphase, producing PxPairFlag::eNOTIFY_TOUCH_FOUND events instead of PxPairFlag::eNOTIFY_TOUCH_PERSISTS events.
* PxConvexMeshGeometryFlag::eTIGHT_BOUNDS is now enabled by default for PxConvexMeshGeometry.
* The default max angular velocity for rigid bodies has been changed, from 7 to 100.

        
    


## Extensions
    
### Added:
        
* PxD6Joint now supports per-axis linear limit pairs.
* Added PxSphericalJoint::getSwingYAngle and PxSphericalJoint::getSwingZAngle.
* Added PxD6Joint distance limit debug visualization.
* Added PxD6JointCreate.h file with helper functions to setup the D6 joint in various common configurations.
* Added pyramidal swing limits to the D6 joint.
        

### Removed:
        
* PxComputeHeightFieldPenetration has a new signature.
* PxComputeMeshPenetration has been removed. Use PxComputeTriangleMeshPenetration instead.
        

### Changed:
        
* PxRevoluteJoint now properly supports a -PI*2 to +PI*2 range for its limits, and the accuracy of limits has been improved. In order to use extended limit ranges, PxConstraintFlag::eENABLE_EXTENDED_LIMITS must be raised on the constraint.
* PxD6Joint now properly supports a -PI*2 to +PI*2 range for its twist limit, and the accuracy of the twist limit has been improved. In order to use extended limit ranges, PxConstraintFlag::eENABLE_EXTENDED_LIMITS must be raised on the constraint.
* The accuracy of the D6 joint swing limits has been improved.
* PxDistanceJoint does now always insert constraint row, this change does increase the limit precision.
* PxDistanceJoint::getDistance does not anymore return squared distance.
* PxD6Joint::setDriveVelocity, PxD6Joint::setDrivePosition and PxRevoluteJoint::setDriveVelocity have now additional parameter autowake, which will wake the joint rigids up if true (default behavior).
* Joint shaders now take a bool to define whether to use extended joint limits or not.
* Joint shaders must now provide the cA2w and cB2w vectors, defining the world-space location of the joint anchors for both bodies.
        

### Deprecated:
        
* PxD6Joint::getTwist() has been deprecated. Please use PxD6Joint::getTwistAngle() now.
* The previous PxD6Joint::setLinearLimit() and PxD6Joint::getLinearLimit() functions (supporting a single linear limit value) have been deprecated. Please use PxD6Joint::setDistanceLimit() and PxD6Joint::getDistanceLimit() instead. Or you can also use the new PxD6Joint::setLinearLimit() and PxD6Joint::getLinearLimit() functions, which now support pairs of linear limit values.
        
    

## Articulations
    
### Added:
        
* New reduced coordinate articulation implementation, supporting a wider range of joint types, more accurate drive model, inverse dynamics and joint torque control.
* PxArticulationJoint::getParentArticulationLink and PxArticulationJoint::getChildArticulationLink has been added.
        
    

## Scene queries
    
### Removed:
        
* PxHitFlag::eDISTANCE has been removed.
* The PxVolumeCache feature has been removed.
* The PxSpatialIndex feature has been removed.
* The deprecated PxSceneFlag::eSUPPRESS_EAGER_SCENE_QUERY_REFIT has been removed.
        
    

## Cooking
    
### Added:
        
* PxBVHStructure added, it computes and stores BVH structure for given bounds. The structure can be used for actors with large amount of shapes to perform scene queries actor centric rather than shape centric. For more information please see guide or snippets.
        
### Removed:
        
* PxPlatform enum has been removed. PhysX supported platforms all share the same endianness
* The deprecated PxCookingParams::meshCookingHint and PxCookingParams::meshSizePerformanceTradeOff parameters have been removed.
* The deprecated PxGetGaussMapVertexLimitForPlatform has been removed, use PxCookingParams::gaussMapLimit instead.
* The deprecated PxConvexMeshCookingType::eINFLATION_INCREMENTAL_HULL and PxCookingParams::skinWidth have been removed.
* PxBVH34MidphaseDesc::numTrisPerLeaf has been renamed to PxBVH34MidphaseDesc::numPrimsPerLeaf
        
    


# v3.4.2.25354359
December 2018

## General
    
### Changed:
        
* Changed GitHub distribution to BSD license.
        
    

## Supported Platforms

<table cellspacing=3>
    <tr>
        <th width="24" />
        <th align="left">Runtime</th>
        <th width="18" />
        <th align="left">Development</th>
    </tr>
    <tr>
        <td />
        <td>Apple iOS (tested on 12.1)</td>
        <td />
        <td>Xcode (tested with 10.1)</td>
    </tr>
    <tr>
        <td />
        <td>Apple macOS (tested on 10.13)</td>
        <td />
        <td>Xcode (tested with 10.1)</td>
    </tr>
    <tr>
        <td />
        <td>Google Android ARM (tested with API Level 16, Android 4.1 - JELLY_BEAN)</td>
        <td />
        <td>NDK r13b-win32</td>
    </tr>
    <tr>
        <td />
        <td>Linux (tested on Ubuntu 16.04, GPU acceleration: NVIDIA Driver version R361+ and CUDA ARCH 3.0)</td>
        <td />
        <td>GCC (tested with 4.8)</td>
    </tr>
    <tr>
        <td />
        <td>Microsoft Windows XP or later (NVIDIA Driver version R304 or later is required for GPU acceleration)</td>
        <td />
        <td>Microsoft Visual Studio 2012, 2013, 2015</td>
    </tr>
    <tr>
        <td />
        <td>Microsoft XBox One*</td>
        <td />
        <td></td>
    </tr>
    <tr>
        <td />
        <td>Nintendo Switch*</td>
        <td />
        <td></td>
    </tr>
    <tr>
        <td />
        <td>Sony Playstation 4*</td>
        <td />
        <td></td>
    </tr>
</table>

\* Console code subject to platform NDA not available on GitHub.  Developers licensed by respective platform owners please contact NVIDIA for access.



# v3.4.2.25256367
November 2018

## General
    
### Changed:
        
* A couple of serialization write functions have been optimized.
* Rtree cooking has been slightly optimized.
* Renamed PxSerializer::requires to PxSerializer::requiresObjects due to an erroneous clash with C++20 keyword with apple-clang.
        
### Fixed:
### Moved external vector math includes out of PhysX namespaces.
    
    

## Rigid Bodies
    
### Fixed:
        
* Fixed an incorrect trigger behavior when a trigger was removed and inserted within the same frame.
        
    

## Scene query
    
### Fixed:
        
* Fixed a bug in BVH34. Raycasts could fail on binary deserialized BVH34 triangle meshes.
        
    



# v3.4.2.24990349
September 2018

## General
    
### Fixed:
        
* PxMeshQuery::getTriangle adjacency information for heightfield geometry fixed.
* Removed PxSetPhysXGpuDelayLoadHook from API. Delay loaded dynamically linked library names are now provided through PxSetPhysXDelayLoadHook.
* Fixed a source of non-determinism with GPU rigid bodies.
        
    

## Rigid Bodies
    
### Fixed:
        
* Fixed a divide by zero bug when gjk is trying to calculate the barycentric coordinate for two identical/nearly identical points. 
* Fixed an incorrect mesh index reported in contact buffer when stabilization flag was used. 
        
    



# v3.4.2.24698370
August 2018

## Rigid Bodies
    
### Fixed:
        
* Fixed a crash bug when EPA's edge buffer overflow. 
* GPU rigid bodies fixed for Volta GPUs.
        
### Added:
        
* Aggregate broad phase now runs in parallel
        
    

## Cooking
    
### Fixed:
        
### Added:
        
* PxHeightField::getSample has been added.
        
    

## Character controller
    
### Fixed:
        
* Capsule controller with a very small height could degenerate to spheres (with a height of exactly zero) far away from the origin, which would then triggers errors in Debug/Checked builds. This has been fixed.
* The triangle array growing strategy has been changed again to prevent performance issues when tessellation is used. Memory usage may increase in these cases.
* Some internal debug visualization code has been disabled and a crash in it has been fixed.
        
    

## Scene query
    
### Fixed:
        
* Fixed possible buffer overrun when PxPruningStructure was used.
* Raycasts against a heightfield may have missed if a large distance was used.
* Sq raycasts against heightfield or triangle mesh could return a mildly negative values, this has been fixed.
        
    



# v3.4.2.24214033
May 2018

## General
    
### Fixed:
        
* Fixed clang 7 unionCast issues.
* Fixed the binary meta data in PhysX_3.4/Tools/BinaryMetaData for conversion of vehicles.
        
    

## Rigid Bodies
    
### Deprecated:
        
* PxSceneFlag::eENABLE_KINEMATIC_STATIC_PAIRS and PxSceneDesc::eENABLE_KINEMATIC_PAIRS have been deprecated. Use the new PxPairFilteringMode parameters in PxSceneDesc instead.
        
### Fixed:
        
* A sequence of shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false) / shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true) without simulation calls in-between could produce errors in the broadphase. This has been fixed.
* Fixed a bug in the broadphase (SAP) that got recently introduced in SDK 3.4.1.23933511. It should only have generated a few false positives (more overlaps than strictly necessary).
* Fixed a bug in PxShape::checkMaterialSetup.
* Fixed intermittent crash with GPU rigid bodies when materials were destroyed.
* Fixed bug where setting maxImpulse to 0 on CCD contact modification meant contact was not reported in the frame's contact reports.
        
    

## Cooking
    
### Fixed:
        
* Big convex mesh serialization used together with insertion callback stored incorrect memory for big convex data. This has been fixed.
        
    

## Scene query
    
### Fixed:
        
* Fixed a bug in extended bucket pruner, when a pruning structure was added and immediatelly released.
        
    



# v3.4.1.23933511
April 2018

## General
    
### Added:
        
* Added snippet for deformable meshes, added section in the guide for them.
        
    

## Rigid Bodies
    
### Fixed:
        
* PxTriangleMesh::refitBVH() was crashing with PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE. This has been fixed.
* SQ-only shapes contained in aggregates could result in crashes or memory corruption when removed from the aggregate. This has been fixed.
* Assert no longer fired if a dynamic body contacting a static is converted to kinematic with kinematic-static pairs enabled.
* Assert reporting broad phase as being "inconsistent" could fire when using speculative CCD when sleeping objects were activated.
        
    

## Cooking
    
### Fixed:
        
* Convex hull cooking could have produced hulls with vertices too far from convex hull planes, this has been fixed.
        
### Changed:
        
* PxCooking::createTriangleMesh does now take additional output parameter PxTriangleMeshCookingResult::Enum.
* PxCooking::createConvexMesh does now take additional output parameter PxConvexMeshCookingResult::Enum.
        
    

## Scene query
    
### Changed:
        
* PxSceneFlag::eSUPPRESS_EAGER_SCENE_QUERY_REFIT has been marked as deprecated. Was replaced with PxSceneQueryUpdateMode enum, if this new enum is set the flag gets ignored.
        
### Added:
        
* PxSceneQueryUpdateMode was added to control work done during fetchResults.
* PxScene::sceneQueriesUpdate, PxScene::checkQueries and PxScene::fetchQueries were added to run separate scene query update, see manual for more details.
        
    



# v3.4.1.23584284
February 2018

## General
    
### Fixed:
        
* PhysX sometimes froze in a spinlock after certain sequences of read & write locks. This has been fixed.
        
    

## Scene queries
    
### Fixed:
        
* Raycasts against heightfields were sometimes missing hits for vertical rays were located exactly at the heightfield's boundaries. This has been fixed.
        
    

## Rigid Bodies
    
### Fixed:
        
* Avoid edge-face collisions on boundary edges when eNO_BOUNDARY_EDGES flag is raised on heightfield when using PCM and unified HF collision.
        
    



# v3.4.1.23472123
January 2018

## General
    
### Added:
        
* Visual Studio 2017 15.5.1 and newer is now supported. Samples are currently not supported with Visual Studio 2017.
        

### Removed:
        
* Visual Studio 2012 support is discontinued.
        
    

## Cooking
    
### Fixed:
        
* Cooked mesh structures contained a mix of little-endian and big-endian data (the midphase structures were always saved as big-endian). This made loading of cooked files slower than necessary. This has been fixed.
        
    

## Scene queries
    
### Fixed:
        
* Buffered moves were sometimes not properly taken into account by scene queries, leading to invalid results for one frame. This has been fixed.
* Pruning structure failed to build when actor had more shapes. This has been fixed.
        
    



# v3.4.1.23173160
November 2017

## Extensions
    
### Fixed:
        
* An issue with CCD sweeps against meshes that could potentially lead to the earliest impact not being detected has been fixed.
        
    



# v3.4.1.23131702
November 2017

## General
    
### Fixed:
        
* A bug in the management of internal interaction objects has been fixed.
        
    

## Extensions
    
### Fixed:
        
* A regression in prismatic constraint stability introduced in PhysX 3.4.1 has been fixed.
* PxRevoluteJoint::getAngle(), PxD6Joint::getTwist(), PxD6Joint::getSwingYAngle() and PxD6Joint::getSwingZAngle() did not always return the correct angle. This problem has been fixed.
* The "double cone" case of the D6 joint had errors both in the debug visualization and the code dealing with limits. This has been fixed.
* The debug visualization of the D6 joint in the twist case did not properly color-code the angular limits. This has been fixed.
* The debug visualization of distance joints has been fixed.
* The debug visualization of prismatic joints has been fixed.
* The debug visualization of revolute joints has been fixed. It now renders active limits properly (in red when the limit is active, grey otherwise).
* Proper visualization flags are now passed to the PxConstraintVisualize function. Previously all available flags were active, even if PxVisualizationParameter::eJOINT_LOCAL_FRAMES and/or PxVisualizationParameter::eJOINT_LIMITS were set to zero.
* PxRevoluteJoint::getVelocity has been fixed.
        
    

## Scene queries
    
### Fixed:
        
* Sweep queries using the eMTD flag could generate incorrect normals in sphere/sphere, sphere/capsule or capsule/capsule cases, when the objects were exactly overlapping each-other. This has been fixed.
* Sweep convex mesh vs heightfield queries using the eMTD flag did not fill correctly returned faceIndex. This has been fixed.
        
    



# v3.4.1
September 2017

## Changes and Resolved Issues


<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>



## General
    
### Deprecated:
        
* The PhysX cloth feature has been deprecated.
        
### Changed:
        
* The meaning of PxVisualizationParameter::eCULL_BOX has changed. It is now used to visualize the current culling box, while it was previously used to enable or disable the feature. Please simply use PxScene::setVisualizationCullingBox() to enable the feature from now on.
* PxVisualizationParameter::eBODY_JOINT_GROUPS has been deprecated.
* Performance of setCMassLocalPose function has been improved.
        
### Added:
        
* PhysXGpu: Added warnings for the case when the dynamic gpu library fails to load.
        
    

## Rigid Bodies
    
### Fixed:
        
* A potential crash when calling detachShape has been fixed.
* Fixed assert that fired if application switched a body from dynamic to kinematic, then queried kinematic target without setting one. This assert only fired if the modifications overlapped simulation so were buffered.
* PxArticulation::getStabilizationThreshold() and PxArticulation::setStabilizationThreshold() were accessing the sleep threshold instead of the stabilization threshold. This has been fixed.
* Fixed an internal edge bug in PCM sphere vs mesh code
* Make sure sphere vs sphere and sphere vs box in PCM contact gen generate contacts consistently on the second body when the sphere center is contained in the other shape
        
### Changed:
        
* Improved convex vs mesh contact generation when using GPU rigid bodies. Requires the mesh to be recooked.
* Improved convex vs convex contact generation when using GPU rigid bodies.
* Reduced memory footprint of GPU triangle meshes significantly. Requires the mesh to be recooked.
        
### Added:
        
* Support for modifying friction and restitution coefficients has been added to contact modification. 
* Added PxRigidBodyFlag::eENABLE_CCD_MAX_CONTACT_IMPULSE to enable maxContactImpulse member of PxRigidBody to be used in CCD. This is disabled by default. It is useful in some circumstances, e.g. shooting a small ball through a plate glass window and triggering it to break, but it can also result in behavioral artifacts so it is disabled by default.
* Added PxSceneDesc::solverOffsetSlop. This is defaulted to a value of 0. A positive, non-zero value defines a tolerance used in the solver below which a contacts' offset from the COM of the body is negligible and therefore snapped to zero. This clamping occurs in a space tangential to the contact or friction direction. This is aimed at pool or golf simulations, where small numerical imprecision in either contact points or normals can lead to balls curving slightly when there are relatively high angular velocities involved.
* Added PxConvexMeshGeometry::maxMargin. This allows the application to tune the maximum amount by which PCM collision detection will shrink convex shapes in contact generation. This shrinking approach leads to some noticeable clipping around edges and vertices, but should not lead to clipping with face collisions. By default, the mesh is shrunk by an amount that is automatically computed based on the shape's properties. This allows you to limit by how much the shape will be shrunk. If the maxMargin is set to 0, then the original shape will be used for collision detection. 
        

    

## Scene queries
    
### Fixed:
        
* A rare invalid memory read that could lead to incorrect sweep results in case of an initial overlap has been fixed.
        
    

## Serialization
    
### Fixed:
        
* Binary serialization didn't preserve PxConstraintFlags, e.g. projection flags.
* Xml serialization failed if a shape referencing a PxTriangleMesh was added to another (dependent) collection.
        


# v3.4.0.22387197
June 2017

## Changes and Resolved Issues


<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>



## Cooking
    
### Fixed:
        
* Fixed issue when PxConvexFlag::e16_BIT_INDICES was used together with PxConvexFlag::eCOMPUTE_CONVEX.
* Fixed issue in convex hull cooking when postHullMerge was not executed.
* Fixed crash in CCD when using bodies with 0 mass.
        
    

## Rigid Bodies
    
### Fixed:
        
* Fixed behavioral differences when comparing the results of a given scene to the results when simulating a subset of the islands in that scene. In order for this case to be deterministic, it is necessary to raise PxSceneFlag::eENABLE_ENHANCED_DETERMINISM. 
        

### Added:
        
* Introduced maxBiasCoefficient in PxSceneDesc to be able to limit the coefficient used to scale error to produce the bias used in the constraint solver to correct geometric error. The default value is PX_MAX_F32 and, therefore, a value of 1/dt will be used. This value can be useful to reduce/remove jitter in scenes with variable or very small time-steps.</i>

# v3.4.0.22121272
May

## Changes and Resolved Issues


<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>



## Rigid Bodies
    
### Fixed:
        
* Fixed a bug in convex vs convex PCM contact gen code. There were cases in which full contact gen should have been triggered but was not. 
* Fixed a jittering bug on both GPU/CPU codepath in PCM contact gen code. This is due to the contact recycling condition not considering the toleranceLength. 
* Fixed a bug causing simulation to behave incorrectly when greater than 65536 bodies were simulated.
        
    

## Scene queries
    
### Fixed:
        
* A rare crash that could happen with sphere overlap calls has been fixed.
* Fixed a stack corruption in CCD contact modification callback.
* Fixed a bug where external forces were not cleared correctly with PxArticulations.
        
    

## Cooking
    
### Changed:
        
* Convex hull cooking now reuses edge information, perf optimization.
* PxCooking API is now const if possible.


# v3.4.0.22017166
April 2017

## Changes and Resolved Issues


<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>



## General
    
### Added:
        
* PxConvexMeshGeometry::maxMargin has been added. This will let the application limit how much the shape is shrunk in GJK when using by PCM contact gen
        
    

## Rigid Bodies
    
### Fixed:
        
* Fixed a bug with joint breaking where sometimes joints would not break as expected.
* Fix a race condition between cloth/particle/trigger interactions and the parallel filtering of rigid body interaction.
* GPU rigid body feature now issues a performance warning in checked build if feature is enabled but PCM contact gen is not.
* Fixed a bug with applying external force/torque to a body in buffered insertion stage.
* Fixed a bug with CCD involving rotated static mesh actors.
* Fixed a memory leak in CCD.
        
### Changed:
        
* Optimizations for GPU rigid body feature, including reduced memory footprint and improvements to performance spikes when many objects are woken in a single frame.
        
    

## Midphase
    
### Fixed:
        
* Fix Crash in BV4 code (serialization bug).
        
    

## Cooking
    
### Fixed:
        
* Fix endless loop in convex hull cooking.
        

# v3.4.0.21821222
March 2017

## Supported Platforms

## Runtime
        
* Apple iOS
* Apple Mac OS X
* Google Android ARM (version 2.2 or later required for SDK, 2.3 or later required for snippets)
* Linux (tested on Ubuntu)
* Microsoft Windows XP or later (NVIDIA Driver version R304 or later is required for GPU acceleration)
* Microsoft XBox One
* Nintendo Switch
* Sony Playstation 4
        
## Development
        
* Microsoft Windows XP or later
* Microsoft Visual Studio 2012, 2013, 2015
* Xcode 8.2
        

## Known Issues

        

## Changes and Resolved Issues


<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>



## General
    
### Fixed:
        
* A rare crash happening in the debug visualization code has been fixed.
        
    

## Rigid Bodies
    
### Fixed:
        
* Fixed a bug in PCM capsule vs plane contact gen.
* A crash happening with more than 64K interactions has been fixed.
* Fixed an island management issue in CCD when using multiple CCD passes.
* Fixed a bug with GPU rigid bodies with non-simulation scene query-only shapes.
* Fixed a bug in convex vs convex PCM contact gen code. There were cases in which full contact gen should have been triggered but was not. 
* Fixed a jittering bug on both GPU/CPU codepath in PCM contact gen code. This is due to the contact recycling condition not considering the toleranceLength. 
        
### Added:
        
* getFrozenActors has been added to allow application queries the frozen actors
* PxRigidDynamic::setKinematicSurfaceVelocity has been added, permitting the user to set a persistent velocity on a kinematic actor which behaves like a conveyor belt
* PxSceneDesc::solverOffsetSlop added. This defines a threshold distance from a body's COM under which a contact will be snapped to the COM of the body inside the solver along any principal component axis
        
    

## Scene queries
    
### Fixed:
        
* A bug in the BVH34 overlap code sometimes made PxMeshOverlapUtil::findOverlap assert (reporting an incorrect buffer overflow). This has been fixed.
* Fix a bug in the case of two primitives just touching in sweep with eMTD flag on. 



# v3.4
February 2017

## Supported Platforms

## Runtime
        
* Apple iOS
* Apple Mac OS X
* Google Android ARM (version 2.2 or later required for SDK, 2.3 or later required for snippets)
* Linux (tested on Ubuntu)
* Microsoft Windows XP or later (NVIDIA Driver version R304 or later is required for GPU acceleration)
* Microsoft XBox One
* Nintendo Switch
* Sony Playstation 4
        
## Development
        
* Microsoft Windows XP or later
* Microsoft Visual Studio 2012, 2013, 2015
* Xcode 8.2
        

## Known Issues

        

## Changes and Resolved Issues
<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>

## General
        
* Added:
            
  * nbAggregates, nbArticulations, nbDiscreteContactPairsTotal, nbDiscreteContactPairsWithCacheHits and nbDiscreteContactPairsWithContacts have been added to PxSimulationStatistics.
  * PxSceneLimits::maxNbBroadPhaseOverlaps has been added.
  * A new midphase has been added. See PxMeshMidPhase enum for details.
  * Midphase descriptor has been added. See PxMidphaseDesc for details.
  * PxHeightField::getTimestamp() has been added
  * PxCloneShape has been added to enable cloning of shapes.
  * acquireReference() has been added to increment the reference count of shapes, materials, triangle meshes, convex meshes heightfields and cloth fabrics.
  * The global filter shader data block can now be set through PxScene::setFilterShaderData().
  * PxGpuLoadHook and PxSetPhysXGpuLoadHook has been added to support loading of GPU dll with a different name than the default.
  * A new profile zone has been added for debug visualization.
            
* Changed:
            
  * PxMathUtils.h has moved from include/common to include/foundation
  * GPU support requires SM2.x (Fermi architecture, GeForce 400 series) or later hardware. SM1.x is no longer supported.
  * Windows XP 64-bit and Windows Vista no longer support GPU acceleration. Windows XP 32-bit still supports GPU acceleration.
  * PxTaskManager::createTaskManager requires error callback and does not accept SPU task manager.
  * PxCreatePhysics and  PxCreateBasePhysics now take an optional pointer to an  physx::PxPvd instance.  
  * PxConstraintConnector::updatePvdProperties now takes an optional pointer to an physx::pvdsdk::PvdDataStream instance.
  * PxInitExtensions now takes an optional pointer to an physx::PxPvd instance.
  * Shared objects (triangle mesh, convex mesh, heightfield, material, shape, cloth fabric) no longer issue an eUSER_RELEASE event when their release() method is called
  * PxBase::isReleasable() is now a property only of the type of an object, not the object state. In particular, isReleasable() returns true for PxShape objects whose only counted reference belongs to their owning actor.
  * PxCollection::releaseObjects() now calls release() even on shapes whose only counted reference belongs to their owning actor. An optional parameter releaseExclusiveShapes, which defaults to true, has been added to this method to assist with common scenarios in which all shapes are created with the deprecated method PxRigidActor::createShape() or its replacement PxRigidActorExt::createExclusiveShape()
  * Negative mesh scale is now supported for PxTriangleMeshGeometry. Negative scale corresponds to reflection and scale along the corresponding axis. In addition to reflection PhysX will flip the triangle normals.
  * PxDelayLoadHook is now inherited from PxFoundationDelayLoadHook. PxFoundation dll and PxPvdSDK dll are now delay loaded inside the SDK, their names can be provided through the delay load hook.
            
* Removed:
            
  * Sony Playstation 3 is not supported any longer. Any related APIs have been removed.
  * Microsoft XBox 360 is not supported any longer. Any related APIs have been removed.
  * Nintendo Wii U is not supported any longer. Any related APIs have been removed.
  * Sony Playstation Vita is not supported any longer. Any related APIs have been removed.
  * Visual Studio 2010 is not supported any longer.
  * Microsoft Windows RT is not supported any longer.
  * Google Android X86 is not supported any longer.
  * PhysX Samples are not supported anymore except on Microsoft Windows.
  * Linux 32-bit no longer support GPU acceleration. Linux 64-bit still supports GPU acceleration.
            
* Fixed:
            
  * PxScene::setFlag() does now properly send error messages in CHECKED builds if a non-mutable scene flag gets passed in.
  * Fixed a bug in force threshold based contact reports, which caused events to be lost.
  * Fixed a bug in aggregates that led to a crash when rigid bodies are added to an aggregate after removing all rigid bodies from an aggregate. This only occurred with aggregates that were added to the scene and with rigid bodies that had shapes attached.
  * Fixed a bug where non-breakable joints could break, leading to a crash. 
  * Fixed RepX load of kinematic rigid bodies with mesh shapes.
  * Fixed a divide by zero bug in SIMD distanceSegmentTriangle function.
  * Debug visualization for compound bounds (PxVisualizationParameter::eCOLLISION_COMPOUNDS) now works even when all the compounds' shapes have their debug viz flag (PxShapeFlag::eVISUALIZATION) disabled.
  * Double-buffering now works properly for the debug visualization culling box.
  * The default debug visualization culling box now works correctly with heightfields.
  * Rare crashes in PxShape::setGeometry() and PxShape::getMaterials() have been fixed.
  * A crash happening when doing an origin shift on a scene containing an empty aggregate has been fixed.
            
* Deprecated:
            
  * PxSceneLimits::maxNbObjectsPerRegion has been deprecated. It is currently not used.
  * PxComputeHeightFieldPenetration has a new signature, and the old one has been deprecated
  * PxComputeMeshPenetration has been deprecated. Use PxComputeTriangleMeshPenetration.
  * The PhysX particle feature has been deprecated.
  * PxTolerancesScale::mass has been deprecated.  It is currently not used.
  * PxActorClientBehaviorFlag has been marked as deprecated and will be removed in future releases.
            
* Removed deprecated API:
            
  * PxPairFlag::eCCD_LINEAR removed. Use PxPairFlag::eDETECT_CCD_CONTACT | PxPairFlag::eSOLVE_CONTACT instead.
  * PxPairFlag::eRESOLVE_CONTACTS removed. Use PxPairFlag::eDETECT_DISCRETE_CONTACT | PxPairFlag::eSOLVE_CONTACT instead.
  * PxTriangleMeshFlag::eHAS_16BIT_TRIANGLE_INDICE removed. Use PxTriangleMeshFlag::e16_BIT_INDICES instead.
  * PxTriangleMeshFlag::eHAS_ADJACENCY_INFO removed. Use PxTriangleMeshFlag::eADJACENCY_INFO instead.
  * PxTriangleMeshDesc::convexEdgeThreshold removed.
  * PxSceneQueryFlag renamed to PxHitFlag.
  * PxHitFlag::eDIRECT_SWEEP renamed to PxHitFlag::ePRECISE_SWEEP.
  * PxHitFlag::eIMPACT renamed to PxHitFlag::ePOSITION.
  * PxSceneQueryHitType renamed to PxQueryHitType.
  * PxSceneQueryCache renamed to PxQueryCache.
  * PxSceneQueryFilterFlag renamed to PxQueryFlag.
  * PxSceneQueryFilterFlags renamed to PxQueryFlags.
  * PxSceneQueryFilterData renamed to PxQueryFilterData.
  * PxSceneQueryFilterCallback renamed to PxQueryFilterCallback.
  * PxScene::raycastAll,PxScene::raycastSingle,PxScene::raycastAny replaced by PxScene::raycast.
  * PxScene::overlapAll,PxScene::overlapAny replaced by PxScene::overlap.
  * PxScene::sweepAll,PxScene::sweepSingle,PxScene::sweepAny replaced by PxScene::sweep.
  * PxQuat, PxTranform, PxMat33, PxMat44 createIdentity and createZero removed. Use PxIdentity, PxZero in constructor.
  * PxJointType::Enum, PxJoint::getType() removed. Use PxJointConcreteType instead.
  * PxVisualDebugger removed. Use PxPvd instead.
  * PxControllerFlag renamed to PxControllerCollisionFlag.
  * PxCCTHit renamed to PxControllerHit.
  * PxCCTNonWalkableMode renamed to PxControllerNonWalkableMode.
  * PxControllerNonWalkableMode::eFORCE_SLIDING changed to PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING.
  * PxControllerDesc::interactionMode, groupsBitmask, callback removed.
  * PxController::setInteraction, getInteraction, setGroupsBitmask, getGroupsBitmask removed.
  * PxControllerManager::createController no longer needs PxPhysics and PxScene.
  * PxControllerFilters::mActiveGroups replaced with PxControllerFilters::mCCTFilterCallback.
  * PxSerialization::createBinaryConverter(PxSerializationRegistry&) changed to PxSerialization::createBinaryConverter().
  * PxConstraintDominance renamed to PxDominanceGroupPair.
  * PxScene::flush renamed to PxScene::flushSimulation.
  * PxClothFabric::getPhaseType removed.
  * PxCollection::addRequired removed.
  * PxRigidActor::createShape discontinued support for initial transform.
  * PxShape::resetFiltering removed.
  * PxParticleBase::resetFiltering removed.
  * PxSceneDesc::meshContactMargin removed.
  * PxSceneDesc::contactCorrelationDistance removed.
  * Indexing operators taking signed integers in PxVec3, PxVec4, PxMat33, PxMat44, PxStrideIterator have been removed.
  * PxHitFlag::ePOSITION, PxHitFlag::eDISTANCE and PxHitFlag::eNORMAL are now supported in PxMeshQuery::sweep function.
  * PxClothFlag::eGPU renamed to PxClothFlag::eCUDA.
  * PxActorTypeSelectionFlag/PxActorTypeSelectionFlags. Use PxActorTypeFlag/PxActorTypeFlags instead.
  * PxConstraintFlag::eDEPRECATED_32_COMPATIBILITY flag removed.
            
        

## PxShared
        
APEX 1.4 can now be used independently of PhysX. In order to achieve that a new shared code base was created called "PxShared". PhysX functionality such as common types, PxFoundation, the task infrastructure are now part of PxShared.
        

## Rigid Bodies
        
* Added:
            
  * An alternative simulation API has been introduced. This makes use of the following new functions: PsScene:collide(), PxScene::fetchCollision() and PxScene::advance(). Expected usage of these functions is illustrated in a new snippet SnippetSplitSim. This feature is also described in the manual in Section Simulation->SplitSim.
  * PxSceneFlag::eDEPRECATED_TRIGGER_TRIGGER_REPORTS has been introduced to re-enable the legacy behavior of trigger shape pairs sending reports. This flag and the corresponding legacy behavior will be removed in version 4.
  * The active actors feature has been added. See PxSceneFlag::eENABLE_ACTIVE_ACTORS.
  * Functionality to compute and manipulate mass, inertia tensor and center of mass of objects has been exposed in the new class PxMassProperties.
  * The option to exclude kinematics from the active actors/transforms list has been added. See PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS for details.
  * The integrated pose of dynamic rigid bodies can be accessed earlier in the pipeline through a new callback (see the API documentation for PxSimulationEventCallback::onAdvance() and PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW for details).
  * PxConvexMeshGeometryFlag::eTIGHT_BOUNDS has been added. See the user manual for details.
  * New split fetchResults methods introduced, see PxScene::fetchResultsBegin(), PxScene::fetchResultsFinish() and PxScene::processCallbacks(). This is intended to permit the application to parallelize the event notification callbacks.
  * New flag introduced to suppress updating scene query pruner trees inside fetchResults, see PxSceneFlag::eSUPPRESS_EAGER_SCENE_QUERY_REFIT. Instead, pruners will be updated during the next query.
  * Introduced GPU rigid body simulation support, see PxSceneFlag::eENABLE_GPU_DYNAMICS. GPU rigid body support requires SM3.0 or later.
  * Introduced a new GPU-accelerated broad phase. See PxBroadPhaseType::eGPU. GPU broad phase support requires SM3.0 or later.
  * Introduced a new enhanced determinism mode. See PxSceneFlag::eENABLE_ENHANCED_DETERMINISM. This provides additional levels of rigid body simulation determinism at the cost of some performance.
  * Introduced a new speculative contacts CCD approach. See PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD. This is a slightly cheaper, less robust solution to PxRigidBodyFlag::eENABLE_CCD. There is no need to turn CCD on the scene using PxSceneFlag::eENABLE_CCD or enable PxPairFlag::eDETECT_CCD_CONTACT with this CCD mode as it functions as an extension to the discrete time-stepping scheme. This form of CCD can be enabled on kinematic actors. 
  * New "immediate mode" API has been added. This exposes access to the low-level contact generation and constraint solver, which allows the application to use these PhysX low-level components to perform its own simulations without needing to populate and simulate a PxScene.
  * RigidDynamic lock flags added which permit the application to disallow rotation/translation of PxRigidDynamics around specific axes.
            
* Changed:
            
  * PxRigidStatic objects can now have zero shapes while being part of a scene.
  * PxContactPairFlag::eINTERNAL_HAS_FACE_INDICES is obsolete and has been removed.
  * PxConstraintFlag::eDEPRECATED_32_COMPATIBILITY was previously only implemented for spring constraints. It is now correctly implemented for equality constraints.
  * PxSceneFlag::eENABLE_PCM is enabled by default. This means PhysX uses PCM distance-based collision detection by default. 
  * Calls to PxRigidDynamic::setWakeCounter() following PxScene::collide() do now explicitly get taken into account in the subsequent call to PxScene::advance().
  * Calls to contact modification callbacks can be made from multiple threads simultaneously. Therefore, modification callbacks should must be thread-safe. 
  * Unified heightfield contact generation is now the default heightfield contact generation approach. This approach offers similar performance and behavior to contact generation with triangle meshes. Unified heightfields have no thickness because contact generation operates on triangles so objects may tunnel if CCD is not enabled.
  * When unified heightfield contact generation is in use, the bounds of heightfield shapes are no longer extruded by "thickness".
  * PxArticulationJoint::setTwistLimit and PxArticulationJoint::getTwistLimit were incorrectly documented with zLimit and yLimit in the wrong order.  The behavior of both functions remains unchanged but now they are correctly documented with zLimit and yLimit in the correct order. This is simply a clarification of the existing function behavior.
            
* Removed:
            
  * The deprecated class PxFindOverlapTriangleMeshUtil has been removed. Please use PxMeshOverlapUtil instead.
  * The deprecated flag PxConstraintFlag::eREPORTING has been removed. Force reports are now always generated.
  * The following deprecated simulation event flags have been removed: PxContactPairHeaderFlag::eDELETED_ACTOR_0, ::eDELETED_ACTOR_1, PxContactPairFlag::eDELETED_SHAPE_0, ::eDELETED_SHAPE_1, PxTriggerPairFlag::eDELETED_SHAPE_TRIGGER, ::eDELETED_SHAPE_OTHER. Please use the following flags instead: PxContactPairHeaderFlag::eREMOVED_ACTOR_0, ::eREMOVED_ACTOR_1, PxContactPairFlag::eREMOVED_SHAPE_0, ::eREMOVED_SHAPE_1, PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER, ::REMOVED_SHAPE_OTHER.
  * The deprecated method PxPhysics::createHeightField(const PxHeightFieldDesc&) has been removed. Please use PxCooking::createHeightField(const PxHeightFieldDesc&, PxPhysicsInsertionCallback&) instead. The insertion callback can be obtained through PxPhysics::getPhysicsInsertionCallback().
            
* Deprecated:
            
  * PxRigidActor::createShape() has been deprecated in favor of PxRigidActorExt::createExclusiveShape()
  * Trigger notification events for trigger-trigger pairs have been deprecated and will be omitted by default. See the 3.4 migration guide for more information.
  * The active transforms feature (PxSceneFlag::eENABLE_ACTIVETRANSFORMS) has been deprecated. Please use PxSceneFlag::eENABLE_ACTIVE_ACTORS instead. 
  * PxRegisterHeightFields has been modified to register unified heightfields, which are now the default implementation. PxRegisterLegacyHeightFields() has been added to register the legacy (deprecated) heightfield contact gen approach.
  * PxHeightFieldDesc::thickness has been deprecated, as the new unified height field (see PxRegisterUnifiedHeightFields()) does not support thickness any longer.
            
* Fixed:
            
  * The capsule-vs-heightfield contact generation had a rare bug where a vertical capsule standing exactly on a shared edge could fall through a mesh. This has been fixed.
  * The bounding box of a shape was not always properly updated when the contact offset changed.
  * Fixed a bug in the GJK sweep caused by lost precision in the math
  * Calls to PxScene::shiftOrigin() can crash when PxRigidDynamic actors with PxActorFlag::eDISABLE_SIMULATION are present.
  * Fixed a bug when PxShape::setMaterials was called with less materials than shape had before.
  * Fixed a bug in CCD that could lead to a hang in the simulation.
  * Fixed a bug in PCM mesh edge-edge check for the parallel case. 
  * Fixed a bug in CCD where contact modify callbacks could be called when the CCD did not detect a contact.
  * Fixed a bug with applying external force/torque to a body in buffered insertion stage.
  * A rare capsule-vs-mesh contact generation bug has been fixed.
  * A rare crash due to an invalid assert in the MBP broad-phase has been fixed. This error only affected debug & checked builds; release & profile builds were unaffected.
            
        

## Particles
        
* Gpu: Maxwell Optimizations
* The PhysX particle feature has been deprecated.
* Fixed particle collision issue with PxParticleBaseFlag::ePER_PARTICLE_COLLISION_CACHE_HINT (on by default). When particles collided against very dense triangle mesh areas an assert would be triggered or particles would leak through the triangle mesh. A workaround was to disable PxParticleBaseFlag::ePER_PARTICLE_COLLISION_CACHE_HINT.
        

## Cloth
        
* Continuous collision (PxClothFlag::eSWEPT_CONTACT) behavior has been optimized to reduce cloth sticking to collision shape.
* Added air resistance feature (see PxCloth::setWindVelocity(PxVec3), PxCloth::setWindDrag(PxReal), PxCloth::setWindLift(PxReal), PxClothFabricDesc::nbTriangles, PxClothFabricDesc::triangles.
        

## Serialization
        
* Fixed:
            
  * PxTriangleMesh instances with adjacency information were not correctly initialized when created with cooking.createTriangleMesh. This caused a crash when converting the binary serialized triangle mesh data. 
            
        

## Character controller
        
* Added:
            
  * Profile zones have been added for the character controller.
  * Added PxControllerDesc::registerDeletionListener boolean defining if deletion listener for CCT should be registered.
            
* Fixed:
            
  * Character controllers cannot stand on dynamic triggers anymore.
  * Fixed: the capsule-vs-sphere sweep now returns a normal in the correct direction.
  * Fixed a bug where CCT shapes initially overlapping static geometry would be moved down by an incorrect amount (the length of the step offset).
  * The overlap recovery module now works against kinematic objects.
            
        

## Vehicles
        
* Added:
            
  * Anti-roll suspension has been added.    The class PxVehicleAntiRollBar and the functions PxVehicleWheelsSimData::setAntiRollBar, PxVehicleWheelsSimData::getAntiRollBar, PxVehicleWheelsSimData::getNbAntiRollBars allow anti-roll bars to be configured and queried.
                
  * A new function PxVehicleSuspensionSweeps has been introduced.  This sweeps the PxShape that represents the wheel along the suspension direction. The hit planes resulting from the sweep are used as driving surfaces similar to those found by PxVehicleSuspensionRaycasts.
                
  * A new snippet SnippetVehicleContactMod has been added.  This snippet demonstrates how to use sweeps and contact modification to allow the wheel's volume to fully interact with the environment.
                
  * A new function PxVehicleModifyWheelContacts has been introduced.  This function analyses contact in the contact modification callback and rejects contact points that represent drivable surfaces.
  * A new function PxVehicleSetMaxHitActorAcceleration has been introduced.  This function sets the maximum acceleration experienced by a PxRigidDynamic that finds itself under the wheel of a vehicle.
            
* Changed:
            
  * In checked build the functions PxVehicleDrive4W::allocate, PxVehicleDriveNW::allocate, PxVehicleDriveTank::allocate, PxVehicleNoDrive::allocate all return NULL and issue a warning if called before PxInitVehicleSDK.
                
  * Tire width is no longer accounted for when computing the suspension compression from raycasts (PxVehicleSuspensionRaycasts).  Instead, tire width is incorporated into the suspension compression arising from swept wheels (PxVehicleSuspensionSweeps).  It is recommended to use PxVehicleSuspensionSweeps if there is a strict requirement that the inside and outside of the wheel don't visibly penetrate geometry.
            
* Fixed:
            
  * Suspension force calculation now applies an extra force perpendicular to the spring travel direction. This force is calculated to satisfy the constraint that the sprung mass only has motion along the spring travel direction.  This change mostly affects vehicles with suspension travel directions that are not vertical.
  * PxVehicleWheelsSimData::mThresholdLongitudinalSpeed and PxVehicleWheelsSimData::mMinLongSlipDenominator are now given default values that reflect the length scale set in PxTolerancesScale.
  * Unphysically large constraint forces were generated to resolve the suspension compression beyond its limit when the suspension direction and the hit normal under the wheel approach perpendicularity.  This has been fixed so that the constraint force approaches zero as the angle between the hit normal and suspension direction approaches a right angle.
            
        

## Scene queries
        
* Added:
            
  * PxPruningStructure was introduced as an optimization structure to accelerate scene queries against large sets of newly added actors.
  * PxScene::addActors(PxPruningStructure& ) has been added.
  * PxMeshQuery::sweep now supports PxHitFlag::eMESH_ANY.
  * PxHitFlag::eFACE_INDEX was introduced to reduce the perf cost for convex hull face index computation. In order to receive face index for sweeps against a convex hull, the flag PxHitFlag::eFACE_INDEX has to be set. Note that the face index can also be computed externally using the newly introduced method PxFindFaceIndex from the extensions library.
  * PxGeometryQuery::isValid was added to check provided geometry validity.
            
* Changed:
            
  * Raycasts against triangle meshes with PxHitFlag::eMESH_MULTIPLE flag now return all hits, code for discarding hits close to each other has been removed.
  * PxPruningStructure enum has been renamed to PxPruningStructureType
            
* Deprecated:
            
  * PxHitFlag::eDISTANCE has been deprecated.
  * The batched query feature has been deprecated.
  * Volume cache feature has been deprecated.
  * Spatial index feature has been deprecated.
            
* Fixed:
            
  * PxScene::sweep now properly implements PxHitFlag::eMESH_BOTH_SIDES (returned normal follows the same convention as for raycasts).
  * Raycasts against heightfields now correctly return multiple hits when PxHitFlag::eMESH_MULTIPLE flag is used.
  * PxSweepHit.faceIndex was computed incorrectly for sweep tests initially overlapping convex objects. The face index is now set to 0xffffffff in these cases.
  * Convex vs convex sweeps in PxGeometryQuery::sweep() do now correctly return the face index of the convex mesh that gets passed in as parameter geom1 (and not the one from geom0).
  * PxMeshQuery::sweep now supports PxHitFlag::eMESH_ANY.
  * Deprecated definition PxSceneQueryHit has been removed. Please use PxQueryHit instead.
  * PxGeometryQuery::computePenetration with convex geometries.
  * On Android platforms, the eDYNAMIC_AABB_TREE pruning structure could pass already released objects into the scene query filter callback.
            
        


## Cooking
        
* Added:
            
  * PxTriangleMeshCookingResult added, cookTriangleMesh now does return additional PxTriangleMeshCookingResult. Please see the manual for more information.
  * New convex hull generator added. It is now possible to switch between a new quickhull implementation and the legacy inflation based hull. Quickhull is the default algorithm.
  * Convex hulls can now be directly inserted in PxPhysics as triangle meshes and height fields.
  * A separate convex hull validation function has been added, it is now possible to create hulls without validation.
  * Convex hull generator vertex limit has two different algorithms - plane shifting and OBB slicing.
  * PxConvexFlag::eFAST_INERTIA_COMPUTATION added. When enabled, the inertia tensor is computed faster but with less precision.
  * PxConvexFlag::eGPU_COMPATIBLE added. When enabled convex hulls are created with vertex limit set to 64 and vertex limit per face is 32.
  * PxConvexFlag::eSHIFT_VERTICES added. When enabled input points are shifted to be around origin to improve computation stability.
  * PxCookingParams::gaussMapLimit has been added. The limit can now be fully user-defined. Please refer to the migration guide and best practices sections of the manual.
            
* Changed:
            
  * The performance of convex creation from polygons has been improved.
            
* Deprecated:
            
  * The PxPlatform enum and the PxGetGaussMapVertexLimitForPlatform function have been deprecated.
            
* Removed:
            
  * The deprecated flags PxMeshPreprocessingFlag::eREMOVE_UNREFERENCED_VERTICES and ::eREMOVE_DUPLICATED_TRIANGLES have been removed. Meshes get cleaned up by default unless PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH is set.
            
* Fixed:
            
  * Mesh cooking was sometimes crashing for meshes with less than 4 triangles. This has been fixed.
  * Cooking convex mesh from a flat input mesh produced incorrect large mesh.
            
        

## Extensions
        
* Added:
            
  * PxRaycastCCD has been added, to improve the visibility of the raycast-based CCD solution, which was previously only available in the Sample framework. This is a simpler and potentially cheaper alternative to the SDK's built-in continuous collision detection algorithm.
  * PxFindFaceIndex has been added. The function computes the closest polygon on a convex mesh from a given hit point and direction.
            
* Changed:
            
  * Memory churn of PxDefaultMemoryOutputStream has been reduced.
  * The signatures for the PxComputeMeshPenetration and PxComputeHeightFieldPenetration functions have changed.
            
        

## Profiling
        
* Changed:
            
  * Profiling information is now available only in debug, checked and profile configuration.
  * PxProfileZoneManager::createProfileZoneManager now takes PxAllocatorCallback as input parameter instead of PxFoundation.
            
        

## Physx Visual Debugger
        
* PhysXVisualDebuggerSDK, PvdRuntime projects replaced with PxPvdSDK.
* PxPvdSceneClient::drawPoints now takes physx::pvdsdk::PvdDebugPoint as input parameter instead of PxDebugPoint. drawLines, drawTriangles, drawText and so on.
* The SDK's Debug visualization data is not sent to PVD anymore in ePROFILE mode.
* PxPvdSceneFlag::eTRANSMIT_CONTACTS (instead of PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS) was sometimes incorrectly used to control the transmission of constraint-related data. This has been fixed. In addition, the PxPvdSceneFlag flags are now consistently ignored when PxPvdInstrumentationFlag::eDEBUG is not set.
        

## Aggregates
        

## Snippets
        
* Snippet profile zone has been removed.
        

# v3.3.4
October 2015

## Supported Platforms

## Runtime
        
* Apple iOS
* Apple Mac OS X
* Google Android ARM & x86 (version 2.2 or later required for SDK, 2.3 or later required for samples)
* Linux (tested on Ubuntu)
* Microsoft Windows XP or later (NVIDIA Driver version R304 or later is required for GPU acceleration)
* Microsoft Windows RT (formerly known as Windows on ARM) (SDK only, no samples yet)
* Microsoft XBox One (SDK only, no samples)
* Microsoft XBox 360
* Nintendo Wii U
* Sony Playstation 3
* Sony Playstation 4 (SDK only, no samples)
* Sony Playstation Vita
        
## Development
        
* Microsoft Windows XP or later
* Microsoft Visual Studio 2012, 2013 and 2015
* Xcode 6.3
        

## Changes and Resolved Issues

<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>

## General
        
* Added support for Microsoft Visual Studio 2015 for Windows builds. Note that the GPU features are not currently supported with Visual Studio 2015.
* Removed support for Microsoft Visual Studio 2010
* Added support for Android platform API Level 16 and removed support for platform API Level 8 and 9.
* Fixed:
            
  * Fixed a bug in aggregates that led to a crash when rigid bodies are added to an aggregate after removing all rigid bodies from an aggregate.   This only occurred with aggregates that were added to the scene and with rigid bodies that had shapes attached.
            
        

## Rigid Bodies
        
* Fixed:
            
  * Creating a PxConstraint or using PxConstraint::setActors() could cause a crash in situations where one of the two newly connected actors was part of a previously simulated graph of connected constraints (with some of them having projection enabled, i.e., PxConstraintFlag::ePROJECT_TO_ACTOR0 or ::ePROJECT_TO_ACTOR1 set).
  * PxD6Joint::getTwist(), getSwingY(), getSwingZ() returned incorrect angle values when the corresponding components of the quaternion were negative
  * The thickness of a heightfield was incorrectly applied when the heightfield transform had a non-identity quaternion. 
  * PxD6Joint angular projection now functions correctly when there is one free axis and it is not the twist axis.
  * The bounding box of a shape was not always properly updated when the contact offset changed.
  * Fixed an edge case bug in PCM contact gen that could result in a QNAN reported as the contact point.
  * Fixed an uninitialized variable bug in the GJK algorithm resulting in uninitialized closest points reported.
  * Fixed an edge case in which the constraint solver could access invalid memory in constraint partitioning.
  * Fixed a bug in capsule vs heightfield contact generation that could produce incorrect penetration depths.
  * Fixed a bug in Unified MTD code path which transformed the normal twice for the polygon index calculation. 
  * Fixed a crash in height fields when a capsule collided with an edge whose shared triangles had a hole material.
            
        

## Serialization
        
* Fixed:
            
  * PxTriangleMesh instances with adjacency information were not correctly initialized when created with cooking.createTriangleMesh. This caused a crash when converting the binary serialized triangle mesh data.
            
        

## Scene Queries
        
* Fixed:
            
  * Sweeps against scaled meshes.
            
        

## Cooking
        
* Fixed:
            
  * Mesh cooking was sometimes crashing for meshes with less than 4 triangles. This has been fixed.
            
        


# v3.3.3
January 2015

## Supported Platforms

## Runtime
        
* Apple iOS
* Apple Mac OS X
* Google Android ARM & x86 (version 2.2 or later required for SDK, 2.3 or later required for samples)
* Linux (tested on Ubuntu)
* Microsoft Windows XP or later (NVIDIA Driver version R304 or later is required for GPU acceleration)
* Microsoft Windows RT (formerly known as Windows on ARM) (SDK only, no samples yet)
* Microsoft XBox One (SDK only, no samples)
* Microsoft XBox 360
* Nintendo Wii U
* Sony Playstation 3
* Sony Playstation 4 (SDK only, no samples)
* Sony Playstation Vita
        
## Development
        
* Microsoft Windows XP or later
* Microsoft Visual Studio 2010, 2012 and 2013
* Xcode 6.2
        

## Known Issues

        
* The combination of releasing an actor and reassigning the actors of any affected joint so that the joint no longer references the released actor will lead to a crash if these operations are performed as buffered calls ie after simulate but before fetchResults.
        

## Changes and Resolved Issues

<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>

## General
        
* Added support for Microsoft Visual Studio 2013 and removed support for Microsoft Visual Studio 2008.
* Where applicable, mutexes on Unix flavored platforms now inherit priority to avoid priority inversion. This is a default behavior of Windows mutexes.
* Added arm64 support for the iOS version of the PhysX SDK.
* Removed samples from the iOS version of the PhysX SDK.
* Fixed:
            
  * x64 samples running on Windows 8.
  * Concurrent calls to PxPhysics::(un)registerDeletionListener() and PxPhysics::(un)registerDeletionListenerObjects() were not safe.
  * PxGeometryQuery::pointDistance() could sometimes read uninitialized memory. This has been fixed.
  * The SDK will not crash anymore if an object is involved in more than 65535 interactions. Instead, it will emit an error message and the additional interactions will be ignored.
  * The PxPhysics::getMaterials() function did not work with a non-zero 'startIndex' parameter. This has been fixed.
  * The following static Android libs are now packed into a single library called PhysX3: LowLevel, LowLevelCloth, PhysX3, PhysXProfileSDK, PhysXVisualDebuggerSDK, PvdRuntime, PxTask, SceneQuery and SimulationController. This fixes cyclic dependencies between these libraries.
  * FSqrt(0), V4Sqrt(0), V4Length(0) and V3Length(0) will return 0 instead of QNan in Android and iOS.
            
        

## Rigid Bodies
        
* Fixed:
            
  * The Prismatic joint limit now acts correctly when its frame is not the identity.
  * Calling PxRigidDynamic::setGlobalPose() with the autowake parameter set to true could result in an assert when the rigid body got released and had the PxActorFlag::eDISABLE_SIMULATION flag set.
  * Added errors on misuse of PxRegister[Unified]Heightfields() function, and documented it.
  * PxConstraint has a eDISABLE_PREPROCESSING flag and minResponseThreshold attribute to assist in stabilizing stiffness caused by infinite inertias or mass modification.
  * Island manager performance in the presence of large numbers of kinematic actors has been improved.
  * Using PxConstraint::setActors() or PxJoint::setActors() could cause a crash if the new actors resulted in the constraint/joint being removed from the scene.
  * The functions PxRigidActor::setGlobalPose and PxShape::setLocalPose failed to update cached contact data internal to PhysX.  This led to contacts being generated with transforms that were no longer valid.  Similarly, contacts could be missed due to transforms being invalid.  This defect affected the classes PxRigidStatic and PxRigidDynamic, though it was more immediately noticeable with the PxRigidStatic class.
  * The sphere-vs-mesh contact generation code has been improved. It could previously generate wrong contacts. This has been fixed.
  * The capsule-vs-convex contact generation had a bug that could lead to rare invalid contacts. This has been fixed.
  * The mesh contact generation had a bug on PS3 that could lead to invalid contacts. This has been fixed.
  * The PxRigidBody::clearForce() and PxRigidBody::clearTorque() were not properly decoupled - they both cleared the force and the torque. This has been fixed.
  * Repeatedly calling PxRigidActor::attachShape and PxRigidActor::detachShape in between calls to simulate resulted in a memory leak.  This has been fixed.
            
* Added:
            
  * Enabling CCD on kinematic actors is now disallowed. When the kinematic flags are raised on a CCD-enabled actor, CCD is automatically disabled. 
            
        

## Particles
        
* Fixed:
            
  * Consistency between GPU and CPU particles has been improved in the case of a spatial date structure overflow. The positions and velocities of particles that have the PxParticleFlag::eSPATIAL_DATA_STRUCTURE_OVERFLOW set are now updated also for CPU particles.
  * Fixed potential deadlocks from occurring in the GPU particle kernels running on GM204 and above GPUs.
  * Fixed fluid simulation crash on Titan X.
            
        

## Cloth
        
* Fixed:
            
  * A bug related to hitting the sphere or plane limit while world collision is enabled has been fixed.
  * PxCloth::getParticleAccelerations() implementation was fixed for GPU cloth.
            
        

## Character controller
        
* Fixed:
            
  * Character controllers cannot stand on dynamic triggers anymore.
            
* Added:
            
  * added lockingEnabled parameter to PxCreateControllerManager(), to support thread-safe release of objects while the character controller's move() routine is executing.
            
        

## Scene Queries
        
* Fixed:
            
  * Raycasts against triangle meshes with large scales potentially failed to register a hit.
  * Overlaps against height field meshes with the flag eNO_BOUNDARY_EDGES potentially failed to register a hit.
  * Sweeps using shapes modelled around a space significantly offset from their geometric center could fail to register a hit. 
            
        

## Vehicles
        
* Fixed:
            
  * Sticky tire friction was unreliable with more than one substep but is now fixed.  This defect led to vehicles sliding down slopes where the sticky friction should have held firm.
  * An error in the jounce speed calculation that led to lift force at high forward speeds has been fixed.  This defect led to instabilities at high speed.
  * Improved documentation for PxVehicleSuspsensionData::mSprungMass.
            
        

## Cooking
        
* Fixed:
            
  * Convex meshes generated from PhysX 3.2 were not able to load inside PhysX 3.3.

# v3.3.2
September 2014

## Supported Platforms

### Runtime
        
* Apple iOS
* Apple Mac OS X
* Google Android ARM & x86 (version 2.2 or later required for SDK, 2.3 or later required for samples)
* Linux (tested on Ubuntu)
* Microsoft Windows XP or later (NVIDIA Driver version R304 or later is required for GPU acceleration)
* Microsoft Windows RT (formerly known as Windows on ARM) (SDK only, no samples yet)
* Microsoft XBox One (SDK only, no samples)
* Microsoft XBox 360
* Nintendo Wii U
* Sony Playstation 3
* Sony Playstation 4 (SDK only, no samples)
* Sony Playstation Vita
        
### Development
        
* Microsoft Windows XP or later
* Microsoft Visual Studio 2008, 2010, 2012 (Windows RT only)
* Xcode 4.6|5.0|5.0.1
        

## Known Issues

        
* The combination of releasing an actor and reassigning the actors of any affected joint so that the joint no longer references the released actor will lead to a crash if these operations are performed as buffered calls ie after simulate but before fetchResults.
        

## Changes and Resolved Issues


<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>

### General
        
#### Added:
            
*  The PhysXCommon/64.dll, nvcuda.dll and PhysXUpdateLoader/64.dll are loaded and checked for the NVIDIA Corporation digital signature. The signature is expected on all NVIDIA Corporation provided dlls. The application will exit if the signature check fails.
*  Added the PxDefaultBufferedProfiler extension for simplified SDK profile events extraction.
*  PxSceneDesc::sanityBounds allows a bounding box to be set for validating the position coordinates of inserted or updated rigid actors and articulations.
*  Linux: Now supports GPU PhysX.
*  Added set/getRunProfiled() for PxDefaultCpuDispatcher to control profiling at task level.
*  Android: Support for x86 based devices was added.
*  PxProfileEventHandler::durationToNanoseconds() added. Translates event duration in timestamp (cycles) into nanoseconds.
*  Added SnippetProfileZone to show how to retrieve profiling information.
*  Added SnippetCustomJoint to better illustrate custom joint implementation, and removed SnippetExtension.
*  Added SnippetStepper to demonstrate kinematic updates while substepping with tasks.
            
#### Fixed:
            
*  PxTask::runProfiled() now takes threadId as a parameter.
*  The static pruner now issues a performance warning in debug and checked configurations when a tree rebuild occurs and the tree is not empty.
*  PxSceneDesc::staticStructure now defaults to PxPruningStructure::eDYNAMIC_AABB_TREE.
*  Linux: Switched to shared libraries.
*  Profile zone event names changed to match function calls.
*  Overlapping read/write errors will now issue a PxErrorCode::eINVALID_OPERATION rather than PxErrorCode::eDEBUG_INFO.
*  Improved SnippetToleranceScale to better demonstrate the intended use case.
*  Increased 126 characters limit for warnings on unix platforms, 1k limit on all platforms.
*  PhysXCommon dll load within PhysX dll now respects dll name. Please see the manual's PhysXCommon DLL load section.
*  Significant revision of the user's guide.  Both structure and most content have been modified.
*  Fixed search function of user's guide.
*  Foundation math classes now have in-place arithmetic operators (+= etc).
*  PxScene::checkResults() no longer counts as a read API method. Hence it is now possible to call this method in blocking mode without causing all writes to block until it returns.
            
#### Deprecated:
            
*  Indexing operators taking signed integers in PxVec3, PxVec4, PxMat33, PxMat44, PxStrideIterator have been deprecated.
            
        

### Rigid Bodies
        
#### Fixed:
            
*  A minor bug in contact generation between capsules and triangle meshes has been fixed, reducing the amount of tunneling cases when CCD is not used.
*  Discrete contact reports are no longer produced for pairs without PxPairFlag::eDETECT_DISCRETE_CONTACT raised in the filter shader. Previously, discrete contact generation would always have been performed regardless of the presence of the PxPairFlag::eDETECT_DISCRETE_CONTACT flag. This change potentially improves performance when using  specific shapes for CCD-only collision, which would have previously generated discrtete contacts and then ignored them in the solver. 
*  Trigger reports are no longer produced for pairs without PxPairFlag::eDETECT_DISCRETE_CONTACT raised in the filter shader. PxPairFlag::eTRIGGER_DEFAULT has been modified to include the PxPairFlag::eDETECT_DISCRETE_CONTACT flag.
*  An incorrect PX_DEPRECATED annotation on the default constructor for PxD6JointDrive has been removed.
*  PxRigidDynamic::getKinematicTarget() returned a wrong transform if the actor center of mass pose was different from the actor global pose.
*  Switching a PxRigidDynamic from dynamic to kinematic did not properly suppress existing pairs which turned into kinematic-kinematic or kinematic-static pairs.
*  PxRigidDynamic::isSleeping() did not return the correct value on the frame the object got inserted if PxScene::addActors() was used and if the object was awake.
*  PxSceneFlag::eDISABLE_CONTACT_CACHE now correctly works on PS3/SPU.
*  If an object was added to the scene, put asleep and had overlap with another sleeping object then contact points for that pair might not have been reported once the object woke up.
*  Potential crash when calling PxScene::resetFiltering() multiple times for the same actor while the simulation was running has been fixed.
*  Potential crash when using PxScene::resetFiltering() with shapes that were just added while the simulation was running has been fixed.
*  A crash in MBP when deleting an object that just went out of broad-phase bounds has been fixed.
*  A new drive mode has been added to drive articulation joints using rotation vectors.
*  In contact and trigger reports, the shape references in PxTriggerPair and PxContactPair might not have been properly marked as removed shapes if the removal took place while the simulation was running.
*  PxPairFlag::eSOLVE_CONTACT is now properly observed if the flag is not set on a contacting pair. A consequence of this fix is that sleeping actors will no longer be woken up due to contact or lost contact with awake actors if PxPairFlag::eSOLVE_CONTACT is not set for the pair.  This also affects kinematic-kinematic pairs if one kinematic of the pair moves out of contact with the other.  Behaviour is unaffected for any pair that has PxPairFlag::eSOLVE_CONTACT set.
*  A memory leak with buffered shape and actor removal has been fixed.  The memory leak occurred when the release of an actor's shapes was followed by the release of the actor, all in-between simulate() and fetchResults().
*  A bug was fixed which caused incorrect force reports to sometimes be reported.
*  Fixed a bug where incorrect normals were reported when using PCM contact gen.
*  Fixed some issues with scaled convex hulls in the PCM contact gen code path.
*  The accuracy of capsule collision code has been improved.
*  An isValid() method has been added to constraints, that is satisfied if and only if at least one actor is a dynamic body or articulation link
*  A check has been added to prevent constraint construction or modification that would leave the constraint invalid.
*  In checked builds the PxScene methods addActor(), addActors(), addAggregate() and addCollection() will warn and return if an invalid constraint would be added to the scene
            
#### Deprecated:
            
*  The following flags have been renamed and deprecated because the name did not properly reflect the root cause.
                
* PxContactPairHeaderFlag
                    
  * eDELETED_ACTOR_0 (use eREMOVED_ACTOR_0 instead)
  * eDELETED_ACTOR_1 (use eREMOVED_ACTOR_1 instead)
                    
* PxContactPairFlag
                    
  * eDELETED_SHAPE_0 (use eREMOVED_SHAPE_0 instead)
  * eDELETED_SHAPE_1 (use eREMOVED_SHAPE_1 instead)
                    
* PxTriggerPairFlag
                    
  * eDELETED_SHAPE_TRIGGER (use eREMOVED_SHAPE_TRIGGER instead)
  * eDELETED_SHAPE_OTHER (use eREMOVED_SHAPE_OTHER instead)
                    
                
            
        

### Vehicles
        
#### Added:
            
*  In profile config the functions PxVehicleUpdates, PxVehiclePostUpdates and PxVehicleSuspensionRaycasts are now tracked with profile events (provided that a PxProfileZoneManager instance was passed to PxCreatePhysics).  These profile events can be viewed in profile view in pvd, where the names of the profile events match the names of the tracked vehicle functions.
            
#### Fixed:
            
*  In checked config PxVehicleDriveTank::allocate enforces the rule that only tanks with even numbers of wheels are legal and warns when this rule is broken.
*   In checked config PxVehicleDrive4W::allocate enforces the rule that only vehicles with 4 wheels or more are legal and warns when this rule is broken.
*  PxWheelQueryResult::localPose was incorrectly only set when the vehicle had a corresponding PxShape, as described by PxVehicleWheelsSimData::setWheelShapeMapping.  The localPose is now properly set independent of the mapping between wheel and shape.
*  Wheels resting on moving actors now properly observe the relative speed of the two actors when their relative speed is small.  This fixes a bug where at small relative speeds the velocity of the other actor was assumed to be zero.
*  Repx serialization failed to serialize PxVehicleWheelsSimData::setMinLongSlipDenominator, PxVehicleWheelsSimData::setSubStepCount, PxVehicleWheelsSimData::disableWheel, PxVehicleWheelsSimData::enableWheel and the number of entries in the engine torque curve.  These have now been fixed.
*  PxVehicleConcreteType used for vehicle serialization is now in the public API and has been moved to PxVehicleSDK.h.
*  Very small longitudinal and lateral slip angles could lead to numerical instabilities in some circumstances.  A threshold has been introduced to reject very small slip angles by setting them to zero when they are below the threshold.
*  Vehicles now account for rigid bodies that have been given a zero inertia component in order to lock rotation around the corresponding axis.
*  Fixed a bug where the sticky wheel constraints sometimes didn't function correctly.
            
        

### Cloth
        
#### Fixed:
            
*  The version number written to the fabric stream changed from PX_PHYSICS_VERSION to 1. A fabric can be created from streams written with version 3.3.0 and later until the stream format changes. Previously, the version of the producer and the consumer of the stream needed to match.
*  GPU cloth friction against convexes has been fixed.
*  A crash resulting from deleting a shape in proximity of a cloth with scene collision enabled has been fixed.
            
        

### Scene Queries
        
#### Fixed:
            
*  PxMeshQuery::sweep now respects PxHitFlag::eMESH_BOTH_SIDES, and supports double-sided input triangles.
*  PxRigidBodyExt::linearSweepSingle and PxRigidBodyExt::linearSweepMultiple now correctly use query filter data instead of simulation filter data if filter data is not provided.
*  The spec for raycasts whose origin is located inside a solid object (sphere, box, capsule, convex) has changed back to what it was in 3.3.0. It was accidentally changed in 3.3.1. See the manual for details.
*  Convex sweeps against heightfields worked only when the heightfield had the identity transform.  This has now been fixed to support arbitrary transforms again.
            
        

### Cooking
        
#### Added:
            
*  Using PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES will always cook meshes with 32-bit triangle indices.
            
#### Fixed:
            
*  Convex hull cooking fix. Some input points could be ignored during cooking, fixed.
*  Inserted triangle meshes now respect 16 bit indices flag.


            
        

### Geometry
        
#### Fixed:
            
*  PxHeightFieldDesc::thickness is now limited to [-PX_MAX_BOUNDS_EXTENTS, PX_MAX_BOUNDS_EXTENTS range]. (Previously unbounded).
            
        

### Particles
        
#### Fixed:
            
*  Setting PxParticleReadDataFlag::eREST_OFFSET_BUFFER on a PxParticleBase instance that was not created with the per particle rest offset option (see PxPhysics::createParticleSystem, PxPhysics::createParticleFluid and PxParticleBaseFlag::ePER_PARTICLE_REST_OFFSET) is not supported. The unsupported configuration may have resulted in crashes. The SDK now rejects this configuration on calling PxParticleBase::setParticleBaseFlag and issues an appropriate warning to the error stream.
*  Performance improvements on Kepler and above GPUs running SPH.
*  In rare cases particle systems could access released memory when all interactions with a rigid body shape were lost.
            
        

### Serialization
        
#### Fixed:
            
*  PxBinaryConverter::convert potentially failed in checked mode with allocators that don't set 0xcd pattern. This has been fixed now.
            
        








# v3.3.1
December 2013

## Supported Platforms

### Runtime
        
* Apple iOS
* Apple Mac OS X
* Google Android (version 2.2 or later for SDK, 2.3 or later required for samples)
* Linux (tested on Ubuntu)
* Microsoft Windows XP or later (NVIDIA Driver version R304 or later is required for GPU acceleration)
* Microsoft Windows RT (formerly known as Windows on ARM) (SDK only, no samples yet)
* Microsoft XBox One
* Microsoft XBox 360
* Nintendo Wii U
* Sony Playstation 3
* Sony Playstation 4 (SDK only, no samples)
* Sony Playstation Vita
        
### Development
        
* Microsoft Windows XP or later
* Microsoft Visual Studio 2008, 2010, 2012 (Windows RT only)
* Xcode 4.6|5.0|5.0.1
        

## Known Issues

        

## Changes and Resolved Issues


<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>

### General
        
#### Added:
            
*  The friction model can now be changed after scene instantiation with PxScene::setFrictionType.  The friction model can also be queried with PxScene::getFrictionType.
            
#### Changed:
            
*  PxDefaultSimulationFilterShader now supports particles and cloth as well.
*  PxSimulationFilterCallback: the provided actor and shape pointers are now defined as const. Note: this is no behavior change, it was never allowed to write to those objects from within the callback.
*  The PxTriangleMeshFlag::eHAS_16BIT_TRIANGLE_INDICES and PxTriangleMeshFlag::eHAS_ADJACENCY_INFO enums have been deprecated. Please use PxTriangleMeshFlag::e16_BIT_INDICES and PxTriangleMeshFlag::eADJACENCY_INFO instead.
*  Removed following functions from the API for platforms which do not support CUDA: PxGetSuggestedCudaDeviceOrdinal, PxCreateCudaContextManager, PxLoadPhysxGPUModule.
            
#### Fixed:
            
*  Fixed concurrency issue on windows. Calling PxScene::simulate on multiple scenes concurrently may have caused a deadlock. This only happened if the scenes shared a single PxCpuDispatcher and the dispatcher was configured to use one worker thread only.
            
        

### Rigid Bodies
        
#### Added:
            
*  The projection direction for constraints can now be specified through the flags PxConstraintFlag::ePROJECT_TO_ACTOR0, ::ePROJECT_TO_ACTOR1.
*  A parameter has been added to PxRigidBodyExt::updateMassAndInertia() and ::setMassAndUpdateInertia() to optionally take non-simulation shapes into account for computing the mass and the inertia tensor of a rigid body.
*  It is now possible to retrieve additional information in contact reports. See the API documentation of PxContactPairHeader.extraDataStream, PxPairFlag::ePRE_SOLVER_VELOCITY, ::ePOST_SOLVER_VELOCITY, ::eCONTACT_EVENT_POSE for details.
*  The contact report system has been extended to support multiple notification events if the same two objects collide multiple times in multipass CCD scenarios. See the API documentation of PxPairFlag::eNOTIFY_TOUCH_CCD for details.
            

#### Changed:
            
*  If touching objects were added to the scene asleep and one of them got woken up, then all contact pairs of the touching objects which contained a static rigid body were resolved with a delay of one simulation step. Now these pairs get resolved without delay in the next simulation step.
*  If touching objects were added to the scene asleep, eNOTIFY_TOUCH_FOUND contact reports were sent out for pairs of dynamic rigid bodies if requested. These reports will not be sent at the end of the simulation step after insertion anymore but rather at the end of the simulation step after the touching objects have been woken up.
*  Rigid bodies now permit zeroes in passed to setMass and setMassSpaceInertiaTensor. Zeroes are interpreted to indicate infinite mass or infinite moment of inertia around a given principal axis of inertia. Previously, zeroes were illegal values to these methods. Note that zeroes are still illegal for instances of PxArticulationLink.
            

#### Fixed:
            
*  Reading back the kinematic target in the PxSimulationEventCallback::onContact() callback through PxRigidDynamic::getKinematicTarget() will now work.
*  Contact reports are no longer generated for contact pairs involving two sleeping kinematic actors or for pairs involving a sleeping kinematic actor in contact with a static actor.  This fixes a bug that was introduced in 3.3.0. 
*  No PxPairFlag::eNOTIFY_TOUCH_LOST event was sent in contact reports if a pair of sleeping rigid bodies got woken up after setting the pose on one of them (with autowake parameter set to false) and if the bounding boxes of the two objects still overlapped.
*  No PxPairFlag::eNOTIFY_TOUCH_PERSISTS event was sent in contact reports during the first simulation step after a pair of sleeping rigid bodies got woken up.
*  The inertia tensor computation for convex meshes has been adjusted to be more stable in certain cases where floating point precision issues arise. Furthermore, a fallback routine has been added to use an approximation if the diagonalized inertia tensor still ends up with invalid entries.
*  PxRigidBody::clearForce() and ::clearTorque() did not properly clear the respective properties if used with force mode PxForceMode::eIMPULES or PxForceMode::eVELOCITY_CHANGE.
*  Setting PxSceneFlag::eENABLE_KINEMATIC_STATIC_PAIRS also enabled PxSceneFlag::eENABLE_KINEMATIC_PAIRS internally and vice versa.
*  Missing validation checks for some joint set() methods have been added. Similarly to other API calls, when validation fails in the checked build PhysX will report an error and return without updating the joint.
*  Switching a kinematic rigid body to dynamic could lead to a crash in a subsequent simulation step, if the kinematic was moved and connected to another kinematic through a breakable PxConstraint/PxJoint.
*  Deleting a breakable PxConstraint/PxJoint while the simulation is running could lead to a crash if the PxConstraint/PxJoint broke in the same simulation step.
*  A bug in the PxScene::addBroadPhaseRegion() function, that could lead to a crash when using 'populateRegion=true', has been fixed.
            
        

### Particles
        
#### Added:
            
*  Added triangle mesh cache statistics for GPU particles.  Triangle mesh cache statistics are also captured by PVD as part of simulation statistics.
*  Added new option to query approximate particle velocities relative to colliding rigid actors.  This can be used for debris rotation on moving objects.  Enable with PxParticleReadDataFlag::eCOLLISION_VELOCITY_BUFFER and read from PxParticleReadData::collisionVelocityBuffer.
            
#### Fixed:
            
*  Fixed a bug which might lead to GPU particle pipeline failures on low end GPUs.
*  Enabled warning when a spatial data structure overflow occurred for GPU particles (see the guide for more information).
            
        

### Cloth
        
#### Fixed:
            
*  PxFilterFlag::eSUPPRESS was ignored for collision pairs that involved a PxCloth object. This does work now, however, please note that PxFilterFlag::eSUPPRESS is treated as PxFilterFlag::eKILL for pairs with a PxCloth object.
            
        

### Serialization
        
#### Added:
            
*  Support for binary compatibility between different sdk releases and patches has been added (PX_BINARY_SERIAL_VERSION). The current sdk version can load binary data of the older sdk versions listed in the documentation of PX_BINARY_SERIAL_VERSION. 
*  SnippetLoadCollection has been added. It illustrates loading repx or binary serialized collections and instantiating the objects in a scene. It only compiles and runs on authoring platforms (windows, osx and linux). 
*  SnippetConvert has been added. It illustrates how to convert PhysX 3 serialized binary files from one platform to another. It only compiles and runs on authoring platforms (windows, osx and linux). 
            
#### Deprecated:
            
*  Method PxCollection::addRequire is deprecated, use PxCollection::add and PxCollection::contains instead. 
*  Method PxCollection::createBinaryConverter(PxSerializationRegistry&) is deprecated, use PxCollection::createBinaryConverter() instead. 
            
        

### Character controller
        
#### Added:
            
*  PxControllerManager::setPreventVerticalSlidingAgainstCeiling() has been added, to control the behaviour of characters against ceilings.
            
        


### Vehicles
        
#### Added:
            
*  Vehicles may now be updated concurrently through the addition of a new function PxVehiclePostUpdates and passing a PxVehicleConcurrentUpdateData array to PxVehicleupdates.
*  A new snippet SnippetVehicleMultiThreading has been added to show the operation of concurrent vehicle updates.
*  PxVehicleDriveTankControl and PxVehicleDriveTankControlModel now have improved doxy comments.
*  A new function PxVehicleUpdateCMassLocalPose has been added to help update a vehicle after the center of mass pose of the vehicle's actor has been modified.
*  PxWheelQueryResult now records the local pose of the wheel.
            
#### Changed:
            
*  PxVehcicleDrive4W::setup now tests that at least 4 wheels are specified and returns wtih an error message if numWheels < 4.  It is only possible to create a PxVehicleDrive4W instance with less than 4 active wheels by disabling wheels after instantiating a 4-wheeled car.
*  In debug and checked build configurations PxVehicleComputeSprungMasses now reports whether the sprung masses were successfully computed.   Warnings are passed to the error stream in checked configuration if the function does not complete successfully.  Apart from error checking the operation of the function is unchanged.
            
#### Fixed:
            
*  The doxy comment describing the default setting for PxVehicleWheelsSimData::setWheelShapeMapping was incorrect.  This now correctly documents the default mapping as PxVehicleWheelsSimData::setWheelShapeMapping(i,i).
*  Suspensions raycasts that start inside geometry are ignored for all geometry types.  Prior to this release this was true for all geometry types except for heightfields and triangle meshes.  This inconsistency has now been fixed so that all geometry types obey the rule that suspension raycasts starting inside geometry are neglected.
            
        

### Scene queries
        
#### Added:
            
*  Added eMTD flag. If an initial overlap is detected, this flag triggers the sweep to compute the MTD (Minimum Translation Direction), which can be used to de-penetrate the query shape from the shape with which an initial overlap was found. In this case, the distance reported will be negative. This negative distance can be used to scale the reported normal to generate the translation vector required to de-penetrate the query shape. 
*  Added PxTriangle::pointFromUV.
            
#### Fixed:
            
*  A rare ray-capsule intersection bug has been fixed, when the capsule's height is close to zero.
*  A capsule-capsule distance bug has been fixed, when the tested capsules are large and parallel.
*  Raycasts against heightfields now correctly return triangle UVs.
            
#### Changed:
            
*  PxBatchQuery::raycast, overlap and sweep previously had an incorrect const modifier indicating that these methods were safe to call from multiple threads simultaneously. This has been removed. Multiple batch queries can still be executed (via PxBatchQuery::execute()) in parallel.
                
            
        

### Cooking
        
#### Added:
            
*  PxCookingParams::meshSizePerformanceTradeOff parameter can be used to make the mesh smaller at the expense of reduced simulation and scene query performance (or the other way around).
*  PxCookingParams::meshCookingHint parameter can be used to specify mesh hierarchy construction preference (cooking speed or simulation speed).
*  PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH disables mesh clean process. Vertices duplicities are not searched, huge triangles test is not done. Vertices welding is not done. Does speed up the cooking.
*  PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE disables vertex edge precomputation. Makes cooking faster but slow up contact generation.
*  PxPhysicsInsertionCallback adds the support for inserting cooked triangle mesh or height field directly into PxPhysics without the stream serialization.
*  PxCooking::createTriangleMesh creates triangle mesh and inserts it to PxPhysics without using the stream serialization.
*  PxCooking::createHeightField creates height field and inserts it to PxPhysics without using the stream serialization.
*  PxCooking::validateTriangleMesh validates mesh in separate function before it can be cooked without the mesh cleaning.
*  PxConvexFlag::eCHECK_ZERO_AREA_TRIANGLES checks and removes almost zero-area triangles during the computation of the convex hull.
*  PxCookingParams::areaTestEpsilon triangle area size was added. This epsilon is used for the zero-area test in the computation of the convex hull.
            
#### Changed:
            
*  PxCooking::computeHullPolygons does now return also vertices used by the polygons. Redundant vertices are removed.
*  PxCooking::cookConvexMesh now returns a PxConvexMeshCookingResult::Enum with additional error information.
            
        

### Aggregates
        
#### Added:
            
*  PxSceneLimits has a new member variable maxNbAggregates.  Setting this value to a good approximation of the peak number of aggregates will avoid the need for internal run-time allocations that can occur when aggregates are added to the scene. 
            
#### Fixed:
            
*  PxScene::removeActor will auotmatically remove that actor from all PxAggregate instances that contain the removed actor.  Likewise, PxScene::removeArticulation will automatically remove all articulation links from all relevant aggregates.  This fix upholds the rule that all actors of an aggregate must be in same scene.
*  The condition of an internal assert that triggered after calling PxScene::addAggregate has been corrected. This assert triggered when an aggregate was added to the scene after removal of all aggregates from the scene.  The operation of the function PxScene::addAggregate is unchanged apart from the asserted condtition.
            
        

### Samples
        
#### Changed:
            
*  Starting with Release 302 drivers, application developers can direct the Optimus driver at runtime to use the High Performance Graphics to render any application - even those applications for which there is no existing application profile. The samples now make use of this feature to enable High Performance Graphics by default.
            
        






# v3.3
September 2013

## Release Highlights

        
* Added PhysXDevice/64.dll to the PC packages. See the Windows readme for details.
* Added support for the NVIDIA Kepler GPU architecture.
* Added support for the Nintendo Wii U console.
* Added support for Windows 8 Modern UI applications (ARM and x86).
* Ported our SIMD library to the ARM NEON architecture.
* Multi Box Pruning (MBP) is offered as an alternative broad phase algorithm to Sweep And Prune (SAP). MBP shows improved performance when most objects are moving or when inserting large numbers of objects. SAP can be faster in scenarios with few moving (many sleeping) objects.
* Significant performance and stability optimizations for rigid body solver.
* New function to compute the minimum translational distance and direction to separate two overlapping geometry objects.
* New 'PCM' contact generation mode which is often faster and more robust than the still available legacy path.
* Improved performance of scene queries and contact reports.
* Improved behavior and performance of Continuous Collision Detection (CCD).
* Reduced memory footprint of rigid body classes.
* Added support for sharing shapes among rigid bodies.
* Significantly improved cloth behavior and GPU performance.
* Added support for cloth colliding against itself, other cloth instances, and scene geometry.
* Improved useability of binary and xml serialization.
* Memory can be saved for objects that do not participate in the simulation and are used for scene queries only. For details see the new flag PxActorFlag::eDISABLE_SIMULATION.
        


## Supported Platforms

### Runtime
        
* Apple iOS
* Apple Mac OS X
* Google Android (version 2.2 or later for SDK, 2.3 or later required for samples)
* Linux (tested on Ubuntu)
* Microsoft Windows XP or later (NVIDIA Driver version R304 or later is required for GPU acceleration)
* Microsoft Windows RT (formerly known as Windows on ARM) (SDK only, no samples yet)
* Microsoft XBox One
* Microsoft XBox 360
* Nintendo Wii U
* Sony Playstation 3
* Sony Playstation 4 (SDK only, no samples yet)
* Sony Playstation Vita
        
### Development
        
* Microsoft Windows XP or later
* Microsoft Visual Studio 2008, 2010, 2012 (Windows RT/PS4/XboxOne only)
* Xcode 4.6
        


## Known Issues

        
* PxSimulationOrder::eSOLVE_COLLIDE feature is not implemented in this release.  Calls to PxScene::solve() and PxScene::collide() will be ignored with a warning added to the error stream
* Reading back the kinematic target through PxRigidDynamic::getKinematicTarget() in the PxSimulationEventCallback::onContact() callback will fail.
* Cloth self-collision without rest position is not supported on GPUs with SM capability lower than 2.0.
        

## Changes and Resolved Issues
    

<i>Note: Platform specific issues and changes can be found in the readme file of the corresponding platform.</i>

### General
            
####  Added:
                
*  PxScene::setLimits: Hint to preallocate capacities of various data structures. See PxSceneLimits.
*  Scalar constructors PxQuat(r), PxMat33(r), PxMat33(r).
*  identity constructors for PxMat33, PxMat44, PxQuat, PxTransform - e.g. PxTransform(PxIdentity).
*  zero constructors for PxMat33, PxMat44, PxVec3 - e.g. PxMat33(PxZero).
*  PxTransform(x, y, z) constructor was added as a shortcut for PxTransform(PxVec3(x,y,z)).
*  The GPU code uses CUDA version 5.0 and supports Kepler GPUs (SM version 3.0 and 3.5).
*  Helper method PxContactPair::bufferContacts() has been added to copy the contact pair data stream into a user buffer.
*  PxGeometryQuery::computePenetration() has been added, to compute the Minimum Translational Distance between geometry objects.
*  Ability for APEX and other PhysX extensions to change the PhysX Visual Indicator.
*  Reporting allocation names can now be enabled or disabled (see PxFoundation::setReportAllocationNames). When enabled, some platforms allocate memory through 'malloc'.
*  The scene origin can now be shifted to better support big world scenarios. See PxScene::shiftOrigin() for details.
*  PxAssertHandler got extended with a boolean parameter to support ignoring specific asserts. See windows implementation of DefaultAssertHandler.
*  Added new PxPairFlags - PxPairFlag::eSOLVE_CONTACT and PxPairFlag::eDETECT_DISCRETE_CONTACT.
                
####  Removed:
                
*  The obsolete PxConvexFlag::eUSE_UNCOMPRESSED_NORMALS flag has been removed.
*  The PxSceneFlag::eDISABLE_SSE was obsolete, and has now been removed.
*  The obsolte PxPtrArray has been removed
                
####  Changed:
                
*  Mesh BVH construction was significantly improved for meshes with a mix of large and small triangles. Mesh sizes are now slightly increased, but traversals are substantially faster. As a side effect, cooked mesh format has changed. This requires meshes to be recooked!
*  The specification for valid PxBounds3 representations has changed. See the API documentation for details (especially the newly introduced PxBounds3::isValid() method).
*  PxBounds3::transform(), ::scale() and ::fatten() have been split into a fast and a safe version to avoid unnecessary checks and improve stability for empty bounds respectively.
*  PxBounds3::setInfinite() has been renamed to PxBounds3::setMaximal() to better fit the actual behavior.
*  PxTask: PVD profile events for tasks are now only emitted in profile builds (all platforms).
*  Platform mutex implementations now verify that lock/unlock come from correct thread in debug builds.
*  PxWindowsDelayLoadHook.h has been moved from Include/foundation/windows to Include/common/windows.
*  API instances of 'Num' as in 'maxNum' and 'getNum' have changed uniformly to 'Nb'.
*  The following classes have been renamed:
                        
  * PxSceneQueryHit to PxQueryHit
  * PxSceneQueryFlags to PxHitFlags
  * PxSceneQueryHitType to PxQueryHitType
  * PxSceneQueryFilterData to PxQueryFilterData
  * PxSceneQueryFilterCallback to PxQueryFilterCallback
  * PxSceneQueryFilterFlags to PxQueryFlags
  * PxSceneQueryCache to PxQueryCache
  * PxCCTNonWalkableMode to PxControllerNonWalkableMode
  * PxControllerFlags to PxControllerCollisionFlags
  * PxCCTHit to PxControllerHit
  * PxConstraintDominance to PxDominanceGroupPair
  * PxActorTypeSelectionFlags to PxActorTypeFlags
  * PxFindOverlapTriangleMeshUtil to PxMeshOverlapUtil
                        
* The previous names have been retained for compatibility but are deprecated.
                    
*  PX_SLEEP_INTERVAL has been replaced with the new parameter PxSceneDesc::wakeCounterResetValue to specify the wake counter value to set when wakeUp() gets called on dynamic objects.
*  PxClientBehaviorBit has been renamed PxClientBehaviorFlag, PxActorClientBehaviorBit has been renamed PxActorClientBehaviorFlag. Names of related functions have also changed.
*  queryClient parameter in raycast(), sweep(), overlap() functions was moved inside of PxQueryFilterData struct.
*  The PxObserver/PxObservable system has been replaced by the PxDeletionListener API. The supported object types have been extended from PxActor to all core objects inheriting from PxBase. Furthermore, two kinds of deletion events are now distinguished: user release and memory release. Please read the API documentation for details.
*  Deprecated PxPairFlag::eRESOLVE_CONTACT. Use PxPairFlag::eDETECT_DISCRETE_CONTACT and PxPairFlag::eSOLVE_CONTACT instead.
*  Number of materials per shape is now PxU16 instead of PxU32, contact material information also now returns PxU16.
*  Maximum number of touching hits in batched queries is now PxU16 instead of PxU32.
*  SweepEpsilonDistance has been replaced by meshContactMargin and marked as deprecated. Please read the API documentation for details.
*  PxShape::resetFiltering() and PxParticleBase::resetFiltering() have been deprecated. Please use one of the new overloaded methods PxScene::resetFiltering() instead.
*  The pxtask namespace has been removed and it's types have been added to the physx namespace with a Px* prefix
*  The delay load hook PxDelayLoadHook::setPhysXInstance has been renamed to PxSetPhysXDelayLoadHook and PxDelayLoadHook::setPhysXCookingInstance has been renamed to PxSetPhysXCookingDelayLoadHook
                
####  Fixed:
                
*  Calling Thread::setAffinityMask() before the thread has been started is now supported.
*  PxDefaultSimulationFilterShader ignored the value of the second filter constant in the collision filtering equation and used the value of the first filter constant instead.
                
####  Deprecated:
                
*  The PxScene::flush() method has been deprecated, please use PxScene::flushSimulation().
*  PxRigidDynamicFlag has been deprecated and replaced with PxRigidBodyFlag to allow flags to be shared between PxArticulationLink and PxRigidDynamic.
*  PxTransform::createIdentity(), PxQuat::createIdentity(), PxMat33::createIdentity(), PxMat44::createIdentity(), PxMat33::createZero(), PxMat44::createZero()
                
            

### Character controller
            
####  Added:
                
*  The PxControllerBehaviorFlag::eCCT_USER_DEFINED_RIDE flag has been added.
*  A new helper function PxController::resize() has been added to facilitate character controller resizing.
*  A new runtime tessellation feature has been added that can help reducing FPU accuracy issues in the sweep tests.
*  PxControllerFilterCallback has been added to make CCT-vs-CCT filtering more flexible.
*  An overlap recovery module has been added. See PxControllerManager::setOverlapRecoveryModule
*  PxObstacle notifications has been added to handle touched obstacles.
*  PxObstacleContext::getObstacleByHandle has been added.
*  The origin of character controllers and obstacles can now be shifted to stay in sync when the origin of the underlying PxScene is shifted. See PxControllerManager::shiftOrigin() for details.
                
####  Changed:
                
*  The PxControllerManager is now tightly coupled to a PxScene which has to be provided on creation. See PxCreateControllerManager().
*  The PxObstacleContext instances of a PxControllerManager will now get released automatically when the manager is released.
*  PxController::reportSceneChanged() has been renamed to PxController::invalidateCache().
*  PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT is not the default behavior anymore.
*  PxCCTNonWalkableMode::eFORCE_SLIDING has been renamed to PxCCTNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING.
*  PxCCTInteractionMode has been renamed PxControllerInteractionMode
*  PxCCTNonWalkableMode has been renamed PxControllerNonWalkableMode
*  PxCCTHit has been renamed PxControllerHit
*  Touched PxObstacle has been replaced by touched ObstacleHandle.
*  PxControllerInteractionMode and PxControllerFilters::mActiveGroups have been removed. Please use the new PxControllerFilterCallback instead.
                
####  Fixed:
                
*  Bugs with respect to deleting shapes from a touching actor have been fixed.
*  Touched actor's scene is checked in CCT::move before rideOnTouchedObject is called to ensure that the touched shape is valid.
*  Touched shape's scene query flag is checked in CCT::move before rideOnTouchedObject is called to ensure that the touched shape is valid.
*  Touched shape's user CCT filtering is triggered in CCT::move before rideOnTouchedObject is called to ensure that the touched shape is valid.
*  The PxControllerBehaviorCallback was not called when a PxUserControllerHitReport was not defined.
*  It is not possible anymore to create a CCT whose contact offset is zero. The doc has always said a non-zero value was expected, but corresponding check was missing.
*  Box & capsule controllers' resize function now properly take the up direction into account
                
            

### CCD
            
####  Added:
                
*  Introduced PxRigidBodyFlag::eENABLE_CCD_FRICTION to control whether friction is applied inside CCD on a given body. This is disabled by default. In general, disabling friction in CCD improves the behavior of high-speed collisions.
*  Introduced PxSceneDesc::ccdMaxPasses field, which controls the maximum number of CCD passes. Each CCD pass will advance each object to its next TOI. By default, we use 1 pass. Increasing the number of passes can reduced the likelihood of time being dropped inside the CCD system at the cost of extra CCD procesing overhead.
*  Introduced per-body value to control how far past the initial time of impact the CCD advances the simulation. Advancing further can improve fluidity at the increased risk of tunneling.
*  CCD now has limited support contact modification. It supports disabling response to contacts by setting max impulse to 0. It does not yet support target velocity, non-zero max impulse values or scaling mass or inertia.
*  Introduced new PxPairFlag eDETECT_CCD_CONTACT. This flags is used to control whether CCD performs sweep tests for a give pair. Decision over whether any collisions are responded to is made by the presence of the flag eSOLVE_CONTACT.
                
####  Removed:
                
*  CCD is now enabled per-body instead of per-shape. As a result, PxShapeFlag::eUSE_SWEPT_BOUNDS has been removed and replaced with PxRigidBodyFlag::eENABLE_CCD.
                
####  Changed:
                
*  API attributes previously named SWEPT_INTEGRATION have now been renamed 'ccd': specifically PxSceneFlag::eENABLE_SWEPT_INTEGRATION, PxSceneFlag::eSWEPT_INTEGRATION_LINEAR, PxSceneDesc::sweptIntegrationLinearSpeedFactor, PxSceneDesc::sweptIntegrationAngularSpeedFactor, PxPairFlag::eENABLE_SWEPT_INTEGRATION
*  PxPairFlag::eCCD_LINEAR has been deprecated. Use (PxPairFlag::eDETECT_CCD_CONTACT | PxPairFlag::eSOLVE_CONTACT) instead.
                
####  Fixed:
                
*  Updated CCD algorithm which improves the fluidity of motion of the CCD system while reducing the processing overhead significantly.
*  Contact notification is now reliable in CCD. CCD contacts are appended to the end of the contact list for a given pair so that both discrete and continuous contacts are reported
*  CCD contact notification now reports applied forces. These contacts will be correctly filtered by force thresholds.
                
            

### Serialization
            
####  Added:
                
*  Added extension PxSerialization::isSerializable method to query whether a collection is serializable.
*  Added extension PxSerialization::complete which allows to prepare a collection for serialization.
*  Added extension PxSerialization::createNames which adds reference names to serializables.
*  Added extension PxCollectionExt::remove which removes all serializables of a certain type and optionally adds them to another collection.
*  Added extension function PxCollectionExt::releaseObjects to remove and release objects from a collection
*  Added class PxSerializationRegistry for handling custom serializable types.
*  Added PxSerialization::serializeCollectionToXml and PxSerialization::createCollectionFromXml
*  Included pre-built binary meta data with SDK at [path to installed PhysX SDK]/Tools/BinaryMetaData
*  Added class PxSerializer to provide serialization specific functionality to serializable classes.
*  Added classes PxSerializationContext and PxDeserializationContext to provide functionality for serialization and deserialization operations.
                
####  Removed:
                
*  Removed PxUserReferences class and PxPhysics::createUserReferences.
*  Removed RepX.h and unified serialization interfaces for xml and binary serialization.
                
####  Changed:
                
*  PxCollection was reworked to improve reference management rendering PxUserReferences obsolete. PxCollection was also decoupled more from Serialization/Deserialization functionality. Serialization of incomplete collections fails now early at serialization as opposed to late at deserialization.
*  Replaced PxPhysics::createCollection with PxCreateCollection.
*  Replaced RepXCollection with PxCollection, and unified corresponding interfaces.
*  Replaced PxCollection::serialize with PxSerialization::serializeCollectionToBinary.
*  Replaced PxCollection::deserialize with PxSerialization::createCollectionFromBinary.
*  Replaced PxSerializable::collectForExport(PxCollection& c) with PxSerializer::requires. The new method works in a non-recursive way.
*  Replaced PxDumpMetaData with PxSerialization::dumpMetaData.
*  Replaced PxCollectForExportSDK with PxCollectionExt::createCollection(PxPhysics& sdk).
*  Replaced PxCollectForExportScene with PxCollectionExt::createCollection(PxScene& scene).
*  Moved PxCooking::createBinaryConverter to PxSerialization
*  Changed PxShape release semantics for shapes. As a consequence deserialized shapes are never autmatically released, but need to be released by the application. Exception: PxPhysics::release.
                
            

### Cloth
            
####  Added:
                
*  Improved GPU cloth performance significantly with new parallel solver.
*  Added tether constraints, which allow minimum amount of cloth stretching even for large gravity scales. See PxClothFabric.
*  Added support for dynamic addition and deletion of collision primitives.  See PxCloth::addCollision* and PxCloth::removeCollision*.
*  Added triangle mesh collider support. See PxCloth::setCollisionTriangles. 
*  Added support for self collision and inter-cloth collision. See PxCloth::setSelfCollision*() and PxScene::setClothInterCollision*().
*  Added direct access to CUDA particle data for graphics interoperability, see PxCloth::lockParticleData()
*  Added PxRegisterCloth to reduce binary size by stripping unused code on platforms where static linking is used.
*  Added methods setWakeCounter/getWakeCounter() (see the corresponding API documentation for details).
                    
  * It is illegal to call wakeUp/putToSleep/isSleeping() on a PxCloth that has not been added to a scene.
                    
                
####  Changed:
                
*  Cloth solver does not use fibers any more. See PxClothFabric for changes in fabric API.
*  Moved PxCooking.cookClothFabric() to extensions. See PxClothFabricCooker and PxClothFabricCreate.
*  PxClothMeshDesc has been moved to extensions and now supports both triangle and quad representations.  See PxClothMeshDesc.
*  The scaling of damping and stiffness coefficients has been separated from the solver frequency and can now be set indepedently using PxCloth::setStiffnessFrequency().
*  PxCloth::setInertiaScale() has been split into linear, angular, and centrifugal components.  See PxCloth::set*IntertiaScale.
*  Drag coefficient has been split into linear and angular coefficient. See PxCloth::setLinearDragCoefficient and PxCloth::setAngularDragCoefficient.
*  Renamed PxCloth::lockClothReadData() to lockParticleData().  Added support for update operations on the returned particle arrays (as an alternative to setParticles()).
*  PxCloth::wakeUp() does not have a parameter anymore. Use setWakeCounter() instead to set a specific value.
*  PxCloth::getNbCollisionSpherePairs() has been renamed to PxCloth::getNbCollisionCapsules()
                
####  Fixed:
                
*  Fixed a crash bug in the clothing collision code appearing on iOS.
                
            

### Rigid Bodies
            
####  Added:
                
*  The contact distance parameter for a limit is automatically estimated if not supplied in the constructor for the limit structure.
*  The new callback PxConstraintConnector::onOriginShift() has been introduced. It gets called for all constraints of a scene, when its origin is shifted.
*  New helper function PxRigidBodyExt::computeVelocityDeltaFromImpulse has been added.
*  Shapes may be declared as shared on creation, and then attached to multiple actors, see the user manual for restrictions.
*  Since a shape is no longer necessarily associated with a unique actor, references to shapes in callbacks from the engine are accompanied by the a reference to the associated actor
*  Joints and contact modification now support tuning relative mass and inertia for the bodies on a per-contact basis. Inertia and mass can be tuned independently.
                
####  Removed:
                
*  PxShape::overlap(), PxShape::sweep() and PxShape::raycast() have been removed. Equivalent functionality is provided in PxGeometryQuery.
                
####  Changed:
                
*  It is illegal to call resetFiltering() on a PxShape or PxParticleBase if they have not been added to a scene.
*  It is illegal to call addForce/addTorque/clearForce/clearTorque() on a PxRigidBody that has not been added to a scene.
*  The sleep behavior of dynamic rigid bodies has changed significantly (for details on the current behavior see the API documentation of isSleeping(), wakeUp(), putToSleep(), setKinematicTarget(), PxRigidBodyFlag::eKINEMATIC, ...). Among the changes are:
                        
  * The methods setWakeCounter/getWakeCounter() have been added for PxRigidDynamic and PxArticulation objects (see the corresponding API documentation for details).
  * The wakeUp() method of PxRigidDynamic and PxArticulation has lost the wake counter parameter. Use setWakeCounter() instead to set a specific value.
  * It is illegal to call wakeUp/putToSleep/isSleeping() on a PxRigidDynamic or PxArticulation that has not been added to a scene.
  * Putting a dynamic rigid actor to sleep will clear any pending force updates.
  * Switching a dynamic actor to kinematic will put the actor to sleep immediately.
  * Switching a kinematic actor back to dynamic will not affect the sleep state (previously the actor was woken up).
  * Calling wakeUp/putToSleep() on a kinematically controlled dynamic actor is not valid any longer. The sleep state of a kinematic actor is solely defined based on whether a target pose has been set (see API documentation of isSleeping() for details).
  * A call to PxRigidBody::setCMassLocalPose() does not wake up the actor anymore. Note: this also affects related methods in PhysXExtensions like PxRigidBodyExt::updateMassAndInertia() etc.
  * If a non-zero velocity or force is set through PxRigidBody::setLinearVelocity(), ::setAngularVelocity(), ::addForce() or ::addTorque(), the actor will get woken up automatically even if the autowake parameter is false.
  * PxRigidBody::clearForce() and ::clearTorque() do not have the autowake parameter, to optionally wake the actor up, anymore. These methods will not change the sleep state any longer. Call ::wakeUp() subsequently to get the old default behavior.
  * Adding or removing a PxConstraint/PxJoint to/from the scene does not wake the connected actors up automatically anymore.
  * It is now possible to avoid automatic wake up of previously touching objects on scene removal. See the additional parameter wakeOnLostTouch in PxScene::removeActor(), ::removeArticulation(), ::removeAggregte(), PxRigidActor::detachShape().
                        
                    
*  PxJointLimit and PxJointLimitPair are now PxJointLinearLimit, PxJointLinearLimitPair, PxJointAngularLimitPair, depending on whether the limit is linear or angular.
*  Joints now solve for the entire position error rather than a ratio of 0.7 of it. The flag PxConstraintFlag::eDEPRECATED_32_COMPATIBILITY can be used to restore this behavior
*  PxConstraintFlag::Type has been renamed to PxConstraintFlag::Enum
*  
*  The spring constant parameter in joints and articulations that was previously 'spring' is now 'stiffness'.
*  The tangential spring constant parameter in articulations that was previously 'tangentialSpring' is now 'tangentialStiffness'.
*  Constraints do not respect PxDominanceGroup settings. Use PxJoint::setInvMassScale and setInvInertiaScale
*  Shapes are reference counted. PxShape::release() now decrements the reference count on a shape, and its use is deprecated for detaching a shape from its actor - use detachShape() instead.
*  Shape creation methods do not take a local transform parameter anymore. Instead PxShapeFlags can be specified. Triangle meshes, height fields and plane geometry shapes cannot be combined with non-kinematic PxRigidDynmic actors if PxShapeFlag::eSIMULATION_SHAPE is specified. Corresponding calls to PxRigidActor::createShape() or PxRigidActor::attachShape() are not supported.
*  PxShape::getActor() now returns a pointer, which is NULL if the shape is shareable.
*  PxShape::getWorldBounds() has been replaced with PxShapeExt::getWorldBounds().
*  PxContactPoint has been renamed PxFeatureContact.
*  The internal format for contact storage has been modified; applications directly accessing the internal contact representation rather than PxContactPair::extractContacts should be modified accordingly.
*  Friction mode flags eENABLE_ONE_DIRECTIONAL_FRICTION and eENABLE_TWO_DIRECTIONAL_FRICTION have been replaced by PxFrictionType::Enum PxSceneDesc::frictionType.
*  PxSceneDesc::contactCorrelationDistance has been deprecated.
*  PxSceneDesc::contactCorrelationDistance no longer has an influence on how many friction anchors are created in a single frame, only on when they are removed in later frames.  This may cause a very minor change in friction behavior.
                
####  Fixed:
                
*  Rigid bodies now properly accumulate the forces/torques that are applied with addForce/addTorque while scene insertion is still pending.  This affects bodies added to the scene while the scene is simulating and then given forces/torques with addForce/addTorque while the scene is simulating.  These accumulated forces and torques are applied during the next simulate() call.  Prior to this fix the forces/torques that accumulated while scene insertion was pending were lost and never applied.
*  Its now possible to serialize collections with jointed actors without including the corresponding joints in the collection. The deserialized actors will not be jointed anymore.
*  Joint drive force limits are actual force limits rather than impulse limits. Set the flag PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES to false to support legacy behavior
*  Angular drive constraints for revolute joints were wrongly computed, resulting in a negative angular velocity when a positive drive target was set.
*  Scene addActors will now correctly remove all actors that were passed to add, if some insert failed.
*  Contact modification now enforces max impulse field correctly. Previously, it only enforced it if max impulse was set to 0.
*  Contact modification now supports target velocity in all directions. Previously, it only enforced the target velocity components that were perpendicular to the contact normal.
*  Jittering of small spheres & capsules on very large triangles has been fixed.
*  Setting sleep threshold to 0 now guarantees that bodies won't fall asleep even if their kinetic energy reaches exactly 0.
                
####  Deprecated:
                
*  PxShape::release() now decrements the reference count on a shape, and its use is deprecated for detaching a shape from its actor - use detachShape() instead.
*  PxJoint::getType() is deprecated - use getConcreteType() instead.
*  PxConstraintFlag::eREPORTING is deprecated - constraints always generate force reports
*  PxConstraintDominance is deprecated - use PxDominanceGroupPair instead.
                
            

### Scene queries
            
####  Added:
                
*  PxVolumeCache, a volumetric cache for local collision geometry.
*  Parameter inflation was added to some PxGeometryQuery functions.
*  Flag eMESH_BOTH_SIDES can be now used to control triangle mesh culling for raycasts and sweeps.
*  Added PxBatchQueryMemory as a part of PxBatchQueryDesc, to allow memory buffers to be set before execution.
*  PxBatchQueryDesc() constructor now requires 3 parameters at initialization - see migration guide for more details.
*  PxBatchQuery::raycast, sweep, overlap calls will now issue a warning and discard the query when over maximum allowed queries.
*  There is a new flag to allow manually updating the scene query representation, see: PxSceneFlags::eENABLE_MANUAL_SQ_UPDATE and PxScene::flushQueryChanges().
*  Added PxScene::forceDynamicTreeRebuild() function to immediately rebuild the scene query structures.
*  Added bool PxSweepHit::hadInitialOverlap() returning true if a sweep hit occurred early due to initial overlap at distance=0.
                
####  Removed:
                
*  PxBatchQuery::linearCompoundGeometrySweepSingle and PxBatchQuery::linearCompoundGeometrySweepMultiple functions are no longer supported.
*  Globals PxPS3ConfigParam::eSPU_OVERLAP, eSPU_RAYCAST, eSPU_SWEEP that were previous set via setSceneParamInt call are replaced with PxBatchQueryDesc::runOnSpu. See migration guide for more details.
                
####  Changed:
                
*  Scene Query raycastAny/Single/Multiple APIs were merged into a single raycast() call (same for overlaps and sweeps). Please refer to user and migration guides for details.
*  Scene Query raycast() API now uses a PxRaycastBuffer or a PxRaycastCallback parameter for reporting hits. Blocking hits are now reported separately from toching and PxRaycastCallback class supports reporting an unbounded number of results (same for overlaps and sweeps).
*  A const templated PxRaycastBufferN<int> object was added to allow convenient creation of fixed size scene query touch buffers. Same for overlaps and sweeps.
*  Support for compound sweeps was moved out from core SDK to extensions.
*  Support for compound sweeps was moved from PxScene to extensions (see PxRigidBodyExt::linearSweepSingle, PxRigidBodyExt::linearSweepMultiple).
*  PxQueryFilterCallback::preFilter now passes an actor pointer as well as a shape pointer.
*  PxSceneQueryFlag::eINITIAL_OVERLAP and PxSceneQueryFlag::eINITIAL_OVERLAP_KEEP have been replaced with PxHitFlag::eINITIAL_OVERLAP_DISABLE and PxLocationHit::hadInitialOverlap(). Note that checking for initial overlap is now the defaut for sweeps.
*  Sweeps in 3.3 execute using a new faster code path, in some cases with reduced precision. If you encounter precision issues not previously experienced in earlier versions of PhysX, use ePRECISE_SWEEP flag to enable the backwards compatible more accurate sweep code.
*  The default behavior for overlap queries with query filters returning eBLOCK has changed to only return one of eBLOCK hits. Please refer to the migration guide for details.
                
####  Fixed:
                
*  Scene Query performance was significantly improved in a variety of scenarios.
*  Fixed a bug in capsule/mesh overlap code that occasionally caused unreported and misreported faces.
*  Fixed a crash in raycastMultiple when a convex was hit by the ray and eMESH_MULTIPLE flag was specified as a query flag.
*  Fixed a rare crash in heightfield raycast code.
*  Internal limit of 65536 results has been removed.
*  Accuracy in sphere/capsule-vs-triangle overlap and sweep tests has been improved.
                
            

### Cooking
            
####  Added:
                
*  Added support for convex hull creation with limited output vertices count.
*  Added support for convex hull creation directly from polygons rather than triangles.
*  Added support function computeHullPolygons in cooking, that creates hull polygons from given vertices and triangles. The resulting polygons can be used to create the convex hull directly.
                
####  Changed:
                
*  Changed convex hull volume integrals.
*  PxCookingParams constructor requires now PxTolerancesScale as an additional parameter. This enables us to perform further checks on the triangles during cooking. A warning will be emitted to the error stream if too huge triangles were found. This will ensure better simulation stability.
                
####  Fixed:
                
*  Optimized heightfield load code for no-endian conversion case.
                
            

### Triangle meshes
            
####  Added:
                
*  Added PxTriangleMeshFlag::eHAS_ADJACENCY_INFO flag for adjacency information checks.
                
####  Removed:
                
*  Removed has16BitTriangleIndices(), replaced by triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::eHAS_16BIT_TRIANGLE_INDICES.
                
            

### Particles
            
####  Added:
                
*  Direct read access to CUDA particle data has been added for graphics interoperability, see PxParticleBase::lockParticleReadData(PxDataAccessFlags flags) and PxParticleFluid::lockParticleFluidReadData(PxDataAccessFlags flags)
*  Added PxRegisterParticles to reduce binary size by stipping unused code on platforms where static linking is used.
*  Added setExplicitCudaFlushCountHint to allow early flushing of the cuda push buffer.
*  Added caching of triangle meshes. setTriangleMeshCacheSizeHint is supported on Kepler and above GPUs.
                
####  Fixed:
                
*  Creating and immediately setting the position of particles failed and possibly crashed with GPU acceleration enabled. This would only happen after one or more simulation updates.
*  Creating many spread out particles might have crashed the GPU pipeline.
                
            

### Broad Phase
            
####  Added:
                
*  The SDK now supports multiple broad-phase algorithms (SAP & MBP). See Release Highlights, PxBroadPhaseType and PxBroadPhaseDesc.
*  PxVisualizationParameter::eMBP_REGIONS has been added to visualize MBP regions
                
####  Fixed:
                
*  The sap broadphase now gracefully handles PxShape instances whose axis-aligned bounding boxes have min/max limits with value  +/- PX_MAX_F32 or QNAN or INF. Such bounds values can occur if a PxShape is given a global transform that is either illegal or close to the upper limit of the floating point range. Prior to this release, the sap broadphase could crash when the axis-aligned bounds of shapes had values that weren't within the floating point range.  This has now been fixed.  The overlap pairs reported by the broadphase for bounds with such values is undefined.
                
            

### Vehicles
            
####  Added:
                
*  Vehicles now support serialization. A PxSerializationRegistry instance may be passed into PxInitVehicleSdk and PxCloseVehicleSdk in order to enable support.
*  Vehicle telemetry graphs can now be queried per channel for the most recent value with the function PxVehicleGraph::getLatestValue.
*  New vehicle PxVehicleDriveNW type has been introduced.  This class makes use of a new differential type PxVehicleDifferentialNW, which allows specified wheels to be equally coupled to the differential, and allows all for some or all of the N wheels to be driven. 
*  Support for camber angles has been added to the PxVehicleSuspensionData class.
*  Moment of inertia parameter has been added to the PxVehicleEngineData class. Prior to this a value of 1.0 was assumed internally.  A default value of 1.0 has been used in the constructor for backwards compatability.
*  Vehicle manual contains new section describing the conversion of default vehicle parameter values from SI units to any system of units with particular reference to the use of centimetres instead of metres .
*  The SI units of each parameter in PxVehicleComponents.h has been documented. 
*  Vehicle manual contains updated troubleshooting section.
*  The requirements for disabled wheels (PxVehicleWheelsSimData::disableWheel) have been documented to make it clear that disabled wheels must be no longer associated with a PxShape, must have zero rotation speed, and must be decoupled from the differential.  This is also now discussed in the guide.
*  Wheel raycasts documentation has been improved to clarify the start and end points of each raycast.
*  Suspension line raycasts do not need to be performed for each vehicle prior to update with PxVehicleUpdates.  This feature is implemented with a boolean array passed as an extra function argument to PxVehicleSuspensionRaycasts.  This feature is useful for vehicles that require only low level of detail.
*  The clutch model now supports two accuracy modes (PxVehicleClutchAccuracyMode::eESTIMATE and PxVehicleClutchAccuracyMode::eBEST_POSSIBLE).  If the estimate mode is chosen the computational cost and accuracy of the clutch can be tuned with PxVehicleClutchData::mEstimateIterations.
*  PxVehicleSuspensionData now has a function setMassAndPreserveNaturalFrequency.  This modifies the mass and stiffness of the spring in a way that preserves the spring natural frequency.
*  A new helper function PxVehicleCopyDynamicsData has been added that allows dynamics data such as engine rotation speed, wheel rotation speed, gear etc to be copied from one vehicle to another of the same type.  This is particularly useful if a vehicle has a number of different versions where each represents a different level of detail.
*  A new function PxVehicleWheelsSimData::copy has been added to allow per wheel dynamics data to be copied from one vehicle to another.
*  The vehicle manual contains a new section "Level of Detail" describing the available options for vehicles that require only a low level of detail.
*  PxVehicleTireLoadFilterData now requires that mMinNormalisedLoad is greater than or equal to zero.
*  PxVehicleTireLoadFilterData now has a new member variable mMinFilteredNormalisedLoad.  This value describes the filtered normalised load that occurs when the normalised is less than or equal to mMinNormalisedLoad.
*  PxVehicleWheelsSimData now has a new function setMinLongSlipDenominator.  This can be used to tune stability issues that can arise when the vehicle slows down in the absence of brake and drive torques.
*  A new section "PxVehicleAutoBoxData" has been added to the vehicle tuning guide to describe operation of the automatic gearbox.
*  A new section "The Vehicle Under-steers Then Over-steers" has been added to the vehicle troubleshooting guide to describe steps to avoid twitchy handling on bumpy surfaces. A new section "The Vehicle Never Goes Beyond First Gear" has been added to the vehicle troubleshooting guide to describe a common scenario that occurs when the automatic gearbox is given a latency time that is shorter than the time taken to complete a gear change.
*  A new section "The Vehicle Slows Down Unnaturally" has been added to the vehicle troubleshooting guide to describe the steps that can be taken to help the vehicle slow down more smoothly.
*  A number of vehicle snippets have been added.
                
####  Changed:
                
*  Minor api change for consistency: PxVehicleDrive4WWheelOrder has been introduced to replace the enum beginning with PxVehicleDrive4W::eFRONT_LEFT_WHEEL.
*  Minor api change for consistency: PxVehicleDrive4WControl has been introduced to replace the enum beginning with PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL.
*  Minor api change for consistency: PxVehicleDriveTankWheelOrder has been introduced to replace the enum beginning with PxVehicleDriveTankWheelOrder::eFRONT_LEFT.
*  Minor api change for consistency: PxVehicleDriveTankControl has been introduced to replace the enum beginning with PxVehicleDriveTank::eANALOG_INPUT_ACCEL.
*  Minor api change for consistency: PxVehicleDriveTankControlModel has been introduced to replace the enum beginning with PxVehicleDriveTank::eDRIVE_MODEL_STANDARD.
*  Minor api change for consistency: PxVehicleTypes has been introduced to replace the enum beginning with eVEHICLE_TYPE_DRIVE4W.
*  Minor api change for consistency: PxVehicleWheelGraphChannel has been introduced to replace the enum beginning with PxVehicleGraph::eCHANNEL_JOUNCE.
*  Minor api change for consistency: PxVehicleDriveGraphChannel has been introduced to replace the enum beginning with PxVehicleGraph::eCHANNEL_ENGINE_REVS.
*  Minor api change for consistency: PxVehicleGraphType has been introduced to replace the enum beginning with PxVehicleGraph::eGRAPH_TYPE_WHEEL.
*  PxVehicleUpdates now checks in checked and debug config that the wheel positioning of the driven wheels in a PxVehicleDrive4W obey the wheel ordering specified in PxVehicleDrive4WWheelOrder.  Vehicles that are back-to-front or right-to-left are also allowed. A warning is issued if the check fails.
*  PxVehicleUpdates now checks in checked and debug config that the odd wheels of a PxVehicleDriveTank are either all to the left or all to the right of their even-wheeled complement, as specified in PxVehicleDriveTankWheelOrder.  A warning is issued if the check fails.
*  To improve api consistency the arguments of the function PxVehicleDriveDynData::setAnalogInput(const PxReal analogVal, const PxU32 type) have been swapped so that it is now of the form PxVehicleDriveDynData::setAnalogInput(const PxU32 type, const PxReal analogVal).
*  Non-persistent wheel dynamics data (slip, friction, suspension force, hit data etc) has been moved out of the PxVehicleWheelsDynData class and is now recorded in a PxWheelQueryResult buffer that is passed as a function argument to the PxVehicleUpdates function.
*  PxVehicleWheels::isInAir() has been replaced with PxVehicleIsInAir(const PxVehicleWheelQueryResult& vehWheelQueryResults) to reflect the change to non-persistent data storage.
*  The origin of vehicles can now be shifted to stay in sync when the origin of the underlying PxScene is shifted. See PxVehicleShiftOrigin() for details.
*  PxVehicleWheels::setWheelShapeMapping and PxVehicleWheels::getWheelShapeMapping have been moved to PxVehicleWheelsSimData::setWheelShapeMapping and PxVehicleWheelsSimData::getWheelShapeMapping
*  PxVehicleWheels::setSceneQueryFilterData and PxVehicleWheels::getSceneQueryFilterData have been moved to PxVehicleWheelsSimData::setSceneQueryFilterData and PxVehicleWheelsSimData::getSceneQueryFilterData
*  PxVehicle4WEnable3WTadpoleMode and PxVehicle4WEnable3WDeltaMode now take an extra function argument: a non-const reference to a PxVehicleWheelsDynData.
*  The section "SI Units" in the vehicle guide has been updated to include the new functon PxVehicleWheelsSimData::setMinLongSlipDenominator.
*  PxVehicleTireData::mCamberStiffness has been replaced with PxVehicleTireData::mCamberStiffnessPerUnitGravity.  PxVehicleTireData::mCamberStiffnessPerUnitGravity should be set so that it is equivalent to the old value of PxVehicleTireData::mCamberStiffness divided by the magnitude of gravitational acceleration.
*  PxVehicleComputeTireForceDefault has been removed from the public vehicle api. Custom tire shaders that call PxVehicleComputeTireForceDefault are best implemented by taking a copy of PxVehicleComputeTireForceDefault and calling the copy instead.
                
####  Fixed:
                
*  Sticky tire friction is now activated in the tire's lateral direction at the tire force application point when the velocity at the base of the tire has low longitudinal and low lateral components. Longitudinal sticky tire friction is unaffected and is still activated when the vehicle has low forward speed. This fixes a minor bug where vehicles positioned on a slope perpendicular to the slope's downwards direction can slowly drift down the slope.
*  Bugs in the suspension force and tire load computation have been fixed that affected handling when the car was upside down.
*  The tire load passed to the tire force computation is now clamped so that it never falls below zero.
*  A bug in the tank damping forces has now been fixed.  Tanks now slow down more aggressively from engine and wheel damping forces.
                
            

        
        
        


# v3.2.4

April 2013

## What's New In NVIDIA PhysX 3.2.4

        
#### General
                
*  <i>Note: PS3 platform specific changes can be found in the PS3 readme file.</i>
*  Fixed a bug which caused actors to return wrong world bounds if the bounds minimum was above 10000 on any axis.
*  Reporting allocation names can now be enabled or disabled (see PxFoundation::setReportAllocationNames). When enabled, some platforms allocate memory through 'malloc'.
*  eEXCEPTION_ON_STARTUP is removed from PxErrorCode and it is no longer needed.
*  Added boilerplate.txt to the Tools folder. SpuShaderDump.exe and clang.exe require it.
*  PxWindowsDelayLoadHook.h has been moved from Include/foundation/windows to Include/common/windows.
*  PxScene::saveToDesc now reports the bounceThresholdVelocity value.
*  Fixed a bug in PxDefaultSimulationFilterShader: the value of the second filter constant in the collision filtering equation was ignored and instead the value of the first filter constant was used.
*  Fixed a crash bug in PCM collision.
                


#### Rigid Bodies
                
*  Forces applied to bodies (with PxRigidBody::addForce) that go to sleep in the subsequent update now have their applied forces cleared when the body is set to sleep to avoid them being applied in a later update when the body is once more awake.  This bug broke the rule that forces applied with PxRigidBody::addForce do not persist beyond the next scene update.
*  Jittering of small spheres & capsules on very large triangles has been fixed.
                

#### Cooking
                
*  PxCookingParams constructor is now marked as deprecated. PxToleranceScale is needed for PxCookingParams in order to perform additional triangle check during cooking. This triangle check will trigger warning if too huge triangles are used. This check will ensure better simulation stability.
                

#### Scene Queries
                
*  Added PxScene::forceDynamicTreeRebuild() function to immediately rebuild the scene query structures.
*  Accuracy in sphere/capsule-vs-triangle overlap and sweep tests has been improved.
                

#### Broad Phase
                
*  Fixed assert in debug mode that wrongly asserted when an overlap pair was removed with perfect equality between the min of one bound and the max of the other along at least one axis.
                

#### Character controller
                
*  PxControllerFilterCallback has been added, to make CCT-vs-CCT filtering more flexible.
*  Fixed a bug where PxControllerBehaviorCallback was not called when a PxUserControllerHitReport was not defined.
*  PxObstacle notifications has been added to handle touched obstacles.
*  Touched PxObstacle has been replaced by touched ObstacleHandle.
*  PxObstacleContext::getObstacleByHandle has been added.
*  Touched actors scene is checked before ride on shape, to ensure valid touched shape.
*  Touched shape scene query flag is checked before ride on shape, to ensure valid touched shape.
*  Touched shape user CCT filtering is triggered before ride on shape, to ensure valid touched shape.
*  Box & capsule controllers' resize function now properly take the up direction into account
                

#### Vehicles
                
*  Documented potential problem with PxVehicleWheelsDynData::getTireDrivableSurfaceType() and PxVehicleWheelsDynData::getTireDrivableSurfaceShape() whereby the pointer returned may reference a PxShape or PxMaterial that has been released in-between storing the pointer in PxVehicleUpdates and any subsequent query.
*  PxVehicleWheelsSimData::disableWheel has been documented in more detail.  PxVehicleWheelsSimData::enableWheel has been added.
*  Fixed a bug where the denominator of the longitudinal slip calculation didn't take into account the value of PxTolerancesScale::length.  This will only have an impact if PxTolerancesScale::length != 1.0f.
*  Fixed a bug where the engine torque would be incorrectly applied if PxTolerancesScale::length != 1.0f.  This will only have an impact if PxTolerancesScale::length != 1.0f.
                

#### Particles
                
*  Creating and immediately setting the position of particles failed and possibly crashed with GPU acceleration enabled. This would only happen after one or more simulation updates.
                

        

## Supported Platforms

        
Unchanged from  from 3.2.3 except:
#### Development
                
*  Upgraded to Xcode 4.6
                
        

## Known Issues And Limitations

        
Unchanged from  from 3.2.3.
        
        


        
        
        


# v3.2.3

November 2012

## What's New In NVIDIA PhysX 3.2.3

        
#### General
                
*  <i>Note: PS3 platform specific changes can be found in the PS3 readme file.</i>
*  Quaternions passed through the API are now considered valid if their magnitude is between 0.99 and 1.01.
*  Fixed crash when running out of memory on creation of a triangle mesh.
*  For applications using floating point exceptions, the SDK will now mask or avoid exceptions arising from invalid floating point operations (inexact and underflow exceptions may still be generated).
*  Fixed a bug with recursive use of the PxScene read/write lock.
*  Fixed a shutdown bug with very short lived threads on Linux platforms.
*  PhysX version number in error messages now printed in hex for easier reading.
*  Fixed memory barrier and prefetch implementation for all posix based platforms (android, ios, osx, linux).
                

#### Broad Phase
                
*  Fixed a broad phase crash bug that occurred when deleting shapes with bounds very far from the origin.
                


#### Collision Detection
                
*  Documentation of limits of PxShape counts has been added for affected platforms.
*  Made kinematics interact better with CCD.
*  Adding support for disabled contact response in CCD by respecting the dominance setting.  In this case, CCD will emit events but will not alter the motion of the bodies.
*  Fixed potential crash in eENABLE_PCM codepath.
                


#### Rigid Bodies
                
*  Fixed bug in force based contact reports. An assert could occur when PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS was set and PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND was not set.
*  Twist Limit range is documented for revolute and D6 joints, and validated.
*  Reduced the number of SDK allocations when using CCD.
                


#### Scene Queries
                
*  Raycasts against heighfields now return correct actual mesh index, which can be used for getTriangle().
*  Fixed bug that caused scene queries to miss hits for static rigid bodies that got moved around (after having been added to the scene).
*  Fixed rebuilding the dynamic structure properly when used for static rigid bodies.
*  Fixed a rare crash in heightfield raycast code.
                

#### Character controller
                
*  A new runtime tessellation feature has been added, that can help reducing FPU accuracy issues in the sweep tests.
                

#### Convex hulls
                
*  Zero-sized convex hull data double delete detection fix.
                

#### Vehicles
                
*  Vehicles with rigid body actors that are asleep and also have no acceleration or steer inputs are treated as though they are asleep too; that is, they are bypassed completely in the PxVehicleUpdates function.  Vehicles with sleeping rigid body actors but with non-zero acceleration or steer inputs are processed as normal in PxVehicleUpdates and will automatically have their rigid body actor woken up.
*  New function PxVehicleSetUpdateMode to allow PxVehicleUpdates to select between applying accelerations to vehicle rigid bodies or immediate updating of their velocity.
                

#### Particles
                
*  Fixed a non-deterministic crash appearing with rigid bodies using CCD and gpu particles in the same scene.
                

#### Physx Visual Debugger
                
*  Material release events are now correctly sent to PVD. 
                

#### RepX
                
*  Add more RepX class information in PhysXAPI document. 
                

        

## Supported Platforms

        
#### Runtime
                
*  Apple iOS
*  Apple Mac OS X
*  Google Android (version 2.2 or later for SDK, 2.3 or later required for samples)
*  Linux (tested on Ubuntu)
*  Microsoft Windows XP or later (NVIDIA Driver ver 295.73 or later is required for GPU acceleration)
*  Microsoft Windows RT (formerly known as Windows on ARM) (SDK only, no samples yet)
*  Microsoft XBox 360
*  Sony Playstation 3
                
#### Development
                
*  Microsoft Windows XP or later
*  Microsoft Visual Studio 2008, 2010
*  Xcode 4.5
                
        

## Known Issues And Limitations

        
Unchanged from  from 3.2.2.
        
        


        
        
        


# v3.2.2

October 2012

## What's New In NVIDIA PhysX 3.2.2

        
#### General
                
*  <i>Note: PS3 platform specific changes can be found in the PS3 readme file.</i>

*  Added Microsoft Windows RT (formerly known as Windows on ARM) support.
*  Suspended Sony Playstation Vita support.
*  PxScene now exposes methods to make multithreaded sharing of scenes easier, see PxSceneFlags::eREQUIRE_RW_LOCK for details.
*  Enabled Win64 DEBUG build to use SIMD enabled code path.
*  Fixed bug in quaternion to axis/angle routine which failed on negative w values.
*  Fixed crash when using ConvX on a PxCollection with external references.
*  Fixed a spurious overlapping read/write error report when using simulation call backs in checked builds.
*  The bounce threshold velocity can be set at run-time with PxScene::setBounceThresholdVelocity. Likewise, it can be queried with PxScene::getBounceThresholdVelocity
*  Fixed return value of Ps::atomicExchange for POSIX based platforms: Linux, Android, IOS and OSX
*  PxGeometryQuery::computePenetration() has been added, to compute the Minimum Translational Distance between geometry objects.
                

#### Broad Phase
                
*  Fixed a broad phase crash bug.
                

#### Collision Detection
                
*  Collision detection now more robust when confronted with ill-conditioned scenarios.
*  Fixed crash when SDK is unable to create more contact pairs.  Now a warning is emitted and the contacts are ignored.
                

#### Rigid Bodies
                
*  Improved the numerical stability of articulations.
*  The target pose of a kinematically controlled dynamic actor can now get extracted through PxRigidDynamic::getKinematicTarget().
*  The new flag PxRigidDynamicFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES makes scene queries use the target pose of a kinematically controlled dynamic actor instead of the actual pose.
*  Fixed PxConstraintFlag::eVISUALIZATION having no effect.
*  Fixed bug that caused sleep/wake-up notification events to get lost.
*  Fixed a bug where sleeping objects were not waking up properly.
*  Fixed potential assert when switching a rigid body between kinematic and dynamic with contact reports enabled.
*  Fixed a bug where CCD didn't consider the scaling transform for triangle meshes.
*  Fixed a potential crash bug when PxConstraintFlag::ePROJECTION gets raised after a joint (that connects to other projecting joints) has been created.
*  Fixed a bug that resulted in joint breakage being reported every frame for any broken joint
*  Added PxArticulationDriveCache for applying an impulse to an entire articulation
*  fixed various crash bugs when sleeping articulations were removed from the scene and re-added
*  
                

#### GPU Physics
                
*  Fixed a possible out of bounds array access in GPU particle collision code.
*  Updated the GPU PhysX Visual Indicator to allow APEX to hook into it.
*  Fixed sample particles crash when there is a cuda kernel error.
*  Upgraded GPU tech to CUDA 4.2.
                

#### Scene Queries
                
*  PxSceneQueryHit::faceIndex is now filled in for sweeps against convex meshes.
*  Added support for sphere and capsule shaped heightfield overlap queries.
*  Added an inflation parameter to all sweep queries in PxGeometryQuery, PxBatchQuery, and PxScene.  This can be used to maintain a minimum distance between objects when moving them using sweeps.
*  Made sure that raycast multiple will include the closest triangle in the results even if the output cannot hold all the triangles that are hit.
*  Fixed swept sphere against capsule not properly testing for initial overlap.
*  Fixed the normal vector returned from sweeps being sometimes negated.
*  Fixed a scenario where raycasting could miss an actor after the user has moved it using setGlobalPose().
*  Fixed swept convex against plane sometimes reporting false hits
*  Fixed swept/overlap/raycast with convexes that don't have identity scale rotation.
*  Fixed a culling bug for box-triangle mesh sweep.
                

#### Convex hulls
                
*  Convex hull is rejected if it has less than 4 polygons.
*  Additional convex hull check has been added to detect open volumes.
                

#### Triangle meshes
                
*  Added triangle mesh flags for 16bit indices and adjacency information.
*  Fixed adjacency information order for getTriangle with triangle meshes to respect the vertex order.
                

#### HeightFields
                
*  Fixed bug where capsules would bounce as they roll across heightfield edges.
*  Fixed bug where spheres would bounce as they roll across heightfield vertices.
*  Added adjacency information for getTriangle with height fields.
                

#### Particles
                
*  Fixed triangle mesh shapes with ePARTICLE_DRAIN colliding with particles.
*  Fixed crash with GPU particles when a lot of grid cells get occupied and vacated within one time step.
                

#### Character Controller
                
*  The PxControllerBehaviorFlag::eCCT_USER_DEFINED_RIDE flag has been added.
*  Fixed character controller not walking up steps properly if they are not exactly 90 degrees vertical.
*  Fixed a bug where the character controller was not able to move up slopes when using a small step offset setting.
*  Fixed a bug where the character controller could rise off the ground when blocked by a step.
*  Fixed a bug where the character controller could rise off the ground when hitting another character controller.
*  Fixed a bug where releasing a shape of a touching actor and then releasing the character controller would crash. Releasing shapes of actors in touch may still lead to crashes in other situations. PxController::invalidateCache() can be used to work around these situations.
                
####  CCD 
                
*  Fixed culling bug in CCD sweeps with meshes with transforms that caused contacts to be missed 
                

#### Vehicles
                
*  The vehicle sdk used to make the assumption that wheels 0 and 1 (the front wheels) of a PxVehicleDrive4W responded positively to the input steering wheel angle, while wheels 2 and 3 (the rear wheels) responded in the opposite direction to that of the input steering wheel angle.  A consequence of this assumed behaviour was the restriction that PxVehicleWheelData::mMaxSteer was limited to positive values.  This restriction has now been relaxed with the caveat that PxVehicleWheelData::mMaxSteer must be in range (-Pi/2, Pi/2).  It is now possible to turn each wheel positively or negatively relative to the input steering wheel angle by choosing positive or negative values for PxVehicleWheelData::mMaxSteer.  Ackermann steer correction might result in the front and rear wheels competing against each other if the rear and front all steer in the same direction relative to the input steering wheel angle. If this is the case it will be necessary to set the Ackermann accuracy to zero.
*  It is now possible to set the engine rotation speed (PxVehicleDriveDynData::setEngineRotationSpeed), the rotation speed of each wheel (PxVehicleWheelsDynData::setWheelRotationSpeed) and the rotation angle of each wheel (PxVehicleWheelsDynData::setWheelRotationAngle). The default values for each of these properties remains zero.
*  Wheel contact reporting has been improved with the addition of a number of query functions to the PxVehicleWheelsDynData class.  These are getTireDrivableSurfaceContactPoint, getTireDrivableSurfaceContactNormal, getTireLongitudinalDir, getTireLateralDir, getSuspensionForce, getTireDrivableSurfaceShape.
*  It is now possible to store a userData pointer per wheel.  This allows, for example, each wheel to be associated with a game object.  The relevant functions are PxVehicleWheelsDynData::setUserData and PxVehicleWheelsDynData::getUserData.
*  The default behavior of PxVehicleWheels::setWheelShapeMapping has changed. Previously, default values were automatically assigned to each wheel at construction so that the ith wheel was mapped to the ith body shape.  This, however, made the assumption that there was a wheel shape for each wheel, which is not always true.  As a consequence, the default value is now -1, meaning any mapping between body shape and wheel has to be explictily made by calling setWheelShapeMapping.
*  It is now possible to query the mapping between wheel and body shape with PxVehicleWheels::getWheelShapeMapping.
*  It is now possible to query the tire shader data that has been applied to each wheel with PxVehicleWheelsDynData::getTireForceShaderData.
*  The helper function PxVehicleComputeSprungMasses has been added to aid the setup of the suspension sprung masses from the rigid body centre of mass and wheel center coordinates.
*  The scene query filter data applied to the suspension raycasts was previously taken from the filter data of the associated body shape.  This makes the assumption of a body shape per wheel, which is not always true.  As a consequence, the filter data must be explictly set per wheel by calling PxVehicleWheels::setSceneQueryFilterData.  The filter data can be queried with PxVehicleWheels::getSceneQueryFilterData.
*  Sub-stepping of the vehicle update can now be applied per vehicle with PxVehicleWheelsSimData::setSubStepCount.
*  PxVehicleDrivableSurfaceToTireFrictionPairs has been modified so that the dictionary of material pointers can be updated without the necessity of further allocation.  The create function has been replaced with separate allocate and setup functions.
*  A new vehicle type PxVehicleNoDrive has been added to provide a close approximation to backwards compatibility with the api of the 2.8.x NxWheelShape.
                


#### Visual Remote Debugger
                
*  Added PVD compatible profile zones for batched queries.
*  Added the ability to capture and inspect scene queries in PVD.
*  SDK will now flush the pvd connection stream immediately after cloth or cloth fabric is created or released.
*  Fixed the PVD support for articulations.
*  Fixed PVD rendering wrong constraint limits.
                


#### Documentation
                
*  Wrong statement in PxRigidStatic::release() has been corrected. Static rigid bodies do wake up touching dynamic rigid bodies on release.
*  Wrong statement in PxShape::setFlag() has been corrected. It is a valid operation to clear all flags.
*  Retroactively added more detail about changes to 3.2.1 release notes below.
                
        

## Supported Platforms

        
#### Runtime
                
*  Apple iOS
*  Apple Mac OS X
*  Google Android (version 2.2 or later for SDK, 2.3 or later required for samples)
*  Linux (tested on Ubuntu)
*  Microsoft Windows XP or later (NVIDIA Driver ver 295.73 or later is required for GPU acceleration)
*  Microsoft Windows RT (formerly known as Windows on ARM) (SDK only, no samples yet)
*  Microsoft XBox 360
*  Sony Playstation 3
                
#### Development
                
*  Microsoft Windows XP or later
*  Microsoft Visual Studio 2008, 2010
*  Xcode 4.2
                
        




## Known Issues And Limitations

        
#### General
                
*  Memory leaks might get reported when using the debug versions of malloc and free together with the debug version of PhysX on Microsoft Windows platforms with Visual Studio 2010. This is caused by a bug in the Visual Studio 2010 runtime libraries. If such a leak appears immediately after releasing PhysX and all its components, please contact us for information about workarounds.
*  Use of articulations may require an increase of 64K in the stack size for threads making API calls and engine worker threads
                
        

Please also see the previous lists  from 3.2.1 and earlier.
        


        
        
        


# v3.2.1

June 2012

## What's New In NVIDIA PhysX 3.2.1

        
#### General
                
*  <i>Note: PS3 platform specific changes can be found in the PS3 readme file.</i>
*  Added GRB hooks for APEX 1.2.1.
*  Some incorrect usages of __restrict have been fixed.
*  A crash when the user's code had FPU exceptions enabled has been fixed.
*  A rare crash on Win64 has been fixed.
*  Removed no longer needed RapidXML library from distribution.
*  Binary serialization can now save the names of actors and shapes.
*  Removed a RepXUtility.h reinterpret_cast compile warning that happened on some platforms.
*  Fixed a spurious overlapping read/write error report when using simulation call backs in checked builds.
*  Fixed a bug in PxBinaryConverter when processing PxConvexMesh and PxMaterial assets. 
*  Fixed bug in implementations of array accessors (PxPhysics::getScenes(), GuMeshFactory::getTriangleMeshes(), GuMeshFactory::getConvexMeshes(), GuMeshFactory::getHeightFields(), PxAggregate::getActors(), PxScene::getAggregates(), PxScene::getArticulations(), PxScene::getConstraints()  ) when startIndex > bufferSize.
*  Reduced the number of libraries provided for game consoles. The following core libraries are included in the PhysX3 library and are not available separately anymore: LowLevel, LowLevelCloth, PhysX3Common, PhysXProfileSDK, PhysXVisualDebuggerSDK, PvdRuntime, PxTask, SceneQuery, SimulationController.
                

#### Documentation
                
*  Clarified documentation regarding use of eSEND_SLEEP_NOTIFIES flag.
*  Clarified documentation regarding using different CRT libraries.
*  Removed some confusing statements about meta data from the documentation.
*  Updated PsPool, PsSort so they can use the user allocator.
                

#### Mesh Cooking
                
*  A warning message about negative volumes in the convex mesh cooker could cause a crash.
*  Fixed failure to create valid convex hull in cooking when using PxConvexFlag::eINFLATE_CONVEX.
*  The convex mesh cooking has been made more robust and properly outputs some error messages that were previously missing.
*  Fixed crash bug in x64 version of ClothEdgeQuadifier.
*  Adjacency information option for TriangleMeshes.  This is controlled using the PxCookingParams::buildTriangleAdjacencies flag, and returned if available by PxMeshQuery::getTriangle().
                

#### Broad Phase
                
*  The sdk gracefully handles the case of more than 65536 broadphase pairs and reports a warning that some contacts will be dropped in the event that this limit is exceeded.  This affects all platforms except win32/win64/linux/linux64, which support a maximum number of 4294967296 pairs.
                

#### Collision Detection
                
*  Fixed a memory corruption bug in heightfield code.
*  Fixed a bug in sphere vs mesh contact generation that could result in bad normals.
*  Added a flag to enable or disable contact caching, to permit users to make a performance vs memory tradeoff.
*  Fixed a crash bug in ContactReport cleanup code.
                

#### Rigid Bodies
                
*  The simultaneous raising of both the PxShapeFlag::eSIMULATION_SHAPE and PxShapeFlag::eTRIGGER_SHAPE flags is now explicitly forbidden by the sdk.  If any of the two is raised then any attempt to raise the other is rejected by the sdk and an error is passed to the error stream. 
                

#### Articulations
                
*  The API stubbed in 3.2.0 for applying an impulse to an entire articulation is now implemented.
                

#### GPU Physics
                
*  Much more specific error messages for CUDA compute capability related failures.
*  Added SM35 support for GK110 GPUs.
*  The CUDA contexts provided by the gpu dispatcher can now be shared across scenes.
*  Fixed a possible out of bounds array access in GPU particle collision code.
                


#### Scene Queries
                
*  Resolved poor GJK sweep convergence.
*  Batched queries accept sphere geometry for sweeps.
*  Optimized performance of raycasts.
*  An internal limit of 65536 objects in the scene-query subsytem has been lifted.
*  Fixed a bug where raycasts that sliced through two adjoining heightfields did not return correct results.
*  Fixed a crash bug in PxFindOverlapTriangleMeshUtil when doing a sphere overlap query against a heightfield.
                


#### Cloth
                
*  PxCloth::setMotionConstraints() now works with NULL parameter.
                

#### Character Controller
                
*  PhysX CCT code no longer sets PxShape::userData.  
*  Intersection of pairs of CCTs now uses the supplied filter data and the supplied callback prefilter.  The callback postfilter is not yet hooked up.
*  A bug has been fixed whereby the filterData was ignored in one of the scene queries initiated by the PhysX CCT code.
*  Introduced a more automatic mechanism for invelidating the character controller's scene cache.  As part of this, PxController::reportSceneChanged() was replaced with PxController::invalidateCache().
*  Added helper functions PxController::get/setFootPosition() to let user specify the bottom point of the character controller, rather than the center.
*  A new helper function, PxController::resize(), has been added to facilitate character controller resizing.
*  PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT is not the default behavior anymore.
*  The slope limit is only observed when walking on static convex meshes, static triangle meshes, static boxes and static heightfields.  The slope limit is not observed when walking on dynamic or kinematic rigid bodies or static capsules or static spheres.  This partially fixes a bug where the slope limit was inadvertently considered for shapes attached to dynamic rigid bodies and inadvertently ignored for boxes attached to static shapes.
                

#### Vehicles
                
*  The vehicle sdk now reports an error to the error stream and exits from PxVehicleUpates if PxInitVehicleSdk has not been called in advance.  This avoids a divide-by-zero error that can arise if the vehicle sdk has not been initialised correctly.
                

#### Visual Debugger
                
*  Releasing of cloth fabrics is reported to the VRD.
*  Ext::Joint::setActors() call now reported to PVD.
*  Fixed crash bug when removing an aggregate containing a PxArticulation, while PVD is running.
                
        

## Supported Platforms

        
Unchanged from  from 3.2.
        

## Known Issues And Limitations

        
Unchanged from  from 3.2.
        


        
        
        


# v3.2

December 2011

## What's New In NVIDIA PhysX 3.2

        
#### General
                
*  Three new sample applications:  SampleCharacterCloth (character with cloth cape and cloth flags), SampleBridges (character controller walking on dynamic bridges and moving platforms), SampleCustomGravity (character controller with arbitrary up vector).
*  On Windows, the PxFoundation instance is now a process wide singleton and part of the new PhysX3Common.dll library
*  PxCreatePhysics() does not create a PxFoundation instance any longer. The PxFoundation instance has to be created in advance through PxCreateFoundation().
*  Calls to PxCreatePhysics() are not valid anymore if a PxPhysics instance already exists.
*  If profiling information should be sent to the PhysX Visual Debugger, a PxProfileZoneManager instance has to be provided when creating the PxPhysics instance.
*  The version number constant PX_PUBLIC_FOUNDATION_VERSION has been replaced with PX_PHYSICS_VERSION. Both PxFoundation and PxPhysics use the same version number now.
*  The API now distinguishes between input and output stream types.
*  Added mechanism to reduce code size by not linking optional components.  See PxCreateBasePhysics() and the PxRegister*() functions.
*  Added getConcreteTypeName() to API classes to provide run time type information.
*  Added PxScene::getTimestamp() to retrieve the simulation counter.
*  PxGetFoundation has been moved to PxGetFoundation.h
*  Changed the functions PxPhysics::releaseUserReferences(), releaseCollection(), addCollection() and releaseCollected() to now take a reference rather than a pointer.
*  The signature of PxCreatePhysics has changed:  The Foundation SDK instance must be passed in explicitly.  One can also hook profiling information by passing a PxProfileZoneManager.
*  Platform conversion for serialized data has been moved from the ConvX command line tool to the PxBinaryConverter interface in the cooking library
*  contact data block allocation now provides statistics on usage and max usage
*  On all platforms except PS3, contact data blocks can be progressively allocated
*  PxExtensionVisualDebugger has been renamed to PxVisualDebuggerExt, PxExtensionsConnectionType renamed to PxVisualDebuggerConnectionFlag
*  Default implementations of memory and file streams added in PxDefaultStreams.h
*  Renamed PxPhysics::getMetaData() to ::PxGetSDKMetaData().
*  Scene::simulate() now takes a memory block which is used for allocation of temporary data during simulation
*  On Windows, CudaContextManagerDesc support appGUID now. It only works on release build. If your application employs PhysX modules that use CUDA you need to use a GUID so that patches for new architectures can be released for your game.You can obtain a GUID for your application from Nvidia.
                

#### Rigid Bodies
                
*  Introduced a new contact generation mode, see eENABLE_PCM.  Note that this is an experimental feature that still shows simulation artifacts in some scenarios.
*  Introduced two new friction simulation modes, see eENABLE_ONE_DIRECTIONAL_FRICTION and eENABLE_TWO_DIRECTIONAL_FRICTION.
*  Introduced a new scene query flag PxSceneQueryFlag::eINITIAL_OVERLAP_KEEP to control how initial overhaps are treated in scene queries.
*  Per-triangle materials have been implemented.
*  Changes to material properties are automatically reflected in contact resolution.
*  New helper methods to compute mass properties for a dynamic rigid body taking per shape density/mass values into account (see documentation on PxRigidBodyExt for details).
*  A new set of methods for overlap, sweep and raycast tests based on PxGeometry objects has been introduced. See documentation on PxMeshQuery and PxGeometryQuery for details).
*  The contact report API has changed (for details see the documentation on PxSimulationEventCallback::onContact()). Among the changes are:
                        
  * Reports only get sent for shape pairs which request them. Previously, reports were sent for an actor pair even if the requested shape pair event was not triggered (for example because other shapes of the same actors started colliding etc.)
  * The following PxPairFlags have been removed eNOTIFY_CONTACT_FORCES, eNOTIFY_CONTACT_FORCE_PER_POINT, eNOTIFY_CONTACT_FEATURE_INDICES_PER_POINT. Forces and feature indices are now always provided if applicable.
  * It is much easier now to skip shape pairs or contact point information when traversing the contact report data.
  * The memory footprint of contact reports has been reduced.
                        
                    
*  The members featureIndex0/1 of PxContactPoint have been renamed to internalFaceIndex0/1 for consistency.
*  For trigger reports, the eNOTIFY_TOUCH_PERSISTS event has been deprecated and will be removed in the next release. For performance and flexibility reasons it is recommended to use eNOTIFY_TOUCH_FOUND and eNOTIFY_TOUCH_LOST only and manage the persistent state separately.
*  Added PxConstraintVisualizer interface and code to visualize joint frames and limits.
*  Improved PxBatchQuery API.
*  PxPhysics::getProfileZoneManager() now returns a pointer rather than a reference.
*  PxRigidDynamic::moveKinematic() has been renamed to setKinematicTarget() to underline its precise semantics.
*  Added new function PxShape::getGeometry and class PxGeometryHolder to improve Geometry APIs.
*  PxCreatePlane now takes a PxPlane equation as a parameter. Note that the interpretation of the distance value is negated relative to 3.1
*  Added new actor creation helpers PxCloneStatic, PxCloneDynamic, PxScaleActor.
*  Added new functions PxTransformFromSegment, PxTransformFromPlaneEquation to simplify creation of planes and capsules.
*  added PxJoint::getConstraint() to access the underlying constraint object, from which the constraint force can be read
*  Some methods of PxAggregate have been renamed for consistency or replaced for extended functionality.
                        
  * getMaxSize() is now called getMaxNbActors().
  * getCurrentSize() is now called getNbActors().
  * getActor() has been replaced by getActors() which copies the actor pointers to a user buffer.
                        
                    
*  Added support for kinematic triangle meshes, planes and heighfields.
                

#### Scene queries
                
*  Dynamic AABBTree has been set as the default dynamic pruning structure.
                

#### Particles
                
*  Removed descriptors from particle API: The properties maxParticles and PxParticleBaseFlag::ePER_PARTICLE_REST_OFFSET need to be specified when calling PxPhysics::createParticleSystem() and createParticleFluid(). All other properties can be adjusted after creation through set methods.
                

#### Cloth
                
*  Added convex collision shapes, see PxCloth::addCollisionConvex()
*  Added friction support, see PxCloth::setFrictionCoefficient()
*  Added angle based bending constraints, see PxClothPhaseSolverConfig::SolverType::eBENDING
*  Added separation constraints, a spherical volume that particles should stay outside of, see PxCloth::setSeparationConstraints()
*  Added drag, see PxCloth::setDragCoefficient()
*  Added inertia scaling, controls how much movement due to PxCloth::setTargetPose() will affect the cloth
*  Added support for setting particle previous positions, see PxCloth::setParticles()
*  Added controls for scaling particle mass during collision, this can help reduce edge stretching around joints on characters, see PxCloth::setCollisionMassScale()
*  Particle data is now copied asynchronously from the GPU after simulate (rather than on demand)
*  Improved fabric layout, you can now share fabric data across multiple phases to reduce memory usage, see PxClothFabric
*  Fixed bug in collision when capsules are tapered at a slope > 1
                

#### Vehicles
                
*  Added PxVehicleDriveTank, a vehicle class that enables tank behaviors.
*  Support for vehicles with more than 4 wheels, see PxVehicleDrive4W, PxVehicleDriveTank.
*  Significant refactor of vehicle api to allow further types of vehicle to be added.
*  Americal English spelling used in vehicle api.
*  PxVehicle4W replaced with PxVehicleDrive4W, PxVehicle4WSimulationData replaced with PxVehicleDriveSimData4W.
*  Removal of scene query helper functions and structs: PxVehicle4WSceneQueryData, PxVehicle4WSetUpSceneQuery, PxWheelRaycastPreFilter, PxSetupDrivableShapeQueryFilterData, PxSetupNonDrivableShapeQueryFilterData, PxSetupVehicleShapeQueryFilterData. See SampleVehicle_SceneQuery.h for their implementation in SampleVehicle.
*  PxVehicle4WSimpleSetup and PxCreateVehicle4WSimulationData have been removed and replaced with default values in vehicle components, see PxVehicleComponents.h.
*  PxVehicle4WTelemetryData has been replaced with PxVehicleTelemetryData, a class that supports vehicles with any number of wheels, see PxVehicleTelemetryData
*  PxVehicleDrivableSurfaceType no longer stored in PxMaterial::userData.  A hash table of PxMaterial pointers is instead used to associate each PxMaterial with a PxVehicleDrivableSurfaceType, see PxVehicleDrivableSurfaceToTireFrictionPairs.
*  PxVehicleTyreData::mLongitudinalStiffness has been replaced with PxVehicleTireData::mLongitudinalStiffnessPerUnitGravity, see PxVehicleTireData.
*  Tire forces now computed from a shader to allow user-specified tire force functions, see PxVehicleTireForceCalculator.
*  Added helper functions to quickly configure 3-wheeled cars, see PxVehicle4WEnable3WTadpoleMode, PxVehicle4WEnable3WDeltaMode.
                

#### Serialization
                
*  Changed the functions PxPhysics::releaseUserReferences(), releaseCollection(), addCollection() and releaseCollected() to now take a reference rather than a pointer.
*  Platform conversion for serialized data has been moved from the ConvX command line tool to the PxBinaryConverter interface in the cooking library.
*  Changed some functions in RepXUtility.h and RepX.h to take a reference rather than a pointer.
                


#### What we removed:
                
*  Deformables have been removed. Use the optimized solution for clothing simulation instead (see documentation on PxCloth for details).
*  PxSweepCache was replaced with PxVolumeCache.
*  PVD is no longer enabled in the release build.
*  Removed anisotropic friction.
*  Removed the CCD mode eSWEPT_CONTACT_PAIRS.
*  PxActorDesc has been removed.
*  The ConvX tool has been removed.
*  Removed empty default implementations of functions in PxSimulationEventCallback for consistency and because it can create bugs in user code if function prototypes change between releases.  Users must now supply (eventually blank) implementations for all functions.
*  Octree and quadtree pruning structures have been removed.
                

#### Fixed Bugs
                
*  PxScene::getActors() might not work properly when the startIndex parameter is used.
*  Improved the doc-comment of PxConvexMesh::getMassInformation().
*  RepX instantiation can lose all internal references when addOriginalIdsToObjectMap is false.
*  PxSetGroup crashed when used on a compound.
*  PhysXCommon.dll can be delay loaded.
*  ContactReportStream can now handle huge report numbers and size (resize-able flag) can be set in PxSceneDesc.h.
*  Fixed assert in sweep tests.
*  Concurrent read/write operations during a PxScene::fetchResults() call were not detected properly and no warning message got sent in checked builds. Forbidden write operations during callbacks triggered by PxScene::fetchResults() (contact/trigger reports etc.) were not covered either.
*  Fixed crash bug that occurred during collision detection when more than 16K of contact data was generated.  Contacts that generate more than 16K of contact data are now fully supported.
*  Fixed crash bug when PhysX Visual Debugger is connected and an object gets modified and then released while the simulation is running.
                

        

## Supported Platforms

        
#### Runtime
                
*  Apple iOS
*  Apple Mac OS X
*  Google Android (version 2.2 or later for SDK, 2.3 or later required for samples)
*  Linux (tested on Ubuntu)
*  Microsoft Windows XP or later (NVIDIA Driver ver 295.73 or later is required for GPU acceleration)
*  Microsoft XBox 360
*  Sony Playstation 3
*  Sony Playstation Vita
                
#### Development
                
*  Microsoft Windows XP or later
*  Microsoft Visual Studio 2008, 2010
*  Xcode 4.2
                
        

## Known Issues And Limitations

        

#### Binary Serialization
                
*  Meta data generation for PxParticleFluid and PxParticleSystem serializables currently fails.
*  For collections that contain jointed rigid bodies all corresponding joints need to be added as well, otherwise deserialization will fail.
                
#### Rigid Body Simulation
                
*  Capsules and spheres can struggle to come to rest on even perfectly flat surfaces. To ensure these objects come to rest, it is necessary to increase angular damping on rigid bodies made of these shapes. In addition, flagging the capsule/sphere's material with physx::PxMaterialFlag::eDISABLE_STRONG_FRICTION can help bring these shapes to rest.
                
#### Character Cloth Sample
                
*  An NVIDIA GPU with Compute Capability 2.0 or higher is required for GPU accelerated simulation in the SampleCharacterCloth sample, if no such device is present then simulation will be performed on the CPU.
*  Note that this is not a general SDK requirement, the clothing SDK supports cards with Compute Capability < 2.0 but with limitations on mesh size.
                
#### Character Controller
                
*  Releasing shapes of actors that are in touch with a character controller may lead to crashes. Releasing whole actors doesn't lead to the same problems. PxController::invalidateCache() can be used to work around these issues.
                

        

        
Please also see the previous lists  from 3.1.1 and earlier.
        


        
        
        


# v3.1.2

December 2011

## What's New In NVIDIA PhysX 3.1.2

        
#### General
                
*  Fixed wrong write/read clash checks.
*  Removed some compiler warnings from public header files.
*  Fixed PxScene::getActors() returning wrong actors when a start index is specified.
                


#### Rigid Bodies
                
*  Fixed broken joint projection in connection with kinematics.
*  Fixed inaccurate normals returned from height field scene queries.
*  Fixed a crash when the geometry of a shape changes and then the actor gets removed from the scene while the simulation is running.
*  Fixed a crash when re-adding scene-query shape actors to scene.
                


#### Particles
                
*  Fixed crash bug in particle simulation code on GPU.
                


#### Cloth
                
*  Fixed a crash when GPU fabrics are shared between cloths.
*  Fixed a hang in cloth fiber cooker when handed non-manifold geometry.
                

#### Samples
                
*  Fixed SampleVehicles doing an invalid write.
*  Fixed SampleVehicle jitter in profile build.
                

        

## Supported Platforms (available in separate packages)

        
Unchanged from  from 3.1.1.
        

## Known Issues And Limitations

        
Unchanged from  from 3.1.
        


        
        
        


# v3.1.1

November 2011

## What's New In NVIDIA PhysX 3.1.1

        
#### General
                
*  Ported samples to Linux.
*  Fixed crash bug in ConvX.
*  Fixed crash bug in the allocator code of PXC_NP_MEM_BLOCK_EXTENSIBLE.
*  Fixed crash bug when connected to PVD on various platforms.
*  Fixed bogus asserts due to overly strict validation of quaternions.
*  Fixed one frame lag in PVD scene statistics.
*  Fixed a number of OSX PVD sockets issues.
*  Fixed SampleSubmarine code that violated concurrent read/writes restriction.
*  Added warnings about read/write hazards to the checked build.
*  Fixed RepX not reading joint properties.
*  Fixed support for concurrent scene queries.
*  Fixed PhysX GPU Visual Indicator support.
*  Made it more clear in documentation that simulate(0) is not allowed.
                

#### Rigid Bodies
                
*  eNOTIFY_TOUCH_LOST trigger events do now get reported if one of the objects in contact gets deleted (see documentation of PxTriggerPair for details).
*  Dynamic rigid bodies with trigger shapes only do not wake up other touching bodies anymore.
*  Added lost touch events for trigger reports when objects get deleted.
*  Fixed dynamic triggers waking up actors they are triggered by.
*  Removed an inapropriate assert from articulation code.
*  Fixed problem with the angular momentum conservation of articulations.
*  Fixed articulation sleep problems.
*  Fixed a linear velocity related bug in CCD.
*  Fixed crash bug CCD.
*  Optimized performance of joint information being sent to PVD.
                

        

## Supported Platforms (available in separate packages)

        
#### Runtime
                
*  Microsoft Windows XP or later
*  Microsoft XBox 360
*  Sony Playstation 3
*  Android 2.2 or later for SDK, 2.3 or later required for samples
*  Linux (tested on Ubuntu)
*  Mac OS X
                
#### Development
                
*  Microsoft Windows XP or later
*  Microsoft Visual Studio 2008, 2010
*  Xcode 3
                
        

## Known Issues And Limitations

        
Unchanged from  from 3.1.
        


        
        
        


# v3.1

September 2011

## What's New In NVIDIA PhysX 3.1

        
#### General
                
*  VC10 support has been introduced.
*  VC8 support has been discontinued.
*  Namespaces cleaned up.
*  Extensions, Character Controller and Vehicle source code made available in binary distribution.
*  Added x86,x64 suffix to PxTaskCUDA.dll
*  Removed boolean return value from PxScene::addActor(...), and similar API calls.
*  Added MacOS, Android and Linux to the list of supported platforms.  See Supported Platforms below for details.
*  Upgraded GPU tech to CUDA 4.
*  Cleaned up a large number of warnings at C++ warning level 4, and set SDK to compile with warnings as errors.
*  Removed individual sample executables in favor of one Samples executable from PC and console builds.
*  Fixed alpha blending in samples.
*  Simplified some code in samples.
*  Improved ambient lighting in samples.
*  Made samples work with older graphics cards.
*  Improved and added more content the user's guide.
*  No longer passing NULL pointers to user allocator to deallocate.
*  Various improvements to Foundation and classes shared with APEX.
                
#### Rigid Bodies
                
*  Rigid Body: High performance alternative convex narrow phase code available to source licensees. See PERSISTENT_CONTACT_MANIFOLD in the code.
*  Significant advancements in the continuous collision detection algorithm.
*  Optimizations and robustness improvements for articulations.
*  Added some helper code to the API.
*  Added sleep code for articulations.
*  Added support for vehicles with more than one chassis shape.
*  Solver iteration count for articulations.
*  Articulation limit padding configurable.
*  The reference count of meshes does now take the application's reference into acount as well and thus has increased by 1 (it used to count the number of objects referencing the mesh only). Note that a mesh does only get destroyed and removed from the list of meshes once the reference count reaches 0.
*  Fixed autowake parameter sometimes being ignored.
*  Constraint solver optimizations.
*  Improved behavior of character controller on steep slopes.
*  Binary serialization now saves names.
*  Removed some descriptors from API.
*  Removed the angular velocity term in the joint positional drive error formula.
*  Fixed bug in capsule sweep versus mesh.
*  Fixed a crash bug in the tire model.
*  Fixed crashing of single link articulations.
*  Fixed bug related to removing elements of an aggregate.
*  Fixed swapped wheel graphs in sample vehicle.
*  Fixed some slow moving bodies falling asleep in midair.
*  Fixed missing collisions after a call to resetFiltering.
*  Fixed broken autowake option in setAngularVelocity.
*  Fixed D6 joint linear limits being uninitialized.
*  A large number of misc. bug fixes and optimizations.
*  Improved documentation and error messages associated with running out of narrow phase buffer blocks.
*  Added articulation documentation.
*  Expanded manual sections on joints.
*  Improved reference doc for PxSceneQueryHitType.
*  Added reference doc for PxSerializable.
                

#### Particles
                
*  Particle index allocation removed from SDK. Added index allocation pool to extensions.
*  Replaced GPU specific side band API PxPhysicsGpu and PxPhysics::getPhysicsGpu() with PxParticleGpu.
*  Memory optimizations on all platforms and options to reduce memory usage according to use case with new per particle system flags:
                        
  * PxParticleBaseFlag::eCOLLISION_WITH_DYNAMIC_ACTORS
  * PxParticleBaseFlag::ePER_PARTICLE_COLLISION_CACHE_HINT
                        
                    
*  Fixed rare crash appearing with multi-threaded non-GPU particle systems and rigid bodies.
*  Fixed particles leaking through triangle mesh geometry on GPU.
*  Fixed fast particles tunneling through scene geometry in some cases.
*  Fixed erroneous collision of particles with teleporting rigid shapes (setGlobalPose).
*  Fixed particle sample behavior with some older GPU models.
*  Fixed a GPU particle crash bug.
                


#### Cloth
                
*  A new solution for simulating cloth and clothing.
                


#### Deformables
                
*  Deformables are deprecated and will be removed in the next release. There is a new optimized solution for clothing simulation (see documentation on PxCloth for details).
                
        

## Supported Platforms (available in separate packages)

        
#### Runtime
                
*  Microsoft Windows XP or later
*  Microsoft XBox 360
*  Sony Playstation 3
*  Android 2.2 or later for SDK, 2.3 or later required for samples
*  Linux (SDK tested on Ubuntu, samples not yet ported) 
*  Mac OS X
                
#### Development
                
*  Microsoft Windows XP or later
*  Microsoft Visual Studio 2008, 2010
*  Xcode 3
                
        

## Known Issues And Limitations

        
#### General
                
*  Under VC10, you may get warnings due to conflicting build configuration flags.  Workaround: Clear the "Inherit from parent or project defaults" flag for all projects in Project->Properties->C/C++->Command Line.  We plan to fix this for the 3.1 general availability release.
                
#### Scene Query
                
*  Querying the scene (e.g. using raycastSingle()) from multiple threads simultaneously is not safe.
                
#### Cloth
                
*  Even simple parameters of a PxCloth can not be set or accessed while the simulation is running.
                
#### RepX
                
*  RepX fails to load elements of aggregate joint parameters (PxD6JointDrive etc.)
                
        

        
Please also see the previous lists  from 3.0.
        


        
        
        


# v3.0

February 14<sup>th</sup>2011

## What's New In NVIDIA PhysX 3.0

        
#### General

This third major version of the SDK is a significant rewrite of the entire technology.  We did away with a large amount of legacy clutter and replaced them with a wealth of new features and improvements.
Because even the API changes are so significant, it is easier to see it as a whole new product rather than a long list of changes.

#### What we removed:
                
*  The dedicated NVIDIA PhysX PPU hardware is not supported any more.
*  Scene compartments are not supported anymore. All created physical objects are now part of one and the same compartment.
*  Force fields are not part of the NVIDIA PhysX SDK anymore.
*  Splitting a simulation timestep into multiple substeps is not a functionality of the NVIDIA PhysX SDK any longer and has to be implemented above the SDK.
                

#### Key new features:
                
*  Articulations: A way to create very stiff joint assemblies.
*  Serialization: Save objects in a binary format and load them back quickly!
*  Broad Phase Clustering: Put objects that belong together into a single broadphase volume.
*  Driverless Model: No need to worry about system software on PC anymore.
*  Dynamic Character Controller: A character controller that can robustly walk on dynamic objects.
*  Vehicle Library: A toolkit to make vehicles, including an all new tire model.
*  Non-Simulation Objects: A staging are outside of the simulation from where you can add things into the simulation at high speed.
*  Simulation Task Manager: Take control of the management of simulation tasks.
*  Stable Depenetration: Large penetrations can be gracefully recovered from.
*  Double Buffering: You can read from and write to the simulated objects while the simulation is running on another thread.
*  Mesh Scaling: Create different nonuniformly scaled instances of your meshes and convexes without duplicating the memory.
*  Distance Based Collision Detection: Have the simulation create contacts before objects touch, and do away with unsightly overlaps.
*  Fast Continuous Collision Detection: Have small and high speed objects collide correctly without sacrificing performance.
*  Significantly increased performance and memory footprint, especially on consoles.
*  Unified solver for deformables and rigid bodies for great interaction.
*  Triangle collision detection with deformables.
*  Support for our new Physics Visual Debugger, including integrated profiling.
                


#### Math classes
                
*  Matrix based transforms have been replaced by quaternions.
*  All angles are now expressed in radians. IN PARTICULAR the PxQuat constructor from axis and angle as well as the getAngleAxis and fromAngleAxis methods now use radians rather than degrees.
                    
                

#### Rigid Bodies
                
*  Capsules are now defined to extend along the x rather than the y axis.
*  Triangle meshes do not support heightfield functionality anymore. Use the dedicated PxHeightField class instead.
*  Dynamic triangle mesh actors are not supported any longer. However, you can decompose your mesh into convex parts and create a dynamic actor consisting of these convex parts.
*  The deprecated heightfield property NxHeightFieldDesc::verticalExtent is not supported any longer. Please use the PxHeightFieldDesc::thickness parameter instead.
*  NxSpringAndDamperEffector is not supported anymore. Use PxDistanceJoint instead.
*  Joints are now part of the PhysX extensions library (PhysXExtensions).
*  Wheel shapes have been replaced by the more flexible entity PxWheel. A default wheel implementation, encapsulating most of the old wheel functionality, can be found in the PhysX extensions library (see PxDefaultWheel).
*  The NxUtilLib library has been removed. Sweep/overlap/raycast tests and other helper methods can be found in the new GeomUtils library.
*  Materials can no longer be accessed through indices. Per triangle material meshes need a material table which can be specified per shape (see PxShape::setMaterials() for details).
*  The default material is not available anymore.
                

#### Particle Systems, Particle Fluids
                
* The NxFluid class has been replaced with two classes for separation of functionality and ease of use.
                        
  * PxParticleSystem: Particles colliding against the scene.
  * PxParticleFluid: Particles simulating a fluid (sph).
                        
                    
* Simplified parameterization for particle systems.
                        
  * Using absolute distances instead of relative multipliers to rest spacing
  * Simplified sph parameters
  * Unified collision parameters with deformable and rigid body features
                        
                    
* Creating and releasing particles is now fully controlled by the application.
                        
  * Particle lifetime management isn't provided through the SDK anymore.
  * Emitters have been removed from the SDK.
  * Drain shapes don't cause particles to be deleted directly, but to be flagged instead.
  * Initial particle creation from the particle system descriptor isn't supported anymore.
                        
                    
*  Particle data buffer handling has been moved to the SDK.
*  Per particle collision rest offset.
*  GPU accelerated particle systems.
                        
  * Application controllable mesh mirroring to device memory.
  * Runtime switching between software and GPU accelerated implementation.
                        
                    
                
        

## Supported Platforms (available in separate packages)

        
#### Runtime
                
*  Microsoft Windows XP or and later
*  Microsoft XBox360
*  Sony Playstation 3
                
#### Development
                
*  Microsoft Windows XP or and later
*  Microsoft Visual Studio 2008
                
        

## Known Issues And Limitations

        
#### Rigid Bodies
                
*  Adding or removing a PxAggregate object to the scene is not possible while the simulation is running.
                
#### Particle Systems
                
* Releasing the Physics SDK may result in a crash when using GPU accelerated particle systems. 
* This can be avoided by doing the following before releasing the Physics SDK:
                        
  * Releasing the PxScene objects that contain the GPU accelerated particle systems.
  * Releasing application mirrored meshes by calling PxPhysicsGpu::releaseTriangleMeshMirror(...), PxPhysicsGpu::releaseHeightFieldMirror(...) or PxPhysicsGpu::releaseConvexMeshMirror(...).
                        

