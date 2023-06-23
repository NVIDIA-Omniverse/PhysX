# Changelog

## [5.0.1] - 22-June-2023

### Bugfixes
- Use proper constructors for NvTransform and NvVec3 to avoid using garbage data


## [5.0.0] - 23-Jan-2023

### Changes
- Removed all PhysX dependencies from code outside of the ExtPx extension
- Replaced Px types with NvShared types
- NvFoundation headers in include/shared/NvFoundation
  - Includes NvPreprocessor.h and NvcTypes.h (formerly in include/lowlevel)
  - Include basic Nv types, such as NvVec3 (used by the Tk library)
- Consolidated header structure
  - include/lowlevel/NvBlastPreprocessor.h is gone
  - Previously-defined NVBLAST_API has been renamed NV_C_API and is now defined in NvPreprocessor.h


## [4.0.2] - 31-Aug-2022

### Bugfixes
- Stress solver Linux crash fix.  Explicitly allocating aligned data buffers for use with simd data.


## [4.0.1] - 10-Aug-2022

### Bugfixes
- Stress solver fixes:
  - More robust conversion from angular pressures to linear pressures.
  - Better error tolerance checking.
  - Force sign consistency.


## [4.0.0] - 31-May-2022

### New Features
- Fully integrated stress-damage system.  A stress solver is used to determine how bond forces react to impacts and other externally-supplied accelerations.
Stress limits (elastic and fatal) determine how bond health (area) deteriorates with bond force.  When bonds break and new actors are generated,
excess forces are applied to the previously joined bodies.  Using a new stress solver with better convergence properties.
- Documentation publishing.


## [3.1.3] - 28-Feb-2022

### Changes
- Update triangulation ear clipping algorithm to avoid outputting triangle slivers.


## [3.1.2] - 24-Feb-2022

### Changes
- Change ExtStressSolver::create() (and downstream functions/classes) to take const NvBlastFamily.
- Change the order colors are compressed in PxVec4ToU32Color()

### Bug Fixes
- Fixed triangulation ear clipping bug that could cause input verts to not be used in output triangulation, leading to T junctions.


## [3.1.1] - 2022-01-12

### Changes
- Exposing NvcVec2 and NvcVec3 operators in new file include/globals/NvCMath.h.


## [3.1.0] - 2022-01-10

### Changes
- Exposing boolean tool API, along with spatial accelerators used with the tool.
  - include/extensions/authoring/NvBlastExtAuthoringBooleanTool.h contains virtual API class Nv::Blast::BooleanTool.
  - include/extensions/authoringCommon/NvBlastExtAuthoringAccelerator.h contains virtual API classes Nv::Blast::SpatialAccelerator and SpatialGrid.
  - include/extensions/authoring/NvBlastExtAuthoring.h has global functions:
    - NvBlastExtAuthoringCreateBooleanTool()
    - NvBlastExtAuthoringCreateSpatialGrid(...)
    - NvBlastExtAuthoringCreateGridAccelerator(...)
    - NvBlastExtAuthoringCreateSweepingAccelerator(...)
    - NvBlastExtAuthoringCreateBBoxBasedAccelerator(...)


## [3.0.0] - 2021-10-13

### Changes
- Rearranged folder layout.  Public include files are now under a top-level "include" folder.


## [2.1.7] - 2021-07-18

### Bug Fixes
- Fixed edge case with convex hull overlap test in processWithMidplanes(), so that (0,0,0) normals aren't generated.
- Prevented crash when no viable chunk is found by findClosestNode(), leading to lookup by invalid chunk index in damage shaders.


## [2.1.6] - 2021-06-24

### Changes
- Prioritize convex hulls over triangles in processWithMidplanes()
- Store local convex hulls longer in createFullBondListAveraged() so they can be used in processWithMidplanes()
- Fix bug with buildDescFromInternalFracture() when there are multiple root chunks.


## [2.1.5] - 2021-05-10

### Changes
- Bond centroid and normal calculations improved when splitting a child chunk.  The normal will only be different from before with non-planar splitting surfaces.
- Mesh facet user data, which stores a splitting plane (or surface) identifier, will stay unique if the split mesh is fed back into a new instance of the fracture tool.  The new IDs generated will be larger than any ID input using FractureTool::setChunkMesh(...).


## [2.1.4] - 2021-04-08

### Bug Fixes
- OM-29933: Crash fracturing dynamic attachment


## [2.1.3] - 2021-04-05

### Bug Fixes
- Bond area calculation was producing values twice the correct value.
- Fixed exception in TkGroupImpl.


## [2.1.2] - 2021-03-15

### Bug Fixes
- MR #18: Fix asset joint serialization (BlastTk)
- MR #19: Fix index out of bounds (BlastTk)


## [2.1.1] - 2021-03-02

### Changes
- Added Cap'n Proto serialization path for Family to match Asset.
- Fix bug with BlastAsset::getSupportChunkHealthMax() returning the wrong value.
- Add get/set/size data interface to FixedBoolArray.
- Allocate asset memory based on how much space it needs, not serialized data size.
- Release asset memory if deserialization fails.
- Removed FamilyHeader::getActorBufferSize(), use FamilyHeader::getActorsArraySize() instead.


## [2.0.1] - 2021-03-01

### Changes
- Added .pdb files to windows package.
- Bumping version to update dependency chain with linux built from gcc 5.5.0 (for CentOS7 compatibility).


## [2.0.0] - 2021-02-19

### Changes
- Add optional chunkId params to FractureTool::setSourceMeshes() and FractureTool::setChunkMesh()
- Rename functions and variables to better indicate what the indices are used for instead of using generic "chunkIndex" for everything


## [1.4.7] - 2020-10-20

### Changes
- Don't include bonds that can't take damage (already broken or unbreakable) in results when applying damage

### Bug Fixes
- Make sure all fields (specifically userData) on NvBlastBondDesc are initialized when creating bonds


## [1.4.6] - 2020-10-08

### Changes
- Updated license file
- Updated copyright dates

### Bug Fixes
- Pull request #15 "Fix Blast bond generation"
- Pull request #16 "Fix invalid pointer access in authoring tools"


## [1.4.5] - 2020-09-30

### Bug Fixes
- Allocate on heap instead of stack in importerHullsInProximityApexFree() to prevent crash


## [1.4.4] - 2020-09-29

### Changes
- Support unbreakable bonds and chunks by setting their health above Nv::Blast::kUnbreakableLimit
- Consolidate code when node is removed


## [1.4.3] - 2020-09-26

### Changes
- Per-chunk internal scaling.  ChunkInfo contains the struct TransformST (scale & translation)

### Bug Fixes
- Fixes many fracturing instabilities with per-chunk scaling


## [1.4.2] - 2020-08-28

### Bug Fixes
- Fixed mesh generation bug when using FractureToolImpl::createChunkMesh


## [1.4.1] - 2020-06-26

### Changes
- Change API references to 'external' instead of 'world' bonds
- Deprecate 'world' versions, should be removed on next major version bump


## [1.2.0] - 2020-01-23

### Changes
- Removed BlastTool
- Removed ApexImporter tool
- Removed ExtImport extension (for Apex)

### New Features
- Reenabling runtime fracture

### Known Issues
- Damage shaders in extensions can miss bonds if the damage volume is too small.
- Authoring code does not use the user-defined allocator (NvBlastGlobals) exclusively.


## [1.1.5] - 2019-09-16

### Changes
- Extensions API refactored to eliminate use of Px types.
- Numerous API ### changes to meet new coding conventions.
- Packman package manager updated to v. 5.7.2, cleaned up dependency files.
- Chunks created from islands use padded bounds to determine connectivity.
- FractureTool::deleteAllChildrenOfChunk renamed FractureTool::deleteChunkSubhierarchy, added ability to delete chunks.
- NvBlastAsset::testForValidChunkOrder (used when creating an NvBlastAsset) is now more strict, requiring parent chunk descriptors to come before their children.  It is still less strict than the order created by NvBlastBuildAssetDescChunkReorderMap.

### New Features
- Authoring tools:
  - Ability to pass chunk connectivity info to uniteChunks function, enabling chunks split by island detection to  be united.
  - Option to remove original merged chunks in uniteChunks function.
  - The function uniteChunks allows the user to specify a chunk set to merge.  Chunks from that set, and all descendants, are considered for merging.
  - Ability to delete chunks (see note about FractureTool::deleteChunkSubhierarchy in ### Changes section, above).
  - Added FractureTool::setApproximateBonding function.  Signals the tool to create bonds by proximity instead of just using cut plane data.

### Bug Fixes
- Authoring tools:
  - Fixed chunk reordering bug in BlastTool.
  - Chunks which have been merged using the uniteChunks function may be merged again
  - Restored chunk volume calculation
- NvBlastBuildAssetDescChunkReorderMap failure cases fixed.

### Known Issues
- Damage shaders in extensions can miss bonds if the damage volume is too small.
- Authoring code does not use the user-defined allocator (NvBlastGlobals) exclusively.


## [1.1.4] - 2018-10-24

### Changes
- Unity plugin example updated to work with latest Blast SDK.

### New Features
- Authoring tools:
  - Island detection function islandDetectionAndRemoving has a new parameter, createAtNewDepth.
  - Bonds created between island-based chunks.
  - Added "agg" (aggregate) commandline switch to AuthoringTool.  This allows multiple convex hulls per chunk to be generated.
  - Damage pattern authoring interface.

### Bug Fixes
- Build working on later C++ versions (e.g. deprecated UINT32_MAX removed).
- Authoring tools:
  - Fixed .obj material loading when obj folder is same as working directory.
  - Degenerate face generation fix. 
  - Fixed memory leak in FractureTool.
- Proper memory releasing in samples.
- Single-actor serialization bugfix when actor has world bonds.
- Updated PhysX package for Win64 (vc14 and vc15) and Linux64 to 3.4.24990349, improving GRB behavior and fixing GRB crash/failure on Volta and Turing.
- Documented JSON collision export option introduced in previous version.

### Known Issues
- Damage shaders in extensions can miss bonds if the damage volume is too small.
- Authoring code does not use the user-defined allocator (NvBlastGlobals) exclusively.


## [1.1.3] - 2018-05-30

### Changes
- No longer testing Win32 project scripts.  Note generate_projects_vc14win32.bat has been renamed generate_projects_vc14win32_untested.bat.
- Using a PhysX Packman package that no longer includes APEX.
- Updated documentation:
  - Authoring documentation mentions restrictions for meshes to be fractured.
  - Added BlastTool reference to README.md.
  - Updated documentation paths in README.md.
- Using Packman5 for external packages.
- Authoring tools:
  - In NoiseConfiguration, surfaceResolution changed to samplingInterval.  The latter is reciprocal of resolution and defined for all 3 axes.
  - Improved cutout robustness.
- Exporter (used by both authoring tools and ApexImporter) has a JSON collision export option.

### New Features
- VC15 Win64 project scripts.  Run generate_projects_vc15win64.bat.
- Authoring tools:
  - Noisy cutout fracture.
  - Conic cutout option (tapers cut planes relative to central point).
  - Cutout option "useSmoothing."  Add generatad faces to the same smoothing group as original face without noise.
  - Periodic cutout boundary conditions.

### Bug Fixes
- Packman target platform dependencies no longer pulling windows packages into other platforms.
- Fixed bond generation for cutout fracture.

### Known Issues
- Damage shaders in extensions can miss bonds if the damage volume is too small.
- Authoring code does not use the user-defined allocator (NvBlastGlobals) exclusively.


## [1.1.2] - 2018-01-26

### Changes
- Improvements to uniteChunks for hierarchy optimization.
- NvBlastExtAuthoringFindAssetConnectingBonds optimized.
- APEX dependency has been removed (ExtImport used it).  Now ExtImport has a built-in NvParameterized read that can load an APEX Destructible asset.

### New Features
- FractureTool::setChunkMesh method.
- Distance threshold added to NvBlastExtAuthoringFindAssetConnectingBonds.
- NvBlastExtExporter: IMeshFileWriter::setInteriorIndex function, for control of interior material.
- Cutout and cut fracture methods: NvBlastExtAuthoringCreateCutoutSet and Nv::Blast::CutoutSet API, FractureTool::cut and FractureTool::cutout APIs.
- NvBlastExtAuthoring:
  - NvBlastExtAuthoringCreateMeshFromFacets function.
  - NvBlastExtUpdateGraphicsMesh function.
  - NvBlastExtAuthoringBuildCollisionMeshes function.
- UV fitting on interior materials using new FractureTool::fitUvToRect and FractureTool::fitAllUvToRect functions.
- Multi-material support in OBJ file format.

### Bug Fixes
- Fixed bug causing normals on every other depth level to be flipped when exporting Blast meshes.
- Fixed bug where faces are missed after hierarchy optimization on a sliced mesh.
- Fixed subtree chunk count generated in Nv::Blast::Asset::Create (led to a crash in authoring tools, fracturing a pre-fractured mesh).
- Fixed a crash when loading an obj with bad material indices.
- Fixed Actor::split so that visibility lists are correctly updated even when the number of split actors exceeds newActorsMaxCount.

### Known Issues
- Damage shaders in extensions can miss bonds if the damage volume is too small.
- Authoring code does not use the user-defined allocator (NvBlastGlobals) exclusively.


## [1.1.1] - 2017-10-10

### Changes
- NvBlastProgramParams moved to NvBlastExtDamageShaders
- Materials removed from NvBlastTk

### New Features
- Damage shader acceleration structure
- Extended support structures via new asset merge functions in NvBlastExtAssetUtils
- Ability to scale asset components when merging assets with NvBlastExtAssetUtilsMergeAssets
- NvBlastExtAuthoring
  - Option to fit multiple convex hulls to a chunk (uses VHACD)
  - deleteAllChildrenOfChunk and uniteChunks APIs
- Triangle damage shader for swept segments
- Impact damage spread shaders

### Bug Fixes
- Linux build fixes
- NvBlastExtAuthoring
  - Fracturing tools chunk index fix
  - VoronoiSitesGeneratorImpl::generateInSphere fix
  - More consistent use of NVBLAST_ALLOC and NVBLAST_FREE
  - Boolean tool bug fix

### Known Issues
- Damage shaders in extensions can miss bonds if the damage volume is too small.
- Authoring code does not use the user-defined allocator (NvBlastGlobals) exclusively.


## [1.1.0] - 2017-08-28

### Changes
- VC12 is no longer supported.
- New license header, consistent with PhysX license header.
- New serialization extension.  NvBlastExtSerialization is now a modular serialization manager.  It loads serializers sets for low-level, Tk, and ExtPx.  Each serializer handles a particular file format and object type.  Currently the universally available format for all object types is Cap'n Proto binary.  The file format is universal, as it uses a header to inform the serialization manager which serializer is needed to deserialize the contained data.  All authoring and import tools write using this format to files with a ".blast" filename extension.
- Corresponding to the new serialization, the old formats have been deprecated.  In particular, the DataConverter tool has been removed.  Instead see LegacyConverter in the ### New Features section.
- TkSerializable virtual base class has been removed.  TkAsset and TkFamily are now derived directly from TkIdentifiable.  Serialization functions have been removed, replaced by the new serialization extension.
- ExtPxAsset serialization functions have been removed, replaced by the new serialization extension.
- World bonds.  A bond descriptor can now take the invalid index for one of its chunkIndices.  This will cause an additional support graph node to be created within an asset being created with this descriptor.  This node will not correspond to any chunk (it maps to the invalid index in the graph's chunkIndices array).  Actors that contain this new "world node" may be kept static by the user, emulating world attachment.  This is easily tested using the new low-level function NvBlastActorIsBoundToWorld.
- With the addition of world bonds (see above), the NvBlastExtImport extension no longer creates an extra "earth chunk" to bind chunks to the world.  Instead, it creates world bonds.
- ExtPxAsset now contains an NvBlastActorDesc, which is used as the default actor descriptor when creating an ExtPxFamily from the asset.
- TkFramework no longer has its own allocator and message handler.  Instead, this is part of a new NvBlastGlobals API.  This way, extensions and TkFramework may share the same allocator.
- SampleAssetViewer
  - Physics simulation now runs concurrently with graphics and some of the sample/blast logic.
  - New Damage tool added: line segment damage
  - Damage tool radius can be set individually for each tool (radial, cutter, line segment, hierarchical).
  - Cubes now removed when a scene is reloaded.
  - Cube throw velocity can be "charged" by holding down the 'F' key.
- New damage system built around "health," see API ### changes in NvBlastExtShaders and ### changes in NvBlastExtImpactDamageManager.
- NvBlastExtShearGraphShader uses a chunk-based method to find the closest graph node, improving performance.
- TkGroup no longer uses physx::PxTaskManager interface for task management.  Instead, a TkGroupWorker interface has been added.  The NvBlastExtPhysX extension uses the physx::PxTaskManager to implement this interface.
- Better error handling in AuthoringTool (stderr and user error handler).
- More consistent commandline switches in AuthoringTool and ApexImporter (--ll, --tk, --px flags).
- Various small clean-ups.

### New Features
- NvBlastExtAssetUtils extension
  - Merge multiple assets into one.
  - Add "world bonds" to an asset (see "World bonds" in the ### Changes section).
  - Transform an NvBlastAsset's geometric data in-place.
- NvBlastExtAuthoring
  - Open edge detection.
  - Rotation of voronoi cells used for fracturing.
- "Globals" code (under sdk/globals).  Includes a global allocator, message handler, and profiler API used by TkFramework and extensions.
- NvBlastExtStress extension, a PhysX-independent API for performing stress calculations with low-level Blast actors.
- NvBlastActorIsSplitRequired() function for low-level actors.  If this function returns false, NvBlastActorSplit() may be skipped as it will have no effect.
- NvBlastExtShaders
  - New "Segment Radial Damage" shader.  Damages everything within a given distance of a line segment.
- New NvBlastExtExporter extension, used commonly by import and authoring tools.  Allows collision data to be stored in one of three ways:
  - JSON format.
  - FBX mesh format (seprate file).
  - FBX mesh format in a second "collision" layer, alongside the graphics mesh nodes corresponding to Blast chunks.
- LegacyConverter tool has been added, which converts .llasset, .tkasset, .bpxa, .pllasset, .ptkasset, and .pbpxa asset files to the new .blast format using the universal serialization scheme in the new NvBlastExtSerialization extension.
- NvBlastExtAuthoring
  - Mesh cleaner, tries to remove self intersections and open edges in the interior of a mesh.
  - Ability to set interior material to existing (external) material, or a new material id.
  - Material ID remapping API.

### Bug Fixes
- NvBlastExtAuthoring
  - Slicing normals fix.
- Various instances of &array[0] to get the data buffer from a std::vector now use data() member function.  This had led to some crashes with empty vectors.
- SampleAssetViewer
  - Fixed dragging kinematic actor.
  - Now loads the commandline-defined asset also when sample resources were not downloaded yet.
- Serialization documented.
- Fixed smoothing groups in FBX exporter code.
- Impulse passing from parent to child chunks fixed.
- Reading unskinned fbx meshes correctly.
- Collision hull generation from fbx meshes fixed.
- Win32/64 PerfTest crash fix.

### Known Issues
- Damage shaders in extensions can miss bonds if the damage volume is too small.
- Authoring extension does not perform convex decomposition to fit chunks with multiple collision hulls.
- Authoring code does not use the user-defined allocator (NvBlastGlobals) exclusively.


## [1.0.0] - 2017-02-24

### Changes
- tclap, imgui, moved to Packman package
- Models and textures for the sample application have been moved to Packman
- Packman packages with platform-specific sections have been split into platform-specific packages
- Improvements to fracturing tools
- TkJoint events no longer contain actor data
- API cleanup:
  - NvBlastActorCreate -> NvBlastFamilyCreateFirstActor
  - NvBlastActorRelease -> NvBlastActorDeactivate
  - NvBlastActorDeserialize -> NvBlastFamilyDeserializeActor
  - Functions that operate on an object start with NvBlast[ObjectName]
  - Functions that create an object purely from a desc start with NvBlastCreate
  - Functions that get scratch start with NvBlast[Object]GetScratchFor[functionname], etc.
  - Object functions take the object as the first input parameter (non-optional output parameters always come first)
  - Removal of NvBlastCommon.h
- More consistent parameter checking in low-level API
- NvBlastAlloc and NvBlastFree functions have been removed.  Blast low-level no longer does (de)allocation.  All memory is passed in and managed by the user
- All Blast low-level functions take a log (NvBlastLog) function pointer (which may still be NULL)
- Authoring tool now handles FBX mesh format
- Constructor for TkAssetDesc sets sane defaults
- Sample uses skinning for the 38k tower, for perf improvement
- Further optimzations to sample, including using 4 instead of 2 CPU cores and capping the actor count at 40k
- Linux build (SDK and tests)
- Renamed TkJointUpdateEvent::eventSubtype -> TkJointUpdateEvent::subtype
- "LowLevel" extension renamed "ConvertLL"
- Renamed TkEventReceiver -> TkEventListener

### New Features
- Serialization enabled for XBoxOne

### Bug Fixes
- Can change worker thread count in CPU dispatcher
- TkJoints created from the TkFramework::createJoint function are now released when the TkFramework is released
- Various fixes to unit tests
- Crash fix in CPU dispatcher
- Returning enough buffer space to handle hierarchical fracturing cases

### Known Issues
- Serialization requires documentation


## [1.0.0-beta] - 2017-01-24

### Changes
- Material API simplified (NvBlastProgramParams)
- Nv::Blast::ExtPhysics renamed Nv::Blast::ExtPx
- Various small ### changes to the low-level API (function renaming, argument list ### changes, etc.)
- Extensions libraries reconfigured according to major dependencies and functionality:
  - Authoring
  - Import (depends on PhysX and APEX)
  - PhysX (depends on PhysX)
  - Serialization (depends on PhysX and Cap'n Proto)
  - Shaders
- Source folder reorganization: low-level, Tk, and extensions all under an sdk folder

### New Features
- TkFamily serialization
- Versioned data serialization extensions for both low-level and Tk, based on Cap'n Proto
- TkJoint API, can create joints at runtime, attachments to Newtonian Reference Frame supported
- CMake projects
- PackMan used for dependencies
- Per-bond and per-chunk health initialization
- XBoxOne and Windows support for perf zones
- Timers in Tk
- Stress solver (automatic bond breaking)
- ExtPx asset serialization, combined TkAsset + PhysX collision meshes (.bpxa files)

### Removed Features
- TkComposite objects.  Composites may be created using the new TkJoint API in the TkFramework

### Known Issues
- Serialization requires documentation


## [1.0.0-alpha] - 2016-10-21

### Features
- Blast (low-level) library
- BlastTk (high-level) library
- BlastExt (extensions) library including:
  - AssetAuthoring
  - DataConverter
  - BlastID Utilities
  - ApexImporter Utilities
  - Materials
  - Physics Manager
  - Sync Layer
- Tools:
  - ApexImporter
  - DataConverter
  - AuthoringTool
- Samples:
  - SampleAssetViewer

### Known Issues
- Documentation incomplete
- TkFamily cannot be serialized
- Data conversion utility for Tk library does not exist
- Material API is still changing
