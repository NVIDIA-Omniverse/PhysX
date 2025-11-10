PhysxSchema Migration Guide
===========================

May 2021
----------
* PhysxMeshCollisionAPI was removed. It was replaced by PhysxConvexHullCollisionAPI, PhysxConvexDecompositionCollisionAPI and 
PhysxTriangleMeshSimplificationCollisionAPI. Based on the UsdPhysicsMeshCollisionAPI:approximation attribute the matching schema
is used. 
* PhysxCookingDataAPI was changed to multiple applied schema, the schema token defines which cooked data are stored. This allows us to store multiple versions of cooked data per mesh, for example convex approximation for a dynamic collision and triangle mesh for a static collision.

March 2021
----------

* General: 
   - For vehicles, many IsA schemas have been converted to API schemas. This allows for a more compact (fewer prims) vehicle setup for cases where there is no intent to share vehicle components among vehicle instances. For details, see further below.
* PhysxVehicleGlobalSettings:
  - Was renamed to PhysxSchemaPhysxVehicleContextAPI and is now an API schema.
  - In addition, this API has to be applied to a UsdPhysicsScene prim now.
* PhysxVehicleWheel:
   - Was renamed to PhysxSchemaPhysxVehicleWheelAPI and is now an API schema.
   - Optionally, this API can be applied directly to the prim that has PhysxSchemaPhysxVehicleWheelAttachmentAPI applied instead of using the physxVehicleWheelAttachment:wheel relationship
* PhysxVehicleTire:
   - Was renamed to PhysxSchemaPhysxVehicleTireAPI and is now an API schema.
   - Optionally, this API can be applied directly to the prim that has PhysxSchemaPhysxVehicleWheelAttachmentAPI applied instead of using the physxVehicleWheelAttachment:tire relationship
* PhysxVehicleSuspension:
   - Was renamed to PhysxSchemaPhysxVehicleSuspensionAPI and is now an API schema.
   - Optionally, this API can be applied directly to the prim that has PhysxSchemaPhysxVehicleWheelAttachmentAPI applied instead of using the physxVehicleWheelAttachment:suspension relationship
* PhysxVehicleEngine:
   - Was renamed to PhysxSchemaPhysxVehicleEngineAPI and is now an API schema.
   - Optionally, this API can be applied directly to the prim that has PhysxSchemaPhysxVehicleDriveStandardAPI applied instead of using the physxVehicleDriveStandard:engine relationship
* PhysxVehicleGears:
   - Was renamed to PhysxSchemaPhysxVehicleGearsAPI and is now an API schema.
   - Optionally, this API can be applied directly to the prim that has PhysxSchemaPhysxVehicleDriveStandardAPI applied instead of using the physxVehicleDriveStandard:gears relationship
* PhysxVehicleAutoGearBox:
   - Was renamed to PhysxSchemaPhysxVehicleAutoGearBoxAPI and is now an API schema.
   - Optionally, this API can be applied directly to the prim that has PhysxSchemaPhysxVehicleDriveStandardAPI applied instead of using the physxVehicleDriveStandard:autoGearBox relationship
* PhysxVehicleClutch:
   - Was renamed to PhysxSchemaPhysxVehicleClutchAPI and is now an API schema.
   - Optionally, this API can be applied directly to the prim that has PhysxSchemaPhysxVehicleDriveStandardAPI applied instead of using the physxVehicleDriveStandard:clutch relationship
* PhysxVehicleDriveBasic:
   - Was renamed to PhysxSchemaPhysxVehicleDriveBasicAPI and is now an API schema.
   - Optionally, this API can be applied directly to the prim that has PhysxSchemaPhysxVehicleAPI applied instead of using the physxVehicle:drive relationship
* PhysxVehicleDriveStandard:
   - Was renamed to PhysxSchemaPhysxVehicleDriveStandardAPI and is now an API schema.
   - Optionally, this API can be applied directly to the prim that has PhysxSchemaPhysxVehicleAPI applied instead of using the physxVehicle:drive relationship
