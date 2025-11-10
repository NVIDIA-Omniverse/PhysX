local arch = os.target()

group "ux"
    dofile ("source/omni.kit.property.physx/premake5.lua")
    dofile ("source/omni.physx.bundle/premake5.lua")
    dofile ("source/omni.physx.graph/premake5.lua")
    dofile ("source/omni.physx.cct/premake5.lua")
    dofile ("source/omni.physx.ui/premake5.lua")
    if arch ~= "arm64" then
        dofile ("source/omni.physx.pvd/premake5.lua")
    end
    dofile ("source/omni.physx.vehicle/premake5.lua")
    dofile ("source/omni.physx.vehicle.tests/premake5.lua")    
    dofile ("source/omni.physx.camera/premake5.lua")
    dofile ("source/omni.physx.tests/premake5.lua")
    dofile ("source/omni.physx.tests.visual/premake5.lua")
    dofile ("source/omni.physx.demos/premake5.lua")
    dofile ("source/omni.physx.commands/premake5.lua")
    dofile ("source/omni.physx.supportui/premake5.lua")
    dofile ("source/omni.usdphysics.ui/premake5.lua")
    dofile ("source/omni.physx.stageupdate/premake5.lua")
    dofile ("source/omni.physx.asset_validator/premake5.lua")
    dofile ("source/omni.usd.metrics.assembler.physics/premake5.lua")
