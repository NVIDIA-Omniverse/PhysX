local ext_name = "omni.physx.foundation"
local ext_dir = extsPhysics_dir.."/"..ext_name

-- helpers
shell_ext=".sh"
if os.target() == "windows" then
    shell_ext=".bat"
end

abs_path = bin_dir
abs_path = abs_path:gsub("%%{root}", root)
abs_path = abs_path:gsub("%%{platform}", platform_host)
abs_path = abs_path:gsub("%%{config}", "release")

function copy_extensions_from_cache(ext_name)
    postbuildcommands ("{MKDIR} "..extsPhysics_dir.."/../exts/"..ext_name)
    if os.target() == "windows" then
        postbuildcommands {"{COPYDIR} "..extsPhysics_dir.."/../exts/"..ext_name.." "..extsPhysics_dir.."/"..ext_name}
    else
        postbuildcommands {"cp -rL " .. extsPhysics_dir.."/../exts/"..ext_name .. " " .. extsPhysics_dir.."/"..ext_name}
    end
end

group ("simulation")

project ("omni.physx.foundation")
    repo_build.utility_project()
    location (workspaceDir.."/%{prj.name}")

    -- copy foundation, that is precached from precacheforbuild.kit and bump it
    copy_extensions_from_cache("omni.physx.foundation")

    -- currently we need to copy current physxgpu.dll to a copy of omni.physx.foundation, this copies the base
    -- the dll copy is in the foundation ext premake so it's done also when building in MSVS
    -- bump version in the copy, otherwise appdata folder of the same version will be prioritized
    exts_dir = abs_path.."/extsPhysics"
    postbuildcommands {root.."/_build/target-deps/python/python "..root.."/tools/bump_foundation_version.py "..exts_dir.." 107.3.0-oss.0 107.3.0-oss.1"}

    -- now postcache extensions to copy the rest of the exts from
    local call_cmd = abs_path.."/kit/kit "..abs_path.."/apps/postcacheforbuild.kit --allow-root --portable --ext-precache-mode --/crashreporter/gatherUserStory=0 --/app/settings/persistent=0 --/app/settings/loadUserConfig=0 --/app/extensions/parallelPullEnabled=1 --/exts/omni.kit.registry.nucleus/omitExtVersion=1 --/app/enableStdoutOutput=1 --/app/extensions/detailedSolverExplanation=1 --/app/extensions/registryEnabled=1 --/app/extensions/mkdirExtFolders=0 --/app/extensions/registryCacheFull="..abs_path.."/exts --/app/extensions/target/config=release --ext-folder "..abs_path.."/extsPhysics --portable-root "..abs_path
    print(call_cmd)
    postbuildcommands {call_cmd}

    -- copy to main folder so the exts are prioritized over non-oss ones
    copy_extensions_from_cache("omni.kvdb")
    copy_extensions_from_cache("omni.localcache")
    copy_extensions_from_cache("omni.physx.cooking")
    copy_extensions_from_cache("omni.physx.telemetry")
    copy_extensions_from_cache("omni.physics")
    copy_extensions_from_cache("omni.physics.stageupdate")
    copy_extensions_from_cache("omni.physics.physx")

    -- currently we need to copy current physxgpu.dll to a copy of omni.physx.foundation
    if _OPTIONS["devphysx"] then
        if os.target() == "windows" then
            -- turning off fastupdate will force this project to build each time and thus running the DLL copy
            -- this workarounds our inability to run this project's postbuild whenever a change is done in the PhysXGPU project
            fastuptodate "Off"
            dependson { "PhysXGpu" }
        end

        -- copy physxgpu.dll
        local ext_bin_dir = ext_dir.."/bin"
        
        local physx_win_dir = root.."/_build/target-deps/"..physxVersion..'/bin/win.x86_64.vc142.md/'
        local physx_linux_dir = root.."/_build/target-deps/"..physxVersion..'/bin/linux.x86_64/'
        local physx_aarch64_dir = root.."/_build/target-deps/"..physxVersion..'/bin/linux.aarch64/'

        filter { "system:windows", "platforms:x86_64", "configurations:release" }
            repo_build.copy_to_dir(physx_win_dir..'/'..physxLibs..'/PhysXGPU_64.dll', ext_bin_dir)
        filter { "system:linux", "platforms:x86_64", "configurations:release" }
            repo_build.copy_to_dir(physx_linux_dir..'/'..physxLibs..'/libPhysXGpu_64.so', ext_bin_dir)
        filter { "system:linux", "platforms:aarch64", "configurations:release" }
            repo_build.copy_to_dir(physx_aarch64_dir..'/'..physxLibs..'/libPhysXGpu_64.so', ext_bin_dir)
        filter {}
    end
