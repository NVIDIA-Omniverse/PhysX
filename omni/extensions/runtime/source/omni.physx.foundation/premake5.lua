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

    -- copy precached extension
    copy_extensions_from_cache("omni.physx.foundation")

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
