PHYSX_LIBS_DEFAULT = "checked"
BOOST_LIB = "boost_python311"
PYTHON_LIB = "python3.11"

function extension_physxsdk_deps(targetDepsDirIn, physxVersionIn, physxLibsIn)    
    local physxLibs = get_param_or_global_or_default(physxLibsIn, physxLibs, PHYSX_LIBS_DEFAULT)
    includedirs { targetDepsDirIn.."/"..physxVersionIn.."/include" }
    filter { "system:windows", "platforms:x86_64", "configurations:debug" }
        libdirs { targetDepsDirIn.."/"..physxVersionIn.."/bin/win.x86_64.vc142.md/debug" }
        defines {  "PX_PHYSX_STATIC_LIB", "_DEBUG" }
    filter { "system:windows", "platforms:x86_64", "configurations:release" }
        libdirs { targetDepsDirIn.."/"..physxVersionIn.."/bin/win.x86_64.vc142.md/"..physxLibs}
        defines {  "PX_PHYSX_STATIC_LIB", "NDEBUG" }
    filter { "system:windows" }
        links { "PhysXVehicle2_static_64", "PhysXExtensions_static_64", "PhysXCharacterKinematic_static_64", "PhysX_static_64", "PhysXPvdSDK_static_64","PhysXCooking_static_64","PhysXCommon_static_64", "PhysXFoundation_static_64" }
    filter { "system:linux" }            
        linkoptions { "-lpthread -Wl,--start-group -lPhysXExtensions_static_64 -lPhysX_static_64 -lPhysXPvdSDK_static_64 -lPhysXVehicle2_static_64 -lPhysXCharacterKinematic_static_64 -lPhysXCooking_static_64 -lPhysXCommon_static_64 -lPhysXFoundation_static_64 -Wl,--end-group -ldl" }        
    filter { "system:linux", "platforms:x86_64", "configurations:debug" }
        buildoptions { "-Wno-invalid-offsetof" }        
        defines {  "PX_PHYSX_STATIC_LIB", "_DEBUG" }
        libdirs { targetDepsDirIn.."/"..physxVersionIn.."/bin/linux.x86_64/debug" }   
    filter { "system:linux", "platforms:aarch64", "configurations:debug" }     
        buildoptions { "-Wno-invalid-offsetof" }    
        defines {  "PX_PHYSX_STATIC_LIB", "_DEBUG" }    
        libdirs { targetDepsDirIn.."/"..physxVersionIn.."/bin/linux.aarch64/debug" }   
    filter { "system:linux", "platforms:x86_64", "configurations:release" }
        buildoptions { "-Wno-invalid-offsetof" }
        defines {  "PX_PHYSX_STATIC_LIB", "NDEBUG" }
        libdirs { targetDepsDirIn.."/"..physxVersionIn.."/bin/linux.x86_64/"..physxLibs }        
    filter { "system:linux", "platforms:aarch64", "configurations:release" }
        buildoptions { "-Wno-invalid-offsetof" }
        defines {  "PX_PHYSX_STATIC_LIB", "NDEBUG" }
        libdirs { targetDepsDirIn.."/"..physxVersionIn.."/bin/linux.aarch64/"..physxLibs }        
    filter {} 
end

function link_boost_for_windows_wdefault(libs)
    local libs = libs or {BOOST_LIB}
    BOOST_COMPILER_TYPE = "vc142-mt"
    BOOST_VERSION = "x64-1_82"

    defines { "BOOST_ALL_NO_LIB", "BOOST_ALL_DYN_LINK" }

    filter { "system:windows", "configurations:debug" }
        for _, l in ipairs(libs) do
            links { l.."-"..BOOST_COMPILER_TYPE.."-gd-"..BOOST_VERSION}
        end

    filter { "system:windows", "configurations:release" }
        for _, l in ipairs(libs) do
            links { l.."-"..BOOST_COMPILER_TYPE.."-"..BOOST_VERSION}
        end

    filter {}
end

function usd_links(...)
    USD_PREFIX = "usd_"
    local libs = {...}
    prefixed_list = {}
    for _, lib in ipairs(libs[1]) do
        table.insert(prefixed_list, USD_PREFIX .. lib)
    end
    links(prefixed_list)
end

function get_platform()
    local platform = _OPTIONS["platform-target"] or platform_host or _OPTIONS["platform-host"]
    if platform == nil then
        error("--platform-target, --platform-host or platform_host must be specified")
    end
    return platform
end

function create_repo_runner(name, repo_tool, config, extra_args)
    RUNNER_SHELL_TEMPLATE = {
    ["windows"] = [[
@echo off
setlocal
call "%%~dp0..\..\..\repo.bat" %s %s %%*
]],
    ["linux"] = [[
#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
${EXEC:-exec} "$SCRIPT_DIR/../../../repo.sh" %s %s "$@"
]],
    }

    platform = get_platform()
    local os_target = os.target()
    file_ext = ".bat"

    if os_target ~= "windows" then
        file_ext = ".sh"
    end

    local file_dir = root.."/_build/"..platform.."/"..config
    local file_path = file_dir.."/"..name..file_ext
    local f = io.open(file_path, 'w')
    f:write(string.format(RUNNER_SHELL_TEMPLATE[os_target], repo_tool, extra_args))
    f:close()

    if os_target ~= "windows" then
        os.chmod(file_path, 755)
    end
end

function define_physics_test_experience(ext_name, args)
    local args = args or {}
    local ext_dir = get_value_or_default(args, "dir", "extsPhysics")
    local suite = get_value_or_default(args, "suite", "python")
    local script_dir_token = (os.target() == "windows") and "%~dp0" or "$SCRIPT_DIR"
    local extra_test_args = get_value_or_default(args, "test_args", {})
    local suffix = get_value_or_default(args, "suffix", "")

    local test_args
    if os.target() == "windows" then
        test_args = {
            script_dir_token.."apps/omni.bloky.test_ext.kit",
            "--/app/exts/folders/0='${kit}/../"..ext_dir.."'",
            "--/exts/omni.kit.test/testExtApp="..script_dir_token.."apps/omni.bloky.test_ext.kit",
            "--empty", -- Start empty kit
            "--portable",
            "--enable omni.kit.test", -- We always need omni.kit.test extension as testing framework
            "--/app/enableStdoutOutput=0", -- Do not print startup/shutdown messages for test runner itself (less noise).
            "--/exts/omni.kit.test/testExts/0='"..ext_name.."'", -- Only include tests from the python module
            "--ext-folder \""..script_dir_token.."/apps\" ", -- Auto add "exts" folder near batch files. For external repos.
        }
    else
        test_args = {
            script_dir_token.."/apps/omni.bloky.test_ext.kit",
            "--/app/exts/folders/0='${kit}/../"..ext_dir.."'",
            "--/app/exts/folders/1='${kit}/../apps'",
            "--/exts/omni.kit.test/testExtApp="..script_dir_token.."'/apps/omni.bloky.test_ext.kit'",
            "--empty", -- Start empty kit
            "--portable",
            "--enable omni.kit.test", -- We always need omni.kit.test extension as testing framework
            "--/app/enableStdoutOutput=0", -- Do not print startup/shutdown messages for test runner itself (less noise).
            "--/exts/omni.kit.test/testExts/0='"..ext_name.."'", -- Only include tests from the python module            
        }
    end

    test_args = concat_arrays(test_args, extra_test_args)

    local exp_args = {
        config_path = "",
        extra_args = table.concat(test_args, " "),
        define_project = false
    }
    exp_args = merge_tables(exp_args, args)

    define_experience("tests-"..suite.."-"..ext_name..suffix, exp_args)
end

function add_folder_overrides(args)
    if os.target() == "windows" then
        args = concat_arrays(args, {
            "--/app/exts/devFolders=['${kit}/../extsPhysics', '${kit}/../extsPhysicsRepo']",
            "--/app/exts/folders/0='${kit}/../extsPhysics'",
            "--/app/exts/folders/1='${kit}/../extsPhysicsRepo'",
        })
    else
        args = concat_arrays(args, {
            "--/app/exts/devFolders/0=\"$SCRIPT_DIR/extsPhysics\"",
            "--/app/exts/devFolders/1=\"$SCRIPT_DIR/extsPhysicsRepo\"",
            "--/app/exts/folders/0=\"$SCRIPT_DIR/extsPhysics\"",
            "--/app/exts/folders/1=\"$SCRIPT_DIR/extsPhysicsRepo\"",
        })
    end
end

function define_etm_test_experience(ext_name)
    -- not relevant for OSS release
end

function get_prebuild_files()
    return { os.matchfiles("premake5.lua"), os.matchfiles("deps/*"), os.matchfiles("tools/buildscripts/*.*") }
end

function get_version()
    local file = io.open("VERSION", "r")
    local version = "0"
    if file then
        version = file:read "*all"
        file:close()        
    end
    return version
end

function get_param_or_global_or_default(param, glb, default)
    if param ~= nil then
        return param
    end
    if glb ~= nil then
        return glb
    end
    return default
end

function get_include_string(includes)
    cmdString =" ";
    for k, v in pairs(includes) do
        cmdString = cmdString.." -I "..tostring(v);
    end
    return cmdString
end

function trim(s)
    return (s:gsub("^%s*(.-)%s*$", "%1"))
end

-- replaces spaces with commas (used to pack arguments to nvcc -Xcompiler)
function commaficate(options)
    return string.gsub(trim(options), "%s+", ",")
end

function make_nvcc_command(nvccPath, nvccHostCompilerVS, nvccHostCompilerFlags, nvccFlags)
    if os.target() == "windows" then
        ext = ".obj"
        local compilerBindir = " --compiler-bindir "..nvccHostCompilerVS
        local buildString =  "\""..nvccPath.."\"".." "..nvccFlags..compilerBindir.." -t0 -Xcompiler="..commaficate(nvccHostCompilerFlags)..",/Zc:__cplusplus --std c++17 -c %{get_include_string(cfg.includedirs)} %{file.abspath} -o %{cfg.objdir}/%{file.basename}"..ext
        buildmessage (buildString)
        buildcommands { buildString }
        buildoutputs { "%{cfg.objdir}/%{file.basename}"..ext }
    end
    if os.target() == "linux" then
        ext = ".o"
        local buildString =  "\""..nvccPath.."\" -std=c++17 "..nvccFlags.." -t0 -Xcompiler="..commaficate(nvccHostCompilerFlags).." --std c++17 -c %{get_include_string(cfg.includedirs)} %{file.abspath} -o %{cfg.objdir}/%{file.basename}"..ext
        buildcommands { "{MKDIR} %{cfg.objdir} ", buildString }
        buildoutputs { "%{cfg.objdir}/%{file.basename}"..ext }
    end
end

function add_nvcc_commands()
    local nvccPath = root.."/_build/target-deps/cuda/bin/nvcc"
    local nvccHostCompilerVS = root.."/_build/host-deps/msvc/VC"

    -- Note: This is mostly copy'n'pasted from the rendering premake5.lua
    -- Create pre-compiled SASS for all architectures that are supported by CUDA 11.5 (Maxwell to Ampere)
    local sass = "-gencode=arch=compute_52,code=sm_52 ".. -- Maxwell
                 "-gencode=arch=compute_53,code=sm_53 "..
                 "-gencode=arch=compute_60,code=sm_60 ".. -- Pascal
                 "-gencode=arch=compute_61,code=sm_61 "..
                 "-gencode=arch=compute_62,code=sm_62 "..
                 "-gencode=arch=compute_70,code=sm_70 ".. -- Volta
                 "-gencode=arch=compute_72,code=sm_72 "..
                 "-gencode=arch=compute_75,code=sm_75 ".. -- Turing
                 "-gencode=arch=compute_80,code=sm_80 ".. -- Ampere
                 "-gencode=arch=compute_86,code=sm_86 "..
                 "-gencode=arch=compute_87,code=sm_87 "..
                 "-gencode=arch=compute_89,code=sm_89 ".. -- Ada
                 "-gencode=arch=compute_90,code=sm_90 "   -- Hopper
    -- For forward compatibility with future architectures we also include a Hopper-based PTX that can be JIT compiled at runtime
    local ptx = "-gencode=arch=compute_90,code=compute_90 "

    filter { "files:**.cu", "system:windows", "configurations:debug" }
        make_nvcc_command(nvccPath, nvccHostCompilerVS, "/MDd", sass..ptx.."-g -G")
    filter { "files:**.cu", "system:windows", "configurations:release" }
        make_nvcc_command(nvccPath, nvccHostCompilerVS, "/MD /DNDEBUG", sass..ptx)
    filter { "files:**.cu", "system:linux", "configurations:debug" }
        make_nvcc_command(nvccPath, nvccHostCompilerVS, "-fPIC -g -D_DEBUG", sass..ptx.."-g")
    filter { "files:**.cu", "system:linux", "configurations:release" }
        make_nvcc_command(nvccPath, nvccHostCompilerVS, "-fPIC -O3 -DNDEBUG", sass..ptx)
    filter {}
end

function add_cuda_dirs()
    includedirs {
        root.."/_build/target-deps/cuda/include",
    }
    filter { "system:windows" }
        libdirs {
            root.."/_build/target-deps/cuda/lib/x64",
        }
    filter { "system:linux" }
        libdirs {
            root.."/_build/target-deps/cuda/lib64",
            root.."/_build/target-deps/cuda/lib64/stubs",
        }
    filter {}
end

function add_cuda_deps(targetDepsDirIn)
    includedirs {
        targetDepsDirIn.."/cuda/include",
    }

    filter { "system:windows" }
        libdirs {
            targetDepsDirIn.."/cuda/lib/x64",
        }
        links { "cuda", "delayimp" }
        linkoptions { "/DELAYLOAD:nvcuda.dll" }
    filter { "system:linux" }
        -- On Linux we don't link cuda at all, we just make sure it's in the libdirs
        libdirs {
            targetDepsDirIn.."/cuda/lib64",
            targetDepsDirIn.."/cuda/lib64/stubs",
        }
        -- These are needed to prevent symbols from "cuda" lib being loaded on no-gpu runs
        removelinkoptions("-Wl,--no-undefined")
        removelinkoptions("-Wl,-z,now")
    filter {}
end

function extension_omniui_deps()
    libdirs {                
        kit_sdk_exts_build.."/omni.ui.scene/bin", 
        kit_sdk_exts_build.."/omni.ui/bin",
        kit_sdk_exts_deps.."/omni.ui.scene/bin", 
        kit_sdk_exts_deps.."/omni.ui/bin",
    }
end

function extension_imgui_deps()
    defines { "IMGUI_NVIDIA" }
    includedirs {
        targetDeps_dir.."/imgui",
    }
    libdirs {                
        kit_sdk_exts_build.."/omni.kit.renderer.imgui/bin",
        kit_sdk_exts_deps.."/omni.kit.renderer.imgui/bin",
    }
    links { "imgui" }
end

function extension_usd_deps(targetDepsDirIn, hostDepsDirIn)    
    staticruntime "Off"
    exceptionhandling "On"
    rtti "On"
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
    filter {}

    dependson { "prebuild", "physxSchema", "physicsSchemaTools", "_physxSchema", "_physicsSchemaTools" }
    includedirs {
        extensions_root_dir.."/pch",
        targetDepsDirIn.."/python/include",
        targetDepsDirIn.."/omni-usd-plugin/%{cfg.buildcfg}/include",
        schema_package_dir.."/include",
        targetDepsDirIn.."/usd_audio_schema/%{cfg.buildcfg}/include",
        targetDepsDirIn.."/usd_schema_semantics/%{cfg.buildcfg}/include",
        targetDepsDirIn.."/usd/%{cfg.buildcfg}/include",
        hostDepsDirIn.."/mirror/include",
        targetDepsDirIn.."/rtx_plugins/include",
        targetDeps_dir.."/gsl/include",   -- Support for std::span
    }
    libdirs {
        schema_package_dir.."/lib",
        targetDepsDirIn.."/usd_audio_schema/%{cfg.buildcfg}/lib",
        targetDepsDirIn.."/usd_schema_semantics/%{cfg.buildcfg}/lib",
        targetDepsDirIn.."/usd/%{cfg.buildcfg}/lib",
        kit_sdk_exts_build.."/omni.usd.core/bin",
        kit_sdk_exts_deps.."/omni.usd.core/bin",
    }
    usd_links {
        "ar", "arch", "gf", "js", "kind", "pcp", "plug", "sdf", "tf", "trace", "usd", "usdGeom", "usdSkel", "usdShade", "vt", "work", "pxOsd",
        "hdx", "hd", "usdImaging",  "usdLux", "usdUtils", "usdPhysics"
    }
    filter { "system:windows" }
        if not _OPTIONS["nopch"] then
            removeflags { "NoPCH" }
            pchheader "UsdPCH.h"
            pchsource (extensions_root_dir.."/pch/UsdPCH.cpp")
            files { extensions_root_dir.."/pch/UsdPCH.cpp" }
        end
        libdirs {   targetDepsDirIn.."/python/libs",
                    targetDepsDirIn.."/tbb/lib/intel64/vc14",
                    targetDepsDirIn.."/usd/%{cfg.buildcfg}/lib" }
        includedirs { targetDepsDirIn.."/opensubdiv/%{cfg.buildcfg}/include" }
    filter { "system:linux" }
        exceptionhandling "On"
        removeflags { "UndefinedIdentifiers" }
        includedirs { targetDepsDirIn.."/usd/%{cfg.buildcfg}/include/boost",
                    targetDepsDirIn.."/python/include/"..PYTHON_LIB }
        buildoptions { "-pthread" }
        libdirs { targetDepsDirIn.."/python/lib" }
        links { BOOST_LIB, PYTHON_LIB, "tbb", "dl", "pthread" }
        defines { "TBB_SUPPRESS_DEPRECATED_MESSAGES",}
    filter {}
end

function extension_usd_deps_tests(targetDepsDirIn, hostDepsDirIn)    
    staticruntime "Off"
    exceptionhandling "On"
    rtti "On"
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
    filter {}
    includedirs { extensions_root_dir.."/pch",
                  targetDepsDirIn.."/python/include",
                  targetDepsDirIn.."/omni-usd-plugin/%{cfg.buildcfg}/include",
                  schema_package_dir.."/include",
                  targetDepsDirIn.."/usd_audio_schema/%{cfg.buildcfg}/include",
                  targetDepsDirIn.."/usd_schema_semantics/%{cfg.buildcfg}/include",
                  targetDepsDirIn.."/usd/%{cfg.buildcfg}/include",
                  hostDepsDirIn.."/mirror/include",
                  targetDepsDirIn.."/rtx_plugins/include" }
    libdirs { schema_package_dir.."/lib",
              targetDepsDirIn.."/usd_audio_schema/%{cfg.buildcfg}/lib",
              targetDepsDirIn.."/usd_schema_semantics/%{cfg.buildcfg}/lib",
              targetDepsDirIn.."/usd/%{cfg.buildcfg}/lib"}
    usd_links {
        "arch", "sdf", "tf", "gf", "vt", "usd", "usdGeom", "usdUtils", "usdShade", "usdLux", "usdPhysics"
    }
    filter { "system:windows" }
        if not _OPTIONS["nopch"] then
            removeflags { "NoPCH" }
            pchheader "UsdPCH.h"
            pchsource (extensions_root_dir.."/pch/UsdPCH.cpp")
            files { extensions_root_dir.."/pch/UsdPCH.cpp" }
        end
        libdirs {   targetDepsDirIn.."/python/libs",
                    targetDepsDirIn.."/tbb/lib/intel64/vc14",
                    targetDepsDirIn.."/usd/%{cfg.buildcfg}/lib" }
        includedirs { targetDepsDirIn.."/opensubdiv/%{cfg.buildcfg}/include" }
    filter { "system:linux" }
        exceptionhandling "On"
        removeflags { "UndefinedIdentifiers" }
        includedirs { targetDepsDirIn.."/usd/%{cfg.buildcfg}/include/boost",
                    targetDepsDirIn.."/python/include/"..PYTHON_LIB }
        buildoptions { "-pthread" }
        links { BOOST_LIB, "tbb", "dl", "pthread" }
    filter {}
end

 -- Retrieves a list of paths from input where each individual path
-- is separated by the character specified by path_separator
function get_path_list(input, path_separator, prepend_path)
    local paths = {}
    for path in input:gmatch('[^'..path_separator..']+') do
        paths[#paths + 1] = prepend_path..path
    end

    return paths
end

function getLibSuffix()
    filter {"system:windows"}
        suffix = ".lib"
    filter {"system:linux"}
        suffix = ""
    filter {}

    return suffix
end

function firstToUpper(str)
    return (str:gsub("^%l", string.upper))
end

function schema_usd_deps(targetDepsDirIn)
    staticruntime "Off"
    exceptionhandling "On"
    rtti "On"
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
    filter {}
    includedirs {
                  targetDepsDirIn.."/python/include",
                  targetDepsDirIn.."/usd/%{cfg.buildcfg}/include",
    }
    libdirs {
              targetDepsDirIn.."/usd/%{cfg.buildcfg}/lib"}
    usd_links {
        "ar", "arch", "gf", "js", "kind", "pcp", "plug", "sdf", "tf", "trace", "usd", "usdGeom", "usdSkel", "usdShade", "vt", "work", "pxOsd",
        "hdx", "hd", "usdImaging",  "usdLux", "usdUtils", "usdPhysics"
    }
    filter { "system:windows" }
        libdirs {   targetDepsDirIn.."/python/libs",
                    targetDepsDirIn.."/usd/%{cfg.buildcfg}/lib" }
    filter { "system:linux" }
        exceptionhandling "On"
        removeflags { "UndefinedIdentifiers" }
        includedirs { targetDepsDirIn.."/usd/%{cfg.buildcfg}/include/boost",
        targetDepsDirIn.."/python/include/"..PYTHON_LIB }
        buildoptions { "-pthread" }
        links { BOOST_LIB, "tbb", "dl", "pthread" }
    filter {}
 end

function project_usd_schema(schemaname, source, targetdir)
    targetname(schemaname)

    os.remove(source.."/premake5.lua")

    -- copies
    repo_build.prebuild_copy
    {
        { source.."/generatedSchema.usda", targetdir.."/resources" },
        { source.."/schema.usda", targetdir.."/resources/"..schemaname },
        { source.."/plugInfo_%{platform}.json", targetdir.."/resources/plugInfo.json" },
    }

    -- specify the files to include
    files { source.."/**.h", source.."/**.cpp"}
    removefiles { source.."/wrap*.cpp" }

    local schemaname_upper = schemaname:upper()
    local schemaname_first_upper = firstToUpper(schemaname)

    -- preprocessor directives
    defines {
        "PXR_PYTHON_ENABLED=1",
        "MFB_PACKAGE_NAME="..schemaname,
        "MFB_ALT_PACKAGE_NAME="..schemaname,
        "MFB_PACKAGE_MODULE="..schemaname_first_upper,
        "NOMINMAX",
        "BOOST_ALL_DYN_LINK",
        "PXR_PYTHON_MODULES_ENABLED=1",
        schemaname_upper.."_EXPORTS"
    }

    usd_links(
        {"usd", "sdf", "tf", "arch", "vt", "usdGeom"}
    )

    filter {"system:windows", "configurations:release" }
        defines { "WIN32", "_WINDOWS", "NDEBUG", "TBB_USE_THREADING_TOOLS" }
        buildoptions { "/Zc:rvalueCast", "/Zc:inline-", "/W3", "/WX-", "/Zi", "/GR" }
    filter {"system:windows", "configurations:debug" }
        defines { "WIN32", "_WINDOWS", "_DEBUG", "TBB_USE_DEBUG", "TBB_USE_ASSERT", "TBB_USE_THREADING_TOOLS" }
        buildoptions { "/Zc:rvalueCast", "/Zc:inline-", "/W3", "/WX-", "/Zi", "/GR" }
    filter {"system:linux", "configurations:release" }
        defines { "LINUX", "NDEBUG", "TBB_USE_THREADING_TOOLS" }
        buildoptions { "-Wno-deprecated", "-Wno-deprecated-declarations", "-Wno-inconsistent-missing-override" }
    filter {"system:linux", "configurations:debug" }
        buildoptions { "-Wno-deprecated", "-Wno-deprecated-declarations", "-Wno-inconsistent-missing-override" }
        defines { "LINUX", "TBB_USE_DEBUG", "TBB_USE_ASSERT", "TBB_USE_THREADING_TOOLS" }
    filter {}

    local additional_python_files = get_path_list(source.."/module.cpp", ";", "")
    removefiles { additional_python_files }
end

function project_usd_schema_python(schemaname, source)
    targetname("_"..schemaname)
    targetprefix("")

    local schemaname_upper = schemaname:upper()
    local schemaname_first_upper = firstToUpper(schemaname)

    -- preprocessor directives
    defines {
        "PXR_PYTHON_ENABLED=1",
        "MFB_PACKAGE_NAME="..schemaname,
        "MFB_ALT_PACKAGE_NAME="..schemaname,
        "MFB_PACKAGE_MODULE="..schemaname_first_upper,
        "NOMINMAX",
        "BOOST_ALL_DYN_LINK",
        "BOOST_PYTHON_NO_PY_SIGNATURES",
        "_"..schemaname_upper.."_EXPORTS"
    }

    filter {"system:windows", "configurations:release" }
        defines { "WIN32", "_WINDOWS", "NDEBUG", "TBB_USE_THREADING_TOOLS" }
        buildoptions { "/WX-" }
    filter {"system:windows", "configurations:debug" }
        defines { "WIN32", "_WINDOWS", "_DEBUG", "TBB_USE_DEBUG", "TBB_USE_ASSERT", "TBB_USE_THREADING_TOOLS" }
        buildoptions { "/WX-" }
    filter {"system:linux", "configurations:release" }
        defines { "LINUX", "NDEBUG", "TBB_USE_THREADING_TOOLS" }
        buildoptions { "-Wno-deprecated", "-Wno-deprecated-declarations", "-Wno-inconsistent-missing-override" }
    filter {"system:linux", "configurations:debug" }
        buildoptions { "-Wno-deprecated", "-Wno-deprecated-declarations", "-Wno-inconsistent-missing-override" }
        defines { "LINUX", "TBB_USE_DEBUG", "TBB_USE_ASSERT", "TBB_USE_THREADING_TOOLS" }
    filter {}

    usd_links({"usd", "sdf", "tf", "arch", "vt", "usdGeom" })

    -- python library also needs to link the C++ library
    links( schemaname..getLibSuffix())

    filter {"system:windows"}
        targetextension(".pyd")
    filter {"system:linux"}
        targetextension(".so")
    filter {}

    -- specify the files to include
    files { source.."/wrap*.cpp" }

    local additional_python_files = get_path_list(source.."/module.cpp", ";", "")
    files { additional_python_files }
end

function create_test_runner(name, config, env_path)
    if os.target() == "windows" then
        local bat_file_dir = root.."/_build/windows-x86_64/"..config
        local bat_file_path = bat_file_dir.."/"..name..".physics.bat"        
        local f = io.open(bat_file_path, 'w')        
        f:write(string.format([[
@echo off
setlocal
set PATH="%%PATH%%";%s
call "%%~dp0\plugins\%s.exe" %%*
        ]], env_path, name))
        f:close()
    else
        local cmd_io = assert(io.popen('uname -m', 'r'))
        local arch_name = assert(cmd_io:read('*a'))
        cmd_io:close()
        arch_name = arch_name:gsub('[%c%s\"\']', '')
        local sh_file_dir = string.format("%s/_build/linux-%s/%s", root, arch_name, config)
        local sh_file_path = sh_file_dir.."/"..name..".physics.sh"   
        local f = io.open(sh_file_path, 'w')
        f:write(string.format([[
#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:%s
"$SCRIPT_DIR/plugins/%s" "$@"
        ]], env_path, name))
        f:close()
        os.chmod(sh_file_path, 755)
    end
end

-- common plugins settings (this function is called in every plugin project)
function carbonitePlugin(args)
    targetdir (targetPluginsDir)
    if args.targetname then            
        targetname (args.targetname)
    end
    kind "SharedLib"
    staticruntime "Off"
    defines { "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)" }    
    location (workspaceDir.."/%{prj.name}")
    includedirs { 
        runtime_include_dir,
        extensions_root_dir.."/common/include",
        repo_root_dir.."/include/extras",
    }
    if type(args.ifaces) == "string" and type(args.impl) == "string" then
        filesTable = {}
        vpathsTable = {}
        table.insert(filesTable, args.impl.."/**.*")
        vpathsTable["impl/*"] = args.impl.."/**.*"
        ifaces_str = args.ifaces .. ";"
        ifaces = repo_build.split(ifaces_str, ";")
        for idx, ifacePath in ipairs(ifaces) do
            interfaceHeader = string.find(ifacePath, "%.h$")
            if interfaceHeader == nil then
                -- Interface folder specified
                table.insert(filesTable, ifacePath.."/*.h")
                lastSlash = string.find(ifacePath, "/[^/]*$")
                if lastSlash == nil then
                    vpathsTable[ifacePath] = ifacePath.."/*.h"
                else
                    vpathsTable[string.sub(ifacePath, lastSlash + 1)] = ifacePath.."/*.h"
                end
            else
                -- Interface header specified
                table.insert(filesTable, ifacePath)
                lastSlash = string.find(ifacePath, "/[^/]*$")
                ifacePathOnly = string.sub(ifacePath, 0, lastSlash - 1)
                lastSlash = string.find(ifacePathOnly, "/[^/]*$")
                if lastSlash == nil then
                    vpathsTable[ifacePath] = ifacePath
                else
                    currentIfacePath = string.sub(ifacePathOnly, lastSlash + 1)
                    vpathsTable[currentIfacePath] = vpathsTable[currentIfacePath] or {}
                    table.insert(vpathsTable[currentIfacePath], ifacePath)
                end
            end
        end
        files { filesTable }
        vpaths { vpathsTable }
    end
end

function carboniteBindingsPython(args)
    local name = args.name
    local namespace = "carb"
    if type(args.namespace) == "string" then
        namespace = args.namespace
    end
    local folder = "source/bindings/python/"..namespace.."."..name
    if type(args.folder) == "string" then
        folder = args.folder
    end

    targetdir (targetDir.."/plugins/bindings-python/"..namespace)
    files { folder.."/*.*", "source/bindings/python/*.*" }
    vpaths { [''] = folder.."/*.*", ['common'] = "source/bindings/python/*.*" }
    dependson { "carb" }
    links {"carb" }
    location (workspaceDir.."/%{prj.name}")
    includedirs { 
        runtime_include_dir,
        extensions_root_dir.."/common/include",
    }

    local python_folder = root.."/_build/target-deps/python"
    repo_build.define_bindings_python(name, python_folder, "3.11")
end

function workspace_defaults(currentAbsPath, hostDepsDir, targetDepsDir)
    configurations { "release" }
    startproject "omni.bloky.kit"

    winsdk_version = "10.0.17763.0"
    targetDeps_dir = root.."/_build/target-deps"
    hostDeps_dir = root.."/_build/host-deps"

    repo_build.ccache_compiler_options()

    repo_build.setup_workspace {
        msvc_version = "14.29.30133",
        winsdk_version = winsdk_version,
        extra_warnings = false,
        security_hardening = true,
        host_deps_dir = hostDeps_dir,
        target_deps_dir = targetDeps_dir,
        fix_cpp_version = true,
        use_stack_clash_protection = false, -- OVX gcc does not support this
    }

    platform = repo_build.platform()
    targetDir = repo_build.target_dir()
    local targetPluginsDir = targetDir.."/plugins"
    exts_dir = "extsPhysics"
    exts_target_dir = targetDir.."/"..exts_dir

    -- defining CARB_SDK dependencies
    local carbSDKPath = currentAbsPath.."/_build/target-deps/carb_sdk_plugins"
    local carbSDKInclude = carbSDKPath.."/include"
    local carbSDKLibs = carbSDKPath.."/_build/"..repo_build.platform().."/%{cfg.buildcfg}"

    local pybindInclude = currentAbsPath.."/_build/target-deps/pybind11/include"

    location (workspaceDir)
    targetdir (targetDir)
    if _OPTIONS["no-symbols"] then
        print("Turning off symbols generation.")
        symbols "Off"
    else
        symbols "Full"
    end
    staticruntime "On"
    buildoptions { "-DPX_ENABLE_FEATURES_UNDER_CONSTRUCTION=1" }

    includedirs { "_build/generated/include", "_build/target-deps", "_build/mirrored", "_build/host-deps", "include", "shaders", "pch" }
    syslibdirs { carbSDKLibs, targetPluginsDir }
    externalincludedirs { carbSDKInclude, pybindInclude }

    filter { "system:linux", "configurations:release" }
        -- there are just too many of these in release, they drown out other warnings in the noise
        disablewarnings { "unused-variable", "unused-but-set-variable", "unused-function" }
    filter {}

    filter { "system:windows" }
        platforms { "x86_64" }       
        -- all of our source strings and executable strings are utf8
        buildoptions {"/utf-8"}
        buildoptions {"/permissive-"}

        -- A.B. temp added here, should be added only for schema builds
        disablewarnings { "4244", "4305", "4996" }

    filter { "system:linux" }
        platforms { "x86_64", "aarch64" }
        defaultplatform "x86_64"
        buildoptions { "-fvisibility=hidden -Wfatal-errors" }
        -- add library origin directory to dlopen() search path
        linkoptions { "-Wl,-rpath,'$$ORIGIN'" }
        enablewarnings { "all", "error=invalid-pch" }
        disablewarnings {
            -- this is temporary. once the build warnings are fixed, these should all be enabled again
            "error=unused-variable",
            "error=unused-but-set-variable",
        }
        if repo_build.ccache_path() then
            gccprefix (repo_build.ccache_path().." ")
        end

    filter { "platforms:x86_64" }
        architecture "x86_64"

    filter { "platforms:aarch64" }
        architecture "ARM"
        buildoptions { "-ffp-contract=off" }
        local hostDependencyPlatform = _OPTIONS["platform-host"] or platform;
        if hostDependencyPlatform ~= "linux-aarch64" then
            defines { "CARB_TEGRA=1" }
            minimalArmKit = true
        else
            defines { "CARB_TEGRA=0" }
            minimalArmKit = false
        end

    filter { "configurations:debug" }
        runtime "Debug"
        defines { "DEBUG", "_DEBUG" }
        optimize "Off"

    filter { "system:windows", "configurations:debug" }
        -- Work around https://github.com/intel/tbb/issues/154
        defines { "TBB_USE_DEBUG=1" }

    filter  { "configurations:release" }
        runtime "Release"
        defines { "NDEBUG" }
        optimize "Speed"

    filter {}
end

function do_ogn_config_quickfix()
    repo_build.prebuild_copy {    
        { "_build/target-deps/kit_sdk_%{config}/dev/ogn/config/**", "_build/ogn/config" },
    }
end

-- Prevent command line warning D9025: overriding '/Zc:inline' with '/Zc:inline-'
-- from "/Zc:inline-" set by use_usd in premake5-usd.lua
function do_usd_zcinline_fix()
    filter { "system:windows" }
    filter { "toolset:msc-v142" }
    buildoptions { removeunreferencedcodedata ("off") }
    filter {}
end

function define_options()   
    newoption {
        trigger     = "devphysx",
        description = "(Optional) Development build which includes PhysX SDK solution",
        value = "PATH",
    }

    newoption {
        trigger     = "nopch",
        description = "(Optional) Disable precompile headers",    
    }
    
    newoption {
        trigger     = "notensors",
        description = "(Optional) Don't build tensor API extensions.  A HACK to work around a CUDA build bug.",
    }

    newoption {
        trigger     = "devschema",
        description = "(Optional) Use local schema",
    }

    newoption {
        trigger     = "no-symbols",
        description = "(Optional) Generate no debug info",
    }
end

function define_paths(repo_root_dir_rel, extension_dir_rel)
    -- roots
    root = repo_build.get_abs_path(".")
    repo_build.root = root
    repo_root_dir = path.getabsolute(root..repo_root_dir_rel)
    extensions_root_dir = path.getabsolute(root..extension_dir_rel)
    runtime_include_dir = path.getabsolute(root.."/include")

    print("repo_root_dir: "..repo_root_dir)
    print("extensions_root_dir: "..extensions_root_dir)
    print("runtime_include_dir: "..runtime_include_dir)

    -- base paths
    hostDepsDir = "_build/host-deps"
    mirroredDir = "_build/mirrored"
    targetDepsDir = "_build/target-deps"
    currentAbsPath = repo_build.get_abs_path(".");
    targetName = _ACTION
    workspaceDir = currentAbsPath.."/_compiler/"..targetName

    -- path to kit sdk
    kit_sdk = root.."/_build/target-deps/kit_sdk_%{config}" -- abs with kit tokens
    kit_sdk_bin_dir = root.."/_build/%{platform}/%{config}/kit"

    kit_sdk_dir = root.."/_build/target-deps/kit_sdk_%{cfg.buildcfg}" -- abs with premake tokens
    kit_sdk_exts_build = kit_sdk_bin_dir.."/../exts" -- cached exts by kit-kernel
    kit_sdk_exts_deps = kit_sdk_dir.."/exts" -- exts bundled with kit and when building with local kit
    kit_sdk_includes = kit_sdk_dir.."/dev/include"

    extsbuild_dir = kit_sdk_exts_build -- needed for ogn_helpers to find the omni.graph exts

    -- path to physics schema
    local_schema_dir = root.."/_build/%{platform}/%{config}/schema"
    deps_schema_dir = root.."/"..targetDepsDir.."/usd_ext_physics/%{cfg.buildcfg}"
    
    schema_package_dir = iif(_OPTIONS["devschema"], local_schema_dir, deps_schema_dir)
    schema_source_dir = root.."/schema/source"
    print("schema_package_dir: "..schema_package_dir)
    repo_root_dir = path.getabsolute(root..repo_root_dir_rel)
    print("repo_root_dir: "..repo_root_dir)
    extensions_root_dir = path.getabsolute(root..extension_dir_rel)
    print("extensions_root_dir: "..extensions_root_dir)
    runtime_include_dir = path.getabsolute(extensions_root_dir.."/runtime/include")
    print("runtime_include_dir: "..runtime_include_dir)
end

function define_tags()
    physxVersion = "physx"
    physxTag = ""
    extensionVersion = get_version().."-5.1"
    extsPhysics_dir = extsPhysics_dir or bin_dir.."/extsPhysics"
    extsPhysics_repo_dir = extsPhysics_repo_dir or bin_dir.."/extsPhysicsRepo"
end

function include_helpers()
    local premakePublicFile = "premake5-public.lua"
    local ognHelperFile = "ogn/ogn_helpers.lua"

    local devDirR = targetDepsDir.."/kit_sdk_release/dev/"
    local devDirD = targetDepsDir.."/kit_sdk_debug/dev/"

    -- include Kit SDK public premake, it defines few global variables and helper functions. Look inside to get more info.    
    local _ = dofileopt (devDirR..premakePublicFile) or dofileopt (devDirD..premakePublicFile)
    -- include OmniGraph tool processing script, kitsdk's premake5-public.lua is not including it properly from a package
    local _ = dofileopt (devDirR..ognHelperFile) or dofileopt (devDirD..ognHelperFile)
end

function define_common(repo_root_dir_rel, extension_dir_rel)
    repo_build = require("omni/repo/build")
    repo_build.setup_options()

    define_options()
    define_paths(repo_root_dir_rel, extension_dir_rel)
    include_helpers()
    define_tags()

    physxLibs = PHYSX_LIBS_DEFAULT

    if os.target() == "windows" then
        -- Windows custom build tools merge everything together into one script so the Python batch file
        -- has to be "called" or it will only run the first one.
        pythonScriptPath = "call "..repo_root_dir.."/tools/packman/python.bat"
    else
        -- path.getabsolute gives the realpath rather than the symbolic path in linux
        pythonScriptPath = repo_root_dir.."/tools/packman/python.sh"
    end
    print("pythonScriptPath: "..pythonScriptPath)
    
    -- setup where to write generate prebuild.toml file
    repo_build.set_prebuild_file('_build/generated/prebuild.toml')

    do_ogn_config_quickfix()

    -- include optimization settings
    include(repo_root_dir.."/premake5.user.defaults.lua")
    local opti_user_file = repo_build.root.."/premake5.user.lua"
    if os.isfile(opti_user_file) then
        include(opti_user_file)
    end
end

function add_generate_solution_project()
    if os.target() == "windows" then
        project "generate_solution"
            kind "None" -- This project only builds if manually build/rebuild in Visual Studio. It's not included in buildchain.
            location (workspaceDir.."/%{prj.name}")
            files ( get_prebuild_files() ) -- please add files into the get_prebuild_files function.
            buildcommands { "echo Generating Visual Studio solution...", "cd ..\\..\\..", "prebuild.bat" }
            rebuildcommands { "echo Regenerating Visual Studio solution...","cd ..\\..\\..", "rebuild.bat -g" }
    end
end

function get_uuid(vcxprojPath)
    local file = io.open(vcxprojPath, "r")
    if file then
        local content = file:read("*all")
        file:close()
        local startIdx, endIdx, uuid = content:find("<ProjectGuid>{(.-)}</ProjectGuid>")
        
        if uuid then
            return uuid
        end
    end

    return os.uuid(vcxprojPath)
end


function include_physxsdk(physxLibsIn)
    local physxLibs = get_param_or_global_or_default(physxLibsIn, physxLibs, PHYSX_LIBS_DEFAULT)
    local physxSourceDir = _OPTIONS["devphysx"]

    group "PhysX SDK"
        local physxSDKSourceDir = physxSourceDir.."/compiler/vc16win64-carbonite/sdk_source_bin"
        local projectFiles = os.matchfiles(physxSDKSourceDir.."/*.vcxproj")        
        for i, projectFile in ipairs(projectFiles) do
            k, l = string.find(projectFile, ".vcxproj")                
            local projectName = (string.sub(projectFile, string.len(physxSDKSourceDir) + 2, k - 1))
            if projectName ~= "INSTALL" and projectName ~= "ALL_BUILD" then
                externalproject (projectName)
                    location (physxSDKSourceDir)
                    local uid = get_uuid(projectFile)
                    uuid(uid)
                    kind "StaticLib"
                    language "C++"
                    systemversion (winsdk_version)
                    configmap {
                        ["debug"] = "debug",
                        ["release"] = physxLibs,
                    }
            end
        end

    group "PhysX SDK GPU"
        local physxSDKSourceDir = physxSourceDir.."/compiler/vc16win64-carbonite/sdk_gpu_source_bin"
        local projectFiles = os.matchfiles(physxSDKSourceDir.."/*.vcxproj")
        for i, projectFile in ipairs(projectFiles) do
            k, l = string.find(projectFile, ".vcxproj")                
            local projectName = (string.sub(projectFile, string.len(physxSDKSourceDir) + 2, k - 1))
            if projectName ~= "INSTALL" and projectName ~= "ALL_BUILD" then
                externalproject (projectName)
                    location (physxSDKSourceDir)
                    local uid = os.uuid(projectFile)
                    uuid (uid)
                    kind "StaticLib"
                    language "C++"
                    systemversion (winsdk_version)
                    configmap {
                        ["debug"] = "debug",
                        ["release"] = physxLibs,
                    }
            end
        end

    group "PhysX SDK OmniPvd"
        local physxSDKSourceDir = physxSourceDir.."/compiler/vc16win64-carbonite/pvdruntime_bin"
        local projectFiles = os.matchfiles(physxSDKSourceDir.."/*.vcxproj")
        for i, projectFile in ipairs(projectFiles) do
            k, l = string.find(projectFile, ".vcxproj")                
            local projectName = (string.sub(projectFile, string.len(physxSDKSourceDir) + 2, k - 1))
            if projectName ~= "INSTALL" and projectName ~= "ALL_BUILD" then
                externalproject (projectName)
                    location (physxSDKSourceDir)
                    local uid = os.uuid(projectFile)
                    uuid (uid)
                    kind "SharedLib"
                    language "C++"
                    systemversion (winsdk_version)
                    configmap {
                        ["debug"] = "debug",
                        ["release"] = physxLibs,
                    }
            end
        end
end

function get_git_info(params, env_var)
    local val = os.getenv(env_var)
    if val ~= nil then
        return val
    end

    local str = nil
    local cmd = "git "..params

    local proc = io.popen(cmd)
    if proc then
        str = proc:read("*a")
        local success, what, code = proc:close()
        if success then
            str = string.explode(str, "\n")[1]
        else
            str = nil
        end
    end

    if str == nil then
        str = "MISSING "..env_var
    end
    return str
end

function generate_version_header()
    shortSha = get_git_info("rev-parse --short HEAD", "PHYSICS_BUILD_SHA")
    commitDate = get_git_info("show -s --format=\"%ad\" --date=format:\"%b-%d-%Y\"", "PHYSICS_BUILD_DATE")
    branch = get_git_info("rev-parse --abbrev-ref HEAD", "PHYSICS_BUILD_BRANCH")
    version = get_git_info("show HEAD:omni/VERSION", "PHYSICS_BUILD_VERSION")
    print("Generating version header file: "..branch.." "..shortSha.." "..version.." "..commitDate)

    os.mkdir("_build/generated/include/omni/physx")
    local new_text = "#pragma once\n#define PHYSICS_BUILD_SHA \""..shortSha.."\"\n#define PHYSICS_BUILD_DATE \""..commitDate.."\"\n#define PHYSICS_BUILD_BRANCH \""..branch.."\"\n#define PHYSICS_BUILD_VERSION \""..version.."\"\n"

    local file = io.open("_build/generated/include/omni/physx/Version.h", "r")
    local old_text = ""
    if file then
        old_text = file:read "*all"
        file:close()
    end

    -- if we overwrite Version.h with identical content, anything including that header will always get rebuilt
    -- let's try to avoid that
    if old_text and new_text == old_text then
        return
    end

    file = io.open("_build/generated/include/omni/physx/Version.h", "w")
    file:write(new_text)
    file:close(file)
end
