newoption {
    trigger     = "platform-host",
    description = "(Optional) Specify host platform for cross-compilation"
}

-- Include omni.repo.build premake tools
local repo_build = require('omni/repo/build')

-- Path defines
local target_deps = "_build/target-deps"

-- Enable /sourcelink flag for VS
repo_build.enable_vstudio_sourcelink()

-- Remove /JMC parameter for visual studio
repo_build.remove_vstudio_jmc()

-- Wrapper funcion around path.getabsolute() which makes drive letter lowercase on windows.
-- Otherwise drive letter can alter the case depending on environment and cause solution to reload.
function get_abs_path(p)
    p = path.getabsolute(p)
    if os.target() == "windows" then
        p = p:gsub("^%a:", function(c) return c:lower() end)
    end
    return p
end

function copy_to_file(filePath, newPath)
    local filePathAbs = get_abs_path(filePath)
    local dir = newPath:match("(.*[\\/])")
    if os.target() == "windows" then
        if dir ~= "" then
            --dir = dir:gsub('/', '\\')
            postbuildcommands { "{MKDIR} \""..dir.."\"" }
        end
        -- Using {COPY} on Windows adds an IF EXIST with an extra backslash which doesn't work
        filePathAbs = filePathAbs:gsub('/', '\\')
        newPath = newPath:gsub('/', '\\')
        postbuildcommands { "copy /Y \""..filePathAbs.."\" \""..newPath.."\"" }
    else
        if dir ~= "" then
            postbuildcommands { "$(SILENT) {MKDIR} "..dir }
        end
        postbuildcommands { "$(SILENT) {COPY} "..filePathAbs.." "..newPath }
    end
end

function os.capture(cmd, raw)
  local f = assert(io.popen(cmd, 'r'))
  local s = assert(f:read('*a'))
  f:close()
  if raw then return s end
  s = string.gsub(s, '^%s+', '')
  s = string.gsub(s, '%s+$', '')
  s = string.gsub(s, '[\n\r]+', ' ')
  return s
end

premake.override(premake.vstudio.vc2010, "projectReferences", function(base, prj)
   local refs = premake.project.getdependencies(prj, 'linkOnly')
   if #refs > 0 then
      premake.push('<ItemGroup>')
      for _, ref in ipairs(refs) do
         local relpath = premake.vstudio.path(prj, premake.vstudio.projectfile(ref))
         premake.push('<ProjectReference Include=\"%s\">', relpath)
         premake.callArray(premake.vstudio.vc2010.elements.projectReferences, prj, ref)
         premake.vstudio.vc2010.element("UseLibraryDependencyInputs", nil, "true")
         premake.pop('</ProjectReference>')
      end
      premake.pop('</ItemGroup>')
   end
end)

local hostDepsDir = "_build/host-deps"
local targetDepsDir = "_build/target-deps"
local capnp_gen_path = "source/sdk/extensions/serialization/generated"

local workspace_name = "blast-sdk"

local root = repo_build.get_abs_path(".")

-- Copy headers and licenses
repo_build.prebuild_copy {
    { "include", "_build/%{platform}/%{config}/"..workspace_name.."/include" },
    { "source/sdk/common", "_build/%{platform}/%{config}/"..workspace_name.."/source/sdk/common" },
    { "source/shared/NsFoundation", "_build/%{platform}/%{config}/"..workspace_name.."/source/shared/NsFoundation" },
    { "PACKAGE-LICENSES", "_build/%{platform}/%{config}/"..workspace_name.."/PACKAGE-LICENSES" }
}


-- Preprocess to generate Cap'n Proto files
function capn_proto_precompile_step(dirpath, capnp_files)
    local capnp_src = get_abs_path("_build/host-deps/CapnProto/src")
    local abs_dir_path = get_abs_path(dirpath)

    if os.target() == "windows" then
        local capnp_bin = get_abs_path("_build/host-deps/CapnProto/tools/win32"):gsub('/', '\\')
        local abs_capnp_gen_path = get_abs_path(capnp_gen_path):gsub('/', '\\')
        prebuildcommands { "if not exist "..abs_capnp_gen_path.."\\ mkdir "..abs_capnp_gen_path } -- make the generated source folder
        -- capnp compile
        for _, filename in pairs(capnp_files) do
            prebuildcommands { "if exist "..abs_capnp_gen_path.."\\"..filename..".cpp del /Q "..abs_capnp_gen_path.."\\"..filename..".cpp" }
            local command = capnp_bin.."\\capnp.exe compile -o "..capnp_bin.."\\capnpc-c++.exe:"..abs_capnp_gen_path.." -I "..capnp_src.." --src-prefix "..abs_dir_path.." "..abs_dir_path.."/"..filename
            prebuildcommands { command }
            -- prebuildcommands { "ren "..abs_capnp_gen_path.."\\"..filename..".c++ "..filename..".cpp" }
        end
    elseif os.target() == "linux" then
        local capnp_bin = get_abs_path("_build/host-deps/CapnProto/tools/ubuntu64")
        local abs_capnp_gen_path = get_abs_path(capnp_gen_path)
        prebuildcommands { "mkdir -p "..abs_capnp_gen_path } -- make the generated source folder
        -- capnp compile
        for _, filename in pairs(capnp_files) do
            local command = capnp_bin.."/capnp compile -o "..capnp_bin.."/capnpc-c++:"..abs_capnp_gen_path.." -I "..capnp_src.." --src-prefix "..abs_dir_path.." "..abs_dir_path.."/"..filename
            prebuildcommands { command }
            -- prebuildcommands { "mv "..abs_capnp_gen_path.."/"..filename..".c++ "..abs_capnp_gen_path.."/"..filename..".cpp" }
        end
    end
end

capn_proto_precompile_step("source/sdk/extensions/serialization", {
    "NvBlastExtLlSerialization-capn",
    "NvBlastExtTkSerialization-capn",
})


-- Custom rule for .c++ files
if os.target() == "linux" then
    rule "c++"
        fileExtension { ".c++" }
        buildoutputs  { "$(OBJDIR)/%{file.objname}.o" }
        buildmessage  '$(notdir $<)'
        buildcommands {'$(CXX) %{premake.modules.gmake2.cpp.fileFlags(cfg, file)} $(FORCE_INCLUDE) -o "$@" -MF "$(@:%.o=%.d)" -c "$<"'}
end

-- premake5.lua
workspace (workspace_name)
    configurations { "debug", "release" }
    startproject "NvBlast"
    local targetName = _ACTION
    local workspaceDir = "_compiler/"..targetName
    -- common dir name to store platform specific files
    local platform = "%{cfg.system}-%{cfg.platform}"
    local targetDependencyPlatform = "%{cfg.system}-%{cfg.platform}";
    local hostDependencyPlatform = _OPTIONS["platform-host"] or targetDependencyPlatform;
    local sdkTargetDir = "_build/"..platform.."/%{cfg.buildcfg}/%{wks.name}"
    local targetDir = sdkTargetDir.."/bin"
    -- defining anything related to the VS or SDK version here because they will most likely be changed in the future..
    local msvcInclude = hostDepsDir.."/msvc/VC/Tools/MSVC/14.16.27023/include"
    local msvcLibs = hostDepsDir.."/msvc/VC/Tools/MSVC/14.16.27023/lib/onecore/x64"
    local sdkInclude = { hostDepsDir.."/winsdk/include/winrt", hostDepsDir.."/winsdk/include/um", hostDepsDir.."/winsdk/include/ucrt", hostDepsDir.."/winsdk/include/shared" }
    local sdkLibs = { hostDepsDir.."/winsdk/lib/ucrt/x64", hostDepsDir.."/winsdk/lib/um/x64" }

    location (workspaceDir)
    targetdir (targetDir)
    -- symbolspath ("_build/"..targetName.."/symbols/%{cfg_buildcfg}/%{prj.name}.pdb")
    objdir ("_build/tmp/%{cfg.system}/%{prj.name}")
    exceptionhandling "On"
    rtti "Off"
    staticruntime "Off"
    flags { "FatalCompileWarnings", "MultiProcessorCompile", "NoPCH", "UndefinedIdentifiers", "NoIncrementalLink" }
    cppdialect "C++14"
    includedirs { "include" }

    characterset( "ASCII" )

    defines { "LOG_COMPONENT=\"%{prj.name}\"" }

    sysincludedirs { targetDepsDir }

    filter { "system:windows" }
        platforms { "x86_64" }
        symbols "Full"
        -- add .editorconfig to all projects so that VS 2017 automatically picks it up
        files {".editorconfig"}
        editandcontinue "Off"
        bindirs { hostDepsDir.."/msvc/VC/Tools/MSVC/14.16.27023/bin/HostX64/x64", hostDepsDir.."/msvc/MSBuild/15.0/bin", hostDepsDir.."/winsdk/bin/x64" }
        systemversion "10.0.17763.0"
        -- this is for the include and libs from the SDK.
        syslibdirs { msvcLibs, sdkLibs }
        sysincludedirs { msvcInclude, sdkInclude }
        -- all of our source strings and executable strings are utf8
        buildoptions {"/utf-8", "/bigobj"}
        buildoptions {"/permissive-"}
        buildoptions { "/WX" } -- warnings as errors
        warnings "Extra"
        defines { "_CRT_NONSTDC_NO_DEPRECATE" }
        defines { "BOOST_USE_WINDOWS_H=1" }

    filter { "system:linux" }
        platforms { "x86_64", "aarch64" }
        defaultplatform "x86_64"
    filter { "system:linux", "platforms:x86_64" }
        defines { "_GLIBCXX_USE_CXX11_ABI=0" }
        architecture "x86_64"
    filter { "system:linux", "platforms:aarch64" }
        defines { "_GLIBCXX_USE_CXX11_ABI=1" }
        architecture "ARM"
        local hostDependencyPlatform = _OPTIONS["platform-host"]
        print(hostDependencyPlatform)
        -- If cross-compiling, set the toolset explicitly
        if (localDependencyPlatform and localDependencyPlatform == "linux-x86_64") then
            local toolchain_path = "_build/host-deps/gcc-x86_64"
            local toolchain = dofile(toolchain_path .. "/toolchain.lua")
            use_gcc_local_toolchain(toolchain_path)
            toolset("gcc-local_9_2_0_arch64")
        end
    filter { "system:linux" }
        -- defines { "__STDC_FORMAT_MACROS" }
        symbols "On"

        buildoptions { "-pthread -fvisibility=hidden -fnon-call-exceptions -D_FILE_OFFSET_BITS=64 -fabi-version=8" }
        -- enforces RPATH instead of RUNPATH, needed for ubuntu version > 16.04
        linkoptions { "-pthread", 
                      "-Wl,--no-undefined",
                      "-Wl,--disable-new-dtags",
                      --"-Wl,--version-script=../../../omniclient.so.ld",
                      "-Wl,-rpath,'$$ORIGIN' -Wl,--export-dynamic" }
        -- add library origin directory to dlopen() search path
        enablewarnings { "all", "vla" }
        disablewarnings {
            "unused-variable",
            "switch",
            "unused-but-set-variable",
            "unused-result",
            "deprecated",
            "deprecated-declarations",
            "unknown-pragmas",
            "multichar",
            "parentheses",
            "nonnull-compare"
        }
        links { "stdc++fs" }
        if repo_build.ccache_path() then
            gccprefix (repo_build.ccache_path().." ")
        end
    filter {}

    filter { "system:linux", "configurations:debug" }
        buildoptions { "-ggdb", "-g3" }
    filter { "system:linux", "configurations:release" }
        buildoptions { "-ggdb", "-g2" }

    filter { "configurations:debug" }
        optimize "Off"
        defines { "_DEBUG", "CARB_DEBUG=1" }
    filter  { "configurations:release" }
        defines { "NDEBUG", "CARB_DEBUG=0" }
    filter  { "configurations:release", "system:windows" }
        optimize "Speed"
    -- Linux/GCC has some issues on thread exit when the "Speed" optimizations are enabled.
    -- We'll leave those off on Linux for the moment.
    filter { "configurations:release", "system:linux" }
        optimize "On"
    filter {}

function blast_sdklib_bare_setup(name)
    kind "SharedLib"
    location (workspaceDir.."/%{prj.name}")

    filter { "system:windows" }
        -- defines { "ISOLATION_AWARE_ENABLED=1" }
    filter { "system:linux" }
        buildoptions { "-fPIC" }
        links { "rt" }
    filter{}

    includedirs {
        "source/sdk/common",
        "include/"..name,
        "source/sdk/"..name,
    }
end

function blast_sdklib_common_files()
    files {
        "source/sdk/common/*.cpp",
    }

    vpaths {
        ["common/*"] = "source/sdk/common",
    }
end

function blast_sdklib_standard_setup(name)
    blast_sdklib_bare_setup(name)
    blast_sdklib_common_files()

    files {
        "source/sdk/"..name.."/*.cpp",
    }

    vpaths {
        ["include/*"] = "include/"..name,
        ["source/*"] = "source/sdk/"..name.."/",
    }
end

function link_dependents(names)
    libdirs { targetDir }
    for _, name in pairs(names) do
        dependson {name}
        links(name)
    end
end

function add_files(rootpath, filenames)
    for _, filename in pairs(filenames) do
        local file = rootpath.."/"..filename
        files { file }
    end
end

function add_capn_proto_source()
    add_files("_build/host-deps/CapnProto/src/capnp",
        {
            "arena.c++",
            "blob.c++",
            "layout.c++",
            "message.c++",
            "serialize.c++",
        }
    )
    add_files("_build/host-deps/CapnProto/src/kj",
        {
            "array.c++",
            "common.c++",
            "debug.c++",
            "exception.c++",
            "io.c++",
            "mutex.c++",
            "string.c++",
            "units.c++",
        }
    )

    -- cap'n proto source produces a lot of warnings
    filter { "system:windows" }
        disablewarnings {
            "4018", -- 'token' : signed/unsigned mismatch
            "4100", -- unreferenced formal parameter
            "4189", -- 'identifier' : local variable is initialized but not referenced
            "4244", -- conversion from 'type1' to 'type2', possible loss of data
            "4245", -- conversion from 'type1' to 'type2', signed/unsigned mismatch
            "4267", -- conversion from 'size_t' to 'type', possible loss of data
            "4456", -- declaration of 'identifier' hides previous local declaration
            "4541", -- 'identifier' used on polymorphic type 'type' with /GR-; unpredictable behavior may result
            "4702", -- unreachable code
            "4714", -- function 'function' marked as __forceinline not inlined
        }
    filter { "system:linux"}
        disablewarnings {
            "undef",
            "sign-compare"
        }
    filter {}
end

group "sdk"
    project "NvBlast"
        blast_sdklib_standard_setup("lowlevel")
        includedirs {
            "include/shared/NvFoundation",
        }

    project "NvBlastGlobals"
        blast_sdklib_standard_setup("globals")
        includedirs {
            "include/lowlevel",
            "source/shared/NsFoundation/include",
            "include/shared/NvFoundation",
        }

    project "NvBlastExtShaders"
        link_dependents({"NvBlast", "NvBlastGlobals"})
        blast_sdklib_standard_setup("extensions/shaders")
        includedirs {
            "include/lowlevel",
            "include/globals",
            "include/shared/NvFoundation",
            "source/shared/NsFoundation/include",
        }
        filter { "system:windows" }
            disablewarnings {
                "4267", -- conversion from 'size_t' to 'type', possible loss of data
            }
        filter { "system:linux"}
            disablewarnings {
                "strict-aliasing"
            }
        filter {}

    project "NvBlastExtAssetUtils"
        link_dependents({"NvBlast", "NvBlastGlobals"})
        blast_sdklib_standard_setup("extensions/assetutils")
        includedirs {
            "include/lowlevel",
            "include/globals",
            "include/shared/NvFoundation",
        }

    project "NvBlastExtAuthoring"
        link_dependents({"NvBlast", "NvBlastGlobals"})
        blast_sdklib_standard_setup("extensions/authoring")
        includedirs {
            "include/lowlevel",
            "include/globals",
            "include/extensions/assetutils",
            "include/extensions/authoringCommon",
            "source/sdk/extensions/authoring",
            "source/sdk/extensions/authoringCommon",
            "source/sdk/extensions/authoring/VHACD/inc",
            "source/sdk/extensions/authoring/VHACD/public",
            "include/shared/NvFoundation",
            "source/shared/NsFoundation/include",
            target_deps.."/BoostMultiprecision",
        }
        files {
            "source/sdk/extensions/authoringCommon/*.cpp",
            "source/sdk/extensions/authoring/VHACD/src/*.cpp",
        }
        vpaths {
            ["VHACD/*"] = "source/sdk/extensions/authoring/VHACD/",
            ["authoringCommon/include/*"] = "include/extensions/authoringCommon/",
            ["authoringCommon/source/*"] = "source/sdk/extensions/authoringCommon/",
        }
        filter { "system:windows" }
            disablewarnings {
                "4244", -- conversion from 'type1' to 'type2', possible loss of data
                "4267", -- conversion from 'size_t' to 'type', possible loss of data
            }
        filter { "system:linux"}
            disablewarnings {
                "misleading-indentation",
                "undef",
                "strict-aliasing",
                "maybe-uninitialized"
            }
        filter {}


    project "NvBlastTk"
        link_dependents({"NvBlast", "NvBlastGlobals"})
        blast_sdklib_standard_setup("toolkit")
        includedirs {
            "include/lowlevel",
            "include/globals",
            "source/sdk/globals",
            "include/shared/NvFoundation",
            "source/shared/NsFoundation/include",
            "source/shared/NsFileBuffer/include",
            "source/shared/NvTask/include",
        }

    project "NvBlastExtStress"
        filter { "system:linux"}
            buildoptions { "-march=haswell" }
        filter {}
        link_dependents({"NvBlast", "NvBlastGlobals"})
        blast_sdklib_standard_setup("extensions/stress")
        includedirs {
            "include/lowlevel",
            "include/globals",
            "include/shared/NvFoundation",
            "source/shared/NsFoundation/include",
            "source/shared/stress_solver",
        }
        files {
            "source/shared/stress_solver/stress.cpp",
        }
        filter { "system:windows" }
            disablewarnings {
                "4324", -- structure was padded due to alignment specifier
                "4505", -- unreferenced local function has been removed
            }
        filter { "system:linux"}
            disablewarnings {
                "maybe-uninitialized",
                "padded",
                "ignored-attributes",
                "unused-function"
                }
        filter {}

    project "NvBlastExtSerialization"
        filter { "system:linux"}
            rules { "c++" }
        filter {}
        link_dependents({"NvBlast", "NvBlastGlobals"})
        blast_sdklib_bare_setup("extensions/serialization")
        defines { "KJ_HEADER_WARNINGS=0"}
        includedirs {
            "source/sdk/extensions/serialization/DTO",
            "include/lowlevel",
            "source/sdk/lowlevel",
            "include/globals",
            "_build/host-deps/CapnProto/src",
            capnp_gen_path,
            "include/shared/NvFoundation",
            "source/shared/NsFoundation/include",
            "source/shared/NsFileBuffer/include",
            "source/shared/NvTask/include",
        }
        blast_sdklib_common_files()
        add_files("source/sdk/extensions/serialization",
            {
                "NvBlastExtSerialization.cpp",
                "NvBlastExtLlSerialization.cpp",
                "NvBlastExtOutputStream.cpp",
                "NvBlastExtInputStream.cpp",
            }
        )
        add_files("source/sdk/extensions/serialization/DTO",
            {
                "ActorDTO.cpp",
                "AssetDTO.cpp",
                "FamilyDTO.cpp",
                "FamilyGraphDTO.cpp",
                "NvBlastChunkDTO.cpp",
                "NvBlastBondDTO.cpp",
                "NvBlastIDDTO.cpp",
            }
        )
        add_capn_proto_source()
        add_files(capnp_gen_path,
            { "NvBlastExtLlSerialization-capn.c++" }
        )
        vpaths {
            ["include/*"] = "include/extensions/serialization/",
            ["source/*"] = "source/sdk/extensions/serialization/",
        }
        filter {}

    project "NvBlastExtTkSerialization"
        filter { "system:linux"}
            rules { "c++" }
        filter {}
        dependson {"NvBlastExtSerialization"}
        link_dependents({"NvBlast", "NvBlastGlobals", "NvBlastTk"})
        blast_sdklib_bare_setup("extensions/serialization")
        defines { "KJ_HEADER_WARNINGS=0"}
        includedirs {
            "source/sdk/extensions/serialization/DTO",
            "include/lowlevel",
            "include/toolkit",
            "source/sdk/lowlevel",
            "include/globals",
            "_build/host-deps/CapnProto/src",
            capnp_gen_path,
            "include/shared/NvFoundation",
            "source/shared/NsFoundation/include",
            "source/shared/NsFileBuffer/include",
            "source/shared/NvTask/include",
        }
        blast_sdklib_common_files()
        add_files("source/sdk/extensions/serialization",
            {
                "NvBlastExtTkSerialization.cpp",
                "NvBlastExtTkSerializerRAW.cpp",
                "NvBlastExtOutputStream.cpp",
                "NvBlastExtInputStream.cpp",
            }
        )
        add_files("source/sdk/extensions/serialization/DTO",
            {
                "AssetDTO.cpp",
                "TkAssetDTO.cpp",
                "NvVec3DTO.cpp",
                "NvBlastChunkDTO.cpp",
                "NvBlastBondDTO.cpp",
                "NvBlastIDDTO.cpp",
                "TkAssetJointDescDTO.cpp",
            }
        )
        add_capn_proto_source()
        add_files(capnp_gen_path,
            {
                "NvBlastExtLlSerialization-capn.c++",
                "NvBlastExtTkSerialization-capn.c++",
            }
        )
        vpaths {
            ["include/*"] = "include/extensions/serialization/",
            ["source/*"] = "source/sdk/extensions/serialization/",
        }


group "tests"
    project "UnitTests"
        kind "ConsoleApp"
        location (workspaceDir.."/%{prj.name}")
        link_dependents({"NvBlast", "NvBlastGlobals", "NvBlastExtAssetUtils", "NvBlastExtShaders", "NvBlastTk", "NvBlastExtSerialization", "NvBlastExtTkSerialization"})

        filter { "system:windows" }
            -- defines { "ISOLATION_AWARE_ENABLED=1" }
        filter { "system:linux" }
            buildoptions { "-fPIC" }
            links { "rt" }
        filter{}

        blast_sdklib_common_files()

        add_files("source/test/src/unit", {
            "AssetTests.cpp",
            "ActorTests.cpp",
            "APITests.cpp",
            "CoreTests.cpp",
            "FamilyGraphTests.cpp",
            "MultithreadingTests.cpp",
            "TkCompositeTests.cpp",
            "TkTests.cpp",
        })

        add_files("source/test/src/utils", {
            "TestAssets.cpp",
        })

        add_files("source/sdk/lowlevel", {
            "NvBlastActor.cpp",
            "NvBlastFamilyGraph.cpp",
            "NvBlastActorSerializationBlock.cpp",
            "NvBlastAsset.cpp",
            "NvBlastFamily.cpp",
        })

        add_files("source/sdk/toolkit", {
            "NvBlastTkTaskManager.cpp",
        })

        add_files("source/shared/utils", {
            "AssetGenerator.cpp",
        })

        includedirs {
            "include/globals",
            "include/lowlevel",
            "include/toolkit",
            "include/extensions/assetutils",
            "include/extensions/shaders",
            "include/extensions/serialization",
            "source/sdk/common",
            "source/sdk/globals",
            "source/sdk/lowlevel",
            "source/sdk/extensions/serialization",
            "source/test/src",
            "source/test/src/unit",
            "source/test/src/utils",
            "source/shared/filebuf/include",
            "source/shared/utils",
            "include/shared/NvFoundation",
            "source/shared/NsFoundation/include",
            "source/shared/NsFileBuffer/include",
            "source/shared/NvTask/include",
            target_deps.."/googletest/include",
        }

    filter { "system:windows", "configurations:debug" }
        libdirs { target_deps.."/googletest/lib/vc14win64-cmake/Debug" }
    filter { "system:windows", "configurations:release" }
        libdirs { target_deps.."/googletest/lib/vc14win64-cmake/Release" }
    filter { "system:linux" }
        libdirs { target_deps.."/googletest/lib/gcc-4.8" }
    filter{}

    links { "gtest_main", "gtest" }

    filter { "system:windows" }
        -- links { "PhysXFoundation_64", "PhysXTask_static_64" }
        disablewarnings {
            "4002", -- too many actual parameters for macro 'identifier'
            "4100", -- unreferenced formal parameter
            "4127", -- conditional expression is constant
            "4189", -- 'identifier' : local variable is initialized but not referenced
            "4244", -- conversion from 'type1' to 'type2', possible loss of data
            "4456", -- declaration of 'identifier' hides previous local declaration
            "4996", -- code uses a function, class member, variable, or typedef that's marked deprecated
        }
    filter { "system:linux"}
        -- links { "PhysXFoundation_static_64" }
        disablewarnings {
            "undef",
            "sign-compare"
        }
    filter {}