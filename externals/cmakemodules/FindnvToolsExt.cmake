# - Try to find nvToolsExt
# Once done this will define
#  NVTOOLSEXT_FOUND - System has nvToolsExt
#  NVTOOLSEXT_INCLUDE_DIRS - The nvToolsExt include directories
#  NVTOOLSEXT_LIB - The lib needed to use nvToolsExt
#  NVTOOLSEXT_DLL - The dll needed to use nvToolsExt
#  NVTOOLSEXT_DEFINITIONS - Compiler switches required for using nvToolsExt

INCLUDE(FindPackageHandleStandardArgs)

#TODO: Proper version support
FIND_PATH(		NVTOOLSEXTSDK_PATH include/nvToolsExt.h
				# PATHS 
				# ${GW_DEPS_ROOT}/nvToolsExt/${nvToolsExt_FIND_VERSION}
				# ${GW_DEPS_ROOT}/Externals/nvToolsExt/1
				# ${GW_DEPS_ROOT}/sw/physx/externals/nvToolsExt/1
				)
		
SET(NVTOOLSEXT_INCLUDE_DIRS
	${NVTOOLSEXTSDK_PATH}/include
)

if (CMAKE_CL_64)
	SET(NVTOOLSEXT_ARCH_FOLDER "x64")
	SET(NVTOOLSEXT_ARCH_FILE "64")
else()
	SET(NVTOOLSEXT_ARCH_FOLDER "Win32")
	SET(NVTOOLSEXT_ARCH_FILE "32")
endif()	
				
IF(TARGET_BUILD_PLATFORM STREQUAL "windows")
	# NOTE: Doesn't make sense for all platforms - ARM
	if (CMAKE_CL_64)
		SET(NVTOOLSEXT_ARCH_FOLDER "x64")
		SET(NVTOOLSEXT_ARCH_FILE "64")
	else()
		SET(NVTOOLSEXT_ARCH_FOLDER "Win32")
		SET(NVTOOLSEXT_ARCH_FILE "32")
	endif()

	SET(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".dll")
			
					
	FIND_LIBRARY(	NVTOOLSEXT_LIB
					NAMES nvToolsExt${NVTOOLSEXT_ARCH_FILE}_1
					PATHS
					${NVTOOLSEXTSDK_PATH}/lib/${NVTOOLSEXT_ARCH_FOLDER}
					)
	
	find_library(	NVTOOLSEXT_DLL
					NAMES nvToolsExt${NVTOOLSEXT_ARCH_FILE}_1
					PATHS 
					${NVTOOLSEXTSDK_PATH}/bin/${NVTOOLSEXT_ARCH_FOLDER}
	)

	
	FIND_PACKAGE_HANDLE_STANDARD_ARGS(nvToolsExt DEFAULT_MSG NVTOOLSEXT_LIB NVTOOLSEXT_DLL NVTOOLSEXT_INCLUDE_DIRS)
ELSE()
	# Exclude the libraries for non-windows platforms
	FIND_PACKAGE_HANDLE_STANDARD_ARGS(nvToolsExt DEFAULT_MSG NVTOOLSEXT_INCLUDE_DIRS)
ENDIF()

mark_as_advanced(NVTOOLSEXT_INCLUDE_DIRS NVTOOLSEXT_DLL NVTOOLSEXT_LIB)