#
# Build emscripten/WASM Bindings
#

FIND_PACKAGE(Python3)
SET(PYTHON ${Python3_EXECUTABLE} CACHE STRING "Python path")
SET(EMSCRIPTEN_ROOT $ENV{EMSDK}/upstream/emscripten CACHE STRING "Emscripten path")
SET(CMAKE_TOOLCHAIN_FILE ${EMSCRIPTEN_ROOT}/cmake/Modules/Platform/Emscripten.cmake)
SET(WEBIDL_BINDER_SCRIPT ${EMSCRIPTEN_ROOT}/tools/webidl_binder.py)

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(PHYSX_WASM_SOURCE_DIR ${PHYSX_SOURCE_DIR}/webidlbindings/src)

SET(PHYSXWASM_INCLUDE_DIR ${PHYSX_ROOT_DIR}/include)
SET(PHYSXWASM_GLUE_WRAPPER ${PHYSX_WASM_SOURCE_DIR}/wasm/PhysXWasm.cpp)
SET(PHYSXWASM_IDL_FILE ${PHYSX_WASM_SOURCE_DIR}/wasm/PhysXWasm.idl)
SET(EMCC_WASM_ARGS
		--post-js glue.js
		--post-js ${PHYSX_WASM_SOURCE_DIR}/wasm/onload.js
		-sMODULARIZE=1
		-sEXPORT_ES6=1
		-sEXPORT_NAME=PhysX
		-sENVIRONMENT=web,worker
		-sNO_FILESYSTEM=1
		-sALLOW_TABLE_GROWTH=1
		-sALLOW_MEMORY_GROWTH=1
		-sTOTAL_MEMORY=268435456
		-fno-rtti
		-fno-exceptions
		${WASM_EXPORTED_FUNCTIONS}
		${PHYSX_WASM_PTHREAD}
		${PHYSX_WASM_THREAD_POOL_SZ}
)

SET(EMCC_GLUE_ARGS
		-c
		-DNDEBUG
		-O3
		-msimd128
		${PHYSX_WASM_PTHREAD}
		-I${PHYSXWASM_INCLUDE_DIR}
		# todo: maybe find a more elegant way to include generated glue.cpp
		-I${PHYSX_ROOT_DIR}/compiler/emscripten-release/sdk_source_bin
)

ADD_CUSTOM_COMMAND(
		OUTPUT glue.cpp glue.js
		BYPRODUCTS parser.out WebIDLGrammar.pkl
		COMMAND ${PYTHON} ${WEBIDL_BINDER_SCRIPT} ${PHYSXWASM_IDL_FILE} glue
		DEPENDS ${PHYSXWASM_IDL_FILE}
		COMMENT "Generating physx-js-webidl bindings"
		VERBATIM
)

ADD_CUSTOM_COMMAND(
		OUTPUT glue.o
		COMMAND em++ ${PHYSXWASM_GLUE_WRAPPER} ${EMCC_GLUE_ARGS} -o glue.o
		DEPENDS glue.cpp
		COMMENT "Building physx-js-webidl bindings"
		VERBATIM
)
ADD_CUSTOM_TARGET(physx-js-bindings ALL DEPENDS glue.js glue.o)

SET(PHYSX_TARGETS PhysX PhysXCharacterKinematic PhysXCommon PhysXCooking PhysXExtensions PhysXFoundation PhysXVehicle2 PhysXPvdSDK)
FOREACH(_TARGET ${PHYSX_TARGETS})
	LIST(APPEND PHYSX_LIBS $<TARGET_FILE:${_TARGET}>)
ENDFOREACH()

ADD_CUSTOM_COMMAND(
		OUTPUT physx-js-webidl.mjs physx-js-webidl.wasm
		COMMAND em++ glue.o ${PHYSX_LIBS} ${EMCC_WASM_ARGS} -o physx-js-webidl.mjs
		DEPENDS physx-js-bindings ${PHYSX_TARGETS}
		COMMENT "Building physx-js-webidl webassembly"
		VERBATIM
)
ADD_CUSTOM_TARGET(PhysXWasmBindings ALL DEPENDS physx-js-webidl.mjs physx-js-webidl.wasm)
