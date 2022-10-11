#!/bin/bash +x

export PHYSX_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export PM_CMakeModules_PATH="$PHYSX_ROOT_DIR/../externals/cmakemodules"
export PM_opengllinux_PATH="$PHYSX_ROOT_DIR/../externals/opengl-linux"
export PM_TARGA_PATH="$PHYSX_ROOT_DIR/../externals/targa"
export PM_CGLINUX_PATH="$PHYSX_ROOT_DIR/../externals/cg-linux"
export PM_GLEWLINUX_PATH="$PHYSX_ROOT_DIR/../externals/glew-linux"
export PM_WAVEFRONT_PATH="$PHYSX_ROOT_DIR/../externals/wavefront"
export PM_rapidjson_PATH="$PHYSX_ROOT_DIR/../externals/rapidjson"
export PM_PATHS="$PM_opengllinux_PATH;$PM_TARGA_PATH;$PM_CGLINUX_PATH;$PM_GLEWLINUX_PATH;$PM_WAVEFRONT_PATH;$PM_rapidjson_PATH"


cd "$( dirname "${BASH_SOURCE[0]}" )"
python3 ./buildtools/cmake_generate_projects.py $1
status=$?
if [ "$status" -ne "0" ]; then
echo "Error $status"
exit 1
fi