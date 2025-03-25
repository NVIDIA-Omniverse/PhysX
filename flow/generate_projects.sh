#!/bin/bash
SCRIPT_DIR=$(dirname ${BASH_SOURCE})

$SCRIPT_DIR/external/premake/linux-x86_64/premake5 gmake2 --file="$SCRIPT_DIR/premake5.lua" || error_exit "Error while generating projects with premake"
