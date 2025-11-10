#!/bin/bash
set -e
SCRIPT_DIR="$(dirname "${BASH_SOURCE}")"
args="$*"

# Schema gen
while [ $# -gt 0 ]
do
    if [[ "$1" == "--devschema" ]]
    then
        RUN_SCHEMA_GEN="1"
    fi
    shift
done

if [[ $RUN_SCHEMA_GEN  -eq  "1" ]]
then
   pushd $SCRIPT_DIR/schema
   "./repo.sh" build --fetch-only $args || exit $?
   "./repo.sh" usd || exit $?
   popd
fi

# Main build
source "$SCRIPT_DIR/repo.sh" build $args || exit $?
