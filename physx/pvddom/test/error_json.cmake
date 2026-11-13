## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

# CMake parses the value. This framing check additionally requires one flat object.
# This is specific to the load-error schema, not a general strict JSON parser.
function(flat_error_json output value)
    string(JSON type ERROR_VARIABLE parseError TYPE "${value}")
    string(REGEX REPLACE [=["([^"\\]|\\.)*"]=] "@" masked "${value}")
    if(NOT parseError AND type STREQUAL "OBJECT" AND
        masked MATCHES "^[ \t\r\n]*\\{[^{}]*\\}[ \t\r\n]*$")
        set(${output} TRUE PARENT_SCOPE)
    else()
        set(${output} FALSE PARENT_SCOPE)
    endif()
endfunction()
function(error_document value expectedCode expectedRole expectedOption)
    flat_error_json(framed "${value}")
    if(NOT framed)
        message(FATAL_ERROR "Expected one complete flat load-error object: ${value}")
    endif()
    string(JSON format GET "${value}" format)
    string(JSON version GET "${value}" version)
    string(JSON versionType TYPE "${value}" version)
    string(JSON code GET "${value}" code)
    string(JSON role GET "${value}" role)
    string(JSON option GET "${value}" limitOption)
    string(JSON optionType TYPE "${value}" limitOption)
    if(NOT format STREQUAL "physx.ovd.error" OR NOT version EQUAL 1 OR
        NOT versionType STREQUAL "NUMBER" OR NOT code STREQUAL expectedCode OR
        NOT role STREQUAL expectedRole OR
        (expectedOption STREQUAL "" AND NOT optionType STREQUAL "NULL") OR
        (NOT expectedOption STREQUAL "" AND (NOT optionType STREQUAL "STRING" OR NOT option STREQUAL expectedOption)))
        message(FATAL_ERROR "Incorrect load-error contract: ${value}")
    endif()
endfunction()
set(framingSample [=[{"message":"braces {} and escaped quote \" and slash \\"}]=])
foreach(suffix "" " \t\n" "{}" " text" "\"tail\"" "${framingSample}")
    flat_error_json(valid "${framingSample}${suffix}")
    if((suffix MATCHES "^[ \t\r\n]*$" AND NOT valid) OR
        (NOT suffix MATCHES "^[ \t\r\n]*$" AND valid))
        message(FATAL_ERROR "Load-error framing failed for suffix: ${suffix}")
    endif()
endforeach()
