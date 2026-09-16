# SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Wrapper around configure_file that serializes concurrent writers with a file lock.
function(Configure_File_MT IN_TEMPLATE OUTPUT_FILENAME)

	file(LOCK ${OUTPUT_FILENAME}.lock
		GUARD FUNCTION
		RESULT_VARIABLE LOCK_RESULT
		TIMEOUT 30)

	if (NOT LOCK_RESULT EQUAL 0)
		message(WARNING "Failed to lock file ${OUTPUT_FILENAME} for output ERROR: ${LOCK_RESULT}")
		return()
	endif()

	configure_file("${IN_TEMPLATE}" "${OUTPUT_FILENAME}" @ONLY)

	file(LOCK ${OUTPUT_FILENAME}.lock RELEASE)

endfunction()
