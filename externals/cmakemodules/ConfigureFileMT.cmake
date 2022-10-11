# A simple wrapper around configure_file to try to make it multi-thread safe with a file lock.
FUNCTION(Configure_File_MT IN_TEMPLATE OUTPUT_FILENAME)

	FILE(LOCK ${OUTPUT_FILENAME}.lock
		GUARD FUNCTION 
		RESULT_VARIABLE LOCK_RESULT 
		TIMEOUT 30)

	IF (NOT LOCK_RESULT EQUAL 0)
		MESSAGE(WARNING "Failed to lock file ${OUTPUT_FILENAME} for output ERROR: ${LOCK_RESULT}")
		return()
	ENDIF()

	CONFIGURE_FILE("${IN_TEMPLATE}" "${OUTPUT_FILENAME}" @ONLY)

	FILE(LOCK ${OUTPUT_FILENAME}.lock RELEASE)

ENDFUNCTION()