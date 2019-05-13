# Need to find both ccd
FIND_PATH(CCD_INCLUDE_DIR ccd.h
    ~/Libs/libccd/src/ccd
#     /usr/include/ccd
#     /opt/local/include/ccd
#     /usr/local/include/ccd
  )

# find_library(CCD_LIBRARY
#   NAMES ccd libccd libccd.so
#   PATHS ~/Libs/libccd/build
#         ~/Libs/libccd/build/src
# #         /usr/lib
# #         /usr/lib/x86_64-linux-gnu
# #         /usr/local/lib
# #         /opt/local/lib
# )

SET(CCD_LIBRARY "~/Libs/libccd/build/src/libccd.so")
SET(CCD_BINARY_DIR "~/Libs/libccd/build/src/")

IF(CCD_INCLUDE_DIR AND CCD_LIBRARY)
	SET(CCD_FOUND TRUE CACHE STRING "Whether CCD was found or not")
ENDIF(CCD_INCLUDE_DIR AND CCD_LIBRARY)

IF(CCD_FOUND)
	IF(NOT CCD_FIND_QUIETLY)
		MESSAGE(STATUS "Found CCD: ${CCD_LIBRARY}")
	ENDIF (NOT CCD_FIND_QUIETLY)
ELSE(CCD_FOUND)
	IF(CCD_FIND_REQUIRED)
		MESSAGE(FATAL_ERROR "Could not find CCD")
	ENDIF(CCD_FIND_REQUIRED)
ENDIF(CCD_FOUND)
