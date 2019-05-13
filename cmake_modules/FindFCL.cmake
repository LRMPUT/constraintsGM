# Need to find both fcl
FIND_PATH(FCL_INCLUDE_DIR fcl.h
    ~/Libs/fcl/build/include
    ~/Libs/fcl/build/include/fcl
#     /usr/include/fcl
#     /opt/local/include/fcl
#     /usr/local/include/fcl
  )

  FIND_PATH(FCL_INCLUDE_DIR2 fcl.h.in
    ~/Libs/fcl/include
    ~/Libs/fcl/include/fcl
#     /usr/include/fcl
#     /opt/local/include/fcl
#     /usr/local/include/fcl
  )
  
# find_library(FCL_LIBRARY
#   NAMES fcl libfcl libfcl.so
#   PATHS ~/Libs/fcl/build
#         ~/Libs/fcl/build/lib
# #         /usr/lib
# #         /usr/lib/x86_64-linux-gnu
# #         /usr/local/lib
# #         /opt/local/lib
# )

SET(FCL_LIBRARY "~/Libs/fcl/build/lib/libfcl.so")
SET(FCL_BINARY_DIR "~/Libs/fcl/build/lib")

IF(FCL_INCLUDE_DIR AND FCL_INCLUDE_DIR2 AND FCL_LIBRARY)
    string(REPLACE fcl "" FCL_INCLUDE_DIR ${FCL_INCLUDE_DIR})
    string(REPLACE // "/fcl/" FCL_INCLUDE_DIR ${FCL_INCLUDE_DIR})
    
    string(REPLACE fcl "" FCL_INCLUDE_DIR2 ${FCL_INCLUDE_DIR2})
    string(REPLACE // "/fcl/" FCL_INCLUDE_DIR2 ${FCL_INCLUDE_DIR2})
	SET(FCL_FOUND TRUE CACHE STRING "Whether FCL was found or not")
ENDIF(FCL_INCLUDE_DIR AND FCL_INCLUDE_DIR2 AND FCL_LIBRARY)

IF(FCL_FOUND)
	IF(NOT FCL_FIND_QUIETLY)
		MESSAGE(STATUS "Found FCL include 1: ${FCL_INCLUDE_DIR}")
		MESSAGE(STATUS "Found FCL include 2: ${FCL_INCLUDE_DIR2}")
		MESSAGE(STATUS "Found FCL lib: ${FCL_LIBRARY}")
	ENDIF (NOT FCL_FIND_QUIETLY)
ELSE(FCL_FOUND)
	IF(FCL_FIND_REQUIRED)
		MESSAGE(FATAL_ERROR "Could not find FCL")
	ENDIF(FCL_FIND_REQUIRED)
ENDIF(FCL_FOUND)
