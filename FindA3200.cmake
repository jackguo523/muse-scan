# - Find A3200 CLibrary in Window system
# Author : Jiaming Guo. Jackguo523@gmail.com


FIND_LIBRARY( A3200_LIBRARY_PATH NAMES A3200C 
	PATHS $ENV{A3200_PATH}/Lib )

SET( A3200_INCLUDE_DIR $ENV{A3200_PATH}/Include)

FIND_PACKAGE_HANDLE_STANDARD_ARGS( A3200 DEFAULT_MSG A3200_LIBRARY_PATH A3200_INCLUDE_DIR )

if( A3200_FOUND )
    SET( A3200_INCLUDE_DIRS ${A3200_INCLUDE_DIR} )
    SET( A3200_LIBRARY ${A3200_LIBRARY_PATH})
endif()