
# - Try to find ETISS
# Once done, this will define
#
#  ETISS_FOUND
#  ETISS_INCLUDE_DIRS
#  ETISS_LIBRARIES
#
# and the following imported targets:
#
#  ETISS::etiss - The etiss library


find_path(ETISS_INCLUDE_DIR
  NAMES etiss
  PATHS ${ETISS_PREFIX} ${ETISS_HOME}
  PATH_SUFFIXES include
)

find_library(ETISS_LIBRARY
  NAMES ETISS
  PATHS ${ETISS_PREFIX} ${ETISS_HOME}
  PATH_SUFFIXES lib
)

if(NOT ETISS_LIBRARY)
  message(FATAL_ERROR
    "ETISS not found. Please set ETISS_PREFIX to its install location"
  )
endif()

set(ETISS_FOUND TRUE)
set(ETISS_INCLUDE_DIRS ${ETISS_INCLUDE_DIR})
set(ETISS_LIBRARIES ${ETISS_LIBRARY})


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ETISS
  FOUND_VAR ETISS_FOUND
  REQUIRED_VARS ETISS_INCLUDE_DIRS ETISS_LIBRARIES
)

if (NOT TARGET ETISS::etiss)
  add_library(ETISS::etiss INTERFACE IMPORTED)
  set_target_properties(ETISS::etiss PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${ETISS_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${ETISS_LIBRARIES}"
  )
endif()
