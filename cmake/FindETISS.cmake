####################################################################################################
# Copyright 2022 Chair of EDA, Technical University of Munich
#
# Licensed under the Apache License, Version 2.0 (the License);
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an AS IS BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
####################################################################################################

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

#find_library(ETISS_LIBRARY
#    NAMES ETISS
#    PATHS ${ETISS_PREFIX} ${ETISS_HOME}
#    PATH_SUFFIXES lib
#)

set(ETISS_DIR "${ETISS_PREFIX}/lib/CMake/ETISS")
set(ETISS_DISABLE_COMPILERFLAGS ON)
find_package(ETISS CONFIG REQUIRED)

find_library(ETISS_LIBRARY
    NAMES ETISS
    PATHS ${ETISS_LIB_DIR}
)

if(NOT ETISS_LIBRARY)
    message(WARNING
        "ETISS not found. Please set ETISS_PREFIX to its install location"
    )
else()
    get_filename_component(ETISS_LIB_PATH ${ETISS_LIBRARY} DIRECTORY)
    message(INFO
        "ETISS libraries found: ${ETISS_LIB_PATH}."
    )
    find_path(ETISS_INCLUDE_DIR
        NAMES etiss third_party
        PATHS ${ETISS_LIB_PATH}/..
        PATH_SUFFIXES include
    )
    message(INFO
        "ETISS include path: ${ETISS_INCLUDE_DIR}."
    )
    set(ETISS_FOUND TRUE)
    set(ETISS_INCLUDE_DIRS ${ETISS_INCLUDE_DIRS})
    set(ETISS_LIBRARIES ${ETISS_LIBRARY})

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(ETISS
        FOUND_VAR ETISS_FOUND
        REQUIRED_VARS ETISS_INCLUDE_DIRS ETISS_LIBRARIES
    )
    message("ETISS_INCLUDE_DIRS: ${ETISS_INCLUDE_DIRS}.")
    message("ETISS_LIBRARIES: ${ETISS_LIBRARIES}.")
    add_library(ETISS INTERFACE IMPORTED GLOBAL)
    set_target_properties(ETISS PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ETISS_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${ETISS_LIBRARIES}"
    )

    if (NOT TARGET ETISS::etiss)
        add_library(ETISS::etiss ALIAS ETISS)
    endif()
endif()
