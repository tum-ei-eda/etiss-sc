####################################################################################################
# Copyright [2019-2022] Chair of EDA, Technical University of Munich
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
####################################################################################################

cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
include(FetchContent)

if(NOT TARGET scc) # etiss-sc is standalone, i.e. not embedded alongside scc in a project
    FetchContent_Declare(
        scc
        GIT_REPOSITORY https://github.com/VP-Vibes/SystemC-Components.git
        GIT_TAG        6b6e810
    )
    FetchContent_GetProperties(scc)
    if(NOT scc_POPULATED)
        FetchContent_Populate(scc)
    endif()

    list(APPEND CMAKE_MODULE_PATH ${scc_SOURCE_DIR}/cmake PARENT_SCOPE)

    include(ConanInline)
    set(CONAN_OPTIONS
        boost:shared=True
        boost:header_only=False
        fmt:header_only=True
    )

    set(CONAN_PACKAGE_LIST
        boost/1.76.0
        elfio/3.8
        fmt/6.1.2
        zlib/1.2.11
    )
    set(CONAN_PACKAGE_OPTIONS ${CONAN_OPTIONS})

    if(NOT USE_CWR_SYSTEMC AND NOT USE_NCSC_SYSTEMC AND NOT DEFINED ENV{SYSTEMC_HOME})
        set(CONAN_PACKAGE_LIST ${CONAN_PACKAGE_LIST} systemc/2.3.3 systemc-cci/1.0.0)
        set(CONAN_PACKAGE_OPTIONS ${CONAN_PACKAGE_OPTIONS} systemc:shared=True systemc-cci:shared=True systemc:disable_virtual_bind=False)
    endif()

    conan_check()
    conan_cmake_configure(
        REQUIRES ${CONAN_PACKAGE_LIST}
        GENERATORS cmake_find_package
        OPTIONS ${CONAN_PACKAGE_OPTIONS}
    )

    conan_cmake_autodetect(settings)
    if(CMAKE_CXX_STANDARD)
        set(settings ${settings} compiler.cppstd=${CMAKE_CXX_STANDARD})
    endif()

    message("Conan settings are ${settings}")
    conan_cmake_install(
        PATH_OR_REFERENCE ${CMAKE_CURRENT_BINARY_DIR}
        BUILD missing
        SETTINGS ${settings}
    )
    set(Boost_NO_BOOST_CMAKE ON)

    set(SCC_LIB_ONLY ON)
    option(ENABLE_SCV "Enable the use of SCV" OFF)

    include(${scc_SOURCE_DIR}/cmake/SystemCPackage.cmake)

    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE)
    find_package(fmt REQUIRED)

    add_subdirectory(${scc_SOURCE_DIR} ${scc_BINARY_DIR})

endif()

###################################################################
# simpleini
if(NOT TARGET simpleini)
    FetchContent_Declare(
        simpleini
        GIT_REPOSITORY https://github.com/brofield/simpleini.git
        GIT_TAG        16b813a6af2456a027a1cd878c9f08e6809ad905
    )

    FetchContent_GetProperties(simpleini)
    if(NOT simpleini_POPULATED)
        FetchContent_Populate(simpleini)
    endif()

    add_library(simpleini INTERFACE)
    target_include_directories(simpleini INTERFACE
        ${simpleini_SOURCE_DIR}
    )
endif()

####################################################################################################
# ETISS (include before etiss-sc) Try to find from given prefix first
if(NOT TARGET ETISS)
    find_package(ETISS REQUIRED)
endif()
####################################################################################################
# Boost
if(NOT Boost_FOUND)
    find_package(Boost REQUIRED
        COMPONENTS filesystem program_options
    )
endif()

####################################################################################################
# ELFIO
if(NOT TARGET elfio)
    # elfio not already defined as target somewhere
    find_package(elfio REQUIRED)
endif()
if(NOT TARGET elfio::elfio)
    add_library(elfio::elfio ALIAS elfio)
else()
    # make sure (most likely imported) target is so globally for usage in parent CMakeLists.txt
    set_target_properties(elfio::elfio PROPERTIES
                          IMPORTED_GLOBAL TRUE
    )
endif()
