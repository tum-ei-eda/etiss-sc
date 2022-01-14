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

####################################################################################################
# cmake project settings 
cmake_minimum_required(VERSION 3.15)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_BINARY_DIR})

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

project(etiss-sc VERSION 0.9)
####################################################################################################

####################################################################################################
# cmake global compiler settings 
option (FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." True)
if (${FORCE_COLORED_OUTPUT})
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
       add_compile_options (-fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
       add_compile_options (-fcolor-diagnostics)
    endif ()
endif ()
####################################################################################################

####################################################################################################
# cmake install settings 
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/installed" CACHE PATH 
        "default install path" FORCE)
endif()
set(CMAKE_INSTALL_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_DATADIR "${CMAKE_INSTALL_PREFIX}/share")
set(CMAKE_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_PREFIX}/include")
set(CMAKE_INSTALL_BINDIR "${CMAKE_INSTALL_PREFIX}/bin")
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

include(GNUInstallDirs)
####################################################################################################

####################################################################################################
# Add lib subdirecty, decides on standalone based on scc being present (etiss-sc is submodule), or
# not 
add_subdirectory(lib)
####################################################################################################

####################################################################################################
# cmake packages 
find_package(ETISS REQUIRED)
find_package(elfio REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS filesystem program_options
)
####################################################################################################

add_library(${PROJECT_NAME} SHARED
    src/tlm/base/vp.cpp
    src/tlm/base/soc.cpp

    src/tlm/generic/cpu.cpp
    src/tlm/generic/reset_gen.cpp
    src/tlm/generic/bus.cpp
    src/tlm/generic/mem.cpp

    src/utils/logging.cpp
    src/utils/config.cpp
    src/utils/plugins.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${SystemC_LIBRARIES}
    ETISS::etiss
    elfio::elfio
    scc
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    Boost::program_options
    Boost::filesystem
)

add_subdirectory(examples)

set_target_properties(scc PROPERTIES
	  PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/src/scc.h
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/static
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
