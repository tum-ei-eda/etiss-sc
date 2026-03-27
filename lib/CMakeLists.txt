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

####################################################################################################
# FMT before SCC
set(FMT_INSTALL ON CACHE BOOL "Generate install/export targets for the bundled fmt build." FORCE)
add_subdirectory(fmt)
set(ETISS_SC_FMT_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/fmt-package")
file(MAKE_DIRECTORY "${ETISS_SC_FMT_CONFIG_DIR}")
file(WRITE "${ETISS_SC_FMT_CONFIG_DIR}/fmtConfig.cmake" [=[
if(NOT TARGET fmt::fmt AND NOT TARGET fmt::fmt-header-only)
  message(FATAL_ERROR "The bundled fmt targets are unavailable.")
endif()
set(fmt_FOUND TRUE)
]=])
file(WRITE "${ETISS_SC_FMT_CONFIG_DIR}/fmt-config.cmake" [=[
include("${CMAKE_CURRENT_LIST_DIR}/fmtConfig.cmake")
]=])
set(fmt_DIR "${ETISS_SC_FMT_CONFIG_DIR}" CACHE PATH "Path to the bundled fmt CMake package." FORCE)

####################################################################################################
# SPDLOG before SCC
set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "Use the bundled fmt package from etiss-sc/lib." FORCE)
set(SPDLOG_INSTALL ON CACHE BOOL "Generate install/export targets for the bundled spdlog build." FORCE)
add_subdirectory(spdlog)
set(ETISS_SC_SPDLOG_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/spdlog-package")
file(MAKE_DIRECTORY "${ETISS_SC_SPDLOG_CONFIG_DIR}")
file(WRITE "${ETISS_SC_SPDLOG_CONFIG_DIR}/spdlogConfig.cmake" [=[
if(NOT TARGET spdlog::spdlog AND NOT TARGET spdlog::spdlog_header_only)
  message(FATAL_ERROR "The bundled spdlog targets are unavailable.")
endif()
set(spdlog_FOUND TRUE)
]=])
file(WRITE "${ETISS_SC_SPDLOG_CONFIG_DIR}/spdlog-config.cmake" [=[
include("${CMAKE_CURRENT_LIST_DIR}/spdlogConfig.cmake")
]=])
set(spdlog_DIR "${ETISS_SC_SPDLOG_CONFIG_DIR}" CACHE PATH "Path to the bundled spdlog CMake package." FORCE)

####################################################################################################
# YAML-CPP before SCC
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Disable yaml-cpp tools in the bundled etiss-sc build." FORCE)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Disable yaml-cpp tests in the bundled etiss-sc build." FORCE)
set(YAML_CPP_INSTALL ON CACHE BOOL "Generate install/export targets for the bundled yaml-cpp build." FORCE)
add_subdirectory(yaml-cpp)
set(ETISS_SC_YAML_CPP_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/yaml-cpp-package")
file(MAKE_DIRECTORY "${ETISS_SC_YAML_CPP_CONFIG_DIR}")
file(WRITE "${ETISS_SC_YAML_CPP_CONFIG_DIR}/yaml-cppConfig.cmake" [=[
if(NOT TARGET yaml-cpp::yaml-cpp AND NOT TARGET yaml-cpp)
  message(FATAL_ERROR "The bundled yaml-cpp targets are unavailable.")
endif()
set(yaml-cpp_FOUND TRUE)
set(YAML_CPP_FOUND TRUE)
if(NOT TARGET yaml-cpp AND TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp INTERFACE IMPORTED)
  target_link_libraries(yaml-cpp INTERFACE yaml-cpp::yaml-cpp)
endif()
]=])
file(WRITE "${ETISS_SC_YAML_CPP_CONFIG_DIR}/yaml-cpp-config.cmake" [=[
include("${CMAKE_CURRENT_LIST_DIR}/yaml-cppConfig.cmake")
]=])
set(yaml-cpp_DIR "${ETISS_SC_YAML_CPP_CONFIG_DIR}" CACHE PATH "Path to the bundled yaml-cpp CMake package." FORCE)

####################################################################################################
# SCC
set(SCC_LIB_ONLY ON)
option(ENABLE_SCV "Enable the use of SCV" OFF)
add_subdirectory(scc)
# bundled SCC assembles `scc-shared` from object files, so these
# component targets need PIC even when SCC is embedded via etiss-sc.
set_property(TARGET scc-util scc-sysc interfaces scv-tr tlm-interfaces PROPERTY POSITION_INDEPENDENT_CODE ON)

####################################################################################################
# ETISS (include before etiss-sc) Try to find from given prefix first
if(NOT TARGET ETISS)
    find_package(ETISS REQUIRED)
else()
    if(NOT TARGET ETISS::ETISS)
        add_library(ETISS::etiss ALIAS ETISS)
    endif()
endif()

####################################################################################################
# ELFIO
if(NOT TARGET elfio::elfio AND NOT ETISS_AS_SUBMODULE)
    add_subdirectory(elfio)
endif()
