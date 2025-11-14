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
add_subdirectory(fmt)

####################################################################################################
# SCC
set(SCC_LIB_ONLY ON)
option(ENABLE_SCV "Enable the use of SCV" OFF)
add_subdirectory(scc)

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
