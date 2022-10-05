/*
 * Copyright 2022 Chair of EDA, Technical University of Munich
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	 http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file cpu_base.cpp
/// @date 2022-05-30
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "etiss-sc/tlm/generic/cpu_base.h"

int etiss_sc::CPUBase::id = 0;

etiss_sc::CPUBase::CPUBase(sc_core::sc_module_name name, CPUParams &&cpu_params)
    : sc_core::sc_module(name), cpu_params_{ std::move(cpu_params) }, cpu_id_(CPUBase::id++)
{
}

size_t etiss_sc::CPUBase::getNumIRQs() const
{
    return cpu_params_.num_irqs_;
}
