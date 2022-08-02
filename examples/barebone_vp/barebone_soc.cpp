/*
 * Copyright 2021 Chair of EDA, Technical University of Munich
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
/// @file barebone_soc.cpp
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "barebone_soc.h"

#define ID_ETISSVP_BAREBONESOC "etiss-sc: BareboneSoC"

BareboneSoC::BareboneSoC(sc_core::sc_module_name name, BareboneSoCParams &&barebone_soc_params,
                         etiss_sc::SoCParams &&soc_params)
    : etiss_sc::SoC(name, std::move(soc_params)), barebone_soc_params_{ std::move(barebone_soc_params) }
{
}

void BareboneSoC::setup()
{
    auto cpu_name = soc_params_.cpu_factory_.get()->get_name();
    auto bus_name = soc_params_.bus_params_.name_;
    auto mem_base_addr = barebone_soc_params_.mem_params_.base_addr_;

    etiss_sc::SoC::setup();

    addMem(barebone_soc_params_.mem_params_.name_, std::move(barebone_soc_params_.mem_params_), bus_name);

    etiss_sc::CPUBase *cpu{ nullptr };
    try
    {
        cpu = cpus_.at(cpu_name).get();
    }
    catch (const std::out_of_range &)
    {
        XREPORT_FATAL("cpu not found in BareboneSoC::setup()");
    }
    if (etiss::cfg().get<bool>("etiss.enable_dmi", true))
    {
        cpu->setupDMI(mem_base_addr);
    }
}

BareboneSoCFactory::BareboneSoCFactory(const etiss_sc::Config &cfg, etiss::Initializer *etiss_init)
    : etiss_sc::SoCFactory(cfg, etiss_init)
{
}

void BareboneSoCFactory::initParams()
{
    etiss_sc::SoCFactory::initParams();

    soc_params_.cpu_factory_ = std::make_unique<etiss_sc::CPUFactory>(cfg_, etiss_init_);

    auto mem_params = cfg_.getSubConfig("mem");
    barebone_soc_params_.mem_params_.base_addr_ = mem_params.get<uint64_t>("start");
    barebone_soc_params_.mem_params_.size_ = mem_params.get<size_t>("size");
    barebone_soc_params_.mem_params_.is_rom_ = mem_params.get<bool>("is_rom");
    barebone_soc_params_.mem_params_.has_ram_ = mem_params.get<bool>("has_ram");
    barebone_soc_params_.mem_params_.ACCESS_TIME =
        sc_core::sc_time(mem_params.get<unsigned>("delay_ps"), sc_core::SC_PS);
}

void BareboneSoCFactory::generate(sc_core::sc_module_name name)
{
    genHelper<BareboneSoC, BareboneSoCParams, etiss_sc::SoCParams>(name, std::move(barebone_soc_params_),
                                                                  std::move(soc_params_));
}
