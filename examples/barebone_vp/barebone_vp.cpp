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
/// @file barebone_vp.cpp
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "barebone_vp.h"
#include "barebone_soc.h"

BareboneVPDefaultConfig::BareboneVPDefaultConfig()
{
    // BUS
    set("bus.name", "bus");
    set("bus.slaves", 1);
    set("bus.masters", 2);
    set("bus.type", 2);
    // CPU
    set("cpu.name", "rv32core");
    set("cpu.arch", "RV32IMACFD");
    set("cpu.irqs", 12); // only RISC-V defined interrupt lines, no custom platform feed!
    // MEM
    set("mem.name", "mem");
    set("mem.start", 0x00000000);
    set("mem.size", 0x00100000);
    set("mem.is_rom", "false");
    set("mem.has_ram", "true");
    set("mem.delay_ps", 0);
}

BareboneVP::BareboneVP(sc_core::sc_module_name name, BareboneVPParams &&barebone_vp_params,
                       etiss_sc::VPParams &&vp_params)
    : etiss_sc::VP(name, std::move(vp_params)), barebone_vp_params_{ std::move(barebone_vp_params) }
{
}

void BareboneVP::setup()
{
    etiss_sc::VP::setup();
}

BareboneVPFactory::BareboneVPFactory(const etiss_sc::Config &cfg, etiss::Initializer *etiss_init)
    : etiss_sc::VPFactory(cfg, etiss_init)
{
}

void BareboneVPFactory::initParams()
{
    etiss_sc::VPFactory::initParams();

    vp_params_.soc_factory_ = std::make_unique<BareboneSoCFactory>(cfg_, etiss_init_);
}

void BareboneVPFactory::generate(sc_core::sc_module_name name)
{
    genHelper<BareboneVP, BareboneVPParams, etiss_sc::VPParams>(name, std::move(barebone_vp_params_),
                                                                std::move(vp_params_));
}
