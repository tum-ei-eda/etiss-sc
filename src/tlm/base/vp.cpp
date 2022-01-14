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
/// @file vp.cpp
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "etiss-sc/tlm/base/vp.h"

#define ID_ETISS_SC_VP "etiss-sc: VP"

etiss_sc::VP::VP(sc_core::sc_module_name name, etiss_sc::VPParams &&vp_params)
    : sc_core::sc_module(name), vp_params_{ std::move(vp_params) }
{
}

const etiss_sc::SoC *etiss_sc::VP::getSoC(std::string name) const
{
    SoC *soc{ nullptr };
    try
    {
        soc = socs_.at(name).get();
    }
    catch (const std::out_of_range &)
    {
        SC_REPORT_FATAL(ID_ETISS_SC_VP, "soc not found in VP::getSoC()");
    }
    return soc;
}

void etiss_sc::VP::setup()
{
    rst_gen_ = std::make_unique<etiss_sc::ResetGenerator>("reset_generator", vp_params_.clk_period_ns_,
                                                          vp_params_.reset_active_high_);
    rst_gen_->rst_o_(rst_);

    addSoC(vp_params_.soc_factory_.get(), "soc");
}

void etiss_sc::VP::addSoC(SoCFactory *soc_factory, std::string name)
{
    auto soc = soc_factory->create(name.c_str());
    soc->rst_i_(rst_);

    socs_.insert(std::make_pair(name, std::move(soc)));
}

etiss_sc::VPFactory::VPFactory(const etiss_sc::Config &cfg, etiss::Initializer *etiss_init)
    : etiss_sc::Factory<VP>(cfg, etiss_init)
{
}

void etiss_sc::VPFactory::initParams()
{
    vp_params_.etiss_init_ = etiss_init_;
}

void etiss_sc::VPFactory::generate(sc_core::sc_module_name name)
{
    //
}
