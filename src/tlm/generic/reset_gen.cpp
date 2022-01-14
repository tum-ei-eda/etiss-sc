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
/// @file reset_gen.cpp
/// @date 2019-03-06
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "etiss-sc/tlm/generic/reset_gen.h"

etiss_sc::ResetGenerator::ResetGenerator(sc_core::sc_module_name name, uint64_t clk_period, bool active_high)
    : sc_core::sc_module(name), clk_period_{ clk_period }, active_high_(active_high)
{
    SC_THREAD(genRst);
}

void etiss_sc::ResetGenerator::genRst()
{
    rst_o_.write(!active_high_);
    wait(clk_period_, SC_NS);
    rst_o_.write(active_high_);
    wait(clk_period_ * 10, SC_NS);
    rst_o_.write(!active_high_);
}
