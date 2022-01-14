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
/// @file reset_gen.h
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_TLM_GENERIC_RESET_GEN_H__
#define __ETISS_SC_TLM_GENERIC_RESET_GEN_H__

#include "systemc.h"

namespace etiss_sc
{

class ResetGenerator final : public sc_core::sc_module
{
    SC_HAS_PROCESS(ResetGenerator);
    bool active_high_;

  public:
    sc_core::sc_out<bool> rst_o_{ "reset_out" };

    ResetGenerator(sc_core::sc_module_name name, uint64_t clk_period, bool active_high = true);

  private:
    uint64_t clk_period_{ 0 };

    void genRst();
};
} // namespace etiss_sc

#endif // __ETISS_SC_TLM_GENERIC_RESET_GEN_H__
