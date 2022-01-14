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
/// @file vp.h
/// @date 2019-03-09
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_TLM_BASE_VP_H__
#define __ETISS_SC_TLM_BASE_VP_H__

#include <map>
#include <memory>
#include <string>

#include "etiss/ETISS.h"
#include "systemc"

#include "etiss-sc/utils/common.h"
#include "etiss-sc/utils/xreport.hpp"

#include "etiss-sc/tlm/generic/reset_gen.h"
#include "etiss-sc/tlm/base/soc.h"

#include "etiss-sc/utils/config.h"

namespace etiss_sc
{
  
class VPParams final
{
  public:
    etiss::Initializer *etiss_init_{ nullptr };
    std::unique_ptr<SoCFactory> soc_factory_{ nullptr };
    uint64_t clk_period_ns_{ 10 };
    bool reset_active_high_{ true };
};

// abstract class
class VP : sc_core::sc_module
{
  public:
    VP(sc_core::sc_module_name, VPParams &&);
    virtual ~VP() = default;

    const SoC *getSoC(std::string) const;
    virtual void setup();

  protected:
    VPParams vp_params_{};
    sc_core::sc_signal<bool> rst_{ "reset_sig" };
    std::unique_ptr<ResetGenerator> rst_gen_{ nullptr };
    std::map<std::string, std::unique_ptr<SoC>> socs_{};

    virtual void addSoC(SoCFactory *soc_factory, std::string name);
};

// abstract class
class VPFactory : public Factory<VP>
{
  public:
    explicit VPFactory(const etiss_sc::Config &, etiss::Initializer *);

  protected:
    using Factory<VP>::genHelper;
    VPParams vp_params_{};

    void initParams() override;
    void generate(sc_core::sc_module_name name) override;
};

} // namespace etiss_sc

#endif // __ETISS_SC_TLM_BASE_VP_H__
