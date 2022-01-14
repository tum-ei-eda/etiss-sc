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
/// @file barebone_soc.h
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __BAREBONESOC_H__
#define __BAREBONESOC_H__

#include <memory>

#include "etiss-sc/tlm/base/soc.h"
#include "etiss-sc/utils/xreport.hpp"

class BareboneSoCParams final
{
  public:
    etiss_sc::MemParams mem_params_{};
};

class BareboneSoC : virtual public etiss_sc::SoC
{
  public:
    BareboneSoC(sc_core::sc_module_name name, BareboneSoCParams &&barebone_soc_params, etiss_sc::SoCParams &&soc_params);

    void setup() override;

  protected:
    BareboneSoCParams barebone_soc_params_{};
};

class BareboneSoCFactory : public etiss_sc::SoCFactory
{
  public:
    explicit BareboneSoCFactory(const etiss_sc::Config &cfg, etiss::Initializer *etiss_init);

  protected:
    BareboneSoCParams barebone_soc_params_{};

    void initParams() override;
    void generate(sc_core::sc_module_name name) override;
};

#endif // __BAREBONESOC_H__
