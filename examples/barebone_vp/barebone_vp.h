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
/// @file barebone_vp.h
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __BAREBONEVP_H__
#define __BAREBONEVP_H__

#include <memory>

#include "etiss-sc/tlm/base/vp.h"
#include "etiss-sc/utils/config.h"

class BareboneVPDefaultConfig : public etiss_sc::Config
{
  public:
    BareboneVPDefaultConfig();
};

class BareboneVPParams final
{
};

class BareboneVP : virtual public etiss_sc::VP
{
  public:
    BareboneVP(sc_core::sc_module_name name, BareboneVPParams &&barebone_vp_params, etiss_sc::VPParams &&vp_params);

    void setup() override;

  protected:
    BareboneVPParams barebone_vp_params_{};
};

class BareboneVPFactory : public etiss_sc::VPFactory
{
  public:
    explicit BareboneVPFactory(const etiss_sc::Config &cfg, etiss::Initializer *etiss_init);

  protected:
    BareboneVPParams barebone_vp_params_{};

    void initParams() override;
    void generate(sc_core::sc_module_name name) override;
};

#endif // __BAREBONEVP_H__
