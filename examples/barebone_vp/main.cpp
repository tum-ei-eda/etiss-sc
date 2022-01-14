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
/// @file main.cpp
/// @date 2019-03-09
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "etiss/ETISS.h"
#include "systemc"

#include "barebone_vp.h"
#include "etiss-sc/utils/logging.h"
#include "etiss-sc/utils/common.h"

int sc_main(int argc, char **argv)
{

    etiss_sc::CommandLineParser<char> cl{ argc, argv };

    etiss_sc::SetCmdlineArgs(cl.get_parsed_size(), cl.get_parsed());
    auto etiss_init = etiss_sc::GetETISSInit();

    // Handling of reports
    if (cl.get_log_file_path() != "")
    {
        sc_report_handler::set_log_file_name(cl.get_log_file_path().c_str()); // Logging file
    }
    // Configure VP:
    // 1) Use defaults
    BareboneVPDefaultConfig cfg;
    // 2) Load/Replace with sets from ext. config file
    auto vp_cfg_fpath = cl.get_vp_config_path();
    if (vp_cfg_fpath != "")
    {
        cfg.loadFromFile(vp_cfg_fpath);
    }

    BareboneVPFactory factory{ cfg, etiss_init };

    auto vp = factory.create("barebone_vp");

    auto vp_sim_time = etiss::cfg().get<int>("vp.simulation_time_us", 0);
    if (vp_sim_time <= 0)
    {
        SC_REPORT_WARNING("sc_main", "vp.simulation_time_us <= 0: Simulate until SystemC scheduler completes.");
        sc_core::sc_start();
    }
    else
    {
        sc_core::sc_start(static_cast<double>(vp_sim_time), sc_core::SC_US);
    }

    return 0;
}
