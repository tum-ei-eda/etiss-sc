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
/// @file cpu_base.h
/// @date 2022-05-30
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_TLM_GENERIC_CPU_BASE_H__
#define __ETISS_SC_TLM_GENERIC_CPU_BASE_H__

#include <forward_list> // for systemc dmi
#include <string>

#include "systemc"
#include "tlm.h"

#include "etiss-sc/utils/common.h"
#include "etiss-sc/utils/xreport.hpp"

#include "etiss/ETISS.h"

namespace etiss_sc
{
class CPUParams
{
  public:
    std::string name_{ "cpu" };
    etiss::Initializer *etiss_init_{ nullptr };
    size_t num_irqs_{ 0 };
    etiss::InterruptType irq_handler_type_{ etiss::EDGE_TRIGGERED };
};

// abstract base class
class CPUBase : public sc_core::sc_module
{
  protected:
    sc_core::sc_time cpu_time_ps_{0, sc_core::SC_PS};
  public:
    sc_core::sc_in<bool> rst_i_{ "reset_in" };
    sc_core::sc_in<bool> clk_i_{ "clock_in" };

    std::unique_ptr<tlm::tlm_initiator_socket<>> data_sock_i_{
        nullptr
    }; ///< tlm_initiator_socket is hierarchically bound to the simple_initiator_socket of the core (given the core is
       ///< not an ETISS core and has a simple_initiator_socket)
    std::unique_ptr<tlm::tlm_initiator_socket<>> instr_sock_i_{
        nullptr
    }; ///< tlm_initiator_socket is hierarchically bound to the simple_initiator_socket of the core (given the core is
       ///< not an ETISS core and has a simple_initiator_socket)

    CPUBase(sc_core::sc_module_name name, CPUParams &&cpu_params);
    virtual ~CPUBase() = default;

    virtual void setup() = 0;
    virtual void setupDMI(uint64_t addr){}; // should be overloaded if CPU supports DMI
    size_t getNumIRQs() const;
    virtual void bindIRQ(size_t id, sc_core::sc_signal<bool> &irq) const = 0;

    virtual void final(){};

    int get_cpu_id() { return cpu_id_; }

  protected:
    // Every subclass must have its own core
    // std::shared_ptr<CORE_TYPE> core_{ nullptr };
    CPUParams cpu_params_{};

    static int id;
    int cpu_id_;
};

} // namespace etiss_sc

#endif // __ETISS_SC_TLM_GENERIC_CPU_BASE_H__
