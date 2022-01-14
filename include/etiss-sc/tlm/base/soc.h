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
/// @file soc.h
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_TLM_BASE_SOC_H__
#define __ETISS_SC_TLM_BASE_SOC_H__

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "etiss/ETISS.h"
#include "systemc"

#include "scc/tlm_target_bfs.h"

#include "etiss-sc/utils/xreport.hpp"
#include "etiss-sc/tlm/generic/cpu.h"
#include "etiss-sc/tlm/generic/bus.h"
#include "etiss-sc/tlm/generic/mem.h"

#include "elfio/elfio.hpp"

#define ID_ETISS_SC_SOC "etiss-sc: SoC"

namespace etiss_sc
{
class SoCParams final
{
  public:
    etiss::Initializer *etiss_init_{ nullptr };
    std::unique_ptr<CPUFactory> cpu_factory_{ nullptr };
    BusParams bus_params_{};
};

class VP;
// abstract class
class SoC : public sc_core::sc_module
{
  public:
    sc_core::sc_in<bool> rst_i_{ "reset_in" };

    SoC(sc_core::sc_module_name, SoCParams &&);
    virtual ~SoC() = default;

    const CPU &getCPU(std::string) const;
    const Mem *getMem(std::string) const;
    sc_core::sc_signal<bool> *getIRQ(std::string, size_t);
    virtual void setup();

    template <class per_t>
    void connectExternalDevice(std::string per_name, tlm::tlm_base_target_socket<> *ext_sock_t) const
    {
        if (!ext_sock_t)
        {
            XREPORT_FATAL("ext_sock_t=0 in SoC::connectExternalDevice<>()");
        }

        per_t *per{ nullptr };
        try
        {
            per = dynamic_cast<per_t *>(pers_.at(per_name).get());
            if (!per)
            {
                XREPORT_FATAL("per couldn't be casted properly in"
                              "SoC::connectExternalDevice<>()");
            }
        }
        catch (const std::out_of_range &)
        {
            XREPORT_FATAL("per not found in SoC::connectExternalDevice<>()");
        }

        per->sock_i_->bind(*ext_sock_t);
    }

    template <class per_t>
    void connectExternalDevice(std::string per_name, tlm::tlm_base_target_socket<> *ext_sock_t,
                               tlm::tlm_base_initiator_socket<> *ext_sock_i) const
    {
        if (!ext_sock_t || !ext_sock_i)
        {
            XREPORT("ext_sock=0 in SoC::connectExternalDevice<>()");
        }

        per_t *per{ nullptr };
        try
        {
            per = dynamic_cast<per_t *>(pers_.at(per_name).get());
            if (!per)
            {
                XREPORT_FATAL("per couldn't be casted properly in"
                              "SoC::connectExternalDevice<>()");
            }
        }
        catch (const std::out_of_range &)
        {
            XREPORT_FATAL("per not found in SoC::connectExternalDevice<>()");
        }

        per->sock_i_->bind(*ext_sock_t);
        ext_sock_i->bind(*per->sock_t_ext_);
    }

  protected:
    SoCParams soc_params_{};
    std::map<std::string, std::unique_ptr<CPU>> cpus_{};
    std::map<std::string, std::unique_ptr<Bus>> buses_{};
    std::map<std::string, std::unique_ptr<Mem>> mems_{};
    std::map<std::string, std::unique_ptr<scc::tlm_target_bfs_base<etiss_sc::SoC>>> pers_{};
    std::map<const CPU *, std::vector<sc_core::sc_signal<bool>>> irqs_{};

    std::vector<std::unique_ptr<ELFIO::elfio>> elf_readers_{};

    virtual void addBus(std::string name, BusParams &&bus_params);
    virtual const CPU *addCPU(CPUFactory *cpu_factory, std::string bus_name);
    virtual const Mem *addMem(std::string name, MemParams &&mem_params, std::string bus_name);
    void burnMem(Mem *);

    template <class per_t>
    void addPer(std::string name, scc::tlm_target_bfs_params &&per_params, std::string bus_name)
    {
        auto per = std::make_unique<per_t>(name.c_str(), std::move(per_params), this);
        per->rst_in_(this->rst_i_);

        Bus *bus{ nullptr };
        try
        {
            bus = (buses_.at(bus_name)).get();
        }
        catch (const std::out_of_range &)
        {
            SC_REPORT_FATAL(ID_ETISS_SC_SOC, "bus not found in SoC::addPer<>()\n");
        }
        bus->connectSlave(per->sock_t_.get(), per_params.base_addr, per_params.base_addr + per_params.size);

        pers_.insert(std::make_pair(name, std::move(per)));
    }
};

// abstract class
class SoCFactory : public Factory<SoC>
{
  public:
    explicit SoCFactory(const etiss_sc::Config &, etiss::Initializer *);

  protected:
    using Factory<SoC>::genHelper;
    SoCParams soc_params_{};

    void initParams() override;
    void generate(sc_core::sc_module_name name) override;
};

} // namespace etiss_sc

#endif // __ETISS_SC_TLM_BASE_SOC_H__
