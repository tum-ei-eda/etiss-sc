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
/// @file bus.h
/// @date 2019-03-09
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_TLM_GENERIC_BUS_H__
#define __ETISS_SC_TLM_GENERIC_BUS_H__

#include <memory>
#include <vector>
#include <string>

#include "systemc"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "etiss-sc/utils/xreport.hpp"

namespace etiss_sc
{
class BusParams final
{
  public:
    std::string name_{ "bus" };
    size_t num_masters_{ 0 };
    size_t num_slaves_{ 0 };
};

class Bus : public sc_core::sc_module
{
    class PortMapping
    {
      public:
        uint64_t start_{ 0 };
        uint64_t end_{ 0 };

        PortMapping(uint64_t start, uint64_t end) : start_{ start }, end_{ end } {}
        bool contains(uint64_t addr) { return (addr >= start_) && (addr < end_); }
        uint64_t global2Local(uint64_t addr) { return (addr - start_); }
        uint64_t local2Global(uint64_t addr) { return (addr + start_); }
    };

  public:
    std::vector<std::unique_ptr<tlm_utils::simple_target_socket_tagged<Bus>>> slave_sock_t_{};
    std::vector<std::unique_ptr<tlm_utils::simple_initiator_socket_tagged<Bus>>> master_sock_i_{};

    Bus(sc_core::sc_module_name name, BusParams &&params);
    virtual ~Bus() = default;
    Bus(const Bus &) = default;
    Bus(Bus &&) = default;
    Bus &operator=(const Bus &) = default;
    Bus &operator=(Bus &&) = default;

    virtual void b_transport(int id, tlm::tlm_generic_payload &gp, sc_core::sc_time &t);
    virtual unsigned transport_dbg(int id, tlm::tlm_generic_payload &gp);
    virtual bool get_direct_mem_ptr(int id, tlm::tlm_generic_payload &gp, tlm::tlm_dmi &dmi_data);
    virtual void invalidate_direct_mem_ptr(int id, sc_dt::uint64 start, sc_dt::uint64 end);
    void connectMaster(tlm::tlm_base_initiator_socket<> *sock);
    void connectSlave(tlm::tlm_base_target_socket<> *sock, uint64_t start_addr, uint64_t end_addr);

  protected:
    BusParams params_{};
    size_t num_masters_connected_{ 0 };
    size_t num_slaves_connected_{ 0 };
    std::vector<std::unique_ptr<PortMapping>> mapping_{};

    void setMapping(int id, uint64_t start, uint64_t end);
    int decode(uint64_t addr, uint64_t &offset);
    void updateRange(int id, tlm::tlm_dmi &dmi_data);
};
} // namespace etiss_sc

#endif // __ETISS_SC_TLM_GENERIC_BUS_H__
