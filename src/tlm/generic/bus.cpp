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
/// @file bus.cpp
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "etiss/ETISS.h"

#include "etiss-sc/tlm/generic/bus.h"

#define ID_ETISS_SC_BUS "etiss-sc: Bus"

etiss_sc::Bus::Bus(sc_core::sc_module_name name, BusParams &&params)
    : sc_core::sc_module(name)
    , params_{ std::move(params) }
    , slave_sock_t_{ params.num_masters_ }
    , master_sock_i_{ params.num_slaves_ }
{
    // make sure that slave-sockets and master-sockets are properly setup
    if (!slave_sock_t_.size() || !master_sock_i_.size())
    {
        // XREPORT_FATAL("bus sockets not setup properly in Bus::Bus()");
        SC_REPORT_FATAL(ID_ETISS_SC_BUS, "bus sockets not setup properly in Bus::Bus()");
    }

    for (size_t i = 0; i < slave_sock_t_.size(); ++i)
    {
        slave_sock_t_[i] = std::make_unique<tlm_utils::simple_target_socket_tagged<Bus>>(
            std::string{ "slave_sock_" + std::to_string(i) }.c_str());
        slave_sock_t_[i]->register_b_transport(this, &Bus::b_transport, i);
        slave_sock_t_[i]->register_transport_dbg(this, &Bus::transport_dbg, i);
        slave_sock_t_[i]->register_get_direct_mem_ptr(this, &Bus::get_direct_mem_ptr, i);
    }

    for (size_t i = 0; i < master_sock_i_.size(); ++i)
    {
        master_sock_i_[i] = std::make_unique<tlm_utils::simple_initiator_socket_tagged<Bus>>(
            std::string{ "master_sock_" + std::to_string(i) }.c_str());
        master_sock_i_[i]->register_invalidate_direct_mem_ptr(this, &Bus::invalidate_direct_mem_ptr, i);

        mapping_.emplace_back(nullptr);
    }
}

void etiss_sc::Bus::b_transport(int id, tlm::tlm_generic_payload &gp, sc_core::sc_time &t)
{
    //  auto cmd = gp.get_command();
    //  auto addr = gp.get_address();
    //  auto len = gp.get_data_length();
    //  auto ptr = gp.get_data_ptr();
    //  std::cout << sc_core::sc_time_stamp() << " " << this->name()
    //            << ((cmd == tlm::TLM_READ_COMMAND) ? " Read " : " Write")
    //            << " to addr = 0x" << std::hex << addr << "  length   " << len
    //            << ";  Data 0x" << std::setw(2) << (unsigned)ptr[0] << std::setw(2)
    //            << (unsigned)ptr[1] << std::setw(2) << (unsigned)ptr[2]
    //            << std::setw(2) << (unsigned)ptr[3] << std::dec << std::endl;

    uint64_t offset{ 0 };
    auto port_id = decode(gp.get_address(), offset);
    if (port_id >= 0)
    {
        gp.set_address(offset);
        (*master_sock_i_[port_id])->b_transport(gp, t);
    }
    else
    {
        gp.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    }
}

unsigned etiss_sc::Bus::transport_dbg(int id, tlm::tlm_generic_payload &gp)
{
    uint64_t offset{ 0 };
    auto port_id = decode(gp.get_address(), offset);
    if (port_id >= 0)
    {
        gp.set_address(offset);
        return (*master_sock_i_[port_id])->transport_dbg(gp);
    }
    else
    {
        gp.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    }
    return 0;
}

bool etiss_sc::Bus::get_direct_mem_ptr(int id, tlm::tlm_generic_payload &gp, tlm::tlm_dmi &dmi_data)
{
    uint64_t offset{ 0 };
    auto port_id = decode(gp.get_address(), offset);
    if (port_id >= 0)
    {
        gp.set_address(offset);
        auto result = (*master_sock_i_[port_id])->get_direct_mem_ptr(gp, dmi_data);
        updateRange(port_id, dmi_data);
        return result;
    }
    else
    {
        XREPORT("invalid access request for direct_mem_ptr in "
                "Bus::get_direct_mem_ptr()");
    }
    return false;
}

void etiss_sc::Bus::invalidate_direct_mem_ptr(int id, sc_dt::uint64 start, sc_dt::uint64 end)
{
    for (size_t i = 0; i < params_.num_masters_; ++i)
    {
        (*slave_sock_t_[i])
            ->invalidate_direct_mem_ptr(mapping_[id]->local2Global(start), mapping_[id]->local2Global(end));
    }
}

void etiss_sc::Bus::connectMaster(tlm::tlm_base_initiator_socket<> *sock)
{
    if (num_masters_connected_ == params_.num_masters_)
    {
        XREPORT_FATAL("not enough master socket in Bus::connectMaster()");
    }

    sock->bind(*slave_sock_t_[num_masters_connected_++]);
}

void etiss_sc::Bus::connectSlave(tlm::tlm_base_target_socket<> *sock, uint64_t start_addr, uint64_t end_addr)
{
    if (!sock)
    {
        XREPORT_FATAL("sock=0 in Bus::connectSlave()");
    }

    if (num_slaves_connected_ == params_.num_slaves_)
    {
        XREPORT_FATAL("not enough sockets in Bus::connectSlave");
    }

    master_sock_i_[num_slaves_connected_]->bind(*sock);
    setMapping(num_slaves_connected_++, start_addr, end_addr);
}

void etiss_sc::Bus::setMapping(int id, uint64_t start, uint64_t end)
{
    if (mapping_[id])
    {
        XREPORT_FATAL("bus port " + std::to_string(id) + " already mapped in Bus::setMapping()");
    }
    mapping_[id] = std::make_unique<PortMapping>(start, end);
}

int etiss_sc::Bus::decode(uint64_t addr, uint64_t &offset)
{
    for (size_t i = 0; i < mapping_.size(); ++i)
    {
        if (mapping_[i]->contains(addr))
        {
            offset = mapping_[i]->global2Local(addr);
            return i;
        }
    }

    return -1;
}

void etiss_sc::Bus::updateRange(int id, tlm::tlm_dmi &dmi_data)
{
    dmi_data.set_start_address(mapping_[id]->local2Global(dmi_data.get_start_address()));
    dmi_data.set_end_address(std::min(mapping_[id]->end_, mapping_[id]->local2Global(dmi_data.get_end_address())));
}
