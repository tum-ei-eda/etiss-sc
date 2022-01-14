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

#ifndef __ETISS_SC_TLM_GENERIC_MEM_H__
#define __ETISS_SC_TLM_GENERIC_MEM_H__

#include <memory>
#include <string>
#include <vector>

#include "etiss-sc/utils/xreport.hpp"
#include "systemc"
#include "tlm_utils/simple_target_socket.h"

// #define SC_MEM_WRITE_TRACE

namespace etiss_sc
{
class MemParams final
{
  public:
    std::string name_{ "mem" };
    sc_core::sc_time ACCESS_TIME{ sc_core::SC_ZERO_TIME };
    uint64_t base_addr_{ 0 };
    size_t size_{ 0 };
    bool has_ram_{ false };
    bool is_rom_{ false };
};

class Mem : public sc_core::sc_module
{
    SC_HAS_PROCESS(Mem);

  public:
    sc_core::sc_in<bool> rst_i_{ "reset_in" };
    std::unique_ptr<tlm_utils::simple_target_socket<Mem>> sock_t_{ nullptr };
    size_t dyn_mem_size_{ 0 };
    MemParams params_{};

    Mem(sc_core::sc_module_name name, MemParams &&params);
    virtual ~Mem();
    Mem(const Mem &) = default;
    Mem(Mem &&) = default;
    Mem &operator=(const Mem &) = default;
    Mem &operator=(Mem &&) = default;

    virtual void b_transport(tlm::tlm_generic_payload &gp, sc_core::sc_time &t);
    virtual unsigned int transport_dbg(tlm::tlm_generic_payload &gp);
    virtual bool get_direct_mem_ptr(tlm::tlm_generic_payload &trans, tlm::tlm_dmi &dmi_data);
    virtual void setByte(uint64_t addr, uint8_t value);
    virtual uint8_t getByte(uint64_t addr) const;
    void loadDataFile(const std::vector<uint8_t> &);

  protected:
    std::vector<uint8_t> mem_{};
    std::vector<uint8_t> rst_mem_{};

    void garbageMem();
    virtual void reset();
    void transaction(tlm::tlm_generic_payload &gp);
};
} // namespace etiss_sc

#endif // __ETISS_SC_TLM_GENERIC_MEM_H__
