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
/// @file mem.cpp
/// @date 2019-03-09
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <random>

#include "etiss/ETISS.h"

#include "etiss-sc/tlm/generic/mem.h"

//#define SC_MEM_WRITE_TRACE
#define ID_ETISS_SC_MEM "etiss-sc: mem"

#ifdef SC_MEM_WRITE_TRACE
inline void mem_trace_writer(tlm::tlm_generic_payload *gp, const char *module_name, bool close_file)
{
    static std::map<const char *, std::ofstream> trace_files;

    std::ofstream &trace_file = trace_files[module_name];

    if (close_file)
    {
        if (trace_file.is_open())
            trace_file.close();
    }
    else
    {
        if (not trace_file.is_open()) // open the file
        {
            assert(("mem_trace_writer(): param module_name cannot be NULL", module_name));
            std::string filename = std::string(module_name) + "_memtrace.csv";
#ifdef SC_MEM_TRACE_FILE_PREFIX
            filename = std::string(MEM_TRACE_FILE_PREFIX) + filename;
#endif
            trace_file.open(filename.c_str());
        }

        tlm::tlm_command cmd = gp->get_command();
        uint64_t adr = gp->get_address();
        unsigned char *ptr = gp->get_data_ptr();
        unsigned int len = gp->get_data_length();
        unsigned char *byt = gp->get_byte_enable_ptr();
        unsigned int bel = gp->get_byte_enable_length();

        // print raw data as in the payload; these are a bit strange as the
        // endianness handling heavily uses byte enable masks
        uint64_t mask{ 0xffffffffffffffff };
        if (byt)
        {
            mask = *(reinterpret_cast<uint64_t *>(byt));
        }
        // mask = mask & (0xffffffff >> (4 - len) * 8);
        trace_file << (uint64_t)(sc_core::sc_time_stamp().to_seconds() * 1e9) << ","
                   << ((cmd == tlm::TLM_WRITE_COMMAND) ? "w" : "r") << ',' << std::setw(16) << std::setfill('0')
                   << std::hex << adr << "," << std::dec << len << "," << std::hex << std::setw(16) << std::setfill('0')
                   << *(reinterpret_cast<uint64_t *>(ptr)) << "," << std::hex << std::setw(16) << std::setfill('0')
                   << mask << "," << (gp->is_response_error() ? '1' : '0') << std::dec << std::endl;
    }
}
#endif

etiss_sc::Mem::Mem(sc_core::sc_module_name name, MemParams &&params)
    : sc_core::sc_module(name), params_{ std::move(params) }
{
    SC_METHOD(reset);
    dont_initialize();
    sensitive << rst_i_.pos();

    sock_t_ = std::make_unique<tlm_utils::simple_target_socket<Mem>>("socket");
    sock_t_->register_b_transport(this, &Mem::b_transport);
    sock_t_->register_transport_dbg(this, &Mem::transport_dbg);
    sock_t_->register_get_direct_mem_ptr(this, &Mem::get_direct_mem_ptr);

    mem_.resize(params_.size_);
    std::fill(mem_.begin(), mem_.end(), 0);
    // garbageMem();
}

etiss_sc::Mem::~Mem()
{
#ifdef SC_MEM_WRITE_TRACE
    mem_trace_writer(NULL, NULL, true);
#endif
}

void etiss_sc::Mem::reset()
{
    mem_ = rst_mem_;
}

void etiss_sc::Mem::garbageMem()
{
    return;
    static std::default_random_engine generator{ static_cast<uint64_t>(0) };
    std::uniform_int_distribution<int> random_char_{ 0, 255 };

    for (auto i = 0; i < params_.size_; ++i)
    {
        mem_[i] = random_char_(generator);
    }
}

void etiss_sc::Mem::loadDataFile(const std::vector<uint8_t> &data)
{
    mem_ = data;
    rst_mem_ = mem_;
}

void etiss_sc::Mem::setByte(uint64_t addr, uint8_t value)
{
    auto offset = (addr - params_.base_addr_);
    mem_[offset] = value;
}

uint8_t etiss_sc::Mem::getByte(uint64_t addr) const
{
    auto offset = (addr - params_.base_addr_);
    return mem_[offset];
}

void etiss_sc::Mem::transaction(tlm::tlm_generic_payload &gp)
{
    auto cmd = gp.get_command();
    auto addr = gp.get_address();
    auto ptr = gp.get_data_ptr();
    auto len = gp.get_data_length();

    auto be = gp.get_byte_enable_ptr();
    auto be_len = gp.get_byte_enable_length();

    gp.set_response_status(tlm::TLM_OK_RESPONSE);

    if (addr >= mem_.size() || (addr + len) > mem_.size())
    {
        gp.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
        return;
    }
    if (cmd == tlm::TLM_WRITE_COMMAND)
    {
        if (likely(be == nullptr))
            memcpy(mem_.data() + addr, ptr, len);
        else
        {
            for (size_t i_byte = 0; i_byte < len; ++i_byte)
            {
                mem_.data()[addr + i_byte] = (mem_.data()[addr + i_byte] & ~be[i_byte]) ^ (ptr[i_byte] & be[i_byte]);
            }
        }
    }
    else if (cmd == tlm::TLM_READ_COMMAND)
    {
        memcpy(ptr, mem_.data() + addr, len);
    }
    else
    {
        gp.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
    }
}

void etiss_sc::Mem::b_transport(tlm::tlm_generic_payload &gp, sc_core::sc_time &t)
{
    if (params_.has_ram_)
    {
        static size_t last_mem{ params_.size_ };

        auto addr = gp.get_address();
        if ((addr >= (params_.size_ / 2)) && (addr < last_mem))
        {
            last_mem = addr;
            dyn_mem_size_ = params_.size_ - last_mem;
        }
    }

    transaction(gp);
    t += params_.ACCESS_TIME;

#ifdef SC_MEM_WRITE_TRACE
    mem_trace_writer(&gp, name(), false);
#endif
}

unsigned int etiss_sc::Mem::transport_dbg(tlm::tlm_generic_payload &gp)
{
    transaction(gp);

#ifdef SC_MEM_WRITE_TRACE
    mem_trace_writer(&gp, "dbg", false);
#endif

    if (gp.get_response_status() == tlm::TLM_OK_RESPONSE)
    {
        return gp.get_data_length();
    }
    else
    {
        return 0;
    }
}

bool etiss_sc::Mem::get_direct_mem_ptr(tlm::tlm_generic_payload &trans, tlm::tlm_dmi &dmi_data)
{
    dmi_data.set_dmi_ptr(reinterpret_cast<unsigned char *>(mem_.data()));
    dmi_data.set_start_address(0);
    dmi_data.set_end_address(mem_.size() - 1);
    dmi_data.set_read_latency(params_.ACCESS_TIME / 4);
    dmi_data.set_write_latency(params_.ACCESS_TIME / 4);
    dmi_data.allow_read_write();
    return true;
}
