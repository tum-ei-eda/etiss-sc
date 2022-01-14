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
/// @file cpu.h
/// @date 2019-03-09
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_TLM_GENERIC_CPU_H__
#define __ETISS_SC_TLM_GENERIC_CPU_H__

#include <forward_list> // for systemc dmi
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "etiss/ETISS.h"
#include "systemc"
#include "tlm_utils/simple_initiator_socket.h"

#include "etiss-sc/utils/common.h"
#include "etiss-sc/utils/xreport.hpp"

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

class CPU : public ETISS_System, public sc_core::sc_module
{
    SC_HAS_PROCESS(CPU);

  protected:
    class IRQ : public sc_core::sc_module
    {
        SC_HAS_PROCESS(IRQ);

      public:
        sc_core::sc_in<bool> rst_i_{ "reset_in" };
        sc_core::sc_in<bool> irq_i_{ "irq_in" };

        IRQ(sc_core::sc_module_name name, size_t id, etiss::InterruptHandler *irq_handler);

      private:
        size_t id_{ 0 };
        etiss::InterruptHandler *irq_handler_{ nullptr };

        void execute();
    };

    class ResetTerminatePlugin : public etiss::CoroutinePlugin
    {
        enum class State
        {
            IDLE,
            TRIGGERED,
            SENT,
            PENDING
        };

      public:
        ResetTerminatePlugin() = default;

        etiss_int32 execute() override;
        std::string _getPluginName() const override { return "ResetTerminatePlugin"; }

        void reset(bool value);
        void terminate();

      private:
        State state_{ State::TRIGGERED };
        bool reset_{ false };
        bool terminate_{ false };
    };

  public:
    using outcome_t = std::map<std::string, bool>;

    sc_core::sc_in<bool> rst_i_{ "reset_in" };
    std::unique_ptr<tlm_utils::simple_initiator_socket<CPU>> data_sock_i_{ nullptr };
    std::unique_ptr<tlm_utils::simple_initiator_socket<CPU>> instr_sock_i_{ nullptr };
    std::vector<std::unique_ptr<IRQ>> irq_i_{};
    static size_t id;

    CPU(sc_core::sc_module_name name, CPUParams &&cpu_params);
    virtual ~CPU();

    virtual void setup();
    virtual void setupDMI(uint64_t addr);
    size_t getNumIRQs() const;
    void bindIRQ(size_t id, sc_core::sc_signal<bool> &irq) const;

    virtual void systemCallSyncTime(ETISS_CPU *cpu);
    virtual etiss_int32 systemCallIRead(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint32 length);
    virtual etiss_int32 systemCallIWrite(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);
    virtual etiss_int32 systemCallDRead(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);
    virtual etiss_int32 systemCallDWrite(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);
    virtual etiss_int32 systemCallDbgRead(etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);
    virtual etiss_int32 systemCallDbgWrite(etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);

    int32_t get_etiss_status(void) { return etiss_status_; }

  protected:
    CPUParams cpu_params_{};
    std::shared_ptr<etiss::CPUCore> etiss_core_{ nullptr };
    std::shared_ptr<etiss::InterruptHandler> irq_handler_{ nullptr };
    std::shared_ptr<ResetTerminatePlugin> reset_terminate_handler_{ nullptr };
    std::forward_list<tlm::tlm_dmi> dmi_objects_{};
    uint64_t quantum_{ 0 };
    tlm::tlm_generic_payload payload_{};
    enum class CPUStatus
    {
        ACTIVE,
        FINISHED,
        TERMINATED
    } status_{};
    int32_t etiss_status_{ etiss::RETURNCODE::NOERROR };
    std::vector<uint32_t> dwrites_{};

    virtual void resetMethod();
    virtual void execute();
    void transaction(ETISS_CPU *cpu, uint64_t addr, uint8_t *buffer, uint32_t length, tlm::tlm_command cmd,
                     tlm_utils::simple_initiator_socket<CPU> &socket);
    uint32_t dbgTransaction(uint64_t addr, uint8_t *buffer, uint32_t length, tlm::tlm_command cmd,
                            tlm_utils::simple_initiator_socket<CPU> &socket);
    void dmiAccess(uint8_t *dst, uint8_t *src, unsigned len, bool flip_endianness = false);
    sc_core::sc_time getTimeOffset(ETISS_CPU *cpu);
    void updateCPUTime(ETISS_CPU *cpu, const sc_core::sc_time &time_offset);
    void updateSystemCTime(sc_core::sc_time &time_offset);
    void configurePayload(uint64_t addr, tlm::tlm_command cmd, uint8_t *buffer = nullptr, uint32_t length = 0);
};

// abstract class
class CPUFactory : public Factory<CPU>
{
  public:
    explicit CPUFactory(const etiss_sc::Config &cfg, etiss::Initializer *etiss_init);
    std::string get_name() { return cpu_params_.name_; }

  protected:
    using Factory<CPU>::genHelper;
    CPUParams cpu_params_{};

    void initParams() override;
    void generate(sc_core::sc_module_name name) override;
};

} // namespace etiss_sc

#endif // __ETISS_SC_TLM_GENERIC_CPU_H__
