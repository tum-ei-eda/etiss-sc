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
/// @file cpu.h
/// @date 2022-05-30
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_TLM_GENERIC_CPU_H__
#define __ETISS_SC_TLM_GENERIC_CPU_H__

#include "etiss-sc/tlm/generic/cpu_base.h"
#include "tlm_utils/simple_initiator_socket.h"

#include <string>
#include <vector>
#include <atomic>

namespace etiss_sc
{

// Note: CPU is a systemc CPU with an etiss::CPUCore and ETISS_CPU is a struct from etiss
class CPU : public CPUBase, public ETISS_System
{
    SC_HAS_PROCESS(CPU);
    bool aligned_{ false };

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
    enum class CPUStatus
    {
        ACTIVE,
        FINISHED,
        TERMINATED,
        STUCK
    };

  public:
    void set_terminate_callback(std::function<void(CPUStatus)> func) { terminate_callback_ = func; }

    std::vector<std::unique_ptr<IRQ>> irq_i_{}; ///< Interrupt vector

    CPU(sc_core::sc_module_name name, CPUParams &&cpu_params);
    virtual ~CPU();

    void setup();
    void setupDMI(uint64_t addr) override;
    void bindIRQ(size_t id, sc_core::sc_signal<bool> &irq) const;

    virtual void systemCallSyncTime(ETISS_CPU *cpu);
    virtual etiss_int32 systemCallIRead(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint32 length);
    virtual etiss_int32 systemCallIWrite(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);
    virtual etiss_int32 systemCallDRead(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);
    virtual etiss_int32 systemCallDWrite(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);
    virtual etiss_int32 systemCallDbgRead(etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);
    virtual etiss_int32 systemCallDbgWrite(etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length);

    int32_t get_etiss_status(void) { return etiss_status_; }
    std::shared_ptr<etiss::VirtualStruct> get_core_struct(void) { return etiss_core_->getStruct(); }
    std::shared_ptr<etiss::CPUCore> get_core(void) { return etiss_core_; }
    ETISS_CPU *get_etiss_cpu_struct(void) { return etiss_core_->getState(); }

    void align_cpu_to_systemc_time(void);

    void freeze_cpu()
    {
        freeze_cpu_ = true;
        std::cout << "        +++ iss_cpu frozen" << std::endl;
    };
    void wake_up_cpu()
    {
        if (freeze_cpu_.load() == true)
        {
            wake_up_cpu_.notify();
            freeze_cpu_ = false;
        }
        std::cout << "        +++ iss_cpu woken up" << std::endl;
    };

    std::shared_ptr<etiss::CPUCore> etiss_core_{ nullptr };

  protected:
    std::function<void(CPUStatus)> terminate_callback_{};
    std::shared_ptr<etiss::InterruptHandler> irq_handler_{ nullptr };
    std::shared_ptr<ResetTerminatePlugin> reset_terminate_handler_{ nullptr };

    sc_core::sc_event wake_up_cpu_{ "wake_up_cpu_event" };
    std::atomic<bool> freeze_cpu_{ false };

    std::forward_list<tlm::tlm_dmi> dmi_objects_{};
    sc_core::sc_time const quantum_;
    tlm::tlm_generic_payload payload_{};

    CPUStatus status_{};
    int32_t etiss_status_{ etiss::RETURNCODE::NOERROR };

    virtual void resetMethod();
    virtual void execute();

    void transaction(ETISS_CPU *cpu, uint64_t addr, uint8_t *buffer, uint32_t length, tlm::tlm_command cmd,
                     tlm::tlm_initiator_socket<> &socket);
    virtual uint32_t dbgTransaction(uint64_t addr, uint8_t *buffer, uint32_t length, tlm::tlm_command cmd,
                                    tlm::tlm_initiator_socket<> &socket);
    virtual void dmiAccess(uint8_t *dst, uint8_t *src, unsigned len, bool flip_endianness = false);

    sc_core::sc_time getTimeOffset(ETISS_CPU *cpu) const; ///< LEGACY: use getTimeOffset with explicit time stamp
    sc_core::sc_time getTimeOffset(ETISS_CPU *cpu, sc_core::sc_time const &current_time_stamp) const;
    void updateCPUTime(ETISS_CPU *cpu,
                       const sc_core::sc_time &time_offset); ///< LEGACY: use updateCPUTime with explicit time stamp
    void updateCPUTime(ETISS_CPU *cpu, const sc_core::sc_time &time_offset, sc_core::sc_time const &current_time_stamp);
    void progressSystemCTime(sc_core::sc_time &time_offset);
    void configurePayload(uint64_t addr, tlm::tlm_command cmd, uint8_t *buffer = nullptr, uint32_t length = 0);
};

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
