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
/// @file iss_cpu.cpp
/// @date 2022-05-30
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "etiss-sc/tlm/generic/iss_cpu.h"
#include "etiss/fault/Stressor.h"
#include "etiss-sc/utils/plugins.h"

#define ID_ETISS_SC_ISS_CPU "etiss-sc: ISS_CPU"

void system_call_syncTime(void *handle, ETISS_CPU *cpu)
{
    static_cast<etiss_sc::ISS_CPU *>(handle)->systemCallSyncTime(cpu);
}

etiss_int32 system_call_iread(void *handle, ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint32 length)
{
    return static_cast<etiss_sc::ISS_CPU *>(handle)->systemCallIRead(cpu, addr, length);
}

etiss_int32 system_call_iwrite(void *handle, ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    return static_cast<etiss_sc::ISS_CPU *>(handle)->systemCallIWrite(cpu, addr, buffer, length);
}

etiss_int32 system_call_dread(void *handle, ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    return static_cast<etiss_sc::ISS_CPU *>(handle)->systemCallDRead(cpu, addr, buffer, length);
}

etiss_int32 system_call_dwrite(void *handle, ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    return static_cast<etiss_sc::ISS_CPU *>(handle)->systemCallDWrite(cpu, addr, buffer, length);
}

etiss_int32 system_call_dbg_read(void *handle, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    return static_cast<etiss_sc::ISS_CPU *>(handle)->systemCallDbgRead(addr, buffer, length);
}

etiss_int32 system_call_dbg_write(void *handle, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    return static_cast<etiss_sc::ISS_CPU *>(handle)->systemCallDbgWrite(addr, buffer, length);
}


/************************************************************************
 * ISS_CPU::IRQ                                                         *
 ************************************************************************/

etiss_sc::ISS_CPU::IRQ::IRQ(sc_core::sc_module_name name, size_t id, etiss::InterruptHandler *irq_handler)
    : sc_module(name), id_{ id }, irq_handler_{ irq_handler }
{
    if (!irq_handler_)
    {
        SC_REPORT_FATAL(ID_ETISS_SC_ISS_CPU, "invalid irq_handler passed in ISS_CPU::IRQ::IRQ()");
    }

    SC_METHOD(execute);
    sensitive << irq_i_;
    dont_initialize();
}

void etiss_sc::ISS_CPU::IRQ::execute()
{
    if (irq_i_.read())
    {
        irq_handler_->setLine(id_, true, static_cast<etiss::uint64>(sc_core::sc_time_stamp().to_seconds() * 1e12));
    }
    else
    {
        irq_handler_->setLine(id_, false, static_cast<etiss::uint64>(sc_core::sc_time_stamp().to_seconds() * 1e12));
    }
}


/************************************************************************
 * ISS_CPU::ResetTerminatePlugin                                                                    *
 ************************************************************************/

etiss_int32 etiss_sc::ISS_CPU::ResetTerminatePlugin::execute()
{
    if (terminate_)
    {
        terminate_ = false;
        return etiss::RETURNCODE::CPUTERMINATED;
    }
    else
    {
        if (state_ != State::IDLE)
        {
            switch (state_)
            {
            case State::TRIGGERED:
                state_ = State::SENT;
                break;
            case State::PENDING:
                state_ = State::IDLE;
                break;
            default:
                break;
            }

            return etiss::RETURNCODE::RESET;
        }

        return etiss::RETURNCODE::NOERROR;
    }
}

void etiss_sc::ISS_CPU::ResetTerminatePlugin::reset(bool value)
{
    switch (state_)
    {
    case State::IDLE:
        if (value)
        {
            state_ = State::TRIGGERED;
        }
        break;
    case State::TRIGGERED:
        if (!value)
        {
            state_ = State::PENDING;
        }
        break;
    case State::SENT:
        if (!value)
        {
            state_ = State::IDLE;
        }
        break;
    case State::PENDING:
        if (value)
        {
            state_ = State::TRIGGERED;
        }
        break;
    default:
        SC_REPORT_FATAL(ID_ETISS_SC_ISS_CPU, "undefined state in ISS_CPU::ResetTerminatePlugin::reset()");
    }
}

void etiss_sc::ISS_CPU::ResetTerminatePlugin::terminate()
{
    terminate_ = true;
}


/************************************************************************
 * ISS CPU                                                              *
 ************************************************************************/

etiss_sc::ISS_CPU::ISS_CPU(sc_core::sc_module_name name, CPUParams &&cpu_params)
    : CPUBase(name, std::move(cpu_params))
    , quantum_{ etiss::cfg().get<uint64_t>("etiss.cpu_quantum_ps", 0) }
{
    SC_THREAD(execute);
    SC_METHOD(resetMethod);
    dont_initialize();
    sensitive << rst_i_;

    data_sock_i_ = std::make_unique<tlm_utils::simple_initiator_socket<CPUBase>>("data_socket");
    instr_sock_i_ = std::make_unique<tlm_utils::simple_initiator_socket<CPUBase>>("instr_socket");

    this->iread = system_call_iread;
    this->iwrite = system_call_iwrite;
    this->dread = system_call_dread;
    this->dwrite = system_call_dwrite;
    this->dbg_read = system_call_dbg_read;
    this->dbg_write = system_call_dbg_write;
    this->syncTime = system_call_syncTime;
    handle = this;
}

etiss_sc::ISS_CPU::~ISS_CPU()
{
    auto cpu_time = sc_core::sc_time{ static_cast<double>(etiss_core_->getState()->cpuTime_ps), sc_core::SC_PS };
    if (status_ == CPUStatus::ACTIVE)
    {
        auto clk_period_ns = etiss::cfg().get<int>("arch.cpu_cycle_time_ps", 10000) * 1000;
        if (cpu_time + sc_core::sc_time{ 100.0 * clk_period_ns, sc_core::SC_NS } > sc_core::sc_time_stamp())
        {
            reset_terminate_handler_->terminate();

            sc_core::sc_start(200.0 * clk_period_ns, sc_core::SC_NS);

            if (status_ == CPUStatus::ACTIVE)
            {
                XREPORT_FATAL("CPU stuck and not exiting properly in ISS_CPU::~ISS_CPU()");
            }
        }
        else
        {
            XREPORT_FATAL("CPU stuck and not exited properly in ISS_CPU::~ISS_CPU()");
        }
    }

    etiss_core_->removePlugin(irq_handler_);
    etiss_core_->removePlugin(reset_terminate_handler_);
}

void etiss_sc::ISS_CPU::setup()
{
    etiss_core_ = etiss::CPUCore::create(etiss::cfg().get<std::string>("arch.cpu", ""), "core" + std::to_string(CPUBase::id));

    if (!etiss_core_) XREPORT_FATAL("failed to create ETISS-CPUCore in ISS_CPU::setup()");

    etiss::VirtualStruct::root()->mountStruct("core" + std::to_string(CPUBase::id), etiss_core_->getStruct());

    CPUBase::id++;
    cpu_params_.etiss_init_->loadIniPlugins(etiss_core_);
    cpu_params_.etiss_init_->loadIniJIT(etiss_core_);

    if (etiss_core_->getInterruptVector())
    {
        irq_handler_ = std::make_shared<etiss::InterruptHandler>(
            etiss_core_->getInterruptVector(), etiss_core_->getArch(), cpu_params_.irq_handler_type_, false);
        etiss_core_->addPlugin(irq_handler_);

        auto num_irq =
            std::min(cpu_params_.num_irqs_, static_cast<size_t>(etiss_core_->getInterruptVector()->width() * 8));
        for (size_t i = 0; i < num_irq; ++i)
        {
            irq_i_.emplace_back(
                std::make_unique<IRQ>(std::string{ "irq_i_" + std::to_string(i) }.c_str(), i, irq_handler_.get()));
            irq_i_[i]->rst_i_(this->rst_i_);
        }
    }

    reset_terminate_handler_ = std::make_shared<ResetTerminatePlugin>();
    etiss_core_->addPlugin(reset_terminate_handler_);

    if (etiss::cfg().get<bool>("etiss.log_pc", false))
    {
        addPcTraceLogger(etiss_core_, etiss::cfg().get<std::string>("etiss.output_path_prefix", "") + "pc_trace.bin",
                         etiss::cfg().get<int>("etiss.write_pc_trace_from_time_us", 0),
                         etiss::cfg().get<int>("etiss.write_pc_trace_until_time_us", INT_MAX));
    }

    etiss_core_->setTimer(etiss::cfg().get<bool>("etiss.timer", false));
}

void etiss_sc::ISS_CPU::setupDMI(uint64_t addr)
{
    std::cout << "---------------------------- [Lasse] im iss cpu dmi setup: " << std::endl;
    dmi_objects_.push_front(tlm::tlm_dmi());
    configurePayload(addr, tlm::TLM_READ_COMMAND);
    if (!((*data_sock_i_)->get_direct_mem_ptr(payload_, dmi_objects_.front()) && dmi_objects_.front().get_dmi_ptr()))
    {
        dmi_objects_.pop_front();
        XREPORT_FATAL("DMI not successful in CPU::setupDMI()");
    }
}

void etiss_sc::ISS_CPU::bindIRQ(size_t id, sc_core::sc_signal<bool> &irq) const
{
    if (id >= irq_i_.size()) XREPORT_FATAL("not enough interrupts in ISS_CPU::bindIRQ()");

    irq_i_[id]->irq_i_.bind(irq);
}

void etiss_sc::ISS_CPU::systemCallSyncTime(ETISS_CPU *cpu)
{
    auto offset = getTimeOffset(cpu);
    updateSystemCTime(offset);
}

etiss_int32 etiss_sc::ISS_CPU::systemCallIRead(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint32 length)
{
    auto return_val = reset_terminate_handler_->execute();
    if (return_val != etiss::RETURNCODE::NOERROR)
    {
        return return_val;
    }

    std::vector<uint8_t> buffer(length);
    transaction(cpu, addr, buffer.data(), length, tlm::TLM_READ_COMMAND, *instr_sock_i_);
    auto response = payload_.get_response_status();
    if (response != tlm::TLM_OK_RESPONSE)
    {
        return etiss::RETURNCODE::IBUS_READ_ERROR;
    }

    return etiss::RETURNCODE::NOERROR;
}

etiss_int32 etiss_sc::ISS_CPU::systemCallIWrite(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    auto return_val = reset_terminate_handler_->execute();
    if (return_val != etiss::RETURNCODE::NOERROR)
    {
        return return_val;
    }

    transaction(cpu, addr, buffer, length, tlm::TLM_WRITE_COMMAND, *instr_sock_i_);
    auto response = payload_.get_response_status();
    if (response != tlm::TLM_OK_RESPONSE)
    {
        return etiss::RETURNCODE::IBUS_WRITE_ERROR;
    }
    return etiss::RETURNCODE::NOERROR;
}

etiss_int32 etiss_sc::ISS_CPU::systemCallDRead(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    auto return_val = reset_terminate_handler_->execute();
    if (return_val != etiss::RETURNCODE::NOERROR)
    {
        return return_val;
    }

    transaction(cpu, addr, buffer, length, tlm::TLM_READ_COMMAND, *data_sock_i_);
    auto response = payload_.get_response_status();
    if (response != tlm::TLM_OK_RESPONSE)
    {
        return etiss::RETURNCODE::DBUS_READ_ERROR;
    }

    return etiss::RETURNCODE::NOERROR;
}

etiss_int32 etiss_sc::ISS_CPU::systemCallDWrite(ETISS_CPU *cpu, etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    auto return_val = reset_terminate_handler_->execute();
    if (return_val != etiss::RETURNCODE::NOERROR)
    {
        return return_val;
    }

    transaction(cpu, addr, buffer, length, tlm::TLM_WRITE_COMMAND, *data_sock_i_);
    auto response = payload_.get_response_status();
    if (response != tlm::TLM_OK_RESPONSE)
    {
        return etiss::RETURNCODE::DBUS_WRITE_ERROR;
    }
    return etiss::RETURNCODE::NOERROR;
}

etiss_int32 etiss_sc::ISS_CPU::systemCallDbgRead(etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    auto response_len = dbgTransaction(addr, buffer, length, tlm::TLM_READ_COMMAND, *instr_sock_i_);
    if (response_len != length)
    {
        return etiss::RETURNCODE::IBUS_READ_ERROR;
    }

    return etiss::RETURNCODE::NOERROR;
}

etiss_int32 etiss_sc::ISS_CPU::systemCallDbgWrite(etiss_uint64 addr, etiss_uint8 *buffer, etiss_uint32 length)
{
    auto response_len = dbgTransaction(addr, buffer, length, tlm::TLM_WRITE_COMMAND, *instr_sock_i_);
    if (response_len != length)
    {
        return etiss::RETURNCODE::IBUS_WRITE_ERROR;
    }
    return etiss::RETURNCODE::NOERROR;
}

void etiss_sc::ISS_CPU::resetMethod()
{
    reset_terminate_handler_->reset(rst_i_.read());
}

void etiss_sc::ISS_CPU::execute()
{
    auto etiss_status_ = etiss_core_->execute(*this);

    if (etiss_status_ == etiss::RETURNCODE::CPUFINISHED)
    {
        status_ = CPUStatus::FINISHED;
    }
    else if (etiss_status_ == etiss::RETURNCODE::CPUTERMINATED)
    {
        status_ = CPUStatus::TERMINATED;
    }
    else if (etiss_status_ < 0)
    {
        std::cout << std::hex << etiss_status_ << std::dec << std::endl;
        XREPORT("execute from CPUCore not done properly in ISS_CPU::execute()");
    }

    sc_core::sc_stop();
}

void etiss_sc::ISS_CPU::transaction(ETISS_CPU *cpu, uint64_t addr, uint8_t *buffer, uint32_t length, tlm::tlm_command cmd,
                                tlm::tlm_initiator_socket<> &socket)
{
    auto time_offset = getTimeOffset(cpu);
    updateSystemCTime(time_offset);

    // NOTE: in case we have exceeded simulation time, we are not guaranteed that
    // the bound sockets are still in scope and hence we are returning early
    static auto sim_time =
        sc_core::sc_time{ static_cast<double>(etiss::cfg().get<int>("vp.simulation_time_us", 0)), sc_core::SC_US };
    if (sim_time != sc_core::SC_ZERO_TIME && sc_core::sc_time_stamp() >= sim_time)
    {
        etiss::log(etiss::WARNING, "Skipped memory access because transaction overran simulation time");
        payload_.set_response_status(tlm::TLM_OK_RESPONSE);
        return;
    }

    for (auto d : dmi_objects_)
    {
        if (addr >= d.get_start_address() && addr <= d.get_end_address())
        {
            if (cmd == tlm::TLM_WRITE_COMMAND && d.is_write_allowed())
            {
                dmiAccess(d.get_dmi_ptr() + addr - d.get_start_address(), buffer, length);
                payload_.set_response_status(tlm::TLM_OK_RESPONSE);
                time_offset += d.get_write_latency() * length;
            }
            else if (cmd == tlm::TLM_READ_COMMAND && d.is_read_allowed())
            {
                dmiAccess(buffer, d.get_dmi_ptr() + addr - d.get_start_address(), length);
                payload_.set_response_status(tlm::TLM_OK_RESPONSE);
                time_offset += d.get_write_latency() * length;
            }
            else
            {
                // not having dmi access priviliges
                configurePayload(addr, cmd, buffer, length);
                socket->b_transport(payload_, time_offset);
            }

            updateCPUTime(cpu, time_offset);
            updateSystemCTime(time_offset);
            return;
        }
    }

    // coulnd't do transaction via dmi for some reason so trying via bus
    configurePayload(addr, cmd, buffer, length);
    socket->b_transport(payload_, time_offset);
    updateCPUTime(cpu, time_offset);
    updateSystemCTime(time_offset);
}

uint32_t etiss_sc::ISS_CPU::dbgTransaction(uint64_t addr, uint8_t *buffer, uint32_t length, tlm::tlm_command cmd,
                                           tlm::tlm_initiator_socket<> &socket)
{
    for (auto d : dmi_objects_)
    {
        if (d.get_start_address() <= addr && addr <= d.get_end_address())
        {
            if (cmd == tlm::TLM_WRITE_COMMAND && d.is_write_allowed())
            {
                dmiAccess(d.get_dmi_ptr() + addr - d.get_start_address(), buffer, length);
                return length;
            }
            else if (cmd == tlm::TLM_READ_COMMAND && d.is_read_allowed())
            {
                dmiAccess(buffer, d.get_dmi_ptr() + addr - d.get_start_address(), length);
                return length;
            }
            else
            {
                configurePayload(addr, cmd, buffer, length);
                return socket->transport_dbg(payload_);
            }
        }
    }
    // couldn't do transaction via dmi so trying via bus
    configurePayload(addr, cmd, buffer, length);
    return socket->transport_dbg(payload_);
}

void etiss_sc::ISS_CPU::dmiAccess(uint8_t *dst, uint8_t *src, unsigned len, bool flip_endianness)
{
    for (size_t i = 0; i < len; ++i)
    {
        dst[i] = src[i];
    }

    if (flip_endianness)
    {
        etiss_sc::flipEndianness(dst, len);
    }
}

sc_core::sc_time etiss_sc::ISS_CPU::getTimeOffset(ETISS_CPU *cpu)
{
    return (sc_core::sc_time(cpu->cpuTime_ps, sc_core::SC_PS) - sc_core::sc_time_stamp());
}

void etiss_sc::ISS_CPU::updateCPUTime(ETISS_CPU *cpu, const sc_core::sc_time &time_offset)
{
    cpu->cpuTime_ps = (sc_core::sc_time_stamp().to_seconds() + time_offset.to_seconds()) * 1e12;
}

void etiss_sc::ISS_CPU::updateSystemCTime(sc_core::sc_time &time_offset)
{
    if (sc_core::sc_time_stamp() == sc_core::SC_ZERO_TIME ||
        time_offset > sc_core::sc_time{ static_cast<double>(quantum_), sc_core::SC_PS })
    {
        wait(time_offset);
        time_offset = sc_core::sc_time{ 0, sc_core::SC_PS };
    }
}

void etiss_sc::ISS_CPU::configurePayload(uint64_t addr, tlm::tlm_command cmd, uint8_t *buffer, uint32_t length)
{
    payload_.set_command(cmd);
    payload_.set_address(addr);
    payload_.set_data_ptr(buffer);
    payload_.set_data_length(length);
    payload_.set_streaming_width(length);
    payload_.set_byte_enable_ptr(0);
    payload_.set_dmi_allowed(false);
    payload_.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
}


/************************************************************************
 * CPU Factory                                                          *
 ************************************************************************/

etiss_sc::CPUFactory::CPUFactory(const etiss_sc::Config &cfg, etiss::Initializer *etiss_init)
    : etiss_sc::Factory<ISS_CPU>(cfg, etiss_init)
{
}

void etiss_sc::CPUFactory::initParams()
{
    cpu_params_.etiss_init_ = etiss_init_;

    auto cpu_params = cfg_.getSubConfig("cpu");
    cpu_params_.num_irqs_ = cpu_params.get<size_t>("irqs");
    cpu_params_.name_ = cpu_params.get<std::string>("name");
}

void etiss_sc::CPUFactory::generate(sc_core::sc_module_name name)
{
    genHelper<ISS_CPU, CPUParams>(name, std::move(cpu_params_));
}
