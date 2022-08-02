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
/// @file soc.cpp
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "etiss-sc/tlm/base/soc.h"

#define ID_ETISS_SC_SOC "etiss-sc: SoC"

etiss_sc::SoC::SoC(sc_core::sc_module_name name, etiss_sc::SoCParams &&soc_params)
    : sc_core::sc_module(name), soc_params_{ std::move(soc_params) }
{
}

const etiss_sc::CPUBase &etiss_sc::SoC::getCPU(std::string name) const
{
    CPUBase *cpu{ nullptr };
    try
    {
        cpu = cpus_.at(name).get();
    }
    catch (const std::out_of_range &)
    {
        SC_REPORT_FATAL(ID_ETISS_SC_SOC, "cpu not found in SoC::getCPU()");
    }

    return *cpu;
}

const etiss_sc::Mem *etiss_sc::SoC::getMem(std::string name) const
{
    Mem *mem{ nullptr };
    try
    {
        mem = mems_.at(name).get();
    }
    catch (const std::out_of_range &)
    {
        SC_REPORT_FATAL(ID_ETISS_SC_SOC, "mem not found in SoC::getMem()");
    }

    return mem;
}

sc_core::sc_signal<bool> *etiss_sc::SoC::getIRQ(std::string cpu_name, size_t num)
{
    CPUBase *cpu{ nullptr };
    try
    {
        cpu = cpus_.at(cpu_name).get();
    }
    catch (const std::out_of_range &)
    {
        SC_REPORT_FATAL(ID_ETISS_SC_SOC, "cpu not found in SoC::getIRQ()");
    }

    return &(irqs_[cpu][num]);
}

void etiss_sc::SoC::setup()
{
    auto bus_name = soc_params_.bus_params_.name_;
    auto cpu_name = soc_params_.cpu_factory_.get()->get_name();

    addBus(bus_name, std::move(soc_params_.bus_params_));
    addCPU(soc_params_.cpu_factory_.get(), bus_name);

    const CPUBase *cpu{ nullptr };
    try
    {
        cpu = &getCPU(cpu_name);
    }
    catch (const std::out_of_range &)
    {
        XREPORT_FATAL("cpu not found in SoC::setup()");
    }
    irqs_[cpu] = std::vector<sc_core::sc_signal<bool>>(cpu->getNumIRQs());
    auto &irq = irqs_[cpu];
    for (size_t i = 0; i < irq.size(); ++i)
    {
        cpu->bindIRQ(i, irq[i]);
    }

    std::string range = etiss::cfg().get<std::string>("vp.elf_file", "");

    if (range != "")
    {
        std::vector<std::string> elf_file_paths;
        size_t pos = 0;
        std::string delimiter = ",";
        std::string elf_file_path;

        do
        {
            pos = range.find(delimiter);
            elf_file_path = range.substr(0, pos);
            range.erase(0, pos + delimiter.length());
            if (elf_file_path != "")
            {
                try
                {
                    auto elf_reader = std::make_unique<ELFIO::elfio>();
                    elf_reader->load(elf_file_path);
                    elf_readers_.push_back(std::move(elf_reader));
                }
                catch (const std::out_of_range &)
                {
                    XREPORT_FATAL("elf_reader couldn't load elf file in SoC::setup()");
                }
            }
        } while (pos != std::string::npos);
    }
    else
    {
        XREPORT_FATAL("No ELF files found");
    }
}

void etiss_sc::SoC::addBus(std::string name, BusParams &&bus_params)
{
    std::unique_ptr<etiss_sc::Bus> bus{ nullptr };
    bus = std::make_unique<etiss_sc::Bus>(name.c_str(), std::move(bus_params));
    buses_.insert(std::make_pair(name, std::move(bus)));
}

const etiss_sc::CPUBase *etiss_sc::SoC::addCPU(CPUFactory *cpu_factory, std::string bus_name)
{
    auto cpu_name = soc_params_.cpu_factory_.get()->get_name();
    auto cpu = cpu_factory->create(cpu_name.c_str());
    cpu->rst_i_(this->rst_i_);

    Bus *bus{ nullptr };
    try
    {
        bus = (buses_.at(bus_name)).get();
    }
    catch (const std::out_of_range &)
    {
        XREPORT_FATAL("bus not found in SoC::addCPU()");
    }
    bus->connectMaster(cpu->instr_sock_i_.get());
    bus->connectMaster(cpu->data_sock_i_.get());

    cpus_.insert(std::make_pair(cpu_name, std::move(cpu)));
    return cpus_[cpu_name].get();
}

const etiss_sc::Mem *etiss_sc::SoC::addMem(std::string name, MemParams &&mem_params, std::string bus_name)
{
    auto mem = std::make_unique<etiss_sc::Mem>(name.c_str(), std::move(mem_params));
    mem->rst_i_(this->rst_i_);
    burnMem(mem.get());

    Bus *bus{ nullptr };
    try
    {
        bus = buses_.at(bus_name).get();
    }
    catch (const std::out_of_range &)
    {
        SC_REPORT_FATAL(ID_ETISS_SC_SOC, "bus not found in SoC::addMem()");
    }
    bus->connectSlave(mem->sock_t_.get(), mem_params.base_addr_, mem_params.base_addr_ + mem_params.size_);

    mems_.insert(std::make_pair(name, std::move(mem)));
    return mems_[name].get();
}

void etiss_sc::SoC::burnMem(Mem *mem)
{
    std::vector<uint8_t> data{};
    data.resize(mem->params_.size_);

    for (auto &elf_reader : elf_readers_)
    {
        for (auto &seg : elf_reader->segments)
        {
            if (seg->get_physical_address() >= mem->params_.base_addr_ &&
                seg->get_physical_address() < (mem->params_.base_addr_ + mem->params_.size_))
            {
                auto seg_data{ seg->get_data() };
                for (size_t i = 0; i < seg->get_file_size(); ++i)
                {
                    data[(seg->get_physical_address() - mem->params_.base_addr_) + i] =
                        static_cast<uint8_t>(seg_data[i] & 0xff);
                }
            }
        }
    }
    mem->loadDataFile(data);
}

etiss_sc::SoCFactory::SoCFactory(const etiss_sc::Config &cfg, etiss::Initializer *etiss_init)
    : etiss_sc::Factory<SoC>(cfg, etiss_init)
{
}

void etiss_sc::SoCFactory::initParams()
{
    soc_params_.etiss_init_ = etiss_init_;

    auto bus_params = cfg_.getSubConfig("bus");
    soc_params_.bus_params_.name_ = bus_params.get<std::string>("name");
    soc_params_.bus_params_.num_masters_ = bus_params.get<size_t>("masters");
    soc_params_.bus_params_.num_slaves_ = bus_params.get<size_t>("slaves");
}

void etiss_sc::SoCFactory::generate(sc_core::sc_module_name name)
{
    //
}
