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
/// @file plugins.h
/// @date 2019-03-06
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_UTILS_PLUGINS_H__
#define __ETISS_SC_UTILS_PLUGINS_H__

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include "systemc"

#include "etiss/ClassDefs.h"
#include "etiss/ETISS.h"
#include "etiss/jit/ReturnCode.h"
#include "etiss/IntegratedLibrary/InstructionSpecificAddressCallback.h"
#include "etiss/IntegratedLibrary/VariableValueLogger.h"
#include "etiss/CPUCore.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief A simple logger dedicated to print PC trace. Most accurate when Translation::MaxBlockSize
///        in ETISS.int is set to 1.
/// @param terminateAddr: Terminate simulation when hit certain address
/// @param [Optional] printOnScreen: Print PC on screen if true
/// @param [Optional] terminateAddr:Terminate simulation only when hit
///	       the address *terminateHit* times.
class TracePrinter : public etiss::CoroutinePlugin
{
  public:
    // ctor
    TracePrinter(etiss::uint32 terminateAddr, bool printOnScreen = false, char terminateHit = 1)
        : terminateAddr_(terminateAddr), printOnScreen_(printOnScreen), terminateHit_(terminateHit), CoroutinePlugin()
    {
    }

    // dtor
    ~TracePrinter() { printLog(); }

    etiss::int32 execute()
    {
        if (printOnScreen_)
            std::cout << "[INFO] {TracePrinter} : PC = 0x"  << std::hex << cpu_->instructionPointer << std::dec << "|" << cpu_->instructionPointer << std::endl;

        pcTrace_ << "[INFO] {TracePrinter}: PC = 0x" << std::hex << cpu_->instructionPointer << std::dec << "|" << cpu_->instructionPointer << std::endl;

        if (this->cpu_->instructionPointer == terminateAddr_)
        {
            if (++hitTimes_ == terminateHit_)
            {
                printLog();
                return etiss::RETURNCODE::CPUFINISHED;
            }
        }
        return etiss::RETURNCODE::NOERROR;
    }
    std::string _getPluginName() const { return std::string("TracePrinter"); }

    void init(ETISS_CPU *cpu, ETISS_System *system, etiss::CPUArch *arch)
    {
        this->cpu_ = cpu;
        this->system_ = system;
        this->arch_ = arch;
    }

    void cleanup()
    {
        cpu_ = nullptr;
        system_ = nullptr;
        arch_ = nullptr;
    }

  private:
    ETISS_CPU *cpu_;
    ETISS_System *system_;
    etiss::CPUArch *arch_;

    std::stringstream pcTrace_;
    char hitTimes_ = 0;
    bool printOnScreen_;
    const etiss::uint32 terminateAddr_;
    const char terminateHit_;

    void printLog()
    {
        std::ofstream outPcTrace;
        outPcTrace.open("Trace.log");
        outPcTrace << pcTrace_.str();
        outPcTrace.close();
    }
};

void addPcTraceLogger(std::shared_ptr<etiss::CPUCore> cpu, std::string pc_trace_filename = "",
                      unsigned int startlogging_at_us = 0, unsigned int stoplogging_at_us = INT_MAX);

class InjectorCallback : public etiss::plugin::InstructionSpecificAddressCallback
{
  public:
    virtual bool callback() { return plugin_core_->getStruct()->instructionAccurateCallback(plugin_cpu_->cpuTime_ps); }
};
void addBootAndInjectionCallback(std::shared_ptr<etiss::CPUCore> cpu);

class EXIT_ON_INFLOOP : public etiss::CoroutinePlugin
{
  private:
    uint64_t max_cycles = std::numeric_limits<int>::max();

  public:
    virtual etiss::int32 execute()
    {
        if (this->plugin_cpu_->cpuTime_ps >= max_cycles * this->plugin_cpu_->cpuCycleTime_ps)
        {
            std::cout << "Abort simulation: time exceed maximum: " << max_cycles
                      << " cycles = " << this->plugin_cpu_->cpuTime_ps << "ps at " << this->plugin_cpu_->cpuCycleTime_ps
                      << " ps/cycle" << std::endl;

            std::fstream pffs(etiss::cfg(getLastAssignedCoreName()).get<std::string>("resultpath", "") + "fault_type" +
                                  etiss::cfg(getLastAssignedCoreName()).get<std::string>("iteration", ""),
                              std::ios::in | std::ios::out);
            std::string line;
            getline(pffs, line);
            if (line.empty())
                line = "1;0;0;0;";
            else
                line.replace(0, 1, "1");

            pffs.clear();
            pffs.seekg(0, pffs.beg);
            pffs << line;
            pffs.flush();
            pffs.close();

            return etiss::RETURNCODE::CPUFINISHED;
        }
        return etiss::RETURNCODE::NOERROR;
    }

    void addedToCPUCore(etiss::CPUCore *core)
    {
        max_cycles = etiss::cfg(getLastAssignedCoreName())
                         .get<uint64_t>("faultInjection::EXIT_ON_CYCLE", std::numeric_limits<int>::max());
    }

    virtual std::string _getPluginName() const { return "EXIT_ON_INFLOOP"; }
};
void addExitOnInfloopPlugin(std::shared_ptr<etiss::CPUCore> cpu);

#endif // __ETISS_SC_UTILS_PLUGINS_H__
