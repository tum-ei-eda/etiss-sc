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
/// @file plugins.cpp
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "etiss-sc/utils/plugins.h"

#define ID_ETISS_SC_PLUGINS "etiss-sc: plugins"

void addPcTraceLogger(std::shared_ptr<etiss::CPUCore> cpu, std::string pc_trace_filename,
                      unsigned int startlogging_at_us, unsigned int stoplogging_at_us)
{
    // variables to be captured by the lambda function
    const std::string &cpuName = (cpu->getName());
    ETISS_CPU *cpuState = cpu->getState();
    const bool write_to_stdout = pc_trace_filename == "";

    // setup VariableValueLogger, using a lambda function as callback function.
    // A lambda function can be used as a function pointer, whereas a functor
    // cannot, even if also lambda functions can have a state (via captures).
    std::shared_ptr<etiss::plugin::VariableValueLogger> vvl(new etiss::plugin::VariableValueLogger(
        "instructionPointer", pc_trace_filename,
        [cpuName, cpuState, write_to_stdout, startlogging_at_us,
         stoplogging_at_us](std::ostream &vvlout, const std::string &field, uint64_t pc_val) mutable
        {
            // static variable definitions
            // map:
            //    [start_pc] -> number of times this pc has been a jump
            //    destination (the beginning of a block)
            // (not really needed for fault generation but handy when
            // manually looking at pc trace files)
            static std::unordered_map<uint64_t, unsigned> pcount;
            static uint64_t last_block_start_ptr = 0;
            static uint64_t last_pc = 0;
            static bool first_call = true;

            // stdout or file?
            static std::ostream &out = write_to_stdout ? std::cout : vvlout;

            // If called the first time: write header
            if (unlikely(first_call))
            {
                out << cpuName << "::" << field << std::endl;
                first_call = false;
            }

            // If there is a jump => write block
            if (last_pc + 4 != pc_val)
            {
                // calculate block length of the block we just left:
                unsigned block_length = (last_pc - last_block_start_ptr) / 4 + 1;

                // write data of block, don't print reset values
                if ((last_block_start_ptr != 0) && (cpuState->cpuTime_ps >= (uint64_t)(startlogging_at_us)*1000000) &&
                    (cpuState->cpuTime_ps <= (uint64_t)(stoplogging_at_us)*1000000))
                {
                    out << std::hex << last_block_start_ptr << std::dec << ";" // pc
                        << block_length << ";"                                 // block length (in cmds, not byte!)
                        << pcount[last_block_start_ptr]                        // count for the first pc
                                                                               // of the block
                        << std::endl;
                }

                // set last values for next iteration
                last_block_start_ptr = pc_val;
            }

            // count the number of times the current pc values has been called.
            // This counts all executions of the pc,
            // not just the times it has been a jump destination.
            // This must be done after writing the block data, otherwise the
            // count is one to high if the current block has the same start
            // address as the last one.
            unsigned &count = pcount[pc_val];
            count++;

            // set last values for next iteration
            last_pc = pc_val;

            // std::cout << "pc: " << std::hex << pc_val << std::dec <<
            // std::endl;
        }));

    cpu->addPlugin(vvl);
}

void addBootAndInjectionCallback(std::shared_ptr<etiss::CPUCore> cpu)
{
    InjectorCallback *cb = new InjectorCallback();
    cpu->addPlugin(std::shared_ptr<etiss::Plugin>(cb));

    cpu->getStruct()->acceleratedTrigger_ = [cb](const etiss::fault::Trigger &t, int32_t id)
    {
        etiss::fault::Trigger const *endTrigger = &t;
        // select sub Trigger if Trigger is a meta counter
        while (endTrigger->getType() == +etiss::fault::Trigger::type_t::META_COUNTER)
            endTrigger = &endTrigger->getSubTrigger();

        // only variable value trigger can be handled with this callback
        // for time trigger an other callback type must be choosen e.g.
        // InstructionAccurateCallback -> But this has a perfomance leck of a
        // factor by circa 10, cause it injects for every instruction a callback
        if (endTrigger->getType() == +etiss::fault::Trigger::type_t::VARIABLEVALUE)
        {
            cb->addCallbackAddress(t.getTriggerFieldValue());
            std::stringstream ss;
            ss << "Added callback for instruction 0x" << std::hex << t.getTriggerFieldValue() << std::dec << std::endl;
            SC_REPORT_INFO(ID_ETISS_SC_PLUGINS, ss.str().c_str());
        }
        return false; // Trigger is not handeled internally. Injector must handle
                      // it.
    };
}

void addExitOnInfloopPlugin(std::shared_ptr<etiss::CPUCore> cpu)
{
    cpu->addPlugin(std::shared_ptr<etiss::Plugin>(new EXIT_ON_INFLOOP()));
}
