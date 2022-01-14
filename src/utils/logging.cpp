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
/// @file logging.cpp
/// @date 2019-03-10
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "etiss-sc/utils/logging.h"
#include <systemc>

using namespace sc_core;

void etiss_sc::set_log_level(etiss::Verbosity log_level, bool use_logging_file)
{
    uint8_t base_flag = SC_DISPLAY;
    if (use_logging_file)
    {
        base_flag |= SC_LOG;
        sc_report_handler::set_log_file_name("etiss_sc_logging");
    }
    switch (log_level)
    {
    case etiss::SILENT:
        sc_report_handler::set_actions(SC_INFO, SC_DO_NOTHING);
        sc_report_handler::set_actions(SC_WARNING, SC_DO_NOTHING);
        sc_report_handler::set_actions(SC_ERROR, SC_DO_NOTHING);
        sc_report_handler::set_actions(SC_FATAL, SC_ABORT);
        break;

    case etiss::FATALERROR:
        sc_report_handler::set_actions(SC_INFO, SC_DO_NOTHING);
        sc_report_handler::set_actions(SC_WARNING, SC_DO_NOTHING);
        sc_report_handler::set_actions(SC_ERROR, SC_DO_NOTHING);
        sc_report_handler::set_actions(SC_FATAL, SC_LOG | SC_DISPLAY | SC_ABORT);
        break;

    case etiss::ERROR:
        sc_report_handler::set_actions(SC_INFO, SC_DO_NOTHING);
        sc_report_handler::set_actions(SC_WARNING, SC_DO_NOTHING);
        sc_report_handler::set_actions(SC_ERROR, SC_LOG | SC_DISPLAY);
        sc_report_handler::set_actions(SC_FATAL, SC_LOG | SC_DISPLAY | SC_ABORT);
        break;

    case etiss::WARNING:
        sc_report_handler::set_actions(SC_INFO, SC_DO_NOTHING);
        sc_report_handler::set_actions(SC_WARNING, SC_LOG | SC_DISPLAY);
        sc_report_handler::set_actions(SC_ERROR, SC_LOG | SC_DISPLAY);
        sc_report_handler::set_actions(SC_FATAL, SC_LOG | SC_DISPLAY | SC_ABORT);
        break;

    case etiss::INFO:
        sc_report_handler::set_actions(SC_INFO, SC_LOG | SC_DISPLAY);
        sc_report_handler::set_actions(SC_WARNING, SC_LOG | SC_DISPLAY);
        sc_report_handler::set_actions(SC_ERROR, SC_LOG | SC_DISPLAY);
        sc_report_handler::set_actions(SC_FATAL, SC_LOG | SC_DISPLAY | SC_ABORT);
        break;

    case etiss::VERBOSE:
        sc_report_handler::set_actions(SC_INFO, SC_LOG | SC_DISPLAY);
        sc_report_handler::set_actions(SC_WARNING, SC_LOG | SC_DISPLAY);
        sc_report_handler::set_actions(SC_ERROR, SC_LOG | SC_DISPLAY);
        sc_report_handler::set_actions(SC_FATAL, SC_LOG | SC_DISPLAY | SC_ABORT);
        break;
    };
}
