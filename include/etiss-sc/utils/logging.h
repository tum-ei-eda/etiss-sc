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
/// @file logging.h
/// @date 2019-03-09
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_UTILS_LOGGING_H__
#define __ETISS_SC_UTILS_LOGGING_H__

#include "etiss/Misc.h"
#include "systemc"

namespace etiss_sc
{
void set_log_level(etiss::Verbosity log_level, bool use_logging_file = false);

} // namespace etiss_sc

#endif // __ETISS_SC_UTILS_LOGGING_H__
