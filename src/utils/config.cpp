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

#include "etiss-sc/utils/config.h"

#include "etiss-sc/utils/logging.h"

#include "SimpleIni.h"
#include <etiss/ETISS.h>
#include <cassert>

static bool g_argsset = false;
static int g_argc = 0;
static char **g_argv = nullptr;

void etiss_sc::SetCmdlineArgs(int argc, char *argv[])
{
    g_argsset = true;
    g_argc = argc;
    g_argv = argv;
}

etiss::Initializer *etiss_sc::GetETISSInit()
{
    assert(g_argsset && "Must set cmdline args with etiss_sc::SetCmdlineArgs");
    static etiss::Initializer init{ g_argc, g_argv };

    // TODO what is the best place for this?
    etiss_sc::set_log_level(etiss::verbosity());

    return &init;
}

etiss::Configuration &etiss_sc::GetETISSConfig()
{
    (void)GetETISSInit();
    return etiss::cfg();
}

void etiss_sc::Config::loadFromFile(const std::string &path)
{
    CSimpleIni ini(true, true, true);
    if (ini.LoadFile(path.c_str()) < 0)
    {
        throw std::runtime_error(std::string("could not load config file: ") + path);
    }
    CSimpleIni::TNamesDepend keys;
    if (ini.GetAllKeys("etiss_sc", keys))
    {
        for (const auto &key : keys)
        {
            CSimpleIni::TNamesDepend values;
            if (ini.GetAllValues("etiss_sc", key.pItem, values))
            {
                if (values.size() != 1)
                {
                    throw std::runtime_error("multiple values not allowed for single key");
                }
                set(key.pItem, values.front().pItem);
            }
        }
    }
}

etiss_sc::Config etiss_sc::Config::applyDefaults(const Config &cfg) const
{
    Config defaultCfg = *this;
    for (const auto &k2v : cfg.keyToValue_)
    {
        if (keyToValue_.count(k2v.first) == 0)
        {
            defaultCfg.set(k2v.first, k2v.second);
        }
    }
    return defaultCfg;
}

etiss_sc::Config etiss_sc::Config::getSubConfig(const std::string &key) const
{
    auto keyPrefix = key + ".";
    Config subCfg;
    for (const auto &k2v : keyToValue_)
    {
        if (!k2v.first.compare(0, keyPrefix.size(), keyPrefix))
        {
            subCfg.set(k2v.first.substr(keyPrefix.size()), k2v.second);
        }
    }
    subCfg.dbgTrace_ = dbgTrace_ + keyPrefix;
    return subCfg;
}

void etiss_sc::Config::print() const
{
    for (const auto &k2v : keyToValue_)
    {
        printf("%s = %s\n", k2v.first.c_str(), k2v.second.c_str());
    }
}
