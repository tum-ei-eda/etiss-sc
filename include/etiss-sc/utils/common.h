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
/// @file common.h
/// @date 2019-03-09
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_UTILS_COMMON_H__
#define __ETISS_SC_UTILS_COMMON_H__

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

#include "etiss/ETISS.h"
#include "systemc"

#include "etiss-sc/utils/config.h"

#define ID_ETISS_SC_FACTORY "etiss-sc: Factory"

namespace etiss_sc
{
inline void swap(unsigned char &a, unsigned char &b)
{
    auto tmp = a;
    a = b;
    b = tmp;
}

inline void flipEndianness(unsigned char *ptr, unsigned len = 4)
{
    for (size_t i = 0; i < len / 2; ++i)
    {
        swap(ptr[i], ptr[(len - 1) - i]);
    }
}

template <class base_t>
class Factory
{
  public:
    explicit Factory(const etiss_sc::Config &cfg, etiss::Initializer *etiss_init) : etiss_init_{ etiss_init }, cfg_(cfg)
    {
    }

    virtual std::unique_ptr<base_t> create(sc_core::sc_module_name name)
    {
        initParams();
        generate(name);
        prod_->setup();

        return std::move(prod_);
    }

  protected:
    std::unique_ptr<base_t> prod_{ nullptr };
    etiss::Initializer *etiss_init_{ nullptr };
    const etiss_sc::Config &cfg_;
    virtual void initParams() = 0;
    virtual void generate(sc_core::sc_module_name name) = 0;

    template <class prod_t, class... Ts>
    void genHelper(sc_core::sc_module_name name, Ts &&...ts)
    {
        prod_ = std::make_unique<prod_t>(name, std::forward<Ts>(ts)...);
    }
};

template <typename argvT, typename... lisT>
class CommandLineParser
{
    std::filesystem::path temp_directory_path_;
    std::filesystem::path temp_config_file_path_;
    std::vector<std::string> parsed{};
    std::vector<char *> parsed_{};

    struct
    {
        std::string etiss_cfg_fpath_;
        std::string vp_cfg_fpath_;
        std::string log_fpath_;
    } options_{};

    static std::filesystem::path make_temp_directory_path()
    {
        auto base = std::filesystem::temp_directory_path();
        auto seed = static_cast<std::mt19937_64::result_type>(
            std::chrono::high_resolution_clock::now().time_since_epoch().count());
        std::mt19937_64 generator(seed);

        for (int attempt = 0; attempt < 128; ++attempt)
        {
            auto candidate = base / ("etiss-sc-" + std::to_string(generator()));
            std::error_code ec;
            if (std::filesystem::create_directory(candidate, ec))
            {
                return candidate;
            }
        }

        throw std::runtime_error("failed to create a temporary directory for etiss-sc");
    }

    static bool parse_named_option(const std::string &arg, std::string &name, std::string &value, bool &has_inline_value)
    {
        if (arg.rfind("--", 0) != 0 || arg.size() <= 2)
        {
            return false;
        }

        auto option = arg.substr(2);
        auto equals = option.find('=');
        if (equals == std::string::npos)
        {
            name = option;
            value.clear();
            has_inline_value = false;
        }
        else
        {
            name = option.substr(0, equals);
            value = option.substr(equals + 1);
            has_inline_value = true;
        }
        return true;
    }

    static void print_help()
    {
        std::cout << "Allowed options\n"
                  << "  --help              produce help message\n"
                  << "  --etiss <path>      etiss configuration file\n"
                  << "  --vp <path>         virtual prototype configuration file\n"
                  << "  --elfs <value>      ELF files to load into soc memory\n"
                  << "  --tgdb <value>      TCP port to host gdb server\n"
                  << "  --log <path>        optional log file path for vp reports" << std::endl;
    }

  public:
    char **get_parsed(void) { return parsed_.data(); }
    size_t get_parsed_size(void) { return parsed_.size(); }

    std::string get_etiss_config_path(void) { return std::string("-i") + temp_config_file_path_.string(); }
    std::string get_vp_config_path(void) { return options_.vp_cfg_fpath_; }
    std::string get_log_file_path(void) { return options_.log_fpath_; }

    CommandLineParser(int argc, argvT **argv, lisT... args_append)
        : temp_directory_path_(make_temp_directory_path())
    {
        (void)sizeof...(args_append);
        temp_config_file_path_ = temp_directory_path_ / "config.ini";
        std::string elf_file_paths;
        std::string remote_debug_port;
        std::cout << "temporaries:" << temp_config_file_path_ << std::endl;

        static_assert(std::is_same<const char, argvT>::value || std::is_same<char, argvT>::value,
                      "argv must be of type const char or char");

        if (argc > 0)
        {
            parsed.emplace_back(argv[0]);
        }

        bool show_help = false;
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--")
            {
                for (++i; i < argc; ++i)
                {
                    parsed.emplace_back(argv[i]);
                }
                break;
            }

            std::string name;
            std::string value;
            bool has_inline_value = false;
            if (!parse_named_option(arg, name, value, has_inline_value))
            {
                parsed.push_back(arg);
                continue;
            }

            auto require_value = [&](std::string &target) {
                if (has_inline_value)
                {
                    target = value;
                    return;
                }
                if (i + 1 >= argc)
                {
                    throw std::runtime_error("missing value for option '--" + name + "'");
                }
                target = argv[++i];
            };

            if (name == "help")
            {
                show_help = true;
            }
            else if (name == "etiss")
            {
                require_value(options_.etiss_cfg_fpath_);
            }
            else if (name == "vp")
            {
                require_value(options_.vp_cfg_fpath_);
            }
            else if (name == "elfs")
            {
                require_value(elf_file_paths);
            }
            else if (name == "tgdb")
            {
                require_value(remote_debug_port);
            }
            else if (name == "log")
            {
                require_value(options_.log_fpath_);
            }
            else
            {
                parsed.push_back(arg);
            }
        }

        if (show_help)
        {
            print_help();
        }
        else
        {
            std::cout << "ETISS configuration file: " << options_.etiss_cfg_fpath_ << std::endl;
            std::cout << "virtual protoype configuration file: " << options_.vp_cfg_fpath_ << std::endl;
            std::cout << "ELF files to load into soc memory: " << elf_file_paths << std::endl;
            std::cout << "TGDB TCP port [-1:deactivated]: " << remote_debug_port << std::endl;
        }

        std::ifstream etiss_base_cfg_file(options_.etiss_cfg_fpath_);
        if (etiss_base_cfg_file.is_open())
        {
            std::ofstream temp_config_file(temp_config_file_path_);
            if (temp_config_file.is_open())
            {
                std::string line;
                while (std::getline(etiss_base_cfg_file, line))
                {
                    temp_config_file << line << std::endl;
                }
                etiss_base_cfg_file.close();

                if (elf_file_paths != "")
                {
                    temp_config_file << "[StringConfigurations]" << std::endl
                                     << " vp.elf_file=" << elf_file_paths << std::endl;
                }

                if (remote_debug_port != "")
                {
                    temp_config_file << "[Plugin gdbserver]" << std::endl << " plugin.gdbserver.port=" << remote_debug_port << std::endl;
                }

                temp_config_file.close();

                parsed.push_back(get_etiss_config_path());
            }
        }

        parsed_.reserve(parsed.size());
        for (size_t i = 0; i < parsed.size(); ++i)
        {
            parsed_.push_back(const_cast<char *>(parsed[i].c_str()));
        }
    }

    virtual ~CommandLineParser(void)
    {
        // remove temporary directory if still exists
        std::error_code ec;
        std::filesystem::remove_all(temp_directory_path_, ec);
    }
};

} // namespace etiss_sc

#endif // __ETISS_SC_UTILS_COMMON_H__
