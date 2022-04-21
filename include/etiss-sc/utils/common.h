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

#include <memory>
#include <utility>

#include "etiss/ETISS.h"
#include "systemc"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
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
    boost::filesystem::path temp_directory_path_;
    boost::filesystem::path temp_config_file_path_;
    std::vector<std::string> parsed{};
    std::vector<char *> parsed_{};

    struct
    {
        std::string etiss_cfg_fpath_;
        std::string vp_cfg_fpath_;
        std::string log_fpath_;
    } options_{};

  public:
    char **get_parsed(void) { return &(parsed_[0]); }
    size_t get_parsed_size(void) { return parsed_.size(); }

    std::string get_etiss_config_path(void) { return std::string("-i") + temp_config_file_path_.c_str(); }
    std::string get_vp_config_path(void) { return options_.vp_cfg_fpath_; }
    std::string get_log_file_path(void) { return options_.log_fpath_; }

    CommandLineParser(int argc, argvT **argv, lisT... args_append)
        : temp_directory_path_(boost::filesystem::temp_directory_path() / boost::filesystem::unique_path())
    {
        temp_config_file_path_ = temp_directory_path_ / "config.ini";
        std::string elf_file_paths;
        std::string remote_debug_port;
        std::cout << "temporaries:" << temp_config_file_path_ << std::endl;

        boost::filesystem::create_directories(temp_directory_path_);
        boost::program_options::options_description desc{ "Allowed options" };
        boost::program_options::variables_map vm{};

        desc.add_options()("help", "produce help message")(
            "etiss", boost::program_options::value<std::string>(&(options_.etiss_cfg_fpath_))->default_value(""),
            "etiss configuration file")(
            "vp", boost::program_options::value<std::string>(&(options_.vp_cfg_fpath_))->default_value(""),
            "virtual prototype configuration file")(
            "elfs", boost::program_options::value<std::string>(&elf_file_paths)->default_value(""),
            "ELF files to load into soc memory")(
            "tgdb", boost::program_options::value<std::string>(&remote_debug_port)->default_value(""),
            "TCP port to host gdb server")(
            "log", boost::program_options::value<std::string>(&options_.log_fpath_)->default_value(""),
            "optional log file path for vp reports");

        static_assert(std::is_same<const char, argvT>::value || std::is_same<char, argvT>::value,
                      "argv must be of type const char or char");

        boost::program_options::parsed_options po =
            boost::program_options::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
        boost::program_options::store(po, vm);

        parsed = boost::program_options::collect_unrecognized(po.options, boost::program_options::include_positional);
        parsed.insert(parsed.begin(), std::string(argv[0])); // add opt 0 = exectuable back

        boost::program_options::notify(vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
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
            std::ofstream temp_config_file(temp_config_file_path_.c_str());
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
            parsed_.push_back(const_cast<char *>(parsed[i].c_str()));
    }

    virtual ~CommandLineParser(void)
    {
        // remove temporary directory if still exists
        if (boost::filesystem::exists(temp_directory_path_))
        {
            boost::filesystem::remove_all(temp_directory_path_);
        }
    }
};

} // namespace etiss_sc

#endif // __ETISS_SC_UTILS_COMMON_H__
