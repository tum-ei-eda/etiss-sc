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
/// @file config.h
/// @date 2019-03-09
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __ETISS_SC_UTILS_CONFIG_H__
#define __ETISS_SC_UTILS_CONFIG_H__

#include <string>
#include <unordered_map>
#include <stdexcept>
#include <sstream>
#include <iostream>

#include <boost/lexical_cast.hpp>

namespace etiss
{
class Initializer;
class Configuration;
} // namespace etiss

namespace etiss_sc
{

/// Assigns command line arguments for ETISS.
void SetCmdlineArgs(int argc, char *argv[]);

/// Returns the ETISS initializer object. It is created if required.
etiss::Initializer *GetETISSInit();

/// Returns the configuration object of ETISS.
etiss::Configuration &GetETISSConfig();

/**
 * Represents a configuration consisting of key-value-pairs.
 * Final config values are determined as follows:
 * 1.) Take the default value set with applyDefaults.
 * 2.) Take the value defined by the config file that was loaded.
 * 3.) Take the value that was overwritten in code by "set".
 * Steps 2.) and 3.) can be exchanged or mixed depending on when loadFromFile
 * and set are called.
 *
 * Supported configuration types:
 * - String (std::string): A normal string. All keys are strings. ("hello
 *world")
 * - Integer (int, unsigned int, uint32_t, ...): Any integer value. (15, 0xff)
 * - Float (float, double): Any floating point value. (3.14)
 * - Boolean (bool): False if integer 0 or string "false". Otherwise true.
 **/
class Config
{
  public:
    /// Loads all configs from the given file path. The file must be in INI
    /// format.
    void loadFromFile(const std::string &path);

    /// Copies every setting from the given config to the returned one, but only
    /// if the key is not contained already.
    Config applyDefaults(const Config &cfg) const;

    /// Assigns value to specific config.
    template <typename T>
    void set(const std::string &key, const T &value);

    /// Return the value for the given key.
    template <typename T = std::string>
    T get(const std::string &key) const;

    /// Returns a new Config instance that contains all configs at top-level that
    /// start with "key" in this config.
    Config getSubConfig(const std::string &key) const;

    /// Prints all config values to stdout.
    void print() const;

  private:
    std::unordered_map<std::string, std::string> keyToValue_;
    std::string dbgTrace_;
};

typedef enum cast_type
{
    INTEGRAL = 0,
    FLOATING,
    BOOLEAN,
    STRING
} cast_type_t;

template <typename T>
constexpr cast_type_t switch_type()
{
    if (std::is_same<T, bool>::value)
        return BOOLEAN;
    if (std::is_integral<T>::value || std::is_enum<T>::value)
        return INTEGRAL;
    if (std::is_floating_point<T>::value)
        return FLOATING;
    return STRING;
}

// partial templates for constexpr return types
template <typename T, cast_type_t CT>
struct to_T;
template <typename T, cast_type_t CT>
struct from_T;

template <typename T>
void Config::set(const std::string &key, const T &value)
{
    keyToValue_[key] = from_T<T, switch_type<T>()>::make(value);
    ;
}

template <typename T>
T Config::get(const std::string &key) const
{
    auto itValue = keyToValue_.find(key);
    if (itValue == keyToValue_.end())
    {
        throw std::runtime_error("Config value '" + dbgTrace_ + key + "' does not exist");
    }
    auto &value = itValue->second;
    return to_T<T, switch_type<T>()>::make(value);
}

template <typename T>
struct from_T<T, INTEGRAL>
{
    static std::string make(const T &value) { return boost::lexical_cast<std::string>(value); }
};
template <typename T>
struct from_T<T, FLOATING>
{
    static std::string make(const T &value) { return boost::lexical_cast<std::string>(value); }
};
template <typename T>
struct from_T<T, BOOLEAN>
{
    static std::string make(const T &value) { return value ? "true" : "false"; }
};
template <typename T>
struct from_T<T, STRING>
{
    static std::string make(const T &value) { return value; }
};
template <typename T>
struct to_T<T, INTEGRAL>
{
    static T make(const std::string &value) {
        T ret;
        try {
            ret = boost::lexical_cast< T >(value);
        }
        catch(boost::bad_lexical_cast const& e)
        {    
            if (value.find("0x") == std::string::npos)
            {
                std::cout << "Error (INTEGRAL type): " << e.what() << "\n" << value << " to " << typeid(ret).name() << std::endl;
                throw e; // exception can not be ignored throw again
            }
        }
        return ret; 
    }
};
template <typename T>
struct to_T<T, FLOATING>
{
    static T make(const std::string &value) { return (T)boost::lexical_cast<double>(value); }
};
template <typename T>
struct to_T<T, BOOLEAN>
{
    static T make(const std::string &value) { return (T)(value != "0" || value != "false"); }
};
template <typename T>
struct to_T<T, STRING>
{
    static T make(const std::string &value) { return value; }
};

} // namespace etiss_sc

#endif // __ETISS_SC_UTILS_CONFIG_H__
