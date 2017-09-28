#ifndef ATTPC_COMMON_CONFIGSTORE_H
#define ATTPC_COMMON_CONFIGSTORE_H

#include <string>
#include <boost/optional.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include "yaml_conversions.h"

namespace attpc {
namespace common {

/**
 * @brief A container for configuration options that can be read from a YAML file.
 *
 * This class can read a YAML config file and fetch the options stored in it. A file can be
 * read using ConfigStore::load, or by calling the constructor that takes a string argument.
 * Once a file is loaded, options from that file can be fetched using ConfigStore::getValue.
 * If the config file contains any sub-nodes (dictionary-like keys that contain a set of keys),
 * these can be parsed as sub-configurations using ConfigStore::getSubConfig. This returns another
 * ConfigStore object.
 *
 * Values are returned using boost::optional objects. These wrappers allow for missing values in the
 * config file. A boost::optional object can be tested using `if` to check if the value was present in
 * the config file:
 *
 *     ConfigStore config {"/path/to/file.yml"};
 *     boost::optional<int> value = config.getValue<int>("integer_option");
 *     if (value) {
 *         // Value was present in config file
 *         someFunction(*value);  // Use the value by dereferencing it with `*`
 *     }
 *     else {
 *         // Value was not present in config file. Dereferencing it would cause undefined behavior.
 *     }
 *
 * This also provides a simple way to assign a default value for a parameter:
 *
 *     ConfigStore config {"/path/to/file.yml"};
 *     // Use boost::optional::value_or to set a default value if the parameter
 *     // is not present in the config file.
 *     int value = config.getValue<int>("integer_option").value_or(15);
 *
 * Parameters can be interpreted as different types by providing the desired type as the template parameter
 * for ConfigStore::getValue:
 *
 *     ConfigStore config {"/path/to/file.yml"};
 *     boost::optional<int> intValue = config.getValue<int>("some_option");
 *     boost::optional<float> floatValue = config.getValue<float>("some_option");  // Same value, interpreted as float
 *
 */
class ConfigStore {
public:
    /**
     * @brief Default constructor
     *
     * The object built with this constructor will have no configuration values associated with it. You can read
     * a configuration into a default-constructed object with ConfigStore::load.
     */
    ConfigStore() = default;

    /**
     * @brief Construct a ConfigStore by reading options from the given YAML file.
     *
     * The file is parsed using ConfigStore::load.
     *
     * @param filename Path to a YAML file.
     */
    ConfigStore(const std::string& filename);

    /**
     * @brief Load the configuration stored in the given YAML file.
     *
     * Any values stored in the object will be replaced by the values in the file. In effect,
     * this reinitializes the ConfigStore object.
     *
     * @param filename Path to a YAML file.
     */
    void load(const std::string& filename);

    /**
     * @brief Get a value from the config file.
     *
     * The value will be interpreted as whatever type is given as the template argument. Most
     * basic types should work, and additional conversions for, e.g., Eigen arrays are defined
     * in yaml_conversions.h.
     *
     * @param  name The name of the option in the config file.
     * @return      The value, wrapped in a boost::optional type. This will be boost::none if the
     *              value was not present in the config file.
     */
    template <class T>
    boost::optional<T> getValue(const std::string& name) const {
        auto subnode = config[name];
        if (subnode) {
            return subnode.as<T>();
        }
        else {
            return boost::none;
        }
    }

    /**
     * @brief Get a sub-node from the config file.
     *
     * Imagine you have a config file that looks like this:
     *
     * ~~~{.yml}
     * # File config.yml
     * int_option: 5
     * float_option: 4.6
     * option_collection:
     *     another_int: 4
     *     another_float: 3.8
     * ~~~
     *
     * The sub-node `option_collection` can be retrieved using this method.
     *
     * ~~~{.cpp}
     * ConfigStore config {"config.yml"};
     * boost::optional<ConfigStore> subconfig = config.getSubConfig("option_collection");
     * ~~~
     *
     * @param  name The name of the sub-node.
     * @return      The sub-node, as another ConfigStore object, wrapped in a boost::optional.
     */
    boost::optional<ConfigStore> getSubConfig(const std::string& name) const;

private:
    YAML::Node config;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_CONFIGSTORE_H */
