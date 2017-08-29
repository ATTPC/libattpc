#ifndef ATTPC_COMMON_CONFIGSTORE_H
#define ATTPC_COMMON_CONFIGSTORE_H

#include <string>
#include <boost/optional.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include "yaml_conversions.h"

namespace attpc {
namespace common {

class ConfigStore {
public:
    ConfigStore() = default;
    ConfigStore(const std::string& filename);

    void load(const std::string& filename);

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

private:
    YAML::Node config;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_CONFIGSTORE_H */
