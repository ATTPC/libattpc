#include "ConfigStore.h"
#include <yaml-cpp/yaml.h>

namespace attpc {
namespace common {

ConfigStore::ConfigStore(const std::string& filename) {
    load(filename);
}

void ConfigStore::load(const std::string& filename) {
    config = YAML::LoadFile(filename);
}

boost::optional<ConfigStore> ConfigStore::getSubConfig(const std::string& name) const {
    YAML::Node subnode = config[name];
    if (subnode) {
        ConfigStore subconfig {};
        subconfig.config = subnode;
        return subconfig;
    }
    else {
        return boost::none;
    }
}

}
}
