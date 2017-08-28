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

}
}
