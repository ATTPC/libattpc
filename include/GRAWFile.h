#ifndef ATTPC_MERGERS_GRAWFILE_H
#define ATTPC_MERGERS_GRAWFILE_H

#include <fstream>
#include <string>
#include "GRAWFrame.h"

namespace attpc {
namespace mergers {

class GRAWFile {
public:
    GRAWFile(const std::string& filename, const bool readonly = true);

    void open(const std::string& filename, const bool readonly = true);
    void close();

    GRAWFrame readFrame();

private:
    std::fstream file;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWFILE_H */
