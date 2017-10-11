#ifndef ATTPC_MERGERS_GRAWFILE_H
#define ATTPC_MERGERS_GRAWFILE_H

#include <fstream>
#include <string>
#include "attpc/mergers/GRAWFrame.h"

namespace attpc {
namespace mergers {

class GRAWFile {
public:
    GRAWFile(const std::string& filename, const bool readonly = true);
    GRAWFile(const GRAWFile&) = delete;
    GRAWFile(GRAWFile&&) = default;

    GRAWFrame readFrame();

    class FileReadError : public std::runtime_error {
    public:
        FileReadError() : std::runtime_error::runtime_error("GRAW file read error") {}
    };

private:
    std::fstream file;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWFILE_H */
