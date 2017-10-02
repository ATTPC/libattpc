#ifndef ATTPC_MERGERS_MERGEMANAGER_H
#define ATTPC_MERGERS_MERGEMANAGER_H

#include <vector>
#include <string>
#include "attpc/mergers/GRAWFile.h"
#include "attpc/mergers/FrameAccumulator.h"
#include "attpc/common/HDF5DataFile.h"
#include "attpc/mergers/Merger.h"

namespace attpc {
namespace mergers {

class MergeManager {
public:
    MergeManager(const std::vector<std::string>& grawPaths, const std::string& outPath, MergeKeyFunction method,
                 size_t maxAccumulatorNumEvents_);

    void mergeFiles();

private:
    std::vector<GRAWFile> grawFiles;
    common::HDF5DataFile outFile;
    FrameAccumulator accum;
    size_t maxAccumulatorNumEvents;
    Merger merger;
};

extern "C" void mergerSignalHandler(int signal);

}
}

#endif /* end of include guard: ATTPC_MERGERS_MERGEMANAGER_H */
