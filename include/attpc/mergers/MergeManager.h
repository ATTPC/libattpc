#ifndef ATTPC_MERGERS_MERGEMANAGER_H
#define ATTPC_MERGERS_MERGEMANAGER_H

#include <vector>
#include "attpc/mergers/GRAWFile.h"
#include "attpc/mergers/FrameAccumulator.h"
#include "attpc/common/HDF5DataFile.h"
#include "attpc/mergers/Merger.h"

namespace attpc {
namespace mergers {

class MergeManager {
public:
    MergeManager(MergeKeyFunction method, size_t maxAccumulatorNumEvents_);

    void mergeFiles(std::vector<GRAWFile>& grawFiles, common::HDF5DataFile& outFile);

private:
    common::FullTraceEvent buildNextEvent();

    FrameAccumulator accum;
    size_t maxAccumulatorNumEvents;
    Merger merger;
};

extern "C" void mergerSignalHandler(int signal);

}
}

#endif /* end of include guard: ATTPC_MERGERS_MERGEMANAGER_H */
