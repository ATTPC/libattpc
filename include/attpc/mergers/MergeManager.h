#ifndef ATTPC_MERGERS_MERGEMANAGER_H
#define ATTPC_MERGERS_MERGEMANAGER_H

#include <vector>
#include <memory>
#include "attpc/common/types.h"
#include "attpc/mergers/GRAWFile.h"
#include "attpc/mergers/FrameAccumulator.h"
#include "attpc/common/HDF5DataFile.h"
#include "attpc/mergers/Merger.h"
#include "attpc/common/PadLookupTable.h"

namespace attpc {
namespace mergers {

class MergeManager {
public:
    MergeManager(MergeKeyFunction method, size_t maxAccumulatorNumEvents_,
                 std::shared_ptr<common::PadLookupTable> lookupPtr_ = nullptr, bool keepFPN_ = false);

    void mergeFiles(std::vector<GRAWFile>& grawFiles, common::HDF5DataFile& outFile);

private:
    FrameAccumulator accum;
    size_t maxAccumulatorNumEvents;
    Merger merger;
};

extern "C" void mergerSignalHandler(int signal);

}
}

#endif /* end of include guard: ATTPC_MERGERS_MERGEMANAGER_H */
