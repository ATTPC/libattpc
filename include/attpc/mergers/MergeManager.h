#ifndef ATTPC_MERGERS_MERGEMANAGER_H
#define ATTPC_MERGERS_MERGEMANAGER_H

#include <vector>
#include <memory>
#include <mutex>
#include <future>
#include "attpc/common/types.h"
#include "attpc/mergers/GRAWFile.h"
#include "attpc/mergers/FrameAccumulator.h"
#include "attpc/common/HDF5DataFile.h"
#include "attpc/mergers/Merger.h"
#include "attpc/common/PadLookupTable.h"
#include "attpc/mergers/ThreadsafeQueue.h"

namespace attpc {
namespace mergers {

class MergeManager {
public:
    MergeManager(MergeKeyFunction method, size_t maxAccumulatorNumEvents_,
                 std::shared_ptr<common::PadLookupTable> lookupPtr_ = nullptr, bool keepFPN_ = false);

    void mergeFiles(std::vector<GRAWFile>& grawFiles, common::HDF5DataFile& outFile);

private:
    using BuildTask = std::packaged_task<common::FullTraceEvent()>;
    using WriteTask = std::packaged_task<void()>;

    void readFiles(std::vector<GRAWFile>& grawFiles, std::promise<void>& doneReading);
    void processAccumulatedFrames(
            size_t numToLeaveBehind,
            std::shared_ptr<ThreadsafeQueue<BuildTask>> buildQueue,
            std::shared_ptr<ThreadsafeQueue<WriteTask>> writeQueue,
            common::HDF5DataFile& outFile);
    void writeEventFromFuture(common::HDF5DataFile& outFile, std::shared_future<common::FullTraceEvent> future);

    FrameAccumulator accum;
    std::mutex accumMutex;
    std::condition_variable accumCond;
    std::mutex coutMutex;
    size_t maxAccumulatorNumEvents;
    Merger merger;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_MERGEMANAGER_H */
