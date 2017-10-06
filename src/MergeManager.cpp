#include "attpc/mergers/MergeManager.h"
#include "attpc/mergers/ThreadsafeQueue.h"
#include "attpc/mergers/Worker.h"
#include <queue>
#include <csignal>
#include <iostream>
#include <algorithm>
#include <cassert>
#include <mutex>
#include <future>
#include <functional>
#include <memory>

namespace {
    volatile std::sig_atomic_t signalFlag = 0;

    std::mutex coutMutex;

    void writeEvent(attpc::common::HDF5DataFile& file, std::shared_future<attpc::common::FullTraceEvent> future) {
        auto event = future.get();
        file.write(event);
        {
            std::unique_lock<std::mutex> coutLock {coutMutex};
            std::cout << "Wrote event " << event.getEventId() << std::endl;
        }
    }
}

namespace attpc {
namespace mergers {

MergeManager::MergeManager(MergeKeyFunction method, size_t maxAccumulatorNumEvents_,
                           std::shared_ptr<common::PadLookupTable> lookupPtr_, bool keepFPN_)
: accum(method)
, maxAccumulatorNumEvents(maxAccumulatorNumEvents_)
, merger(lookupPtr_, keepFPN_)
{}

void MergeManager::mergeFiles(std::vector<GRAWFile>& grawFiles, common::HDF5DataFile& outFile) {
    using grawIterType = decltype(grawFiles.begin());
    std::vector<grawIterType> finishedFiles;
    std::vector<grawIterType> unfinishedFiles;

    for (auto iter = grawFiles.begin(); iter != grawFiles.end(); ++iter) {
        unfinishedFiles.push_back(iter);
    }

    using BuildTask = std::packaged_task<common::FullTraceEvent()>;
    auto buildQueue = std::make_shared<ThreadsafeQueue<BuildTask>>();
    Worker<BuildTask> builder {buildQueue};

    using WriteTask = std::packaged_task<void()>;
    auto writeQueue = std::make_shared<ThreadsafeQueue<WriteTask>>();
    Worker<WriteTask> writer {writeQueue};

    while (signalFlag == 0 && !unfinishedFiles.empty()) {
        assert(finishedFiles.size() + unfinishedFiles.size() == grawFiles.size());

        std::vector<grawIterType> filesToProcess;
        std::move(unfinishedFiles.begin(), unfinishedFiles.end(), std::back_inserter(filesToProcess));
        unfinishedFiles.clear();

        assert(finishedFiles.size() + filesToProcess.size() == grawFiles.size());

        for (auto file : filesToProcess) {
            try {
                accum.addFrame(file->readFrame());
            }
            catch (const GRAWFile::FileReadError&) {
                finishedFiles.push_back(file);
                continue;
            }

            unfinishedFiles.push_back(file);
        }

        while (accum.size() > maxAccumulatorNumEvents) {
            FrameAccumulator::KeyType key;
            FrameAccumulator::FrameVector frames;
            std::tie(key, frames) = accum.extractOldest();

            BuildTask btask {std::bind(&Merger::mergeAndProcessEvent, &merger, std::move(frames))};
            WriteTask wtask {std::bind(&writeEvent, std::ref(outFile), btask.get_future().share())};
            writeQueue->push(std::move(wtask));
            buildQueue->push(std::move(btask));
        }
    }

    while (accum.size() > 0) {
        FrameAccumulator::KeyType key;
        FrameAccumulator::FrameVector frames;
        std::tie(key, frames) = accum.extractOldest();

        BuildTask btask {std::bind(&Merger::mergeAndProcessEvent, &merger, std::move(frames))};
        WriteTask wtask {std::bind(&writeEvent, std::ref(outFile), btask.get_future().share())};
        writeQueue->push(std::move(wtask));
        buildQueue->push(std::move(btask));
    }

    buildQueue->finish();
    writeQueue->finish();
}

extern "C" void mergerSignalHandler(int) {
    signalFlag = 1;
}

}
}
