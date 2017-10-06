#include "attpc/mergers/MergeManager.h"
#include "attpc/mergers/ThreadsafeQueue.h"
#include "attpc/mergers/Worker.h"
#include "attpc/mergers/GuardedThread.h"
#include "attpc/mergers/signal_handling.h"
#include <queue>
#include <iostream>
#include <algorithm>
#include <cassert>
#include <future>
#include <functional>
#include <memory>
#include <chrono>

namespace {
    template <class R>
    bool future_is_ready(const std::future<R>& future) {
        using namespace std::literals::chrono_literals;
        return future.wait_for(0s) == std::future_status::ready;
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

void MergeManager::readFiles(std::vector<GRAWFile>& grawFiles, std::promise<void>& doneReading) {
    using grawIterType = decltype(grawFiles.begin());
    std::vector<grawIterType> finishedFiles;
    std::vector<grawIterType> unfinishedFiles;

    for (auto iter = grawFiles.begin(); iter != grawFiles.end(); ++iter) {
        unfinishedFiles.push_back(iter);
    }

    while (!abortWasCalled() && !unfinishedFiles.empty()) {
        assert(finishedFiles.size() + unfinishedFiles.size() == grawFiles.size());

        std::vector<grawIterType> filesToProcess;
        std::move(unfinishedFiles.begin(), unfinishedFiles.end(), std::back_inserter(filesToProcess));
        unfinishedFiles.clear();

        assert(finishedFiles.size() + filesToProcess.size() == grawFiles.size());

        {
            std::unique_lock<std::mutex> accumLock {accumMutex};
            accumCond.wait(accumLock, [this](){ return accum.size() < maxAccumulatorNumEvents; });

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

            accumCond.notify_all();
        }
    }

    doneReading.set_value();
    accumCond.notify_all();
}

void MergeManager::processAccumulatedFrames(
        size_t numToLeaveBehind,
        std::shared_ptr<ThreadsafeQueue<BuildTask>> buildQueue,
        std::shared_ptr<ThreadsafeQueue<WriteTask>> writeQueue,
        common::HDF5DataFile& outFile
) {
    auto accumFullPredicate = [this, numToLeaveBehind](){ return accum.size() >= numToLeaveBehind; };

    std::unique_lock<std::mutex> accumLock {accumMutex};
    accumCond.wait(accumLock, accumFullPredicate);

    while (accumFullPredicate()) {
        FrameAccumulator::KeyType key;
        FrameAccumulator::FrameVector frames;
        std::tie(key, frames) = accum.extractOldest();

        BuildTask btask {std::bind(&Merger::mergeAndProcessEvent, &merger, std::move(frames))};
        WriteTask wtask {std::bind(&MergeManager::writeEventFromFuture, this, std::ref(outFile), btask.get_future().share())};
        writeQueue->push(std::move(wtask));
        buildQueue->push(std::move(btask));
    }

    accumCond.notify_all();
}

void MergeManager::writeEventFromFuture(attpc::common::HDF5DataFile& file, std::shared_future<attpc::common::FullTraceEvent> future) {
    auto event = future.get();
    file.write(event);
    {
        std::unique_lock<std::mutex> coutLock {coutMutex};
        std::cout << "Wrote event " << event.getEventId() << std::endl;
    }
}

void MergeManager::mergeFiles(std::vector<GRAWFile>& grawFiles, common::HDF5DataFile& outFile) {
    std::promise<void> doneReading;
    GuardedThread reader {std::bind(&MergeManager::readFiles, this, std::ref(grawFiles), std::ref(doneReading))};

    auto buildQueue = std::make_shared<ThreadsafeQueue<BuildTask>>();
    Worker<BuildTask> builder {buildQueue};

    auto writeQueue = std::make_shared<ThreadsafeQueue<WriteTask>>();
    Worker<WriteTask> writer {writeQueue};

    auto doneReadingFuture = doneReading.get_future();
    while (!abortWasCalled() && !future_is_ready(doneReadingFuture)) {
        processAccumulatedFrames(maxAccumulatorNumEvents, buildQueue, writeQueue, outFile);
    }

    doneReadingFuture.get();

    const bool shouldWriteRemaining = !abortWasCalled();

    if (shouldWriteRemaining) {
        processAccumulatedFrames(0, buildQueue, writeQueue, outFile);
    }
    else {
        std::unique_lock<std::mutex> coutLock {coutMutex};
        std::cout << "Aborting merge. Remaining events will not be written.\n";
    }

    buildQueue->finish(shouldWriteRemaining);
    writeQueue->finish(shouldWriteRemaining);
}

}
}
