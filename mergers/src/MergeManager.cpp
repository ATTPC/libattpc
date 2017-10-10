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
    /**
     * @brief Check if the result has been set in the future object.
     *
     * If the future is ready, calling future::get will not block.
     *
     * @param  future A future object.
     * @return        True if ready.
     */
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
    /* This method manages the list of files to read by shuffling them between three vectors.
     * (Well, technically we move around iterators into `grawFiles` instead, but the idea is the same.)
     * Initially, all files begin in the `unfinishedFiles` vector. At the beginning of a frame read
     * cycle, all files from this vector are moved to another called `filesToProcess`. A frame is
     * read from each file in this vector. If the read succeeds, the file is put back into the
     * `unfinishedFiles` vector. Otherwise, it is put into the `finishedFiles` vector instead. Once
     * all files are in the `finishedFiles` vector, reading is complete.
     */
    using grawIterType = decltype(grawFiles.begin());  // The type of an iterator to into `grawFiles`
    std::vector<grawIterType> finishedFiles;           // Files we've finished reading
    std::vector<grawIterType> unfinishedFiles;         // Files we haven't finished reading yet

    // All files are unfinished at the beginning
    for (auto iter = grawFiles.begin(); iter != grawFiles.end(); ++iter) {
        unfinishedFiles.push_back(iter);
    }

    while (!abortWasCalled() && !unfinishedFiles.empty()) {
        assert(finishedFiles.size() + unfinishedFiles.size() == grawFiles.size()); // Make sure we didn't lose any files

        std::vector<grawIterType> filesToProcess;
        std::move(unfinishedFiles.begin(), unfinishedFiles.end(), std::back_inserter(filesToProcess));
        unfinishedFiles.clear();  // std::move leaves the objects in the source container in a moved-from state

        assert(finishedFiles.size() + filesToProcess.size() == grawFiles.size());  // We've still got them all

        // Lock the accumulator mutex in this scope
        {
            std::unique_lock<std::mutex> accumLock {accumMutex};
            // Wait until the accumulator is not full, then unblock and add frames
            accumCond.wait(accumLock, [this](){ return accum.size() < maxAccumulatorNumEvents; });

            for (auto file : filesToProcess) {
                try {
                    accum.addFrame(file->readFrame());
                }
                catch (const GRAWFile::FileReadError&) {
                    // Frame read failed, so this file is finished
                    finishedFiles.push_back(file);
                    continue;
                }
                // Frame read succeeded if we got here, so this file is not finished yet
                unfinishedFiles.push_back(file);
            }

            accumCond.notify_all();  // Tell waiting threads that we've added new frames
        }
    }

    doneReading.set_value();  // Set the value of the promise so the main thread can know we're done here
    accumCond.notify_all();   // Must unblock any waiting threads before returning
}

void MergeManager::processAccumulatedFrames(
        size_t numToLeaveBehind,
        std::shared_ptr<ThreadsafeQueue<BuildTask>> buildQueue,
        std::shared_ptr<ThreadsafeQueue<WriteTask>> writeQueue,
        common::HDF5DataFile& outFile
) {
    auto accumFullPredicate = [this, numToLeaveBehind](){ return accum.size() >= numToLeaveBehind; };

    std::unique_lock<std::mutex> accumLock {accumMutex};
    accumCond.wait(accumLock, accumFullPredicate);  // Wait if the accumulator is not full

    while (accumFullPredicate()) {
        FrameAccumulator::KeyType key;
        FrameAccumulator::FrameVector frames;
        std::tie(key, frames) = accum.extractOldest();

        // Create the tasks and put them on the task queues
        BuildTask btask {std::bind(&Merger::mergeAndProcessEvent, &merger, std::move(frames))};
        WriteTask wtask {std::bind(&MergeManager::writeEventFromFuture, this, std::ref(outFile),
                         btask.get_future().share())};  // This future must be shared since packaged_task copies it
        writeQueue->push(std::move(wtask));
        buildQueue->push(std::move(btask));
    }

    accumCond.notify_all();  // Tell waiting threads that we've removed events from the accumulator
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
    std::promise<void> doneReading;  // Lets the reader tell us when it's done

    // Create task queues
    auto buildQueue = std::make_shared<ThreadsafeQueue<BuildTask>>();
    auto writeQueue = std::make_shared<ThreadsafeQueue<WriteTask>>();

    // Spawn worker threads. `reader` will begin working immediately, but the others will
    // wait until there are tasks on the queues.
    GuardedThread reader {std::bind(&MergeManager::readFiles, this, std::ref(grawFiles), std::ref(doneReading))};
    Worker<BuildTask> builder {buildQueue};
    Worker<WriteTask> writer {writeQueue};

    // This thread keeps track of the accumulator. If it gets full, take some events out of it
    // and create build and write tasks for them.
    auto doneReadingFuture = doneReading.get_future();  // This will be "ready" when the reader is done
    while (!abortWasCalled() && !future_is_ready(doneReadingFuture)) {
        processAccumulatedFrames(maxAccumulatorNumEvents, buildQueue, writeQueue, outFile);
    }

    doneReadingFuture.get();  // Barrier. Requires reader to be finished before continuing.

    // If the merge wasn't aborted, finish the remaining tasks and write all events to the file.
    // Otherwise, abandon the remaining events and quit quickly.
    const bool shouldWriteRemaining = !abortWasCalled();
    if (shouldWriteRemaining) {
        processAccumulatedFrames(0, buildQueue, writeQueue, outFile);
    }
    else {
        std::unique_lock<std::mutex> coutLock {coutMutex};
        std::cout << "Aborting merge. Remaining events will not be written.\n";
    }

    // Block until the queues are finished. The threads will be joined automatically.
    buildQueue->finish(shouldWriteRemaining);
    writeQueue->finish(shouldWriteRemaining);
}

}
}
