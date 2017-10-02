#include "attpc/mergers/MergeManager.h"
#include <queue>
#include <csignal>
#include <iostream>
#include <algorithm>
#include <cassert>

namespace {
    volatile std::sig_atomic_t signalFlag = 0;
}

namespace attpc {
namespace mergers {

MergeManager::MergeManager(MergeKeyFunction method, size_t maxAccumulatorNumEvents_)
: accum(method)
, maxAccumulatorNumEvents(maxAccumulatorNumEvents_)
{}

void MergeManager::mergeFiles(std::vector<GRAWFile>& grawFiles, common::HDF5DataFile& outFile) {
    using grawIterType = decltype(grawFiles.begin());
    std::vector<grawIterType> finishedFiles;
    std::vector<grawIterType> unfinishedFiles;

    for (auto iter = grawFiles.begin(); iter != grawFiles.end(); ++iter) {
        unfinishedFiles.push_back(iter);
    }

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
            common::FullTraceEvent event = buildNextEvent();
            outFile.write(event);
            std::cout << "Wrote event " << event.getEventId() << "\n";
        }
    }

    while (accum.size() > 0) {
        common::FullTraceEvent event = buildNextEvent();
        outFile.write(event);
        std::cout << "Wrote event " << event.getEventId() << "\n";
    }
}

common::FullTraceEvent MergeManager::buildNextEvent() {
    FrameAccumulator::KeyType key;
    FrameAccumulator::FrameVector frames;
    std::tie(key, frames) = accum.extractOldest();

    common::FullTraceEvent event = merger.buildEvent(frames);

    return event;
}

extern "C" void mergerSignalHandler(int signal) {
    signalFlag = 1;
}

}
}
