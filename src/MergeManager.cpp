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

const std::vector<channelid_type> MergeManager::fpnChannels = {11, 22, 45, 56};

MergeManager::MergeManager(MergeKeyFunction method, size_t maxAccumulatorNumEvents_,
                           std::shared_ptr<common::PadLookupTable> lookupPtr_, bool keepFPN_)
: accum(method)
, maxAccumulatorNumEvents(maxAccumulatorNumEvents_)
, lookupPtr(std::move(lookupPtr_))
, keepFPN(keepFPN_)
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

    if (!keepFPN) {
        discardFPN(event);
    }
    if (lookupPtr) {
        setPadNumbers(event);
    }

    return event;
}

void MergeManager::discardFPN(common::FullTraceEvent& event) {
    auto newEnd = std::remove_if(event.begin(), event.end(), [](const common::Trace& tr) {
        // Predicate returns True if channel number is in the list of FPN channels.
        auto begin = MergeManager::fpnChannels.begin();
        auto end = MergeManager::fpnChannels.end();
        return std::find(begin, end, tr.getHardwareAddress().channel) != end;
    });
    event.erase(newEnd, event.end());
}

void MergeManager::setPadNumbers(common::FullTraceEvent& event) {
    if (lookupPtr) {
        for (common::Trace& trace : event) {
            trace.setPad(lookupPtr->find(trace.getHardwareAddress()));
        }
    }
}

extern "C" void mergerSignalHandler(int) {
    signalFlag = 1;
}

}
}
