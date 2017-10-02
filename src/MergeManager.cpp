#include "attpc/mergers/MergeManager.h"
#include <csignal>
#include <iostream>

namespace {
    volatile std::sig_atomic_t signalFlag = 0;
}

namespace attpc {
namespace mergers {

MergeManager::MergeManager(const std::vector<std::string>& grawPaths,
    const std::string& outPath, MergeKeyFunction method, size_t maxAccumulatorNumEvents_)
: outFile(outPath, common::HDF5DataFile::Mode::create)
, accum(method)
, maxAccumulatorNumEvents(maxAccumulatorNumEvents_)
{
    for (const std::string& path : grawPaths) {
        grawFiles.emplace_back(path);
    }
}

void MergeManager::mergeFiles() {
    while (signalFlag == 0) {
        std::cout << "Reading" << std::endl;
        for (auto& file : grawFiles) {
            for (int numFrames = 0; numFrames < 4; numFrames++) {
                accum.addFrame(file.readFrame());
            }
        }

        while (accum.size() > maxAccumulatorNumEvents) {
            std::cout << "Writing" << std::endl;
            FrameAccumulator::KeyType key;
            FrameAccumulator::FrameVector frames;
            std::tie(key, frames) = accum.extractOldest();

            common::FullTraceEvent event = merger.buildEvent(frames);

            outFile.write(event);
            std::cout << "Wrote event " << key << "\n";
        }
    }
}

extern "C" void mergerSignalHandler(int signal) {
    signalFlag = 1;
}

}
}
