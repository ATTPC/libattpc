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

/**
 * @brief Controls a multi-threaded file merge procedure.
 *
 * This class is, unfortunately, rather complicated. Its purpose is to coordinate the merging of a
 * set of GRAW files using multiple threads of execution. It therefore contains a lot of machinery to
 * synchronize threads and generate tasks for those threads.
 *
 * The main method of this class is mergeFiles. This merges the frames from the files you provide to it and
 * writes the results to the output file provided.
 */
class MergeManager {
public:
    /**
     * @brief Construct a MergeManager instance
     *
     * @param method                    The merging method. Chooses whether to merge by event ID or timestamp.
     * @param maxAccumulatorNumEvents_  The maxmimum number of events to keep in the accumulator. (e.g. 20) An event
     *                                  is built and written to disk when the accumulator has more events than this.
     * @param lookupPtr_                Pointer to the pad lookup table.
     * @param keepFPN_                  If true, keep the FPN data. If false, discard it.
     */
    MergeManager(MergeKeyFunction method, size_t maxAccumulatorNumEvents_,
                 std::shared_ptr<common::PadLookupTable> lookupPtr_ = nullptr, bool keepFPN_ = false);

    /**
     * @brief Merge the given GRAW files into the given output file.
     *
     * This is the main method of this class. It spawns threads to perform a multithreaded merge of the
     * input files. It can be called multiple times to merge multiple sets of files, although
     * it will create (and later destroy) a new set of threads each time it is called.
     *
     * @note This procedure will monitor the state of the flag controlled by the SIGINT signal handler. This
     * allows the merge to be interrupted using Control-C in the terminal. If interrupted, the merge will
     * end gracefully, all threads should be shut down correctly, and the output file should not be corrupted.
     * See the functions in signal_handling.h for more information.
     *
     * @param grawFiles The set of open GRAW files.
     * @param outFile   The open output file.
     *
     * @sa signal_handling.h
     */
    void mergeFiles(std::vector<GRAWFile>& grawFiles, common::HDF5DataFile& outFile);

private:
    //! A std::packaged_task for building events
    using BuildTask = std::packaged_task<common::FullTraceEvent()>;
    //! A std::packaged_task for writing built events to the output file
    using WriteTask = std::packaged_task<void()>;

    /**
     * @brief Main function used by the reader thread.
     *
     * This function is called in a separate thread to continuously read frames from the GRAW files into
     * the FrameAccumulator. It reads frames until all of the files have reached their ends, and then
     * it signals completion back to the parent thread by setting the value of the provided promise object.
     * Check for completion by checking the state of the std::future corresponding to the promise, or
     * block until reading is complete by calling `get` on the std::future.
     *
     * @param grawFiles   The set of open GRAW files to read. All frames will be read from all files.
     * @param doneReading A promise that will be set when reading is completed. Get the state of this
     *                    thread using the corresponding std::future object.
     */
    void readFiles(std::vector<GRAWFile>& grawFiles, std::promise<void>& doneReading);

    void waitUntilAccumIsFull(const std::function<bool()>& interruptFunc);

    /**
     * @brief Pulls groups of frames out of the accumulator and creates tasks for them.
     *
     * This is called in the main thread. When called, it waits for the FrameAccumulator to hold more events
     * than the given threshold. Then it locks the accumulator, pulls out an unmerged FrameVector, and creates
     * build and write tasks for that event. The tasks are pushed onto the build and write queues provided as
     * arguments. This is repeated until the size of the FrameAccumulator is below the threshold.
     *
     * The threshold can be set to 0 to completely drain the FrameAccumulator at the end of the merge.
     *
     * @param numToLeaveBehind The number of events to leave in the FrameAccumulator after this method finishes.
     * @param buildQueue       The queue of build tasks.
     * @param writeQueue       The queue of write tasks.
     * @param outFile          The output file.
     */
    void processAccumulatedFrames(
            size_t numToLeaveBehind,
            std::shared_ptr<ThreadsafeQueue<BuildTask>> buildQueue,
            std::shared_ptr<ThreadsafeQueue<WriteTask>> writeQueue,
            common::HDF5DataFile& outFile);
    /**
     * @brief Get a built event from a std::future and write it to the file.
     *
     * This will block the thread it's called on if the future is not ready.
     *
     * @param outFile The output file to which the event should be written.
     * @param future  The std::future that will contain the event.
     */
    void writeEventFromFuture(common::HDF5DataFile& outFile, std::shared_future<common::FullTraceEvent> future);

    FrameAccumulator accum;  //! The FrameAccumulator, which sorts frames by event
    std::mutex accumMutex;  //! Controls access to the accumulator between threads
    std::condition_variable accumCond;  //! Allows signalling between threads with respect to the accumulator
    std::mutex coutMutex;  //! Controls multithreaded access to std::cout
    size_t maxAccumulatorNumEvents;  //! The maximum number of events to keep in the accumulator
    Merger merger;  //! The merger instance, which builds the events and processes them
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_MERGEMANAGER_H */
