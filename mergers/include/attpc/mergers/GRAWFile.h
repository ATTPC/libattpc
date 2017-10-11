#ifndef ATTPC_MERGERS_GRAWFILE_H
#define ATTPC_MERGERS_GRAWFILE_H

#include <fstream>
#include <string>
#include "attpc/mergers/GRAWFrame.h"

namespace attpc {
namespace mergers {

/**
 * @brief Interface to a GRAW file.
 *
 * This can be used to read frames from a GRAW file in order.
 *
 * In terms of implementation, this class is only responsible for reading raw frames from the file.
 * The GRAWFrame, GRAWHeader, etc. classes do the actual unpacking and parsing of the data.
 */
class GRAWFile {
public:
    /**
     * @brief Open a GRAWFile
     * @param filename The path to the file
     * @param readonly Whether to open the file in read-only mode
     */
    GRAWFile(const std::string& filename, const bool readonly = true);
    //! GRAWFiles cannot be copied
    GRAWFile(const GRAWFile&) = delete;
    //! Move the GRAWFile
    GRAWFile(GRAWFile&&) = default;

    /**
     * @brief Read the next frame from the file
     *
     * This reads a frame from the file starting at the current file position. As noted above, the
     * actual processing of the frame is done by the GRAWFrame class and its components, such as the
     * GRAWHeader.
     *
     * @return The processed frame
     *
     * @throws FileReadError If there is an error when reading the raw frame from the file.
     */
    GRAWFrame readFrame();

    //! Indicates an error while reading raw data from the GRAWFile.
    class FileReadError : public std::runtime_error {
    public:
        FileReadError() : std::runtime_error::runtime_error("GRAW file read error") {}
    };

private:
    std::fstream file;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWFILE_H */
