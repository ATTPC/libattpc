#include "attpc/mergers/GRAWFile.h"
#include "attpc/mergers/utilities.h"
#include "attpc/mergers/RawFrame.h"

namespace {
using attpc::mergers::utilities::parseValue;
}

namespace attpc {
namespace mergers {

GRAWFile::GRAWFile(const std::string& filename, const bool readonly) {
    std::ios_base::openmode flags;
    if (readonly) {
        flags = std::ios_base::binary | std::ios_base::in;
    }
    else {
        flags = std::ios_base::binary | std::ios_base::in | std::ios_base::out;
    }

    file = std::fstream(filename, flags);
}

GRAWFrame GRAWFile::readFrame() {
    // NOTE: The reinterpret_casts in this function may or may not be a good idea, and may or may not
    // *technically* invoke undefined behavior, but they seem to work, and the internet suggests
    // that this is the canonical way to read data into a buffer of unsigned chars.

    using FrameSizeField = decltype(GRAWHeader::frameSize);

    const auto start = file.tellg();  // Remember where we were

    // Find the size of the next frame by reading that field from its header
    file.seekg(FrameSizeField::offset, std::ios_base::cur);
    RawFrame rawSize (FrameSizeField::size);
    if (!file.read(reinterpret_cast<char*>(rawSize.getDataPtr()), rawSize.size())) {
        throw FileReadError();
    }
    FrameSizeField::type frameSize = parseValue<FrameSizeField::type>(rawSize.begin(), rawSize.end());

    // Go back to the beginning of the frame, and then read the whole thing
    file.seekg(start);
    RawFrame rawFrame (frameSize * 256);  // The frameSize in the header is in units of 256 bytes
    if (!file.read(reinterpret_cast<char*>(rawFrame.getDataPtr()), rawFrame.size())) {
        throw FileReadError();
    }

    return GRAWFrame{rawFrame};
}

}
}
