#include "GRAWFile.h"
#include "utilities.h"
#include "RawFrame.h"

namespace {
using attpc::mergers::utilities::parseValue;
}

namespace attpc {
namespace mergers {

GRAWFile::GRAWFile(const std::string& filename, const bool readonly) {
    open(filename, readonly);
}

void GRAWFile::open(const std::string& filename, const bool readonly) {
    std::ios_base::openmode flags;
    if (readonly) {
        flags = std::ios_base::binary | std::ios_base::in;
    }
    else {
        flags = std::ios_base::binary | std::ios_base::in | std::ios_base::out;
    }

    file = std::fstream(filename, flags);
}

void GRAWFile::close() {
    file.close();
}

GRAWFrame GRAWFile::readFrame() {
    using FrameSizeField = decltype(GRAWHeader::frameSize);

    const auto start = file.tellg();
    file.seekg(decltype(GRAWHeader::frameSize)::offset, std::ios_base::cur);
    RawFrame rawSize (FrameSizeField::size);
    file.read(reinterpret_cast<char*>(rawSize.getDataPtr()), rawSize.size());
    FrameSizeField::type frameSize = parseValue<FrameSizeField::type>(rawSize.begin(), rawSize.end());
    file.seekg(start);

    RawFrame rawFrame (frameSize * 256);
    file.read(reinterpret_cast<char*>(rawFrame.getDataPtr()), rawFrame.size());

    return GRAWFrame{rawFrame};
}

}
}
