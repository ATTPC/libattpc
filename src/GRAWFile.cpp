#include "GRAWFile.h"
#include "utilities.h"

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
    const auto start = file.tellg();
    file.seekg(decltype(GRAWHeader::frameSize)::offset, std::ios_base::cur);
    std::vector<uint8_t> rawSize (decltype(GRAWHeader::frameSize)::size);
    file.read(reinterpret_cast<char*>(rawSize.data()), rawSize.size());
    uint32_t frameSize = parseValue<uint32_t>(rawSize.begin(), rawSize.end());
    file.seekg(start);

    std::vector<uint8_t> rawFrame (frameSize * 256);
    file.read(reinterpret_cast<char*>(rawFrame.data()), rawFrame.size());

    return GRAWFrame(rawFrame);
}

}
}
