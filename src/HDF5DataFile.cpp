#include "HDF5DataFile.h"

namespace {

using H5Mode = decltype(H5F_ACC_TRUNC);

H5Mode getH5AccessMode(const attpc::common::HDF5DataFile::Mode mode) {
    using Mode = attpc::common::HDF5DataFile::Mode;
    switch (mode) {
        case Mode::readonly:
            return H5F_ACC_RDONLY;
        case Mode::readwrite:
            return H5F_ACC_RDWR;
        case Mode::truncate:
            return H5F_ACC_TRUNC;
        case Mode::create:
            return H5F_ACC_EXCL;
    }
}

}

namespace attpc {
namespace common {

HDF5DataFile::HDF5DataFile(const std::string& filename, const Mode mode)
{
    H5Mode hmode = getH5AccessMode(mode);
    file = H5::H5File(filename, hmode);
}

boost::optional<FullTraceEvent> HDF5DataFile::read(const evtid_type eventId, const std::string& groupName) {
    H5::Group group = file.openGroup(groupName);

    const std::string datasetName = std::to_string(eventId);
    H5::DataSet dataset;
    if (group.exists(datasetName)) {
        group.openDataSet(datasetName);
    }
    else {
        return boost::none;
    }

    H5::DataSpace dataspace = dataset.getSpace();
    if (dataspace.getSimpleExtentNdims() != 2) {
        throw BadData("Invalid number of dimensions in event read from HDF5 file.");
    }
    hsize_t dims[2];
    dataspace.getSimpleExtentDims(&dims[0]);

    const int expectedNumCols = 512 + 5;
    if (dims[1] != expectedNumCols) {
        throw BadData("Event in HDF5 file had incorrect number of columns. Should be 517.");
    }

    Eigen::Array<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data {dims[0], dims[1]};
    dataset.read(data.data(), H5::PredType::NATIVE_INT16);

    FullTraceEvent event {};
    event.setEventId(eventId);
    for (Eigen::Index rowNum = 0; rowNum < data.rows(); ++rowNum) {
        const coboid_type    cobo    = data(rowNum, 0);
        const asadid_type    asad    = data(rowNum, 1);
        const agetid_type    aget    = data(rowNum, 2);
        const channelid_type channel = data(rowNum, 3);
        const padid_type     pad     = data(rowNum, 4);
        const HardwareAddress addr {cobo, asad, aget, channel};

        Trace tr {addr, pad, data.block<1, expectedNumCols - 5>(rowNum, 5)};

        bool wasInserted;
        std::tie(std::ignore, wasInserted) = event.insertTrace(std::move(tr));
        if (!wasInserted) {
            throw BadData("Traces with duplicate hardware addresses present in event.");
        }
    }

    return event;
}

void HDF5DataFile::write(const FullTraceEvent& event, const std::string& groupName) {
    const Eigen::Index numRows = event.numTraces();
    const Eigen::Index numCols = 512 + 5;  // num time buckets + (cobo, asad, aget, channel, pad)

    // The data must be put in a row-major array since HDF5 writes data in a row-major format
    Eigen::Array<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data {numRows, numCols};

    for (auto iter = event.cbegin(); iter != event.cend(); ++iter) {
        Eigen::Index rowNum = std::distance(event.cbegin(), iter);
        const auto& traceAddr = iter->getHardwareAddress();
        const boost::optional<padid_type>& tracePad = iter->getPad();
        const Eigen::ArrayXi& traceData = iter->getData();

        data(rowNum, 0) = traceAddr.cobo;
        data(rowNum, 1) = traceAddr.asad;
        data(rowNum, 2) = traceAddr.aget;
        data(rowNum, 3) = traceAddr.channel;
        data(rowNum, 4) = tracePad.value_or(-1);
        data.block(rowNum, 5, 1, numCols - 5) = traceData;
    }

    const int dataspaceRank = 2;
    const hsize_t dataspaceDims[2] = {static_cast<hsize_t>(numRows), numCols};
    H5::DataSpace dataspace (dataspaceRank, dataspaceDims);

    H5::Group group = openOrCreateGroup(groupName);

    const std::string datasetName = std::to_string(event.getEventId());

    H5::DataSet dataset = group.createDataSet(datasetName, H5::PredType::NATIVE_INT16, dataspace);
    dataset.write(data.data(), H5::PredType::NATIVE_INT16);
}

H5::Group HDF5DataFile::openOrCreateGroup(const std::string& groupName) {
    if (file.exists(groupName)) {
        return file.openGroup(groupName);
    }
    else {
        return file.createGroup(groupName);
    }
}

}
}
