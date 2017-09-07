#include "catch.hpp"
#include "HDF5DataFile.h"
#include "FullTraceEvent.h"
#include <Eigen/Core>
#include <cstdio>
#include <H5Cpp.h>
#include <string>

using namespace attpc::common;

namespace {

FullTraceEvent makeTestEvent() {
    FullTraceEvent evt {4, 5};
    for (Eigen::Index trIdx = 0; trIdx < 10; ++trIdx) {
        Trace::ArrayType trData = Trace::ArrayType::LinSpaced(512, trIdx, trIdx + 512);
        Trace tr (HardwareAddress(0, 0, 0, trIdx), trIdx + 1, trData);

        evt.insertTrace(std::move(tr));
    }

    return evt;
}

HDF5DataFile::EncodedEventArrayType makeTestArray() {
    HDF5DataFile::EncodedEventArrayType array {10, 512 + 5};
    for (Eigen::Index rowIdx = 0; rowIdx < array.rows(); ++rowIdx) {
        array(rowIdx, 0) = rowIdx;
        array(rowIdx, 1) = rowIdx % 4;
        array(rowIdx, 2) = (rowIdx + 1) % 4;
        array(rowIdx, 3) = (rowIdx + 2) % 4;
        array(rowIdx, 4) = rowIdx * 2;
        array(rowIdx, 5) = rowIdx * 10;

        for (Eigen::Index colIdx = 0; colIdx < array.cols(); ++colIdx) {
            array(rowIdx, colIdx) = colIdx + rowIdx;
        }
    }

    return array;
}

void compareEventAndArray(const FullTraceEvent& evt,
                          const Eigen::Ref<const HDF5DataFile::EncodedEventArrayType>& array) {
    for (auto trIter = evt.cbegin(); trIter != evt.end(); ++trIter) {
        Eigen::Index rowIdx = std::distance(evt.cbegin(), trIter);

        auto addr = trIter->getHardwareAddress();
        REQUIRE(array(rowIdx, 0) == addr.cobo);
        REQUIRE(array(rowIdx, 1) == addr.asad);
        REQUIRE(array(rowIdx, 2) == addr.aget);
        REQUIRE(array(rowIdx, 3) == addr.channel);

        REQUIRE(array(rowIdx, 4) == trIter->getPad().value());

        const auto& trData = trIter->getData();

        // This is ugly, but much easier than making the two have the same shape...
        for (Eigen::Index colIdx = 0; colIdx < 512; ++colIdx) {
            REQUIRE(array(rowIdx, colIdx + 5) == trData(colIdx));
        }
    }
}

}

TEST_CASE("Can encode FullTraceEvent into array", "[HDF5DataFile]") {
    FullTraceEvent evt = makeTestEvent();

    HDF5DataFile::EncodedEventArrayType array = HDF5DataFile::encodeEvent(evt);

    SECTION("Array has correct shape and ordering") {
        CHECK(array.rows() == evt.numTraces());
        CHECK(array.cols() == evt.begin()->getData().size() + 5);
        CHECK(array.IsRowMajor);
    }

    SECTION("Array contents are correct") {
        compareEventAndArray(evt, array);
    }
}

TEST_CASE("Can decode array into FullTraceEvent", "[HDF5DataFile]") {
    HDF5DataFile::EncodedEventArrayType array = makeTestArray();

    FullTraceEvent evt = HDF5DataFile::decodeEvent(array);

    SECTION("Event has correct number of traces") {
        REQUIRE(evt.numTraces() == array.rows());
    }

    SECTION("Event contents are correct") {
        compareEventAndArray(evt, array);
    }
}

TEST_CASE("Can write event to HDF5 file", "[HDF5DataFile]") {
    const std::string filename = "testdata.h5";
    HDF5DataFile datafile {filename, HDF5DataFile::Mode::truncate};

    // Make and write the event
    FullTraceEvent evt = makeTestEvent();
    datafile.write(evt);
    datafile.close();

    // Open using a plain HDF5 to test contents
    H5::H5File rawfile {filename, H5F_ACC_RDONLY};

    // Make sure the group was created
    REQUIRE(rawfile.exists("get"));
    H5::Group getGroup = rawfile.openGroup("get");

    // Make sure the dataset was created
    REQUIRE(getGroup.exists(std::to_string(evt.getEventId())));
    H5::DataSet dset = getGroup.openDataSet(std::to_string(evt.getEventId()));
    H5::DataSpace dspace = dset.getSpace();

    // Did the dataset have the right rank and dimensions?
    REQUIRE(dspace.getSimpleExtentNdims() == 2);
    hsize_t dims[2];
    dspace.getSimpleExtentDims(&dims[0]);
    REQUIRE(dims[0] == evt.numTraces());
    REQUIRE(dims[1] == evt.begin()->getData().size() + 5);

    HDF5DataFile::EncodedEventArrayType fileArray {dims[0], dims[1]};
    dset.read(fileArray.data(), H5::PredType::NATIVE_INT16);

    HDF5DataFile::EncodedEventArrayType evtArray = HDF5DataFile::encodeEvent(evt);  // For comparison

    CAPTURE(fileArray);
    CAPTURE(evtArray);

    // Is the data correct?
    REQUIRE((fileArray == evtArray).all());

    // Check that the timestamp attribute was created
    REQUIRE(dset.attrExists("timestamp"));
    H5::Attribute tsAttr = dset.openAttribute("timestamp");
    attpc::timestamp_type ts;
    tsAttr.read(H5::PredType::NATIVE_UINT64, &ts);
    REQUIRE(ts == evt.getTimestamp());
}

TEST_CASE("Can read event from HDF5 file", "[HDF5DataFile]") {
    const std::string filename = "testdata.h5";
    HDF5DataFile datafile {filename, HDF5DataFile::Mode::truncate};

    FullTraceEvent evt = makeTestEvent();
    datafile.write(evt);

    boost::optional<FullTraceEvent> readEvt = datafile.read(evt.getEventId());
    REQUIRE(bool{readEvt});  // Check it's not boost::none

    REQUIRE(*readEvt == evt);
}
