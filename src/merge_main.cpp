#include "attpc/mergers/MergeManager.h"
#include "attpc/common/PadLookupTable.h"
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <csignal>
#include <memory>

namespace po = boost::program_options;

struct MergerArgs {
    std::vector<std::string> inputPaths;
    std::string outputPath;
    attpc::mergers::MergeKeyFunction method;
    boost::optional<std::string> padLookupTablePath;
};

attpc::mergers::MergeKeyFunction parseMethodArg(const std::string& argString) {
    if (argString == "id") {
        return attpc::mergers::GetMergeKeyFromEventId;
    }
    else if (argString == "timestamp") {
        return attpc::mergers::GetMergeKeyFromTimestamp;
    }
    else {
        throw std::runtime_error("Invalid merge method provided.");
    }
}

MergerArgs parseOptions(const int argc, const char** argv) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("method,m", po::value<std::string>(), "merging method")
        ("lookup,l", po::value<std::string>(), "path to pad lookup table as CSV file")
        ("output,o", po::value<std::string>()->default_value("output.h5"), "path to output file")
        ("input", po::value<std::vector<std::string>>(), "path to input file")
    ;

    po::positional_options_description pos;
    pos.add("input", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        std::exit(0);
    }

    MergerArgs args;
    args.inputPaths = vm["input"].as<std::vector<std::string>>();
    args.outputPath = vm["output"].as<std::string>();
    args.method = parseMethodArg(vm["method"].as<std::string>());
    if (vm.count("lookup")) {
        args.padLookupTablePath = vm["lookup"].as<std::string>();
    }

    return args;
}

std::vector<attpc::mergers::GRAWFile> openGRAWFiles(const std::vector<std::string>& paths) {
    std::vector<attpc::mergers::GRAWFile> files;
    for (const auto& path : paths) {
        files.emplace_back(path);
    }
    return files;
}

int main(const int argc, const char** argv) {
    using std::cout;

    MergerArgs options = parseOptions(argc, argv);

    std::signal(SIGINT, attpc::mergers::mergerSignalHandler);

    std::shared_ptr<attpc::common::PadLookupTable> lookupPtr;
    if (options.padLookupTablePath) {
        lookupPtr = std::make_shared<attpc::common::PadLookupTable>(*options.padLookupTablePath);
    }

    std::vector<attpc::mergers::GRAWFile> grawFiles = openGRAWFiles(options.inputPaths);
    attpc::common::HDF5DataFile outFile {options.outputPath, attpc::common::HDF5DataFile::Mode::create};

    attpc::mergers::MergeManager mgr {options.method, 20, lookupPtr};
    mgr.mergeFiles(grawFiles, outFile);
}
