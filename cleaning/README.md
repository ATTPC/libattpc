# libattpc-cleaning

This repository contains the data cleaning components of the AT-TPC data analysis library. Currently, this
includes two cleaning algorithms: one based on the Hough transform, and one based on hierarchical clustering.

## Citations

The linear Hough transform algorithm, in its current form, was originally described by [Duda and Hart]. It was adapted
for use with circles by [Illingworth and Kittler], although the version used in this library was based on the
description given by [Heinze]. The application of the Hough transform in the context of cleaning AT-TPC spirals has been
described briefly by [Ayyad et al], and will be described more thoroughly in a future publication.

The hierarchical clustering algorithm used in this library was developed by Lukas Aymans as part of his
master's thesis in the Faculty of Electrical Engineering and Computer Science at the [Hochschule Niederrhein].
Questions about this algorithm should be directed to him.

[Duda and Hart]: https://dx.doi.org/10.1145/361237.361242
[Illingworth and Kittler]: https://dx.doi.org/10.1109/TPAMI.1987.4767964
[Heinze]: https://dx.doi.org/10.3204/DESY-THESIS-2013-055
[Ayyad et al]: https://dx.doi.org/10.1088/1742-6596/876/1/012003
[Hochschule Niederrhein]: https://www.hs-niederrhein.de/home-en/

## Building the code

This code is intended to be used as part of the larger [libattpc](https://github.com/attpc/libattpc) library;
however, it can also be compiled independently if desired. Compilation requires the following dependencies:

- A C++ compiler that supports C++14 and OpenMP 3.1. (GCC 4.8+ and Clang 3.9+ should work)
- [CMake] 3.1+
- [Point Cloud Library (PCL)] 1.2+
- [Eigen] 3.3+
- [Doxygen] (optional, for building documentation)

To build the library, first clone the repository. Then make a build directory, run CMake, and compile the code:

```shell
git clone https://github.com/attpc/libattpc-cleaning
cd libattpc-cleaning
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..  # See the note about this below
make
```

When running CMake, it is important to specify the build type. Two build types are supported: `Release` and `Debug`. In
a `Release` build, all assertions are disabled, and compiler optimizations are enabled. Therefore, a `Release` build
should always be used when performing data analysis since it will run significantly faster. However, a `Debug` build
should be used when testing the code since the assertions it enables will check for out-of-bounds errors and other
mistakes. The build type is provided to CMake with the `-DCMAKE_BUILD_TYPE` flag as shown above.

[CMake]: https://cmake.org/
[Point Cloud Library (PCL)]: http://pointclouds.org/
[Eigen]: http://eigen.tuxfamily.org/
[Doxygen]: http://www.doxygen.org

## Running tests

This library includes a suite of unit tests that check various parts of the algorithms for correctness. These tests can
be built and run using the commands below:

```shell
cd build
make test_attpc_cleaning
./test_attpc_cleaning
```

If the tests succeed, you should see output that looks something like this (the number of assertions and test cases may
be different):

```text
===============================================================================
All tests passed (11052 assertions in 17 test cases)

```

Otherwise, the tests should show what's wrong and give the name of the test that failed. All tests are defined in `cpp`
files in the `test` directory.

## Checking performance

The `prof` directory contains a basic performance test for the Hough transform code. It can be built and run as follows:

```shell
cd build
make prof_LinearHoughTransform
./prof_LinearHoughTransform
```

The code accepts arguments that control the number of iterations to run, the number of points to simulate, and the
number of bins to use in the Hough space. Run it with the flag `-h` for details.

## Building documentation

To build the documentation, Doxygen is required. Run these commands to make the documents:

```shell
cd build
make doc
```

The documentation is built in the subdirectory `doc` of the build directory. To view it, open the file
`build/doc/html/index.html` in a web browser.
