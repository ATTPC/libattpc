#ifndef ATTPC_CLEANING_EIGEN_COMMON_H
#define ATTPC_CLEANING_EIGEN_COMMON_H

#include <stdexcept>

class EigenAssertFailure : public std::logic_error {
    using std::logic_error::logic_error;
};

/*
 * Redefine the eigen_assert macro before including Eigen. This will make Eigen throw an
 * exception when an assertion fails instead of calling abort. This makes testing much
 * easier since we can test for uncaught exceptions instead of just crashing the test
 * suite. This override is supported by the Eigen authors and mentioned in its documentation.
 *
 * If NDEBUG is defined (i.e. in a release build), Eigen defines EIGEN_NO_DEBUG which disables
 * the builtin assertions anyway, so we don't need to redefine eigen_assert in that case.
 */

#ifndef NDEBUG
#define eigen_assert(x) \
    if (!(x)) { throw (EigenAssertFailure(#x)); }
#endif

#include <Eigen/Core>

#endif /* end of include guard: ATTPC_CLEANING_EIGEN_COMMON_H */
