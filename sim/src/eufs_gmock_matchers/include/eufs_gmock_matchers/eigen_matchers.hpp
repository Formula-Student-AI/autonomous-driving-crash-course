#ifndef INCLUDE_EUFS_GMOCK_MATCHERS_EIGEN_MATCHERS_HPP_
#define INCLUDE_EUFS_GMOCK_MATCHERS_EIGEN_MATCHERS_HPP_

#include <gmock/gmock.h>
#include <Eigen/Dense>

namespace eufs::testing::matchers {

template <class Base>
class PrintWrapper : public Base {
    friend void PrintTo(const PrintWrapper &m, ::std::ostream *o) {
        *o << "\n" << m;
    }
};

template <class Base>
const PrintWrapper<Base> &PrintWrap(const Base &base) {
    return static_cast<const PrintWrapper<Base> &>(base);
}

MATCHER_P(EigenEq, exp, "") { return arg.isApprox(exp); }

}  // namespace eufs::testing::matchers

#endif  // INCLUDE_EUFS_GMOCK_MATCHERS_EIGEN_MATCHERS_HPP_
