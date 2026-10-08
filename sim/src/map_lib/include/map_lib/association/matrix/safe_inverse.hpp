#ifndef INCLUDE_MAP_LIB_ASSOCIATION_MATRIX_SAFE_INVERSE_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_MATRIX_SAFE_INVERSE_HPP_

#include <Eigen/LU>

#include <stdexcept>

namespace eufs::map::association {

template <typename MatrixType>
MatrixType SafeInverse(MatrixType matrix) {
  MatrixType inverse;
  bool invertible = false;
  matrix.computeInverseWithCheck(inverse, invertible);
  if (!invertible) {
    throw std::runtime_error("Matrix is not invertible.");
  }
  return inverse;
}

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_MATRIX_SAFE_INVERSE_HPP_
