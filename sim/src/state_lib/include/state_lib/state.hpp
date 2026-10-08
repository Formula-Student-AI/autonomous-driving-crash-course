#ifndef INCLUDE_STATE_LIB_STATE_HPP_
#define INCLUDE_STATE_LIB_STATE_HPP_

#include <Eigen/Dense>

#include "state_lib/state_mask.hpp"
#include "state_lib/state_matrix.hpp"
#include "state_lib/state_vars.hpp"
#include "state_lib/state_vector.hpp"

namespace eufs::state {

template <StateVars vars, typename DataType = double>
struct StateVariable {
  // Useful types
  static constexpr auto size = GetVarsNumber(vars);

  using Vector = StateVector<vars, DataType>;
  using Matrix = StateMatrix<vars, DataType>;
  using EigenVector = Eigen::Matrix<DataType, size, 1>;
  using EigenMatrix = Eigen::Matrix<DataType, size, size>;

  Vector state;
  Matrix covariance;
};

}  // namespace eufs::state

#endif  // INCLUDE_STATE_LIB_STATE_HPP_
