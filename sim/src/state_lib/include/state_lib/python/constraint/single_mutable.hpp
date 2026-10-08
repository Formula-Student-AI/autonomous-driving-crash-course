#ifndef INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_SINGLE_MUTABLE_HPP_
#define INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_SINGLE_MUTABLE_HPP_

namespace eufs::state::constraint {

template<typename T, StateVars vars, typename DataType>
concept SingleMutable = requires(
    T a,
    DataType value
) {
    a[vars] = value;
};  // NOLINT

}  // namespace eufs::state::constraint

#endif  // INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_SINGLE_MUTABLE_HPP_
