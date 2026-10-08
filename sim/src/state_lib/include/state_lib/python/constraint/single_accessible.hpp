#ifndef INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_SINGLE_ACCESSIBLE_HPP_
#define INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_SINGLE_ACCESSIBLE_HPP_

#include <concepts>

namespace eufs::state::constraint {

template<typename T, StateVars vars, typename DataType>
concept SingleAccessible = requires(T a) {
    { a(vars) } -> std::convertible_to<DataType>;
};  // NOLINT

}  // namespace eufs::state::constraint

#endif  // INCLUDE_STATE_LIB_PYTHON_CONSTRAINT_SINGLE_ACCESSIBLE_HPP_
