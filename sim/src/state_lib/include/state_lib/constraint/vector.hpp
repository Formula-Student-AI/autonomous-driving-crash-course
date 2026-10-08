#ifndef INCLUDE_STATE_LIB_CONSTRAINT_VECTOR_HPP_
#define INCLUDE_STATE_LIB_CONSTRAINT_VECTOR_HPP_

#include "state_lib/state_vector.hpp"

namespace eufs::state::constraint {

template<typename T>
concept Vector = IsVector<T>::value;

}  // namespace eufs::state::constraint

#endif  // INCLUDE_STATE_LIB_CONSTRAINT_VECTOR_HPP_
