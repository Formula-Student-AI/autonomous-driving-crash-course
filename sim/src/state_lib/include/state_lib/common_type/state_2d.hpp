#ifndef INCLUDE_STATE_LIB_COMMON_TYPE_STATE_2D_HPP_
#define INCLUDE_STATE_LIB_COMMON_TYPE_STATE_2D_HPP_

#include "state_lib/state_vars.hpp"
#include "state_lib/state_vector.hpp"

namespace eufs::state::example {

struct State2DVars {
  inline static constexpr StateVars<0> _x = {};
  inline static constexpr StateVars<1> _y = {};
  inline static constexpr StateVars<2> _yaw = {};
  inline static constexpr StateVars<3> _v_x = {};
  inline static constexpr StateVars<4> _v_y = {};
  inline static constexpr StateVars<5> _v_yaw = {};
  inline static constexpr StateVars<6> _a_x = {};
  inline static constexpr StateVars<7> _a_y = {};

  inline static constexpr auto vars = (
    State2DVars::_x & State2DVars::_y & State2DVars::_yaw &
    State2DVars::_v_x & State2DVars::_v_y & State2DVars::_v_yaw &
    State2DVars::_a_x & State2DVars::_a_y);

  inline static auto pose = (_x & _y & _yaw);
};

using State2DVector = state::StateVector<State2DVars::vars, double>;

}  // namespace eufs::state::example

#endif  // INCLUDE_STATE_LIB_COMMON_TYPE_STATE_2D_HPP_
