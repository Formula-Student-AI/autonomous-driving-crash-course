#ifndef TEST_TYPE_STATE_HPP_
#define TEST_TYPE_STATE_HPP_

#include <state_lib/state.hpp>
#include <state_lib/state_vars.hpp>

namespace eufs::testing {

struct Vars {
  inline static constexpr state::StateVars<0> _x = {};
  inline static constexpr state::StateVars<1> _y = {};
  inline static constexpr state::StateVars<2> _yaw = {};
  inline static constexpr state::StateVars<3> _v_x = {};
  inline static constexpr state::StateVars<4> _v_y = {};
  inline static constexpr state::StateVars<5> _v_yaw = {};
  inline static constexpr state::StateVars<6> _a_x = {};
  inline static constexpr state::StateVars<7> _a_y = {};
  inline static constexpr state::StateVars<8> _steering = {};
};

using State = state::StateVariable<Vars::_x & Vars::_y & Vars::_yaw & Vars::_v_x & Vars::_v_y &
                                   Vars::_v_yaw & Vars::_a_x & Vars::_a_y & Vars::_steering>;
}  // namespace eufs::testing

#endif  // TEST_TYPE_STATE_HPP_
