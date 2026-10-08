#ifndef INCLUDE_VEHICLE_MODELS_STATE_BASE_2D_HPP_
#define INCLUDE_VEHICLE_MODELS_STATE_BASE_2D_HPP_

#include <state_lib/state_vars.hpp>
#include <state_lib/state_vector.hpp>
#include <state_lib/state.hpp>

#include "vehicle_models/state/vars.hpp"

namespace eufs::vehicle_models::state {

struct Base2DVars {
  inline static constexpr auto _x = Vars::_x;
  inline static constexpr auto _y = Vars::_y;
  inline static constexpr auto _yaw = Vars::_yaw;
  inline static constexpr auto _v_x = Vars::_v_x;
  inline static constexpr auto _v_y = Vars::_v_y;
  inline static constexpr auto _v_yaw = Vars::_v_yaw;
  inline static constexpr auto _a_x = Vars::_a_x;
  inline static constexpr auto _a_y = Vars::_a_y;

  inline static constexpr auto vars = (
    Base2DVars::_x & Base2DVars::_y & Base2DVars::_yaw &
    Base2DVars::_v_x & Base2DVars::_v_y & Base2DVars::_v_yaw &
    Base2DVars::_a_x & Base2DVars::_a_y);

  inline static constexpr auto pose = (
    Base2DVars::_x & Base2DVars::_y & Base2DVars::_yaw);
};

using Base2DState = eufs::state::StateVariable<Base2DVars::vars>;
using Base2DVector = eufs::state::StateVector<Base2DVars::vars>;

}  // namespace eufs::vehicle_models::state

#endif  // INCLUDE_VEHICLE_MODELS_STATE_BASE_2D_HPP_
