#ifndef INCLUDE_VEHICLE_MODELS_STATE_VARS_HPP_
#define INCLUDE_VEHICLE_MODELS_STATE_VARS_HPP_

#include <state_lib/state_vars.hpp>

namespace eufs::vehicle_models::state {

struct Vars {
  inline static constexpr eufs::state::StateVars<0> _x = {};
  inline static constexpr eufs::state::StateVars<1> _y = {};
  inline static constexpr eufs::state::StateVars<2> _yaw = {};
  inline static constexpr eufs::state::StateVars<3> _v_x = {};
  inline static constexpr eufs::state::StateVars<4> _v_y = {};
  inline static constexpr eufs::state::StateVars<5> _v_yaw = {};
  inline static constexpr eufs::state::StateVars<6> _a_x = {};
  inline static constexpr eufs::state::StateVars<7> _a_y = {};
};

}  // namespace eufs::vehicle_models::state

#endif  // INCLUDE_VEHICLE_MODELS_STATE_VARS_HPP_
