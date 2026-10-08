#include <gmock/gmock.h>

#include "vehicle_models/models/angular_accel_2d/angular_accel_2d.hpp"
#include "vehicle_models/types/param.hpp"
#include "vehicle_models/types/command.hpp"
#include "vehicle_models/state/base_2d.hpp"

using Vars = eufs::vehicle_models::state::Base2DVars;
using Model = eufs::vehicle_models::AngularAccel2D<eufs::vehicle_models::state::Base2DState, Vars>;

Model CreateModel() {
  return Model(eufs::vehicle_models::Param(),
    eufs::vehicle_models::state::Base2DState::EigenMatrix::Identity());
}

TEST(CheckConstructor, Default) { ASSERT_NO_THROW(CreateModel()); }
