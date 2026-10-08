#include <gmock/gmock.h>

#include "vehicle_models/models/angular_accel_2d/base_angular_accel_2d.hpp"
#include "vehicle_models/state/base_2d.hpp"
#include "vehicle_models/types/command.hpp"
#include "vehicle_models/types/param.hpp"

using eufs::vehicle_models::BaseAngularAccel2D;
using Vars = eufs::vehicle_models::state::Base2DVars;
using Model = BaseAngularAccel2D<eufs::vehicle_models::state::Base2DVector, Vars>;

Model CreateModel() { return Model(eufs::vehicle_models::Param()); }

TEST(CheckConstructor, Default) { ASSERT_NO_THROW(CreateModel()); }
