#include <gmock/gmock.h>

#include "vehicle_models/models/acceleration/base_acceleration.hpp"
#include "vehicle_models/types/param.hpp"
#include "vehicle_models/types/command.hpp"
#include "vehicle_models/state/base_2d.hpp"

using eufs::vehicle_models::BaseAccelerationModel;
using Vars = eufs::vehicle_models::state::Base2DVars;
using Vector = eufs::vehicle_models::state::Base2DVector;
using Model = BaseAccelerationModel<Vector, Vars>;

Model CreateModel() { return Model(eufs::vehicle_models::Param()); }

TEST(CheckConstructor, Default) { ASSERT_NO_THROW(CreateModel()); }
