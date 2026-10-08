#include <gmock/gmock.h>

#include "vehicle_models/models/kinematic_bicycle/base_kinematic_bicycle.hpp"
#include "vehicle_models/types/param.hpp"
#include "vehicle_models/state/base_2d.hpp"

using eufs::vehicle_models::BaseKinematicBicycle;
using Vars = eufs::vehicle_models::state::Base2DVars;
using Model = BaseKinematicBicycle<eufs::vehicle_models::state::Base2DVector, Vars>;

Model CreateModel() { return Model(eufs::vehicle_models::Param()); }

TEST(CheckConstructor, Default) { ASSERT_NO_THROW(CreateModel()); }
