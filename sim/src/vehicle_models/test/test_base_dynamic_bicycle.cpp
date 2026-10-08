#include <gmock/gmock.h>

#include "vehicle_models/models/dynamic_bicycle/base_dynamic_bicycle.hpp"
#include "vehicle_models/types/param.hpp"
#include "vehicle_models/types/command.hpp"
#include "vehicle_models/state/base_2d.hpp"

using eufs::vehicle_models::BaseDynamicBicycle;
using Vars = eufs::vehicle_models::state::Base2DVars;
using Vector = eufs::vehicle_models::state::Base2DVector;
using Model = BaseDynamicBicycle<Vector, Vars>;

Model CreateModel() { return Model(eufs::vehicle_models::Param()); }

TEST(CheckConstructor, Default) { ASSERT_NO_THROW(CreateModel()); }
