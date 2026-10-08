#include <gmock/gmock.h>

#include "vehicle_models/models/dynamic_bicycle/dynamic_bicycle.hpp"
#include "vehicle_models/types/param.hpp"
#include "vehicle_models/state/base_2d.hpp"

using Vars = eufs::vehicle_models::state::Base2DVars;
using Model = eufs::vehicle_models::DynamicBicycle<eufs::vehicle_models::state::Base2DState, Vars>;

Model CreateModel() {
  return Model(eufs::vehicle_models::Param(),
    eufs::vehicle_models::state::Base2DState::EigenMatrix::Identity());
}

TEST(CheckConstructor, Default) { ASSERT_NO_THROW(CreateModel()); }
