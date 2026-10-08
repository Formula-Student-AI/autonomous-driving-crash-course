#include <gmock/gmock.h>

#include "vehicle_models/models/uniform_2d/uniform_2d.hpp"
#include "type/state.hpp"

using eufs::vehicle_models::UniformMotion2D;
using Model = UniformMotion2D<eufs::testing::State, eufs::testing::Vars>;

Model CreateModel() {
  return Model(eufs::vehicle_models::Param(), eufs::testing::State::EigenMatrix::Identity());
}

TEST(CheckConstructor, Default) { ASSERT_NO_THROW(CreateModel()); }
