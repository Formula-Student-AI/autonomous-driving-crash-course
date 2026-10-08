#include <gmock/gmock.h>

#include "vehicle_models/models/uniform_2d/base_uniform_2d.hpp"
#include "type/state.hpp"

using eufs::vehicle_models::BaseUniformMotion2D;
using Model = BaseUniformMotion2D<typename eufs::testing::State::Vector, eufs::testing::Vars>;

Model CreateModel() { return Model(eufs::vehicle_models::Param()); }

TEST(CheckConstructor, Default) { ASSERT_NO_THROW(); }
