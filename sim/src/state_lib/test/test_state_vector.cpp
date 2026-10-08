#include <gmock/gmock.h>

#include <Eigen/Dense>
#include <eufs_gmock_matchers/eigen_matchers.hpp>

#include "state_lib/state_vars.hpp"
#include "state_lib/state_vector.hpp"

using eufs::state::StateMask;
using eufs::state::StateVars;
using eufs::state::StateVector;

// Initialise state for tests
constexpr auto D = 6;
using DataType = double;
using EigenVector = Eigen::Matrix<DataType, D, 1>;
namespace StateMember {
inline constexpr StateVars<0> _x = {};
inline constexpr StateVars<1> _y = {};
inline constexpr StateVars<2> _z = {};
inline constexpr StateVars<3> _dummy = {};
inline constexpr StateVars<4> _roll = {};
inline constexpr StateVars<5> _pitch = {};
inline constexpr StateVars<6> _yaw = {};

// Define commonly used groupings of elements
inline constexpr auto _pose = (_x, _y, _yaw);
}  // namespace StateMember

inline constexpr auto full_state = (StateMember::_x & StateMember::_y & StateMember::_z &
                                    StateMember::_roll & StateMember::_pitch & StateMember::_yaw);

inline EigenVector CreateVector() {
  EigenVector v;
  v << 0.4, 0.6, 0.8, 1.0, 0.0, -0.6;
  return v;
}

TEST(CheckConstructor, CheckEmptyInitialisation) {
  auto s = StateVector<full_state, DataType>();
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(EigenVector::Zero()));
}

TEST(CheckConstructor, CheckVectorInitialisation) {
  EigenVector v = CreateVector();
  auto s = StateVector<full_state, DataType>(v);
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateVector()));
}

TEST(CheckAssignment, CheckAssignmentEigen) {
  EigenVector v = CreateVector();
  StateVector<full_state, DataType> s;
  s = v;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateVector()));
}

TEST(CheckAssignment, CheckAssignmentArrayBrace) {
  StateVector<full_state, DataType> s;
  s = {0.4, 0.6, 0.8, 1.0, 0.0, -0.6};
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateVector()));
}

TEST(CheckAssignment, CheckAssignmentArray) {
  StateVector<full_state, DataType> s;
  std::array<DataType, D> a = {0.4, 0.6, 0.8, 1.0, 0.0, -0.6};
  s = a;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateVector()));
}

TEST(CheckAssignment, CheckAssignmentInitialization) {
  EigenVector v = CreateVector();
  StateVector<full_state, DataType> s = v;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateVector()));
}

TEST(CheckPlusAssignment, CheckTrue) {
  StateVector<full_state, DataType> s(CreateVector());
  Eigen::Matrix<double, D, 1> a;
  a << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  s += a;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq((CreateVector() + a).eval()));
}

TEST(CheckMinusAssignment, CheckTrue) {
  StateVector<full_state, DataType> s(CreateVector());
  Eigen::Matrix<double, D, 1> a;
  a << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  s -= a;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq((CreateVector() - a).eval()));
}

TEST(CheckSquareBracketOperator, CheckMultipleAssignmentArray) {
  EigenVector v = CreateVector();
  auto s = StateVector<full_state, DataType>(v);
  s[StateMember::_pose] = {-0.4, -0.6, 0.6};

  EigenVector expected;
  expected << -0.4, -0.6, 0.8, 1.0, 0.0, 0.6;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckSquareBracketOperator, CheckMultipleAssignmentEigen) {
  EigenVector v = CreateVector();
  auto s = StateVector<full_state, DataType>(v);
  s[StateMember::_pose] = Eigen::Vector3d{-0.4, -0.6, 0.6};

  EigenVector expected;
  expected << -0.4, -0.6, 0.8, 1.0, 0.0, 0.6;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckSquareBracketOperator, CheckSingleAssignment) {
  EigenVector v = CreateVector();
  auto s = StateVector<full_state, DataType>(v);
  s[StateMember::_roll] = -1.0;

  EigenVector expected;
  expected << 0.4, 0.6, 0.8, -1.0, 0.0, -0.6;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckSquareBracketOperator, CheckSingleStorage) {
  EigenVector v = CreateVector();
  const auto s = StateVector<full_state, DataType>(v);
  auto a = s[StateMember::_roll];
  EXPECT_DOUBLE_EQ(a, 1.0);
}

TEST(CheckRoundBracketOperator, CheckMultipleAccess) {
  EigenVector v = CreateVector();
  const auto s = StateVector<full_state, DataType>(v);
  auto r = s(StateMember::_pose);
  EXPECT_THAT(r, eufs::testing::matchers::EigenEq(Eigen::Vector3d{0.4, 0.6, -0.6}));
}

TEST(CheckRoundBracketOperator, CheckMultipleArgumentAccess) {
  EigenVector v = CreateVector();
  const auto s = StateVector<full_state, DataType>(v);
  auto r = s(StateMember::_x, StateMember::_y);
  EXPECT_THAT(r, eufs::testing::matchers::EigenEq(Eigen::Vector2d{0.4, 0.6}));
}

TEST(CheckRoundBracketOperator, CheckSingleAccess) {
  EigenVector v = CreateVector();
  const auto s = StateVector<full_state, DataType>(v);
  auto y = s(StateMember::_y);
  EXPECT_DOUBLE_EQ(y, 0.6);
}

TEST(CheckRoundBracketOperator, CheckSingleAssignment) {
  EigenVector v = CreateVector();
  auto s = StateVector<full_state, DataType>(v);
  s(StateMember::_roll) = -1.0;

  EigenVector expected;
  expected << 0.4, 0.6, 0.8, -1.0, 0.0, -0.6;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}
