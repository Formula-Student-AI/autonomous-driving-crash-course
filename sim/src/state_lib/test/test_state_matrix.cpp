#include <gmock/gmock.h>

#include <Eigen/Dense>
#include <eufs_gmock_matchers/eigen_matchers.hpp>

#include "state_lib/state_matrix.hpp"
#include "state_lib/state_vars.hpp"

using eufs::state::StateMask;
using eufs::state::StateMatrix;
using eufs::state::StateVars;

// Initialise state for tests
constexpr auto D = 4;
using DataType = double;
using EigenMatrix = Eigen::Matrix<DataType, D, D>;
namespace StateMember {
inline constexpr StateVars<0> _x = {};
inline constexpr StateVars<1> _dummy = {};
inline constexpr StateVars<2> _y = {};
inline constexpr StateVars<3> _z = {};
inline constexpr StateVars<4> _yaw = {};

// Define commonly used groupings of elements
inline constexpr auto _position_2d = (_x, _y);
inline constexpr auto _pose = (_x, _y, _yaw);
}  // namespace StateMember

inline constexpr auto full_state =
    (StateMember::_x & StateMember::_y & StateMember::_z & StateMember::_yaw);

inline EigenMatrix CreateMatrix() {
  EigenMatrix m;
  m.row(0) << 0.4, 0.6, 0.8, 1.0;
  m.row(1) << 0.1, 0.2, 0.3, 0.4;
  m.row(2) << 1.4, 1.6, 1.8, 2.0;
  m.row(3) << 2.4, 2.6, 2.8, 3.0;
  return m;
}

TEST(CheckConstructor, CheckEmptyInitialisation) {
  auto s = StateMatrix<full_state, DataType>();
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(EigenMatrix::Zero()));
}

TEST(CheckConstructor, CheckMatrixInitialisation) {
  EigenMatrix m = CreateMatrix();
  auto s = StateMatrix<full_state, DataType>(m);
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateMatrix()));
}

TEST(CheckAssignment, CheckAssignmentEigen) {
  EigenMatrix m = CreateMatrix();
  StateMatrix<full_state, DataType> s;
  s = m;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateMatrix()));
}

TEST(CheckAssignment, CheckAssignmentArray1D) {
  StateMatrix<full_state, DataType> s;
  std::array<DataType, D* D> a = {0.4, 0.6, 0.8, 1.0, 0.1, 0.2, 0.3, 0.4,
                                  1.4, 1.6, 1.8, 2.0, 2.4, 2.6, 2.8, 3.0};
  s = a;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateMatrix()));
}

TEST(CheckAssignment, CheckAssignmentArray2D) {
  StateMatrix<full_state, DataType> s;
  std::array<std::array<DataType, D>, D> a = {
      {{0.4, 0.6, 0.8, 1.0}, {0.1, 0.2, 0.3, 0.4}, {1.4, 1.6, 1.8, 2.0}, {2.4, 2.6, 2.8, 3.0}}};
  s = a;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateMatrix()));
}

TEST(CheckAssignment, CheckAssignmentInitialization) {
  EigenMatrix m = CreateMatrix();
  StateMatrix<full_state, DataType> s = m;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(CreateMatrix()));
}

TEST(CheckPlusAssignment, CheckTrue) {
  StateMatrix<full_state, DataType> s(CreateMatrix());
  Eigen::Matrix<double, D, D> a;
  a.row(0) << 0.1, 0.1, 0.1, 0.1;
  a.row(1) << 0.1, 0.1, 0.1, 0.1;
  a.row(2) << 0.1, 0.1, 0.1, 0.1;
  a.row(3) << 0.1, 0.1, 0.1, 0.1;
  s += a;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq((CreateMatrix() + a).eval()));
}

TEST(CheckMinusAssignment, CheckTrue) {
  StateMatrix<full_state, DataType> s(CreateMatrix());
  Eigen::Matrix<double, D, D> a;
  a.row(0) << 0.1, 0.1, 0.1, 0.1;
  a.row(1) << 0.1, 0.1, 0.1, 0.1;
  a.row(2) << 0.1, 0.1, 0.1, 0.1;
  a.row(3) << 0.1, 0.1, 0.1, 0.1;
  s -= a;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq((CreateMatrix() - a).eval()));
}

TEST(CheckSquareBracketOperator, CheckMultipleAssignmentArrayOne) {
  EigenMatrix m = CreateMatrix();
  auto s = StateMatrix<full_state, DataType>(m);
  std::array<DataType, 9> a = {-0.4, 0.0, 0.0, 0.0, -0.6, 0.0, 0.0, 0.0, 0.6};
  s[StateMember::_pose] = a;

  EigenMatrix expected;
  expected.row(0) << -0.4, 0.0, 0.8, 0.0;
  expected.row(1) << 0.0, -0.6, 0.3, 0.0;
  expected.row(2) << 1.4, 1.6, 1.8, 2.0;
  expected.row(3) << 0.0, 0.0, 2.8, 0.6;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckSquareBracketOperator, CheckMultipleAssignmentArrayTwo) {
  EigenMatrix m = CreateMatrix();
  auto s = StateMatrix<full_state, DataType>(m);
  std::array<std::array<DataType, 3>, 3> a = {
      {{-0.4, 0.0, 0.0}, {0.0, -0.6, 0.0}, {0.0, 0.0, 0.6}}};
  s[StateMember::_pose] = a;

  EigenMatrix expected;
  expected.row(0) << -0.4, 0.0, 0.8, 0.0;
  expected.row(1) << 0.0, -0.6, 0.3, 0.0;
  expected.row(2) << 1.4, 1.6, 1.8, 2.0;
  expected.row(3) << 0.0, 0.0, 2.8, 0.6;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckSquareBracketOperator, CheckMultipleAssignmentEigen) {
  EigenMatrix m = CreateMatrix();
  auto s = StateMatrix<full_state, DataType>(m);
  Eigen::Matrix3d v;
  v.row(0) << -0.4, 0.0, 0.0;
  v.row(1) << 0.0, -0.6, 0.0;
  v.row(2) << 0.0, 0.0, 0.6;
  s[StateMember::_pose] = v;

  EigenMatrix expected;
  expected.row(0) << -0.4, 0.0, 0.8, 0.0;
  expected.row(1) << 0.0, -0.6, 0.3, 0.0;
  expected.row(2) << 1.4, 1.6, 1.8, 2.0;
  expected.row(3) << 0.0, 0.0, 2.8, 0.6;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckSquareBracketOperator, CheckSingleAssignment) {
  EigenMatrix m = CreateMatrix();
  auto s = StateMatrix<full_state, DataType>(m);
  s[StateMember::_z] = -1.0;

  EigenMatrix expected = CreateMatrix();
  expected(2, 2) = -1.0;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckSquareBracketOperator, CheckSingleStorage) {
  EigenMatrix m = CreateMatrix();
  const auto s = StateMatrix<full_state, DataType>(m);
  auto a = s[StateMember::_z];
  EXPECT_DOUBLE_EQ(a, 1.8);
}

TEST(CheckRoundBracketOperator, CheckMultipleAccess) {
  EigenMatrix m = CreateMatrix();
  const auto s = StateMatrix<full_state, DataType>(m);
  auto r = s(StateMember::_pose);

  Eigen::Matrix3d expected;
  expected.row(0) << 0.4, 0.6, 1.0;
  expected.row(1) << 0.1, 0.2, 0.4;
  expected.row(2) << 2.4, 2.6, 3.0;
  EXPECT_THAT(r, eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckRoundBracketOperator, CheckMultipleArgumentAccess) {
  EigenMatrix m = CreateMatrix();
  const auto s = StateMatrix<full_state, DataType>(m);
  auto r = s(StateMember::_x, StateMember::_y);
  Eigen::Matrix2d expected = CreateMatrix().block(0, 0, 2, 2);
  EXPECT_THAT(r, eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckRoundBracketOperator, CheckSingleAccess) {
  EigenMatrix m = CreateMatrix();
  const auto s = StateMatrix<full_state, DataType>(m);
  auto y = s(StateMember::_y);
  EXPECT_DOUBLE_EQ(y, 0.2);
}

TEST(CheckRoundBracketOperator, CheckSingleAssignment) {
  EigenMatrix m = CreateMatrix();
  auto s = StateMatrix<full_state, DataType>(m);
  s(StateMember::_yaw) = -1.0;

  EigenMatrix expected = CreateMatrix();
  expected(3, 3) = -1.0;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckAtMethod, CheckSingleAssignment) {
  EigenMatrix m = CreateMatrix();
  auto s = StateMatrix<full_state, DataType>(m);
  s.at(StateMember::_yaw) = -1.0;

  EigenMatrix expected = CreateMatrix();
  expected(3, 3) = -1.0;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckAtMethod, CheckSingleAccess) {
  EigenMatrix m = CreateMatrix();
  const auto s = StateMatrix<full_state, DataType>(m);
  auto y = s.at(StateMember::_y);
  EXPECT_DOUBLE_EQ(y, 0.2);
}

TEST(CheckAtMethod, CheckOffDiagonalAssignmentOneArg) {
  EigenMatrix m = CreateMatrix();
  auto s = StateMatrix<full_state, DataType>(m);
  s.at(StateMember::_position_2d) = -1.0;

  EigenMatrix expected = CreateMatrix();
  expected(0, 1) = -1.0;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckAtMethod, CheckOffDiagonalAccessOneArg) {
  EigenMatrix m = CreateMatrix();
  const auto s = StateMatrix<full_state, DataType>(m);
  auto y = s.at(StateMember::_position_2d);
  EXPECT_DOUBLE_EQ(y, 0.6);
}

TEST(CheckAtMethod, CheckOffDiagonalAssignmentTwoArg) {
  EigenMatrix m = CreateMatrix();
  auto s = StateMatrix<full_state, DataType>(m);
  s.at(StateMember::_x, StateMember::_yaw) = -1.0;

  EigenMatrix expected = CreateMatrix();
  expected(0, 3) = -1.0;
  EXPECT_THAT(s(), eufs::testing::matchers::EigenEq(expected));
}

TEST(CheckAtMethod, CheckOffDiagonalAccessTwoArg) {
  EigenMatrix m = CreateMatrix();
  const auto s = StateMatrix<full_state, DataType>(m);
  auto y = s.at(StateMember::_x, StateMember::_y);
  EXPECT_DOUBLE_EQ(y, 0.6);
}
