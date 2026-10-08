#ifndef INCLUDE_STATE_LIB_STATE_MATRIX_HPP_
#define INCLUDE_STATE_LIB_STATE_MATRIX_HPP_

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <utility>

#include "state_lib/state_mask.hpp"
#include "state_lib/state_vars.hpp"

namespace eufs::state {

template <StateVars vars, typename DataType = double>
class StateMatrix;  // Forward declaration

template <typename DataType, typename Matrix, StateElement... elems>
struct StateMatrixView {
  static constexpr size_t size = sizeof...(elems);

  /**
   * @brief Construct view of a StateMatrix
   * @param m Reference to the original StateMatrix
   */
  explicit StateMatrixView(Matrix &m) : mat_(m) {}

  /**
   * @brief Assign new values to each element in the view
   * @param array Values to assign
   */
  StateMatrixView &operator=(std::array<DataType, size * size> array) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) = array[row_idx * size + column_idx++]; }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Assign new values to each element in the view
   * @param array Values to assign
   */
  StateMatrixView &operator=(std::array<std::array<DataType, size>, size> array) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) = array[row_idx][column_idx++]; }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Assign new values to each element in the view
   * @param array Values to assign
   */
  template <typename Derived>
  StateMatrixView &operator=(const Eigen::MatrixBase<Derived> &array) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) = array(row_idx, column_idx++); }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Increment each value in the view
   * @param array Values to increase by
   */
  StateMatrixView &operator+=(std::array<DataType, size * size> array) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) += array[row_idx * size + column_idx++]; }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Increment each value in the view
   * @param array Values to increase by
   */
  StateMatrixView &operator+=(std::array<std::array<DataType, size>, size> array) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) += array[row_idx][column_idx++]; }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Increment each value in the view
   * @param array Values to increase by
   */
  template <typename Derived>
  StateMatrixView &operator+=(const Eigen::MatrixBase<Derived> &array) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) += array(row_idx, column_idx++); }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Decrement each value in the view
   * @param array Values to decrease by
   */
  StateMatrixView &operator-=(std::array<DataType, size * size> array) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) -= array[row_idx * size + column_idx++]; }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Decrement each value in the view
   * @param array Values to decrease by
   */
  StateMatrixView &operator-=(std::array<std::array<DataType, size>, size> array) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) -= array[row_idx][column_idx++]; }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Decrement each value in the view
   * @param array Values to decrease by
   */
  template <typename Derived>
  StateMatrixView &operator-=(const Eigen::MatrixBase<Derived> &array) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) -= array(row_idx, column_idx++); }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Multiply each value in the view
   * @param v Factor
   */
  StateMatrixView &operator*=(const DataType v) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) *= v; }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

  /**
   * @brief Divide each value in the view
   * @param v Divisor
   */
  StateMatrixView &operator/=(const DataType v) {
    size_t row_idx = 0;
    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ([&] { mat_.at(StateMask<i, elems>{}) /= v; }(), ...);

          row_idx++;
        }(),
        ...);
    return *this;
  }

 private:
  Matrix &mat_;
};

template <StateVars vars, typename DataType>
class StateMatrix {
  static constexpr size_t size = GetVarsNumber(vars);
  typedef Eigen::Matrix<DataType, size, size, Eigen::RowMajor> Matrix;

 public:
  /**
   * @brief Construct new StateMatrix object (and initialise with 0s)
   */
  StateMatrix() : mat_{Matrix::Zero(size, size)} {}

  /**
   * @brief Construct new StateMatrix object
   * @param m Initialise StateMatrix with m
   */
  template <typename Derived>
  StateMatrix(const Eigen::MatrixBase<Derived> &m) : mat_{m} {}  // NOLINT

  /**
   * @brief Assign Eigen matrix to StateMatrix
   * @param m New values to assign
   */
  template <typename Derived>
  StateMatrix &operator=(const Eigen::MatrixBase<Derived> &m) {
    mat_ = m;
    return *this;
  }

  /**
   * @brief Assign 2D array to StateMatrix
   * @param m New values to assign
   */
  StateMatrix &operator=(std::array<std::array<DataType, size>, size> m) {
    for (size_t i = 0; i < size; ++i) {
      mat_.row(i) = Eigen::Map<Eigen::Matrix<DataType, size, 1>>(m[i].data(), size);
    }
    return *this;
  }

  /**
   * @brief Assign 1D array to StateVector (row major)
   * @param m New values to assign
   */
  StateMatrix &operator=(std::array<DataType, size * size> m) {
    mat_ = Eigen::Map<Matrix>(m.data(), size, size);
    return *this;
  }

  /**
   * @brief Add matrix to current StateMatrix
   * @param v Eigen matrix of values
   */
  template <typename Derived>
  StateMatrix &operator+=(const Eigen::MatrixBase<Derived> &v) {
    mat_ = (mat_ + v).eval();
    return *this;
  }

  /**
   * @brief Add values to current StateMatrix
   * @param v Values to increase by
   */
  StateMatrix &operator+=(std::array<std::array<DataType, size>, size> m) {
    for (size_t i = 0; i < size; ++i) {
      mat_.row(i) += Eigen::Map<Eigen::Matrix<DataType, size, 1>>(m[i].data(), size);
    }
    return *this;
  }

  /**
   * @brief Add values to current StateMatrix
   * @param v Values to increase by
   */
  StateMatrix &operator+=(std::array<DataType, size * size> m) {
    mat_ += Eigen::Map<Matrix>(m.data(), size, size);
    return *this;
  }

  /**
   * @brief Subtract values from the current StateMatrix
   * @param v Eigen matrix of values
   */
  template <typename Derived>
  StateMatrix &operator-=(const Eigen::MatrixBase<Derived> &v) {
    mat_ = (mat_ - v).eval();
    return *this;
  }

  /**
   * @brief Subtract values from the current StateMatrix
   * @param v Values to decrease by
   */
  StateMatrix &operator-=(std::array<std::array<DataType, size>, size> m) {
    for (size_t i = 0; i < size; ++i) {
      mat_.row(i) -= Eigen::Map<Eigen::Matrix<DataType, size, 1>>(m[i].data(), size);
    }
    return *this;
  }

  /**
   * @brief Subtract matrix from the current StateMatrix
   * @param v Values to decrease by
   */
  StateMatrix &operator-=(std::array<DataType, size * size> m) {
    mat_ -= Eigen::Map<Matrix>(m.data(), size, size);
    return *this;
  }

  /**
   * @brief Return a StateMatrixView with the corresponding matrix elements
   * @tparam elems Elements to keep in the view
   * @param _ StateMask
   * @return A StateMatrixView of the given elements
   */
  template <StateElement... elems>
  auto operator[](const StateMask<elems...> &) {
    return StateMatrixView<DataType, StateMatrix, elems...>(*this);
  }

  /**
   * @brief Return a reference to the value at (elem, elem).
   * @tparam elem Index position of the value to access
   * @return Reference of a value
   */
  template <StateElement elem>
  DataType &operator[](const StateVars<elem> &el) {
    return (*this)(el);
  }

  /**
   * @brief Return the value at (elem, elem).
   * @tparam elem Index position of the value to access
   * @return Value
   */
  template <StateElement elem>
  DataType operator[](const StateVars<elem> &el) const {
    return (*this)(el);
  }

  /**
   * @brief Return a Eigen matrix of the entire matrix
   * @return Eigen matrix
   */
  auto operator()() const { return mat_; }

  /**
   * @brief Return a Eigen matrix of requested state elements
   * @tparam elems State elements to be a part of the view
   * @return Eigen matrix of requested state elements
   */
  template <StateElement... elems>
  auto operator()(const StateMask<elems...> &) const {
    Eigen::Matrix<DataType, sizeof...(elems), sizeof...(elems)> m;
    size_t row_idx = 0;

    (
        [&] {
          size_t column_idx = 0;
          constexpr auto i = elems;

          // Inner for loop
          ((m(row_idx, column_idx++) = at(StateVars<i>(), StateVars<elems>())), ...);

          row_idx++;
        }(),
        ...);

    return m;
  }

  /**
   * @brief If multiple StateMasks are passed to the () operator, combine them into one.
   * This is required because `,` has special meaning in () operators, and StateMask
   * overloads the `,` operator to combine StateMasks. This overload ensures we can enforce
   * higher priority over combining StateMask objects.
   * @tparam Args StateMask type
   * @param e StateMask objects
   * @return Eigen Matrix containing values of requested elements
   */
  template <class... Args, class = std::enable_if_t<std::conjunction_v<IsStateMask<Args>...> ||
                                                    std::conjunction_v<IsVars<Args>...>>>
  auto operator()(Args... e) const {
    return (*this)((e, ...));
  }

  /**
   * @brief Return a reference to the value at (elem, elem).
   * @tparam elem Index position of the value to access
   * @return Reference of a value
   */
  template <StateElement elem>
  DataType &operator()(const StateVars<elem> &el) {
    return at(el);
  }

  /**
   * @brief Return the value at (elem, elem).
   * @tparam elem Index position of the value to access
   * @return Value
   */
  template <StateElement elem>
  DataType operator()(const StateVars<elem> &el) const {
    return at(el);
  }

 public:
  /**
   * @brief Return a reference to the value at (elem, elem).
   * @tparam elem Index position of the value to access
   * @return Reference of a value
   */
  template <StateElement elem>
  DataType &at(const StateVars<elem> &el) {
    return at(el, el);
  }

  /**
   * @brief Return the value at (elem, elem).
   * @tparam elem Index position of the value to access
   * @return Value
   */
  template <StateElement elem>
  DataType at(const StateVars<elem> &el) const {
    return at(el, el);
  }

  /**
   * @brief Return a reference to the value at (e1, e2).
   * @tparam e1 Row position of the value to access
   * @tparam e2 Column position of the value to access
   * @return Reference of a value
   */
  template <StateElement e1, StateElement e2>
  DataType &at(const StateMask<e1, e2> &) {
    return at(StateVars<e1>(), StateVars<e2>());
  }

  /**
   * @brief Return the value at (e1, e2).
   * @tparam e1 Row position of the value to access
   * @tparam e2 Column position of the value to access
   * @return Value
   */
  template <StateElement e1, StateElement e2>
  DataType at(const StateMask<e1, e2> &) const {
    return at(StateVars<e1>(), StateVars<e2>());
  }

  /**
   * @brief Return a reference to the value at (e1, e2).
   * @tparam e1 Row position of the value to access
   * @tparam e2 Column position of the value to access
   * @return Reference of a value
   */
  template <StateElement e1, StateElement e2>
  DataType &at(const StateVars<e1> &, const StateVars<e2> &) {
    return std::invoke(
        [&]<StateElement... elems>(const StateVars<elems...> &)->DataType &{
              int i = -1;
              ((++i, e1 == elems) || ...);

              int j = -1;
              ((++j, e2 == elems) || ...);

              return mat_(i, j);
            },
        vars);
  }

  /**
   * @brief Return the value at (e1, e2).
   * @tparam e1 Row position of the value to access
   * @tparam e2 Column position of the value to access
   * @return Value
   */
  template <StateElement e1, StateElement e2>
  DataType at(const StateVars<e1> &, const StateVars<e2> &) const {
    return std::invoke(
        [&]<StateElement... elems>(const StateVars<elems...> &) {
          int i = -1;
          ((++i, e1 == elems) || ...);

          int j = -1;
          ((++j, e2 == elems) || ...);

          return mat_(i, j);
        },
        vars);
  }

 private:
  Matrix mat_;
};

}  // namespace eufs::state

#endif  // INCLUDE_STATE_LIB_STATE_MATRIX_HPP_
