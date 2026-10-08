#ifndef INCLUDE_STATE_LIB_STATE_VECTOR_HPP_
#define INCLUDE_STATE_LIB_STATE_VECTOR_HPP_

#include <Eigen/Dense>
#include <array>
#include <functional>
#include <map>
#include <type_traits>

#include "state_lib/state_mask.hpp"
#include "state_lib/state_vars.hpp"

namespace eufs::state {

template <StateVars vars, typename DataType = double>
class StateVector;  // Forward declaration

template <typename DataType, typename Vector, StateElement... elems>
struct StateVectorView {
  static constexpr size_t size = sizeof...(elems);

  /**
   * @brief Construct view of a StateVector
   * @param v Reference to the original StateVector
   */
  explicit StateVectorView(Vector &v) : vec_{v} {}

  /**
   * @brief Assign new values to each element in the view
   * @param v Values to assign
   */
  template <typename Derived>
  StateVectorView &operator=(const Eigen::MatrixBase<Derived> &v) {
    size_t i = 0;
    ([&] { vec_[StateVars<elems>{}] = v[i++]; }(), ...);
    return *this;
  }

  /**
   * @brief Assign new values to each element in the view
   * @param v Values to assign
   */
  StateVectorView &operator=(const std::array<DataType, size> &v) {
    size_t i = 0;
    ([&] { vec_[StateVars<elems>{}] = v[i++]; }(), ...);
    return *this;
  }

  /**
   * @brief Increment each value in the view
   * @param v Values to increase by
   */
  template <typename Derived>
  StateVectorView &operator+=(const Eigen::MatrixBase<Derived> &v) {
    size_t i = 0;
    ([&] { vec_[StateVars<elems>{}] += v[i++]; }(), ...);
    return *this;
  }

  /**
   * @brief Increment each value in the view
   * @param v Values to increase by
   */
  StateVectorView &operator+=(const std::array<DataType, size> &v) {
    size_t i = 0;
    ([&] { vec_[StateVars<elems>{}] += v[i++]; }(), ...);
    return *this;
  }

  /**
   * @brief Decrement each value in the view
   * @param v Values to decrease by
   */
  template <typename Derived>
  StateVectorView &operator-=(const Eigen::MatrixBase<Derived> &v) {
    size_t i = 0;
    ([&] { vec_[StateVars<elems>{}] -= v[i++]; }(), ...);
    return *this;
  }

  /**
   * @brief Decrement each value in the view
   * @param v Values to decrease by
   */
  StateVectorView &operator-=(const std::array<DataType, size> &v) {
    size_t i = 0;
    ([&] { vec_[StateVars<elems>{}] -= v[i++]; }(), ...);
    return *this;
  }

  /**
   * @brief Multiply each element in the view
   * @param v Factor
   */
  StateVectorView &operator*=(const DataType &v) {
    size_t i = 0;
    ([&] { vec_[StateVars<elems>{}] *= v; }(), ...);
    return *this;
  }

  /**
   * @brief Divide each element in the view
   * @param v Divisor
   */
  StateVectorView &operator/=(const DataType &v) {
    size_t i = 0;
    ([&] { vec_[StateVars<elems>{}] /= v; }(), ...);
    return *this;
  }

 private:
  Vector &vec_;
};

template <StateVars vars, typename DataType>
class StateVector {
  static constexpr size_t size = GetVarsNumber(vars);
  typedef Eigen::Matrix<DataType, size, 1> Vector;

 public:
  /**
   * @brief Construct new StateVector object (and initialise with 0s)
   */
  StateVector() : vec_{Vector::Zero(size)} {}

  /**
   * @brief Construct new StateVector object
   * @param v Initialise StateVector with v
   */
  template <typename Derived>
  StateVector(const Eigen::MatrixBase<Derived> &v) : vec_{v} {}  // NOLINT

  /**
   * @brief Construct new StateVector object
   * @param v Initialise StateVector with v
   */
  explicit StateVector(std::array<DataType, size> v)
    : vec_{Eigen::Map<Vector>(v.data(), size, 1)} {}

  /**
   * @brief Assign Eigen vector to StateVector
   * @param v New values to assign
   */
  template <typename Derived>
  StateVector &operator=(const Eigen::MatrixBase<Derived> &v) {
    vec_ = v;
    return *this;
  }

  /**
   * @brief Assign array to StateVector
   * @param v New values to assign
   */
  StateVector &operator=(std::array<DataType, size> v) {
    vec_ = Eigen::Map<Vector>(v.data(), size, 1);
    return *this;
  }

  /**
   * @brief Add vector to current StateVector
   * @param v Eigen vector of values
   */
  template <typename Derived>
  StateVector &operator+=(const Eigen::MatrixBase<Derived> &v) {
    vec_ = (vec_ + v).eval();
    return *this;
  }

  /**
   * @brief Add values to the current StateVector
   * @param v Values to increase by
   */
  StateVector &operator+=(std::array<DataType, size> v) {
    vec_ += Eigen::Map<Vector>(v.data(), size, 1);
    return *this;
  }

  /**
   * @brief Subtract vector from the current StateVector
   * @param v Eigen vector of values
   */
  template <typename Derived>
  StateVector &operator-=(const Eigen::MatrixBase<Derived> &v) {
    vec_ = (vec_ - v).eval();
    return *this;
  }

  /**
   * @brief Subtract vector from the current StateVector
   * @param v New values to assign
   */
  StateVector &operator-=(std::array<DataType, size> v) {
    vec_ -= Eigen::Map<Vector>(v.data(), size, 1);
    return *this;
  }

  /**
   * @brief Return a StateVectorView with the requested state elements
   * @tparam elems Elements to be a part of the StateVectorView
   * @param _ StateMask
   * @return View of the requested elements
   */
  template <StateElement... elems>
  auto operator[](const StateMask<elems...> &) {
    return StateVectorView<DataType, StateVector, elems...>(*this);
  }

  /**
   * @brief Get reference to value of state element
   * @tparam elem State element to access
   * @return Reference to value
   */
  template <StateElement elem>
  DataType &operator[](const StateVars<elem> &var) {
    return (*this)(var);
  }

  /**
   * @brief Get copy of value of state element
   * @tparam elem State element to access
   * @return Copy of value
   */
  template <StateElement elem>
  DataType operator[](const StateVars<elem> &var) const {
    return (*this)(var);
  }

  /**
   * @brief Return a Eigen vector of the entire vector
   * @return Eigen vector
   */
  Vector operator()() const { return vec_; }

  /**
   * @brief Return a Eigen vector of requested state elements
   * @tparam elems State element to return
   * @return Eigen vector of requested state elements
   */
  template <StateElement... elems>
  auto operator()(const StateMask<elems...> &) const {
    return Eigen::Matrix<DataType, sizeof...(elems), 1>{(*this)(StateVars<elems>{})...};
  }

  /**
   * @brief If multiple StateMasks are passed to the () operator, combine them into one.
   * This is required because `,` has special meaning in () operators, and StateMask
   * overloads the `,` operator to combine StateMasks. This overload ensures we can enforce
   * higher priority over combining StateMask objects.
   * @tparam Args StateMask type
   * @param e StateMask objects
   * @return Eigen vector containing values of requested elements
   */
  template <class... Args, class = std::enable_if_t<std::conjunction_v<IsStateMask<Args>...> ||
                                                    std::conjunction_v<IsVars<Args>...>>>
  auto operator()(Args... e) const {
    return (*this)((e, ...));
  }

  /**
   * @brief Get reference to value of state element
   * @tparam elem State element to access
   * @return Reference to value
   */
  template <StateElement elem>
  DataType &operator()(const StateVars<elem> &) {
    return std::invoke(
        [&]<StateElement... elems>(const StateVars<elems...> &)->DataType & {
              int i = -1;
              ((i++, elem == elems) || ...);
              return vec_(i);
            },
        vars);
  }

  /**
   * @brief Get copy of value of state element
   * @tparam elem State element to access
   * @return Copy of value
   */
  template <StateElement elem>
  DataType operator()(const StateVars<elem> &) const {
    return std::invoke(
        [&]<StateElement... elems>(const StateVars<elems...> &) {
          int i = -1;
          ((i++, elem == elems) || ...);
          return vec_(i);
        },
        vars);
  }

 private:
  Vector vec_;
};

template <class T>
struct IsVector : std::false_type {};

template <StateVars vars, typename DataType>
struct IsVector<StateVector<vars, DataType>> : std::true_type {};

template <StateVars vars, typename DataType>
struct IsVector<const StateVector<vars, DataType>> : std::true_type {};

}  // namespace eufs::state

#endif  // INCLUDE_STATE_LIB_STATE_VECTOR_HPP_
