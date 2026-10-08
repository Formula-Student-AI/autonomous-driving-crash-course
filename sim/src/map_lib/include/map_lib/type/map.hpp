#ifndef INCLUDE_MAP_LIB_TYPE_MAP_HPP_
#define INCLUDE_MAP_LIB_TYPE_MAP_HPP_

#include <concepts>
#include <iostream>
#include <optional>
#include <vector>
#include <utility>
#include <algorithm>

#include "map_lib/type/cone.hpp"

namespace eufs::map {

template <typename ConeType>
class Map {
 public:  // Map types
  using InternalRepresentation = std::vector<ConeType>;
  using iterator = typename InternalRepresentation::iterator;
  using const_iterator = typename InternalRepresentation::const_iterator;

  using value_type = ConeType;

 public:  // Constructors
  /**
   * @brief Construct a new Map object
   */
  Map() = default;

  /**
   * @brief Construct a new Map object
   *
   * @param cones List of cones to initialise map with
   */
  explicit Map(const std::vector<ConeType> &cones) : cones_(cones) {}

  explicit Map(std::vector<ConeType> &&cones) : cones_(std::move(cones)) {}

  Map &operator=(const std::vector<ConeType> &cones) {
    cones_ = cones;
    return *this;
  }

  Map &operator=(std::vector<ConeType> &&cones) {
    cones_ = std::move(cones);
    return *this;
  }

 public:  // Add methods
  /**
   * @brief Add cone to map
   *
   * @param cone Cone to add
   */
  void Add(const ConeType &cone) { cones_.push_back(cone); }

  /**
   * @brief Add cones to map
   *
   * @param cones Cones to add to map
   */
  void Add(const std::vector<ConeType> &cones) {
    cones_.insert(cones_.end(), cones.begin(), cones.end());
  }

 public:  // Remove methods
  /**
   * @brief Remove cone at given index position
   *
   * @param idx Index position to delete at
   */
  void Remove(const size_t &idx) { cones_.erase(std::next(cones_.begin(), idx)); }

  /**
   * @brief Remove cones based on predicate condition
   *
   * @tparam Predicate Function type of predicate
   * @param predicate
   */
  template <typename Predicate>
  requires std::predicate<Predicate, ConeType>
  void EraseIf(Predicate predicate) {
    auto res = std::remove_if(this->cones_.begin(), this->cones_.end(),
                              [&predicate](const ConeType &elem) { return predicate(elem); });
    this->cones_.erase(res, this->cones_.end());
  }

 public:  // Accessor methods
  /**
   * @brief Find all cones (of a certain colour)
   *
   * @param color Color to filter by
   * @return all cones (matching the given colour)
   */
  std::vector<ConeType> Get(std::optional<Color> color = {}) const {
    if (color) {
      std::vector<ConeType> copy;
      std::copy_if(cones_.cbegin(), cones_.cend(), std::back_inserter(copy),
                   [&](const ConeType &cone) { return cone.GetColor() == color.value(); });
      return copy;
    }
    return cones_;
  }

  /**
   * @brief Get cones at specified indices
   *
   * @param r Collection of indices
   * @return Cones at the indices
   */
  template <std::ranges::input_range Range>
  std::vector<ConeType> Get(const Range &r) const {
    std::vector<ConeType> cones;

    std::ranges::transform(r, std::back_inserter(cones), [&](const size_t idx) { return at(idx); });

    return cones;
  }

  /**
   * @brief Find all cone positions (of a certain colour)
   *
   * @param color Color to filter by
   * @return all cone positions (matching the given colour)
   */
  std::vector<Eigen::Vector2d> GetPositions(std::optional<Color> color = {}) const {
    std::vector<Eigen::Vector2d> positions;

    std::for_each(cones_.cbegin(), cones_.cend(), [&](const ConeType &cone) {
      if (!color || cone.GetColor() == color.value()) {
        positions.push_back(cone.position);
      }
    });
    return positions;
  }

  /**
   * @brief Get cone positions at specified indices
   *
   * @param r Collection of indices
   * @return Cone positions at the indices
   */
  template <std::ranges::input_range Range>
  std::vector<Eigen::Vector2d> GetPositions(const Range &r) const {
    std::vector<Eigen::Vector2d> positions;

    std::ranges::transform(r, std::back_inserter(positions),
                           [&](const size_t idx) { return at(idx).position; });

    return positions;
  }

 public:  // Iterators
  /**
   * @brief Returns a read/write iterator that points to the first element.
   *
   * @return Iterator
   */
  iterator begin() { return cones_.begin(); }

  /**
   * @brief Returns a read-only (constant) iterator that points to the first element.
   *
   * @return ConstIterator
   */
  const_iterator begin() const { return cones_.begin(); }

  /**
   * @brief Returns a read-only (constant) iterator that points to the first element.
   *
   * @return ConstIterator
   */
  const_iterator cbegin() const { return cones_.cbegin(); }

  /**
   * @brief Returns a read/write iterator that points one past the last element.
   *
   * @return Iterator
   */
  iterator end() { return cones_.end(); }

  /**
   * @brief Returns a read-only (constant) iterator that points one past the last element.
   *
   * @return ConstIterator
   */
  const_iterator end() const { return cones_.end(); }

  /**
   * @brief Returns a read-only (constant) iterator that points one past the last element.
   *
   * @return ConstIterator
   */
  const_iterator cend() const { return cones_.cend(); }

 public:  // Access methods
  /**
   * @brief Return read-only (constant) reference to cone at given index position
   *
   * @return Cone
   */
  const ConeType &at(size_t idx) const { return cones_.at(idx); }

  /**
   * @brief Return reference to cone at given index position
   *
   * @return Cone
   */
  ConeType &at(size_t idx) { return cones_.at(idx); }

 public:  // Size methods
  /**
   * @brief Returns the number of elements in the map.
   *
   * @return Size
   */
  size_t size() const { return cones_.size(); }

 public:  // Return different representations of map
  /**
   * @brief Return a copy of the map as a vector of Cone objects
   *
   * @return Vector of cones
   */
  std::vector<ConeType> ToCones() const { return cones_; }

 protected:
  InternalRepresentation cones_;
};

}  // namespace eufs::map

#endif  // INCLUDE_MAP_LIB_TYPE_MAP_HPP_
