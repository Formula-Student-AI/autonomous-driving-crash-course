#ifndef INCLUDE_MAP_LIB_FOV_HPP_
#define INCLUDE_MAP_LIB_FOV_HPP_

#include <angles/angles.h>

namespace eufs::map {

struct SectorParameters {
  /**
   * @brief Range of vision in meters
  */
  const double max_horizontal_radius;

  /**
   * @brief Horizontal field of view in radians
  */
  const double horizontal_fov;
};

struct AnnulusParameters {
  /**
   * @brief Range of vision in meters
  */
  const double max_horizontal_radius;

  /**
   * @brief Horizontal field of view in radians
  */
  const double horizontal_fov;

  /**
   * @brief Radius in which cones are occluded
  */
  const double min_horizontal_radius;

  /**
   * @brief Center of field of view in radians
  */
  const double fov_center;
};

struct WithinSectorChecker {
  /**
   * @brief Construct new WithinSectorChecker object
   *
   * @param param Sector parameters
  */
  explicit WithinSectorChecker(const SectorParameters &param) : param_(param) {}

  /**
   * @brief Checks if the given cone is within the sector
   *
   * @param c Cone
   * @return true If cone is within sector
   * @return false If cone is outside sector
  */
  template <typename ConeType>
  bool operator()(const ConeType &c) const {
    const bool range_condition = c.position.norm() <= param_.max_horizontal_radius;

    const bool angle_condition = std::abs(std::atan2(c.position(1), c.position(0)))
      <= param_.horizontal_fov / 2;
    return range_condition && angle_condition;
  }

 private:
  SectorParameters param_;
};

struct WithinAnnulusChecker {
  /**
   * @brief Construct new WithinAnnulusChecker object
   *
   * @param param Annulus parameters
  */
  explicit WithinAnnulusChecker(const AnnulusParameters &param) : param_(param) {}

  /**
   * @brief Checks if the given cone is within the annulus
   *
   * @param c Cone
   * @return true If cone is within sector
   * @return false If cone is outside sector
  */
  template <typename ConeType>
  bool operator()(const ConeType &c) const {
    const bool range_condition =
      param_.min_horizontal_radius <= c.position.norm()
      && c.position.norm() <= param_.max_horizontal_radius;

    const bool angle_condition = std::abs(
      ::angles::normalize_angle(
        std::atan2(c.position(1), c.position(0)) - param_.fov_center))
        <= param_.horizontal_fov / 2;

    return range_condition && angle_condition;
  }

 private:
  AnnulusParameters param_;
};

}  // namespace eufs::map

#endif  // INCLUDE_MAP_LIB_FOV_HPP_
