#ifndef INCLUDE_MAP_LIB_ASSOCIATION_ALIGN_ALIGN_SVD_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_ALIGN_ALIGN_SVD_HPP_

#include <eigen3/Eigen/SVD>
#include <numeric>
#include <algorithm>

#include "map_lib/type/map.hpp"

namespace eufs::map::association::align {

template <template <typename> class Container>
class AlignSVD {
 public:
  AlignSVD(const Container<Eigen::Vector2d> &origin, const Container<Eigen::Vector2d> &dest)
    : origin_(origin), dest_(dest) {}

  Eigen::Isometry2d FindTransformation() const {
    const Eigen::Vector2d origin_mean =
      std::reduce(origin_.begin(), origin_.end()) / origin_.size();
    const Eigen::Vector2d dest_mean = std::reduce(dest_.begin(), dest_.end()) / dest_.size();

    Eigen::Matrix2Xd normalized_origin(2, origin_.size());
    Eigen::Matrix2Xd normalized_dest(2, dest_.size());

    std::transform(origin_.begin(), origin_.end(), normalized_origin.colwise().begin(),
                  [&](const Eigen::Vector2d &point) { return (point - origin_mean).eval(); });

    std::transform(dest_.begin(), dest_.end(), normalized_dest.colwise().begin(),
                  [&](const Eigen::Vector2d &point) { return (point - dest_mean).eval(); });

    // Rotation matrix
    Eigen::Matrix2d H = normalized_origin * normalized_dest.transpose();
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix2d rotation = svd.matrixV() * svd.matrixU().transpose();

    if (rotation.determinant() < 0) {
      return Eigen::Isometry2d::Identity();
    }

    // Translation
    Eigen::Vector2d translation = dest_mean - rotation * origin_mean;

    Eigen::Isometry2d transform;
    transform.linear() = rotation;
    transform.translation() = translation;

    return transform;
  }

 private:
  Container<Eigen::Vector2d> origin_;
  Container<Eigen::Vector2d> dest_;
};

}  // namespace eufs::map::association::align

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_ALIGN_ALIGN_SVD_HPP_
