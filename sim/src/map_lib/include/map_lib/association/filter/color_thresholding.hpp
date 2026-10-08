#ifndef INCLUDE_MAP_LIB_ASSOCIATION_FILTER_COLOR_THRESHOLDING_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_FILTER_COLOR_THRESHOLDING_HPP_

#include <map_lib/type/cone.hpp>

namespace eufs::map::association {

template <typename MetricType>
struct ColorThreshold {
  /**
   * @brief Checks if the metric between two cones is below a threshold
   *
   * @tparam Metric Must take as argument two Cone types
   * @param c_1 First cone
   * @param c_2 Second cone
   * @param thresh Threshold below which metric must be to return true
   * @return true If metric is below threshold
   * @return false If metric is above threshold
   */
  template <typename ConeType1, typename ConeType2>
  bool operator()(const ConeType1 &c_1, const ConeType2 &c_2, double thresh) const {
    if (c_1.GetColor() == map::Color::UNKNOWN || c_2.GetColor() == map::Color::UNKNOWN) {
      return metric_(c_1, c_2) < thresh;
    }
    return metric_(c_1, c_2) < thresh && c_1.GetColor() == c_2.GetColor();
  }

 private:
  MetricType metric_;
};

}  // namespace eufs::map::association

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_FILTER_COLOR_THRESHOLDING_HPP_
