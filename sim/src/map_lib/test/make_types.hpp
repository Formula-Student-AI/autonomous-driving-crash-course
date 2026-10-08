#ifndef TEST_MAKE_TYPES_HPP_
#define TEST_MAKE_TYPES_HPP_

#include <map_lib/type/cone.hpp>

eufs::map::Cone MakeCone(const std::array<double, 2> &pos = {0, 0},
              const std::array<double, 4> &cov = {0, 0, 0, 0},
              const eufs::map::Color &color = eufs::map::Color::UNKNOWN) {
  return eufs::map::Cone(pos, cov, color);
}

#endif  // TEST_MAKE_TYPES_HPP_
