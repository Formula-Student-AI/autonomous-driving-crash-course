#include "map_lib/io/reader.hpp"

namespace eufs::map::io {

bool ValidEUFSFormat(const std::string header) {
  std::string name;
  std::stringstream header_ss(header);
  for (size_t i = 0; i < EUFS_HEADER.size(); ++i) {
    std::getline(header_ss, name, ',');
    if (name != EUFS_HEADER.at(i)) {
      return false;
    }
  }
  return true;
}

}  // namespace eufs::map::io
