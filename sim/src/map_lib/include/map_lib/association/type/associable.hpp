#ifndef INCLUDE_MAP_LIB_ASSOCIATION_TYPE_ASSOCIABLE_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_TYPE_ASSOCIABLE_HPP_

template <typename MapLike>
concept Associable = requires(MapLike map_like, std::size_t idx) {
  map_like.at(idx);
  map_like.size();
};  // NOLINT

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_TYPE_ASSOCIABLE_HPP_
