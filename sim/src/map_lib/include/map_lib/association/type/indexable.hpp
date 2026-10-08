#ifndef INCLUDE_MAP_LIB_ASSOCIATION_TYPE_INDEXABLE_HPP_
#define INCLUDE_MAP_LIB_ASSOCIATION_TYPE_INDEXABLE_HPP_

template <typename MapLike>
concept Indexable = requires(MapLike map_like, std::size_t idx) { map_like.at(idx); }; // NOLINT

#endif  // INCLUDE_MAP_LIB_ASSOCIATION_TYPE_INDEXABLE_HPP_
