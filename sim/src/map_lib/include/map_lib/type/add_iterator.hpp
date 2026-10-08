#ifndef INCLUDE_MAP_LIB_TYPE_ADD_ITERATOR_HPP_
#define INCLUDE_MAP_LIB_TYPE_ADD_ITERATOR_HPP_

namespace eufs::map {

template <class C>
class AddIterator {
 protected:
  C* container;

 public:
  typedef C container_type;
  typedef void value_type;
  typedef int difference_type;
  typedef void pointer;
  typedef void reference;

  explicit AddIterator(C& __x) : container(&__x) {}

  AddIterator<C>& operator=(const typename C::value_type& val) {
    container->Add(val);
    return *this;
  }

  AddIterator<C>& operator*() { return *this; }

  AddIterator<C>& operator++() { return *this; }

  AddIterator<C>& operator++(int) { return *this; }
};

}  // namespace eufs::map

#endif  // INCLUDE_MAP_LIB_TYPE_ADD_ITERATOR_HPP_
