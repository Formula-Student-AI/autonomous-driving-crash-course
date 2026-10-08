#include <gmock/gmock.h>

#include "map_lib/association/batch/sequential_random.hpp"
#include "map_lib/association/filter/thresholding.hpp"
#include "map_lib/association/metric/euclidean.hpp"

using eufs::map::association::Euclidean;
using eufs::map::association::SequentialRandomAssociation;
using eufs::map::association::Threshold;

///////////////////
//  Check Build  //
///////////////////

TEST(CheckBuild, Build) { SequentialRandomAssociation<Threshold<Euclidean>, Euclidean>(1.0); }
