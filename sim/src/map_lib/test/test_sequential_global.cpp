#include <gmock/gmock.h>

#include "map_lib/association/batch/sequential_global.hpp"
#include "map_lib/association/filter/thresholding.hpp"
#include "map_lib/association/metric/euclidean.hpp"

using eufs::map::association::Euclidean;
using eufs::map::association::SequentialGlobalAssociation;
using eufs::map::association::Threshold;

///////////////////
//  Check Build  //
///////////////////

TEST(CheckBuild, Build) { SequentialGlobalAssociation<Threshold<Euclidean>, Euclidean>(1.0); }
