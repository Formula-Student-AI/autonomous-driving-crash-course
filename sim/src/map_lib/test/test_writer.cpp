#include <gmock/gmock.h>

#include <array>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "map_lib/io/writer.hpp"
#include "map_lib/type/cone.hpp"
#include "map_lib/testing/cone.hpp"

using eufs::map::Color;
using eufs::map::Cone;

///////////////////////////////////
//  Helper method to make cones  //
///////////////////////////////////

Cone MakeCone(const std::array<double, 2> &pos, const std::array<double, 4> &cov,
              const Color &color = Color::UNKNOWN) {
  return Cone(pos, cov, color);
}

//////////////////////////////
//  Check SaveCones Method  //
//////////////////////////////

TEST(CheckSaveCones, CheckSuccess) {
    auto package_share_directory = ament_index_cpp::get_package_share_directory("map_lib");
    auto sample_map_path = std::filesystem::path(package_share_directory) / "test" / "test_map.csv";
    auto write_map_path =
      std::filesystem::path(package_share_directory) / "test" / "test_write_map.csv";

    // Save to a csv file, then load it back in and check the contents
    auto blue = MakeCone({1.1, 0.9}, {0.1, 0.3, 0.3, 0.2}, Color::BLUE);
    auto yellow = MakeCone({0.1, 0.2}, {0.4, 0.6, 0.6, 0.5}, Color::YELLOW);

    eufs::map::io::SaveCones<Cone>({blue, yellow}, write_map_path);
    EXPECT_THAT(eufs::map::io::LoadCones<Cone>(write_map_path),
              ::testing::ElementsAreArray(
                {eufs::testing::ConeEq(blue), eufs::testing::ConeEq(yellow)}));


    // Compare the csv file which we saved to line-by-line directly with a sample map
    std::fstream samplefile(sample_map_path, std::ios::in);  // Sample file
    std::fstream writefile(write_map_path, std::ios::in);  // Write file

    EXPECT_TRUE(samplefile.is_open() && writefile.is_open());  // Test if the files open

    std::string samplefile_line, writefile_line;
    std::getline(samplefile, samplefile_line);  // Get a line from the sample file
    std::getline(writefile, writefile_line);  // Get a line from the write file

    EXPECT_EQ(samplefile_line, writefile_line);

    // Loop through all the lines in the file
    while (std::getline(samplefile, samplefile_line)) {
      EXPECT_TRUE(std::getline(writefile, writefile_line));
      EXPECT_EQ(samplefile_line, writefile_line);
    }

    samplefile.close();
    writefile.close();
}
