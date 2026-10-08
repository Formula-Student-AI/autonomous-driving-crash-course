# eufs_gmock_matchers

A library of gmock matchers for ROS messages.

## Usage

You can use the matchers in your unit tests like follows

```cpp
#include <gmock/gmock.h>
#include <eufs_msgs/msg/cone_with_covariance.hpp>

TEST(CheckToMsgCone, Check) {
  eufs_msgs::msg::ConeWithCovariance result = // Call some function you're testing;
  eufs_msgs::msg::ConeWithCovariance expected = // Expected result;

  // To check that the two messages are equal:
  EXPECT_THAT(result, eufs::testing::ConeWithCovarianceEq(exp));
}
```

## Examples

Lots of repositories within the [SLAM group](https://gitlab.com/eufs/localisation_group/slam) utilise this library if you want further examples.