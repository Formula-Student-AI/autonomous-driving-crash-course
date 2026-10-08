# eufs-sim-foxglove-plugins

A foxglove extension for eufs_sim & eufs_sim2.


## ConeArrayWithCovariance

This plugin provides a visualisation of the [`eufs_msgs::msg::ConeArrayWithCovariance`](https://gitlab.com/eufs/eufs_msgs/-/blob/master/msg/ConeArrayWithCovariance.msg) message. It allows visualisations of cones and their covariance. The cone and covariance visuals can individually be toggled under the namespace property in the display panel.


## WheelForceArrows

This plugin visualises [`eufs_msgs::msg::CarForces`](https://gitlab.com/eufs/eufs_msgs/-/blob/master/msg/CarForces.msg) messages as individual wheel force arrows for easy debugging and aesthetics, as shown in [this gif](https://gitlab.com/-/project/51879351/uploads/6f29f5b4a39a5c8725ddd75806547442/arrows.gif).


## ROS2 message types

To update the TypeScript interfaces for EUFS custom ROS2 message types in the `src/types/ros_msgs.ts` file, run the following command:

```sh
npm run generate-msg-types
```


## Installing local changes

To install any changes made locally, run the following command:

```sh
npm run local-install
```


## Publishing extensions

To publish local changes to the organisation, follow the guidance [here](https://docs.foxglove.dev/docs/visualization/extensions/publish/).