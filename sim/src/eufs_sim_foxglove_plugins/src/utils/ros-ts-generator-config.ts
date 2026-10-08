const ROS_SHARE_PATH = "/opt/ros/humble/share";
const EUFS_MASTER = process.env.EUFS_MASTER;

const config = {
    output: "./src/types/ros_msgs.ts",
    rosVersion: 2,
    input: [
        {
            namespace: "std_msgs",
            path: ROS_SHARE_PATH + "/std_msgs",
        },
        {
            namespace: "geometry_msgs",
            path: ROS_SHARE_PATH + "/geometry_msgs",
        },
        {
            namespace: "visualization_msgs",
            path: ROS_SHARE_PATH + "/visualization_msgs",
        },
        {
            namespace: "sensor_msgs",
            path: ROS_SHARE_PATH + "/sensor_msgs",
        },
        {
            namespace: "eufs_msgs",
            path: EUFS_MASTER + "/src/infrastructure/eufs_msgs",
        },
    ],
    typePrefix: "",
};

export default config;
