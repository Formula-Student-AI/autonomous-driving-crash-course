import type {
    EufsMsgsCarForces,
    GeometryMsgsVector3,
} from "./types/ros_msgs";

import { VisualizationMsgsMarkerConst } from "./types/ros_msgs";

function arrowBuilder(frame: string, strength: number, direction: GeometryMsgsVector3, id: number, namespace: string) {
    const scaleFactor = 0.005;

    // Offset by 0.3 if it's a left wheel, and -0.3 if a right one
    let offset = 0.3;
    if (frame[0] == 'r') {
        offset *= -1;
    }

    return {
        header: {
            stamp: {sec: 0, nanosec: 0},
            frame_id: frame
        },
        ns: namespace,
        id: id,
        type: VisualizationMsgsMarkerConst.ARROW,
        action: VisualizationMsgsMarkerConst.ADD,
        pose: {
            position: {x: 0, y: offset, z: 0},
            orientation: { x: direction.x, y: direction.z, z: direction.y, w: 1.0 },
        },
        scale: {
            x: scaleFactor * strength,
            y: scaleFactor * strength * 0.1,
            z: scaleFactor * strength * 0.1
        },
        color: {
            r: direction.x,
            g: direction.y,
            b: direction.z,
            a: 0.3
        },
        lifetime: { sec: 0, nanosec: 0 },
        frame_locked: true
    };
}

function convertCarForces(inputMessage: EufsMsgsCarForces) {
    const Colour = {
        Red: {x: 1, y: 0, z:0},
        Green: {x: 0, y: 1, z:0},
        Blue: {x: 0, y: 0, z:1},
    }

    // x is longitudinal and red
    // y is lateral and green
    // z is vertical and blue

    let markers = [
        arrowBuilder("left_front_wheel", inputMessage.front_left_longitudinal, Colour.Red, 0, "x"),
        arrowBuilder("left_front_wheel", inputMessage.front_left_lateral, Colour.Green, 1, "y"),
        arrowBuilder("left_front_wheel", inputMessage.front_left_vertical, Colour.Blue, 2, "z"),
        arrowBuilder("right_front_wheel", inputMessage.front_right_longitudinal, Colour.Red, 3, "x"),
        arrowBuilder("right_front_wheel", inputMessage.front_right_lateral, Colour.Green, 4, "y"),
        arrowBuilder("right_front_wheel", inputMessage.front_right_vertical, Colour.Blue, 5, "z"),
        arrowBuilder("left_rear_wheel", inputMessage.rear_left_longitudinal, Colour.Red, 6, "x"),
        arrowBuilder("left_rear_wheel", inputMessage.rear_left_lateral, Colour.Green, 7, "y"),
        arrowBuilder("left_rear_wheel", inputMessage.rear_left_vertical, Colour.Blue, 8, "z"),
        arrowBuilder("right_rear_wheel", inputMessage.rear_right_longitudinal, Colour.Red, 9, "x"),
        arrowBuilder("right_rear_wheel", inputMessage.rear_right_lateral, Colour.Green, 10, "y"),
        arrowBuilder("right_rear_wheel", inputMessage.rear_right_vertical, Colour.Blue, 11, "z"),
    ]

    return { markers };
}

const carForcesConverter = {
    fromSchemaName: "eufs_msgs/msg/CarForces",
    toSchemaName: "visualization_msgs/msg/MarkerArray",
    converter: convertCarForces,
};
export default carForcesConverter;
