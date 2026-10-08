import { atan2, cos, eigs, sin, sqrt } from "mathjs";

import type {
    EufsMsgsConeArrayWithCovariance,
    EufsMsgsConeWithCovariance,
    GeometryMsgsPoint,
    StdMsgsHeader,
} from "./types/ros_msgs";
import { VisualizationMsgsMarkerConst } from "./types/ros_msgs";

type ConeType = "BLUE" | "YELLOW" | "ORANGE" | "BIG_ORANGE" | "UNKNOWN";

const MAX_CONES_TO_VISUALIZE = 250;

function getConeColor(coneType: ConeType) {
    switch (coneType) {
        case "BLUE":
            return { r: 0.0, g: 0.0, b: 1.0, a: 1.0 };
        case "YELLOW":
            return { r: 1.0, g: 1.0, b: 0.0, a: 1.0 };
        case "ORANGE":
            return { r: 1.0, g: 0.549, b: 0.0, a: 1.0 };
        case "BIG_ORANGE":
            return { r: 1.0, g: 0.271, b: 0.0, a: 1.0 };
        case "UNKNOWN":
            return { r: 0.5, g: 0.5, b: 0.5, a: 1.0 };
    }
}

function calculateCovarianceShape(
    origin: GeometryMsgsPoint,
    covariance: number[][]
) {
    const b = !covariance![0]![1];
    const c = !covariance![1]![0];

    // Check that the covariance matrix is in the correct format (basically just c == b)
    // Took the "correct" covariance form from https://cookierobotics.com/007/
    // Hopefully this eliminates the crash lmao
    if (c != b) {
        return {
            scale: {x:0, y:0, z:0},
            pose: {position: origin, orientation: {x:0, y:0, z:0, w:1}}
        }
    }

    const ans = eigs(covariance);

    const eigenvalues = ans.values as [number, number];
    const eigenvectors = ans.vectors as [[number, number], [number, number]];

    // 5.991 represents 95% confidence interval (see https://bit.ly/2ngvYxv)
    const scale_x = sqrt(5.991 * eigenvalues[0]) as number;
    const scale_y = sqrt(5.991 * eigenvalues[1]) as number;
    const scale = {
        x: 2 * scale_x,
        y: 2 * scale_y,
        z: 0.01,
    };

    const angle = atan2(eigenvectors[1][0], eigenvectors[0][0]);
    const angle_num = angle as unknown as number;
    const pose = {
        position: origin,
        orientation: {
            x: 0,
            y: 0,
            z: sin(0.5 * angle_num),
            w: cos(0.5 * angle_num),
        },
    };

    return {
        scale: scale,
        pose: pose,
    };
}

function deleteAllMarker(header: StdMsgsHeader) {
    return {
        header: header,
        ns: "",
        id: 0,
        type: VisualizationMsgsMarkerConst.MESH_RESOURCE,
        action: VisualizationMsgsMarkerConst.DELETEALL,
        pose: {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        },
        scale: { x: 1.0, y: 1.0, z: 1.0 },
        color: { r: 0.0, g: 0.0, b: 0.0, a: 1.0 },
        mesh_resource: "",
        lifetime: { sec: 0, nanosec: 0 },
        frame_locked: false,
        points: [],
        colors: [],
        text: "",
        mesh_use_embedded_materials: false,
    };
}

function convertCones(
    coneMessages: EufsMsgsConeWithCovariance[],
    header: StdMsgsHeader,
    baseIndex: number,
    coneType: ConeType
) {
    const scale =
        coneType == "BIG_ORANGE"
            ? {
                  x: 0.285,
                  y: 0.285,
                  z: 0.5,
              }
            : {
                  x: 0.228,
                  y: 0.228,
                  z: 0.325,
              };

    return coneMessages
        .map((msg: EufsMsgsConeWithCovariance, index: number) => {
            const cov = [
                [msg.covariance[0], msg.covariance[1]],
                [msg.covariance[2], msg.covariance[3]],
            ] as number[][];
            const covShape = calculateCovarianceShape(msg.point, cov);
            return [
                {
                    header: header,
                    ns: coneType.toLowerCase(),
                    id: index,
                    type: VisualizationMsgsMarkerConst.CYLINDER,
                    action: VisualizationMsgsMarkerConst.ADD,
                    pose: {
                        position: {
                            x: msg.point.x,
                            y: msg.point.y,
                            z: msg.point.z + scale.z / 2,
                        },
                        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                    },
                    scale,
                    color: getConeColor(coneType),
                    mesh_resource: "",
                    lifetime: { sec: 0, nanosec: 0 },
                    frame_locked: false,
                    points: [],
                    colors: [],
                    text: "",
                    mesh_use_embedded_materials: false,
                },
                {
                    header: header,
                    ns: "covariance",
                    id: baseIndex + index,
                    type: VisualizationMsgsMarkerConst.SPHERE,
                    action: VisualizationMsgsMarkerConst.ADD,
                    pose: covShape.pose,
                    scale: covShape.scale,
                    color: { r: 1.0, g: 0.271, b: 0.271, a: 0.7 },
                    mesh_resource: "",
                    lifetime: { sec: 0, nanosec: 0 },
                    frame_locked: false,
                    points: [],
                    colors: [],
                    text: "",
                    mesh_use_embedded_materials: false,
                },
            ];
        })
        .flat();
}

function convertConeArrayWithCovariance(
    inputMessage: EufsMsgsConeArrayWithCovariance
) {
    let markers = [deleteAllMarker(inputMessage.header)];
    const coneCount =
        inputMessage.blue_cones.length +
        inputMessage.yellow_cones.length +
        inputMessage.orange_cones.length +
        inputMessage.big_orange_cones.length +
        inputMessage.unknown_color_cones.length;

    if (coneCount > MAX_CONES_TO_VISUALIZE) {
        return { markers };
    }

    markers = markers.concat(
        convertCones(
            inputMessage.blue_cones,
            inputMessage.header,
            markers.length,
            "BLUE"
        )
    );
    markers = markers.concat(
        convertCones(
            inputMessage.yellow_cones,
            inputMessage.header,
            markers.length,
            "YELLOW"
        )
    );
    markers = markers.concat(
        convertCones(
            inputMessage.orange_cones,
            inputMessage.header,
            markers.length,
            "ORANGE"
        )
    );
    markers = markers.concat(
        convertCones(
            inputMessage.big_orange_cones,
            inputMessage.header,
            markers.length,
            "BIG_ORANGE"
        )
    );
    markers = markers.concat(
        convertCones(
            inputMessage.unknown_color_cones,
            inputMessage.header,
            markers.length,
            "UNKNOWN"
        )
    );
    return { markers };
}

// Full description of the ConeArrayWithCovariance converter
const coneArrayWithCovarianceConverter = {
    fromSchemaName: "eufs_msgs/msg/ConeArrayWithCovariance",
    toSchemaName: "visualization_msgs/msg/MarkerArray",
    converter: convertConeArrayWithCovariance,
};
export default coneArrayWithCovarianceConverter;
