import type { StdMsgsHeader } from "./types/ros_msgs";
import { VisualizationMsgsMarkerConst } from "./types/ros_msgs";

interface ConeWithColorProbability {
    id: number;
    blue_prob: number;
    yellow_prob: number;
    orange_prob: number;
    big_orange_prob: number;
    unknown_prob: number;
    point: { x: number; y: number; z: number };
    covariance: number[];
}

interface ConeWithColorProbabilityArray {
    header: StdMsgsHeader;
    cones: ConeWithColorProbability[];
}

type ConeColor = { r: number; g: number; b: number; a: number };

function getDominantColor(cone: ConeWithColorProbability): ConeColor {
    const probs = [
        { color: { r: 0, g: 0, b: 1, a: 1 } as ConeColor,       p: cone.blue_prob },
        { color: { r: 1, g: 1, b: 0, a: 1 } as ConeColor,       p: cone.yellow_prob },
        { color: { r: 1, g: 0.549, b: 0, a: 1 } as ConeColor,   p: cone.orange_prob },
        { color: { r: 1, g: 0.271, b: 0, a: 1 } as ConeColor,   p: cone.big_orange_prob },
        { color: { r: 0.5, g: 0.5, b: 0.5, a: 1 } as ConeColor, p: cone.unknown_prob },
    ];
    return probs.reduce((a, b) => (b.p > a.p ? b : a)).color;
}

function isBigOrange(cone: ConeWithColorProbability): boolean {
    return cone.big_orange_prob >= Math.max(
        cone.blue_prob, cone.yellow_prob, cone.orange_prob, cone.unknown_prob
    );
}

function convertConeWithColorProbabilityArray(
    msg: ConeWithColorProbabilityArray
) {
    const deleteAll = {
        header: msg.header,
        ns: "",
        id: 0,
        type: VisualizationMsgsMarkerConst.CYLINDER,
        action: VisualizationMsgsMarkerConst.DELETEALL,
        pose: {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
        scale: { x: 0, y: 0, z: 0 },
        color: { r: 0, g: 0, b: 0, a: 0 },
        frame_locked: false,
        points: [],
        colors: [],
        text: "",
        mesh_resource: "",
        mesh_use_embedded_materials: false,
        lifetime: { sec: 0, nanosec: 0 },
    };

    const coneMarkers = msg.cones.map((cone, idx) => {
        const big = isBigOrange(cone);
        return {
            header: msg.header,
            ns: "cones",
            id: idx + 1,
            type: VisualizationMsgsMarkerConst.CYLINDER,
            action: VisualizationMsgsMarkerConst.ADD,
            pose: {
                position: { x: cone.point.x, y: cone.point.y, z: big ? 0.15 : 0.1525 / 2 },
                orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
            scale: { x: big ? 0.23 : 0.228, y: big ? 0.23 : 0.228, z: big ? 0.3 : 0.3225 },
            color: getDominantColor(cone),
            frame_locked: false,
            points: [],
            colors: [],
            text: "",
            mesh_resource: "",
            mesh_use_embedded_materials: false,
            lifetime: { sec: 0, nanosec: 500000000 },
        };
    });

    return { markers: [deleteAll, ...coneMarkers] };
}

const ConeWithColorProbabilityArrayConverter = {
    fromSchemaName: "eufs_msgs/msg/ConeWithColorProbabilityArray",
    toSchemaName: "visualization_msgs/msg/MarkerArray",
    converter: convertConeWithColorProbabilityArray,
};

export default ConeWithColorProbabilityArrayConverter;
