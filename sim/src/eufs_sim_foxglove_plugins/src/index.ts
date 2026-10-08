import { ExtensionContext } from "@foxglove/studio";
import { initExamplePanel } from "./GrossFunk";
import { initJoystickPanel } from "./JoystickPanel";
import ConeArrayWithCovarianceConverter from "./ConeArrayWithCovariance";
import ConeWithColorProbabilityArrayConverter from "./ConeWithColorProbabilityArray";
import CarForcesConverter from "./WheelForceArrows";
import {initMapSetPanel} from "./MapSetPanel";
import {initMissionStatePanel} from "./MissionStatePanel";


export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Gross Funk", initPanel: initExamplePanel });
  extensionContext.registerPanel({ name: "Joystick", initPanel: initJoystickPanel });
  extensionContext.registerPanel({ name: "Set Map", initPanel: initMapSetPanel });
  extensionContext.registerPanel({ name: "Mission State", initPanel: initMissionStatePanel });
  extensionContext.registerMessageConverter(ConeArrayWithCovarianceConverter);
  extensionContext.registerMessageConverter(ConeWithColorProbabilityArrayConverter);
  extensionContext.registerMessageConverter(CarForcesConverter);
}
