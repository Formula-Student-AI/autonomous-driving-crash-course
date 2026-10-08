/* eslint-disable @typescript-eslint/no-unsafe-member-access */
import {
  PanelExtensionContext,
  Topic,
  // MessageEvent,
  SettingsTreeAction,
  SettingsTreeNode,
  SettingsTreeNodes,
} from "@foxglove/studio";
import React, { useCallback, useLayoutEffect, useEffect, useState } from "react";
import ReactDOM from "react-dom";
import { JoystickManagerOptions, Position } from "nipplejs";
import nipplejs from "nipplejs";
import { set } from "lodash";

type PanelState = {
  outmsg?: string;
};

let CURRENT_SCHEME: string = "";

let currentTopic: string = "/cmd";

type Config = {
  topic: string;
  publishRate: number;
  maxLinearSpeed: number;
  maxAngularSpeed: number;
};

function buildSettingsTree(config: Config, topics: readonly Topic[]): SettingsTreeNodes {
  const general: SettingsTreeNode = {
    label: "General",
    fields: {
      topic: {
        label: "Topic",
        input: "autocomplete",
        value: config.topic,
        items: topics.map((t) => t.name),
      },
      publishRate: { label: "Publish rate", input: "number", value: config.publishRate },
      maxLinearSpeed: { label: "Max linear", input: "number", value: config.maxLinearSpeed },
      maxAngularSpeed: { label: "Max angular", input: "number", value: config.maxAngularSpeed },
    },
  };

  return { general };
}

function JoystickPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<readonly Topic[]>([]);

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const { saveState } = context;

  const [colorScheme, setColorScheme] = useState<"dark" | "light">("light");

  const [state, setState] = useState<PanelState>(() => {
    return context.initialState as PanelState;
  });

  const [config, setConfig] = useState<Config>(() => {
    const partialConfig = context.initialState as Config;

    const {
      topic = currentTopic,
      publishRate = 5,
      maxLinearSpeed = 1,
      maxAngularSpeed = 1,
    } = partialConfig;

    return {
      topic,
      publishRate,
      maxLinearSpeed,
      maxAngularSpeed,
    };
  });

  const settingsActionHandler = useCallback((action: SettingsTreeAction) => {
    if (action.action !== "update") {
      return;
    }
    setState({ outmsg: action.payload.path.slice(1) + " " + action.payload.value });

    setConfig((previous) => {
      const newConfig = { ...previous };
      set(newConfig, action.payload.path.slice(1), action.payload.value);

      if (newConfig.publishRate < 1) {
        newConfig.publishRate = 1;
      }
      if (newConfig.maxLinearSpeed < 0) {
        newConfig.maxLinearSpeed = 0;
      }
      if (newConfig.maxAngularSpeed < 0) {
        newConfig.maxAngularSpeed = 0;
      }

      if (newConfig.topic !== currentTopic) {
        context.unadvertise?.(currentTopic);
        currentTopic = newConfig.topic;
        context.advertise?.(currentTopic, "ackermann_msgs/msg/AckermannDriveStamped");
      }

      config.topic = newConfig.topic;
      config.publishRate = newConfig.publishRate;
      config.maxLinearSpeed = newConfig.maxLinearSpeed;
      config.maxAngularSpeed = newConfig.maxAngularSpeed;

      return newConfig;
    });
  }, []);

  let startPoint: Position;
  let timer: ReturnType<typeof setInterval> | undefined;
  let diffValue: [number, number];

  const message = {
    header: {
      seq: 0,
      stamp: { sec: 0, nsec: 0 },
      frame_id: "",
    },
    drive: {
      steering_angle: 0,
      steering_angle_velocity: 0,
      speed: 0,
      acceleration: 0,
      jerk: 0,
    },
  };

  let cmdMove = (lx: number, az: number) => {
    message.drive.acceleration = lx * config.maxLinearSpeed;
    message.drive.steering_angle = az * config.maxAngularSpeed;
    context.publish?.(currentTopic, message);
  };

  let manager: nipplejs.JoystickManager;

  let init_nipple = (colorScheme: string) => {
    let options: JoystickManagerOptions = {
      zone: document.getElementById("nipple_zone") as HTMLDivElement,
      color: colorScheme === "light" ? "black" : "white",
      size: 200,
      restOpacity: 0.8,
      mode: "static",
      dynamicPage: true,
      position: { left: "50%", top: "50%" },
    };
    manager = nipplejs.create(options);

    manager.on("start", (_, data) => {
      startPoint = data.position;
      timer = setInterval(() => {
        cmdMove(diffValue[0], diffValue[1]);
      }, 1000 / config.publishRate);
    });

    manager.on("move", (evt, data) => {
      console.log(evt);
      let x = startPoint.x - data.position.x;
      let y = startPoint.y - data.position.y;
      let resultX = (x / 100) * 1.5707;
      let resultY = (y / 100) * 1.0;
      diffValue = [resultY, resultX];
    });

    manager.on("end", () => {
      cmdMove(0, 0);
      clearInterval(timer);
    });
  };

  useEffect(() => {
    const tree = buildSettingsTree(config, topics);
    context.updatePanelSettingsEditor({
      actionHandler: settingsActionHandler,
      nodes: tree,
    });
    saveState(config);
  }, [config, context, saveState, settingsActionHandler, topics]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      setTopics(renderState.topics ?? []);

      if (renderState.colorScheme) {
        setColorScheme(renderState.colorScheme);
        if (renderState.colorScheme !== CURRENT_SCHEME) {
          CURRENT_SCHEME = renderState.colorScheme;
          manager.destroy();
          init_nipple(renderState.colorScheme);
        }
      }
    };

    context.watch("topics");

    context.advertise?.(currentTopic, "ackermann_msgs/msg/AckermannDriveStamped");

    context.watch("colorScheme");

    if (CURRENT_SCHEME === "") {
      CURRENT_SCHEME = colorScheme;
    }
    init_nipple(CURRENT_SCHEME);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const [acc, setAcc] = useState(0);
  const [st, setSt] = useState(0);

  useEffect(() => {
    const timer = setInterval(() => {
      cmdMove(acc, st);
    }, 1000 / config.publishRate);

    return () => {
      clearInterval(timer);
    };
  }, [acc, st]);

  // const handleKeyDown = useCallback(
  //   (e: React.KeyboardEvent) => {
  //     switch (e.code) {
  //       case "KeyW":
  //         if (acc === 0) {
  //           setAcc(1);
  //         }
  //         break;
  //       case "KeyA":
  //         if (st === 0) {
  //           setSt(-1);
  //         }
  //         break;
  //       case "KeyS":
  //         if (acc === 0) {
  //           setAcc(-1);
  //         }
  //         break;
  //       case "KeyD":
  //         if (st === 0) {
  //           setSt(1);
  //         }
  //         break;
  //       default:
  //         break;
  //     }
  //     e.stopPropagation();
  //     e.preventDefault();
  //   },
  //   [acc, st],
  // );
  const handleKeyUp = useCallback(
    (e: React.KeyboardEvent) => {
      switch (e.code) {
        case "KeyW":
          if (acc === 1) {
            setAcc(0);
          }
          break;
        case "KeyA":
          if (st === -1) {
            setSt(0);
          }
          break;
        case "KeyS":
          if (acc === -1) {
            setAcc(0);
          }
          break;
        case "KeyD":
          if (st === 1) {
            setSt(0);
          }
          break;
        default:
          break;
      }
      e.stopPropagation();
      e.preventDefault();
    },
    [acc, st],
  );

  const onKeyDown = useCallback(
    (event: React.KeyboardEvent) => {
      if (event.key === "3") {
        setColorScheme(colorScheme === "dark" ? "light" : "dark");
        event.stopPropagation();
        event.preventDefault();
      }
    },
    [setColorScheme, colorScheme],
  );

  // useEffect(() => {
  //   document.addEventListener("keydown", handleKeyDown);
  //   document.addEventListener("keyup", handleKeyUp);
  //   return () => {
  //     document.removeEventListener("keydown", handleKeyDown);
  //     document.removeEventListener("keyup", handleKeyUp);
  //   };
  // });

  return (
    <div style={{ padding: "1rem" }} tabIndex={0} onKeyDown={onKeyDown} onKeyUp={handleKeyUp}>
      <div id="nipple_zone"></div>

      <div>{state.outmsg}</div>
      <div>{colorScheme}</div>
    </div>
  );
}

export function initJoystickPanel(context: PanelExtensionContext): void {
  ReactDOM.render(<JoystickPanel context={context} />, context.panelElement);
}
