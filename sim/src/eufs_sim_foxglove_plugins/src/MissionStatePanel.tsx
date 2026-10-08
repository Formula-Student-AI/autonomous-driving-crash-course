/* eslint-disable @typescript-eslint/no-unsafe-member-access */
import {PanelExtensionContext} from "@foxglove/studio";
import ReactDOM from "react-dom";
import "./stylesheets/general.css"
import "./stylesheets/MissionStatePanel.css"
import React, {useCallback, useLayoutEffect, useState} from "react";
import {StatusText} from "./general_components/StatusText";
import {STATUS} from "./utils/Enums";
import {ErrorAlert} from "./general_components/ErrorAlert";
import {number} from "mathjs";

type State = {
    serviceName: string;
    request: string;
    response?: unknown;
    error?: Error | undefined;
};
function MissionStatePanel({ context }: { context: PanelExtensionContext }): JSX.Element {
    const mission_list = [
        "Select Mission",
        'Acceleration',
        'Skidpad',
        'Autocross',
        'Track Drive',
        'Manual Driving',
        'ADS EBS Test',
        'ADS Inspection' ,
        'DDT Inspection A',
        'DDT Inspection B',
        'DDT Autonomous Demo'
];

    const [selectedOption, setSelectedOption] = useState<number>(0);
    const handleSelectChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
        setSelectedOption(number(event.target.value));
    };
    const [state, setState] = useState<State>({ serviceName: "/ros_can/set_mission", request: "{}" });
    const [as_state, setAs_state] = useState<String | undefined>("off")
    const [mission_curr, setMission_curr] = useState<String | undefined>("Select Mission")


    useLayoutEffect(() => {
        context.watch("currentFrame");
        context.subscribe([{topic: "/sim/ros_can/state_str"}])
    }, []);

    context.onRender = (renderState, done) => {
        let new_state = (renderState.currentFrame?.[0]?.["message"] as any)?.["data"]?.split(":")[2]
        let new_mission = (renderState.currentFrame?.[0]?.["message"] as any)?.["data"]?.split(":")[1].split(" ")[1]
        if(new_mission !== mission_curr && new_mission !== undefined) {
            setMission_curr(new_mission);
        }
        if(new_state !== as_state && new_state !== undefined) {
            setAs_state(new_state);
        }
        done();
    };

    const callService = useCallback(
        async (serviceName: string, request: string) => {
            if (!context.callService) {
                return;
            }
            setState({...state, serviceName})
            try {
                const response = await context.callService(serviceName, JSON.parse(request));
                JSON.stringify(response);
                setState((oldState) => ({
                    ...oldState,
                    response,
                    error: undefined,
                }));
            } catch (error) {
                setState((oldState) => ({ ...oldState, error: error as Error }));
                console.error(error);
            }
        },
        [state, context],
    );

    return (
        <div style={{width: "100%", display: "grid", placeItems: "center", position: "absolute", height: "97%", padding: "1rem" }}>
            <div className="" style={{width:"100%", height:"100%", paddingBottom:"1rem", display: "grid", placeItems: "center", gridTemplateRows: "1fr 3fr 1fr"}}>
                <select className="custom-dropdown" id="dropdown" value={selectedOption} onChange={handleSelectChange}>
                    {
                        mission_list.map((mission, index) => {
                            const cA = 220;
                            const cB = 53 - (index * 53) / mission_list.length;
                        return <option key={index} className="suppress_selection" style={{color:`rgb(${cA}, ${cA}, ${cA})` , backgroundColor: `rgb(${cB}, ${cB}, ${cB})`}}
                    value={index}>{mission}</option>
                })
                    }
                </select>
                <div className="max_out middle_block">
                    <div className="max_out curr_state flex items-center" style={{alignItems: "center"}}>Mission: <StatusText message={mission_curr as string} status={selectedOption === 0 ? STATUS.INACTIVE : STATUS.ACTIVE} /></div>
                    <div className="max_out curr_state flex items-center" style={{alignItems: "center"}}>State: <StatusText message={as_state as string} status={STATUS.WAITING} /></div>
                </div>
                <div style={{display: "flex",height:"100%", width: "100%"}}>
                    <button style={{ padding: "0.25rem"}} className="max_out" onClick={async () => {
                        if (selectedOption !== 0) {
                            const request = {
                                mission: selectedOption
                            }
                            await callService("/set_mission", JSON.stringify(request))
                        }
                    }}>
                        <div>Set Mission</div>
                    </button>
                    <button style={{ padding: "0.25rem"}} className="max_out_red" onClick={async () => {
                        await callService("/reset", "{}")
                    }}>
                        <div>Reset</div>
                    </button>
                </div>
                {context.callService == undefined && (
                    <ErrorAlert message="Calling services is not supported by this connection"/>
                )}
            </div>
        </div>
    );
}

export function initMissionStatePanel(context: PanelExtensionContext): () => void {
    ReactDOM.render(<MissionStatePanel context={context}/>, context.panelElement);

    return () => {
        ReactDOM.unmountComponentAtNode(context.panelElement);
    };
}