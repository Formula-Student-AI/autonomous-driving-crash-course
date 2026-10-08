import {PanelExtensionContext} from "@foxglove/studio";
import ReactDOM from "react-dom";
import "./stylesheets/map_set.css"
import "./stylesheets/MissionStatePanel.css"
import React, {useCallback, useEffect, useState} from "react";
import { ErrorAlert } from "./general_components/ErrorAlert";

type State = {
    serviceName: string;
    request: string;
    response?: unknown;
    error?: Error | undefined;
};
function MapSetPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
    const [selectedOption, setSelectedOption] = useState<string>("Select Map");
    const [retrievedMaps, setRetrievedMaps] = useState<string[] | undefined>([])
    const handleSelectChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
        setSelectedOption(event.target.value);
    };

    const [state, setState] = useState<State>({ serviceName: "/get_map", request: "{}" });
    const callService = useCallback(
        async (serviceName: string, request: string) => {
            if (!context.callService) {
                return;
            }
            setState({...state, serviceName})
            try {
                const response = await context.callService(serviceName, JSON.parse(request));
                if(serviceName === "/eufs_sim2/get_map") {
                    setRetrievedMaps((response as { map_path?: string })?.map_path?.toString().split(","));
                }
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
        [context.callService],
    );

    useEffect(() => {
        callService("/eufs_sim2/get_map", "{}").then(()=>{})
    }, []);

    return (
        <div style={{width: "100%", display: "flex", justifyContent: "center", alignItems: "center", height: "100%", padding: "1rem"}}>
            <div className="middle_block_alpha">
                <select className="custom-dropdown" id="dropdown" value={selectedOption} onChange={handleSelectChange}>`
                    <option style={{backgroundColor: "rgb(107, 24, 0)"}} value="Select Map">Select Map</option>
                    {
                        retrievedMaps?.map((path, index) => {
                            const cA = 220;
                            const cB = 53 - (index * 53) / retrievedMaps?.length ;
                            return <option key={index} className="suppress_selection" style={{
                                color: `rgb(${cA}, ${cA}, ${cA})`,
                                backgroundColor: `rgb(${cB}, ${cB}, ${cB})`
                            }} value={path}>{path.match(/\/([^/]+)\.csv$/)?.[1]}</option>
                        })
                    }
                </select>
                <button onClick={()=>{
                    if(selectedOption === "Select Map") return;
                    callService("/eufs_sim2/file", JSON.stringify({data: selectedOption})).then(()=>{}).catch(e=>console.log(e))
                }} className="set_map">Set</button>
            </div>
            {context.callService == undefined && (
                <ErrorAlert message="Calling services is not supported by this connection"/>
            )}
        </div>
    );  
};

export function initMapSetPanel(context: PanelExtensionContext): () => void {
    ReactDOM.render(<MapSetPanel context={context}/>, context.panelElement);

    return () => {
        ReactDOM.unmountComponentAtNode(context.panelElement);
    };
}
