import { PanelExtensionContext } from "@foxglove/studio";
import {useCallback, useLayoutEffect, useState} from "react";
import ReactDOM from "react-dom";
import {MainSwitch} from "./gross_funk_utils/MainSwitch";
import {EBS} from "./gross_funk_utils/EBS";
import "./stylesheets/general.css"
import {ErrorAlert} from "./general_components/ErrorAlert";
import "./stylesheets/general.css"

type State = {
  serviceName: string;
  request: string;
  response?: unknown;
  error?: Error | undefined;
};

function GrossFunk({ context }: { context: PanelExtensionContext }): JSX.Element {
    const [state, setState] = useState<State>({ serviceName: "/ebs", request: "{}" });
    const [as_state, setAs_state] = useState<String | undefined>("off")



    useLayoutEffect(() => {
        context.watch("currentFrame");
        context.subscribe([{topic: "/sim/ros_can/state_str"}])
    }, []);

    context.onRender = (renderState, done) => {
        let new_state = (renderState.currentFrame?.[0]?.["message"] as any)?.["data"]?.split(":")[2].trim()
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
                JSON.stringify(response); // Attempt serializing the response, to throw an error on failure
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

    return (
        <div style={{width: "100%", display: "grid", placeItems: "center", height: "97%", position: "absolute", bottom: "0", padding: "1rem" }}>
            <div className="" style={{width:"70%",height:"100%", display: "grid", placeItems: "center", gridTemplateRows: "2fr 1fr"}}>
                <MainSwitch callService={callService} serviceName={"/go"} as_state={as_state}/>
                <EBS callService={callService} serviceName={"/ebs"}/>
                {context.callService == undefined && (
                    <ErrorAlert  message="Calling services is not supported by this connection"/>
                )}
            </div>
        </div>
    );
}

export function initExamplePanel(context: PanelExtensionContext): () => void {
        ReactDOM.render(<GrossFunk context={context} />, context.panelElement);

      // Return a function to run when the panel is remove
    return () => {
        ReactDOM.unmountComponentAtNode(context.panelElement);
    };
}
