import "../stylesheets/gross_funk.css"

export const MainSwitch = ({ callService, serviceName, as_state }: { callService: (serviceName: string, request: string) => Promise<void>, serviceName: string, as_state: String | undefined}) => {
    return (
        <div className="main_switch_background" style={{backgroundColor: (as_state == "DRIVING" ? "rgb(64, 122, 0)" : "rgb(137, 34, 0)")}}>
            <div className={"main_switch_button " + (as_state == "DRIVING" ? "switch_up" : "switch_down")} onClick={async () => {
                await callService(serviceName, "{}")
            }}>
            </div>
        </div>
    );
};