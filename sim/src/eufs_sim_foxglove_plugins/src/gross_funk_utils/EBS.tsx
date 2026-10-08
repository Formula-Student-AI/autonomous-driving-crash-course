import "../stylesheets/gross_funk.css"

export const EBS = ({ callService, serviceName }: { callService: (serviceName: string, request: string) => Promise<void>, serviceName: string}) => {

    return (
        <button className="ebs_main clear_button" style={{width: "120px", height:"120px"}} onClick={async () => await callService(serviceName, "{}")}>

        </button>
    );
};