import {STATUS} from "../utils/Enums";
import React from "react";

export const StatusText: React.FC<status_props> = ({message, status}) => {

    return (
        <div style={{paddingLeft: "0.5em", color: status, textShadow: status === STATUS.READY || status === STATUS.EMERGENCY ? "0px 0px 3px rgba(255, 255, 255, 0.5)" : "none"}}>
            {message}
        </div>
    );
};
interface status_props {
    message: string;
    status: STATUS; // Specify the enum type in the prop
}