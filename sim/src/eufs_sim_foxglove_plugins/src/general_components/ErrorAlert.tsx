import "../stylesheets/general.css"

export const ErrorAlert = ({ message } : { message : String }) => {
    return (
        <div className="alert_block_container">
            <p className="alert">{message}</p>
        </div>
    );
};