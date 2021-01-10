import React, {useEffect, useMemo, useState} from 'react';
import useWebSocket, {ReadyState } from 'react-use-websocket';

const RobotContext = React.createContext(null);

export function RobotContextProvider(props) {
    const [numMessagesBroadcast, setNumMessagesBroadcast] = useState(0);
    const [numMessagesReceived, setNumMessagesReceived] = useState(0);
    const [leftMotorSpeed, setLeftMotorSpeed] = useState(0);
    const [rightMotorSpeed, setRightMotorSpeed] = useState(0);

    const {
        sendMessage,
        lastMessage,
        readyState,
    } = useWebSocket(`ws://${window.location.hostname}:8080/realtime`, {
        shouldReconnect: (closeEvent) => true,
        reconnectAttempts: Number.MAX_SAFE_INTEGER,
        reconnectInterval: 5000,
    });

    useEffect(() => {
        if (!lastMessage) {
            return;
        }
        console.log(lastMessage);
        const jsonValue = JSON.parse(lastMessage.data);
        setNumMessagesReceived((i) => i + 1);
        if (jsonValue.key === "arduino.mleft") {
            setLeftMotorSpeed(Number.parseFloat(jsonValue.value));
        }
        if (jsonValue.key === "arduino.mright") {
            setRightMotorSpeed(Number.parseFloat(jsonValue.value));
        }
    }, [lastMessage]);

    const context = useMemo(() => ({
        online: readyState === ReadyState.OPEN,
        numMessagesBroadcast,
        numMessagesReceived,
        leftMotorSpeed,
        rightMotorSpeed,
        sendMessage: (message) => {
            setNumMessagesBroadcast((i) => i + 1);
            sendMessage(JSON.stringify(message));
        }
    }), [
        leftMotorSpeed,
        rightMotorSpeed,
        readyState,
        sendMessage,
        numMessagesBroadcast,
        numMessagesReceived
    ]);

    return (
        <RobotContext.Provider value={context} {...props} />
        );
}

export default RobotContext;
