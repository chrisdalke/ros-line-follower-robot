import React, {useEffect, useMemo, useState} from 'react';
import useWebSocket, {ReadyState } from 'react-use-websocket';

const RobotContext = React.createContext(null);

export function RobotContextProvider(props) {
    const [numMessagesBroadcast, setNumMessagesBroadcast] = useState(0);
    const [numMessagesReceived, setNumMessagesReceived] = useState(0);

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
        setNumMessagesReceived((i) => i + 1);
    }, [lastMessage]);

    const context = useMemo(() => ({
        online: readyState === ReadyState.OPEN,
        numMessagesBroadcast,
        numMessagesReceived,
        sendMessage: (message) => {
            setNumMessagesBroadcast((i) => i + 1);
            sendMessage(JSON.stringify(message));
        }
    }), [readyState, sendMessage, numMessagesBroadcast, numMessagesReceived]);
    return (
        <RobotContext.Provider value={context} {...props} />
        );
}

export default RobotContext;
