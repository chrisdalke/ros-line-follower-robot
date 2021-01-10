import React, {useMemo} from 'react';

const RobotContext = React.createContext(null);

export function RobotContextProvider(props) {
    const context = useMemo(() => ({
        online: false,
        numMessagesBroadcast: 0,
        numMessagesReceived: 0
    }), []);
    return (
        <RobotContext.Provider value={context} {...props} />
        );
}

export default RobotContext;
