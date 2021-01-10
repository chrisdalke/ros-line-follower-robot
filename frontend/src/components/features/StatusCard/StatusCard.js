import {Card, Tag, Intent} from '@blueprintjs/core';
import CardHeading from '../../ui/CardHeading/CardHeading';
import './StatusCard.scss';
import RobotContext from '../../context/RobotContext/RobotContext';
import {useContext} from 'react';

function StatusCard() {
    const {
        online,
        numMessagesBroadcast,
        numMessagesReceived
    } = useContext(RobotContext);

    return (<Card>
        <CardHeading title="Link Status" />
        <div className="StatusCard">
            <Tag minimal intent={online ? Intent.SUCCESS : Intent.DANGER}>Socket Link Status: {online ? 'Online' : 'Offline'}</Tag><br />
            <Tag minimal>Packets Broadcast: {numMessagesBroadcast}</Tag>
            <Tag minimal>Packets Received: {numMessagesReceived}</Tag>
        </div>
    </Card>)
}

export default StatusCard;
