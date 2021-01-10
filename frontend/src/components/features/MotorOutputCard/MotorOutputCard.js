import {Card, ProgressBar, Intent} from '@blueprintjs/core';
import CardHeading from '../../ui/CardHeading/CardHeading';
import RobotContext from '../../context/RobotContext/RobotContext';
import {useContext} from 'react';
import './MotorOutputCard.scss';

function MotorOutputCard() {
    const {
        leftMotorSpeed,
        rightMotorSpeed
    } = useContext(RobotContext);

    return (<Card>
        <CardHeading title="Motor Output" />
        <div className="MotorOutputCard">
            <div className="MotorOutputCard__outputChannel">
                <div className="MotorOutputCard__outputChannel__label">
                    <div style={{flex: '1'}}>Left Motor</div>
                    <div>{(leftMotorSpeed * 100).toFixed(0)}%</div>
                </div>
                <div className="MotorOutputCard__outputChannel__value">
                    Control Signal: {leftMotorSpeed.toFixed(2)} PWM: {(leftMotorSpeed * 255).toFixed(2)}
                </div>
                <ProgressBar animate={false} stripes={false} intent={Intent.PRIMARY} value={leftMotorSpeed} />
            </div>
            <div className="MotorOutputCard__outputChannel">
                <div className="MotorOutputCard__outputChannel__label">
                    <div style={{flex: '1'}}>Right Motor</div>
                    <div>{(rightMotorSpeed * 100).toFixed(0)}%</div>
                </div>
                <div className="MotorOutputCard__outputChannel__value">
                    Control Signal: {rightMotorSpeed.toFixed(2)} PWM: {(rightMotorSpeed * 255).toFixed(2)}
                </div>
                <ProgressBar animate={false} stripes={false} intent={Intent.PRIMARY} value={rightMotorSpeed} />
            </div>
        </div>
    </Card>)
}

export default MotorOutputCard;
