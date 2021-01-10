import {Card} from '@blueprintjs/core';
import CardHeading from '../../ui/CardHeading/CardHeading';

function MotorOutputCard() {
    return (<Card>
        <CardHeading title="Motor Output" />
        <h4>Left Motor</h4>
        <h4>Right Motor</h4>
    </Card>)
}

export default MotorOutputCard;
