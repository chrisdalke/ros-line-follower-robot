import {Card} from '@blueprintjs/core';
import CardHeading from '../../ui/CardHeading/CardHeading';
import './CameraCard.scss';

function CameraCard() {
    return (<Card>
        <CardHeading title="Camera Card" />
        <div className="CameraCard">
            <div className="CameraCard__noSignal">
                No Signal
            </div>
        </div>
    </Card>)
}

export default CameraCard;
