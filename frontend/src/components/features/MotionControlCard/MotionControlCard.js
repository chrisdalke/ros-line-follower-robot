import {Button, Card, Switch} from '@blueprintjs/core';
import CardHeading from '../../ui/CardHeading/CardHeading';
import './MotionControlCard.scss';
import {useContext, useEffect, useRef, useState} from 'react';
import RobotContext from '../../context/RobotContext/RobotContext';
import {useThrottleFn} from 'react-use';
import moment from 'moment';

function MotionControlCard() {
    const {
        sendMessage
    } = useContext(RobotContext);

    const returnToCenterMode = useRef(true);
    const [returnToCenter, setReturnToCenter] = useState(true);
    const containerRef = useRef(null);

    const size = 250;

    const isDragging = useRef(false);
    const [draggingState, setDraggingState] = useState({
        dragging: false,
        speed: 0,
        dir: 0
    });

    const clearMotion = () => {
        isDragging.current = false;
        setDraggingState({
            dragging: false,
            speed: 0,
            dir: 0
        });
        sendMessage({
            timestamp: moment().valueOf(),
            key: "speed",
            value: "0"
        });
        sendMessage({
            timestamp: moment().valueOf(),
            key: "dir",
            value: "0"
        });
    }

    const computeSpeedDirection = (event) => {
        const bounds = containerRef.current.getBoundingClientRect();
        const relX = ((event.clientX - bounds.left) / size);
        const relY = ((event.clientY - bounds.top) / size);

        // Compute distance between x,y and center of circle
        var a = (relX - 0.5) * 2;
        var b = (relY - 0.5) * 2;
        var dist = Math.min(Math.max(Math.sqrt(a*a + b*b), 0), 1);
        if (dist > 0.8) {
            dist = 1;
        }

        var dy = relX - 0.5;
        var dx = relY - 0.5;
        var dir = (Math.atan2(dy, dx) * 180 / Math.PI) + 180;


        setDraggingState({
            dragging: true,
            speed: dist,
            dir: dir
        });
    }

    const onMouseDown = (event) => {
        if (event.target !== containerRef.current) {
            return;
        }
        computeSpeedDirection(event);

        isDragging.current = true;
    };

    const onMouseMove = (event) => {
        if (isDragging.current) {
            computeSpeedDirection(event);
        }
    };
    const throttledValue = useThrottleFn(draggingState => {
        if (isDragging.current) {
            sendMessage({
                timestamp: moment().valueOf(),
                key: "speed",
                value: `${draggingState.speed}`
            });
            sendMessage({
                timestamp: moment().valueOf(),
                key: "dir",
                value: `${draggingState.dir}`
            });
        }
    }, 50, [draggingState]);

    const onMouseUp = (event) => {
        if (isDragging.current) {
            isDragging.current = false;
            if (returnToCenterMode.current) {
                clearMotion();
            } else {
                setDraggingState((s) => ({
                    dragging: false,
                    ...s
                }));
            }
        }
    };

    useEffect(() => {
        window.addEventListener('mousedown', onMouseDown);
        window.addEventListener('mousemove', onMouseMove);
        window.addEventListener('mouseup', onMouseUp);

        return () => {
            window.removeEventListener('mousedown', onMouseDown);
            window.removeEventListener('mousemove', onMouseMove);
            window.removeEventListener('mouseup', onMouseUp);
        };
    }, []);

    const handleX = 0.5 + Math.cos((-(Math.PI / 2) - draggingState.dir * (Math.PI / 180))) * (draggingState.speed / 2);
    const handleY = 0.5 + Math.sin((-(Math.PI / 2) - draggingState.dir * (Math.PI / 180))) * (draggingState.speed / 2);
    const handleStyle = {
        top: `${(handleY * 100)}%`,
        left: `${(handleX * 100)}%`
    }
    const horizLineStyle = {
        top: `${(handleY * 100)}%`
    }
    const verticalLineStyle = {
        left: `${(handleX * 100)}%`
    }

    return (<Card>
        <CardHeading title="Motion Control" />
        <div className="MotionControlCard">
        <div ref={containerRef} className="MotionControlContainer">
            <div className="MotionControlContainer__circle" />
            <div className="MotionControlContainer__handle" style={handleStyle} />
            <div className="MotionControlContainer__horizontalLine" style={horizLineStyle} />
            <div className="MotionControlContainer__verticalLine" style={verticalLineStyle} />
            <div className="MotionControlContainer__labels">
                dragging: {draggingState.dragging ? 'true' : 'false'}<br/>
                speed: {draggingState.speed.toFixed(3)}<br/>
                dir: {draggingState.dir.toFixed(2)}<br/>
            </div>
        </div>
        <div>
        <b>Motion Settings</b><br />
        <Switch checked={returnToCenter} label="Return to Center" onChange={(event) => {
            returnToCenterMode.current = !returnToCenter;
            setReturnToCenter(returnToCenterMode.current);
            if (returnToCenterMode.current) {
                clearMotion();
            }
        }} />
        <Button disabled={returnToCenter} small onClick={clearMotion}>Clear</Button>
        </div>
        </div>
    </Card>)
}

export default MotionControlCard;
