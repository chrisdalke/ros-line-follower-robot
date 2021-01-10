import {Card, Navbar, Spinner} from '@blueprintjs/core';
import CameraCard from '../../features/CameraCard/CameraCard';
import MotionControlCard from '../../features/MotionControlCard/MotionControlCard';
import MotorOutputCard from '../../features/MotorOutputCard/MotorOutputCard';
import OdometryCard from '../../features/OdometryCard/OdometryCard';
import StatusCard from '../../features/StatusCard/StatusCard';
import RobotContext, {RobotContextProvider} from '../../context/RobotContext/RobotContext';
import {useContext} from 'react';
import './MainPage.scss';

function OfflineOverlay() {
    const {
        online
    } = useContext(RobotContext);

    if (online) {
        return null;
    } else {
        return (<div className="OfflineOverlay">
            <div className="OfflineOverlay__label">
                <div>Socket Link Status: Offline<br/></div>
                <div className="OfflineOverlay__loading">Waiting for Connection...<Spinner size={16}/></div>
            </div>
        </div>);
    }
}

function MainPage() {
  return (
      <RobotContextProvider>
          <OfflineOverlay />
          <Navbar>
              <div className="bp3-navbar-group bp3-align-left">
                  <div className="bp3-navbar-heading">Cambot Control Panel</div>
              </div>
          </Navbar>
          <section>
              <div className="row">
                  <div className="col-xs-12 col-md-7">
                      <div className="row">
                          <div className="col-xs-12">
                              <CameraCard />
                          </div>
                          <div className="col-xs-12">
                              <MotionControlCard />
                          </div>
                      </div>
                  </div>
                  <div className="col-xs-12 col-md-5">
                      <StatusCard />
                      <OdometryCard />
                      <MotorOutputCard />
                  </div>
              </div>
          </section>
      </RobotContextProvider>
  );
}

export default MainPage;
