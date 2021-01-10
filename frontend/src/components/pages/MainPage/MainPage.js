import {Card, Navbar} from '@blueprintjs/core';
import CameraCard from '../../features/CameraCard/CameraCard';
import MotionControlCard from '../../features/MotionControlCard/MotionControlCard';
import MotorOutputCard from '../../features/MotorOutputCard/MotorOutputCard';
import OdometryCard from '../../features/OdometryCard/OdometryCard';
import StatusCard from '../../features/StatusCard/StatusCard';
import {RobotContextProvider} from '../../context/RobotContext/RobotContext';

function MainPage() {
  return (
      <RobotContextProvider>
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
