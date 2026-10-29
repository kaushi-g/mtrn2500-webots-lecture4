#include "EPuckGreenPlusRobot.hpp"

EPuckGreenPlusRobot::EPuckGreenPlusRobot()
  :mGPS{getGPS("gps")},
   mDisplay1{getDisplay("display1")},
   mDP1Width{mDisplay1->getWidth()},
   mDP1Height{mDisplay1->getHeight()}
   {
   mGPS->enable(TIME_STEP);
   
}

void EPuckGreenPlusRobot::run() {
  while(step(TIME_STEP) != -1){
    telecontrol();
    readGPS();
    displayTrajectory();
    
  }
}
    
void EPuckGreenPlusRobot::readGPS() {
  const double* gpsValues = mGPS->getValues();
  mPosition = {gpsValues[0], gpsValues[1]};
  //std::cout << "Position: X: " << mPosition.first <<
            //"Position: Y: " << mPosition.second << std::endl;
};

void EPuckGreenPlusRobot::displayTrajectory () const{
  mDisplay1->drawPixel((WORLD_WIDTH/2 + mPosition.first) / WORLD_WIDTH * mDP1Width,
  (WORLD_HEIGHT/2 - mPosition.second) / WORLD_HEIGHT * mDP1Height);
}