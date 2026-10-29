#include "EPuckGreenPlusRobot.hpp"
#include <iostream>
#include <cmath>

EPuckGreenPlusRobot::EPuckGreenPlusRobot()
  :mGPS{getGPS("gps")},
   mDisplay1{getDisplay("display1")},
   mCompass {getCompass("compass")},
   mLidar{getLidar("lidar")},
   mDP1Width{mDisplay1->getWidth()},
   mDP1Height{mDisplay1->getHeight()},
   
   mDisplay2{getDisplay("display2")},
   mDP2Width{mDisplay2->getWidth()},
   mDP2Height{mDisplay2->getHeight()}
   {
   mGPS->enable(TIME_STEP);
   mCompass->enable(TIME_STEP);
   mLidar->enable(TIME_STEP);
   mLidar->enablePointCloud();
}

void EPuckGreenPlusRobot::run() {
  while(step(TIME_STEP) != -1){
    telecontrol();
    readGPS();
    displayTrajectory();
    readCompass();
    readLidar();
    displayMap();
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

void EPuckGreenPlusRobot::readCompass(){
  const double *compassValues{mCompass->getValues()};
  mHeading = atan2(compassValues[1], compassValues[0]);
  //std::cout << "Heading: " << mHeading / 3.14159265 * 180 << std::endl;
  
}

void EPuckGreenPlusRobot::readLidar(){
  const webots::LidarPoint* pointCloud{mLidar->getLayerPointCloud(0)};
  mPointCloud.clear();
  for(int i{}; i < mLidar->getNumberOfPoints(); i++){
    mPointCloud.push_back({pointCloud[i].x,pointCloud[i].y});
    
  }
}
void EPuckGreenPlusRobot::displayMap() const{
  for(const auto& elem : mPointCloud){
    const double xLocal{elem.first};
    const double yLocal{elem.second};
    
    const double xWorld {mPosition.first + xLocal * sin(mHeading) -yLocal * cos(mHeading)};
    const double yWorld {mPosition.second + xLocal * cos(mHeading) +yLocal * sin(mHeading)};
    
    mDisplay2->drawPixel((WORLD_WIDTH/2 + xWorld) / WORLD_WIDTH * mDP2Width,
                         (WORLD_HEIGHT/2 - yWorld) / WORLD_HEIGHT * mDP2Height);
  }
}
  