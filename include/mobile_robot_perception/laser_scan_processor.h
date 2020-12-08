
#ifndef MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_
#define MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <string>
#include <iostream>

namespace mobile_robot_perception { 

class ScanSectorMeasurements {
 public:
  ScanSectorMeasurements ();

  ~ScanSectorMeasurements ();

 private:
  std::string scan_measurements_;

};


}; // namespace mobile_robot_perception

#endif  // MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_