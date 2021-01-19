#ifndef MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_
#define MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

namespace mobile_robot_perception { 

class LaserScanProcessor {
 public:
  LaserScanProcessor(const sensor_msgs::LaserScan& _scan);

  ~LaserScanProcessor();

  // ScanSectorMeasurements getSector(int _sector_origin, int _sector_size);

 private:
  sensor_msgs::LaserScan scan_;
};

}; // namespace mobile_robot_perception

#endif  // MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_