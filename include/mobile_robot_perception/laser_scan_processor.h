
#ifndef MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_
#define MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

namespace mobile_robot_perception { 

class ScanSectorMeasurements {
 public:
  ScanSectorMeasurements(const std::vector<float>& _scan_measurements, float _angle_increment);

  ~ScanSectorMeasurements();

  float getInclination();

  float getInclinationDegree();

  float getMax();

  float getMin();

  void setScanMeasurements(const std::vector<float>& _scan_measurements);

 private:
  std::vector<float> scan_measurements_;
  float angle_increment_;
  float max_measurement_;
  float min_measurement_;

};



}; // namespace mobile_robot_perception

#endif  // MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_