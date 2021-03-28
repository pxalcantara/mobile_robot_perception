#ifndef MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_
#define MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_

#include <ros/ros.h>

#include <mobile_robot_perception/scan_sector_measurement.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

namespace mobile_robot_perception { 

/**
 * @class LaserScanProcessor laser_scan_processor.h
 *
 * @brief Class to extract some features of a LaserScan ROS msg
 */
class LaserScanProcessor {
 public:
   /**
   * @brief Constructor of the class Laser Scan Processor.
   *
   * @param _scan ROS message of type sensor_msgs::LaserScan.
   */
  LaserScanProcessor(const sensor_msgs::LaserScan& _scan);

  ~LaserScanProcessor();
  /**
   * @brief Get a sector of the LaserScan msgs.
   *
   * @param _sector_origin The origin angle, in degrees, of the sector. The origin is the center of the sector
   * @param _sector_size The width, in degrees, of the sector. Considering the _sector_origin as reference, the 
   * sector has +(_sector_size / 2) and -(_sector_size / 2) considering the Righ Hand reference.
   * 
   * @return A ScanSectorMeasurement object.
   */
  ScanSectorMeasurements getSector(float _sector_origin, float _sector_size);

  /**
   * @brief Convert an angle, in degrees, into an index of the measurements array.
   * 
   * To make this convertion, the angle_increment of the LaserScan msg is used. If the calculation doesn't give an 
   * integer value, the function will return the next bigger integer.
   *
   * @param _angle Angle to be converted into the array index.
   * 
   * @return The array index.
   */
  int angleToIndex(float _angle);

 private:
  sensor_msgs::LaserScan scan_;
  float angle_range_;
};

}; // namespace mobile_robot_perception

#endif  // MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_