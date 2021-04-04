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
   * @param _sector_origin The origin angle, in radian, of the sector. The origin is the center of the sector
   * @param _sector_size The width, in radian, of the sector. Considering the _sector_origin as reference, the 
   * sector has +(_sector_size / 2) and -(_sector_size / 2) considering the Righ Hand reference.
   * 
   * @return A ScanSectorMeasurement object.
   */
  ScanSectorMeasurements getSector(float _sector_origin, float _sector_size);

  /**
   * @brief Convert an angle, in radian, into an index of the measurements array.
   * 
   * To make this convertion, the angle_increment of the LaserScan msg is used. If the calculation doesn't give an 
   * integer value, the function will return the previews integer. The _angle is truncated to 2 decimals
   *
   * @param _angle Angle to be converted into the array index.
   * 
   * @return The array index.
   * @retval -1 if the angle is out of the angle range
   */
  int angleToIndex(float _angle);

 private:
  sensor_msgs::LaserScan scan_;  /// scan msg object to be processed.
  float angle_range_;  /// angle range of the scan msg.
};

}; // namespace mobile_robot_perception

#endif  // MOBILE_ROBOT_PERCEPTION_LASER_SCAN_PROCESSOR_H_