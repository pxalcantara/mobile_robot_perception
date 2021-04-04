
#ifndef MOBILE_ROBOT_PERCEPTION_SCAN_SECTOR_MEASUREMENT_H_
#define MOBILE_ROBOT_PERCEPTION_SCAN_SECTOR_MEASUREMENT_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

namespace mobile_robot_perception {
/**
 * @class ScanSectorMeasurementes scan_sector_measurement.h
 *
 * @brief Class to extend some functionalyties of a vector of measurements in a Laser Scan msg.
 */
class ScanSectorMeasurements {
 public:
  /**
   * @brief Constructor of the class Scan Sector Measurements.
   *
   * @param _scan_measurements Vector with the measurements of the Lase Scan msgs. Part of the sensor_msgs::LaserScan msg.
   * @param _angle_increment Angle diference between two consecutive scan measuraments. Part of the sensor_msgs::LaserScan msg.
   */
  ScanSectorMeasurements(const std::vector<float>& _scan_measurements, float _angle_increment);

  ~ScanSectorMeasurements();

  /**
   * @brief Get the inclination of the surface scanned by the sensor.
   *
   * Considering the scan sector as a line, this method returns the inclination of this line.
   * 
   * @return The inclination of the surface in radians
   */
  float getInclination();

  /**
   * @brief Get the inclination of the surface scanned by the sensor.
   *
   * Considering the scan sector as a line, this method returns the inclination of this line.
   * 
   * @return The inclination of the surface in degrees.
   */
  float getInclinationDegree();

  /**
   * @brief Get the maximun measurment of the scanned sector.
   *
   * 
   * @return The maximun measurement value of the sector in meters.
   */
  float getMax();

  /**
   * @brief Get the minimun measurment of the scanned sector.
   *
   * 
   * @return The minimun measurement value of the sector in meters.
   */
  float getMin();

  /**
   * @brief Get the size of the object.
   *
   * The size of the object is the number of elements of the scan_measurement_ atribute which if the number of beams of
   * the scan sector.
   * 
   * @return The size of the scan_measurement_ attribute.
   */
  unsigned int getSize();

  /**
   * @brief Set the scan_measurement atribute.
   *
   */
  void setScanMeasurements(const std::vector<float>& _scan_measurements);

 private:
  
  std::vector<float> scan_measurements_;  ///  vector with the measurements of the scan msg.
  float angle_increment_;  ///  angular increment from one scan beam to the next beam in the scan msg.
  float max_measurement_;  ///  maximun value of distance of the scan measuraments;
  float min_measurement_;  ///  minimun value of distance of the scan measuraments;
  
};

}; // namespace mobile_robot_perception

#endif  // MOBILE_ROBOT_PERCEPTION_SCAN_SECTOR_MEASUREMENT_H_