#include <mobile_robot_perception/scan_sector_measurement.h>

namespace mobile_robot_perception {

ScanSectorMeasurements::ScanSectorMeasurements(const std::vector<float>& _scan_measurements, float _angle_increment) :
  scan_measurements_(_scan_measurements),
  angle_increment_(_angle_increment) {
}
ScanSectorMeasurements::~ScanSectorMeasurements() {}

float ScanSectorMeasurements::getInclination() {
  float linear_distance = (scan_measurements_.size() - 1) * angle_increment_;
 
  float angular_inclination = std::atan2((scan_measurements_.back() - scan_measurements_.front()), linear_distance);

  return angular_inclination;
}

float ScanSectorMeasurements::getInclinationDegree() {
  return getInclination() * 180 / M_PI;
}

float ScanSectorMeasurements::getMax() {
  max_measurement_ = *max_element(scan_measurements_.begin(), scan_measurements_.end());
  return max_measurement_;
}

float ScanSectorMeasurements::getMin() {
  min_measurement_ = *min_element(scan_measurements_.begin(), scan_measurements_.end());
  return min_measurement_;
}

void ScanSectorMeasurements::setScanMeasurements(const std::vector<float>& _scan_measurements) {
  scan_measurements_ = _scan_measurements;
}

}; // namespace mobile_robot_perception