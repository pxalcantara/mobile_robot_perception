#include <mobile_robot_perception/laser_scan_processor.h>

namespace mobile_robot_perception {

ScanSectorMeasurements::ScanSectorMeasurements (const std::vector<float>& _scan_measurements, float _angle_increment) :
  scan_measurements_(_scan_measurements),
  angle_increment_(_angle_increment) {
}
ScanSectorMeasurements::~ScanSectorMeasurements () {}

float ScanSectorMeasurements::getInclination() {
  float linear_distance = (scan_measurements_.size() - 1) * angle_increment_;
 
  float angular = std::atan2((scan_measurements_.back() - scan_measurements_.front()), linear_distance);

  return angular;
}

float ScanSectorMeasurements::getInclinationDegree() {
  return getInclination() * 180 / M_PI;
}

void ScanSectorMeasurements::setScanMeasurements(const std::vector<float>& _scan_measurements) {
  scan_measurements_ = _scan_measurements;
}

}; // namespace mobile_robot_perception