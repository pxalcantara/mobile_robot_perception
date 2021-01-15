#include <mobile_robot_perception/laser_scan_processor.h>

namespace mobile_robot_perception {

ScanSectorMeasurements::ScanSectorMeasurements (const std::vector<float>& _scan_measurements, float _angle_increment) :
  scan_measurements_(_scan_measurements),
  angle_increment_(_angle_increment) {
  std::cout << "Scan Sector Class" << std::endl;
}
ScanSectorMeasurements::~ScanSectorMeasurements () {}

float ScanSectorMeasurements::getInclination() {
  float first_scan = scan_measurements_[0];
  float last_scan = scan_measurements_.at(9);

  float linear_distance = (scan_measurements_.size() - 1) * angle_increment_;

  std::cout << "first scan: " << first_scan << std::endl;
  std::cout << "last scan: " << last_scan << std::endl;
  std::cout << "scan size: " << scan_measurements_.size() << std::endl;
  std::cout << "linear distance: " << linear_distance << std::endl;
  
  float angular = atan2((last_scan - first_scan), linear_distance);

  return angular;

}

void ScanSectorMeasurements::setScanMeasurements(const std::vector<float>& _scan_measurements) {
  scan_measurements_ = _scan_measurements;
}

}; // namespace mobile_robot_perception