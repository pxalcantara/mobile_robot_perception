#include <mobile_robot_perception/laser_scan_processor.h>
#include <mobile_robot_perception/scan_sector_measurement.h>

namespace mobile_robot_perception {

  LaserScanProcessor::LaserScanProcessor(const sensor_msgs::LaserScan& _scan) : scan_ (_scan) {
    std::cout << "Laser processor" << std::endl; 
    angle_range_ = scan_.angle_max - scan_.angle_min;
  }

  LaserScanProcessor::~LaserScanProcessor() {}

  ScanSectorMeasurements LaserScanProcessor::getSector(float _sector_origin_angle, float _sector_size) {
    std::vector<float> sector_measurements = {};

    int center_position = this->angleToIndex(_sector_origin_angle);

    float start_angle = _sector_origin_angle - (_sector_size/2);
    if (start_angle > scan_.angle_max) {
      start_angle = scan_.angle_max;
    }

    float final_angle = _sector_origin_angle + (_sector_size/2);
    if (final_angle < scan_.angle_min) {
      final_angle = scan_.angle_min;
    }

    int start_position = this->angleToIndex(start_angle);
    int final_position = this->angleToIndex(final_angle);

    std::cout << " center position: " << center_position << std::endl;
    std::cout << " start position: " << start_position << std::endl;
    std::cout << " final position: " << final_position << std::endl;

    for (int i = start_position; i < final_position; i++) {
      sector_measurements.push_back(scan_.ranges[i]);
    }

    ScanSectorMeasurements scan_sector(sector_measurements, scan_.angle_increment);
    return scan_sector;
  }

  int LaserScanProcessor::angleToIndex(float _angle) {
    // trunc the _angle value to get just 2 decimals;
    float angle = std::truncf(100 * _angle) / 100;
    
    if ((angle ) < scan_.angle_min || (angle ) > scan_.angle_max) {
      return -1;
    }

    int index = (angle + (angle_range_ / 2)) / scan_.angle_increment;
    return index;
  }
} 