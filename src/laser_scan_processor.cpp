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

    for (int i = 0; i < _sector_size; i++) {
      sector_measurements.push_back(scan_.ranges[ _sector_origin_angle + i]);
    }

    ScanSectorMeasurements scan_sector(sector_measurements, scan_.angle_increment);
    return scan_sector;
  }

  int LaserScanProcessor::angleToIndex(float _angle) {
    int index = (_angle + (angle_range_ / 2)) / scan_.angle_increment;
    return index;
  }
} 