#include <mobile_robot_perception/laser_scan_processor.h>
#include <mobile_robot_perception/scan_sector_measurement.h>

namespace mobile_robot_perception {

  LaserScanProcessor::LaserScanProcessor(const sensor_msgs::LaserScan& _scan) : scan_ (_scan) {
    std::cout << "Laser processor" << std::endl; 
  }

  LaserScanProcessor::~LaserScanProcessor() {}

  // ScanSectorMeasurements LaserScanProcessor::getSector(int _sector_origin, int _sector_size) {

  // }
}