#include "mobile_robot_perception/laser_scan_processor.h"
#include "mobile_robot_perception/scan_sector_measurement.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // sensor_msgs::LaserScan::Ptr _msg = &msg;
  mobile_robot_perception::LaserScanProcessor scan_processor(*msg);

  mobile_robot_perception::ScanSectorMeasurements front_sector = scan_processor.getSector(0, 0.52);

  // ROS_INFO_STREAM_THROTTLE(5, "Front min: " << front_sector.getMin());

}

int main (int argc, char** argv) {

  ros::init(argc, argv, "scan_processor_node");
  ros::NodeHandle nh("");

  ros::Subscriber scan_sub = nh.subscribe("/quimera_robot/scan", 10, scanCB);

  return 0;
}