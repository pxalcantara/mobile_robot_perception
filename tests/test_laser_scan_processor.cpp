#include <gtest/gtest.h>

#include <mobile_robot_perception/laser_scan_processor.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>
#include <vector>

#define PI 3.14159

using mobile_robot_perception::LaserScanProcessor;

struct LaserScanProcessorTest : public ::testing::Test {

  sensor_msgs::LaserScan scan_; 
  std::unique_ptr<LaserScanProcessor> scan_processor_ptr_;

  LaserScanProcessorTest() {

    scan_.angle_min = -1.57;
    scan_.angle_max = 1.57;
    scan_.angle_increment = 0.0314;
    scan_.ranges.assign(100, 1);  

    scan_processor_ptr_ = std::unique_ptr<LaserScanProcessor> ( new LaserScanProcessor(scan_));
    // std::cout << "================ SCAN ===============" << std::endl;
  }

  void SetUp() override {
  }

  void TearDown() override {}

};

TEST_F(LaserScanProcessorTest, angleToIndexTest) {
  EXPECT_EQ(50, scan_processor_ptr_->angleToIndex(0));
  EXPECT_EQ(0, scan_processor_ptr_->angleToIndex(-PI/2));
  EXPECT_EQ(25, scan_processor_ptr_->angleToIndex(-PI/4));
  EXPECT_EQ(16, scan_processor_ptr_->angleToIndex(-PI/3));
  EXPECT_EQ(66, scan_processor_ptr_->angleToIndex(PI/6));
  EXPECT_EQ(100, scan_processor_ptr_->angleToIndex(PI/2));

  EXPECT_EQ(-1, scan_processor_ptr_->angleToIndex(-PI));
  EXPECT_EQ(-1, scan_processor_ptr_->angleToIndex(1.6));
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  ros::Time::init();
  return RUN_ALL_TESTS();
}
