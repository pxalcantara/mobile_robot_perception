#include <gtest/gtest.h>

#include <mobile_robot_perception/laser_scan_processor.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>
// #include <boost>

using mobile_robot_perception::LaserScanProcessor;

struct LaserScanProcessorTest : public ::testing::Test {

  // mobile_robot_perception::LaserScanProcessor scan_processor;
  sensor_msgs::LaserScan scan_; 
  std::unique_ptr<LaserScanProcessor> scan_processor_ptr_;
  // LaserScanProcessorTest() : scan(), scan_processor(scan) {
  LaserScanProcessorTest() {

    scan_.angle_min = -1.57;
    scan_.angle_max = 1.57;
    scan_.angle_increment = 0.0314;
    scan_.ranges[100] = {1};

    scan_processor_ptr_ = std::unique_ptr<LaserScanProcessor> ( new LaserScanProcessor(scan_));
  }

  void SetUp() override {
  }

  void TearDown() override {}

};

TEST_F(LaserScanProcessorTest, angleToIndexTest) {
  int index = scan_processor_ptr_->angleToIndex(0);
  std::cout << "================== Index: " << scan_.angle_max << std::endl;
  EXPECT_TRUE(true);
  // EXPECT_EQ(0, scanclass.getInclination());
  // EXPECT_EQ(0, scanclass.getInclinationDegree());
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  ros::Time::init();
  return RUN_ALL_TESTS();
}
