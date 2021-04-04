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

TEST_F(LaserScanProcessorTest, getSectorTest) {
  
  // 0.79 radian = ~45 degrees
  mobile_robot_perception::ScanSectorMeasurements scan_sector = scan_processor_ptr_->getSector(0, 0.79);

  EXPECT_EQ(0, scan_sector.getInclination());
  EXPECT_EQ(1, scan_sector.getMax());
  EXPECT_EQ(25, scan_sector.getSize());

  int ranges_pos = 42;
  float scan = 1;
  for (int i=0; i <= 15; i++) {

    scan_.ranges[ranges_pos] = scan;
    ranges_pos += 1;
    scan += 0.0314;
  }

  scan_processor_ptr_ = std::unique_ptr<LaserScanProcessor> ( new LaserScanProcessor(scan_));   

  scan_sector = scan_processor_ptr_->getSector(0, 0.5);
  
  EXPECT_NEAR(0.785398, scan_sector.getInclination(), 0.0001);
  EXPECT_NEAR(1.4396, scan_sector.getMax(), 0.0001);
  EXPECT_EQ(1, scan_sector.getMin());
  EXPECT_EQ(15, scan_sector.getSize());
    
}

TEST_F(LaserScanProcessorTest, getSectorLimitsTest) {

  mobile_robot_perception::ScanSectorMeasurements scan_sector = scan_processor_ptr_->getSector(-PI/2, 0.79);

  EXPECT_EQ(12, scan_sector.getSize());

  scan_sector = scan_processor_ptr_->getSector(-PI/2 + 2*0.0314, 0.79);

  EXPECT_EQ(14, scan_sector.getSize());

  scan_sector = scan_processor_ptr_->getSector(PI/2 , 0.5);

  EXPECT_EQ(8, scan_sector.getSize());
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  ros::Time::init();
  return RUN_ALL_TESTS();
}
