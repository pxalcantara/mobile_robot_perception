#include <gtest/gtest.h>

#include <mobile_robot_perception/laser_scan_processor.h>

struct ScanSectorMeasurementTest : public ::testing::Test {
  
  int angle_increment;
  
  std::vector<float> scan_measurements;

  mobile_robot_perception::ScanSectorMeasurements scanclass;

  ScanSectorMeasurementTest() :
  angle_increment(1),
  scan_measurements(10), 
  scanclass(scan_measurements, angle_increment) {
  }

  void SetUp() override {
    scan_measurements = {1,2,3,4,5,6,7,8,9,10};
    scanclass.setScanMeasurements(scan_measurements);
  }

  void TearDown() override {}




};

TEST_F(ScanSectorMeasurementTest, getInclinationTest) {
  float inclination = scanclass.getInclination(); 
  EXPECT_EQ(0, inclination);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  ros::Time::init();
  return RUN_ALL_TESTS();
}
