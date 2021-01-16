#include <gtest/gtest.h>

#include <mobile_robot_perception/laser_scan_processor.h>

struct ScanSectorMeasurementTest : public ::testing::Test {
  
  int angle_increment;
  
  std::vector<float> scan_measurements;

  mobile_robot_perception::ScanSectorMeasurements scanclass;

  ScanSectorMeasurementTest() :
  angle_increment(1),
  scan_measurements(1), 
  scanclass(scan_measurements, angle_increment) {
  }

  void SetUp() override {
  }

  void TearDown() override {}




};

TEST_F(ScanSectorMeasurementTest, get0InclinationTest) {
  std::vector<float> scan_measurements(5,1);  
  scanclass.setScanMeasurements(scan_measurements); 
  EXPECT_EQ(0, scanclass.getInclination());
  EXPECT_EQ(0, scanclass.getInclinationDegree());
}

TEST_F(ScanSectorMeasurementTest, get45InclinationTest) {
  std::vector<float> scan_measurements = {1,2,3,4,5,6,7,8,9,10,11,12,13};  
  scanclass.setScanMeasurements(scan_measurements);
  EXPECT_FLOAT_EQ(0.785398, scanclass.getInclination());
  EXPECT_FLOAT_EQ(45, scanclass.getInclinationDegree());
}

TEST_F(ScanSectorMeasurementTest, getMinus45InclinationTest) {
  std::vector<float> scan_measurements = {11,10,9,8,7,6,5};  
  scanclass.setScanMeasurements(scan_measurements);
  EXPECT_FLOAT_EQ(-0.785398, scanclass.getInclination());
  EXPECT_FLOAT_EQ(-45, scanclass.getInclinationDegree());
}

TEST_F(ScanSectorMeasurementTest, getMinus30InclinationTest) {
  std::vector<float> scan_measurements;
  for (int i = 1; i < 15; i++) {
    scan_measurements.push_back(i * tan(-0.523599));
  }  
  scanclass.setScanMeasurements(scan_measurements);
  EXPECT_NEAR(-0.523599, scanclass.getInclination(), 0.001);
  EXPECT_NEAR(-30, scanclass.getInclinationDegree(), 0.001);
}

TEST_F(ScanSectorMeasurementTest, getMaxTest) {
  std::vector<float> scan_measurements1 = {11.6, 10.4, 9, 8, 7, 6, 5};  
  std::vector<float> scan_measurements2 = {11.6, 10.4, 9, 80, 77, 56, 6, 500.93};  
  scanclass.setScanMeasurements(scan_measurements1);
  EXPECT_FLOAT_EQ(11.6, scanclass.getMax());

  scanclass.setScanMeasurements(scan_measurements2);
  EXPECT_FLOAT_EQ(500.93, scanclass.getMax());
}

TEST_F(ScanSectorMeasurementTest, getMinTest) {
  std::vector<float> scan_measurements1 = {11.6, 10.4, 9, 8, 7, 6, 0.5};  
  std::vector<float> scan_measurements2 = {-11.6, -10.4, 9, 80, 77, -56.6, 6, 500.93};  
  scanclass.setScanMeasurements(scan_measurements1);
  EXPECT_FLOAT_EQ(0.5, scanclass.getMin());

  scanclass.setScanMeasurements(scan_measurements2);
  EXPECT_FLOAT_EQ(-56.6, scanclass.getMin());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  ros::Time::init();
  return RUN_ALL_TESTS();
}
