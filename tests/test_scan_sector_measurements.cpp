#include <gtest/gtest.h>

#include <mobile_robot_perception/laser_scan_processor.h>

struct ScanSectorMeasurementTest : public::testing::Test {
  ScanSectorMeasurementTest() {}

  void SetUp() override {}

  void TearDown() override {}

  
  mobile_robot_perception::ScanSectorMeasurements scanclass;
};

TEST_F(ScanSectorMeasurementTest, test) {
  ASSERT_TRUE(true);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  ros::Time::init();
  return RUN_ALL_TESTS();
}
