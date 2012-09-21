#include "digital.hpp"
#include <gtest/gtest.h>


TEST(ScaleTest, 1024_scaleto_100) {
   Digital d(0, 1024, 0, 100);
  EXPECT_EQ(0, d.eu(0));
  EXPECT_EQ(50, d.eu(512));
  EXPECT_EQ(100, d.eu(1024));
}

TEST(ScaleTest, 1024_scaleto_24000) {
   Digital d(0, 1024, 0, 24000);
  EXPECT_EQ(0, d.eu(0));
  EXPECT_EQ(12000, d.eu(512));
  EXPECT_EQ(24000, d.eu(1024));
}

TEST(ScaleTest, 20_to_1024_scaleto_24000) {
   Digital d(20, 1024, 0, 24000);
  EXPECT_EQ(0, d.eu(20));
  EXPECT_EQ(12000, d.eu(522));
  EXPECT_EQ(24000, d.eu(1024));
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
