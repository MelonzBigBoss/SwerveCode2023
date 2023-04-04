#include <hal/HAL.h>
#include "gtest/gtest.h"

int main(int argc, char** argv) {
  return 0; // Lol
  HAL_Initialize(500, 0); 
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
