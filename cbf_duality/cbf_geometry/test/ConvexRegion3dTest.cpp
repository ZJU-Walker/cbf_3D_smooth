#include <gtest/gtest.h>
#include "cbf_geometry/DistanceQp3d.h"
#include <math.h>

using namespace cbf;

// TEST(DistanceQp3D, Rectangle3d_qp) {
//   vector_t pose0(6), pose1(6); // (x, y, z, yaw, pitch, roll)
//   pose0 << 2.63841, 1.81153, 0.323023, 1.25743, 0.0192528, -0.00679648;
//   pose1 << 2.0, 2.0, 0.25, 0, 0, 0;
  
//   vector_t dimensions0(3), dimensions1(3); // dimensions of the cuboids (length, width, height)
//   dimensions0 << 0.27, 0.18, 0.08;
//   dimensions1 << 0.5, 0.5, 0.5;

//   Rectangle3d<scalar_t> region0(pose0, dimensions0), region1(pose1, dimensions1);
//   DistanceQp3d qp(region0, region1);
//   Duality3d qp_dual(region0, region1);

//   EXPECT_NEAR(qp.getDistance(), 0.143868, 1e-10);
//   // EXPECT_NEAR(qp.getDistance(), qp_dual.getDistance(), 1e-10);
// }

TEST(DistanceQp3D, A_b_test) {
  vector_t pose0(6), pose1(6); // (x, y, z, yaw, pitch, roll)
  pose0 << 0, 0, 0.04, 0, 0, 0;
  pose1 << 0, 3, 0.5, 0, 0, 0;
  
  vector_t dimensions0(3), dimensions1(3); // dimensions of the cuboids (length, width, height)
  dimensions0 << 0.27, 0.18, 0.08;
  dimensions1 << 1, 1, 1;

  Rectangle3d<scalar_t> region0(pose0, dimensions0), region1(pose1, dimensions1);
  DistanceQp3d qp(region0, region1);
  Duality3d qp_dual(region0, region1);
  std::cout << "distance: " << qp_dual.getDistance() << std::endl;

  EXPECT_NEAR(qp.getDistance(), 2.41, 1e-10);
  // EXPECT_NEAR(qp.getDistance(), qp_dual.getDistance(), 1e-10);
}

// TEST(DistanceQp3D, Rectangle3d_dual) {
//   vector_t pose0(6), pose1(6); // (x, y, z, yaw, pitch, roll)
//   pose0 << 2.63841, 1.81153, 0.323023, 1.25743, 0.0192528, -0.00679648;
//   pose1 << 2.0, 2.0, 0.25, 0, 0, 0;
  
//   vector_t dimensions0(3), dimensions1(3); // dimensions of the cuboids (length, width, height)
//   dimensions0 << 0.27, 0.18, 0.08;
//   dimensions1 << 0.5, 0.5, 0.5;

//   Rectangle3d<scalar_t> region0(pose0, dimensions0), region1(pose1, dimensions1);
//   DistanceQp3d qp(region0, region1);
//   Duality3d qp_dual(region0, region1);

//   // EXPECT_NEAR(qp.getDistance(), 2-sqrt(2), 1e-10);
//   EXPECT_NEAR(qp_dual.getDistance(), qp.getDistance(), 1e-10);
//   // EXPECT_NEAR(qp.getDistance(), 2-sqrt(3)/2-0.5, 1e-10);
// }

// main function
int main(int argc, char **argv) {
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
