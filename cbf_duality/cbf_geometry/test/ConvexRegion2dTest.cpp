//
// Created by qiayuan on 2022/6/29.
//
#include <gtest/gtest.h>
#include <iostream>
#include "cbf_geometry/DistanceQp2d.h"

using namespace cbf;

TEST(DistanceQp2D, 3D_project) {
  vector_t pose0(3), pose1(3);
  vector_t size0(2), size1(2);
  pose0 << 0, 0, 0;
  pose1 << 2.0, 2.0, 0;
  size0 << 0.27, 0.18;
  size1 << 0.5, 0.5;
  Rectangle2d<scalar_t> region0(pose0, size0), region1(pose1, size1);
  DistanceQp2d qp(region0, region1);
  Duality2d qp_dual(region0, region1);
  std::cout << "dis project: " << qp.getDistance() << std::endl;

  EXPECT_NEAR(qp.getDistance(), 2.1668, 1e-10);
  EXPECT_NEAR(qp.getDistance(), qp_dual.getDistance(), 1e-10);
}

// TEST(DistanceQp2D, twoTriangle) {
//   size_t num_points = 3;
//   vector_t points0(num_points * 2), points1(num_points * 2);
//   points0 << 0, 0, 1, -1, 1, 1;
//   points1 << 2, 0, 3, -1, 3, 1;
//   Vertex2d<scalar_t> region0(num_points, points0), region1(num_points + 6, points1);
//   DistanceQp2d qp(region0, region1);
//   Duality2d qp_dual(region0, region1);

//   EXPECT_NEAR(qp.getDistance(), 1., 1e-10);
//   EXPECT_NEAR(qp.getDistance(), qp_dual.getDistance(), 1e-10);
// }

// TEST(ConvexRegion2D, sameRegion) {
//   vector_t pose(3), size(2);
//   pose << 1, 0, 0;
//   size << 1.0, 1.0;
//   Rectangle2d<scalar_t> rectangle(pose, size);

//   vector_t points(4 * 2);
//   points << 0.5, 0.5, 0.5, -0.5, 1.5, -0.5, 1.5, 0.5;
//   Vertex2d<scalar_t> vertex(4, points);

//   // TODO random points
//   vector_t test_point(2);
//   test_point << 1, 0;
//   vector_t rect_h = rectangle.getA() * test_point - rectangle.getB();
//   vector_t vertex_h = vertex.getA() * test_point - vertex.getB();
//   size_t rect_i = 0, vertex_i = 0;
//   for (size_t i = 0; i < rect_h.size(); ++i) {
//     if (rect_h(i) > 0) rect_i++;
//     if (vertex_h(i) > 0) rect_i++;
//   }
//   EXPECT_EQ(rect_i, vertex_i);
// }

int main(int argc, char **argv) {
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}