# TEST Usage

File `ConvexRegion2dTest.cpp` is for testing `ConvexRegion2d.cpp`.

The usage is as below:

1. Add TEST case in `ConvexRegion2dTest.cpp` using `gtest`

2. Add `main` function as below:

   ```cpp
   int main(int argc, char **argv) {
     printf("Running main() from %s\n", __FILE__);
     testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
   }
   ```

3. Run the follow command in the terminal:

   catkin build cbf_geometry --verbose --catkin-make-args run_tests

   you can change `cbf_geometry` to another package.

4. Switch to workspace and run the command as below:

   - `roscore`

   - `rosrun cbf_geometry cbf_geometry_test`

