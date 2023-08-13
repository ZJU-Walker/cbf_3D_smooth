#include <iostream>
#include <Eigen/Dense>
#include <cmath>

// 3D点结构体
struct Point3D {
    double x, y, z;

    Point3D() : x(0.0), y(0.0), z(0.0) {}
    Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

// 3D姿态结构体（欧拉角表示）
struct Pose3D {
    double yaw, pitch, roll;

    Pose3D() : yaw(0.0), pitch(0.0), roll(0.0) {}
    Pose3D(double _yaw, double _pitch, double _roll) : yaw(_yaw), pitch(_pitch), roll(_roll) {}
};

// 将thigh坐标系中的点T转换到world坐标系中
// base_origin - base坐标系原点在world坐标系中的坐标
// base_pose - base坐标系的位姿
// thigh_origin - thigh坐标系原点在base坐标系中的坐标
// thigh_pose - thigh坐标系的位姿
// point_in_thigh - thigh坐标系中的点T的坐标
// point_in_world - T在world坐标系中的坐标
// pose_in_world - thigh在world坐标系中的位姿
void thighToWorld(const Point3D& base_origin, const Pose3D& base_pose,
                  const Point3D& thigh_origin, const Pose3D& thigh_pose,
                  const Point3D& point_in_thigh, Point3D& point_in_world, Pose3D& pose_in_world) {
    Eigen::Matrix3d R_thigh_to_base;
    R_thigh_to_base = Eigen::AngleAxisd(thigh_pose.roll, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(thigh_pose.pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(thigh_pose.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d point_thigh(point_in_thigh.x, point_in_thigh.y, point_in_thigh.z);
    Eigen::Vector3d point_base = R_thigh_to_base * point_thigh + Eigen::Vector3d(thigh_origin.x, thigh_origin.y, thigh_origin.z);

    Eigen::Matrix3d R_base_to_world;
    R_base_to_world = Eigen::AngleAxisd(base_pose.roll, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(base_pose.pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(base_pose.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d point_world = R_base_to_world * point_base + Eigen::Vector3d(base_origin.x, base_origin.y, base_origin.z);

    Eigen::Matrix3d R_thigh_to_world = R_base_to_world * R_thigh_to_base;
    Eigen::Vector3d euler_angles = R_thigh_to_world.eulerAngles(2, 1, 0); // ZYX顺序

    point_in_world.x = point_world.x();
    point_in_world.y = point_world.y();
    point_in_world.z = point_world.z();

    pose_in_world.yaw = euler_angles[0];
    pose_in_world.pitch = euler_angles[1];
    pose_in_world.roll = euler_angles[2];
}

// 将calf坐标系中的点C转换到world坐标系中
void calfToWorld(const Point3D& base_origin, const Pose3D& base_pose,
                 const Point3D& thigh_origin, const Pose3D& thigh_pose,
                 const Point3D& calf_origin, const Pose3D& calf_pose,
                 const Point3D& point_in_calf, Point3D& point_in_world, Pose3D& pose_in_world) {
    Eigen::Matrix3d R_thigh_to_base;
    R_thigh_to_base = Eigen::AngleAxisd(thigh_pose.roll, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(thigh_pose.pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(thigh_pose.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d R_calf_to_thigh;
    R_calf_to_thigh = Eigen::AngleAxisd(calf_pose.roll, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(calf_pose.pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(calf_pose.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d point_calf(point_in_calf.x, point_in_calf.y, point_in_calf.z);
    Eigen::Vector3d point_thigh = R_calf_to_thigh * point_calf + Eigen::Vector3d(calf_origin.x, calf_origin.y, calf_origin.z);

    Eigen::Vector3d point_base = R_thigh_to_base * point_thigh + Eigen::Vector3d(thigh_origin.x, thigh_origin.y, thigh_origin.z);

    Eigen::Matrix3d R_base_to_world;
    R_base_to_world = Eigen::AngleAxisd(base_pose.roll, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(base_pose.pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(base_pose.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d point_world = R_base_to_world * point_base + Eigen::Vector3d(base_origin.x, base_origin.y, base_origin.z);

    Eigen::Matrix3d R_calf_to_world = R_base_to_world * R_thigh_to_base * R_calf_to_thigh;
    Eigen::Vector3d euler_angles = R_calf_to_world.eulerAngles(2, 1, 0); // ZYX顺序

    point_in_world.x = point_world.x();
    point_in_world.y = point_world.y();
    point_in_world.z = point_world.z();

    pose_in_world.yaw = euler_angles[0];
    pose_in_world.pitch = euler_angles[1];
    pose_in_world.roll = euler_angles[2];
}


int main() {
    // 已知base坐标系原点在world坐标系中的坐标和位姿
    Point3D base_origin(0,-1.0,0); // base坐标系原点在world坐标系中的坐标(x,y,z)
    Pose3D base_pose(0,0,M_PI_2);    // base坐标系的位姿(yaw, pitch, roll)

    // 已知thigh坐标系原点在base坐标系中的坐标和位姿
    Point3D thigh_origin(0,0,0); // thigh坐标系原点在base坐标系中的坐标(x,y,z)
    Pose3D thigh_pose(0,0,M_PI_2);    // thigh坐标系的位姿(yaw, pitch, roll)

    // 已知calf坐标系原点在thigh坐标系中的坐标和位姿
    Point3D calf_origin(0, 0, 1.0); // calf坐标系原点在thigh坐标系中的坐标(x, y, z)
    Pose3D calf_pose(0, 0, M_PI_4); // calf坐标系的位姿(yaw, pitch, roll)

    // 已知点T在thigh坐标系中的坐标
    Point3D point_in_thigh(0, 0, 1.0); // thigh坐标系中的点T的坐标(xt, yt, zt)

    // 已知点C在calf坐标系中的坐标
    Point3D point_in_calf(0.5, 0.5, 0.5); // calf坐标系中的点C的坐标(xc, yc, zc)

    // 存储T在world坐标系中的坐标和位姿
    Point3D point_in_world_thigh;
    Pose3D pose_in_world_thigh;

    // 存储C在world坐标系中的坐标和位姿
    Point3D point_in_world_calf;
    Pose3D pose_in_world_calf;

    // 将点T从thigh坐标系转换到world坐标系，并求出位姿
    thighToWorld(base_origin, base_pose, thigh_origin, thigh_pose, point_in_thigh, point_in_world_thigh, pose_in_world_thigh);

    // 将点C从calf坐标系转换到world坐标系，并求出位姿
    calfToWorld(base_origin, base_pose, thigh_origin, thigh_pose, calf_origin, calf_pose, point_in_calf, point_in_world_calf, pose_in_world_calf);

    // 输出点T在world坐标系中的坐标和位姿
    std::cout << "Point T in world coordinates: (" << point_in_world_thigh.x << ", " << point_in_world_thigh.y << ", " << point_in_world_thigh.z << ")" << std::endl;
    std::cout << "Pose of thigh in world: (yaw: " << pose_in_world_thigh.yaw << ", pitch: " << pose_in_world_thigh.pitch << ", roll: " << pose_in_world_thigh.roll << ")" << std::endl;

    // 输出点C在world坐标系中的坐标和位姿
    std::cout << "Point C in world coordinates: (" << point_in_world_calf.x << ", " << point_in_world_calf.y << ", " << point_in_world_calf.z << ")" << std::endl;
    std::cout << "Pose of calf in world: (yaw: " << pose_in_world_calf.yaw << ", pitch: " << pose_in_world_calf.pitch << ", roll: " << pose_in_world_calf.roll << ")" << std::endl;


    return 0;
}
