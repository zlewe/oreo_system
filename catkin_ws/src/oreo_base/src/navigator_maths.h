#ifndef OREO_BASE_NAVIGATOR_MATHS_H_
#define OREO_BASE_NAVIGATOR_MATHS_H_

#include <cmath>
#include "geometry_msgs/Pose2D.h"

namespace navigator_maths {
  inline double DegreeToRadian(double degree) {
    return degree / 180.0 * M_PI;
  }

  inline double RadianToDegree(double radian) {
    return radian / M_PI * 180.0;
  }

  inline void RotatePose(geometry_msgs::Pose2D* pose, double radian) {
    auto x = pose->x;
    auto y = pose->y;
    pose->x = x * cos(radian) - y * sin(radian);
    pose->y = x * sin(radian) + y * cos(radian);
  }

  inline geometry_msgs::Pose2D GlobalToLocal(
      const geometry_msgs::Pose2D& global,
      const geometry_msgs::Pose2D& robot_pose) {
    geometry_msgs::Pose2D local;
    local.x = global.x - robot_pose.x;
    local.y = global.y - robot_pose.y;
    RotatePose(&local, -robot_pose.theta);
    local.theta = global.theta - robot_pose.theta;
    return local;
  }

  inline geometry_msgs::Pose2D LocalToGlobal(
      const geometry_msgs::Pose2D& local,
      const geometry_msgs::Pose2D& robot_pose) {
    geometry_msgs::Pose2D global = local;
    navigator_maths::RotatePose(&global, robot_pose.theta);
    global.x += robot_pose.x;
    global.y += robot_pose.y;
    global.theta += robot_pose.theta;
    return global;
  }
}

#endif
