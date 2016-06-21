#ifndef ROBOT_PTP_MOTION_HPP_
#define ROBOT_PTP_MOTION_HPP_

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

class RobotPTPMotion
{
public:
    RobotPTPMotion();
    RobotPTPMotion(const std::string movegroup);
    bool planPose(double x, double y, double z, double roll, double pitch, double yaw);
    bool homeRobot();
    void moveRobot(double max_velocity_scale_factor);
private:
    moveit::planning_interface::MoveGroup manipulator;
};

#endif // ROBOT_PTP_MOTION_HPP_
