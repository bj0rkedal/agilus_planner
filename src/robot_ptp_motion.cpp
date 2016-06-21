#include "../include/qt_agilus_planner/robot_ptp_motion.hpp"

RobotPTPMotion::RobotPTPMotion(const std::string move_group): manipulator(move_group)
{
}

bool RobotPTPMotion::planPose(double x, double y, double z, double roll, double pitch, double yaw)
{
    geometry_msgs::Pose target_pose;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    manipulator.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroup::Plan pose_plan;
    bool success = manipulator.plan(pose_plan);

    ROS_INFO("Visualizing plan (pose goal) %s \n\t Pos: x=%f, y=%f, z=%f,\n\t Rot: x=%f, y=%f, z=%f, w=%f",
             success?"":"FAILED",
             target_pose.position.x,
             target_pose.position.y,
             target_pose.position.z,
             target_pose.orientation.x,
             target_pose.orientation.y,
             target_pose.orientation.z,
             target_pose.orientation.w);
    return success;
}

bool RobotPTPMotion::homeRobot()
{
    std::vector<double> group_variable_values;
    //manipulator->setMaxVelocityScalingFactor(0.1);
    manipulator.getCurrentState()->copyJointGroupPositions(
                manipulator.getCurrentState()->getRobotModel()->getJointModelGroup(manipulator.getName()), group_variable_values);
    group_variable_values[0] = 0.0;
    group_variable_values[1] = -M_PI/2;
    group_variable_values[2] = M_PI/2;
    group_variable_values[3] = 0.0;
    group_variable_values[4] = M_PI/2;
    group_variable_values[5] = 0.0;
    manipulator.setJointValueTarget(group_variable_values);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = manipulator.plan(my_plan);

    ROS_INFO("Visualizing plan (joint space home goal) %s \n\t Joints: A1=%f, A2=%f, A3=%f,\n\t\t A4=%f, A5=%f, A6=%f",
             success?"":"FAILED",
             group_variable_values[0],
             group_variable_values[1],
             group_variable_values[2],
             group_variable_values[3],
             group_variable_values[4],
             group_variable_values[5]);
    return success;
}

void RobotPTPMotion::moveRobot(double max_velocity_scale_factor)
{
    manipulator.setMaxVelocityScalingFactor(max_velocity_scale_factor);
    manipulator.move();
}



