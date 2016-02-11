#include "agilus_planner/Pose.h"
#include "robot_planning_execution.hpp"

ih::RobotPlanningExecution *robot;

bool planPoseService(agilus_planner::Pose::Request &req, agilus_planner::Pose::Response &res) {
    if ((bool) !req.relative) {
        if ((bool) req.set_position && (bool) !req.set_orientation) {
            res.progress = robot->planPoseByXYZ(
                    (double) req.position_x, (double) req.position_y, (double) req.position_z);
        }
        if ((bool) !req.set_position && (bool) req.set_orientation) {
            res.progress = robot->planPoseByRPY(
                    (double) req.orientation_r, (double) req.orientation_p, (double) req.orientation_y);
        }
        if ((bool) req.set_position && (bool) req.set_orientation) {
            res.progress = robot->planPoseByXYZRPY(
                    (double) req.position_x, (double) req.position_y, (double) req.position_z,
                    (double) req.orientation_r, (double) req.orientation_p, (double) req.orientation_y);
        }
    }
    else {
        if ((bool) req.set_position && (bool) !req.set_orientation) {
            res.progress = robot->planRelativePoseByXYZ(
                    (double) req.position_x, (double) req.position_y, (double) req.position_z);
        }
        if ((bool) !req.set_position && (bool) req.set_orientation) {
            res.progress = robot->planRelativePoseByRPY(
                    (double) req.orientation_r, (double) req.orientation_p, (double) req.orientation_y);
        }
        if ((bool) req.set_position && (bool) req.set_orientation) {
            res.progress = robot->planRelativePoseByXYZRPY(
                    (double) req.position_x, (double) req.position_y, (double) req.position_z,
                    (double) req.orientation_r, (double) req.orientation_p, (double) req.orientation_y);
        }
    }
}

bool goToPoseService(agilus_planner::Pose::Request &req, agilus_planner::Pose::Response &res) {
    if ((bool) !req.relative) {
        if ((bool) req.set_position && (bool) !req.set_orientation) {
            res.progress = robot->goToPoseByXYZ(
                    (double) req.position_x, (double) req.position_y, (double) req.position_z);
        }
        if ((bool) !req.set_position && (bool) req.set_orientation) {
            res.progress = robot->goToPoseByRPY(
                    (double) req.orientation_r, (double) req.orientation_p, (double) req.orientation_y);
        }
        if ((bool) req.set_position && (bool) req.set_orientation) {
            res.progress = robot->goToPoseByXYZRPY(
                    (double) req.position_x, (double) req.position_y, (double) req.position_z,
                    (double) req.orientation_r, (double) req.orientation_p, (double) req.orientation_y);
        }
    }
    else {
        if ((bool) req.set_position && (bool) !req.set_orientation) {
            res.progress = robot->goToRelativePoseByXYZ(
                    (double) req.position_x, (double) req.position_y, (double) req.position_z);
        }
        if ((bool) !req.set_position && (bool) req.set_orientation) {
            res.progress = robot->goToRelativePoseByRPY(
                    (double) req.orientation_r, (double) req.orientation_p, (double) req.orientation_y);
        }
        if ((bool) req.set_position && (bool) req.set_orientation) {
            res.progress = robot->goToRelativePoseByXYZRPY(
                    (double) req.position_x, (double) req.position_y, (double) req.position_z,
                    (double) req.orientation_r, (double) req.orientation_p, (double) req.orientation_y);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_movement_service");
    ros::NodeHandle node_handle("~");

    // Initializing robot arguments used if ROS server has no parameters
    std::string group_name = "manipulator";
    double max_vel_scale_factor = 0.1;
    int planning_time = 10;
    int num_planning_attempts = 5;
    int options = 2;

    // Get or set group_name
    if (node_handle.hasParam("group_name")) {
        node_handle.getParam("group_name", group_name);
        ROS_INFO("Got group_name: %s", group_name.c_str());
    }
    else {
        node_handle.setParam("group_name", group_name);
        ROS_INFO("No group_name found. Default used: %s", group_name.c_str());
    }
    // Get or set max_vel_scale_factor
    if (node_handle.hasParam("max_vel_scale_factor")) {
        node_handle.getParam("max_vel_scale_factor", max_vel_scale_factor);
        ROS_INFO("Got max_vel_scale_factor: %f", max_vel_scale_factor);
    }
    else {
        node_handle.setParam("max_vel_scale_factor", max_vel_scale_factor);
        ROS_INFO("No max_vel_scale_factor found. Default used: %f", max_vel_scale_factor);
    }
    // Get or set planning_time
    if (node_handle.hasParam("planning_time")) {
        node_handle.getParam("planning_time", planning_time);
        ROS_INFO("Got planning_time: %d", planning_time);
    }
    else {
        node_handle.setParam("planning_time", planning_time);
        ROS_INFO("No planning_time found. Default used: %d", planning_time);
    }
    // Get or set planning_time
    if (node_handle.hasParam("num_planning_attempts")) {
        node_handle.getParam("num_planning_attempts", num_planning_attempts);
        ROS_INFO("Got num_planning_attempts: %d", num_planning_attempts);
    }
    else {
        node_handle.setParam("num_planning_attempts", num_planning_attempts);
        ROS_INFO("No num_planning_attempts found. Default used: %d", num_planning_attempts);
    }
    // Set options regardless of server parameter
    node_handle.setParam("options", options);

    // Initializing robot
    robot = new ih::RobotPlanningExecution(
            group_name,
            max_vel_scale_factor,
            planning_time,
            num_planning_attempts,
            ih::RobotOptionFlagFromInt(options));

    // Advertise the services
    ros::ServiceServer goto_service = node_handle.advertiseService("go_to_pose", goToPoseService);
    ros::ServiceServer plan_service = node_handle.advertiseService("plan_pose", planPoseService);

    ROS_INFO("robot_movement_service ready to use");
    ros::spin();

    return 0;
}
