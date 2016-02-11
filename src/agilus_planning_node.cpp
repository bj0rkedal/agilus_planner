//
// Created by Asgeir Bjørkedal on 05.02.16.
//

#include <agilus_planner/Pose.h>
#include "agilus_planning_node.h"

const std::string groupname1 = "agilus1";
const std::string groupname2 = "agilus2";
const double max_vel_scale_factor = 0.1;
const int num_planning_attempts = 1;
const int planning_time = 5;
const ih::RobotOptionFlag options = ih::ROBOT_OPTION_DUMMY_ROBOT | ih::ROBOT_OPTION_VERBOSE_INFO;

AgilusPlanningNode::AgilusPlanningNode() {

}

AgilusPlanningNode::~AgilusPlanningNode() {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "agilus_planning_node");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create the service clients for controlling the Agilus manipulators
    ros::ServiceClient goToClient_ag1 = node_handle.serviceClient<agilus_planner::Pose>("/robot_service_ag1/go_to_pose");
    ros::ServiceClient goToClient_ag2 = node_handle.serviceClient<agilus_planner::Pose>("/robot_service_ag2/go_to_pose");
    ros::ServiceClient planClient_ag1 = node_handle.serviceClient<agilus_planner::Pose>("/robot_service_ag1/plan_pose");
    ros::ServiceClient planClient_ag2 = node_handle.serviceClient<agilus_planner::Pose>("/robot_service_ag2/plan_pose");

    // Instantiate a service object
    agilus_planner::Pose pose_service;

    // Set service request parameters
    pose_service.request.header.frame_id = "/world";
    pose_service.request.relative = 0;
    pose_service.request.set_position = 1;
    pose_service.request.position_x = 0.5;
    pose_service.request.position_y = -0.4025;
    pose_service.request.position_z = 1.6;
    pose_service.request.set_orientation = 1;
    pose_service.request.orientation_r = 0.0;
    pose_service.request.orientation_p = M_PI/2.0;
    pose_service.request.orientation_y = 0.0;

    // Call the service
    goToClient_ag1.call(pose_service);

    //sleep(5.0);

    pose_service.request.position_y = 0.4025;
    //goToClient_ag2.call(pose_service);

    return 0;
}
