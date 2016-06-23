//
// Created by Asgeir Bjørkedal on 21.06.16.
// Last edit: 23.06.16.
// A simple node testing the LIN and PTP movements.
//
#include "../include/agilus_planner/robot_planning.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_planning");
    ros::NodeHandle nh;

    try {
        agilus1 = new ih::RobotPlanningExecution("agilus1", 0.5, 5, 10, ih::ROBOT_OPTION_VERBOSE_INFO);
        agilus2 = new ih::RobotPlanningExecution("agilus2", 0.5, 5, 10, ih::ROBOT_OPTION_VERBOSE_INFO);
    } catch (...) {
        ROS_ERROR("Check that the move groups exist!");
        return 0;
    }
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    // LIN
    agilus1->planPoseByXYZRPY(0.35, -0.6025, 1.5, 0.0, M_PI, M_PI / 3);
    std::cout << "Press Enter to Continue";
    std::cin.ignore();
    agilus1->goToPoseByXYZRPY(0.35, -0.6025, 1.5, 0.0, M_PI, M_PI / 3);
    sleep(1);

    // PTP
    agilus1->planPosePTP(0.1, -0.1, 1.4, 0.0, M_PI, M_PI / 3);
    std::cout << "Press Enter to Continue";
    std::cin.ignore();
    agilus1->moveRobot(0.1);
    sleep(1);

    // PTP
    agilus1->planPosePTP(0.1, -0.1, 1.4, 0.0, M_PI / 2, 0.0);
    std::cout << "Press Enter to Continue";
    std::cin.ignore();
    agilus1->moveRobot(0.1);
    sleep(1);

    // Relative PTP
    agilus1->goToRelativePosePTP(0.3,0.0,0.0,0.0,0.0,0.0);

    // Home agilus 1
    agilus1->homeRobot();

    // Relative LIN
    agilus2->planRelativePoseByXYZRPY(0.0, 0.0, -0.1, 0.0, 0.0, 0.0);
    std::cout << "Press Enter to Continue";
    std::cin.ignore();
    agilus2->goToRelativePoseByXYZRPY(0.0, 0.0, -0.1, 0.0, 0.0, 0.0);
    sleep(1);

    // LIN
    agilus2->planPoseByXYZRPY(0.445, 0.5, 1.6, 0.0, M_PI, 0.0);
    std::cout << "Press Enter to Continue";
    std::cin.ignore();
    agilus2->goToPoseByXYZRPY(0.445, 0.5, 1.6, 0.0, M_PI, 0.0);
    sleep(1);

    // PTP
    agilus2->planPosePTP(0.0, 0.0, 1.4, 0.0, M_PI, 0.0);
    std::cout << "Press Enter to Continue";
    std::cin.ignore();
    agilus2->moveRobot(0.1);
    sleep(1);

    // Home agilus 2
    agilus2->homeRobot();

    ROS_INFO("Shutting down PTP demo!");
    ros::shutdown();
    ros::waitForShutdown();
    return 0;
}

