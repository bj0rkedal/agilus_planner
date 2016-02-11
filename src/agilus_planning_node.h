//
// Created by Asgeir Bjørkedal on 05.02.16.
//

#ifndef AGILUS_PLANNER_AGILUS_PLANNING_NODE_H
#define AGILUS_PLANNER_AGILUS_PLANNING_NODE_H

#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "robot_planning_execution.hpp"


class AgilusPlanningNode {
public:
    AgilusPlanningNode();

    ~AgilusPlanningNode();
};

#endif //AGILUS_PLANNER_AGILUS_PLANNING_NODE_H
