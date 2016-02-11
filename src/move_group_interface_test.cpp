#include "move_group_interface_test.h"

using namespace Eigen;
using namespace std;
using namespace ih;

Matrix3d tool_rot;
Quaterniond tool_quat;


    int main(int argc, char **argv) {
        ros::init(argc, argv, "move_group_interface_test");
        ros::NodeHandle node_handle;
        ros::AsyncSpinner spinner(1);
        spinner.start();

        /* This sleep is ONLY to allow Rviz to come up */
        //sleep(5.0);

        // Setup
        // ^^^^^
        // The :move_group_interface:`MoveGroup` class can be easily setup using just the name
        // of the group you would like to control and plan for.
        moveit::planning_interface::MoveGroup group("agilus1");

        // We will use the :planning_scene_interface:`PlanningSceneInterface`
        // class to deal directly with the world. USED FOR ADDING AND REMOVEING OBJECTS IN WORLD!
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Create a publisher for visualizing plans in Rviz.
        ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
                "/move_group/display_planned_path", 1, true);
        moveit_msgs::DisplayTrajectory display_trajectory;

        // Getting Basic Information
        // ^^^^^^^^^^^^^^^^^^^^^^^^^
        // We can print the name of the reference frame for this robot.
        ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

        // We can also print the name of the end-effector link for this group.
        ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

        // Planning to a joint-space goal
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Let's set a joint space goal and move towards it.  This will replace the pose target we set above.
        //
        // First get the current set of joint values for the group.
        std::vector<double> group_variable_values;
        group.getCurrentState()->copyJointGroupPositions(
                group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

        // Now, let's modify one of the joints, plan to the new joint
        // space goal and visualize the plan.
        group_variable_values[0] = 0.0;
        group_variable_values[1] = -M_PI / 2.0;
        group_variable_values[2] = M_PI / 2.0;
        group_variable_values[3] = 0.0;
        group_variable_values[4] = M_PI / 2.0;
        group_variable_values[5] = 0.0;
        group.setJointValueTarget(group_variable_values);

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group to actually move the robot.
        moveit::planning_interface::MoveGroup::Plan my_plan;
        bool success = group.plan(my_plan);

        ROS_INFO("Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(5.0);

        // Visualizing plans
        // ^^^^^^^^^^^^^^^^^
        // Now that we have a plan we can visualize it in Rviz.  This is not necessary because the group.plan() call we
        // made above did this automatically.  But explicitly publishing plans is useful in cases that we
        // want to visualize a previously created plan.
        if (1) {
            ROS_INFO("Visualizing plan 1 (again)");
            display_trajectory.trajectory_start = my_plan.start_state_;
            display_trajectory.trajectory.push_back(my_plan.trajectory_);
            display_publisher.publish(display_trajectory);
            /* Sleep to give Rviz time to visualize the plan. */
            sleep(5.0);
        }

        /* Uncomment below line when working with a real robot*/
        /* group.move() */


// END_TEST

        ros::shutdown();
        return 0;
    }



