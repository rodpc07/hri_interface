#include "hri_interface/hri_interface.h"

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <memory>
#include <functional>
#include <algorithm>
#include <Eigen/Geometry>

HRI_Interface::HRI_Interface(ros::NodeHandle n,
                             std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_mgi,
                             const moveit::core::JointModelGroup *arm_jmg,
                             std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_mgi,
                             const moveit::core::JointModelGroup *gripper_jmg,
                             std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
                             std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools)
    : n_(n), arm_mgi_(arm_mgi), arm_jmg_(arm_jmg), gripper_mgi_(gripper_mgi), gripper_jmg_(gripper_jmg), planning_scene_interface_(planning_scene_interface), visual_tools_(visual_tools)
{
    ROS_INFO("Initializing HRI_INTERFACE");
    planningSceneClient = n_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(arm_mgi_->getRobotModel());
    planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), true);

    prePlanPick();
}

void HRI_Interface::exagerateTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan, Eigen::Vector3d xyz)
{
    // REFERENCE: https://github.com/ros-planning/moveit/issues/1556

    robot_state::RobotStatePtr kinematic_state(arm_mgi_->getCurrentState());

    int num_waypoints = plan.trajectory_.joint_trajectory.points.size(); // gets the number of waypoints in the trajectory
    int mid_waypoint = num_waypoints / 2;

    const std::vector<std::string> joint_names = plan.trajectory_.joint_trajectory.joint_names; // gets the names of the joints being updated in the trajectory
    Eigen::Affine3d current_end_effector_state;
    std::vector<double> previous_joint_values = plan.trajectory_.joint_trajectory.points.at(0).positions;

    double scale = 0;
    double previous_max_scale = 0;

    Eigen::Vector3d xyz_max = xyz;

    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();

    bool success_IK = true;

    std::string gripper = gripper_mgi_->getLinkNames().at(0);
    ROS_INFO("Gripper: %s", gripper.c_str());
    // FIRST LOOPS TO SET THE MAXIMUM RANGE OF THE ARM
    do
    {
        previous_max_scale = 0;

        for (int i = 0; i < num_waypoints; i++)
        {
            // set joint positions of waypoint
            kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(i).positions);
            current_end_effector_state = kinematic_state->getGlobalLinkTransform(gripper);

            // FIRST LOOP TO FIND THE MAX TRANSLATION

            target_pose = Eigen::Isometry3d::Identity();
            target_pose.translation() = current_end_effector_state.translation();

            if (i < mid_waypoint)
            {
                scale = i / static_cast<double>(mid_waypoint);
            }
            else if (i > mid_waypoint)
            {
                scale = 1 - (i - mid_waypoint) / static_cast<double>(mid_waypoint);
            }
            else
            {
                scale = 1;
            }

            ROS_INFO_STREAM(scale);

            target_pose.translate(xyz_max * scale);
            target_pose.linear() = current_end_effector_state.rotation();

            visual_tools_->publishAxisLabeled(target_pose, "Pose");
            visual_tools_->trigger();

            kinematic_state->setJointGroupPositions(arm_jmg_, previous_joint_values);
            success_IK = kinematic_state->setFromIK(arm_jmg_, target_pose);

            std::stringstream tmp;
            tmp << i;
            char const *iteration = tmp.str().c_str();
            ROS_INFO_NAMED("First Loop", "Iteration %s", success_IK ? iteration : "FAILED");

            visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

            if (!success_IK)
            {
                xyz_max = xyz_max * previous_max_scale;
                break;
            }

            previous_max_scale = i <= mid_waypoint ? scale : previous_max_scale;
        }
    } while (!success_IK);

    visual_tools_->deleteAllMarkers();

    for (int i = 0; i < num_waypoints; i++)
    {
        // set joint positions of waypoint
        kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(i).positions);
        current_end_effector_state = kinematic_state->getGlobalLinkTransform(gripper);

        bool success_IK = true;

        target_pose = Eigen::Isometry3d::Identity();
        target_pose.translation() = current_end_effector_state.translation();

        if (i < mid_waypoint)
        {
            scale = i / static_cast<double>(mid_waypoint);
        }
        else if (i > mid_waypoint)
        {
            scale = 1 - (i - mid_waypoint) / static_cast<double>(mid_waypoint);
        }
        else
        {
            scale = 1;
        }

        ROS_INFO_STREAM(scale);

        target_pose.translate(xyz_max * scale);
        target_pose.linear() = current_end_effector_state.rotation();

        visual_tools_->publishAxisLabeled(target_pose, "Pose");
        visual_tools_->trigger();

        kinematic_state->setJointGroupPositions(arm_jmg_, previous_joint_values);
        success_IK = kinematic_state->setFromIK(arm_jmg_, target_pose);

        std::stringstream tmp;
        tmp << i;
        char const *iteration = tmp.str().c_str();
        ROS_INFO_NAMED("tutorial", "Iteration %s", success_IK ? iteration : "FAILED");

        visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        kinematic_state->copyJointGroupPositions(arm_jmg_, plan.trajectory_.joint_trajectory.points.at(i).positions);

        previous_joint_values = plan.trajectory_.joint_trajectory.points.at(i).positions;
    }

    arm_mgi_->execute(plan);
}

bool HRI_Interface::waving(std::string target_frame)
{
    ROS_INFO("Attempting \"Waving\"");

    geometry_msgs::TransformStamped targetTransform;

    // Get transform from human position in reference to base_link

    if (!transformListener(target_frame, "yumi_base_link", targetTransform))
    {
        ROS_ERROR("Can't perform transform");
        return false;
    }

    arm_mgi_->setStartStateToCurrentState();
    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(1));

    // This joint values set the pose of the arm in a V shape (depends on robot morphology)

    std::vector<double> joint_group_positions;
    if (!arm_mgi_->getName().compare("right_arm"))
    {
        joint_group_positions = {125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 90 * (M_PI / 180), 0, 0};
    }
    else
    {
        joint_group_positions = {-125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), -90 * (M_PI / 180), 0, 0};
    }

    arm_state.setJointGroupPositions(arm_jmg_, joint_group_positions);

    const Eigen::Isometry3d &end_effector_state = arm_state.getGlobalLinkTransform(gripper_mgi_->getLinkNames().at(0));

    // Next is done the calculations to obtain the angle for yumi_joint_2 needed to set the arm aligned with the human

    double xTarget = targetTransform.transform.translation.x - linkTransform.translation().x();
    double yTarget = targetTransform.transform.translation.y - linkTransform.translation().y();

    double xEE = end_effector_state.translation().x() - linkTransform.translation().x();
    double yEE = end_effector_state.translation().y() - linkTransform.translation().y();

    double target_angle = atan2(yTarget, xTarget);
    double ee_angle = atan2(yEE, xEE);

    double angle = ee_angle - target_angle;

    joint_group_positions.at(2) = angle;
    arm_mgi_->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_1;
    if (!(arm_mgi_->plan(arm_plan_1) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for first movement");
        return false;
    }

    // Next are planned in advance 4 motions to simulate waving, later executed sequecially

    arm_state.setJointGroupPositions(arm_jmg_, arm_plan_1.trajectory_.joint_trajectory.points.back().positions);
    joint_group_positions.at(5) = -25 * (M_PI / 180);
    arm_mgi_->setStartState(arm_state);
    arm_mgi_->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_2;
    if (!(arm_mgi_->plan(arm_plan_2) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for second movement");
        return false;
    }

    arm_state.setJointGroupPositions(arm_jmg_, arm_plan_2.trajectory_.joint_trajectory.points.back().positions);
    joint_group_positions.at(5) = 25 * (M_PI / 180);
    arm_mgi_->setStartState(arm_state);
    arm_mgi_->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_3;
    if (!(arm_mgi_->plan(arm_plan_3) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for third movement");
        return false;
    }

    arm_state.setJointGroupPositions(arm_jmg_, arm_plan_3.trajectory_.joint_trajectory.points.back().positions);
    joint_group_positions.at(5) = -25 * (M_PI / 180);
    arm_mgi_->setStartState(arm_state);
    arm_mgi_->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_4;
    if (!(arm_mgi_->plan(arm_plan_4) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for fourth movement");
        return false;
    }

    arm_state.setJointGroupPositions(arm_jmg_, arm_plan_4.trajectory_.joint_trajectory.points.back().positions);
    joint_group_positions.at(5) = 0 * (M_PI / 180);
    arm_mgi_->setStartState(arm_state);
    arm_mgi_->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_5;
    if (!(arm_mgi_->plan(arm_plan_5) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for fifth movement");
        return false;
    }

    arm_mgi_->setStartStateToCurrentState();
    arm_mgi_->execute(arm_plan_1);
    arm_mgi_->execute(arm_plan_2);
    arm_mgi_->execute(arm_plan_3);
    arm_mgi_->execute(arm_plan_4);
    arm_mgi_->execute(arm_plan_5);

    return true;
}

bool HRI_Interface::comeClose(std::string target_frame)
{

    ROS_INFO("Attempting \"Come Close\"");

    geometry_msgs::TransformStamped targetTransform;

    // Get transform from human position in reference to base_link

    if (!transformListener(target_frame, "yumi_base_link", targetTransform))
    {
        ROS_ERROR("Can't perform transform");
        return false;
    }

    arm_mgi_->setStartStateToCurrentState();
    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(1));

    // This joint values set the pose of the arm in a V shape (depends on robot morphology)

    std::vector<double> joint_group_positions;
    if (!arm_mgi_->getName().compare("right_arm"))
    {
        joint_group_positions = {125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 0, 0, -90 * (M_PI / 180)};
    }
    else
    {
        joint_group_positions = {-125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 0, 0, -90 * (M_PI / 180)};
    }

    arm_state.setJointGroupPositions(arm_jmg_, joint_group_positions);

    const Eigen::Isometry3d &end_effector_state = arm_state.getGlobalLinkTransform(gripper_mgi_->getLinkNames().at(0));

    // Next is done the calculations to obtain the angle for yumi_joint_2 needed to set the arm aligned with the human

    double xTarget = targetTransform.transform.translation.x - linkTransform.translation().x();
    double yTarget = targetTransform.transform.translation.y - linkTransform.translation().y();

    double xEE = end_effector_state.translation().x() - linkTransform.translation().x();
    double yEE = end_effector_state.translation().y() - linkTransform.translation().y();

    double target_angle = atan2(yTarget, xTarget);
    double ee_angle = atan2(yEE, xEE);

    double angle = ee_angle - target_angle;

    joint_group_positions.at(2) = angle;
    arm_mgi_->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_1;
    if (!(arm_mgi_->plan(arm_plan_1) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for first movement");
        return false;
    }

    // Next are planned in advance 4 motions to simulate waving, later executed sequecially

    arm_state.setJointGroupPositions(arm_jmg_, arm_plan_1.trajectory_.joint_trajectory.points.back().positions);
    joint_group_positions.at(5) = 45 * (M_PI / 180);
    arm_mgi_->setStartState(arm_state);
    arm_mgi_->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_2;
    if (!(arm_mgi_->plan(arm_plan_2) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for second movement");
        return false;
    }

    arm_state.setJointGroupPositions(arm_jmg_, arm_plan_2.trajectory_.joint_trajectory.points.back().positions);
    joint_group_positions.at(5) = 0 * (M_PI / 180);
    arm_mgi_->setStartState(arm_state);
    arm_mgi_->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_3;
    if (!(arm_mgi_->plan(arm_plan_3) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for third movement");
        return false;
    }

    arm_mgi_->setStartStateToCurrentState();
    arm_mgi_->execute(arm_plan_1);
    arm_mgi_->execute(arm_plan_2);
    arm_mgi_->execute(arm_plan_3);
    arm_mgi_->execute(arm_plan_2);
    arm_mgi_->execute(arm_plan_3);

    return true;
}

bool HRI_Interface::goAway(std::string target_frame)
{

    ROS_INFO("Attempting \"Go Away\"");

    geometry_msgs::TransformStamped targetTransform;

    // Get transform from human position in reference to base_link

    if (!transformListener(target_frame, "yumi_base_link", targetTransform))
    {
        ROS_ERROR("Can't perform transform");
        return false;
    }

    arm_mgi_->setStartStateToCurrentState();
    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(1));

    // This joint values set the pose of the arm in a V shape (depends on robot morphology)

    std::vector<double> joint_group_positions;
    if (!arm_mgi_->getName().compare("right_arm"))
    {
        joint_group_positions = {125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 0, 0, -90 * (M_PI / 180)};
    }
    else
    {
        joint_group_positions = {-125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 0, 0, -90 * (M_PI / 180)};
    }

    arm_state.setJointGroupPositions(arm_jmg_, joint_group_positions);

    const Eigen::Isometry3d &end_effector_state = arm_state.getGlobalLinkTransform(gripper_mgi_->getLinkNames().at(0));

    // Next is done the calculations to obtain the angle for yumi_joint_2 needed to set the arm aligned with the human

    double xTarget = targetTransform.transform.translation.x - linkTransform.translation().x();
    double yTarget = targetTransform.transform.translation.y - linkTransform.translation().y();

    double xEE = end_effector_state.translation().x() - linkTransform.translation().x();
    double yEE = end_effector_state.translation().y() - linkTransform.translation().y();

    double target_angle = atan2(yTarget, xTarget);
    double ee_angle = atan2(yEE, xEE);

    double angle = ee_angle - target_angle;

    joint_group_positions.at(2) = angle;
    arm_mgi_->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_1;
    if (!(arm_mgi_->plan(arm_plan_1) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for first movement");
        return false;
    }

    // Next are planned in advance 4 motions to simulate waving, later executed sequecially

    arm_state.setJointGroupPositions(arm_jmg_, arm_plan_1.trajectory_.joint_trajectory.points.back().positions);
    joint_group_positions.at(5) = -45 * (M_PI / 180);
    arm_mgi_->setStartState(arm_state);
    arm_mgi_->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_2;
    if (!(arm_mgi_->plan(arm_plan_2) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for second movement");
        return false;
    }

    arm_state.setJointGroupPositions(arm_jmg_, arm_plan_2.trajectory_.joint_trajectory.points.back().positions);
    joint_group_positions.at(5) = 0 * (M_PI / 180);
    arm_mgi_->setStartState(arm_state);
    arm_mgi_->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_3;
    if (!(arm_mgi_->plan(arm_plan_3) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for third movement");
        return false;
    }

    arm_mgi_->setStartStateToCurrentState();
    arm_mgi_->execute(arm_plan_1);
    arm_mgi_->execute(arm_plan_2);
    arm_mgi_->execute(arm_plan_3);
    arm_mgi_->execute(arm_plan_2);
    arm_mgi_->execute(arm_plan_3);

    return true;
}

bool HRI_Interface::screw_unscrew(bool mode, geometry_msgs::Pose input_pose)
{
    if (mode)
        ROS_INFO("Attempting \"Screwing\"");
    else
        ROS_INFO("Attempting \"Unscrewing\"");

    arm_mgi_->setStartStateToCurrentState();

    double distance = 0.05;
    double maxBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).max_position_;
    double minBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).min_position_;

    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    Eigen::Isometry3d ref_eigen;
    visual_tools_->convertPoseSafe(input_pose, ref_eigen);
    ref_eigen.translate(Eigen::Vector3d(0, 0, 0.14));

    auto ref_pose = visual_tools_->convertPose(ref_eigen);

    if (!computeLookPose(arm_state, input_pose, ref_pose, 5, 5, 0.1, 0.75, 0.75))
    {
        ROS_ERROR("The arm can't find a suitable pose");
        return false;
    }

    std::vector<double> unscrew_joint_group_positions;
    arm_state.copyJointGroupPositions(arm_jmg_, unscrew_joint_group_positions);

    // Set gripper joint to limit to allow a full range of rotation

    unscrew_joint_group_positions.back() = maxBound;
    arm_state.setJointGroupPositions(arm_jmg_, unscrew_joint_group_positions);

    // Perform action of EndEffector
    geometry_msgs::Pose start_end_effector_pose = visual_tools_->convertPose(arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().back()));
    Eigen::Isometry3d goal_end_effector_eigen;

    double turn_angle = -M_PI_4;
    int distance_ratio = (maxBound - minBound) / abs(turn_angle);
    Eigen::Vector3d translation(0, 0, -distance / distance_ratio);

    double fraction;
    moveit_msgs::RobotTrajectory trajectory;

    for (int attempt = 0; fraction < 0.8 && attempt < 5; attempt++)
    {
        geometry_msgs::Pose goal_end_effector_pose = start_end_effector_pose;
        std::vector<geometry_msgs::Pose> waypoints;

        for (double angle = 0; angle <= 458 * (M_PI / 180); angle += abs(turn_angle))
        {
            // Translation
            visual_tools_->convertPoseSafe(goal_end_effector_pose, goal_end_effector_eigen);
            goal_end_effector_eigen.translate(translation);
            goal_end_effector_pose = visual_tools_->convertPose(goal_end_effector_eigen);

            // Rotation
            tf2::Quaternion q_orig, q_rot, q_new;
            tf2::convert(goal_end_effector_pose.orientation, q_orig);
            q_rot.setRPY(0, 0, turn_angle);
            q_new = q_orig * q_rot;
            q_new.normalize();
            geometry_msgs::Quaternion rotated_quat;
            tf2::convert(q_new, rotated_quat);
            goal_end_effector_pose.orientation = rotated_quat;

            waypoints.push_back(goal_end_effector_pose);
        }

        arm_mgi_->setStartState(arm_state);

        if (mode)
        {
            ROS_INFO("REVERSE VECTOR");
            std::reverse(waypoints.begin(), waypoints.end());

            arm_state.setFromIK(arm_jmg_, waypoints.at(0), 0.1);

            std::vector<double> screw_joint_group_positions;
            arm_state.copyJointGroupPositions(arm_jmg_, screw_joint_group_positions);

            // Set gripper joint to limit to allow a full range of rotation

            screw_joint_group_positions.back() = minBound;
            arm_state.setJointGroupPositions(arm_jmg_, screw_joint_group_positions);

            arm_mgi_->setStartState(arm_state);
        }
        else
        {
            arm_state.setJointGroupPositions(arm_jmg_, unscrew_joint_group_positions);
            arm_mgi_->setStartState(arm_state);
        }

        const double jump_threshold = 4;
        const double eef_step = 0.05;

        fraction = arm_mgi_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        visual_tools_->deleteAllMarkers();

        for (std::size_t i = 0; i < waypoints.size(); ++i)
            visual_tools_->publishAxis(waypoints[i]);
        visual_tools_->trigger();

        if (fraction < 0.8 && attempt < 5)
        {
            translation.z() = translation.z() * 0.95;
            ROS_INFO_NAMED("tutorial", "Cartesian path FAILED (%.2f%% achieved)", fraction * 100.0);
        }
        else if (fraction >= 0.8)
        {
            ROS_INFO_NAMED("tutorial", "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);
        }
        else
        {
            ROS_ERROR("Can't plan for full motion");
            return false;
        }
    }

    arm_mgi_->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan planSetInitialPosition;
    arm_mgi_->setJointValueTarget(trajectory.joint_trajectory.points.at(0).positions);

    if (!(arm_mgi_->plan(planSetInitialPosition) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for initial position");
        return false;
    }

    gripper_mgi_->setNamedTarget("close");
    gripper_mgi_->move();
    arm_mgi_->execute(planSetInitialPosition);
    arm_mgi_->execute(trajectory);
    visual_tools_->deleteAllMarkers();

    return true;
}

void HRI_Interface::signalPick()
{

    ROS_INFO("Attempting \"Picking\"");

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> planList;
    gripper_mgi_->setStartStateToCurrentState();

    if (abs(gripper_mgi_->getCurrentJointValues().at(0) - openedJointValues.at(0)) > 0.001)
        planList.push_back(openPlan);

    planList.push_back(closePlan);
    planList.push_back(openPlan);
    planList.push_back(closePlan);

    for (const auto &planValue : planList)
    {
        gripper_mgi_->execute(planValue);
    }
}

bool HRI_Interface::signalRotate(std::string object_id, Eigen::Vector3d rotationInfo)
{

    ROS_INFO("Attempting \"Rotating\"");

    visual_tools_->deleteAllMarkers();

    // Obtain object from scene
    std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_->getObjects();

    const double tolerance = 1e-6;
    if (objects.count(object_id) == 0)
    {
        ROS_ERROR("Object does not exist.");
        return false;
    }
    if (!(rotationInfo.isApprox(Eigen::Vector3d(1, 0, 0), tolerance) || rotationInfo.isApprox(Eigen::Vector3d(-1, 0, 0), tolerance) ||
          rotationInfo.isApprox(Eigen::Vector3d(0, 1, 0), tolerance) || rotationInfo.isApprox(Eigen::Vector3d(0, -1, 0), tolerance) ||
          rotationInfo.isApprox(Eigen::Vector3d(0, 0, 1), tolerance) || rotationInfo.isApprox(Eigen::Vector3d(0, 0, -1), tolerance)))
    {
        ROS_ERROR("Rotation information is invalid! Must only provide information about one axis [x, y or z] and direction [1 or -1]");
        return false;
    }

    // Find position of object
    moveit_msgs::CollisionObject object = objects[object_id];
    geometry_msgs::Pose object_pose = object.pose;

    double radius = sqrt(pow(object.primitives[0].dimensions[0], 2) + pow(object.primitives[0].dimensions[1], 2) + pow(object.primitives[0].dimensions[2], 2));
    visual_tools_->publishAxis(object_pose);
    visual_tools_->trigger();

    Eigen::Isometry3d objectEigen;
    visual_tools_->convertPoseSafe(object_pose, objectEigen);

    std::vector<Eigen::Isometry3d> approach_options;
    for (int i = 0; i < 2; i++)
    {
        approach_options.push_back(objectEigen);
        approach_options[i].translate(rotationInfo * pow(-1, i) * (radius + 0.12));
    }

    arm_mgi_->setStartStateToCurrentState();
    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(0));

    std::vector<Eigen::Isometry3d> closestApproachOption = findClosestApproachOption(approach_options, linkTransform);

    visual_tools_->publishAxis(approach_options[0]);
    visual_tools_->publishAxis(approach_options[1]);
    visual_tools_->publishAxisLabeled(closestApproachOption[0], "Closest");
    visual_tools_->trigger();

    bool sucess_pose = false;
    geometry_msgs::Pose lookPose;

    for (const auto approach : closestApproachOption)
    {

        double xTarget = object_pose.position.x - approach.translation().x();
        double yTarget = object_pose.position.y - approach.translation().y();
        double zTarget = object_pose.position.z - approach.translation().z();

        double sideAngle = atan2(yTarget, xTarget);
        double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

        tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
        tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
        tf2::Quaternion qresult = q1 * q2;
        qresult.normalize();

        geometry_msgs::Quaternion q_msg;
        tf2::convert(qresult, q_msg);

        lookPose.position.x = approach.translation().x();
        lookPose.position.y = approach.translation().y();
        lookPose.position.z = approach.translation().z();
        lookPose.orientation = q_msg;

        visual_tools_->publishAxis(lookPose);
        visual_tools_->trigger();

        if (computeLookPose(arm_state, lookPose, object_pose, 10, 5, 0.2, 1.0, 1.0))
        {
            sucess_pose = true;
            break;
        }
    }

    if (!sucess_pose)
    {
        ROS_ERROR("Can't find suitable pose");
        return false;
    }

    std::vector<double> lookPose_joint_values;
    arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_values);

    double maxBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).max_position_;
    double minBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).min_position_;

    if (abs(maxBound - lookPose_joint_values.back()) < M_PI_2)
    {
        lookPose_joint_values.back() -= M_PI_2 - abs(maxBound - lookPose_joint_values.back());
    }
    else if (abs(minBound - lookPose_joint_values.back()) < M_PI_2)
    {
        lookPose_joint_values.back() += M_PI_2 - abs(minBound - lookPose_joint_values.back());
    }

    arm_mgi_->setJointValueTarget(lookPose_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan first_movement;
    if (!(arm_mgi_->plan(first_movement) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for pointing movement");
        return false;
    }

    // Set Rotated Position as JointValueTarget and Plan
    lookPose_joint_values = first_movement.trajectory_.joint_trajectory.points.back().positions;
    arm_state.setJointGroupPositions(arm_jmg_, lookPose_joint_values);
    arm_mgi_->setStartState(arm_state);
    // Change End-Effector Rotation
    std::vector<double> rotated_joint_values = lookPose_joint_values;

    // Check if the Z axis of the EE is aligned with the object rotation axis
    Eigen::Quaterniond look_orientation(lookPose.orientation.w, lookPose.orientation.x, lookPose.orientation.y, lookPose.orientation.z);
    Eigen::Vector3d look_z_axis = look_orientation * Eigen::Vector3d::UnitZ();

    Eigen::Quaterniond object_orientation(object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z);
    Eigen::Vector3d object_rotation_vector = object_orientation * rotationInfo;

    if (look_z_axis.dot(object_rotation_vector) > 0)
    {
        rotated_joint_values.back() += M_PI_2;
    }
    else
    {
        rotated_joint_values.back() -= M_PI_2;
    }

    arm_mgi_->setJointValueTarget(rotated_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan first_rotation;
    if (!(arm_mgi_->plan(first_rotation) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for first rotation");
        return false;
    }

    // Set Pre-Rotation Position as JointValueTarget and Plan
    arm_state.setJointGroupPositions(arm_jmg_, rotated_joint_values);
    arm_mgi_->setStartState(arm_state);
    arm_mgi_->setJointValueTarget(lookPose_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan second_rotation;
    if (!(arm_mgi_->plan(second_rotation) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for second rotation");
        return false;
    }

    gripper_mgi_->execute(openPlan);
    arm_mgi_->execute(first_movement);
    arm_mgi_->execute(first_rotation);
    arm_mgi_->execute(second_rotation);

    return true;
}

bool HRI_Interface::pointToPoint(geometry_msgs::Point point)
{

    ROS_INFO("Attempting \"Pointing To Point\"");

    visual_tools_->deleteAllMarkers();

    arm_mgi_->setStartStateToCurrentState();
    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    // Create a vector from shoulder to object to calculate pose

    Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(0));

    double xTarget = point.x - linkTransform.translation().x();
    double yTarget = point.y - linkTransform.translation().y();
    double zTarget = point.z - linkTransform.translation().z();

    double distance = sqrt(pow(xTarget, 2) + pow(yTarget, 2) + pow(zTarget, 2));

    double targetDistance = distance - 0.20 >= 0.6 ? 0.6 : distance - 0.20;

    double scalingFactor = (targetDistance) / distance;

    xTarget *= scalingFactor;
    yTarget *= scalingFactor;
    zTarget *= scalingFactor;

    double sideAngle = atan2(yTarget, xTarget);
    double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

    tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
    tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
    tf2::Quaternion qresult = q1 * q2;
    qresult.normalize();

    geometry_msgs::Quaternion q_msg;
    tf2::convert(qresult, q_msg);

    geometry_msgs::Pose lookPose;
    lookPose.position.x = linkTransform.translation().x() + xTarget;
    lookPose.position.y = linkTransform.translation().y() + yTarget;
    lookPose.position.z = linkTransform.translation().z() + zTarget;
    lookPose.orientation = q_msg;

    visual_tools_->publishAxis(lookPose);
    visual_tools_->trigger();

    geometry_msgs::Pose pointPose;
    pointPose.position = point;
    pointPose.orientation.w = 1.0;

    if (!computeLookPose(arm_state, lookPose, pointPose, 10, 5, 0.2, 2.0, 2.0))
    {
        return false;
    }

    std::vector<double> lookPose_joint_positions;
    arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_positions);
    arm_mgi_->setJointValueTarget(lookPose_joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!(arm_mgi_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for pointing movement");
        return false;
    }

    gripper_mgi_->setNamedTarget("close");
    gripper_mgi_->move();
    arm_mgi_->execute(plan);
    return true;
}

bool HRI_Interface::pointToObject(std::string object_id)
{

    ROS_INFO("Attempting \"Pointing To Object\"");

    visual_tools_->deleteAllMarkers();

    // Obtain object from scene
    std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_->getObjects();

    // Find position of object
    moveit_msgs::CollisionObject object = objects[object_id];
    geometry_msgs::Pose object_pose = object.pose;

    double radius = sqrt(pow(object.primitives[0].dimensions[0], 2) + pow(object.primitives[0].dimensions[1], 2) + pow(object.primitives[0].dimensions[2], 2));
    visual_tools_->publishSphere(object_pose, rviz_visual_tools::PINK, radius);
    visual_tools_->publishAxis(object.pose);
    visual_tools_->trigger();

    visual_tools_->prompt("");

    // Create a vector from shoulder to object to calculate pose

    arm_mgi_->setStartStateToCurrentState();
    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(0));

    double xTarget = object_pose.position.x - linkTransform.translation().x();
    double yTarget = object_pose.position.y - linkTransform.translation().y();
    double zTarget = object_pose.position.z - linkTransform.translation().z();

    double distance = sqrt(pow(xTarget, 2) + pow(yTarget, 2) + pow(zTarget, 2));

    double targetDistance = distance - radius - 0.15 >= 0.6 ? 0.6 : distance - radius - 0.15;

    double scalingFactor = (targetDistance) / distance;

    xTarget *= scalingFactor;
    yTarget *= scalingFactor;
    zTarget *= scalingFactor;

    double sideAngle = atan2(yTarget, xTarget);
    double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

    tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
    tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
    tf2::Quaternion qresult = q1 * q2;
    qresult.normalize();

    geometry_msgs::Quaternion q_msg;
    tf2::convert(qresult, q_msg);

    geometry_msgs::Pose lookPose;
    lookPose.position.x = linkTransform.translation().x() + xTarget;
    lookPose.position.y = linkTransform.translation().y() + yTarget;
    lookPose.position.z = linkTransform.translation().z() + zTarget;
    lookPose.orientation = q_msg;

    visual_tools_->publishAxis(lookPose);
    visual_tools_->trigger();

    visual_tools_->prompt("");

    if (!computeLookPose(arm_state, lookPose, object_pose, 10, 5, 0.2, 2.0, 2.0))
    {
        return false;
    }

    std::vector<double> lookPose_joint_positions;
    arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_positions);
    arm_mgi_->setJointValueTarget(lookPose_joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!(arm_mgi_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for pointing movement");
        return false;
    }

    gripper_mgi_->setNamedTarget("close");
    gripper_mgi_->move();
    arm_mgi_->execute(plan);

    return true;
}

bool HRI_Interface::pointToObjectSide(std::string object_id, Eigen::Vector3d sideInfo)
{
    ROS_INFO("Attempting \"Pointing To Object Side\"");

    visual_tools_->deleteAllMarkers();

    // Obtain object from scene
    std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_->getObjects();

    const double tolerance = 1e-6;
    if (objects.count(object_id) == 0)
    {
        ROS_ERROR("Object does not exist.");
        return false;
    }
    if (!(sideInfo.isApprox(Eigen::Vector3d(1, 0, 0), tolerance) || sideInfo.isApprox(Eigen::Vector3d(-1, 0, 0), tolerance) ||
          sideInfo.isApprox(Eigen::Vector3d(0, 1, 0), tolerance) || sideInfo.isApprox(Eigen::Vector3d(0, -1, 0), tolerance) ||
          sideInfo.isApprox(Eigen::Vector3d(0, 0, 1), tolerance) || sideInfo.isApprox(Eigen::Vector3d(0, 0, -1), tolerance)))
    {
        ROS_ERROR("Side information is invalid! Must only provide information about one axis [x, y or z] and direction [1 or -1]");
        return false;
    }

    // Find position of object
    moveit_msgs::CollisionObject object = objects[object_id];
    geometry_msgs::Pose object_pose = object.pose;

    double radius = sqrt(pow(object.primitives[0].dimensions[0], 2) + pow(object.primitives[0].dimensions[1], 2) + pow(object.primitives[0].dimensions[2], 2));
    visual_tools_->publishAxis(object_pose);
    visual_tools_->trigger();

    // Create a vector from shoulder to object to calculate pose

    Eigen::Isometry3d objectEigen;
    visual_tools_->convertPoseSafe(object_pose, objectEigen);

    objectEigen.translate(sideInfo * (radius + 0.1));

    visual_tools_->publishAxis(objectEigen);
    visual_tools_->trigger();

    double xTarget = object_pose.position.x - objectEigen.translation().x();
    double yTarget = object_pose.position.y - objectEigen.translation().y();
    double zTarget = object_pose.position.z - objectEigen.translation().z();

    double sideAngle = atan2(yTarget, xTarget);
    double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

    tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
    tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
    tf2::Quaternion qresult = q1 * q2;
    qresult.normalize();

    geometry_msgs::Quaternion q_msg;
    tf2::convert(qresult, q_msg);

    geometry_msgs::Pose lookPose;
    lookPose.position.x = objectEigen.translation().x();
    lookPose.position.y = objectEigen.translation().y();
    lookPose.position.z = objectEigen.translation().z();
    lookPose.orientation = q_msg;

    visual_tools_->publishAxis(lookPose);
    visual_tools_->trigger();

    arm_mgi_->setStartStateToCurrentState();
    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    if (!computeLookPose(arm_state, lookPose, object_pose, 10, 5, 0.2, 0.75, 0.75))
    {
        return false;
    }

    std::vector<double>
        lookPose_joint_values;
    arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_values);

    arm_mgi_->setJointValueTarget(lookPose_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!(arm_mgi_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for pointing movement");
        return false;
    }

    gripper_mgi_->setNamedTarget("close");
    gripper_mgi_->move();
    arm_mgi_->execute(plan);

    return true;
}

bool HRI_Interface::pointToHuman(std::string target_frame)
{
    ROS_INFO("Attempting \"Pointing To Human\"");

    visual_tools_->deleteAllMarkers();

    // Calculate New Pose Looking at Human
    geometry_msgs::TransformStamped targetTransform;

    // Get transform from human position in reference to base_link

    if (!transformListener(target_frame, "yumi_base_link", targetTransform))
    {
        ROS_ERROR("Can't perform transform");
        return false;
    }

    arm_mgi_->setStartStateToCurrentState();
    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(0));

    double xTarget = targetTransform.transform.translation.x - linkTransform.translation().x();
    double yTarget = targetTransform.transform.translation.y - linkTransform.translation().y();
    double zTarget = targetTransform.transform.translation.z - linkTransform.translation().z();

    double distance = sqrt(pow(xTarget, 2) + pow(yTarget, 2) + pow(zTarget, 2));

    double targetDistance = distance - 0.5 >= 0.6 ? 0.6 : distance - 0.5;

    double scalingFactor = (targetDistance) / distance;

    xTarget *= scalingFactor;
    yTarget *= scalingFactor;
    zTarget *= scalingFactor;

    double sideAngle = atan2(yTarget, xTarget);
    double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

    tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
    tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
    tf2::Quaternion qresult = q1 * q2;
    qresult.normalize();

    geometry_msgs::Quaternion q_msg;
    tf2::convert(qresult, q_msg);

    geometry_msgs::Pose lookPose;
    lookPose.position.x = linkTransform.translation().x() + xTarget;
    lookPose.position.y = linkTransform.translation().y() + yTarget;
    lookPose.position.z = linkTransform.translation().z() + zTarget;
    lookPose.orientation = q_msg;

    visual_tools_->publishAxis(lookPose);
    visual_tools_->trigger();

    geometry_msgs::Pose humanPose;
    humanPose.position.x = targetTransform.transform.translation.x;
    humanPose.position.y = targetTransform.transform.translation.y;
    humanPose.position.z = targetTransform.transform.translation.z;
    humanPose.orientation.w = 1;

    if (!computeLookPose(arm_state, lookPose, humanPose, 10, 5, 0.2, 2.0, 2.0))
    {
        return false;
    }

    std::vector<double> lookPose_joint_positions;
    arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_positions);
    arm_mgi_->setJointValueTarget(lookPose_joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!(arm_mgi_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Can't plan for pointing movement");
        return false;
    }

    gripper_mgi_->setNamedTarget("close");
    gripper_mgi_->move();
    arm_mgi_->execute(plan);

    return true;
}

std::vector<geometry_msgs::Pose> HRI_Interface::testPose(rviz_visual_tools::colors color)
{

    tf2::Quaternion q1(tf2::Vector3(0, 1, 0), M_PI_2);
    tf2::Quaternion q2(tf2::Vector3(1, 0, 0), M_PI_2);
    tf2::Quaternion q3(tf2::Vector3(1, 0, 0), M_PI);
    tf2::Quaternion q4(tf2::Vector3(1, 0, 0), -M_PI_2);
    tf2::Quaternion q5(tf2::Vector3(0, 1, 0), M_PI);

    std::vector<geometry_msgs::Quaternion> q_vector;

    geometry_msgs::Quaternion q_msg;

    tf2::convert(q1, q_msg);
    q_vector.push_back(q_msg);

    tf2::Quaternion qresult;

    qresult = q1 * q2;
    qresult.normalize();
    tf2::convert(qresult, q_msg);
    q_vector.push_back(q_msg);

    qresult = q1 * q3;
    qresult.normalize();
    tf2::convert(qresult, q_msg);
    q_vector.push_back(q_msg);

    qresult = q1 * q4;
    qresult.normalize();
    tf2::convert(qresult, q_msg);
    q_vector.push_back(q_msg);

    qresult = q5;
    tf2::convert(qresult, q_msg);
    q_vector.push_back(q_msg);

    arm_mgi_->setStartStateToCurrentState();

    std::vector<geometry_msgs::Pose> range_points;

    double precision = 0.05;

    bool sucess = false;

    for (double z = -0.05; z < 0.35; z += 0.05)
    {
        for (double x = -0.1; x < 0.85; x += precision)
        {
            for (double y = -0.50; y < 0.50; y += precision)
            {
                geometry_msgs::Pose pose;
                for (const auto q_value : q_vector)
                {
                    pose.position.x = x;
                    pose.position.y = y;
                    pose.position.z = z;
                    pose.orientation = q_value;

                    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

                    if (arm_state.setFromIK(arm_jmg_, pose, 0.005, std::bind(&HRI_Interface::isStateValid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)))
                    {
                        sucess = true;
                        range_points.push_back(pose);
                        visual_tools_->publishSphere(pose, color, 0.025);
                        visual_tools_->trigger();
                        visual_tools_->publishRobotState(arm_state, rviz_visual_tools::GREEN);
                        visual_tools_->trigger();
                    }
                }

                if (!sucess)
                {
                    visual_tools_->publishSphere(pose, rviz_visual_tools::RED, 0.015);
                    visual_tools_->trigger();
                }

                sucess = false;
            }
        }
    }

    return range_points;
}

std::vector<geometry_msgs::Pose> HRI_Interface::testPose2(rviz_visual_tools::colors color)
{

    tf2::Quaternion q1(tf2::Vector3(0, 1, 0), M_PI_2);
    tf2::Quaternion q2(tf2::Vector3(1, 0, 0), M_PI_2);
    tf2::Quaternion q3(tf2::Vector3(1, 0, 0), M_PI);
    tf2::Quaternion q4(tf2::Vector3(1, 0, 0), -M_PI_2);
    tf2::Quaternion q5(tf2::Vector3(0, 1, 0), M_PI);
    tf2::Quaternion q6(tf2::Vector3(0, 0, 0), 0);

    std::vector<geometry_msgs::Quaternion> q_vector;

    geometry_msgs::Quaternion q_msg;

    tf2::convert(q1, q_msg);
    q_vector.push_back(q_msg);

    tf2::Quaternion qresult;

    qresult = q1 * q2;
    qresult.normalize();
    tf2::convert(qresult, q_msg);
    q_vector.push_back(q_msg);

    qresult = q1 * q3;
    qresult.normalize();
    tf2::convert(qresult, q_msg);
    q_vector.push_back(q_msg);

    qresult = q1 * q4;
    qresult.normalize();
    tf2::convert(qresult, q_msg);
    q_vector.push_back(q_msg);

    qresult = q5;
    tf2::convert(qresult, q_msg);
    q_vector.push_back(q_msg);

    qresult = q6;
    tf2::convert(qresult, q_msg);
    q_vector.push_back(q_msg);

    arm_mgi_->setStartStateToCurrentState();

    std::vector<geometry_msgs::Pose> range_points;

    double precision = 0.05;

    bool sucess = false;

    moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

    Eigen::Isometry3d pos = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(0));

    // visual_tools_->publishAxisLabeled(pos, arm_mgi_->getLinkNames().at(0), rviz_visual_tools::LARGE);
    // visual_tools_->trigger();

    std::cout << arm_mgi_->getLinkNames().at(0).c_str() << "\n";

    geometry_msgs::Point ref_point;
    ref_point.x = pos.translation().x();
    ref_point.y = pos.translation().y();
    ref_point.z = pos.translation().z();

    geometry_msgs::Point point = ref_point;
    point.x += 0.50;

    visual_tools_->publishSphere(ref_point, rviz_visual_tools::GREEN);
    visual_tools_->publishSphere(point, rviz_visual_tools::GREEN);
    visual_tools_->trigger();

    std::vector<geometry_msgs::Pose> poses = computePointsOnSphere(20, 15, point, ref_point, 0.20, 2 * M_PI, 2 * M_PI);

    for (auto &p : poses)
    {
        visual_tools_->publishSphere(p, rviz_visual_tools::RED, 0.01);
    }
    visual_tools_->trigger();

    for (auto pose : poses)
    {
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        tf2::Quaternion q_inverse(tf2::Vector3(0, 1, 0), M_PI);

        q = q * q_inverse;
        q.normalize();

        tf2::convert(q, pose.orientation);

        if (arm_state.setFromIK(arm_jmg_, pose, 0.005, std::bind(&HRI_Interface::isStateValid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)))
        {
            range_points.push_back(pose);
            visual_tools_->publishSphere(pose, color, 0.025);
            visual_tools_->publishRobotState(arm_state, rviz_visual_tools::GREEN);
            visual_tools_->trigger();
            continue;
        }

        for (const auto q_value : q_vector)
        {
            pose.orientation = q_value;

            if (arm_state.setFromIK(arm_jmg_, pose, 0.005, std::bind(&HRI_Interface::isStateValid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)))
            {
                range_points.push_back(pose);
                visual_tools_->publishSphere(pose, color, 0.025);
                visual_tools_->publishRobotState(arm_state, rviz_visual_tools::GREEN);
                visual_tools_->trigger();
                break;
            }
        }
    }

    double maxRange = -INFINITY;

    for (auto &point : range_points)
    {

        double distance = std::sqrt(
            std::pow(point.position.x - ref_point.x, 2) +
            std::pow(point.position.y - ref_point.y, 2) +
            std::pow(point.position.z - ref_point.z, 2));

        if (distance > maxRange)
            maxRange = distance;
    }

    ROS_INFO("Max Radius From %s: %f ", arm_mgi_->getLinkNames().at(0).c_str(), maxRange);

    return range_points;
}

// HELPER FUNCTIONS

bool HRI_Interface::transformListener(std::string source_frame, std::string target_frame, geometry_msgs::TransformStamped &transform_stamped)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(50.0);

    ros::Time startTime = ros::Time::now();

    while (ros::Time::now() - startTime <= ros::Duration(5.0))
    {
        try
        {
            transform_stamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));

            if (transform_stamped.header.stamp.isValid())
                return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
    }

    return false;
}

void HRI_Interface::updatePlanningScene(std::shared_ptr<planning_scene::PlanningScene> planning_scene)
{
    moveit_msgs::GetPlanningScene::Request req;
    moveit_msgs::GetPlanningScene::Response res;

    if (planningSceneClient.call(req, res))
        planning_scene->setPlanningSceneMsg(res.scene); // apply result to actual PlanningScene
}

void HRI_Interface::prePlanPick()
{
    // Pre-plan close and open gripper motions
    closedJointValues = {gripper_mgi_->getNamedTargetValues("close").begin()->second};
    openedJointValues = {gripper_mgi_->getNamedTargetValues("open").begin()->second};

    moveit::core::RobotState gripper_state(*gripper_mgi_->getCurrentState());

    // Plan Close Trajectory
    gripper_state.setJointGroupActivePositions(gripper_jmg_, openedJointValues);
    gripper_mgi_->setStartState(gripper_state);
    gripper_mgi_->setNamedTarget("close");
    gripper_mgi_->plan(closePlan);

    // Plan Open Trajectory
    gripper_state.setJointGroupActivePositions(gripper_jmg_, closedJointValues);
    gripper_mgi_->setStartState(gripper_state);
    gripper_mgi_->setNamedTarget("open");
    gripper_mgi_->plan(openPlan);
}

std::vector<Eigen::Isometry3d> HRI_Interface::findClosestApproachOption(const std::vector<Eigen::Isometry3d> &approach_options, const Eigen::Isometry3d &linkTransform)
{
    std::vector<Eigen::Isometry3d> sorted_approach_options = approach_options;
    std::sort(sorted_approach_options.begin(), sorted_approach_options.end(), [&linkTransform](const Eigen::Isometry3d &p1, const Eigen::Isometry3d &p2)
              {
        double d1 = (linkTransform.translation() - p1.translation()).norm();
        double d2 = (linkTransform.translation() - p2.translation()).norm();
        return (d1 < d2); });

    return sorted_approach_options;
}

std::vector<geometry_msgs::Pose> HRI_Interface::computePointsOnSphere(int numPoints, int num_layers, geometry_msgs::Point point, geometry_msgs::Point reference_position, double extent, double theta_distance, double phi_distance)
{
    double theta, phi; // Polar and azimuthal angles

    // Convert given point to spherical coordinates

    double x = point.x - reference_position.x;
    double y = point.y - reference_position.y;
    double z = point.z - reference_position.z;

    double r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    theta = acos(z / r);
    phi = atan2(y, x);

    std::vector<geometry_msgs::Pose> poses;

    // Compute points around the given point on the sphere
    double radiusValue;
    int iteration;

    for (radiusValue = r, iteration = 0; radiusValue <= r + extent && iteration < num_layers; radiusValue += extent / num_layers, iteration++)
    {
        for (int i = -numPoints / 2; i < numPoints / 2; ++i)
        {
            double newTheta = theta + (i * theta_distance / numPoints); // Incrementing the polar angle
            for (int j = -numPoints / 2; j < numPoints / 2; ++j)
            {
                double newPhi = phi + (j * phi_distance / numPoints); // Incrementing the azimuthal angle
                geometry_msgs::Pose newPoint;
                newPoint.position.x = radiusValue * sin(newTheta) * cos(newPhi);
                newPoint.position.y = radiusValue * sin(newTheta) * sin(newPhi);
                newPoint.position.z = radiusValue * cos(newTheta);

                double sideAngle = atan2(newPoint.position.y, newPoint.position.x);
                double tiltAngle = -M_PI_2 - atan2(newPoint.position.z, sqrt(pow(newPoint.position.x, 2) + pow(newPoint.position.y, 2)));

                tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
                tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
                tf2::Quaternion qresult = q1 * q2;
                qresult.normalize();

                geometry_msgs::Quaternion q_msg;
                tf2::convert(qresult, q_msg);

                newPoint.position.x += reference_position.x;
                newPoint.position.y += reference_position.y;
                newPoint.position.z += reference_position.z;
                newPoint.orientation = q_msg;

                poses.push_back(newPoint);

                // visual_tools_->publishAxis(newPoint);
            }
        }
        // visual_tools_->trigger();
        // visual_tools_->prompt("");
    }

    std::sort(poses.begin(), poses.end(), [&point](const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
              {
            double d1 = sqrt(pow(p1.position.x - point.x, 2) + pow(p1.position.y - point.y, 2) + pow(p1.position.z - point.z, 2));
            double d2 = sqrt(pow(p2.position.x - point.x, 2) + pow(p2.position.y - point.y, 2) + pow(p2.position.z - point.z, 2));
            return (d1 < d2); });

    return poses;
}

bool HRI_Interface::isStateValid(moveit::core::RobotState *arm_state, const moveit::core::JointModelGroup *group, const double *joint_group_variable_values)
{
    updatePlanningScene(planning_scene_);

    arm_state->setJointGroupPositions(group, joint_group_variable_values);
    arm_state->update();

    // visual_tools_->publishRobotState(*arm_state, rviz_visual_tools::GREEN);
    // visual_tools_->trigger();

    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = false;
    collision_request.distance = false;
    collision_request.cost = false;
    collision_detection::CollisionResult collision_result;
    collision_result.clear();
    planning_scene_->getCurrentStateNonConst() = *arm_state;
    planning_scene_->checkCollision(collision_request, collision_result);

    ROS_INFO_STREAM("COLLISION RESULT is " << ((!collision_result.collision) ? "valid" : "not valid"));

    return !collision_result.collision;
}

bool HRI_Interface::computeLookPose(moveit::core::RobotState &arm_state, geometry_msgs::Pose lookPose, geometry_msgs::Pose focus_position, int numPoints, int num_layers, double extent, double theta, double phi)
{
    if (!arm_state.setFromIK(arm_jmg_, lookPose, 0.1, std::bind(&HRI_Interface::isStateValid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)))
    {
        std::vector<geometry_msgs::Pose> pose_vector = computePointsOnSphere(numPoints, num_layers, lookPose.position, focus_position.position, extent, theta, phi);

        for (const auto &pose : pose_vector)
        {
            visual_tools_->publishAxis(pose);
            visual_tools_->trigger();

            if (arm_state.setFromIK(arm_jmg_, pose, 0.1, std::bind(&HRI_Interface::isStateValid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)))
            {
                return true;
            }
        }

        ROS_ERROR("Pose cannot be achieved.");
        return false;
    }

    return true;
}
