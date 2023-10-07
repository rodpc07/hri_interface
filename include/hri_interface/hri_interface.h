#ifndef HRI_INTERFACE_H
#define HRI_INTERFACE_H

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
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <memory>
#include <functional>
#include <algorithm>
#include <Eigen/Geometry>

class HRI_Interface
{
public:
    HRI_Interface(ros::NodeHandle n,
                  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_mgi,
                  const moveit::core::JointModelGroup *arm_jmg,
                  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_mgi,
                  const moveit::core::JointModelGroup *gripper_jmg,
                  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
                  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools);

    void exagerateTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan, Eigen::Vector3d xyz);
    bool waving(std::string target_frame);
    bool comeClose(std::string target_frame);
    bool goAway(std::string target_frame);
    /* bool mode (true = screw, false = unscrew) */
    bool screw_unscrew(bool mode, geometry_msgs::Pose input_pose);
    void signalPick();
    bool signalRotate(std::string object_id, Eigen::Vector3d rotationInfo);
    bool pointToPoint(geometry_msgs::Point point);
    bool pointToObject(std::string object_id);
    bool pointToObjectSide(std::string object_id, Eigen::Vector3d sideInfo);
    bool pointToHuman(std::string target_frame);

    std::vector<geometry_msgs::Pose> testPose(rviz_visual_tools::colors color);
    std::vector<geometry_msgs::Pose> testPose2(rviz_visual_tools::colors color);

private:
    ros::NodeHandle n_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_mgi_;
    const moveit::core::JointModelGroup *arm_jmg_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_mgi_;
    const moveit::core::JointModelGroup *gripper_jmg_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;

    ros::ServiceClient planningSceneClient;

    moveit::planning_interface::MoveGroupInterface::Plan openPlan;
    moveit::planning_interface::MoveGroupInterface::Plan closePlan;
    std::vector<double> closedJointValues;
    std::vector<double> openedJointValues;

    bool transformListener(std::string source_frame, std::string target_frame,
                           geometry_msgs::TransformStamped &transform_stamped);
    void updatePlanningScene(std::shared_ptr<planning_scene::PlanningScene> planning_scene);
    void prePlanPick();
    std::vector<Eigen::Isometry3d> findClosestApproachOption(const std::vector<Eigen::Isometry3d> &approach_options, const Eigen::Isometry3d &linkTransform);
    std::vector<geometry_msgs::Pose> computePointsOnSphere(int numPoints, int num_layers, geometry_msgs::Point point, geometry_msgs::Point reference_position, double extent, double theta_distance, double phi_distance);
    bool isStateValid(moveit::core::RobotState *arm_state, const moveit::core::JointModelGroup *group, const double *joint_group_variable_values);
    bool computeLookPose(moveit::core::RobotState &arm_state, geometry_msgs::Pose lookPose, geometry_msgs::Pose focus_position, int numPoints, int num_layers, double extent, double theta, double phi);
};

#endif