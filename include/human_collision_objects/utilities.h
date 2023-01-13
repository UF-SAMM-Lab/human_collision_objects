#pragma once
#pragma GCC diagnostic ignored "-Wsign-compare"

// ros
#include <ros/ros.h>
#include <ros/package.h>
//messages
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/ApplyPlanningScene.h>
//cnr utilities
#include <subscription_notifier/subscription_notifier.h>
#include <rosparam_utilities/rosparam_utilities.h>
//moveit
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
// standard libs
#include <exception>
#include <fstream>
#include <cmath>
#include <string>
#include <mutex>
#include <filesystem>
#include <iostream>
#include <boost/filesystem.hpp>

void clearObstacles(void);

class humanCollisionObjects {
    protected:
        ros::NodeHandle nh;
    private:
        moveit_msgs::PlanningScene planning_scene;
        moveit::planning_interface::PlanningSceneInterface psi;
        std::vector<Eigen::Vector3f> link_centroids;
        std::vector<Eigen::Vector3f> human_points;
        std::vector<Eigen::Quaternionf> human_quats;
        planning_scene::PlanningScenePtr planning_scene_;
        ros::Publisher planning_scene_diff_publisher;
        std::vector<moveit_msgs::CollisionObject> collisionObjects;
        std::vector<Eigen::Quaternionf> link_quats;
        std::vector<std::vector<float>> pose_sequence;
        void forward_kinematics(std::vector<float> pose_elements, Eigen::Isometry3f transform_to_world);
        void moveCollisionObject(moveit_msgs::CollisionObject &msg, Eigen::Vector3f pos, Eigen::Quaternionf quat);
        moveit_msgs::CollisionObject createCollisionObject(Eigen::Vector3f pos, Eigen::Quaternionf quat, double length, double radius, std::string id);
        std::vector<std::string> co_ids = {"human_torso","human_neck","human_l_upper", "human_l_fore","human_r_upper","human_r_fore"};
        double shoulder_len = 0.35;
        std::vector<double> link_lengths_ = {0.5,0.2,0.35,0.45,0.35,0.45};
        std::vector<double> link_radii_ = {0.17,0.1,0.1,0.07,0.1,0.07};
        std::vector<double> act_lengths;
        std::vector<double> act_radii;
        double dt = 0;
        Eigen::Isometry3f live_transform_to_world;
        Eigen::Isometry3f record_transform_to_world;
        ros::Time timer_start;
        double min_dist_;
        ros::Timer udpate_timer;
        double elapsed_time;
        void update_timer(const ros::TimerEvent& event);
        bool pause_live = false;
        double extra_link_len = 0.0;
    public:
        void setJointLocations(std::vector<Eigen::Vector3f> joints);
        humanCollisionObjects(ros::NodeHandle node_handle, const planning_scene::PlanningScenePtr &planning_scene_ptr, std::vector<double> lengths, std::vector<double> radii, double min_dist, Eigen::Isometry3f transform, double extra_len);
        void removeHumans(void);
        void read_human_task(int task_num, Eigen::Isometry3f transform);
        void updateCollisionObjects(double t);
        void liveCollisionObjectsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void start_live_obs(void);
        void inflate_obs(void);
        void resume_obs(void);
        void start_obs(void);
        bool inflate = true;
        ros::Subscriber sub_live_quats;
        void stop_live_obs(void);
        void resume_live_obs(void);
        void inflate_live_obs(void);
        void pause_obs(void);
        void getLinkData(double t,std::vector<Eigen::Vector3f>& points,std::vector<Eigen::Quaternionf>& quats);
        double end_time=0.0;


};
