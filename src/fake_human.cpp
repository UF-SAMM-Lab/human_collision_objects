#include <fake_human/utilities.h>

int main(int argc, char** argv) {
    ros::init(argc,argv,"fake_human");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher pub_keypoints = nh.advertise<std_msgs::Float32MultiArray>( "/skeleton", 0,false);
    ros::Publisher pub_skel_pts = nh.advertise<std_msgs::Float32MultiArray>( "/skeleton_points", 0,false);
    ros::Publisher pub_skel_quats = nh.advertise<std_msgs::Float32MultiArray>( "/skeleton_quats", 0,false);
    ros::Publisher poses_pub = nh.advertise<geometry_msgs::PoseArray>( "/poses", 0,false);

    //do a test with no obsctacles:
    std::string plan_group = "manipulator";
    if (!nh.getParam("/plan_group",plan_group))
    {
      ROS_WARN("plan_group is not defined");
    }
    std::string ctrl_ns = "manipulator";
    if (!nh.getParam("/ctrl_ns",ctrl_ns))
    {
      ROS_WARN("ctrl_ns is not defined");
    }

    moveit::planning_interface::MoveGroupInterface move_group(plan_group);
    const robot_state::JointModelGroup* joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(plan_group);
    robot_model_loader::RobotModelLoaderPtr robot_model_loader = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
    const robot_model::RobotModelPtr& model = robot_model_loader->getModel();
    std::shared_ptr<planning_scene::PlanningScene> scene(new planning_scene::PlanningScene(model));
    planning_scene_monitor::PlanningSceneMonitorPtr monitor;
    monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    if(monitor->getPlanningScene())
    {
      monitor->startSceneMonitor("/move_group/monitored_planning_scene");
      monitor->startWorldGeometryMonitor();
      monitor->startStateMonitor();
    }
    else
    {
      exit(EXIT_FAILURE);
    }

    std::string resp = "y";

    int human_task_num;
    nh.getParam("/human_task_num", human_task_num);

    std::cout<<"human_task_num:"<<human_task_num<<std::endl;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success;
    Eigen::Isometry3f transform_to_world = Eigen::Isometry3f::Identity();
    std::vector<double> workcell_transform = {0,0,0,0,0,0,1};
    std::string wc_tf;
    nh.getParam("/sim_human/workcell_transform",wc_tf);
    std::stringstream ss(wc_tf);
    std::string s;
    int i=0;
    while (std::getline(ss, s, ',')) {
        workcell_transform[i] = std::stof(s);
        i++;
    }
    std::cout<<"transform:";
    for (int i=0;i<7;i++) std::cout<<workcell_transform[i]<<",";
    std::cout<<std::endl;
    transform_to_world.linear() = Eigen::Matrix3f(Eigen::Quaternionf(workcell_transform[3],workcell_transform[4],workcell_transform[5],workcell_transform[6]));
    transform_to_world.translation() = Eigen::Vector3f(workcell_transform[0],workcell_transform[1],workcell_transform[2]);
    std::vector<double> human_link_lengths; 
    std::vector<double> human_link_radii; 
    nh.getParam("/human_link_lengths", human_link_lengths);
    nh.getParam("/human_link_radii", human_link_radii);
    std::vector<double> human_link_lengths2; 
    std::vector<double> human_link_radii2; 
    for (int i=0;i<human_link_radii.size();i++) {
      std::cout<<human_link_lengths[i]<<",";
      if (i==2) continue;
      human_link_lengths2.push_back(human_link_lengths[i]);
      human_link_radii2.push_back(human_link_radii[i]);
    }
    std::cout<<std::endl;

    humanCollisionObjects co_human(nh,scene,human_link_lengths2,human_link_radii2, 0.2,Eigen::Isometry3f::Identity());
    co_human.read_human_task(human_task_num,transform_to_world);
    double t=0.0;
    ros::Time start_time = ros::Time::now();
    std::vector<Eigen::Vector3f> human_points;
    std::vector<Eigen::Quaternionf> human_quats;
    std_msgs::Float32MultiArray mp_points;
    //                           0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28
    std::vector<int> hp_to_mp = {2,2,2,2,2,2,2,2,2,2, 2, 3, 6, 4, 7, 5, 8, 5, 8, 5, 8, 5, 8, 0, 0, 0, 0, 0, 0};
    co_human.start_obs();
    while (t<co_human.end_time) {
      co_human.getLinkData(t,human_points,human_quats);  
      geometry_msgs::PoseArray poses;
      geometry_msgs::Pose pose;
      mp_points.data.clear();
      for (int i=0;i<human_points.size();i++) {
        pose.position.x = human_points[i][0];
        pose.position.y = human_points[i][1];
        pose.position.z = human_points[i][2];
        poses.poses.push_back(pose);
        for (int j=0;j<3;j++) mp_points.data.push_back(human_points[i][j]);
      }
      poses.header.frame_id="world";
      poses.header.stamp=ros::Time::now();
      poses_pub.publish(poses);
      pub_keypoints.publish(mp_points);
      std::vector<Eigen::Vector3f> coco_joint_locations(18);
      for (int i=0;i<18;i++) coco_joint_locations[i] = Eigen::Vector3f::Zero();
      coco_joint_locations[0] = human_points[2];
      coco_joint_locations[1] = human_points[1];
      coco_joint_locations[2] = human_points[3];
      coco_joint_locations[3] = human_points[4];
      coco_joint_locations[4] = human_points[5];
      coco_joint_locations[5] = human_points[6];
      coco_joint_locations[6] = human_points[7];
      coco_joint_locations[7] = human_points[8];
      coco_joint_locations[8] = human_points[0];
      coco_joint_locations[11] = human_points[0];
      std_msgs::Float32MultiArray coco_msg;
      for (int i=0;i<coco_joint_locations.size();i++) {
        for (int j=0;j<3;j++) coco_msg.data.push_back(coco_joint_locations[i][j]);
      }
      pub_skel_pts.publish(coco_msg);
      std_msgs::Float32MultiArray quat_msg;
      for (int i=0;i<human_quats.size();i++) {
        quat_msg.data.push_back(human_quats[i].w());
        quat_msg.data.push_back(human_quats[i].x());
        quat_msg.data.push_back(human_quats[i].y());
        quat_msg.data.push_back(human_quats[i].z());
      }
      pub_skel_quats.publish(quat_msg);
      ros::Duration(0.03).sleep();
      t = (ros::Time::now()-start_time).toSec();
    }
    ROS_INFO_STREAM("made it!");
    return 0;
}
