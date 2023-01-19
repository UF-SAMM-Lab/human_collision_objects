#include <human_collision_objects/utilities.h>

bool inflate = false;

void human_inflate_callback(const std_msgs::Bool::ConstPtr& msg){
  inflate =  msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"live_human");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Subscriber sub_inflate = nh.subscribe("human_inflate", 1, human_inflate_callback); //get status of moveit trajectory execution


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


    Eigen::Isometry3f transform_to_world = Eigen::Isometry3f::Identity();
    std::vector<double> workcell_transform = {0,0,0,0,0,0,1};
    std::string wc_tf;
    nh.getParam("/live_human/workcell_transform",wc_tf);
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
      human_link_radii2.push_back(human_link_radii[i]+0.07);
    }
    human_link_radii2[0]+= 0.05;
    human_link_radii2[1]+= 0.1;
    std::cout<<std::endl;

    humanCollisionObjects co_human(nh,scene,human_link_lengths2,human_link_radii2, 0.00,Eigen::Isometry3f::Identity(),0.1);
    co_human.start_live_obs();
    bool inflated = false;
    while (ros::ok()) { 
      if ((inflate)&&(!inflated)) {
        co_human.inflate_live_obs();
        inflated = true;
      } else if ((!inflate)&&(inflated)) {
        co_human.resume_live_obs();
        inflated = false;
      }
      
      ros::Duration(0.01).sleep();
    }
    ROS_INFO_STREAM("made it!");
    return 0;
}
