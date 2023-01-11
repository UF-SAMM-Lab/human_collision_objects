#include <human_collision_objects/utilities.h>

void clearObstacles(void) {
    moveit::planning_interface::PlanningSceneInterface current_scene;
    std::vector<std::string> obstacles;
    obstacles = current_scene.getKnownObjectNames();
    std::cout<<"num obstacles = "<<obstacles.size()<<std::endl;
    for (int i=0;i<obstacles.size();i++) {
        if (obstacles[i]=="table") {
            obstacles.erase(obstacles.begin()+i);
        }
    }
    while (obstacles.size()>0) {
        current_scene.removeCollisionObjects(obstacles);
        obstacles = current_scene.getKnownObjectNames();
        for (int i=0;i<obstacles.size();i++) {
            std::cout<<obstacles[i]<<std::endl;
            if (obstacles[i]=="table") {
                obstacles.erase(obstacles.begin()+i);
            }
        }
        std::cout<<"num obstacles = "<<obstacles.size()<<std::endl;
    }
}

humanCollisionObjects::humanCollisionObjects(ros::NodeHandle node_handle, const planning_scene::PlanningScenePtr &planning_scene_ptr, std::vector<double> lengths, std::vector<double> radii, double min_dist,Eigen::Isometry3f transform):planning_scene_(planning_scene_ptr),nh(node_handle),link_lengths_(lengths),link_radii_(radii), min_dist_(min_dist),live_transform_to_world(transform)  {
  
  planning_scene.is_diff = true;
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  ros::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();


  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
        
  ros::WallDuration(1.0).sleep();
  timer_start = ros::Time::now();
  act_lengths = link_lengths_;
  act_radii = link_radii_;
}

void humanCollisionObjects::update_timer(const ros::TimerEvent& event) {
  elapsed_time += (event.current_real-timer_start).toSec();
  updateCollisionObjects(elapsed_time);
  timer_start = ros::Time::now();
}

void humanCollisionObjects::start_live_obs(void) {
  sub_live_quats = nh.subscribe<std_msgs::Float32MultiArray>("/skeleton_quats",1,&humanCollisionObjects::liveCollisionObjectsCallback,this);
}

void humanCollisionObjects::stop_live_obs(void) {
  sub_live_quats.shutdown();
}

void humanCollisionObjects::liveCollisionObjectsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  // if (pause_live) return;
  forward_kinematics(msg->data,live_transform_to_world);
  // ROS_INFO_STREAM("link_quats"<<link_quats.size());
  if (planning_scene.world.collision_objects.empty()) {
      collisionObjects.clear();
      for (int i=0;i<link_quats.size();i++) {
        collisionObjects.push_back(createCollisionObject(link_centroids[i],link_quats[i],act_lengths[i],act_radii[i],co_ids[i]));
      }
      // psi.addCollisionObjects(collisionObjects);
    } else {
      for (int i=0;i<link_quats.size();i++) {
          moveCollisionObject(collisionObjects[i],link_centroids[i],link_quats[i]);
      }
      // psi.applyCollisionObjects(collisionObjects);

    }
    planning_scene.world.collision_objects = collisionObjects;

    planning_scene_diff_publisher.publish(planning_scene);
}
void humanCollisionObjects::resume_live_obs(void) {
  inflate = false;
  removeHumans();
  for (int i=0;i<link_radii_.size();i++) act_radii[i] = link_radii_[i];
  // pause_live = false;
}

void humanCollisionObjects::inflate_live_obs(void) {
  // pause_live = true;
  removeHumans();
  for (int i=0;i<link_radii_.size();i++) act_radii[i] = link_radii_[i]+min_dist_;
}

void humanCollisionObjects::start_obs(void) {
  elapsed_time = 0.0;
  resume_obs();
}
void humanCollisionObjects::resume_obs(void) {
  inflate = false;
  removeHumans();
  for (int i=0;i<link_radii_.size();i++) act_radii[i] = link_radii_[i];
  updateCollisionObjects(elapsed_time);
  timer_start = ros::Time::now();
  udpate_timer = nh.createTimer(ros::Duration(1.0/10.0), &humanCollisionObjects::update_timer, this);
}

void humanCollisionObjects::inflate_obs(void) {
  udpate_timer.stop();
  removeHumans();
  for (int i=0;i<link_radii_.size();i++) act_radii[i] = link_radii_[i]+min_dist_;
  updateCollisionObjects(elapsed_time);
}

void humanCollisionObjects::pause_obs(void) {
  udpate_timer.stop();
}

void humanCollisionObjects::updateCollisionObjects(double t) {
  // std::cout<<"t:"<<t<<","<<dt<<","<<int(t/dt)<<std::endl;
  forward_kinematics(pose_sequence[std::min(int(t/dt),int(pose_sequence.size())-1)],record_transform_to_world);
  // ROS_INFO_STREAM("link_quats"<<link_quats.size());
  if (planning_scene.world.collision_objects.empty()) {
      collisionObjects.clear();
      for (int i=0;i<link_quats.size();i++) {
        collisionObjects.push_back(createCollisionObject(link_centroids[i],link_quats[i],act_lengths[i],act_radii[i],co_ids[i]));
      }
      // psi.addCollisionObjects(collisionObjects);
    } else {
      for (int i=0;i<link_quats.size();i++) {
          moveCollisionObject(collisionObjects[i],link_centroids[i],link_quats[i]);
      }
      // psi.applyCollisionObjects(collisionObjects);

    }
    planning_scene.world.collision_objects = collisionObjects;

    planning_scene_diff_publisher.publish(planning_scene);
}

void humanCollisionObjects::getLinkData(double t,std::vector<Eigen::Vector3f>& points,std::vector<Eigen::Quaternionf>& quats) {
  std::cout<<"t:"<<t<<","<<dt<<","<<int(t/dt)<<std::endl;
  forward_kinematics(pose_sequence[std::min(int(t/dt),int(pose_sequence.size())-1)],record_transform_to_world);
  points = human_points;
  quats = human_quats;

}

void humanCollisionObjects::removeHumans(void) {
  std::cout<<"removing human"<<std::endl;
  for (int i=0;i<collisionObjects.size();i++) {
      collisionObjects[i].operation = collisionObjects[i].REMOVE;
      planning_scene.world.collision_objects[i] = collisionObjects[i];
  }
      // psi.applyCollisionObjects(collisionObjects);
  //publish the updated planning scene
  planning_scene_diff_publisher.publish(planning_scene);
  while (!planning_scene.world.collision_objects.empty()) {
      collisionObjects.pop_back();
      planning_scene.world.collision_objects.pop_back();
  }
}

moveit_msgs::CollisionObject humanCollisionObjects::createCollisionObject(Eigen::Vector3f pos, Eigen::Quaternionf quat, double length, double radius, std::string id)
{
    moveit_msgs::CollisionObject collisionObj;
    collisionObj.header.frame_id = "world";
    collisionObj.id = id;
    shape_msgs::SolidPrimitive co_shape;
    co_shape.type = co_shape.CYLINDER;
    co_shape.dimensions.resize(2);
    co_shape.dimensions[0] = length;
    co_shape.dimensions[1] = radius;

    geometry_msgs::Pose co_pose;
    co_pose.position.x = pos[0];
    co_pose.position.y = pos[1];
    co_pose.position.z = pos[2];
    co_pose.orientation.x = quat.x();
    co_pose.orientation.y = quat.y();
    co_pose.orientation.z = quat.z();
    co_pose.orientation.w = quat.w();  
    collisionObj.pose=co_pose;
    collisionObj.primitives.push_back(co_shape);
    co_pose.position.x = 0;
    co_pose.position.y = 0;
    co_pose.position.z = 0;
    co_pose.orientation.x = 0;
    co_pose.orientation.y = 0;
    co_pose.orientation.z = 0;
    co_pose.orientation.w = 1;
    collisionObj.primitive_poses.push_back(co_pose);
    collisionObj.operation = collisionObj.ADD;
    // collisionObj.pose.orientation.w = 1.0;

    return collisionObj;
}


void humanCollisionObjects::moveCollisionObject(moveit_msgs::CollisionObject &msg, Eigen::Vector3f pos, Eigen::Quaternionf quat)
{   
    if (pos.hasNaN() || quat.coeffs().hasNaN()) return;
    geometry_msgs::Pose co_pose;
    msg.pose.position.x = pos[0];
    msg.pose.position.y = pos[1];
    msg.pose.position.z = pos[2];

    // std::cout<<quat.w()<<","<<quat.x()<<","<<quat.y()<<","<<quat.z()<<std::endl;
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    msg.primitives.clear();
    msg.operation = msg.MOVE;
}

void humanCollisionObjects::forward_kinematics(std::vector<float> pose_elements, Eigen::Isometry3f transform_to_world) {
    // ROS_INFO_STREAM("forward kinematics "<<pose_elements.size());
    // if (prediction.data[31]>0) {
    //     ROS_INFO_STREAM("error");
    //     ROS_INFO_STREAM(prediction);
    // }
    Eigen::Vector3f pelvis_loc = {pose_elements[1],pose_elements[2],pose_elements[3]};
    pelvis_loc = transform_to_world*pelvis_loc;
    Eigen::Quaternionf quat_to_world(transform_to_world.rotation());

    Eigen::Quaternionf z_axis_quat(0,0,0,1);
    Eigen::Quaternionf x_axis_quat(0.707,0,-0.707,0);
    std::vector<Eigen::Quaternionf> quats;
    Eigen::Quaternionf q;
    for (int i=0;i<7;i++){
        q = quat_to_world*Eigen::Quaternionf(pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6],pose_elements[i*4+7]);
        double q_norm = q.squaredNorm();
        if (q_norm<0.1) {
          q.w() = 1;
          q.x() = 0;
          q.y() = 0;
          q.z() = 0;
        } else {
          q.w() = q.w()/q_norm;
          q.x() = q.x()/q_norm;
          q.y() = q.y()/q_norm;
          q.z() = q.z()/q_norm;
        }
        // std::cout<<"quat "<<i<<":<"<<pose_elements[i*4+4]<<","<<pose_elements[i*4+5]<<","<<pose_elements[i*4+6]<<","<<pose_elements[i*4+7]<<">"<<std::endl;
        quats.push_back(q);
        // ROS_INFO_STREAM("quat "<<q.w()<<" "<<q.vec().transpose());
    }
    human_quats = quats;
    link_centroids.clear();
    link_quats.clear();
    human_points.clear();
    Eigen::Quaternionf z_spine = quats[0]*z_axis_quat*quats[0].inverse();
    Eigen::Quaternionf x_spine = quats[0]*x_axis_quat*quats[0].inverse();
    // link_quats.push_back(z_spine);
    Eigen::Vector3f spine_top = pelvis_loc+link_lengths_[0]*z_spine.vec();
    human_points.push_back(pelvis_loc);
    human_points.push_back(spine_top);
    link_centroids.push_back(pelvis_loc+0.5*link_lengths_[0]*z_spine.vec());
    // ROS_INFO_STREAM("spine top "<<spine_top.transpose());
    Eigen::Quaternionf z_neck = quats[1]*z_axis_quat*quats[1].inverse();
    Eigen::Quaternionf x_neck = quats[1]*x_axis_quat*quats[1].inverse();
    // link_quats.push_back(z_neck);
    Eigen::Vector3f head = spine_top+link_lengths_[1]*z_neck.vec();
    human_points.push_back(head);
    link_centroids.push_back(spine_top+0.5*link_lengths_[1]*z_neck.vec());
    // ROS_INFO_STREAM("head top "<<head.transpose());
    Eigen::Quaternionf z_shoulders = quats[2]*z_axis_quat*quats[2].inverse();
    Eigen::Vector3f l_shoulder = spine_top-0.5*shoulder_len*z_shoulders.vec();
    human_points.push_back(l_shoulder);
    // ROS_INFO_STREAM("l_shoulder "<<l_shoulder.transpose());
    Eigen::Quaternionf z_e1 = quats[3]*z_axis_quat*quats[3].inverse();
    Eigen::Quaternionf x_e1 = quats[3]*x_axis_quat*quats[3].inverse();
    // link_quats.push_back(x_e1);
    Eigen::Vector3f e1 = l_shoulder+link_lengths_[2]*z_e1.vec();
    human_points.push_back(e1);
    link_centroids.push_back(l_shoulder+0.5*link_lengths_[2]*z_e1.vec());
    Eigen::Quaternionf z_w1 = quats[4]*z_axis_quat*quats[4].inverse();
    Eigen::Quaternionf x_w1 = quats[4]*x_axis_quat*quats[4].inverse();
    // link_quats.push_back(x_w1);
    Eigen::Vector3f w1 = e1+(link_lengths_[3]+0.1)*z_w1.vec();
    human_points.push_back(w1);
    link_centroids.push_back(e1+0.5*(link_lengths_[3])*z_w1.vec());
    Eigen::Vector3f r_shoulder = spine_top+0.5*shoulder_len*z_shoulders.vec();
    human_points.push_back(r_shoulder);
    // ROS_INFO_STREAM("r_shoulder "<<r_shoulder.transpose());
    Eigen::Quaternionf z_e2 = quats[5]*z_axis_quat*quats[5].inverse();
    Eigen::Quaternionf x_e2 = quats[5]*x_axis_quat*quats[5].inverse();
    // link_quats.push_back(x_e2);
    Eigen::Vector3f e2 = r_shoulder+link_lengths_[4]*z_e2.vec();
    human_points.push_back(e2);
    link_centroids.push_back(r_shoulder+0.5*link_lengths_[4]*z_e2.vec());
    Eigen::Quaternionf z_w2 = quats[6]*z_axis_quat*quats[6].inverse();
    Eigen::Quaternionf x_w2 = quats[6]*x_axis_quat*quats[6].inverse();
    // link_quats.push_back(x_w2);
    Eigen::Vector3f w2 = e2+(link_lengths_[5]+0.1)*z_w2.vec();
    human_points.push_back(w2);
    link_centroids.push_back(e2+0.5*(link_lengths_[5])*z_w2.vec());

    link_quats.push_back(quats[0]);
    link_quats.push_back(quats[1]);
    link_quats.push_back(quats[3]);
    link_quats.push_back(quats[4]);
    link_quats.push_back(quats[5]);
    link_quats.push_back(quats[6]);

    
}

void humanCollisionObjects::read_human_task(int task_num, Eigen::Isometry3f transform) {
    record_transform_to_world = transform;
    boost::filesystem::path pkg_path(ros::package::getPath("human_collision_objects"));
    boost::filesystem::path data_path("data");
    boost::filesystem::path file("task_"+std::to_string(task_num) + "_t_means.csv");
    std::string file_name = boost::filesystem::path(pkg_path / data_path / file).string();
    std::ifstream myfile (file_name); 
    std::string line;
    std::string prev_line;
    std::vector<std::string> all_lines;
    double prev_time = 0;
    std::stringstream ss;
    dt = 0;

    if (myfile.is_open()) { 
        // ROS_INFO_STREAM("reading file ");
        while (std::getline(myfile,line)) {
            all_lines.push_back(line);
        }

        ROS_INFO_STREAM("all lines "<<all_lines.size());
        myfile.close();
        pose_sequence.clear();
        for (int i=0;i<all_lines.size();i++) {
          std::vector<float> pose_elements;
          ss=std::stringstream(all_lines[i]);
          std::string substr;
          while (std::getline(ss,substr,',')) {
              pose_elements.push_back(std::stof(substr.c_str()));
          }   

          if ((dt==0)&&(pose_elements[0]>prev_time)) dt = pose_elements[0]-prev_time;

          pose_sequence.push_back(pose_elements);

        }
        end_time = pose_sequence.back()[0];

    }

}