/**
 * @file test_ssc_with_eudm.cc
 * @author HKUST Aerial Robotics Group
 * @brief test ssc planner with eudm behavior planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "eudm_planner/eudm_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"

DECLARE_BACKWARD;
double ssc_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

planning::SscPlannerServer* p_ssc_server_{nullptr};
planning::EudmPlannerServer* p_bp_server_{nullptr};

int BehaviorUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ssc_server_) p_ssc_server_->PushSemanticMap(smm);
  return 0;
}

int SemanticMapUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");
  
  // 从ros参数空间获取配置参数
  // 下面的配置可以从 launch/test_ssc_with_mpdm_ros.launch得到
  int ego_id;
  if (!nh.getParam("ego_id", ego_id)) {  // 0
    ROS_ERROR("Failed to get param %d", ego_id);
    assert(false);
  }
  // core/playgrounds/highway_v1.0/agent_config.json
  std::string agent_config_path;
  if (!nh.getParam("agent_config_path", agent_config_path)) {
    ROS_ERROR("Failed to get param agent_config_path %s",
              agent_config_path.c_str());
    assert(false);
  }
  // util/eudm_planner/config/eudm_config.pb.txt
  std::string bp_config_path;
  if (!nh.getParam("bp_config_path", bp_config_path)) {
    ROS_ERROR("Failed to get param bp_config_path %s", bp_config_path.c_str());
    assert(false);
  }
  // util/ssc_planner/config/ssc_config.pb.txt
  std::string ssc_config_path;
  if (!nh.getParam("ssc_config_path", ssc_config_path)) {
    ROS_ERROR("Failed to get param ssc_config_path %s",
              ssc_config_path.c_str());
    assert(false);
  }
  
  // 语义地图进行载参
  semantic_map_manager::SemanticMapManager semantic_map_manager(
      ego_id, agent_config_path);
  semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager);
  // 绑定回调，将更新的 语义地图 push到 EUDM Planner
  smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback);

  double desired_vel;
  nh.param("desired_vel", desired_vel, 6.0);
  // Declare bp
  p_bp_server_ = new planning::EudmPlannerServer(nh, bp_work_rate, ego_id);
  p_bp_server_->set_user_desired_velocity(desired_vel);
  // 绑定回调函数，当 EUDM 计算出 behavior 后，通过语义地图更新，传递给 ssc planner
  p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);

  p_ssc_server_ =
      new planning::SscPlannerServer(nh, ssc_planner_work_rate, ego_id);
  // 传入 EUDM 和 SSC Planner的参数
  p_bp_server_->Init(bp_config_path);
  p_ssc_server_->Init(ssc_config_path);
  smm_ros_adapter.Init();
  
  // 决策和轨迹规划分别是两个独立的线程，开启线程
  p_bp_server_->Start();
  p_ssc_server_->Start();

  // TicToc timer;
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
