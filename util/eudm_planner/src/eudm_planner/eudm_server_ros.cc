/**
 * @file eudm_server_ros.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/eudm_server_ros.h"

namespace planning {

EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, int ego_id)
    : nh_(nh), work_rate_(20.0), ego_id_(ego_id) {
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_, ego_id);
  // 消息缓存
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, double work_rate,
                                     int ego_id)
    : nh_(nh), work_rate_(work_rate), ego_id_(ego_id) {
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

void EudmPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  // 将数据放入缓存
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

void EudmPlannerServer::PublishData() {
  p_visualizer_->PublishDataWithStamp(ros::Time::now());
}

void EudmPlannerServer::Init(const std::string &bp_config_path) {
  bp_manager_.Init(bp_config_path, work_rate_);
  // 订阅仿真调试工具发布的agent的信息
  joy_sub_ = nh_.subscribe("/joy", 10, &EudmPlannerServer::JoyCallback, this);
  nh_.param("use_sim_state", use_sim_state_, true);
  p_visualizer_->Init();
  p_visualizer_->set_use_sim_state(use_sim_state_);
}

// Reports the state of a joysticks axes and buttons.
// Header header           // timestamp in the header is the time the data is received from the joystick
// float32[] axes          // the axes measurements from a joystick
// int32[] buttons         // the buttons measurements from a joystick 
void EudmPlannerServer::JoyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  int msg_id;
  if (std::string("").compare(msg->header.frame_id) == 0) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }
  if (msg_id != ego_id_) return;
  // ~ buttons[2] --> 1 -->  lcl
  // ~ buttons[1] --> 1 -->  lcr
  // ~ buttons[3] --> 1 -->  +1m/s
  // ~ buttons[0] --> 1 -->  -1m/s
  // msg 内容都是0，即仿真没有对agent进行控制
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 &&
      msg->buttons[3] == 0 && msg->buttons[4] == 0 && msg->buttons[5] == 0 &&
      msg->buttons[6] == 0)
    return;
  // 从terminal_server.py的程序来看，控制的选中的车辆
  // 在这里选择对主车的控制指令
  if (msg->buttons[2] == 1) {
    // 向左变道指令
    // 下面的逻辑看不懂，连续选择同一个变道指令会取消变道？？
    if (task_.user_perferred_behavior != -1) {
      task_.user_perferred_behavior = -1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[1] == 1) {
    // 向右变道指令
    if (task_.user_perferred_behavior != 1) {
      task_.user_perferred_behavior = 1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[3] == 1) {
    // 期望车速增加指令
    task_.user_desired_vel = task_.user_desired_vel + 1.0;
  } else if (msg->buttons[0] == 1) {
    // 期望车速降低指令
    task_.user_desired_vel = std::max(task_.user_desired_vel - 1.0, 0.0);
  } else if (msg->buttons[4] == 1) {
    // 切换 是否允许左侧换道的状态
    task_.lc_info.forbid_lane_change_left = !task_.lc_info.forbid_lane_change_left;
  } else if (msg->buttons[5] == 1) {
    // 切换 是否允许右侧换道的状态
    task_.lc_info.forbid_lane_change_right = !task_.lc_info.forbid_lane_change_right;
  } else if (msg->buttons[6] == 1) {
    // 切换自动驾驶状态
    task_.is_under_ctrl = !task_.is_under_ctrl;
  }
}

void EudmPlannerServer::Start() {
  // 开启线程，决策开始计算
  std::thread(&EudmPlannerServer::MainThread, this).detach();
  task_.is_under_ctrl = true;
}

void EudmPlannerServer::MainThread() {
  using namespace std::chrono;
  // 决策计算频频控制
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  // 频率 20 Hz
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
  // loop，频率控制
  while (true) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

void EudmPlannerServer::PlanCycleCallback() {
  if (p_input_smm_buff_ == nullptr) return;

  bool has_updated_map = false;
  // 获取最新的 语义地图
  while (p_input_smm_buff_->try_dequeue(smm_)) {
    has_updated_map = true;
  }
  // 如果语义地图没有更新，则不进行新的规划
  if (!has_updated_map) return;
  // 使用 语义地图 指针进行后续操作
  auto map_ptr =
      std::make_shared<semantic_map_manager::SemanticMapManager>(smm_);
  // 频率 20 Hz， replan_duration = 0.05s
  decimal_t replan_duration = 1.0 / work_rate_;
  // floor向下取整，圆整 EUDM Planner的时间戳，为什么呢？
  double stamp =
      std::floor(smm_.time_stamp() / replan_duration) * replan_duration;
  // 进行决策，传入 时间戳，语义地图，HMI指令
  if (bp_manager_.Run(stamp, map_ptr, task_) == kSuccess) {
    common::SemanticBehavior behavior;
    // 决策成功
    bp_manager_.ConstructBehavior(&behavior);
    smm_.set_ego_behavior(behavior);
  }

  if (has_callback_binded_) {
    // 调用 PushSemanticMap，为什么用这么复杂，直接调用不行？
    // 将更新了 behavior 结果的 语义地图 发送给 SSC Planner
    private_callback_fn_(smm_);
  }

  PublishData();
}

void EudmPlannerServer::BindBehaviorUpdateCallback(
    std::function<int(const SemanticMapManager &)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

void EudmPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  task_.user_desired_vel = desired_vel;
}

decimal_t EudmPlannerServer::user_desired_velocity() const {
  return task_.user_desired_vel;
}

}  // namespace planning
