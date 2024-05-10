#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_

#include <string>
#include <unordered_map>
#include <vector>

namespace planning {
namespace eudm {

struct LaneChangeInfo {
  bool forbid_lane_change_left = false;
  bool forbid_lane_change_right = false;
  bool lane_change_left_unsafe_by_occu = false;
  bool lane_change_right_unsafe_by_occu = false;
  bool left_solid_lane = false;
  bool right_solid_lane = false;
  bool recommend_lc_left = false;
  bool recommend_lc_right = false;
};

struct Task {
  // 是否自动驾驶
  bool is_under_ctrl = false;
  // 用户期望速度
  double user_desired_vel;
  // 用户期望的换道行为
  // -1:向左变道， 0:直行， 1:向右变道
  int user_perferred_behavior = 0;
  LaneChangeInfo lc_info;
};

}  // namespace eudm
}  // namespace planning

#endif
