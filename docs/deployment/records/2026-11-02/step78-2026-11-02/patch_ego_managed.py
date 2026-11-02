from pathlib import Path
root=Path('/root/autodl-tmp/uav/ego-planner')
pm=root/'src/planner/plan_manage'

def edit(path,old,new,count=1):
    s=path.read_text()
    assert s.count(old)>=count,(str(path),old[:70])
    path.write_text(s.replace(old,new,count))

(pm/'msg/GoalCommand.msg').write_text('''std_msgs/Header header
string planner_session_id
string mission_id
uint64 goal_id
string command
geometry_msgs/Point goal
''')
(pm/'msg/GoalStatus.msg').write_text('''std_msgs/Header header
string planner_session_id
string mission_id
uint64 goal_id
string status
string reason
bool map_ready
''')
with (pm/'msg/Bspline.msg').open('a') as f:
    f.write('\n# Identified managed planning; empty in upstream legacy mode.\nstring planner_session_id\nstring mission_id\nuint64 goal_id\nbool emergency_stop\n')
edit(pm/'CMakeLists.txt','  Bspline.msg','  GoalCommand.msg\n  GoalStatus.msg\n  Bspline.msg')
h=pm/'include/plan_manage/ego_replan_fsm.h'
edit(h,'#include <ego_planner/Bspline.h>','#include <ego_planner/Bspline.h>\n#include <ego_planner/GoalCommand.h>\n#include <ego_planner/GoalStatus.h>\n#include <set>\n#include <map>')
edit(h,'    /* ROS utils */','''    bool managed_ = false;
    bool odom_velocity_in_body_ = false;
    std::string managed_session_, managed_mission_;
    uint64_t managed_goal_ = 0;
    std::map<std::string, uint64_t> highest_goals_;
    std::set<std::string> retired_missions_;
    ros::WallTime odom_wall_, planning_wall_;
    ros::WallTimer managed_watchdog_;
    ros::Subscriber managed_command_sub_;
    ros::Publisher managed_status_pub_;
    bool managedMapReady();
    void managedCommand(const ego_planner::GoalCommandConstPtr &msg);
    void managedWatchdog(const ros::WallTimerEvent &event);
    void managedStatus(const std::string &status, const std::string &reason = "");
    void managedStop(const std::string &status, const std::string &reason);

    /* ROS utils */''')
c=pm/'src/ego_replan_fsm.cpp'
edit(c,'#include <plan_manage/ego_replan_fsm.h>','#include <plan_manage/ego_replan_fsm.h>\n#include <boost/uuid/uuid_generators.hpp>\n#include <boost/uuid/uuid_io.hpp>')
edit(c,'    have_odom_ = false;','''    have_odom_ = false;
    trigger_ = false;
    have_new_target_ = false;
    flag_escape_emergency_ = true;
    nh.param("fsm/managed_control", managed_, false);
    nh.param("fsm/odom_velocity_in_body", odom_velocity_in_body_, false);
    managed_session_ = boost::uuids::to_string(boost::uuids::random_generator()());''')
edit(c,'    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)','''    if (managed_)
    {
      managed_command_sub_ = nh.subscribe("/uav/ego/command", 10, &EGOReplanFSM::managedCommand, this);
      managed_status_pub_ = nh.advertise<ego_planner::GoalStatus>("/uav/ego/status", 10, true);
      managed_watchdog_ = nh.createWallTimer(ros::WallDuration(0.1), &EGOReplanFSM::managedWatchdog, this);
    }
    else if (target_type_ == TARGET_TYPE::MANUAL_TARGET)''')
edit(c,'    if (msg->poses[0].pose.position.z < -0.1)','    if (msg->poses.empty() || msg->poses[0].pose.position.z < -0.1)')
edit(c,'msg->poses[0].pose.position.y, 1.0;','msg->poses[0].pose.position.y, msg->poses[0].pose.position.z;')
edit(c,'    odom_vel_(0) = msg->twist.twist.linear.x;','    odom_vel_(0) = msg->twist.twist.linear.x;')
edit(c,'    have_odom_ = true;','''    if (odom_velocity_in_body_)
      odom_vel_ = odom_orient_.normalized().toRotationMatrix() * odom_vel_;
    have_odom_ = odom_orient_.norm() > 0.99 && odom_orient_.norm() < 1.01 &&
      odom_pos_.allFinite() && odom_vel_.allFinite();
    if (managed_ && (msg->header.frame_id != "map" || msg->child_frame_id != "base_link"))
      have_odom_ = false;
    if (have_odom_) odom_wall_ = ros::WallTime::now();''')
edit(c,'    switch (exec_state_)','''    if (managed_)
    {
      if (!managedMapReady() || !have_target_) return;
      if ((exec_state_ == GEN_NEW_TRAJ || exec_state_ == REPLAN_TRAJ) &&
          (ros::WallTime::now() - planning_wall_).toSec() > 8.0)
      { managedStop("FAILED", "planning_deadline"); return; }
    }
    switch (exec_state_)''')
edit(c,'    exec_state_ = new_state;','''    if (managed_ && new_state != exec_state_ &&
        (new_state == GEN_NEW_TRAJ || new_state == REPLAN_TRAJ))
      planning_wall_ = ros::WallTime::now();
    exec_state_ = new_state;''')
edit(c,'      if (t_cur > info->duration_ - 1e-2)\n      {','''      if (t_cur > info->duration_ - 1e-2)
      {
        if (managed_ && ((odom_pos_ - end_pt_).norm() > 0.2 || odom_vel_.norm() > 0.1))
        { changeFSMExecState(GEN_NEW_TRAJ, "ACTUAL_STATE"); return; }
        if (managed_) managedStatus("COMPLETED");''')
edit(c,'    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)','    if ((managed_ && (!have_target_ || !managedMapReady())) || exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)')
edit(c,'    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);','''    if (managed_ && (odom_pos_ - info->position_traj_.evaluateDeBoorT(std::min(t_cur, info->duration_))).norm() > 0.2)
    {
      start_pt_ = odom_pos_; start_vel_ = odom_vel_; start_acc_.setZero();
      return callReboundReplan(true, false);
    }
    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);''')
first='      bspline_pub_.publish(bspline);'
edit(c,first,'''      if (managed_)
      {
        if (!have_target_) return false;
        bspline.planner_session_id = managed_session_;
        bspline.mission_id = managed_mission_; bspline.goal_id = managed_goal_;
      }
      bspline_pub_.publish(bspline);''')
edit(c,'    bspline_pub_.publish(bspline);','''    if (managed_)
    {
      bspline.planner_session_id = managed_session_;
      bspline.mission_id = managed_mission_; bspline.goal_id = managed_goal_;
      bspline.emergency_stop = true;
    }
    bspline_pub_.publish(bspline);''') if False else None
# Replace the emergency publication specifically, not the earlier indented normal one.
s=c.read_text(); start=s.index('  bool EGOReplanFSM::callEmergencyStop(')
part=s[start:].replace('    bspline_pub_.publish(bspline);','''    if (managed_)
    {
      bspline.planner_session_id = managed_session_;
      bspline.mission_id = managed_mission_; bspline.goal_id = managed_goal_;
      bspline.emergency_stop = true;
    }
    bspline_pub_.publish(bspline);''',1)
c.write_text(s[:start]+part)
edit(c,'if (t > planner_manager_->global_data_.global_duration_)','if (t >= planner_manager_->global_data_.global_duration_ - 1e-6)')
methods=r'''
  bool EGOReplanFSM::managedMapReady()
  {
    return have_odom_ && (ros::WallTime::now()-odom_wall_).toSec() < 0.6 &&
      planner_manager_->grid_map_->inputFresh(0.6);
  }

  void EGOReplanFSM::managedStatus(const std::string &status, const std::string &reason)
  {
    ego_planner::GoalStatus out;
    out.header.stamp = ros::Time::now(); out.header.frame_id = "map";
    out.planner_session_id = managed_session_;
    out.mission_id = managed_mission_; out.goal_id = managed_goal_;
    out.status = status; out.reason = reason; out.map_ready = managedMapReady();
    managed_status_pub_.publish(out);
  }

  void EGOReplanFSM::managedStop(const std::string &status, const std::string &reason)
  {
    if (have_target_) callEmergencyStop(odom_pos_);
    have_target_ = false; have_new_target_ = false; trigger_ = false;
    changeFSMExecState(WAIT_TARGET, "MANAGED_STOP");
    managedStatus(status, reason);
  }

  void EGOReplanFSM::managedWatchdog(const ros::WallTimerEvent &)
  {
    if (!managedMapReady() && have_target_) managedStop("NOT_READY", "stale_map_or_odometry");
    else managedStatus(managedMapReady() ? "READY" : "NOT_READY");
  }

  void EGOReplanFSM::managedCommand(const ego_planner::GoalCommandConstPtr &msg)
  {
    auto reject = [&](const std::string &reason) {
      ego_planner::GoalStatus out;
      out.header.stamp = ros::Time::now(); out.header.frame_id = "map";
      out.planner_session_id = managed_session_; out.mission_id = msg->mission_id;
      out.goal_id = msg->goal_id; out.status = "REJECTED"; out.reason = reason;
      out.map_ready = managedMapReady(); managed_status_pub_.publish(out);
    };
    if (msg->planner_session_id != managed_session_) { reject("planner_session_mismatch"); return; }
    if (msg->mission_id.empty() || !msg->goal_id || msg->header.frame_id != "map")
      { reject("invalid_identity_or_frame"); return; }
    if (msg->command == "CANCEL")
    {
      if (msg->mission_id != managed_mission_ || msg->goal_id != managed_goal_)
        { reject("cancel_identity_mismatch"); return; }
      managedStop("CANCELLED", "requested"); return;
    }
    if (msg->command != "GOAL") { reject("unknown_command"); return; }
    if (retired_missions_.count(msg->mission_id)) { reject("retired_mission"); return; }
    if (msg->mission_id == managed_mission_ && msg->goal_id == managed_goal_ && have_target_)
      { managedStatus("ACCEPTED", "duplicate_active_goal"); return; }
    if (msg->goal_id <= highest_goals_[msg->mission_id]) { reject("retired_goal"); return; }
    if (!managedMapReady()) { reject("MAP_NOT_READY"); return; }
    Eigen::Vector3d target(msg->goal.x,msg->goal.y,msg->goal.z);
    if (!target.allFinite() || !planner_manager_->grid_map_->isInMap(target) ||
        target.z() < 0.5 || target.z() > 2.4 ||
        planner_manager_->grid_map_->getInflateOccupancy(target) == 1)
      { reject("invalid_or_occupied_goal"); return; }
    if (have_target_) managedStop("CANCELLED", "superseded");
    if (!managed_mission_.empty() && managed_mission_ != msg->mission_id)
      retired_missions_.insert(managed_mission_);
    managed_mission_ = msg->mission_id; managed_goal_ = msg->goal_id;
    highest_goals_[managed_mission_] = managed_goal_;
    changeFSMExecState(WAIT_TARGET, "MANAGED_GOAL");
    nav_msgs::Path::Ptr path(new nav_msgs::Path);
    geometry_msgs::PoseStamped goal; goal.pose.position = msg->goal;
    path->poses.push_back(goal); waypointCallback(path);
    if (have_target_) managedStatus("ACCEPTED");
    else managedStatus("FAILED", "global_initialization_failed");
  }
'''
s=c.read_text(); at=s.rfind('} // namespace ego_planner'); assert at>=0
c.write_text(s[:at]+methods+'\n'+s[at:])

gm=root/'src/planner/plan_env/src/grid_map.cpp'
edit(gm,'  // use odometry and point cloud','''  // Independent clouds use world coordinates and must be explicitly selected.
  bool enable_independent_cloud = true;
  node_.param("grid_map/enable_independent_cloud", enable_independent_cloud, true);
  if (enable_independent_cloud) {
  // use odometry and point cloud''')
edit(gm,'  occ_timer_ = node_.createTimer','  }\n\n  occ_timer_ = node_.createTimer')
edit(gm,'  /* get depth image */','''  if (fabs((img->header.stamp-pose->header.stamp).toSec()) > 0.05 ||
      fabs((ros::Time::now()-img->header.stamp).toSec()) > 0.5) return;
  depth_wall_ = ros::WallTime::now();
  /* get depth image */''')
gh=root/'src/planner/plan_env/include/plan_env/grid_map.h'
edit(gh,'  bool hasDepthObservation();','''  bool hasDepthObservation();
  ros::WallTime depth_wall_;
  bool inputFresh(double timeout) {
    return hasDepthObservation() && (ros::WallTime::now()-depth_wall_).toSec() < timeout;
  }''')
print('EGO_MANAGED_PATCH_APPLIED')
