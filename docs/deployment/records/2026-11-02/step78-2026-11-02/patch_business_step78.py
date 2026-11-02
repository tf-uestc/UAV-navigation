from pathlib import Path
r=Path('/root/autodl-tmp/uav/UAV-navigation/ws/src/uav_vln_bringup')
c=r/'CMakeLists.txt';s=c.read_text()
s=s.replace('  scripts/detector_node.py','  scripts/managed_planner_node.py\n  scripts/camera_pose_bridge.py\n  scripts/detector_node.py',1)
c.write_text(s)
p=r/'package.xml';s=p.read_text().replace('  <export>','''  <exec_depend>message_filters</exec_depend>
  <exec_depend>nodelet</exec_depend>
  <exec_depend>depth_image_proc</exec_depend>
  <exec_depend>ego_planner</exec_depend>
  <exec_depend>roslaunch</exec_depend>
  <test_depend>rostest</test_depend>
  <export>''',1);p.write_text(s)
p=r/'config/ego_planner_params.yaml';s=p.read_text()
for a,b in [('cx: 320.5','cx: 319.5'),('cy: 240.5','cy: 239.5'),('fx: 554.254','fx: 343.15907310693535'),('fy: 554.254','fy: 343.15907310693535'),('max_vel: 1.5','max_vel: 0.8'),('max_acc: 2.0','max_acc: 0.8')]:s=s.replace(a,b)
p.write_text(s)
p=r/'scripts/mission_manager.py';s=p.read_text()
s=s.replace('        self._plan_key = 0','''        self._identified_planning = rospy.get_param('~identified_planning', False)
        self._gateway_session = None
        self._plan_key = 0''',1)
s=s.replace('        self._cancel_pub = rospy.Publisher(','''        self._planning_request_pub = rospy.Publisher('/uav/planning/request', String, queue_size=10)
        rospy.Subscriber('/uav/planning/readiness', String, self._on_planning_readiness, queue_size=10)
        rospy.Subscriber('/uav/planning/result', String, self._on_planning_result, queue_size=30)
        self._cancel_pub = rospy.Publisher(''',1)
s=s.replace('    def _on_trajectory(self, msg):\n','    def _on_trajectory(self, msg):\n        if self._identified_planning:\n            return  # Unidentified compatibility Paths never complete a managed request.\n',1)
s=s.replace('    def _on_planner_status(self, msg):\n','    def _on_planner_status(self, msg):\n        if self._identified_planning:\n            return  # Legacy diagnostic strings are not business acknowledgements.\n',1)
s=s.replace('        self._approach_target_pub.publish(target)','''        self._approach_target_pub.publish(target)
        if self._identified_planning:
            self._publish_identified_plan()''',1)
s=s.replace('                self._approach_target_pub.publish(self._last_plan_message)','''                self._approach_target_pub.publish(self._last_plan_message)
                if self._identified_planning:
                    self._publish_identified_plan()''',1)
idx=s.index('    def _is_expected_detection(')
methods='''    def _publish_identified_plan(self):
        if self._last_plan_message is None:
            return
        p=self._last_plan_message.point
        self._planning_request_pub.publish(String(data=json.dumps(dict(schema_version=1,
            mission_id=self._mission_id,goal_id=self._plan_key,goal=[p.x,p.y,p.z],
            frame_id='map',gateway_session_id=self._gateway_session))))

    @_serialized
    def _on_planning_readiness(self,msg):
        if not self._identified_planning: return
        try:
            d=json.loads(msg.data)
            if d.get('schema_version')!=1 or not isinstance(d.get('gateway_session_id'),str): return
            session=d['gateway_session_id']
            if self._gateway_session and session!=self._gateway_session and self._state==STATE_APPROACHING:
                self._set_state(STATE_HOVER)
            self._gateway_session=session
            self._planner_ready=d.get('ready') is True
        except (ValueError,TypeError,AttributeError): return

    @_serialized
    def _on_planning_result(self,msg):
        if not self._identified_planning: return
        try:
            d=json.loads(msg.data)
            if (d.get('schema_version')!=1 or d.get('mission_id')!=self._mission_id or
                type(d.get('goal_id')) is not int or d['goal_id']!=self._plan_key or
                d.get('gateway_session_id')!=self._gateway_session): return
        except (ValueError,TypeError,AttributeError): return
        if self._state!=STATE_APPROACHING: return
        status=d.get('status')
        if status=='OK' and self._approach_phase==APPROACH_WAIT_PLAN:
            if self._plan_wait.complete(self._plan_key):
                self._trajectory_after_target=True
                self._approach_phase=APPROACH_FLYING
                self._last_approach_mono=self._clock()
                self._approach_hold_goal=None
        elif status=='MAP_NOT_READY':
            self._planner_ready=False
            if self._approach_phase==APPROACH_FLYING: self._set_state(STATE_HOVER)
        elif status in ('FAILED','REJECTED','SESSION_CHANGED','STOP'):
            self._set_state(STATE_HOVER)

'''
s=s[:idx]+methods+s[idx:];p.write_text(s)
print('BUSINESS_STEP78_PATCHED')
