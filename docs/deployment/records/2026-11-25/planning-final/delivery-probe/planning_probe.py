"""Run the installed SITL planning validator with additional read-only subscribers."""
import json
import rospy
from ego_planner.msg import Bspline
from std_msgs.msg import String
import validate_planning_sitl as installed


class ProbeTrial(installed.PlanningTrial):
    def __init__(self, args):
        super().__init__(args)
        self.raw_sub = rospy.Subscriber('/planning/bspline', Bspline, self.tagged_spline,
                                        'native_spline', queue_size=150)
        self.forwarded_sub = rospy.Subscriber('/uav/planning/trajectory', Bspline, self.tagged_spline,
                                              'forwarded_spline', queue_size=150)
        self.disposition_sub = rospy.Subscriber('/uav/planning/trajectory_disposition', String,
                                                self.gateway_disposition, queue_size=150)

    def gateway_disposition(self, msg):
        self.record('gateway_disposition', json.loads(msg.data))

    def tagged_spline(self, msg, label):
        self.record(label, dict(planner_session_id=msg.planner_session_id,
                    mission_id=msg.mission_id, goal_id=msg.goal_id, traj_id=msg.traj_id,
                    emergency=msg.emergency_stop, start_time_ns=msg.start_time.to_nsec()))

    def configuration(self):
        super().configuration()
        self.wait(lambda: self.raw_sub.get_num_connections() and self.forwarded_sub.get_num_connections(),
                  10., 'native and forwarded trajectory recorders connected')


if __name__ == '__main__':
    installed.PlanningTrial = ProbeTrial
    installed.main()
