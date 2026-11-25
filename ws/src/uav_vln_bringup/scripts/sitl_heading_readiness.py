#!/usr/bin/env python3
"""Bridge the local PX4 SITL heading validity to ROS using read-only uORB samples."""
import json
import os
from pathlib import Path
import re
import subprocess
import time


def parse_position(output):
    values=dict(re.findall(r'^\s*(timestamp_sample|heading_good_for_control|heading_reset_counter):\s*(\S+)',output,re.M))
    good=values['heading_good_for_control'].lower()
    if good not in ('true','false','1','0'):
        raise ValueError('unknown PX4 heading validity')
    return int(values['timestamp_sample'])*1e-6,good in ('true','1'),int(values['heading_reset_counter'])


class HeadingSamples:
    def __init__(self):
        self.since=self.counter=self.last=None

    def update(self,stamp,good,counter,sim_now):
        if not good or not -.02 <= sim_now-stamp <= .65:
            self.since=self.last=None
            return False
        if self.last is None or counter!=self.counter or stamp<self.last or stamp-self.last>.65:
            self.since=stamp
        self.last,self.counter=stamp,counter
        return stamp-self.since>=1.


def main():
    import rospy
    from std_msgs.msg import String
    rospy.init_node('sitl_heading_readiness')
    binary=Path(os.environ['PX4_AUTOPILOT_DIR'])/'build/px4_sitl_default/bin/px4-listener'
    if not binary.is_file():
        raise RuntimeError('PX4 SITL uORB listener missing')
    pub=rospy.Publisher('/uav/sitl/heading_ready',String,queue_size=1)
    tracked=HeadingSamples()
    while not rospy.is_shutdown():
        try:
            proc=subprocess.run([str(binary),'vehicle_local_position','-n','1'],text=True,
                                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=.8,check=True)
            stamp,good,counter=parse_position(proc.stdout)
            ready=tracked.update(stamp,good,counter,rospy.Time.now().to_sec())
        except (KeyError,ValueError,OSError,subprocess.SubprocessError):
            tracked=HeadingSamples()
            stamp,good,counter,ready=rospy.Time.now().to_sec(),False,None,False
        pub.publish(String(data=json.dumps(dict(stamp=stamp,heading_good=good,counter=counter,ready=ready))))
        time.sleep(.16)


if __name__=='__main__':
    main()
