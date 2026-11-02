#!/usr/bin/env python3
"""Standard rostest entry for the detailed, isolated real-ROS fault suite."""
import json,os,subprocess,tempfile,unittest
from pathlib import Path
import rospkg,rostest

class LifecycleIntegration(unittest.TestCase):
    def test_real_ros_lifecycle(self):
        output=Path(os.environ['UAV_STAGE78_RESULTS']) if 'UAV_STAGE78_RESULTS' in os.environ else Path(tempfile.mkdtemp(prefix='uav-lifecycle-'))
        helper=Path(rospkg.RosPack().get_path('uav_vln_bringup'))/'test'/'run_stage78.py'
        result=subprocess.run(['/usr/bin/python3',str(helper),'--stage','lifecycle','--clock','sim','--out',str(output)],timeout=150)
        self.assertEqual(result.returncode,0,'Detailed ROS test results: '+str(output))
        self.assertTrue(json.loads((output/'results.json').read_text())['passed'])

if __name__=='__main__':
    rostest.rosrun('uav_vln_bringup','lifecycle_integration',LifecycleIntegration)
