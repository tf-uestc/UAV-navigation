"""Transport stand-ins live only in tests; they copy and overwrite Header.seq."""
import copy
import importlib.util
import math
import sys
import types
from pathlib import Path
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT/"ws/src/uav_vln_bringup/scripts"
sys.path.insert(0,str(SCRIPTS))

class Time:
    current = 0.0
    def __init__(self, secs=0, nsecs=0): self.value=float(secs)+nsecs*1e-9
    def to_sec(self): return self.value
    @classmethod
    def now(cls): return cls(cls.current)
    @classmethod
    def from_sec(cls,v): return cls(v)
    def __sub__(self,o): return Time(self.value-o.value)
    def __add__(self,o): return Time(self.value+o.value)
    def __lt__(self,o): return self.value<o.value
    def __le__(self,o): return self.value<=o.value
    def __eq__(self,o): return isinstance(o,Time) and self.value==o.value
class Header:
    def __init__(self,seq=0,stamp=None,frame_id=""):
        self.seq,self.stamp,self.frame_id=seq,stamp or Time(),frame_id
class Point:
    def __init__(self,x=0,y=0,z=0): self.x,self.y,self.z=x,y,z
class Quaternion:
    def __init__(self,x=0,y=0,z=0,w=1): self.x,self.y,self.z,self.w=x,y,z,w
class PoseStamped:
    def __init__(self,header=None,pose=None):
        self.header=header or Header()
        self.pose=pose or types.SimpleNamespace(position=Point(),orientation=Quaternion())
class PointStamped:
    def __init__(self,header=None,point=None): self.header,self.point=header or Header(),point or Point()
class PathMessage:
    def __init__(self,header=None,poses=None): self.header,self.poses=header or Header(),poses or []
class String:
    def __init__(self,data=""): self.data=data
class Publisher:
    def __init__(self,topic,*a,**kw):
        self.topic,self.messages,self.sequence,self.on_publish=topic,[],0,None
    def publish(self,message):
        sent=copy.deepcopy(message)
        if hasattr(sent,"header"):
            sent.header.seq=self.sequence
            self.sequence+=1
        self.messages.append(sent)
        if self.on_publish: self.on_publish(sent)
def ros_modules():
    rospy=types.ModuleType("rospy")
    rospy.Time,rospy.Duration,rospy.Publisher=Time,Time,Publisher
    rospy.get_param=lambda key,default=None: default
    for name in ["init_node","loginfo","logwarn","logerr","loginfo_throttle","logwarn_throttle","logerr_throttle"]:
        setattr(rospy,name,lambda *a,**kw: None)
    rospy.Subscriber=rospy.Timer=lambda *a,**kw: None
    rospy.is_shutdown=lambda: False
    rospy.AnyMsg=object
    rospy.sleep=lambda seconds: None
    modules={"rospy":rospy}
    messages={"geometry_msgs":{"Point":Point,"Quaternion":Quaternion,"PoseStamped":PoseStamped,"PointStamped":PointStamped},
        "nav_msgs":{"Path":PathMessage,"Odometry":object},"std_msgs":{"Header":Header,"String":String},
        "ego_planner":{"Bspline":object},"sensor_msgs":{"CameraInfo":object,"Image":object}}
    for name,attributes in messages.items():
        parent,child=types.ModuleType(name),types.ModuleType(name+".msg")
        for key,value in attributes.items(): setattr(child,key,value)
        parent.msg=child
        modules[name],modules[name+".msg"]=parent,child
    tf,transforms=types.ModuleType("tf"),types.ModuleType("tf.transformations")
    transforms.quaternion_from_euler=lambda r,p,y:(0,0,math.sin(y/2),math.cos(y/2))
    tf.transformations=transforms
    modules.update({"tf":tf,"tf.transformations":transforms})
    return modules
def load_node(name):
    with patch.dict(sys.modules,ros_modules()):
        spec=importlib.util.spec_from_file_location("offline_"+name,SCRIPTS/(name+".py"))
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

class IdleThread:
    def __init__(self,target=None,args=(),**kwargs):
        self.target,self.args=target,args
    def start(self): pass
    def is_alive(self): return False

def detector_modules():
    modules=ros_modules()
    import numpy as np
    cv2=types.ModuleType("cv2")
    cv2.COLOR_BGR2RGB=cv2.COLOR_RGB2BGR=1
    cv2.cvtColor=lambda data,code:data[...,::-1].copy()
    modules["cv2"]=cv2
    requests=types.ModuleType("requests")
    def no_network(*args,**kwargs):
        raise AssertionError("network calls forbidden in offline tests")
    requests.post=no_network
    requests.exceptions=types.SimpleNamespace(
        Timeout=type("Timeout",(Exception,),{}),
        RequestException=type("RequestException",(Exception,),{}))
    modules["requests"]=requests
    bridge=types.ModuleType("cv_bridge")
    class CvBridge:
        def imgmsg_to_cv2(self,msg,encoding): return msg.array
        def cv2_to_imgmsg(self,array,encoding): return array
    bridge.CvBridge=CvBridge
    modules["cv_bridge"]=bridge
    tf2=types.ModuleType("tf2_ros")
    for name in ("LookupException","ConnectivityException","ExtrapolationException"):
        setattr(tf2,name,type(name,(Exception,),{}))
    tf2.Buffer=lambda *a: None
    tf2.TransformListener=lambda *a: None
    modules["tf2_ros"]=tf2
    modules["tf2_geometry_msgs"]=types.ModuleType("tf2_geometry_msgs")
    return modules

def load_detector(name="detector_node"):
    with patch.dict(sys.modules,detector_modules()):
        spec=importlib.util.spec_from_file_location("offline_"+name,SCRIPTS/(name+".py"))
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
