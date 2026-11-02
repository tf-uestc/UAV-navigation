"""Deterministic ROS integration fixture. Never calls any external API."""
import time
from base_detector import Detection, Detector

class FixtureDetector(Detector):
    def __init__(self,**_kwargs): pass
    def detect(self,rgb,depth,target_text,cam_info):
        time.sleep(.6 if target_text.startswith('slow') else .08)
        if target_text=='missing': return None
        return Detection((2,2,6,6),(10.,0.,2.),2.,1.,target_text=target_text)
