import ast
import unittest
from _support import ROOT,Publisher,PointStamped,Header
class SmokeTests(unittest.TestCase):
    def test_python38_grammar(self):
        for path in (ROOT/"ws/src/uav_vln_bringup/scripts").rglob("*.py"):
            ast.parse(path.read_text(encoding="utf-8-sig"),filename=str(path),feature_version=(3,8))
    def test_transport_copies_and_changes_seq(self):
        p=Publisher("/test"); m=PointStamped(header=Header(seq=901)); p.publish(m)
        self.assertEqual(p.messages[0].header.seq,0)
        self.assertEqual(m.header.seq,901)


    def test_launch_and_package_xml(self):
        import xml.etree.ElementTree as ET
        package=ROOT/"ws/src/uav_vln_bringup"
        for path in list((package/"launch").glob("*.launch"))+[package/"package.xml"]:
            ET.parse(path)
