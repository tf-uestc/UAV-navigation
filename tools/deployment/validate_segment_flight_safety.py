#!/usr/bin/env python3
"""Verify command revocation and landing evidence with isolated ROS and a fake FCU."""
import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time
import xmlrpc.client
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace

MASTER = "http://127.0.0.1:11315"


def terminate(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run(stack, out, checks):
    # Refuse to attach to any existing master, including other tests on 11315.
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 11315))
    os.environ["ROS_MASTER_URI"] = MASTER
    os.environ["ROS_IP"] = "127.0.0.1"
    os.environ.pop("ROS_HOSTNAME", None)
    import rospy
    from mavros_msgs.msg import ExtendedState
    from std_msgs.msg import String
    from validate_flight_execution import FakeFcu, Trial

    def start(name, argv):
        stream = stack.enter_context(out.with_suffix("." + name + ".log").open("w"))
        process = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT)
        stack.callback(terminate, process)
        return process

    master = start("roscore", ["roscore", "-p", "11315"])
    for _ in range(100):
        if master.poll() is not None:
            raise RuntimeError("isolated master exited during startup")
        try:
            if xmlrpc.client.ServerProxy(MASTER).getPid("/safety_verifier")[0] == 1:
                break
        except OSError:
            pass
        time.sleep(.1)
    else:
        raise RuntimeError("isolated master startup timed out")
    rospy.init_node("verify_segment_safety", disable_signals=True)
    stack.callback(rospy.signal_shutdown, "isolated test finished")
    rospy.set_param("/uav/segment_config/fence", dict(x_min=-6., x_max=6., y_min=-4., y_max=4.,
                                                      z_min=1.1, z_max=2.35))

    class RecordedFcu(FakeFcu):
        def __init__(self):
            self.disarms = []
            super().__init__()

        def arm(self, request):
            if not request.value:
                self.disarms.append(dict(wall=time.monotonic(), height=self.position[2]))
            return super().arm(request)

    fake = RecordedFcu()
    stack.callback(fake.done.set)
    trial = Trial(SimpleNamespace(mock=False, real_ego=False))
    status, requests, cancels, commands = {}, [], [], []
    forward = dict(status=True, hover=True, ground="native")
    status_pub = rospy.Publisher("/safety_test/flight_status", String, queue_size=10)
    ext_pub = rospy.Publisher("/safety_test/extended_state", ExtendedState, queue_size=1)
    mission_pub = rospy.Publisher("/uav/segment/command", String, queue_size=1)

    def extended(msg):
        if forward["ground"] == "stale":
            return
        if forward["ground"] == "forced":
            msg.landed_state = 1
        ext_pub.publish(msg)

    def command(msg):
        data = json.loads(msg.data)
        commands.append((time.monotonic(), data))
        if data["command"] != "HOVER" or forward["hover"]:
            trial.command.publish(msg)

    def update_status(msg):
        status.update(json.loads(msg.data))

    rospy.Subscriber("/uav/segment/status", String, update_status)
    rospy.Subscriber("/uav/planning/request", String, lambda m: requests.append(json.loads(m.data)))
    rospy.Subscriber("/uav/cancel_planning", String,
                     lambda m: cancels.append((time.monotonic(), json.loads(m.data))))
    rospy.Subscriber("/uav/flight/status", String, lambda m: status_pub.publish(m) if forward["status"] else None)
    rospy.Subscriber("/safety_test/flight_command", String, command)
    rospy.Subscriber("/mavros/extended_state", ExtendedState, extended)
    scripts = Path(__file__).resolve().parents[2] / "ws/src/uav_vln_bringup/scripts"
    assert os.environ["ROS_MASTER_URI"] == MASTER
    start("executor", [sys.executable, str(scripts / "flight_executor.py"), "_mock_fcu:=true",
                       "/mavros/extended_state:=/safety_test/extended_state"])
    start("mission", [sys.executable, str(scripts / "segment_mission.py"),
                      "/uav/flight/status:=/safety_test/flight_status",
                      "/uav/flight/command:=/safety_test/flight_command"])
    trial.wait(lambda: status and trial.status and trial.status.get("ready") and
               mission_pub.get_num_connections() and status_pub.get_num_connections(), 15, "startup")
    time.sleep(.4)

    def send(cmd, mission, **payload):
        mission_pub.publish(String(data=json.dumps(dict(command=cmd, mission_id=mission,
                            mission_session_id=status["mission_session_id"], **payload))))

    forward["ground"] = "forced"
    send("START", "takeoff-cancel", takeoff=True, waypoint=[1., 0., 1.6])
    trial.wait(lambda: fake.position[2] > .10, 12, "climb with contradicted landed flag")
    forward["ground"] = "stale"
    cancel_height = fake.position[2]
    send("CANCEL", "takeoff-cancel")
    trial.wait(lambda: fake.mode == "AUTO.LAND", 2, "request landing while state reports ground")
    time.sleep(.4)
    assert not fake.disarms, fake.disarms
    forward["ground"] = "native"
    trial.wait(lambda: status.get("phase") == "LANDED" and not fake.armed, 8, "normal touchdown")
    assert fake.disarms and max(d["height"] for d in fake.disarms) < .04, fake.disarms
    checks.append(dict(name="cancel_climb_rejects_stale_landed_and_disarms_after_touchdown",
                       cancel_height_m=cancel_height, disarms=fake.disarms[:]))

    send("START", "feedback-loss", takeoff=True, waypoint=[1.5, 0., 1.6])
    trial.wait(lambda: requests and requests[-1]["mission_id"] == "feedback-loss", 22, "plan request")
    req = requests[-1]
    trial.mission, trial.goal = req["mission_id"], req["goal_id"]
    old = trial.plan(fake.position, req["goal"], duration=6.)
    trial.wait(lambda: status.get("phase") == "TRACK" and fake.position[0] > .05, 5, "tracked movement")
    assert any(d["command"] == "ACK_STATUS" for _,d in commands)
    forward["status"] = forward["hover"] = False
    interrupted = time.monotonic()
    trial.wait(lambda: trial.status.get("reason") == "mission_feedback_timeout"
               and trial.status["context"] is None, 1.5, "executor independently revoked expired authority")
    revocation_delay = time.monotonic() - interrupted
    trial.wait(lambda: status.get("phase") == "WAIT_ABORT_STOP"
               and any(t >= interrupted and d["command"] == "HOVER" for t,d in commands)
               and any(d["mission_id"] == trial.mission for _,d in cancels), 2, "both nodes interrupted old work")
    trial.wait(lambda: trial.status.get("stop_confirmed"), 5, "measured halt")
    held = fake.position
    old.start_time = rospy.Time.now()
    old.traj_id += 100
    trial.plan_pub.publish(old)
    time.sleep(.4)
    assert trial.status["mode"] == "HOLD" and trial.status["context"] is None
    assert math.dist(held, fake.position) < .03
    forward["status"] = forward["hover"] = True
    trial.wait(lambda: status.get("phase") == "FAILED" and
               status.get("reason") == "flight_feedback_unavailable", 3, "mission observed halt after recovery")
    checks.append(dict(name="one_way_status_loss_revokes_old_trajectory", measured_revoke_delay_s=revocation_delay,
                       reported_reason=trial.status["reason"], motion_after_delayed_spline_m=math.dist(held, fake.position),
                       cancellation=cancels[-1][1], terminal_reason=status["reason"]))
    send("LAND", "feedback-loss")
    trial.wait(lambda: status.get("phase") == "LANDED" and not fake.armed, 10, "landing")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    out = parser.parse_args().out
    out.parent.mkdir(parents=True, exist_ok=True)
    checks = []
    report = dict(scope="isolated ROS topics, fake MAVROS services and kinematics", passed=False, checks=checks)
    try:
        with ExitStack() as stack:
            run(stack, out, checks)
        report["passed"] = True
    except Exception as exc:
        report["error"] = repr(exc)
        raise
    finally:
        out.write_text(json.dumps(report, indent=2) + "\n")
