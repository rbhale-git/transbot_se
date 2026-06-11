"""One-shot actuation-path diagnosis for the Transbot.

The robot's old rosbridge (0.11/Melodic) can half-fail a publisher
registration and silently drop every command on that topic while others keep
working (docs/AI_NOTEBOOK.md) — the classic symptom is "gimbal moves, drive
or arm dead". This script checks each path from the outside:

  1. node list                      — driver / mux / watchdog alive?
  2. subscriber lists               — is anyone listening on each command topic?
  3. /mux/status sample             — mux heartbeat + which source it honors
  4. /manual/cmd_vel -> /cmd_vel    — end-to-end echo using ZERO twists (no motion)
  5. /CurrentAngle service          — driver's service path responds?

Run from the repo root:  python tools/diagnose_actuation.py [--url ws://IP:9090]
"""

import argparse
import time

import roslibpy

COMMAND_TOPICS = ("/cmd_vel", "/manual/cmd_vel", "/ai/cmd_vel",
                  "/TargetAngle", "/PWMServo")

ZERO_TWIST = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
              "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}


def call(ros, name, srv_type, req=None, timeout=5):
    try:
        service = roslibpy.Service(ros, name, srv_type)
        return service.call(roslibpy.ServiceRequest(req or {}), timeout=timeout)
    except Exception as exc:  # report, don't die — later checks still matter
        return {"_error": str(exc)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://192.168.0.109:9090")
    args = parser.parse_args()

    host, _, port = args.url.replace("ws://", "").rstrip("/").partition(":")
    ros = roslibpy.Ros(host=host, port=int(port or 9090))
    ros.run(timeout=10)
    print(f"connected to {args.url}: {ros.is_connected}")
    if not ros.is_connected:
        return

    print("\n-- nodes --")
    nodes = call(ros, "/rosapi/nodes", "rosapi/Nodes")
    for node in sorted(nodes.get("nodes", [])):
        print("  ", node)
    if "_error" in nodes:
        print("  ERROR:", nodes["_error"])

    print("\n-- command-topic subscribers (empty = nobody is listening!) --")
    for topic in COMMAND_TOPICS:
        resp = call(ros, "/rosapi/subscribers", "rosapi/Subscribers",
                    {"topic": topic})
        print(f"  {topic:18s} -> {resp.get('subscribers', resp)}")

    print("\n-- /mux/status heartbeat (2.5 s listen) --")
    samples = []
    mux = roslibpy.Topic(ros, "/mux/status", "std_msgs/String")
    mux.subscribe(lambda m: samples.append(m["data"]))
    time.sleep(2.5)
    mux.unsubscribe()
    if samples:
        print(f"  {len(samples)} msgs; last: {samples[-1]}")
    else:
        print("  NONE — mux is not publishing (dead or not deployed)")

    print("\n-- /manual/cmd_vel -> /cmd_vel echo (zero twists, no motion) --")
    echoes = []
    cmd_vel = roslibpy.Topic(ros, "/cmd_vel", "geometry_msgs/Twist")
    cmd_vel.subscribe(lambda m: echoes.append(m))
    time.sleep(2.0)
    baseline = len(echoes)
    print(f"  baseline /cmd_vel rate: {baseline / 2.0:.1f} Hz")

    manual = roslibpy.Topic(ros, "/manual/cmd_vel", "geometry_msgs/Twist")
    manual.advertise()
    time.sleep(1.0)  # settle — registration racing traffic is the known bug
    for _ in range(5):
        manual.publish(roslibpy.Message(ZERO_TWIST))
        time.sleep(0.2)
    time.sleep(1.0)
    manual.unadvertise()
    cmd_vel.unsubscribe()
    after = len(echoes) - baseline
    print(f"  /cmd_vel msgs in the 3 s around 5 manual publishes: {after}")
    print("  (with an idle mux this should be >= the published count;"
          " 0 with a 0 Hz baseline = manual path dead)")

    print("\n-- /CurrentAngle service --")
    arm = call(ros, "/CurrentAngle", "transbot_msgs/RobotArm", {"apply": ""})
    print("  ", arm)

    ros.terminate()


if __name__ == "__main__":
    main()
