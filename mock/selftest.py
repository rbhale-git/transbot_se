"""Protocol self-test for the mock rosbridge server.

Connects as a rosbridge client (same ops roslib.js uses) and verifies:
  1. subscribing to /voltage yields a publish within 3 s
  2. publishing /cmd_vel is accepted and echoed back via /transbot/get_vel
  3. calling /CurrentAngle returns a service_response with arm angles
  4. /TargetAngle publishes update the pose /CurrentAngle reports

Run with the mock already up:  python mock/selftest.py
Exits 0 on success, 1 on failure.
"""

import asyncio
import json
import sys

from websockets.asyncio.client import connect

URL = "ws://localhost:9090"


async def expect(ws, predicate, timeout=3.0, what=""):
    """Read frames until one matches predicate, or fail."""
    async with asyncio.timeout(timeout):
        async for raw in ws:
            m = json.loads(raw)
            if predicate(m):
                return m
    raise AssertionError(f"timed out waiting for {what}")


async def main():
    async with connect(URL) as ws:
        # 1. battery telemetry
        await ws.send(json.dumps({"op": "subscribe", "topic": "/voltage", "type": "std_msgs/Float32"}))
        m = await expect(ws, lambda m: m.get("topic") == "/voltage", what="/voltage publish")
        assert isinstance(m["msg"]["data"], (int, float)), m
        print(f"PASS  /voltage publish received: {m['msg']['data']} V")

        # 2. cmd_vel accepted and echoed in measured velocity
        await ws.send(json.dumps({"op": "subscribe", "topic": "/transbot/get_vel", "type": "geometry_msgs/Twist"}))
        await ws.send(json.dumps({"op": "publish", "topic": "/cmd_vel", "msg": {
            "linear": {"x": 0.2, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}}))
        m = await expect(
            ws,
            lambda m: m.get("topic") == "/transbot/get_vel" and abs(m["msg"]["linear"]["x"] - 0.2) < 0.05,
            what="get_vel echo of cmd_vel",
        )
        print(f"PASS  /transbot/get_vel echoes cmd: lin={m['msg']['linear']['x']:.3f}")

        # 3. /CurrentAngle service
        await ws.send(json.dumps({"op": "call_service", "service": "/CurrentAngle", "id": "t1"}))
        m = await expect(ws, lambda m: m.get("op") == "service_response" and m.get("id") == "t1",
                         what="/CurrentAngle response")
        angles = {a["id"]: a["angle"] for a in m["values"]["angles"]}
        assert set(angles) == {7, 8, 9}, angles
        print(f"PASS  /CurrentAngle responds: {angles}")

        # 4. /TargetAngle moves the simulated arm
        await ws.send(json.dumps({"op": "publish", "topic": "/TargetAngle", "msg": {
            "joint": [{"id": 7, "angle": 42, "run_time": 800}]}}))
        await asyncio.sleep(0.1)
        await ws.send(json.dumps({"op": "call_service", "service": "/CurrentAngle", "id": "t2"}))
        m = await expect(ws, lambda m: m.get("op") == "service_response" and m.get("id") == "t2",
                         what="/CurrentAngle after TargetAngle")
        angles = {a["id"]: a["angle"] for a in m["values"]["angles"]}
        assert angles[7] == 42, angles
        print(f"PASS  /TargetAngle updates pose: j7={angles[7]}")

        # 5. /rosapi/get_time (used by the dashboard's RTT readout)
        await ws.send(json.dumps({"op": "call_service", "service": "/rosapi/get_time", "id": "t3"}))
        m = await expect(ws, lambda m: m.get("op") == "service_response" and m.get("id") == "t3",
                         what="/rosapi/get_time response")
        assert m["values"]["time"]["secs"] > 0, m
        print(f"PASS  /rosapi/get_time responds: secs={m['values']['time']['secs']}")

    print("ALL PASS")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except (AssertionError, TimeoutError) as e:
        print(f"FAIL  {e}")
        sys.exit(1)
