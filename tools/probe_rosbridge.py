"""Read-only probe of a live rosbridge server.

Verifies the dashboard's telemetry path end-to-end without sending any
command that could move the robot: subscribes to /voltage, /transbot/get_vel,
and /imu/data, calls /CurrentAngle and /rosapi/get_time (RTT), and reports.

Usage:  python tools/probe_rosbridge.py [ws://192.168.0.109:9090]
"""

import asyncio
import json
import sys
import time

from websockets.asyncio.client import connect

URL = sys.argv[1] if len(sys.argv) > 1 else "ws://192.168.0.109:9090"


async def expect(ws, predicate, timeout=6.0, what=""):
    async with asyncio.timeout(timeout):
        async for raw in ws:
            m = json.loads(raw)
            if predicate(m):
                return m
    raise AssertionError(f"timed out waiting for {what}")


async def main():
    print(f"Connecting to {URL} ...")
    async with connect(URL) as ws:
        print("PASS  WebSocket connected")

        # RTT via rosapi
        t0 = time.perf_counter()
        await ws.send(json.dumps({"op": "call_service", "service": "/rosapi/get_time", "id": "rtt"}))
        m = await expect(ws, lambda m: m.get("id") == "rtt", what="get_time")
        print(f"PASS  /rosapi/get_time RTT {int((time.perf_counter()-t0)*1000)} ms")

        # Battery
        await ws.send(json.dumps({"op": "subscribe", "topic": "/voltage", "type": "transbot_msgs/Battery"}))
        m = await expect(ws, lambda m: m.get("topic") == "/voltage", timeout=15, what="/voltage")
        print(f"PASS  battery: {m['msg'].get('Voltage')} V")

        # Measured velocity
        await ws.send(json.dumps({"op": "subscribe", "topic": "/transbot/get_vel", "type": "geometry_msgs/Twist"}))
        m = await expect(ws, lambda m: m.get("topic") == "/transbot/get_vel", timeout=15, what="get_vel")
        print(f"PASS  get_vel: lin={m['msg']['linear']['x']:.3f} ang={m['msg']['angular']['z']:.3f}")

        # Filtered IMU
        await ws.send(json.dumps({"op": "subscribe", "topic": "/imu/data", "type": "sensor_msgs/Imu"}))
        m = await expect(ws, lambda m: m.get("topic") == "/imu/data", timeout=15, what="/imu/data")
        acc = m["msg"]["linear_acceleration"]
        print(f"PASS  imu/data: accel z={acc['z']:.2f}")

        # Arm pose service
        await ws.send(json.dumps({"op": "call_service", "service": "/CurrentAngle",
                                  "args": {"apply": "GetJoint"}, "id": "arm"}))
        m = await expect(ws, lambda m: m.get("id") == "arm", what="/CurrentAngle")
        joints = m.get("values", {}).get("RobotArm", {}).get("joint", [])
        print(f"PASS  /CurrentAngle: {[(j['id'], round(j['angle'], 1)) for j in joints]}"
              if joints else f"NOTE  /CurrentAngle returned: {m.get('values')}")

    print("ALL CHECKS DONE")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"FAIL  {type(e).__name__}: {e}")
        sys.exit(1)
