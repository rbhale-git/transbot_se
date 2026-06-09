"""Mock rosbridge server — lets the dashboard run end-to-end with NO robot.

Speaks the subset of the rosbridge v2 JSON protocol that roslib.js uses
(subscribe / unsubscribe / advertise / publish / call_service) on
ws://localhost:9090, and simulates the Transbot driver:

  - /transbot/get_vel : echoes the last received /cmd_vel with noise, decaying
                        to zero when commands stop (so deadman behavior is
                        visible in the MEAS panel).
  - /transbot/imu     : noisy IMU samples.
  - /voltage          : battery that slowly drains from 12.4 V to 9.5 V and
                        resets, exercising the low-battery warning.
  - /CurrentAngle     : service returning the simulated arm pose, which
                        follows received /TargetAngle commands.

Every message received from the dashboard is logged to the console, so you
can watch exactly what would have been sent to the real robot.

Run:  python mock/mock_rosbridge.py
"""

import asyncio
import json
import math
import random
import time

from websockets.asyncio.server import serve

HOST, PORT = "localhost", 9090

# ---------------------------------------------------------------------------
# Simulated robot state (shared across clients)
# ---------------------------------------------------------------------------
state = {
    "last_cmd": {"linear": 0.0, "angular": 0.0, "at": 0.0},
    "gimbal": {1: 90, 2: 90},
    "arm": {7: 110, 8: 150, 9: 90},
    "battery": 12.4,
    "t0": time.monotonic(),
}

CMD_STALE_S = 0.4  # measured velocity decays to 0 if no cmd_vel within this


def log(direction: str, text: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {direction} {text}", flush=True)


# ---------------------------------------------------------------------------
# Inbound message handling
# ---------------------------------------------------------------------------
def handle_publish(topic: str, msg: dict) -> None:
    if topic == "/cmd_vel":
        lin = float(msg.get("linear", {}).get("x", 0.0))
        ang = float(msg.get("angular", {}).get("z", 0.0))
        state["last_cmd"] = {"linear": lin, "angular": ang, "at": time.monotonic()}
        log("RECV", f"/cmd_vel        lin={lin:+.3f} m/s  ang={ang:+.3f} rad/s")
    elif topic == "/PWMServo":
        sid, angle = msg.get("id"), msg.get("angle")
        if sid in (1, 2):
            state["gimbal"][sid] = angle
        log("RECV", f"/PWMServo       id={sid} angle={angle}")
    elif topic == "/TargetAngle":
        joints = msg.get("joint", [])
        for j in joints:
            jid = j.get("id")
            if jid in state["arm"]:
                state["arm"][jid] = j.get("angle")
        log("RECV", f"/TargetAngle    {joints}")
    else:
        log("RECV", f"{topic} {json.dumps(msg)[:120]}")


def service_response(service: str, call_id):
    if service == "/CurrentAngle":
        values = {
            "angles": [
                {"id": jid, "angle": angle} for jid, angle in state["arm"].items()
            ]
        }
    else:
        values = {}
    resp = {"op": "service_response", "service": service, "values": values, "result": True}
    if call_id is not None:
        resp["id"] = call_id
    return resp


# ---------------------------------------------------------------------------
# Outbound telemetry generators
# ---------------------------------------------------------------------------
def gen_get_vel() -> dict:
    cmd = state["last_cmd"]
    fresh = (time.monotonic() - cmd["at"]) < CMD_STALE_S
    lin = cmd["linear"] if fresh else 0.0
    ang = cmd["angular"] if fresh else 0.0
    noise = lambda: random.gauss(0, 0.005)
    return {
        "linear": {"x": lin + noise(), "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": ang + noise()},
    }


def gen_imu() -> dict:
    t = time.monotonic() - state["t0"]
    n = lambda s: random.gauss(0, s)
    return {
        "linear_acceleration": {"x": n(0.05), "y": n(0.05), "z": 9.81 + n(0.05)},
        "angular_velocity": {"x": n(0.01), "y": n(0.01), "z": 0.05 * math.sin(t / 5) + n(0.01)},
        "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(t / 40), "w": math.cos(t / 40)},
    }


def gen_battery() -> dict:
    state["battery"] -= 0.02
    if state["battery"] < 9.5:
        state["battery"] = 12.4
    return {"data": round(state["battery"], 2)}


# topic -> (message generator, publish period in seconds)
TELEMETRY = {
    "/transbot/get_vel": (gen_get_vel, 0.1),
    "/transbot/imu": (gen_imu, 0.2),
    "/voltage": (gen_battery, 1.0),
}


# ---------------------------------------------------------------------------
# Connection handling
# ---------------------------------------------------------------------------
async def telemetry_loop(ws, topic: str, subs: set):
    gen, period = TELEMETRY[topic]
    while topic in subs:
        await ws.send(json.dumps({"op": "publish", "topic": topic, "msg": gen()}))
        await asyncio.sleep(period)


async def handler(ws):
    peer = ws.remote_address
    log("CONN", f"client connected {peer}")
    subs: set = set()
    tasks: dict = {}
    try:
        async for raw in ws:
            try:
                m = json.loads(raw)
            except json.JSONDecodeError:
                continue
            op = m.get("op")

            if op == "subscribe":
                topic = m.get("topic")
                log("SUB ", topic)
                if topic in TELEMETRY and topic not in subs:
                    subs.add(topic)
                    tasks[topic] = asyncio.ensure_future(telemetry_loop(ws, topic, subs))
            elif op == "unsubscribe":
                topic = m.get("topic")
                subs.discard(topic)
                if topic in tasks:
                    tasks.pop(topic).cancel()
            elif op == "publish":
                handle_publish(m.get("topic", ""), m.get("msg", {}))
            elif op == "call_service":
                service = m.get("service")
                log("SRV ", f"{service} called")
                await ws.send(json.dumps(service_response(service, m.get("id"))))
            elif op in ("advertise", "unadvertise", "set_level"):
                pass  # nothing to do in the mock
            else:
                log("RECV", f"unhandled op: {op}")
    finally:
        for t in tasks.values():
            t.cancel()
        log("CONN", f"client disconnected {peer}")


async def main():
    print(f"Mock rosbridge listening on ws://{HOST}:{PORT}  (Ctrl+C to stop)")
    print("Simulating: /transbot/get_vel, /transbot/imu, /voltage, /CurrentAngle")
    async with serve(handler, HOST, PORT):
        await asyncio.get_running_loop().create_future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nmock stopped")
