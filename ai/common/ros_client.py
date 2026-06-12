"""Thin roslibpy wrapper for AI behaviors.

Wire shapes were verified against the live robot (FINDINGS.md). This module
owns the websocket connection and topic advertising; behaviors only ever
hand it plain dicts built by the pure helpers below (which is what the unit
tests cover — the connection path is exercised live).

Stage 2 additions: /ai/cmd_vel (chassis, arbitrated by the robot-side mux),
/ai/status (behavior state for the dashboard), /ai/enabled subscription
(defense in depth — behaviors stop publishing motion when disarmed, even
though the mux already enforces it).
"""

import json
import time

import roslibpy

PWM_SERVO_TOPIC = "/PWMServo"
PWM_SERVO_TYPE = "transbot_msgs/PWMServo"
AI_CMD_VEL_TOPIC = "/ai/cmd_vel"
AI_CMD_VEL_TYPE = "geometry_msgs/Twist"
AI_STATUS_TOPIC = "/ai/status"
AI_STATUS_TYPE = "std_msgs/String"
AI_ENABLED_TOPIC = "/ai/enabled"
AI_ENABLED_TYPE = "std_msgs/Bool"
ROSAPI_PUBLISHERS_SERVICE = "/rosapi/publishers"
ROSAPI_PUBLISHERS_TYPE = "rosapi/Publishers"

# Pause between advertising and first publish. The robot's old rosbridge
# (0.11/Melodic) can hit "Internal error processing topic" if its publisher
# registration races traffic — seen live 2026-06-10, where it silently
# dropped every command for that session (docs/AI_NOTEBOOK.md).
ADVERTISE_SETTLE_S = 1.0


def pwm_servo_message(servo_id, angle_deg):
    """Verified shape: transbot_msgs/PWMServo = {int32 id, int32 angle}."""
    return {"id": int(servo_id), "angle": int(round(angle_deg))}


def twist_message(linear_x, angular_z):
    """geometry_msgs/Twist dict; only the two driven fields are non-zero."""
    return {
        "linear": {"x": float(linear_x), "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)},
    }


class RosClient:
    """Connection + publishers for one AI behavior process."""

    def __init__(self, url):
        # roslibpy wants host/port split; accept the dashboard-style ws:// URL.
        host, port = self._parse(url)
        self._ros = roslibpy.Ros(host=host, port=port)
        self._pwm = roslibpy.Topic(self._ros, PWM_SERVO_TOPIC, PWM_SERVO_TYPE)
        self._twist = roslibpy.Topic(self._ros, AI_CMD_VEL_TOPIC, AI_CMD_VEL_TYPE)
        self._status = roslibpy.Topic(self._ros, AI_STATUS_TOPIC, AI_STATUS_TYPE)
        self._enabled = roslibpy.Topic(self._ros, AI_ENABLED_TOPIC, AI_ENABLED_TYPE)
        self._pubs = (self._pwm, self._twist, self._status)

    @staticmethod
    def _parse(url):
        stripped = url.replace("ws://", "").rstrip("/")
        host, _, port = stripped.partition(":")
        return host, int(port or 9090)

    @property
    def connected(self):
        return self._ros.is_connected

    def connect(self, timeout_s=10):
        self._ros.run(timeout=timeout_s)
        if not self._ros.is_connected:
            raise RuntimeError("rosbridge connection failed")
        # Advertise up front instead of lazily on first publish, then let the
        # registration settle before any traffic.
        for topic in self._pubs:
            topic.advertise()
        time.sleep(ADVERTISE_SETTLE_S)

    def send_pwm_servo(self, servo_id, angle_deg):
        self._pwm.publish(roslibpy.Message(pwm_servo_message(servo_id, angle_deg)))

    def send_twist(self, linear_x, angular_z):
        self._twist.publish(roslibpy.Message(twist_message(linear_x, angular_z)))

    def send_status(self, status_dict):
        self._status.publish(roslibpy.Message({"data": json.dumps(status_dict)}))

    def publishers_of(self, topic, timeout_s=3.0):
        """Nodes registered as publishers of `topic`, straight from rosapi
        (which runs inside the rosbridge process — the honest check for
        the silent-drop registration bug). Raises on no answer; callers
        treat that as 'unverifiable', never as a fault."""
        service = roslibpy.Service(self._ros, ROSAPI_PUBLISHERS_SERVICE,
                                   ROSAPI_PUBLISHERS_TYPE)
        response = service.call(roslibpy.ServiceRequest({"topic": topic}),
                                timeout=timeout_s)
        return list(response["publishers"])

    def on_ai_enabled(self, callback):
        """callback(bool) fires on every /ai/enabled message.

        The dashboard advertises this latched, so if it is already up the
        current armed state arrives right after subscribing. If the AI
        process starts first, nothing arrives until the operator touches
        the ARM switch — RosSink therefore defaults to disarmed.
        """
        self._enabled.subscribe(lambda msg: callback(bool(msg["data"])))

    def disconnect(self):
        """Close the websocket but keep the process's event loop running.

        roslibpy shares one Twisted reactor per process and a reactor can
        never be restarted — terminate() here would make every future
        RosClient in this process fail. Use this between reconnect attempts;
        close() only at process exit.
        """
        for topic in self._pubs:
            try:
                topic.unadvertise()
            except Exception:
                pass
        try:
            self._enabled.unsubscribe()
        except Exception:
            pass
        self._ros.close()

    def close(self):
        for topic in self._pubs:
            try:
                topic.unadvertise()
            except Exception:
                pass
        try:
            self._ros.terminate()
        except Exception:
            # roslibpy teardown is unreliable after a disconnect/reconnect
            # cycle (its event-loop manager may lack a _thread). Only ever
            # called at process exit, so swallowing this is safe.
            pass
