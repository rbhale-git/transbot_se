"""Thin roslibpy wrapper for AI behaviors.

Wire shapes were verified against the live robot (FINDINGS.md). This module
owns the websocket connection and topic advertising; behaviors only ever
hand it plain dicts built by the pure helpers below (which is what the unit
tests cover — the connection path is exercised live).
"""

import time

import roslibpy

PWM_SERVO_TOPIC = "/PWMServo"
PWM_SERVO_TYPE = "transbot_msgs/PWMServo"

# Pause between advertising and first publish. The robot's old rosbridge
# (0.11/Melodic) can hit "Internal error processing topic" if its publisher
# registration races traffic — seen live 2026-06-10, where it silently
# dropped every command for that session (docs/AI_NOTEBOOK.md).
ADVERTISE_SETTLE_S = 1.0


def pwm_servo_message(servo_id, angle_deg):
    """Verified shape: transbot_msgs/PWMServo = {int32 id, int32 angle}."""
    return {"id": int(servo_id), "angle": int(round(angle_deg))}


class RosClient:
    """Connection + publishers for one AI behavior process."""

    def __init__(self, url):
        # roslibpy wants host/port split; accept the dashboard-style ws:// URL.
        host, port = self._parse(url)
        self._ros = roslibpy.Ros(host=host, port=port)
        self._pwm = roslibpy.Topic(self._ros, PWM_SERVO_TOPIC, PWM_SERVO_TYPE)

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
        self._pwm.advertise()
        time.sleep(ADVERTISE_SETTLE_S)

    def send_pwm_servo(self, servo_id, angle_deg):
        self._pwm.publish(roslibpy.Message(pwm_servo_message(servo_id, angle_deg)))

    def disconnect(self):
        """Close the websocket but keep the process's event loop running.

        roslibpy shares one Twisted reactor per process and a reactor can
        never be restarted — terminate() here would make every future
        RosClient in this process fail. Use this between reconnect attempts;
        close() only at process exit.
        """
        try:
            self._pwm.unadvertise()
        except Exception:
            pass
        self._ros.close()

    def close(self):
        try:
            self._pwm.unadvertise()
        except Exception:
            pass
        try:
            self._ros.terminate()
        except Exception:
            # roslibpy teardown is unreliable after a disconnect/reconnect
            # cycle (its event-loop manager may lack a _thread). Only ever
            # called at process exit, so swallowing this is safe.
            pass
