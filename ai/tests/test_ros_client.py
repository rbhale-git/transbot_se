"""Tests for ai.common.ros_client message building.

The wire shape was verified on the live robot (FINDINGS.md):
transbot_msgs/PWMServo = {int32 id, int32 angle}.
Connection handling itself is exercised live, not unit-tested.
"""

from ai.common.ros_client import PWM_SERVO_TOPIC, PWM_SERVO_TYPE, pwm_servo_message


class TestPwmServoMessage:
    def test_builds_verified_wire_shape(self):
        assert pwm_servo_message(1, 90) == {"id": 1, "angle": 90}

    def test_rounds_angle_to_int(self):
        msg = pwm_servo_message(2, 21.6)
        assert msg["angle"] == 22
        assert isinstance(msg["angle"], int)

    def test_topic_constants_match_robot(self):
        assert PWM_SERVO_TOPIC == "/PWMServo"
        assert PWM_SERVO_TYPE == "transbot_msgs/PWMServo"
