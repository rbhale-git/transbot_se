# FINDINGS — Verified Robot Interface (Phase 1)

Source: live introspection on the robot, 2026-06-09 (`discovery/transbot_discovery.txt`).
ROS Melodic 1.14.12, Ubuntu 18.04.6, Jetson Nano (aarch64), hostname `Transbot`.
**All dashboard code references this document, not the vendor-doc hints in the brief.**

## Control topics (the dashboard publishes these)

| Topic | Type | Fields | Notes |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` m/s, `angular.z` rad/s | subscribed by `/transbot_node` (the driver). Driver-side limits confirmed by factory joy params: linear 0.45, angular 2.0 |
| `/PWMServo` | `transbot_msgs/PWMServo` | `int32 id`, `int32 angle` | gimbal; id 1 = pan (X), id 2 = tilt (Y) |
| `/TargetAngle` | `transbot_msgs/Arm` | `Joint[] joint`; Joint = `int32 id`, `int32 run_time`, `float32 angle` | arm; servos 7/8/9 |

Both custom message shapes match what the dashboard's builders assumed — no
code changes were needed in `buildPwmServoMessage()` / `buildArmMessage()`.

## Telemetry topics (the dashboard subscribes to these)

| Topic | Type | Notes |
| --- | --- | --- |
| `/transbot/get_vel` | `geometry_msgs/Twist` | measured velocity from the driver |
| `/voltage` | `transbot_msgs/Battery` | single field `float32 Voltage` (capital V). Sample read: 11.6 V |
| `/imu/data` | `sensor_msgs/Imu` | **filtered** (madgwick) — has real orientation, used by the dashboard |
| `/transbot/imu` | `sensor_msgs/Imu` | raw driver IMU — orientation zeros, accel in g-units; kept as alternative |
| `/odom` | `nav_msgs/Odometry` | EKF-fused odometry, available for future use |

## Services

| Service | Type | Request | Response |
| --- | --- | --- | --- |
| `/CurrentAngle` | `transbot_msgs/RobotArm` | `string apply` | `RobotArm.joint[]` of `{id, run_time, angle}` |
| `/Buzzer` | `transbot_msgs/Buzzer` | `int32 buzzer` | `bool result` — future feature |
| `/Headlight` | `transbot_msgs/Headlight` | `int32 Headlight` | `bool result` — future feature |
| `/RGBLight` | `transbot_msgs/RGBLight` | `int32 effect, int32 speed` | `bool result` — future feature |
| `/CamDevice` | `transbot_msgs/CamDevice` | `string GetGev` | `string camDevice` |

## Camera

Image pipeline: the factory app (`/BigTransbot`) publishes `/image_msg`
(custom packed image) → `/msgToimg` converts → **`/image`**
(`sensor_msgs/Image`). `web_video_server` is installed AND already running in
the factory stack, so the MJPEG URL is:

    http://<robot-ip>:8080/stream?topic=/image&type=mjpeg

Caveat to verify during Phase 4: `/image` may only flow while the factory app
has the camera open. If the pane is black, check
`http://<robot-ip>:8080/` for the list of available streams.

## Factory stack / autostart (critical for Phase 2)

- Boot chain: desktop autologin → `~/.config/autostart/bash.desktop` →
  `~/Transbot/transbot/start_transbot.sh` → gnome-terminal running
  `transbot_main.pyc` → which itself spawns
  `roslaunch transbot_program Transbot_Program.launch` (the full stack:
  driver, EKF, IMU filter, joy, patrol, vision nodes, web_video_server).
- Takes ~1–2 minutes after power-on (needs the GUI session).
- A separate systemd unit `yb-transbot_oled.service` runs the OLED display.
- **Never launch `bringup.launch` manually** — the factory watchdog restarts
  its own driver and the name collision kills both.
- The driver (`transbot_node`) and `web_video_server` are already running at
  boot. **Phase 2 therefore only needs to add `rosbridge_server`.**

## Installation status

| Package | Status |
| --- | --- |
| `ros-melodic-web-video-server` | installed (0.2.1) and running at boot |
| `ros-melodic-rosbridge-suite` | **NOT installed** — the one Phase 2 install, needs robot internet or offline .debs |

## Differences vs. the brief's hint list

1. The battery topic is `/voltage` (`transbot_msgs/Battery.Voltage`) — the
   brief said "find its exact name"; found.
2. `/TargetAngle`'s type is `transbot_msgs/Arm` with a `Joint[]` array
   (`angle` is float32) — consistent with the hint's intent.
3. The camera topic is `/image`, not `/usb_cam/image_raw`.
4. The dashboard uses `/imu/data` (filtered) rather than `/transbot/imu`
   (raw): the raw topic has no orientation and g-unit accelerations.
5. The driver was assumed to be started by us via `bringup.launch`; in
   reality the factory app autostarts everything at boot, including
   web_video_server. Phase 2 scope shrank to rosbridge only.
