# Transbot SE Laptop Control Dashboard

A laptop-based control and telemetry web dashboard for the Yahboom Transbot SE
robot (Jetson Nano B01, ROS Melodic).

## Architecture (v1)

- **Robot (Jetson Nano):** the stock Yahboom ROS driver owns all hardware.
  `rosbridge_server` exposes ROS topics over WebSocket (port 9090) and
  `web_video_server` serves the camera as MJPEG over HTTP (port 8080).
- **Laptop:** a static web dashboard (HTML/CSS/JS + roslib.js) served locally.
  It publishes control topics and subscribes to telemetry via rosbridge, and
  embeds the MJPEG stream. No ROS install needed on the laptop.

## Network profiles

The robot is reachable two ways, and both are first-class:

- **Home Wi-Fi (primary, used for development):** the Jetson joins the same
  Wi-Fi network as the laptop in client mode. Both devices keep internet
  access, which is required for installing packages on the robot and for
  development on the laptop.
- **Robot hotspot (fallback, for use away from any Wi-Fi):** the laptop joins
  the robot's own "transbot" access point. Fully offline operation; the laptop
  has no internet while connected.

The dashboard config holds a named profile for each network (rosbridge URL and
video URL per profile). Switching networks is a single profile selection — no
code changes. The robot-side launch is identical on both networks.

## Status

- [x] Phase 0 (partial) — repo created; SSH setup pending robot availability
- [ ] Phase 1 — interface discovery on the robot (`FINDINGS.md`)
- [ ] Phase 2 — robot-side setup (rosbridge + web_video_server + launch file)
- [x] Phase 3 (against mock) — dashboard built and tested against a local mock
      rosbridge; custom message types marked TO-VERIFY until Phase 1
- [ ] Phase 4 — test and safety pass on the real robot
- [ ] Phase 5 — docs

## Running the dashboard (mock mode, no robot needed)

Two terminals from the repo root:

```powershell
# 1. mock rosbridge server (simulated driver telemetry on ws://localhost:9090)
python mock/mock_rosbridge.py

# 2. static server for the dashboard
python -m http.server 8000 --directory dashboard
```

Open http://localhost:8000 and make sure the NET profile is `MOCK (local sim)`.
The mock logs every command the dashboard sends, simulates measured velocity
(echoing `/cmd_vel`, decaying to zero when commands stop), IMU noise, a
draining battery (to exercise the low-voltage warning), and a `/CurrentAngle`
service that follows `/TargetAngle` commands.

Protocol self-test for the mock: `python mock/selftest.py` (mock must be running).

### Key bindings (v1, also shown in the dashboard footer)

| Keys | Action |
| --- | --- |
| W / S (hold) | drive forward / back |
| A / D (hold) | rotate left / right (combinable with W/S) |
| SPACE | e-stop — immediate zero Twist, highest priority |
| Arrow keys | gimbal pan/tilt step, C recenter |
| U/J, I/K, O/L | arm joints 7 / 8 / 9 step |
| G | gripper toggle, H = arm home pose |

Deadman behavior: motion stops on key release, on tab blur or page hide, and
on rosbridge disconnect.

See `transbot_dashboard_build_prompt.md` for the full build brief.
