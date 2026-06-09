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

- [x] Phase 0 — repo and SSH setup
- [ ] Phase 1 — interface discovery on the robot (`FINDINGS.md`)
- [ ] Phase 2 — robot-side setup (rosbridge + web_video_server + launch file)
- [ ] Phase 3 — web dashboard
- [ ] Phase 4 — test and safety pass
- [ ] Phase 5 — docs

See `transbot_dashboard_build_prompt.md` for the full build brief.
