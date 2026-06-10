# Transbot SE — Dashboard Bring-Up Notebook

Project: laptop-based control and telemetry dashboard for a Yahboom Transbot SE
Repository: https://github.com/rbhale-git/transbot_se
Period: June 2026 (bring-up complete 2026-06-10)
Status: all build-brief phases complete; robot in active use

## 1. Project Goal

Replace the vendor phone app with a self-built, laptop-hosted web dashboard that
teleoperates the Transbot SE over Wi-Fi: keyboard/gamepad control of the tracks,
camera gimbal, and 3-DOF arm, with live video and telemetry, engineered around
explicit safety behaviors (deadman, e-stop, clamping, link-loss protection). The
actuation layer was deliberately built input-agnostic so future autonomous
behavior nodes (AI features) can drive the same interfaces without modification.

## 2. Hardware and Base Software

| Item | Detail |
| --- | --- |
| Robot | Yahboom Transbot SE, tracked chassis, 2-axis camera gimbal, 3 bus-servo arm (J7, J8, J9 = gripper) |
| Computer | NVIDIA Jetson Nano B01 (4 GB, aarch64) |
| Robot OS | Ubuntu 18.04.6 LTS, ROS Melodic 1.14.12 (Yahboom factory image) |
| Camera | UVC USB camera on /dev/video0; MJPG up to 1080p30 / 720p60, YUYV 720p limited to 9 fps |
| Laptop | Windows 11, no ROS installation required by design |
| Robot login | user jetson (passwordless sudo enabled on factory image) |

## 3. Architecture

Two-layer ownership contract:

- On the robot, only the Yahboom driver (transbot_driver.py) touches hardware.
  All control is performed by publishing ROS messages to its topics.
- On the laptop, only three publisher modules (motion, gimbal, arm) write to
  control topics. Keyboard, gamepad, sliders, pose buttons, and any future AI
  node are clients of those modules.

Transport between laptop and robot:

- rosbridge_server exposes ROS over WebSocket (port 9090, JSON protocol);
  the dashboard uses roslib.js (vendored locally for offline use).
- web_video_server serves the camera as MJPEG over HTTP (port 8080); the
  dashboard renders it in an aspect-locked viewport.

Data flow summary:

| Direction | Path |
| --- | --- |
| Drive commands | dashboard -> ws:9090 -> rosbridge -> /cmd_vel -> driver -> motors |
| Gimbal | dashboard -> /PWMServo -> driver -> PWM servos |
| Arm | dashboard -> /TargetAngle -> driver -> bus servos |
| Telemetry | driver -> /transbot/get_vel, /voltage, /imu/data -> rosbridge -> dashboard |
| Arm state | dashboard polls /CurrentAngle service every 2 s |
| Video | usb_cam -> /usb_cam/image_raw -> web_video_server -> MJPEG -> img tag |

## 4. Verified Robot Interface (Phase 1 discovery)

All message shapes were verified by live introspection on the robot
(discovery/transbot_discovery.txt, summarized in FINDINGS.md). Code references
these findings, not vendor documentation.

### Control topics

| Topic | Type | Fields / notes |
| --- | --- | --- |
| /cmd_vel | geometry_msgs/Twist | linear.x m/s (driver limit 0.45), angular.z rad/s (limit 2.0) |
| /PWMServo | transbot_msgs/PWMServo | int32 id (1=pan, 2=tilt), int32 angle (0-180) |
| /TargetAngle | transbot_msgs/Arm | Joint[] joint; Joint = int32 id, int32 run_time, float32 angle |

### Telemetry

| Topic | Type | Notes |
| --- | --- | --- |
| /transbot/get_vel | geometry_msgs/Twist | measured velocity |
| /voltage | transbot_msgs/Battery | single field: float32 Voltage (capital V) |
| /imu/data | sensor_msgs/Imu | madgwick-filtered (raw alternative: /transbot/imu) |
| /odom | nav_msgs/Odometry | EKF-fused; available for future navigation |
| /usb_cam/camera_info | sensor_msgs/CameraInfo | one per captured frame; used as FPS meter |

### Services

| Service | Request | Response |
| --- | --- | --- |
| /CurrentAngle (transbot_msgs/RobotArm) | string apply | RobotArm.joint[] of id, run_time, angle |
| /Buzzer, /Headlight, /RGBLight | int32 args | bool result (unused; future features) |

### Differences vs vendor documentation discovered

- Battery topic is /voltage with field Voltage, not a std_msgs type.
- Camera topic under the factory stack was /image, not /usb_cam/image_raw.
- /transbot/imu carries no orientation and g-unit accelerations; /imu/data used instead.
- The factory app autostarts the whole stack at boot; bringup.launch must never
  be launched manually alongside it (node-name collision kills both drivers).

## 5. Robot-Side Stack (replaces factory autostart)

The factory autostart (desktop-login entry running transbot_main.pyc) held the
camera device exclusively and supervised its own stack. With user approval it
was parked in ~/factory_autostart_disabled/ (reversible by moving bash.desktop
back to ~/.config/autostart/). The robot now boots our stack headless:

systemd unit rosbridge-dashboard.service (enabled, Restart=on-failure) runs
~/transbot_dashboard/start_dashboard.sh, which sources the workspace chain
(/opt/ros/melodic, ~/software/transbot_library/devel, ~/transbot_ws/devel) and
launches transbot_dashboard.launch:

1. transbot_bringup/bringup.launch — driver, DeviceSrv, IMU calib + filter, EKF, state publishers
2. usb_cam — /dev/video0, 1280x720, MJPG, 30 fps (YUYV cannot do 720p30)
3. web_video_server — MJPEG on :8080
4. rosbridge_websocket — WebSocket on :9090
5. cmd_vel_watchdog.py — SAFETY: stops the robot on command silence (section 8)

Also on the robot:

- yb-transbot_oled.service (factory, untouched) — OLED shows current IP.
- wifi-failover.timer + wifi_failover.sh — every 60 s, if NO Wi-Fi link is
  active: try home_wifi (if SSID visible), else start the Transbot hotspot AP.
  Never preempts an active connection (cannot drop a live teleop session).
- Calibration file ~/.ros/camera_info/head_camera.yaml updated to 1280x720
  (camera_info reports calibration dimensions, not capture dimensions).

## 6. Networking

| Profile | Details |
| --- | --- |
| HOME WIFI (primary/dev) | robot joins TP-Link_8A2B_5G as client; DHCP address 192.168.0.109; laptop and robot share LAN and internet |
| ROBOT HOTSPOT (field fallback) | robot is AP "Transbot" (WPA2, password on the unit); robot = gateway 192.168.1.11; fully offline |

NetworkManager profile priorities: home_wifi autoconnect priority 10, Transbot
AP priority 0; the failover timer guarantees recovery if neither connects.
Switching the dashboard between networks is a NET profile dropdown selection.

Manual switching on the robot:
sudo nmcli con up home_wifi / sudo nmcli con up Transbot

Operational gotchas (verified the hard way):

- A VPN on the laptop (NordVPN/NordLynx) breaks or resets connections to the
  robot. Exit the VPN or enable its allow-LAN option during robot work.
- Joining the robot hotspot removes laptop internet (and any cloud tooling).
- The hotspot subnet (192.168.1.x) is distinct from the home subnet (192.168.0.x).

## 7. SSH and Remote Administration

- ed25519 keypair generated on the laptop; public key installed into
  ~/.ssh/authorized_keys on the robot; passwordless login verified.
- All robot administration is performed over SSH (apt installs, systemd units,
  file deployment via scp, reboots).
- Robot-side fixes performed during bring-up: system clock was stuck in Aug 2021
  (no RTC battery; broke apt validity checks - fixed by setting date + NTP),
  expired ROS repository GPG key (new key pushed from laptop; robot cannot
  reach GitHub), corrupted apt list cache (cleared and refreshed).

## 8. Safety Engineering

Layered, defense-in-depth model:

| Layer | Mechanism |
| --- | --- |
| Value safety | Every outgoing value clamps to verified ranges; motion clamps twice (operating cap, then hard driver limit). Caps live-tunable in SETTINGS, never above hard limits |
| Deadman (input) | Hold-to-move only; key release publishes a zero-Twist burst (x3) |
| Deadman (focus) | Tab blur / page hide releases all keys and stops |
| Deadman (link) | rosbridge disconnect clears held state; reconnection never resumes prior motion |
| E-stop | SPACE, registered in capture phase, works from any focus, highest priority; zero-burst + cancels pending arm sequences; also gamepad B |
| Robot-side watchdog | cmd_vel_watchdog node: if /cmd_vel is silent 0.5 s while the last command was non-zero, publishes zero-Twist burst. Protects against silent link degradation, browser crash, laptop sleep - independent of the dashboard |
| Arm gentleness | Every arm command carries run_time (800 ms default) so joints never slam; no flash/persistent parameter writes anywhere |

Field incident that drove the watchdog: the original disconnect test (clean
Wi-Fi off) passed, but a degrading link in real use is a silent TCP blackhole -
rosbridge kept believing a client was attached and the driver executed the last
command indefinitely. Lesson recorded: a safety property that depends on the
remote end is not a safety property; the robot must protect itself.

## 9. Dashboard Features

Mission-console UI (dark instrument aesthetic, Chakra Petch + JetBrains Mono,
fonts vendored for offline use). Served by tools/serve_dashboard.py
(Cache-Control: no-store - plain http.server allowed stale ES-module mixes that
killed the whole import graph).

### Panels and HUD

| Element | Content |
| --- | --- |
| Camera stage | aspect-locked to the true stream ratio; HUD overlays: commanded LIN/ANG (top-left), REC marker, IMU ACC/GYR (top-right, dims when stale), FPS + capture resolution (bottom-left, from camera_info) |
| DRIVE panel | commanded vs measured linear/angular with center-zero gauges; stale dimming |
| GIMBAL panel | 2D position map (live dot), per-axis slider + numeric entry, GIMBAL HOME button |
| ARM panel | three sliders + numeric entries, HOME / READY / STOW buttons, live /CurrentAngle readout |
| SETTINGS panel | linear/angular caps, gimbal/arm step sizes, arm run_time; persisted in localStorage; RESET DEFAULTS button |
| Header | NET profile selector, rosbridge URL, PAD indicator, REC/SAVE (session recorder -> JSONL), battery widget (voltage, segmented bar, LOW alert), link lamp + RTT (rosapi/get_time round trip) |

### Input bindings

| Input | Action |
| --- | --- |
| W/S, A/D (hold) | drive fwd/back, rotate; diagonals combine; 10 Hz republish |
| SPACE | e-stop |
| Arrow keys / C | gimbal step / gimbal home |
| U/J, I/K, O/L | arm J7 / J8 / J9 (gripper) step |
| H | arm home pose |
| Gamepad | left stick drive (analog), right stick gimbal, LT/RT gripper, D-pad J7/J8, B e-stop, A home; stick release = stop; pad disconnect = stop |

Typing in any input field never triggers robot keys (except SPACE e-stop);
all controls blur after use so driving keys are never captured.

### Operator-tuned poses (config.js is the single source of truth)

| Pose | Values | Behavior |
| --- | --- | --- |
| Gimbal home | pan 90, tilt 22 | C key or button |
| Arm HOME | J7 225, J8 30, J9 30 | all joints including gripper |
| Arm READY | J7 110, J8 175, J9 30 | staged: J8 leads, J7+J9 start after ~5 deg of J8 travel |
| Arm STOW | J7 225, J8 70 | gripper (J9) untouched - keeps its grip |

Pose reliability: the vendor driver drops /TargetAngle commands that arrive
back-to-back (subscriber queue of 1 + per-servo serial writes). All multi-joint
poses therefore send one command per joint paced 150 ms apart
(interCommandMs), then poll /CurrentAngle once and re-send any joint more than
8 deg off target (verifyToleranceDeg). Sequences abort on e-stop or when
superseded.

## 10. Test and Verification Record

Phase 4 checklist (robot on stand) - all PASS: drive + clamps, deadman on key
release / tab blur, e-stop incl. from input focus, gimbal/arm controls and
poses, stop-on-disconnect (clean), reconnect without motion resume. Amended
post-pass with the field watchdog fix (section 8). Detailed table in README.

Standing verification tooling:

| Tool | Purpose |
| --- | --- |
| mock/mock_rosbridge.py | full robot simulator speaking the rosbridge protocol; mirrors verified message shapes; exercises telemetry, battery drain, /CurrentAngle |
| mock/selftest.py | 5-check protocol test against the mock |
| tools/probe_rosbridge.py | read-only end-to-end probe of the live robot (RTT, battery, vel, IMU, arm service) |
| tools/smoke_boot.mjs | boots the entire dashboard module graph in Node with a strict DOM shim (only real HTML ids resolve) - catches missing-element crashes without a browser |
| Session recorder | in-dashboard capture of all topic/service traffic to timestamped JSONL |

## 11. Repository Layout

| Path | Contents |
| --- | --- |
| dashboard/ | static web app: index.html, css/, js/ (config, ros, keyboard, gamepad, telemetry, video, settings, latency, recorder, publishers/motion-gimbal-arm), vendored roslib + fonts |
| robot/ | everything deployed to the Jetson: transbot_dashboard.launch, start_dashboard.sh, rosbridge-dashboard.service, cmd_vel_watchdog.py, wifi_failover.sh + service/timer |
| mock/ | mock rosbridge server + selftest |
| tools/ | setup_ssh.ps1, run_discovery.ps1, deploy_robot.ps1, robot_join_wifi.sh, discovery.sh, probe_rosbridge.py, serve_dashboard.py, smoke_boot.mjs |
| discovery/ + FINDINGS.md | raw introspection dump and the verified interface document |
| README.md | architecture diagram, quick start, bindings, config reference, Phase 4 results, Extending-this guide |

## 12. Incidents and Lessons Learned

| Incident | Root cause | Lesson |
| --- | --- | --- |
| bringup.launch killed the running robot stack | factory app autostarts the driver; duplicate node names | discover what already runs before launching anything |
| Dashboard camera pane dead | factory app held /dev/video0 exclusively | device ownership matters; replaced autostart with our own stack |
| Arm poses moved only one joint sometimes | driver subscriber queue of 1 drops back-to-back commands | pace commands; verify and repair (closed-loop over open-loop trust) |
| "Nothing works" after an update | browser cached a stale ES module; one bad import kills the whole graph | serve with no-store; smoke-test the module graph |
| Robot ran away on link loss | silent TCP blackhole vs clean close; driver executes last command forever | safety must live on the robot (cmd_vel watchdog); test the ugly failure mode, not the clean one |
| camera_info reported 640x480 at 720p capture | CameraInfo comes from the calibration file, not the sensor | distinguish measured values from declared values |
| apt refused all packages | clock stuck in 2021 (no RTC), expired ROS GPG key, corrupt lists | embedded images rot; check time first |
| SSH sessions resetting on the hotspot | NordVPN killing no-internet networks; Windows auto-switching Wi-Fi | rule out the network layer before debugging the application |
| ssh-keygen "too many arguments" | PowerShell 5.1 drops empty-string args to native exes | platform quirks deserve regression notes |

## 13. Forward Plan (AI features)

Agreed direction: AI behaviors run on the laptop first (modern Python 3, fast
iteration), consuming the MJPEG stream via OpenCV and publishing commands via
roslibpy over the existing rosbridge - the same publisher contract as the
dashboard, already protected by the robot-side watchdog. Before first
autonomous motion: a command-priority mux (e-stop > manual > AI, AI with its
own lower speed cap) and a dashboard AI arm/disarm panel. Staged roadmap:

1. Face detection + gimbal tracking (no chassis motion; teaches the perceive-error-command loop)
2. Person following (proportional control of angular/linear from bbox; lost target = stop)
3. Marker-based waypoint navigation; evaluate lidar purchase for true SLAM/nav
4. Autonomous pick and place: visual servo of chassis + calibrated grasp sequences built on the existing READY/STOW pose primitives

End of notebook.
