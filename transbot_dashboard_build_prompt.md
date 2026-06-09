# Build Brief: Transbot SE Laptop Control Dashboard (v1)

## Your role
You are building a laptop-based control and telemetry dashboard for a Yahboom Transbot SE robot. Work phase by phase. Confirm the spec and your plan before you start, show results and commit after each phase, and pause for my go-ahead before the robot physically moves for the first time.

## Environment (verify, do not assume)
- **Laptop:** Windows 10/11. You (Claude Code) run here. Use built-in OpenSSH for remote work.
- **Robot:** Yahboom Transbot SE on an NVIDIA Jetson Nano B01, running ROS Melodic on Ubuntu 18.04 (Yahboom factory image). Catkin workspace at `~/transbot_ws`. Relevant packages: `transbot_bringup`, `transbot_ctrl`, `transbot_se_moveit_config`. The bringup is `roslaunch transbot_bringup bringup.launch`.
- **Network:** the robot is on the same Wi-Fi as the laptop. SSH from laptop to robot is NOT set up yet. Ask me for the Jetson IP and login interactively. Do not write credentials into any file or commit them.

## Architecture (locked, do not redesign)
- The robot already runs a ROS driver node (`transbot_driver.py`, started by `bringup.launch`) that owns all hardware. Never bypass it and never call the `Transbot_Lib` Python library directly. All control is done by publishing to existing ROS topics.
- **Bridge:** run `rosbridge_server` on the Jetson to expose ROS topics over WebSocket (`ws://<jetson-ip>:9090`). Run `web_video_server` on the Jetson to serve the camera as MJPEG over HTTP (default port 8080).
- **Client:** a static web dashboard (HTML, CSS, JS, and roslib.js) served locally on the laptop. It connects to rosbridge over WebSocket for control and telemetry, and embeds the MJPEG stream for video. The laptop needs no ROS install.

## Known interface (from vendor docs, treat as a hint, VERIFY in Phase 1)
Topics the driver subscribes to:
- `/cmd_vel` (`geometry_msgs/Twist`): track motion. `linear.x` in m/s within [-0.45, 0.45], `angular.z` in rad/s within [-2.0, 2.0].
- `/PWMServo` (custom msg): camera gimbal. Servo id X = 1, Y = 2. Angle within [0, 180].
- `/TargetAngle` (custom msg): 3-DOF arm. Bus servos 7 within [0, 225], 8 within [30, 270], 9 within [30, 180] where 9 is reported as the gripper. Optional run_time in ms.

Topics and services the driver publishes:
- `/transbot/get_vel`: measured linear and angular velocity.
- `/transbot/imu`: IMU data.
- A battery voltage feedback topic (find its exact name).
- `/CurrentAngle`: a service to read current arm joint angles.

Camera image topic: unknown, likely `/usb_cam/image_raw` or similar. Discover it.

The exact message package, type strings, and field names for `/PWMServo` and `/TargetAngle` are NOT confirmed. Discover them with live introspection and generate code from what you find, not from this list. If discovery contradicts anything above, trust discovery and tell me what differed.

## Features (v1)
**Control**
- **Motion (WASD):** W and S = forward and back, A and D = rotate left and right. Hold to move, release to stop (deadman behavior). Allow diagonal combinations such as W plus A. Spacebar is an immediate e-stop that publishes a zero Twist.
- **Gimbal (arrow keys):** Left and Right step servo X, Up and Down step servo Y, each clamped to [0, 180]. Include a key to recenter the gimbal.
- **Arm (keyboard, non-conflicting keys):** per-joint increment keys for the arm servos, a gripper open/close toggle, and a home/reset key that sends a safe neutral pose. Suggested bindings that avoid WASD and the arrow keys: U/J, I/K, O/L for the three arm servos, G to toggle the gripper, H for arm home. Confirm via discovery which servo is the gripper and document the final bindings.

**Display**
- Live camera pane from the MJPEG stream.
- Telemetry panels: commanded versus measured speed, IMU readout, battery voltage with a low-voltage warning, and live arm joint angles polled from `/CurrentAngle`.
- A clear rosbridge connection status indicator that shows when the link drops.

## Code standards
- **Config-driven:** one config module holds all topic names, message type strings, key bindings, speed limits, servo ranges, the rosbridge URL, and the video URL. No magic values scattered through the logic.
- **Modular publishers:** separate modules for motion, gimbal, and arm. Each exposes a clean function (for example `publishMotion(linear, angular)`). The keyboard handler calls these functions, and so will a future AI node or external device.
- **Input-agnostic actuation:** the dashboard is just one publisher. Document clearly where a future autonomous behavior node or an external arm-teleoperation device (an arm-tracking glove with encoders) would publish in order to drive the same topics, with no change to the actuation layer.
- Vanilla JS plus roslib.js is fine for v1. No heavy framework required. Keep it readable and commented.

## Safety (required)
- Clamp every outgoing value to the discovered valid range before publishing. Never exceed a documented range.
- Start conservative. Cap motion well below the maximum (for example 0.2 m/s and 1.0 rad/s) with the full range available in config so I can raise it.
- Deadman behavior: motion must stop on key release, on browser tab blur or focus loss, and on rosbridge disconnect.
- The spacebar e-stop must be the highest-priority handler.
- Never set servo, PID, or speed parameters with permanent or flash storage, and never call any flash-reset routine. Use a moderate run_time on arm moves so joints do not slam.
- Before the first command that physically moves the robot, stop and tell me exactly what you are about to send, then wait for my go-ahead.

## Phases (commit to GitHub after each one)
**Phase 0, repo and SSH**
- Create a new GitHub repo (ask me for the name; my GitHub username is rbhale-git). Initialize with a README and a sensible .gitignore.
- Set up SSH from this Windows laptop to the Jetson using OpenSSH: generate an ed25519 key, install the public key into the Jetson's `~/.ssh/authorized_keys`, and verify passwordless login. Ask me for the Jetson IP and login interactively. Keep credentials out of all files and commits.

**Phase 1, discovery (read-only on the robot)**
- SSH in, source the workspace, and start the bringup. Enumerate the topics, services, and message types for motion, gimbal, arm, IMU, odometry, battery, and camera. Capture exact message type strings and field layouts using `rostopic info`, `rosmsg show`, and `rossrv show`. Identify the camera image topic. Confirm whether `rosbridge_server` and `web_video_server` are installed.
- Write `FINDINGS.md` (committed) documenting the verified interface. All later code must reference this, not the hint list above.

**Phase 2, robot-side setup**
- If missing, install `ros-melodic-rosbridge-suite` and `ros-melodic-web-video-server` (ask me before installing anything on the robot). Create a single launch file that starts the driver, `rosbridge_server`, and `web_video_server` together, plus a one-command start script. Confirm rosbridge serves on port 9090 and the MJPEG stream is viewable in a browser.

**Phase 3, web dashboard**
- Build the static site per Features and Code standards. Serve it locally (for example `python -m http.server` or `npx serve`). Implement the config module, the three publisher modules, keyboard handling with deadman and e-stop, the video pane, the telemetry subscriptions, `/CurrentAngle` polling, and the connection status indicator.

**Phase 4, test and safety pass**
- With the robot on a stand or with its tracks clear of the ground for the first motion test, verify every control and every safety behavior (range clamps, deadman, e-stop, stop-on-disconnect). Record a short test checklist and the results in the README.

**Phase 5, docs**
- README with an architecture diagram (mermaid or text), setup and run instructions, the final key bindings, a config reference, and an "Extending this" section explaining how an AI behavior node and an external arm-teleoperation device plug into the same topics.

## Working style
- Confirm your understanding of this brief and lay out your phase plan before starting Phase 0.
- Work one phase at a time. Show me results and commit before moving to the next.
- Ask before anything that installs software on the robot, and before the robot first moves.
- If anything you discover contradicts this brief, trust what you find on the actual robot and flag the difference.
