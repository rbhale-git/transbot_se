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

```mermaid
flowchart LR
    subgraph Laptop["Laptop (Windows, no ROS)"]
        KB["keyboard.js<br/>deadman + e-stop"]
        GP["gamepad.js"]
        UI["panels / sliders<br/>settings / recorder"]
        PUB["publisher modules<br/>motion · gimbal · arm<br/>(clamp everything)"]
        ROSJS["ros.js<br/>rosbridge client"]
        VID["video pane"]
        KB --> PUB
        GP --> PUB
        UI --> PUB
        PUB --> ROSJS
    end

    subgraph Jetson["Jetson Nano (ROS Melodic)"]
        RB["rosbridge_server<br/>ws :9090"]
        WVS["web_video_server<br/>http :8080"]
        DRV["transbot_driver.py<br/>owns ALL hardware"]
        CAM["camera node"]
        HW["motors · servos · IMU · battery"]
        RB -- "/cmd_vel<br/>/PWMServo<br/>/TargetAngle" --> DRV
        DRV -- "/transbot/get_vel<br/>/transbot/imu<br/>battery topic<br/>/CurrentAngle (srv)" --> RB
        DRV --- HW
        CAM --> WVS
    end

    ROSJS <-- "WebSocket :9090" --> RB
    VID -- "MJPEG :8080" --> WVS
```

The contract that makes this safe and extensible: **only `transbot_driver.py`
touches hardware**, and **only the publisher modules talk to the control
topics** from the laptop side. Everything else — keyboard, gamepad, panels,
and any future input — is a client of those two layers.

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
| U/J, I/K, O/L | arm joints 7 / 8 / 9 step (J9 is the gripper joint) |
| H | arm home pose |

The gimbal and arm panels also have a slider plus a numeric entry field per
axis (type a value and press Enter). All five axes — pan, tilt, J7, J8, and
the gripper joint J9 — are continuous; ranges come from `config.js` and every
value is clamped before publishing. Keyboard steps, sliders, and typed values
all stay in sync.

### Gamepad (standard-mapping controller, e.g. Xbox)

| Control | Action |
| --- | --- |
| Left stick | drive — analog, partial deflection = partial speed |
| Right stick | gimbal pan/tilt |
| LT / RT | gripper joint J9 close / open |
| D-pad | J7 (left/right), J8 (up/down) |
| B | e-stop (same path as spacebar) |
| A | arm home pose |

Stick release stops the robot (deadman parity with the keyboard); a
disconnecting pad sends an immediate stop. The PAD indicator in the header
lights when a controller is detected (press any button to wake it).

### Other dashboard tools

- **SETTINGS panel** — live-tune the speed caps (never above the hard driver
  limits), gimbal/arm step sizes, and arm `run_time`. Persisted across
  reloads in localStorage.
- **RTT readout** — round-trip latency over rosbridge (via `/rosapi/get_time`),
  next to the link lamp; turns amber above 250 ms.
- **Session recorder** — ● REC in the header captures every outgoing command,
  telemetry message, and service response with timestamps; SAVE downloads a
  JSONL file. This is the evidence trail for the Phase 4 safety pass.

## Robot-day tooling (Phases 0–2, prepared in advance)

```powershell
.\tools\setup_ssh.ps1     -JetsonIp <ip> -User <user>  # Phase 0: key install + verify
.\tools\run_discovery.ps1 -JetsonIp <ip> -User <user>  # Phase 1: read-only discovery
.\tools\deploy_robot.ps1  -JetsonIp <ip> -User <user>  # Phase 2: push launch + start script
```

`robot/transbot_dashboard.launch` starts the driver bringup, rosbridge
(:9090), and web_video_server (:8080) together; `robot/start_dashboard.sh` is
the one-command robot-side entry point. Package installs on the robot are
deliberately not scripted — they happen only after an explicit go-ahead.

## Config reference (`dashboard/js/config.js`)

Everything tunable lives in one module. Entries marked **TO-VERIFY** are
vendor-doc hints that Phase 1 discovery will confirm or correct; all code
reads them from config, so fixes land in one place.

| Block | Holds | Notes |
| --- | --- | --- |
| `PROFILES` | rosbridge + video URL per network | `mock`, `home` (192.168.1.11), `hotspot` (IP TO-VERIFY) |
| `TOPICS` | name + message type per topic | `/cmd_vel` verified (ROS standard); `/PWMServo`, `/TargetAngle`, telemetry topics TO-VERIFY |
| `SERVICES` | `/CurrentAngle` (TO-VERIFY), `/rosapi/get_time` | latter ships with rosbridge |
| `MOTION` | `hardMax` (driver limits, never exceeded) and `cap` (operating limit) per axis | caps are live-tunable in SETTINGS, clamped to `hardMax` |
| `GIMBAL` / `ARM` | servo ids, ranges, home angles, step sizes, arm `run_time` | ranges from the brief, TO-VERIFY |
| `POWER` | low-voltage warning threshold | 9.9 V for the 3S pack, TO-VERIFY |
| `KEYS` | every key binding (`event.code` values) | legend renders from this, so docs can't drift |
| `GAMEPAD` | deadzone, rates, button mapping | standard-mapping layout |
| `TELEMETRY` / `RECONNECT` | poll intervals, staleness window, backoff | — |

Message-shape assumptions for the two custom topics are isolated in
`buildPwmServoMessage()` (gimbal.js) and `buildArmMessage()` (arm.js) — the
only two functions to edit after discovery.

## Extending this

The dashboard is deliberately just *one publisher among potential many*. The
driver doesn't know or care who publishes; anything that writes valid,
range-clamped messages to the control topics drives the robot identically.

**Topic contract (verify exact types against `FINDINGS.md` after Phase 1):**

| Topic | Type | Carries |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` m/s, `angular.z` rad/s |
| `/PWMServo` | custom (TO-VERIFY) | gimbal servo id (1=pan, 2=tilt) + angle 0–180° |
| `/TargetAngle` | custom (TO-VERIFY) | arm joint id (7/8/9) + angle + `run_time` ms |

**An AI behavior node** (e.g. person-following, line tracking) should run on
the Jetson as a native ROS node (rospy/roscpp) and publish the topics above
directly — no rosbridge hop, lowest latency. It can subscribe to the same
telemetry (`/transbot/get_vel`, IMU, camera) the dashboard uses. The dashboard
keeps working as a monitor while the node drives; the spacebar e-stop still
publishes zero Twists, but note ROS does not arbitrate between publishers —
a safety-minded behavior node should subscribe to a mute/e-stop signal or be
stopped before manual override matters.

**An external arm-teleoperation device** (the encoder glove idea) has two
clean integration points:

1. *Through the dashboard (quickest):* feed device readings to
   `setArmJoint('j7'|'j8'|'j9', angleDeg)` and `setGimbalX/Y()` — e.g. from a
   small WebSocket/serial bridge page-side. Clamping, `run_time`, and the
   recorder come for free, exactly like the gamepad integration did.
2. *As its own publisher:* a standalone process (any language) connects to
   `ws://<jetson>:9090` with a rosbridge client (roslibpy, roslibjs) — or runs
   as a native ROS node on the Jetson — and publishes `/TargetAngle` itself.
   It must then own its safety duties: clamp to the discovered ranges, send a
   moderate `run_time`, and stop publishing on sensor dropout.

**Rules for any new publisher:** clamp every value to the discovered ranges,
never touch flash/persistent servo parameters, include `run_time` on arm
moves, and implement deadman behavior — if your input source dies, your last
command must be a stop (zero Twist) or a hold (arm).

Deadman behavior: motion stops on key release, on tab blur or page hide, and
on rosbridge disconnect.

See `transbot_dashboard_build_prompt.md` for the full build brief.
