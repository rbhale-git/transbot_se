// ============================================================================
// CONFIG — single source of truth for the whole dashboard.
// Every topic name, message type, key binding, limit, range, and URL lives
// here. No other module may hard-code any of these values.
//
// Items marked TO-VERIFY are vendor-doc hints that MUST be confirmed against
// FINDINGS.md (Phase 1 live discovery on the robot) before first real use.
// ============================================================================

// ---- Network profiles ------------------------------------------------------
// Switching networks = switching profiles. Nothing else changes.
export const PROFILES = {
  mock: {
    label: 'MOCK (local sim)',
    rosbridgeUrl: 'ws://localhost:9090',
    videoUrl: '', // mock has no video; pane shows NO SIGNAL
  },
  home: {
    label: 'HOME WIFI',
    // TO-VERIFY: robot's DHCP address once it joins the home network.
    rosbridgeUrl: 'ws://192.168.1.50:9090',
    // TO-VERIFY: camera topic name comes from Phase 1 discovery.
    videoUrl: 'http://192.168.1.50:8080/stream?topic=/usb_cam/image_raw&type=mjpeg',
  },
  hotspot: {
    label: 'ROBOT HOTSPOT',
    // Verified: robot is the gateway at 192.168.1.11 on its own "Transbot"
    // AP (WPA2, SSID "Transbot"). Note: same 192.168.1.x subnet as many home
    // networks — the laptop can only be on one of the two at a time anyway.
    rosbridgeUrl: 'ws://192.168.1.11:9090',
    videoUrl: 'http://192.168.1.11:8080/stream?topic=/usb_cam/image_raw&type=mjpeg',
  },
};

export const DEFAULT_PROFILE = 'mock';

// ---- Topics and services ---------------------------------------------------
// `verified: false` => type/name is a hint, not discovery-confirmed.
export const TOPICS = {
  // Drive. geometry_msgs/Twist is a ROS standard and locked by the brief.
  cmdVel: { name: '/cmd_vel', type: 'geometry_msgs/Twist', verified: true },

  // Camera gimbal (2 PWM servos). Custom Yahboom message.
  // TO-VERIFY: package name, type string, and field names.
  pwmServo: { name: '/PWMServo', type: 'transbot_msgs/PWMServo', verified: false },

  // 3-DOF arm (bus servos 7/8/9). Custom Yahboom message.
  // TO-VERIFY: package name, type string, and field names.
  targetAngle: { name: '/TargetAngle', type: 'transbot_msgs/Arm', verified: false },

  // Telemetry the driver publishes.
  getVel: { name: '/transbot/get_vel', type: 'geometry_msgs/Twist', verified: false }, // TO-VERIFY type
  imu: { name: '/transbot/imu', type: 'sensor_msgs/Imu', verified: false },            // TO-VERIFY type
  battery: { name: '/voltage', type: 'std_msgs/Float32', verified: false },            // TO-VERIFY name+type
};

export const SERVICES = {
  // Reads current arm joint angles.
  // TO-VERIFY: service type and response field layout.
  currentAngle: { name: '/CurrentAngle', type: 'transbot_msgs/RobotArm', verified: false },

  // Standard rosapi service (ships with rosbridge_server) — used to measure
  // round-trip latency over the WebSocket.
  getTime: { name: '/rosapi/get_time', type: 'rosapi/GetTime', verified: true },
};

// ---- Motion limits ---------------------------------------------------------
// hardMax = absolute driver limits from the brief; never exceeded, ever.
// cap     = conservative v1 operating limit (brief: start well below max).
export const MOTION = {
  linear: { hardMax: 0.45, cap: 0.20 },   // m/s
  angular: { hardMax: 2.0, cap: 1.0 },    // rad/s
  publishRateHz: 10,                      // continuous re-publish while driving
  stopBurstCount: 3,                      // extra zero-Twists sent on stop/e-stop
};

// ---- Gimbal ----------------------------------------------------------------
export const GIMBAL = {
  x: { servoId: 1, min: 0, max: 180, home: 90 },  // pan
  y: { servoId: 2, min: 0, max: 180, home: 90 },  // tilt
  stepDeg: 3,
};

// ---- Arm -------------------------------------------------------------------
// Three continuous DOF; servo 9 is the gripper joint (per brief) but is
// treated exactly like the others — full-range slider/keyboard control.
// TO-VERIFY: which servo is really the gripper, and a safe home pose.
export const ARM = {
  joints: {
    j7: { servoId: 7, min: 0, max: 225, home: 110 },
    j8: { servoId: 8, min: 30, max: 270, home: 150 },
    j9: { servoId: 9, min: 30, max: 180, home: 90 }, // gripper joint (per brief)
  },
  stepDeg: 4,
  runTimeMs: 800, // moderate move time so joints never slam (brief requirement)
};

// ---- Power -----------------------------------------------------------------
export const POWER = {
  lowVoltage: 9.9,   // 3S pack warning threshold; TO-VERIFY against vendor docs
  fullVoltage: 12.6,
};

// ---- Telemetry behavior ----------------------------------------------------
export const TELEMETRY = {
  armPollMs: 2000,      // /CurrentAngle service poll interval
  staleAfterMs: 3000,   // mark a panel stale if nothing arrived in this window
  latencyPollMs: 2000,  // RTT measurement interval (rosapi/get_time round trip)
};

// ---- Gamepad (standard-mapping controller, e.g. Xbox) -----------------------
export const GAMEPAD = {
  pollMs: 50,             // 20 Hz input polling
  deadzone: 0.15,         // stick deflection below this is ignored
  gimbalRateDegS: 45,     // right-stick full deflection sweeps this fast
  armRateDegS: 40,        // d-pad / trigger joint speed
  buttons: {              // standard mapping indices
    eStop: 1,             // B
    armHome: 0,           // A
  },
};

// ---- Key bindings ----------------------------------------------------------
// event.code values (layout-independent physical keys).
export const KEYS = {
  motion: {
    forward: 'KeyW',
    backward: 'KeyS',
    rotateLeft: 'KeyA',
    rotateRight: 'KeyD',
  },
  eStop: 'Space', // highest-priority handler, always
  gimbal: {
    left: 'ArrowLeft',
    right: 'ArrowRight',
    up: 'ArrowUp',
    down: 'ArrowDown',
    recenter: 'KeyC',
  },
  arm: {
    j7Up: 'KeyU', j7Down: 'KeyJ',
    j8Up: 'KeyI', j8Down: 'KeyK',
    j9Up: 'KeyO', j9Down: 'KeyL', // j9 = gripper joint
    home: 'KeyH',
  },
};

// ---- Reconnect policy ------------------------------------------------------
export const RECONNECT = {
  initialDelayMs: 1000,
  maxDelayMs: 8000,
};

/** Clamp helper used by every publisher before anything leaves the laptop. */
export function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}
