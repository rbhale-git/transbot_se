// ============================================================================
// gamepad.js — standard-mapping controller support (Xbox/PS layout).
//
//   Left stick   : drive (Y = linear, X = rotate) — analog, scaled by the
//                  live caps, so partial deflection = partial speed.
//   Right stick  : gimbal pan/tilt at a configured rate.
//   LT / RT      : gripper joint (J9) close / open at a configured rate.
//   D-pad        : J7 (left/right) and J8 (up/down) at a configured rate.
//   B            : E-STOP (same path as spacebar).
//   A            : arm home pose.
//
// Deadman parity with the keyboard: sticks returning to neutral publish a
// zero-Twist stop, and a disconnected pad stops motion immediately. The pad
// drives the exact same publisher functions as the keyboard — nothing here
// touches topics directly.
// ============================================================================

import { MOTION, GAMEPAD, GIMBAL, stepSign } from './config.js';
import { publishMotion, stopMotion } from './publishers/motion.js';
import { setGimbalX, setGimbalY, onGimbalChange } from './publishers/gimbal.js';
import { setArmJoint, armHome, onArmChange } from './publishers/arm.js';
import { triggerEStop } from './keyboard.js';

const padListeners = []; // fn(connected: boolean, id: string)
export function onPadStatus(fn) { padListeners.push(fn); }

// Mirrors of the commanded angles, kept in sync via publisher listeners so
// rate-based control integrates from the true current command.
const mirror = { gx: 90, gy: 90, j7: 0, j8: 0, j9: 0 };

let driving = false;          // were we publishing motion last tick?
let prevButtons = [];         // for edge detection
let padWasConnected = false;

function dz(v) {
  return Math.abs(v) > GAMEPAD.deadzone ? v : 0;
}

function pollPad() {
  const pad = navigator.getGamepads?.()[0] ?? null;
  const connected = pad !== null && pad.connected;

  if (connected !== padWasConnected) {
    padWasConnected = connected;
    if (!connected && driving) { driving = false; stopMotion(); }
    for (const fn of padListeners) fn(connected, pad?.id ?? '');
  }
  if (!connected) return;

  const dt = GAMEPAD.pollMs / 1000;
  const btn = (i) => pad.buttons[i]?.pressed ?? false;
  const val = (i) => pad.buttons[i]?.value ?? 0;
  const edge = (i) => btn(i) && !prevButtons[i];

  // --- E-stop first, before any motion processing ---
  if (edge(GAMEPAD.buttons.eStop)) {
    prevButtons = pad.buttons.map((b) => b.pressed);
    driving = false;
    triggerEStop();
    return;
  }

  // --- Drive: left stick (axes 0/1, up = -1) ---
  const lx = dz(pad.axes[0] ?? 0);
  const ly = dz(pad.axes[1] ?? 0);
  if (lx !== 0 || ly !== 0) {
    driving = true;
    // Stick right (+x) = clockwise turn = negative angular.z in ROS.
    publishMotion(-ly * MOTION.linear.cap, -lx * MOTION.angular.cap);
  } else if (driving) {
    driving = false;
    stopMotion(); // deadman: stick released => robot stops
  }

  // --- Gimbal: right stick (axes 2/3) rate control ---
  const rx = dz(pad.axes[2] ?? 0);
  const ry = dz(pad.axes[3] ?? 0);
  if (rx !== 0) setGimbalX(mirror.gx + stepSign(GIMBAL.x) * rx * GAMEPAD.gimbalRateDegS * dt);
  if (ry !== 0) setGimbalY(mirror.gy - stepSign(GIMBAL.y) * ry * GAMEPAD.gimbalRateDegS * dt);

  // --- Gripper joint J9: triggers (6 = LT close, 7 = RT open), analog ---
  const grip = val(7) - val(6);
  if (grip !== 0) setArmJoint('j9', mirror.j9 + grip * GAMEPAD.armRateDegS * dt);

  // --- J7/J8: d-pad (12 up, 13 down, 14 left, 15 right) ---
  const j7dir = (btn(15) ? 1 : 0) - (btn(14) ? 1 : 0);
  const j8dir = (btn(12) ? 1 : 0) - (btn(13) ? 1 : 0);
  if (j7dir !== 0) setArmJoint('j7', mirror.j7 + j7dir * GAMEPAD.armRateDegS * dt);
  if (j8dir !== 0) setArmJoint('j8', mirror.j8 + j8dir * GAMEPAD.armRateDegS * dt);

  // --- Arm home: A (edge-triggered) ---
  if (edge(GAMEPAD.buttons.armHome)) armHome();

  prevButtons = pad.buttons.map((b) => b.pressed);
}

export function initGamepad() {
  onGimbalChange(({ x, y }) => { mirror.gx = x; mirror.gy = y; });
  onArmChange(({ angles }) => {
    mirror.j7 = angles.j7; mirror.j8 = angles.j8; mirror.j9 = angles.j9;
  });
  setInterval(pollPad, GAMEPAD.pollMs);
}
