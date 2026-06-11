// ============================================================================
// publishers/gimbal.js — the ONLY module that publishes /PWMServo.
//
// Keeps the commanded pan/tilt state (the driver does not echo servo
// positions) and exposes step/recenter functions. All angles clamp to the
// configured [min, max] before publishing.
//
// Message shape VERIFIED in Phase 1: transbot_msgs/PWMServo {int32 id,
// int32 angle} — see FINDINGS.md.
// ============================================================================

import { TOPICS, GIMBAL, clamp, stepSign } from '../config.js';
import { publish } from '../ros.js';

// Commanded angles (degrees). Start at home; sync on recenter.
const state = { x: GIMBAL.x.home, y: GIMBAL.y.home };

const listeners = [];

export function onGimbalChange(fn) {
  listeners.push(fn);
  fn({ ...state });
}

function notify() {
  for (const fn of listeners) fn({ ...state });
}

/** Verified: transbot_msgs/PWMServo = {int32 id, int32 angle}. */
function buildPwmServoMessage(servoId, angleDeg) {
  return { id: servoId, angle: Math.round(angleDeg) };
}

function publishAxis(axisCfg, angleDeg) {
  const a = clamp(angleDeg, axisCfg.min, axisCfg.max);
  publish(TOPICS.pwmServo, buildPwmServoMessage(axisCfg.servoId, a));
  return a;
}

/** Step the pan servo by `direction` (+1 right, -1 left). */
export function stepGimbalX(direction) {
  setGimbalX(state.x + stepSign(GIMBAL.x) * direction * GIMBAL.stepDeg);
}

/** Step the tilt servo by `direction` (+1 up, -1 down). */
export function stepGimbalY(direction) {
  setGimbalY(state.y + stepSign(GIMBAL.y) * direction * GIMBAL.stepDeg);
}

/** Set the pan servo to an absolute angle (sliders / typed values). */
export function setGimbalX(angleDeg) {
  if (!Number.isFinite(angleDeg)) return;
  state.x = publishAxis(GIMBAL.x, angleDeg);
  notify();
}

/** Set the tilt servo to an absolute angle (sliders / typed values). */
export function setGimbalY(angleDeg) {
  if (!Number.isFinite(angleDeg)) return;
  state.y = publishAxis(GIMBAL.y, angleDeg);
  notify();
}

/** Drive both axes to a named pose from GIMBAL.poses. */
export function gimbalPose(name) {
  const pose = GIMBAL.poses[name];
  if (!pose) return;
  state.x = publishAxis(GIMBAL.x, pose.x);
  state.y = publishAxis(GIMBAL.y, pose.y);
  notify();
}

/** Back to the driving (home) pose — the C key and CAMERA DRIVE button. */
export function recenterGimbal() {
  gimbalPose('drive');
}
