// ============================================================================
// publishers/arm.js — the ONLY module that publishes /TargetAngle.
//
// Tracks commanded joint angles and exposes per-joint stepping, absolute
// positioning (sliders / typed values), and a home pose. All three joints —
// including the gripper (servo 9) — are continuous DOF. Every move carries a
// moderate run_time so joints never slam (brief safety requirement). All
// angles clamp to configured ranges before publishing.
//
// TO-VERIFY (Phase 1): exact message type and field names for /TargetAngle,
// and which servo is truly the gripper.
// buildArmMessage() is the single place to fix once discovered.
// ============================================================================

import { TOPICS, ARM, clamp } from '../config.js';
import { publish } from '../ros.js';

// Commanded joint angles (degrees), keyed j7/j8/j9. Start at home pose.
const state = {};
for (const [key, j] of Object.entries(ARM.joints)) state[key] = j.home;

const listeners = [];

export function onArmChange(fn) {
  listeners.push(fn);
  fn(snapshot());
}

function snapshot() {
  return { angles: { ...state } };
}

function notify() {
  for (const fn of listeners) fn(snapshot());
}

/** TO-VERIFY: assumed layout {id, angle, run_time} per joint, wrapped in a
 *  joint array — vendor-doc hint, must match rosmsg show output. */
function buildArmMessage(moves) {
  return {
    joint: moves.map(({ servoId, angleDeg }) => ({
      id: servoId,
      angle: Math.round(angleDeg),
      run_time: ARM.runTimeMs,
    })),
  };
}

/** Publish one or more joint targets after clamping each to its range. */
function publishJoints(moves) {
  publish(TOPICS.targetAngle, buildArmMessage(moves));
}

/** Step one joint ('j7' | 'j8' | 'j9') by direction (+1 / -1). */
export function stepArmJoint(jointKey, direction) {
  setArmJoint(jointKey, state[jointKey] + direction * ARM.stepDeg);
}

/** Set one joint to an absolute angle (sliders / typed values). */
export function setArmJoint(jointKey, angleDeg) {
  const j = ARM.joints[jointKey];
  if (!j || !Number.isFinite(angleDeg)) return;
  state[jointKey] = clamp(angleDeg, j.min, j.max);
  publishJoints([{ servoId: j.servoId, angleDeg: state[jointKey] }]);
  notify();
}

/** Send all joints to the configured safe neutral pose. */
export function armHome() {
  const moves = [];
  for (const [key, j] of Object.entries(ARM.joints)) {
    state[key] = j.home;
    moves.push({ servoId: j.servoId, angleDeg: j.home });
  }
  publishJoints(moves);
  notify();
}
