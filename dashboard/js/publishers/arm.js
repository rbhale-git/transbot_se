// ============================================================================
// publishers/arm.js — the ONLY module that publishes /TargetAngle.
//
// Tracks commanded joint angles, exposes per-joint stepping, a gripper
// toggle, and a home pose. Every move carries a moderate run_time so joints
// never slam (brief safety requirement). All angles clamp to configured
// ranges before publishing.
//
// TO-VERIFY (Phase 1): exact message type and field names for /TargetAngle,
// which servo is truly the gripper, and the open/closed angles.
// buildArmMessage() is the single place to fix once discovered.
// ============================================================================

import { TOPICS, ARM, clamp } from '../config.js';
import { publish } from '../ros.js';

// Commanded joint angles (degrees), keyed j7/j8/j9. Start at home pose.
const state = {};
for (const [key, j] of Object.entries(ARM.joints)) state[key] = j.home;
let gripperClosed = false;

const listeners = [];

export function onArmChange(fn) {
  listeners.push(fn);
  fn(snapshot());
}

function snapshot() {
  return { angles: { ...state }, gripperClosed };
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
  const j = ARM.joints[jointKey];
  state[jointKey] = clamp(state[jointKey] + direction * ARM.stepDeg, j.min, j.max);
  publishJoints([{ servoId: j.servoId, angleDeg: state[jointKey] }]);
  notify();
}

/** Toggle the gripper between configured open and closed angles. */
export function toggleGripper() {
  gripperClosed = !gripperClosed;
  const key = ARM.gripper.jointKey;
  const j = ARM.joints[key];
  state[key] = clamp(gripperClosed ? ARM.gripper.closed : ARM.gripper.open, j.min, j.max);
  publishJoints([{ servoId: j.servoId, angleDeg: state[key] }]);
  notify();
}

/** Send all joints to the configured safe neutral pose. */
export function armHome() {
  const moves = [];
  for (const [key, j] of Object.entries(ARM.joints)) {
    state[key] = j.home;
    moves.push({ servoId: j.servoId, angleDeg: j.home });
  }
  gripperClosed = false;
  publishJoints(moves);
  notify();
}
