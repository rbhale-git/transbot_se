// ============================================================================
// publishers/arm.js — the ONLY module that publishes /TargetAngle.
//
// Tracks commanded joint angles and exposes per-joint stepping, absolute
// positioning (sliders / typed values), and a home pose. All three joints —
// including the gripper (servo 9) — are continuous DOF. Every move carries a
// moderate run_time so joints never slam (brief safety requirement). All
// angles clamp to configured ranges before publishing.
//
// Message shape VERIFIED in Phase 1: transbot_msgs/Arm {Joint[] joint},
// Joint = {int32 id, int32 run_time, float32 angle} — see FINDINGS.md.
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

/** Verified: transbot_msgs/Arm = {joint: [{id, run_time, angle}]}. */
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
  cancelArmSequence(); // a pending staged move must not fire after homing
  const moves = [];
  for (const [key, j] of Object.entries(ARM.joints)) {
    state[key] = j.home;
    moves.push({ servoId: j.servoId, angleDeg: j.home });
  }
  publishJoints(moves);
  notify();
}

// ---- Staged READY pose -----------------------------------------------------

let seqTimer = null;

/** Abort a pending staged move (called on e-stop and before new sequences). */
export function cancelArmSequence() {
  clearTimeout(seqTimer);
  seqTimer = null;
}

/**
 * Drive to the READY pose in two stages: the lead joint moves first, and the
 * remaining joints start once it has progressed `followAfterDeg` — estimated
 * from runTimeMs, since the driver interpolates each move over run_time.
 */
export function armReady() {
  cancelArmSequence();
  const { angles, leadJoint, followAfterDeg } = ARM.readyPose;

  const startDeg = state[leadJoint];
  setArmJoint(leadJoint, angles[leadJoint]);
  const delta = Math.abs(state[leadJoint] - startDeg); // post-clamp travel

  // Time for the lead joint to cover followAfterDeg of its travel.
  const fraction = delta > 0 ? Math.min(1, followAfterDeg / delta) : 0;
  const delayMs = Math.round(ARM.runTimeMs * fraction);

  seqTimer = setTimeout(() => {
    seqTimer = null;
    for (const key of Object.keys(angles)) {
      if (key !== leadJoint) setArmJoint(key, angles[key]);
    }
  }, delayMs);
}
