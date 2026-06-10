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

import { TOPICS, SERVICES, ARM, clamp } from '../config.js';
import { publish, callService, isConnected } from '../ros.js';

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

// ---- Pose sequences ----------------------------------------------------------
// The vendor driver drops /TargetAngle commands that arrive back-to-back
// (subscriber queue of 1 + per-servo serial writes), so multi-joint poses are
// sent one joint at a time, paced ARM.interCommandMs apart, and verified
// against /CurrentAngle afterwards with a single re-send for any dropped joint.

let activeSeq = 0; // bumping this aborts any in-flight sequence

/** Abort a pending pose sequence (called on e-stop and before new poses). */
export function cancelArmSequence() {
  activeSeq += 1;
}

const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function runPose(angles, { leadJoint = null, followAfterDeg = 0 } = {}) {
  cancelArmSequence();
  const seq = activeSeq;

  const keys = Object.keys(ARM.joints).filter((k) => k in angles);
  const order = leadJoint
    ? [leadJoint, ...keys.filter((k) => k !== leadJoint)]
    : keys;
  const leadStartDeg = leadJoint ? state[leadJoint] : 0;

  for (let i = 0; i < order.length; i++) {
    if (seq !== activeSeq) return; // cancelled (e-stop / newer pose)
    const key = order[i];

    if (i === 1 && leadJoint) {
      // Staging: followers start once the lead has covered followAfterDeg,
      // estimated from run_time (driver interpolates moves over run_time).
      const delta = Math.abs(state[leadJoint] - leadStartDeg);
      const fraction = delta > 0 ? Math.min(1, followAfterDeg / delta) : 0;
      const stageDelay = Math.round(ARM.runTimeMs * fraction);
      await wait(Math.max(stageDelay, ARM.interCommandMs));
      if (seq !== activeSeq) return;
    }

    setArmJoint(key, angles[key]);
    if (i < order.length - 1 && !(i === 0 && leadJoint)) {
      await wait(ARM.interCommandMs); // pacing so the driver keeps every command
    }
  }

  // Verify once after the move settles; re-send anything the driver dropped.
  await wait(ARM.runTimeMs + 300);
  if (seq !== activeSeq || !isConnected()) return;
  try {
    const resp = await callService(SERVICES.currentAngle, { apply: 'GetJoint' });
    const reported = resp?.RobotArm?.joint ?? [];
    for (const r of reported) {
      const key = keys.find((k) => ARM.joints[k].servoId === r.id);
      if (!key) continue;
      if (Math.abs(Number(r.angle) - state[key]) > ARM.verifyToleranceDeg) {
        if (seq !== activeSeq) return;
        setArmJoint(key, state[key]); // one corrective re-send
        await wait(ARM.interCommandMs);
      }
    }
  } catch { /* verification is best-effort; never throws into the UI */ }
}

/** Send all joints to the configured safe neutral pose (paced + verified). */
export function armHome() {
  const angles = {};
  for (const [key, j] of Object.entries(ARM.joints)) angles[key] = j.home;
  runPose(angles);
}

/** Staged READY pose: lead joint first, followers after followAfterDeg. */
export function armReady() {
  const { angles, leadJoint, followAfterDeg } = ARM.readyPose;
  runPose(angles, { leadJoint, followAfterDeg });
}
