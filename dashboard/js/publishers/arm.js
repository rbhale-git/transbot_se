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
    joint: moves.map(({ servoId, angleDeg, runTimeMs }) => ({
      id: servoId,
      angle: Math.round(angleDeg),
      run_time: runTimeMs ?? ARM.runTimeMs,
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
export function setArmJoint(jointKey, angleDeg, runTimeMs = undefined) {
  const j = ARM.joints[jointKey];
  if (!j || !Number.isFinite(angleDeg)) return;
  state[jointKey] = clamp(angleDeg, j.min, j.max);
  publishJoints([{ servoId: j.servoId, angleDeg: state[jointKey], runTimeMs }]);
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

/** One verification pass: poll /CurrentAngle and re-send any joint that the
 *  driver dropped (off target by more than verifyToleranceDeg). */
async function verifyAndRepair(seq, keys) {
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

/** Paced multi-joint pose: one command per joint, interCommandMs apart. */
async function runPose(angles) {
  cancelArmSequence();
  const seq = activeSeq;
  const keys = Object.keys(ARM.joints).filter((k) => k in angles);

  for (const key of keys) {
    if (seq !== activeSeq) return; // cancelled (e-stop / newer pose)
    setArmJoint(key, angles[key]);
    await wait(ARM.interCommandMs); // pacing so the driver keeps every command
  }

  await wait(ARM.runTimeMs + 300); // let the move settle
  await verifyAndRepair(seq, keys);
}

/** Send all joints to the configured safe neutral pose (paced + verified). */
export function armHome() {
  const angles = {};
  for (const [key, j] of Object.entries(ARM.joints)) angles[key] = j.home;
  runPose(angles);
}

/** Stow pose from config (J7/J8 only) — the gripper (J9) stays put. */
export function armStow() {
  runPose({ ...ARM.stowPose.angles });
}

/**
 * Staged READY pose: the lead joint moves first in one normal-speed command;
 * followers start once it has covered followAfterDeg (estimated from
 * run_time), paced apart so the driver keeps every command.
 */
export async function armReady() {
  cancelArmSequence();
  const seq = activeSeq;
  const { angles, leadJoint, followAfterDeg } = ARM.readyPose;
  const keys = Object.keys(ARM.joints).filter((k) => k in angles);
  const followers = keys.filter((k) => k !== leadJoint);

  const j = ARM.joints[leadJoint];
  const start = state[leadJoint];
  const target = clamp(angles[leadJoint], j.min, j.max);
  const delta = Math.abs(target - start);

  setArmJoint(leadJoint, target);

  // Followers start once the lead has covered followAfterDeg of its travel.
  const fraction = delta > 0 ? Math.min(1, followAfterDeg / delta) : 0;
  const stageDelay = Math.round(ARM.runTimeMs * fraction);
  await wait(Math.max(stageDelay, ARM.interCommandMs));

  for (const key of followers) {
    if (seq !== activeSeq) return;
    setArmJoint(key, angles[key]);
    await wait(ARM.interCommandMs);
  }

  await wait(ARM.runTimeMs + 300);
  await verifyAndRepair(seq, keys);
}
