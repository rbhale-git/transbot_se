// ============================================================================
// actuation_check.js — detects the Melodic rosbridge silent-drop bug.
//
// The robot's rosbridge (0.11) can half-fail a publisher registration: it
// accepts our advertise, but never becomes a publisher of the topic on the
// ROS side, and every command published on it vanishes with no error reaching
// the browser. Seen live 2026-06-11: gimbal fine, /manual/cmd_vel and
// /TargetAngle dead; journalctl shows "is not a publisher of [...]".
//
// rosapi runs inside the same rosbridge process, so the honest check is to
// ask it whether /rosbridge_websocket is registered as a publisher of each
// command topic. On failure the proven workaround is a fresh session (same
// strategy as ai/common/connect.py) — forced at most ACTUATION
// .maxFreshSessions times, after which the operator is told to restart
// rosbridge-dashboard.service on the robot.
//
// States emitted: 'ok' | 'fault' (with dead topic list) | 'unverified'
// (rosapi didn't answer — e.g. the mock server; never treated as a fault).
// ============================================================================

import { TOPICS, SERVICES, ACTUATION } from './config.js';
import { advertise, callService, kickConnection, onStatus } from './ros.js';

// Every topic the operator's controls publish commands on.
const CHECKED = [TOPICS.cmdVel, TOPICS.pwmServo, TOPICS.targetAngle];

const listeners = [];
let freshSessions = 0;
let runId = 0; // invalidates an in-flight check when the link drops

export function onActuationCheck(fn) {
  listeners.push(fn);
}

function emit(result) {
  for (const fn of listeners) fn(result);
}

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function runCheck(id) {
  for (const t of CHECKED) advertise(t);
  await sleep(ACTUATION.settleMs);
  if (id !== runId) return;

  const dead = [];
  for (const t of CHECKED) {
    let resp;
    try {
      resp = await callService(SERVICES.publishers, { topic: t.name });
    } catch {
      // rosapi unreachable (mock server, or bridge died mid-check): nothing
      // to conclude — a dead bridge already shows as LINK DOWN.
      emit({ state: 'unverified' });
      return;
    }
    if (!(resp.publishers || []).includes('/rosbridge_websocket')) {
      dead.push(t.name);
    }
  }
  if (id !== runId) return;

  if (dead.length === 0) {
    freshSessions = 0;
    emit({ state: 'ok' });
    return;
  }
  if (freshSessions < ACTUATION.maxFreshSessions) {
    freshSessions += 1;
    console.warn(`actuation check: rosbridge dropped ${dead.join(', ')} — `
      + `forcing fresh session (${freshSessions}/${ACTUATION.maxFreshSessions})`);
    kickConnection(); // reconnect re-runs the check
    return;
  }
  emit({ state: 'fault', dead });
}

export function initActuationCheck() {
  onStatus((status) => {
    runId += 1; // any status change invalidates an in-flight check
    if (status === 'connected') runCheck(runId);
  });
}
