// ============================================================================
// publishers/gimbal.js — the ONLY module that publishes /PWMServo.
//
// Keeps the commanded pan/tilt state (the driver does not echo servo
// positions) and exposes step/recenter functions. All angles clamp to the
// configured [min, max] before publishing.
//
// TO-VERIFY (Phase 1): exact message type and field names for /PWMServo.
// buildPwmServoMessage() is the single place to fix once discovered.
// ============================================================================

import { TOPICS, GIMBAL, clamp } from '../config.js';
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

/** TO-VERIFY: assumed field layout {id, angle} from vendor docs. */
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
  state.x = publishAxis(GIMBAL.x, state.x + direction * GIMBAL.stepDeg);
  notify();
}

/** Step the tilt servo by `direction` (+1 up, -1 down). */
export function stepGimbalY(direction) {
  state.y = publishAxis(GIMBAL.y, state.y + direction * GIMBAL.stepDeg);
  notify();
}

/** Drive both axes back to their configured home angles. */
export function recenterGimbal() {
  state.x = publishAxis(GIMBAL.x, GIMBAL.x.home);
  state.y = publishAxis(GIMBAL.y, GIMBAL.y.home);
  notify();
}
