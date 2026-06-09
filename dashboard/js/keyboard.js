// ============================================================================
// keyboard.js — turns physical key state into publisher calls, and owns
// every deadman behavior. This module is the safety boundary for input:
//
//   1. Hold-to-move: a 10 Hz loop publishes the Twist computed from the keys
//      currently held; releasing all motion keys publishes a zero burst.
//   2. Spacebar e-stop: registered in the CAPTURE phase on window so nothing
//      can intercept it first. Zeroes motion immediately and clears all held
//      state, so movement only resumes on a fresh key press.
//   3. Tab blur / page hide: treated exactly like releasing every key.
//   4. rosbridge disconnect: held state cleared so reconnection cannot
//      resume stale motion. (The robot-side stop on link loss is enforced by
//      sending zeros while we still can; once the link is gone, rosbridge's
//      own connection drop plus the driver's behavior are the backstop —
//      verified in Phase 4.)
// ============================================================================

import { KEYS, MOTION } from './config.js';
import { publishMotion, stopMotion } from './publishers/motion.js';
import { stepGimbalX, stepGimbalY, recenterGimbal } from './publishers/gimbal.js';
import { stepArmJoint, armHome } from './publishers/arm.js';
import { onStatus } from './ros.js';

const held = new Set();      // event.codes of currently held motion keys
let publishTimer = null;     // 10 Hz republish loop while driving
let motionActive = false;

const eStopListeners = [];
export function onEStop(fn) { eStopListeners.push(fn); }

// ---- Motion (held-key) handling -------------------------------------------

function computeTwist() {
  const m = KEYS.motion;
  const linear =
    (held.has(m.forward) ? 1 : 0) - (held.has(m.backward) ? 1 : 0);
  // ROS convention: positive angular.z = counter-clockwise = rotate left.
  const angular =
    (held.has(m.rotateLeft) ? 1 : 0) - (held.has(m.rotateRight) ? 1 : 0);
  return {
    linear: linear * MOTION.linear.cap,
    angular: angular * MOTION.angular.cap,
  };
}

function refreshMotion() {
  const { linear, angular } = computeTwist();
  const anyKey = linear !== 0 || angular !== 0;

  if (anyKey && !motionActive) {
    motionActive = true;
    publishTimer = setInterval(() => {
      const t = computeTwist();
      publishMotion(t.linear, t.angular);
    }, 1000 / MOTION.publishRateHz);
  }
  if (!anyKey && motionActive) {
    motionActive = false;
    clearInterval(publishTimer);
    publishTimer = null;
    stopMotion(); // deadman: keys released => robot stops, immediately
    return;
  }
  if (anyKey) publishMotion(linear, angular); // immediate, don't wait a tick
}

/** Release everything and stop — shared by blur, disconnect, and e-stop. */
function releaseAll() {
  held.clear();
  motionActive = false;
  clearInterval(publishTimer);
  publishTimer = null;
  stopMotion();
}

function eStop() {
  releaseAll(); // includes the zero-Twist burst
  for (const fn of eStopListeners) fn();
}

// ---- Key handlers ----------------------------------------------------------

const MOTION_CODES = new Set(Object.values(KEYS.motion));

/** True when focus is in a form control (sliders, number inputs, selects) —
 *  those keys belong to the control, not the robot. E-stop is exempt. */
function inFormControl(e) {
  const tag = e.target?.tagName;
  return tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA';
}

function handleKeyDown(e) {
  // E-stop first and unconditionally — works even while typing in an input.
  if (e.code === KEYS.eStop) {
    e.preventDefault();
    e.stopImmediatePropagation();
    eStop();
    return;
  }
  if (inFormControl(e)) return;
  if (e.repeat && MOTION_CODES.has(e.code)) return; // held motion keys: loop handles it

  if (MOTION_CODES.has(e.code)) {
    e.preventDefault();
    held.add(e.code);
    refreshMotion();
    return;
  }

  // Gimbal: native key repeat gives hold-to-sweep stepping for free.
  const g = KEYS.gimbal;
  switch (e.code) {
    case g.left: e.preventDefault(); stepGimbalX(-1); return;
    case g.right: e.preventDefault(); stepGimbalX(+1); return;
    case g.up: e.preventDefault(); stepGimbalY(+1); return;
    case g.down: e.preventDefault(); stepGimbalY(-1); return;
    case g.recenter: recenterGimbal(); return;
  }

  // Arm.
  const a = KEYS.arm;
  switch (e.code) {
    case a.j7Up: stepArmJoint('j7', +1); return;
    case a.j7Down: stepArmJoint('j7', -1); return;
    case a.j8Up: stepArmJoint('j8', +1); return;
    case a.j8Down: stepArmJoint('j8', -1); return;
    case a.j9Up: stepArmJoint('j9', +1); return;
    case a.j9Down: stepArmJoint('j9', -1); return;
    case a.home: if (!e.repeat) armHome(); return;
  }
}

function handleKeyUp(e) {
  if (MOTION_CODES.has(e.code)) {
    held.delete(e.code);
    refreshMotion();
  }
}

// ---- Wiring ----------------------------------------------------------------

export function initKeyboard() {
  // E-stop intercepts in the capture phase, ahead of every other listener.
  window.addEventListener('keydown', handleKeyDown, { capture: true });
  window.addEventListener('keyup', handleKeyUp, { capture: true });

  // Deadman on focus loss: any way the page stops seeing keys => stop.
  window.addEventListener('blur', releaseAll);
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) releaseAll();
  });

  // Deadman on link state change: never carry held keys across a reconnect.
  onStatus((status) => {
    if (status !== 'connected') releaseAll();
  });
}
