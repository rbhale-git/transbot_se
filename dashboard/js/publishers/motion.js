// ============================================================================
// publishers/motion.js — the ONLY module that publishes TOPICS.cmdVel
// (= /manual/cmd_vel; the robot-side mux forwards it to /cmd_vel).
//
// Exposes publishMotion(linear, angular) and stopMotion(). Any input source —
// the keyboard handler today, an AI behavior node or external device tomorrow
// — drives the tracks through these two functions. Every value is clamped to
// the conservative cap, then to the hard driver limit, before leaving.
// ============================================================================

import { TOPICS, MOTION, clamp } from '../config.js';
import { publish } from '../ros.js';

const listeners = []; // UI observers of the last commanded values

/** Observe outgoing commands (for the commanded-vs-measured panel). */
export function onMotionCommand(fn) {
  listeners.push(fn);
}

function notify(linear, angular) {
  for (const fn of listeners) fn(linear, angular);
}

/**
 * Command track motion.
 * @param {number} linear  m/s, forward positive
 * @param {number} angular rad/s, counter-clockwise (left) positive
 */
export function publishMotion(linear, angular) {
  // Two-stage clamp: operating cap first, absolute driver limit second.
  const lin = clamp(clamp(linear, -MOTION.linear.cap, MOTION.linear.cap),
    -MOTION.linear.hardMax, MOTION.linear.hardMax);
  const ang = clamp(clamp(angular, -MOTION.angular.cap, MOTION.angular.cap),
    -MOTION.angular.hardMax, MOTION.angular.hardMax);

  publish(TOPICS.cmdVel, {
    linear: { x: lin, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: ang },
  });
  notify(lin, ang);
}

/** Send a zero Twist burst. Used by deadman release, blur, and e-stop. */
export function stopMotion() {
  for (let i = 0; i < MOTION.stopBurstCount; i++) {
    publish(TOPICS.cmdVel, {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
  }
  notify(0, 0);
}
