// ============================================================================
// telemetry.js — subscribes to driver telemetry and polls /CurrentAngle.
//
// Field layouts for the custom topics are unverified until Phase 1, so the
// extractors below are deliberately defensive: they look for the expected
// fields but fall back to rendering whatever arrived, rather than breaking.
// ============================================================================

import { TOPICS, SERVICES, TELEMETRY } from './config.js';
import { subscribe, callService, isConnected, onStatus } from './ros.js';

const listeners = { vel: [], imu: [], battery: [], arm: [] };
const unsubs = [];
let armPollTimer = null;

export function onTelemetry(channel, fn) {
  listeners[channel].push(fn);
}

function emit(channel, data) {
  for (const fn of listeners[channel]) fn(data);
}

// ---- Defensive extractors (shapes confirmed in Phase 1) --------------------

function extractVel(msg) {
  // geometry_msgs/Twist shape, or a flat {linear, angular} custom msg.
  const linear = msg?.linear?.x ?? (typeof msg?.linear === 'number' ? msg.linear : null);
  const angular = msg?.angular?.z ?? (typeof msg?.angular === 'number' ? msg.angular : null);
  return { linear, angular, raw: msg };
}

function extractImu(msg) {
  return {
    accel: msg?.linear_acceleration ?? null,
    gyro: msg?.angular_velocity ?? null,
    orientation: msg?.orientation ?? null,
    raw: msg,
  };
}

function extractBattery(msg) {
  // Verified: transbot_msgs/Battery = {float32 Voltage} (capital V).
  const v = typeof msg?.Voltage === 'number' ? msg.Voltage
    : typeof msg?.data === 'number' ? msg.data
    : typeof msg === 'number' ? msg : null;
  return { voltage: v, raw: msg };
}

// ---- Lifecycle --------------------------------------------------------------

function startSubscriptions() {
  stopSubscriptions();
  unsubs.push(subscribe(TOPICS.getVel, (m) => emit('vel', extractVel(m))));
  unsubs.push(subscribe(TOPICS.imu, (m) => emit('imu', extractImu(m))));
  unsubs.push(subscribe(TOPICS.battery, (m) => emit('battery', extractBattery(m))));

  // Poll arm joint angles via the /CurrentAngle service.
  // Verified request shape: {apply: string}; response: {RobotArm: {joint: []}}.
  armPollTimer = setInterval(async () => {
    if (!isConnected()) return;
    try {
      const resp = await callService(SERVICES.currentAngle, { apply: 'GetJoint' });
      emit('arm', { ok: true, raw: resp });
    } catch {
      emit('arm', { ok: false, raw: null });
    }
  }, TELEMETRY.armPollMs);
}

function stopSubscriptions() {
  while (unsubs.length) unsubs.pop()();
  clearInterval(armPollTimer);
  armPollTimer = null;
}

export function initTelemetry() {
  onStatus((status) => {
    if (status === 'connected') startSubscriptions();
    else stopSubscriptions();
  });
}
