// ============================================================================
// app.js — wires config, connection, publishers, telemetry, and DOM together.
// All DOM lookups live here; the other modules know nothing about the page.
// ============================================================================

import {
  PROFILES, DEFAULT_PROFILE, MOTION, POWER, TELEMETRY, KEYS,
} from './config.js';
import { connect, onStatus } from './ros.js';
import { onMotionCommand } from './publishers/motion.js';
import { onGimbalChange } from './publishers/gimbal.js';
import { onArmChange } from './publishers/arm.js';
import { initKeyboard, onEStop } from './keyboard.js';
import { initTelemetry, onTelemetry } from './telemetry.js';
import { initVideo, setVideoUrl } from './video.js';

const $ = (id) => document.getElementById(id);

const fmt = (v, digits = 2) =>
  (typeof v === 'number' && Number.isFinite(v)) ? v.toFixed(digits) : '--';

// ---- Profile selection ------------------------------------------------------

function applyProfile(key) {
  const p = PROFILES[key];
  localStorage.setItem('transbot.profile', key);
  $('rosbridge-url').textContent = p.rosbridgeUrl;
  setVideoUrl(p.videoUrl);
  connect(p.rosbridgeUrl);
}

function initProfileSelect() {
  const sel = $('profile-select');
  for (const [key, p] of Object.entries(PROFILES)) {
    const opt = document.createElement('option');
    opt.value = key;
    opt.textContent = p.label;
    sel.appendChild(opt);
  }
  const saved = localStorage.getItem('transbot.profile');
  const initial = PROFILES[saved] ? saved : DEFAULT_PROFILE;
  sel.value = initial;
  sel.addEventListener('change', () => {
    applyProfile(sel.value);
    sel.blur(); // keep focus free for driving keys
  });
  applyProfile(initial);
}

// ---- Connection lamp --------------------------------------------------------

function initStatusLamp() {
  const lamp = $('link-lamp');
  const text = $('link-text');
  onStatus((status) => {
    lamp.dataset.status = status;
    text.textContent =
      status === 'connected' ? 'LINK UP'
      : status === 'connecting' ? 'CONNECTING'
      : 'LINK DOWN';
    document.body.classList.toggle('link-down', status !== 'connected');
  });
}

// ---- Drive panel ------------------------------------------------------------

function speedBar(el, value, max) {
  // Bar fills from center: positive right, negative left.
  const pct = Math.max(-1, Math.min(1, max ? value / max : 0)) * 50;
  el.style.left = pct < 0 ? `${50 + pct}%` : '50%';
  el.style.width = `${Math.abs(pct)}%`;
}

function initDrivePanel() {
  onMotionCommand((linear, angular) => {
    $('cmd-linear').textContent = fmt(linear);
    $('cmd-angular').textContent = fmt(angular);
    speedBar($('cmd-linear-bar'), linear, MOTION.linear.cap);
    speedBar($('cmd-angular-bar'), angular, MOTION.angular.cap);
  });

  let lastVelAt = 0;
  onTelemetry('vel', ({ linear, angular }) => {
    lastVelAt = performance.now();
    $('meas-linear').textContent = fmt(linear);
    $('meas-angular').textContent = fmt(angular);
    speedBar($('meas-linear-bar'), linear ?? 0, MOTION.linear.cap);
    speedBar($('meas-angular-bar'), angular ?? 0, MOTION.angular.cap);
  });

  // Stale-data dimming.
  setInterval(() => {
    $('drive-panel').classList.toggle(
      'stale', performance.now() - lastVelAt > TELEMETRY.staleAfterMs);
  }, 1000);
}

// ---- IMU panel --------------------------------------------------------------

function initImuPanel() {
  let lastAt = 0;
  onTelemetry('imu', ({ accel, gyro }) => {
    lastAt = performance.now();
    $('imu-ax').textContent = fmt(accel?.x);
    $('imu-ay').textContent = fmt(accel?.y);
    $('imu-az').textContent = fmt(accel?.z);
    $('imu-gx').textContent = fmt(gyro?.x);
    $('imu-gy').textContent = fmt(gyro?.y);
    $('imu-gz').textContent = fmt(gyro?.z);
  });
  setInterval(() => {
    $('imu-panel').classList.toggle(
      'stale', performance.now() - lastAt > TELEMETRY.staleAfterMs);
  }, 1000);
}

// ---- Power panel --------------------------------------------------------------

function initPowerPanel() {
  let lastAt = 0;
  onTelemetry('battery', ({ voltage }) => {
    lastAt = performance.now();
    $('battery-voltage').textContent = fmt(voltage, 1);
    const low = typeof voltage === 'number' && voltage < POWER.lowVoltage;
    $('power-panel').classList.toggle('alert', low);
    $('battery-warning').classList.toggle('hidden', !low);
    const pct = typeof voltage === 'number'
      ? Math.max(0, Math.min(1, (voltage - POWER.lowVoltage) /
          (POWER.fullVoltage - POWER.lowVoltage)))
      : 0;
    $('battery-bar').style.width = `${(pct * 100).toFixed(0)}%`;
  });
  setInterval(() => {
    $('power-panel').classList.toggle(
      'stale', performance.now() - lastAt > TELEMETRY.staleAfterMs);
  }, 1000);
}

// ---- Gimbal + arm panels ------------------------------------------------------

function initGimbalPanel() {
  onGimbalChange(({ x, y }) => {
    $('gimbal-x').textContent = `${x}°`;
    $('gimbal-y').textContent = `${y}°`;
  });
}

function initArmPanel() {
  onArmChange(({ angles, gripperClosed }) => {
    $('arm-j7').textContent = `${angles.j7}°`;
    $('arm-j8').textContent = `${angles.j8}°`;
    $('arm-j9').textContent = `${angles.j9}°`;
    $('gripper-state').textContent = gripperClosed ? 'CLOSED' : 'OPEN';
  });
  onTelemetry('arm', ({ ok, raw }) => {
    // Response shape unverified until Phase 1 — render whatever came back.
    $('arm-current').textContent = ok ? JSON.stringify(raw) : 'no response';
  });
}

// ---- E-stop banner ------------------------------------------------------------

function initEStopBanner() {
  const banner = $('estop-banner');
  let timer = null;
  onEStop(() => {
    banner.classList.remove('hidden');
    clearTimeout(timer);
    timer = setTimeout(() => banner.classList.add('hidden'), 1500);
  });
}

// ---- Key legend (rendered from config so it can never drift) ------------------

function initLegend() {
  const pretty = (code) => code.replace('Key', '').replace('Arrow', '');
  const rows = [
    ['Drive', `${pretty(KEYS.motion.forward)}/${pretty(KEYS.motion.backward)} fwd/back · ${pretty(KEYS.motion.rotateLeft)}/${pretty(KEYS.motion.rotateRight)} rotate (hold)`],
    ['E-STOP', 'SPACE'],
    ['Gimbal', `${pretty(KEYS.gimbal.left)}/${pretty(KEYS.gimbal.right)}/${pretty(KEYS.gimbal.up)}/${pretty(KEYS.gimbal.down)} arrows · ${pretty(KEYS.gimbal.recenter)} recenter`],
    ['Arm', `${pretty(KEYS.arm.j7Up)}/${pretty(KEYS.arm.j7Down)} j7 · ${pretty(KEYS.arm.j8Up)}/${pretty(KEYS.arm.j8Down)} j8 · ${pretty(KEYS.arm.j9Up)}/${pretty(KEYS.arm.j9Down)} j9`],
    ['Gripper', `${pretty(KEYS.arm.gripperToggle)} toggle · ${pretty(KEYS.arm.home)} arm home`],
  ];
  $('legend').innerHTML = rows
    .map(([k, v]) => `<div class="legend-row"><span class="legend-key">${k}</span><span>${v}</span></div>`)
    .join('');
}

// ---- Boot ---------------------------------------------------------------------

initVideo($('video-stream'), $('video-nosignal'));
initTelemetry();
initKeyboard();
initStatusLamp();
initDrivePanel();
initImuPanel();
initPowerPanel();
initGimbalPanel();
initArmPanel();
initEStopBanner();
initLegend();
initProfileSelect(); // last: triggers the first connect
