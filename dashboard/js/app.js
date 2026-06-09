// ============================================================================
// app.js — wires config, connection, publishers, telemetry, and DOM together.
// All DOM lookups live here; the other modules know nothing about the page.
// ============================================================================

import {
  PROFILES, DEFAULT_PROFILE, MOTION, POWER, TELEMETRY, KEYS, GIMBAL, ARM,
} from './config.js';
import { connect, onStatus } from './ros.js';
import { onMotionCommand } from './publishers/motion.js';
import { onGimbalChange, setGimbalX, setGimbalY } from './publishers/gimbal.js';
import { onArmChange, setArmJoint } from './publishers/arm.js';
import { initKeyboard, onEStop } from './keyboard.js';
import { initTelemetry, onTelemetry } from './telemetry.js';
import { initVideo, setVideoUrl } from './video.js';
import { loadSettings, applySetting, SETTING_DEFS } from './settings.js';
import { initGamepad, onPadStatus } from './gamepad.js';
import { initLatency, onLatency } from './latency.js';
import {
  initRecorder, onRecorderChange, startRecording, stopRecording,
  isRecording, downloadRecording,
} from './recorder.js';

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

/**
 * Bind one axis to a slider + number input pair.
 * - min/max come from config, never from the HTML.
 * - Slider drags are throttled (trailing edge) so we don't flood the topic.
 * - Controls blur after use so WASD driving keys are never captured.
 * Returns { update } for reflecting publisher state back into the controls
 * (programmatic .value writes don't re-fire input events, so no feedback loop).
 */
function bindAxisControl({ sliderId, numId, slider: sliderEl, num: numEl, min, max, step = 1, set }) {
  const slider = sliderEl ?? $(sliderId);
  const num = numEl ?? $(numId);
  slider.min = num.min = min;
  slider.max = num.max = max;
  slider.step = num.step = step;

  let pending = null;
  let timer = null;
  const flush = () => {
    timer = null;
    if (pending !== null) { set(pending); pending = null; }
  };
  slider.addEventListener('input', () => {
    pending = Number(slider.value);
    if (!timer) timer = setTimeout(flush, 80); // ~12 Hz while dragging
  });
  slider.addEventListener('change', () => {  // drag released / arrow-key step
    clearTimeout(timer);
    flush();
    slider.blur();
  });
  num.addEventListener('change', () => {      // Enter or focus-out
    set(Number(num.value));
    num.blur();
  });

  return {
    update(angle) {
      slider.value = angle;
      num.value = angle;
    },
  };
}

function initGimbalPanel() {
  const pan = bindAxisControl({
    sliderId: 'gimbal-x-slider', numId: 'gimbal-x-num',
    min: GIMBAL.x.min, max: GIMBAL.x.max, set: setGimbalX,
  });
  const tilt = bindAxisControl({
    sliderId: 'gimbal-y-slider', numId: 'gimbal-y-num',
    min: GIMBAL.y.min, max: GIMBAL.y.max, set: setGimbalY,
  });
  onGimbalChange(({ x, y }) => {
    pan.update(x);
    tilt.update(y);
  });
}

function initArmPanel() {
  const controls = {};
  for (const [key, j] of Object.entries(ARM.joints)) {
    controls[key] = bindAxisControl({
      sliderId: `arm-${key}-slider`, numId: `arm-${key}-num`,
      min: j.min, max: j.max, set: (a) => setArmJoint(key, a),
    });
  }
  onArmChange(({ angles }) => {
    for (const key of Object.keys(controls)) controls[key].update(angles[key]);
  });
  onTelemetry('arm', ({ ok, raw }) => {
    // Verified shape: {RobotArm: {joint: [{id, run_time, angle}]}}.
    const joints = raw?.RobotArm?.joint;
    if (ok && Array.isArray(joints)) {
      $('arm-current').textContent = joints
        .map((j) => `J${j.id} ${Number(j.angle).toFixed(0)}°`)
        .join('  ');
    } else {
      $('arm-current').textContent = ok ? JSON.stringify(raw) : 'no response';
    }
  });
}

// ---- Settings panel -------------------------------------------------------------
// Rows are generated from SETTING_DEFS so panel and behavior can't drift.

function initSettingsPanel() {
  loadSettings();
  const host = $('settings-rows');
  for (const [key, def] of Object.entries(SETTING_DEFS)) {
    const row = document.createElement('div');
    row.className = 'ctl';
    row.innerHTML = `
      <div class="ctl-head">
        <span class="ctl-label">${def.label}</span>
        <input type="number" class="ctl-num">
        <span class="unit">${def.unit}</span>
      </div>
      <input type="range" class="ctl-slider">`;
    host.appendChild(row);

    const ctl = bindAxisControl({
      slider: row.querySelector('.ctl-slider'),
      num: row.querySelector('.ctl-num'),
      min: def.min, max: def.max, step: def.step,
      set: (v) => ctl.update(applySetting(key, v)),
    });
    ctl.update(def.get());
  }
}

// ---- Header widgets: gamepad indicator, RTT, recorder ----------------------------

function initPadIndicator() {
  const el = $('pad-indicator');
  onPadStatus((connected, id) => {
    el.classList.toggle('on', connected);
    el.title = connected ? id : 'no gamepad';
  });
}

function initLatencyReadout() {
  const el = $('rtt');
  onLatency((rtt) => {
    el.textContent = rtt === null ? '--' : `${rtt}ms`;
    el.classList.toggle('slow', rtt !== null && rtt > 250);
  });
}

function initRecorderControls() {
  const recBtn = $('rec-btn');
  const saveBtn = $('save-btn');
  recBtn.addEventListener('click', () => {
    if (isRecording()) stopRecording();
    else startRecording();
    recBtn.blur(); // keep focus free for driving keys
  });
  saveBtn.addEventListener('click', () => {
    downloadRecording();
    saveBtn.blur();
  });
  onRecorderChange(({ recording, count }) => {
    recBtn.classList.toggle('recording', recording);
    recBtn.textContent = recording ? `■ ${count}` : '● REC';
    saveBtn.disabled = recording || count === 0;
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
    ['Arm', `${pretty(KEYS.arm.j7Up)}/${pretty(KEYS.arm.j7Down)} j7 · ${pretty(KEYS.arm.j8Up)}/${pretty(KEYS.arm.j8Down)} j8 · ${pretty(KEYS.arm.j9Up)}/${pretty(KEYS.arm.j9Down)} j9 (grip) · ${pretty(KEYS.arm.home)} home`],
    ['Pad', 'L-stick drive · R-stick gimbal · LT/RT grip · D-pad J7/J8 · B e-stop · A home'],
  ];
  $('legend').innerHTML = rows
    .map(([k, v]) => `<div class="legend-row"><span class="legend-key">${k}</span><span>${v}</span></div>`)
    .join('');
}

// ---- Boot ---------------------------------------------------------------------

initVideo($('video-stream'), $('video-nosignal'));
initTelemetry();
initKeyboard();
initGamepad();
initLatency();
initRecorder();
initStatusLamp();
initDrivePanel();
initImuPanel();
initPowerPanel();
initGimbalPanel();
initArmPanel();
initSettingsPanel();
initPadIndicator();
initLatencyReadout();
initRecorderControls();
initEStopBanner();
initLegend();
initProfileSelect(); // last: triggers the first connect
