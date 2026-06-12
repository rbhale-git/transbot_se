// ============================================================================
// app.js — wires config, connection, publishers, telemetry, and DOM together.
// All DOM lookups live here; the other modules know nothing about the page.
// ============================================================================

import {
  PROFILES, DEFAULT_PROFILE, MOTION, POWER, TELEMETRY, KEYS, GIMBAL, ARM,
  VIDEO, TOPICS, stepSign,
} from './config.js';
import { connect, onStatus, kickConnection } from './ros.js';
import { initActuationCheck, onActuationCheck } from './actuation_check.js';
import { onMotionCommand } from './publishers/motion.js';
import {
  onGimbalChange, setGimbalX, setGimbalY, recenterGimbal, gimbalPose,
} from './publishers/gimbal.js';
import {
  onArmChange, setArmJoint, armHome, armReady, armStow, cancelArmSequence,
} from './publishers/arm.js';
import { initKeyboard, onEStop, triggerEStop } from './keyboard.js';
import { initTelemetry, onTelemetry } from './telemetry.js';
import {
  initVideo, setVideoUrl, kickVideo, setStreamParams, setStreamEnabled,
} from './video.js';
import { loadSettings, applySetting, resetSettings, SETTING_DEFS } from './settings.js';
import { initGamepad, onPadStatus } from './gamepad.js';
import { initLatency, onLatency } from './latency.js';
import {
  initRecorder, onRecorderChange, startRecording, stopRecording,
  isRecording, downloadRecording,
} from './recorder.js';
import { initAiPanel } from './ai_panel.js';

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
    // MJPEG streams can end silently when the robot stack restarts; the
    // bridge and video server live together, so reconnect = re-kick video.
    if (status === 'connected') kickVideo();
  });
}

// ---- Actuation fault warning --------------------------------------------------
// Driven by actuation_check.js: when rosbridge silently drops a command-topic
// registration, light the header chip and flag the affected panel(s) red.

function initActuationWarning() {
  const chip = $('actuation-warning');
  const healBtn = $('actuation-heal-btn');
  const panelFor = {
    [TOPICS.cmdVel.name]: 'drive-panel',
    [TOPICS.pwmServo.name]: 'gimbal-panel',
    [TOPICS.targetAngle.name]: 'arm-panel',
  };

  // The heal needs serve_dashboard.py (a browser can't SSH); when the
  // behavior API isn't there the chip's manual instructions still stand.
  // Probed per fault (not via the AI panel's poll) — a fault can fire
  // before that panel's first poll lands.
  async function healApiUp() {
    try { return (await fetch('/api/behavior')).ok; } catch { return false; }
  }

  onActuationCheck(async ({ state, dead = [] }) => {
    const fault = state === 'fault';
    chip.classList.toggle('hidden', !fault);
    for (const [topic, panelId] of Object.entries(panelFor)) {
      $(panelId).classList.toggle('alert', fault && dead.includes(topic));
    }
    if (fault) {
      chip.title = `rosbridge dropped publisher registration for ${dead.join(', ')}`
        + ' — commands on these topics are being silently discarded.'
        + ' Restart rosbridge-dashboard.service on the robot.';
    }
    healBtn.classList.toggle('hidden', !(fault && await healApiUp()));
  });

  healBtn.addEventListener('click', async () => {
    healBtn.disabled = true;
    healBtn.textContent = 'RESTARTING…';
    let fail = null;
    try {
      const resp = await fetch('/api/heal', {
        method: 'POST',
        body: JSON.stringify({ profile: $('profile-select').value }),
      });
      const out = await resp.json().catch(() => ({}));
      if (resp.ok) kickConnection(); // reconnect re-runs the self-check
      else fail = out.error ?? out.detail ?? `heal failed (${resp.status})`;
    } catch {
      fail = 'behavior API unreachable';
    }
    if (fail) {
      healBtn.textContent = 'HEAL FAILED';
      healBtn.title = fail;
      setTimeout(() => { healBtn.textContent = 'RESTART ROSBRIDGE'; }, 4000);
    } else {
      healBtn.textContent = 'RESTART ROSBRIDGE';
    }
    healBtn.disabled = false;
    healBtn.blur(); // keep focus free for driving keys
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
    // Mirror commanded velocity into the video HUD.
    $('hud-lin').textContent = fmt(linear);
    $('hud-ang').textContent = fmt(angular);
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

// ---- IMU strip (DRIVE panel) ---------------------------------------------------

function initImuStrip() {
  let lastAt = 0;
  onTelemetry('imu', ({ accel, gyro }) => {
    lastAt = performance.now();
    $('imu-acc').textContent =
      `${fmt(accel?.x, 1)} ${fmt(accel?.y, 1)} ${fmt(accel?.z, 1)}`;
    $('imu-gyr').textContent =
      `${fmt(gyro?.x, 2)} ${fmt(gyro?.y, 2)} ${fmt(gyro?.z, 2)}`;
  });
  setInterval(() => {
    $('imu-strip').classList.toggle(
      'stale', performance.now() - lastAt > TELEMETRY.staleAfterMs);
  }, 1000);
}

// ---- Video stream controls (camera panel title row) ---------------------------
// STREAM on/off frees a Nano encoder while an AI behavior runs; SD/HD trades
// picture quality against robot CPU (see VIDEO in config.js).

function initVideoControls() {
  let on = true;
  let preset = VIDEO.defaultPreset;

  const apply = () => {
    setStreamEnabled(on);
    setStreamParams(VIDEO.presets[preset].params);
    $('video-power-btn').textContent = on ? 'STREAM: ON' : 'STREAM: OFF';
    $('video-quality-btn').textContent = VIDEO.presets[preset].label;
    $('video-quality-btn').disabled = !on;
  };
  $('video-power-btn').addEventListener('click', () => { on = !on; apply(); });
  $('video-quality-btn').addEventListener('click', () => {
    preset = preset === 'sd' ? 'hd' : 'sd';
    apply();
  });
  apply();
}

// ---- FPS counter (bottom-left of the camera stage) -----------------------------

function initFpsCounter() {
  onTelemetry('fps', (info) => {
    if (info === null) {
      $('hud-fps').textContent = '--';
      $('hud-res').textContent = '';
      return;
    }
    $('hud-fps').textContent = info.fps;
    $('hud-res').textContent =
      (info.width && info.height) ? `${info.width}×${info.height}` : '';
  });
}

// ---- Power widget (header) -------------------------------------------------------

function initPowerWidget() {
  let lastAt = 0;
  onTelemetry('battery', ({ voltage }) => {
    lastAt = performance.now();
    $('battery-voltage').textContent = fmt(voltage, 1);
    const low = typeof voltage === 'number' && voltage < POWER.lowVoltage;
    $('power-widget').classList.toggle('alert', low);
    $('battery-warning').classList.toggle('hidden', !low);
    const pct = typeof voltage === 'number'
      ? Math.max(0, Math.min(1, (voltage - POWER.lowVoltage) /
          (POWER.fullVoltage - POWER.lowVoltage)))
      : 0;
    $('battery-bar').style.width = `${(pct * 100).toFixed(0)}%`;
  });
  setInterval(() => {
    $('power-widget').classList.toggle(
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
function bindAxisControl({ sliderId, numId, slider: sliderEl, num: numEl, min, max, step = 1, set, flip = false }) {
  const slider = sliderEl ?? $(sliderId);
  const num = numEl ?? $(numId);
  slider.min = num.min = min;
  slider.max = num.max = max;
  slider.step = num.step = step;
  // flip: mirror the SLIDER scale when the servo's positive direction is
  // reversed (drag right = camera right). The number box stays the true
  // servo angle. Self-inverse, so one mapping serves both directions.
  const toSlider = (a) => (flip ? min + max - a : a);

  let pending = null;
  let timer = null;
  const flush = () => {
    timer = null;
    if (pending !== null) { set(pending); pending = null; }
  };
  slider.addEventListener('input', () => {
    pending = toSlider(Number(slider.value));
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
      slider.value = toSlider(angle);
      num.value = angle;
    },
  };
}

function initGimbalPanel() {
  // Pan: +angle physically pans the view LEFT (same servo truth behind
  // invertStep), so the slider scale and the map dot are mirrored to make
  // drag-right = camera-right. Angles themselves stay raw everywhere.
  const flipPan = stepSign(GIMBAL.x) < 0;
  const pan = bindAxisControl({
    sliderId: 'gimbal-x-slider', numId: 'gimbal-x-num',
    min: GIMBAL.x.min, max: GIMBAL.x.max, set: setGimbalX, flip: flipPan,
  });
  const tilt = bindAxisControl({
    sliderId: 'gimbal-y-slider', numId: 'gimbal-y-num',
    min: GIMBAL.y.min, max: GIMBAL.y.max, set: setGimbalY,
  });
  const dot = $('gimbal-dot');
  onGimbalChange(({ x, y }) => {
    pan.update(x);
    tilt.update(y);
    // Position the map dot: x left→right (mirrored with the slider), y bottom→top.
    let px = ((x - GIMBAL.x.min) / (GIMBAL.x.max - GIMBAL.x.min)) * 100;
    if (flipPan) px = 100 - px;
    const py = (1 - (y - GIMBAL.y.min) / (GIMBAL.y.max - GIMBAL.y.min)) * 100;
    dot.style.left = `${px}%`;
    dot.style.top = `${py}%`;
  });
  $('gimbal-drive-btn').addEventListener('click', (e) => {
    recenterGimbal();
    e.target.blur(); // keep focus free for driving keys
  });
  $('gimbal-ap-btn').addEventListener('click', (e) => {
    gimbalPose('ap');
    e.target.blur(); // keep focus free for driving keys
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
  $('arm-home-btn').addEventListener('click', (e) => {
    armHome();
    e.target.blur(); // keep focus free for driving keys
  });
  $('arm-ready-btn').addEventListener('click', (e) => {
    armReady();
    e.target.blur();
  });
  $('arm-stow-btn').addEventListener('click', (e) => {
    armStow();
    e.target.blur();
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
  const ctls = {};
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
    ctls[key] = ctl;
  }
  $('settings-reset-btn').addEventListener('click', (e) => {
    resetSettings();
    for (const [key, def] of Object.entries(SETTING_DEFS)) ctls[key].update(def.get());
    e.target.blur(); // keep focus free for driving keys
  });
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
    $('hud-rec').classList.toggle('hidden', !recording);
  });
}

// ---- E-stop banner ------------------------------------------------------------

function initEStopButton() {
  $('estop-btn').addEventListener('click', (e) => {
    triggerEStop(); // same path as the SPACE key
    e.target.blur(); // keep focus free for driving keys
  });
}

function initEStopBanner() {
  const banner = $('estop-banner');
  let timer = null;
  onEStop(() => {
    cancelArmSequence(); // a staged arm move must never fire after an e-stop
    banner.classList.remove('hidden');
    clearTimeout(timer);
    timer = setTimeout(() => banner.classList.add('hidden'), 1500);
  });
}

// ---- Key legend (rendered from config so it can never drift) ------------------

function initLegend() {
  const GLYPHS = { ArrowLeft: '←', ArrowRight: '→', ArrowUp: '↑', ArrowDown: '↓' };
  const pretty = (code) => GLYPHS[code] ?? code.replace('Key', '').replace('Arrow', '');
  const kbd = (code) => `<kbd>${pretty(code)}</kbd>`;
  const m = KEYS.motion; const g = KEYS.gimbal; const a = KEYS.arm;
  const rows = [
    ['Drive', `${kbd(m.forward)}${kbd(m.backward)} fwd/back · ${kbd(m.rotateLeft)}${kbd(m.rotateRight)} rotate (hold)`],
    ['E-STOP', '<kbd class="kbd-estop">SPACE</kbd>'],
    ['Gimbal', `${kbd(g.left)}${kbd(g.right)}${kbd(g.up)}${kbd(g.down)} step · ${kbd(g.recenter)} recenter`],
    ['Arm', `${kbd(a.j7Up)}${kbd(a.j7Down)} j7 · ${kbd(a.j8Up)}${kbd(a.j8Down)} j8 · ${kbd(a.j9Up)}${kbd(a.j9Down)} grip · ${kbd(a.home)} home`],
    ['Pad', 'L-stick drive · R-stick gimbal · LT/RT grip · D-pad J7/J8 · B e-stop · A home'],
  ];
  $('legend').innerHTML = rows
    .map(([k, v]) => `<div class="legend-row"><span class="legend-key">${k}</span><span class="legend-val">${v}</span></div>`)
    .join('');
}

// ---- Boot ---------------------------------------------------------------------

initVideo($('video-stream'), $('video-nosignal'));
initVideoControls();
initTelemetry();
initKeyboard();
initGamepad();
initLatency();
initRecorder();
initStatusLamp();
initActuationCheck();
initActuationWarning();
initDrivePanel();
initImuStrip();
initFpsCounter();
initPowerWidget();
initGimbalPanel();
initArmPanel();
initSettingsPanel();
initAiPanel();
initPadIndicator();
initLatencyReadout();
initRecorderControls();
initEStopButton();
initEStopBanner();
initLegend();
initProfileSelect(); // last: triggers the first connect
