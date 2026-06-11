// ============================================================================
// ai_panel.js — ARM/DISARM switch for AI chassis control + arbitration status,
// plus runner start/stop via the serve_dashboard.py behavior API.
//
// Publishes /ai/enabled (Bool). The robot-side cmd_vel_mux is the enforcement
// point — this panel is just the operator's switch and readout. Policy:
// disarmed on every page load and on every (re)connect; e-stop disarms.
// Status comes from /mux/status (who is driving) and /ai/status (behavior
// state), both JSON-in-String, marked stale after TELEMETRY.staleAfterMs.
//
// Runner controls: a web page cannot spawn local processes, so START/STOP go
// through the localhost-only API on the static server (tools/
// serve_dashboard.py). ARM stays a separate, deliberate second click —
// starting the process grants no permission to drive. The runner controls
// build their own DOM here so the whole runner feature lives in one module.
// ============================================================================

import { TOPICS, TELEMETRY } from './config.js';
import { publish, subscribe, onStatus } from './ros.js';
import { onEStop } from './keyboard.js';

const $ = (id) => document.getElementById(id);

const RUNNER_API = '/api/behavior';
const RUNNER_POLL_MS = 2000;

let armed = false;
let lastMux = 0;
let lastAi = 0;
let runnerUp = false;

function render() {
  const btn = $('ai-arm-btn');
  btn.innerHTML = armed ? '&#9632; DISARM AI' : '&#9655; ARM AI';
  btn.classList.toggle('armed', armed);
}

function setArmed(next) {
  armed = next;
  publish(TOPICS.aiEnabled, { data: armed });
  render();
}

// ---- Runner controls (serve_dashboard.py behavior API) ---------------------

function buildRunnerControls() {
  const panel = $('ai-panel');
  const btn = document.createElement('button');
  btn.id = 'ai-runner-btn';
  btn.className = 'panel-btn';
  btn.title = 'start/stop the person-following process on the laptop '
    + '(headless; this video pane is your eye). ARM is still a separate step.';
  btn.innerHTML = '&#9655; START RUNNER';
  btn.disabled = true; // until the API answers a poll
  const row = document.createElement('div');
  row.className = 'kv';
  row.innerHTML = '<span>RUNNER</span><code id="ai-runner-state" class="dim small">--</code>';
  panel.appendChild(btn);
  panel.appendChild(row);
}

function renderRunner(status) {
  const btn = $('ai-runner-btn');
  const state = $('ai-runner-state');
  if (status === null) { // API unreachable (static hosting / server down)
    runnerUp = false;
    btn.disabled = true;
    btn.innerHTML = '&#9655; START RUNNER';
    state.textContent = 'api offline';
    return;
  }
  runnerUp = status.running;
  btn.disabled = false;
  btn.innerHTML = runnerUp ? '&#9632; STOP RUNNER' : '&#9655; START RUNNER';
  state.textContent = runnerUp
    ? `up ${status.started_s_ago ?? 0}s  pid ${status.pid}`
    : (status.last_exit == null ? 'down' : `down (exit ${status.last_exit})`);
}

async function runnerRequest(path, body) {
  try {
    const resp = await fetch(RUNNER_API + path, body === undefined
      ? undefined
      : { method: 'POST', body: JSON.stringify(body) });
    if (!resp.ok && resp.status !== 409) return null;
    return await resp.json();
  } catch {
    return null;
  }
}

function initRunnerControls() {
  buildRunnerControls();
  $('ai-runner-btn').addEventListener('click', async () => {
    $('ai-runner-btn').disabled = true; // re-enabled by the next poll
    if (runnerUp) {
      await runnerRequest('/stop', {});
    } else {
      // Starting a behavior never implies permission to drive: drop the arm
      // switch first so the latched armed state can't move the robot the
      // moment the fresh runner connects.
      setArmed(false);
      await runnerRequest('/start', {});
    }
    renderRunner(await runnerRequest(''));
  });
  setInterval(async () => renderRunner(await runnerRequest('')), RUNNER_POLL_MS);
  runnerRequest('').then(renderRunner);
}

export function initAiPanel() {
  $('ai-arm-btn').addEventListener('click', () => setArmed(!armed));
  onEStop(() => setArmed(false));   // e-stop also pulls the AI switch
  initRunnerControls();

  onStatus((status) => {
    if (status !== 'connected') return;
    // Fresh link: declare disarmed (page-load default) and (re)subscribe.
    setArmed(false);
    subscribe(TOPICS.muxStatus, (msg) => {
      lastMux = Date.now();
      try {
        const s = JSON.parse(msg.data);
        $('ai-source').textContent = s.source.toUpperCase();
        $('ai-armed').textContent = s.armed ? 'YES' : 'no';
      } catch { /* malformed status: leave last value, staleness will catch it */ }
    });
    subscribe(TOPICS.aiStatus, (msg) => {
      lastAi = Date.now();
      try {
        const s = JSON.parse(msg.data);
        $('ai-behavior').textContent =
          `${s.state} ${s.target_class ?? ''} ${s.fps != null ? s.fps + 'fps' : ''}`.trim();
      } catch { /* ignore */ }
    });
  });

  setInterval(() => {
    const now = Date.now();
    if (now - lastMux > TELEMETRY.staleAfterMs) {
      $('ai-source').textContent = '--';
      $('ai-armed').textContent = '--';
    }
    if (now - lastAi > TELEMETRY.staleAfterMs) {
      $('ai-behavior').textContent = '--';
    }
  }, 1000);

  render();
}
