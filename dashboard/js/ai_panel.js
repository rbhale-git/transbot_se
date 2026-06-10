// ============================================================================
// ai_panel.js — ARM/DISARM switch for AI chassis control + arbitration status.
//
// Publishes /ai/enabled (Bool). The robot-side cmd_vel_mux is the enforcement
// point — this panel is just the operator's switch and readout. Policy:
// disarmed on every page load and on every (re)connect; e-stop disarms.
// Status comes from /mux/status (who is driving) and /ai/status (behavior
// state), both JSON-in-String, marked stale after TELEMETRY.staleAfterMs.
// ============================================================================

import { TOPICS, TELEMETRY } from './config.js';
import { publish, subscribe, onStatus } from './ros.js';
import { onEStop } from './keyboard.js';

const $ = (id) => document.getElementById(id);

let armed = false;
let lastMux = 0;
let lastAi = 0;

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

export function initAiPanel() {
  $('ai-arm-btn').addEventListener('click', () => setArmed(!armed));
  onEStop(() => setArmed(false));   // e-stop also pulls the AI switch

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
