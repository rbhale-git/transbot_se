// ============================================================================
// recorder.js — session traffic recorder.
//
// While armed, captures every outgoing command, incoming telemetry message,
// and service response (via the ros.js traffic tap) with millisecond
// timestamps relative to recording start. Download produces a JSONL file —
// one {t, dir, topic, msg} object per line — which is the evidence trail for
// the Phase 4 safety pass ("what exactly was sent, and when").
// ============================================================================

import { onTraffic } from './ros.js';

const MAX_ENTRIES = 50000; // hard cap so a forgotten recorder can't eat the tab

let recording = false;
let buf = [];
let t0 = 0;

const listeners = []; // fn({recording, count})
export function onRecorderChange(fn) { listeners.push(fn); }

function notify() {
  for (const fn of listeners) fn({ recording, count: buf.length });
}

export function startRecording() {
  buf = [];
  t0 = performance.now();
  recording = true;
  notify();
}

export function stopRecording() {
  recording = false;
  notify();
}

export function isRecording() {
  return recording;
}

/** Download the captured session as a .jsonl file. */
export function downloadRecording() {
  if (buf.length === 0) return;
  const lines = buf.map((e) => JSON.stringify(e)).join('\n');
  const blob = new Blob([lines], { type: 'application/jsonl' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  const stamp = new Date().toISOString().replace(/[:.]/g, '-');
  a.href = url;
  a.download = `transbot-session-${stamp}.jsonl`;
  a.click();
  URL.revokeObjectURL(url);
}

export function initRecorder() {
  onTraffic((dir, topic, msg) => {
    if (!recording || buf.length >= MAX_ENTRIES) return;
    buf.push({ t: Math.round(performance.now() - t0), dir, topic, msg });
    // Notify sparsely — every message would thrash the DOM at telemetry rates.
    if (buf.length % 25 === 0) notify();
  });
}
