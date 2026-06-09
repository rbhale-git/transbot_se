// ============================================================================
// latency.js — round-trip time over the rosbridge WebSocket.
//
// Calls the standard /rosapi/get_time service (ships with rosbridge_server)
// on an interval and reports the wall-clock round trip. On Wi-Fi teleop the
// difference between 40 ms and 800 ms of lag is a safety-relevant fact the
// operator should see at all times.
// ============================================================================

import { SERVICES, TELEMETRY } from './config.js';
import { callService, isConnected, onStatus } from './ros.js';

const listeners = []; // fn(rttMs | null)
export function onLatency(fn) { listeners.push(fn); }

function emit(rtt) {
  for (const fn of listeners) fn(rtt);
}

let timer = null;

async function measure() {
  if (!isConnected()) return;
  const t0 = performance.now();
  try {
    await callService(SERVICES.getTime, {}, TELEMETRY.latencyPollMs);
    emit(Math.round(performance.now() - t0));
  } catch {
    emit(null); // service missing or timed out — show unknown, not stale
  }
}

export function initLatency() {
  onStatus((status) => {
    clearInterval(timer);
    timer = null;
    if (status === 'connected') {
      measure();
      timer = setInterval(measure, TELEMETRY.latencyPollMs);
    } else {
      emit(null);
    }
  });
}
