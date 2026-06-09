// ============================================================================
// settings.js — live-tunable operating values, persisted in localStorage.
//
// The brief requires conservative default caps with the full range available
// in config; this module is how the operator raises (or lowers) them at
// runtime without editing files. Every setting:
//   - is clamped to a bound that itself never exceeds the hard driver limits
//   - takes effect immediately (publishers read the config objects live)
//   - survives reloads via localStorage
// Hard limits (MOTION.*.hardMax, servo ranges) are NOT settable here, ever.
// ============================================================================

import { MOTION, GIMBAL, ARM, clamp } from './config.js';

const STORAGE_KEY = 'transbot.settings';

// Definitions drive both the behavior and the SETTINGS panel UI.
export const SETTING_DEFS = {
  linearCap: {
    label: 'LIN CAP', unit: 'm/s', min: 0.05, max: MOTION.linear.hardMax, step: 0.05,
    get: () => MOTION.linear.cap,
    set: (v) => { MOTION.linear.cap = v; },
  },
  angularCap: {
    label: 'ANG CAP', unit: 'rad/s', min: 0.1, max: MOTION.angular.hardMax, step: 0.1,
    get: () => MOTION.angular.cap,
    set: (v) => { MOTION.angular.cap = v; },
  },
  gimbalStep: {
    label: 'GIMBAL STEP', unit: '°', min: 1, max: 15, step: 1,
    get: () => GIMBAL.stepDeg,
    set: (v) => { GIMBAL.stepDeg = v; },
  },
  armStep: {
    label: 'ARM STEP', unit: '°', min: 1, max: 15, step: 1,
    get: () => ARM.stepDeg,
    set: (v) => { ARM.stepDeg = v; },
  },
  armRunTime: {
    label: 'ARM RUN_TIME', unit: 'ms', min: 200, max: 2000, step: 100,
    get: () => ARM.runTimeMs,
    set: (v) => { ARM.runTimeMs = v; },
  },
};

function persist() {
  const out = {};
  for (const [key, def] of Object.entries(SETTING_DEFS)) out[key] = def.get();
  localStorage.setItem(STORAGE_KEY, JSON.stringify(out));
}

/** Apply one setting (clamped). Returns the value actually applied. */
export function applySetting(key, value) {
  const def = SETTING_DEFS[key];
  if (!def || !Number.isFinite(value)) return def?.get();
  const v = clamp(value, def.min, def.max);
  def.set(v);
  persist();
  return v;
}

/** Restore persisted settings at boot (silently ignores bad data). */
export function loadSettings() {
  let saved = {};
  try {
    saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}');
  } catch { /* corrupted storage — keep config defaults */ }
  for (const [key, def] of Object.entries(SETTING_DEFS)) {
    if (Number.isFinite(saved[key])) def.set(clamp(saved[key], def.min, def.max));
  }
}
