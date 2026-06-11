// Boots the dashboard module graph outside the browser with minimal DOM/ROSLIB
// stubs. If app.js throws during boot, this prints the same error the browser
// console would show. Run: node tools/smoke_boot.mjs
/* eslint-disable */

import { readFileSync } from 'node:fs';

// Only ids that really exist in index.html resolve — unknown ids return null
// exactly like the browser, so missing-element crashes reproduce here.
const html = readFileSync(new URL('../dashboard/index.html', import.meta.url), 'utf8');
const realIds = new Set([...html.matchAll(/id="([^"]+)"/g)].map((m) => m[1]));

const els = new Map();

function makeEl(id) {
  const el = {
    id,
    style: {},
    dataset: {},
    children: [],
    classList: { toggle() {}, add() {}, remove() {} },
    addEventListener() {},
    // browser parity: nodes appended into the tree become findable by id
    appendChild(c) { this.children.push(c); if (c.id) els.set(c.id, c); },
    querySelector() { return makeEl(`${id}-q`); },
    closest() { return makeEl(`${id}-closest`); },
    blur() {},
    set textContent(v) {}, get textContent() { return ''; },
    set innerHTML(v) {}, get innerHTML() { return ''; },
    value: '', title: '', disabled: false,
    min: 0, max: 0, step: 1,
    removeAttribute() {},
  };
  return el;
}

globalThis.document = {
  getElementById(id) {
    if (els.has(id)) return els.get(id); // dynamically appended elements
    if (!realIds.has(id)) return null; // match browser behavior
    if (!els.has(id)) els.set(id, makeEl(id));
    return els.get(id);
  },
  createElement(tag) { return makeEl(`<${tag}>`); },
  addEventListener() {},
  hidden: false,
  body: makeEl('body'),
};
globalThis.window = {
  addEventListener() {},
};
globalThis.localStorage = {
  store: {},
  getItem(k) { return this.store[k] ?? null; },
  setItem(k, v) { this.store[k] = String(v); },
};
Object.defineProperty(globalThis, 'navigator', {
  value: { getGamepads: () => [] },
  configurable: true,
});
globalThis.ROSLIB = {
  Ros: class { on() {} close() {} get isConnected() { return false; } },
  Topic: class { publish() {} subscribe() {} unsubscribe() {} },
  Message: class {},
  Service: class { callService() {} },
  ServiceRequest: class {},
};

try {
  await import('../dashboard/js/app.js');
  console.log('BOOT OK — no exception during module evaluation');
  // boot is the scope of this test: timers/polls left running would hit
  // stub limits (innerHTML is a no-op), not real bugs — stop here.
  process.exit(0);
} catch (e) {
  console.error('BOOT FAILED:');
  console.error(e);
  process.exit(1);
}
