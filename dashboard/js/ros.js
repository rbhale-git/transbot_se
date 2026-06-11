// ============================================================================
// ros.js — single owner of the rosbridge connection.
//
// Wraps ROSLIB.Ros with: auto-reconnect (capped backoff), cached topic
// handles, and a status callback the UI and the safety layer subscribe to.
// Publishing while disconnected is a silent no-op (the deadman layer
// separately guarantees the robot was told to stop before the link died).
// ============================================================================

import { RECONNECT } from './config.js';

/* global ROSLIB */

let ros = null;
let currentUrl = null;
let reconnectDelay = RECONNECT.initialDelayMs;
let reconnectTimer = null;
let intentionalClose = false;

const topicCache = new Map();   // topicName -> ROSLIB.Topic
const statusListeners = [];     // fn('connecting' | 'connected' | 'disconnected')
const trafficTaps = [];         // fn(direction: 'out'|'in'|'svc', name, payload)

export function onStatus(fn) {
  statusListeners.push(fn);
}

/** Observe all topic/service traffic (used by the recorder). */
export function onTraffic(fn) {
  trafficTaps.push(fn);
}

function emitTraffic(direction, name, payload) {
  for (const fn of trafficTaps) fn(direction, name, payload);
}

function emitStatus(status) {
  for (const fn of statusListeners) fn(status);
}

export function isConnected() {
  return ros !== null && ros.isConnected;
}

/** Connect (or switch) to a rosbridge URL. Tears down any prior link. */
export function connect(url) {
  disconnect();
  currentUrl = url;
  intentionalClose = false;
  openSocket();
}

export function disconnect() {
  intentionalClose = true;
  clearTimeout(reconnectTimer);
  topicCache.clear();
  if (ros) {
    ros.close();
    ros = null;
  }
}

function openSocket() {
  emitStatus('connecting');
  const sock = new ROSLIB.Ros({ url: currentUrl });
  ros = sock;

  sock.on('connection', () => {
    if (ros !== sock) return; // stale socket from a previous profile
    reconnectDelay = RECONNECT.initialDelayMs;
    emitStatus('connected');
  });

  // 'error' fires alongside 'close' on failure; close drives the retry.
  sock.on('error', () => {});

  sock.on('close', () => {
    // A profile switch closes the old socket asynchronously; its late close
    // event must not flip the status or spawn a second reconnect loop.
    if (ros !== sock) return;
    topicCache.clear();
    emitStatus('disconnected');
    if (!intentionalClose) {
      clearTimeout(reconnectTimer);
      reconnectTimer = setTimeout(openSocket, reconnectDelay);
      reconnectDelay = Math.min(reconnectDelay * 2, RECONNECT.maxDelayMs);
    }
  });
}

function getTopic(topicCfg) {
  let t = topicCache.get(topicCfg.name);
  if (!t) {
    t = new ROSLIB.Topic({
      ros,
      name: topicCfg.name,
      messageType: topicCfg.type,
      latch: topicCfg.latch === true,
    });
    topicCache.set(topicCfg.name, t);
  }
  return t;
}

/** Explicitly advertise a config-defined topic so its registration can be
 *  verified — roslib otherwise advertises lazily on the first publish. */
export function advertise(topicCfg) {
  if (!isConnected()) return;
  getTopic(topicCfg).advertise();
}

/** Drop the socket and let auto-reconnect build a FRESH session — the
 *  workaround for the robot rosbridge's corrupt publisher registration. */
export function kickConnection() {
  if (ros) ros.close(); // close handler clears caches and schedules reconnect
}

/** Publish `msg` (plain object) to a config-defined topic. No-op offline. */
export function publish(topicCfg, msg) {
  if (!isConnected()) return false;
  getTopic(topicCfg).publish(new ROSLIB.Message(msg));
  emitTraffic('out', topicCfg.name, msg);
  return true;
}

/** Subscribe to a config-defined topic. Returns an unsubscribe function. */
export function subscribe(topicCfg, callback) {
  if (!isConnected()) return () => {};
  const t = getTopic(topicCfg);
  const wrapped = (msg) => {
    emitTraffic('in', topicCfg.name, msg);
    callback(msg);
  };
  t.subscribe(wrapped);
  return () => t.unsubscribe(wrapped);
}

/** Call a config-defined service. Resolves with the response, rejects on
 *  failure or after `timeoutMs` (rosbridge won't answer for dead services). */
export function callService(serviceCfg, args = {}, timeoutMs = 3000) {
  return new Promise((resolve, reject) => {
    if (!isConnected()) return reject(new Error('not connected'));
    const svc = new ROSLIB.Service({
      ros,
      name: serviceCfg.name,
      serviceType: serviceCfg.type,
    });
    const timer = setTimeout(() => reject(new Error('service timeout')), timeoutMs);
    svc.callService(
      new ROSLIB.ServiceRequest(args),
      (resp) => {
        clearTimeout(timer);
        emitTraffic('svc', serviceCfg.name, resp);
        resolve(resp);
      },
      (err) => { clearTimeout(timer); reject(new Error(err)); },
    );
  });
}
