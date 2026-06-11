// ============================================================================
// video.js — MJPEG camera pane. The stream is just an <img> whose src points
// at web_video_server; on error we show NO SIGNAL and retry periodically.
//
// The effective URL is baseUrl (from the network profile) + preset params
// (SD downscale by default — see VIDEO in config.js). The stream can be
// switched off entirely: every encoded client costs the Nano real CPU, so
// the operator can free it while an AI behavior runs its own feed.
// ============================================================================

const RETRY_MS = 5000;

let imgEl = null;
let noSignalEl = null;
let baseUrl = '';
let presetParams = '';
let enabled = true;
let currentUrl = '';
let retryTimer = null;

export function initVideo(imgElement, noSignalElement) {
  imgEl = imgElement;
  noSignalEl = noSignalElement;

  imgEl.addEventListener('error', () => {
    showNoSignal(true);
    scheduleRetry();
  });
  imgEl.addEventListener('load', () => {
    showNoSignal(false);
    // Lock the stage to the stream's true aspect ratio, so the bezel and
    // HUD hug the picture exactly regardless of camera resolution.
    const stage = imgEl.closest('.video-stage');
    if (stage && imgEl.naturalWidth > 0 && imgEl.naturalHeight > 0) {
      stage.style.aspectRatio = `${imgEl.naturalWidth} / ${imgEl.naturalHeight}`;
    }
  });
}

/** Set the profile's base stream URL (no quality params). */
export function setVideoUrl(url) {
  baseUrl = url || '';
  restart();
}

/** Set the quality params appended to the base URL (VIDEO preset). */
export function setStreamParams(params) {
  presetParams = params || '';
  restart();
}

/** Stream master switch — off frees one encoder on the Nano. */
export function setStreamEnabled(on) {
  enabled = on;
  restart();
}

function restart() {
  clearTimeout(retryTimer);
  currentUrl = enabled && baseUrl ? baseUrl + presetParams : '';
  noSignalEl.querySelector('span').textContent =
    enabled ? 'NO SIGNAL' : 'STREAM OFF';
  if (!currentUrl) {
    imgEl.removeAttribute('src');
    showNoSignal(true);
    return;
  }
  showNoSignal(true); // until first frame loads
  imgEl.src = currentUrl;
}

function showNoSignal(on) {
  noSignalEl.classList.toggle('hidden', !on);
  imgEl.classList.toggle('hidden', on);
}

function scheduleRetry() {
  clearTimeout(retryTimer);
  if (!currentUrl) return;
  retryTimer = setTimeout(kickVideo, RETRY_MS);
}

/** Force the stream to reconnect. MJPEG streams can END without firing an
 *  error event (e.g. the robot-side server restarts), leaving a frozen pane
 *  with no retry scheduled — callers kick on rosbridge reconnect. */
export function kickVideo() {
  if (!currentUrl) return;
  // Cache-bust so the browser actually re-attempts the stream.
  imgEl.src = currentUrl + (currentUrl.includes('?') ? '&' : '?') + 'retry=' + Date.now();
}
