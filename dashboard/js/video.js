// ============================================================================
// video.js — MJPEG camera pane. The stream is just an <img> whose src points
// at web_video_server; on error we show NO SIGNAL and retry periodically.
// ============================================================================

const RETRY_MS = 5000;

let imgEl = null;
let noSignalEl = null;
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

export function setVideoUrl(url) {
  clearTimeout(retryTimer);
  currentUrl = url || '';
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
  retryTimer = setTimeout(() => {
    // Cache-bust so the browser actually re-attempts the stream.
    imgEl.src = currentUrl + (currentUrl.includes('?') ? '&' : '?') + 'retry=' + Date.now();
  }, RETRY_MS);
}
