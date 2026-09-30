(function () {
  'use strict';

  window.CableTrayDebugLog = function (level, event, details) {
    var record = {
      level: level || 'INFO',
      event: event || 'UNKNOWN',
      details: details || null,
      time: new Date().toISOString()
    };
    try {
      console.log('[CableTray]', record.level, record.event, record.details || '');
    } catch (_) {}
    try {
      fetch('/api/debug/log', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(record),
        keepalive: true
      }).catch(function () {});
    } catch (_) {}
  };

  window.addEventListener('error', function (event) {
    window.CableTrayDebugLog('ERROR', 'WINDOW_ERROR', {
      message: event.message,
      source: event.filename,
      line: event.lineno,
      column: event.colno
    });
  });

  window.addEventListener('unhandledrejection', function (event) {
    var reason = event.reason || {};
    window.CableTrayDebugLog('ERROR', 'UNHANDLED_REJECTION', {
      name: reason.name,
      message: reason.message || String(reason),
      stack: reason.stack
    });
  });

  var modelPickerBusy = false;

  async function openModelPicker() {
    if (modelPickerBusy) return;
    modelPickerBusy = true;
    var button = document.getElementById('loadModelBtn');
    if (button) button.disabled = true;
    window.CableTrayDebugLog('INFO', 'LOAD_MODEL_CLICK', {
      readyState: document.readyState,
      location: window.location.href
    });

    try {
      window.CableTrayDebugLog('INFO', 'PICK_REQUEST_START');

      var response = await fetch('/api/model/pick', {
        method: 'POST',
        cache: 'no-store'
      });

      var result = await response.json();

      window.CableTrayDebugLog('INFO', 'PICK_RESPONSE', {
        status: response.status,
        ok: response.ok,
        name: result.name,
        extension: result.extension,
        format: result.format,
        hasPath: !!result.path,
        hasUrl: !!result.url,
        nativeFormat: result.native_format
      });

      if (!response.ok) {
        throw new Error(result.error || 'Model picker failed.');
      }

      if (!result.path) {
        window.CableTrayDebugLog('INFO', 'PICK_CANCELLED');
        return;
      }

      if (typeof window.CableTrayAcceptModel !== 'function') {
        throw new Error('3D model engine is not ready.');
      }

      window.CableTrayDebugLog('INFO', 'HANDOFF_TO_MODEL_ENGINE', {
        extension: result.extension,
        format: result.format,
        name: result.name,
        hasUrl: !!result.url
      });

      await window.CableTrayAcceptModel(result);

      window.CableTrayDebugLog('INFO', 'MODEL_LOAD_COMPLETE');
    } catch (error) {
      window.CableTrayDebugLog('ERROR', 'MODEL_LOAD_FAILED', {
        name: error && error.name,
        message: error && error.message,
        stack: error && error.stack
      });
      console.error(error);
      alert('Load Model failed. Check F:\\Cable_tray\\logs\\cable_tray_debug.log');
    } finally {
      modelPickerBusy = false;
      if (button) button.disabled = false;
    }
  }

  document.addEventListener('DOMContentLoaded', function () {
    var button = document.getElementById('loadModelBtn');
    if (!button) {
      window.CableTrayDebugLog('ERROR', 'LOAD_MODEL_BUTTON_MISSING');
      return;
    }
    window.CableTrayDebugLog('INFO', 'LOAD_MODEL_HANDLER_REGISTERED');
    button.addEventListener('click', openModelPicker);
  });
})();
