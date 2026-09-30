(function () {
  'use strict';

  async function openModelPicker() {
    try {
      document.body.dataset.modelPicker = 'opening';
      const response = await fetch('/api/model/pick', { method: 'POST', cache: 'no-store' });
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || 'Model picker failed.');
      if (!result.path) {
        document.body.dataset.modelPicker = 'cancelled';
        return;
      }
      document.body.dataset.modelPicker = 'selected';
      if (typeof window.CableTrayAcceptModel !== 'function') {
        throw new Error('3D model engine is not ready. Refresh the page with Ctrl+F5.');
      }
      await window.CableTrayAcceptModel(result);
    } catch (error) {
      document.body.dataset.modelPicker = 'error';
      console.error(error);
      alert('Load Model failed:\n' + error.message);
    } finally {
      if (document.body.dataset.modelPicker === 'opening') {
        document.body.dataset.modelPicker = '';
      }
    }
  }

  window.CableTrayOpenModel = openModelPicker;

  document.addEventListener('DOMContentLoaded', function () {
    const button = document.getElementById('loadModelBtn');
    if (button) button.addEventListener('click', openModelPicker);
  });
})();
