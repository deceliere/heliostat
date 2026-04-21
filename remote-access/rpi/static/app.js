async function loadHealth() {
  const response = await fetch("/api/health", { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`health request failed: ${response.status}`);
  }
  return response.json();
}

async function loadState() {
  const response = await fetch("/api/state", { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`state request failed: ${response.status}`);
  }
  return response.json();
}

async function loadPresets() {
  const response = await fetch("/api/presets", { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`presets request failed: ${response.status}`);
  }
  return response.json();
}

async function loadDriftSamples() {
  const response = await fetch("/api/drift", { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`drift request failed: ${response.status}`);
  }
  return response.json();
}

async function postJson(url, payload) {
  const response = await fetch(url, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(payload),
  });

  if (!response.ok) {
    throw new Error(`${url} failed: ${response.status}`);
  }

  return response.json();
}

let selectedStepDeg = 0.1;
let jogRepeatTimer = null;
let activeJogKey = null;
let latestState = {};
let latestPresets = { site_locations: [], beam_directions: [] };
let latestDriftSamples = [];
let driftBaseline = null;
let scanSpeedSlider = null;

function scanSliderToMoveSpeedDegPerSec(speedValue) {
  const speed = Math.max(1, Math.min(20, Number(speedValue) || 6));
  return 1 + ((speed - 1) * ((120 - 1) / 19));
}

function formatScanSpeedDegPerSec(speedDegPerSec) {
  if (speedDegPerSec < 10) {
    return `${speedDegPerSec.toFixed(1)}°/s`;
  }
  return `${speedDegPerSec.toFixed(0)}°/s`;
}

function readScanRangeOverrideDeg() {
  const input = document.getElementById("scan-width");
  const widthDeg = Number(input?.value ?? "");
  if (!Number.isFinite(widthDeg) || widthDeg <= 0.0) {
    return null;
  }
  return widthDeg / 2.0;
}

function suppressNativeTouchBehavior(element) {
  if (!element) {
    return;
  }

  const preventNative = (event) => {
    event.preventDefault();
  };

  element.addEventListener("touchstart", preventNative, { passive: false });
  element.addEventListener("touchend", preventNative, { passive: false });
  element.addEventListener("touchcancel", preventNative, { passive: false });
}

function setText(id, value) {
  const node = document.getElementById(id);
  if (node) {
    node.textContent = value;
  }
}

function setStatusPill(id, value, tone = "slate", blink = false) {
  const node = document.getElementById(id);
  if (!node) {
    return;
  }

  node.textContent = value;
  node.className = `status-pill tone-${tone}${blink ? " blink" : ""}`;
}

function formatUtc(unixUtc) {
  if (!unixUtc) {
    return "Unknown";
  }
  return new Date(unixUtc * 1000).toISOString().replace(".000Z", "Z");
}

function formatUnixMs(unixMs) {
  if (!unixMs) {
    return "Unknown";
  }
  return new Date(unixMs).toISOString().replace(".000Z", "Z");
}

function formatAnglePair(a, b, suffix = "°") {
  if (typeof a !== "number" || typeof b !== "number") {
    return "Unknown";
  }
  return `${a.toFixed(2)} / ${b.toFixed(2)}${suffix}`;
}

function formatAngleValue(value, suffix = "°") {
  if (typeof value !== "number") {
    return "—";
  }
  return `${value.toFixed(2)}${suffix}`;
}

function voltageTone(voltage) {
  if (typeof voltage !== "number") {
    return "amber";
  }
  if (voltage >= 12.0) {
    return "green";
  }
  if (voltage >= 11.0) {
    return "amber";
  }
  return "red";
}

function renderDriftSamples() {
  const body = document.getElementById("drift-samples-body");
  if (!body) {
    return;
  }

  if (!latestDriftSamples.length) {
    body.innerHTML = '<tr><td colspan="9">No samples</td></tr>';
    return;
  }

  body.innerHTML = latestDriftSamples
    .slice()
    .reverse()
    .map((sample) => {
      const state = sample.state ?? {};
      const time = formatUnixMs(sample.timestamp_unix_ms);
      const label = sample.label || "—";
      const panCorrection = Number.isFinite(Number(sample.pan_correction_deg))
        ? Number(sample.pan_correction_deg).toFixed(2)
        : "—";
      const tiltCorrection = Number.isFinite(Number(sample.tilt_correction_deg))
        ? Number(sample.tilt_correction_deg).toFixed(2)
        : "—";
      const pan = typeof state.pan_deg === "number" ? state.pan_deg.toFixed(2) : "—";
      const tilt = typeof state.tilt_deg === "number" ? state.tilt_deg.toFixed(2) : "—";
      const sun = formatAnglePair(state.sun_bearing_deg, state.sun_elevation_deg, "°");
      const error = formatAnglePair(state.pan_tracking_error_deg, state.tilt_tracking_error_deg, "°");
      const note = sample.note || "—";
      return `<tr>
        <td>${time}</td>
        <td>${label}</td>
        <td>${panCorrection}</td>
        <td>${tiltCorrection}</td>
        <td>${pan}</td>
        <td>${tilt}</td>
        <td>${sun}</td>
        <td>${error}</td>
        <td>${note}</td>
      </tr>`;
    })
    .join("");
}

function snapshotDriftBaseline() {
  if (typeof latestState.pan_deg !== "number" || typeof latestState.tilt_deg !== "number") {
    return null;
  }

  return {
    timestamp_unix_ms: Date.now(),
    mode: latestState.mode ?? "unknown",
    pan_deg: latestState.pan_deg,
    tilt_deg: latestState.tilt_deg,
    pan_target_deg: latestState.pan_target_deg,
    tilt_target_deg: latestState.tilt_target_deg,
    sun_bearing_deg: latestState.sun_bearing_deg,
    sun_elevation_deg: latestState.sun_elevation_deg,
    predicted_pan_deg: latestState.predicted_pan_deg,
    predicted_tilt_deg: latestState.predicted_tilt_deg,
  };
}

function updateDriftBaselinePill() {
  if (!driftBaseline) {
    setStatusPill("drift-baseline", "Not armed", "amber");
    return;
  }

  setStatusPill(
    "drift-baseline",
    `${formatUnixMs(driftBaseline.timestamp_unix_ms)} | ${driftBaseline.pan_deg.toFixed(2)} / ${driftBaseline.tilt_deg.toFixed(2)}`,
    "slate"
  );
}

function readAbsoluteTargets() {
  return {
    pan_deg: Number(document.getElementById("pan-absolute")?.value ?? 90),
    tilt_deg: Number(document.getElementById("tilt-absolute")?.value ?? 90),
  };
}

function readApproxTargetDirection() {
  return {
    bearing_deg: Number(document.getElementById("approx-bearing")?.value ?? 180),
    elevation_deg: Number(document.getElementById("approx-elevation")?.value ?? 0),
  };
}

function readSiteLocation() {
  return {
    latitude_deg: Number(document.getElementById("site-latitude")?.value ?? 0),
    longitude_deg: Number(document.getElementById("site-longitude")?.value ?? 0),
  };
}

function renderPresetSelect(selectId, items) {
  const select = document.getElementById(selectId);
  if (!select) {
    return;
  }
  const currentValue = select.value;
  select.innerHTML = '<option value="">No preset</option>';
  items.forEach((item) => {
    const option = document.createElement("option");
    option.value = item.name;
    option.textContent = item.name;
    select.appendChild(option);
  });
  if (items.some((item) => item.name === currentValue)) {
    select.value = currentValue;
  }
}

function renderPresets() {
  renderPresetSelect("site-preset-select", latestPresets.site_locations ?? []);
  renderPresetSelect("beam-preset-select", latestPresets.beam_directions ?? []);
}

function applySitePresetToInputs(preset) {
  if (!preset) {
    return;
  }
  const latitudeInput = document.getElementById("site-latitude");
  const longitudeInput = document.getElementById("site-longitude");
  const nameInput = document.getElementById("site-preset-name");
  if (latitudeInput) latitudeInput.value = String(preset.latitude_deg);
  if (longitudeInput) longitudeInput.value = String(preset.longitude_deg);
  if (nameInput) nameInput.value = preset.name;
}

function applyBeamPresetToInputs(preset) {
  if (!preset) {
    return;
  }
  const bearingInput = document.getElementById("approx-bearing");
  const elevationInput = document.getElementById("approx-elevation");
  const nameInput = document.getElementById("beam-preset-name");
  if (bearingInput) bearingInput.value = String(preset.bearing_deg);
  if (elevationInput) elevationInput.value = String(preset.elevation_deg);
  if (nameInput) nameInput.value = preset.name;
}

async function sendJog(axis, direction, multiplier = 1.0) {
  await postJson("/api/cmd/mode", { mode: "manual" });
  await postJson("/api/cmd/jog", {
    axis,
    delta_deg: direction * selectedStepDeg * multiplier,
  });
}

async function stopJogging() {
  if (jogRepeatTimer !== null) {
    window.clearInterval(jogRepeatTimer);
    jogRepeatTimer = null;
  }
  activeJogKey = null;
}

function startJogging(axis, direction) {
  const performJog = async () => {
    await sendJog(axis, direction);
    await refreshUi();
  };

  stopJogging().catch((error) => console.error(error));
  performJog().catch((error) => console.error(error));
  jogRepeatTimer = window.setInterval(() => {
    performJog().catch((error) => console.error(error));
  }, 140);
}

function bindStepButtons() {
  const buttons = document.querySelectorAll(".step-button");
  const activeButton = document.querySelector(".step-button.active");
  if (activeButton) {
    selectedStepDeg = Number(activeButton.dataset.step ?? "0.1");
  }
  buttons.forEach((button) => {
    button.addEventListener("dblclick", (event) => {
      event.preventDefault();
    });
    button.addEventListener("click", () => {
      selectedStepDeg = Number(button.dataset.step ?? "1");
      buttons.forEach((candidate) => candidate.classList.remove("active"));
      button.classList.add("active");
    });
  });
}

function bindJogButtons() {
  const jogButtons = document.querySelectorAll(".jog-button[data-axis]");
  jogButtons.forEach((button) => {
    const axis = button.dataset.axis;
    const direction = Number(button.dataset.direction ?? "0");

    suppressNativeTouchBehavior(button);
    button.addEventListener("dblclick", (event) => {
      event.preventDefault();
    });
    button.addEventListener("contextmenu", (event) => {
      event.preventDefault();
    });
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      startJogging(axis, direction);
    });
  });

  window.addEventListener("pointerup", () => {
    stopJogging().catch((error) => console.error(error));
  });
  window.addEventListener("pointercancel", () => {
    stopJogging().catch((error) => console.error(error));
  });
}

function bindKeyboardJog() {
  const keyMap = {
    ArrowLeft: { axis: "pan", direction: -1 },
    ArrowRight: { axis: "pan", direction: 1 },
    ArrowUp: { axis: "tilt", direction: -1 },
    ArrowDown: { axis: "tilt", direction: 1 },
  };

  window.addEventListener("keydown", (event) => {
    if (event.target instanceof HTMLInputElement) {
      return;
    }
    const entry = keyMap[event.key];
    if (!entry) {
      return;
    }

    event.preventDefault();
    const multiplier = event.shiftKey ? 10.0 : event.altKey ? 0.1 : 1.0;
    if (activeJogKey === `${event.key}:${multiplier}`) {
      return;
    }

    stopJogging().catch((error) => console.error(error));
    activeJogKey = `${event.key}:${multiplier}`;

    const performJog = async () => {
      await sendJog(entry.axis, entry.direction, multiplier);
      await refreshUi();
    };

    performJog().catch((error) => console.error(error));
    jogRepeatTimer = window.setInterval(() => {
      performJog().catch((error) => console.error(error));
    }, 140);
  });

  window.addEventListener("keyup", (event) => {
    if (keyMap[event.key]) {
      stopJogging().catch((error) => console.error(error));
    }
  });
}

function bindControlButtons() {
  const manualButton = document.getElementById("mode-manual");
  const autoButton = document.getElementById("mode-auto");
  const captureButton = document.getElementById("capture-target");
  const printDiagButton = document.getElementById("print-diag");
  const recenterButton = document.getElementById("recenter");
  const moveToButton = document.getElementById("move-to");
  const loadCurrentButton = document.getElementById("load-current");
  const applySiteLocationButton = document.getElementById("apply-site-location");
  const loadSiteLocationButton = document.getElementById("load-site-location");
  const loadSitePresetButton = document.getElementById("load-site-preset");
  const saveSitePresetButton = document.getElementById("save-site-preset");
  const deleteSitePresetButton = document.getElementById("delete-site-preset");
  const moveApproxTargetButton = document.getElementById("move-approx-target");
  const loadApproxTargetButton = document.getElementById("load-approx-target");
  const loadBeamPresetButton = document.getElementById("load-beam-preset");
  const saveBeamPresetButton = document.getElementById("save-beam-preset");
  const deleteBeamPresetButton = document.getElementById("delete-beam-preset");
  const syncTimeButton = document.getElementById("sync-time");
  const startCoarseScanButton = document.getElementById("start-coarse-scan");
  const beamSeenButton = document.getElementById("beam-seen");
  const startFineScanButton = document.getElementById("start-fine-scan");
  const startMicroScanButton = document.getElementById("start-micro-scan");
  const stopScanButton = document.getElementById("stop-scan");
  const resumeBackwardScanButton = document.getElementById("resume-backward-scan");
  const resumeForwardScanButton = document.getElementById("resume-forward-scan");
  const useScanLockButton = document.getElementById("use-scan-lock");
  const armDriftBaselineButton = document.getElementById("arm-drift-baseline");
  const recordDriftSampleButton = document.getElementById("record-drift-sample");
  const exportDriftSamplesButton = document.getElementById("export-drift-samples");
  const clearDriftSamplesButton = document.getElementById("clear-drift-samples");
  scanSpeedSlider = document.getElementById("scan-speed");

  const refreshScanSpeedLabel = () => {
    if (!scanSpeedSlider) {
      return;
    }
    setStatusPill("scan-speed-value", formatScanSpeedDegPerSec(scanSliderToMoveSpeedDegPerSec(scanSpeedSlider.value)), "slate");
  };

  scanSpeedSlider?.addEventListener("input", refreshScanSpeedLabel);
  refreshScanSpeedLabel();

  const buildScanCommand = (stage, panCenter, tiltCenter) => {
    const command = {
      stage,
      center_pan_deg: panCenter,
      center_tilt_deg: tiltCenter,
      move_speed_deg_per_sec: scanSliderToMoveSpeedDegPerSec(scanSpeedSlider?.value),
      direction: "forward",
    };
    const rangeOverrideDeg = readScanRangeOverrideDeg();
    if (rangeOverrideDeg !== null) {
      command.range_pan_deg = rangeOverrideDeg;
      command.range_tilt_deg = rangeOverrideDeg;
    }
    return command;
  };

  manualButton?.addEventListener("click", async () => {
    await stopJogging();
    driftBaseline = snapshotDriftBaseline();
    updateDriftBaselinePill();
    await postJson("/api/cmd/mode", { mode: "manual" });
    await refreshUi();
  });

  autoButton?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/mode", { mode: "auto" });
    await refreshUi();
  });

  captureButton?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/action", { action: "capture_target" });
    await refreshUi();
  });

  printDiagButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/action", { action: "print_diag" });
    await refreshUi();
  });

  recenterButton?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/move-to", { pan_deg: 90.0, tilt_deg: 90.0 });
    await refreshUi();
  });

  moveToButton?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/move-to", readAbsoluteTargets());
    await refreshUi();
  });

  loadCurrentButton?.addEventListener("click", () => {
    const panInput = document.getElementById("pan-absolute");
    const tiltInput = document.getElementById("tilt-absolute");
    if (panInput && typeof latestState.pan_deg === "number") {
      panInput.value = String(latestState.pan_deg);
    }
    if (tiltInput && typeof latestState.tilt_deg === "number") {
      tiltInput.value = String(latestState.tilt_deg);
    }
  });

  applySiteLocationButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/location", readSiteLocation());
    await refreshUi();
  });

  loadSiteLocationButton?.addEventListener("click", () => {
    const latitudeInput = document.getElementById("site-latitude");
    const longitudeInput = document.getElementById("site-longitude");
    if (latitudeInput && typeof latestState.site_latitude_deg === "number") {
      latitudeInput.value = String(latestState.site_latitude_deg);
    }
    if (longitudeInput && typeof latestState.site_longitude_deg === "number") {
      longitudeInput.value = String(latestState.site_longitude_deg);
    }
  });

  loadSitePresetButton?.addEventListener("click", () => {
    const selectedName = document.getElementById("site-preset-select")?.value ?? "";
    const preset = (latestPresets.site_locations ?? []).find((item) => item.name === selectedName);
    applySitePresetToInputs(preset);
  });

  saveSitePresetButton?.addEventListener("click", async () => {
    const name = document.getElementById("site-preset-name")?.value ?? "";
    const result = await postJson("/api/presets/site", { name, ...readSiteLocation() });
    latestPresets = result.presets ?? latestPresets;
    renderPresets();
    const select = document.getElementById("site-preset-select");
    if (select) {
      select.value = name.trim();
    }
  });

  deleteSitePresetButton?.addEventListener("click", async () => {
    const selectedName = document.getElementById("site-preset-select")?.value ?? "";
    if (!selectedName) {
      return;
    }
    const result = await postJson("/api/presets/site/delete", { name: selectedName });
    latestPresets = result.presets ?? latestPresets;
    renderPresets();
  });

  moveApproxTargetButton?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/approx-target", readApproxTargetDirection());
    await refreshUi();
  });

  loadApproxTargetButton?.addEventListener("click", () => {
    const panInput = document.getElementById("pan-absolute");
    const tiltInput = document.getElementById("tilt-absolute");
    if (panInput && typeof latestState.approx_target_pan_deg === "number") {
      panInput.value = String(latestState.approx_target_pan_deg);
    }
    if (tiltInput && typeof latestState.approx_target_tilt_deg === "number") {
      tiltInput.value = String(latestState.approx_target_tilt_deg);
    }
  });

  loadBeamPresetButton?.addEventListener("click", () => {
    const selectedName = document.getElementById("beam-preset-select")?.value ?? "";
    const preset = (latestPresets.beam_directions ?? []).find((item) => item.name === selectedName);
    applyBeamPresetToInputs(preset);
  });

  saveBeamPresetButton?.addEventListener("click", async () => {
    const name = document.getElementById("beam-preset-name")?.value ?? "";
    const result = await postJson("/api/presets/beam", { name, ...readApproxTargetDirection() });
    latestPresets = result.presets ?? latestPresets;
    renderPresets();
    const select = document.getElementById("beam-preset-select");
    if (select) {
      select.value = name.trim();
    }
  });

  deleteBeamPresetButton?.addEventListener("click", async () => {
    const selectedName = document.getElementById("beam-preset-select")?.value ?? "";
    if (!selectedName) {
      return;
    }
    const result = await postJson("/api/presets/beam/delete", { name: selectedName });
    latestPresets = result.presets ?? latestPresets;
    renderPresets();
  });

  syncTimeButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/time", {
      unix_utc: Math.floor(Date.now() / 1000),
      time_scale: 1.0,
    });
    await refreshUi();
  });

  startCoarseScanButton?.addEventListener("click", async () => {
    const targets = readAbsoluteTargets();
    await postJson("/api/cmd/scan/start", buildScanCommand("coarse", targets.pan_deg, targets.tilt_deg));
    await refreshUi();
  });

  beamSeenButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/action", { action: "beam_seen" });
    await refreshUi();
  });

  startFineScanButton?.addEventListener("click", async () => {
    const panCenter =
      typeof latestState.scan_lock_pan_deg === "number" ? latestState.scan_lock_pan_deg : readAbsoluteTargets().pan_deg;
    const tiltCenter =
      typeof latestState.scan_lock_tilt_deg === "number" ? latestState.scan_lock_tilt_deg : readAbsoluteTargets().tilt_deg;
    await postJson("/api/cmd/scan/start", buildScanCommand("fine", panCenter, tiltCenter));
    await refreshUi();
  });

  startMicroScanButton?.addEventListener("click", async () => {
    const panCenter =
      typeof latestState.scan_lock_pan_deg === "number" ? latestState.scan_lock_pan_deg : readAbsoluteTargets().pan_deg;
    const tiltCenter =
      typeof latestState.scan_lock_tilt_deg === "number" ? latestState.scan_lock_tilt_deg : readAbsoluteTargets().tilt_deg;
    await postJson("/api/cmd/scan/start", buildScanCommand("micro", panCenter, tiltCenter));
    await refreshUi();
  });

  stopScanButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/scan/stop", {});
    await refreshUi();
  });

  resumeBackwardScanButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/scan/resume", { direction: "backward" });
    await refreshUi();
  });

  resumeForwardScanButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/scan/resume", { direction: "forward" });
    await refreshUi();
  });

  useScanLockButton?.addEventListener("click", async () => {
    const panInput = document.getElementById("pan-absolute");
    const tiltInput = document.getElementById("tilt-absolute");
    const hasLock =
      typeof latestState.scan_lock_pan_deg === "number" &&
      typeof latestState.scan_lock_tilt_deg === "number";
    if (!hasLock) {
      return;
    }
    if (panInput) {
      panInput.value = String(latestState.scan_lock_pan_deg);
    }
    if (tiltInput) {
      tiltInput.value = String(latestState.scan_lock_tilt_deg);
    }
    await postJson("/api/cmd/move-to", {
      pan_deg: latestState.scan_lock_pan_deg,
      tilt_deg: latestState.scan_lock_tilt_deg,
    });
    await refreshUi();
  });

  armDriftBaselineButton?.addEventListener("click", async () => {
    driftBaseline = snapshotDriftBaseline();
    updateDriftBaselinePill();
  });

  recordDriftSampleButton?.addEventListener("click", async () => {
    if (!driftBaseline) {
      driftBaseline = snapshotDriftBaseline();
      updateDriftBaselinePill();
      return;
    }

    const panCorrectionDeg =
      typeof latestState.pan_deg === "number" ? latestState.pan_deg - driftBaseline.pan_deg : null;
    const tiltCorrectionDeg =
      typeof latestState.tilt_deg === "number" ? latestState.tilt_deg - driftBaseline.tilt_deg : null;

    const payload = {
      label: document.getElementById("drift-label")?.value ?? "",
      pan_correction_deg: panCorrectionDeg,
      tilt_correction_deg: tiltCorrectionDeg,
      note: document.getElementById("drift-note")?.value ?? "",
      baseline: driftBaseline,
      state: latestState,
    };
    if (!Number.isFinite(payload.pan_correction_deg)) {
      delete payload.pan_correction_deg;
    }
    if (!Number.isFinite(payload.tilt_correction_deg)) {
      delete payload.tilt_correction_deg;
    }
    const result = await postJson("/api/drift/sample", payload);
    latestDriftSamples = result.samples ?? latestDriftSamples;
    renderDriftSamples();
  });

  exportDriftSamplesButton?.addEventListener("click", () => {
    const blob = new Blob([JSON.stringify(latestDriftSamples, null, 2)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `heliostat-drift-${new Date().toISOString().replace(/[:.]/g, "-")}.json`;
    anchor.click();
    URL.revokeObjectURL(url);
  });

  clearDriftSamplesButton?.addEventListener("click", async () => {
    const result = await postJson("/api/drift/clear", {});
    latestDriftSamples = result.samples ?? [];
    renderDriftSamples();
  });
}

async function refreshUi() {
  try {
    const [health, state, presets, drift] = await Promise.all([loadHealth(), loadState(), loadPresets(), loadDriftSamples()]);
    latestState = state;
    latestPresets = presets;
    latestDriftSamples = drift.samples ?? [];
    renderPresets();
    renderDriftSamples();
    const panInput = document.getElementById("pan-absolute");
    const tiltInput = document.getElementById("tilt-absolute");
    if (panInput) {
      if (typeof state.pan_min_deg === "number") panInput.min = String(state.pan_min_deg);
      if (typeof state.pan_max_deg === "number") panInput.max = String(state.pan_max_deg);
    }
    if (tiltInput) {
      if (typeof state.tilt_min_deg === "number") tiltInput.min = String(state.tilt_min_deg);
      if (typeof state.tilt_max_deg === "number") tiltInput.max = String(state.tilt_max_deg);
    }
    setStatusPill("backend-status", health.ok ? "Online" : "Offline", health.ok ? "green" : "red");
    setStatusPill("mqtt-status", health.mqtt_connected ? "Connected" : "Disconnected", health.mqtt_connected ? "green" : "red");
    setStatusPill("remote-status", health.remote_online ? "Online" : "Offline", health.remote_online ? "green" : "red");

    const mode = state.mode ?? "Unknown";
    if (mode === "auto") {
      setStatusPill("remote-mode", "Auto", "green", true);
    } else if (mode === "manual") {
      setStatusPill("remote-mode", "Manual", "rose");
    } else if (mode === "captured") {
      setStatusPill("remote-mode", "Captured", "amber");
    } else {
      setStatusPill("remote-mode", mode, "slate");
    }

    setStatusPill("remote-time", formatUtc(state.remote_utc), state.remote_utc ? "green" : "amber");

    const timeSource = state.time_source ?? "unknown";
    if (timeSource === "ntp") {
      setStatusPill("time-source", "NTP", "green");
    } else if (timeSource === "mqtt") {
      setStatusPill("time-source", "MQTT", "amber");
    } else {
      setStatusPill("time-source", timeSource, "slate");
    }

    setStatusPill("state-updated", formatUnixMs(state.last_state_update_unix_ms), state.last_state_update_unix_ms ? "slate" : "amber");
    if (typeof state.pan_voltage_v === "number") {
      setStatusPill("pan-voltage", `${state.pan_voltage_v.toFixed(1)} V`, voltageTone(state.pan_voltage_v));
    } else {
      setStatusPill("pan-voltage", "Unknown", "amber");
    }
    if (typeof state.tilt_voltage_v === "number") {
      setStatusPill("tilt-voltage", `${state.tilt_voltage_v.toFixed(1)} V`, voltageTone(state.tilt_voltage_v));
    } else {
      setStatusPill("tilt-voltage", "Unknown", "amber");
    }
    if (typeof state.site_latitude_deg === "number" && typeof state.site_longitude_deg === "number") {
      setStatusPill("site-gps", `${state.site_latitude_deg.toFixed(5)} / ${state.site_longitude_deg.toFixed(5)}`, "slate");
    } else {
      setStatusPill("site-gps", "Unknown", "amber");
    }

    const scanStage = state.scan_stage ?? "idle";
    const scanPaused = !!state.scan_paused;
    if (state.scan_active) {
      setStatusPill("scan-stage", scanStage, "green", true);
    } else if (scanPaused && scanStage !== "idle") {
      setStatusPill("scan-stage", `${scanStage} paused`, "amber");
    } else if (scanStage !== "idle") {
      setStatusPill("scan-stage", scanStage, "amber");
    } else {
      setStatusPill("scan-stage", "Idle", "slate");
    }

    const scanDirection = state.scan_direction ?? "forward";
    setStatusPill("scan-direction", scanDirection, scanDirection === "backward" ? "rose" : "slate");

    const scanPointIndex = Number(state.scan_point_index ?? 0);
    const scanPointsTotal = Number(state.scan_points_total ?? 0);
    setStatusPill("scan-progress", `${scanPointIndex} / ${scanPointsTotal}`, scanPointsTotal > 0 ? "slate" : "amber");

    if (typeof state.scan_range_pan_deg === "number" && typeof state.scan_range_tilt_deg === "number" &&
        state.scan_range_pan_deg > 0 && state.scan_range_tilt_deg > 0) {
      setStatusPill(
        "scan-width-current",
        `${(state.scan_range_pan_deg * 2).toFixed(2)} x ${(state.scan_range_tilt_deg * 2).toFixed(2)}`,
        "slate"
      );
    } else {
      setStatusPill("scan-width-current", "Default", "amber");
    }

    if (state.scan_lock_valid && typeof state.scan_lock_pan_deg === "number" && typeof state.scan_lock_tilt_deg === "number") {
      setStatusPill("scan-lock", `${state.scan_lock_pan_deg.toFixed(2)} / ${state.scan_lock_tilt_deg.toFixed(2)}`, "green");
    } else {
      setStatusPill("scan-lock", "None", "amber");
    }

    if (typeof state.pan_deg === "number" && typeof state.tilt_deg === "number") {
      setStatusPill("scan-current", `${state.pan_deg.toFixed(2)} / ${state.tilt_deg.toFixed(2)}`, "slate");
    } else {
      setStatusPill("scan-current", "Unknown", "amber");
    }

    if (state.approx_target_valid && typeof state.approx_target_pan_deg === "number" && typeof state.approx_target_tilt_deg === "number") {
      setStatusPill("approx-position", `${state.approx_target_pan_deg.toFixed(2)} / ${state.approx_target_tilt_deg.toFixed(2)}`, "green");
    } else {
      setStatusPill("approx-position", "Unknown", "amber");
    }

    if (typeof state.scan_move_speed_deg_per_sec === "number" && state.scan_move_speed_deg_per_sec > 0) {
      setStatusPill("scan-speed-active", formatScanSpeedDegPerSec(state.scan_move_speed_deg_per_sec), "slate");
    } else {
      setStatusPill("scan-speed-active", "Unknown", "amber");
    }

    setStatusPill("drift-sun", formatAnglePair(state.sun_bearing_deg, state.sun_elevation_deg), typeof state.sun_bearing_deg === "number" ? "slate" : "amber");
    setStatusPill("drift-normal", formatAnglePair(state.normal_bearing_deg, state.normal_elevation_deg), typeof state.normal_bearing_deg === "number" ? "slate" : "amber");
    setStatusPill("drift-target", formatAnglePair(state.target_bearing_deg, state.target_elevation_deg), typeof state.target_bearing_deg === "number" ? "slate" : "amber");
    setStatusPill("drift-desired-normal", formatAnglePair(state.desired_normal_bearing_deg, state.desired_normal_elevation_deg), typeof state.desired_normal_bearing_deg === "number" ? "slate" : "amber");
    setStatusPill("drift-predicted", formatAnglePair(state.predicted_pan_deg, state.predicted_tilt_deg), typeof state.predicted_pan_deg === "number" ? "slate" : "amber");
    const errorTone =
      typeof state.pan_tracking_error_deg === "number" &&
      Math.abs(state.pan_tracking_error_deg) < 0.2 &&
      typeof state.tilt_tracking_error_deg === "number" &&
      Math.abs(state.tilt_tracking_error_deg) < 0.2
        ? "green"
        : "amber";
    setStatusPill("drift-error", formatAnglePair(state.pan_tracking_error_deg, state.tilt_tracking_error_deg), typeof state.pan_tracking_error_deg === "number" ? errorTone : "amber");
    updateDriftBaselinePill();

    setText("state-actual", `${formatAngleValue(state.pan_deg)} / ${formatAngleValue(state.tilt_deg)}`);
    setText("state-target", `${formatAngleValue(state.pan_target_deg)} / ${formatAngleValue(state.tilt_target_deg)}`);
    setText("state-predicted", `${formatAngleValue(state.predicted_pan_deg)} / ${formatAngleValue(state.predicted_tilt_deg)}`);
    setText("state-error", `${formatAngleValue(state.pan_tracking_error_deg)} / ${formatAngleValue(state.tilt_tracking_error_deg)}`);
    setText("state-output", JSON.stringify(state, null, 2));
  } catch (error) {
    setStatusPill("backend-status", "Error", "red");
    setText("state-actual", "—");
    setText("state-target", "—");
    setText("state-predicted", "—");
    setText("state-error", "—");
    setText("state-output", String(error));
  }
}

bindStepButtons();
bindJogButtons();
bindKeyboardJog();
bindControlButtons();
document.addEventListener("selectstart", (event) => {
  if (event.target instanceof HTMLElement && event.target.closest("button")) {
    event.preventDefault();
  }
});
refreshUi();
setInterval(refreshUi, 250);
