async function loadHealth() {
  const response = await fetch("/api/health");
  if (!response.ok) {
    throw new Error(`health request failed: ${response.status}`);
  }
  return response.json();
}

async function loadState() {
  const response = await fetch("/api/state");
  if (!response.ok) {
    throw new Error(`state request failed: ${response.status}`);
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

function setText(id, value) {
  const node = document.getElementById(id);
  if (node) {
    node.textContent = value;
  }
}

function bindControls() {
  const manualButton = document.getElementById("mode-manual");
  const autoButton = document.getElementById("mode-auto");
  const captureButton = document.getElementById("capture-target");
  const recenterButton = document.getElementById("recenter");
  const panRate = document.getElementById("pan-rate");
  const tiltRate = document.getElementById("tilt-rate");
  const precision = document.getElementById("precision");

  manualButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/mode", { mode: "manual" });
    await refreshUi();
  });

  autoButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/mode", { mode: "auto" });
    await refreshUi();
  });

  captureButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/action", { action: "capture_target" });
    await refreshUi();
  });

  recenterButton?.addEventListener("click", async () => {
    await postJson("/api/cmd/action", { action: "recenter" });
    await refreshUi();
  });

  const publishManual = async () => {
    await postJson("/api/cmd/manual", {
      pan_rate: Number(panRate?.value ?? 0),
      tilt_rate: Number(tiltRate?.value ?? 0),
      precision: Boolean(precision?.checked),
    });
  };

  panRate?.addEventListener("change", publishManual);
  tiltRate?.addEventListener("change", publishManual);
  precision?.addEventListener("change", publishManual);
}

async function refreshUi() {
  try {
    const [health, state] = await Promise.all([loadHealth(), loadState()]);
    setText("backend-status", health.ok ? "Online" : "Offline");
    setText("mqtt-status", health.mqtt_connected ? "Connected" : "Disconnected");
    setText("remote-status", health.remote_online ? "Online" : "Offline");
    setText("remote-mode", state.mode ?? "Unknown");
    setText("state-output", JSON.stringify(state, null, 2));
  } catch (error) {
    setText("backend-status", "Error");
    setText("state-output", String(error));
  }
}

bindControls();
refreshUi();
setInterval(refreshUi, 5000);
