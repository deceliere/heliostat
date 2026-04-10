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

let manualPublishTimer = null;

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
  const stopManualButton = document.getElementById("stop-manual");
  const panRate = document.getElementById("pan-rate");
  const tiltRate = document.getElementById("tilt-rate");
  const precision = document.getElementById("precision");

  const readManualCommand = () => ({
    pan_rate: Number(panRate?.value ?? 0),
    tilt_rate: Number(tiltRate?.value ?? 0),
    precision: Boolean(precision?.checked),
  });

  const stopManualPublishing = async () => {
    if (manualPublishTimer !== null) {
      window.clearInterval(manualPublishTimer);
      manualPublishTimer = null;
    }

    panRate.value = "0";
    tiltRate.value = "0";
    await postJson("/api/cmd/manual", { ...readManualCommand(), pan_rate: 0, tilt_rate: 0 });
  };

  const startManualPublishing = async () => {
    const publish = async () => {
      await postJson("/api/cmd/manual", readManualCommand());
    };

    if (manualPublishTimer === null) {
      manualPublishTimer = window.setInterval(() => {
        publish().catch((error) => {
          console.error(error);
        });
      }, 100);
    }

    await postJson("/api/cmd/mode", { mode: "manual" });
    await publish();
    await refreshUi();
  };

  manualButton?.addEventListener("click", async () => {
    await stopManualPublishing();
    await postJson("/api/cmd/mode", { mode: "manual" });
    await refreshUi();
  });

  autoButton?.addEventListener("click", async () => {
    await stopManualPublishing();
    await postJson("/api/cmd/mode", { mode: "auto" });
    await refreshUi();
  });

  captureButton?.addEventListener("click", async () => {
    await stopManualPublishing();
    await postJson("/api/cmd/action", { action: "capture_target" });
    await refreshUi();
  });

  recenterButton?.addEventListener("click", async () => {
    await stopManualPublishing();
    await postJson("/api/cmd/action", { action: "recenter" });
    await refreshUi();
  });

  stopManualButton?.addEventListener("click", async () => {
    await stopManualPublishing();
    await refreshUi();
  });

  panRate?.addEventListener("input", startManualPublishing);
  tiltRate?.addEventListener("input", startManualPublishing);
  precision?.addEventListener("change", startManualPublishing);
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
setInterval(refreshUi, 250);
