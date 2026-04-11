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

let selectedStepDeg = 1.0;
let jogRepeatTimer = null;
let activeJogKey = null;
let latestState = {};

function setText(id, value) {
  const node = document.getElementById(id);
  if (node) {
    node.textContent = value;
  }
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

function readAbsoluteTargets() {
  return {
    pan_deg: Number(document.getElementById("pan-absolute")?.value ?? 90),
    tilt_deg: Number(document.getElementById("tilt-absolute")?.value ?? 90),
  };
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
    ArrowLeft: { axis: "pan", direction: 1 },
    ArrowRight: { axis: "pan", direction: -1 },
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
  document.getElementById("mode-manual")?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/mode", { mode: "manual" });
    await refreshUi();
  });

  document.getElementById("mode-auto")?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/mode", { mode: "auto" });
    await refreshUi();
  });

  document.getElementById("capture-target")?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/action", { action: "capture_target" });
    await refreshUi();
  });

  document.getElementById("print-diag")?.addEventListener("click", async () => {
    await postJson("/api/cmd/action", { action: "print_diag" });
    await refreshUi();
  });

  document.getElementById("recenter")?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/move-to", { pan_deg: 90.0, tilt_deg: 90.0 });
    await refreshUi();
  });

  document.getElementById("move-to")?.addEventListener("click", async () => {
    await stopJogging();
    await postJson("/api/cmd/move-to", readAbsoluteTargets());
    await refreshUi();
  });

  document.getElementById("load-current")?.addEventListener("click", () => {
    const panInput = document.getElementById("pan-absolute");
    const tiltInput = document.getElementById("tilt-absolute");
    if (panInput && typeof latestState.pan_deg === "number") {
      panInput.value = String(latestState.pan_deg);
    }
    if (tiltInput && typeof latestState.tilt_deg === "number") {
      tiltInput.value = String(latestState.tilt_deg);
    }
  });

  document.getElementById("sync-time")?.addEventListener("click", async () => {
    await postJson("/api/cmd/time", {
      unix_utc: Math.floor(Date.now() / 1000),
      time_scale: 1.0,
    });
    await refreshUi();
  });
}

async function refreshUi() {
  try {
    const [health, state] = await Promise.all([loadHealth(), loadState()]);
    latestState = state;
    setText("backend-status", health.ok ? "Online" : "Offline");
    setText("mqtt-status", health.mqtt_connected ? "Connected" : "Disconnected");
    setText("remote-status", health.remote_online ? "Online" : "Offline");
    setText("remote-mode", state.mode ?? "Unknown");
    setText("remote-time", formatUtc(state.remote_utc));
    setText("time-source", state.time_source ?? "unknown");
    setText("state-updated", formatUnixMs(state.last_state_update_unix_ms));
    setText("state-output", JSON.stringify(state, null, 2));
  } catch (error) {
    setText("backend-status", "Error");
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
