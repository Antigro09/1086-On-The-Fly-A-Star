"use strict";

(() => {
  const $ = id => document.getElementById(id);
  const clone = value => JSON.parse(JSON.stringify(value));
  const finite = value => typeof value === "number" && Number.isFinite(value);
  const number = value => Number(value);
  const deg = radians => radians * 180 / Math.PI;
  const rad = degrees => degrees * Math.PI / 180;
  const wrap = angle => Math.atan2(Math.sin(angle), Math.cos(angle));
  const fmt = (value, digits = 2) => Number.isFinite(Number(value)) ? Number(value).toFixed(digits).replace(/\.00$/, "") : "—";
  const big = value => { try { return BigInt(value ?? 0); } catch { return 0n; } };
  const DEFAULT = {
    field: {season: "offseason-synthetic", map_id: "dashboard-lab-8x4", geometry_revision: "1"},
    start: {x_m: 1, y_m: 2, heading_rad: 0, vx_mps: 0, vy_mps: 0, omega_radps: 0},
    goal: {x_m: 7, y_m: 2, heading_rad: Math.PI / 2, position_tolerance_m: .05, heading_tolerance_rad: .05, velocity_tolerance_mps: .05},
    footprint: {length_m: .6, width_m: .6},
    constraints: {max_speed_mps: 2, max_acceleration_mps2: 2, max_angular_speed_radps: 3},
    bounds: {min_x_m: 0, min_y_m: 0, max_x_m: 8, max_y_m: 4},
    obstacles: [{id: "static-lab", x_m: 4, y_m: 2, radius_m: .55, uncertainty_margin_m: .05, dynamic: false},
      {id: "dynamic-lab", x_m: 5.5, y_m: 3, radius_m: .25, uncertainty_margin_m: .05, dynamic: true}],
    field_map: null
  };
  function validateScenario(value) {
    if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error("Scenario must be an object.");
    const b = value.bounds;
    if (!b || ![b.min_x_m, b.min_y_m, b.max_x_m, b.max_y_m].every(v => finite(v) && Math.abs(v) <= 1000)
        || b.max_x_m <= b.min_x_m || b.max_y_m <= b.min_y_m || b.max_x_m - b.min_x_m > 100 || b.max_y_m - b.min_y_m > 100) throw new Error("Field bounds must be finite, ordered, and at most 100 × 100 m.");
    for (const key of ["start", "goal"]) if (!value[key] || ![value[key].x_m, value[key].y_m, value[key].heading_rad].every(v => finite(v) && Math.abs(v) <= 10000)) throw new Error("Start and goal coordinates/headings must be finite and bounded.");
    const f = value.footprint, c = value.constraints;
    if (!f || ![f.length_m, f.width_m].every(v => finite(v) && v > 0 && v <= 20)) throw new Error("Footprint dimensions must be positive and at most 20 m.");
    if (!c || ![c.max_speed_mps, c.max_acceleration_mps2, c.max_angular_speed_radps].every(v => finite(v) && v > 0 && v <= 30)) throw new Error("Planner limits must be positive finite values no greater than 30.");
    if (!Array.isArray(value.obstacles) || value.obstacles.length > 4096 || value.obstacles.some(o => !o || typeof o.id !== "string" || !o.id || ![o.x_m, o.y_m, o.radius_m, o.uncertainty_margin_m ?? 0].every(v => finite(v) && Math.abs(v) <= 1000) || o.radius_m <= 0 || (o.uncertainty_margin_m ?? 0) < 0)) throw new Error("Occupancy circles need bounded finite geometry and positive radii (maximum 4096).");
    if (value.field_map) {
      const map = value.field_map;
      if (!map.map || ![map.map.width_m, map.map.height_m].every(v => finite(v) && v > 0 && v <= 100) || !Array.isArray(map.obstacles) || map.obstacles.length > 256) throw new Error("Imported field geometry exceeds display limits.");
      for (const polygon of [map.boundary, ...map.obstacles]) for (const ring of [polygon?.outer, ...(polygon?.holes || [])]) if (!Array.isArray(ring) || ring.length > 257 || ring.some(p => !Array.isArray(p) || p.length !== 2 || !p.every(v => finite(v) && Math.abs(v) <= 1000))) throw new Error("Field rings need bounded finite vertices.");
    }
    return value;
  }
  function validateLongNumbers(value, key = "") {
    if (value && typeof value === "object") { for (const [name, child] of Object.entries(value)) validateLongNumbers(child, name); return; }
    if ((/(?:_us|_ns)$/.test(key) || ["epoch", "snapshot_id", "obstacle_map_version", "sequence", "generation"].includes(key)) && typeof value === "number" && !Number.isSafeInteger(value)) throw new Error(`${key} must be an exact decimal string when outside JavaScript's safe integer range.`);
  }
  function validateJsonNumbers(value) {
    const pending = [[value, 0]]; let count = 0;
    while (pending.length) {
      const [item, depth] = pending.pop();
      if (++count > 60000 || depth > 32) throw new Error("JSON structure exceeds display limits.");
      if (typeof item === "number" && !Number.isFinite(item)) throw new Error("JSON numeric values must be finite.");
      if (item && typeof item === "object") for (const child of Object.values(item)) pending.push([child, depth + 1]);
    }
  }
  let mode = "sandbox", scenario = clone(DEFAULT), state = null, plan = null;
  let connected = false, arrival = 0, pollInFlight = false, editRevision = 0, modeRevision = 0;
  let mutationQueue = Promise.resolve(), pendingMutations = 0, localError = "", busy = false, draftEditing = false;
  let draftInput = null, draftSaveTimer = null;
  let scenarioImportRevision = 0;
  let tool = "inspect", selectedObstacle = null, drag = null, zoom = 1, image = null, imageConfig = null;
  let replayTimer = null, replayCount = 0, replayIndex = 0, replayRevision = 0, replayRequestPending = false;
  let preview = null, previewFrame = null, previewLastTime = 0;
  const canvas = $("field-canvas"), ctx = canvas.getContext("2d");
  let view = {width: 800, height: 420, scale: 80, left: 50, bottom: 50};

  async function api(path, options = {}) {
    const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), 4500);
    try {
      const response = await fetch(path, {...options, signal: controller.signal, cache: "no-store",
        headers: options.body ? {"Content-Type": "application/json"} : undefined});
      const data = await response.json();
      if (!response.ok) throw new Error(data.detail || data.error || (data.errors || []).join("; ") || `HTTP ${response.status}`);
      return data;
    } finally { clearTimeout(timeout); }
  }

  function adaptScenario(data) {
    if (data.scenario) {
      const display = {...clone(DEFAULT), ...clone(data.scenario)};
      if (!display.start && data.robot?.pose) display.start = {...clone(DEFAULT.start), ...data.robot.pose, ...(data.robot.velocity || {})};
      return display;
    }
    const fallback = clone(DEFAULT);
    if (data.robot?.pose) fallback.start = {...fallback.start, ...data.robot.pose};
    if (Array.isArray(data.obstacles)) fallback.obstacles = clone(data.obstacles);
    const points = data.plan?.positions || [];
    if (points.length) fallback.goal = {...fallback.goal, ...points[points.length - 1]};
    return fallback;
  }

  function accept(data, expectedMode = mode, force = false) {
    if (expectedMode !== mode || (data.mode && data.mode !== mode)) return;
    state = data; connected = true; arrival = performance.now();
    if (localError.startsWith("Local server unavailable:")) localError = "";
    if (!draftEditing && (force || (!drag && pendingMutations === 0))) scenario = adaptScenario(data);
    const nextPlan = draftEditing ? null : data.plan || null;
    if (preview && (nextPlan?.request_id !== preview.requestId || nextPlan?.status !== "SUCCESS")) stopPreview("Stopped: plan changed or expired");
    plan = nextPlan;
    if (data.replay) { replayCount = number(data.replay.frame_count) || 0; replayIndex = number(data.replay.index) || 0; }
    renderUI(); draw();
  }

  async function poll() {
    if (pollInFlight || drag || pendingMutations || (mode === "replay" && replayRequestPending)) return;
    const expectedMode = mode, revision = modeRevision, edits = editRevision, replay = replayRevision;
    pollInFlight = true;
    try {
      const data = await api(`/api/state?mode=${expectedMode}`);
      if (revision === modeRevision && edits === editRevision && (expectedMode !== "replay" || replay === replayRevision)) accept(data, expectedMode);
    } catch (error) {
      if (revision === modeRevision) { connected = false; localError = `Local server unavailable: ${error.message}. Retry after starting the dashboard.`; renderUI(); draw(); }
    } finally { pollInFlight = false; }
  }

  function points() {
    if (!plan || plan.display_limit_exceeded || ["INVALID_INPUT", "REJECTED"].includes(plan.status)) return [];
    const values = plan.positions || [];
    return values.length <= 10000 ? values.filter(p => finite(p.x_m) && finite(p.y_m) && finite(p.heading_rad)) : [];
  }
  function pathLength(values) { let length = 0; for (let i = 1; i < values.length; i++) length += Math.hypot(values[i].x_m - values[i - 1].x_m, values[i].y_m - values[i - 1].y_m); return length; }
  function setText(id, value) { $(id).textContent = value; }
  function status() { return !freshHttp() ? "DISCONNECTED" : busy ? "PLANNING" : plan?.status || "IDLE"; }
  function freshHttp() { return connected && arrival > 0 && performance.now() - arrival < 1000; }
  function matchingContext() {
    if (!plan || !state || plan !== state.plan || typeof plan.request_id !== "string" || typeof plan.task_id !== "string" || ["epoch", "snapshot_id", "obstacle_map_version"].some(key => String(plan[key]) !== String(state[key]))) return false;
    if (mode === "sandbox") return !!plan.field && !!state.field && ["season", "map_id", "geometry_revision"].every(key => plan.field[key] === state.field[key]);
    return true;
  }
  function planCurrent() { return mode === "sandbox" && !draftEditing && freshHttp() && state?.connection?.connected && !state?.connection?.stale && matchingContext() && !busy && !pendingMutations && plan?.status === "SUCCESS" && points().length >= 2 && big(state?.robot_us) < big(plan.valid_until_us); }

  function renderUI() {
    const sandbox = mode === "sandbox";
    $("sandbox-controls").hidden = !sandbox; $("preview-controls").hidden = !sandbox;
    $("replay-controls").hidden = mode !== "replay"; $("replay-inspector").hidden = mode !== "replay";
    $("live-controls").hidden = mode !== "live";
    $("field-import-panel").hidden = !sandbox;
    document.querySelectorAll(".mode-tab").forEach(button => { const active = button.dataset.mode === mode; button.classList.toggle("active", active); button.setAttribute("aria-selected", String(active)); });
    document.querySelectorAll("[data-tool]").forEach(button => { button.disabled = !sandbox || (!!scenario.field_map && button.dataset.tool === "obstacle"); button.classList.toggle("active", button.dataset.tool === tool); });
    $("add-obstacle").disabled = !!scenario.field_map;
    setText("mode-description", sandbox ? "Edit synthetic geometry. Plan locally. Preview a simple kinematic follower." : mode === "replay" ? "Inspect recorded frames. Timeline playback never invokes the planner." : "Observe local telemetry. Editing, planning and execution controls are unavailable.");
    const b = scenario.bounds || DEFAULT.bounds, fieldMap = scenario.field_map;
    setText("field-title", `${fmt(b.max_x_m - b.min_x_m)} × ${fmt(b.max_y_m - b.min_y_m)} m ${fieldMap ? "map view" : "laboratory"}`);
    setText("source-badge", fieldMap ? "REVIEWED MAP INPUT" : "SYNTHETIC");
    setText("field-subtitle", fieldMap?.source?.label || "Generic test space · not official FRC field geometry");
    setText("plan-status", status());
    setText("solver-time", plan?.solver_duration_ns != null ? `${fmt(Number(plan.solver_duration_ns) / 1e6, 2)} ms` : "—");
    const path = points(); setText("path-length", path.length ? `${fmt(pathLength(path))} m` : "—");
    const remaining = plan ? big(plan.valid_until_us) - big(state?.estimated_robot_us ?? state?.robot_us) : 0n;
    setText("validity", plan ? remaining > 0 ? `${fmt(Number(remaining) / 1e6, 1)} s` : "EXPIRED" : "—");
    setText("request-id", plan?.request_id || "—");
    setText("context-identity", `${state?.epoch ?? "—"} / ${state?.snapshot_id ?? "—"}`);
    setText("map-version", state?.obstacle_map_version ?? "—");
    setText("backend-id", plan?.backend_id || "—");
    const identity = state?.field || {};
    setText("map-identity", `${identity.map_id || identity.mapId || fieldMap?.map?.id || DEFAULT.field.map_id} / ${identity.geometry_revision || identity.geometryRevision || "1"}`);
    const warnings = [...(state?.errors || []), ...(scenario.warnings || [])].map(value => typeof value === "string" ? value : JSON.stringify(value));
    let message = localError || warnings.join(" · ") || plan?.detail || (sandbox ? "Drag start, goal or occupancy circles to edit. Plan geometry before starting the preview." : "Waiting for a diagnostic frame.");
    if (sandbox && fieldMap && scenario.planning_supported === false) message = "This map can be inspected, but its geometry is unsupported for planning. " + message;
    if (mode === "live" && state?.connection && !state.connection.connected) message = state.connection.reason || "No live feed connected. Local synthetic mocks only; no robot connection is implemented.";
    if (busy) message = "Planning locally… Cancel revokes the proposal immediately. Solver time is computation, not robot motion time.";
    if (!freshHttp() && state && !localError) message = "HTTP state is older than one second. Retained geometry is historical; preview is stopped. Waiting for a fresh local response.";
    if (draftEditing && !localError) message = "Editing a scenario value. Valid input saves after a short pause, Tab or Enter; the previous proposal has been revoked.";
    const banner = $("status-banner"); banner.textContent = message;
    banner.className = "status-banner" + (busy ? " busy" : localError || ["INVALID_INPUT", "NO_PATH", "REJECTED"].includes(plan?.status) ? " error" : state?.connection?.stale || !connected || ["STALE_RESULT", "TIMEOUT", "CANCELLED"].includes(plan?.status) ? " warning" : "");
    $("plan-button").disabled = !connected || busy || draftEditing || scenario.planning_supported === false;
    $("plan-button").textContent = busy ? "Planning…" : "Plan path →";
    $("preview-play").disabled = !planCurrent();
    $("preview-play").textContent = preview?.playing ? "Pause preview" : preview?.finished ? "Play again" : preview ? "Resume preview" : "Play preview";
    document.querySelectorAll("[data-edit]").forEach(input => {
      if (document.activeElement === input || draftInput === input || drag) return;
      const [section, key] = input.dataset.edit.split(".");
      input.value = fmt(input.hasAttribute("data-degrees") ? deg(scenario[section]?.[key] || 0) : scenario[section]?.[key], 3);
    });
    $("field-width").disabled = !!fieldMap; $("field-height").disabled = !!fieldMap;
    if (document.activeElement !== $("field-width") && draftInput !== $("field-width")) $("field-width").value = fmt(b.max_x_m - b.min_x_m);
    if (document.activeElement !== $("field-height") && draftInput !== $("field-height")) $("field-height").value = fmt(b.max_y_m - b.min_y_m);
    renderObstacles();
    const c = state?.connection || {};
    setText("connection-status", !connected ? "Dashboard disconnected" : c.connected ? c.stale ? "Feed stale" : "Synthetic feed connected" : "Waiting for a local mock feed");
    $("connection-status").classList.toggle("offline", !connected || !c.connected || !!c.stale);
    setText("connection-age", arrival ? `${fmt(Number(big(state?.age_us)) / 1e6, 1)} s feed / ${fmt((performance.now() - arrival) / 1000, 1)} s HTTP` : "—");
    setText("source-session", state?.session_id || state?.source?.session_id || "—");
    setText("source-sequence", state?.sequence ?? "—"); setText("real-feed", state?.real_feed_implemented ? "Owner feed supplied" : "Not implemented · synthetic only");
    $("replay-timeline").max = Math.max(0, replayCount - 1); $("replay-timeline").value = replayIndex;
    setText("replay-position", replayCount ? `${replayIndex + 1} / ${replayCount}` : "0 / 0");
    $("replay-play").textContent = replayTimer ? "Pause replay" : "Play replay";
    $("replay-play").disabled = replayCount < 2; $("replay-step").disabled = !replayCount || replayIndex >= replayCount - 1;
  }

  function renderObstacles() {
    const list = $("obstacle-list");
    const ids = new Set((scenario.obstacles || []).map(o => o.id));
    for (const row of [...list.children]) if (!ids.has(row.dataset.obstacleId)) row.remove();
    if (!scenario.obstacles?.length) { if (!list.firstChild) { const note = document.createElement("p"); note.className = "empty-list"; note.textContent = "No circles. Add one on the field."; list.append(note); } return; }
    scenario.obstacles.forEach((obstacle, index) => {
      let row = [...list.children].find(child => child.dataset.obstacleId === obstacle.id);
      if (row) {
        row.classList.toggle("selected", selectedObstacle === index);
        row.querySelectorAll("[data-obstacle-key]").forEach(input => { input.disabled = !!scenario.field_map; if (document.activeElement !== input && draftInput !== input) input.value = fmt(obstacle[input.dataset.obstacleKey], 3); });
        row.querySelector(".remove-obstacle").disabled = !!scenario.field_map;
        const checkbox = row.querySelector("input[type=checkbox]"); checkbox.disabled = !!scenario.field_map; checkbox.checked = !!obstacle.dynamic;
        row.querySelector(".obstacle-kind").textContent = scenario.field_map ? "Derived from approved map · edit in image wizard" : "Dynamic envelope · conservative, not space-time";
        return;
      }
      row = document.createElement("div"); row.dataset.obstacleId = obstacle.id; row.className = "obstacle-row" + (selectedObstacle === index ? " selected" : "");
      const obstacleId = obstacle.id;
      const title = document.createElement("div"); title.className = "obstacle-row-title";
      const name = document.createElement("span"); name.textContent = obstacle.id;
      const remove = document.createElement("button"); remove.type = "button"; remove.className = "remove-obstacle"; remove.textContent = "×"; remove.setAttribute("aria-label", `Remove ${obstacle.id}`);
      remove.disabled = !!scenario.field_map;
      remove.onclick = () => { if (scenario.field_map) return; scenario.obstacles = scenario.obstacles.filter(o => o.id !== obstacleId); selectedObstacle = null; draftEditing = false; commitScenario(); };
      title.append(name, remove); row.append(title);
      const fields = document.createElement("div"); fields.className = "obstacle-row-fields";
      for (const [key, label] of [["x_m", "X m"], ["y_m", "Y m"], ["radius_m", "Radius m"], ["uncertainty_margin_m", "Margin m"]]) {
        const wrapper = document.createElement("label"); wrapper.textContent = label;
        const input = document.createElement("input"); input.type = "number"; input.dataset.obstacleKey = key; input.step = ".05"; input.value = fmt(obstacle[key], 3); input.setAttribute("aria-label", `${obstacle.id} ${label}`);
        if (key.includes("radius") || key.includes("margin")) input.min = "0";
        input.disabled = !!scenario.field_map;
        attachNumericEditor(input);
        input.onchange = () => {
          if (!input.value.trim() || !finite(Number(input.value))) { localError = "Occupancy values must be finite numbers."; renderUI(); return; }
          const next = clone(scenario), current = next.obstacles.find(o => o.id === obstacleId);
          if (!current || scenario.field_map) return;
          current[key] = Number(input.value);
          try { validateScenario(next); scenario = next; draftEditing = false; commitScenario(); } catch (error) { localError = error.message; renderUI(); }
        };
        wrapper.append(input); fields.append(wrapper);
      }
      row.append(fields);
      const toggle = document.createElement("label"); toggle.className = "toggle";
      const check = document.createElement("input"); check.type = "checkbox"; check.checked = !!obstacle.dynamic;
      check.disabled = !!scenario.field_map;
      check.onchange = () => { const current = scenario.obstacles.find(o => o.id === obstacleId); if (!current || scenario.field_map) return; current.dynamic = check.checked; draftEditing = false; commitScenario(); };
      const kind = document.createElement("span"); kind.className = "obstacle-kind"; kind.textContent = scenario.field_map ? "Derived from approved map · edit in image wizard" : "Dynamic envelope · conservative, not space-time";
      toggle.append(check, kind); row.append(toggle); list.append(row);
    });
  }

  function resized() {
    const rect = canvas.getBoundingClientRect(), ratio = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(rect.width * ratio); canvas.height = Math.round(rect.height * ratio);
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0); view.width = rect.width; view.height = rect.height;
    const b = scenario.bounds || DEFAULT.bounds;
    view.scale = Math.min((rect.width - 95) / (b.max_x_m - b.min_x_m), (rect.height - 80) / (b.max_y_m - b.min_y_m)) * zoom;
    view.left = (rect.width - (b.max_x_m - b.min_x_m) * view.scale) / 2;
    view.bottom = (rect.height - (b.max_y_m - b.min_y_m) * view.scale) / 2;
  }
  function screen(p) { const b = scenario.bounds || DEFAULT.bounds; return {x: view.left + (p.x_m - b.min_x_m) * view.scale, y: view.height - view.bottom - (p.y_m - b.min_y_m) * view.scale}; }
  function world(event) { const r = canvas.getBoundingClientRect(), b = scenario.bounds || DEFAULT.bounds; return {x_m: b.min_x_m + (event.clientX - r.left - view.left) / view.scale, y_m: b.min_y_m + (view.height - view.bottom - event.clientY + r.top) / view.scale}; }
  function circle(p, radius, fill, stroke, dash = []) { const s = screen(p); ctx.beginPath(); ctx.arc(s.x, s.y, Math.max(0, radius * view.scale), 0, Math.PI * 2); ctx.fillStyle = fill; ctx.fill(); ctx.strokeStyle = stroke; ctx.setLineDash(dash); ctx.stroke(); ctx.setLineDash([]); }
  function line(a, b, color, width = 1) { const p = screen(a), q = screen(b); ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); }
  function label(p, text, color, offset = -15) { const s = screen(p); ctx.font = "10px Menlo, monospace"; ctx.textAlign = "center"; ctx.fillStyle = color; ctx.fillText(text, s.x, s.y + offset); }
  function robot(p, previewRobot = false) {
    const s = screen(p), f = scenario.footprint || DEFAULT.footprint, radius = Math.hypot(f.length_m, f.width_m) / 2 + .02;
    if ($("show-envelopes").checked) circle(p, radius, "#247e790c", "#247e7970", [4, 4]);
    ctx.save(); ctx.translate(s.x, s.y); ctx.rotate(-p.heading_rad); ctx.fillStyle = previewRobot ? "#dd592d" : "#247e79"; ctx.strokeStyle = "#fffef8"; ctx.lineWidth = 2;
    ctx.fillRect(-f.length_m * view.scale / 2, -f.width_m * view.scale / 2, f.length_m * view.scale, f.width_m * view.scale);
    ctx.strokeRect(-f.length_m * view.scale / 2, -f.width_m * view.scale / 2, f.length_m * view.scale, f.width_m * view.scale);
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(f.length_m * view.scale / 2 + 10, 0); ctx.strokeStyle = "#173b39"; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(f.length_m * view.scale / 2 + 10, 0); ctx.lineTo(f.length_m * view.scale / 2 + 4, -4); ctx.lineTo(f.length_m * view.scale / 2 + 4, 4); ctx.closePath(); ctx.fillStyle = "#173b39"; ctx.fill(); ctx.restore();
  }

  function draw() {
    const bounds = scenario.bounds;
    if (!bounds || ![bounds.min_x_m, bounds.min_y_m, bounds.max_x_m, bounds.max_y_m].every(finite) || bounds.max_x_m <= bounds.min_x_m || bounds.max_y_m <= bounds.min_y_m || bounds.max_x_m - bounds.min_x_m > 100 || bounds.max_y_m - bounds.min_y_m > 100) return;
    resized(); ctx.clearRect(0, 0, view.width, view.height);
    const b = scenario.bounds || DEFAULT.bounds, left = screen({x_m: b.min_x_m, y_m: b.max_y_m}), right = screen({x_m: b.max_x_m, y_m: b.min_y_m});
    ctx.fillStyle = "#f9fcf5"; ctx.fillRect(left.x, left.y, right.x - left.x, right.y - left.y);
    const h = imageConfig?.calibration?.image_to_field;
    if (image && image.complete && h?.length === 9 && h[6] === 0 && h[7] === 0 && h[8] === 1) {
      ctx.save(); ctx.globalAlpha = .48; ctx.transform(view.scale * h[0], -view.scale * h[3], view.scale * h[1], -view.scale * h[4], view.left + view.scale * (h[2] - b.min_x_m), view.height - view.bottom - view.scale * (h[5] - b.min_y_m)); ctx.drawImage(image, 0, 0); ctx.restore();
    }
    ctx.lineWidth = 1; ctx.font = "9px Menlo, monospace"; ctx.textAlign = "center";
    const spacing = Math.max(.5, Math.ceil((b.max_x_m - b.min_x_m) / 20 * 2) / 2);
    for (let x = Math.ceil(b.min_x_m / spacing) * spacing; x <= b.max_x_m + .001; x += spacing) {
      line({x_m: x, y_m: b.min_y_m}, {x_m: x, y_m: b.max_y_m}, "#dce6d9"); const s = screen({x_m: x, y_m: b.min_y_m}); ctx.fillStyle = "#738979"; ctx.fillText(fmt(x, 1), s.x, s.y + 17);
    }
    for (let y = Math.ceil(b.min_y_m / spacing) * spacing; y <= b.max_y_m + .001; y += spacing) {
      line({x_m: b.min_x_m, y_m: y}, {x_m: b.max_x_m, y_m: y}, "#dce6d9"); const s = screen({x_m: b.min_x_m, y_m: y}); ctx.fillStyle = "#738979"; ctx.fillText(fmt(y, 1), s.x - 22, s.y + 3);
    }
    ctx.strokeStyle = "#8da891"; ctx.lineWidth = 1.5; ctx.strokeRect(left.x, left.y, right.x - left.x, right.y - left.y);
    ctx.fillStyle = "#607f6a"; ctx.fillText("x / meters →", (left.x + right.x) / 2, right.y + 34);
    ctx.save(); ctx.translate(left.x - 41, (left.y + right.y) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText("y / meters →", 0, 0); ctx.restore();
    const physical = scenario.field_map?.obstacles || [];
    physical.forEach(obstacle => { if (!Array.isArray(obstacle.outer)) return; ctx.beginPath(); for (const ring of [obstacle.outer, ...(obstacle.holes || [])]) { ring.forEach((p, i) => { const s = screen({x_m: p[0], y_m: p[1]}); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); }); ctx.closePath(); } ctx.fillStyle = "#7c858d30"; ctx.fill("evenodd"); ctx.strokeStyle = "#666e7a"; ctx.lineWidth = 1.5; ctx.stroke(); });
    (scenario.obstacles || []).forEach((o, i) => {
      const radius = Number(o.radius_m) + Number(o.uncertainty_margin_m || 0);
      if (!finite(radius) || radius < 0) return;
      ctx.lineWidth = selectedObstacle === i && mode === "sandbox" ? 2 : 1;
      circle(o, radius, o.dynamic ? "#426a9a20" : "#7b8b7638", o.dynamic ? "#426a9a" : "#7b8b76", o.dynamic ? [5, 3] : []);
      if ($("show-envelopes").checked) { const a = screen({x_m: o.x_m - radius, y_m: o.y_m + radius}); ctx.setLineDash([3, 4]); ctx.strokeStyle = "#8a948785"; ctx.strokeRect(a.x, a.y, 2 * radius * view.scale, 2 * radius * view.scale); ctx.setLineDash([]); }
      label(o, o.id, "#657164", 4);
    });
    const path = points(), stale = draftEditing || pendingMutations > 0 || !freshHttp() || !matchingContext() || plan?.status !== "SUCCESS" || state?.connection?.stale || (mode === "live" && !state?.connection?.connected);
    if (path.length) { ctx.setLineDash(stale ? [5, 6] : []); for (let i = 1; i < path.length; i++) line(path[i - 1], path[i], stale ? "#8b9691" : "#dd592d", 3); ctx.setLineDash([]); path.forEach(p => circle(p, .025, stale ? "#8b9691" : "#dd592d", "#fffef8")); }
    const showRobot = mode !== "live" || state?.robot || state?.connection?.connected;
    if (showRobot && scenario.start) { robot(scenario.start); label(scenario.start, mode === "sandbox" ? "START" : "ROBOT", "#247e79", -Math.hypot(scenario.footprint.length_m, scenario.footprint.width_m) / 2 * view.scale - 10); }
    if (showRobot && scenario.goal) { const g = scenario.goal; circle(g, g.position_tolerance_m || .05, "#426a9a10", "#426a9a", [3, 3]); circle(g, .09, "#426a9a", "#fffef8"); line(g, {x_m: g.x_m + .3 * Math.cos(g.heading_rad), y_m: g.y_m + .3 * Math.sin(g.heading_rad)}, "#426a9a", 2); label(g, "GOAL", "#426a9a", -17); }
    if (preview?.pose && mode === "sandbox") robot(preview.pose, true);
    ctx.lineWidth = 1;
  }

  function clearProposal() { stopPreview("Ready after a new plan"); preview = null; $("preview-progress").value = 0; plan = null; localError = ""; }
  function beginScenarioDraft(event) {
    if (mode !== "sandbox") return;
    draftInput = event.target;
    clearTimeout(draftSaveTimer);
    if (!draftEditing) { draftEditing = true; editRevision++; clearProposal(); }
    const input = draftInput;
    draftSaveTimer = setTimeout(() => { if (mode === "sandbox" && draftEditing && draftInput === input) input.onchange?.(); }, 350);
    renderUI(); draw();
  }
  function attachNumericEditor(input) {
    input.oninput = beginScenarioDraft;
    input.addEventListener("blur", () => { if (draftEditing && draftInput === input) input.onchange?.(); });
    input.addEventListener("keydown", event => { if (event.key === "Enter") { event.preventDefault(); input.blur(); if (draftEditing && draftInput === input) input.onchange?.(); } });
  }
  function finishScenarioDraft() { clearTimeout(draftSaveTimer); draftSaveTimer = null; draftInput = null; draftEditing = false; }
  function commitScenario(replan = $("auto-replan").checked) {
    if (mode !== "sandbox") return;
    try { validateScenario(scenario); } catch (error) { localError = error.message; clearProposal(); localError = error.message; renderUI(); draw(); return Promise.resolve(); }
    finishScenarioDraft(); clearProposal(); editRevision++; const revision = editRevision, snapshot = clone(scenario);
    pendingMutations++; busy = true; renderUI(); draw();
    mutationQueue = mutationQueue.catch(() => {}).then(async () => {
      try {
        const data = await api("/api/sandbox/scenario", {method: "POST", body: JSON.stringify(snapshot)});
        if (revision !== editRevision || mode !== "sandbox") return;
        accept(data, "sandbox", true);
        if (replan && scenario.planning_supported !== false) {
          const result = await api("/api/sandbox/plan", {method: "POST", body: JSON.stringify({budget_ms: 250, valid_for_ms: 30000})});
          if (revision === editRevision && mode === "sandbox") accept(result, "sandbox", true);
        }
      } catch (error) { if (revision === editRevision) { localError = error.message; connected = true; } }
      finally { pendingMutations--; if (!pendingMutations) busy = false; renderUI(); draw(); }
    });
    return mutationQueue;
  }
  async function cancelPlan() {
    if (mode !== "sandbox") return;
    finishScenarioDraft(); editRevision++; clearProposal(); busy = false; plan = {status: "CANCELLED", detail: "Synthetic planning cancelled. No proposal is current."}; renderUI(); draw();
    try { const data = await api("/api/sandbox/cancel", {method: "POST", body: "{}"}); if (mode === "sandbox") accept(data, "sandbox", true); }
    catch (error) { localError = error.message; renderUI(); }
  }
  function addCircle(position) { if (scenario.field_map) return; scenario.obstacles ||= []; scenario.obstacles.push({id: `circle-${Date.now().toString(36)}`, x_m: position.x_m, y_m: position.y_m, radius_m: .25, uncertainty_margin_m: .05, dynamic: false}); selectedObstacle = scenario.obstacles.length - 1; commitScenario(); }

  canvas.addEventListener("pointerdown", event => {
    if (mode !== "sandbox" || event.button !== 0) return;
    const p = world(event), mouse = {x: event.clientX - canvas.getBoundingClientRect().left, y: event.clientY - canvas.getBoundingClientRect().top};
    if (tool === "obstacle") { addCircle(p); return; }
    const near = pose => { const s = screen(pose); return Math.hypot(s.x - mouse.x, s.y - mouse.y) < 23; };
    let target = tool === "start" ? scenario.start : tool === "goal" ? scenario.goal : near(scenario.start) ? scenario.start : near(scenario.goal) ? scenario.goal : null;
    if (!target && !scenario.field_map) { const index = (scenario.obstacles || []).findIndex(o => Math.hypot(o.x_m - p.x_m, o.y_m - p.y_m) <= o.radius_m + o.uncertainty_margin_m + .12); if (index >= 0) { target = scenario.obstacles[index]; selectedObstacle = index; } }
    if (!target) return;
    clearProposal(); editRevision++; drag = {target, offsetX: tool === "inspect" ? target.x_m - p.x_m : 0, offsetY: tool === "inspect" ? target.y_m - p.y_m : 0};
    canvas.setPointerCapture(event.pointerId); movePointer(event); renderUI();
  });
  function movePointer(event) {
    const p = world(event);
    setText("canvas-caption", `BLUE_FIELD · x ${fmt(p.x_m)} m · y ${fmt(p.y_m)} m · CCW heading`);
    if (!drag) return;
    const b = scenario.bounds; drag.target.x_m = Math.max(b.min_x_m, Math.min(b.max_x_m, p.x_m + drag.offsetX)); drag.target.y_m = Math.max(b.min_y_m, Math.min(b.max_y_m, p.y_m + drag.offsetY)); draw();
  }
  canvas.addEventListener("pointermove", movePointer);
  function finishDrag() { if (!drag) return; drag = null; commitScenario(); }
  canvas.addEventListener("pointerup", finishDrag); canvas.addEventListener("pointercancel", finishDrag);
  canvas.addEventListener("wheel", event => { event.preventDefault(); zoom = Math.max(.65, Math.min(3, zoom * (event.deltaY > 0 ? .92 : 1.08))); draw(); }, {passive: false});
  $("fit-view").onclick = () => { zoom = 1; draw(); };
  $("show-envelopes").onchange = draw;
  document.querySelectorAll("[data-tool]").forEach(button => button.onclick = () => { tool = button.dataset.tool; renderUI(); });
  document.querySelectorAll("[data-edit]").forEach(input => input.onchange = () => {
    const value = Number(input.value); if (!input.value.trim() || !finite(value)) { localError = "All scenario coordinates and limits must be finite."; renderUI(); return; }
    const [section, key] = input.dataset.edit.split("."); const next = clone(scenario); next[section][key] = input.hasAttribute("data-degrees") ? rad(value) : value;
    try { validateScenario(next); scenario = next; commitScenario(); } catch (error) { localError = error.message; renderUI(); }
  });
  document.querySelectorAll("[data-edit], [data-bound]").forEach(attachNumericEditor);
  document.querySelectorAll("[data-bound]").forEach(input => input.onchange = () => { if (scenario.field_map) return; const value = Number(input.value); if (!finite(value) || value <= 0 || value > 100) { localError = "Field dimensions must be finite and between 0 and 100 m."; renderUI(); return; } const b = scenario.bounds; if (input.dataset.bound === "width") b.max_x_m = b.min_x_m + value; else b.max_y_m = b.min_y_m + value; zoom = 1; commitScenario(); });
  $("plan-button").onclick = () => commitScenario(true); $("cancel-button").onclick = cancelPlan;
  $("add-obstacle").onclick = () => { const b = scenario.bounds; addCircle({x_m: (b.min_x_m + b.max_x_m) / 2, y_m: (b.min_y_m + b.max_y_m) / 2}); };
  $("reset-scenario").onclick = () => { scenario = clone(DEFAULT); image = null; imageConfig = null; selectedObstacle = null; zoom = 1; commitScenario(false); };

  function download(filename, value) { const url = URL.createObjectURL(new Blob([JSON.stringify(value, null, 2)], {type: "application/json"})); const a = document.createElement("a"); a.href = url; a.download = filename; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000); }
  $("export-scenario").onclick = () => download("field-lab-scenario.json", scenario);
  $("import-scenario").onclick = () => $("scenario-file").click();
  async function readJson(file) { if (!file || file.size > 262144) throw new Error("Choose a JSON file no larger than 256 KiB."); const data = JSON.parse(await file.text()); validateJsonNumbers(data); validateLongNumbers(data); return data; }
  $("scenario-file").onchange = async event => {
    const revision = ++scenarioImportRevision, context = modeRevision, edits = editRevision;
    const current = () => mode === "sandbox" && context === modeRevision && edits === editRevision && revision === scenarioImportRevision;
    try {
      if (!current()) return;
      const data = await readJson(event.target.files[0]);
      if (!current()) return;
      const next = data.scenario || data;
      if (!next.bounds || !next.start || !next.goal || !Array.isArray(next.obstacles)) throw new Error("Scenario needs bounds, start, goal and obstacles.");
      const candidate = {...clone(DEFAULT), ...next}; validateScenario(candidate);
      scenario = candidate; image = null; imageConfig = null; zoom = 1; commitScenario(false);
    } catch (error) {
      if (current()) { localError = `Import rejected: ${error.message}`; renderUI(); }
    } finally { if (revision === scenarioImportRevision) event.target.value = ""; }
  };

  function pauseReplay() { if (replayTimer) clearInterval(replayTimer); replayTimer = null; renderUI(); }
  async function replaySample(index) { if (mode !== "replay") return; const revision = ++replayRevision; replayRequestPending = true; try { const data = await api(`/api/replay/sample?index=${Math.max(0, Math.min(replayCount - 1, index))}`); if (mode === "replay" && revision === replayRevision) { localError = ""; accept(data, "replay", true); } } catch (error) { if (revision === replayRevision) { localError = error.message; pauseReplay(); } } finally { if (revision === replayRevision) replayRequestPending = false; } }
  $("replay-play").onclick = () => { if (replayTimer) { pauseReplay(); return; } if (replayIndex >= replayCount - 1) replaySample(0); replayTimer = setInterval(() => { if (replayRequestPending) return; if (replayIndex >= replayCount - 1) pauseReplay(); else replaySample(replayIndex + 1); }, Number($("replay-delay").value)); renderUI(); };
  $("replay-step").onclick = () => { pauseReplay(); replaySample(replayIndex + 1); };
  $("replay-reset").onclick = () => { pauseReplay(); replaySample(0); };
  $("replay-timeline").oninput = event => { const index = Number(event.target.value); pauseReplay(); replaySample(index); };
  $("replay-delay").onchange = pauseReplay;
  $("import-replay").onclick = () => $("replay-file").click();
  $("replay-file").onchange = async event => { const revision = ++replayRevision, context = modeRevision; replayRequestPending = true; try { pauseReplay(); const data = await readJson(event.target.files[0]); if (mode !== "replay" || context !== modeRevision || revision !== replayRevision) return; const result = await api("/api/replay/load", {method: "POST", body: JSON.stringify(data)}); if (mode === "replay" && context === modeRevision && revision === replayRevision) { localError = ""; accept(result, "replay", true); } } catch (error) { if (context === modeRevision && revision === replayRevision) { localError = `Replay rejected: ${error.message}`; renderUI(); } } finally { if (revision === replayRevision) replayRequestPending = false; event.target.value = ""; } };
  $("refresh-live").onclick = () => { localError = ""; poll(); };

  function segmentDistance(a, b, p) { const dx = b.x_m - a.x_m, dy = b.y_m - a.y_m, length = dx * dx + dy * dy; const t = length ? Math.max(0, Math.min(1, ((p.x_m - a.x_m) * dx + (p.y_m - a.y_m) * dy) / length)) : 0; return Math.hypot(p.x_m - a.x_m - t * dx, p.y_m - a.y_m - t * dy); }
  function intersectsBox(a, b, box) { let lo = 0, hi = 1; const dx = b.x_m - a.x_m, dy = b.y_m - a.y_m; for (const [p, q] of [[-dx, a.x_m - box.minX], [dx, box.maxX - a.x_m], [-dy, a.y_m - box.minY], [dy, box.maxY - a.y_m]]) { if (Math.abs(p) < 1e-12) { if (q < 0) return false; } else { const r = q / p; if (p < 0) lo = Math.max(lo, r); else hi = Math.min(hi, r); if (lo > hi) return false; } } return true; }
  function safeSweep(a, b) {
    const f = scenario.footprint, radius = Math.hypot(f.length_m, f.width_m) / 2 + .02, bounds = scenario.bounds;
    for (const p of [a, b]) if (p.x_m - radius <= bounds.min_x_m || p.x_m + radius >= bounds.max_x_m || p.y_m - radius <= bounds.min_y_m || p.y_m + radius >= bounds.max_y_m) return false;
    const circles = [...(scenario.obstacles || [])];
    for (const polygon of scenario.field_map?.obstacles || []) { const outer = polygon.outer || []; if (!outer.length) continue; const xs = outer.map(p => p[0]), ys = outer.map(p => p[1]); const x = (Math.min(...xs) + Math.max(...xs)) / 2, y = (Math.min(...ys) + Math.max(...ys)) / 2; circles.push({x_m: x, y_m: y, radius_m: Math.max(...outer.map(p => Math.hypot(p[0] - x, p[1] - y))), uncertainty_margin_m: 0}); }
    for (const o of circles) {
      const r = o.radius_m + (o.uncertainty_margin_m || 0), box = {minX: o.x_m - r, maxX: o.x_m + r, minY: o.y_m - r, maxY: o.y_m + r};
      if (intersectsBox(a, b, box)) return false;
      const corners = [{x_m: box.minX, y_m: box.minY}, {x_m: box.maxX, y_m: box.minY}, {x_m: box.maxX, y_m: box.maxY}, {x_m: box.minX, y_m: box.maxY}];
      const pointBoxDistance = p => Math.hypot(Math.max(box.minX - p.x_m, 0, p.x_m - box.maxX), Math.max(box.minY - p.y_m, 0, p.y_m - box.maxY));
      if (Math.min(pointBoxDistance(a), pointBoxDistance(b), ...corners.map(p => segmentDistance(a, b, p))) <= radius) return false;
    }
    return true;
  }
  function stopPreview(message) { if (previewFrame) cancelAnimationFrame(previewFrame); previewFrame = null; if (preview) preview.playing = false; previewLastTime = 0; setText("preview-state", message || "Preview paused"); }
  function preparePreview() {
    const path = clone(points()), stages = [];
    const maximumSpeed = Math.min(Number($("preview-speed").value), scenario.constraints.max_speed_mps), acceleration = Math.min(Number($("preview-acceleration").value), scenario.constraints.max_acceleration_mps2);
    if (!finite(maximumSpeed) || !finite(acceleration) || maximumSpeed <= 0 || acceleration <= 0) { localError = "Set positive finite preview speed and acceleration."; renderUI(); return false; }
    for (let i = 1; i < path.length; i++) {
      const a = path[i - 1], b = path[i], distance = Math.hypot(b.x_m - a.x_m, b.y_m - a.y_m);
      if (distance > 1e-8) {
        const accelerationTime = Math.min(maximumSpeed / acceleration, Math.sqrt(distance / acceleration));
        const peakSpeed = acceleration * accelerationTime, cruiseTime = Math.max(0, (distance - peakSpeed * accelerationTime) / peakSpeed);
        stages.push({kind: "move", a: {...a}, b: {...b, heading_rad: a.heading_rad}, distance,
          acceleration, accelerationTime, peakSpeed, cruiseTime, duration: 2 * accelerationTime + cruiseTime, elapsed: 0});
      }
      const angle = wrap(b.heading_rad - a.heading_rad);
      if (Math.abs(angle) > 1e-8) stages.push({kind: "rotate", a: {...b, heading_rad: a.heading_rad}, b: {...b}, distance: Math.abs(angle), sign: Math.sign(angle)});
    }
    preview = {requestId: plan.request_id, taskId: plan.task_id, generation: String(plan.generation), issuedUs: big(plan.issued_us), expiresUs: big(plan.valid_until_us), stages, index: 0, position: 0, speed: 0, elapsed: 0, pose: {...path[0]}, playing: false, finished: false}; $("preview-progress").value = 0;
    return true;
  }
  function tickPreview(time) {
    if (!preview?.playing || mode !== "sandbox" || !planCurrent() || preview.requestId !== plan.request_id || preview.taskId !== plan.task_id || preview.generation !== String(plan.generation)) { stopPreview("Stopped: proposal no longer current"); renderUI(); return; }
    const dt = previewLastTime ? Math.min(.04, (time - previewLastTime) / 1000) : 0; previewLastTime = time; preview.elapsed += dt;
    if (preview.issuedUs + BigInt(Math.round(preview.elapsed * 1e6)) >= preview.expiresUs) { stopPreview("Stopped: virtual request validity expired"); renderUI(); return; }
    const stage = preview.stages[preview.index];
    if (!stage) { preview.finished = true; stopPreview("Preview arrived · simulation only"); $("preview-progress").value = 1; renderUI(); draw(); return; }
    const previous = {...preview.pose};
    if (stage.kind === "move") {
      stage.elapsed = Math.min(stage.duration, stage.elapsed + dt);
      if (stage.elapsed <= stage.accelerationTime) preview.position = .5 * stage.acceleration * stage.elapsed ** 2;
      else if (stage.elapsed <= stage.accelerationTime + stage.cruiseTime) preview.position = .5 * stage.acceleration * stage.accelerationTime ** 2 + stage.peakSpeed * (stage.elapsed - stage.accelerationTime);
      else preview.position = stage.distance - .5 * stage.acceleration * (stage.duration - stage.elapsed) ** 2;
      if (stage.elapsed === stage.duration) preview.position = stage.distance;
      const t = preview.position / stage.distance; preview.pose = {x_m: stage.a.x_m + (stage.b.x_m - stage.a.x_m) * t, y_m: stage.a.y_m + (stage.b.y_m - stage.a.y_m) * t, heading_rad: stage.a.heading_rad};
    } else {
      const omega = Math.min(1.2, scenario.constraints.max_angular_speed_radps); preview.position = Math.min(stage.distance, preview.position + omega * dt); preview.pose = {...stage.a, heading_rad: wrap(stage.a.heading_rad + stage.sign * preview.position)};
    }
    if (!safeSweep(previous, preview.pose)) { preview.pose = previous; stopPreview("Stopped: conservative sweep collision"); renderUI(); draw(); return; }
    if (preview.position >= stage.distance - 1e-8) { preview.pose = {...stage.b}; preview.index++; preview.position = 0; preview.speed = 0; }
    $("preview-progress").value = preview.stages.length ? preview.index / preview.stages.length : 1;
    setText("preview-state", `${stage.kind === "move" ? "Translation" : "Stationary rotation"} · virtual ${fmt(preview.elapsed, 1)} s`); draw(); previewFrame = requestAnimationFrame(tickPreview);
  }
  $("preview-play").onclick = () => { if (preview?.playing) { stopPreview("Preview paused · virtual clock paused"); renderUI(); return; } if (!planCurrent()) return; if ((!preview || preview.finished || preview.requestId !== plan.request_id) && !preparePreview()) return; preview.playing = true; previewLastTime = 0; renderUI(); previewFrame = requestAnimationFrame(tickPreview); };
  $("preview-reset").onclick = () => { stopPreview(); preview = null; $("preview-progress").value = 0; setText("preview-state", "Ready after a successful plan"); renderUI(); draw(); };
  for (const id of ["preview-speed", "preview-acceleration"]) $(id).onchange = () => { stopPreview("Preview limits changed · restart from the path start"); preview = null; $("preview-progress").value = 0; renderUI(); draw(); };

  function switchMode(next) { if (next === mode) return; stopPreview(); preview = null; pauseReplay(); finishScenarioDraft(); mode = next; modeRevision++; editRevision++; replayRevision++; replayRequestPending = false; localError = ""; plan = null; state = null; scenario = clone(DEFAULT); zoom = 1; image = null; imageConfig = null; drag = null; renderUI(); draw(); poll(); }
  document.querySelectorAll("[data-mode]").forEach(button => button.onclick = () => switchMode(button.dataset.mode));
  window.addEventListener("resize", draw);
  $("host-label").textContent = `LOCAL / ${location.port || "8086"}`;
  renderUI(); draw(); poll(); setInterval(poll, 200);
  setInterval(() => {
    if (arrival && performance.now() - arrival >= 1000 && connected) {
      connected = false; stopPreview("Stopped: HTTP state is older than one second"); renderUI(); draw();
    }
    if (mode === "live" && arrival) setText("connection-age", `${fmt(Number(big(state?.age_us)) / 1e6, 1)} s feed / ${fmt((performance.now() - arrival) / 1000, 1)} s HTTP`);
  }, 100);
  import("/field-import.js").then(() => {
    if (!window.FieldImport?.init) return;
    window.FieldImport.init($("field-import-panel"), {
      getMode: () => mode,
      onSave: (config, imageUrl) => {
        if (mode !== "sandbox") return;
        const candidate = {...clone(scenario), field_map: clone(config), bounds: {min_x_m: 0, min_y_m: 0, max_x_m: config.map.width_m, max_y_m: config.map.height_m}};
        validateScenario(candidate); scenario = candidate;
        imageConfig = clone(config); image = imageUrl ? new Image() : null; if (image) { image.onload = draw; image.src = imageUrl; }
        zoom = 1; const revision = editRevision + 1;
        return commitScenario(false).then(() => { if (localError || revision !== editRevision || mode !== "sandbox") throw new Error(localError || "Scenario context changed before save completed."); });
      }
    });
  }).catch(() => { $("field-import-panel").querySelector(".help").textContent = "Field image import is unavailable in this build. Metric scenario editing remains available."; });
})();
