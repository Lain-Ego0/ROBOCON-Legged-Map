(() => {
  "use strict";
  const $ = (s) => document.querySelector(s);
  const $$ = (s) => Array.from(document.querySelectorAll(s));
  const fmtTime = (t) => {
    if (!isFinite(t)) t = 0;
    const m = Math.floor(t / 60), s = t % 60;
    return `${String(m).padStart(2, "0")}:${s.toFixed(1).padStart(4, "0")}`;
  };
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const statusZh = { storage: "在库", carried: "搬运中", home: "已归位", stack: "叠放", loose: "散落" };

  let config = null;
  let state = null;
  let mode = "logistics";
  let started = false;
  let paused = false;
  let cameraMode = "chase";
  let mapView = "task";
  let mapImg = null;
  let lastPoll = performance.now();
  let lastSimTime = 0;
  let simRate = 1.0;
  let errorHistory = [];
  let inlierHistory = [];
  let dragging = null;
  let toastTimer = null;

  const modeButtons = [];
  const treeNodes = new Map();
  const cargoCards = new Map();

  function toast(text, tone = "ok") {
    const el = $("#toast");
    el.textContent = text;
    el.style.borderColor = tone === "warn" ? "rgba(244,179,90,.6)" : "rgba(89,239,199,.45)";
    el.style.color = tone === "warn" ? "#f4b35a" : "#59efc7";
    el.classList.add("show");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => el.classList.remove("show"), 2200);
  }

  async function post(url, body) {
    try {
      const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
      if (!r.ok) throw new Error(await r.text());
      return await r.json().catch(() => ({}));
    } catch (e) {
      toast("请求失败: " + e.message, "warn");
      return null;
    }
  }

  // ------------------------------------------------------------- bootstrap
  async function boot() {
    try {
      config = await (await fetch("/api/config", { cache: "no-store" })).json();
    } catch {
      setTimeout(boot, 800);
      return;
    }
    buildModeButtons();
    buildLegend();
    buildTree();
    buildCargoGrid();
    mapImg = new Image();
    mapImg.src = "/map.png";
    resizeCanvas();
    window.addEventListener("resize", resizeCanvas);
    bindControls();
    poll();
    setInterval(poll, 120);
  }

  function buildModeButtons() {
    const seg = $("#modeSeg");
    seg.innerHTML = "";
    for (const [id, m] of Object.entries(config.modes)) {
      const b = document.createElement("button");
      b.textContent = m.short;
      b.title = m.label;
      b.onclick = () => setMode(id);
      b.dataset.mode = id;
      seg.appendChild(b);
      modeButtons.push(b);
    }
    setModeUI("logistics");
  }

  function setModeUI(id) {
    mode = id;
    for (const b of modeButtons) b.classList.toggle("on", b.dataset.mode === id);
    $("#modeDesc").textContent = config.modes[id]?.description || "";
    $("#manualPad").classList.toggle("show", id === "manual");
    if (id === "manual") setCamera("wide");
  }

  async function setMode(id) {
    if (mode === id) return;
    setModeUI(id);
    const ok = await post("/api/mode", { mode: id });
    if (ok) {
      started = false;
      paused = false;
      $("#btnStart").textContent = "启动自主任务";
      toast("切换模式 · " + config.modes[id].label);
    }
  }

  function buildLegend() {
    const box = $("#legend");
    box.innerHTML = "";
    for (const c of config.categories) {
      const d = document.createElement("div");
      d.className = "item";
      d.innerHTML = `<span class="dot" style="background:${c.color};box-shadow:0 0 8px ${c.color}66"></span><span>${c.zh}</span>`;
      box.appendChild(d);
    }
  }

  function buildTree() {
    const host = $("#tree");
    host.innerHTML = "";
    const walk = (node, depth, parent) => {
      const wrap = document.createElement("div");
      const row = document.createElement("div");
      row.className = "tree-node" + (node.leaf ? " leaf" : "");
      row.dataset.id = node.id;
      row.innerHTML = `<span class="junc"></span><span class="lbl">${node.label}</span>` +
        (node.hint ? `<span class="hint">${node.hint}</span>` : "");
      wrap.appendChild(row);
      treeNodes.set(node.id, row);
      if (node.children && node.children.length) {
        const kids = document.createElement("div");
        kids.className = "tree-children";
        for (const child of node.children) walk(child, depth + 1, kids);
        wrap.appendChild(kids);
      }
      parent.appendChild(wrap);
    };
    walk(config.tree, 0, host);
  }

  function updateTree(path, nodeStates) {
    for (const n of treeNodes.values()) {
      n.classList.remove("active", "done");
    }
    const active = new Set(path || []);
    for (const id of active) {
      const n = treeNodes.get(id);
      if (n) n.classList.add("active");
    }
    if (nodeStates) {
      for (const [id, val] of Object.entries(nodeStates)) {
        const n = treeNodes.get(id);
        if (n && val === "active") n.classList.add("active");
      }
    }
  }

  function buildCargoGrid() {
    const grid = $("#cargoGrid");
    grid.innerHTML = "";
    for (let i = 0; i < 8; i++) {
      const el = document.createElement("div");
      el.className = "cargo";
      el.innerHTML = `<span class="cat"></span><div class="meta"><b></b><span></span></div><em></em>`;
      grid.appendChild(el);
      cargoCards.set(i, el);
    }
  }

  function bindControls() {
    $("#btnStart").onclick = async () => {
      const ok = await post("/api/mission", { action: "start" });
      if (ok) {
        started = true;
        paused = false;
        $("#btnPause").textContent = "暂停";
        $("#btnStart").textContent = "任务进行中";
        toast("自主任务已启动");
      }
    };
    $("#btnMission").onclick = async () => {
      const ok = await post("/api/mission", { action: "start" });
      if (ok) {
        started = true;
        $("#btnStart").textContent = "任务进行中";
        toast("自主任务已启动");
      }
    };
    $("#btnPause").onclick = async () => {
      paused = !paused;
      await post("/api/mission", { action: paused ? "pause" : "resume" });
      $("#btnPause").textContent = paused ? "继续" : "暂停";
      toast(paused ? "仿真已暂停" : "仿真继续");
    };
    $("#btnReset").onclick = async () => {
      await post("/api/mission", { action: "reset" });
      started = false;
      paused = false;
      $("#btnPause").textContent = "暂停";
      $("#btnStart").textContent = "启动自主任务";
      toast("场景已重置");
    };
    $("#btnReloc").onclick = async () => {
      await post("/api/mission", { action: "relocalize" });
      toast("已触发 GLIM 全局重定位");
    };
    $$(".cam").forEach((b) => {
      b.onclick = () => setCamera(b.dataset.cam);
    });
    $$(".map-tabs button").forEach((b) => {
      b.onclick = () => {
        mapView = b.dataset.view;
        $$(".map-tabs button").forEach((x) => x.classList.toggle("on", x === b));
        $("#mapLabel").textContent = mapView === "task" ? "TASK FIELD · 任务赛区" : "FULL COURSE · 全场";
      };
    });
    // Manual key handling.
    const valid = new Set(["w", "a", "s", "d", "q", "e", "space", "shift"]);
    const pressed = new Set();
    const syncKeys = () => post("/api/keys", { keys: [...pressed] });
    const norm = (e) => ({ KeyW: "w", KeyS: "s", KeyA: "a", KeyD: "d", KeyQ: "q", KeyE: "e", Space: "space", ShiftLeft: "shift", ShiftRight: "shift" })[e.code] || e.key.toLowerCase();
    document.addEventListener("keydown", (e) => {
      if (["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName)) return;
      const k = norm(e);
      if (!valid.has(k) || mode !== "manual") return;
      e.preventDefault();
      if (!pressed.has(k)) { pressed.add(k); syncKeys(); }
    });
    document.addEventListener("keyup", (e) => {
      const k = norm(e);
      if (pressed.delete(k)) { e.preventDefault(); syncKeys(); }
    });
    window.addEventListener("blur", () => { if (pressed.size) { pressed.clear(); syncKeys(); } });
    $$(".manual-pad button").forEach((b) => {
      const k = b.dataset.key;
      const down = (e) => { e.preventDefault(); b.classList.add("on"); pressed.add(k); syncKeys(); };
      const up = () => { b.classList.remove("on"); pressed.delete(k); syncKeys(); };
      b.addEventListener("pointerdown", down);
      ["pointerup", "pointercancel", "pointerleave"].forEach((ev) => b.addEventListener(ev, up));
    });
    // Camera orbit drag.
    const stage = $("#stage");
    stage.addEventListener("pointerdown", (e) => {
      if (e.target.closest("button")) return;
      dragging = { x: e.clientX, y: e.clientY, moved: 0 };
      stage.setPointerCapture(e.pointerId);
    });
    stage.addEventListener("pointermove", (e) => {
      if (!dragging) return;
      const dx = e.clientX - dragging.x, dy = e.clientY - dragging.y;
      dragging.x = e.clientX; dragging.y = e.clientY;
      dragging.moved += Math.abs(dx) + Math.abs(dy);
      if (dragging.moved > 4) {
        if (cameraMode !== "orbit") setCamera("orbit", true);
        post("/api/camera", { mode: "orbit_delta", dazimuth: -dx * 0.28, delevation: dy * 0.22 });
      }
    });
    const stopDrag = () => { dragging = null; };
    stage.addEventListener("pointerup", stopDrag);
    stage.addEventListener("pointercancel", stopDrag);
    stage.addEventListener("wheel", (e) => {
      if (e.target.closest("button")) return;
      e.preventDefault();
      if (cameraMode !== "orbit") setCamera("orbit", true);
      post("/api/camera", { mode: "orbit_delta", ddistance: e.deltaY * 0.006 });
    }, { passive: false });
  }

  function setCamera(id, silent = false) {
    cameraMode = id;
    $$(".cam").forEach((b) => b.classList.toggle("active", b.dataset.cam === id));
    if (!silent) post("/api/camera", { mode: id });
  }

  function resizeCanvas() {
    const c = $("#minimap");
    const r = c.getBoundingClientRect();
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    c.width = Math.max(300, Math.round(r.width * dpr));
    c.height = Math.max(150, Math.round(r.height * dpr));
    drawMap();
  }

  // ------------------------------------------------------------------ poll
  async function poll() {
    let s;
    try {
      s = await (await fetch("/api/state", { cache: "no-store" })).json();
    } catch {
      $("#streamText").textContent = "RECONNECTING";
      $("#streamDot").classList.remove("on");
      return;
    }
    const now = performance.now();
    if (s.sim_time > lastSimTime) {
      const dt = (now - lastPoll) / 1000;
      simRate = clamp((s.sim_time - lastSimTime) / Math.max(dt, 1e-3), 0.1, 4);
    }
    lastSimTime = s.sim_time;
    lastPoll = now;
    state = s;
    if (s.mode && s.mode !== mode) {
      mode = s.mode;
      setModeUI(mode);
    }
    started = !!s.started;
    paused = !!s.paused;
    render();
  }

  function render() {
    if (!state || !config) return;
    const d = state.decision || {};
    const sc = state.score || {};
    const robot = state.robot || {};
    const loc = state.loc || {};
    // Header
    $("#timer").textContent = fmtTime(d.time || 0);
    $("#timerSub").textContent = d.finished ? "已结束" : (state.started ? (state.paused ? "已暂停" : "计时中") : "待机");
    if (d.finished) { $("#btnStart").textContent = "任务已结束"; $("#btnMission").textContent = "任务已结束"; }
    else if (state.started) { $("#btnStart").textContent = state.paused ? "已暂停" : "任务进行中"; $("#btnMission").textContent = "任务进行中"; }
    else { $("#btnStart").textContent = "启动自主任务"; $("#btnMission").textContent = "开始任务"; }
    $("#btnPause").textContent = state.paused ? "继续" : "暂停";
    $("#score").textContent = `${sc.total ?? 0}`;
    $("#carry").textContent = `${state.stats?.carried ?? 0} / 2`;
    $("#carrySub").textContent = (state.stats?.carried ?? 0) === 0 ? "空载" : "侧面夹持";
    const locEl = $("#locState");
    const status = loc.status || "TRACKING";
    locEl.textContent = status;
    locEl.className = status === "TRACKING" ? "mint" : status === "DEGRADED" ? "amber" : "red";
    $("#locErr").textContent = `误差 ${(loc.error ?? 0).toFixed(3)} m`;
    $("#hudLoc").textContent = status;
    $("#hudLocHz").textContent = `${(loc.latency_ms ?? 0).toFixed(1)} ms`;
    const dot = $("#locDot");
    dot.className = "dot" + (status === "TRACKING" ? "" : status === "DEGRADED" ? " warn" : " bad");
    $("#hudErr").textContent = `${(loc.error ?? 0).toFixed(3)} m`;
    $("#hudInlier").textContent = `${loc.inliers ?? 0}`;
    $("#hudRmse").textContent = `${(loc.rmse ?? 0).toFixed(3)} m`;
    $("#inlierBar").style.width = `${clamp((loc.inliers ?? 0) / 360 * 100, 0, 100)}%`;
    // Phase
    const task = d.task;
    $("#phase").textContent = task?.note || d.status || "待机";
    $("#phaseSub").textContent = d.finished ? "任务结束" : (d.reason || "等待自主任务启动");
    // Telemetry
    const cmd = robot.command || [0, 0, 0];
    const vel = robot.velocity || [0, 0];
    setBar("#barVx", cmd[0], 1.15, "#valVx");
    setBar("#barVy", cmd[1], 0.55, "#valVy");
    setBar("#barWz", cmd[2], 1.2, "#valWz");
    $("#valSpeed").textContent = Math.hypot(vel[0], vel[1]).toFixed(2);
    $("#valZ").textContent = (robot.z ?? 0).toFixed(2);
    $("#valDist").textContent = (state.stats?.distance ?? 0).toFixed(1);
    // Decision tree
    updateTree(d.decision_path, d.node_states);
    $("#decisionReason").textContent = d.reason || "等待决策树";
    if (task) {
      const labels = { pick: "取箱任务", deliver: "投放任务", return: "返回任务", tour: "航线任务" };
      $("#taskName").textContent = labels[task.kind] || task.kind || "任务";
      $("#taskState").textContent = (task.state || "").toUpperCase();
      $("#taskNote").textContent = task.note || "—";
      const prog = task.timeout > 0 ? clamp((task.elapsed || 0) / task.timeout * 100, 0, 100) : 0;
      $("#taskBar").style.width = `${prog}%`;
    } else {
      $("#taskName").textContent = "—"; $("#taskState").textContent = "IDLE";
      $("#taskNote").textContent = "决策树空闲"; $("#taskBar").style.width = "0%";
    }
    // Cargo
    for (const c of state.cargo || []) {
      const el = cargoCards.get(c.id);
      if (!el) continue;
      const cat = config.categories.find((x) => x.id === c.category);
      el.querySelector(".cat").style.background = cat?.color || "#888";
      el.querySelector("b").textContent = cat?.zh || c.category;
      el.querySelector(".meta span").textContent = `#${c.id} · ${statusZh[c.status] || c.status}`;
      const item = (state.score?.items || []).find((x) => x.id === c.id);
      el.querySelector("em").textContent = item ? `+${item.points}` : "";
      el.className = "cargo" + (c.status === "carried" ? " carried" : c.status === "home" || c.status === "stack" ? " home" : c.status === "loose" ? " loose" : "");
    }
    // KV
    $("#kvEst").textContent = fmtPose(robot.est);
    $("#kvGt").textContent = fmtPose(robot.gt);
    $("#kvYawErr").textContent = `${((loc.yaw_error ?? 0) * 180 / Math.PI).toFixed(1)}°`;
    $("#kvCorr").textContent = `${(loc.correction ?? 0).toFixed(3)} m`;
    $("#kvLatency").textContent = `${(loc.latency_ms ?? 0).toFixed(1)} ms`;
    // Logs
    renderLogs(d.logs || []);
    // stream status
    $("#streamDot").classList.toggle("on", !state.error);
    $("#streamText").textContent = state.error ? "SIM ERROR" : "LIVE MUJOCO";
    $("#simFps").textContent = `SIM ${(simRate * 50).toFixed(0)} Hz · ${state.paused ? "PAUSED" : "RUNNING"}`;
    $("#policyInfo").textContent = config.policy || "S-DOG POLICY";
    if (state.error) toast("仿真错误: " + state.error, "warn");
    if (state.render_error) toast("渲染错误: " + state.render_error, "warn");
    // history for scan chart
    errorHistory.push(loc.error ?? 0); inlierHistory.push(loc.inliers ?? 0);
    if (errorHistory.length > 140) { errorHistory.shift(); inlierHistory.shift(); }
    drawScanChart();
    drawMap();
  }

  function fmtPose(p) {
    if (!p) return "--";
    return `${p[0].toFixed(2)}, ${p[1].toFixed(2)}, ${(p[2] * 180 / Math.PI).toFixed(0)}°`;
  }

  function setBar(sel, value, limit, valSel) {
    const bar = $(sel);
    if (!bar) return;
    const pct = clamp(Math.abs(value) / limit * 50, 0, 50);
    bar.style.width = `${pct * 2}%`;
    bar.style.marginLeft = value < 0 ? `${50 - pct * 2}%` : "50%";
    bar.style.background = value < 0 ? "linear-gradient(90deg,#f4b35a,#ff8a5c)" : "linear-gradient(90deg,#58b6ff,#59efc7)";
    if (valSel) $(valSel).textContent = value.toFixed(2);
  }

  function renderLogs(logs) {
    const box = $("#logs");
    const rows = logs.slice(-9).reverse();
    box.innerHTML = rows.map((l) => `<div class="log ${l.kind}"><span class="t">${fmtTime(l.t).slice(0,5)}</span><span class="k"></span><span>${escapeHtml(l.text)}</span></div>`).join("");
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  }

  // ---------------------------------------------------------------- minimap
  const WORLD = [-7.35, 14.35, -3.15, 3.15];
  function viewBounds() {
    if (mapView === "task") return [7.05, 14.32, -2.28, 2.28];
    return WORLD;
  }

  function drawMap() {
    const c = $("#minimap");
    if (!c) return;
    const ctx = c.getContext("2d");
    const w = c.width, h = c.height;
    ctx.clearRect(0, 0, w, h);
    const [x0, x1, y0, y1] = viewBounds();
    const pad = 8;
    const sx = (w - pad * 2) / (x1 - x0);
    const sy = (h - pad * 2) / (y1 - y0);
    const s = Math.min(sx, sy);
    const ox = pad + ((w - pad * 2) - (x1 - x0) * s) / 2;
    const oy = pad + ((h - pad * 2) - (y1 - y0) * s) / 2;
    const X = (x) => ox + (x - x0) * s;
    const Y = (y) => oy + (y1 - y) * s;
    // background image crop
    if (mapImg && mapImg.complete) {
      const iw = mapImg.naturalWidth, ih = mapImg.naturalHeight;
      const [wx0, wx1, wy0, wy1] = WORLD;
      const srcX = (x0 - wx0) / (wx1 - wx0) * iw;
      const srcY = (wy1 - y1) / (wy1 - wy0) * ih;
      const srcW = (x1 - x0) / (wx1 - wx0) * iw;
      const srcH = (y1 - y0) / (wy1 - wy0) * ih;
      ctx.globalAlpha = 0.92;
      ctx.drawImage(mapImg, srcX, srcY, srcW, srcH, ox, oy, (x1 - x0) * s, (y1 - y0) * s);
      ctx.globalAlpha = 1;
    } else {
      ctx.fillStyle = "#050a10"; ctx.fillRect(0, 0, w, h);
    }
    // Home zones pulse
    const t = performance.now() / 900;
    if (config) {
      for (const [cat, p] of Object.entries(config.home)) {
        const color = config.categories.find((x) => x.id === cat)?.color || "#fff";
        ctx.strokeStyle = color; ctx.lineWidth = Math.max(1.5, s * 0.018);
        ctx.globalAlpha = 0.55 + 0.35 * Math.sin(t + p[1]);
        ctx.strokeRect(X(p[0] - 0.15), Y(p[1] + 0.15), 0.3 * s, 0.3 * s);
        ctx.globalAlpha = 1;
      }
    }
    // Path
    const path = state?.path || [];
    if (path.length > 1) {
      ctx.beginPath();
      path.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])));
      ctx.strokeStyle = "rgba(88,182,255,.8)"; ctx.lineWidth = Math.max(1.5, s * 0.012);
      ctx.setLineDash([s * 0.12, s * 0.08]); ctx.stroke(); ctx.setLineDash([]);
    }
    if (state?.target) {
      const [tx, ty] = state.target;
      ctx.strokeStyle = "#f4b35a"; ctx.lineWidth = Math.max(1.4, s * 0.01);
      ctx.beginPath(); ctx.arc(X(tx), Y(ty), Math.max(3, s * 0.11), 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(X(tx) - s * 0.18, Y(ty)); ctx.lineTo(X(tx) + s * 0.18, Y(ty));
      ctx.moveTo(X(tx), Y(ty) - s * 0.18); ctx.lineTo(X(tx), Y(ty) + s * 0.18); ctx.stroke();
    }
    // Scan points transformed by estimated pose
    const loc = state?.loc;
    if (loc && loc.scan && loc.scan.length) {
      const ex = state.robot?.est?.[0] ?? 0, ey = state.robot?.est?.[1] ?? 0, eyaw = state.robot?.est?.[2] ?? 0;
      const c0 = Math.cos(eyaw), s0 = Math.sin(eyaw);
      ctx.fillStyle = "rgba(120,190,220,.55)";
      for (const [px, py] of loc.scan) {
        const wx = ex + c0 * px - s0 * py, wy = ey + s0 * px + c0 * py;
        ctx.fillRect(X(wx), Y(wy), Math.max(1, s * 0.012), Math.max(1, s * 0.012));
      }
    }
    // Trail
    if (loc?.trajectory?.length > 1) {
      ctx.beginPath();
      loc.trajectory.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])));
      ctx.strokeStyle = "rgba(89,239,199,.35)"; ctx.lineWidth = Math.max(1, s * 0.008); ctx.stroke();
    }
    // Cargo
    for (const c of state?.cargo || []) {
      const cat = config.categories.find((x) => x.id === c.category);
      const color = cat?.color || "#aaa";
      const [bx, by] = c.pos;
      ctx.fillStyle = color;
      ctx.globalAlpha = c.status === "carried" ? 0.95 : 0.8;
      ctx.fillRect(X(bx - 0.1), Y(by + 0.1), 0.2 * s, 0.2 * s);
      ctx.globalAlpha = 1;
      if (c.status === "carried") {
        ctx.strokeStyle = "#59efc7"; ctx.lineWidth = 2;
        ctx.strokeRect(X(bx - 0.15), Y(by + 0.15), 0.3 * s, 0.3 * s);
      }
    }
    // Robot
    if (state?.robot?.est) {
      const [rx, ry, ryaw] = state.robot.est;
      const size = Math.max(5, s * 0.24);
      ctx.save(); ctx.translate(X(rx), Y(ry)); ctx.rotate(-ryaw + Math.PI / 2);
      ctx.shadowColor = "#59efc7"; ctx.shadowBlur = 12;
      ctx.fillStyle = "#59efc7";
      ctx.beginPath(); ctx.moveTo(0, -size); ctx.lineTo(size * 0.62, size * 0.7); ctx.lineTo(0, size * 0.34); ctx.lineTo(-size * 0.62, size * 0.7); ctx.closePath(); ctx.fill();
      ctx.restore();
      ctx.strokeStyle = "rgba(89,239,199,.35)"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(X(rx), Y(ry), size * 1.9, 0, Math.PI * 2); ctx.stroke();
    }
  }

  // ------------------------------------------------------------- scan chart
  function drawScanChart() {
    const c = $("#scanChart");
    if (!c) return;
    const r = c.getBoundingClientRect();
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    if (c.width !== Math.round(r.width * dpr) || c.height !== Math.round(r.height * dpr)) {
      c.width = Math.max(80, Math.round(r.width * dpr));
      c.height = Math.max(50, Math.round(r.height * dpr));
    }
    const ctx = c.getContext("2d");
    const w = c.width, h = c.height;
    ctx.clearRect(0, 0, w, h);
    ctx.strokeStyle = "rgba(80,130,160,.12)"; ctx.lineWidth = 1;
    for (let i = 1; i < 4; i++) { const y = h * i / 4; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
    const draw = (arr, max, color, fill) => {
      if (arr.length < 2) return;
      ctx.beginPath();
      arr.forEach((v, i) => {
        const x = i / (arr.length - 1) * w;
        const y = h - clamp(v / max, 0, 1) * (h - 4) - 2;
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      });
      ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke();
      if (fill) {
        ctx.lineTo(w, h); ctx.lineTo(0, h); ctx.closePath();
        ctx.fillStyle = fill; ctx.fill();
      }
    };
    draw(errorHistory, 0.35, "#f4b35a", "rgba(244,179,90,.08)");
    draw(inlierHistory, 360, "#59efc7", "rgba(89,239,199,.07)");
    ctx.font = `${9 * dpr}px monospace`;
    ctx.fillStyle = "#f4b35a"; ctx.fillText("ERR", 5 * dpr, 11 * dpr);
    ctx.fillStyle = "#59efc7"; ctx.fillText("INLIERS", 34 * dpr, 11 * dpr);
  }

  boot();
})();
