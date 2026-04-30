const http = require("http");
const path = require("path");
const fs = require("fs");
const { spawn } = require("child_process");

const ROOT = path.resolve(__dirname, "..");
const DIST_DIR = path.join(ROOT, "dist");
const HOST = process.env.HOST || "0.0.0.0";
const PORT = Number(process.env.PORT || 8787);
const MAX_BODY_BYTES = Number(process.env.MAX_BODY_MB || 256) * 1024 * 1024;

let rosbridgeProcess = null;
let rosbridgeLastOutput = "";
let bagProcess = null;
let bagLastOutput = "";
let bagOptions = null;
let bagFinished = false;
let bagStartedAtMs = 0;
let bagPaused = false;
let bagPausedAtMs = 0;
let bagPausedTotalMs = 0;

function appendOutput(current, data) {
  return (current + data.toString()).slice(-4000);
}

function appendBagOutput(data) {
  bagLastOutput = appendOutput(bagLastOutput, data);
}

function appendRosbridgeOutput(data) {
  rosbridgeLastOutput = appendOutput(rosbridgeLastOutput, data);
}

function rosSetupCommand() {
  return 'if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then source "/opt/ros/$ROS_DISTRO/setup.bash"; elif ! command -v ros2 >/dev/null 2>&1; then for setup in /opt/ros/*/setup.bash; do [ -f "$setup" ] && source "$setup" && break; done; fi';
}

function stopProcessGroup(proc) {
  if (!proc) return false;
  try {
    if (proc.pid) process.kill(-proc.pid, "SIGINT");
  } catch (e) {
    try { proc.kill("SIGINT"); } catch (_) {}
  }
  setTimeout(() => {
    try {
      if (!proc.killed && proc.pid) process.kill(-proc.pid, "SIGTERM");
    } catch (e) {
      try { proc.kill("SIGTERM"); } catch (_) {}
    }
  }, 1500);
  return true;
}

function stopRosbridgeProcess() {
  if (!rosbridgeProcess) return false;
  const proc = rosbridgeProcess;
  rosbridgeProcess = null;
  return stopProcessGroup(proc);
}

function startRosbridgeProcess(options = {}) {
  if (rosbridgeProcess) return { running: true, pid: rosbridgeProcess.pid };
  rosbridgeLastOutput = "";

  const port = Number(options.port) || 9090;
  const address = options.address || "0.0.0.0";
  const cmd = [
    rosSetupCommand(),
    `exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=${address} port:=${port}`,
  ].join("; ");

  const child = spawn("bash", ["-lc", cmd], {
    detached: true,
    stdio: ["ignore", "pipe", "pipe"],
  });
  rosbridgeProcess = child;
  child.stdout.on("data", appendRosbridgeOutput);
  child.stderr.on("data", appendRosbridgeOutput);
  child.on("close", (code, signal) => {
    appendRosbridgeOutput(`\n[rosbridge exited code=${code} signal=${signal || ""}]\n`);
    if (rosbridgeProcess?.pid === child.pid) rosbridgeProcess = null;
  });
  return { running: true, pid: child.pid, port, address };
}

function readBagInfo(bagPath) {
  if (!bagPath || !fs.existsSync(bagPath)) return { duration: 0, messageCount: 0 };
  const metaPath = path.join(bagPath, "metadata.yaml");
  if (!fs.existsSync(metaPath)) return { duration: 0, messageCount: 0 };
  const text = fs.readFileSync(metaPath, "utf-8");
  const durationMatch = text.match(/duration:\s*\n\s*nanoseconds:\s*([0-9]+)/);
  const countMatch = text.match(/message_count:\s*([0-9]+)/);
  const ns = durationMatch ? Number(durationMatch[1]) : 0;
  return {
    duration: Number.isFinite(ns) ? ns / 1e9 : 0,
    messageCount: countMatch ? Number(countMatch[1]) : 0,
  };
}

function currentBagOffset() {
  if (!bagProcess || !bagOptions) return bagOptions?.startOffset || 0;
  const now = bagPaused ? bagPausedAtMs : Date.now();
  const elapsedMs = Math.max(0, now - bagStartedAtMs - bagPausedTotalMs);
  return (bagOptions.startOffset || 0) + elapsedMs / 1000 * (bagOptions.rate || 1);
}

function stopBagProcess() {
  if (!bagProcess) return false;
  const proc = bagProcess;
  const offset = currentBagOffset();
  if (bagOptions) bagOptions = { ...bagOptions, startOffset: offset };
  bagProcess = null;
  bagPaused = false;
  bagFinished = false;
  return stopProcessGroup(proc);
}

function startBagProcess(options = {}) {
  const bagPath = options.path;
  if (!bagPath) throw new Error("bag path is required");
  if (!fs.existsSync(bagPath)) throw new Error(`bag path not found: ${bagPath}`);

  stopBagProcess();
  bagLastOutput = "";

  const rate = Number(options.rate);
  const safeRate = Number.isFinite(rate) && rate > 0 ? Math.min(rate, 100) : 1;
  const startOffset = Math.max(0, Number(options.startOffset) || 0);
  const extra = [
    options.clock !== false ? "--clock" : "",
    options.loop ? "--loop" : "",
    safeRate !== 1 ? `--rate ${safeRate}` : "",
    startOffset > 0 ? `--start-offset ${startOffset}` : "",
  ].filter(Boolean).join(" ");
  const cmd = [
    rosSetupCommand(),
    `exec ros2 bag play "$BAG_PATH" ${extra}`,
  ].join("; ");

  const child = spawn("bash", ["-lc", cmd], {
    env: { ...process.env, BAG_PATH: bagPath },
    detached: true,
    stdio: ["ignore", "pipe", "pipe"],
  });
  bagProcess = child;
  const info = readBagInfo(bagPath);
  bagOptions = { path: bagPath, clock: options.clock !== false, loop: !!options.loop, rate: safeRate, startOffset, duration: info.duration };
  bagFinished = false;
  bagStartedAtMs = Date.now();
  bagPaused = false;
  bagPausedAtMs = 0;
  bagPausedTotalMs = 0;

  child.stdout.on("data", appendBagOutput);
  child.stderr.on("data", appendBagOutput);
  child.on("close", (code, signal) => {
    appendBagOutput(`\n[rosbag exited code=${code} signal=${signal || ""}]\n`);
    if (bagProcess?.pid === child.pid) {
      const finalOffset = currentBagOffset();
      const duration = bagOptions?.duration || 0;
      const reachedEnd = !bagOptions?.loop && (code === 0 || (duration > 0 && finalOffset >= Math.max(0, duration - 0.5)));
      bagOptions = { ...bagOptions, startOffset: reachedEnd ? 0 : finalOffset };
      bagFinished = reachedEnd;
      bagProcess = null;
      bagPaused = false;
      bagPausedAtMs = 0;
      bagPausedTotalMs = 0;
    }
  });

  return { running: true, paused: false, pid: child.pid, offset: startOffset, duration: info.duration };
}

function sendJson(res, status, payload) {
  res.writeHead(status, {
    "Content-Type": "application/json",
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "GET,POST,OPTIONS",
    "Access-Control-Allow-Headers": "Content-Type",
  });
  res.end(JSON.stringify(payload));
}

function sendOk(res, data) {
  sendJson(res, 200, { ok: true, data });
}

function sendError(res, error, status = 500) {
  sendJson(res, status, { ok: false, error: error?.message || String(error) });
}

function readJsonBody(req) {
  return new Promise((resolve, reject) => {
    let body = "";
    req.on("data", chunk => {
      body += chunk;
      if (body.length > MAX_BODY_BYTES) {
        reject(new Error(`request body too large, limit ${MAX_BODY_BYTES} bytes`));
        req.destroy();
      }
    });
    req.on("end", () => {
      if (!body.trim()) return resolve({});
      try { resolve(JSON.parse(body)); }
      catch (e) { reject(new Error(`invalid JSON body: ${e.message}`)); }
    });
    req.on("error", reject);
  });
}

const apiRoutes = {
  "POST /api/file/read": async body => {
    if (!body.path) throw new Error("path is required");
    if (body.encoding) return { data: fs.readFileSync(body.path, body.encoding), binary: false };
    return { data: fs.readFileSync(body.path).toString("base64"), binary: true };
  },
  "POST /api/file/write": async body => {
    if (!body.path) throw new Error("path is required");
    const data = body.binary ? Buffer.from(body.data || "", "base64") : String(body.data ?? "");
    fs.writeFileSync(body.path, data, body.binary ? undefined : (body.encoding || "utf-8"));
    return true;
  },
  "POST /api/fs/readdir": async body => {
    if (!body.path) throw new Error("path is required");
    return fs.readdirSync(body.path);
  },
  "POST /api/rosbridge/start": async body => startRosbridgeProcess(body || {}),
  "POST /api/rosbridge/stop": async () => ({ running: false, stopped: stopRosbridgeProcess() }),
  "GET /api/rosbridge/status": async () => ({ running: !!rosbridgeProcess, output: rosbridgeLastOutput }),
  "POST /api/rosbag/play": async body => startBagProcess(body || {}),
  "POST /api/rosbag/info": async body => readBagInfo(body.path),
  "POST /api/rosbag/stop": async () => ({ running: false, stopped: stopBagProcess() }),
  "POST /api/rosbag/pause": async () => {
    if (!bagProcess || bagPaused) return { running: !!bagProcess, paused: bagPaused, offset: currentBagOffset() };
    process.kill(-bagProcess.pid, "SIGSTOP");
    bagPaused = true;
    bagPausedAtMs = Date.now();
    return { running: true, paused: true, offset: currentBagOffset() };
  },
  "POST /api/rosbag/resume": async () => {
    if (!bagProcess || !bagPaused) return { running: !!bagProcess, paused: bagPaused, offset: currentBagOffset() };
    process.kill(-bagProcess.pid, "SIGCONT");
    bagPausedTotalMs += Date.now() - bagPausedAtMs;
    bagPaused = false;
    bagPausedAtMs = 0;
    return { running: true, paused: false, offset: currentBagOffset() };
  },
  "POST /api/rosbag/seek": async body => {
    const base = bagOptions || body;
    if (!base?.path) throw new Error("bag path is required");
    const requested = body.offset != null ? Number(body.offset) : currentBagOffset() + Number(body.delta || 0);
    const offset = Math.max(0, requested || 0);
    return startBagProcess({ ...base, ...body, path: base.path, startOffset: offset });
  },
  "GET /api/rosbag/status": async () => ({
    running: !!bagProcess,
    paused: bagPaused,
    finished: bagFinished,
    offset: currentBagOffset(),
    duration: bagOptions?.duration || 0,
    output: bagLastOutput,
  }),
  "GET /api/health": async () => ({ ok: true, pid: process.pid }),
};

function contentType(filePath) {
  const ext = path.extname(filePath).toLowerCase();
  return {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
  }[ext] || "application/octet-stream";
}

function serveStatic(req, res) {
  if (!fs.existsSync(DIST_DIR)) {
    res.writeHead(200, { "Content-Type": "text/plain; charset=utf-8" });
    res.end("dist/ not found. Run `npm run build` first, or use `npm run robot:dev`.");
    return;
  }
  const url = new URL(req.url, `http://${req.headers.host || "localhost"}`);
  const cleanPath = decodeURIComponent(url.pathname).replace(/^\/+/, "");
  const candidate = path.resolve(DIST_DIR, cleanPath || "index.html");
  const relative = path.relative(DIST_DIR, candidate);
  const insideDist = relative && !relative.startsWith("..") && !path.isAbsolute(relative);
  const filePath = insideDist && fs.existsSync(candidate) && fs.statSync(candidate).isFile()
    ? candidate
    : path.join(DIST_DIR, "index.html");
  res.writeHead(200, { "Content-Type": contentType(filePath) });
  fs.createReadStream(filePath).pipe(res);
}

const server = http.createServer(async (req, res) => {
  res.setHeader("Access-Control-Allow-Origin", "*");
  res.setHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  res.setHeader("Access-Control-Allow-Headers", "Content-Type");
  if (req.method === "OPTIONS") {
    res.writeHead(204);
    res.end();
    return;
  }

  const routeKey = `${req.method} ${new URL(req.url, `http://${req.headers.host || "localhost"}`).pathname}`;
  const route = apiRoutes[routeKey];
  if (route) {
    try {
      const body = req.method === "POST" ? await readJsonBody(req) : {};
      sendOk(res, await route(body));
    } catch (e) {
      sendError(res, e);
    }
    return;
  }

  if (routeKey.includes(" /api/")) {
    sendError(res, new Error(`unknown API route: ${routeKey}`), 404);
    return;
  }

  serveStatic(req, res);
});

function shutdown() {
  stopRosbridgeProcess();
  stopBagProcess();
  server.close(() => process.exit(0));
  setTimeout(() => process.exit(0), 2000).unref();
}

process.on("SIGINT", shutdown);
process.on("SIGTERM", shutdown);

server.on("error", error => {
  console.error(`Robot backend failed to listen on ${HOST}:${PORT}: ${error.message}`);
  process.exit(1);
});

server.listen(PORT, HOST, () => {
  console.log(`Robot backend listening on http://${HOST}:${PORT}`);
  console.log(`Open from another PC: http://<robot-pc-ip>:${PORT}`);
});
