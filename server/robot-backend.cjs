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
let slamProcess = null;
let slamLastOutput = "";
let slamOptions = null;
let nav2Process = null;
let nav2LastOutput = "";
let nav2Options = null;
let bagProcess = null;
let bagLastOutput = "";
let bagOptions = null;
let bagRecordProcess = null;
let bagRecordLastOutput = "";
let bagRecordOptions = null;
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

function appendBagRecordOutput(data) {
  bagRecordLastOutput = appendOutput(bagRecordLastOutput, data);
}

function appendRosbridgeOutput(data) {
  rosbridgeLastOutput = appendOutput(rosbridgeLastOutput, data);
}

function appendSlamOutput(data) {
  slamLastOutput = appendOutput(slamLastOutput, data);
}

function appendNav2Output(data) {
  nav2LastOutput = appendOutput(nav2LastOutput, data);
}

function rosSetupCommand() {
  return 'if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then source "/opt/ros/$ROS_DISTRO/setup.bash"; elif ! command -v ros2 >/dev/null 2>&1; then for setup in /opt/ros/*/setup.bash; do [ -f "$setup" ] && source "$setup" && break; done; fi';
}

function shellQuote(value) {
  return `'${String(value ?? "").replace(/'/g, `'"'"'`)}'`;
}

const DEFAULT_BAG_RECORD_DIR = process.env.MAP_UI_BAG_RECORD_DIR || "/home/nvidia/rby1_nav2/src/rby1_nav2/bag";
const DEFAULT_BAG_RECORD_TOPICS = [
  "/tf",
  "/odom",
  "/joint_states",
  "/scan_merged",
  "/camera/camera_head/color/image_raw/compressed",
  "/camera/camera_left/color/image_rect_raw/compressed",
  "/camera/camera_right/color/image_rect_raw/compressed",
];

const THOR_DEFAULTS = {
  host: process.env.THOR_IP || "192.168.78.11",
  user: process.env.THOR_USER || "thor",
  password: process.env.THOR_PASSWORD || "thor",
  destDir: process.env.THOR_SEMANTIC_DIR || "/home/thor/bt_ws/map_json",
  mdDir: process.env.THOR_MD_DIR || "/home/thor/inha_ws/arena_info/InhaDreamOpen2026",
};

function runShellCommand(command, options = {}) {
  return new Promise((resolve, reject) => {
    let stdout = "";
    let stderr = "";
    const child = spawn("bash", ["-lc", command], {
      cwd: options.cwd || process.cwd(),
      env: { ...process.env, ...(options.env || {}) },
      detached: false,
      stdio: ["ignore", "pipe", "pipe"],
    });
    child.stdout.on("data", data => { stdout += data.toString(); });
    child.stderr.on("data", data => { stderr += data.toString(); });
    child.on("error", reject);
    child.on("close", (code, signal) => {
      if (code === 0) {
        resolve({ code, signal, stdout, stderr });
        return;
      }
      const tail = (stderr || stdout).trim().split(/\r?\n/).slice(-3).join("\n");
      const error = new Error(`command failed with code ${code}${signal ? ` signal ${signal}` : ""}${tail ? `: ${tail}` : ""}`);
      error.code = code;
      error.signal = signal;
      error.stdout = stdout;
      error.stderr = stderr;
      reject(error);
    });
  });
}

async function copySemanticToThor(options = {}) {
  const sourcePath = String(options.sourcePath || "").trim();
  if (!sourcePath) throw new Error("sourcePath is required");
  if (!fs.existsSync(sourcePath)) throw new Error(`semantic JSON not found: ${sourcePath}`);

  const host = String(options.host || THOR_DEFAULTS.host).trim();
  const user = String(options.user || THOR_DEFAULTS.user).trim();
  const password = String(options.password || THOR_DEFAULTS.password);
  const destDir = String(options.destDir || THOR_DEFAULTS.destDir).replace(/\/+$/, "");
  const fileName = path.basename(String(options.fileName || sourcePath));
  if (!host || !user || !destDir || !fileName) throw new Error("Thor copy target is incomplete");

  const remote = `${user}@${host}`;
  const destPath = `${destDir}/${fileName}`;
  const command = [
    'command -v sshpass >/dev/null 2>&1 || { echo "sshpass is required for password SSH copy" >&2; exit 127; }',
    `sshpass -p "$THOR_PASSWORD" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null ${shellQuote(remote)} ${shellQuote(`mkdir -p ${shellQuote(destDir)}`)}`,
    `sshpass -p "$THOR_PASSWORD" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null ${shellQuote(sourcePath)} ${shellQuote(`${remote}:${destPath}`)}`,
  ].join("; ");
  const result = await runShellCommand(command, { env: { THOR_PASSWORD: password } });
  return { sourcePath, destPath, host, user, ...result };
}

function thorSshTarget(options = {}) {
  const host = String(options.host || THOR_DEFAULTS.host).trim();
  const user = String(options.user || THOR_DEFAULTS.user).trim();
  const password = String(options.password || THOR_DEFAULTS.password);
  if (!host || !user) throw new Error("Thor SSH target is incomplete");
  return { host, user, password, remote: `${user}@${host}` };
}

async function runThorCommand(remoteCommand, options = {}) {
  const target = thorSshTarget(options);
  const sshOptions = "-o ConnectTimeout=5 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null";
  const plainSsh = `ssh ${target.password ? "-o BatchMode=yes " : ""}${sshOptions} ${shellQuote(target.remote)} ${shellQuote(remoteCommand)}`;
  const command = target.password
    ? `if command -v sshpass >/dev/null 2>&1; then sshpass -p "$THOR_PASSWORD" ssh ${sshOptions} ${shellQuote(target.remote)} ${shellQuote(remoteCommand)}; else ${plainSsh}; fi`
    : plainSsh;
  const result = await runShellCommand(command, { env: { THOR_PASSWORD: target.password } });
  return { ...target, ...result };
}

function thorMdDir(options = {}) {
  return String(options.dir || THOR_DEFAULTS.mdDir).replace(/\/+$/, "");
}

function safeThorRelativePath(value) {
  const rel = String(value || "").replace(/\\/g, "/").replace(/^\/+/, "");
  if (!rel || rel.includes("\0") || rel.split("/").some(part => part === "..")) {
    throw new Error("invalid Thor markdown path");
  }
  return rel;
}

async function listThorMarkdownFiles(options = {}) {
  const dir = thorMdDir(options);
  const maxDepth = Math.max(1, Math.min(5, Number(options.maxDepth || 2) || 2));
  const remoteCommand = `cd ${shellQuote(dir)} && find . -maxdepth ${maxDepth} -type f -iname '*.md' -printf '%P\t%s\t%T@\\n'`;
  const result = await runThorCommand(remoteCommand, options);
  const files = result.stdout.split(/\r?\n/)
    .filter(Boolean)
    .map(line => {
      const [relativePath, size, mtime] = line.split("\t");
      return {
        name: path.posix.basename(relativePath || ""),
        relativePath,
        path: `${dir}/${relativePath}`,
        size: Number(size) || 0,
        mtimeMs: (Number(mtime) || 0) * 1000,
      };
    })
    .filter(file => file.relativePath)
    .sort((a, b) => a.relativePath.localeCompare(b.relativePath));
  return { dir, host: result.host, user: result.user, files };
}

async function readThorMarkdownFile(options = {}) {
  const dir = thorMdDir(options);
  const rel = safeThorRelativePath(options.path || options.relativePath);
  const remoteCommand = `cd ${shellQuote(dir)} && cat -- ${shellQuote(rel)}`;
  const result = await runThorCommand(remoteCommand, options);
  return {
    dir,
    host: result.host,
    user: result.user,
    name: path.posix.basename(rel),
    relativePath: rel,
    path: `${dir}/${rel}`,
    text: result.stdout,
  };
}

function escapeRegExp(value) {
  return String(value).replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

function findCallEnd(source, openParenIndex) {
  let depth = 0;
  let quote = "";
  let escaped = false;
  for (let i = openParenIndex; i < source.length; i += 1) {
    const ch = source[i];
    if (quote) {
      if (escaped) {
        escaped = false;
      } else if (ch === "\\") {
        escaped = true;
      } else if (ch === quote) {
        quote = "";
      }
      continue;
    }
    if (ch === "\"" || ch === "'") {
      quote = ch;
    } else if (ch === "(") {
      depth += 1;
    } else if (ch === ")") {
      depth -= 1;
      if (depth === 0) return i;
    }
  }
  return -1;
}

function firstStringArg(callText, callName) {
  const match = callText.match(new RegExp(`^${escapeRegExp(callName)}\\s*\\(\\s*(['"])(.*?)\\1`, "s"));
  return match?.[2] || "";
}

function findValueEnd(callText, valueStart) {
  let depth = 0;
  let quote = "";
  let escaped = false;
  for (let i = valueStart; i < callText.length; i += 1) {
    const ch = callText[i];
    if (quote) {
      if (escaped) {
        escaped = false;
      } else if (ch === "\\") {
        escaped = true;
      } else if (ch === quote) {
        quote = "";
      }
      continue;
    }
    if (ch === "\"" || ch === "'") {
      quote = ch;
    } else if (ch === "(" || ch === "[" || ch === "{") {
      depth += 1;
    } else if (ch === ")" || ch === "]" || ch === "}") {
      if (depth === 0) return i;
      depth -= 1;
    } else if (ch === "," && depth === 0) {
      return i;
    }
  }
  return callText.length;
}

function replaceKeywordValue(callText, keyword, quotedValue) {
  const match = new RegExp(`\\b${escapeRegExp(keyword)}\\s*=`).exec(callText);
  if (!match) return { text: callText, changed: false };
  let valueStart = match.index + match[0].length;
  while (/\s/.test(callText[valueStart] || "")) valueStart += 1;
  const valueEnd = findValueEnd(callText, valueStart);
  if (valueEnd <= valueStart) return { text: callText, changed: false };
  const nextText = `${callText.slice(0, valueStart)}${quotedValue}${callText.slice(valueEnd)}`;
  return { text: nextText, changed: nextText !== callText };
}

function replaceCallKeywordForArg(text, callName, argName, keyword, quotedValue) {
  const callPattern = new RegExp(`${escapeRegExp(callName)}\\s*\\(`, "g");
  let result = "";
  let cursor = 0;
  let count = 0;
  let match;
  while ((match = callPattern.exec(text))) {
    const openParenIndex = text.indexOf("(", match.index);
    const closeParenIndex = findCallEnd(text, openParenIndex);
    if (closeParenIndex < 0) break;
    const callText = text.slice(match.index, closeParenIndex + 1);
    if (firstStringArg(callText, callName) === argName) {
      const replaced = replaceKeywordValue(callText, keyword, quotedValue);
      if (replaced.changed) {
        result += text.slice(cursor, match.index) + replaced.text;
        cursor = closeParenIndex + 1;
        count += 1;
      }
    }
    callPattern.lastIndex = closeParenIndex + 1;
  }
  return count ? { text: result + text.slice(cursor), count } : { text, count: 0 };
}

function updateLaunchMapText(text, mapPath, argName = "map") {
  const normalized = String(mapPath || "").replace(/\\/g, "/");
  if (!normalized) return text;
  const quotedMapPath = JSON.stringify(normalized);
  let updated = String(text || "");
  let changed = 0;
  for (const [callName, keyword] of [
    ["LaunchConfiguration", "default"],
    ["DeclareLaunchArgument", "default_value"],
  ]) {
    const result = replaceCallKeywordForArg(updated, callName, argName, keyword, quotedMapPath);
    updated = result.text;
    changed += result.count;
  }
  return changed ? updated : text;
}

function buildWorkspaceCommand(options = {}) {
  const custom = String(options.command || "").trim();
  if (custom) return custom;
  const packages = Array.isArray(options.packages)
    ? options.packages
    : String(options.packageName || "")
        .split(",")
        .map(part => part.trim())
        .filter(Boolean);
  const packageArgs = packages.length ? ` --packages-select ${packages.map(shellQuote).join(" ")}` : "";
  return `colcon build${packageArgs}`;
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

function stopSlamProcess() {
  if (!slamProcess) return false;
  const proc = slamProcess;
  slamProcess = null;
  return stopProcessGroup(proc);
}

function startSlamProcess(options = {}) {
  if (slamProcess) return { running: true, pid: slamProcess.pid, options: slamOptions };
  slamLastOutput = "";

  const mode = options.mode === "online_sync" ? "online_sync" : "online_async";
  const launchFile = `${mode}_launch.py`;
  const useSimTime = options.useSimTime === true || options.use_sim_time === true;
  const paramsFile = String(options.paramsFile || options.slam_params_file || "").trim();
  const launchArgs = [`use_sim_time:=${useSimTime ? "true" : "false"}`];
  if (paramsFile) launchArgs.push('slam_params_file:="$SLAM_PARAMS_FILE"');
  const cmd = [
    rosSetupCommand(),
    `exec ros2 launch slam_toolbox ${launchFile} ${launchArgs.join(" ")}`,
  ].join("; ");

  const child = spawn("bash", ["-lc", cmd], {
    env: { ...process.env, SLAM_PARAMS_FILE: paramsFile },
    detached: true,
    stdio: ["ignore", "pipe", "pipe"],
  });
  slamProcess = child;
  slamOptions = { mode, useSimTime, paramsFile };
  child.stdout.on("data", appendSlamOutput);
  child.stderr.on("data", appendSlamOutput);
  child.on("close", (code, signal) => {
    appendSlamOutput(`\n[slam_toolbox exited code=${code} signal=${signal || ""}]\n`);
    if (slamProcess?.pid === child.pid) slamProcess = null;
  });
  return { running: true, pid: child.pid, options: slamOptions };
}

function stopNav2Process() {
  if (!nav2Process) return false;
  const proc = nav2Process;
  nav2Process = null;
  return stopProcessGroup(proc);
}

function startNav2Process(options = {}) {
  if (nav2Process) return { running: true, pid: nav2Process.pid, options: nav2Options };
  if (options.allowMotion !== true) {
    return { running: false, blocked: true, reason: "Nav2 launch is blocked by default to prevent robot motion" };
  }
  nav2LastOutput = "";

  const useSimTime = options.useSimTime === true || options.use_sim_time === true;
  const autostart = options.autostart !== false;
  const paramsFile = String(options.paramsFile || options.params_file || "").trim();
  const launchArgs = [
    `use_sim_time:=${useSimTime ? "true" : "false"}`,
    `autostart:=${autostart ? "true" : "false"}`,
  ];
  if (paramsFile) launchArgs.push('params_file:="$NAV2_PARAMS_FILE"');
  const cmd = [
    rosSetupCommand(),
    `exec ros2 launch nav2_bringup navigation_launch.py ${launchArgs.join(" ")}`,
  ].join("; ");

  const child = spawn("bash", ["-lc", cmd], {
    env: { ...process.env, NAV2_PARAMS_FILE: paramsFile },
    detached: true,
    stdio: ["ignore", "pipe", "pipe"],
  });
  nav2Process = child;
  nav2Options = { useSimTime, autostart, paramsFile, launchFile: "navigation_launch.py" };
  child.stdout.on("data", appendNav2Output);
  child.stderr.on("data", appendNav2Output);
  child.on("close", (code, signal) => {
    appendNav2Output(`\n[nav2 exited code=${code} signal=${signal || ""}]\n`);
    if (nav2Process?.pid === child.pid) nav2Process = null;
  });
  return { running: true, pid: child.pid, options: nav2Options };
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

function fsRoots() {
  return {
    home: process.env.HOME || ROOT,
    cwd: process.cwd(),
    root: "/",
  };
}

function readDirDetailed(dirPath) {
  const entries = fs.readdirSync(dirPath, { withFileTypes: true }).map(entry => {
    const fullPath = path.join(dirPath, entry.name);
    let stat = null;
    try { stat = fs.statSync(fullPath); } catch (_) {}
    return {
      name: entry.name,
      path: fullPath,
      isDirectory: entry.isDirectory(),
      isFile: entry.isFile(),
      isSymbolicLink: entry.isSymbolicLink(),
      size: stat?.size || 0,
      mtimeMs: stat?.mtimeMs || 0,
    };
  });
  entries.sort((a, b) => {
    if (a.isDirectory !== b.isDirectory) return a.isDirectory ? -1 : 1;
    return a.name.localeCompare(b.name);
  });
  return { path: dirPath, entries };
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

function stopBagRecordProcess() {
  if (!bagRecordProcess) return false;
  const proc = bagRecordProcess;
  bagRecordProcess = null;
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
  }, 10000);
  return true;
}

function bagRecordName() {
  const d = new Date();
  const pad = n => String(n).padStart(2, "0");
  return `bag_${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
}

function uniqueBagRecordOutput(outputDir, name) {
  let candidateName = name;
  let outputPath = path.join(outputDir, candidateName);
  for (let i = 2; fs.existsSync(outputPath); i += 1) {
    candidateName = `${name}_${i}`;
    outputPath = path.join(outputDir, candidateName);
  }
  return { name: candidateName, outputPath };
}

function startBagRecordProcess(options = {}) {
  if (bagRecordProcess) return { running: true, pid: bagRecordProcess.pid, options: bagRecordOptions };
  bagRecordLastOutput = "";

  const outputDir = String(options.outputDir || options.output_dir || DEFAULT_BAG_RECORD_DIR).trim() || DEFAULT_BAG_RECORD_DIR;
  let name = String(options.name || options.bagName || bagRecordName()).trim() || bagRecordName();
  const topics = Array.isArray(options.topics) && options.topics.length
    ? options.topics.map(String).filter(Boolean)
    : DEFAULT_BAG_RECORD_TOPICS;
  fs.mkdirSync(outputDir, { recursive: true });
  const uniqueOutput = uniqueBagRecordOutput(outputDir, name);
  name = uniqueOutput.name;
  const outputPath = uniqueOutput.outputPath;
  const cmd = [
    rosSetupCommand(),
    `exec ros2 bag record -o "$BAG_RECORD_OUTPUT" ${topics.map(shellQuote).join(" ")}`,
  ].join("; ");

  const child = spawn("bash", ["-lc", cmd], {
    env: { ...process.env, BAG_RECORD_OUTPUT: outputPath },
    detached: true,
    stdio: ["ignore", "pipe", "pipe"],
  });
  bagRecordProcess = child;
  bagRecordOptions = { outputDir, name, outputPath, topics };
  child.stdout.on("data", appendBagRecordOutput);
  child.stderr.on("data", appendBagRecordOutput);
  child.on("close", (code, signal) => {
    appendBagRecordOutput(`\n[rosbag record exited code=${code} signal=${signal || ""}]\n`);
    if (bagRecordProcess?.pid === child.pid) bagRecordProcess = null;
  });
  return { running: true, pid: child.pid, options: bagRecordOptions };
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
  "POST /api/thor/copy-semantic": async body => copySemanticToThor(body || {}),
  "POST /api/thor/list-md": async body => listThorMarkdownFiles(body || {}),
  "POST /api/thor/read-md": async body => readThorMarkdownFile(body || {}),
  "POST /api/workspace/update-launch-map": async body => {
    if (!body.launchPath) throw new Error("launchPath is required");
    if (!body.mapPath) throw new Error("mapPath is required");
    const text = fs.readFileSync(body.launchPath, "utf-8");
    const argName = body.argName || "map";
    const updated = updateLaunchMapText(text, body.mapPath, argName);
    if (updated !== text) {
      fs.writeFileSync(body.launchPath, updated, "utf-8");
    }
    return { launchPath: body.launchPath, mapPath: body.mapPath, argName, changed: updated !== text };
  },
  "POST /api/workspace/build": async body => {
    const cwd = body.cwd || process.cwd();
    const command = buildWorkspaceCommand(body);
    const result = await runShellCommand(`${rosSetupCommand()}; exec ${command}`, { cwd });
    return { cwd, command, ...result };
  },
  "POST /api/fs/readdir": async body => {
    if (!body.path) throw new Error("path is required");
    if (body.withFileTypes) return readDirDetailed(body.path);
    return fs.readdirSync(body.path);
  },
  "GET /api/fs/roots": async () => fsRoots(),
  "POST /api/rosbridge/start": async body => startRosbridgeProcess(body || {}),
  "POST /api/rosbridge/stop": async () => ({ running: false, stopped: stopRosbridgeProcess() }),
  "GET /api/rosbridge/status": async () => ({ running: !!rosbridgeProcess, output: rosbridgeLastOutput }),
  "POST /api/slam/start": async body => startSlamProcess(body || {}),
  "POST /api/slam/stop": async () => ({ running: false, stopped: stopSlamProcess() }),
  "GET /api/slam/status": async () => ({ running: !!slamProcess, output: slamLastOutput, options: slamOptions }),
  "POST /api/nav2/start": async body => startNav2Process(body || {}),
  "POST /api/nav2/stop": async () => ({ running: false, stopped: stopNav2Process() }),
  "GET /api/nav2/status": async () => ({ running: !!nav2Process, output: nav2LastOutput, options: nav2Options }),
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
  "POST /api/rosbag/record/start": async body => startBagRecordProcess(body || {}),
  "POST /api/rosbag/record/stop": async () => ({ running: false, stopped: stopBagRecordProcess(), options: bagRecordOptions }),
  "GET /api/rosbag/record/status": async () => ({ running: !!bagRecordProcess, output: bagRecordLastOutput, options: bagRecordOptions }),
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
  stopSlamProcess();
  stopNav2Process();
  stopBagProcess();
  stopBagRecordProcess();
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
