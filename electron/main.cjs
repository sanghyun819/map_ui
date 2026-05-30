const { app, BrowserWindow, ipcMain, dialog } = require("electron");
const path = require("path");
const fs = require("fs");
const { spawn } = require("child_process");

let mainWindow;
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

function appendBagOutput(data) {
  bagLastOutput = (bagLastOutput + data.toString()).slice(-4000);
}

function appendBagRecordOutput(data) {
  bagRecordLastOutput = (bagRecordLastOutput + data.toString()).slice(-4000);
}

function appendRosbridgeOutput(data) {
  rosbridgeLastOutput = (rosbridgeLastOutput + data.toString()).slice(-4000);
}

function appendSlamOutput(data) {
  slamLastOutput = (slamLastOutput + data.toString()).slice(-4000);
}

function appendNav2Output(data) {
  nav2LastOutput = (nav2LastOutput + data.toString()).slice(-4000);
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
  "/tf_static",
  "/odom",
  "/joint_states",
  "/scan_merged",
  "/livox/lidar",
  "/camera/camera_head/color/image_raw/compressed",
  "/camera/camera_left/color/image_rect_raw/compressed",
  "/camera/camera_right/color/image_rect_raw/compressed",
];

const THOR_DEFAULTS = {
  host: process.env.THOR_IP || "192.168.78.11",
  user: process.env.THOR_USER || "thor",
  password: process.env.THOR_PASSWORD || "thor",
  destDir: process.env.THOR_SEMANTIC_DIR || "/home/thor/bt_ws/map_json",
  mdDir: process.env.THOR_MD_DIR || "/home/thor/inha_ws/arena_info",
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

function safeThorRelativeDir(value) {
  const rel = String(value || "")
    .replace(/\\/g, "/")
    .replace(/^\/+/, "")
    .replace(/\/+/g, "/")
    .replace(/\/+$/, "");
  if (rel.includes("\0") || rel.split("/").some(part => part === "..")) {
    throw new Error("invalid Thor markdown directory");
  }
  return rel === "." ? "" : rel;
}

async function listThorMarkdownDir(options = {}) {
  const dir = thorMdDir(options);
  const rel = safeThorRelativeDir(options.path || options.relativePath || "");
  const searchPath = rel ? `./${rel}` : ".";
  const remoteCommand = `cd ${shellQuote(dir)} && find ${shellQuote(searchPath)} -maxdepth 1 -mindepth 1 \\( -type d -o -iname '*.md' \\) -printf '%p\t%f\t%y\t%s\t%T@\\n'`;
  const result = await runThorCommand(remoteCommand, options);
  const entries = result.stdout.split(/\r?\n/)
    .filter(Boolean)
    .map(line => {
      const [rawPath, name, type, size, mtime] = line.split("\t");
      const relativePath = String(rawPath || "").replace(/^\.\//, "");
      const isDirectory = type === "d";
      return {
        name: name || path.posix.basename(relativePath),
        relativePath,
        path: `${dir}/${relativePath}`,
        isDirectory,
        isFile: !isDirectory,
        size: Number(size) || 0,
        mtimeMs: (Number(mtime) || 0) * 1000,
      };
    })
    .filter(entry => entry.relativePath)
    .sort((a, b) => {
      if (a.isDirectory !== b.isDirectory) return a.isDirectory ? -1 : 1;
      return a.name.localeCompare(b.name);
    });
  return {
    dir,
    path: rel,
    absolutePath: rel ? `${dir}/${rel}` : dir,
    host: result.host,
    user: result.user,
    entries,
  };
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

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1400,
    height: 900,
    title: "Nav2 Map Editor",
    webPreferences: {
      preload: path.join(__dirname, "preload.cjs"),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });

  // dev mode: Vite dev server / production: built files
  if (process.env.VITE_DEV_SERVER_URL) {
    mainWindow.loadURL(process.env.VITE_DEV_SERVER_URL);
  } else {
    mainWindow.loadFile(path.join(__dirname, "../dist/index.html"));
  }
}

app.whenReady().then(createWindow);

app.on("window-all-closed", () => {
  stopRosbridgeProcess();
  stopSlamProcess();
  stopNav2Process();
  stopBagProcess();
  stopBagRecordProcess();
  app.quit();
});

app.on("activate", () => {
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});

// --- IPC Handlers: file system access ---

// Open file dialog
ipcMain.handle("dialog:openFile", async (event, options) => {
  const result = await dialog.showOpenDialog(mainWindow, options);
  if (result.canceled) return null;
  if (options?.properties?.includes("multiSelections")) return result.filePaths;
  return result.filePaths[0];
});

// Save file dialog
ipcMain.handle("dialog:saveFile", async (event, options) => {
  const result = await dialog.showSaveDialog(mainWindow, options);
  if (result.canceled) return null;
  return result.filePath;
});

// Read file
ipcMain.handle("fs:readFile", async (event, filePath, encoding) => {
  return fs.readFileSync(filePath, encoding || null);
});

// Write file
ipcMain.handle("fs:writeFile", async (event, filePath, data, encoding) => {
  fs.writeFileSync(filePath, data, encoding || "utf-8");
  return true;
});

ipcMain.handle("thor:copySemantic", async (event, options = {}) => {
  return copySemanticToThor(options);
});

ipcMain.handle("thor:listMdFiles", async (event, options = {}) => {
  return listThorMarkdownFiles(options);
});

ipcMain.handle("thor:listMdDir", async (event, options = {}) => {
  return listThorMarkdownDir(options);
});

ipcMain.handle("thor:readMdFile", async (event, options = {}) => {
  return readThorMarkdownFile(options);
});

ipcMain.handle("workspace:updateLaunchMap", async (event, options = {}) => {
  const launchPath = options.launchPath;
  const mapPath = options.mapPath;
  if (!launchPath) throw new Error("launchPath is required");
  if (!mapPath) throw new Error("mapPath is required");
  const text = fs.readFileSync(launchPath, "utf-8");
  const argName = options.argName || "map";
  const updated = updateLaunchMapText(text, mapPath, argName);
  if (updated !== text) {
    fs.writeFileSync(launchPath, updated, "utf-8");
  }
  return { launchPath, mapPath, argName, changed: updated !== text };
});

ipcMain.handle("workspace:build", async (event, options = {}) => {
  const cwd = options.cwd || process.cwd();
  const command = buildWorkspaceCommand(options);
  const result = await runShellCommand(`${rosSetupCommand()}; exec ${command}`, { cwd });
  return { cwd, command, ...result };
});

// Read directory
ipcMain.handle("fs:readDir", async (event, dirPath) => {
  return fs.readdirSync(dirPath);
});

// rosbridge websocket launch control.
ipcMain.handle("rosbridge:start", async (event, options = {}) => {
  return startRosbridgeProcess(options);
});

ipcMain.handle("rosbridge:stop", async () => {
  const stopped = stopRosbridgeProcess();
  return { running: false, stopped };
});

ipcMain.handle("rosbridge:status", async () => {
  return { running: !!rosbridgeProcess, output: rosbridgeLastOutput };
});

// slam_toolbox launch control.
ipcMain.handle("slam:start", async (event, options = {}) => {
  return startSlamProcess(options);
});

ipcMain.handle("slam:stop", async () => {
  const stopped = stopSlamProcess();
  return { running: false, stopped };
});

ipcMain.handle("slam:status", async () => {
  return { running: !!slamProcess, output: slamLastOutput, options: slamOptions };
});

// Nav2 navigation launch control. In SLAM mode, slam_toolbox supplies map->odom;
// this launch starts navigation/costmap nodes without AMCL localization.
ipcMain.handle("nav2:start", async (event, options = {}) => {
  return startNav2Process(options);
});

ipcMain.handle("nav2:stop", async () => {
  const stopped = stopNav2Process();
  return { running: false, stopped };
});

ipcMain.handle("nav2:status", async () => {
  return { running: !!nav2Process, output: nav2LastOutput, options: nav2Options };
});

// ROS2 bag playback. Assumes the app was launched with ROS2 environment,
// or ROS_DISTRO points to /opt/ros/<distro>/setup.bash.
ipcMain.handle("rosbag:play", async (event, options = {}) => {
  return startBagProcess(options);
});

ipcMain.handle("rosbag:info", async (event, bagPath) => {
  return readBagInfo(bagPath);
});

ipcMain.handle("rosbag:stop", async () => {
  const stopped = stopBagProcess();
  return { running: false, stopped };
});

ipcMain.handle("rosbag:pause", async () => {
  if (!bagProcess || bagPaused) return { running: !!bagProcess, paused: bagPaused, offset: currentBagOffset() };
  try {
    process.kill(-bagProcess.pid, "SIGSTOP");
    bagPaused = true;
    bagPausedAtMs = Date.now();
  } catch (e) {
    throw new Error(`pause failed: ${e.message}`);
  }
  return { running: true, paused: true, offset: currentBagOffset() };
});

ipcMain.handle("rosbag:resume", async () => {
  if (!bagProcess || !bagPaused) return { running: !!bagProcess, paused: bagPaused, offset: currentBagOffset() };
  try {
    process.kill(-bagProcess.pid, "SIGCONT");
    bagPausedTotalMs += Date.now() - bagPausedAtMs;
    bagPaused = false;
    bagPausedAtMs = 0;
  } catch (e) {
    throw new Error(`resume failed: ${e.message}`);
  }
  return { running: true, paused: false, offset: currentBagOffset() };
});

ipcMain.handle("rosbag:seek", async (event, options = {}) => {
  const base = bagOptions || options;
  if (!base?.path) throw new Error("bag path is required");
  const requested = options.offset != null
    ? Number(options.offset)
    : currentBagOffset() + Number(options.delta || 0);
  const offset = Math.max(0, requested || 0);
  return startBagProcess({ ...base, ...options, path: base.path, startOffset: offset });
});

ipcMain.handle("rosbag:status", async () => {
  return { running: !!bagProcess, paused: bagPaused, finished: bagFinished, offset: currentBagOffset(), duration: bagOptions?.duration || 0, output: bagLastOutput };
});

ipcMain.handle("rosbagRecord:start", async (event, options = {}) => {
  return startBagRecordProcess(options);
});

ipcMain.handle("rosbagRecord:stop", async () => {
  const stopped = stopBagRecordProcess();
  return { running: false, stopped, options: bagRecordOptions };
});

ipcMain.handle("rosbagRecord:status", async () => {
  return { running: !!bagRecordProcess, output: bagRecordLastOutput, options: bagRecordOptions };
});
