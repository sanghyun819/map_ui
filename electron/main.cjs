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
let bagProcess = null;
let bagLastOutput = "";
let bagOptions = null;
let bagFinished = false;
let bagStartedAtMs = 0;
let bagPaused = false;
let bagPausedAtMs = 0;
let bagPausedTotalMs = 0;

function appendBagOutput(data) {
  bagLastOutput = (bagLastOutput + data.toString()).slice(-4000);
}

function appendRosbridgeOutput(data) {
  rosbridgeLastOutput = (rosbridgeLastOutput + data.toString()).slice(-4000);
}

function appendSlamOutput(data) {
  slamLastOutput = (slamLastOutput + data.toString()).slice(-4000);
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
  stopBagProcess();
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
