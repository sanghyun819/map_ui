const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("electronAPI", {
  // File dialogs
  openFileDialog: (options) => ipcRenderer.invoke("dialog:openFile", options),
  saveFileDialog: (options) => ipcRenderer.invoke("dialog:saveFile", options),

  // File system
  readFile: (filePath, encoding) =>
    ipcRenderer.invoke("fs:readFile", filePath, encoding),
  writeFile: (filePath, data, encoding) =>
    ipcRenderer.invoke("fs:writeFile", filePath, data, encoding),
  copySemanticToThor: (options) =>
    ipcRenderer.invoke("thor:copySemantic", options),
  listThorMdFiles: (options) =>
    ipcRenderer.invoke("thor:listMdFiles", options),
  listThorMdDir: (options) =>
    ipcRenderer.invoke("thor:listMdDir", options),
  readThorMdFile: (options) =>
    ipcRenderer.invoke("thor:readMdFile", options),
  readDir: (dirPath) => ipcRenderer.invoke("fs:readDir", dirPath),
  updateLaunchMap: (options) => ipcRenderer.invoke("workspace:updateLaunchMap", options),
  buildWorkspace: (options) => ipcRenderer.invoke("workspace:build", options),

  // rosbridge websocket
  rosbridgeStart: (options) => ipcRenderer.invoke("rosbridge:start", options),
  rosbridgeStop: () => ipcRenderer.invoke("rosbridge:stop"),
  rosbridgeStatus: () => ipcRenderer.invoke("rosbridge:status"),

  // slam_toolbox
  slamStart: (options) => ipcRenderer.invoke("slam:start", options),
  slamStop: () => ipcRenderer.invoke("slam:stop"),
  slamStatus: () => ipcRenderer.invoke("slam:status"),

  // Nav2 navigation/costmap stack
  nav2Start: (options) => ipcRenderer.invoke("nav2:start", options),
  nav2Stop: () => ipcRenderer.invoke("nav2:stop"),
  nav2Status: () => ipcRenderer.invoke("nav2:status"),

  // ROS2 bag playback
  rosbagPlay: (options) => ipcRenderer.invoke("rosbag:play", options),
  rosbagInfo: (bagPath) => ipcRenderer.invoke("rosbag:info", bagPath),
  rosbagStop: () => ipcRenderer.invoke("rosbag:stop"),
  rosbagPause: () => ipcRenderer.invoke("rosbag:pause"),
  rosbagResume: () => ipcRenderer.invoke("rosbag:resume"),
  rosbagSeek: (options) => ipcRenderer.invoke("rosbag:seek", options),
  rosbagStatus: () => ipcRenderer.invoke("rosbag:status"),
  rosbagRecordStart: (options) => ipcRenderer.invoke("rosbagRecord:start", options),
  rosbagRecordStop: () => ipcRenderer.invoke("rosbagRecord:stop"),
  rosbagRecordStatus: () => ipcRenderer.invoke("rosbagRecord:status"),

  // Check if running in Electron
  isElectron: true,
});
