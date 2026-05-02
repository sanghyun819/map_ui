import { useRef, useEffect, useCallback } from "react";
import {
  laserScanToPoints,
  pointCloud2ToPoints,
  transformPoints,
  transformPoints3D,
  composeTransform3D,
  invertTransform3D,
  worldToPixel,
  tfToPose2D,
  tfToTransform3D,
} from "./Ros2Bridge.js";

// Compose two 2D poses: parent ∘ child
function composePose(a, b) {
  const cos = Math.cos(a.theta), sin = Math.sin(a.theta);
  return {
    x: a.x + b.x * cos - b.y * sin,
    y: a.y + b.x * sin + b.y * cos,
    theta: a.theta + b.theta,
  };
}

// Inverse of 2D pose transform: parent→child  =>  child→parent
function invertPose(p) {
  const c = Math.cos(p.theta), s = Math.sin(p.theta);
  return {
    x: -(p.x * c + p.y * s),
    y: p.x * s - p.y * c,
    theta: -p.theta,
  };
}

function isLidarDisplay(v = {}) {
  const typeSuffix = (v.type || "").split("/").pop();
  return v.viz === "lidar" || typeSuffix === "LaserScan" || typeSuffix === "PointCloud2";
}

function isPoseDisplay(v = {}) {
  const typeSuffix = (v.type || "").split("/").pop();
  return v.viz === "pose" || typeSuffix === "Odometry" || typeSuffix === "PoseStamped" || typeSuffix === "PoseWithCovarianceStamped";
}

function isPathDisplay(v = {}) {
  const typeSuffix = (v.type || "").split("/").pop();
  return v.viz === "path" || typeSuffix === "Path";
}

function isCostmapDisplay(v = {}) {
  return v.viz === "costmap";
}

function isFootprintDisplay(v = {}) {
  const typeSuffix = (v.type || "").split("/").pop();
  return v.viz === "footprint" || typeSuffix === "PolygonStamped";
}

function isRobotModelDisplay(v = {}) {
  return v.viz === "robot_model";
}

function isTfDisplay(v = {}) {
  const typeSuffix = (v.type || "").split("/").pop();
  return v.viz === "tf" || typeSuffix === "TFMessage";
}

function isCameraDisplay(v = {}) {
  const typeSuffix = (v.type || "").split("/").pop();
  return v.viz === "camera" || typeSuffix === "CompressedImage" || typeSuffix === "Image";
}

function clampDecay(v) {
  const d = Number(v);
  if (!Number.isFinite(d) || d < 0) return 0;
  return d;
}

function pruneLayerSamples(layer, decaySec, nowMs) {
  if (!layer) return [];
  const samples = layer.samples || [];
  if (samples.length === 0) return samples;

  if (decaySec <= 0) {
    const last = samples[samples.length - 1];
    layer.samples = last ? [last] : [];
    return layer.samples;
  }

  const limit = decaySec * 1000;
  layer.samples = samples.filter(s => nowMs - s.ts <= limit);
  if (layer.samples.length > 400) {
    layer.samples.splice(0, layer.samples.length - 400);
  }
  return layer.samples;
}

function flattenWorldSamples(samples, maxPoints = 250000) {
  const out = [];
  if (!samples || samples.length === 0) return out;

  for (const s of samples) {
    if (s?.world?.length) out.push(...s.world);
  }

  if (out.length <= maxPoints) return out;
  const step = Math.max(1, Math.floor(out.length / maxPoints));
  const reduced = [];
  for (let i = 0; i < out.length; i += step) reduced.push(out[i]);
  return reduced;
}

function bytesToBase64(bytes) {
  let bin = "";
  const chunk = 0x8000;
  for (let i = 0; i < bytes.length; i += chunk) {
    bin += String.fromCharCode(...bytes.subarray(i, i + chunk));
  }
  return btoa(bin);
}

function decodeRosBytes(data) {
  if (!data) return new Uint8Array();
  if (typeof data === "string") {
    const bin = atob(data);
    const out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }
  if (Array.isArray(data)) return Uint8Array.from(data);
  if (ArrayBuffer.isView(data)) return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
  if (data instanceof ArrayBuffer) return new Uint8Array(data);
  return new Uint8Array();
}

function compressedImageDataUrl(msg) {
  const bytes = decodeRosBytes(msg.data);
  if (!bytes.length) return null;
  const fmt = String(msg.format || "jpeg").toLowerCase();
  const mime = fmt.includes("png") ? "png" : fmt.includes("webp") ? "webp" : "jpeg";
  return `data:image/${mime};base64,${bytesToBase64(bytes)}`;
}

function rawImageDataUrl(msg) {
  const width = Number(msg.width), height = Number(msg.height);
  if (!width || !height) return null;
  const bytes = decodeRosBytes(msg.data);
  if (!bytes.length) return null;

  const encoding = String(msg.encoding || "rgb8").toLowerCase();
  const step = Number(msg.step) || width;
  const le = !msg.is_bigendian;
  const out = new ImageData(width, height);
  const data = out.data;

  if (encoding === "rgb8" || encoding === "bgr8" || encoding === "rgba8" || encoding === "bgra8") {
    const channels = encoding.includes("rgba") || encoding.includes("bgra") ? 4 : 3;
    const bgr = encoding.startsWith("bgr");
    for (let y = 0; y < height; y++) {
      const row = y * step;
      for (let x = 0; x < width; x++) {
        const src = row + x * channels;
        const dst = (y * width + x) * 4;
        const c0 = bytes[src] || 0, c1 = bytes[src + 1] || 0, c2 = bytes[src + 2] || 0;
        data[dst] = bgr ? c2 : c0;
        data[dst + 1] = c1;
        data[dst + 2] = bgr ? c0 : c2;
        data[dst + 3] = channels === 4 ? (bytes[src + 3] ?? 255) : 255;
      }
    }
  } else if (encoding === "mono8" || encoding === "8uc1") {
    for (let y = 0; y < height; y++) {
      const row = y * step;
      for (let x = 0; x < width; x++) {
        const v = bytes[row + x] || 0;
        const dst = (y * width + x) * 4;
        data[dst] = data[dst + 1] = data[dst + 2] = v;
        data[dst + 3] = 255;
      }
    }
  } else if (encoding === "16uc1" || encoding === "mono16" || encoding === "32fc1") {
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const floatDepth = encoding === "32fc1";
    const bpp = floatDepth ? 4 : 2;
    const vals = [];
    let min = Infinity, max = -Infinity;
    for (let y = 0; y < height; y++) {
      const row = y * step;
      for (let x = 0; x < width; x++) {
        const off = row + x * bpp;
        if (off + bpp > bytes.byteLength) { vals.push(0); continue; }
        const v = floatDepth ? view.getFloat32(off, le) : view.getUint16(off, le);
        const ok = Number.isFinite(v) && v > 0;
        vals.push(ok ? v : 0);
        if (ok) { min = Math.min(min, v); max = Math.max(max, v); }
      }
    }
    if (!Number.isFinite(min) || max <= min) { min = 0; max = floatDepth ? 5 : 5000; }
    for (let i = 0; i < vals.length; i++) {
      const v = vals[i];
      const norm = v > 0 ? Math.max(0, Math.min(1, (v - min) / (max - min))) : 0;
      const g = Math.round(norm * 255);
      const dst = i * 4;
      data[dst] = g;
      data[dst + 1] = Math.round(g * 0.9);
      data[dst + 2] = 255 - g;
      data[dst + 3] = v > 0 ? 255 : 80;
    }
  } else {
    return null;
  }

  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  canvas.getContext("2d").putImageData(out, 0, 0);
  return canvas.toDataURL("image/png");
}

function normalizeOccupancyValue(v) {
  const n = Number(v);
  if (!Number.isFinite(n)) return -1;
  return n > 127 ? n - 256 : n;
}

function decodeOccupancyValues(data) {
  if (!data) return [];
  if (typeof data === "string") {
    const bin = atob(data);
    const out = new Int16Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = normalizeOccupancyValue(bin.charCodeAt(i));
    return out;
  }
  if (Array.isArray(data)) return data.map(normalizeOccupancyValue);
  if (ArrayBuffer.isView(data)) {
    const out = new Int16Array(data.length);
    for (let i = 0; i < data.length; i++) out[i] = normalizeOccupancyValue(data[i]);
    return out;
  }
  if (data instanceof ArrayBuffer) {
    const bytes = new Uint8Array(data);
    const out = new Int16Array(bytes.length);
    for (let i = 0; i < bytes.length; i++) out[i] = normalizeOccupancyValue(bytes[i]);
    return out;
  }
  return [];
}

function hexToRgb(hex, fallback = [255, 80, 80]) {
  const m = String(hex || "").trim().match(/^#?([0-9a-f]{6})$/i);
  if (!m) return fallback;
  const n = parseInt(m[1], 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

function occupancyGridInfo(msg) {
  const info = msg?.info || {};
  const width = Math.max(0, Math.floor(Number(info.width) || 0));
  const height = Math.max(0, Math.floor(Number(info.height) || 0));
  const resolution = Math.max(1e-6, Number(info.resolution) || 0.05);
  const originPose = info.origin || {};
  const pos = originPose.position || {};
  const rot = originPose.orientation || {};
  const theta = Math.atan2(2 * ((rot.w ?? 1) * (rot.z ?? 0) + (rot.x ?? 0) * (rot.y ?? 0)), 1 - 2 * ((rot.y ?? 0) ** 2 + (rot.z ?? 0) ** 2));
  return {
    width,
    height,
    resolution,
    frameId: msg?.header?.frame_id?.replace(/^\//, "") || "map",
    originPose: { x: Number(pos.x) || 0, y: Number(pos.y) || 0, theta: Number.isFinite(theta) ? theta : 0 },
  };
}

function costmapImageCanvas(msg, color, alpha = 0.65) {
  const { width, height } = occupancyGridInfo(msg);
  if (!width || !height) return null;
  const values = decodeOccupancyValues(msg?.data);
  if (values.length < width * height) return null;
  const rgb = hexToRgb(color, [255, 80, 80]);
  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(width, height);
  for (let y = 0; y < height; y++) {
    const srcY = height - 1 - y;
    for (let x = 0; x < width; x++) {
      const cost = values[srcY * width + x];
      const dst = (y * width + x) * 4;
      if (cost <= 0) {
        img.data[dst + 3] = 0;
        continue;
      }
      const t = Math.max(0.05, Math.min(1, cost / 100));
      img.data[dst] = Math.round(255 * t + rgb[0] * (1 - t));
      img.data[dst + 1] = Math.round(rgb[1] * (1 - t) + 40 * t);
      img.data[dst + 2] = Math.round(rgb[2] * (1 - t) + 20 * t);
      img.data[dst + 3] = Math.round(255 * Math.min(1, alpha) * (0.25 + 0.75 * t));
    }
  }
  ctx.putImageData(img, 0, 0);
  return canvas;
}

function parseNumberList(value = "", fallback = []) {
  const nums = String(value).trim().split(/\s+/).map(Number).filter(Number.isFinite);
  return nums.length ? nums : fallback;
}

function directChild(el, tagName) {
  const want = tagName.toLowerCase();
  return Array.from(el?.children || []).find(c => c.tagName?.toLowerCase() === want) || null;
}

function originPoseFromElement(el) {
  const origin = directChild(el, "origin");
  const xyz = parseNumberList(origin?.getAttribute("xyz"), [0, 0, 0]);
  const rpy = parseNumberList(origin?.getAttribute("rpy"), [0, 0, 0]);
  return { x: xyz[0] || 0, y: xyz[1] || 0, theta: rpy[2] || 0 };
}

function circlePoints(radius, n = 28) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2;
    pts.push({ x: Math.cos(a) * radius, y: Math.sin(a) * radius });
  }
  return pts;
}

function parseUrdfShape(geometry) {
  const box = directChild(geometry, "box");
  if (box) {
    const size = parseNumberList(box.getAttribute("size"), [0.2, 0.2, 0.1]);
    const hx = Math.max(0.01, (size[0] || 0.2) / 2);
    const hy = Math.max(0.01, (size[1] || 0.2) / 2);
    return { points: [{ x: -hx, y: -hy }, { x: hx, y: -hy }, { x: hx, y: hy }, { x: -hx, y: hy }] };
  }

  const cylinder = directChild(geometry, "cylinder");
  if (cylinder) {
    const radius = Math.max(0.01, Number(cylinder.getAttribute("radius")) || 0.1);
    return { points: circlePoints(radius) };
  }

  const sphere = directChild(geometry, "sphere");
  if (sphere) {
    const radius = Math.max(0.01, Number(sphere.getAttribute("radius")) || 0.1);
    return { points: circlePoints(radius) };
  }

  const mesh = directChild(geometry, "mesh");
  if (mesh) {
    const scale = parseNumberList(mesh.getAttribute("scale"), [0.18, 0.18, 0.18]);
    const hx = Math.max(0.04, (scale[0] || 0.18) / 2);
    const hy = Math.max(0.04, (scale[1] || 0.18) / 2);
    return { points: [{ x: 0, y: -hy }, { x: hx, y: 0 }, { x: 0, y: hy }, { x: -hx, y: 0 }] };
  }

  return null;
}

function parseRobotDescription(xmlText) {
  const text = String(xmlText || "").trim();
  if (!text) return null;
  const doc = new DOMParser().parseFromString(text, "application/xml");
  if (doc.querySelector("parsererror")) throw new Error("robot_description XML parse failed");

  const robot = doc.documentElement;
  const links = [];
  for (const linkEl of Array.from(doc.getElementsByTagName("link"))) {
    const name = linkEl.getAttribute("name");
    if (!name) continue;
    const geometryParents = Array.from(linkEl.children || [])
      .filter(c => ["collision", "visual"].includes(c.tagName?.toLowerCase()));
    geometryParents.sort((a, b) => (a.tagName.toLowerCase() === "collision" ? -1 : 1) - (b.tagName.toLowerCase() === "collision" ? -1 : 1));

    const shapes = [];
    for (const parent of geometryParents) {
      const geometry = directChild(parent, "geometry");
      const shape = parseUrdfShape(geometry);
      if (!shape) continue;
      shapes.push({ ...shape, origin: originPoseFromElement(parent) });
      if (shapes.length >= 2) break;
    }
    if (shapes.length) links.push({ name, shapes });
  }
  return { name: robot?.getAttribute("name") || "robot", links };
}

function transformLocalPolygon(points, pose) {
  const cos = Math.cos(pose.theta || 0);
  const sin = Math.sin(pose.theta || 0);
  return points.map(p => ({
    x: (pose.x || 0) + p.x * cos - p.y * sin,
    y: (pose.y || 0) + p.x * sin + p.y * cos,
  }));
}

/**
 * useRos2Overlay — rviz2-style visualization.
 *
 * Each message carries header.frame_id. The TF tree is used to resolve
 * fixed_frame → message_frame automatically, just like rviz2.
 *
 * @param {object} frames — { fixed: "map" } — only fixed frame needed
 */
export default function useRos2Overlay(bridge, vis, meta, canvasSize, requestDraw, frames, onFramesDiscovered) {
  const lidarRef = useRef([]);      // aggregated pixel points [{x,y}]
  const lidarWorldRef = useRef([]); // aggregated world points [{x,y,z}]
  const lidarLayersRef = useRef({}); // topic -> { samples: [{ts,pix,world}] }

  const poseLayersRef = useRef({});  // topic -> { pose:{x,y,theta}, ts }
  const pathLayersRef = useRef({});  // topic -> { pix:[{x,y}], world:[{x,y,z}], ts }
  const costmapLayersRef = useRef({}); // topic -> { image, pose, resolution, width, height, ts }
  const footprintLayersRef = useRef({}); // topic -> { pix:[{x,y}], world:[{x,y,z}], ts }
  const robotModelLayersRef = useRef({}); // topic -> parsed URDF model
  const pathRef = useRef([]);
  const pathWorldRef = useRef([]);
  const cameraRef = useRef(null);
  const unsubsRef = useRef([]);

  // TF tree
  const tfStore = useRef({});
  const tfStore3D = useRef({});
  const seenFrames = useRef(new Set());

  // Debug stats
  const statsRef = useRef({
    lidarCount: 0,
    lidarTopicCount: 0,
    poseTopicCount: 0,
    pathTopicCount: 0,
    costmapTopicCount: 0,
    footprintTopicCount: 0,
    robotModelLinkCount: 0,
    robotDescriptionLoaded: false,
    tfResolved: false,
    tfChain: "",
    robotPose: null,
    lastError: "",
    resolvedFrames: {},
  });

  const fixedFrame = frames?.fixed || "map";

  // Convert world → pixel
  const w2p = useCallback((wx, wy) => {
    if (!meta || !canvasSize.h) return { x: 0, y: 0 };
    return worldToPixel(wx, wy, meta.origin, meta.resolution, canvasSize.h);
  }, [meta, canvasSize.h]);

  const processTfMessage = useCallback((msg) => {
    const tfs = msg?.transforms || [];
    let newFrameFound = false;
    for (const tf of tfs) {
      const child = tf.child_frame_id?.replace(/^\//, "");
      const parent = tf.header?.frame_id?.replace(/^\//, "");
      if (!child || !parent) continue;
      tfStore.current[`${parent}→${child}`] = tfToPose2D(tf.transform);
      tfStore3D.current[`${parent}→${child}`] = tfToTransform3D(tf.transform);
      if (!seenFrames.current.has(parent)) { seenFrames.current.add(parent); newFrameFound = true; }
      if (!seenFrames.current.has(child)) { seenFrames.current.add(child); newFrameFound = true; }
    }
    if (newFrameFound && onFramesDiscovered) onFramesDiscovered([...seenFrames.current].sort());
    requestDraw();
  }, [onFramesDiscovered, requestDraw]);

  /**
   * lookupTransform(targetFrame) — rviz2-style TF lookup.
   * Returns pose+chain of targetFrame in fixedFrame coordinates.
   * Uses BFS over both forward and inverse TF edges.
   */
  const lookupTransform = useCallback((targetFrame) => {
    if (!targetFrame) return null;
    const fixed = fixedFrame.replace(/^\//, "");
    const target = targetFrame.replace(/^\//, "");
    if (!fixed || !target) return null;
    if (target === fixed) return { x: 0, y: 0, theta: 0, chain: [fixed] };

    const store = tfStore.current;
    const edges = new Map(); // frame -> [{to, pose}]
    const addEdge = (from, to, pose) => {
      if (!edges.has(from)) edges.set(from, []);
      edges.get(from).push({ to, pose });
    };

    for (const [key, pose] of Object.entries(store)) {
      const [parent, child] = key.split("→");
      if (!parent || !child || !pose) continue;
      addEdge(parent, child, pose);
      addEdge(child, parent, invertPose(pose));
    }

    const q = [{ frame: fixed, pose: { x: 0, y: 0, theta: 0 }, chain: [fixed] }];
    const visited = new Set([fixed]);

    for (let i = 0; i < q.length; i++) {
      const cur = q[i];
      const next = edges.get(cur.frame) || [];
      for (const e of next) {
        if (visited.has(e.to)) continue;
        const nextPose = composePose(cur.pose, e.pose);
        const nextChain = [...cur.chain, e.to];
        if (e.to === target) return { ...nextPose, chain: nextChain };
        visited.add(e.to);
        q.push({ frame: e.to, pose: nextPose, chain: nextChain });
      }
    }
    return null;
  }, [fixedFrame]);

  // 3D TF lookup for PointCloud2
  const lookupTransform3D = useCallback((targetFrame) => {
    if (!targetFrame) return null;
    const fixed = fixedFrame.replace(/^\//, "");
    const target = targetFrame.replace(/^\//, "");
    if (!fixed || !target) return null;
    if (target === fixed) {
      return {
        tf: { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 },
        chain: [fixed],
      };
    }

    const store = tfStore3D.current;
    const edges = new Map(); // frame -> [{to, tf}]
    const addEdge = (from, to, tf) => {
      if (!edges.has(from)) edges.set(from, []);
      edges.get(from).push({ to, tf });
    };

    for (const [key, tf] of Object.entries(store)) {
      const [parent, child] = key.split("→");
      if (!parent || !child || !tf) continue;
      addEdge(parent, child, tf);
      addEdge(child, parent, invertTransform3D(tf));
    }

    const q = [{
      frame: fixed,
      tf: { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 },
      chain: [fixed],
    }];
    const visited = new Set([fixed]);

    for (let i = 0; i < q.length; i++) {
      const cur = q[i];
      const next = edges.get(cur.frame) || [];
      for (const e of next) {
        if (visited.has(e.to)) continue;
        const nextTf = composeTransform3D(cur.tf, e.tf);
        const nextChain = [...cur.chain, e.to];
        if (e.to === target) return { tf: nextTf, chain: nextChain };
        visited.add(e.to);
        q.push({ frame: e.to, tf: nextTf, chain: nextChain });
      }
    }
    return null;
  }, [fixedFrame]);

  // Subscribe/unsubscribe
  useEffect(() => {
    unsubsRef.current.forEach(fn => fn());
    unsubsRef.current = [];

    if (!bridge.connected) {
      tfStore.current = {};
      tfStore3D.current = {};
      seenFrames.current = new Set();
      lidarLayersRef.current = {};
      poseLayersRef.current = {};
      pathLayersRef.current = {};
      costmapLayersRef.current = {};
      footprintLayersRef.current = {};
      robotModelLayersRef.current = {};
      lidarRef.current = [];
      lidarWorldRef.current = [];
      pathRef.current = [];
      pathWorldRef.current = [];
      cameraRef.current = null;
      statsRef.current.tfResolved = false;
      statsRef.current.tfChain = "";
      statsRef.current.robotPose = null;
      statsRef.current.lidarCount = 0;
      statsRef.current.lidarTopicCount = 0;
      statsRef.current.poseTopicCount = 0;
      statsRef.current.pathTopicCount = 0;
      statsRef.current.costmapTopicCount = 0;
      statsRef.current.footprintTopicCount = 0;
      statsRef.current.robotModelLinkCount = 0;
      statsRef.current.robotDescriptionLoaded = false;
      return;
    }

    // Always subscribe TF like rviz2
    const tfTopicsAuto = new Set(["/tf", "/tf_static"]);
    for (const tfTopic of tfTopicsAuto) {
      const throttle = tfTopic === "/tf" ? 50 : 0;
      const opts = tfTopic === "/tf_static"
        ? { qos: { reliability: "reliable", durability: "transient_local", history: "keep_last", depth: 1 } }
        : {};
      const unsub = bridge.subscribe(tfTopic, "tf2_msgs/msg/TFMessage", processTfMessage, throttle, opts);
      unsubsRef.current.push(unsub);
    }

    const fixed = fixedFrame.replace(/^\//, "");
    if (fixed && !seenFrames.current.has(fixed)) {
      seenFrames.current.add(fixed);
      if (onFramesDiscovered) onFramesDiscovered([...seenFrames.current].sort());
    }

    const activeLidarTopics = new Set();
    const activePoseTopics = new Set();
    const activePathTopics = new Set();
    const activeCostmapTopics = new Set();
    const activeFootprintTopics = new Set();
    const activeRobotModelTopics = new Set();
    const activeCameraTopics = new Set();

    for (const [topic, v] of Object.entries(vis)) {
      if (v.enabled === false) continue;
      const typeSuffix = (v.type || "").split("/").pop();

      // ── LaserScan / PointCloud2 ──
      if (isLidarDisplay(v)) {
        activeLidarTopics.add(topic);
        const isPC2 = typeSuffix === "PointCloud2" || (v.type || "").includes("PointCloud2");
        const unsub = bridge.subscribe(topic, v.type, (msg) => {
          try {
            // Parse points in sensor frame
            let pts;
            if (isPC2) {
              pts = pointCloud2ToPoints(msg, v.maxPoints || 30000);
            } else {
              let scanMsg = msg;
              if (typeof msg.ranges === "string") {
                const bin = atob(msg.ranges);
                const arr = new Uint8Array(bin.length);
                for (let i = 0; i < bin.length; i++) arr[i] = bin.charCodeAt(i);
                scanMsg = { ...msg, ranges: Array.from(new Float32Array(arr.buffer)) };
              }
              pts = laserScanToPoints(scanMsg);
            }

            // rviz2 style: lookup fixed_frame → sensor_frame
            const frameId = msg.header?.frame_id?.replace(/^\//, "");
            let pixPts = [];
            let worldPts = [];

            if (isPC2) {
              const tf3 = lookupTransform3D(frameId);
              if (tf3) {
                const world3 = transformPoints3D(pts, tf3.tf);
                pixPts = world3.map(p => w2p(p.x, p.y));
                worldPts = world3;
                statsRef.current.tfResolved = true;
                statsRef.current.tfChain = tf3.chain.join("→");
              } else {
                // For 3D clouds, hide unresolved data (raw display is misleading)
                statsRef.current.tfResolved = false;
                statsRef.current.tfChain = `${fixedFrame}→???→${frameId || "?"}`;
              }
            } else {
              const tfInfo = lookupTransform(frameId);
              if (tfInfo) {
                const { chain, ...tf } = tfInfo;
                const world = transformPoints(pts, tf);
                pixPts = world.map(p => w2p(p.x, p.y));
                worldPts = world.map(p => ({ x: p.x, y: p.y, z: 0 }));
                statsRef.current.tfResolved = true;
                statsRef.current.tfChain = chain.join("→");
              } else {
                // No TF yet — show raw points at origin (like rviz2 warning)
                pixPts = pts.map(p => w2p(p.x, p.y));
                worldPts = pts.map(p => ({ x: p.x, y: p.y, z: 0 }));
                statsRef.current.tfResolved = false;
                statsRef.current.tfChain = `${fixedFrame}→???→${frameId || "?"}`;
              }
            }

            const now = Date.now();
            const decaySec = clampDecay(v.decay);
            const layer = lidarLayersRef.current[topic] || { samples: [] };
            layer.samples.push({ ts: now, pix: pixPts, world: worldPts });
            pruneLayerSamples(layer, decaySec, now);
            lidarLayersRef.current[topic] = layer;

            statsRef.current.lidarCount = pixPts.length;
            statsRef.current.lastError = "";
            requestDraw();
          } catch (e) {
            statsRef.current.lastError = `lidar: ${e.message}`;
            console.warn("[ROS2] Lidar parse error:", e);
          }
        }, 100);
        unsubsRef.current.push(unsub);
      }

      // ── Odometry / PoseStamped ──
      else if (isPoseDisplay(v)) {
        activePoseTopics.add(topic);
        const unsub = bridge.subscribe(topic, v.type, (msg) => {
          const pose = msg.pose?.pose || msg.pose || msg;
          const t = pose.position || pose.translation || { x: 0, y: 0 };
          const r = pose.orientation || pose.rotation || { x: 0, y: 0, z: 0, w: 1 };
          const localPose = { x: t.x, y: t.y, theta: Math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z)) };

          // rviz2: transform into fixed_frame
          const frameId = msg.header?.frame_id?.replace(/^\//, "");
          const tfInfo = lookupTransform(frameId);
          const worldPose = tfInfo ? composePose(tfInfo, localPose) : localPose;

          poseLayersRef.current[topic] = { pose: worldPose, ts: Date.now() };
          statsRef.current.robotPose = worldPose;
          requestDraw();
        }, 50);
        unsubsRef.current.push(unsub);
      }

      // ── TFMessage ──
      else if (isTfDisplay(v)) {
        // Avoid duplicate subscriptions for default TF topics.
        if (tfTopicsAuto.has(topic)) continue;
        const unsub = bridge.subscribe(topic, v.type, processTfMessage, 50);
        unsubsRef.current.push(unsub);
      }

      // ── Path ──
      else if (isPathDisplay(v)) {
        activePathTopics.add(topic);
        const unsub = bridge.subscribe(topic, v.type, (msg) => {
          const frameId = msg.header?.frame_id?.replace(/^\//, "");
          const tfInfo = lookupTransform(frameId);
          const worldPath = [];
          const pixPath = (msg.poses || []).map(p => {
            const pos = p.pose?.position || p.position || { x: 0, y: 0 };
            // Each pose in path is already in the path's frame
            const wp = tfInfo ? {
              x: tfInfo.x + pos.x * Math.cos(tfInfo.theta) - pos.y * Math.sin(tfInfo.theta),
              y: tfInfo.y + pos.x * Math.sin(tfInfo.theta) + pos.y * Math.cos(tfInfo.theta),
            } : pos;
            worldPath.push({ x: wp.x, y: wp.y, z: 0 });
            return w2p(wp.x, wp.y);
          });
          pathLayersRef.current[topic] = { pix: pixPath, world: worldPath, ts: Date.now() };
          requestDraw();
        }, 200);
        unsubsRef.current.push(unsub);
      }

      // ── Nav2 costmap overlay ──
      else if (isCostmapDisplay(v)) {
        activeCostmapTopics.add(topic);
        const unsub = bridge.subscribe(topic, v.type || "nav_msgs/msg/OccupancyGrid", (msg) => {
          try {
            const info = occupancyGridInfo(msg);
            if (!info.width || !info.height) return;
            const frameTf = lookupTransform(info.frameId);
            if (!frameTf) {
              statsRef.current.tfResolved = false;
              statsRef.current.tfChain = `${fixedFrame}→???→${info.frameId || "?"}`;
              return;
            }
            const image = costmapImageCanvas(msg, v.color || "#ff6680", v.alpha ?? 0.65);
            if (!image) return;
            costmapLayersRef.current[topic] = {
              image,
              pose: composePose(frameTf, info.originPose),
              resolution: info.resolution,
              width: info.width,
              height: info.height,
              frameId: info.frameId,
              ts: Date.now(),
            };
            requestDraw();
          } catch (e) {
            statsRef.current.lastError = `costmap: ${e.message}`;
          }
        }, 250, { queue_length: 1 });
        unsubsRef.current.push(unsub);
      }

      // ── Footprint polygon ──
      else if (isFootprintDisplay(v)) {
        activeFootprintTopics.add(topic);
        const unsub = bridge.subscribe(topic, v.type, (msg) => {
          try {
            const frameId = msg.header?.frame_id?.replace(/^\//, "");
            const raw = msg.polygon?.points || msg.points || [];
            const local = raw.map(p => ({ x: Number(p.x) || 0, y: Number(p.y) || 0 })).filter(p => Number.isFinite(p.x) && Number.isFinite(p.y));
            if (local.length < 2) return;
            const tfInfo = lookupTransform(frameId);
            const world2 = tfInfo ? transformPoints(local, tfInfo) : local;
            footprintLayersRef.current[topic] = {
              pix: world2.map(p => w2p(p.x, p.y)),
              world: world2.map(p => ({ x: p.x, y: p.y, z: 0.035 })),
              frameId,
              ts: Date.now(),
            };
            requestDraw();
          } catch (e) {
            statsRef.current.lastError = `footprint: ${e.message}`;
          }
        }, 100);
        unsubsRef.current.push(unsub);
      }

      // ── RobotModel / robot_description URDF ──
      else if (isRobotModelDisplay(v)) {
        activeRobotModelTopics.add(topic);
        const unsub = bridge.subscribe(topic, v.type, (msg) => {
          try {
            const xml = typeof msg === "string" ? msg : msg?.data;
            const model = parseRobotDescription(xml);
            if (!model) return;
            robotModelLayersRef.current[topic] = { ...model, ts: Date.now() };
            statsRef.current.robotDescriptionLoaded = true;
            requestDraw();
          } catch (e) {
            statsRef.current.lastError = `robot_description: ${e.message}`;
          }
        }, 0, { qos: { reliability: "reliable", durability: "transient_local", history: "keep_last", depth: 1 } });
        unsubsRef.current.push(unsub);
      }

      // ── Camera / depth image preview ──
      else if (isCameraDisplay(v)) {
        activeCameraTopics.add(topic);
        const unsub = bridge.subscribe(topic, v.type, (msg) => {
          try {
            const url = typeSuffix === "CompressedImage"
              ? compressedImageDataUrl(msg)
              : rawImageDataUrl(msg);
            if (url) cameraRef.current = { url, topic, encoding: msg.encoding || msg.format || typeSuffix };
            requestDraw();
          } catch (e) {
            statsRef.current.lastError = `image: ${e.message}`;
          }
        }, 200);
        unsubsRef.current.push(unsub);
      }

      // ── Raw/unsupported topic monitor ──
      else {
        const unsub = bridge.subscribe(topic, v.type, () => {
          statsRef.current.lastError = "";
        }, 500);
        unsubsRef.current.push(unsub);
      }
    }

    // Clean removed lidar topics from buffer
    for (const topic of Object.keys(lidarLayersRef.current)) {
      if (!activeLidarTopics.has(topic)) delete lidarLayersRef.current[topic];
    }
    for (const topic of Object.keys(poseLayersRef.current)) {
      if (!activePoseTopics.has(topic)) delete poseLayersRef.current[topic];
    }
    for (const topic of Object.keys(pathLayersRef.current)) {
      if (!activePathTopics.has(topic)) delete pathLayersRef.current[topic];
    }
    for (const topic of Object.keys(costmapLayersRef.current)) {
      if (!activeCostmapTopics.has(topic)) delete costmapLayersRef.current[topic];
    }
    for (const topic of Object.keys(footprintLayersRef.current)) {
      if (!activeFootprintTopics.has(topic)) delete footprintLayersRef.current[topic];
    }
    for (const topic of Object.keys(robotModelLayersRef.current)) {
      if (!activeRobotModelTopics.has(topic)) delete robotModelLayersRef.current[topic];
    }
    if (cameraRef.current && !activeCameraTopics.has(cameraRef.current.topic)) {
      cameraRef.current = null;
    }
    requestDraw();

    return () => {
      unsubsRef.current.forEach(fn => fn());
      unsubsRef.current = [];
    };
  }, [bridge, bridge.state, vis, w2p, requestDraw, lookupTransform, lookupTransform3D, fixedFrame, onFramesDiscovered, processTfMessage]);

  // ── Draw ──
  const drawRos2 = useCallback((ctx) => {
    const visEntries = Object.entries(vis);
    if (visEntries.length === 0) {
      lidarRef.current = [];
      lidarWorldRef.current = [];
      pathRef.current = [];
      pathWorldRef.current = [];
      statsRef.current.lidarCount = 0;
      statsRef.current.lidarTopicCount = 0;
      statsRef.current.poseTopicCount = 0;
      statsRef.current.pathTopicCount = 0;
      return;
    }

    const drawPoseGlyph = (pose, cfg, label = "") => {
      if (!pose) return;
      const pp = w2p(pose.x, pose.y);
      const poseColor = cfg.color;
      const poseSize = cfg.size;
      const poseAlpha = cfg.alpha ?? 1.0;

      ctx.save();
      ctx.globalAlpha = poseAlpha;
      ctx.translate(pp.x + 0.5, pp.y + 0.5);

      ctx.shadowColor = poseColor;
      ctx.shadowBlur = 8;
      ctx.beginPath();
      ctx.arc(0, 0, poseSize, 0, Math.PI * 2);
      ctx.fillStyle = poseColor;
      ctx.fill();
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.shadowBlur = 0;

      const alen = poseSize * 2.4;
      const ax = Math.cos(-pose.theta) * alen;
      const ay = Math.sin(-pose.theta) * alen;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(ax, ay);
      ctx.strokeStyle = poseColor;
      ctx.lineWidth = 2;
      ctx.stroke();

      const ang = Math.atan2(ay, ax);
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(ax - 5 * Math.cos(ang - 0.5), ay - 5 * Math.sin(ang - 0.5));
      ctx.lineTo(ax - 5 * Math.cos(ang + 0.5), ay - 5 * Math.sin(ang + 0.5));
      ctx.closePath();
      ctx.fillStyle = poseColor;
      ctx.fill();

      ctx.font = "bold 7px monospace";
      ctx.fillStyle = poseColor;
      ctx.textAlign = "left";
      ctx.textBaseline = "bottom";
      ctx.fillText(`(${pose.x.toFixed(1)}, ${pose.y.toFixed(1)})`, 8, -4);
      if (label) {
        ctx.font = "bold 6px monospace";
        ctx.fillText(label, 8, 8);
      }
      ctx.restore();
    };

    // Fallback style when pose topics are unavailable
    let fallbackPoseCfg = { color: "#00e676", size: 5, alpha: 1.0 };
    const hasEnabledTfDisplay = visEntries.some(([, v]) => v.enabled !== false && isTfDisplay(v));
    for (const [, v] of visEntries) {
      if (v.enabled === false) continue;
      if (v.viz === "pose" || v.viz === "tf") {
        fallbackPoseCfg = { color: v.color, size: v.size ?? 5, alpha: v.alpha ?? 1.0 };
      }
    }

    // ── Lidar points (per-topic, multi-display, decay) ──
    const now = Date.now();
    let totalLidarCount = 0;
    let lidarTopicCount = 0;
    const aggregatedPix = [];
    const aggregatedWorldSamples = [];

    for (const [topic, v] of visEntries) {
      if (v.enabled === false || !isLidarDisplay(v)) continue;
      const layer = lidarLayersRef.current[topic];
      if (!layer) continue;

      const decaySec = clampDecay(v.decay);
      const samples = pruneLayerSamples(layer, decaySec, now);
      if (samples.length === 0) continue;
      lidarTopicCount += 1;

      const sz = v.size ?? 2;
      const half = sz / 2;
      const baseAlpha = v.alpha ?? 0.8;

      ctx.save();
      ctx.fillStyle = v.color || "#ff3333";

      for (const s of samples) {
        const ageMs = now - s.ts;
        const ageFactor = decaySec > 0 ? Math.max(0, 1 - ageMs / (decaySec * 1000)) : 1;
        const a = Math.max(0.05, baseAlpha * ageFactor);
        ctx.globalAlpha = a;

        const pix = s.pix || [];
        for (let i = 0; i < pix.length; i++) {
          ctx.fillRect(pix[i].x - half, pix[i].y - half, sz, sz);
        }
        totalLidarCount += pix.length;
        aggregatedPix.push(...pix);
        aggregatedWorldSamples.push(s);
      }

      ctx.restore();
    }

    statsRef.current.lidarCount = totalLidarCount;
    statsRef.current.lidarTopicCount = lidarTopicCount;
    lidarRef.current = aggregatedPix;
    lidarWorldRef.current = flattenWorldSamples(aggregatedWorldSamples, 250000);

    // ── Path (multi-topic) ──
    let primaryPath = null;
    let pathTopicCount = 0;
    for (const [topic, v] of visEntries) {
      if (v.enabled === false || !isPathDisplay(v)) continue;
      const layer = pathLayersRef.current[topic];
      const path = layer?.pix || [];
      if (path.length <= 1) continue;
      if (!primaryPath) primaryPath = layer;
      pathTopicCount += 1;

      ctx.save();
      ctx.strokeStyle = v.color || "#ffaa00";
      ctx.lineWidth = v.size ?? 1.5;
      ctx.setLineDash([4, 3]);
      ctx.globalAlpha = v.alpha ?? 0.6;
      ctx.beginPath();
      ctx.moveTo(path[0].x, path[0].y);
      for (let i = 1; i < path.length; i++) ctx.lineTo(path[i].x, path[i].y);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.restore();
    }
    pathRef.current = primaryPath?.pix || [];
    pathWorldRef.current = primaryPath?.world || [];
    statsRef.current.pathTopicCount = pathTopicCount;

    // ── Costmaps, drawn as translucent occupancy grid overlays ──
    let costmapTopicCount = 0;
    for (const [topic, v] of visEntries) {
      if (v.enabled === false || !isCostmapDisplay(v)) continue;
      const layer = costmapLayersRef.current[topic];
      if (!layer?.image) continue;
      const { pose, resolution, width, height, image } = layer;
      const origin = w2p(pose.x, pose.y);
      const xEnd = w2p(pose.x + Math.cos(pose.theta) * width * resolution, pose.y + Math.sin(pose.theta) * width * resolution);
      const yEnd = w2p(pose.x - Math.sin(pose.theta) * height * resolution, pose.y + Math.cos(pose.theta) * height * resolution);
      const xAxis = { x: (xEnd.x - origin.x) / width, y: (xEnd.y - origin.y) / width };
      const yAxis = { x: (yEnd.x - origin.x) / height, y: (yEnd.y - origin.y) / height };

      ctx.save();
      ctx.globalAlpha = 1;
      ctx.translate(origin.x + yAxis.x * height, origin.y + yAxis.y * height);
      ctx.transform(xAxis.x, xAxis.y, -yAxis.x, -yAxis.y, 0, 0);
      ctx.drawImage(image, 0, 0);
      ctx.restore();
      costmapTopicCount += 1;
    }
    statsRef.current.costmapTopicCount = costmapTopicCount;

    // ── Footprint polygons ──
    let footprintTopicCount = 0;
    for (const [topic, v] of visEntries) {
      if (v.enabled === false || !isFootprintDisplay(v)) continue;
      const layer = footprintLayersRef.current[topic];
      const poly = layer?.pix || [];
      if (poly.length < 2) continue;
      footprintTopicCount += 1;

      ctx.save();
      ctx.globalAlpha = v.alpha ?? 0.85;
      ctx.strokeStyle = v.color || "#00bcd4";
      ctx.fillStyle = v.color || "#00bcd4";
      ctx.lineWidth = v.size ?? 2;
      ctx.beginPath();
      ctx.moveTo(poly[0].x, poly[0].y);
      for (let i = 1; i < poly.length; i++) ctx.lineTo(poly[i].x, poly[i].y);
      ctx.closePath();
      ctx.globalAlpha = Math.min(0.22, (v.alpha ?? 0.85) * 0.25);
      ctx.fill();
      ctx.globalAlpha = v.alpha ?? 0.85;
      ctx.stroke();
      ctx.restore();
    }
    statsRef.current.footprintTopicCount = footprintTopicCount;

    // ── Robot description model, projected to map from per-link TF ──
    let robotModelLinkCount = 0;
    let robotDescriptionLoaded = false;
    for (const [topic, v] of visEntries) {
      if (v.enabled === false || !isRobotModelDisplay(v)) continue;
      const model = robotModelLayersRef.current[topic];
      if (!model) continue;
      robotDescriptionLoaded = true;

      ctx.save();
      ctx.strokeStyle = v.color || "#b5f5ff";
      ctx.fillStyle = v.color || "#b5f5ff";
      ctx.lineWidth = v.size ?? 1.5;
      ctx.globalAlpha = v.alpha ?? 0.9;

      for (const link of model.links || []) {
        const linkTf = lookupTransform(link.name);
        if (!linkTf) continue;
        let linkDrawn = false;
        for (const shape of link.shapes || []) {
          const pose = composePose(linkTf, shape.origin || { x: 0, y: 0, theta: 0 });
          const worldPoly = transformLocalPolygon(shape.points || [], pose);
          const pix = worldPoly.map(p => w2p(p.x, p.y));
          if (pix.length < 2) continue;
          ctx.beginPath();
          ctx.moveTo(pix[0].x, pix[0].y);
          for (let i = 1; i < pix.length; i++) ctx.lineTo(pix[i].x, pix[i].y);
          ctx.closePath();
          ctx.globalAlpha = Math.min(0.18, (v.alpha ?? 0.9) * 0.2);
          ctx.fill();
          ctx.globalAlpha = v.alpha ?? 0.9;
          ctx.stroke();
          linkDrawn = true;
        }
        if (linkDrawn) robotModelLinkCount += 1;
      }
      ctx.restore();
    }
    statsRef.current.robotModelLinkCount = robotModelLinkCount;
    statsRef.current.robotDescriptionLoaded = robotDescriptionLoaded;

    // ── Pose (multi-topic) ──
    let primaryPose = null;
    let poseTopicCount = 0;
    for (const [topic, v] of visEntries) {
      if (v.enabled === false || !isPoseDisplay(v)) continue;
      const layer = poseLayersRef.current[topic];
      const pose = layer?.pose;
      if (!pose) continue;

      const cfg = { color: v.color || "#00e676", size: v.size ?? 5, alpha: v.alpha ?? 1.0 };
      const shortTopic = topic.split("/").filter(Boolean).pop() || topic || "pose";
      drawPoseGlyph(pose, cfg, shortTopic);

      if (!primaryPose) primaryPose = pose;
      poseTopicCount += 1;
    }
    statsRef.current.poseTopicCount = poseTopicCount;

    if (primaryPose) statsRef.current.robotPose = primaryPose;
    else statsRef.current.robotPose = null;

    // ── TF tree: also compute & display robot pose from TF ──
    if (!primaryPose) {
      for (const frameName of seenFrames.current) {
        if (!frameName.includes("base")) continue;
        const tf = lookupTransform(frameName);
        if (!tf) continue;

        statsRef.current.robotPose = tf;
        statsRef.current.tfResolved = true;
        if (hasEnabledTfDisplay) drawPoseGlyph(tf, fallbackPoseCfg, frameName);
        break;
      }
    }
  }, [vis, w2p, lookupTransform]);

  return {
    drawRos2,
    lidarPoints: lidarRef,
    lidarWorldPoints: lidarWorldRef,
    cameraDataUrl: cameraRef,
    pathPoints: pathRef,
    pathWorldPoints: pathWorldRef,
    stats: statsRef,
  };
}
