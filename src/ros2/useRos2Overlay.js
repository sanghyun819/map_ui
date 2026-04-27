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
