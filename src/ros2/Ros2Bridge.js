/**
 * Ros2Bridge — Lightweight rosbridge WebSocket client
 * Connects to rosbridge_websocket (default ws://localhost:9090)
 * No external dependencies — uses native WebSocket + rosbridge JSON protocol
 */

const STATES = { DISCONNECTED: 0, CONNECTING: 1, CONNECTED: 2 };

export default class Ros2Bridge {
  constructor() {
    this.ws = null;
    this.state = STATES.DISCONNECTED;
    this.url = "ws://localhost:9090";
    this._subs = {};           // topic → {type, throttle_rate, options, cbs: Set<fn>}
    this._svcId = 0;
    this._svcPending = {};     // id → {resolve, reject}
    this._onState = new Set(); // state-change listeners
    this._reconnectTimer = null;
    this._autoReconnect = true;
  }

  /* ── Connection ────────────────────────────────────────────── */

  connect(url) {
    if (url) this.url = url;
    if (this.ws) this.disconnect();
    this._setState(STATES.CONNECTING);

    try {
      this.ws = new WebSocket(this.url);
    } catch (e) {
      this._setState(STATES.DISCONNECTED);
      return;
    }

    this.ws.onopen = () => {
      this._setState(STATES.CONNECTED);
      clearTimeout(this._reconnectTimer);
      // Re-subscribe all existing topics
      for (const [topic, sub] of Object.entries(this._subs)) {
        this._sendSubscribe(topic, sub.type, sub.throttle_rate, sub.options);
      }
    };

    this.ws.onmessage = (ev) => {
      try {
        const data = JSON.parse(ev.data);
        this._handleMessage(data);
      } catch (e) { /* ignore parse errors */ }
    };

    this.ws.onclose = () => {
      this._setState(STATES.DISCONNECTED);
      this.ws = null;
      if (this._autoReconnect) {
        this._reconnectTimer = setTimeout(() => this.connect(), 3000);
      }
    };

    this.ws.onerror = () => {
      // onclose will fire after this
    };
  }

  disconnect() {
    this._autoReconnect = false;
    clearTimeout(this._reconnectTimer);
    if (this.ws) {
      this.ws.onclose = null;
      this.ws.close();
      this.ws = null;
    }
    this._setState(STATES.DISCONNECTED);
  }

  get connected() { return this.state === STATES.CONNECTED; }

  onStateChange(fn) {
    this._onState.add(fn);
    return () => this._onState.delete(fn);
  }

  _setState(s) {
    if (this.state === s) return;
    this.state = s;
    this._onState.forEach(fn => fn(s));
  }

  _send(obj) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
    }
  }

  /* ── Topic Subscribe / Unsubscribe ─────────────────────────── */

  subscribe(topic, type, callback, throttle_rate = 0, options = {}) {
    if (!this._subs[topic]) {
      this._subs[topic] = { type, throttle_rate, options, cbs: new Set() };
      if (this.connected) this._sendSubscribe(topic, type, throttle_rate, options);
    }
    this._subs[topic].cbs.add(callback);

    // Return unsubscribe function
    return () => {
      const sub = this._subs[topic];
      if (!sub) return;
      sub.cbs.delete(callback);
      if (sub.cbs.size === 0) {
        this._sendUnsubscribe(topic);
        delete this._subs[topic];
      }
    };
  }

  _sendSubscribe(topic, type, throttle_rate = 0, options = {}) {
    const msg = { op: "subscribe", topic, type };
    if (throttle_rate > 0) msg.throttle_rate = throttle_rate;
    if (options?.qos) msg.qos = options.qos;
    if (options?.queue_length != null) msg.queue_length = options.queue_length;
    if (options?.fragment_size != null) msg.fragment_size = options.fragment_size;
    if (options?.compression) msg.compression = options.compression;
    this._send(msg);
  }

  _sendUnsubscribe(topic) {
    this._send({ op: "unsubscribe", topic });
  }

  /* ── Publish ───────────────────────────────────────────────── */

  publish(topic, type, msg) {
    this._send({ op: "publish", topic, type, msg });
  }

  /* ── Service Call ──────────────────────────────────────────── */

  callService(service, type, args = {}) {
    return new Promise((resolve, reject) => {
      const id = `svc_${++this._svcId}`;
      this._svcPending[id] = { resolve, reject };
      this._send({ op: "call_service", id, service, type, args });
      setTimeout(() => {
        if (this._svcPending[id]) {
          delete this._svcPending[id];
          reject(new Error("Service call timeout"));
        }
      }, 10000);
    });
  }

  /* ── Get Topic List ────────────────────────────────────────── */

  getTopics() {
    return this.callService("/rosapi/topics", "rosapi/Topics");
  }

  /* ── Message Handling ──────────────────────────────────────── */

  _handleMessage(data) {
    if (data.op === "publish" && data.topic) {
      const sub = this._subs[data.topic];
      if (sub) sub.cbs.forEach(cb => cb(data.msg, data.topic));
    }
    else if (data.op === "service_response" && data.id) {
      const pending = this._svcPending[data.id];
      if (pending) {
        delete this._svcPending[data.id];
        if (data.result !== false) pending.resolve(data.values || data);
        else pending.reject(new Error(data.values || "Service failed"));
      }
    }
  }
}

/* ── Sensor Data Parsers ──────────────────────────────────────── */

/**
 * Convert LaserScan message to array of {x, y} points in laser frame
 */
export function laserScanToPoints(msg) {
  const points = [];
  const { angle_min, angle_increment, ranges, range_min, range_max } = msg;
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i];
    if (r < range_min || r > range_max || !isFinite(r)) continue;
    const angle = angle_min + i * angle_increment;
    points.push({ x: r * Math.cos(angle), y: r * Math.sin(angle) });
  }
  return points;
}

/**
 * Convert PointCloud2 message to array of {x, y, z} points in sensor frame.
 * Parses binary data (base64/byte-array from rosbridge) and extracts x, y, z fields.
 * @param {number} [maxPoints=2000] — downsample to this many points for performance
 */
export function pointCloud2ToPoints(msg, maxPoints = 2000) {
  const points = [];
  const { fields, point_step, row_step, width, height, data, is_bigendian } = msg;
  if (!fields || !data) return points;

  // Find x, y, z field offsets
  let xOff = -1, yOff = -1, zOff = -1;
  let xType = 7, yType = 7, zType = 7;
  for (const f of fields) {
    if (f.name === "x") { xOff = f.offset; xType = f.datatype; }
    if (f.name === "y") { yOff = f.offset; yType = f.datatype; }
    if (f.name === "z") { zOff = f.offset; zType = f.datatype; }
  }
  if (xOff < 0 || yOff < 0) return points;

  // Decode to ArrayBuffer
  let buf;
  if (typeof data === "string") {
    const bin = atob(data);
    const arr = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) arr[i] = bin.charCodeAt(i);
    buf = arr.buffer;
  } else if (Array.isArray(data)) {
    buf = Uint8Array.from(data).buffer;
  } else if (ArrayBuffer.isView(data)) {
    buf = data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength);
  } else if (data instanceof ArrayBuffer) {
    buf = data;
  } else {
    return points;
  }

  const view = new DataView(buf);
  const le = !is_bigendian;
  const w = Math.max(1, width || 0);
  const h = Math.max(1, height || 1);
  const rs = row_step || (point_step * w);
  const totalPoints = w * h;
  const step = Math.max(1, Math.floor(totalPoints / maxPoints));

  const readFloat = (offset, datatype) => {
    if (datatype === 7) return view.getFloat32(offset, le); // FLOAT32
    if (datatype === 8) return view.getFloat64(offset, le); // FLOAT64
    return 0;
  };

  for (let i = 0; i < totalPoints; i += step) {
    const row = Math.floor(i / w);
    const col = i % w;
    const base = row * rs + col * point_step;
    if (base + point_step > buf.byteLength) break;
    const x = readFloat(base + xOff, xType);
    const y = readFloat(base + yOff, yType);
    const z = zOff >= 0 ? readFloat(base + zOff, zType) : 0;
    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
    if (x === 0 && y === 0 && z === 0) continue; // skip origin points
    points.push({ x, y, z });
  }
  return points;
}

/**
 * Transform points from one frame to map frame using a 2D pose (x, y, theta)
 */
export function transformPoints(points, pose) {
  const { x: tx, y: ty, theta } = pose;
  const cos = Math.cos(theta), sin = Math.sin(theta);
  return points.map(p => ({
    x: tx + p.x * cos - p.y * sin,
    y: ty + p.x * sin + p.y * cos,
  }));
}

function normalizeQuat(q) {
  const x = q?.x ?? 0;
  const y = q?.y ?? 0;
  const z = q?.z ?? 0;
  const w = q?.w ?? 1;
  const n = Math.hypot(x, y, z, w) || 1;
  return { x: x / n, y: y / n, z: z / n, w: w / n };
}

function quatMul(a, b) {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

function quatConj(q) {
  return { x: -q.x, y: -q.y, z: -q.z, w: q.w };
}

function rotateVecByQuat(v, qIn) {
  const q = normalizeQuat(qIn);
  const p = { x: v.x, y: v.y, z: v.z ?? 0, w: 0 };
  const qp = quatMul(q, p);
  const out = quatMul(qp, quatConj(q));
  return { x: out.x, y: out.y, z: out.z };
}

/**
 * Transform points from one frame to another using a 3D transform
 * {x,y,z,qx,qy,qz,qw}. Output keeps x,y,z.
 */
export function transformPoints3D(points, tf) {
  const t = { x: tf.x ?? 0, y: tf.y ?? 0, z: tf.z ?? 0 };
  const q = { x: tf.qx ?? 0, y: tf.qy ?? 0, z: tf.qz ?? 0, w: tf.qw ?? 1 };
  return points.map(p => {
    const r = rotateVecByQuat({ x: p.x, y: p.y, z: p.z ?? 0 }, q);
    return { x: t.x + r.x, y: t.y + r.y, z: t.z + r.z };
  });
}

/**
 * Compose 3D transforms: out = a ∘ b
 * a,b map child coordinates into parent coordinates.
 */
export function composeTransform3D(a, b) {
  const qa = normalizeQuat({ x: a.qx, y: a.qy, z: a.qz, w: a.qw });
  const qb = normalizeQuat({ x: b.qx, y: b.qy, z: b.qz, w: b.qw });
  const rb = rotateVecByQuat({ x: b.x ?? 0, y: b.y ?? 0, z: b.z ?? 0 }, qa);
  const q = normalizeQuat(quatMul(qa, qb));
  return {
    x: (a.x ?? 0) + rb.x,
    y: (a.y ?? 0) + rb.y,
    z: (a.z ?? 0) + rb.z,
    qx: q.x, qy: q.y, qz: q.z, qw: q.w,
  };
}

/**
 * Invert 3D transform
 */
export function invertTransform3D(t) {
  const q = normalizeQuat({ x: t.qx, y: t.qy, z: t.qz, w: t.qw });
  const qi = quatConj(q);
  const rt = rotateVecByQuat({ x: -(t.x ?? 0), y: -(t.y ?? 0), z: -(t.z ?? 0) }, qi);
  return { x: rt.x, y: rt.y, z: rt.z, qx: qi.x, qy: qi.y, qz: qi.z, qw: qi.w };
}

/**
 * Convert world coords (meters) to pixel coords on PGM canvas
 */
export function worldToPixel(wx, wy, origin, resolution, canvasHeight) {
  const ox = origin?.[0] ?? 0;
  const oy = origin?.[1] ?? 0;
  const yaw = origin?.[2] ?? 0;
  const dx = wx - ox;
  const dy = wy - oy;

  // Convert world point into map-local metric coordinates by removing origin yaw.
  const c = Math.cos(-yaw), s = Math.sin(-yaw);
  const lx = dx * c - dy * s;
  const ly = dx * s + dy * c;

  return {
    x: lx / resolution,
    y: canvasHeight - ly / resolution,
  };
}

/**
 * Convert pixel coords on PGM canvas to world coords (meters).
 */
export function pixelToWorld(px, py, origin, resolution, canvasHeight) {
  const ox = origin?.[0] ?? 0;
  const oy = origin?.[1] ?? 0;
  const yaw = origin?.[2] ?? 0;

  const lx = px * resolution;
  const ly = (canvasHeight - py) * resolution;

  // Apply map origin yaw/translation to get world coordinates.
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return {
    x: ox + lx * c - ly * s,
    y: oy + lx * s + ly * c,
  };
}

/**
 * Extract yaw from quaternion {x, y, z, w}
 */
export function quaternionToYaw(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

/**
 * Extract 2D pose from TF transform message
 */
export function tfToPose2D(tf) {
  const t = tf.translation || tf.transform?.translation || { x: 0, y: 0, z: 0 };
  const r = tf.rotation || tf.transform?.rotation || { x: 0, y: 0, z: 0, w: 1 };
  return { x: t.x, y: t.y, theta: quaternionToYaw(r) };
}

/**
 * Extract full 3D transform from TF transform message
 */
export function tfToTransform3D(tf) {
  const t = tf.translation || tf.transform?.translation || { x: 0, y: 0, z: 0 };
  const r = normalizeQuat(tf.rotation || tf.transform?.rotation || { x: 0, y: 0, z: 0, w: 1 });
  return { x: t.x, y: t.y, z: t.z, qx: r.x, qy: r.y, qz: r.z, qw: r.w };
}

export { STATES };
