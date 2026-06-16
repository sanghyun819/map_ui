import { useEffect, useMemo, useRef, useState } from "react";
import { pixelToWorld } from "./Ros2Bridge.js";

const S = {
  wrap: {
    position: "relative",
    width: "100%",
    height: "100%",
    background: "radial-gradient(circle at 30% 20%, #0d1a2e 0%, #060c18 55%, #03070f 100%)",
    borderLeft: "1px solid rgba(0,212,255,0.12)",
  },
  canvas: { width: "100%", height: "100%", display: "block" },
  hud: {
    position: "absolute",
    top: 8,
    left: 8,
    right: 8,
    display: "flex",
    flexWrap: "wrap",
    gap: 5,
    rowGap: 4,
    alignItems: "center",
    fontSize: 10,
    color: "#8eb8c8",
    fontFamily: "'JetBrains Mono','Fira Code',monospace",
    userSelect: "none",
    pointerEvents: "auto",
  },
  btn: (active = false) => ({
    border: `1px solid ${active ? "rgba(0,212,255,0.55)" : "rgba(0,212,255,0.2)"}`,
    color: active ? "#00d4ff" : "#78a9ba",
    background: active ? "rgba(0,212,255,0.2)" : "rgba(0,0,0,0.35)",
    borderRadius: 4,
    fontSize: 10,
    padding: "2px 7px",
    cursor: "pointer",
    whiteSpace: "nowrap",
    flexShrink: 0,
  }),
  info: {
    position: "absolute",
    bottom: 10,
    left: 10,
    fontSize: 10,
    color: "rgba(142,184,200,0.8)",
    fontFamily: "'JetBrains Mono','Fira Code',monospace",
    background: "rgba(0,8,20,0.55)",
    border: "1px solid rgba(0,212,255,0.18)",
    borderRadius: 4,
    padding: "4px 8px",
    userSelect: "none",
    pointerEvents: "none",
  },
};

const MAP_VS = `
attribute vec3 aPos;
attribute vec2 aUv;
uniform mat4 uMVP;
varying vec2 vUv;
void main() {
  vUv = aUv;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
`;

const MAP_FS = `
precision mediump float;
uniform sampler2D uTex;
uniform float uAlpha;
varying vec2 vUv;
void main() {
  vec4 c = texture2D(uTex, vUv);
  float g = (c.r + c.g + c.b) / 3.0;
  gl_FragColor = vec4(vec3(g), uAlpha);
}
`;

const PTS_VS = `
attribute vec3 aPos;
uniform mat4 uMVP;
uniform float uSize;
void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
  gl_PointSize = uSize;
}
`;

const PTS_FS = `
precision mediump float;
uniform vec3 uColor;
uniform float uAlpha;
void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  if (dot(d, d) > 0.25) discard;
  gl_FragColor = vec4(uColor, uAlpha);
}
`;

const CPTS_VS = `
attribute vec3 aPos;
attribute vec3 aCol;
uniform mat4 uMVP;
uniform float uSize;
uniform float uZScale;
varying vec3 vCol;
void main() {
  vCol = aCol;
  vec3 p = vec3(aPos.xy, aPos.z * uZScale);
  gl_Position = uMVP * vec4(p, 1.0);
  gl_PointSize = uSize;
}
`;

const CPTS_FS = `
precision mediump float;
varying vec3 vCol;
uniform float uAlpha;
void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  if (dot(d, d) > 0.25) discard;
  gl_FragColor = vec4(vCol, uAlpha);
}
`;

const LINE_VS = `
attribute vec3 aPos;
uniform mat4 uMVP;
void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
}
`;

const LINE_FS = `
precision mediump float;
uniform vec3 uColor;
uniform float uAlpha;
void main() {
  gl_FragColor = vec4(uColor, uAlpha);
}
`;

function compileShader(gl, type, src) {
  const sh = gl.createShader(type);
  gl.shaderSource(sh, src);
  gl.compileShader(sh);
  if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
    const err = gl.getShaderInfoLog(sh) || "Shader compile failed";
    gl.deleteShader(sh);
    throw new Error(err);
  }
  return sh;
}

function createProgram(gl, vsSrc, fsSrc) {
  const vs = compileShader(gl, gl.VERTEX_SHADER, vsSrc);
  const fs = compileShader(gl, gl.FRAGMENT_SHADER, fsSrc);
  const p = gl.createProgram();
  gl.attachShader(p, vs);
  gl.attachShader(p, fs);
  gl.linkProgram(p);
  gl.deleteShader(vs);
  gl.deleteShader(fs);
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) {
    const err = gl.getProgramInfoLog(p) || "Program link failed";
    gl.deleteProgram(p);
    throw new Error(err);
  }
  return p;
}

function normalize(v) {
  const n = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / n, v[1] / n, v[2] / n];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function sub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2);
  const nf = 1 / (near - far);
  return new Float32Array([
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (far + near) * nf, -1,
    0, 0, (2 * far * near) * nf, 0,
  ]);
}

// RANSAC plane fit on Nx3 points → {nx,ny,nz,d,cnt} (largest inlier set) or null.
function ransacPlane(pts, iters = 160, thr = 0.03) {
  const N = pts.length;
  if (N < 3) return null;
  let best = null;
  for (let it = 0; it < iters; it++) {
    const a = pts[(Math.random() * N) | 0], b = pts[(Math.random() * N) | 0], c = pts[(Math.random() * N) | 0];
    const ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
    const vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
    let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const nl = Math.hypot(nx, ny, nz);
    if (nl < 1e-6) continue;
    nx /= nl; ny /= nl; nz /= nl;
    const d = -(nx * a[0] + ny * a[1] + nz * a[2]);
    let cnt = 0;
    for (let i = 0; i < N; i++) {
      const p = pts[i];
      if (Math.abs(nx * p[0] + ny * p[1] + nz * p[2] + d) < thr) cnt++;
    }
    if (!best || cnt > best.cnt) best = { nx, ny, nz, d, cnt };
  }
  return best;
}

function invert4(m) {
  const inv = new Float32Array(16);
  inv[0] = m[5]*m[10]*m[15]-m[5]*m[11]*m[14]-m[9]*m[6]*m[15]+m[9]*m[7]*m[14]+m[13]*m[6]*m[11]-m[13]*m[7]*m[10];
  inv[4] = -m[4]*m[10]*m[15]+m[4]*m[11]*m[14]+m[8]*m[6]*m[15]-m[8]*m[7]*m[14]-m[12]*m[6]*m[11]+m[12]*m[7]*m[10];
  inv[8] = m[4]*m[9]*m[15]-m[4]*m[11]*m[13]-m[8]*m[5]*m[15]+m[8]*m[7]*m[13]+m[12]*m[5]*m[11]-m[12]*m[7]*m[9];
  inv[12] = -m[4]*m[9]*m[14]+m[4]*m[10]*m[13]+m[8]*m[5]*m[14]-m[8]*m[6]*m[13]-m[12]*m[5]*m[10]+m[12]*m[6]*m[9];
  inv[1] = -m[1]*m[10]*m[15]+m[1]*m[11]*m[14]+m[9]*m[2]*m[15]-m[9]*m[3]*m[14]-m[13]*m[2]*m[11]+m[13]*m[3]*m[10];
  inv[5] = m[0]*m[10]*m[15]-m[0]*m[11]*m[14]-m[8]*m[2]*m[15]+m[8]*m[3]*m[14]+m[12]*m[2]*m[11]-m[12]*m[3]*m[10];
  inv[9] = -m[0]*m[9]*m[15]+m[0]*m[11]*m[13]+m[8]*m[1]*m[15]-m[8]*m[3]*m[13]-m[12]*m[1]*m[11]+m[12]*m[3]*m[9];
  inv[13] = m[0]*m[9]*m[14]-m[0]*m[10]*m[13]-m[8]*m[1]*m[14]+m[8]*m[2]*m[13]+m[12]*m[1]*m[10]-m[12]*m[2]*m[9];
  inv[2] = m[1]*m[6]*m[15]-m[1]*m[7]*m[14]-m[5]*m[2]*m[15]+m[5]*m[3]*m[14]+m[13]*m[2]*m[7]-m[13]*m[3]*m[6];
  inv[6] = -m[0]*m[6]*m[15]+m[0]*m[7]*m[14]+m[4]*m[2]*m[15]-m[4]*m[3]*m[14]-m[12]*m[2]*m[7]+m[12]*m[3]*m[6];
  inv[10] = m[0]*m[5]*m[15]-m[0]*m[7]*m[13]-m[4]*m[1]*m[15]+m[4]*m[3]*m[13]+m[12]*m[1]*m[7]-m[12]*m[3]*m[5];
  inv[14] = -m[0]*m[5]*m[14]+m[0]*m[6]*m[13]+m[4]*m[1]*m[14]-m[4]*m[2]*m[13]-m[12]*m[1]*m[6]+m[12]*m[2]*m[5];
  inv[3] = -m[1]*m[6]*m[11]+m[1]*m[7]*m[10]+m[5]*m[2]*m[11]-m[5]*m[3]*m[10]-m[9]*m[2]*m[7]+m[9]*m[3]*m[6];
  inv[7] = m[0]*m[6]*m[11]-m[0]*m[7]*m[10]-m[4]*m[2]*m[11]+m[4]*m[3]*m[10]+m[8]*m[2]*m[7]-m[8]*m[3]*m[6];
  inv[11] = -m[0]*m[5]*m[11]+m[0]*m[7]*m[9]+m[4]*m[1]*m[11]-m[4]*m[3]*m[9]-m[8]*m[1]*m[7]+m[8]*m[3]*m[5];
  inv[15] = m[0]*m[5]*m[10]-m[0]*m[6]*m[9]-m[4]*m[1]*m[10]+m[4]*m[2]*m[9]+m[8]*m[1]*m[6]-m[8]*m[2]*m[5];
  let det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
  if (!det) return null;
  det = 1.0 / det;
  for (let i = 0; i < 16; i++) inv[i] *= det;
  return inv;
}

function multiply4(a, b) {
  const out = new Float32Array(16);
  for (let i = 0; i < 4; i++) {
    const ai0 = a[i];
    const ai1 = a[i + 4];
    const ai2 = a[i + 8];
    const ai3 = a[i + 12];
    out[i] = ai0 * b[0] + ai1 * b[1] + ai2 * b[2] + ai3 * b[3];
    out[i + 4] = ai0 * b[4] + ai1 * b[5] + ai2 * b[6] + ai3 * b[7];
    out[i + 8] = ai0 * b[8] + ai1 * b[9] + ai2 * b[10] + ai3 * b[11];
    out[i + 12] = ai0 * b[12] + ai1 * b[13] + ai2 * b[14] + ai3 * b[15];
  }
  return out;
}

function lookAt(eye, center, upHint) {
  const f = normalize(sub(center, eye));
  let s = cross(f, upHint);
  if (Math.hypot(s[0], s[1], s[2]) < 1e-6) s = cross(f, [0, 1, 0]);
  s = normalize(s);
  const u = cross(s, f);

  return new Float32Array([
    s[0], u[0], -f[0], 0,
    s[1], u[1], -f[1], 0,
    s[2], u[2], -f[2], 0,
    -dot(s, eye), -dot(u, eye), dot(f, eye), 1,
  ]);
}

function mapQuad(meta, canvasSize) {
  const { w, h } = canvasSize || {};
  if (!meta || !w || !h) return null;

  const bl = pixelToWorld(0, h, meta.origin, meta.resolution, h);
  const br = pixelToWorld(w, h, meta.origin, meta.resolution, h);
  const tr = pixelToWorld(w, 0, meta.origin, meta.resolution, h);
  const tl = pixelToWorld(0, 0, meta.origin, meta.resolution, h);

  return {
    pos: new Float32Array([
      bl.x, bl.y, 0,
      br.x, br.y, 0,
      tr.x, tr.y, 0,
      tl.x, tl.y, 0,
    ]),
    uv: new Float32Array([
      0, 0,
      1, 0,
      1, 1,
      0, 1,
    ]),
    idx: new Uint16Array([0, 1, 2, 0, 2, 3]),
    center: {
      x: (bl.x + tr.x) * 0.5,
      y: (bl.y + tr.y) * 0.5,
      z: 0,
    },
  };
}

function buildPathLine(pathWorldRef) {
  const src = pathWorldRef?.current || [];
  if (src.length < 2) return null;
  const arr = new Float32Array(src.length * 3);
  for (let i = 0; i < src.length; i++) {
    const p = src[i];
    arr[i * 3] = p.x;
    arr[i * 3 + 1] = p.y;
    arr[i * 3 + 2] = (p.z ?? 0) + 0.02;
  }
  return arr;
}

function buildRobotArrow(robotPose) {
  if (!robotPose) return null;
  const x = robotPose.x || 0;
  const y = robotPose.y || 0;
  const z = 0.06;
  const th = robotPose.theta || 0;
  const len = 0.7;
  const ex = x + Math.cos(th) * len;
  const ey = y + Math.sin(th) * len;
  const aw = 0.18;
  const lx = ex - Math.cos(th - 0.6) * aw;
  const ly = ey - Math.sin(th - 0.6) * aw;
  const rx = ex - Math.cos(th + 0.6) * aw;
  const ry = ey - Math.sin(th + 0.6) * aw;
  return new Float32Array([
    x, y, z, ex, ey, z,
    ex, ey, z, lx, ly, z,
    ex, ey, z, rx, ry, z,
  ]);
}

function flattenPoints(points, max = 200000) {
  if (!points?.length) return null;
  const step = Math.max(1, Math.floor(points.length / max));
  const n = Math.ceil(points.length / step);
  const out = new Float32Array(n * 3);
  let j = 0;
  for (let i = 0; i < points.length; i += step) {
    const p = points[i];
    out[j++] = p.x;
    out[j++] = p.y;
    out[j++] = p.z ?? 0;
  }
  return out;
}

function parseHeightMap(json) {
  // Accepts the height_view3d.json produced by 3d_map/build_height_map.py:
  // { count, xyz:[x,y,z,...], rgb:[r,g,b,...] } in map-frame world coords.
  if (!json || !Array.isArray(json.xyz) || !json.xyz.length) return null;
  const pos = Float32Array.from(json.xyz);
  const count = Math.floor(pos.length / 3);
  let col;
  if (Array.isArray(json.rgb) && json.rgb.length >= count * 3) {
    col = Float32Array.from(json.rgb.slice(0, count * 3));
  } else {
    col = new Float32Array(count * 3).fill(0.6); // grey fallback
  }
  return { pos, col, count, version: Date.now() };
}

function BuildPanel({ params, setParams, onPickBag, building, onClose, onBuild }) {
  const set = (k, v) => setParams(p => ({ ...p, [k]: v }));
  const num = (k, v) => set(k, v === "" ? "" : Number(v));
  const row = { display: "flex", alignItems: "center", gap: 6, marginBottom: 6 };
  const lab = { width: 64, color: "#8eb8c8", fontSize: 10, flexShrink: 0 };
  const inp = {
    flex: 1, minWidth: 0, background: "rgba(0,0,0,0.4)", color: "#cfeefb",
    border: "1px solid rgba(0,212,255,0.25)", borderRadius: 4, fontSize: 10, padding: "3px 6px",
  };
  const numStyle = { ...inp, flex: "none", width: 64 };
  return (
    <div style={{
      position: "absolute", top: 40, left: 10, width: 300, zIndex: 20,
      background: "rgba(6,14,28,0.96)", border: "1px solid rgba(0,212,255,0.3)",
      borderRadius: 6, padding: 10, fontFamily: "'JetBrains Mono','Fira Code',monospace",
      boxShadow: "0 6px 24px rgba(0,0,0,0.5)",
    }}>
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 8 }}>
        <span style={{ color: "#00d4ff", fontWeight: "bold", fontSize: 11 }}>🛠 Bag → 3D</span>
        <button style={S.btn(false)} onClick={onClose}>✕</button>
      </div>

      <div style={row}>
        <span style={lab}>bag</span>
        <input style={inp} value={params.bag} placeholder="rosbag 폴더"
          onChange={e => set("bag", e.target.value)} />
        {onPickBag && <button style={S.btn(false)} onClick={async () => { const p = await onPickBag(); if (p) set("bag", p); }}>…</button>}
      </div>
      <div style={row}>
        <span style={lab}>map.yaml</span>
        <input style={inp} value={params.map} placeholder="(선택) 2D 맵 yaml"
          onChange={e => set("map", e.target.value)} />
      </div>

      <div style={row}>
        <span style={lab}>voxel</span>
        <input type="number" step="0.005" style={numStyle} value={params.voxel} onChange={e => num("voxel", e.target.value)} />
        <span style={lab}>stride</span>
        <input type="number" step="1" min="1" style={numStyle} value={params.stride} onChange={e => num("stride", e.target.value)} />
      </div>
      <div style={row}>
        <span style={lab}>z 범위</span>
        <input type="number" step="0.1" style={numStyle} value={params.zMin} onChange={e => num("zMin", e.target.value)} />
        <input type="number" step="0.1" style={numStyle} value={params.zMax} onChange={e => num("zMax", e.target.value)} />
        <span style={lab}>range</span>
        <input type="number" step="1" style={numStyle} value={params.rangeMax} onChange={e => num("rangeMax", e.target.value)} />
      </div>
      <div style={row}>
        <span style={lab}>점 수</span>
        <input type="number" step="50000" style={numStyle} value={params.points} onChange={e => num("points", e.target.value)} />
        <label style={{ ...lab, width: "auto", display: "flex", gap: 3, alignItems: "center", cursor: "pointer" }}>
          <input type="checkbox" checked={params.icp} onChange={e => set("icp", e.target.checked)} /> ICP
        </label>
        <label style={{ ...lab, width: "auto", display: "flex", gap: 3, alignItems: "center", cursor: "pointer" }}>
          <input type="checkbox" checked={params.footprint} onChange={e => set("footprint", e.target.checked)} /> 자기제거
        </label>
      </div>
      <div style={row}>
        <span style={lab}>extra</span>
        <input style={inp} value={params.extra} placeholder="추가 플래그 (예: --color-image …)"
          onChange={e => set("extra", e.target.value)} />
      </div>

      <button
        style={{ ...S.btn(true), width: "100%", justifyContent: "center", padding: "5px", marginTop: 4, opacity: (building || !params.bag) ? 0.5 : 1 }}
        disabled={building || !params.bag}
        onClick={onBuild}
      >
        {building ? "빌드 중…" : "▶ 빌드 & 로드"}
      </button>
    </div>
  );
}

export default function Ros2View3D({
  mapCanvasRef,
  meta,
  canvasSize,
  lidarWorldPoints,
  pathWorldPoints,
  stats,
  fixedFrame,
  viewMode = "free",
  onChangeViewMode,
  externalCloud,
  onOpenHeightJson,
  onBuildFromBag,
  onCancelBuild,
  onPickBag,
  onKeepoutFromPolygon,
  onSemanticFromPolygon,
  zPickActive = false,
  onPickHeight,
  onPickHeightRange,
  defaultBag = "",
  defaultMap = "",
  building = false,
}) {
  const glCanvasRef = useRef(null);
  const glRef = useRef(null);
  const dragRef = useRef(null);
  const cameraRef = useRef({
    yaw: -0.6,
    pitch: 0.85,
    dist: 18,
    target: { x: 0, y: 0, z: 0 },
  });

  const heightMapRef = useRef(null);
  const heightMetaRef = useRef(null);     // origin/resolution/z range from loaded file
  const showHeightRef = useRef(true);
  const zScaleRef = useRef(1);
  const fileInputRef = useRef(null);
  const mvpRef = useRef(null);            // latest MVP, for screen-space point picking
  const editModeRef = useRef(false);
  const zPickRef = useRef(false);
  const undoRef = useRef([]);             // stack of previous {pos,col,count}
  const ptSizeRef = useRef(3.5);
  const [heightInfo, setHeightInfo] = useState(null);
  const [showHeight, setShowHeight] = useState(true);
  const [zScale, setZScale] = useState(1);
  const [ptSize, setPtSize] = useState(3.5);
  const [zCut, setZCut] = useState(2);
  const [editMode, setEditMode] = useState(false);
  const [polygon, setPolygon] = useState([]);   // CSS-px vertices for the lasso
  const [cursor, setCursor] = useState(null);    // live cursor for the rubber-band edge
  const [editMsg, setEditMsg] = useState(null);  // transient "removed N" feedback
  const [paintColor, setPaintColor] = useState("#ff2a2a");  // colour for painting selected points
  const [polyClosed, setPolyClosed] = useState(false);      // right-click finalised the polygon
  const [semType, setSemType] = useState("carrier");        // semantic layer the lasso sends to
  const [showBuildPanel, setShowBuildPanel] = useState(false);
  const [buildParams, setBuildParams] = useState({
    bag: "", map: "", voxel: 0.02, stride: 2, rangeMax: 20, zMin: 0, zMax: 2.5,
    icp: true, footprint: true, points: 300000, extra: "",
  });

  useEffect(() => { showHeightRef.current = showHeight; }, [showHeight]);
  useEffect(() => { zScaleRef.current = zScale; }, [zScale]);
  useEffect(() => { ptSizeRef.current = ptSize; }, [ptSize]);
  useEffect(() => { editModeRef.current = editMode; }, [editMode]);
  useEffect(() => { zPickRef.current = zPickActive; }, [zPickActive]);
  useEffect(() => {
    if (!editMode) return;
    const onKey = (e) => {
      if (["INPUT", "TEXTAREA"].includes(e.target.tagName)) return;
      if (e.key === "Backspace") { e.preventDefault(); setPolygon(p => p.slice(0, -1)); setPolyClosed(false); }
      else if (e.key === "Escape") { setPolygon([]); setCursor(null); setPolyClosed(false); }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [editMode]);

  const loadHeightJson = (json) => {
    const data = parseHeightMap(json);
    if (!data) { setHeightInfo("로드 실패: 빈 데이터"); return false; }
    heightMapRef.current = data;
    heightMetaRef.current = {
      type: json.type || "height_map_view3d",
      frame: json.frame || "map",
      resolution: json.resolution,
      origin: json.origin,
      z_min: json.z_min,
      z_max: json.z_max,
    };
    undoRef.current = [];
    setHeightInfo(`${data.count.toLocaleString()} pts`);
    return true;
  };

  const loadHeightFile = (file) => {
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const json = JSON.parse(reader.result);
        loadHeightJson(json);
      } catch (err) {
        console.error("[3D] height map parse error:", err);
        setHeightInfo("로드 실패: JSON 오류");
      }
    };
    reader.readAsText(file);
  };

  const openHeightJson = async () => {
    if (!onOpenHeightJson) {
      fileInputRef.current?.click();
      return;
    }
    try {
      const loaded = await onOpenHeightJson();
      if (!loaded?.json) return;
      loadHeightJson(loaded.json);
    } catch (err) {
      console.error("[3D] height map host-load error:", err);
      setHeightInfo("로드 실패: JSON 오류");
    }
  };

  // Load a cloud built from a bag (cloud_view3d.json / height_view3d.json) programmatically.
  useEffect(() => {
    const json = externalCloud?.data;
    if (!json) return;
    loadHeightJson(json);
  }, [externalCloud?.key]);

  // ---- CloudCompare-style polygon segmentation -------------------------------
  const pointInPoly = (px, py, poly) => {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const xi = poly[i].x, yi = poly[i].y, xj = poly[j].x, yj = poly[j].y;
      if (((yi > py) !== (yj > py)) && (px < ((xj - xi) * (py - yi)) / (yj - yi) + xi)) {
        inside = !inside;
      }
    }
    return inside;
  };

  const applyDelete = (deleteInside) => {
    const hm = heightMapRef.current;
    const mvp = mvpRef.current;
    const canvas = glCanvasRef.current;
    if (!hm || !mvp || !canvas || polygon.length < 3) return;
    const W = canvas.clientWidth, H = canvas.clientHeight;
    const zs = zScaleRef.current;

    const keep = [];
    for (let i = 0; i < hm.count; i++) {
      const x = hm.pos[i * 3], y = hm.pos[i * 3 + 1], z = hm.pos[i * 3 + 2] * zs;
      const cw = mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15];
      let sel = false;
      if (cw > 1e-6) {
        const cx = mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12];
        const cy = mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13];
        const sx = (cx / cw * 0.5 + 0.5) * W;
        const sy = (1 - (cy / cw * 0.5 + 0.5)) * H;
        sel = pointInPoly(sx, sy, polygon);
      }
      if (deleteInside ? !sel : sel) keep.push(i);
    }

    const np = new Float32Array(keep.length * 3);
    const nc = new Float32Array(keep.length * 3);
    for (let k = 0; k < keep.length; k++) {
      const i = keep[k];
      np[k * 3] = hm.pos[i * 3]; np[k * 3 + 1] = hm.pos[i * 3 + 1]; np[k * 3 + 2] = hm.pos[i * 3 + 2];
      nc[k * 3] = hm.col[i * 3]; nc[k * 3 + 1] = hm.col[i * 3 + 1]; nc[k * 3 + 2] = hm.col[i * 3 + 2];
    }
    undoRef.current.push({ pos: hm.pos, col: hm.col, count: hm.count });
    if (undoRef.current.length > 30) undoRef.current.shift();
    heightMapRef.current = { pos: np, col: nc, count: keep.length, version: Date.now() };
    setHeightInfo(`${keep.length.toLocaleString()} pts`);
    setEditMsg(`−${(hm.count - keep.length).toLocaleString()} (남음 ${keep.length.toLocaleString()})`);
    setPolygon([]);
    setCursor(null);
    setPolyClosed(false);
  };

  // Recolour the points inside (or outside) the polygon — "paint" the lidar region.
  const applyPaint = (inside) => {
    const hm = heightMapRef.current;
    const mvp = mvpRef.current;
    const canvas = glCanvasRef.current;
    if (!hm || !mvp || !canvas || polygon.length < 3) return;
    const W = canvas.clientWidth, H = canvas.clientHeight;
    const zs = zScaleRef.current;
    const h = paintColor.replace("#", "");
    const n = parseInt(h, 16);
    const r = ((n >> 16) & 255) / 255, g = ((n >> 8) & 255) / 255, b = (n & 255) / 255;

    const col = new Float32Array(hm.col);   // copy so undo keeps the original
    let painted = 0;
    for (let i = 0; i < hm.count; i++) {
      const x = hm.pos[i * 3], y = hm.pos[i * 3 + 1], z = hm.pos[i * 3 + 2] * zs;
      const cw = mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15];
      let sel = false;
      if (cw > 1e-6) {
        const cx = mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12];
        const cy = mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13];
        const sx = (cx / cw * 0.5 + 0.5) * W;
        const sy = (1 - (cy / cw * 0.5 + 0.5)) * H;
        sel = pointInPoly(sx, sy, polygon);
      }
      if (inside ? sel : !sel) { col[i * 3] = r; col[i * 3 + 1] = g; col[i * 3 + 2] = b; painted++; }
    }
    undoRef.current.push({ pos: hm.pos, col: hm.col, count: hm.count });
    if (undoRef.current.length > 30) undoRef.current.shift();
    heightMapRef.current = { pos: hm.pos, col, count: hm.count, version: Date.now() };
    setEditMsg(`🎨 ${painted.toLocaleString()}점 색칠`);
    setPolygon([]);
    setCursor(null);
    setPolyClosed(false);
  };

  // Unproject a screen point onto the ground plane (z=0) → world (map) x,y.
  const groundFromScreen = (sx, sy) => {
    const mvp = mvpRef.current, canvas = glCanvasRef.current;
    if (!mvp || !canvas) return null;
    const inv = invert4(mvp);
    if (!inv) return null;
    const W = canvas.clientWidth, H = canvas.clientHeight;
    const ndcx = (sx / W) * 2 - 1, ndcy = 1 - (sy / H) * 2;
    const unp = (ndcz) => {
      const x = inv[0] * ndcx + inv[4] * ndcy + inv[8] * ndcz + inv[12];
      const y = inv[1] * ndcx + inv[5] * ndcy + inv[9] * ndcz + inv[13];
      const z = inv[2] * ndcx + inv[6] * ndcy + inv[10] * ndcz + inv[14];
      const w = inv[3] * ndcx + inv[7] * ndcy + inv[11] * ndcz + inv[15];
      if (Math.abs(w) < 1e-9) return null;
      return { x: x / w, y: y / w, z: z / w };
    };
    const a = unp(-1), b = unp(1);
    if (!a || !b) return null;
    const dz = b.z - a.z;
    if (Math.abs(dz) < 1e-9) return null;
    const t = -a.z / dz;                        // intersect ground plane z=0
    return { x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) };
  };

  // Send the lasso (projected to the ground) to the 2D map as a keepout zone.
  const sendKeepout = () => {
    if (!onKeepoutFromPolygon || polygon.length < 3) return;
    const world = polygon.map(p => groundFromScreen(p.x, p.y)).filter(Boolean);
    if (world.length < 3) { setEditMsg("진입금지 변환 실패 (Top뷰 권장)"); return; }
    onKeepoutFromPolygon(world);
    setEditMsg(`⛔ 진입금지로 전송 (${world.length}각형)`);
    setPolygon([]);
    setCursor(null);
    setPolyClosed(false);
  };

  // Extract the carrier face from the lassoed lidar points: RANSAC plane fit (robust to
  // floor/stray points), then take the z band from the plane INLIERS.
  const sendHeightRange = () => {
    const hm = heightMapRef.current, mvp = mvpRef.current, canvas = glCanvasRef.current;
    if (!onPickHeightRange || polygon.length < 3 || !hm || !mvp || !canvas) return;
    const W = canvas.clientWidth, H = canvas.clientHeight, zs = zScaleRef.current;
    const pts = [];                                  // world (x,y,z) of lassoed points
    for (let i = 0; i < hm.count; i++) {
      const x = hm.pos[i * 3], y = hm.pos[i * 3 + 1], z = hm.pos[i * 3 + 2] * zs;
      const cw = mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15];
      if (cw <= 1e-6) continue;
      const sx = (mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12]) / cw * 0.5 + 0.5;
      const sy = 1 - ((mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13]) / cw * 0.5 + 0.5);
      if (pointInPoly(sx * W, sy * H, polygon)) pts.push([x, y, hm.pos[i * 3 + 2]]);
    }
    if (pts.length < 3) { setEditMsg("영역 안에 점이 부족"); return; }
    const pl = ransacPlane(pts, 160, 0.03);
    const thr = 0.03;
    let mn = Infinity, mx = -Infinity, n = 0;
    for (const p of pts) {
      const onPlane = pl ? Math.abs(pl.nx * p[0] + pl.ny * p[1] + pl.nz * p[2] + pl.d) < thr : true;
      if (onPlane) { if (p[2] < mn) mn = p[2]; if (p[2] > mx) mx = p[2]; n++; }
    }
    if (!n) { mn = Math.min(...pts.map(p => p[2])); mx = Math.max(...pts.map(p => p[2])); n = pts.length; }
    const tilt = pl ? Math.abs(pl.nz) : 0;           // ~1 horizontal plane(top), ~0 vertical(front)
    onPickHeightRange(mn, mx, n);
    setEditMsg(`📐 면추출 z ${mn.toFixed(2)}~${mx.toFixed(2)} m (${n}점, ${tilt > 0.7 ? "윗면" : "앞면"})`);
    setPolygon([]); setCursor(null); setPolyClosed(false);
  };

  // Send the lasso (projected to the ground) to the 2D map as a carrier/room/object.
  const sendSemantic = () => {
    if (!onSemanticFromPolygon || polygon.length < 3) return;
    const world = polygon.map(p => groundFromScreen(p.x, p.y)).filter(Boolean);
    if (world.length < 3) { setEditMsg("시맨틱 변환 실패 (Top뷰 권장)"); return; }
    onSemanticFromPolygon(world, semType);
    setEditMsg(`📦 ${semType}로 전송 (${world.length}각형)`);
    setPolygon([]);
    setCursor(null);
    setPolyClosed(false);
  };

  const deleteByZ = (above, zVal) => {
    const hm = heightMapRef.current;
    if (!hm || hm.count === 0 || !Number.isFinite(zVal)) return;
    const keep = [];
    for (let i = 0; i < hm.count; i++) {
      const z = hm.pos[i * 3 + 2];           // raw world height (not z-scaled)
      const cut = above ? z > zVal : z < zVal;
      if (!cut) keep.push(i);
    }
    if (keep.length === hm.count) { setEditMsg("삭제할 점 없음"); return; }
    const np = new Float32Array(keep.length * 3);
    const nc = new Float32Array(keep.length * 3);
    for (let k = 0; k < keep.length; k++) {
      const i = keep[k];
      np[k * 3] = hm.pos[i * 3]; np[k * 3 + 1] = hm.pos[i * 3 + 1]; np[k * 3 + 2] = hm.pos[i * 3 + 2];
      nc[k * 3] = hm.col[i * 3]; nc[k * 3 + 1] = hm.col[i * 3 + 1]; nc[k * 3 + 2] = hm.col[i * 3 + 2];
    }
    undoRef.current.push({ pos: hm.pos, col: hm.col, count: hm.count });
    if (undoRef.current.length > 30) undoRef.current.shift();
    heightMapRef.current = { pos: np, col: nc, count: keep.length, version: Date.now() };
    setHeightInfo(`${keep.length.toLocaleString()} pts`);
    setEditMsg(`z${above ? ">" : "<"}${zVal} −${(hm.count - keep.length).toLocaleString()} (남음 ${keep.length.toLocaleString()})`);
  };

  const undoEdit = () => {
    const prev = undoRef.current.pop();
    if (!prev) return;
    heightMapRef.current = { ...prev, version: Date.now() };
    setHeightInfo(`${prev.count.toLocaleString()} pts`);
    setEditMsg(`되돌림 (${prev.count.toLocaleString()})`);
  };

  const saveHeightJson = () => {
    const hm = heightMapRef.current;
    if (!hm) return;
    const m = heightMetaRef.current || {};
    const zs = hm.pos.length ? hm.pos.filter((_, i) => i % 3 === 2) : [];
    const out = {
      type: "height_map_view3d",
      frame: m.frame || "map",
      resolution: m.resolution,
      origin: m.origin,
      z_min: m.z_min ?? (zs.length ? Math.min(...zs) : 0),
      z_max: m.z_max ?? (zs.length ? Math.max(...zs) : 0),
      count: hm.count,
      xyz: Array.from(hm.pos, (v) => Math.round(v * 1e4) / 1e4),
      rgb: Array.from(hm.col, (v) => Math.round(v * 1e3) / 1e3),
    };
    const blob = new Blob([JSON.stringify(out)], { type: "application/json" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "height_view3d_edited.json";
    a.click();
    URL.revokeObjectURL(a.href);
  };

  const mapGeom = useMemo(() => mapQuad(meta, canvasSize), [meta, canvasSize]);

  useEffect(() => {
    if (!mapGeom) return;
    cameraRef.current.target = { ...mapGeom.center, z: 0 };
  }, [mapGeom]);

  useEffect(() => {
    const canvas = glCanvasRef.current;
    if (!canvas) return;
    const gl = canvas.getContext("webgl", { antialias: true, alpha: false, preserveDrawingBuffer: false });
    if (!gl) return;
    glRef.current = gl;

    let mapProgram;
    let ptsProgram;
    let cptsProgram;
    let lineProgram;
    let mapPosBuf;
    let mapUvBuf;
    let mapIdxBuf;
    let mapTex;
    let ptsBuf;
    let cptsPosBuf;
    let cptsColBuf;
    let lineBuf;
    let anim = 0;
    let hmUploadedVersion = -1;

    try {
      mapProgram = createProgram(gl, MAP_VS, MAP_FS);
      ptsProgram = createProgram(gl, PTS_VS, PTS_FS);
      cptsProgram = createProgram(gl, CPTS_VS, CPTS_FS);
      lineProgram = createProgram(gl, LINE_VS, LINE_FS);
    } catch (e) {
      console.error("[3D] shader/program error:", e);
      return;
    }

    mapPosBuf = gl.createBuffer();
    mapUvBuf = gl.createBuffer();
    mapIdxBuf = gl.createBuffer();
    ptsBuf = gl.createBuffer();
    cptsPosBuf = gl.createBuffer();
    cptsColBuf = gl.createBuffer();
    lineBuf = gl.createBuffer();

    mapTex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, mapTex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);

    const render = () => {
      const dpr = window.devicePixelRatio || 1;
      const cw = Math.max(1, Math.floor(canvas.clientWidth * dpr));
      const ch = Math.max(1, Math.floor(canvas.clientHeight * dpr));
      if (canvas.width !== cw || canvas.height !== ch) {
        canvas.width = cw;
        canvas.height = ch;
      }
      gl.viewport(0, 0, cw, ch);
      gl.enable(gl.DEPTH_TEST);
      gl.clearColor(0.03, 0.06, 0.11, 1);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

      const cam = cameraRef.current;
      const robot = stats?.current?.robotPose;
      if (viewMode === "top") {
        if (robot) cam.target = { x: robot.x, y: robot.y, z: 0 };
      }

      let eye;
      let center;
      let up;
      if (viewMode === "top") {
        eye = [cam.target.x, cam.target.y, Math.max(8, cam.dist)];
        center = [cam.target.x, cam.target.y, 0];
        up = [0, 1, 0];
      } else {
        const cp = Math.cos(cam.pitch);
        const sp = Math.sin(cam.pitch);
        eye = [
          cam.target.x + cam.dist * cp * Math.cos(cam.yaw),
          cam.target.y + cam.dist * cp * Math.sin(cam.yaw),
          cam.target.z + cam.dist * sp,
        ];
        center = [cam.target.x, cam.target.y, cam.target.z];
        up = [0, 0, 1];
      }

      const proj = perspective((viewMode === "top" ? 42 : 60) * Math.PI / 180, cw / ch, 0.05, 1000);
      const view = lookAt(eye, center, up);
      const mvp = multiply4(proj, view);
      mvpRef.current = mvp;

      if (mapGeom && mapCanvasRef?.current) {
        const mapCanvas = mapCanvasRef.current;
        gl.bindTexture(gl.TEXTURE_2D, mapTex);
        try {
          gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, mapCanvas);
        } catch {
          // keep previous texture
        }

        gl.useProgram(mapProgram);

        const aPos = gl.getAttribLocation(mapProgram, "aPos");
        const aUv = gl.getAttribLocation(mapProgram, "aUv");
        const uMVP = gl.getUniformLocation(mapProgram, "uMVP");
        const uAlpha = gl.getUniformLocation(mapProgram, "uAlpha");
        const uTex = gl.getUniformLocation(mapProgram, "uTex");

        gl.bindBuffer(gl.ARRAY_BUFFER, mapPosBuf);
        gl.bufferData(gl.ARRAY_BUFFER, mapGeom.pos, gl.STATIC_DRAW);
        gl.enableVertexAttribArray(aPos);
        gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 0, 0);

        gl.bindBuffer(gl.ARRAY_BUFFER, mapUvBuf);
        gl.bufferData(gl.ARRAY_BUFFER, mapGeom.uv, gl.STATIC_DRAW);
        gl.enableVertexAttribArray(aUv);
        gl.vertexAttribPointer(aUv, 2, gl.FLOAT, false, 0, 0);

        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, mapIdxBuf);
        gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, mapGeom.idx, gl.STATIC_DRAW);

        gl.uniformMatrix4fv(uMVP, false, mvp);
        gl.uniform1f(uAlpha, 0.9);
        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, mapTex);
        gl.uniform1i(uTex, 0);
        gl.drawElements(gl.TRIANGLES, mapGeom.idx.length, gl.UNSIGNED_SHORT, 0);
      }

      const hm = heightMapRef.current;
      if (showHeightRef.current && hm && hm.count > 0) {
        gl.useProgram(cptsProgram);
        const aPos = gl.getAttribLocation(cptsProgram, "aPos");
        const aCol = gl.getAttribLocation(cptsProgram, "aCol");
        const uMVP = gl.getUniformLocation(cptsProgram, "uMVP");
        const uSize = gl.getUniformLocation(cptsProgram, "uSize");
        const uAlpha = gl.getUniformLocation(cptsProgram, "uAlpha");
        const uZScale = gl.getUniformLocation(cptsProgram, "uZScale");

        if (hm.version !== hmUploadedVersion) {
          gl.bindBuffer(gl.ARRAY_BUFFER, cptsPosBuf);
          gl.bufferData(gl.ARRAY_BUFFER, hm.pos, gl.STATIC_DRAW);
          gl.bindBuffer(gl.ARRAY_BUFFER, cptsColBuf);
          gl.bufferData(gl.ARRAY_BUFFER, hm.col, gl.STATIC_DRAW);
          hmUploadedVersion = hm.version;
        }

        gl.bindBuffer(gl.ARRAY_BUFFER, cptsPosBuf);
        gl.enableVertexAttribArray(aPos);
        gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 0, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, cptsColBuf);
        gl.enableVertexAttribArray(aCol);
        gl.vertexAttribPointer(aCol, 3, gl.FLOAT, false, 0, 0);

        gl.uniformMatrix4fv(uMVP, false, mvp);
        gl.uniform1f(uSize, ptSizeRef.current * (viewMode === "top" ? 0.8 : 1));
        gl.uniform1f(uAlpha, 0.95);
        gl.uniform1f(uZScale, zScaleRef.current);
        gl.drawArrays(gl.POINTS, 0, hm.count);
      }

      const pts = flattenPoints(lidarWorldPoints?.current, 200000);
      if (pts && pts.length >= 3) {
        gl.useProgram(ptsProgram);
        const aPos = gl.getAttribLocation(ptsProgram, "aPos");
        const uMVP = gl.getUniformLocation(ptsProgram, "uMVP");
        const uSize = gl.getUniformLocation(ptsProgram, "uSize");
        const uColor = gl.getUniformLocation(ptsProgram, "uColor");
        const uAlpha = gl.getUniformLocation(ptsProgram, "uAlpha");

        gl.bindBuffer(gl.ARRAY_BUFFER, ptsBuf);
        gl.bufferData(gl.ARRAY_BUFFER, pts, gl.DYNAMIC_DRAW);
        gl.enableVertexAttribArray(aPos);
        gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 0, 0);

        gl.uniformMatrix4fv(uMVP, false, mvp);
        gl.uniform1f(uSize, viewMode === "top" ? 2.0 : 3.0);
        gl.uniform3f(uColor, 1.0, 0.28, 0.22);
        gl.uniform1f(uAlpha, 0.9);
        gl.drawArrays(gl.POINTS, 0, pts.length / 3);
      }

      const path = buildPathLine(pathWorldPoints);
      if (path && path.length >= 6) {
        gl.useProgram(lineProgram);
        const aPos = gl.getAttribLocation(lineProgram, "aPos");
        const uMVP = gl.getUniformLocation(lineProgram, "uMVP");
        const uColor = gl.getUniformLocation(lineProgram, "uColor");
        const uAlpha = gl.getUniformLocation(lineProgram, "uAlpha");

        gl.bindBuffer(gl.ARRAY_BUFFER, lineBuf);
        gl.bufferData(gl.ARRAY_BUFFER, path, gl.DYNAMIC_DRAW);
        gl.enableVertexAttribArray(aPos);
        gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 0, 0);

        gl.uniformMatrix4fv(uMVP, false, mvp);
        gl.uniform3f(uColor, 1.0, 0.73, 0.2);
        gl.uniform1f(uAlpha, 0.9);
        gl.lineWidth(2);
        gl.drawArrays(gl.LINE_STRIP, 0, path.length / 3);
      }

      const arrow = buildRobotArrow(stats?.current?.robotPose);
      if (arrow) {
        gl.useProgram(lineProgram);
        const aPos = gl.getAttribLocation(lineProgram, "aPos");
        const uMVP = gl.getUniformLocation(lineProgram, "uMVP");
        const uColor = gl.getUniformLocation(lineProgram, "uColor");
        const uAlpha = gl.getUniformLocation(lineProgram, "uAlpha");

        gl.bindBuffer(gl.ARRAY_BUFFER, lineBuf);
        gl.bufferData(gl.ARRAY_BUFFER, arrow, gl.DYNAMIC_DRAW);
        gl.enableVertexAttribArray(aPos);
        gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 0, 0);

        gl.uniformMatrix4fv(uMVP, false, mvp);
        gl.uniform3f(uColor, 0.0, 0.9, 0.45);
        gl.uniform1f(uAlpha, 1.0);
        gl.lineWidth(3);
        gl.drawArrays(gl.LINES, 0, arrow.length / 3);
      }

      anim = requestAnimationFrame(render);
    };

    anim = requestAnimationFrame(render);
    return () => {
      cancelAnimationFrame(anim);
      gl.deleteBuffer(mapPosBuf);
      gl.deleteBuffer(mapUvBuf);
      gl.deleteBuffer(mapIdxBuf);
      gl.deleteBuffer(ptsBuf);
      gl.deleteBuffer(cptsPosBuf);
      gl.deleteBuffer(cptsColBuf);
      gl.deleteBuffer(lineBuf);
      gl.deleteTexture(mapTex);
      gl.deleteProgram(mapProgram);
      gl.deleteProgram(ptsProgram);
      gl.deleteProgram(cptsProgram);
      gl.deleteProgram(lineProgram);
    };
  }, [mapCanvasRef, mapGeom, lidarWorldPoints, pathWorldPoints, stats, viewMode]);

  const localXY = (e) => {
    const r = glCanvasRef.current.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  };

  const onMouseDown = (e) => {
    e.preventDefault();
    // Height pick: click the nearest lidar point → report its world z.
    if (zPickRef.current && e.button === 0) {
      const hm = heightMapRef.current, mvp = mvpRef.current, canvas = glCanvasRef.current;
      if (hm && mvp && canvas) {
        const W = canvas.clientWidth, H = canvas.clientHeight, zs = zScaleRef.current;
        const { x: mx, y: my } = localXY(e);
        let bi = -1, bd = 14 * 14;
        for (let i = 0; i < hm.count; i++) {
          const x = hm.pos[i * 3], y = hm.pos[i * 3 + 1], z = hm.pos[i * 3 + 2] * zs;
          const cw = mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15];
          if (cw <= 1e-6) continue;
          const sx = (mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12]) / cw * 0.5 + 0.5;
          const sy = 1 - ((mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13]) / cw * 0.5 + 0.5);
          const dx = sx * W - mx, dy = sy * H - my, dd = dx * dx + dy * dy;
          if (dd < bd) { bd = dd; bi = i; }
        }
        if (bi >= 0) { onPickHeight?.(hm.pos[bi * 3 + 2]); setEditMsg(`📏 z=${hm.pos[bi * 3 + 2].toFixed(2)} m`); }
      }
      return;
    }
    // Edit mode: left-click drops a lasso vertex; right-click (no drag) finishes it.
    if (editModeRef.current && e.button === 0) {
      const pt = localXY(e);
      if (polyClosed) { setPolygon([pt]); setPolyClosed(false); return; }  // start a new polygon
      setPolygon(p => {
        if (p.length) {
          const last = p[p.length - 1];
          if (Math.hypot(pt.x - last.x, pt.y - last.y) < 4) return p; // dedupe (double-click)
        }
        if (p.length >= 3) {
          const d = Math.hypot(pt.x - p[0].x, pt.y - p[0].y);
          if (d < 10) { setPolyClosed(true); return p; } // near first vertex → close
        }
        return [...p, pt];
      });
      return;
    }
    dragRef.current = {
      btn: e.button,
      x: e.clientX,
      y: e.clientY,
      moved: false,
      yaw: cameraRef.current.yaw,
      pitch: cameraRef.current.pitch,
      tx: cameraRef.current.target.x,
      ty: cameraRef.current.target.y,
    };
  };

  const onMouseMove = (e) => {
    if (editModeRef.current) setCursor(localXY(e));
    const d = dragRef.current;
    if (!d) return;
    const dx = e.clientX - d.x;
    const dy = e.clientY - d.y;
    if (Math.abs(dx) + Math.abs(dy) > 3) d.moved = true;
    const cam = cameraRef.current;

    if (viewMode === "free" && d.btn === 0) {
      cam.yaw = d.yaw - dx * 0.008;
      cam.pitch = Math.max(0.1, Math.min(1.45, d.pitch - dy * 0.006));
    } else {
      const scale = Math.max(0.01, cam.dist * 0.002);
      cam.target.x = d.tx - dx * scale;
      cam.target.y = d.ty + dy * scale;
    }
  };

  const onMouseUp = () => {
    const d = dragRef.current;
    dragRef.current = null;
    // Right-click without dragging = finish the polygon selection.
    if (d && d.btn === 2 && !d.moved && editModeRef.current && polygon.length >= 3) {
      setPolyClosed(true);
      setCursor(null);
    }
  };

  const onWheel = (e) => {
    e.preventDefault();
    const cam = cameraRef.current;
    const dz = e.deltaY > 0 ? 1.12 : 0.88;
    cam.dist = Math.max(2, Math.min(200, cam.dist * dz));
  };

  return (
    <div style={S.wrap}>
      <canvas
        ref={glCanvasRef}
        style={{ ...S.canvas, cursor: (editMode || zPickActive) ? "crosshair" : "default" }}
        onMouseDown={onMouseDown}
        onMouseMove={onMouseMove}
        onMouseUp={onMouseUp}
        onMouseLeave={onMouseUp}
        onWheel={onWheel}
        onContextMenu={e => e.preventDefault()}
      />
      {editMode && polygon.length > 0 && (
        <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }}>
          <polyline
            points={[...polygon, ...(polyClosed || !cursor ? [] : [cursor]), polygon[0]]
              .map(p => `${p.x},${p.y}`).join(" ")}
            fill={polyClosed ? "rgba(0,255,136,0.18)" : "rgba(0,212,255,0.12)"}
            stroke={polyClosed ? "#00ff88" : "#00d4ff"} strokeWidth="1.5"
            strokeDasharray={polyClosed ? "0" : "5 4"}
          />
          {polygon.map((p, i) => (
            <circle key={i} cx={p.x} cy={p.y} r={i === 0 ? 4 : 3} fill={i === 0 ? "#00ff88" : "#00d4ff"} />
          ))}
          {!polyClosed && polygon.length >= 3 && cursor && Math.hypot(cursor.x - polygon[0].x, cursor.y - polygon[0].y) < 10 && (
            <circle cx={polygon[0].x} cy={polygon[0].y} r="9" fill="none" stroke="#00ff88" strokeWidth="1.5" />
          )}
        </svg>
      )}
      <div style={S.hud}>
        <span style={{ color: "#00d4ff", fontWeight: "bold" }}>3D</span>
        <button style={S.btn(viewMode === "free")} onClick={() => onChangeViewMode?.("free")}>Free</button>
        <button style={S.btn(viewMode === "top")} onClick={() => onChangeViewMode?.("top")}>Top</button>
        <span style={{ opacity: 0.4 }}>|</span>
        {onBuildFromBag && (
          <button
            style={S.btn(building || showBuildPanel)}
            onClick={() => {
              if (building) { onCancelBuild?.(); return; }
              setBuildParams(p => ({ ...p, bag: p.bag || defaultBag, map: p.map || defaultMap }));
              setShowBuildPanel(v => !v);
            }}
            title={building ? "빌드 취소" : "rosbag에서 3D 포인트 생성"}
          >
            {building ? "⏳ 빌드중… ✕취소" : "🛠 Bag→3D"}
          </button>
        )}
        <button style={S.btn(false)} onClick={openHeightJson} title="height_view3d.json / cloud_view3d.json 로드">Height ⬆</button>
        {heightInfo && (
          <button style={S.btn(showHeight)} onClick={() => setShowHeight(v => !v)} title="높이맵 표시/숨김">
            {showHeight ? "◉" : "○"} {heightInfo}
          </button>
        )}
        {heightInfo && showHeight && (
          <label style={{ display: "flex", alignItems: "center", gap: 4 }} title="높이 과장 배율">
            z×{zScale.toFixed(1)}
            <input
              type="range" min="0.5" max="5" step="0.5" value={zScale}
              onChange={e => setZScale(parseFloat(e.target.value))}
              style={{ width: 56 }}
            />
          </label>
        )}
        {heightInfo && showHeight && (
          <label style={{ display: "flex", alignItems: "center", gap: 4 }} title="포인트 크기">
            ● {ptSize.toFixed(1)}
            <input
              type="range" min="1" max="10" step="0.5" value={ptSize}
              onChange={e => setPtSize(parseFloat(e.target.value))}
              style={{ width: 56 }}
            />
          </label>
        )}
        {heightInfo && showHeight && (
          <label style={{ display: "flex", alignItems: "center", gap: 3 }} title="기준 높이 z [m]">
            z=
            <input
              type="number" step="0.1" value={zCut}
              onChange={e => setZCut(parseFloat(e.target.value))}
              style={{ width: 48, background: "rgba(0,0,0,0.4)", color: "#9fe", border: "1px solid rgba(0,212,255,0.3)", borderRadius: 3, fontSize: 10 }}
            />
            <button style={S.btn(false)} onClick={() => deleteByZ(true, zCut)} title="이 z보다 위 점 모두 삭제">↑날리기</button>
            <button style={S.btn(false)} onClick={() => deleteByZ(false, zCut)} title="이 z보다 아래 점 모두 삭제">↓날리기</button>
          </label>
        )}
        {heightInfo && showHeight && (
          <>
            <span style={{ opacity: 0.4 }}>|</span>
            <button
              style={S.btn(editMode)}
              onClick={() => { setEditMode(v => !v); setPolygon([]); setCursor(null); setPolyClosed(false); }}
              title="포인트 세그먼트 편집 (좌클릭으로 폴리곤, 우드래그로 회전)"
            >
              ✂ Edit
            </button>
          </>
        )}
        {editMode && (
          <>
            <button style={S.btn(false)} disabled={polygon.length < 3}
              onClick={() => applyDelete(true)} title="폴리곤 안쪽 점 삭제">안쪽 삭제</button>
            <button style={S.btn(false)} disabled={polygon.length < 3}
              onClick={() => applyDelete(false)} title="폴리곤 바깥쪽 점 삭제">바깥 삭제</button>
            <span style={{ opacity: 0.4 }}>|</span>
            <input type="color" value={paintColor} onChange={e => setPaintColor(e.target.value)}
              title="색칠 색상" style={{ width: 22, height: 18, padding: 0, border: "1px solid rgba(0,212,255,0.3)", background: "transparent", cursor: "pointer" }} />
            <button style={S.btn(false)} disabled={polygon.length < 3}
              onClick={() => applyPaint(true)} title="폴리곤 안쪽 점을 색칠">🎨 안쪽칠</button>
            <button style={S.btn(false)} disabled={polygon.length < 3}
              onClick={() => applyPaint(false)} title="폴리곤 바깥쪽 점을 색칠">🎨 바깥칠</button>
            {onPickHeightRange && (
              <button style={S.btn(false)} disabled={polygon.length < 3}
                onClick={sendHeightRange} title="폴리곤 안 라이다 점들의 z 최소~최대를 선택 캐리어 높이로">📐 z범위</button>
            )}
            {onKeepoutFromPolygon && (
              <button style={S.btn(false)} disabled={polygon.length < 3}
                onClick={sendKeepout} title="폴리곤을 바닥(z=0)에 투영해 맵의 진입금지 영역으로 보냄 (Top뷰 권장)">⛔ 진입금지로</button>
            )}
            {onSemanticFromPolygon && (
              <>
                <select value={semType} onChange={e => setSemType(e.target.value)} title="시맨틱 종류"
                  style={{ background: "rgba(0,0,0,0.4)", color: "#9fe", border: "1px solid rgba(0,212,255,0.3)", borderRadius: 3, fontSize: 10, height: 20 }}>
                  <option value="carrier">캐리어</option>
                  <option value="room">방</option>
                  <option value="object">객체</option>
                </select>
                <button style={S.btn(false)} disabled={polygon.length < 3}
                  onClick={sendSemantic} title="폴리곤을 바닥에 투영해 시맨틱 영역으로 보냄 (이름 입력 창이 뜸)">📦 시맨틱으로</button>
              </>
            )}
            <span style={{ opacity: 0.4 }}>|</span>
            <button style={S.btn(false)} disabled={!polygon.length}
              onClick={() => { setPolygon(p => p.slice(0, -1)); setPolyClosed(false); }} title="마지막 점 취소 (Backspace)">⮌ 점</button>
            <button style={S.btn(false)} disabled={!polygon.length}
              onClick={() => { setPolygon([]); setCursor(null); setPolyClosed(false); }} title="폴리곤 비우기 (Esc)">✕</button>
          </>
        )}
        {heightInfo && showHeight && (
          <>
            <button style={S.btn(false)} onClick={undoEdit} title="마지막 삭제 되돌리기">↶ Undo</button>
            <button style={S.btn(false)} onClick={saveHeightJson} title="편집 결과 JSON 저장">💾 Save</button>
          </>
        )}
      </div>
      <input
        ref={fileInputRef}
        type="file"
        accept=".json,application/json"
        style={{ display: "none" }}
        onChange={e => { loadHeightFile(e.target.files?.[0]); e.target.value = ""; }}
      />
      {showBuildPanel && onBuildFromBag && (
        <BuildPanel
          params={buildParams}
          setParams={setBuildParams}
          onPickBag={onPickBag}
          building={building}
          onClose={() => setShowBuildPanel(false)}
          onBuild={() => { setShowBuildPanel(false); onBuildFromBag(buildParams); }}
        />
      )}
      <div style={S.info}>
        <div>Frame: {fixedFrame || "map"}</div>
        <div>Pts: {lidarWorldPoints?.current?.length || 0}</div>
        {heightInfo && <div>Height: {heightInfo}</div>}
        {editMsg && <div style={{ color: "#ffd24a" }}>삭제: {editMsg}</div>}
      </div>
    </div>
  );
}
