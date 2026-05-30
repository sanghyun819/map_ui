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
    top: 10,
    left: 10,
    display: "flex",
    gap: 6,
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

  useEffect(() => { showHeightRef.current = showHeight; }, [showHeight]);
  useEffect(() => { zScaleRef.current = zScale; }, [zScale]);
  useEffect(() => { ptSizeRef.current = ptSize; }, [ptSize]);
  useEffect(() => { editModeRef.current = editMode; }, [editMode]);

  const loadHeightFile = (file) => {
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const json = JSON.parse(reader.result);
        const data = parseHeightMap(json);
        if (!data) { setHeightInfo("로드 실패: 빈 데이터"); return; }
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
      } catch (err) {
        console.error("[3D] height map parse error:", err);
        setHeightInfo("로드 실패: JSON 오류");
      }
    };
    reader.readAsText(file);
  };

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
    // Edit mode: left-click drops a lasso vertex; other buttons still drive the camera.
    if (editModeRef.current && e.button === 0) {
      setPolygon(p => [...p, localXY(e)]);
      return;
    }
    dragRef.current = {
      btn: e.button,
      x: e.clientX,
      y: e.clientY,
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

  const onMouseUp = () => { dragRef.current = null; };

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
        style={{ ...S.canvas, cursor: editMode ? "crosshair" : "default" }}
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
            points={[...polygon, ...(cursor ? [cursor] : []), polygon[0]]
              .map(p => `${p.x},${p.y}`).join(" ")}
            fill="rgba(0,212,255,0.12)" stroke="#00d4ff" strokeWidth="1.5" strokeDasharray="5 4"
          />
          {polygon.map((p, i) => (
            <circle key={i} cx={p.x} cy={p.y} r="3" fill="#00d4ff" />
          ))}
        </svg>
      )}
      <div style={S.hud}>
        <span style={{ color: "#00d4ff", fontWeight: "bold" }}>3D</span>
        <button style={S.btn(viewMode === "free")} onClick={() => onChangeViewMode?.("free")}>Free</button>
        <button style={S.btn(viewMode === "top")} onClick={() => onChangeViewMode?.("top")}>Top</button>
        <span style={{ opacity: 0.4 }}>|</span>
        <button style={S.btn(false)} onClick={() => fileInputRef.current?.click()} title="height_view3d.json 로드">Height ⬆</button>
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
              onClick={() => { setEditMode(v => !v); setPolygon([]); setCursor(null); }}
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
            <button style={S.btn(false)} disabled={!polygon.length}
              onClick={() => { setPolygon(p => p.slice(0, -1)); }} title="마지막 점 취소">⮌ 점</button>
            <button style={S.btn(false)} disabled={!polygon.length}
              onClick={() => { setPolygon([]); setCursor(null); }} title="폴리곤 비우기">✕</button>
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
      <div style={S.info}>
        <div>Frame: {fixedFrame || "map"}</div>
        <div>Pts: {lidarWorldPoints?.current?.length || 0}</div>
        {heightInfo && <div>Height: {heightInfo}</div>}
        {editMsg && <div style={{ color: "#ffd24a" }}>삭제: {editMsg}</div>}
      </div>
    </div>
  );
}
