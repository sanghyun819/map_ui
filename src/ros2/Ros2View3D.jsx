import { useEffect, useMemo, useRef } from "react";
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
    let lineProgram;
    let mapPosBuf;
    let mapUvBuf;
    let mapIdxBuf;
    let mapTex;
    let ptsBuf;
    let lineBuf;
    let anim = 0;

    try {
      mapProgram = createProgram(gl, MAP_VS, MAP_FS);
      ptsProgram = createProgram(gl, PTS_VS, PTS_FS);
      lineProgram = createProgram(gl, LINE_VS, LINE_FS);
    } catch (e) {
      console.error("[3D] shader/program error:", e);
      return;
    }

    mapPosBuf = gl.createBuffer();
    mapUvBuf = gl.createBuffer();
    mapIdxBuf = gl.createBuffer();
    ptsBuf = gl.createBuffer();
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
      gl.deleteBuffer(lineBuf);
      gl.deleteTexture(mapTex);
      gl.deleteProgram(mapProgram);
      gl.deleteProgram(ptsProgram);
      gl.deleteProgram(lineProgram);
    };
  }, [mapCanvasRef, mapGeom, lidarWorldPoints, pathWorldPoints, stats, viewMode]);

  const onMouseDown = (e) => {
    e.preventDefault();
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
        style={S.canvas}
        onMouseDown={onMouseDown}
        onMouseMove={onMouseMove}
        onMouseUp={onMouseUp}
        onMouseLeave={onMouseUp}
        onWheel={onWheel}
        onContextMenu={e => e.preventDefault()}
      />
      <div style={S.hud}>
        <span style={{ color: "#00d4ff", fontWeight: "bold" }}>3D</span>
        <button style={S.btn(viewMode === "free")} onClick={() => onChangeViewMode?.("free")}>Free</button>
        <button style={S.btn(viewMode === "top")} onClick={() => onChangeViewMode?.("top")}>Top</button>
      </div>
      <div style={S.info}>
        <div>Frame: {fixedFrame || "map"}</div>
        <div>Pts: {lidarWorldPoints?.current?.length || 0}</div>
      </div>
    </div>
  );
}
