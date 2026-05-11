import { useState, useEffect, useRef, useCallback, useMemo } from "react";
import Ros2Bridge, { STATES } from "./Ros2Bridge.js";

// ── Styles ──
const S = {
  section: { borderTop: "1px solid rgba(0,212,255,0.1)", paddingTop: 8, marginBottom: 8 },
  sectionLabel: { fontSize: 10, color: "rgba(0,212,255,0.5)", letterSpacing: 1, marginBottom: 5, display: "flex", alignItems: "center", justifyContent: "space-between" },
  row: { display: "flex", alignItems: "center", gap: 5, fontSize: 10 },
  label: { fontSize: 9, color: "#6a9aaa", marginBottom: 2 },
  input: {
    background: "rgba(0,0,0,0.45)", color: "#c9fffe",
    border: "1px solid rgba(0,212,255,0.2)", borderRadius: 4,
    padding: "4px 6px", fontSize: 11, fontFamily: "monospace", outline: "none",
  },
  select: {
    background: "rgba(0,0,0,0.45)", color: "#c9fffe",
    border: "1px solid rgba(0,212,255,0.2)", borderRadius: 4,
    padding: "3px 4px", fontSize: 10, fontFamily: "monospace", outline: "none",
  },
  btn: (active = false, danger = false) => ({
    background: danger ? "rgba(255,82,82,0.1)" : active ? "rgba(0,212,255,0.2)" : "rgba(0,212,255,0.07)",
    color: danger ? "#ff5252" : active ? "#00d4ff" : "#8eb8c8",
    border: `1px solid ${danger ? "rgba(255,82,82,0.3)" : active ? "rgba(0,212,255,0.5)" : "rgba(0,212,255,0.15)"}`,
    borderRadius: 4, padding: "3px 8px", cursor: "pointer", fontSize: 11,
    fontFamily: "'JetBrains Mono','Fira Code',monospace",
    display: "inline-flex", alignItems: "center", gap: 4, transition: "all 0.15s", whiteSpace: "nowrap",
  }),
};

const STATE_LABEL = { [STATES.DISCONNECTED]: "Disconnected", [STATES.CONNECTING]: "Connecting...", [STATES.CONNECTED]: "Connected" };
const STATE_COLOR = { [STATES.DISCONNECTED]: "#ff5252", [STATES.CONNECTING]: "#ffaa00", [STATES.CONNECTED]: "#00e676" };

const KNOWN_TYPES = [
  { type: "sensor_msgs/msg/LaserScan", label: "LaserScan", icon: "🔴", viz: "lidar" },
  { type: "sensor_msgs/msg/PointCloud2", label: "PointCloud2", icon: "🟣", viz: "lidar" },
  { type: "nav_msgs/msg/Odometry", label: "Odometry", icon: "🤖", viz: "pose" },
  { type: "geometry_msgs/msg/PoseStamped", label: "PoseStamped", icon: "📍", viz: "pose" },
  { type: "geometry_msgs/msg/PoseWithCovarianceStamped", label: "PoseWithCovariance", icon: "📍", viz: "pose" },
  { type: "tf2_msgs/msg/TFMessage", label: "TF", icon: "🔗", viz: "tf" },
  { type: "sensor_msgs/msg/CompressedImage", label: "CompressedImage", icon: "📷", viz: "camera" },
  { type: "sensor_msgs/msg/Image", label: "Image", icon: "📷", viz: "camera" },
  { type: "nav_msgs/msg/OccupancyGrid", label: "OccupancyGrid", icon: "🗺", viz: "map" },
  { type: "nav_msgs/msg/OccupancyGrid", label: "Costmap", icon: "▦", viz: "costmap" },
  { type: "nav_msgs/msg/Path", label: "Path", icon: "🛤", viz: "path" },
  { type: "geometry_msgs/msg/PolygonStamped", label: "Footprint", icon: "▱", viz: "footprint" },
  { type: "std_msgs/msg/String", label: "RobotDescription", icon: "▤", viz: "robot_model" },
  { type: "sensor_msgs/msg/Imu", label: "Imu", icon: "🧭", viz: "raw" },
  { type: "geometry_msgs/msg/Twist", label: "Twist", icon: "🕹", viz: "raw" },
  { type: "visualization_msgs/msg/Marker", label: "Marker", icon: "📌", viz: "raw" },
  { type: "visualization_msgs/msg/MarkerArray", label: "MarkerArray", icon: "📌", viz: "raw" },
];

const VIZ_DEFAULTS = {
  lidar: { color: "#ff3333", size: 2, alpha: 0.8, maxPoints: 30000, decay: 0 },
  pose:  { color: "#00e676", size: 5, alpha: 1.0 },
  tf:    { color: "#00e676", size: 5, alpha: 1.0 },
  path:  { color: "#ffaa00", size: 1.5, alpha: 0.6 },
  costmap: { color: "#ff6680", size: 1, alpha: 0.6 },
  footprint: { color: "#00bcd4", size: 2, alpha: 0.85 },
  robot_model: { color: "#b5f5ff", size: 1.5, alpha: 0.9 },
  camera:{ color: "#00d4ff", alpha: 1.0 },
  map:   { color: "#80deea", alpha: 0.7 },
  raw:   { color: "#9aa7b2", alpha: 0.7 },
};

function getTypeSuffix(topicType = "") {
  return topicType.split("/").pop() || "";
}

function detectViz(topicType = "", topicName = "") {
  const s = getTypeSuffix(topicType);
  if (s === "LaserScan" || s === "PointCloud2") return "lidar";
  if (s === "Odometry" || s === "PoseStamped" || s === "PoseWithCovarianceStamped") return "pose";
  if (s === "TFMessage") return "tf";
  if (s === "Path") return "path";
  if (s === "OccupancyGrid" && topicName.includes("costmap")) return "costmap";
  if (s === "PolygonStamped") return "footprint";
  if (s === "String" && topicName.includes("robot_description")) return "robot_model";
  if (s === "CompressedImage" || s === "Image") return "camera";
  if (s === "OccupancyGrid") return "map";
  return "raw";
}

function detectKnown(topicType = "", topicName = "") {
  const suffix = getTypeSuffix(topicType);
  if (suffix === "OccupancyGrid" && topicName.includes("costmap")) return KNOWN_TYPES.find(k => k.viz === "costmap");
  return KNOWN_TYPES.find(k => getTypeSuffix(k.type) === suffix);
}

// ── Display Item (rviz2-style expandable) ──
function DisplayItem({ topic, v, onChange, onRemove }) {
  const [expanded, setExpanded] = useState(false);
  const defaults = VIZ_DEFAULTS[v.viz] || {};

  const update = (key, val) => onChange(topic, { ...v, [key]: val });

  return (
    <div style={{ background: "rgba(0,0,0,0.2)", borderRadius: 4, marginBottom: 3, border: "1px solid rgba(0,212,255,0.08)" }}>
      {/* Header row */}
      <div style={{ display: "flex", alignItems: "center", gap: 4, padding: "4px 6px", cursor: "pointer" }}
        onClick={() => setExpanded(e => !e)}>
        <span style={{ fontSize: 8, color: "#4a7080", transform: expanded ? "rotate(90deg)" : "none", transition: "transform 0.15s" }}>▶</span>
        <input type="checkbox" checked={v.enabled !== false}
          onChange={e => { e.stopPropagation(); update("enabled", e.target.checked); }}
          style={{ width: 12, height: 12, accentColor: v.color, cursor: "pointer" }}
          onClick={e => e.stopPropagation()} />
        <span style={{ fontSize: 10 }}>{v.icon}</span>
        <span style={{ flex: 1, fontSize: 10, color: v.enabled !== false ? v.color : "#4a5060", overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>
          {topic}
        </span>
        <button style={{ background: "none", border: "none", color: "#ff5252", cursor: "pointer", fontSize: 10, padding: "0 2px", opacity: 0.6 }}
          onClick={e => { e.stopPropagation(); onRemove(topic); }}>✕</button>
      </div>

      {/* Expanded settings */}
      {expanded && (
        <div style={{ padding: "4px 8px 8px 22px", display: "flex", flexDirection: "column", gap: 4 }}>
          <div style={S.row}>
            <span style={S.label}>Topic</span>
            <span style={{ fontSize: 9, color: "#6a9aaa", flex: 1, overflow: "hidden", textOverflow: "ellipsis" }}>{topic}</span>
          </div>
          <div style={S.row}>
            <span style={S.label}>Type</span>
            <span style={{ fontSize: 9, color: "#4a7080" }}>{v.type.split("/").pop()}</span>
          </div>
          <div style={S.row}>
            <span style={S.label}>Color</span>
            <input type="color" value={v.color || defaults.color}
              onChange={e => update("color", e.target.value)}
              style={{ width: 24, height: 16, padding: 0, border: "none", background: "none", cursor: "pointer" }} />
            <span style={{ fontSize: 9, color: "#4a7080" }}>{v.color || defaults.color}</span>
          </div>
          <div style={S.row}>
            <span style={S.label}>Alpha</span>
            <input type="range" min={0.1} max={1} step={0.05} value={v.alpha ?? defaults.alpha ?? 0.8}
              onChange={e => update("alpha", parseFloat(e.target.value))}
              style={{ flex: 1, height: 12, accentColor: v.color }} />
            <span style={{ fontSize: 9, color: "#4a7080", width: 28, textAlign: "right" }}>{(v.alpha ?? defaults.alpha ?? 0.8).toFixed(2)}</span>
          </div>
          {(v.viz === "lidar") && (<>
            <div style={S.row}>
              <span style={S.label}>Size</span>
              <input type="range" min={1} max={8} step={0.5} value={v.size ?? defaults.size ?? 2}
                onChange={e => update("size", parseFloat(e.target.value))}
                style={{ flex: 1, height: 12, accentColor: v.color }} />
              <span style={{ fontSize: 9, color: "#4a7080", width: 20, textAlign: "right" }}>{v.size ?? defaults.size ?? 2}</span>
            </div>
            <div style={S.row}>
              <span style={S.label}>Max Points</span>
              <input type="number" min={1000} max={300000} step={1000} value={v.maxPoints ?? defaults.maxPoints ?? 30000}
                onChange={e => update("maxPoints", parseInt(e.target.value))}
                style={{ ...S.input, width: 70, padding: "2px 4px", fontSize: 10 }} />
            </div>
            <div style={S.row}>
              <span style={S.label}>Decay (s)</span>
              <input type="number" min={0} max={60} step={0.1} value={v.decay ?? defaults.decay ?? 0}
                onChange={e => update("decay", Math.max(0, parseFloat(e.target.value) || 0))}
                style={{ ...S.input, width: 70, padding: "2px 4px", fontSize: 10 }} />
              <span style={{ fontSize: 8, color: "#4a7080" }}>0 = latest only</span>
            </div>
          </>)}
          {(v.viz === "pose" || v.viz === "tf") && (
            <div style={S.row}>
              <span style={S.label}>Arrow Size</span>
              <input type="range" min={3} max={20} step={1} value={v.size ?? defaults.size ?? 5}
                onChange={e => update("size", parseFloat(e.target.value))}
                style={{ flex: 1, height: 12, accentColor: v.color }} />
              <span style={{ fontSize: 9, color: "#4a7080", width: 20, textAlign: "right" }}>{v.size ?? defaults.size ?? 5}</span>
            </div>
          )}
          {(v.viz === "path" || v.viz === "costmap" || v.viz === "footprint" || v.viz === "robot_model") && (
            <div style={S.row}>
              <span style={S.label}>Line Width</span>
              <input type="range" min={0.5} max={5} step={0.5} value={v.size ?? defaults.size ?? 1.5}
                onChange={e => update("size", parseFloat(e.target.value))}
                style={{ flex: 1, height: 12, accentColor: v.color }} />
              <span style={{ fontSize: 9, color: "#4a7080", width: 20, textAlign: "right" }}>{v.size ?? defaults.size ?? 1.5}</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
}

// ── TF Tree display ──
function TFTree({ availableFrames, stats }) {
  const [expanded, setExpanded] = useState(false);
  const chain = stats?.current?.tfChain || "";

  return (
    <div style={{ ...S.section }}>
      <div style={{ ...S.sectionLabel, cursor: "pointer" }} onClick={() => setExpanded(e => !e)}>
        <span>
          <span style={{ fontSize: 8, color: "#4a7080", marginRight: 4, display: "inline-block", transform: expanded ? "rotate(90deg)" : "none", transition: "transform 0.15s" }}>▶</span>
          TF Tree
        </span>
        <span style={{ fontSize: 9, color: stats?.current?.tfResolved ? "#00e676" : "#ff5252" }}>
          {stats?.current?.tfResolved ? "✓" : "✕"} {chain}
        </span>
      </div>
      {expanded && availableFrames && (
        <div style={{ fontSize: 9, color: "#6a9aaa", padding: "4px 0", maxHeight: 120, overflow: "auto" }}>
          {availableFrames.map(f => (
            <div key={f} style={{ padding: "1px 4px", display: "flex", alignItems: "center", gap: 4 }}>
              <span style={{ color: chain.includes(f) ? "#00e676" : "#4a7080" }}>⊙</span>
              <span style={{ color: chain.includes(f) ? "#c9fffe" : "#6a9aaa" }}>{f}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

// ══════════════════════════════════════════════════════════════════
// Main Panel
// ══════════════════════════════════════════════════════════════════
export default function Ros2Panel({ bridge, defaultUrl = "ws://localhost:9090", vis: externalVis, onVisChange, frames, onFramesChange, availableFrames, stats, meta, canvasSize, cameraDataUrl }) {
  const [url, setUrl] = useState(defaultUrl);
  const [connState, setConnState] = useState(bridge.state);
  const [topics, setTopics] = useState([]);
  const [loadingTopics, setLoadingTopics] = useState(false);
  const [topicFilter, setTopicFilter] = useState("");
  const [vis, setVis] = useState({});
  const [showTopicBrowser, setShowTopicBrowser] = useState(false);
  const [manualTopic, setManualTopic] = useState("");
  const [manualType, setManualType] = useState(KNOWN_TYPES[0].type);

  // Real-time stats refresh (refs don't trigger re-render)
  const [, forceUpdate] = useState(0);
  useEffect(() => {
    if (connState !== STATES.CONNECTED) return;
    const timer = setInterval(() => forceUpdate(v => v + 1), 200);
    return () => clearInterval(timer);
  }, [connState]);

  useEffect(() => {
    const unsub = bridge.onStateChange(s => setConnState(s));
    return unsub;
  }, [bridge]);

  useEffect(() => {
    setUrl(defaultUrl);
  }, [defaultUrl]);

  const fetchTopics = useCallback(async () => {
    if (!bridge.connected) return;
    setLoadingTopics(true);
    try {
      const result = await bridge.getTopics();
      if (result.topics && result.types) {
        const list = result.topics.map((name, i) => ({ name, type: result.types[i] }));
        list.sort((a, b) => a.name.localeCompare(b.name));
        setTopics(list);
      }
    } catch (e) {
      setTopics([]);
    }
    setLoadingTopics(false);
  }, [bridge]);

  useEffect(() => {
    if (connState === STATES.CONNECTED) fetchTopics();
    else setTopics([]);
  }, [connState, fetchTopics]);

  useEffect(() => {
    if (connState !== STATES.CONNECTED || !showTopicBrowser) return;
    const timer = setInterval(fetchTopics, 3000);
    return () => clearInterval(timer);
  }, [connState, showTopicBrowser, fetchTopics]);

  useEffect(() => {
    if (externalVis && externalVis !== vis) setVis(externalVis);
  }, [externalVis]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (externalVis && externalVis !== vis) return;
    if (onVisChange) onVisChange(vis);
  }, [vis, onVisChange, externalVis]);

  const filteredTopics = useMemo(() => {
    const q = topicFilter.trim().toLowerCase();
    if (!q) return topics;
    return topics.filter(t =>
      t.name.toLowerCase().includes(q) ||
      (t.type || "").toLowerCase().includes(q)
    );
  }, [topics, topicFilter]);

  // Add a topic as display
  const addDisplay = (topicName, topicType) => {
    if (vis[topicName]) return; // already exists
    const known = detectKnown(topicType, topicName);
    const vizType = detectViz(topicType, topicName);
    const defaults = VIZ_DEFAULTS[vizType] || {};
    setVis(prev => ({
      ...prev,
      [topicName]: {
        enabled: true, type: topicType, viz: vizType,
        icon: known?.icon || "📡",
        color: defaults.color || "#00d4ff",
        alpha: defaults.alpha ?? 0.8,
        size: defaults.size ?? 2,
        maxPoints: defaults.maxPoints,
        decay: defaults.decay ?? 0,
      },
    }));
  };

  const removeDisplay = (topic) => {
    setVis(prev => { const n = { ...prev }; delete n[topic]; return n; });
  };

  const updateDisplay = (topic, newVal) => {
    setVis(prev => ({ ...prev, [topic]: newVal }));
  };

  const addManualTopic = () => {
    if (!manualTopic.trim()) return;
    addDisplay(manualTopic.trim(), manualType);
    setManualTopic("");
  };

  const st = stats?.current || {};
  const cameraFrame = cameraDataUrl?.current || null;

  return (
    <div style={{ width: 300, background: "#070f1e", borderLeft: "1px solid rgba(0,212,255,0.12)", display: "flex", flexDirection: "column", flexShrink: 0, fontSize: 12, userSelect: "none" }}>

      {/* ── Header ── */}
      <div style={{ padding: "8px 12px", borderBottom: "1px solid rgba(0,212,255,0.12)", display: "flex", alignItems: "center", gap: 8 }}>
        <span style={{ color: "#00d4ff", fontWeight: "bold", letterSpacing: 1, fontSize: 12 }}>Displays</span>
        <div style={{ flex: 1 }} />
        <div style={{ width: 7, height: 7, borderRadius: "50%", background: STATE_COLOR[connState], boxShadow: `0 0 5px ${STATE_COLOR[connState]}` }} />
        <span style={{ fontSize: 9, color: STATE_COLOR[connState] }}>{STATE_LABEL[connState]}</span>
      </div>

      <div style={{ flex: 1, overflow: "auto", padding: "8px 10px" }}>

        {/* ── Global Options (rviz2 style) ── */}
        <div style={{ ...S.section, borderTop: "none", marginTop: 0 }}>
          <div style={S.sectionLabel}><span>Global Options</span></div>

          {/* Connection */}
          <div style={{ display: "flex", gap: 4, marginBottom: 6 }}>
            <input value={url} onChange={e => setUrl(e.target.value)}
              onKeyDown={e => e.key === "Enter" && bridge.connect(url)}
              style={{ ...S.input, flex: 1, fontSize: 10 }} placeholder={defaultUrl} />
            {connState !== STATES.CONNECTED ? (
              <button style={S.btn()} onClick={() => { bridge._autoReconnect = true; bridge.connect(url); }}>▶</button>
            ) : (
              <button style={S.btn(false, true)} onClick={() => bridge.disconnect()}>■</button>
            )}
          </div>

          {/* Fixed Frame */}
          <div style={{ marginBottom: 4 }}>
            <div style={S.label}>Fixed Frame</div>
            {availableFrames && availableFrames.length > 0 ? (
              <select value={frames?.fixed || "map"}
                onChange={e => onFramesChange({ ...frames, fixed: e.target.value })}
                style={{ ...S.select, width: "100%", boxSizing: "border-box" }}>
                {availableFrames.map(f => <option key={f} value={f}>{f}</option>)}
              </select>
            ) : (
              <input value={frames?.fixed || "map"}
                onChange={e => onFramesChange({ ...frames, fixed: e.target.value })}
                style={{ ...S.input, width: "100%", boxSizing: "border-box", fontSize: 10 }} placeholder="map" />
            )}
          </div>

          {/* Robot Frame — not needed, rviz2 style uses header.frame_id + TF tree */}
        </div>

        {/* ── Displays List ── */}
        <div style={S.section}>
          <div style={S.sectionLabel}>
            <span>Displays ({Object.keys(vis).length})</span>
            <button style={{ ...S.btn(), padding: "1px 6px", fontSize: 9 }}
              onClick={() => setShowTopicBrowser(v => !v)}>+ Add</button>
          </div>

          {Object.entries(vis).map(([topic, v]) => (
            <DisplayItem key={topic} topic={topic} v={v}
              onChange={updateDisplay} onRemove={removeDisplay} />
          ))}

          {Object.keys(vis).length === 0 && (
            <div style={{ fontSize: 10, color: "#3a5060", textAlign: "center", padding: 12 }}>
              No displays. Click "+ Add" to add topics.
            </div>
          )}
        </div>

        {/* ── Topic Browser (popup) ── */}
        {showTopicBrowser && connState === STATES.CONNECTED && (
          <div style={{ ...S.section }}>
            <div style={S.sectionLabel}>
              <span>Topic Browser</span>
              <div style={{ display: "flex", gap: 4 }}>
                <button style={{ ...S.btn(), padding: "1px 6px", fontSize: 9 }} onClick={fetchTopics}>
                  {loadingTopics ? "..." : "↻"}
                </button>
                <button style={{ ...S.btn(), padding: "1px 6px", fontSize: 9 }} onClick={() => setShowTopicBrowser(false)}>✕</button>
              </div>
            </div>

            <div style={{ marginBottom: 6 }}>
              <input
                value={topicFilter}
                onChange={e => setTopicFilter(e.target.value)}
                placeholder="Search topic/type..."
                style={{ ...S.input, width: "100%", boxSizing: "border-box", fontSize: 10 }}
              />
            </div>

            {filteredTopics.length > 0 ? (
              <div style={{ maxHeight: 180, overflow: "auto", display: "flex", flexDirection: "column", gap: 1 }}>
                {filteredTopics.map(t => {
                  const active = !!vis[t.name];
                  const known = detectKnown(t.type, t.name);
                  const vizType = detectViz(t.type, t.name);
                  return (
                    <div key={t.name}
                      onClick={() => !active && addDisplay(t.name, t.type)}
                      style={{
                        padding: "3px 6px", borderRadius: 3, cursor: !active ? "pointer" : "default",
                        background: active ? "rgba(0,212,255,0.08)" : "rgba(255,255,255,0.02)",
                        display: "flex", alignItems: "center", gap: 4,
                      }}>
                      <span style={{ fontSize: 10 }}>{known?.icon || "📡"}</span>
                      <div style={{ flex: 1, minWidth: 0 }}>
                        <div style={{ fontSize: 10, color: active ? "#00d4ff" : "#8eb8c8", overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>{t.name}</div>
                        <div style={{ fontSize: 8, color: "rgba(0,212,255,0.3)" }}>{t.type.split("/").pop()}</div>
                      </div>
                      <span style={{ fontSize: 8, color: vizType === "raw" ? "#ffaa00" : "#00e676" }}>
                        {vizType === "raw" ? "RAW" : "VIS"}
                      </span>
                      {active && <span style={{ fontSize: 8, color: "#00e676" }}>✓</span>}
                    </div>
                  );
                })}
              </div>
            ) : (
              <div style={{ color: "#3a5060", fontSize: 10, padding: 8, textAlign: "center" }}>
                {loadingTopics ? "Loading..." : "No topics / rosapi not available"}
              </div>
            )}

            {/* Manual add */}
            <div style={{ marginTop: 6, paddingTop: 6, borderTop: "1px solid rgba(0,212,255,0.08)" }}>
              <div style={S.label}>Manual Add</div>
              <div style={{ display: "flex", gap: 3 }}>
                <input value={manualTopic} onChange={e => setManualTopic(e.target.value)} placeholder="/velodyne_points"
                  onKeyDown={e => e.key === "Enter" && addManualTopic()}
                  style={{ ...S.input, flex: 1, fontSize: 10, padding: "3px 5px" }} />
              </div>
              <div style={{ display: "flex", gap: 3, marginTop: 3 }}>
                <select value={manualType} onChange={e => setManualType(e.target.value)}
                  style={{ ...S.select, flex: 1 }}>
                  {KNOWN_TYPES.map(k => <option key={k.type} value={k.type}>{k.icon} {k.label}</option>)}
                </select>
                <button style={{ ...S.btn(), padding: "2px 8px", fontSize: 9 }} onClick={addManualTopic}>Add</button>
              </div>
            </div>
          </div>
        )}

        {/* ── TF Tree ── */}
        {connState === STATES.CONNECTED && (
          <TFTree availableFrames={availableFrames} stats={stats} />
        )}

        {/* ── Camera / depth preview ── */}
        {connState === STATES.CONNECTED && cameraFrame && (
          <div style={{ ...S.section }}>
            <div style={S.sectionLabel}>
              <span>Camera Preview</span>
              <span style={{ fontSize: 8, color: "#4a7080", maxWidth: 145, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>
                {cameraFrame.topic}
              </span>
            </div>
            <div style={{ border: "1px solid rgba(0,212,255,0.12)", background: "rgba(0,0,0,0.35)", borderRadius: 4, padding: 4 }}>
              {cameraFrame.url ? (
                <img src={cameraFrame.url} alt="" style={{ display: "block", width: "100%", maxHeight: 180, objectFit: "contain", imageRendering: "auto" }} />
              ) : (
                <div style={{ minHeight: 88, display: "flex", alignItems: "center", justifyContent: "center", textAlign: "center", color: "#ff6680", fontSize: 10, lineHeight: 1.5, padding: 8 }}>
                  {cameraFrame.error || "image frame received, but preview decode failed"}
                </div>
              )}
              <div style={{ fontSize: 8, color: "#4a7080", marginTop: 3, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>
                {cameraFrame.encoding}{cameraFrame.width&&cameraFrame.height?` · ${cameraFrame.width}×${cameraFrame.height}`:""}{cameraFrame.bytes?` · ${cameraFrame.bytes} bytes`:""}
              </div>
            </div>
          </div>
        )}

        {/* ── Status Bar (rviz2 bottom info) ── */}
        {connState === STATES.CONNECTED && (
          <div style={{ ...S.section }}>
            <div style={S.sectionLabel}><span>Status</span></div>
            <div style={{ fontSize: 9, color: "#6a9aaa", display: "flex", flexDirection: "column", gap: 2, fontFamily: "monospace" }}>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>Robot</span>
                {st.robotPose ? (
                  <span style={{ color: "#c9fffe" }}>
                    ({st.robotPose.x.toFixed(2)}, {st.robotPose.y.toFixed(2)}) {(st.robotPose.theta * 180 / Math.PI).toFixed(1)}°
                  </span>
                ) : <span style={{ color: "#3a5060" }}>—</span>}
              </div>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>Lidar</span>
                <span style={{ color: st.lidarCount > 0 ? "#00d4ff" : "#3a5060" }}>{st.lidarCount || 0} pts</span>
              </div>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>Lidar T</span>
                <span style={{ color: (st.lidarTopicCount || 0) > 0 ? "#00d4ff" : "#3a5060" }}>{st.lidarTopicCount || 0}</span>
              </div>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>Pose T</span>
                <span style={{ color: (st.poseTopicCount || 0) > 0 ? "#00e676" : "#3a5060" }}>{st.poseTopicCount || 0}</span>
              </div>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>Path T</span>
                <span style={{ color: (st.pathTopicCount || 0) > 0 ? "#ffaa00" : "#3a5060" }}>{st.pathTopicCount || 0}</span>
              </div>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>Costmap</span>
                <span style={{ color: (st.costmapTopicCount || 0) > 0 ? "#ff6680" : "#3a5060" }}>{st.costmapTopicCount || 0}</span>
              </div>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>Footprint</span>
                <span style={{ color: (st.footprintTopicCount || 0) > 0 ? "#00bcd4" : "#3a5060" }}>{st.footprintTopicCount || 0}</span>
              </div>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>Camera</span>
                <span style={{ color: (st.cameraTopicCount || 0) > 0 ? "#00d4ff" : "#3a5060" }}>{st.cameraTopicCount || 0}</span>
              </div>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>Robot</span>
                <span style={{ color: (st.robotModelLinkCount || 0) > 0 ? "#b5f5ff" : "#3a5060" }}>
                  {st.robotModelLinkCount ? `${st.robotModelLinkCount} links` : st.robotDescriptionLoaded ? "URDF loaded" : "—"}
                </span>
              </div>
              <div style={S.row}>
                <span style={{ color: "#4a7080", width: 55 }}>TF</span>
                <span style={{ color: st.tfResolved ? "#00e676" : "#ff5252" }}>{st.tfResolved ? "OK" : "NOT RESOLVED"}</span>
              </div>
              {meta && (
                <div style={S.row}>
                  <span style={{ color: "#4a7080", width: 55 }}>Map</span>
                  <span style={{ color: "#6a9aaa" }}>{canvasSize?.w}×{canvasSize?.h} @ {meta.resolution}m/px</span>
                </div>
              )}
              {meta && (
                <div style={S.row}>
                  <span style={{ color: "#4a7080", width: 55 }}>Origin</span>
                  <span style={{ color: "#6a9aaa" }}>[{meta.origin?.map(v => v.toFixed(2)).join(", ")}]</span>
                </div>
              )}
              {st.lastError && (
                <div style={{ color: "#ff5252", fontSize: 8, marginTop: 2, wordBreak: "break-all" }}>⚠ {st.lastError}</div>
              )}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
