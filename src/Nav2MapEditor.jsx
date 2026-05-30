import { useState, useRef, useEffect, useCallback, useMemo } from "react";
import Ros2Bridge, { STATES as ROS2_STATES, pixelToWorld, worldToPixel } from "./ros2/Ros2Bridge.js";
import Ros2Panel from "./ros2/Ros2Panel.jsx";
import useRos2Overlay from "./ros2/useRos2Overlay.js";
import Ros2View3D from "./ros2/Ros2View3D.jsx";

// ─── Host API detection (Electron or robot backend) ────────────────────────────
const isElectron = !!(window.electronAPI?.isElectron);
const robotBackendEnabled = !isElectron && (
  !!window.__ROBOT_BACKEND_URL__ ||
  import.meta.env.VITE_ROBOT_BACKEND === "1" ||
  window.location.port === "8787"
);
const robotBackendBase = (() => {
  if (isElectron) return "";
  const configured = window.__ROBOT_BACKEND_URL__ || import.meta.env.VITE_ROBOT_BACKEND_URL;
  if (configured) return String(configured).replace(/\/+$/, "");
  const port = import.meta.env.VITE_ROBOT_BACKEND_PORT || "8787";
  return `${window.location.protocol}//${window.location.hostname}:${port}`;
})();

function base64ToArrayBuffer(base64) {
  const binary = atob(base64 || "");
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
  return bytes.buffer;
}

function bytesToBase64(data) {
  const bytes = data instanceof Uint8Array
    ? data
    : data instanceof ArrayBuffer
      ? new Uint8Array(data)
      : ArrayBuffer.isView(data)
        ? new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
        : new TextEncoder().encode(String(data ?? ""));
  let binary = "";
  const chunk = 0x8000;
  for (let i = 0; i < bytes.length; i += chunk) {
    binary += String.fromCharCode(...bytes.subarray(i, i + chunk));
  }
  return btoa(binary);
}

function createRobotBackendAPI(baseUrl) {
  const post = async (path, body = {}) => {
    const res = await fetch(`${baseUrl}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const json = await res.json().catch(() => ({}));
    if (!res.ok || json.ok === false) throw new Error(json.error || `${res.status} ${res.statusText}`);
    return json.data ?? json;
  };
  const get = async (path) => {
    const res = await fetch(`${baseUrl}${path}`);
    const json = await res.json().catch(() => ({}));
    if (!res.ok || json.ok === false) throw new Error(json.error || `${res.status} ${res.statusText}`);
    return json.data ?? json;
  };
  return {
    isRobotBackend: true,
    openFileDialog: async () => null,
    saveFileDialog: async () => null,
    readFile: async (filePath, encoding) => {
      const data = await post("/api/file/read", { path: filePath, encoding: encoding || null });
      return data.binary ? base64ToArrayBuffer(data.data) : data.data;
    },
    writeFile: async (filePath, data, encoding) => {
      const binary = !encoding && (data instanceof Uint8Array || data instanceof ArrayBuffer || ArrayBuffer.isView(data));
      return post("/api/file/write", {
        path: filePath,
        encoding: encoding || null,
        binary,
        data: binary ? bytesToBase64(data) : String(data ?? ""),
      });
    },
    copySemanticToThor: (options) => post("/api/thor/copy-semantic", options),
    listThorMdFiles: (options) => post("/api/thor/list-md", options),
    listThorMdDir: (options) => post("/api/thor/list-md-dir", options),
    readThorMdFile: (options) => post("/api/thor/read-md", options),
    readDir: (dirPath) => post("/api/fs/readdir", { path: dirPath }),
    browseDir: (dirPath) => post("/api/fs/readdir", { path: dirPath, withFileTypes: true }),
    fsRoots: () => get("/api/fs/roots"),
    updateLaunchMap: (options) => post("/api/workspace/update-launch-map", options),
    buildWorkspace: (options) => post("/api/workspace/build", options),
    rosbridgeStart: (options) => post("/api/rosbridge/start", options),
    rosbridgeStop: () => post("/api/rosbridge/stop"),
    rosbridgeStatus: () => get("/api/rosbridge/status"),
    slamStart: (options) => post("/api/slam/start", options),
    slamStop: () => post("/api/slam/stop"),
    slamStatus: () => get("/api/slam/status"),
    nav2Start: (options) => post("/api/nav2/start", options),
    nav2Stop: () => post("/api/nav2/stop"),
    nav2Status: () => get("/api/nav2/status"),
    rosbagPlay: (options) => post("/api/rosbag/play", options),
    rosbagInfo: (bagPath) => post("/api/rosbag/info", { path: bagPath }),
    rosbagStop: () => post("/api/rosbag/stop"),
    rosbagPause: () => post("/api/rosbag/pause"),
    rosbagResume: () => post("/api/rosbag/resume"),
    rosbagSeek: (options) => post("/api/rosbag/seek", options),
    rosbagStatus: () => get("/api/rosbag/status"),
    rosbagRecordStart: (options) => post("/api/rosbag/record/start", options),
    rosbagRecordStop: () => post("/api/rosbag/record/stop"),
    rosbagRecordStatus: () => get("/api/rosbag/record/status"),
  };
}

const hostAPI = window.electronAPI || (robotBackendEnabled ? createRobotBackendAPI(robotBackendBase) : null);
const hasHostAPI = !!hostAPI;

function defaultRosbridgeUrl(port = 9090) {
  if (!hostAPI?.isRobotBackend) return `ws://localhost:${port}`;
  try {
    const u = new URL(robotBackendBase);
    const protocol = u.protocol === "https:" ? "wss" : "ws";
    return `${protocol}://${u.hostname}:${port}`;
  } catch (e) {
    return `ws://${window.location.hostname}:${port}`;
  }
}

// Native/robot-host file save helper with browser download fallback
async function nativeSave(defaultName, filters, data, encoding, pickPath) {
  if (hasHostAPI) {
    const filePath = await (pickPath || hostAPI.saveFileDialog)({
      dialogType: "save",
      defaultPath: defaultName,
      filters: filters,
    });
    if (!filePath) return false;
    await hostAPI.writeFile(filePath, data, encoding);
    return filePath;
  }
  // Browser fallback
  const blob = data instanceof Uint8Array
    ? new Blob([data], { type: "application/octet-stream" })
    : new Blob([data], { type: "text/plain" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = defaultName;
  a.click();
  return defaultName;
}

// Native/robot-host file open helper
async function nativeOpen(filters, pickPath) {
  if (hasHostAPI) {
    const filePath = await (pickPath || hostAPI.openFileDialog)({
      filters: filters,
      properties: ["openFile", "multiSelections"],
    });
    if (!filePath) return null;
    const firstPath = Array.isArray(filePath) ? filePath[0] : filePath;
    const name = firstPath.split("/").pop();
    const ext = name.split(".").pop().toLowerCase();
    const encoding = (ext === "pgm") ? null : "utf-8";
    const raw = await hostAPI.readFile(firstPath, encoding);
    return [{ name, path: firstPath, data: raw }];
  }
  return null;
}

// ─── Nav2 pixel constants ──────────────────────────────────────────────────────
const PX_OCCUPIED = 0, PX_FREE = 254, PX_UNKNOWN = 205;
const SNAP_RADIUS = 5;
const POLY_VERTEX_RADIUS = 1.4;
const POLY_FIRST_VERTEX_RADIUS = 2.1;
const POLY_SNAP_RING_RADIUS = 4.5;
const START_TASKS = ["HRI","PnP","GPSR","DL","Restaurant"];
const DEFAULT_START_TASK = START_TASKS[0];
const QUICK_MAP_DIR = "/home/nvidia/rby1_nav2/src/rby1_nav2/maps";
const QUICK_MAP_NAMES = {1:"map_first",2:"map_second"};
const DEFAULT_WORKSPACE_ROOT = "/home/nvidia/rby1_nav2";
const DEFAULT_MAP_SEARCH_DIR = "/home/nvidia/rby1_nav2/src/rby1_nav2/maps";
const DEFAULT_BAG_RECORD_DIR = "/home/nvidia/rby1_nav2/src/rby1_nav2/bag";
const DEFAULT_THOR_MD_DIR = import.meta.env.VITE_THOR_MD_DIR || "/home/thor/inha_ws/arena_info";
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

function startTaskFromId(startPoses,id){
  const value=String(id||"");
  return Object.entries(startPoses||{}).find(([,pose])=>pose?.id===value)?.[0]||null;
}
function nextStartPoseId(startPoses){
  const used=new Set(Object.values(startPoses||{}).map(p=>p?.id).filter(Boolean));
  let max=-1;
  used.forEach(id=>{const m=String(id).match(/^s(\d+)$/);if(m)max=Math.max(max,Number(m[1]));});
  let i=max+1;
  while(used.has(`s${i}`))i++;
  return `s${i}`;
}
function nextStartTask(startPoses,selected=DEFAULT_START_TASK){
  return START_TASKS.find(task=>!startPoses?.[task]) || (START_TASKS.includes(selected)?selected:DEFAULT_START_TASK);
}

// ─── Semantic type definitions ─────────────────────────────────────────────────
const MAP_TYPES = [
  { id:"map", label:"map", icon:"🗺", color:"#90a4ae", alpha:0.08 },
];
const ROOM_TYPES = [
  { id:"kitchen",      label:"kitchen",      icon:"▣", color:"#ff8c00", alpha:0.18 },
  { id:"living_room",  label:"living room",  icon:"▣", color:"#7b68ee", alpha:0.18 },
  { id:"bedroom",      label:"bedroom",      icon:"▣", color:"#4a90d9", alpha:0.18 },
  { id:"laundry_room", label:"laundry room", icon:"▣", color:"#20b2aa", alpha:0.18 },
  { id:"custom",      label:"custom",   icon:"◈",   color:"#aaaaaa", alpha:0.15 },
];
const CARRIER_TYPES = [
  { id:"desk",      label:"desk",      icon:"🖥",  color:"#4fc3f7", alpha:0.15 },
  { id:"table",     label:"table",     icon:"📋",  color:"#fff176", alpha:0.15 },
  { id:"shelf",     label:"shelf",     icon:"📚",  color:"#ce93d8", alpha:0.15 },
  { id:"cabinet",   label:"cabinet",   icon:"🗄",  color:"#bcaaa4", alpha:0.15 },
  { id:"counter",   label:"counter",   icon:"▬",   color:"#ffb74d", alpha:0.15 },
  { id:"sofa",      label:"sofa",      icon:"🛋",  color:"#a5d6a7", alpha:0.15 },
  { id:"bed",       label:"bed",       icon:"🛏",  color:"#f48fb1", alpha:0.15 },
  { id:"chair",     label:"chair",     icon:"🪑",  color:"#81c784", alpha:0.15 },
  { id:"rack",      label:"rack",      icon:"🔲",  color:"#90caf9", alpha:0.15 },
  { id:"custom",    label:"custom",    icon:"◎",   color:"#cccccc", alpha:0.15 },
];
const OBJECT_TYPES = [
  { id:"monitor",   label:"monitor",  icon:"📺",  color:"#90caf9", point:false },
  { id:"charger",   label:"charger",  icon:"⚡",  color:"#ffee58", point:true  },
  { id:"plant",     label:"plant",    icon:"🌱",  color:"#69f0ae", point:true  },
  { id:"food",      label:"food",     icon:"🍎",  color:"#ff8a65", point:true  },
  { id:"drink",     label:"drink",    icon:"🥤",  color:"#4dd0e1", point:true  },
  { id:"book",      label:"book",     icon:"📖",  color:"#ce93d8", point:true  },
  { id:"laptop",    label:"laptop",   icon:"💻",  color:"#90caf9", point:true  },
  { id:"tool",      label:"tool",     icon:"🔧",  color:"#b0bec5", point:true  },
  { id:"box",       label:"box",      icon:"📦",  color:"#bcaaa4", point:false },
  { id:"door",      label:"door",     icon:"🚪",  color:"#ffb74d", point:false },
  { id:"window",    label:"window",   icon:"🪟",  color:"#80deea", point:false },
  { id:"obstacle",  label:"obstacle", icon:"⬛",  color:"#ff5252", point:false },
  { id:"custom",    label:"custom",   icon:"◎",   color:"#eeeeee", point:true  },
];
const EMPTY_SEAT_TYPE = { id:"empty_seat", label:"empty seat", icon:"▧", color:"#00e676", alpha:0.13 };
const CATALOG_COLORS=["#4fc3f7","#ffb74d","#ce93d8","#81c784","#90caf9","#f48fb1","#fff176","#80deea","#bcaaa4","#a5d6a7"];
const DEFAULT_SEMANTIC_CATALOG={rooms:["kitchen","living room","bedroom","laundry room"],locations:[],objectClasses:[]};
let _catalogSourceIdx=0;

function catalogId(name){
  return String(name||"").toLowerCase().replace(/\(p\)/gi,"").replace(/[^a-z0-9]+/g,"_").replace(/^_+|_+$/g,"")||"custom";
}
function stripPlaceable(name){return String(name||"").replace(/\s*\(p\)\s*/gi,"").trim();}
function mergeById(base,extra){
  const seen=new Set(base.map(x=>x.id));
  const out=[...base];
  extra.forEach(x=>{if(x?.id&&!seen.has(x.id)){seen.add(x.id);out.push(x);}});
  return out;
}
function markdownSection(text,heading){
  const re=new RegExp(`^##\\s+${heading}\\s*$`,"im");
  const m=text.match(re);
  if(!m)return "";
  const start=m.index+m[0].length;
  const rest=text.slice(start);
  const next=rest.search(/^#{1,2}\s+/m);
  return next>=0?rest.slice(0,next):rest;
}
function tableRows(section){
  return section.split("\n")
    .map(line=>line.trim())
    .filter(line=>line.startsWith("|"))
    .map(line=>line.split("|").slice(1,-1).map(c=>c.trim()))
    .filter(cols=>cols.length>0&&!cols.every(c=>/^:?-+:?$/.test(c))&&!cols[0].match(/^name$|^number$/i));
}
function parseSemanticMarkdown(text){
  const catalog={rooms:[],locations:[],objectClasses:[]};
  tableRows(markdownSection(text,"Rooms")).forEach(cols=>{
    const name=cols[0]?.trim();
    if(name)catalog.rooms.push(name);
  });
  tableRows(markdownSection(text,"Locations")).forEach(cols=>{
    const nameRaw=cols[1]||cols[0];
    const label=stripPlaceable(nameRaw);
    if(!label||/^name$/i.test(label))return;
    catalog.locations.push({
      number:Number(cols[0])||null,
      label,
      placeable:/\(p\)/i.test(nameRaw),
      objectCategory:(cols[2]||"").trim()||null,
    });
  });
  const classRe=/^#\s+Class\s+([^(]+?)(?:\s*\(([^)]+)\))?\s*$/gim;
  let match;
  while((match=classRe.exec(text))){
    const classLabel=match[1].trim();
    const classType=(match[2]||classLabel).trim();
    const start=match.index+match[0].length;
    const rest=text.slice(start);
    const next=rest.search(/^#\s+/m);
    const section=next>=0?rest.slice(0,next):rest;
    const objects=tableRows(section).map(cols=>{
      const objectName=(cols[0]||"").trim();
      const img=(cols[1]||"").match(/!\[[^\]]*\]\(([^)]+)\)/)?.[1]||"";
      return objectName?{name:objectName,image:img}:null;
    }).filter(Boolean);
    if(objects.length)catalog.objectClasses.push({label:classLabel,type:classType,objects});
  }
  return catalog;
}
function buildRoomTypes(catalog){
  const rooms=(catalog?.rooms?.length?catalog.rooms:DEFAULT_SEMANTIC_CATALOG.rooms).map((name,i)=>({
    id:catalogId(name),label:name,icon:"▣",color:CATALOG_COLORS[i%CATALOG_COLORS.length],alpha:0.18,
  }));
  return mergeById(rooms,[ROOM_TYPES[ROOM_TYPES.length-1]]);
}
function buildCarrierTypes(catalog){
  if(!catalog?.locations?.length)return CARRIER_TYPES;
  const locs=catalog.locations.map((loc,i)=>({
    id:catalogId(loc.label),label:loc.label,icon:loc.placeable?"▤":"▥",color:CATALOG_COLORS[i%CATALOG_COLORS.length],alpha:0.15,
    catalog:loc,
  }));
  return mergeById(locs,[CARRIER_TYPES[CARRIER_TYPES.length-1]]);
}
function buildObjectTypes(catalog){
  if(!catalog?.objectClasses?.length)return OBJECT_TYPES;
  const objs=[];
  catalog.objectClasses.forEach((cls,ci)=>{
    cls.objects.forEach(obj=>{
      objs.push({
        id:catalogId(obj.name),label:obj.name,icon:"◎",color:CATALOG_COLORS[ci%CATALOG_COLORS.length],point:true,
        objectClass:cls.label,objectType:cls.type,image:obj.image,
      });
    });
  });
  return mergeById(objs,[OBJECT_TYPES[OBJECT_TYPES.length-1]]);
}
function normalizeRoomNames(rooms,fallback=DEFAULT_SEMANTIC_CATALOG.rooms){
  const out=[];
  const seen=new Set();
  (rooms||[]).forEach(room=>{
    const name=String(room||"").trim();
    const id=catalogId(name);
    if(!name||seen.has(id))return;
    seen.add(id);
    out.push(name);
  });
  if(out.length||fallback==null)return out;
  return normalizeRoomNames(fallback,null);
}
function catalogRoomNames(sources){
  return normalizeRoomNames((sources||[]).flatMap(src=>src?.catalog?.rooms||[]),null);
}
function mergeSemanticCatalog(current,next,{rooms=false}={}){
  const roomSet=new Set(current.rooms||[]);
  if(rooms)(next.rooms||[]).forEach(r=>roomSet.add(r));
  const locMap=new Map((current.locations||[]).map(x=>[catalogId(x.label),x]));
  (next.locations||[]).forEach(x=>locMap.set(catalogId(x.label),x));
  const classMap=new Map((current.objectClasses||[]).map(x=>[catalogId(x.label),{...x,objects:[...(x.objects||[])]}]));
  (next.objectClasses||[]).forEach(cls=>{
    const key=catalogId(cls.label);
    const prev=classMap.get(key)||{...cls,objects:[]};
    const objMap=new Map((prev.objects||[]).map(o=>[catalogId(o.name),o]));
    (cls.objects||[]).forEach(o=>objMap.set(catalogId(o.name),o));
    classMap.set(key,{...cls,objects:[...objMap.values()]});
  });
  return{rooms:[...roomSet],locations:[...locMap.values()],objectClasses:[...classMap.values()]};
}
function catalogSourceId(){
  return `md${Date.now().toString(36)}_${++_catalogSourceIdx}`;
}
function defaultSemanticCatalog(){
  return {
    rooms:normalizeRoomNames(DEFAULT_SEMANTIC_CATALOG.rooms),
    locations:[],
    objectClasses:[],
  };
}
function catalogCounts(catalog){
  return {
    rooms:catalog?.rooms?.length||0,
    locations:catalog?.locations?.length||0,
    classes:catalog?.objectClasses?.length||0,
    objects:(catalog?.objectClasses||[]).reduce((n,c)=>n+(c.objects?.length||0),0),
  };
}
function composeSemanticCatalog(sources,rooms=DEFAULT_SEMANTIC_CATALOG.rooms){
  const base={rooms:normalizeRoomNames(rooms),locations:[],objectClasses:[]};
  return (sources||[]).reduce((acc,src)=>mergeSemanticCatalog(acc,src.catalog||{}),base);
}

// ─── Geometry helpers ──────────────────────────────────────────────────────────
function hexRgba(hex, a) {
  const r=parseInt(hex.slice(1,3),16),g=parseInt(hex.slice(3,5),16),b=parseInt(hex.slice(5,7),16);
  return `rgba(${r},${g},${b},${a})`;
}
function pointInPoly(px, py, pts) {
  let inside = false;
  for (let i=0,j=pts.length-1; i<pts.length; j=i++) {
    const xi=pts[i].x,yi=pts[i].y,xj=pts[j].x,yj=pts[j].y;
    if (((yi>py)!==(yj>py))&&(px<(xj-xi)*(py-yi)/(yj-yi)+xi)) inside=!inside;
  }
  return inside;
}
function polyCentroid(pts) {
  let cx=0,cy=0; pts.forEach(p=>{cx+=p.x;cy+=p.y;});
  return {x:cx/pts.length, y:cy/pts.length};
}
function polyBBox(pts) {
  const xs=pts.map(p=>p.x), ys=pts.map(p=>p.y);
  return {x:Math.min(...xs),y:Math.min(...ys),x2:Math.max(...xs),y2:Math.max(...ys)};
}
function rectToPoly(x,y,w,h) {
  return [{x,y},{x:x+w,y},{x:x+w,y:y+h},{x,y:y+h}];
}
function shapeToPoly(shape) {
  if (shape.poly) return shape.poly;
  if (!shape.point && shape.w != null) return rectToPoly(shape.x,shape.y,shape.w,shape.h);
  return null;
}
function pointSegmentDistance(px,py,a,b){
  const vx=b.x-a.x,vy=b.y-a.y,wx=px-a.x,wy=py-a.y;
  const len2=vx*vx+vy*vy;
  const t=len2?Math.max(0,Math.min(1,(wx*vx+wy*vy)/len2)):0;
  const x=a.x+t*vx,y=a.y+t*vy;
  return Math.hypot(px-x,py-y);
}
function pointNearPolyEdge(poly,px,py,tolerance=4){
  if(!poly||poly.length<2)return false;
  for(let i=0;i<poly.length;i++){
    const a=poly[i],b=poly[(i+1)%poly.length];
    if(pointSegmentDistance(px,py,a,b)<=tolerance)return true;
  }
  return false;
}
function hitTestShape(shape, px, py) {
  if (shape.point) return Math.hypot(shape.x-px, shape.y-py) < 10;
  const poly = shapeToPoly(shape);
  if (!poly) return false;
  return pointInPoly(px, py, poly)||pointNearPolyEdge(poly,px,py);
}

// Check if child polygon is mostly (>=threshold) inside parent polygon
function polyMostlyInside(parentPoly, childPoly, threshold=0.8) {
  if(!parentPoly||!childPoly||childPoly.length===0)return false;
  const inside=childPoly.filter(p=>pointInPoly(p.x,p.y,parentPoly)).length;
  return inside/childPoly.length>=threshold;
}

// Move shape by delta
function moveShape(shape, dx, dy) {
  if (shape.point) return { ...shape, x: shape.x + dx, y: shape.y + dy };
  if (shape.poly) return { ...shape, poly: shape.poly.map(p => ({ x: p.x + dx, y: p.y + dy })) };
  if (shape.w != null) return { ...shape, x: shape.x + dx, y: shape.y + dy };
  return shape;
}

// ─── PGM / YAML helpers ────────────────────────────────────────────────────────
function parsePGM(buffer) {
  const bytes=new Uint8Array(buffer); let pos=0;
  const readToken=()=>{
    while(pos<bytes.length){while(pos<bytes.length&&bytes[pos]<=32)pos++;if(pos<bytes.length&&bytes[pos]===35){while(pos<bytes.length&&bytes[pos]!==10)pos++;}else break;}
    let t="";while(pos<bytes.length&&bytes[pos]>32)t+=String.fromCharCode(bytes[pos++]);return t;
  };
  const magic=readToken();
  if(magic!=="P5"&&magic!=="P2")throw new Error("지원하지 않는 PGM: "+magic);
  const width=parseInt(readToken()),height=parseInt(readToken());parseInt(readToken());
  if(magic==="P5"){if(pos<bytes.length&&bytes[pos]<=32)pos++;return{width,height,data:new Uint8Array(buffer.slice(pos,pos+width*height))};}
  const vals=[];while(vals.length<width*height){const t=readToken();if(t)vals.push(parseInt(t));}
  return{width,height,data:new Uint8Array(vals)};
}
function writePGM(w,h,gray){const hb=new TextEncoder().encode(`P5\n${w} ${h}\n255\n`);const out=new Uint8Array(hb.length+gray.length);out.set(hb);out.set(gray,hb.length);return out;}
function canvasToPGM(canvas){
  const imgData=canvas.getContext("2d").getImageData(0,0,canvas.width,canvas.height);
  const gray=new Uint8Array(canvas.width*canvas.height);
  for(let i=0;i<gray.length;i++)gray[i]=imgData.data[i*4];
  return writePGM(canvas.width,canvas.height,gray);
}
function stripYAMLComment(value){
  let quote=null;
  for(let i=0;i<value.length;i++){
    const ch=value[i];
    if((ch==="\""||ch=="'")&&value[i-1]!=="\\"){quote=quote===ch?null:quote||ch;}
    if(ch==="#"&&!quote&&(i===0||/\s/.test(value[i-1])))return value.slice(0,i);
  }
  return value;
}
function cleanYAMLScalar(value){
  const v=stripYAMLComment(value).trim();
  if((v.startsWith("\"")&&v.endsWith("\""))||(v.startsWith("'")&&v.endsWith("'")))return v.slice(1,-1);
  return v;
}
function basenameFromPath(path){
  return (path||"").replace(/\\/g,"/").split("/").pop();
}
function dirnameFromPath(path){
  const clean=(path||"").replace(/\\/g,"/").replace(/\/+$/,"");
  if(!clean)return "";
  const idx=clean.lastIndexOf("/");
  if(idx<=0)return clean.startsWith("/")?"/":"";
  return clean.slice(0,idx);
}
function joinRobotPath(dir,name){
  if(!dir||dir==="/")return `/${name}`;
  return `${dir.replace(/\/+$/,"")}/${name}`;
}
function parentRobotPath(dir){
  const clean=(dir||"/").replace(/\\/g,"/").replace(/\/+$/,"")||"/";
  if(clean==="/")return "/";
  return dirnameFromPath(clean)||"/";
}
function parentRelativePath(path){
  const parts=String(path||"").replace(/\\/g,"/").split("/").filter(Boolean);
  parts.pop();
  return parts.join("/");
}
function pathLooksAbsolute(path){
  return /^\//.test(path||"")||/^[A-Za-z]:[\\/]/.test(path||"");
}
function resolveMapPath(dir, relPath){
  const clean=cleanYAMLScalar(relPath||"");
  if(!clean)return "";
  if(clean.startsWith("/")||/^[A-Za-z]:[\\/]/.test(clean))return clean;
  const prefix=dir.startsWith("/")?"/":"";
  const parts=`${dir}/${clean}`.replace(/\\/g,"/").split("/");
  const stack=[];
  for(const part of parts){
    if(!part||part===".")continue;
    if(part==="..")stack.pop();
    else stack.push(part);
  }
  return prefix+stack.join("/");
}
function inferWorkspaceRootFromPackagePath(filePath){
  const normalized=String(filePath||"").replace(/\\/g,"/");
  if(!normalized)return "";
  const srcIdx=normalized.indexOf("/src/");
  return srcIdx>0?normalized.slice(0,srcIdx):"";
}
function inferPackageNameFromPackagePath(filePath){
  const normalized=String(filePath||"").replace(/\\/g,"/");
  if(!normalized)return "";
  const parts=normalized.split("/").filter(Boolean);
  const srcIdx=parts.lastIndexOf("src");
  return srcIdx>=0&&srcIdx+1<parts.length?parts[srcIdx+1]||"":"";
}
function inferWorkspaceRootFromLaunchPath(launchPath){
  const normalized=String(launchPath||"").replace(/\\/g,"/");
  if(!normalized)return "";
  const packageRoot=inferWorkspaceRootFromPackagePath(normalized);
  if(packageRoot)return packageRoot;
  const launchIdx=normalized.lastIndexOf("/launch/");
  if(launchIdx>0)return normalized.slice(0,launchIdx);
  return dirnameFromPath(dirnameFromPath(normalized))||dirnameFromPath(normalized)||"";
}
function inferPackageNameFromLaunchPath(launchPath){
  const normalized=String(launchPath||"").replace(/\\/g,"/");
  if(!normalized)return "";
  const parts=normalized.split("/").filter(Boolean);
  const launchIdx=parts.lastIndexOf("launch");
  if(launchIdx>0)return parts[launchIdx-1]||"";
  return inferPackageNameFromPackagePath(normalized);
}
function inferLaunchMapArgName(text){
  const source=String(text||"");
  for(const name of ["map","map_yaml","map_file","yaml_filename"]){
    const quoted=`['"]${name}['"]`;
    if(new RegExp(`DeclareLaunchArgument\\s*\\(\\s*${quoted}`,"s").test(source))return name;
    if(new RegExp(`LaunchConfiguration\\s*\\(\\s*${quoted}`,"s").test(source))return name;
  }
  return "";
}
function toArrayBufferData(data){
  if(data instanceof ArrayBuffer)return data;
  if(ArrayBuffer.isView(data))return data.buffer.slice(data.byteOffset,data.byteOffset+data.byteLength);
  return data;
}
function parseYAML(text){
  const m={resolution:0.05,origin:[0,0,0],negate:0,occupied_thresh:0.65,free_thresh:0.196};
  for(const line of text.split("\n")){const t=line.trim();if(!t||t.startsWith("#"))continue;const ci=t.indexOf(":");if(ci<0)continue;const k=t.slice(0,ci).trim(),v=t.slice(ci+1).trim();
  const cv=cleanYAMLScalar(v);
  if(k==="origin")m.origin=cv.replace(/[\[\]]/g,"").split(",").map(Number);else if(["resolution","negate","occupied_thresh","free_thresh"].includes(k))m[k]=parseFloat(cv);else if(k==="image")m.image=cv;}
  return m;
}
function writeYAML(meta,fname){return `image: ${fname}.pgm\nresolution: ${meta.resolution}\norigin: [${meta.origin.join(", ")}]\nnegate: ${meta.negate}\noccupied_thresh: ${meta.occupied_thresh}\nfree_thresh: ${meta.free_thresh}\n`;}
function timestampedMapName(prefix="map"){
  const d=new Date(),pad=n=>String(n).padStart(2,"0");
  return `${prefix}_${pad(d.getMonth()+1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}`;
}
function cleanMapBaseName(name){
  return String(name||"").trim().replace(/\\/g,"/").split("/").pop().replace(/\.(pgm|yaml|yml|json)$/i,"").replace(/_semantic$/i,"");
}
function floodFill(data,W,H,sx,sy,fillV){const target=data[(sy*W+sx)*4];if(target===fillV)return;const stack=[sy*W+sx],vis=new Uint8Array(W*H);while(stack.length){const p=stack.pop(),x=p%W,y=Math.floor(p/W);if(x<0||x>=W||y<0||y>=H||vis[p]||data[p*4]!==target)continue;vis[p]=1;data[p*4]=data[p*4+1]=data[p*4+2]=fillV;stack.push(p+1,p-1,p+W,p-W);}}
function finiteNumber(v,fallback=null){const n=Number(v);return Number.isFinite(n)?n:fallback;}
function finitePoint(p){
  if(!p||typeof p!=="object")return null;
  const x=finiteNumber(p.x),y=finiteNumber(p.y);
  return x==null||y==null?null:{x:Math.round(x),y:Math.round(y)};
}
function finiteRect(r){
  if(!r||typeof r!=="object")return null;
  const x=finiteNumber(r.x),y=finiteNumber(r.y),w=finiteNumber(r.w),h=finiteNumber(r.h);
  return x==null||y==null||w==null||h==null?null:{x:Math.round(x),y:Math.round(y),w:Math.round(w),h:Math.round(h)};
}
function finitePoly(poly,minLen=3){
  if(!Array.isArray(poly))return null;
  const out=poly.map(finitePoint).filter(Boolean);
  return out.length>=minLen?out:null;
}
function semanticImageSize(meta){
  const sz=meta?.image_size||meta?.imageSize;
  if(!sz)return null;
  const w=finiteNumber(sz.w??sz.width),h=finiteNumber(sz.h??sz.height);
  return w&&h?{w:Math.round(w),h:Math.round(h)}:null;
}
function yawFromOrientation(q){
  if(!q)return null;
  const x=finiteNumber(q.x,0),y=finiteNumber(q.y,0),z=finiteNumber(q.z),w=finiteNumber(q.w);
  if(z==null||w==null)return null;
  return Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));
}
function thetaFromSemanticPose(item){
  const explicit=finiteNumber(item?.theta_rad??item?.theta);
  if(explicit!=null)return explicit;
  return yawFromOrientation(item?.orientation||item?.pose?.orientation)||0;
}
function maxIdNumber(prefix,items,initial=0){
  let max=initial;
  (items||[]).forEach(it=>{
    const m=String(it?.id||"").match(new RegExp(`^${prefix}(\\d+)$`));
    if(m)max=Math.max(max,parseInt(m[1],10));
  });
  return max;
}
function compareSemanticIds(a,b){
  const aid=String(a?.id||""),bid=String(b?.id||"");
  const am=aid.match(/^([a-z]+)(\d+)$/i),bm=bid.match(/^([a-z]+)(\d+)$/i);
  if(am&&bm&&am[1]===bm[1])return Number(am[2])-Number(bm[2]);
  return aid.localeCompare(bid,undefined,{numeric:true,sensitivity:"base"});
}
function syncSemanticCounters(next){
  _mIdx=maxIdNumber("m",next.maps);
  _rIdx=maxIdNumber("r",next.rooms);
  _cIdx=maxIdNumber("c",next.carriers);
  _oIdx=maxIdNumber("o",next.objects);
  _eIdx=maxIdNumber("e",next.emptySeats,-1);
  _wIdx=maxIdNumber("w",next.waypoints);
  _gIdx=maxIdNumber("g",next.goals);
  if(Object.prototype.hasOwnProperty.call(next,"goalList"))_glIdx=maxIdNumber("gl",next.goalList);
}

let _mIdx=0,_rIdx=0,_cIdx=0,_oIdx=0,_eIdx=-1,_wIdx=0,_gIdx=0,_glIdx=0;
const muid=()=>`m${++_mIdx}`;
const ruid=()=>`r${++_rIdx}`;
const cuid=()=>`c${++_cIdx}`;
const ouid=()=>`o${++_oIdx}`;
const euid=()=>`e${++_eIdx}`;
const wuid=()=>`w${++_wIdx}`;
const guid=()=>`g${++_gIdx}`;
const gluid=()=>`gl${++_glIdx}`;

// ─── UI Style helpers ──────────────────────────────────────────────────────────
const btn=(active=false,danger=false)=>({
  background:danger?"rgba(255,82,82,0.1)":active?"rgba(0,212,255,0.2)":"rgba(0,212,255,0.07)",
  color:danger?"#ff5252":active?"#00d4ff":"#8eb8c8",
  border:`1px solid ${danger?"rgba(255,82,82,0.3)":active?"rgba(0,212,255,0.5)":"rgba(0,212,255,0.15)"}`,
  borderRadius:5,padding:"4px 10px",cursor:"pointer",fontSize:12,
  fontFamily:"'JetBrains Mono','Fira Code',monospace",
  display:"inline-flex",alignItems:"center",gap:5,transition:"all 0.15s",whiteSpace:"nowrap",
});
const INPUT={background:"rgba(0,0,0,0.45)",color:"#c9fffe",border:"1px solid rgba(0,212,255,0.2)",borderRadius:4,padding:"5px 8px",fontSize:12,fontFamily:"monospace",outline:"none"};
const MODAL={position:"fixed",inset:0,background:"rgba(0,8,20,0.88)",backdropFilter:"blur(5px)",display:"flex",alignItems:"center",justifyContent:"center",zIndex:2000};
const MBOX={background:"#0a1628",border:"1px solid rgba(0,212,255,0.3)",borderRadius:10,padding:28,minWidth:360,boxShadow:"0 0 80px rgba(0,212,255,0.1)"};

const EDIT_TOOLS=[
  {id:"brush",icon:"✏️",label:"브러시",key:"B"},
  {id:"eraser",icon:"⌫",label:"지우개",key:"E"},
  {id:"line",icon:"╱",label:"선",key:"L"},
  {id:"rect",icon:"▭",label:"사각형",key:"R"},
  {id:"circle",icon:"◯",label:"원",key:"C"},
  {id:"fill",icon:"▓",label:"채우기",key:"F"},
];
const SEM_TOOL_GROUPS=[
  {label:"맵",      color:"#90a4ae", tools:[
    {id:"semRectMap",     icon:"▭", label:"사각",    key:""},
    {id:"semPolyMap",     icon:"⬡", label:"다각형",  key:""},
  ]},
  {label:"방",      color:"#4a90d9", tools:[
    {id:"semRectRoom",    icon:"▭", label:"사각",    key:"1"},
    {id:"semPolyRoom",    icon:"⬡", label:"다각형",  key:"2"},
  ]},
  {label:"캐리어",  color:"#4fc3f7", tools:[
    {id:"semRectCarrier", icon:"▭", label:"사각",    key:"3"},
    {id:"semPolyCarrier", icon:"⬡", label:"다각형",  key:"4"},
  ]},
  {label:"객체",    color:"#ffaa00", tools:[
    {id:"semRectObj",     icon:"▭", label:"사각",    key:"5"},
    {id:"semPolyObj",     icon:"⬡", label:"다각형",  key:"6"},
    {id:"semPoint",       icon:"◎", label:"포인트",  key:"7"},
  ]},
  {label:"빈좌석",  color:"#00e676", tools:[
    {id:"semRectEmptySeat",icon:"▭", label:"사각",    key:""},
    {id:"semPolyEmptySeat",icon:"⬡", label:"다각형",  key:"Q"},
  ]},
];
const SEM_EXTRA_TOOLS=[
  {id:"startPose", icon:"⌂", label:"시작점",    key:"S", color:"#00e676"},
  {id:"waypoint",  icon:"◎", label:"웨이포인트", key:"W", color:"#ffaa00"},
  {id:"semGoal",   icon:"🎯", label:"시맨틱골",  key:"9", color:"#ff6680"},
  {id:"semSelect", icon:"↖", label:"선택/이동", key:"0", color:"#00d4ff"},
];
const SEM_CAPTURE_KINDS={startPose:"start",waypoint:"waypoint",semGoal:"goal"};
const SEM_CAPTURE_TITLES={
  startPose:"현재 로봇 pose를 Nav2 시작점으로 지정",
  waypoint:"현재 로봇 pose를 웨이포인트로 추가",
  semGoal:"현재 로봇 pose를 시맨틱 골로 추가",
};
const DRAW_COLORS=[
  {val:PX_OCCUPIED,css:"#0a0a0a",border:"#555",label:"장애물"},
  {val:PX_FREE,css:"#f8f8f8",border:"#aaa",label:"자유공간"},
  {val:PX_UNKNOWN,css:"#cdcdcd",border:"#888",label:"미지공간"},
];
const MAP_EDIT_TOOLS = new Set(EDIT_TOOLS.map(t=>t.id));

function normalizeOccupancyValue(value){
  const n=Number(value);
  if(!Number.isFinite(n))return -1;
  return n>127?n-256:n;
}
function occupancyToGray(value){
  const v=normalizeOccupancyValue(value);
  if(v<0)return PX_UNKNOWN;
  if(v<=0)return PX_FREE;
  if(v>=100)return PX_OCCUPIED;
  return Math.max(PX_OCCUPIED,Math.min(PX_FREE,Math.round(PX_FREE-(v/100)*PX_FREE)));
}
function decodeOccupancyData(data){
  if(!data)return [];
  if(typeof data==="string"){
    const bin=atob(data);
    const out=new Int16Array(bin.length);
    for(let i=0;i<bin.length;i++)out[i]=normalizeOccupancyValue(bin.charCodeAt(i));
    return out;
  }
  if(Array.isArray(data))return data;
  if(ArrayBuffer.isView(data))return data;
  if(data instanceof ArrayBuffer)return new Int8Array(data);
  return [];
}
function occupancyGridProjection(msg){
  const info=msg?.info||{};
  const width=Math.max(0,Math.floor(Number(info.width)||0));
  const height=Math.max(0,Math.floor(Number(info.height)||0));
  const resolution=finiteNumber(info.resolution,0.05);
  const originPose=info.origin||{};
  const pos=originPose.position||{};
  const origin=[
    finiteNumber(pos.x,0),
    finiteNumber(pos.y,0),
    yawFromOrientation(originPose.orientation)||0,
  ];
  return {width,height,resolution,origin,frameId:msg?.header?.frame_id||"map"};
}
function drawOccupancyGridToCanvas(canvas,msg){
  const projection=occupancyGridProjection(msg);
  const {width,height}=projection;
  if(!canvas||!width||!height)throw new Error("invalid occupancy grid size");
  const data=decodeOccupancyData(msg?.data);
  if(data.length<width*height)throw new Error(`occupancy grid data too short: ${data.length}/${width*height}`);
  if(canvas.width!==width)canvas.width=width;
  if(canvas.height!==height)canvas.height=height;
  const ctx=canvas.getContext("2d");
  const img=ctx.createImageData(width,height);
  for(let y=0;y<height;y++){
    const py=height-1-y;
    for(let x=0;x<width;x++){
      const gray=occupancyToGray(data[y*width+x]);
      const dst=(py*width+x)*4;
      img.data[dst]=gray;
      img.data[dst+1]=gray;
      img.data[dst+2]=gray;
      img.data[dst+3]=255;
    }
  }
  ctx.putImageData(img,0,0);
  return projection;
}
function sameMapProjection(aMeta,aSize,bMeta,bSize){
  if(!aMeta||!bMeta||!aSize||!bSize)return false;
  const ao=aMeta.origin||[],bo=bMeta.origin||[];
  return aSize.w===bSize.w&&aSize.h===bSize.h
    &&Math.abs((aMeta.resolution||0)-(bMeta.resolution||0))<1e-9
    &&Math.abs((ao[0]||0)-(bo[0]||0))<1e-9
    &&Math.abs((ao[1]||0)-(bo[1]||0))<1e-9
    &&Math.abs((ao[2]||0)-(bo[2]||0))<1e-9;
}
function reprojectPixelPoint(p,fromMeta,fromSize,toMeta,toSize){
  if(!p||!fromSize?.h||!toSize?.h)return p;
  const w=pixelToWorld(p.x,p.y,fromMeta.origin,fromMeta.resolution,fromSize.h);
  const px=worldToPixel(w.x,w.y,toMeta.origin,toMeta.resolution,toSize.h);
  return {x:Math.round(px.x),y:Math.round(px.y)};
}
function reprojectShapeItem(item,fromMeta,fromSize,toMeta,toSize){
  if(!item)return item;
  if(item.point||item.x!=null&&item.y!=null&&item.w==null&&!item.poly){
    return {...item,...reprojectPixelPoint(item,fromMeta,fromSize,toMeta,toSize)};
  }
  if(item.poly){
    return {...item,poly:item.poly.map(p=>reprojectPixelPoint(p,fromMeta,fromSize,toMeta,toSize))};
  }
  if(item.w!=null){
    return {
      ...item,
      poly:rectToPoly(item.x,item.y,item.w,item.h).map(p=>reprojectPixelPoint(p,fromMeta,fromSize,toMeta,toSize)),
    };
  }
  return item;
}
function reprojectPoseItem(item,fromMeta,fromSize,toMeta,toSize){
  if(!item)return item;
  return {...item,...reprojectPixelPoint(item,fromMeta,fromSize,toMeta,toSize)};
}

// ─── Semantic type dialog ──────────────────────────────────────────────────────
function SemanticDialog({mode, typeOptions, onConfirm, onCancel}) {
  const typesList = mode==="map" ? typeOptions.maps : mode==="room" ? typeOptions.rooms : mode==="carrier" ? typeOptions.carriers : mode==="emptySeat" ? [EMPTY_SEAT_TYPE] : typeOptions.objects;
  const [type,setType]=useState(typesList[0].id);
  const [label,setLabel]=useState("");
  const [semanticId,setSemanticId]=useState("");
  const selT=typesList.find(t=>t.id===type);
  const title = mode==="map" ? "🗺 맵 영역 추가" : mode==="room" ? "🏠 방/영역 추가" : mode==="carrier" ? "📦 캐리어 추가" : mode==="emptySeat" ? "▧ 빈좌석 추가" : "🔹 오브젝트 추가";
  const hasOptionalId=mode==="carrier"||mode==="object"||mode==="emptySeat";
  const idLabel=mode==="carrier"?"캐리어 ID (선택)":mode==="emptySeat"?"빈좌석 ID (선택)":"오브젝트 ID (선택)";
  const confirm=()=>onConfirm(type,mode==="emptySeat"?label:(label||selT?.label),hasOptionalId?semanticId:"");
  return(
    <div style={MODAL}>
      <div style={{...MBOX,minWidth:400}}>
        <div style={{display:"flex",alignItems:"center",gap:10,marginBottom:18}}>
          <span style={{fontSize:22}}>{selT?.icon}</span>
          <h3 style={{margin:0,color:"#00d4ff",letterSpacing:1,fontSize:14}}>{title}</h3>
        </div>
        <div style={{marginBottom:14}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",marginBottom:6,letterSpacing:1}}>유형 선택</div>
          <div style={{display:"grid",gridTemplateColumns:"repeat(4,1fr)",gap:4}}>
            {typesList.map(t=>(
              <button key={t.id} onClick={()=>setType(t.id)} style={{
                ...btn(type===t.id),flexDirection:"column",padding:"7px 4px",justifyContent:"center",gap:2,
                borderColor:type===t.id?t.color:undefined,boxShadow:type===t.id?`0 0 10px ${t.color}44`:"none",
              }}>
                <span style={{fontSize:15}}>{t.icon}</span>
                <span style={{fontSize:8,textAlign:"center",lineHeight:1.2,color:type===t.id?t.color:undefined}}>{t.label}</span>
              </button>
            ))}
          </div>
        </div>
        <div style={{marginBottom:18}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",marginBottom:6,letterSpacing:1}}>라벨</div>
          <input autoFocus placeholder={selT?.label} value={label} onChange={e=>setLabel(e.target.value)}
            onKeyDown={e=>e.key==="Enter"&&confirm()}
            style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:13}}/>
        </div>
        {hasOptionalId&&(
          <div style={{marginBottom:18}}>
            <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",marginBottom:6,letterSpacing:1}}>{idLabel}</div>
            <input placeholder="비워두면 자동 ID" value={semanticId} onChange={e=>setSemanticId(e.target.value)}
              onKeyDown={e=>e.key==="Enter"&&confirm()}
              style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:13}}/>
          </div>
        )}
        <div style={{padding:"7px 12px",background:"rgba(0,0,0,0.3)",borderRadius:5,marginBottom:16,display:"flex",alignItems:"center",gap:8}}>
          <div style={{width:13,height:13,borderRadius:3,background:selT?.color,opacity:0.8}}/>
          <span style={{fontSize:10,color:"rgba(0,212,255,0.45)"}}>{selT?.label} — {selT?.color}</span>
        </div>
        <div style={{display:"flex",gap:8,justifyContent:"flex-end"}}>
          <button style={btn()} onClick={onCancel}>취소</button>
          <button style={{...btn(true),borderColor:selT?.color||"#00d4ff",color:selT?.color||"#00d4ff"}}
            onClick={confirm}>
            {selT?.icon} 추가
          </button>
        </div>
      </div>
    </div>
  );
}

// ─── Goal dialog ─────────────────────────────────────────────────────────────
function GoalDialog({rooms,carriers,objects,emptySeats,goalList,roomId,goalId,typeOptions,onConfirm,onCancel}){
  // Targets are optional. A semantic goal can be just a pose, or it can face a semantic area.
  const carrierTargets=carriers.map(c=>{const ct=typeOptions.carriers.find(t=>t.id===c.type);return{id:c.id,label:c.label,icon:ct?.icon||"📦",type:"carrier",color:ct?.color||"#4fc3f7"};});
  const objectTargets=objects.map(o=>{const ot=typeOptions.objects.find(t=>t.id===o.type);return{id:o.id,label:o.label,icon:ot?.icon||"🔹",type:"object",color:ot?.color||"#ffaa00"};});
  const emptySeatTargets=(emptySeats||[]).map(seat=>({id:seat.id,label:seat.label||seat.id,icon:EMPTY_SEAT_TYPE.icon,type:"emptySeat",color:EMPTY_SEAT_TYPE.color}));
  const allTargets=[...carrierTargets,...objectTargets,...emptySeatTargets];
  const [targetId,setTargetId]=useState(allTargets[0]?.id||"");
  const [goalListId,setGoalListId]=useState("");
  const [label,setLabel]=useState("");
  const room=rooms.find(r=>r.id===roomId);
  const selectedTarget=allTargets.find(t=>t.id===targetId);
  const selectedGoalListItem=(goalList||[]).find(item=>item.id===goalListId);
  const selectedGoalId=String(selectedGoalListItem?.goal_id||"").trim();
  const defaultLabel=label||selectedGoalListItem?.label||(selectedTarget?`${room?`${room.label} to `:"to "}${selectedTarget.label}`:(room?`${room.label} goal`:(goalId||"semantic goal")));
  const confirmGoal=()=>onConfirm(targetId,defaultLabel,selectedGoalId||null,selectedGoalId?goalListId:null);
  const renderTargetButton=(t)=>(
    <button key={t.id} onClick={()=>setTargetId(t.id)} style={{
      ...btn(targetId===t.id),textAlign:"left",padding:"6px 10px",
      borderColor:targetId===t.id?t.color:undefined,
      boxShadow:targetId===t.id?`0 0 8px ${t.color}44`:"none",
    }}>
      <span style={{fontSize:14}}>{t.icon}</span>
      <span style={{fontSize:11,color:targetId===t.id?t.color:undefined}}>{t.label}</span>
    </button>
  );
  return(
    <div style={MODAL}>
      <div style={{...MBOX,minWidth:380}}>
        <div style={{display:"flex",alignItems:"center",gap:10,marginBottom:18}}>
          <span style={{fontSize:22}}>🎯</span>
          <h3 style={{margin:0,color:"#ff6680",letterSpacing:1,fontSize:14}}>시맨틱 골 추가</h3>
        </div>
        <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",marginBottom:10}}>📍 위치: {room?`${room.label} 내부`:"방 미할당"}</div>
        {goalList?.length>0&&(
          <div style={{marginBottom:14}}>
            <div style={{fontSize:10,color:"rgba(179,136,255,0.75)",marginBottom:6,letterSpacing:1}}>GOAL LIST에서 선택 (선택)</div>
            <select value={goalListId} onChange={e=>setGoalListId(e.target.value)}
              style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:12,borderColor:"rgba(179,136,255,0.32)"}}>
              <option value="">새 goal pose로 저장</option>
              {goalList.map(item=>(
                <option key={item.id} value={item.id}>
                  {item.label||item.goal_id||item.id}{item.goal_id?` (${item.goal_id})`:""}
                </option>
              ))}
            </select>
            <div style={{marginTop:5,fontSize:9,color:"rgba(179,136,255,0.5)",lineHeight:1.45}}>
              선택하면 이 pose goal이 해당 goal_id와 매칭됩니다. 목록 자체는 자동으로 추가/수정하지 않습니다.
            </div>
          </div>
        )}
        <div style={{marginBottom:14}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",marginBottom:6,letterSpacing:1}}>바라볼 대상 선택 (선택)</div>
          <div style={{maxHeight:180,overflow:"auto",display:"flex",flexDirection:"column",gap:3}}>
            <button onClick={()=>setTargetId("")} style={{
              ...btn(targetId===""),textAlign:"left",padding:"6px 10px",
              borderColor:targetId===""?"#ff6680":undefined,
              boxShadow:targetId===""?"0 0 8px rgba(255,102,128,0.25)":"none",
            }}>
              <span style={{fontSize:14}}>🎯</span>
              <span style={{fontSize:11,color:targetId===""?"#ff6680":undefined}}>대상 없음</span>
              <span style={{fontSize:9,opacity:.5,marginLeft:"auto"}}>현재 방향 유지</span>
            </button>
            {allTargets.length===0?(
              <div style={{color:"rgba(255,102,128,0.5)",fontSize:10,padding:"8px 10px",textAlign:"center"}}>캐리어/객체/빈좌석 없이도 골을 추가할 수 있습니다</div>
            ):(
              <>
                {carrierTargets.length>0&&(
                  <div style={{display:"flex",flexDirection:"column",gap:3}}>
                    <div style={{fontSize:9,color:"rgba(79,195,247,0.65)",letterSpacing:1,padding:"3px 2px 1px"}}>CARRIERS</div>
                    {carrierTargets.map(renderTargetButton)}
                  </div>
                )}
                {objectTargets.length>0&&(
                  <div style={{display:"flex",flexDirection:"column",gap:3,marginTop:carrierTargets.length?7:0}}>
                    <div style={{fontSize:9,color:"rgba(255,170,0,0.68)",letterSpacing:1,padding:"3px 2px 1px"}}>OBJECTS</div>
                    {objectTargets.map(renderTargetButton)}
                  </div>
                )}
                {emptySeatTargets.length>0&&(
                  <div style={{display:"flex",flexDirection:"column",gap:3,marginTop:(carrierTargets.length||objectTargets.length)?7:0}}>
                    <div style={{fontSize:9,color:"rgba(0,230,118,0.68)",letterSpacing:1,padding:"3px 2px 1px"}}>EMPTY SEATS</div>
                    {emptySeatTargets.map(renderTargetButton)}
                  </div>
                )}
              </>
            )
            }
          </div>
        </div>
        <div style={{marginBottom:18}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",marginBottom:6,letterSpacing:1}}>골 라벨</div>
          <input autoFocus placeholder={defaultLabel} value={label} onChange={e=>setLabel(e.target.value)}
            onKeyDown={e=>e.key==="Enter"&&confirmGoal()}
            style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:13}}/>
        </div>
        <div style={{display:"flex",gap:8,justifyContent:"flex-end"}}>
          <button style={btn()} onClick={onCancel}>취소</button>
          <button style={{...btn(true),borderColor:"#ff6680",color:"#ff6680"}}
            onClick={confirmGoal}>
            🎯 추가
          </button>
        </div>
      </div>
    </div>
  );
}

// ─── Start pose dialog ───────────────────────────────────────────────────────
function StartPoseDialog({startTasks,startPoses,pose,defaultTask,defaultId,onConfirm,onCancel}){
  const initialTask=startTasks.includes(defaultTask)?defaultTask:(startTasks[0]||DEFAULT_START_TASK);
  const [task,setTask]=useState(initialTask);
  const [id,setId]=useState(defaultId||nextStartPoseId(startPoses));
  const pickTask=(nextTask)=>{
    setTask(nextTask);
    setId(startPoses?.[nextTask]?.id||nextStartPoseId(startPoses));
  };
  const confirm=()=>onConfirm(task,id);
  const existing=startPoses?.[task];
  return(
    <div style={MODAL}>
      <div style={{...MBOX,minWidth:380}}>
        <div style={{display:"flex",alignItems:"center",gap:10,marginBottom:18}}>
          <span style={{fontSize:22}}>⌂</span>
          <h3 style={{margin:0,color:"#00e676",letterSpacing:1,fontSize:14}}>시작점 추가</h3>
        </div>
        <div style={{marginBottom:14}}>
          <div style={{fontSize:10,color:"rgba(0,230,118,0.55)",marginBottom:6,letterSpacing:1}}>TASK 선택</div>
          <div style={{display:"grid",gridTemplateColumns:"repeat(2,1fr)",gap:4}}>
            {startTasks.map(t=>(
              <button key={t} onClick={()=>pickTask(t)} style={{
                ...btn(task===t),justifyContent:"center",padding:"7px 8px",
                borderColor:task===t?"rgba(0,230,118,0.65)":undefined,
                color:task===t?"#00e676":undefined,
                boxShadow:task===t?"0 0 8px rgba(0,230,118,0.25)":"none",
              }}>
                <span style={{fontSize:11}}>{t}</span>
                {startPoses?.[t]&&<span style={{fontSize:9,opacity:.5}}>교체</span>}
              </button>
            ))}
          </div>
        </div>
        <div style={{marginBottom:12}}>
          <div style={{fontSize:10,color:"rgba(0,230,118,0.55)",marginBottom:6,letterSpacing:1}}>ID</div>
          <input autoFocus value={id} onChange={e=>setId(e.target.value)}
            onKeyDown={e=>{if(e.key==="Enter")confirm();else if(e.key==="Escape")onCancel();}}
            style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:13}}/>
        </div>
        <div style={{padding:"7px 10px",background:"rgba(0,0,0,0.3)",borderRadius:5,marginBottom:16,color:"rgba(0,230,118,0.48)",fontSize:10,lineHeight:1.7}}>
          <div>label: {task}</div>
          {pose&&<div>pixel: ({pose.x}, {pose.y}) · {Math.round((pose.theta||0)*180/Math.PI)}°</div>}
          {existing&&<div style={{color:"rgba(255,190,80,0.75)"}}>선택한 task의 기존 시작점이 새 위치로 교체됩니다</div>}
        </div>
        <div style={{display:"flex",gap:8,justifyContent:"flex-end"}}>
          <button style={btn()} onClick={onCancel}>취소</button>
          <button style={{...btn(true),borderColor:"#00e676",color:"#00e676"}} onClick={confirm}>⌂ 추가</button>
        </div>
      </div>
    </div>
  );
}

function formatFileSize(bytes){
  if(!Number.isFinite(bytes)||bytes<=0)return "";
  if(bytes<1024)return `${bytes} B`;
  if(bytes<1024*1024)return `${(bytes/1024).toFixed(1)} KB`;
  if(bytes<1024*1024*1024)return `${(bytes/1024/1024).toFixed(1)} MB`;
  return `${(bytes/1024/1024/1024).toFixed(1)} GB`;
}
function filterExtensions(filters){
  const exts=(filters||[]).flatMap(f=>f.extensions||[]).map(x=>String(x).toLowerCase().replace(/^\./,""));
  return exts.includes("*")?[]:exts.filter(Boolean);
}
function fileMatchesFilters(name,exts){
  if(!exts.length)return true;
  const ext=String(name||"").split(".").pop().toLowerCase();
  return exts.includes(ext);
}

function RobotFileBrowser({api,request,onClose}){
  const props=request?.properties||[];
  const saveMode=request?.dialogType==="save";
  const directoryMode=props.includes("openDirectory");
  const multiMode=props.includes("multiSelections");
  const exts=useMemo(()=>filterExtensions(request?.filters),[request]);
  const [roots,setRoots]=useState(null);
  const [currentPath,setCurrentPath]=useState("");
  const [pathDraft,setPathDraft]=useState("");
  const [entries,setEntries]=useState([]);
  const [selected,setSelected]=useState([]);
  const [fileName,setFileName]=useState("");
  const [loading,setLoading]=useState(false);
  const [error,setError]=useState("");

  const loadDir=useCallback(async(dir)=>{
    const next=dir||"/";
    setLoading(true);setError("");
    try{
      const res=await api.browseDir(next);
      const resolved=res?.path||next;
      setCurrentPath(resolved);
      setPathDraft(resolved);
      setEntries(res?.entries||[]);
      setSelected([]);
    }catch(e){
      setError(e.message||String(e));
    }finally{
      setLoading(false);
    }
  },[api]);

  useEffect(()=>{
    let alive=true;
    (async()=>{
      const r=await api.fsRoots?.().catch(()=>({home:"/",cwd:"/",root:"/"}))||{home:"/",cwd:"/",root:"/"};
      if(!alive)return;
      setRoots(r);
      const def=String(request?.defaultPath||"");
      if(saveMode)setFileName(basenameFromPath(def)||"");
      let initial=r.home||r.cwd||r.root||"/";
      if(def&&pathLooksAbsolute(def)){
        const defaultLooksDir=!basenameFromPath(def).includes(".");
        initial=saveMode?dirnameFromPath(def):(directoryMode||defaultLooksDir?def:dirnameFromPath(def));
      }
      await loadDir(initial||"/");
    })();
    return()=>{alive=false;};
  },[api,request,saveMode,directoryMode,loadDir]);

  const visibleEntries=useMemo(()=>entries.filter(e=>{
    if(e.isDirectory)return true;
    if(directoryMode&&!saveMode)return false;
    return fileMatchesFilters(e.name,exts);
  }),[entries,directoryMode,saveMode,exts]);

  const toggleSelected=(entry)=>{
    if(entry.isDirectory){
      if(directoryMode&&!saveMode)setSelected([entry.path]);
      return;
    }
    if(saveMode){
      setFileName(entry.name);
      return;
    }
    if(multiMode){
      setSelected(prev=>prev.includes(entry.path)?prev.filter(p=>p!==entry.path):[...prev,entry.path]);
    }else{
      setSelected([entry.path]);
    }
  };

  const confirm=()=>{
    if(saveMode){
      const name=fileName.trim();
      if(name)onClose(joinRobotPath(currentPath,name));
      return;
    }
    if(directoryMode){
      onClose(selected[0]||currentPath);
      return;
    }
    if(multiMode)onClose(selected);
    else onClose(selected[0]||null);
  };

  const canConfirm=saveMode?!!fileName.trim():directoryMode||selected.length>0;
  const title=request?.title||(saveMode?"로봇 PC에 저장":directoryMode?"로봇 PC 폴더 선택":"로봇 PC 파일 선택");

  return(
    <div style={MODAL}>
      <div style={{...MBOX,width:"min(760px,92vw)",height:"min(620px,86vh)",padding:0,display:"flex",flexDirection:"column",overflow:"hidden"}}>
        <div style={{padding:"12px 14px",borderBottom:"1px solid rgba(0,212,255,0.12)",display:"flex",alignItems:"center",gap:8}}>
          <div style={{color:"#00d4ff",fontWeight:"bold",letterSpacing:1,fontSize:13}}>📁 {title}</div>
          <div style={{marginLeft:"auto",display:"flex",gap:6}}>
            {roots?.home&&<button style={{...btn(),fontSize:10,padding:"3px 7px"}} onClick={()=>loadDir(roots.home)}>Home</button>}
            {roots?.cwd&&<button style={{...btn(),fontSize:10,padding:"3px 7px"}} onClick={()=>loadDir(roots.cwd)}>CWD</button>}
            <button style={{...btn(),fontSize:10,padding:"3px 7px"}} onClick={()=>loadDir("/")}>/</button>
          </div>
        </div>
        <div style={{padding:"10px 12px",borderBottom:"1px solid rgba(0,212,255,0.08)",display:"flex",gap:6}}>
          <button style={{...btn(),padding:"5px 8px"}} onClick={()=>loadDir(parentRobotPath(currentPath))}>↑</button>
          <input value={pathDraft} onChange={e=>setPathDraft(e.target.value)} onKeyDown={e=>e.key==="Enter"&&loadDir(pathDraft)}
            style={{...INPUT,flex:1,minWidth:0}}/>
          <button style={btn(true)} onClick={()=>loadDir(pathDraft)}>이동</button>
        </div>
        {saveMode&&(
          <div style={{padding:"10px 12px",borderBottom:"1px solid rgba(0,212,255,0.08)",display:"flex",alignItems:"center",gap:8}}>
            <span style={{fontSize:10,color:"rgba(0,212,255,0.48)",whiteSpace:"nowrap"}}>파일명</span>
            <input value={fileName} onChange={e=>setFileName(e.target.value)} onKeyDown={e=>e.key==="Enter"&&canConfirm&&confirm()}
              style={{...INPUT,flex:1,minWidth:0}}/>
          </div>
        )}
        <div style={{flex:1,overflow:"auto",padding:8}}>
          {error&&<div style={{color:"#ff6680",fontSize:11,padding:10}}>⚠ {error}</div>}
          {loading&&<div style={{color:"rgba(0,212,255,0.45)",fontSize:11,padding:10}}>읽는 중...</div>}
          {!loading&&visibleEntries.length===0&&!error&&(
            <div style={{color:"rgba(0,212,255,0.25)",fontSize:11,padding:20,textAlign:"center"}}>표시할 항목이 없습니다</div>
          )}
          {visibleEntries.map(entry=>{
            const isSelected=selected.includes(entry.path)||saveMode&&fileName===entry.name;
            return(
              <div key={entry.path}
                onClick={()=>toggleSelected(entry)}
                onDoubleClick={()=>{
                  if(entry.isDirectory)loadDir(entry.path);
                  else if(saveMode)setFileName(entry.name);
                  else if(!directoryMode&&!multiMode)onClose(entry.path);
                }}
                style={{display:"grid",gridTemplateColumns:"24px minmax(0,1fr) 90px 120px",gap:8,alignItems:"center",padding:"6px 8px",borderRadius:5,cursor:"pointer",
                  background:isSelected?"rgba(0,212,255,0.16)":"transparent",border:`1px solid ${isSelected?"rgba(0,212,255,0.38)":"transparent"}`}}>
                <span>{entry.isDirectory?"📁":"📄"}</span>
                <span title={entry.path} style={{overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap",color:entry.isDirectory?"#c9fffe":"#8eb8c8"}}>{entry.name}</span>
                <span style={{fontSize:10,color:"rgba(0,212,255,0.32)",textAlign:"right"}}>{entry.isDirectory?"dir":formatFileSize(entry.size)}</span>
                <span style={{fontSize:10,color:"rgba(0,212,255,0.22)",textAlign:"right"}}>{entry.mtimeMs?new Date(entry.mtimeMs).toLocaleDateString():""}</span>
              </div>
            );
          })}
        </div>
        <div style={{padding:"10px 12px",borderTop:"1px solid rgba(0,212,255,0.1)",display:"flex",gap:8,alignItems:"center"}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.35)",overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>
            {saveMode?joinRobotPath(currentPath,fileName||request?.defaultPath||""):(directoryMode?(selected[0]||currentPath):(multiMode?`${selected.length}개 선택`:(selected[0]||"선택 없음")))}
          </div>
          <div style={{marginLeft:"auto",display:"flex",gap:8}}>
            <button style={btn()} onClick={()=>onClose(null)}>취소</button>
            <button style={{...btn(true),opacity:canConfirm?1:.45}} disabled={!canConfirm} onClick={confirm}>선택</button>
          </div>
        </div>
      </div>
    </div>
  );
}

function ThorMdPicker({request,onCancel,onConfirm,onBrowse}){
  const [entries,setEntries]=useState(request?.entries||[]);
  const [currentPath,setCurrentPath]=useState(request?.path||"");
  const [pathDraft,setPathDraft]=useState(request?.path||"");
  const [selected,setSelected]=useState(()=>({}));
  const [query,setQuery]=useState("");
  const [busy,setBusy]=useState(false);
  const [loading,setLoading]=useState(false);
  const [error,setError]=useState("");
  const keyFor=entry=>entry.relativePath||entry.path||entry.name;
  const visible=useMemo(()=>{
    const q=query.trim().toLowerCase();
    if(!q)return entries;
    return entries.filter(entry=>String(entry.relativePath||entry.name||"").toLowerCase().includes(q));
  },[entries,query]);
  const selectedFiles=Object.values(selected);
  const loadDir=async(path="")=>{
    setLoading(true);setError("");
    try{
      const res=await onBrowse(path);
      const nextPath=res?.path||"";
      setCurrentPath(nextPath);
      setPathDraft(nextPath);
      setEntries(res?.entries||[]);
      setQuery("");
    }catch(err){
      setError(err.message||String(err));
    }finally{
      setLoading(false);
    }
  };
  const toggle=entry=>{
    if(entry.isDirectory){
      loadDir(entry.relativePath);
      return;
    }
    const key=keyFor(entry);
    setSelected(prev=>{
      const next={...prev};
      if(next[key])delete next[key];else next[key]=entry;
      return next;
    });
  };
  const selectVisibleFiles=()=>{
    setSelected(prev=>{
      const next={...prev};
      visible.filter(entry=>!entry.isDirectory).forEach(entry=>{next[keyFor(entry)]=entry;});
      return next;
    });
  };
  const confirm=async()=>{
    if(!selectedFiles.length||busy)return;
    setBusy(true);
    try{await onConfirm(selectedFiles,request?.dir);}
    finally{setBusy(false);}
  };

  return(
    <div style={MODAL}>
      <div style={{...MBOX,width:"min(720px,92vw)",height:"min(560px,84vh)",padding:0,display:"flex",flexDirection:"column",overflow:"hidden"}}>
        <div style={{padding:"12px 14px",borderBottom:"1px solid rgba(0,212,255,0.12)",display:"flex",alignItems:"center",gap:8}}>
          <div style={{color:"#00d4ff",fontWeight:"bold",letterSpacing:1,fontSize:13}}>📋 Thor MD 선택</div>
          <div title={request?.dir} style={{flex:1,minWidth:0,color:"rgba(0,212,255,0.34)",fontSize:10,overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>
            {request?.dir}{currentPath?`/${currentPath}`:""}
          </div>
        </div>
        <div style={{padding:"10px 12px",borderBottom:"1px solid rgba(0,212,255,0.08)",display:"flex",gap:6}}>
          <button style={{...btn(),padding:"5px 8px"}} onClick={()=>loadDir(parentRelativePath(currentPath))} disabled={loading||busy}>↑</button>
          <input value={pathDraft} onChange={e=>setPathDraft(e.target.value)} onKeyDown={e=>e.key==="Enter"&&loadDir(pathDraft)}
            placeholder="상대 경로"
            style={{...INPUT,flex:1,minWidth:0}}/>
          <button style={btn(true)} onClick={()=>loadDir(pathDraft)} disabled={loading||busy}>이동</button>
        </div>
        <div style={{padding:"10px 12px",borderBottom:"1px solid rgba(0,212,255,0.08)",display:"flex",gap:6,alignItems:"center"}}>
          <input value={query} onChange={e=>setQuery(e.target.value)} placeholder="검색"
            style={{...INPUT,flex:1,minWidth:0}}/>
          <button style={{...btn(),fontSize:10,padding:"5px 8px"}} onClick={selectVisibleFiles}>현재 MD 전체</button>
          <button style={{...btn(),fontSize:10,padding:"5px 8px"}} onClick={()=>setSelected({})}>해제</button>
        </div>
        <div style={{flex:1,overflow:"auto",padding:8}}>
          {error&&<div style={{color:"#ff6680",fontSize:11,padding:10}}>⚠ {error}</div>}
          {loading&&<div style={{color:"rgba(0,212,255,0.45)",fontSize:11,padding:10}}>읽는 중...</div>}
          {visible.length===0&&(
            <div style={{color:"rgba(0,212,255,0.25)",fontSize:11,padding:20,textAlign:"center"}}>표시할 폴더 또는 MD 파일이 없습니다</div>
          )}
          {visible.map(entry=>{
            const key=keyFor(entry);
            const checked=!!selected[key];
            return(
              <div key={key}
                onClick={()=>toggle(entry)}
                onDoubleClick={()=>entry.isDirectory&&loadDir(entry.relativePath)}
                style={{display:"grid",gridTemplateColumns:"24px minmax(0,1fr) 90px 120px",gap:8,alignItems:"center",padding:"6px 8px",borderRadius:5,cursor:"pointer",
                background:checked?"rgba(0,212,255,0.16)":"transparent",border:`1px solid ${checked?"rgba(0,212,255,0.38)":"transparent"}`}}>
                <span>{entry.isDirectory?"📁":<input type="checkbox" checked={checked} onChange={()=>toggle(entry)} onClick={e=>e.stopPropagation()} style={{margin:0}}/>}</span>
                <span title={entry.path||entry.relativePath} style={{overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap",color:entry.isDirectory?"#c9fffe":"#8eb8c8"}}>{entry.name}</span>
                <span style={{fontSize:10,color:"rgba(0,212,255,0.32)",textAlign:"right"}}>{entry.isDirectory?"dir":formatFileSize(entry.size)}</span>
                <span style={{fontSize:10,color:"rgba(0,212,255,0.22)",textAlign:"right"}}>{entry.mtimeMs?new Date(entry.mtimeMs).toLocaleDateString():""}</span>
              </div>
            );
          })}
        </div>
        <div style={{padding:"10px 12px",borderTop:"1px solid rgba(0,212,255,0.1)",display:"flex",gap:8,alignItems:"center"}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.35)"}}>{selectedFiles.length}개 선택</div>
          <div style={{marginLeft:"auto",display:"flex",gap:8}}>
            <button style={btn()} onClick={onCancel} disabled={busy}>취소</button>
            <button style={{...btn(true),opacity:selectedFiles.length&&!busy?1:.45}} disabled={!selectedFiles.length||busy} onClick={confirm}>
              {busy?"읽는 중...":"불러오기"}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

// ─── Markdown catalog management panel ───────────────────────────────────────
function CatalogPanel({catalog,sources,placedRooms,placedCarriers,placedObjects,hasHostAPI,onImport,onFileImport,onRemoveSource,onReset,onAddRoom,onUpdateRoom,onDeleteRoom,onResetRooms,onSelectPlaced}) {
  const [view,setView]=useState("sources");
  const [roomName,setRoomName]=useState("");
  const [pickIndexByKey,setPickIndexByKey]=useState({});
  const counts=catalogCounts(catalog);
  const matchRooms=(room)=>{const id=catalogId(room);return (placedRooms||[]).filter(r=>catalogId(r.type)===id||catalogId(r.label)===id).sort(compareSemanticIds);};
  const matchLocations=(loc)=>{const id=catalogId(loc?.label);return (placedCarriers||[]).filter(c=>catalogId(c.type)===id||catalogId(c.label)===id).sort(compareSemanticIds);};
  const matchObjects=(obj)=>{const id=catalogId(obj?.name);return (placedObjects||[]).filter(o=>catalogId(o.type)===id||catalogId(o.label)===id).sort(compareSemanticIds);};
  const placedRowStyle=(matches,color="#00e676")=>{
    const first=matches?.[0];
    return first?{
    background:hexRgba(first.color||color,.12),
    border:`1px solid ${hexRgba(first.color||color,.38)}`,
    cursor:"pointer",
  }:{background:"rgba(255,255,255,0.025)",border:"1px solid rgba(0,212,255,0.06)"};
  };
  const selectPlaced=(key,matches)=>{
    if(!matches?.length)return;
    const idx=pickIndexByKey[key]||0;
    onSelectPlaced?.(matches[idx%matches.length]);
    setPickIndexByKey(p=>({...p,[key]:(idx+1)%matches.length}));
  };
  const countBadge=(count)=>(
    <span title={count?`지도에 ${count}개 지정됨`:"미지정"} style={{
      minWidth:18,textAlign:"center",fontSize:10,fontWeight:"bold",
      color:count?"#00e676":"rgba(0,212,255,0.24)",
    }}>{count||"-"}</span>
  );
  const addRoom=()=>{
    const name=roomName.trim();
    if(!name)return;
    onAddRoom(name);
    setRoomName("");
    setView("rooms");
  };

  return(
    <div style={{width:310,background:"#071121",borderLeft:"1px solid rgba(0,212,255,0.12)",display:"flex",flexDirection:"column",flexShrink:0}}>
      <div style={{padding:"9px 14px",borderBottom:"1px solid rgba(0,212,255,0.12)",color:"#00d4ff",fontWeight:"bold",letterSpacing:1,fontSize:12,display:"flex",justifyContent:"space-between",alignItems:"center"}}>
        <span>📋 MD 목록</span>
        <span style={{opacity:.45,fontSize:10}}>{counts.rooms}R·{counts.locations}L·{counts.objects}O</span>
      </div>
      <div style={{padding:10,borderBottom:"1px solid rgba(0,212,255,0.08)",display:"flex",gap:5,flexWrap:"wrap"}}>
        {hasHostAPI ? (
          <button style={{...btn(true),fontSize:11,padding:"4px 8px"}} onClick={onImport}>＋ MD</button>
        ) : (
          <label style={{...btn(true),fontSize:11,padding:"4px 8px",cursor:"pointer"}}>＋ MD<input type="file" accept=".md" multiple onChange={onFileImport} style={{display:"none"}}/></label>
        )}
        <button style={{...btn(false,true),fontSize:11,padding:"4px 8px",opacity:sources.length?1:.45}} onClick={onReset} disabled={!sources.length}>초기화</button>
      </div>
      <div style={{display:"grid",gridTemplateColumns:"repeat(4,1fr)",gap:3,padding:"8px 10px",borderBottom:"1px solid rgba(0,212,255,0.07)"}}>
        {[["sources","소스",sources.length],["rooms","Rooms",counts.rooms],["locations","Locations",counts.locations],["objects","Objects",counts.objects]].map(([id,label,count])=>(
          <button key={id} onClick={()=>setView(id)} style={{...btn(view===id),justifyContent:"center",padding:"4px 2px",fontSize:10,gap:3}}>
            <span>{label}</span><span style={{opacity:.55}}>{count}</span>
          </button>
        ))}
      </div>
      <div style={{flex:1,overflow:"auto",padding:10}}>
        {view==="sources"&&(
          <div style={{display:"flex",flexDirection:"column",gap:6}}>
            {sources.length===0&&(
              <div style={{color:"rgba(0,212,255,0.24)",textAlign:"center",padding:"20px 8px",lineHeight:1.9,fontSize:11}}>
                Rooms 직접 관리 중<br/>
                <span style={{fontSize:10,opacity:.7}}>{(catalog.rooms||[]).slice(0,4).join(" · ")}{(catalog.rooms||[]).length>4?" · ...":""}</span>
              </div>
            )}
            {sources.map(src=>{
              const c=src.counts||catalogCounts(src.catalog);
              return(
                <div key={src.id} style={{padding:"7px 9px",borderRadius:6,background:"rgba(255,255,255,0.025)",border:"1px solid rgba(0,212,255,0.08)"}}>
                  <div style={{display:"flex",alignItems:"center",gap:6}}>
                    <span style={{fontSize:13}}>{src.kind==="manual"?"＋":"📄"}</span>
                    <div style={{flex:1,minWidth:0}}>
                      <div title={src.path||src.name} style={{color:"#c9fffe",fontSize:11,fontWeight:"bold",overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{src.name}</div>
                      <div style={{color:"rgba(0,212,255,0.35)",fontSize:9}}>{c.rooms} rooms · {c.locations} locations · {c.objects} objects</div>
                    </div>
                    <button style={{...btn(false,true),padding:"1px 4px",fontSize:9}} onClick={()=>onRemoveSource(src.id)}>✕</button>
                  </div>
                </div>
              );
            })}
          </div>
        )}

        {view==="rooms"&&(
          <div style={{display:"flex",flexDirection:"column",gap:8}}>
            <div style={{display:"flex",gap:5}}>
              <input value={roomName} onChange={e=>setRoomName(e.target.value)} onKeyDown={e=>{if(e.key==="Enter")addRoom();}} placeholder="room name"
                style={{...INPUT,flex:1,minWidth:0,padding:"4px 7px",fontSize:11}}/>
              <button style={{...btn(true),fontSize:11,padding:"4px 8px"}} onClick={addRoom}>추가</button>
              <button style={{...btn(),fontSize:11,padding:"4px 8px"}} onClick={onResetRooms}>기본값</button>
            </div>
            <div style={{display:"flex",flexDirection:"column",gap:4}}>
              {(catalog.rooms||[]).map((room,i)=>{
                const matches=matchRooms(room);
                const first=matches[0];
                const key=`room:${catalogId(room)}`;
                return(
                <div key={`${room}-${i}`} onClick={()=>selectPlaced(key,matches)} style={{display:"flex",alignItems:"center",gap:7,padding:"5px 8px",borderRadius:5,...placedRowStyle(matches,CATALOG_COLORS[i%CATALOG_COLORS.length])}}>
                  <span style={{width:10,height:10,borderRadius:2,background:first?.color||CATALOG_COLORS[i%CATALOG_COLORS.length],opacity:.85}}/>
                  {countBadge(matches.length)}
                  <CommitInput value={room} onCommit={next=>onUpdateRoom?.(i,next)}
                    style={{...INPUT,flex:1,minWidth:0,padding:"3px 6px",fontSize:10}}/>
                  <button style={{...btn(false,true),padding:"1px 4px",fontSize:9}} onClick={e=>{e.stopPropagation();onDeleteRoom?.(i);}}>✕</button>
                </div>
                );
              })}
            </div>
          </div>
        )}

        {view==="locations"&&(
          <div style={{display:"flex",flexDirection:"column",gap:5}}>
            {(catalog.locations||[]).length===0&&(
              <div style={{color:"rgba(0,212,255,0.24)",textAlign:"center",padding:"22px 8px",fontSize:11}}>Locations 없음</div>
            )}
            {(catalog.locations||[]).map((loc,i)=>{
              const matches=matchLocations(loc);
              const key=`location:${catalogId(loc?.label)}`;
              return(
              <div key={`${loc.number||i}-${loc.label}`} onClick={()=>selectPlaced(key,matches)} style={{padding:"6px 8px",borderRadius:5,...placedRowStyle(matches,CATALOG_COLORS[i%CATALOG_COLORS.length])}}>
                <div style={{display:"flex",gap:6,alignItems:"center"}}>
                  {countBadge(matches.length)}
                  <span style={{color:"rgba(0,212,255,0.45)",fontSize:10,width:22}}>{loc.number||"-"}</span>
                  <span style={{color:"#c9fffe",fontSize:11,fontWeight:"bold",flex:1,minWidth:0,overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{loc.label}</span>
                  {loc.placeable&&<span style={{color:"#00e676",fontSize:9}}>(p)</span>}
                </div>
                {loc.objectCategory&&<div style={{color:"rgba(0,212,255,0.35)",fontSize:9,marginTop:2,paddingLeft:28}}>{loc.objectCategory}</div>}
              </div>
              );
            })}
          </div>
        )}

        {view==="objects"&&(
          <div style={{display:"flex",flexDirection:"column",gap:7}}>
            {(catalog.objectClasses||[]).length===0&&(
              <div style={{color:"rgba(0,212,255,0.24)",textAlign:"center",padding:"22px 8px",fontSize:11}}>Objects 없음</div>
            )}
            {(catalog.objectClasses||[]).map((cls,i)=>(
              <div key={`${cls.label}-${i}`} style={{padding:"7px 8px",borderRadius:6,background:"rgba(255,255,255,0.025)",border:"1px solid rgba(0,212,255,0.07)"}}>
                <div style={{display:"flex",alignItems:"center",gap:6,marginBottom:5}}>
                  <span style={{width:10,height:10,borderRadius:2,background:CATALOG_COLORS[i%CATALOG_COLORS.length],opacity:.85}}/>
                  <span style={{color:"#c9fffe",fontSize:11,fontWeight:"bold"}}>{cls.label}</span>
                  <span style={{color:"rgba(0,212,255,0.35)",fontSize:9}}>({cls.type})</span>
                  <span style={{marginLeft:"auto",color:"rgba(0,212,255,0.35)",fontSize:9}}>{cls.objects?.length||0}</span>
                </div>
                <div style={{display:"flex",flexWrap:"wrap",gap:4}}>
                  {(cls.objects||[]).map(obj=>{
                    const matches=matchObjects(obj);
                    const first=matches[0];
                    const key=`object:${catalogId(obj?.name)}`;
                    return(
                    <span key={obj.name} onClick={()=>selectPlaced(key,matches)} title={`${matches.length?`지도에 ${matches.length}개 지정됨`:"미지정"} · ${obj.image||obj.name}`} style={{
                      padding:"2px 5px",borderRadius:4,
                      background:first?hexRgba(first.color||"#00e676",.12):"rgba(0,212,255,0.06)",
                      border:`1px solid ${first?hexRgba(first.color||"#00e676",.36):"rgba(0,212,255,0.08)"}`,
                      color:first?"rgba(184,255,216,0.9)":"rgba(201,255,254,0.82)",fontSize:9,
                      cursor:first?"pointer":"default",
                    }}>
                      <span style={{fontWeight:"bold",marginRight:4,color:first?"#00e676":"rgba(0,212,255,0.28)"}}>{matches.length||"-"}</span>{obj.name}
                    </span>
                    );
                  })}
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}

function CommitInput({value,onCommit,style,...props}) {
  const [draft,setDraft]=useState(value||"");
  useEffect(()=>{setDraft(value||"");},[value]);
  const commit=()=>{
    const current=String(value||"");
    const next=String(draft||"").trim();
    if(next===current){setDraft(current);return;}
    const ok=onCommit?.(next);
    if(ok===false)setDraft(current);
  };
  return(
    <input {...props} value={draft}
      onChange={e=>setDraft(e.target.value)}
      onBlur={commit}
      onKeyDown={e=>{
        if(e.key==="Enter")e.currentTarget.blur();
        else if(e.key==="Escape"){setDraft(value||"");e.currentTarget.blur();}
      }}
      onClick={e=>e.stopPropagation()}
      style={style}/>
  );
}

// ─── Semantic panel (4-level hierarchy: map > room > carrier > object) ───────
function SemanticPanel({width=260,maps,rooms,carriers,objects,emptySeats,waypoints,goals,goalList,startPoses,startTasks,setSelectedStartTask,selId,setSelId,selWpIdx,setSelWpIdx,onDeleteMap,onDeleteRoom,onDeleteCarrier,onDeleteObj,onDeleteEmptySeat,onDeleteWp,onDeleteGoal,onDeleteStart,onReassign,onRenameObjId,onRenameEmptySeatId,onRenameGoalId,onRenameStartPoseId,onAddGoalList,onUpdateGoalList,onDeleteGoalList,onImportGoalList,onImportGoalListFile,onExportGoalList,setWaypoints,onImportJSON,onImportFile,onExportJSON,toWorld,resolution,typeOptions}) {
  const res=resolution||0.05;
  const [expanded,setExpanded]=useState({});
  const toggle=(id)=>setExpanded(p=>({...p,[id]:!p[id]}));
  const startPoseCount=Object.values(startPoses||{}).filter(Boolean).length;
  const startPoseEntries=startTasks.map(task=>({task,pose:startPoses?.[task]})).filter(x=>x.pose);
  const goalListCount=goalList.length;
  const GOAL_LIST_COLOR="#b388ff";
  const GOAL_LIST_RGB="179,136,255";
  const [sectionOpen,setSectionOpen]=useState(()=>({
    start:startPoseCount<=2,
    emptySeats:true,
    goalList:true,
    goals:goals.length<=3,
    waypoints:waypoints.length<=3,
  }));
  const toggleSection=(key)=>setSectionOpen(p=>({...p,[key]:!p[key]}));
  const sectionHeader=(key,label,count,color,actions=null)=>(
    <div onClick={()=>toggleSection(key)} style={{padding:"6px 10px 4px",fontSize:10,color,letterSpacing:1,fontWeight:"bold",display:"flex",alignItems:"center",gap:6,cursor:"pointer",userSelect:"none"}}>
      <span style={{fontSize:9,opacity:.55,width:10}}>{sectionOpen[key]?"▼":"▶"}</span>
      <span style={{flex:1,minWidth:0,overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{label} ({count})</span>
      {actions&&<div onClick={e=>e.stopPropagation()} style={{display:"flex",alignItems:"center",gap:4,marginLeft:"auto"}}>{actions}</div>}
    </div>
  );

  useEffect(()=>{
    if(!selId)return;
    const next={};
    const openRoom=(roomId)=>{
      if(!roomId)return;
      next[roomId]=true;
      const room=rooms.find(r=>r.id===roomId);
      if(room?.mapId)next[room.mapId]=true;
    };
    const openCarrier=(carrierId)=>{
      if(!carrierId)return;
      next[carrierId]=true;
      const carrier=carriers.find(c=>c.id===carrierId);
      if(carrier?.roomId)openRoom(carrier.roomId);
    };
    const room=rooms.find(r=>r.id===selId);
    if(room?.mapId)next[room.mapId]=true;
    const carrier=carriers.find(c=>c.id===selId);
    if(carrier?.roomId)openRoom(carrier.roomId);
    const obj=objects.find(o=>o.id===selId);
    if(obj?.carrierId)openCarrier(obj.carrierId);
    else if(obj?.roomId)openRoom(obj.roomId);
    const emptySeat=emptySeats.find(e=>e.id===selId);
    if(emptySeat?.roomId)openRoom(emptySeat.roomId);
    if(Object.keys(next).length)setExpanded(prev=>({...prev,...next}));
  },[selId,maps,rooms,carriers,objects,emptySeats]);

  const insidePoly=(poly,obj)=>{
    if(!poly)return false;
    if(obj.point)return pointInPoly(obj.x,obj.y,poly);
    const childPoly=shapeToPoly(obj)||[];
    if(childPoly.length===0)return false;
    return polyMostlyInside(poly,childPoly,0.8);
  };
  const roomBelongsToMap=(room,map)=>{
    if(room.mapId)return room.mapId===map.id;
    const mp=shapeToPoly(map)||[];
    return mp.length>0&&insidePoly(mp,room);
  };
  const carrierBelongsToRoom=(carrier,room)=>{
    if(carrier.roomId)return carrier.roomId===room.id;
    const rp=shapeToPoly(room)||[];
    return rp.length>0&&insidePoly(rp,carrier);
  };
  const objectBelongsToCarrier=(obj,carrier)=>{
    if(obj.carrierId)return obj.carrierId===carrier.id;
    if(obj.roomId)return false;
    const cp=shapeToPoly(carrier)||[];
    return cp.length>0&&insidePoly(cp,obj);
  };

  // Find carriers inside a room
  const carriersInRoom=(room)=>{
    return carriers.filter(c=>carrierBelongsToRoom(c,room));
  };
  // Find objects inside a carrier
  const objectsOnCarrier=(carrier)=>{
    return objects.filter(o=>objectBelongsToCarrier(o,carrier));
  };
  // Find objects directly in a room (not on any carrier)
  const objectsInRoom=(room)=>{
    const rp=shapeToPoly(room)||[];
    const roomObjs=objects.filter(o=>{
      if(o.carrierId)return false;
      if(o.roomId)return o.roomId===room.id;
      return rp.length>0&&insidePoly(rp,o);
    });
    const onCarriers=new Set();
    carriersInRoom(room).forEach(c=>{
      objectsOnCarrier(c).forEach(o=>onCarriers.add(o.id));
    });
    return roomObjs.filter(o=>!onCarriers.has(o.id));
  };
  // Find rooms inside a map
  const roomsInMap=(map)=>{
    return rooms.filter(r=>roomBelongsToMap(r,map));
  };
  const emptySeatsInRoom=(room)=>{
    const rp=shapeToPoly(room)||[];
    return emptySeats.filter(seat=>{
      if(seat.roomId===room.id)return true;
      const sp=shapeToPoly(seat)||[];
      return rp.length>0&&polyMostlyInside(rp,sp,0.8);
    });
  };
  // Unassigned rooms (not in any map)
  const unassignedRooms=rooms.filter(r=>!maps.some(m=>roomBelongsToMap(r,m)));
  // Unassigned carriers (not in any room)
  const unassignedCarriers=carriers.filter(c=>!rooms.some(r=>carrierBelongsToRoom(c,r)));
  // Unassigned objects (not in any room or carrier)
  const assignedObjIds=new Set();
  rooms.forEach(r=>{
    objectsInRoom(r).forEach(o=>assignedObjIds.add(o.id));
    carriersInRoom(r).forEach(c=>{objectsOnCarrier(c).forEach(o=>assignedObjIds.add(o.id));});
  });
  unassignedCarriers.forEach(c=>{objectsOnCarrier(c).forEach(o=>assignedObjIds.add(o.id));});
  const unassignedObjects=objects.filter(o=>!assignedObjIds.has(o.id));
  const assignedEmptySeatIds=new Set();
  rooms.forEach(r=>emptySeatsInRoom(r).forEach(seat=>assignedEmptySeatIds.add(seat.id)));
  const unassignedEmptySeats=emptySeats.filter(seat=>!assignedEmptySeatIds.has(seat.id));

  const renderEmptySeat=(seat,indent=0)=>{
    const isSel=selId===seat.id;
    const poly=shapeToPoly(seat)||[];
    return(
      <div key={seat.id} onClick={e=>{e.stopPropagation();setSelId(isSel?null:seat.id);}} style={{
        marginLeft:indent,marginTop:3,padding:"5px 8px",borderRadius:5,cursor:"pointer",
        background:isSel?"rgba(0,230,118,0.12)":"rgba(0,230,118,0.035)",
        border:isSel?"1px solid rgba(0,230,118,0.5)":"1px solid rgba(0,230,118,0.12)",
      }}>
        <div style={{display:"flex",alignItems:"center",gap:5}}>
          <span style={{fontSize:12,color:EMPTY_SEAT_TYPE.color}}>▧</span>
          <div style={{flex:1,minWidth:0}}>
            <div style={{color:EMPTY_SEAT_TYPE.color,fontSize:11,fontWeight:"bold",overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{seat.label||seat.id}</div>
            <div style={{color:"rgba(0,230,118,0.42)",fontSize:9}}>{poly.length}꼭짓점</div>
          </div>
          <button onClick={e=>{e.stopPropagation();onDeleteEmptySeat(seat.id);}} style={{...btn(false,true),padding:"1px 4px",fontSize:9}}>✕</button>
        </div>
        {isSel&&(
          <div style={{marginTop:4,display:"flex",flexDirection:"column",gap:3}} onClick={e=>e.stopPropagation()}>
            <div style={{display:"flex",alignItems:"center",gap:4}}>
              <span style={{fontSize:9,color:"rgba(0,230,118,0.5)",whiteSpace:"nowrap"}}>ID</span>
              <CommitInput value={seat.id||""} onCommit={next=>onRenameEmptySeatId?.(seat.id,next)}
                style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10,borderColor:"rgba(0,230,118,0.24)"}}/>
            </div>
            <div style={{display:"flex",alignItems:"center",gap:4}}>
              <span style={{fontSize:9,color:"rgba(0,230,118,0.5)",whiteSpace:"nowrap"}}>라벨</span>
              <input value={seat.label||""} onChange={e=>onReassign("emptySeat",seat.id,"label",e.target.value)}
                onClick={e=>e.stopPropagation()}
                style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10,borderColor:"rgba(0,230,118,0.24)"}}/>
            </div>
            {rooms.length>0&&(
              <div style={{display:"flex",alignItems:"center",gap:4}}>
                <span style={{fontSize:9,color:"rgba(0,230,118,0.5)",whiteSpace:"nowrap"}}>소속방</span>
                <select value={seat.roomId||""} onChange={e=>onReassign("emptySeat",seat.id,"roomId",e.target.value||null)}
                  style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10,borderColor:"rgba(0,230,118,0.24)"}}>
                  <option value="">없음</option>
                  {rooms.map(r=><option key={r.id} value={r.id}>{r.label}</option>)}
                </select>
              </div>
            )}
          </div>
        )}
      </div>
    );
  };

  const renderObj=(obj,indent=0)=>{
    const ot=typeOptions.objects.find(t=>t.id===obj.type)||typeOptions.objects[typeOptions.objects.length-1];
    const isSel=selId===obj.id;
    return(
      <div key={obj.id} onClick={e=>{e.stopPropagation();setSelId(isSel?null:obj.id);}} style={{
        marginLeft:indent,marginTop:3,padding:"5px 8px",borderRadius:5,cursor:"pointer",
        background:isSel?`${ot.color}18`:"transparent",
        border:isSel?`1px solid ${ot.color}55`:"1px solid rgba(255,255,255,0.04)",
      }}>
        <div style={{display:"flex",alignItems:"center",gap:5}}>
          <span style={{fontSize:12}}>{ot.icon}</span>
          <div style={{flex:1,minWidth:0}}>
            <div style={{color:ot.color,fontSize:11,overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{obj.label}</div>
            <div style={{color:"rgba(0,212,255,0.3)",fontSize:9}}>
              {obj.point?(()=>{const w=toWorld(obj.x,obj.y);return `(${w.x}, ${w.y})m`;})():obj.poly?`${obj.poly.length}꼭짓점`:`${(obj.w*res).toFixed(1)}×${(obj.h*res).toFixed(1)}m`}
            </div>
          </div>
          <button onClick={e=>{e.stopPropagation();onDeleteObj(obj.id);}} style={{...btn(false,true),padding:"1px 4px",fontSize:9}}>✕</button>
        </div>
        {isSel&&(
          <div style={{marginTop:4,display:"flex",flexDirection:"column",gap:3}} onClick={e=>e.stopPropagation()}>
            <div style={{display:"flex",alignItems:"center",gap:4}}>
              <span style={{fontSize:9,color:"rgba(0,212,255,0.4)",whiteSpace:"nowrap"}}>ID</span>
              <CommitInput value={obj.id||""} onCommit={next=>onRenameObjId?.(obj.id,next)}
                style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}/>
            </div>
            {carriers.length>0&&(
              <div style={{display:"flex",alignItems:"center",gap:4}}>
                <span style={{fontSize:9,color:"rgba(0,212,255,0.4)",whiteSpace:"nowrap"}}>캐리어</span>
                <select value={obj.carrierId||""} onChange={e=>onReassign("object",obj.id,"carrierId",e.target.value||null)}
                  style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}>
                  <option value="">없음</option>
                  {carriers.map(c=><option key={c.id} value={c.id}>{c.label}</option>)}
                </select>
              </div>
            )}
            {rooms.length>0&&(
              <div style={{display:"flex",alignItems:"center",gap:4}}>
                <span style={{fontSize:9,color:"rgba(0,212,255,0.4)",whiteSpace:"nowrap"}}>소속방</span>
                <select value={obj.roomId||""} onChange={e=>onReassign("object",obj.id,"roomId",e.target.value||null)}
                  style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}>
                  <option value="">없음</option>
                  {rooms.map(r=><option key={r.id} value={r.id}>{r.label}</option>)}
                </select>
              </div>
            )}
          </div>
        )}
      </div>
    );
  };

  const renderCarrier=(carrier,indent=0)=>{
    const ct=typeOptions.carriers.find(t=>t.id===carrier.type)||typeOptions.carriers[typeOptions.carriers.length-1];
    const isSel=selId===carrier.id;
    const isExp=expanded[carrier.id];
    const nested=objectsOnCarrier(carrier);
    const isPoly=!!carrier.poly;
    return(
      <div key={carrier.id} style={{marginLeft:indent,marginTop:3}}>
        <div onClick={e=>{e.stopPropagation();setSelId(isSel?null:carrier.id);toggle(carrier.id);}} style={{
          padding:"6px 9px",borderRadius:5,cursor:"pointer",
          background:isSel?`${ct.color}18`:"rgba(255,255,255,0.02)",
          border:isSel?`1px solid ${ct.color}55`:"1px solid rgba(255,255,255,0.04)",
        }}>
          <div style={{display:"flex",alignItems:"center",gap:5}}>
            <span style={{fontSize:13}}>{ct.icon}</span>
            <div style={{flex:1,minWidth:0}}>
              <div style={{color:ct.color,fontSize:11,fontWeight:"bold",overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{carrier.label}</div>
              <div style={{color:"rgba(0,212,255,0.35)",fontSize:9}}>{ct.label} · {isPoly?`${carrier.poly.length}꼭짓점`:`${(carrier.w*res).toFixed(1)}×${(carrier.h*res).toFixed(1)}m`} · z {(Number(carrier.z)||0).toFixed(2)}m</div>
            </div>
            {nested.length>0&&<span style={{fontSize:9,color:"rgba(0,212,255,0.25)"}}>{isExp?"▲":"▼"}</span>}
            <button onClick={e=>{e.stopPropagation();onDeleteCarrier(carrier.id);}} style={{...btn(false,true),padding:"1px 4px",fontSize:9}}>✕</button>
          </div>
          {nested.length>0&&<div style={{fontSize:9,color:"rgba(0,212,255,0.3)",marginTop:2}}>└ {nested.length}개 객체</div>}
          {isSel&&(
            <div style={{marginTop:5,display:"flex",flexDirection:"column",gap:4}} onClick={e=>e.stopPropagation()}>
              {rooms.length>0&&(
                <div style={{display:"flex",alignItems:"center",gap:4}}>
                  <span style={{fontSize:9,color:"rgba(0,212,255,0.4)"}}>소속방</span>
                  <select value={carrier.roomId||""} onChange={e=>onReassign("carrier",carrier.id,"roomId",e.target.value||null)}
                    style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}>
                    <option value="">없음</option>
                    {rooms.map(r=><option key={r.id} value={r.id}>{r.label}</option>)}
                  </select>
                </div>
              )}
              <div style={{display:"flex",alignItems:"center",gap:4}}>
                <span style={{fontSize:9,color:"rgba(0,212,255,0.4)",whiteSpace:"nowrap"}}>z(m)</span>
                <input type="number" step="0.01" value={Number(carrier.z)||0}
                  onChange={e=>onReassign("carrier",carrier.id,"z",parseFloat(e.target.value)||0)}
                  style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}/>
              </div>
            </div>
          )}
        </div>
        {isExp&&nested.map(o=>renderObj(o,10))}
      </div>
    );
  };

  const renderRoom=(room,indent=0)=>{
    const rt=typeOptions.rooms.find(t=>t.id===room.type)||typeOptions.rooms[typeOptions.rooms.length-1];
    const poly=shapeToPoly(room)||[];
    const nestedCarriers=carriersInRoom(room);
    const nestedObjs=objectsInRoom(room);
    const nestedEmptySeats=emptySeatsInRoom(room);
    const isSel=selId===room.id, isExp=expanded[room.id];
    const isPoly=!!room.poly;
    return(
      <div key={room.id} style={{marginLeft:indent,marginBottom:4}}>
        <div onClick={()=>{setSelId(isSel?null:room.id);toggle(room.id);}} style={{
          padding:"7px 10px",borderRadius:6,cursor:"pointer",
          background:isSel?`${rt.color}20`:"rgba(255,255,255,0.02)",
          border:isSel?`1px solid ${rt.color}88`:"1px solid rgba(255,255,255,0.05)",
        }}>
          <div style={{display:"flex",alignItems:"center",gap:6}}>
            <span style={{fontSize:14}}>{rt.icon}</span>
            <div style={{flex:1,minWidth:0}}>
              <div style={{color:rt.color,fontWeight:"bold",fontSize:11,overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{room.label}</div>
              <div style={{color:"rgba(0,212,255,0.4)",fontSize:9}}>{rt.label} · {isPoly?`${poly.length}꼭짓점`:`${(room.w*res).toFixed(1)}×${(room.h*res).toFixed(1)}m`}</div>
            </div>
            <span style={{fontSize:9,color:"rgba(0,212,255,0.25)"}}>{isExp?"▲":"▼"}</span>
            <button onClick={e=>{e.stopPropagation();onDeleteRoom(room.id);}} style={{...btn(false,true),padding:"1px 5px",fontSize:10}}>✕</button>
          </div>
          {(nestedCarriers.length>0||nestedObjs.length>0||nestedEmptySeats.length>0)&&<div style={{fontSize:9,color:"rgba(0,212,255,0.3)",marginTop:2}}>└ {nestedCarriers.length}캐리어 · {nestedObjs.length}객체 · {nestedEmptySeats.length}빈좌석</div>}
          {isSel&&maps.length>0&&(
            <div style={{marginTop:5,display:"flex",alignItems:"center",gap:4}} onClick={e=>e.stopPropagation()}>
              <span style={{fontSize:9,color:"rgba(0,212,255,0.4)"}}>소속맵</span>
              <select value={room.mapId||""} onChange={e=>onReassign("room",room.id,"mapId",e.target.value||null)}
                style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}>
                <option value="">없음</option>
                {maps.map(m=><option key={m.id} value={m.id}>{m.label}</option>)}
              </select>
            </div>
          )}
        </div>
        {isExp&&<>
          {nestedCarriers.map(c=>renderCarrier(c,14))}
          {nestedObjs.map(o=>renderObj(o,14))}
          {nestedEmptySeats.map(seat=>renderEmptySeat(seat,14))}
        </>}
      </div>
    );
  };

  return(
    <div style={{width,background:"#070f1e",borderLeft:"1px solid rgba(0,212,255,0.12)",display:"flex",flexDirection:"column",flexShrink:0}}>
      <div style={{padding:"9px 14px",borderBottom:"1px solid rgba(0,212,255,0.12)",color:"#00d4ff",fontWeight:"bold",letterSpacing:1,fontSize:12,display:"flex",justifyContent:"space-between"}}>
        <span>🗺 시맨틱 레이어</span><span style={{opacity:.4,fontSize:10}}>{maps.length}맵·{rooms.length}방·{carriers.length}캐·{objects.length}객·{emptySeats.length}석·{goals.length}골·목록{goalListCount}·{waypoints.length}WP{startPoseCount?`·시작${startPoseCount}`:""}</span>
      </div>
      <div style={{flex:1,overflow:"auto",padding:8}}>
        {maps.length===0&&rooms.length===0&&carriers.length===0&&objects.length===0&&emptySeats.length===0&&waypoints.length===0&&goals.length===0&&goalList.length===0&&!startPoseCount&&(
          <div style={{color:"rgba(0,212,255,0.22)",textAlign:"center",padding:"24px 8px",lineHeight:2.2,fontSize:11}}>
            시맨틱 항목 없음<br/>
            <span style={{fontSize:10,opacity:.7}}>맵→방→캐리어→객체 · S:시작점 · 9:골 · W:웨이포인트</span>
          </div>
        )}
        {/* ── Maps ── */}
        {maps.map(m=>{
          const mt=typeOptions.maps.find(t=>t.id===m.type)||typeOptions.maps[typeOptions.maps.length-1];
          const nestedRooms=roomsInMap(m);
          const isSel=selId===m.id, isExp=expanded[m.id];
          const isPoly=!!m.poly;
          return(
            <div key={m.id} style={{marginBottom:6}}>
              <div onClick={()=>{setSelId(isSel?null:m.id);toggle(m.id);}} style={{
                padding:"7px 10px",borderRadius:6,cursor:"pointer",
                background:isSel?"rgba(144,164,174,0.15)":"rgba(255,255,255,0.02)",
                border:isSel?"1px solid rgba(144,164,174,0.5)":"1px solid rgba(144,164,174,0.15)",
              }}>
                <div style={{display:"flex",alignItems:"center",gap:6}}>
                  <span style={{fontSize:14}}>{mt.icon}</span>
                  <div style={{flex:1,minWidth:0}}>
                    <div style={{color:mt.color,fontWeight:"bold",fontSize:11,overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{m.label}</div>
                    <div style={{color:"rgba(144,164,174,0.5)",fontSize:9}}>{mt.label} · {isPoly?`${m.poly.length}꼭짓점`:`${(m.w*res).toFixed(1)}×${(m.h*res).toFixed(1)}m`}</div>
                  </div>
                  <span style={{fontSize:9,color:"rgba(144,164,174,0.3)"}}>{nestedRooms.length}방 {isExp?"▲":"▼"}</span>
                  <button onClick={e=>{e.stopPropagation();onDeleteMap(m.id);}} style={{...btn(false,true),padding:"1px 5px",fontSize:10}}>✕</button>
                </div>
              </div>
              {isExp&&nestedRooms.map(room=>renderRoom(room,12))}
            </div>
          );
        })}
        {/* ── Unassigned rooms ── */}
        {unassignedRooms.map(room=>renderRoom(room,0))}
        {/* ── Unassigned carriers ── */}
        {unassignedCarriers.length>0&&(
          <div style={{marginTop:8}}>
            <div style={{fontSize:9,color:"rgba(0,212,255,0.25)",marginBottom:4,paddingLeft:4,letterSpacing:1}}>── 미할당 캐리어</div>
            {unassignedCarriers.map(c=>renderCarrier(c,0))}
          </div>
        )}
        {/* ── Unassigned objects ── */}
        {unassignedObjects.length>0&&(
          <div style={{marginTop:8}}>
            <div style={{fontSize:9,color:"rgba(0,212,255,0.25)",marginBottom:4,paddingLeft:4,letterSpacing:1}}>── 미할당 객체</div>
            {unassignedObjects.map(o=>renderObj(o,0))}
          </div>
        )}
        {unassignedEmptySeats.length>0&&(
          <div style={{marginTop:8}}>
            <div style={{fontSize:9,color:"rgba(0,230,118,0.32)",marginBottom:4,paddingLeft:4,letterSpacing:1}}>── 미할당 빈좌석</div>
            {unassignedEmptySeats.map(seat=>renderEmptySeat(seat,0))}
          </div>
        )}
      </div>
      {/* ── Start pose section ── */}
      {startPoseCount>0&&(
        <div style={{borderTop:"1px solid rgba(0,230,118,0.16)",marginTop:4}}>
          {sectionHeader("start","⌂ NAV2 START",startPoseCount,"#00e676")}
          {sectionOpen.start&&(
            <div style={{maxHeight:160,overflow:"auto",paddingBottom:1}}>
              {startPoseEntries.map(({task,pose})=>{
                const isSel=selId===pose.id;
                return(
                  <div key={task} onClick={()=>{setSelectedStartTask(task);setSelId(isSel?null:pose.id);}} style={{
                    margin:"0 8px 4px",padding:"5px 8px",borderRadius:5,cursor:"pointer",
                    background:isSel?"rgba(0,230,118,0.1)":"rgba(255,255,255,0.02)",
                    border:isSel?"1px solid rgba(0,230,118,0.45)":"1px solid rgba(0,230,118,0.12)",
                  }}>
                    <div style={{display:"flex",alignItems:"center",gap:5}}>
                      <span style={{fontSize:12,color:"#00e676"}}>⌂</span>
                      <div style={{flex:1,minWidth:0}}>
                        <div style={{color:"#00e676",fontWeight:"bold",fontSize:11}}>{pose.id}: {task}</div>
                        <div style={{color:"rgba(0,230,118,0.48)",fontSize:9}}>
                          {(()=>{const w=toWorld(pose.x,pose.y);return `(${w.x}, ${w.y})m`;})()} · {Math.round(pose.theta*180/Math.PI)}°
                        </div>
                      </div>
                      <button onClick={ev=>{ev.stopPropagation();onDeleteStart(task);}} style={{...btn(false,true),padding:"1px 4px",fontSize:9}}>✕</button>
                    </div>
                    {isSel&&(
                      <div style={{marginTop:5,display:"flex",alignItems:"center",gap:4}} onClick={e=>e.stopPropagation()}>
                        <span style={{fontSize:9,color:"rgba(0,230,118,0.5)",whiteSpace:"nowrap"}}>ID</span>
                        <CommitInput value={pose.id||""} onCommit={next=>onRenameStartPoseId?.(task,next)}
                          style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}/>
                      </div>
                    )}
                  </div>
                );
              })}
            </div>
          )}
        </div>
      )}
      {/* ── Goal list section ── */}
      <div style={{borderTop:`1px solid rgba(${GOAL_LIST_RGB},0.18)`,marginTop:4}}>
        {sectionHeader("goalList","☑ GOAL LIST",goalList.length,GOAL_LIST_COLOR,
          <>
            {onImportGoalList&&<button onClick={onImportGoalList} style={{...btn(),padding:"1px 6px",fontSize:10,color:GOAL_LIST_COLOR,borderColor:`rgba(${GOAL_LIST_RGB},0.28)`}}>열기</button>}
            {!onImportGoalList&&onImportGoalListFile&&(
              <label style={{...btn(),padding:"1px 6px",fontSize:10,color:GOAL_LIST_COLOR,borderColor:`rgba(${GOAL_LIST_RGB},0.28)`,cursor:"pointer"}}>
                열기
                <input type="file" accept=".json" onChange={onImportGoalListFile} style={{display:"none"}}/>
              </label>
            )}
            <button onClick={onExportGoalList} style={{...btn(),padding:"1px 6px",fontSize:10,color:GOAL_LIST_COLOR,borderColor:`rgba(${GOAL_LIST_RGB},0.28)`}}>저장</button>
            <button onClick={onAddGoalList} style={{...btn(),padding:"1px 6px",fontSize:10,color:GOAL_LIST_COLOR,borderColor:`rgba(${GOAL_LIST_RGB},0.34)`}}>＋</button>
          </>
        )}
        {sectionOpen.goalList&&(
          <>
            <datalist id="semantic-goal-id-options">
              {goals.map(g=><option key={g.id} value={g.id}>{g.label||g.id}</option>)}
            </datalist>
            {goalList.length===0?(
              <div style={{margin:"0 8px 6px",padding:"6px 8px",borderRadius:5,color:`rgba(${GOAL_LIST_RGB},0.55)`,fontSize:10,lineHeight:1.5,border:`1px solid rgba(${GOAL_LIST_RGB},0.12)`}}>
                + 버튼으로 라벨과 goal_id를 먼저 만들어 저장할 수 있습니다.
              </div>
            ):(
              <div style={{maxHeight:220,overflow:"auto",paddingBottom:1}}>
                {goalList.map((item,i)=>{
                  const linkedGoal=goals.find(g=>g.id===item.goal_id);
                  return(
                    <div key={item.id||i} style={{margin:"0 8px 5px",padding:"6px 8px",borderRadius:5,background:`rgba(${GOAL_LIST_RGB},0.07)`,border:`1px solid rgba(${GOAL_LIST_RGB},0.18)`}}>
                      <div style={{display:"flex",alignItems:"center",gap:5,marginBottom:5}}>
                        <span style={{color:GOAL_LIST_COLOR,fontSize:10,fontWeight:"bold"}}>{i+1}</span>
                        <span style={{color:`rgba(${GOAL_LIST_RGB},0.58)`,fontSize:9,flex:1,minWidth:0,overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{item.id}</span>
                        <button onClick={()=>onDeleteGoalList(item.id)} style={{...btn(false,true),padding:"1px 4px",fontSize:9}}>✕</button>
                      </div>
                      <div style={{display:"flex",flexDirection:"column",gap:4}}>
                        <input value={item.label||""} onChange={e=>onUpdateGoalList(item.id,{label:e.target.value})}
                          placeholder="라벨" style={{...INPUT,width:"100%",boxSizing:"border-box",padding:"3px 5px",fontSize:10}}/>
                        <input value={item.goal_id||""} onChange={e=>onUpdateGoalList(item.id,{goal_id:e.target.value.trim()})}
                          list="semantic-goal-id-options" placeholder="goal_id" style={{...INPUT,width:"100%",boxSizing:"border-box",padding:"3px 5px",fontSize:10}}/>
                      </div>
                      <div style={{marginTop:4,color:linkedGoal?`rgba(${GOAL_LIST_RGB},0.62)`:"rgba(0,212,255,0.3)",fontSize:8,lineHeight:1.4}}>
                        {linkedGoal?`${linkedGoal.label||linkedGoal.id}에 연결됨`:"pose 없이 저장 가능 · 나중에 같은 goal_id와 매칭"}
                      </div>
                    </div>
                  );
                })}
              </div>
            )}
          </>
        )}
      </div>
      {/* ── Goals section ── */}
      {goals.length>0&&(
        <div style={{borderTop:"1px solid rgba(255,102,128,0.15)",marginTop:4}}>
          {sectionHeader("goals","🎯 GOALS",goals.length,"#ff6680")}
          {sectionOpen.goals&&(
            <div style={{maxHeight:220,overflow:"auto",paddingBottom:1}}>
              {goals.map(g=>{
                const isSel=selId===g.id;
                const room=rooms.find(r=>r.id===g.room_id);
                const target=[...carriers,...objects,...emptySeats].find(s=>s.id===g.target_id);
                return(
                  <div key={g.id} onClick={()=>setSelId(isSel?null:g.id)} style={{
                    margin:"0 8px 4px",padding:"5px 8px",borderRadius:5,cursor:"pointer",
                    background:isSel?"rgba(255,102,128,0.1)":"rgba(255,255,255,0.02)",
                    border:isSel?"1px solid rgba(255,102,128,0.4)":"1px solid rgba(255,102,128,0.1)",
                  }}>
                    <div style={{display:"flex",alignItems:"center",gap:5}}>
                      <span style={{fontSize:11,color:"#ff6680"}}>🎯</span>
                      <div style={{flex:1,minWidth:0}}>
                        <div style={{color:"#ff6680",fontWeight:"bold",fontSize:11,overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{g.id}: {g.label}</div>
                        <div style={{color:"rgba(255,102,128,0.45)",fontSize:9}}>
                          {room?room.label:"미할당"} → {target?target.label:"대상 없음"} · {Math.round(g.theta*180/Math.PI)}°
                        </div>
                        <div style={{color:"rgba(255,102,128,0.35)",fontSize:8}}>
                          {(()=>{const w=toWorld(g.x,g.y);return `(${w.x}, ${w.y})m`;})()}
                        </div>
                      </div>
                      <button onClick={ev=>{ev.stopPropagation();onDeleteGoal(g.id);}} style={{...btn(false,true),padding:"1px 4px",fontSize:9}}>✕</button>
                    </div>
                    {isSel&&(
                      <div style={{marginTop:4,display:"flex",flexDirection:"column",gap:3}} onClick={e=>e.stopPropagation()}>
                        <div style={{display:"flex",alignItems:"center",gap:4}}>
                          <span style={{fontSize:9,color:"rgba(255,102,128,0.5)",whiteSpace:"nowrap"}}>ID</span>
                          <CommitInput value={g.id} onCommit={next=>onRenameGoalId?.(g.id,next)}
                            style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}/>
                        </div>
                        <div style={{display:"flex",alignItems:"center",gap:4}}>
                          <span style={{fontSize:9,color:"rgba(255,102,128,0.5)",whiteSpace:"nowrap"}}>라벨</span>
                          <input value={g.label||""} onChange={e=>onReassign("goal",g.id,"label",e.target.value)}
                            onClick={e=>e.stopPropagation()}
                            style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}/>
                        </div>
                        <div style={{display:"flex",alignItems:"center",gap:4}}>
                          <span style={{fontSize:9,color:"rgba(255,102,128,0.5)",whiteSpace:"nowrap"}}>소속방</span>
                          <select value={g.room_id||""} onChange={e=>onReassign("goal",g.id,"room_id",e.target.value||null)}
                            style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}>
                            <option value="">없음</option>
                            {rooms.map(r=><option key={r.id} value={r.id}>{r.label}</option>)}
                          </select>
                        </div>
                        <div style={{display:"flex",alignItems:"center",gap:4}}>
                          <span style={{fontSize:9,color:"rgba(255,102,128,0.5)",whiteSpace:"nowrap"}}>대상</span>
                          <select value={g.target_id||""} onChange={e=>onReassign("goal",g.id,"target_id",e.target.value||null)}
                            style={{...INPUT,flex:1,padding:"2px 4px",fontSize:10}}>
                            <option value="">없음</option>
                            {carriers.map(c=><option key={c.id} value={c.id}>[캐] {c.label}</option>)}
                            {objects.map(o=><option key={o.id} value={o.id}>[객] {o.label}</option>)}
                            {emptySeats.map(seat=><option key={seat.id} value={seat.id}>[석] {seat.label||seat.id}</option>)}
                          </select>
                        </div>
                      </div>
                    )}
                  </div>
                );
              })}
            </div>
          )}
        </div>
      )}
      {/* ── Waypoints section ── */}
      {waypoints.length>0&&(
        <div style={{borderTop:"1px solid rgba(255,170,0,0.15)",marginTop:4}}>
          {sectionHeader("waypoints","◎ WAYPOINTS",waypoints.length,"#ffaa00")}
          {sectionOpen.waypoints&&(
            <div style={{maxHeight:180,overflow:"auto",paddingBottom:1}}>
              {waypoints.map((wp,i)=>{
                const isSel=selWpIdx===i;
                return(
                  <div key={i} onClick={()=>setSelWpIdx(isSel?null:i)} style={{
                    margin:"0 8px 4px",padding:"5px 8px",borderRadius:5,cursor:"pointer",
                    background:isSel?"rgba(255,170,0,0.1)":"rgba(255,255,255,0.02)",
                    border:isSel?"1px solid rgba(255,170,0,0.4)":"1px solid rgba(255,170,0,0.1)",
                  }}>
                    <div style={{display:"flex",alignItems:"center",gap:5}}>
                      <span style={{fontSize:11,color:"#ffaa00"}}>●</span>
                      <div style={{flex:1,minWidth:0}}>
                        <div style={{color:"#ffaa00",fontWeight:"bold",fontSize:11}}>{wp.id||`w${i+1}`}: {wp.label}</div>
                        <div style={{color:"rgba(255,170,0,0.45)",fontSize:9}}>{(()=>{const w=toWorld(wp.x,wp.y);return `(${w.x}, ${w.y})m`;})()} · {Math.round(wp.theta*180/Math.PI)}°</div>
                      </div>
                      <button onClick={ev=>{ev.stopPropagation();onDeleteWp(i);}} style={{...btn(false,true),padding:"1px 4px",fontSize:9}}>✕</button>
                    </div>
                    {isSel&&(
                      <div style={{display:"flex",gap:5,alignItems:"center",marginTop:5}}>
                        <span style={{fontSize:9,color:"rgba(255,170,0,0.5)"}}>θ(rad)</span>
                        <input type="number" step=".1" value={wp.theta.toFixed(2)}
                          onChange={e=>{const v=parseFloat(e.target.value)||0;setWaypoints(p=>p.map((w,j)=>j===i?{...w,theta:v}:w));}}
                          onClick={ev=>ev.stopPropagation()}
                          style={{...INPUT,width:60,padding:"2px 5px",fontSize:11}}/>
                      </div>
                    )}
                  </div>
                );
              })}
            </div>
          )}
        </div>
      )}
      <div style={{padding:10,borderTop:"1px solid rgba(0,212,255,0.1)",display:"flex",flexDirection:"column",gap:5}}>
        {onImportJSON&&<button style={{...btn(),width:"100%",justifyContent:"center",fontSize:11}} onClick={onImportJSON}>📥 semantic_map.json 불러오기</button>}
        {!onImportJSON&&onImportFile&&(
          <label style={{...btn(),width:"100%",justifyContent:"center",fontSize:11,cursor:"pointer"}}>
            📥 semantic_map.json 불러오기
            <input type="file" accept=".json" onChange={onImportFile} style={{display:"none"}}/>
          </label>
        )}
        <button style={{...btn(true),width:"100%",justifyContent:"center",fontSize:11}} onClick={onExportJSON}>⬇ semantic_map.json 저장</button>
      </div>
    </div>
  );
}

function SlamModePanel({
  hasHostAPI,
  ros2Connected,
  rosbridgeRunning,
  rosbridgeBusy,
  onStartBridge,
  slamMode,
  onEnterMode,
  onExitMode,
  slamRunning,
  slamBusy,
  slamSaveBusy,
  onStartSlam,
  onStopSlam,
  onSaveSlamResult,
  nav2Running,
  nav2Busy,
  nav2ParamsFile,
  onChangeNav2ParamsFile,
  onStartNav2,
  onStopNav2,
  slamSaveName,
  onChangeSaveName,
  slamShowFootprint,
  onChangeShowFootprint,
  slamFootprintTopic,
  onChangeFootprintTopic,
  slamShowRobotModel,
  onChangeShowRobotModel,
  slamRobotDescriptionTopic,
  onChangeRobotDescriptionTopic,
  slamShowCostmaps,
  onChangeShowCostmaps,
  slamGlobalCostmapTopic,
  onChangeGlobalCostmapTopic,
  slamLocalCostmapTopic,
  onChangeLocalCostmapTopic,
  ros2Stats,
  slamMapTopic,
  onChangeMapTopic,
  slamUseSimTime,
  onChangeUseSimTime,
  slamParamsFile,
  onChangeParamsFile,
  slamMapStats,
  defaultMapDir,
  onPickGoalTool,
  onPickSelectTool,
  onOpenWorkspaceSync,
}) {
  const mapOk=!!slamMapStats?.width;
  const rosStats=ros2Stats?.current||{};
  const saveBaseName=cleanMapBaseName(slamSaveName)||"map_MMDD_HHMM";
  return (
    <div style={{width:300,background:"#071121",borderLeft:"1px solid rgba(0,212,255,0.12)",display:"flex",flexDirection:"column",flexShrink:0}}>
      <div style={{padding:"9px 14px",borderBottom:"1px solid rgba(0,212,255,0.12)",display:"flex",alignItems:"center",gap:8}}>
        <span style={{color:"#00d4ff",fontWeight:"bold",letterSpacing:1,fontSize:12}}>SLAM 모드</span>
        <span style={{marginLeft:"auto",fontSize:9,color:slamMode?"#00e676":"rgba(0,212,255,0.35)"}}>{slamMode?"ACTIVE":"STANDBY"}</span>
      </div>
      <div style={{padding:10,display:"flex",flexDirection:"column",gap:9,overflow:"auto"}}>
        <div style={{padding:9,borderRadius:6,background:"rgba(0,0,0,0.24)",border:"1px solid rgba(0,212,255,0.08)",display:"flex",flexDirection:"column",gap:7}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",letterSpacing:1}}>MODE</div>
          <div style={{display:"flex",gap:6}}>
            {slamMode?(
              <button style={{...btn(false,true),flex:1,justifyContent:"center"}} onClick={onExitMode}>종료</button>
            ):(
              <button style={{...btn(true),flex:1,justifyContent:"center"}} onClick={onEnterMode}>SLAM 모드 열기</button>
            )}
          </div>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.34)",lineHeight:1.6}}>
            live /map은 배경으로만 갱신되고, 시맨틱 항목은 map frame 위치를 유지합니다.
          </div>
        </div>

        <div style={{padding:9,borderRadius:6,background:"rgba(0,0,0,0.24)",border:"1px solid rgba(0,212,255,0.08)",display:"flex",flexDirection:"column",gap:7}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",letterSpacing:1}}>ROS</div>
          <div style={{display:"flex",alignItems:"center",gap:6}}>
            <span style={{width:7,height:7,borderRadius:"50%",background:ros2Connected?"#00e676":"#ff5252",boxShadow:`0 0 5px ${ros2Connected?"#00e676":"#ff5252"}`}}/>
            <span style={{fontSize:10,color:ros2Connected?"#00e676":"#ff6680"}}>{ros2Connected?"rosbridge connected":"rosbridge disconnected"}</span>
            {hasHostAPI&&!rosbridgeRunning&&(
              <button style={{...btn(),marginLeft:"auto",padding:"2px 7px",fontSize:10,opacity:rosbridgeBusy ? .45 : 1}} onClick={onStartBridge} disabled={rosbridgeBusy}>Bridge</button>
            )}
          </div>
          <label style={{display:"flex",flexDirection:"column",gap:4}}>
            <span style={{fontSize:10,color:"rgba(0,212,255,0.45)"}}>Map topic</span>
            <input value={slamMapTopic} onChange={e=>onChangeMapTopic(e.target.value)} disabled={slamMode}
              style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:11,opacity:slamMode ? .55 : 1}}/>
          </label>
        </div>

        <div style={{padding:9,borderRadius:6,background:"rgba(0,0,0,0.24)",border:"1px solid rgba(0,212,255,0.08)",display:"flex",flexDirection:"column",gap:7}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",letterSpacing:1}}>SLAM TOOLBOX</div>
          <label style={{display:"flex",alignItems:"center",gap:6,color:"#8eb8c8",fontSize:10}}>
            <input type="checkbox" checked={slamUseSimTime} onChange={e=>onChangeUseSimTime(e.target.checked)} style={{width:12,height:12,accentColor:"#00d4ff"}}/>
            use_sim_time
          </label>
          <label style={{display:"flex",flexDirection:"column",gap:4}}>
            <span style={{fontSize:10,color:"rgba(0,212,255,0.45)"}}>params file</span>
            <input value={slamParamsFile} onChange={e=>onChangeParamsFile(e.target.value)} placeholder="optional"
              style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:11}}/>
          </label>
          <div style={{display:"flex",gap:6}}>
            {slamRunning?(
              <button style={{...btn(false,true),flex:1,justifyContent:"center",opacity:slamBusy ? .45 : 1}} onClick={onStopSlam} disabled={slamBusy}>■ toolbox</button>
            ):(
              <button style={{...btn(),flex:1,justifyContent:"center",opacity:hasHostAPI&&!slamBusy?1:.45}} onClick={onStartSlam} disabled={!hasHostAPI||slamBusy}>▶ toolbox</button>
            )}
          </div>
          {!hasHostAPI&&(
            <div style={{fontSize:10,color:"rgba(255,102,128,0.55)",lineHeight:1.5}}>브라우저 단독 실행에서는 SLAM 프로세스 제어를 할 수 없습니다.</div>
          )}
        </div>

        <div style={{padding:9,borderRadius:6,background:"rgba(0,0,0,0.24)",border:"1px solid rgba(0,212,255,0.08)",display:"flex",flexDirection:"column",gap:7}}>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.5)",letterSpacing:1}}>ROBOT OVERLAY</div>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.34)",lineHeight:1.6}}>
            UI는 Nav2를 실행하지 않고 costmap/footprint/robot_description 토픽만 구독합니다.
          </div>
          <label style={{display:"flex",alignItems:"center",gap:6,color:"#8eb8c8",fontSize:10}}>
            <input type="checkbox" checked={slamShowCostmaps} onChange={e=>onChangeShowCostmaps(e.target.checked)} style={{width:12,height:12,accentColor:"#00d4ff"}}/>
            nav2 costmaps
            <span style={{marginLeft:"auto",color:(rosStats.costmapTopicCount||0)>0?"#00e676":"rgba(255,102,128,0.55)",fontSize:9}}>
              {(rosStats.costmapTopicCount||0)>0?`${rosStats.costmapTopicCount} LIVE`:"WAIT"}
            </span>
          </label>
          <input value={slamGlobalCostmapTopic} onChange={e=>onChangeGlobalCostmapTopic(e.target.value)} disabled={!slamShowCostmaps}
            style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:11,opacity:slamShowCostmaps?1:.55}}/>
          <input value={slamLocalCostmapTopic} onChange={e=>onChangeLocalCostmapTopic(e.target.value)} disabled={!slamShowCostmaps}
            style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:11,opacity:slamShowCostmaps?1:.55}}/>
          <label style={{display:"flex",alignItems:"center",gap:6,color:"#8eb8c8",fontSize:10}}>
            <input type="checkbox" checked={slamShowFootprint} onChange={e=>onChangeShowFootprint(e.target.checked)} style={{width:12,height:12,accentColor:"#00d4ff"}}/>
            footprint
            <span style={{marginLeft:"auto",color:(rosStats.footprintTopicCount||0)>0?"#00e676":"rgba(255,102,128,0.55)",fontSize:9}}>
              {(rosStats.footprintTopicCount||0)>0?"LIVE":"WAIT"}
            </span>
          </label>
          <input value={slamFootprintTopic} onChange={e=>onChangeFootprintTopic(e.target.value)} disabled={!slamShowFootprint}
            style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:11,opacity:slamShowFootprint?1:.55}}/>
          <label style={{display:"flex",alignItems:"center",gap:6,color:"#8eb8c8",fontSize:10}}>
            <input type="checkbox" checked={slamShowRobotModel} onChange={e=>onChangeShowRobotModel(e.target.checked)} style={{width:12,height:12,accentColor:"#00d4ff"}}/>
            robot_description
            <span style={{marginLeft:"auto",color:(rosStats.robotModelLinkCount||0)>0?"#00e676":"rgba(255,102,128,0.55)",fontSize:9}}>
              {rosStats.robotModelLinkCount?`${rosStats.robotModelLinkCount} LINKS`:(rosStats.robotDescriptionLoaded?"URDF":"WAIT")}
            </span>
          </label>
          <input value={slamRobotDescriptionTopic} onChange={e=>onChangeRobotDescriptionTopic(e.target.value)} disabled={!slamShowRobotModel}
            style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:11,opacity:slamShowRobotModel?1:.55}}/>
        </div>

        <div style={{padding:9,borderRadius:6,background:"rgba(0,0,0,0.24)",border:`1px solid ${mapOk?"rgba(0,230,118,0.18)":"rgba(255,102,128,0.14)"}`,display:"flex",flexDirection:"column",gap:5}}>
          <div style={{fontSize:10,color:mapOk?"#00e676":"#ff6680",letterSpacing:1}}>LIVE MAP</div>
          <label style={{display:"flex",flexDirection:"column",gap:4}}>
            <span style={{fontSize:10,color:"rgba(0,212,255,0.45)"}}>저장 이름</span>
            <input value={slamSaveName} onChange={e=>onChangeSaveName(e.target.value)} placeholder="map_name"
              style={{...INPUT,width:"100%",boxSizing:"border-box",fontSize:11}}/>
          </label>
          <div style={{fontSize:9,color:"rgba(0,212,255,0.32)",lineHeight:1.45,wordBreak:"break-all"}}>
            기본 위치: {defaultMapDir}/{saveBaseName}.pgm
          </div>
          {mapOk?(
            <>
              <div style={{fontSize:11,color:"#c9fffe"}}>{slamMapStats.width}×{slamMapStats.height} @ {slamMapStats.resolution}m/px</div>
              <div style={{fontSize:9,color:"rgba(0,212,255,0.38)"}}>origin [{slamMapStats.origin.map(v=>Number(v).toFixed(3)).join(", ")}]</div>
              <div style={{fontSize:9,color:"rgba(0,212,255,0.28)"}}>{slamMapStats.frameId} · {new Date(slamMapStats.receivedAt).toLocaleTimeString()}</div>
              <button style={{...btn(true),justifyContent:"center",marginTop:3,opacity:slamSaveBusy ? .45 : 1}} onClick={onSaveSlamResult} disabled={slamSaveBusy}>
                {slamSaveBusy?"저장 중":"💾 완료 저장"}
              </button>
              {hasHostAPI&&(
                <button style={{...btn(),justifyContent:"center",marginTop:2}} onClick={onOpenWorkspaceSync}>
                  🏗 빌드
                </button>
              )}
            </>
          ):(
            <div style={{fontSize:10,color:"rgba(255,102,128,0.48)",lineHeight:1.5}}>{slamMode?`${slamMapTopic} 수신 대기 중`:"SLAM 모드를 열면 map topic을 구독합니다"}</div>
          )}
        </div>

        <div style={{padding:9,borderRadius:6,background:"rgba(0,0,0,0.24)",border:"1px solid rgba(255,102,128,0.12)",display:"flex",flexDirection:"column",gap:7}}>
          <div style={{fontSize:10,color:"rgba(255,102,128,0.72)",letterSpacing:1}}>ANNOTATION</div>
          <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:6}}>
            <button style={{...btn(true),justifyContent:"center",borderColor:"#ff6680",color:"#ff6680"}} onClick={onPickGoalTool}>🎯 골 찍기</button>
            <button style={{...btn(),justifyContent:"center"}} onClick={onPickSelectTool}>↖ 선택</button>
          </div>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.34)",lineHeight:1.6}}>
            SLAM 모드에서는 브러시/지우개 같은 맵 픽셀 편집 도구가 잠깁니다.
          </div>
        </div>
      </div>
    </div>
  );
}

// ─── Polygon drawing overlay renderer ─────────────────────────────────────────
function drawPolygonItem(ctx, shape, rt, isSel, alpha, showLabel=false) {
  const poly = shapeToPoly(shape);
  if (!poly || poly.length < 2) return;
  ctx.save();
  ctx.beginPath();
  ctx.moveTo(poly[0].x+.5, poly[0].y+.5);
  poly.slice(1).forEach(p=>ctx.lineTo(p.x+.5,p.y+.5));
  ctx.closePath();
  ctx.fillStyle = hexRgba(rt.color, (rt.alpha||0.18) * alpha);
  ctx.fill();
  ctx.strokeStyle = hexRgba(rt.color, isSel ? 0.95 : 0.6 * alpha);
  ctx.lineWidth = isSel ? 2 : 1.5;
  ctx.setLineDash(isSel ? [] : [6,3]);
  ctx.stroke();
  if (shape.poly || isSel) {
    ctx.setLineDash([]);
    poly.forEach((p,i)=>{
      ctx.beginPath(); ctx.arc(p.x+.5,p.y+.5, isSel?2.5:1.5, 0, Math.PI*2);
      ctx.fillStyle = isSel ? rt.color : hexRgba(rt.color, 0.6);
      ctx.fill();
      if (isSel) { ctx.strokeStyle="#fff"; ctx.lineWidth=0.5; ctx.stroke(); }
      if (isSel) {
        ctx.font="bold 6px monospace"; ctx.fillStyle="#fff"; ctx.textAlign="left";
        ctx.fillText(i+1, p.x+3, p.y-2);
      }
    });
  }
  if(showLabel){
    const c = polyCentroid(poly);
    const txt = `${rt.icon} ${shape.label}`;
    ctx.font = `bold 5px 'JetBrains Mono',monospace`;
    ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.setLineDash([]);
    const tm = ctx.measureText(txt);
    ctx.fillStyle = hexRgba("#000010", 0.7); ctx.fillRect(c.x-tm.width/2-2, c.y-4, tm.width+4, 9);
    ctx.fillStyle = hexRgba(rt.color, 0.95); ctx.fillText(txt, c.x, c.y);
  }
  ctx.restore();
}

// ─── Main editor ───────────────────────────────────────────────────────────────
export default function Nav2MapEditor() {
  const canvasRef  = useRef(null);
  const overlayRef = useRef(null);
  const vpRef      = useRef(null);

  const [activeTab,   setActiveTab]   = useState("edit");
  const [toolbarW,    setToolbarW]    = useState(90);
  const tbResizing    = useRef(false);
  const [semPanelW,   setSemPanelW]   = useState(300);
  const semPanelResizing = useRef(false);
  const [tool,        setTool]        = useState("brush");
  const [drawColor,   setDrawColor]   = useState(PX_OCCUPIED);
  const [brushSz,     setBrushSz]     = useState(5);
  const [zoom,        setZoom]        = useState(1);
  const [pan,         setPan]         = useState({x:50,y:50});
  const [rotation,    setRotation]    = useState(0); // degrees
  const [selectedStartTask,setSelectedStartTask]=useState(DEFAULT_START_TASK);
  const [startPoses,  setStartPoses]  = useState({});
  const [waypoints,   setWaypoints]   = useState([]);
  const [selWpIdx,    setSelWpIdx]    = useState(null);
  const [mapLoaded,   setMapLoaded]   = useState(false);
  const [status,      setStatus]      = useState("PGM 파일을 열거나 새 맵을 만드세요");
  const [canvasSize,  setCanvasSize]  = useState({w:0,h:0});
  const [cursor,      setCursor]      = useState({x:0,y:0,vis:false});

  // Semantic state (4-level: maps > rooms > carriers > objects)
  const [maps,        setMaps]        = useState([]);
  const [rooms,       setRooms]       = useState([]);
  const [carriers,    setCarriers]    = useState([]);
  const [objects,     setObjects]     = useState([]);
  const [emptySeats,  setEmptySeats]  = useState([]);
  const [goals,       setGoals]       = useState([]);
  const [goalList,    setGoalList]    = useState([]);
  const [selSemId,    setSelSemId]    = useState(null);
  const [semDlg,      setSemDlg]      = useState(null);
  const [goalDlg,     setGoalDlg]     = useState(null); // {x, y, roomId}
  const [startDlg,    setStartDlg]    = useState(null); // {pose, defaultTask, defaultId}
  const [startDraft,  setStartDraft]  = useState(null);
  const [showSemPanel,setShowSemPanel]= useState(false);
  const [showCatalogPanel,setShowCatalogPanel]= useState(false);
  const [semOpacity,  setSemOpacity]  = useState(0.8);
  const [catalogRooms,setCatalogRooms]=useState(()=>normalizeRoomNames(DEFAULT_SEMANTIC_CATALOG.rooms));
  const [catalogSources,setCatalogSources]=useState([]);
  const semanticCatalog=useMemo(()=>composeSemanticCatalog(catalogSources,catalogRooms),[catalogSources,catalogRooms]);
  const typeOptions=useMemo(()=>({
    maps:MAP_TYPES,
    rooms:buildRoomTypes(semanticCatalog),
    carriers:buildCarrierTypes(semanticCatalog),
    objects:buildObjectTypes(semanticCatalog),
  }),[semanticCatalog]);

  // ROS2 state
  const ros2Bridge = useMemo(() => new Ros2Bridge(), []);
  const [ros2State, setRos2State] = useState(ROS2_STATES.DISCONNECTED);
  const [ros2Vis, setRos2Vis] = useState({});
  const [showRos2Panel, setShowRos2Panel] = useState(false);
  const [ros2Frames, setRos2Frames] = useState({ fixed: "map" });
  const [ros2AvailFrames, setRos2AvailFrames] = useState([]);
  const [rosbridgeRunning, setRosbridgeRunning] = useState(false);
  const [rosbridgeBusy, setRosbridgeBusy] = useState(false);
  const [showSlamPanel, setShowSlamPanel] = useState(false);
  const [slamMode, setSlamMode] = useState(false);
  const [slamRunning, setSlamRunning] = useState(false);
  const [slamBusy, setSlamBusy] = useState(false);
  const [nav2Running, setNav2Running] = useState(false);
  const [nav2Busy, setNav2Busy] = useState(false);
  const [nav2ParamsFile, setNav2ParamsFile] = useState("");
  const [slamSaveBusy, setSlamSaveBusy] = useState(false);
  const [quickSaveBusy, setQuickSaveBusy] = useState(null);
  const [slamMapTopic, setSlamMapTopic] = useState("/map");
  const [slamMapStats, setSlamMapStats] = useState(null);
  const [slamUseSimTime, setSlamUseSimTime] = useState(false);
  const [slamParamsFile, setSlamParamsFile] = useState("");
  const [slamSaveName, setSlamSaveName] = useState(()=>timestampedMapName());
  const [slamShowCostmaps, setSlamShowCostmaps] = useState(true);
  const [slamGlobalCostmapTopic, setSlamGlobalCostmapTopic] = useState("/global_costmap/costmap");
  const [slamLocalCostmapTopic, setSlamLocalCostmapTopic] = useState("/local_costmap/costmap");
  const [slamShowFootprint, setSlamShowFootprint] = useState(true);
  const [slamFootprintTopic, setSlamFootprintTopic] = useState("/local_costmap/published_footprint");
  const [slamShowRobotModel, setSlamShowRobotModel] = useState(true);
  const [slamRobotDescriptionTopic, setSlamRobotDescriptionTopic] = useState("/robot_description");
  const [show3DView, setShow3DView] = useState(false);
  const [view3DMode, setView3DMode] = useState("free");
  const [view3DWidth, setView3DWidth] = useState(460);
  const view3DResizing = useRef(false);
  const [bagPath, setBagPath] = useState("");
  const [bagRunning, setBagRunning] = useState(false);
  const [bagLoop, setBagLoop] = useState(false);
  const [bagRate, setBagRate] = useState(1);
  const [bagOffset, setBagOffset] = useState(0);
  const [bagDuration, setBagDuration] = useState(0);
  const [bagSeekStep, setBagSeekStep] = useState(10);
  const [bagPaused, setBagPaused] = useState(false);
  const [bagBusy, setBagBusy] = useState(false);
  const [bagRecording, setBagRecording] = useState(false);
  const [bagRecordBusy, setBagRecordBusy] = useState(false);
  const [bagRecordDir, setBagRecordDir] = useState(DEFAULT_BAG_RECORD_DIR);
  const [bagRecordName, setBagRecordName] = useState("");
  const [bagRecordPath, setBagRecordPath] = useState("");
  const [fileDialog, setFileDialog] = useState(null);
  const fileDialogResolveRef = useRef(null);
  const [thorMdPicker, setThorMdPicker] = useState(null);
  const [showWorkspaceDlg, setShowWorkspaceDlg] = useState(false);
  const [workspaceSync, setWorkspaceSync] = useState({
    launchPath: "",
    mapPath: "",
    argName: "map",
    workspaceRoot: DEFAULT_WORKSPACE_ROOT,
    packageName: "rby1_nav2",
  });
  const [workspaceBusy, setWorkspaceBusy] = useState(false);

  const setStartPoseForTask=useCallback((task,updater)=>{
    const key=START_TASKS.includes(task)?task:DEFAULT_START_TASK;
    setStartPoses(prev=>{
      const current=prev[key]||null;
      const next=typeof updater==="function"?updater(current):updater;
      const out={...prev};
      if(!next) delete out[key];
      else out[key]={...next,id:next.id||current?.id||nextStartPoseId(prev),task:key,label:key};
      return out;
    });
  },[]);

  const pickHostPath=useCallback((options={})=>{
    if(hostAPI?.isRobotBackend&&hostAPI?.browseDir){
      return new Promise(resolve=>{
        fileDialogResolveRef.current=resolve;
        setFileDialog({...options,key:Date.now()});
      });
    }
    const fn=options.dialogType==="save"?hostAPI?.saveFileDialog:hostAPI?.openFileDialog;
    return fn?fn(options):Promise.resolve(null);
  },[]);

  const closeFileDialog=useCallback((value)=>{
    const resolve=fileDialogResolveRef.current;
    fileDialogResolveRef.current=null;
    setFileDialog(null);
    if(resolve)resolve(value||null);
  },[]);

  const pickWorkspacePath=useCallback(async(options)=>{
    if(!hostAPI?.openFileDialog&&!(hostAPI?.isRobotBackend&&hostAPI?.browseDir))return null;
    return pickHostPath(options);
  },[pickHostPath]);

  const syncWorkspaceMapPath=useCallback((mapPath)=>{
    if(!mapPath)return;
    setWorkspaceSync(prev=>({
      ...prev,
      mapPath,
      workspaceRoot: prev.workspaceRoot || DEFAULT_WORKSPACE_ROOT,
      packageName: prev.packageName || inferPackageNameFromPackagePath(mapPath) || "rby1_nav2",
    }));
  },[]);

  useEffect(() => {
    const unsub = ros2Bridge.onStateChange(s => setRos2State(s));
    return () => { unsub(); ros2Bridge.disconnect(); };
  }, [ros2Bridge]);

  // Polygon-in-progress state
  const [polyVerts,   setPolyVerts]   = useState([]);
  const [polySnap,    setPolySnap]    = useState(false);

  const [meta, setMeta] = useState({resolution:0.05,origin:[-10,-10,0],negate:0,occupied_thresh:0.65,free_thresh:0.196,filename:"map"});
  const [showNewDlg,  setShowNewDlg]  = useState(false);
  const [showMetaDlg, setShowMetaDlg] = useState(false);
  const [newW, setNewW] = useState(400);
  const [newH, setNewH] = useState(400);

  const histRef     = useRef([]);
  const histIdxRef  = useRef(-1);
  const isDrawing   = useRef(false);
  const lastPt      = useRef(null);
  const shapeStart  = useRef(null);
  const snapRef     = useRef(null);
  const isPanning   = useRef(false);
  const panOrigin   = useRef({x:0,y:0});
  const panStart    = useRef({x:0,y:0});
  const zoomRef     = useRef(1);
  const panRef      = useRef({x:50,y:50});
  const rotRef      = useRef(0);
  const polyVertsRef= useRef([]);
  const cursorCanvasRef = useRef({x:0,y:0});
  const metaRef     = useRef(meta);
  const canvasSizeRef = useRef(canvasSize);
  const mapLoadedRef = useRef(mapLoaded);
  const slamReprojectingRef = useRef(false);
  const slamHadMapRef = useRef(false);
  const slamLastStatusAtRef = useRef(0);

  // Drag-move refs
  const dragRef     = useRef(null); // {id, layer:'room'|'carrier'|'object', startPt:{x,y}}
  const hoverSemRef = useRef(null); // id of hovered semantic shape (or 'wp:idx' for waypoints)
  const startDragRef= useRef(null); // Nav2 start pose being direction-dragged on creation
  const startDraftRef= useRef(null); // temporary start pose before task/id dialog confirmation
  const wpDragRef   = useRef(null); // {idx} — waypoint being direction-dragged on creation
  const goalDragRef = useRef(null); // {id, roomId, theta} — goal being direction-dragged on creation

  useEffect(()=>{zoomRef.current=zoom;},[zoom]);
  useEffect(()=>{panRef.current=pan;},[pan]);
  useEffect(()=>{rotRef.current=rotation;},[rotation]);
  useEffect(()=>{polyVertsRef.current=polyVerts;},[polyVerts]);
  useEffect(()=>{metaRef.current=meta;},[meta]);
  useEffect(()=>{canvasSizeRef.current=canvasSize;},[canvasSize]);
  useEffect(()=>{mapLoadedRef.current=mapLoaded;},[mapLoaded]);

  // Screen coords → canvas coords (accounts for pan, zoom, rotation)
  const screenToCanvas = useCallback((sx, sy)=>{
    const dx=sx-panRef.current.x, dy=sy-panRef.current.y;
    const rad=-rotRef.current*Math.PI/180;
    const cos=Math.cos(rad), sin=Math.sin(rad);
    const rx=(dx*cos-dy*sin)/zoomRef.current;
    const ry=(dx*sin+dy*cos)/zoomRef.current;
    return{x:Math.floor(rx),y:Math.floor(ry)};
  },[]);
  const toXY = useCallback((e)=>{
    const vp=vpRef.current;if(!vp)return null;
    const r=vp.getBoundingClientRect();
    return screenToCanvas(e.clientX-r.left, e.clientY-r.top);
  },[screenToCanvas]);
  const inBounds=(p)=>{const c=canvasRef.current;return c&&p.x>=0&&p.y>=0&&p.x<c.width&&p.y<c.height;};

  // ── History (unified: canvas pixels + semantic state) ──
  // Refs to hold latest semantic state for snapshotting (avoids stale closures)
  const mapsRef     = useRef(maps);
  const roomsRef    = useRef(rooms);
  const carriersRef = useRef(carriers);
  const objectsRef  = useRef(objects);
  const emptySeatsRef = useRef(emptySeats);
  const startPosesRef= useRef(startPoses);
  const selectedStartTaskRef= useRef(selectedStartTask);
  const waypointsRef= useRef(waypoints);
  const goalsRef    = useRef(goals);
  const goalListRef = useRef(goalList);
  useEffect(()=>{mapsRef.current=maps;},[maps]);
  useEffect(()=>{roomsRef.current=rooms;},[rooms]);
  useEffect(()=>{carriersRef.current=carriers;},[carriers]);
  useEffect(()=>{objectsRef.current=objects;},[objects]);
  useEffect(()=>{emptySeatsRef.current=emptySeats;},[emptySeats]);
  useEffect(()=>{startPosesRef.current=startPoses;},[startPoses]);
  useEffect(()=>{selectedStartTaskRef.current=selectedStartTask;},[selectedStartTask]);
  useEffect(()=>{waypointsRef.current=waypoints;},[waypoints]);
  useEffect(()=>{goalsRef.current=goals;},[goals]);
  useEffect(()=>{goalListRef.current=goalList;},[goalList]);

  const saveSnap=useCallback(()=>{
    const c=canvasRef.current;if(!c)return;
    const d=c.getContext("2d").getImageData(0,0,c.width,c.height);
    const copy=new ImageData(new Uint8ClampedArray(d.data),d.width,d.height);
    const snap={img:copy, maps:[...mapsRef.current], rooms:[...roomsRef.current], carriers:[...carriersRef.current], objects:[...objectsRef.current], emptySeats:[...emptySeatsRef.current], startPoses:{...startPosesRef.current}, selectedStartTask:selectedStartTaskRef.current, waypoints:[...waypointsRef.current], goals:[...goalsRef.current], goalList:[...goalListRef.current]};
    histRef.current=histRef.current.slice(0,histIdxRef.current+1);
    histRef.current.push(snap);if(histRef.current.length>40)histRef.current.shift();
    histIdxRef.current=histRef.current.length-1;
  },[]);
  const restoreSnap=useCallback((snap)=>{
    canvasRef.current?.getContext("2d").putImageData(snap.img,0,0);
    setMaps(snap.maps||[]);setRooms(snap.rooms);setCarriers(snap.carriers);setObjects(snap.objects);setEmptySeats(snap.emptySeats||[]);setStartPoses(snap.startPoses||{});setSelectedStartTask(snap.selectedStartTask||DEFAULT_START_TASK);setWaypoints(snap.waypoints);setGoals(snap.goals||[]);setGoalList(snap.goalList||[]);
  },[]);
  const undoingRef=useRef(false); // prevent snapshot during undo/redo restore
  const draggingRef=useRef(false); // prevent snapshot during drag-move
  const undo=useCallback(()=>{if(histIdxRef.current<=0)return;undoingRef.current=true;histIdxRef.current--;restoreSnap(histRef.current[histIdxRef.current]);setStatus("↩ 되돌리기");},[restoreSnap]);
  const redo=useCallback(()=>{if(histIdxRef.current>=histRef.current.length-1)return;undoingRef.current=true;histIdxRef.current++;restoreSnap(histRef.current[histIdxRef.current]);setStatus("↪ 다시 실행");},[restoreSnap]);
  // Auto-snapshot when semantic state changes (rooms/carriers/objects/waypoints/start pose)
  const semVersionRef=useRef(0);
  useEffect(()=>{
    if(undoingRef.current){undoingRef.current=false;return;}
    if(slamReprojectingRef.current){slamReprojectingRef.current=false;return;}
    if(draggingRef.current)return; // skip during drag-move
    // Skip initial mount (saveSnap is called in initCanvas/loadPGMData)
    if(semVersionRef.current===0){semVersionRef.current=1;return;}
    saveSnap();
  },[maps,rooms,carriers,objects,emptySeats,startPoses,selectedStartTask,waypoints,goals,goalList]); // eslint-disable-line react-hooks/exhaustive-deps

  // ── Init canvas ──
  const initCanvas=useCallback((w,h,fillV=PX_UNKNOWN)=>{
    const c=canvasRef.current;if(!c)return;
    c.width=w;c.height=h;
    c.getContext("2d").fillStyle=`rgb(${fillV},${fillV},${fillV})`;c.getContext("2d").fillRect(0,0,w,h);
    setCanvasSize({w,h});histRef.current=[];histIdxRef.current=-1;saveSnap();
      setMapLoaded(true);setMaps([]);setStartPoses({});setWaypoints([]);setRooms([]);setCarriers([]);setObjects([]);setEmptySeats([]);setGoals([]);setGoalList([]);setPolyVerts([]);setRotation(0);
    _mIdx=0;_rIdx=0;_cIdx=0;_oIdx=0;_wIdx=0;_gIdx=0;
    const vp=vpRef.current;
    if(vp){const z=Math.min((vp.clientWidth-60)/w,(vp.clientHeight-60)/h,3);setZoom(z);setPan({x:(vp.clientWidth-w*z)/2,y:(vp.clientHeight-h*z)/2});}
  },[saveSnap]);

  // ── Window resize handler ──
  useEffect(()=>{
    const onResize=()=>{
      const vp=vpRef.current,c=canvasRef.current;
      if(!vp||!c||!c.width)return;
      const z=Math.min((vp.clientWidth-60)/c.width,(vp.clientHeight-60)/c.height,3);
      setZoom(z);setPan({x:(vp.clientWidth-c.width*z)/2,y:(vp.clientHeight-c.height*z)/2});
    };
    window.addEventListener("resize",onResize);
    return()=>window.removeEventListener("resize",onResize);
  },[]);

  // ── ROS2 overlay hook ──
  const requestRos2Draw = useCallback(() => { drawOverlayRef.current?.(); }, []);
  const onRos2Frames = useCallback((list) => setRos2AvailFrames(list), []);
  const { drawRos2, stats: ros2Stats, lidarWorldPoints, pathWorldPoints, cameraDataUrl } = useRos2Overlay(
    ros2Bridge, ros2Vis, meta, canvasSize, requestRos2Draw, ros2Frames, onRos2Frames
  );
  const drawOverlayRef = useRef(null);

  useEffect(()=>{
    const desired=[];
    const globalCostmapTopic=slamGlobalCostmapTopic.trim();
    const localCostmapTopic=slamLocalCostmapTopic.trim();
    const fpTopic=slamFootprintTopic.trim();
    const robotTopic=slamRobotDescriptionTopic.trim();
    if(slamMode&&slamShowCostmaps&&globalCostmapTopic){
      desired.push({
        kind:"costmap_global",
        topic:globalCostmapTopic,
        display:{enabled:true,type:"nav_msgs/msg/OccupancyGrid",viz:"costmap",icon:"▦",color:"#ff6680",alpha:.5,size:1,_slamAuto:"costmap_global"},
      });
    }
    if(slamMode&&slamShowCostmaps&&localCostmapTopic){
      desired.push({
        kind:"costmap_local",
        topic:localCostmapTopic,
        display:{enabled:true,type:"nav_msgs/msg/OccupancyGrid",viz:"costmap",icon:"▦",color:"#ffaa00",alpha:.7,size:1,_slamAuto:"costmap_local"},
      });
    }
    if(slamMode&&slamShowFootprint&&fpTopic){
      desired.push({
        kind:"footprint",
        topic:fpTopic,
        display:{enabled:true,type:"geometry_msgs/msg/PolygonStamped",viz:"footprint",icon:"▱",color:"#00bcd4",alpha:.85,size:2,_slamAuto:"footprint"},
      });
    }
    if(slamMode&&slamShowRobotModel&&robotTopic){
      desired.push({
        kind:"robot_model",
        topic:robotTopic,
        display:{enabled:true,type:"std_msgs/msg/String",viz:"robot_model",icon:"▤",color:"#b5f5ff",alpha:.9,size:1.5,_slamAuto:"robot_model"},
      });
    }
    setRos2Vis(prev=>{
      let changed=false;
      const next={...prev};
      const desiredByKey=new Set(desired.map(d=>`${d.kind}:${d.topic}`));
      for(const [topic,v] of Object.entries(next)){
        if(v?._slamAuto&&!desiredByKey.has(`${v._slamAuto}:${topic}`)){delete next[topic];changed=true;}
      }
      for(const d of desired){
        const cur=next[d.topic];
        if(!cur){next[d.topic]=d.display;changed=true;}
        else if(cur._slamAuto){
          next[d.topic]={...cur,...d.display};
          changed=true;
        }
      }
      return changed?next:prev;
    });
  },[slamMode,slamShowCostmaps,slamGlobalCostmapTopic,slamLocalCostmapTopic,slamShowFootprint,slamFootprintTopic,slamShowRobotModel,slamRobotDescriptionTopic]);

  const reprojectSemanticForMap = useCallback((fromMeta, fromSize, toMeta, toSize)=>{
    if(!fromMeta||!toMeta||!fromSize?.h||!toSize?.h)return;
    if(sameMapProjection(fromMeta,fromSize,toMeta,toSize))return;
    const hasSemantic=mapsRef.current.length||roomsRef.current.length||carriersRef.current.length||objectsRef.current.length||emptySeatsRef.current.length||goalsRef.current.length||goalListRef.current.length||waypointsRef.current.length||Object.keys(startPosesRef.current||{}).length;
    if(!hasSemantic)return;
    slamReprojectingRef.current=true;
    setMaps(p=>p.map(item=>reprojectShapeItem(item,fromMeta,fromSize,toMeta,toSize)));
    setRooms(p=>p.map(item=>reprojectShapeItem(item,fromMeta,fromSize,toMeta,toSize)));
    setCarriers(p=>p.map(item=>reprojectShapeItem(item,fromMeta,fromSize,toMeta,toSize)));
    setObjects(p=>p.map(item=>reprojectShapeItem(item,fromMeta,fromSize,toMeta,toSize)));
    setEmptySeats(p=>p.map(item=>reprojectShapeItem(item,fromMeta,fromSize,toMeta,toSize)));
    setWaypoints(p=>p.map(item=>reprojectPoseItem(item,fromMeta,fromSize,toMeta,toSize)));
    setGoals(p=>p.map(item=>reprojectPoseItem(item,fromMeta,fromSize,toMeta,toSize)));
    setStartPoses(p=>Object.fromEntries(Object.entries(p||{}).map(([task,pose])=>[task,reprojectPoseItem(pose,fromMeta,fromSize,toMeta,toSize)])));
  },[]);

  const applySlamMapMessage = useCallback((msg)=>{
    const c=canvasRef.current;
    if(!c)return;
    const oldMeta={...metaRef.current,origin:[...(metaRef.current.origin||[0,0,0])]};
    const oldSize={...canvasSizeRef.current};
    const hadMap=mapLoadedRef.current&&oldSize.w>0&&oldSize.h>0;
    let projection;
    try{
      projection=drawOccupancyGridToCanvas(c,msg);
    }catch(e){
      setStatus(`⚠ SLAM map 처리 실패: ${e.message}`);
      return;
    }
    const nextMeta={...oldMeta,resolution:projection.resolution,origin:projection.origin,filename:oldMeta.filename==="map"?"slam_live_map":oldMeta.filename};
    const nextSize={w:projection.width,h:projection.height};
    const projectionChanged=!sameMapProjection(oldMeta,oldSize,nextMeta,nextSize);
    if(hadMap&&projectionChanged)reprojectSemanticForMap(oldMeta,oldSize,nextMeta,nextSize);
    if(!hadMap||projectionChanged){
      setMeta(nextMeta);
      setCanvasSize(nextSize);
    }
    setMapLoaded(true);
    setSlamMapStats({...projection,receivedAt:Date.now(),topic:slamMapTopic});
    if(!slamHadMapRef.current){
      slamHadMapRef.current=true;
      const vp=vpRef.current;
      if(vp){
        const z=Math.min((vp.clientWidth-60)/projection.width,(vp.clientHeight-60)/projection.height,3);
        setZoom(z);setPan({x:(vp.clientWidth-projection.width*z)/2,y:(vp.clientHeight-projection.height*z)/2});setRotation(0);
      }
    }
    const now=Date.now();
    if(now-slamLastStatusAtRef.current>1500){
      slamLastStatusAtRef.current=now;
      setStatus(`SLAM live map 수신: ${projection.width}×${projection.height} (${slamMapTopic})`);
    }
    drawOverlayRef.current?.();
  },[reprojectSemanticForMap,slamMapTopic]);

  useEffect(()=>{
    if(!slamMode||ros2State!==ROS2_STATES.CONNECTED)return;
    const topic=slamMapTopic.trim()||"/map";
    slamHadMapRef.current=false;
    const unsub=ros2Bridge.subscribe(topic,"nav_msgs/msg/OccupancyGrid",applySlamMapMessage,250,{queue_length:1});
    setStatus(`SLAM 모드: ${topic} 구독 중 · 시맨틱 골을 찍을 수 있습니다`);
    return()=>unsub();
  },[slamMode,ros2State,ros2Bridge,slamMapTopic,applySlamMapMessage]);

  useEffect(()=>{
    if(!hostAPI?.rosbridgeStatus)return;
    let alive=true;
    const sync=async()=>{
      try{
        const st=await hostAPI.rosbridgeStatus();
        if(alive)setRosbridgeRunning(!!st?.running);
      }catch(e){/* ignore */}
    };
    sync();
    const timer=setInterval(sync,1000);
    return()=>{alive=false;clearInterval(timer);};
  },[]);

  useEffect(()=>{
    if(!hostAPI?.slamStatus)return;
    let alive=true;
    const sync=async()=>{
      try{
        const st=await hostAPI.slamStatus();
        if(alive)setSlamRunning(!!st?.running);
      }catch(e){/* ignore */}
    };
    sync();
    const timer=setInterval(sync,1000);
    return()=>{alive=false;clearInterval(timer);};
  },[]);

  useEffect(()=>{
    if(!hostAPI?.nav2Status)return;
    let alive=true;
    const sync=async()=>{
      try{
        const st=await hostAPI.nav2Status();
        if(alive)setNav2Running(!!st?.running);
      }catch(e){/* ignore */}
    };
    sync();
    const timer=setInterval(sync,1000);
    return()=>{alive=false;clearInterval(timer);};
  },[]);

  useEffect(()=>{
    if(!hostAPI?.rosbagStatus)return;
    let alive=true;
    const sync=async()=>{
      try{
        const st=await hostAPI.rosbagStatus();
        if(alive){
          setBagRunning(!!st?.running);
          setBagPaused(!!st?.paused);
          if(st?.finished)setBagOffset(0);
          else if(Number.isFinite(st?.offset))setBagOffset(st.offset);
          if(Number.isFinite(st?.duration)&&st.duration>0)setBagDuration(st.duration);
        }
      }catch(e){/* ignore */}
    };
    sync();
    const timer=setInterval(sync,1000);
    return()=>{alive=false;clearInterval(timer);};
  },[]);

  useEffect(()=>{
    if(!hostAPI?.rosbagRecordStatus)return;
    let alive=true;
    const sync=async()=>{
      try{
        const st=await hostAPI.rosbagRecordStatus();
        if(alive){
          setBagRecording(!!st?.running);
          if(st?.options?.outputPath)setBagRecordPath(st.options.outputPath);
          if(st?.running&&st?.options?.outputDir)setBagRecordDir(st.options.outputDir);
        }
      }catch(e){/* ignore */}
    };
    sync();
    const timer=setInterval(sync,1000);
    return()=>{alive=false;clearInterval(timer);};
  },[]);

  // ── Overlay draw ──
  const drawOverlay=useCallback(()=>{
    const ov=overlayRef.current,c=canvasRef.current;if(!ov||!c)return;
    const dpr=window.devicePixelRatio||1;
    const needResize=ov.width!==c.width*dpr||ov.height!==c.height*dpr;
    if(needResize){ov.width=c.width*dpr;ov.height=c.height*dpr;ov.style.width=c.width+"px";ov.style.height=c.height+"px";}
    const ctx=ov.getContext("2d");ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,c.width,c.height);
    const hov=hoverSemRef.current;
    const alpha=semOpacity;

    // ── Draw completed maps (behind everything) ──
    maps.forEach(m=>{
      const mt=typeOptions.maps.find(t=>t.id===m.type)||typeOptions.maps[typeOptions.maps.length-1];
      const isSel=selSemId===m.id;
      drawPolygonItem(ctx, m, mt, isSel, alpha, isSel||hov===m.id);
    });

    // ── Draw completed rooms ──
    rooms.forEach(room=>{
      const rt=typeOptions.rooms.find(t=>t.id===room.type)||typeOptions.rooms[typeOptions.rooms.length-1];
      const isSel=selSemId===room.id;
      drawPolygonItem(ctx, room, rt, isSel, alpha, isSel||hov===room.id);
    });

    // ── Draw completed carriers ──
    carriers.forEach(carrier=>{
      const ct=typeOptions.carriers.find(t=>t.id===carrier.type)||typeOptions.carriers[typeOptions.carriers.length-1];
      const isSel=selSemId===carrier.id;
      drawPolygonItem(ctx, carrier, ct, isSel, alpha, isSel||hov===carrier.id);
    });

    // ── Draw completed empty seats ──
    emptySeats.forEach(seat=>{
      const isSel=selSemId===seat.id;
      drawPolygonItem(ctx, seat, EMPTY_SEAT_TYPE, isSel, alpha, isSel||hov===seat.id);
    });

    // ── Draw completed objects ──
    objects.forEach(obj=>{
      const ot=typeOptions.objects.find(t=>t.id===obj.type)||typeOptions.objects[typeOptions.objects.length-1];
      const isSel=selSemId===obj.id;
      const showLbl=isSel||hov===obj.id;
      if(obj.point){
        const r=isSel?6:4;
        ctx.save();ctx.shadowColor=ot.color;ctx.shadowBlur=isSel?8:3;
        ctx.beginPath();ctx.arc(obj.x+.5,obj.y+.5,r,0,Math.PI*2);
        ctx.fillStyle=hexRgba(ot.color,0.9*alpha);ctx.fill();
        ctx.strokeStyle="#fff";ctx.lineWidth=0.8;ctx.setLineDash([]);ctx.stroke();
        ctx.shadowBlur=0;ctx.font=`${r+1}px serif`;ctx.textAlign="center";ctx.textBaseline="middle";
        ctx.fillText(ot.icon,obj.x+.5,obj.y+.5);
        if(showLbl){
          ctx.font="bold 5px monospace";ctx.textAlign="left";ctx.textBaseline="top";
          const lt=obj.label,lm=ctx.measureText(lt);
          ctx.fillStyle=hexRgba("#000010",.7);ctx.fillRect(obj.x+r+1,obj.y-3,lm.width+3,8);
          ctx.fillStyle=ot.color;ctx.fillText(lt,obj.x+r+2,obj.y-2);
        }
        ctx.restore();
      } else {
        drawPolygonItem(ctx, obj, ot, isSel, alpha, showLbl);
      }
    });

    // ── Draw task-specific Nav2 start poses ──
    const startDrawItems=START_TASKS
      .map(task=>({task,pose:startPoses[task],draft:false}))
      .filter(x=>x.pose);
    if(startDraft)startDrawItems.push({task:startDraft.task,pose:startDraft,draft:true});
    startDrawItems.forEach(({task,pose,draft})=>{
      const id=pose.id;
      const isSel=selSemId===id;
      const isHov=hov===id;
      const isActive=selectedStartTask===task;
      const isDragging=draft||!!startDragRef.current&&startDragRef.current.task===task;
      const sColor=isDragging?"#00ff88":isSel||isActive?"#00e676":"#36d399";
      const len=isActive?16:12,ax=pose.x+.5+Math.cos(pose.theta)*len,ay=pose.y+.5-Math.sin(pose.theta)*len;
      ctx.save();
      ctx.globalAlpha=draft ? .75 : (isActive?1:.55);
      ctx.shadowColor=sColor;ctx.shadowBlur=isDragging?8:isSel?7:3;
      ctx.beginPath();ctx.moveTo(pose.x+.5,pose.y+.5);ctx.lineTo(ax,ay);
      ctx.strokeStyle=sColor;ctx.lineWidth=isSel||isActive?2:1.2;ctx.setLineDash([]);ctx.stroke();
      const ang=Math.atan2(-(ay-(pose.y+.5)),ax-(pose.x+.5));
      ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(ax-5*Math.cos(ang-.5),ay+5*Math.sin(ang-.5));ctx.lineTo(ax-5*Math.cos(ang+.5),ay+5*Math.sin(ang+.5));ctx.closePath();
      ctx.fillStyle=sColor;ctx.fill();
      ctx.beginPath();ctx.arc(pose.x+.5,pose.y+.5,isSel||isActive?5:4,0,Math.PI*2);
      ctx.fillStyle=sColor;ctx.fill();ctx.strokeStyle="#fff";ctx.lineWidth=0.8;ctx.stroke();
      ctx.shadowBlur=0;ctx.font="bold 6px monospace";ctx.textAlign="center";ctx.textBaseline="middle";ctx.fillStyle="#00140a";ctx.fillText(task[0],pose.x+.5,pose.y+.5);
      if(isDragging){
        const deg=Math.round(pose.theta*180/Math.PI);
        ctx.font="bold 7px monospace";ctx.fillStyle="#00ff88";ctx.textAlign="left";ctx.textBaseline="top";
        ctx.fillText(`${deg}°`,pose.x+len+2,pose.y-7);
      } else if(isSel||isHov||isActive){
        const label=`START ${task}`;
        ctx.font="bold 5px monospace";ctx.textAlign="left";ctx.textBaseline="top";
        const lm=ctx.measureText(label);
        ctx.fillStyle=hexRgba("#000010",.75);ctx.fillRect(pose.x+7,pose.y-4,lm.width+4,9);
        ctx.fillStyle="#00e676";ctx.fillText(label,pose.x+9,pose.y-3);
      }
      ctx.restore();
    });

    // ── Draw waypoints ──
    const wpDragging=wpDragRef.current;
    waypoints.forEach((wp,i)=>{
      const isSel=i===selWpIdx;
      const isDragging=wpDragging&&wpDragging.idx===i;
      ctx.save();
      ctx.shadowColor=isDragging?"#00ff88":isSel?"#ff4455":"#ffaa00";ctx.shadowBlur=isDragging?6:3;
      ctx.beginPath();ctx.arc(wp.x+.5,wp.y+.5,3,0,Math.PI*2);
      ctx.fillStyle=isDragging?"#00cc66":isSel?"#ff2233":"#ff8800";ctx.fill();
      ctx.strokeStyle="#fff";ctx.lineWidth=0.8;ctx.setLineDash([]);ctx.stroke();
      ctx.beginPath();ctx.arc(wp.x+.5,wp.y+.5,1,0,Math.PI*2);ctx.fillStyle="#fff";ctx.shadowBlur=0;ctx.fill();
      const len=10,ax=wp.x+.5+Math.cos(wp.theta)*len,ay=wp.y+.5-Math.sin(wp.theta)*len;
      ctx.beginPath();ctx.moveTo(wp.x+.5,wp.y+.5);ctx.lineTo(ax,ay);
      ctx.strokeStyle=isDragging?"#00ff88":isSel?"#ff4455":"#ffaa00";ctx.lineWidth=1.2;ctx.stroke();
      const ang=Math.atan2(-(ay-(wp.y+.5)),ax-(wp.x+.5));
      ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(ax-4*Math.cos(ang-.5),ay+4*Math.sin(ang-.5));ctx.lineTo(ax-4*Math.cos(ang+.5),ay+4*Math.sin(ang+.5));
      ctx.fillStyle=isDragging?"#00ff88":isSel?"#ff4455":"#ffaa00";ctx.fill();
      ctx.shadowBlur=0;
      // Draw ID label
      ctx.font="bold 6px monospace";ctx.fillStyle="#00d4ff";
      ctx.fillText(wp.id||`w${i+1}`,wp.x+4,wp.y+2);
      // If dragging, show angle hint
      if(isDragging){
        const deg=Math.round(wp.theta*180/Math.PI);
        ctx.font="bold 7px monospace";ctx.fillStyle="#00ff88";ctx.textAlign="left";ctx.textBaseline="top";
        ctx.fillText(`${deg}°`,wp.x+len+2,wp.y-7);
      }
      ctx.restore();
    });

    // ── Draw goals ──
    const goalCreating=goalDragRef.current;
    goals.forEach((g,gi)=>{
      const isSel=selSemId===g.id;
      const isHov=hov===g.id;
      const isDragging=goalCreating&&goalCreating.id===g.id;
      ctx.save();
      // Direction arrow
      const gColor=isDragging?"#00ff88":isSel?"#ff3366":"#ff6680";
      const alen=14,ax=g.x+.5+Math.cos(g.theta)*alen,ay=g.y+.5-Math.sin(g.theta)*alen;
      ctx.beginPath();ctx.moveTo(g.x+.5,g.y+.5);ctx.lineTo(ax,ay);
      ctx.strokeStyle=gColor;ctx.lineWidth=isSel?2:1.3;ctx.setLineDash([]);ctx.stroke();
      const ang=Math.atan2(-(ay-(g.y+.5)),ax-(g.x+.5));
      ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(ax-5*Math.cos(ang-.5),ay+5*Math.sin(ang-.5));ctx.lineTo(ax-5*Math.cos(ang+.5),ay+5*Math.sin(ang+.5));ctx.closePath();
      ctx.fillStyle=gColor;ctx.fill();
      // Dashed line to target
      const target=[...carriers,...objects,...emptySeats].find(s=>s.id===g.target_id);
      if(target){
        let tx,ty;
        if(target.point){tx=target.x;ty=target.y;}
        else{const poly=shapeToPoly(target);if(poly){const c=polyCentroid(poly);tx=c.x;ty=c.y;}else{tx=target.x+(target.w||0)/2;ty=target.y+(target.h||0)/2;}}
        ctx.beginPath();ctx.moveTo(g.x+.5,g.y+.5);ctx.lineTo(tx+.5,ty+.5);
        ctx.strokeStyle="rgba(255,102,128,0.35)";ctx.lineWidth=1;ctx.setLineDash([4,3]);ctx.stroke();ctx.setLineDash([]);
      }
      // Goal icon
      ctx.shadowColor=gColor;ctx.shadowBlur=isDragging?6:isSel?8:4;
      ctx.beginPath();ctx.arc(g.x+.5,g.y+.5,isSel?5:4,0,Math.PI*2);
      ctx.fillStyle=gColor;ctx.fill();
      ctx.strokeStyle="#fff";ctx.lineWidth=0.8;ctx.stroke();
      ctx.shadowBlur=0;
      ctx.font=`${isSel?7:5}px serif`;ctx.textAlign="center";ctx.textBaseline="middle";
      ctx.fillStyle="#fff";ctx.fillText("🎯",g.x+.5,g.y+.5);
      // Label + drag angle
      if(isDragging){
        const deg=Math.round(g.theta*180/Math.PI);
        ctx.font="bold 7px monospace";ctx.fillStyle="#00ff88";ctx.textAlign="left";ctx.textBaseline="top";
        ctx.fillText(`${deg}°`,g.x+alen+2,g.y-7);
      } else if(isSel||isHov){
        ctx.font="bold 5px monospace";ctx.textAlign="left";ctx.textBaseline="top";
        const lt=`${g.id}: ${g.label}`,lm=ctx.measureText(lt);
        ctx.fillStyle=hexRgba("#000010",.75);ctx.fillRect(g.x+6,g.y-4,lm.width+4,9);
        ctx.fillStyle="#ff6680";ctx.fillText(lt,g.x+8,g.y-3);
      }
      ctx.restore();
    });

    // ── Draw ROS2 overlay (lidar, path, robot pose) ──
    drawRos2(ctx);

    // ── Draw in-progress polygon ──
    const pverts=polyVertsRef.current;
    const isPolyTool=["semPolyMap","semPolyRoom","semPolyCarrier","semPolyObj","semPolyEmptySeat"].includes(tool);
    if(isPolyTool&&pverts.length>0){
      const accentColor=tool==="semPolyMap"?"#90a4ae":tool==="semPolyRoom"?"#00d4ff":tool==="semPolyCarrier"?"#4fc3f7":tool==="semPolyEmptySeat"?"#00e676":"#ffaa00";
      const cur=cursorCanvasRef.current;
      ctx.save();

      if(pverts.length>2){
        ctx.beginPath();ctx.moveTo(pverts[0].x+.5,pverts[0].y+.5);
        pverts.slice(1).forEach(p=>ctx.lineTo(p.x+.5,p.y+.5));
        ctx.closePath();
        ctx.fillStyle=hexRgba(accentColor,.07);ctx.fill();
      }

      ctx.strokeStyle=accentColor;ctx.lineWidth=1.5;ctx.setLineDash([5,3]);
      ctx.beginPath();ctx.moveTo(pverts[0].x+.5,pverts[0].y+.5);
      pverts.forEach(p=>ctx.lineTo(p.x+.5,p.y+.5));
      ctx.stroke();

      if(pverts.length>=1){
        ctx.setLineDash([4,4]);ctx.globalAlpha=0.5;
        ctx.beginPath();ctx.moveTo(pverts[pverts.length-1].x+.5,pverts[pverts.length-1].y+.5);
        ctx.lineTo(cur.x+.5,cur.y+.5);ctx.stroke();
        ctx.globalAlpha=1;
      }
      ctx.setLineDash([]);

      pverts.forEach((p,i)=>{
        const isFirst=i===0;
        ctx.beginPath();ctx.arc(p.x+.5,p.y+.5,isFirst?POLY_FIRST_VERTEX_RADIUS:POLY_VERTEX_RADIUS,0,Math.PI*2);
        ctx.fillStyle=polySnap&&isFirst?"#00ff88":accentColor;
        ctx.fill();
        ctx.strokeStyle="#fff";ctx.lineWidth=0.5;ctx.stroke();
        ctx.font="bold 5px monospace";ctx.fillStyle="#fff";ctx.textAlign="left";ctx.textBaseline="top";
        ctx.fillText(i+1,p.x+3,p.y-6);
      });

      if(polySnap&&pverts.length>=3){
        ctx.beginPath();ctx.arc(pverts[0].x+.5,pverts[0].y+.5,POLY_SNAP_RING_RADIUS,0,Math.PI*2);
        ctx.strokeStyle="#00ff88";ctx.lineWidth=0.9;ctx.setLineDash([2,2]);ctx.stroke();
        ctx.setLineDash([]);
        ctx.font="bold 6px monospace";ctx.fillStyle="#00ff88";ctx.textAlign="center";ctx.textBaseline="bottom";
        ctx.fillText("닫기",pverts[0].x,pverts[0].y-7);
      }

      ctx.font="bold 8px monospace";ctx.textAlign="left";ctx.textBaseline="top";
      ctx.fillStyle=hexRgba("#000010",.75);ctx.fillRect(cur.x+8,cur.y+8,52,13);
      ctx.fillStyle=accentColor;ctx.fillText(`꼭짓점 ${pverts.length}개`,cur.x+10,cur.y+9);

      ctx.restore();
    }

    // ── Rect preview for semRect tools ──
    if(["semRectMap","semRectRoom","semRectCarrier","semRectObj","semRectEmptySeat"].includes(tool)&&shapeStart.current&&cursorCanvasRef.current){
      const s=shapeStart.current,e=cursorCanvasRef.current;
      const x=Math.min(s.x,e.x),y=Math.min(s.y,e.y),w=Math.abs(e.x-s.x),h=Math.abs(e.y-s.y);
      if(w>1&&h>1){
        const col=tool==="semRectMap"?"#90a4ae":tool==="semRectRoom"?"#00d4ff":tool==="semRectCarrier"?"#4fc3f7":tool==="semRectEmptySeat"?"#00e676":"#ffaa00";
        ctx.save();ctx.strokeStyle=col;ctx.lineWidth=1.5;ctx.setLineDash([6,3]);
        ctx.fillStyle=hexRgba(col,.06);ctx.fillRect(x,y,w,h);ctx.strokeRect(x+.5,y+.5,w,h);
        ctx.restore();
      }
    }
  },[maps,rooms,carriers,objects,emptySeats,startPoses,startDraft,selectedStartTask,waypoints,goals,selWpIdx,selSemId,semOpacity,polySnap,tool,drawRos2,typeOptions]);

  useEffect(()=>{drawOverlayRef.current=drawOverlay;},[drawOverlay]);
  useEffect(()=>{drawOverlay();},[drawOverlay]);

  // ── Paint helpers ──
  const paintDot=(ctx,x,y,sz,v)=>{ctx.fillStyle=`rgb(${v},${v},${v})`;const h=Math.floor(sz/2);ctx.fillRect(x-h,y-h,sz,sz);};
  const paintSeg=(ctx,x0,y0,x1,y1,sz,v)=>{const dx=x1-x0,dy=y1-y0,steps=Math.max(Math.ceil(Math.sqrt(dx*dx+dy*dy)),1);for(let i=0;i<=steps;i++)paintDot(ctx,Math.round(x0+dx*i/steps),Math.round(y0+dy*i/steps),sz,v);};
  const drawShapePreview=(ctx,x0,y0,x1,y1,shiftKey,t)=>{
    const v=drawColor;ctx.strokeStyle=`rgb(${v},${v},${v})`;ctx.lineWidth=brushSz;ctx.lineCap="round";ctx.setLineDash([]);
    if(t==="line"){ctx.beginPath();ctx.moveTo(x0+.5,y0+.5);ctx.lineTo(x1+.5,y1+.5);ctx.stroke();}
    else if(t==="rect"){let rw=x1-x0,rh=y1-y0;if(shiftKey){const s=Math.min(Math.abs(rw),Math.abs(rh));rw=Math.sign(rw)*s;rh=Math.sign(rh)*s;}ctx.strokeRect(x0+.5,y0+.5,rw,rh);}
    else if(t==="circle"){ctx.beginPath();ctx.arc(x0+.5,y0+.5,Math.sqrt((x1-x0)**2+(y1-y0)**2),0,Math.PI*2);ctx.stroke();}
  };

  // ── Confirm polygon finish ──
  const finishPolygon=useCallback((verts)=>{
    if(verts.length<3)return;
    const mode=tool==="semPolyMap"?"map":tool==="semPolyRoom"?"room":tool==="semPolyCarrier"?"carrier":tool==="semPolyEmptySeat"?"emptySeat":"object";
    setSemDlg({mode, poly:verts});
    setPolyVerts([]);setPolySnap(false);
  },[tool]);

  // ── Mouse Down ──
  const onMouseDown=useCallback((e)=>{
    // Ignore clicks on buttons/labels/inputs inside viewport
    if(e.target.closest("button,label,input,a"))return;
    // Block all drawing when no map is loaded
    if(!mapLoaded)return;
    if(e.button===1||(e.button===0&&e.altKey)){isPanning.current=true;panOrigin.current={...panRef.current};panStart.current={x:e.clientX,y:e.clientY};e.preventDefault();return;}
    if(e.button!==0)return;
    const pt=toXY(e);if(!pt)return;

    // ── Polygon tools ──
    if(tool==="semPolyMap"||tool==="semPolyRoom"||tool==="semPolyCarrier"||tool==="semPolyObj"||tool==="semPolyEmptySeat"){
      if(!inBounds(pt))return;
      const pverts=polyVertsRef.current;
      if(pverts.length>=3){
        const dx=pt.x-pverts[0].x,dy=pt.y-pverts[0].y;
        if(Math.sqrt(dx*dx+dy*dy)<SNAP_RADIUS){
          finishPolygon([...pverts]); return;
        }
      }
      const newVerts=[...pverts,{x:pt.x,y:pt.y}];
      setPolyVerts(newVerts);
      setStatus(`🔷 꼭짓점 ${newVerts.length}개 · 더블클릭 또는 첫 꼭짓점 클릭으로 닫기`);
      return;
    }

    // ── Semantic rect tools ──
    if(tool==="semRectMap"||tool==="semRectRoom"||tool==="semRectCarrier"||tool==="semRectObj"||tool==="semRectEmptySeat"){
      if(!inBounds(pt))return;
      isDrawing.current=true;shapeStart.current=pt;lastPt.current=pt;return;
    }

    // ── Point object ──
    if(tool==="semPoint"){
      if(!inBounds(pt))return;
      setSemDlg({mode:"object",point:true,pt});
      return;
    }

    // ── Semantic goal (click+drag like waypoint; room is assigned when available) ──
    if(tool==="semGoal"){
      if(!inBounds(pt))return;
      const hitRoom=[...rooms].reverse().find(r=>hitTestShape(r,pt.x,pt.y));
      const newId=guid();
      const newGoal={id:newId,x:pt.x,y:pt.y,theta:0,label:newId,room_id:hitRoom?.id||null,target_id:null};
      goalDragRef.current={id:newId,roomId:hitRoom?.id||null,theta:0};
      isDrawing.current=true; shapeStart.current=pt;
      setGoals(p=>[...p, newGoal]);
      setStatus(`🎯 드래그하여 방향 지정${hitRoom?` (${hitRoom.label})`:" (방 미할당)"}`);
      return;
    }

    // ── Semantic select (with drag-move) ──
    if(tool==="semSelect"){
      const START_HIT_R=8;
      const hitStart=START_TASKS.map(task=>({task,pose:startPoses[task]})).filter(x=>x.pose).reverse()
        .find(({pose})=>Math.hypot(pt.x-pose.x,pt.y-pose.y)<=START_HIT_R);
      if(hitStart){
        const id=hitStart.pose.id;
        setSelectedStartTask(hitStart.task);
        setSelSemId(id);setSelWpIdx(null);
        dragRef.current={id, task:hitStart.task, layer:"startPose", startPt:{x:pt.x,y:pt.y}};
        return;
      }
      // Check waypoints first (small targets, priority)
      const WP_HIT_R=5;
      const wpHitIdx=waypoints.findLastIndex(wp=>Math.hypot(pt.x-wp.x,pt.y-wp.y)<=WP_HIT_R);
      if(wpHitIdx>=0){
        setSelWpIdx(wpHitIdx);setSelSemId(null);
        dragRef.current={wpIdx:wpHitIdx, layer:"waypoint", startPt:{x:pt.x,y:pt.y}};
        return;
      }
      // Check goals (point-based hit)
      const GOAL_HIT_R=6;
      const hitGoal=[...goals].reverse().find(g=>Math.hypot(pt.x-g.x,pt.y-g.y)<=GOAL_HIT_R);
      if(hitGoal){
        setSelSemId(hitGoal.id);setSelWpIdx(null);
        dragRef.current={id:hitGoal.id, layer:"goal", startPt:{x:pt.x,y:pt.y}};
        return;
      }
      // Priority: empty seats > objects > carriers > rooms > maps
      const hit=[...emptySeats].reverse().find(s=>hitTestShape(s,pt.x,pt.y))
        || [...objects].reverse().find(s=>hitTestShape(s,pt.x,pt.y))
        || [...carriers].reverse().find(s=>hitTestShape(s,pt.x,pt.y))
        || [...rooms].reverse().find(s=>hitTestShape(s,pt.x,pt.y))
        || [...maps].reverse().find(s=>hitTestShape(s,pt.x,pt.y));
      if(hit){
        setSelSemId(hit.id);setSelWpIdx(null);
        // Determine which layer
        const layer=maps.find(m=>m.id===hit.id)?"map":rooms.find(r=>r.id===hit.id)?"room":carriers.find(c=>c.id===hit.id)?"carrier":emptySeats.find(e=>e.id===hit.id)?"emptySeat":"object";
        dragRef.current={id:hit.id, layer, startPt:{x:pt.x,y:pt.y}};
      } else {
        setSelSemId(null);setSelWpIdx(null);
        dragRef.current=null;
      }
      return;
    }

    // ── Nav2 start pose ──
    if(tool==="startPose"){
      if(inBounds(pt)){
        const defaultTask=nextStartTask(startPoses,selectedStartTask);
        const newId=startPoses[defaultTask]?.id||nextStartPoseId(startPoses);
        const newStart={id:newId,task:defaultTask,x:pt.x,y:pt.y,theta:0,label:defaultTask};
        startDragRef.current={id:newId,task:defaultTask};
        startDraftRef.current=newStart;
        isDrawing.current=true; shapeStart.current=pt;
        setStartDraft(newStart);
        setSelSemId(null);setSelWpIdx(null);
        setStatus("⌂ 드래그하여 시작 방향 지정");
      }
      return;
    }

    // ── Edit tools ──
    if(tool==="waypoint"){
      if(inBounds(pt)){
        // All side-effects OUTSIDE state updater to avoid StrictMode double-invoke bugs
        const newId=wuid();
        const newIdx=waypoints.length;
        const newWp={id:newId,x:pt.x,y:pt.y,theta:0,label:newId};
        wpDragRef.current={idx:newIdx};
        isDrawing.current=true; shapeStart.current=pt;
        setWaypoints(p=>[...p, newWp]);
        setStatus("📍 드래그하여 방향 지정 (또는 클릭으로 기본 방향)");
      }
      return;
    }
    if(slamMode&&MAP_EDIT_TOOLS.has(tool)){
      setStatus("SLAM 모드에서는 live map 픽셀 편집이 잠겨 있습니다. 시맨틱 도구를 사용하세요");
      return;
    }
    if(tool==="fill"){ if(!inBounds(pt))return;const c=canvasRef.current;const ctx=c.getContext("2d");const id=ctx.getImageData(0,0,c.width,c.height);floodFill(id.data,c.width,c.height,pt.x,pt.y,drawColor);ctx.putImageData(id,0,0);saveSnap();return; }
    isDrawing.current=true;shapeStart.current=pt;lastPt.current=pt;
    if(["line","rect","circle"].includes(tool)){const c=canvasRef.current;snapRef.current=c.getContext("2d").getImageData(0,0,c.width,c.height);}
    if((tool==="brush"||tool==="eraser")&&inBounds(pt)){const c=canvasRef.current;paintDot(c.getContext("2d"),pt.x,pt.y,tool==="eraser"?brushSz*2:brushSz,tool==="eraser"?PX_FREE:drawColor);}
  },[tool,drawColor,brushSz,toXY,saveSnap,finishPolygon,maps,rooms,carriers,objects,emptySeats,startPoses,selectedStartTask,waypoints,goals,mapLoaded,slamMode]);

  // ── Double click → close polygon ──
  const onDblClick=useCallback((e)=>{
    if(tool==="semPolyMap"||tool==="semPolyRoom"||tool==="semPolyCarrier"||tool==="semPolyObj"||tool==="semPolyEmptySeat"){
      const pverts=polyVertsRef.current;
      if(pverts.length>=3) finishPolygon([...pverts]);
      e.preventDefault();
    }
  },[tool,finishPolygon]);

  // ── Mouse Move ──
  const onMouseMove=useCallback((e)=>{
    const vp=vpRef.current;if(!vp)return;
    const r=vp.getBoundingClientRect();
    const {x:cx,y:cy}=screenToCanvas(e.clientX-r.left, e.clientY-r.top);
    setCursor({x:cx,y:cy,vis:true});
    cursorCanvasRef.current={x:cx,y:cy};

    if(isPanning.current){setPan({x:panOrigin.current.x+e.clientX-panStart.current.x,y:panOrigin.current.y+e.clientY-panStart.current.y});return;}

    // ── Drag-move semantic items & waypoints ──
    if(tool==="semSelect"&&dragRef.current){
      const d=dragRef.current;
      const dx=cx-d.startPt.x, dy=cy-d.startPt.y;
      if(Math.abs(dx)<2&&Math.abs(dy)<2)return; // deadzone
      d.startPt={x:cx,y:cy};
      draggingRef.current=true;
      if(d.layer==="startPose") setStartPoseForTask(d.task,p=>p?{...p,x:p.x+dx,y:p.y+dy}:p);
      else if(d.layer==="waypoint") setWaypoints(p=>p.map((wp,i)=>i===d.wpIdx?{...wp,x:wp.x+dx,y:wp.y+dy}:wp));
      else if(d.layer==="goal") setGoals(p=>p.map(g=>g.id===d.id?{...g,x:g.x+dx,y:g.y+dy}:g));
      else if(d.layer==="map") setMaps(p=>p.map(m=>m.id===d.id?moveShape(m,dx,dy):m));
      else if(d.layer==="room") setRooms(p=>p.map(r=>r.id===d.id?moveShape(r,dx,dy):r));
      else if(d.layer==="carrier") setCarriers(p=>p.map(c=>c.id===d.id?moveShape(c,dx,dy):c));
      else if(d.layer==="emptySeat") setEmptySeats(p=>p.map(seat=>seat.id===d.id?moveShape(seat,dx,dy):seat));
      else setObjects(p=>p.map(o=>o.id===d.id?moveShape(o,dx,dy):o));
      return;
    }

    // Poly snap detection
    if((tool==="semPolyMap"||tool==="semPolyRoom"||tool==="semPolyCarrier"||tool==="semPolyObj"||tool==="semPolyEmptySeat")&&polyVertsRef.current.length>=3){
      const p0=polyVertsRef.current[0];
      setPolySnap(Math.hypot(cx-p0.x,cy-p0.y)<SNAP_RADIUS);
    }

    // Hover detection for semantic labels (priority: goals > objects > carriers > rooms)
    {
      const hit=START_TASKS.map(task=>({id:startPoses[task]?.id,pose:startPoses[task]})).filter(x=>x.pose&&x.id).reverse().find(({pose})=>Math.hypot(cx-pose.x,cy-pose.y)<=8)
        || [...goals].reverse().find(g=>Math.hypot(cx-g.x,cy-g.y)<=6)
        || [...emptySeats].reverse().find(s=>hitTestShape(s,cx,cy))
        || [...objects].reverse().find(s=>hitTestShape(s,cx,cy))
        || [...carriers].reverse().find(s=>hitTestShape(s,cx,cy))
        || [...rooms].reverse().find(s=>hitTestShape(s,cx,cy))
        || [...maps].reverse().find(s=>hitTestShape(s,cx,cy));
      const newHov=hit?hit.id:null;
      if(hoverSemRef.current!==newHov){hoverSemRef.current=newHov;drawOverlay();}
    }

    // Always redraw overlay for poly/rect preview
    if(["semPolyMap","semPolyRoom","semPolyCarrier","semPolyObj","semPolyEmptySeat","semRectMap","semRectRoom","semRectCarrier","semRectObj","semRectEmptySeat"].includes(tool)){
      drawOverlay();
    }

    if(!isDrawing.current||!shapeStart.current)return;
    const pt=toXY(e);if(!pt)return;

    // Nav2 start direction drag
    if(tool==="startPose"&&startDragRef.current!=null){
      const s=shapeStart.current;if(!s)return;
      const dx=pt.x-s.x, dy=pt.y-s.y;
      if(Math.hypot(dx,dy)>3){
        const theta=Math.atan2(-dy,dx);
        draggingRef.current=true;
        const next=startDraftRef.current?{...startDraftRef.current,theta}:null;
        if(next){
          startDraftRef.current=next;
          setStartDraft(next);
        }
      }
      return;
    }

    // Waypoint direction drag
    if(tool==="waypoint"&&wpDragRef.current!=null){
      const wpd=wpDragRef.current;
      const s=shapeStart.current;if(!s)return;
      const dx=pt.x-s.x, dy=pt.y-s.y;
      if(Math.hypot(dx,dy)>3){
        const theta=Math.atan2(-dy,dx);
        const idx=wpd.idx;
        draggingRef.current=true;
        setWaypoints(p=>p.map((wp,i)=>i===idx?{...wp,theta}:wp));
      }
      return;
    }

    // Goal direction drag
    if(tool==="semGoal"&&goalDragRef.current!=null){
      const gd=goalDragRef.current;
      const s=shapeStart.current;if(!s)return;
      const dx=pt.x-s.x, dy=pt.y-s.y;
      if(Math.hypot(dx,dy)>3){
        const theta=Math.atan2(-dy,dx);
        gd.theta=theta;
        draggingRef.current=true;
        setGoals(p=>p.map(g=>g.id===gd.id?{...g,theta}:g));
      }
      return;
    }

    const c=canvasRef.current;if(!c)return;const ctx=c.getContext("2d");

    if(tool==="brush"||tool==="eraser"){
      if(!inBounds(pt))return;
      const v=tool==="eraser"?PX_FREE:drawColor,sz=tool==="eraser"?brushSz*2:brushSz;
      if(lastPt.current)paintSeg(ctx,lastPt.current.x,lastPt.current.y,pt.x,pt.y,sz,v);else paintDot(ctx,pt.x,pt.y,sz,v);
      lastPt.current=pt;
    } else if(["line","rect","circle"].includes(tool)&&snapRef.current){
      ctx.putImageData(snapRef.current,0,0);drawShapePreview(ctx,shapeStart.current.x,shapeStart.current.y,pt.x,pt.y,e.shiftKey,tool);
    }
    lastPt.current=pt;
  },[tool,drawColor,brushSz,toXY,screenToCanvas,drawOverlay,maps,rooms,carriers,objects,emptySeats,startPoses,setStartPoseForTask,goals]);

  // ── Mouse Up ──
  const onMouseUp=useCallback((e)=>{
    if(isPanning.current){isPanning.current=false;return;}

    // End drag — save snap after move completes (next tick so state refs are fresh)
    if(dragRef.current){
      dragRef.current=null;
      if(draggingRef.current){draggingRef.current=false;setTimeout(saveSnap,0);}
    }

    if(!isDrawing.current)return;
    isDrawing.current=false;
    const pt=lastPt.current;lastPt.current=null;

    // Nav2 start direction drag complete
    if(tool==="startPose"&&startDragRef.current!=null){
      const s=shapeStart.current;
      const draft=startDraftRef.current;
      startDragRef.current=null; shapeStart.current=null;
      draggingRef.current=false;
      if(draft){
        setStartDlg({pose:draft,defaultTask:draft.task,defaultId:draft.id});
        setStatus("⌂ 시작점 task/ID 선택");
      }else if(s){
        setStatus(`⌂ 시작점 (${s.x},${s.y})`);
      }
      return;
    }

    // Waypoint direction drag complete
    if(tool==="waypoint"&&wpDragRef.current!=null){
      const s=shapeStart.current;
      wpDragRef.current=null; shapeStart.current=null;
      draggingRef.current=false;
      if(s) setStatus(`📍 웨이포인트 (${s.x},${s.y})`);
      setTimeout(saveSnap,0);
      return;
    }

    // Goal direction drag complete → open dialog to assign target
    if(tool==="semGoal"&&goalDragRef.current!=null){
      const gd=goalDragRef.current;
      goalDragRef.current=null; shapeStart.current=null;
      draggingRef.current=false;
      // Open dialog for target selection
      const g=goalsRef.current.find(goal=>goal.id===gd.id);
      if(g) setGoalDlg({x:g.x,y:g.y,roomId:gd.roomId,goalId:gd.id,theta:g.theta});
      else setGoalDlg({x:null,y:null,roomId:gd.roomId,goalId:gd.id,theta:gd.theta||0});
      return;
    }

    if(["brush","eraser","line","rect","circle"].includes(tool))saveSnap();

    // Semantic rect complete
    if((tool==="semRectMap"||tool==="semRectRoom"||tool==="semRectCarrier"||tool==="semRectObj"||tool==="semRectEmptySeat")&&shapeStart.current&&pt){
      const x=Math.min(shapeStart.current.x,pt.x),y=Math.min(shapeStart.current.y,pt.y);
      const w=Math.abs(pt.x-shapeStart.current.x),h=Math.abs(pt.y-shapeStart.current.y);
      if(w>5&&h>5){
        const mode=tool==="semRectMap"?"map":tool==="semRectRoom"?"room":tool==="semRectCarrier"?"carrier":tool==="semRectEmptySeat"?"emptySeat":"object";
        setSemDlg({mode,rect:{x,y,w,h}});
      }
    }
    snapRef.current=null;shapeStart.current=null;
  },[tool,saveSnap]);

  const onMouseLeave=useCallback(()=>{setCursor(v=>({...v,vis:false}));if(hoverSemRef.current){hoverSemRef.current=null;drawOverlay();}onMouseUp();},[onMouseUp,drawOverlay]);

  const cancelStartDialog=useCallback(()=>{
    startDraftRef.current=null;
    setStartDraft(null);
    setStartDlg(null);
  },[]);

  const onStartConfirm=useCallback((task,nextRaw)=>{
    if(!startDlg)return false;
    const key=START_TASKS.includes(task)?task:DEFAULT_START_TASK;
    const nextId=String(nextRaw||"").trim();
    if(!nextId){setStatus("⚠ 시작점 ID는 비워둘 수 없습니다");return false;}
    const usedIds=[
      ...maps.map(m=>m.id),...rooms.map(r=>r.id),...carriers.map(c=>c.id),...objects.map(o=>o.id),...emptySeats.map(seat=>seat.id),
      ...goals.map(g=>g.id),...waypoints.map((wp,i)=>wp.id||`w${i+1}`),
      ...Object.entries(startPoses).filter(([t])=>t!==key).map(([,p])=>p?.id).filter(Boolean),
    ];
    if(usedIds.includes(nextId)){
      setStatus(`⚠ 이미 있는 ID: ${nextId}`);
      return false;
    }
    const nextPose={...startDlg.pose,id:nextId,task:key,label:key};
    setStartPoseForTask(key,nextPose);
    setSelectedStartTask(key);
    setSelSemId(nextId);setSelWpIdx(null);
    startDraftRef.current=null;
    setStartDraft(null);
    setStartDlg(null);
    setTimeout(saveSnap,0);
    setStatus(`⌂ 시작점 설정: ${key} (${nextId})`);
    return true;
  },[startDlg,maps,rooms,carriers,objects,emptySeats,goals,waypoints,startPoses,setStartPoseForTask,saveSnap]);

  // ── Semantic confirm ──
  const onSemConfirm=useCallback((type,label,requestedId="")=>{
    if(!semDlg)return;
    const {mode,rect,poly,point,pt}=semDlg;
    const itemInsideParent=(parent,item,threshold=0.8)=>{
      const parentPoly=shapeToPoly(parent)||[];
      if(parentPoly.length===0)return false;
      if(item?.point||item?.x!=null&&item?.y!=null&&item?.w==null&&!item?.poly){
        return pointInPoly(item.x,item.y,parentPoly);
      }
      const childPoly=shapeToPoly(item)||[];
      return childPoly.length>0&&polyMostlyInside(parentPoly,childPoly,threshold);
    };
    const currentParentContains=(parents,parentId,item)=>{
      const parent=parents.find(p=>p.id===parentId);
      return parent?itemInsideParent(parent,item):false;
    };
    const usedIds=()=>[
      ...maps.map(m=>m.id),...rooms.map(r=>r.id),...carriers.map(c=>c.id),...objects.map(o=>o.id),...emptySeats.map(seat=>seat.id),
      ...goals.map(g=>g.id),...waypoints.map((wp,i)=>wp.id||`w${i+1}`),
      ...Object.values(startPoses).map(p=>p?.id).filter(Boolean),
    ];
    const requestedSemanticId=String(requestedId||"").trim();
    const rejectDuplicateId=(id)=>{
      if(id&&usedIds().includes(id)){
        setStatus(`⚠ 이미 있는 ID: ${id}`);
        return true;
      }
      return false;
    };

    if(mode==="map"){
      const mt=typeOptions.maps.find(t=>t.id===type)||typeOptions.maps[typeOptions.maps.length-1];
      const newMap=poly
        ?{id:muid(),type,label,color:mt.color,poly}
        :{id:muid(),type,label,color:mt.color,...rect};
      const adoptedRoomIds=rooms
        .filter(r=>itemInsideParent(newMap,r)&&(!r.mapId||!currentParentContains(maps,r.mapId,r)))
        .map(r=>r.id);
      const adoptedRoomSet=new Set(adoptedRoomIds);
      if(adoptedRoomSet.size>0)setRooms(p=>p.map(r=>adoptedRoomSet.has(r.id)?{...r,mapId:newMap.id}:r));
      setMaps(p=>[...p,newMap]);setSelSemId(newMap.id);
      setStatus(`🗺 맵 영역 추가: ${label} (${poly?`${poly.length}꼭짓점`:`${rect.w}×${rect.h}px`})${adoptedRoomSet.size?` · ${adoptedRoomSet.size}방 포함`:""}`);
    } else if(mode==="room"){
      const rt=typeOptions.rooms.find(t=>t.id===type)||typeOptions.rooms[typeOptions.rooms.length-1];
      const newRoom=poly
        ?{id:ruid(),type,label,color:rt.color,poly}
        :{id:ruid(),type,label,color:rt.color,...rect};
      // Auto map assignment (80% overlap)
      const rpoly=shapeToPoly(newRoom)||[];
      const parentMap=maps.find(m=>{
        const mp=shapeToPoly(m)||[];
        return mp.length>0&&polyMostlyInside(mp,rpoly,0.8);
      });
      if(parentMap)newRoom.mapId=parentMap.id;
      const adoptedCarrierIds=carriers
        .filter(c=>itemInsideParent(newRoom,c)&&(!c.roomId||!currentParentContains(rooms,c.roomId,c)))
        .map(c=>c.id);
      const adoptedEmptySeatIds=emptySeats
        .filter(seat=>itemInsideParent(newRoom,seat)&&(!seat.roomId||!currentParentContains(rooms,seat.roomId,seat)))
        .map(seat=>seat.id);
      const adoptedObjIds=objects
        .filter(o=>itemInsideParent(newRoom,o)&&(!o.roomId||!currentParentContains(rooms,o.roomId,o)))
        .map(o=>o.id);
      const adoptedGoalIds=goals
        .filter(g=>itemInsideParent(newRoom,g)&&(!g.room_id||!currentParentContains(rooms,g.room_id,g)))
        .map(g=>g.id);
      const adoptedCarrierSet=new Set(adoptedCarrierIds);
      const adoptedEmptySeatSet=new Set(adoptedEmptySeatIds);
      const adoptedObjSet=new Set(adoptedObjIds);
      const adoptedGoalSet=new Set(adoptedGoalIds);
      setRooms(p=>[...p,newRoom]);setSelSemId(newRoom.id);
      if(adoptedCarrierSet.size>0)setCarriers(p=>p.map(c=>adoptedCarrierSet.has(c.id)?{...c,roomId:newRoom.id}:c));
      if(adoptedEmptySeatSet.size>0)setEmptySeats(p=>p.map(seat=>adoptedEmptySeatSet.has(seat.id)?{...seat,roomId:newRoom.id}:seat));
      if(adoptedObjSet.size>0)setObjects(p=>p.map(o=>adoptedObjSet.has(o.id)?{...o,roomId:newRoom.id}:o));
      if(adoptedGoalSet.size>0)setGoals(p=>p.map(g=>adoptedGoalSet.has(g.id)?{...g,room_id:newRoom.id}:g));
      const adoptedCount=adoptedCarrierSet.size+adoptedEmptySeatSet.size+adoptedObjSet.size+adoptedGoalSet.size;
      setStatus(`🏠 방 추가: ${label}${parentMap?` (${parentMap.label} 내)`:""} (${poly?`${poly.length}꼭짓점`:`${rect.w}×${rect.h}px`})${adoptedCount?` · ${adoptedCount}개 하위 항목 포함`:""}`);
    } else if(mode==="carrier"){
      const ct=typeOptions.carriers.find(t=>t.id===type)||typeOptions.carriers[typeOptions.carriers.length-1];
      if(rejectDuplicateId(requestedSemanticId))return false;
      const idMatch=requestedSemanticId.match(/^c(\d+)$/);
      if(idMatch)_cIdx=Math.max(_cIdx,Number(idMatch[1]));
      const id=requestedSemanticId||cuid();
      const newCarrier=poly
        ?{id,type,label,color:ct.color,z:0,poly}
        :{id,type,label,color:ct.color,z:0,...rect};
      // Auto room assignment (80% overlap)
      const cpoly=shapeToPoly(newCarrier)||[];
      const parentRoom=rooms.find(r=>{
        const rp=shapeToPoly(r)||[];
        return rp.length>0&&polyMostlyInside(rp,cpoly,0.8);
      });
      if(parentRoom)newCarrier.roomId=parentRoom.id;
      const adoptedObjIds=objects
        .filter(o=>itemInsideParent(newCarrier,o)&&(!o.carrierId||!currentParentContains(carriers,o.carrierId,o)))
        .map(o=>o.id);
      const adoptedObjSet=new Set(adoptedObjIds);
      setCarriers(p=>[...p,newCarrier]);setSelSemId(newCarrier.id);
      if(adoptedObjSet.size>0)setObjects(p=>p.map(o=>adoptedObjSet.has(o.id)?{...o,carrierId:newCarrier.id,roomId:o.roomId||newCarrier.roomId||null}:o));
      setStatus(`📦 캐리어 추가: ${label}${parentRoom?` (${parentRoom.label} 내)`:""}${adoptedObjSet.size?` · ${adoptedObjSet.size}객체 포함`:""}`);
    } else if(mode==="emptySeat"){
      if(rejectDuplicateId(requestedSemanticId))return false;
      const idMatch=requestedSemanticId.match(/^e(\d+)$/);
      if(idMatch)_eIdx=Math.max(_eIdx,Number(idMatch[1]));
      const id=requestedSemanticId||euid();
      const seatLabel=String(label||"").trim()||id;
      const newSeat=poly
        ?{id,type:type||EMPTY_SEAT_TYPE.id,label:seatLabel,color:EMPTY_SEAT_TYPE.color,poly}
        :{id,type:type||EMPTY_SEAT_TYPE.id,label:seatLabel,color:EMPTY_SEAT_TYPE.color,...rect};
      const parentRoom=rooms.find(r=>itemInsideParent(r,newSeat));
      if(parentRoom)newSeat.roomId=parentRoom.id;
      setEmptySeats(p=>[...p,newSeat]);setSelSemId(newSeat.id);
      setStatus(`▧ 빈좌석 추가: ${seatLabel} (${id})${parentRoom?` · ${parentRoom.label} 내`:""}`);
    } else {
      const ot=typeOptions.objects.find(t=>t.id===type)||typeOptions.objects[typeOptions.objects.length-1];
      const isPoint=!!point;
      if(rejectDuplicateId(requestedSemanticId))return false;
      const idMatch=requestedSemanticId.match(/^o(\d+)$/);
      if(idMatch)_oIdx=Math.max(_oIdx,Number(idMatch[1]));
      const id=requestedSemanticId||ouid();
      const newObj=isPoint
        ?{id,type,label,color:ot.color,point:true,x:pt?pt.x:poly?polyCentroid(poly).x:rect.x+rect.w/2,y:pt?pt.y:poly?polyCentroid(poly).y:rect.y+rect.h/2}
        :poly
          ?{id,type,label,color:ot.color,poly}
          :{id,type,label,color:ot.color,...rect};
      // Auto carrier assignment first, then room (80% overlap)
      const shape=shapeToPoly(newObj)||[];
      const parentCarrier=carriers.find(c=>{
        const cp=shapeToPoly(c)||[];
        return cp.length>0&&(newObj.point?pointInPoly(newObj.x,newObj.y,cp):polyMostlyInside(cp,shape,0.8));
      });
      if(parentCarrier)newObj.carrierId=parentCarrier.id;
      const parentRoom=rooms.find(r=>{
        const rp=shapeToPoly(r)||[];
        return rp.length>0&&(newObj.point?pointInPoly(newObj.x,newObj.y,rp):polyMostlyInside(rp,shape,0.8));
      });
      if(parentRoom)newObj.roomId=parentRoom.id;
      setObjects(p=>[...p,newObj]);setSelSemId(newObj.id);
      setStatus(`🔹 객체 추가: ${label} (${newObj.id})${parentCarrier?` · ${parentCarrier.label} 위`:parentRoom?` · ${parentRoom.label} 내`:""}`);
    }
    setSemDlg(null);
    return true;
  },[semDlg,maps,rooms,carriers,objects,emptySeats,goals,waypoints,startPoses,typeOptions]);

  // ── Goal confirm (update goal placed by drag with target + label) ──
  const onGoalConfirm=useCallback((targetId,label,goalIdOverride=null,goalListItemId=null)=>{
    if(!goalDlg)return;
    const {goalId,theta:dragTheta}=goalDlg;
    const nextGoalId=String(goalIdOverride||"").trim();
    if(nextGoalId&&goalsRef.current.some(g=>g.id===nextGoalId&&g.id!==goalId)){
      setStatus(`⚠ 이미 있는 goal_id입니다: ${nextGoalId}`);
      return false;
    }
    const finalGoalId=nextGoalId||goalId;
    // If user didn't drag (theta still 0), auto-calculate direction toward target
    const target=targetId?[...carriers,...objects,...emptySeats].find(s=>s.id===targetId):null;
    setGoals(p=>p.map(g=>{
      if(g.id!==goalId)return g;
      let theta=dragTheta||0;
      // If theta is effectively 0 (no drag), auto-face target
      if(Math.abs(theta)<0.01&&target){
        let tx,ty;
        if(target.point){tx=target.x;ty=target.y;}
        else{const poly=shapeToPoly(target);if(poly){const c=polyCentroid(poly);tx=c.x;ty=c.y;}else{tx=target.x+(target.w||0)/2;ty=target.y+(target.h||0)/2;}}
        theta=Math.atan2(-(ty-g.y),tx-g.x);
      }
      return{...g,id:finalGoalId,label,target_id:targetId||null,theta};
    }));
    if(nextGoalId&&selSemId===goalId)setSelSemId(nextGoalId);
    setGoalDlg(null);
    setTimeout(saveSnap,0);
    setStatus(`🎯 시맨틱 골 설정: ${label}${finalGoalId?` (${finalGoalId})`:""}`);
    return true;
  },[goalDlg,carriers,objects,emptySeats,saveSnap,selSemId]);

  // ── Wheel zoom ──
  const onWheel=useCallback((e)=>{
    e.preventDefault();
    const f=e.deltaY<0?1.12:.89,vp=vpRef.current;if(!vp)return;
    const r=vp.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;
    setZoom(z=>{const nz=Math.min(Math.max(z*f,.05),16);setPan(p=>({x:mx-(mx-p.x)*(nz/z),y:my-(my-p.y)*(nz/z)}));return nz;});
  },[]);
  const rotateMap=useCallback((deg)=>{
    const vp=vpRef.current;if(!vp)return;
    // Rotate around viewport center
    const vcx=vp.clientWidth/2, vcy=vp.clientHeight/2;
    const rad=deg*Math.PI/180;
    const cos=Math.cos(rad), sin=Math.sin(rad);
    const px=panRef.current.x-vcx, py=panRef.current.y-vcy;
    setPan({x:vcx+px*cos-py*sin, y:vcy+px*sin+py*cos});
    setRotation(r=>r+deg);
  },[]);
  useEffect(()=>{const vp=vpRef.current;if(!vp)return;vp.addEventListener("wheel",onWheel,{passive:false});return()=>vp.removeEventListener("wheel",onWheel);},[onWheel]);

  // ── Delete selected semantic item or waypoint ──
  const deleteSelected=useCallback(()=>{
    if(selWpIdx!=null){
      setWaypoints(p=>p.filter((_,i)=>i!==selWpIdx));
      setSelWpIdx(null);
      return;
    }
    if(!selSemId)return;
    const selectedStartTaskById=startTaskFromId(startPoses,selSemId);
    if(selectedStartTaskById){
      const task=selectedStartTaskById||selectedStartTask;
      setStartPoseForTask(task,null);
      setSelSemId(null);
      return;
    }
    setMaps(p=>p.filter(m=>m.id!==selSemId));
    setRooms(p=>p.filter(r=>r.id!==selSemId));
    setCarriers(p=>p.filter(c=>c.id!==selSemId));
    setObjects(p=>p.filter(o=>o.id!==selSemId));
    setEmptySeats(p=>p.filter(seat=>seat.id!==selSemId));
    setGoals(p=>p.filter(g=>g.id!==selSemId));
    setSelSemId(null);
  },[selSemId,selWpIdx,selectedStartTask,startPoses,setStartPoseForTask]);

  // ── Keyboard ──
  useEffect(()=>{
    const onKey=(e)=>{
      if(["INPUT","TEXTAREA"].includes(e.target.tagName))return;
      if((e.ctrlKey||e.metaKey)&&e.key==="z"){e.preventDefault();undo();return;}
      if((e.ctrlKey||e.metaKey)&&(e.key==="y"||(e.shiftKey&&e.key==="Z"))){e.preventDefault();redo();return;}
      if(e.key==="Escape"){
        setSelSemId(null);setSemDlg(null);setGoalDlg(null);setStartDlg(null);setStartDraft(null);
        startDraftRef.current=null;startDragRef.current=null;
        if(polyVertsRef.current.length>0){setPolyVerts([]);setPolySnap(false);setStatus("⎋ 다각형 취소");}
        return;
      }
      if(e.key==="Enter"&&(tool==="semPolyMap"||tool==="semPolyRoom"||tool==="semPolyCarrier"||tool==="semPolyObj"||tool==="semPolyEmptySeat")){
        const pverts=polyVertsRef.current;
        if(pverts.length>=3){finishPolygon([...pverts]);}
        return;
      }
      // Backspace: remove last polygon vertex OR delete selected item/waypoint
      if(e.key==="Backspace"){
        if((tool==="semPolyMap"||tool==="semPolyRoom"||tool==="semPolyCarrier"||tool==="semPolyObj"||tool==="semPolyEmptySeat")&&polyVertsRef.current.length>0){
          setPolyVerts(p=>p.slice(0,-1));
        } else if(selSemId||selWpIdx!=null){
          e.preventDefault();
          deleteSelected();
        }
        return;
      }
      if(e.key==="Delete"&&(selSemId||selWpIdx!=null)){
        deleteSelected();
        return;
      }
      const em={b:"brush",e:"eraser",l:"line",r:"rect",c:"circle",f:"fill"};
      const sm={s:"startPose",w:"waypoint",q:"semPolyEmptySeat","1":"semRectRoom","2":"semPolyRoom","3":"semRectCarrier","4":"semPolyCarrier","5":"semRectObj","6":"semPolyObj","7":"semPoint","8":"semSelect","9":"semGoal","0":"semSelect"};
      // Rotation: [ ] for ±15°, Shift+[ Shift+] for ±90°
      if(e.key==="["){rotateMap(e.shiftKey?-90:-15);return;}
      if(e.key==="]"){rotateMap(e.shiftKey?90:15);return;}
      if(!e.ctrlKey&&!e.metaKey){
        if(em[e.key.toLowerCase()]){
          if(slamMode){
            setActiveTab("semantic");setTool("semGoal");setStatus("SLAM 모드에서는 맵 픽셀 편집 도구가 잠겨 있습니다");
          }else{
            setTool(em[e.key.toLowerCase()]);setActiveTab("edit");if(polyVertsRef.current.length>0){setPolyVerts([]);setPolySnap(false);}
          }
        }
        if(sm[e.key.toLowerCase()]||sm[e.key]){setTool(sm[e.key.toLowerCase()]||sm[e.key]);setActiveTab("semantic");}
      }
    };
    window.addEventListener("keydown",onKey);return()=>window.removeEventListener("keydown",onKey);
  },[undo,redo,selSemId,selWpIdx,tool,finishPolygon,deleteSelected,rotateMap,slamMode]);

  // ── File I/O (host API + browser fallback) ──
  const loadPGMData=(name, buffer)=>{
    try{
      const{width,height,data}=parsePGM(buffer);
      const c=canvasRef.current;c.width=width;c.height=height;
      const ctx=c.getContext("2d");const id=ctx.createImageData(width,height);
      for(let i=0;i<width*height;i++){id.data[i*4]=id.data[i*4+1]=id.data[i*4+2]=data[i];id.data[i*4+3]=255;}
      ctx.putImageData(id,0,0);
      setCanvasSize({w:width,h:height});histRef.current=[];histIdxRef.current=-1;saveSnap();
      setMapLoaded(true);setStartPoses({});setWaypoints([]);setPolyVerts([]);
      setMeta(m=>({...m,filename:name.replace(/\.pgm$/i,"")}));
      setStatus(`✅ 로드: ${width}×${height}px`);
      const vp=vpRef.current;
      if(vp){const z=Math.min((vp.clientWidth-60)/width,(vp.clientHeight-60)/height,3);setZoom(z);setPan({x:(vp.clientWidth-width*z)/2,y:(vp.clientHeight-height*z)/2});}
      return {w:width,h:height};
    }catch(err){setStatus("⚠ PGM 오류: "+err.message);return null;}
  };

  const loadYAMLData=(text,yamlName)=>{
    const parsed=parseYAML(text);
    const imageBase=parsed.image?basenameFromPath(parsed.image).replace(/\.pgm$/i,""):null;
    const yamlBase=yamlName?yamlName.replace(/\.(yaml|yml)$/i,""):null;
    setMeta(m=>({...m,...parsed,filename:imageBase||yamlBase||m.filename}));
    setStatus("✅ YAML 로드");
    return parsed;
  };

  const loadSemanticJSONData=useCallback((text,jsonName="semantic_map.json",options={})=>{
    try{
      const data=typeof text==="string"?JSON.parse(text):text;
      if(!data||typeof data!=="object")throw new Error("JSON 객체가 아닙니다");
      const semMeta=data.metadata||{};
      const loadedRooms=normalizeRoomNames(
        semMeta?.semantic_catalog?.rooms||data?.semantic_catalog?.rooms||semMeta?.room_catalog||data?.room_catalog,
        null
      );
      if(loadedRooms.length)setCatalogRooms(loadedRooms);
      const canvas=canvasRef.current;
      const currentSize=options.canvasSize||(canvas&&canvas.width&&canvas.height?{w:canvas.width,h:canvas.height}:canvasSize);
      const semSize=semanticImageSize(semMeta);
      if((!canvas||!canvas.width||!canvas.height)&&semSize){
        initCanvas(semSize.w,semSize.h,PX_UNKNOWN);
      }else if((!canvas||!canvas.width||!canvas.height)&&!semSize){
        setStatus("⚠ semantic JSON만으로는 맵 크기를 알 수 없습니다. PGM/YAML을 먼저 여세요");
        return false;
      }

      const activeMeta={...meta,...(options.meta||{})};
      const sourceResolution=finiteNumber(semMeta.resolution,finiteNumber(activeMeta.resolution,0.05));
      const rawOrigin=Array.isArray(semMeta.origin)?semMeta.origin:activeMeta.origin;
      const sourceOrigin=[
        finiteNumber(rawOrigin?.[0],0),
        finiteNumber(rawOrigin?.[1],0),
        finiteNumber(rawOrigin?.[2],0),
      ];
      const sourceH=finiteNumber(semSize?.h,finiteNumber(currentSize?.h,canvas?.height||canvasSize.h||0));
      const projectWorld=(p)=>{
        const x=finiteNumber(p?.x),y=finiteNumber(p?.y);
        if(x==null||y==null||!sourceH)return null;
        const px=worldToPixel(x,y,sourceOrigin,sourceResolution,sourceH);
        return finitePoint(px);
      };
      const getPixelPoint=(item)=>{
        return finitePoint(item?._pixel)
          || finitePoint(item?.pixel_pos)
          || finitePoint(item?.pixel_position)
          || finitePoint(item?.pixelPosition)
          || projectWorld(item?.position||item?.world_pos||item?.worldPosition);
      };
      const getPosePoint=(item)=>getPixelPoint(item)||projectWorld(item);
      const getPixelShape=(item)=>{
        const pxPoly=finitePoly(item?._pixel?.polygon)
          || finitePoly(item?.pixel_polygon)
          || finitePoly(item?.pixel_poly)
          || finitePoly(item?.pixelPolygon);
        if(pxPoly)return {poly:pxPoly};
        const pxRect=finiteRect(item?._pixel?.rect)||finiteRect(item?.pixel_rect)||finiteRect(item?.pixelRect);
        if(pxRect)return pxRect;
        const worldPolyRaw=item?.polygon||item?.world_polygon||item?.worldPolygon;
        if(Array.isArray(worldPolyRaw)){
          const poly=worldPolyRaw.map(projectWorld).filter(Boolean);
          if(poly.length>=3)return {poly};
        }
        const wr=item?.world_rect||item?.worldRect;
        if(wr){
          const x=finiteNumber(wr.x),y=finiteNumber(wr.y),w=finiteNumber(wr.w),h=finiteNumber(wr.h);
          if(x!=null&&y!=null&&w!=null&&h!=null){
            const poly=[{x,y},{x:x+w,y},{x:x+w,y:y-h},{x,y:y-h}].map(projectWorld).filter(Boolean);
            if(poly.length>=3)return {poly};
          }
        }
        const bb=item?.bbox;
        if(bb?.min&&bb?.max){
          const min=bb.min,max=bb.max;
          const poly=[{x:min.x,y:max.y},{x:max.x,y:max.y},{x:max.x,y:min.y},{x:min.x,y:min.y}].map(projectWorld).filter(Boolean);
          if(poly.length>=3)return {poly};
        }
        return null;
      };
      const importArea=(item,i,typeList,idPrefix,defaultType,extra={})=>{
        const shape=getPixelShape(item);
        if(!shape)return null;
        const type=item.type||defaultType;
        const tt=typeList.find(t=>t.id===type)||typeList[typeList.length-1];
        return {
          id:String(item.id||`${idPrefix}${i+1}`),
          type,
          label:String(item.label||tt.label||`${idPrefix}${i+1}`),
          color:tt.color,
          ...shape,
          ...extra,
        };
      };

      const nextMaps=(Array.isArray(data.maps)?data.maps:[])
        .map((m,i)=>importArea(m,i,typeOptions.maps,"m","map"))
        .filter(Boolean);
      const importRoomTypes=loadedRooms.length?buildRoomTypes({rooms:loadedRooms}):typeOptions.rooms;
      const nextRooms=(Array.isArray(data.rooms)?data.rooms:[])
        .map((r,i)=>importArea(r,i,importRoomTypes,"r","custom",{mapId:r?.mapId||r?.map_id||null}))
        .filter(Boolean);
      const nextCarriers=(Array.isArray(data.carriers)?data.carriers:[])
        .map((c,i)=>importArea(c,i,typeOptions.carriers,"c","custom",{roomId:c?.roomId||c?.room_id||null,z:finiteNumber(c?.z??c?.z_m??c?.height,0)}))
        .filter(Boolean);
      const nextObjects=(Array.isArray(data.objects)?data.objects:[])
        .map((o,i)=>{
          const type=o.type||"custom";
          const ot=typeOptions.objects.find(t=>t.id===type)||typeOptions.objects[typeOptions.objects.length-1];
          const base={id:String(o.id||`o${i+1}`),type,label:String(o.label||ot.label||`o${i+1}`),color:ot.color,carrierId:o.carrierId||o.carrier_id||null,roomId:o.roomId||o.room_id||null};
          const shape=getPixelShape(o);
          const point=getPixelPoint(o);
          if((o.is_point===true||o.point===true||!shape)&&point)return {...base,point:true,x:point.x,y:point.y};
          return shape?{...base,...shape}:null;
        })
        .filter(Boolean);
      const nextEmptySeats=(Array.isArray(data.empty_seats)?data.empty_seats:Array.isArray(data.emptySeats)?data.emptySeats:[])
        .map((seat,i)=>{
          const shape=getPixelShape(seat);
          if(!shape)return null;
          const id=String(seat.id||`e${i}`);
          return {
            id,
            type:seat.type||EMPTY_SEAT_TYPE.id,
            label:String(seat.label||id),
            color:seat.color||EMPTY_SEAT_TYPE.color,
            roomId:seat.roomId||seat.room_id||null,
            ...shape,
          };
        })
        .filter(Boolean);
      const importStartPose=(raw,task=DEFAULT_START_TASK)=>{
        const p=getPosePoint(raw);
        if(!raw||!p)return null;
        const key=START_TASKS.includes(task)?task:DEFAULT_START_TASK;
        return{id:String(raw.id||nextStartPoseId(nextStartPoses)),task:key,label:key,x:p.x,y:p.y,theta:thetaFromSemanticPose(raw)};
      };
      const nextStartPoses={};
      const startPosesRaw=Array.isArray(data.start_poses)?data.start_poses:[];
      startPosesRaw.forEach((raw,i)=>{
        const task=raw?.task||START_TASKS[i]||DEFAULT_START_TASK;
        const pose=importStartPose(raw,task);
        if(pose)nextStartPoses[pose.task]=pose;
      });
      const nextSelectedStartTask=START_TASKS.find(task=>nextStartPoses[task])||selectedStartTask||DEFAULT_START_TASK;
      const nextWaypoints=(Array.isArray(data.waypoints)?data.waypoints:[])
        .map((wp,i)=>{
          const p=getPosePoint(wp);
          if(!p)return null;
          const id=String(wp.id||`w${i+1}`);
          return{id,label:wp.label||id,x:p.x,y:p.y,theta:thetaFromSemanticPose(wp)};
        })
        .filter(Boolean);
      const nextGoals=(Array.isArray(data.goals)?data.goals:[])
        .map((g,i)=>{
          const p=getPosePoint(g);
          if(!p)return null;
          const id=String(g.id||`g${i+1}`);
          return{id,label:g.label||id,x:p.x,y:p.y,theta:thetaFromSemanticPose(g),room_id:g.room_id||g.roomId||null,target_id:g.target_id||g.targetId||null};
        })
        .filter(Boolean);

      const next={maps:nextMaps,rooms:nextRooms,carriers:nextCarriers,objects:nextObjects,emptySeats:nextEmptySeats,waypoints:nextWaypoints,goals:nextGoals};
      syncSemanticCounters(next);
      setMaps(nextMaps);setRooms(nextRooms);setCarriers(nextCarriers);setObjects(nextObjects);setEmptySeats(nextEmptySeats);
      setStartPoses(nextStartPoses);setSelectedStartTask(nextSelectedStartTask);setWaypoints(nextWaypoints);setGoals(nextGoals);
      setSelSemId(null);setSelWpIdx(null);setPolyVerts([]);setPolySnap(false);
      setActiveTab("semantic");setTool("semSelect");setShowSemPanel(true);
      const metaPatch={};
      if(finiteNumber(semMeta.resolution)!=null)metaPatch.resolution=finiteNumber(semMeta.resolution);
      if(Array.isArray(semMeta.origin)&&semMeta.origin.length>=2)metaPatch.origin=sourceOrigin;
      const semanticBase=jsonName.replace(/_semantic\.json$/i,"").replace(/\.json$/i,"");
      setMeta(m=>({...m,...metaPatch,filename:(m.filename==="map"&&semanticBase)?semanticBase:m.filename}));
      const count=nextMaps.length+nextRooms.length+nextCarriers.length+nextObjects.length+nextEmptySeats.length+nextWaypoints.length+nextGoals.length+Object.keys(nextStartPoses).length;
      setTimeout(()=>{saveSnap();drawOverlayRef.current?.();},0);
      setStatus(`✅ 시맨틱 로드: ${jsonName} (${count}개 항목)`);
      return true;
    }catch(err){
      setStatus(`⚠ semantic JSON 오류: ${err.message}`);
      return false;
    }
  },[meta,canvasSize,initCanvas,saveSnap,typeOptions,selectedStartTask]);

  const tryAutoLoadSemantic=useCallback(async(dir,baseName,options={})=>{
    if(!hostAPI?.readFile||!dir||!baseName)return false;
    const clean=baseName.replace(/\.(pgm|yaml|yml)$/i,"").replace(/_semantic$/i,"");
    const candidates=[`${clean}_semantic.json`,"semantic_map.json"];
    for(const candidate of candidates){
      try{
        const text=await hostAPI.readFile(`${dir}/${candidate}`,"utf-8");
        if(text&&loadSemanticJSONData(text,candidate,options))return true;
      }catch(e){/* not found or unreadable; try next */}
    }
    return false;
  },[loadSemanticJSONData]);

  const handleSemanticOpen=useCallback(async()=>{
    if(!hostAPI?.openFileDialog){setStatus("⚠ semantic JSON 열기는 Electron 또는 robot backend에서만 가능합니다");return;}
    const filePath=await pickHostPath({
      defaultPath:DEFAULT_MAP_SEARCH_DIR,
      filters:[{name:"Semantic JSON",extensions:["json"]},{name:"All files",extensions:["*"]}],
      properties:["openFile"],
    });
    if(!filePath)return;
    const name=filePath.split("/").pop();
    const text=await hostAPI.readFile(filePath,"utf-8");
    loadSemanticJSONData(text,name);
  },[loadSemanticJSONData,pickHostPath]);

  const handleSemanticFile=async(e)=>{
    const file=e.target.files?.[0];
    e.target.value="";
    if(!file)return;
    await loadSemanticJSONData(await file.text(),file.name);
  };

  const normalizeGoalListItems=useCallback((rawItems)=>{
    const goalById=new Map(goalsRef.current.map(g=>[g.id,g]));
    return (rawItems||[]).map((item,i)=>{
      if(typeof item==="string"||typeof item==="number"){
        const goalId=String(item);
        return {id:`gl${i+1}`,label:goalById.get(goalId)?.label||goalId,goal_id:goalId};
      }
      if(!item||typeof item!=="object")return null;
      const rawId=String(item.id||"");
      const goalId=String(item.goal_id??item.goalId??item.goal??item.target_goal??item.targetGoal??(!/^gl\d+$/i.test(rawId)?rawId:""));
      const listId=String(item.list_id??item.listId??(/^gl\d+$/i.test(rawId)?rawId:`gl${i+1}`));
      const label=String(item.label??item.name??goalById.get(goalId)?.label??(goalId||listId));
      return {id:listId,label,goal_id:goalId};
    }).filter(Boolean);
  },[]);

  const loadGoalListJSONData=useCallback((text,jsonName="goal_list.json")=>{
    try{
      const data=typeof text==="string"?JSON.parse(text):text;
      const rawItems=Array.isArray(data)?data
        :Array.isArray(data?.goal_list)?data.goal_list
          :Array.isArray(data?.goals)?data.goals
            :[];
      const nextGoalList=normalizeGoalListItems(rawItems);
      setGoalList(nextGoalList);
      _glIdx=maxIdNumber("gl",nextGoalList);
      setShowSemPanel(true);
      setTimeout(saveSnap,0);
      setStatus(`✅ 골 리스트 로드: ${jsonName} (${nextGoalList.length}개)`);
      return true;
    }catch(err){
      setStatus(`⚠ 골 리스트 JSON 오류: ${err.message}`);
      return false;
    }
  },[normalizeGoalListItems,saveSnap]);

  const handleGoalListOpen=useCallback(async()=>{
    if(!hostAPI?.openFileDialog){setStatus("⚠ 골 리스트 열기는 Electron 또는 robot backend에서만 가능합니다");return;}
    const base=cleanMapBaseName(meta.filename||"map")||"map";
    const filePath=await pickHostPath({
      defaultPath:`${DEFAULT_MAP_SEARCH_DIR}/${base}_goal_list.json`,
      filters:[{name:"Goal list JSON",extensions:["json"]},{name:"All files",extensions:["*"]}],
      properties:["openFile"],
    });
    if(!filePath)return;
    const name=filePath.split("/").pop();
    const text=await hostAPI.readFile(filePath,"utf-8");
    loadGoalListJSONData(text,name);
  },[loadGoalListJSONData,meta.filename,pickHostPath]);

  const handleGoalListFile=async(e)=>{
    const file=e.target.files?.[0];
    e.target.value="";
    if(!file)return;
    await loadGoalListJSONData(await file.text(),file.name);
  };

  const importCatalogTexts=useCallback((items)=>{
    const incoming=items.map(it=>{
      const catalog=parseSemanticMarkdown(it.text);
      return {
        id:catalogSourceId(),
        kind:"md",
        name:it.name||"catalog.md",
        path:it.path||"",
        catalog,
        counts:catalogCounts(catalog),
      };
    });
    if(incoming.length===0)return;
    setCatalogSources(prev=>{
      const incomingKeys=new Set(incoming.map(src=>src.path||src.name));
      const next=[...prev.filter(src=>!incomingKeys.has(src.path||src.name)),...incoming];
      const sourceRooms=catalogRoomNames(next);
      const nextRooms=sourceRooms.length?sourceRooms:catalogRooms;
      if(sourceRooms.length)setCatalogRooms(nextRooms);
      const merged=composeSemanticCatalog(next,nextRooms);
      const counts=catalogCounts(merged);
      setStatus(`✅ MD 목록 갱신: 파일 ${incoming.length} · 방 ${counts.rooms} · 위치 ${counts.locations} · 객체 ${counts.objects}`);
      return next;
    });
    setShowCatalogPanel(true);
    setActiveTab("semantic");
  },[catalogRooms]);

  const removeCatalogSource=useCallback((id)=>{
    setCatalogSources(prev=>{
      const next=prev.filter(src=>src.id!==id);
      const sourceRooms=catalogRoomNames(next);
      const nextRooms=sourceRooms.length?sourceRooms:catalogRooms;
      if(sourceRooms.length)setCatalogRooms(nextRooms);
      const counts=catalogCounts(composeSemanticCatalog(next,nextRooms));
      setStatus(`📋 MD 목록 갱신: 방 ${counts.rooms} · 위치 ${counts.locations} · 객체 ${counts.objects}`);
      return next;
    });
  },[catalogRooms]);

  const resetCatalogSources=useCallback(()=>{
    setCatalogSources([]);
    const defaultRooms=normalizeRoomNames(DEFAULT_SEMANTIC_CATALOG.rooms);
    setCatalogRooms(defaultRooms);
    const counts=catalogCounts(composeSemanticCatalog([],defaultRooms));
    setStatus(`📋 MD 목록 초기화: 방 ${counts.rooms} · 위치 ${counts.locations} · 객체 ${counts.objects}`);
  },[]);

  const addCatalogRoom=useCallback((name)=>{
    const room=String(name||"").trim();
    if(!room)return;
    setCatalogRooms(prev=>{
      if((prev||[]).some(r=>catalogId(r)===catalogId(room))){
        setStatus(`⚠ 이미 있는 Room: ${room}`);
        return prev;
      }
      const next=normalizeRoomNames([...(prev||[]),room]);
      setStatus(`✅ Room 추가: ${room} · 방 ${next.length}`);
      return next;
    });
  },[]);

  const updateCatalogRoom=useCallback((index,name)=>{
    const room=String(name||"").trim();
    if(!room)return false;
    if(index<0||index>=catalogRooms.length)return false;
    if(catalogRooms.some((r,i)=>i!==index&&catalogId(r)===catalogId(room))){
      setStatus(`⚠ 이미 있는 Room: ${room}`);
      return false;
    }
    setCatalogRooms(prev=>prev.map((r,i)=>i===index?room:r));
    setStatus(`✎ Room 수정: ${room}`);
    return true;
  },[catalogRooms]);

  const deleteCatalogRoom=useCallback((index)=>{
    setCatalogRooms(prev=>{
      if(prev.length<=1){
        setStatus("⚠ Room은 최소 1개가 필요합니다");
        return prev;
      }
      if(index<0||index>=prev.length)return prev;
      const removed=prev[index];
      const next=prev.filter((_,i)=>i!==index);
      setStatus(`🗑 Room 삭제: ${removed} · 방 ${next.length}`);
      return next;
    });
  },[]);

  const resetCatalogRooms=useCallback(()=>{
    const next=normalizeRoomNames(DEFAULT_SEMANTIC_CATALOG.rooms);
    setCatalogRooms(next);
    setStatus(`📋 Room 기본값 복구: 방 ${next.length}`);
  },[]);

  const importThorMdSelection=useCallback(async(files,dir)=>{
    const selectedFiles=Array.isArray(files)?files:[];
    if(!selectedFiles.length)return;
    const mdDir=dir||DEFAULT_THOR_MD_DIR;
    setThorMdPicker(null);
    try{
      setStatus(`📋 Thor MD 읽는 중: ${selectedFiles.length}개`);
      const items=[];
      for(const file of selectedFiles){
        const rel=file.relativePath||file.name;
        const read=await hostAPI.readThorMdFile({dir:mdDir,path:rel});
        items.push({
          name:read.name||file.name||String(rel).split("/").pop(),
          path:`thor:${read.path||`${mdDir}/${rel}`}`,
          text:read.text||"",
        });
      }
      importCatalogTexts(items);
    }catch(err){
      setStatus(`⚠ Thor MD 불러오기 실패: ${err.message||String(err)}`);
    }
  },[importCatalogTexts]);

  const browseThorMdDir=useCallback(async(path="")=>{
    return hostAPI.listThorMdDir({dir:DEFAULT_THOR_MD_DIR,path});
  },[]);

  const handleCatalogOpen=useCallback(async()=>{
    if(!hostAPI?.openFileDialog){setStatus("⚠ MD 카탈로그 열기는 Electron 또는 robot backend에서만 가능합니다");return;}
    if(hostAPI?.listThorMdDir&&hostAPI?.readThorMdFile){
      try{
        setStatus(`📋 Thor MD 폴더 읽는 중: ${DEFAULT_THOR_MD_DIR}`);
        const listed=await browseThorMdDir("");
        setThorMdPicker({
          key:Date.now(),
          dir:listed?.dir||DEFAULT_THOR_MD_DIR,
          path:listed?.path||"",
          entries:listed?.entries||[],
        });
        setStatus(`📋 Thor MD 브라우저 열림: ${listed?.entries?.length||0}개 항목`);
        return;
      }catch(err){
        setStatus(`⚠ Thor MD 불러오기 실패: ${err.message||String(err)}`);
        return;
      }
    }
    const selected=await pickHostPath({
      defaultPath:DEFAULT_THOR_MD_DIR,
      filters:[{name:"Markdown catalog",extensions:["md"]},{name:"All files",extensions:["*"]}],
      properties:["openFile","multiSelections"],
    });
    if(!selected)return;
    const paths=Array.isArray(selected)?selected:[selected];
    const items=[];
    for(const p of paths){
      const text=await hostAPI.readFile(p,"utf-8");
      items.push({name:p.split("/").pop(),path:p,text});
    }
    importCatalogTexts(items);
  },[browseThorMdDir,importCatalogTexts,pickHostPath]);

  const handleCatalogFiles=async(e)=>{
    const files=Array.from(e.target.files||[]);
    e.target.value="";
    if(files.length===0)return;
    const items=[];
    for(const file of files)items.push({name:file.name,text:await file.text()});
    importCatalogTexts(items);
  };

  const handleFiles=async(e)=>{
    const files=Array.from(e.target.files||[]);
    e.target.value="";
    if(files.length===0)return;
    const yamlFile=files.find(f=>/\.(yaml|yml)$/i.test(f.name));
    const semanticFile=files.find(f=>/\.json$/i.test(f.name));
    if(yamlFile){
      const parsed=loadYAMLData(await yamlFile.text(),yamlFile.name);
      let loadedSize=null;
      if(parsed.image){
        const imageName=basenameFromPath(parsed.image);
        const pgms=files.filter(f=>/\.pgm$/i.test(f.name));
        const pgmFile=pgms.find(f=>f.name.toLowerCase()===imageName.toLowerCase())||(pgms.length===1?pgms[0]:null);
        if(pgmFile){
          loadedSize=loadPGMData(pgmFile.name, await pgmFile.arrayBuffer());
          setStatus(`✅ YAML + PGM 로드 완료: ${yamlFile.name}`);
        } else {
          setStatus(`✅ YAML 로드 · 브라우저에서는 ${imageName} 파일도 함께 선택하세요`);
        }
      } else {
        setStatus("✅ YAML 로드 (image 항목 없음)");
      }
      if(semanticFile)await loadSemanticJSONData(await semanticFile.text(),semanticFile.name,{meta:parsed,canvasSize:loadedSize});
      return;
    }
    const pgmFile=files.find(f=>/\.pgm$/i.test(f.name));
    if(pgmFile){
      const loadedSize=loadPGMData(pgmFile.name, await pgmFile.arrayBuffer());
      if(semanticFile)await loadSemanticJSONData(await semanticFile.text(),semanticFile.name,{canvasSize:loadedSize});
      return;
    }
    if(semanticFile)await loadSemanticJSONData(await semanticFile.text(),semanticFile.name);
  };

  const handleNativeOpen=async ()=>{
    const filePath = await pickHostPath({
      defaultPath: DEFAULT_MAP_SEARCH_DIR,
      filters: [
        { name: "Map / Semantic files", extensions: ["pgm", "yaml", "yml", "json"] },
        { name: "All files", extensions: ["*"] },
      ],
      properties: ["openFile"],
    });
    if(!filePath) return;
    const name = filePath.split("/").pop();
    const dir = filePath.substring(0, filePath.lastIndexOf("/"));
    if(/\.pgm$/i.test(name)){
      const buf = await hostAPI.readFile(filePath, null);
      const loadedSize=loadPGMData(name, toArrayBufferData(buf));
      // Auto-detect YAML in same directory
      const baseName=name.replace(/\.pgm$/i,"");
      let yamlMeta=null;
      for(const yamlName of [`${baseName}.yaml`,`${baseName}.yml`]){
        try{
          const yamlPath=`${dir}/${yamlName}`;
          const yamlText=await hostAPI.readFile(yamlPath,"utf-8");
          if(yamlText){yamlMeta=loadYAMLData(yamlText,yamlName);syncWorkspaceMapPath(yamlPath);setStatus(`✅ PGM + YAML 로드 완료`);break;}
        }catch(e){/* yaml not found, skip */}
      }
      await tryAutoLoadSemantic(dir,baseName,{meta:yamlMeta,canvasSize:loadedSize});
    } else if(/\.(yaml|yml)$/i.test(name)){
      const text = await hostAPI.readFile(filePath, "utf-8");
      const parsed=loadYAMLData(text,name);
      syncWorkspaceMapPath(filePath);
      let loadedSize=null;
      // Auto-detect PGM from yaml content or same directory
      if(parsed.image){
        const pgmPath=resolveMapPath(dir,parsed.image);
        try{
          const buf=await hostAPI.readFile(pgmPath,null);
          loadedSize=loadPGMData(basenameFromPath(parsed.image),toArrayBufferData(buf));
          setStatus(`✅ YAML + PGM 로드 완료`);
        }catch(e){setStatus("✅ YAML 로드 (PGM 파일을 찾을 수 없음: "+parsed.image+")");}
      }
      const baseName=(parsed.image?basenameFromPath(parsed.image):name).replace(/\.(pgm|yaml|yml)$/i,"");
      await tryAutoLoadSemantic(dir,baseName,{meta:parsed,canvasSize:loadedSize});
    } else if(/\.json$/i.test(name)){
      const text=await hostAPI.readFile(filePath,"utf-8");
      loadSemanticJSONData(text,name);
    }
  };

  const savePGM=async ()=>{
    const c=canvasRef.current;if(!c)return;
    const base=cleanMapBaseName(meta.filename||"map")||"map";
    const defaultName=hasHostAPI?`${DEFAULT_MAP_SEARCH_DIR}/${base}.pgm`:`${base}.pgm`;
    await nativeSave(defaultName, [{ name: "PGM", extensions: ["pgm"] }], canvasToPGM(c), null, pickHostPath);
    setStatus("💾 PGM 저장 완료");
  };

  // px→world coordinate conversion
  const toWorld=useCallback((px,py)=>{
    const w = pixelToWorld(px, py, meta.origin, meta.resolution, canvasSize.h);
    return { x: +w.x.toFixed(3), y: +w.y.toFixed(3) };
  },[meta,canvasSize.h]);

  const robotPoseToCanvas=useCallback(()=>{
    if(!mapLoaded){setStatus("⚠ 맵을 먼저 여세요");return null;}
    const pose=ros2Stats.current?.robotPose;
    if(!pose){setStatus("⚠ 현재 로봇 pose가 없습니다. /tf, /odom, /amcl_pose 등을 표시하거나 bag을 재생하세요");return null;}
    const p=worldToPixel(pose.x,pose.y,meta.origin,meta.resolution,canvasSize.h);
    const out={x:Math.round(p.x),y:Math.round(p.y),theta:pose.theta||0};
    if(!inBounds(out)){setStatus(`⚠ 로봇 pose가 맵 범위 밖입니다: (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)})m`);return null;}
    return out;
  },[mapLoaded,ros2Stats,meta,canvasSize.h]);

  const captureRobotPose=useCallback((kind)=>{
    const pose=robotPoseToCanvas();
    if(!pose)return;
    if(kind==="start"){
      const task=nextStartTask(startPoses,selectedStartTask);
      const newId=startPoses[task]?.id||nextStartPoseId(startPoses);
      setStartDlg({pose:{id:newId,task,label:task,...pose},defaultTask:task,defaultId:newId});
      setStatus("⌂ 시작점 task/ID 선택");
      return;
    }else if(kind==="waypoint"){
      const newId=wuid();
      const wp={id:newId,label:newId,...pose};
      setWaypoints(p=>[...p,wp]);
      setSelWpIdx(waypointsRef.current.length);setSelSemId(null);
      setStatus(`◎ 현재 로봇 pose를 웨이포인트로 추가: ${newId}`);
    }else if(kind==="goal"){
      const hitRoom=[...roomsRef.current].reverse().find(r=>hitTestShape(r,pose.x,pose.y));
      const newId=guid();
      const goal={id:newId,label:newId,...pose,room_id:hitRoom?.id||null,target_id:null};
      setGoals(p=>[...p,goal]);
      setSelSemId(newId);setSelWpIdx(null);
      setStatus(`🎯 현재 로봇 pose를 시맨틱 골로 추가${hitRoom?` (${hitRoom.label})`:""}`);
    }
    setTimeout(saveSnap,0);
  },[robotPoseToCanvas,saveSnap,selectedStartTask,startPoses]);

  const startRosbridge=useCallback(async()=>{
    if(!hostAPI?.rosbridgeStart){setStatus("⚠ rosbridge 실행은 Electron 또는 robot backend에서만 가능합니다");return;}
    setRosbridgeBusy(true);
    try{
      const res=await hostAPI.rosbridgeStart({port:9090});
      setRosbridgeRunning(!!res?.running);
      const bridgeUrl=defaultRosbridgeUrl(9090);
      setStatus(`🌉 rosbridge 실행: ${bridgeUrl}`);
      setTimeout(()=>{
        ros2Bridge._autoReconnect=true;
        ros2Bridge.connect(bridgeUrl);
      },700);
    }catch(e){
      setStatus(`⚠ rosbridge 실행 실패: ${e.message}`);
    }finally{
      setRosbridgeBusy(false);
    }
  },[ros2Bridge]);

  const stopRosbridge=useCallback(async()=>{
    if(!hostAPI?.rosbridgeStop)return;
    setRosbridgeBusy(true);
    try{
      await hostAPI.rosbridgeStop();
      setRosbridgeRunning(false);
      ros2Bridge.disconnect();
      setStatus("🌉 rosbridge 중지");
    }catch(e){
      setStatus(`⚠ rosbridge 중지 실패: ${e.message}`);
    }finally{
      setRosbridgeBusy(false);
    }
  },[ros2Bridge]);

  const enterSlamMode=useCallback(()=>{
    setSlamMode(true);
    setShowSlamPanel(true);
    setShowSemPanel(true);
    setActiveTab("semantic");
    setTool("semGoal");
    setSlamSaveName(timestampedMapName());
    setStatus(`SLAM 모드 시작: ${slamMapTopic.trim()||"/map"} live map 위에 시맨틱 골 작성`);
  },[slamMapTopic]);

  const exitSlamMode=useCallback(()=>{
    setSlamMode(false);
    setStatus("SLAM 모드 종료");
  },[]);

  const startSlamToolbox=useCallback(async()=>{
    if(!hostAPI?.slamStart){setStatus("⚠ slam_toolbox 실행은 Electron 또는 robot backend에서만 가능합니다");return;}
    setSlamBusy(true);
    try{
      const res=await hostAPI.slamStart({
        mode:"online_async",
        useSimTime:slamUseSimTime,
        paramsFile:slamParamsFile.trim()||"",
      });
      setSlamRunning(!!res?.running);
      enterSlamMode();
      setStatus(`▶ slam_toolbox 실행${slamUseSimTime?" (sim time)":""}`);
    }catch(e){
      setStatus(`⚠ slam_toolbox 실행 실패: ${e.message}`);
    }finally{
      setSlamBusy(false);
    }
  },[slamUseSimTime,slamParamsFile,enterSlamMode]);

  const stopSlamToolbox=useCallback(async()=>{
    if(!hostAPI?.slamStop)return;
    setSlamBusy(true);
    try{
      await hostAPI.slamStop();
      setSlamRunning(false);
      setStatus("■ slam_toolbox 중지");
    }catch(e){
      setStatus(`⚠ slam_toolbox 중지 실패: ${e.message}`);
    }finally{
      setSlamBusy(false);
    }
  },[]);

  const startNav2=useCallback(async()=>{
    if(!hostAPI?.nav2Start){setStatus("⚠ Nav2 실행은 Electron 또는 robot backend에서만 가능합니다");return;}
    setNav2Busy(true);
    try{
      const res=await hostAPI.nav2Start({
        useSimTime:slamUseSimTime,
        autostart:true,
        paramsFile:nav2ParamsFile.trim()||"",
      });
      setNav2Running(!!res?.running);
      enterSlamMode();
      setStatus(`▶ Nav2 navigation_launch.py 실행${nav2ParamsFile.trim()?` (${basenameFromPath(nav2ParamsFile.trim())})`:""}`);
    }catch(e){
      setStatus(`⚠ Nav2 실행 실패: ${e.message}`);
    }finally{
      setNav2Busy(false);
    }
  },[slamUseSimTime,nav2ParamsFile,enterSlamMode]);

  const stopNav2=useCallback(async()=>{
    if(!hostAPI?.nav2Stop)return;
    setNav2Busy(true);
    try{
      await hostAPI.nav2Stop();
      setNav2Running(false);
      setStatus("■ Nav2 중지");
    }catch(e){
      setStatus(`⚠ Nav2 중지 실패: ${e.message}`);
    }finally{
      setNav2Busy(false);
    }
  },[]);

  const chooseBagPath=useCallback(async()=>{
    if(!hostAPI?.openFileDialog){setStatus("⚠ bag 실행은 Electron 또는 robot backend에서만 가능합니다");return;}
    const selected=await pickHostPath({
      title:"ROS2 bag 폴더 선택",
      defaultPath:bagPath||DEFAULT_BAG_RECORD_DIR,
      properties:["openDirectory"],
    });
    if(selected){
      setBagPath(selected);setBagOffset(0);
      try{
        const info=await hostAPI.rosbagInfo?.(selected);
        setBagDuration(Number.isFinite(info?.duration)?info.duration:0);
      }catch(e){setBagDuration(0);}
    }
  },[bagPath,pickHostPath]);

  const chooseBagRecordDir=useCallback(async()=>{
    if(!hostAPI?.openFileDialog){setStatus("⚠ bag 저장 폴더 선택은 Electron 또는 robot backend에서만 가능합니다");return;}
    const selected=await pickHostPath({
      title:"ROS2 bag 저장 폴더 선택",
      defaultPath:bagRecordDir||DEFAULT_BAG_RECORD_DIR,
      properties:["openDirectory"],
    });
    if(selected){
      const dir=Array.isArray(selected)?selected[0]:selected;
      setBagRecordDir(dir);
      setStatus(`✅ bag 저장 폴더: ${dir}`);
    }
  },[bagRecordDir,pickHostPath]);

  const playBag=useCallback(async()=>{
    if(!hostAPI?.rosbagPlay){setStatus("⚠ bag 실행은 Electron 또는 robot backend에서만 가능합니다");return;}
    if(!bagPath){setStatus("⚠ 재생할 bag 폴더를 먼저 선택하세요");return;}
    setBagBusy(true);
    try{
      const startOffset=bagDuration>0&&bagOffset>=Math.max(0,bagDuration-0.25)?0:bagOffset;
      const res=await hostAPI.rosbagPlay({path:bagPath,clock:true,loop:bagLoop,rate:bagRate,startOffset});
      setBagRunning(!!res?.running);
      setBagPaused(false);
      if(Number.isFinite(res?.offset))setBagOffset(res.offset);
      if(Number.isFinite(res?.duration)&&res.duration>0)setBagDuration(res.duration);
      setStatus(`▶ bag 재생: ${basenameFromPath(bagPath)} @ ${Math.round(startOffset)}s`);
    }catch(e){
      setStatus(`⚠ bag 재생 실패: ${e.message}`);
    }finally{
      setBagBusy(false);
    }
  },[bagPath,bagLoop,bagRate,bagOffset,bagDuration]);

  const stopBag=useCallback(async()=>{
    if(!hostAPI?.rosbagStop)return;
    setBagBusy(true);
    try{
      await hostAPI.rosbagStop();
      setBagRunning(false);
      setBagPaused(false);
      setStatus("■ bag 재생 중지");
    }catch(e){
      setStatus(`⚠ bag 중지 실패: ${e.message}`);
    }finally{
      setBagBusy(false);
    }
  },[]);

  const startBagRecord=useCallback(async()=>{
    if(!hostAPI?.rosbagRecordStart){setStatus("⚠ bag 저장은 Electron 또는 robot backend에서만 가능합니다");return;}
    const outputDir=(bagRecordDir||DEFAULT_BAG_RECORD_DIR).trim();
    const name=bagRecordName.trim();
    setBagRecordBusy(true);
    try{
      const res=await hostAPI.rosbagRecordStart({
        outputDir,
        ...(name?{name}:{}),
        topics:DEFAULT_BAG_RECORD_TOPICS,
      });
      setBagRecording(!!res?.running);
      const outputPath=res?.options?.outputPath||"";
      if(outputPath)setBagRecordPath(outputPath);
      setStatus(`⏺ bag 저장 시작: ${outputPath||outputDir}`);
    }catch(e){
      setStatus(`⚠ bag 저장 시작 실패: ${e.message}`);
    }finally{
      setBagRecordBusy(false);
    }
  },[bagRecordDir,bagRecordName]);

  const stopBagRecord=useCallback(async()=>{
    if(!hostAPI?.rosbagRecordStop)return;
    setBagRecordBusy(true);
    try{
      const res=await hostAPI.rosbagRecordStop();
      setBagRecording(false);
      const outputPath=res?.options?.outputPath||bagRecordPath;
      setStatus(`■ bag 저장 중지${outputPath?`: ${basenameFromPath(outputPath)}`:""}`);
    }catch(e){
      setStatus(`⚠ bag 저장 중지 실패: ${e.message}`);
    }finally{
      setBagRecordBusy(false);
    }
  },[bagRecordPath]);

  const pauseBag=useCallback(async()=>{
    if(!hostAPI?.rosbagPause)return;
    setBagBusy(true);
    try{
      const res=await hostAPI.rosbagPause();
      setBagPaused(!!res?.paused);
      if(Number.isFinite(res?.offset))setBagOffset(res.offset);
      setStatus(`⏸ bag 일시정지 @ ${Math.round(res?.offset||bagOffset)}s`);
    }catch(e){setStatus(`⚠ bag 일시정지 실패: ${e.message}`);}
    finally{setBagBusy(false);}
  },[bagOffset]);

  const resumeBag=useCallback(async()=>{
    if(!hostAPI?.rosbagResume)return;
    setBagBusy(true);
    try{
      const res=await hostAPI.rosbagResume();
      setBagPaused(!!res?.paused);
      if(Number.isFinite(res?.offset))setBagOffset(res.offset);
      setStatus(`▶ bag 재개 @ ${Math.round(res?.offset||bagOffset)}s`);
    }catch(e){setStatus(`⚠ bag 재개 실패: ${e.message}`);}
    finally{setBagBusy(false);}
  },[bagOffset]);

  const seekBagTo=useCallback(async(nextOffset)=>{
    const max=bagDuration>0?bagDuration:Infinity;
    const next=Math.max(0,Math.min(max,Number(nextOffset)||0));
    setBagOffset(next);
    if(!bagRunning)return;
    if(!hostAPI?.rosbagSeek)return;
    setBagBusy(true);
    try{
      const res=await hostAPI.rosbagSeek({offset:next,loop:bagLoop,rate:bagRate,clock:true});
      setBagRunning(!!res?.running);
      setBagPaused(false);
      if(Number.isFinite(res?.offset))setBagOffset(res.offset);
      if(Number.isFinite(res?.duration)&&res.duration>0)setBagDuration(res.duration);
      setStatus(`↔ bag 이동 @ ${Math.round(next)}s`);
    }catch(e){setStatus(`⚠ bag 이동 실패: ${e.message}`);}
    finally{setBagBusy(false);}
  },[bagDuration,bagRunning,bagLoop,bagRate]);

  const seekBag=useCallback((delta)=>seekBagTo(bagOffset+delta),[bagOffset,seekBagTo]);

  const buildGoalListJSON=useCallback(()=>{
    return JSON.stringify({
      metadata:{map:meta.filename,created:new Date().toISOString()},
      goal_list:goalList.map((item,i)=>({
        id:item.id||`gl${i+1}`,
        label:item.label||item.goal_id||`goal_list_${i+1}`,
        goal_id:item.goal_id||"",
      })),
    },null,2);
  },[goalList,meta.filename]);

  const saveGoalListJSON=useCallback(async()=>{
    const base=cleanMapBaseName(meta.filename||"map")||"map";
    const defaultName=hasHostAPI?`${DEFAULT_MAP_SEARCH_DIR}/${base}_goal_list.json`:`${base}_goal_list.json`;
    const saved=await nativeSave(defaultName,[{name:"Goal list JSON",extensions:["json"]}],buildGoalListJSON(),"utf-8",pickHostPath);
    if(saved)setStatus(`💾 골 리스트 저장 완료: ${basenameFromPath(saved)}`);
    return saved;
  },[buildGoalListJSON,meta.filename,pickHostPath]);

  const buildSemanticJSON=useCallback(()=>{
    const tw=toWorld;
    const polyWorld=(poly)=>poly.map(p=>tw(p.x,p.y));
    const bboxWorld=(bb)=>({min:tw(bb.x,bb.y2),max:tw(bb.x2,bb.y)});
    const poseJson=(pose)=>{
      const pos=tw(pose.x,pose.y);
      const qz=Math.sin(pose.theta/2),qw=Math.cos(pose.theta/2);
      return{id:pose.id,label:pose.label||pose.task||"start",
        position:{x:+pos.x,y:+pos.y,z:0.0},
        orientation:{x:0,y:0,z:+qz.toFixed(5),w:+qw.toFixed(5)},
        theta_rad:+pose.theta.toFixed(4),
        _pixel:{x:pose.x,y:pose.y}};
    };
    const startPoseItems=START_TASKS
      .filter(task=>startPoses[task])
      .map(task=>({...poseJson(startPoses[task]),task,label:task}));
    return JSON.stringify({
      metadata:{
        resolution:meta.resolution,
        origin:meta.origin,
        image_size:{w:canvasSize.w,h:canvasSize.h},
        semantic_catalog:{rooms:catalogRooms},
        created:new Date().toISOString()
      },
      start_poses:startPoseItems,
      maps:maps.map(m=>{
        const poly=shapeToPoly(m)||[];
        const bb=poly.length?polyBBox(poly):{x:m.x,y:m.y,x2:m.x+(m.w||0),y2:m.y+(m.h||0)};
        return{id:m.id,type:m.type,label:m.label,
          polygon:polyWorld(poly),bbox:bboxWorld(bb),
          _pixel:{polygon:poly}};
      }),
      rooms:rooms.map(r=>{
        const poly=shapeToPoly(r)||[];
        const bb=poly.length?polyBBox(poly):{x:r.x,y:r.y,x2:r.x+(r.w||0),y2:r.y+(r.h||0)};
        return{id:r.id,type:r.type,label:r.label,map_id:r.mapId||null,
          polygon:polyWorld(poly),bbox:bboxWorld(bb),
          _pixel:{polygon:poly}};
      }),
      carriers:carriers.map(c=>{
        const poly=shapeToPoly(c)||[];
        const bb=poly.length?polyBBox(poly):{x:c.x,y:c.y,x2:c.x+(c.w||0),y2:c.y+(c.h||0)};
        const z=+(Number(c.z)||0).toFixed(3);
        return{id:c.id,type:c.type,label:c.label,room_id:c.roomId||null,
          z,
          polygon:polyWorld(poly),bbox:bboxWorld(bb),
          _pixel:{polygon:poly}};
      }),
      objects:objects.map(o=>{
        if(o.point)return{id:o.id,type:o.type,label:o.label,carrier_id:o.carrierId||null,room_id:o.roomId||null,
          position:tw(o.x,o.y),
          _pixel:{x:o.x,y:o.y}};
        const poly=shapeToPoly(o)||[];
        return{id:o.id,type:o.type,label:o.label,carrier_id:o.carrierId||null,room_id:o.roomId||null,
          polygon:polyWorld(poly),
          _pixel:{polygon:poly}};
      }),
      empty_seats:emptySeats.map(seat=>{
        const poly=shapeToPoly(seat)||[];
        const bb=poly.length?polyBBox(poly):{x:seat.x,y:seat.y,x2:seat.x+(seat.w||0),y2:seat.y+(seat.h||0)};
        return{id:seat.id,type:seat.type||EMPTY_SEAT_TYPE.id,label:seat.label||seat.id,room_id:seat.roomId||null,
          polygon:polyWorld(poly),bbox:bboxWorld(bb),
          _pixel:{polygon:poly}};
      }),
      waypoints:waypoints.map((wp,i)=>{
        const pos=tw(wp.x,wp.y);
        const qz=Math.sin(wp.theta/2),qw=Math.cos(wp.theta/2);
        return{id:wp.id||`w${i+1}`,label:wp.label,
          position:{x:+pos.x,y:+pos.y,z:0.0},
          orientation:{x:0,y:0,z:+qz.toFixed(5),w:+qw.toFixed(5)},
          theta_rad:+wp.theta.toFixed(4),
          _pixel:{x:wp.x,y:wp.y}};
      }),
      goals:goals.map(g=>{
        const pos=tw(g.x,g.y);
        const qz=Math.sin(g.theta/2),qw=Math.cos(g.theta/2);
        return{id:g.id,label:g.label,room_id:g.room_id,target_id:g.target_id,
          position:{x:+pos.x,y:+pos.y,z:0.0},
          orientation:{x:0,y:0,z:+qz.toFixed(5),w:+qw.toFixed(5)},
          theta_rad:+g.theta.toFixed(4),
          _pixel:{x:g.x,y:g.y}};
      })
    },null,2);
  },[maps,rooms,carriers,objects,emptySeats,startPoses,waypoints,goals,meta,canvasSize,toWorld,catalogRooms]);

  const saveMapBundle=useCallback(async(defaultBase=meta.filename)=>{
    const c=canvasRef.current;if(!c)return null;
    const fallbackBase=cleanMapBaseName(defaultBase||meta.filename||"map")||"map";
    if(hasHostAPI){
      const dirPath = await pickHostPath({
        dialogType:"save",
        defaultPath: `${DEFAULT_MAP_SEARCH_DIR}/${fallbackBase}.pgm`,
        filters: [{ name: "PGM", extensions: ["pgm"] }],
      });
      if(!dirPath) return null;
      const slashIdx = dirPath.lastIndexOf("/");
      const dir = slashIdx >= 0 ? dirPath.substring(0, slashIdx) : ".";
      const baseName = cleanMapBaseName(dirPath)||fallbackBase;

      await hostAPI.writeFile(dirPath.endsWith(".pgm")?dirPath:`${dir}/${baseName}.pgm`, canvasToPGM(c), null);
      await hostAPI.writeFile(`${dir}/${baseName}.yaml`, writeYAML(meta,baseName), "utf-8");
      await hostAPI.writeFile(`${dir}/${baseName}_semantic.json`, buildSemanticJSON(), "utf-8");
      const mapPath = `${dir}/${baseName}.yaml`;
      syncWorkspaceMapPath(mapPath);
      setStatus(`💾 저장 완료: ${baseName}.pgm · .yaml · _semantic.json`);
      return { dir, baseName, mapPath, semanticPath: `${dir}/${baseName}_semantic.json` };
    } else {
      await nativeSave(`${fallbackBase}.pgm`, [{ name: "PGM", extensions: ["pgm"] }], canvasToPGM(c), null, pickHostPath);
      const yblob=new Blob([writeYAML(meta,fallbackBase)],{type:"text/plain"});
      const ya=document.createElement("a");ya.href=URL.createObjectURL(yblob);ya.download=`${fallbackBase}.yaml`;ya.click();
      const jblob=new Blob([buildSemanticJSON()],{type:"application/json"});
      const ja=document.createElement("a");ja.href=URL.createObjectURL(jblob);ja.download=`${fallbackBase}_semantic.json`;ja.click();
      setStatus("💾 전체 저장 완료 (PGM + YAML + JSON)");
      return {baseName:fallbackBase};
    }
  },[buildSemanticJSON,meta,pickHostPath,syncWorkspaceMapPath]);

  const saveAll=async ()=>{
    const saved=await saveMapBundle(meta.filename);
    if(saved)setStatus(`💾 저장 완료: ${saved.baseName}.pgm · .yaml · _semantic.json`);
  };

  const quickSaveNavMap=useCallback(async(slot)=>{
    const c=canvasRef.current;
    if(!c||!mapLoaded){setStatus("⚠ 저장할 맵이 없습니다");return;}
    if(!hasHostAPI){setStatus("⚠ 퀵 저장은 Electron 또는 robot backend에서만 가능합니다");return;}
    const baseName=QUICK_MAP_NAMES[slot]||`map_${slot}`;
    const yamlPath=`${QUICK_MAP_DIR}/${baseName}.yaml`;
    const pgmPath=`${QUICK_MAP_DIR}/${baseName}.pgm`;
    const semanticPath=`${QUICK_MAP_DIR}/${baseName}_semantic.json`;
    setQuickSaveBusy(slot);
    try{
      await hostAPI.writeFile(pgmPath,canvasToPGM(c),null);
      await hostAPI.writeFile(yamlPath,writeYAML(meta,baseName),"utf-8");
      await hostAPI.writeFile(semanticPath,buildSemanticJSON(),"utf-8");
      syncWorkspaceMapPath(yamlPath);

      let thorStatus=" · Thor 복사 생략";
      if(hostAPI.copySemanticToThor){
        try{
          await hostAPI.copySemanticToThor({sourcePath:semanticPath,fileName:`${baseName}_semantic.json`});
          thorStatus=" · Thor semantic JSON 복사 완료";
        }catch(e){
          thorStatus=` · Thor 복사 실패: ${e.message}`;
        }
      }
      setStatus(`💾 ${slot}번맵 저장 완료: ${baseName}.yaml${thorStatus}`);
    }catch(e){
      setStatus(`⚠ ${slot}번맵 저장 실패: ${e.message}`);
    }finally{
      setQuickSaveBusy(null);
    }
  },[mapLoaded,meta,buildSemanticJSON,syncWorkspaceMapPath]);

  const saveSlamResult=useCallback(async()=>{
    if(!slamMapStats?.width){setStatus("⚠ 저장할 SLAM live map이 없습니다");return;}
    setSlamSaveBusy(true);
    try{
      const requestedBase=cleanMapBaseName(slamSaveName)||timestampedMapName();
      const saved=await saveMapBundle(requestedBase);
      if(!saved)return;
      setSlamSaveName(saved.baseName||requestedBase);
      const stopped=[];
      if(nav2Running&&hostAPI?.nav2Stop){
        await hostAPI.nav2Stop();
        setNav2Running(false);
        stopped.push("Nav2");
      }
      if(slamRunning&&hostAPI?.slamStop){
        await hostAPI.slamStop();
        setSlamRunning(false);
        stopped.push("slam_toolbox");
      }
      setSlamMode(false);
      setStatus(`💾 SLAM 결과 저장 완료${stopped.length?` · ${stopped.join(" / ")} 중지`:""}: ${saved.baseName}.pgm · .yaml · _semantic.json`);
    }catch(e){
      setStatus(`⚠ SLAM 결과 저장 실패: ${e.message}`);
    }finally{
      setSlamSaveBusy(false);
    }
  },[saveMapBundle,slamMapStats,slamRunning,slamSaveName,nav2Running]);

  const chooseMapYamlPath=useCallback(async()=>{
    const selected=await pickWorkspacePath({
      title:"맵 YAML 선택",
      defaultPath: DEFAULT_MAP_SEARCH_DIR,
      filters:[{name:"YAML",extensions:["yaml","yml"]},{name:"All files",extensions:["*"]}],
      properties:["openFile"],
    });
    if(!selected)return;
    const mapPath=Array.isArray(selected)?selected[0]:selected;
    setWorkspaceSync(prev=>({...prev,mapPath}));
    setStatus(`✅ 맵 YAML 선택: ${basenameFromPath(mapPath)}`);
  },[pickWorkspacePath]);

  const chooseWorkspaceRoot=useCallback(async()=>{
    const selected=await pickWorkspacePath({
      title:"ROS2 워크스페이스 선택",
      defaultPath: DEFAULT_WORKSPACE_ROOT,
      properties:["openDirectory"],
    });
    if(!selected)return;
    const root=Array.isArray(selected)?selected[0]:selected;
    setWorkspaceSync(prev=>({...prev,workspaceRoot:root}));
    setStatus(`✅ 워크스페이스 선택: ${root}`);
  },[pickWorkspacePath]);

  const buildWorkspaceOnly=useCallback(async()=>{
    if(!hostAPI?.buildWorkspace){
      setStatus("⚠ 빌드는 Electron 또는 robot backend에서만 가능합니다");
      return;
    }
    const workspaceRoot=(workspaceSync.workspaceRoot||DEFAULT_WORKSPACE_ROOT).trim();
    const packageName=(workspaceSync.packageName||"rby1_nav2").trim();
    if(!workspaceRoot){setStatus("⚠ ROS2 워크스페이스 루트를 선택하세요");return;}
    setWorkspaceBusy(true);
    try{
      setStatus(`🏗 빌드 중: ${packageName||"workspace"}`);
      const buildResult=await hostAPI.buildWorkspace({cwd:workspaceRoot,packageName:packageName||""});
      setStatus(`✅ 빌드 완료: ${packageName||basenameFromPath(workspaceRoot||"workspace")}`);
      return buildResult;
    }catch(err){
      setStatus(`⚠ 빌드 실패: ${err.message}`);
      return null;
    }finally{
      setWorkspaceBusy(false);
    }
  },[hostAPI,workspaceSync]);

  const fitView=()=>{
    const c=canvasRef.current,vp=vpRef.current;if(!c||!vp)return;
    const z=Math.min((vp.clientWidth-60)/c.width,(vp.clientHeight-60)/c.height,3);
    setZoom(z);setPan({x:(vp.clientWidth-c.width*z)/2,y:(vp.clientHeight-c.height*z)/2});setRotation(0);
  };

  const isPolyActive=(tool==="semPolyMap"||tool==="semPolyRoom"||tool==="semPolyCarrier"||tool==="semPolyObj")&&polyVerts.length>0;
  const cursorWorld=toWorld(cursor.x,cursor.y);
  const realX=cursorWorld.x;
  const realY=cursorWorld.y;

  const toolTips={
    semRectRoom:"드래그로 사각형 방 영역",semPolyRoom:"클릭으로 꼭짓점 추가 · 더블클릭/Enter로 완성 · Backspace로 되돌리기",
    semRectCarrier:"드래그로 사각형 캐리어",semPolyCarrier:"클릭으로 꼭짓점 추가 · 더블클릭/Enter로 완성",
    semRectObj:"드래그로 사각형 객체",semPolyObj:"클릭으로 꼭짓점 추가 · 더블클릭/Enter로 완성",
    semRectMap:"드래그로 사각형 맵 영역",semPolyMap:"클릭으로 꼭짓점 추가 · 더블클릭/Enter로 완성",
    startPose:"클릭+드래그로 Nav2 시작 위치와 방향 지정",
    waypoint:"클릭+드래그로 웨이포인트 위치와 방향 지정",
    semPoint:"클릭으로 포인트 객체 배치",semGoal:"클릭+드래그로 방향 지정 · 방/대상은 선택 또는 자동 배정",semSelect:"클릭 선택 · 드래그 이동 · Backspace/Del 삭제",
  };

  const onOpenClick = hasHostAPI ? handleNativeOpen : undefined;
  const workspaceAvailable = hasHostAPI;
  const effectiveWorkspaceRoot = (workspaceSync.workspaceRoot || DEFAULT_WORKSPACE_ROOT).trim();
  const canSyncWorkspace = workspaceAvailable && !!effectiveWorkspaceRoot && !workspaceBusy;
  const startPoseCount=Object.values(startPoses||{}).filter(Boolean).length;
  const semanticItemCount=maps.length+rooms.length+carriers.length+objects.length+emptySeats.length+goals.length+goalList.length+waypoints.length+startPoseCount;

  return(
    <div style={{display:"flex",flexDirection:"column",height:"100vh",background:"#050d1a",color:"#8eb8c8",fontFamily:"'JetBrains Mono','Fira Code',monospace",fontSize:12,overflow:"hidden"}}>

      {/* ── TOP BAR ── */}
      <div style={{display:"flex",flexDirection:"column",gap:4,padding:"5px 12px",background:"linear-gradient(90deg,#060e1c,#091525)",borderBottom:"1px solid rgba(0,212,255,0.13)",flexShrink:0}}>
        <div style={{display:"flex",alignItems:"center",gap:5,flexWrap:"wrap"}}>
          <span style={{color:"#00d4ff",fontSize:14}}>◈</span>
          <span style={{color:"#c9fffe",fontWeight:"bold",letterSpacing:1,marginRight:6}}>NAV2 MAP EDITOR</span>
          <div style={{width:1,height:16,background:"rgba(0,212,255,0.15)"}}/>
          {hasHostAPI ? (
            <button style={btn()} onClick={onOpenClick}>📂 열기</button>
          ) : (
            <label style={{...btn(),cursor:"pointer"}}>📂 열기<input type="file" accept=".pgm,.yaml,.yml,.json" multiple onChange={handleFiles} style={{display:"none"}}/></label>
          )}
          <button style={btn()} onClick={()=>setShowNewDlg(true)}>✦ 새 맵</button>
          <div style={{width:1,height:16,background:"rgba(0,212,255,0.15)"}}/>
          <button style={{...btn(),opacity:mapLoaded?1:.4}} onClick={saveAll} disabled={!mapLoaded}>💾 전체저장</button>
          <button
            style={{...btn(),opacity:mapLoaded&&hasHostAPI&&!quickSaveBusy?1:.4}}
            onClick={()=>quickSaveNavMap(1)}
            disabled={!mapLoaded||!hasHostAPI||!!quickSaveBusy}
            title={`${QUICK_MAP_DIR}/map_first.yaml 덮어쓰기 + map_first_semantic.json Thor 복사`}
          >{quickSaveBusy===1?"1번 저장 중":"1번맵 저장"}</button>
          <button
            style={{...btn(),opacity:mapLoaded&&hasHostAPI&&!quickSaveBusy?1:.4}}
            onClick={()=>quickSaveNavMap(2)}
            disabled={!mapLoaded||!hasHostAPI||!!quickSaveBusy}
            title={`${QUICK_MAP_DIR}/map_second.yaml 덮어쓰기 + map_second_semantic.json Thor 복사`}
          >{quickSaveBusy===2?"2번 저장 중":"2번맵 저장"}</button>
          <button style={{...btn(),opacity:mapLoaded?1:.4}} onClick={savePGM} disabled={!mapLoaded}>⬇ PGM</button>
          <button style={{...btn(showWorkspaceDlg),opacity:workspaceAvailable?1:.4}} onClick={()=>setShowWorkspaceDlg(v=>!v)} disabled={!workspaceAvailable}>🏗 빌드</button>
          <div style={{width:1,height:16,background:"rgba(0,212,255,0.15)"}}/>
          <button style={btn()} onClick={undo} title="Ctrl+Z">↩</button>
          <button style={btn()} onClick={redo} title="Ctrl+Y">↪</button>
          <button style={btn()} onClick={fitView}>⊞</button>
          <div style={{width:1,height:16,background:"rgba(0,212,255,0.15)"}}/>
          <button style={btn()} onClick={()=>setShowMetaDlg(true)}>⚙ 메타</button>
          <div style={{flex:"1 1 180px"}}/>
          <div style={{display:"flex",alignItems:"center",gap:5,marginLeft:"auto",flexWrap:"wrap",justifyContent:"flex-end"}}>
            <button style={btn(showSemPanel)} onClick={()=>setShowSemPanel(v=>!v)}>🗺 시맨틱{semanticItemCount>0&&` (${semanticItemCount})`}</button>
            <button style={btn(showCatalogPanel)} onClick={()=>setShowCatalogPanel(v=>!v)}>📋 MD목록 ({catalogCounts(semanticCatalog).rooms}/{catalogCounts(semanticCatalog).locations}/{catalogCounts(semanticCatalog).objects})</button>
            <button style={btn(showRos2Panel)} onClick={()=>setShowRos2Panel(v=>!v)}>🤖 ROS2{ros2State===ROS2_STATES.CONNECTED&&<span style={{marginLeft:4,width:6,height:6,borderRadius:"50%",background:"#00e676",display:"inline-block",boxShadow:"0 0 4px #00e676"}}/>}</button>
            <button style={btn(showSlamPanel||slamMode)} onClick={()=>setShowSlamPanel(v=>!v)}>🧭 SLAM{slamMode&&<span style={{marginLeft:4,width:6,height:6,borderRadius:"50%",background:"#00e676",display:"inline-block",boxShadow:"0 0 4px #00e676"}}/>}</button>
            <button style={btn(show3DView)} onClick={()=>setShow3DView(v=>!v)}>🧊 3D</button>
            {show3DView&&(
              <div style={{display:"flex",gap:4,alignItems:"center",marginLeft:2}}>
                {[["free","Free"],["top","Top"]].map(([id,l])=>(
                  <button key={id} style={{...btn(view3DMode===id),padding:"2px 7px",fontSize:10}} onClick={()=>setView3DMode(id)}>{l}</button>
                ))}
              </div>
            )}
            <span style={{color:"rgba(0,212,255,0.3)",fontSize:10}}>줌 {Math.round(zoom*100)}%{rotation%360!==0&&` · ${rotation%360}°`}</span>
          </div>
        </div>

        <div style={{display:"flex",alignItems:"center",gap:5,flexWrap:"wrap",paddingTop:2,borderTop:"1px solid rgba(0,212,255,0.06)"}}>
          <span style={{fontSize:10,color:"rgba(0,212,255,0.38)",letterSpacing:1,marginRight:2}}>ROS2</span>
          {hasHostAPI&&(
            rosbridgeRunning?(
              <button style={btn(false,true)} onClick={stopRosbridge} disabled={rosbridgeBusy}>■ Bridge</button>
            ):(
              <button style={{...btn(),opacity:rosbridgeBusy ? .45 : 1}} onClick={startRosbridge} disabled={rosbridgeBusy}>🌉 Bridge</button>
            )
          )}
          {hasHostAPI&&(
            <>
              <button style={btn(!!bagPath)} onClick={chooseBagPath} title={bagPath||"ROS2 bag 폴더 선택"}>🎞 {bagPath?basenameFromPath(bagPath):"Bag"}</button>
              <label style={{display:"inline-flex",alignItems:"center",gap:3,color:"#6a9aaa",fontSize:10}}>
                <input type="checkbox" checked={bagLoop} onChange={e=>setBagLoop(e.target.checked)} style={{width:12,height:12,accentColor:"#00d4ff"}}/>
                loop
              </label>
              <input type="number" min={0.1} max={10} step={0.1} value={bagRate} onChange={e=>setBagRate(Math.max(0.1,parseFloat(e.target.value)||1))}
                style={{...INPUT,width:45,padding:"3px 5px",fontSize:10}} title="bag playback rate"/>
              <input type="number" min={1} max={600} step={1} value={bagSeekStep} onChange={e=>setBagSeekStep(Math.max(1,parseFloat(e.target.value)||10))}
                style={{...INPUT,width:42,padding:"3px 5px",fontSize:10}} title="seek step seconds"/>
              <button style={{...btn(),opacity:bagPath&&!bagBusy?1:.45}} onClick={()=>seekBag(-bagSeekStep)} disabled={!bagPath||bagBusy}>⏪ {bagSeekStep}s</button>
              <input type="number" min={0} step={1} value={Math.round(bagOffset)} onChange={e=>setBagOffset(Math.max(0,parseFloat(e.target.value)||0))}
                onKeyDown={e=>{if(e.key==="Enter")seekBagTo(bagOffset);}}
                style={{...INPUT,width:58,padding:"3px 5px",fontSize:10}} title="bag offset seconds"/>
              <button style={{...btn(),opacity:bagPath&&!bagBusy?1:.45}} onClick={()=>seekBag(bagSeekStep)} disabled={!bagPath||bagBusy}>⏩ {bagSeekStep}s</button>
              <input type="range" min={0} max={Math.max(1,Math.ceil(bagDuration||Math.max(60,bagOffset+60)))} step={0.1}
                value={Math.min(bagOffset,Math.max(1,bagDuration||Math.max(60,bagOffset+60)))}
                onChange={e=>setBagOffset(parseFloat(e.target.value)||0)}
                onMouseUp={e=>seekBagTo(parseFloat(e.currentTarget.value)||0)}
                onTouchEnd={e=>seekBagTo(parseFloat(e.currentTarget.value)||0)}
                disabled={!bagPath||bagBusy}
                style={{width:150,accentColor:"#00d4ff",opacity:bagPath&&!bagBusy?1:.4}} title="bag timeline"/>
              <span style={{fontSize:10,color:"rgba(0,212,255,0.4)",minWidth:44}}>{Math.round(bagOffset)}s{bagDuration>0?`/${Math.round(bagDuration)}s`:""}</span>
              {bagRunning&&(
                bagPaused?(
                  <button style={btn()} onClick={resumeBag} disabled={bagBusy}>▶</button>
                ):(
                  <button style={btn()} onClick={pauseBag} disabled={bagBusy}>⏸</button>
                )
              )}
              {bagRunning?(
                <button style={btn(false,true)} onClick={stopBag} disabled={bagBusy}>■ Bag</button>
              ):(
                <button style={{...btn(),opacity:bagPath&&!bagBusy?1:.45}} onClick={playBag} disabled={!bagPath||bagBusy}>▶ Bag</button>
              )}
              <button style={{...btn(),opacity:(bagRecording||bagRecordBusy)? .45 : 1,color:"#ff6680",borderColor:"rgba(255,102,128,0.28)"}} onClick={chooseBagRecordDir} disabled={bagRecording||bagRecordBusy} title={bagRecordDir||DEFAULT_BAG_RECORD_DIR}>저장위치</button>
              <input value={bagRecordDir} onChange={e=>setBagRecordDir(e.target.value)} disabled={bagRecording||bagRecordBusy}
                placeholder={DEFAULT_BAG_RECORD_DIR} title="bag 저장 폴더"
                style={{...INPUT,width:250,padding:"3px 5px",fontSize:10,opacity:bagRecording? .55 : 1,borderColor:"rgba(255,102,128,0.22)"}}/>
              <input value={bagRecordName} onChange={e=>setBagRecordName(e.target.value)} disabled={bagRecording||bagRecordBusy}
                placeholder="bag 이름 자동" title="비워두면 bag_YYYYMMDD_HHMMSS"
                style={{...INPUT,width:112,padding:"3px 5px",fontSize:10,opacity:bagRecording? .55 : 1,borderColor:"rgba(255,102,128,0.22)"}}/>
              {bagRecording?(
                <button style={btn(false,true)} onClick={stopBagRecord} disabled={bagRecordBusy} title={bagRecordPath||DEFAULT_BAG_RECORD_DIR}>■ Rec</button>
              ):(
                <button style={{...btn(),opacity:bagRecordBusy? .45 : 1,color:"#ff6680",borderColor:"rgba(255,102,128,0.35)"}} onClick={startBagRecord} disabled={bagRecordBusy}
                  title={`${bagRecordDir||DEFAULT_BAG_RECORD_DIR}\n${DEFAULT_BAG_RECORD_TOPICS.join("\n")}`}>
                  ⏺ Rec
                </button>
              )}
            </>
          )}
          {isPolyActive&&(
            <div style={{display:"flex",gap:5,marginLeft:6,background:"rgba(0,255,100,0.06)",border:"1px solid rgba(0,255,100,0.25)",borderRadius:5,padding:"2px 10px",alignItems:"center"}}>
              <span style={{color:"#00ff88",fontSize:11}}>🔷 다각형 그리기: {polyVerts.length}개 꼭짓점</span>
              <button style={{...btn(false,true),padding:"1px 7px",fontSize:10}} onClick={()=>{setPolyVerts([]);setPolySnap(false);}}>취소(ESC)</button>
              <button style={{...btn(),padding:"1px 7px",fontSize:10,color:"#00ff88",borderColor:"rgba(0,255,100,0.4)"}} onClick={()=>polyVerts.length>=3&&finishPolygon([...polyVerts])} disabled={polyVerts.length<3}>완성(Enter)</button>
            </div>
          )}
        </div>
      </div>

      <div style={{display:"flex",flex:1,overflow:"hidden"}}>

        {/* ── LEFT TOOLBAR ── */}
        <div style={{width:toolbarW,background:"#060e1c",display:"flex",flexDirection:"column",alignItems:"center",padding:"8px 5px",gap:3,flexShrink:0,overflowY:"auto",position:"relative"}}>
          <div style={{display:"flex",width:"100%",marginBottom:6,gap:2}}>
            {[["edit","✏️ 편집"],["semantic","🗺 시맨틱"]].map(([t,lbl])=>(
              <button key={t} disabled={slamMode&&t==="edit"} onClick={()=>setActiveTab(t)}
                style={{flex:1,border:"none",borderRadius:5,cursor:slamMode&&t==="edit"?"not-allowed":"pointer",padding:"5px 2px",background:activeTab===t?"rgba(0,212,255,0.18)":"transparent",color:activeTab===t?"#00d4ff":"#4a7080",fontSize:10,fontWeight:activeTab===t?"bold":"normal",transition:"all 0.15s",opacity:slamMode&&t==="edit" ? .35 : 1}}>
                {lbl}
              </button>
            ))}
          </div>

          {activeTab==="edit"&&<>
            <span style={{fontSize:10,color:"rgba(0,212,255,0.4)",letterSpacing:1,marginBottom:2,fontWeight:"bold"}}>EDIT</span>
            {EDIT_TOOLS.map(t=>(
              <button key={t.id} title={`${t.label} [${t.key}]`} onClick={()=>setTool(t.id)} style={{
                width:76,height:36,border:"none",borderRadius:6,cursor:"pointer",
                background:tool===t.id?"rgba(0,212,255,0.18)":"transparent",
                boxShadow:tool===t.id?"inset 0 0 0 1px rgba(0,212,255,0.5)":"none",
                color:tool===t.id?"#00d4ff":"#4a7080",
                display:"flex",alignItems:"center",justifyContent:"center",gap:5,transition:"all 0.15s",
              }}>
                <span style={{fontSize:15,lineHeight:1}}>{t.icon}</span>
                <span style={{fontSize:10}}>{t.label}</span>
              </button>
            ))}
            <div style={{width:"80%",height:1,background:"rgba(0,212,255,0.1)",margin:"5px 0"}}/>
            {DRAW_COLORS.map(cl=>(
              <button key={cl.val} title={cl.label} onClick={()=>setDrawColor(cl.val)} style={{width:34,height:34,borderRadius:6,cursor:"pointer",background:cl.css,border:drawColor===cl.val?"2px solid #00d4ff":`2px solid ${cl.border}`,boxShadow:drawColor===cl.val?"0 0 8px rgba(0,212,255,0.5)":"none",transition:"all 0.15s",marginTop:2}}/>
            ))}
            <div style={{width:"80%",height:1,background:"rgba(0,212,255,0.1)",margin:"5px 0"}}/>
            <input type="range" min={1} max={40} value={brushSz} onChange={e=>setBrushSz(Number(e.target.value))} style={{width:56,height:70,writingMode:"vertical-lr",direction:"rtl",cursor:"pointer",accentColor:"#00d4ff"}}/>
            <span style={{fontSize:12,color:"#00d4ff",fontWeight:"bold"}}>{brushSz}</span>
          </>}

          {activeTab==="semantic"&&<>
            {SEM_TOOL_GROUPS.map(g=>(
              <div key={g.label} style={{width:"100%",marginBottom:6}}>
                <div style={{fontSize:11,color:g.color,fontWeight:"bold",letterSpacing:1,textAlign:"center",marginBottom:4}}>{g.label}</div>
                <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:3,padding:"0 3px"}}>
                  {g.tools.map(t=>(
                    <button key={t.id} title={`${g.label} ${t.label}${t.key?` [${t.key}]`:""}`} onClick={()=>{setTool(t.id);if(polyVertsRef.current.length>0&&t.id!==tool){setPolyVerts([]);setPolySnap(false);}}} style={{
                      height:42,border:"none",borderRadius:5,cursor:"pointer",
                      background:tool===t.id?`${g.color}33`:"rgba(255,255,255,0.03)",
                      boxShadow:tool===t.id?`inset 0 0 0 1.5px ${g.color}88`:"none",
                      color:tool===t.id?g.color:"#5a8a9a",
                      display:"flex",flexDirection:"column",alignItems:"center",justifyContent:"center",gap:2,transition:"all 0.15s",
                    }}>
                      <span style={{fontSize:15,lineHeight:1}}>{t.icon}</span>
                      <span style={{fontSize:10}}>{t.label}</span>
                    </button>
                  ))}
                </div>
              </div>
            ))}
            <div style={{width:"80%",height:1,background:"rgba(0,212,255,0.1)",margin:"5px 0"}}/>
            {SEM_EXTRA_TOOLS.map(t=>{
              const captureKind=SEM_CAPTURE_KINDS[t.id];
              const canCapture=mapLoaded&&ros2State===ROS2_STATES.CONNECTED;
              return(
                <div key={t.id} style={{width:"100%",boxSizing:"border-box",display:"flex",gap:3,padding:"0 3px",marginBottom:3}}>
                  <button title={`${t.label} [${t.key}]`} onClick={()=>{setTool(t.id);if(polyVertsRef.current.length>0&&t.id!==tool){setPolyVerts([]);setPolySnap(false);}}} style={{
                    flex:1,minWidth:0,height:42,border:"none",borderRadius:5,cursor:"pointer",
                    background:tool===t.id?"rgba(0,212,255,0.2)":"rgba(255,255,255,0.03)",
                    boxShadow:tool===t.id?`inset 0 0 0 1.5px ${t.color}88`:"none",
                    color:tool===t.id?t.color:"#5a8a9a",
                    display:"flex",flexDirection:"column",alignItems:"center",justifyContent:"center",gap:2,transition:"all 0.15s",
                  }}>
                    <span style={{fontSize:16,lineHeight:1}}>{t.icon}</span>
                    <span style={{fontSize:10,whiteSpace:"nowrap",overflow:"hidden",textOverflow:"ellipsis",maxWidth:"100%"}}>{t.label}</span>
                  </button>
                  {captureKind&&(
                    <button
                      title={SEM_CAPTURE_TITLES[t.id]}
                      onClick={()=>captureRobotPose(captureKind)}
                      disabled={!canCapture}
                      style={{
                        width:30,height:42,border:"none",borderRadius:5,cursor:canCapture?"pointer":"not-allowed",
                        background:canCapture?`${t.color}18`:"rgba(255,255,255,0.03)",
                        boxShadow:canCapture?`inset 0 0 0 1px ${t.color}66`:"none",
                        color:canCapture?t.color:"rgba(90,138,154,0.45)",
                        display:"flex",flexDirection:"column",alignItems:"center",justifyContent:"center",gap:1,
                        opacity:canCapture?1:.45,
                      }}>
                      <span style={{fontSize:14,lineHeight:1}}>⌖</span>
                      <span style={{fontSize:9,lineHeight:1}}>현</span>
                    </button>
                  )}
                </div>
              );
            })}
            <div style={{width:"80%",height:1,background:"rgba(0,212,255,0.1)",margin:"5px 0"}}/>
            <span style={{fontSize:10,color:"rgba(0,212,255,0.45)",fontWeight:"bold",marginBottom:3}}>투명도</span>
            <input type="range" min={.1} max={1} step={.05} value={semOpacity} onChange={e=>setSemOpacity(Number(e.target.value))} style={{width:56,height:60,writingMode:"vertical-lr",direction:"rtl",cursor:"pointer",accentColor:"#00d4ff"}}/>
            <span style={{fontSize:12,color:"#00d4ff",fontWeight:"bold"}}>{Math.round(semOpacity*100)}%</span>
          </>}
        </div>

        {/* ── TOOLBAR RESIZE HANDLE ── */}
        <div style={{width:5,cursor:"col-resize",background:"rgba(0,212,255,0.08)",flexShrink:0,position:"relative"}}
          onMouseDown={e=>{
            e.preventDefault();
            tbResizing.current=true;
            const startX=e.clientX,startW=toolbarW;
            const onMove=ev=>{if(!tbResizing.current)return;const nw=Math.max(60,Math.min(200,startW+ev.clientX-startX));setToolbarW(nw);};
            const onUp=()=>{tbResizing.current=false;window.removeEventListener("mousemove",onMove);window.removeEventListener("mouseup",onUp);};
            window.addEventListener("mousemove",onMove);window.addEventListener("mouseup",onUp);
          }}>
          <div style={{position:"absolute",top:"50%",left:1,transform:"translateY(-50%)",width:3,height:30,borderRadius:2,background:"rgba(0,212,255,0.25)"}}/>
        </div>

        {/* ── CENTER: 2D + 3D VIEWPORTS ── */}
        <div style={{display:"flex",flex:1,minWidth:0,overflow:"hidden"}}>
          {/* ── CANVAS VIEWPORT (2D) ── */}
          <div ref={vpRef} style={{flex:1,minWidth:280,overflow:"hidden",position:"relative",background:"#030a14",cursor:tool==="semSelect"?"default":isPanning.current?"grabbing":"crosshair"}}
            onMouseDown={onMouseDown} onMouseMove={onMouseMove} onMouseUp={onMouseUp} onMouseLeave={onMouseLeave}
            onDoubleClick={onDblClick}>
            <div style={{position:"absolute",inset:0,opacity:.022,backgroundImage:"linear-gradient(rgba(0,212,255,.5) 1px,transparent 1px),linear-gradient(90deg,rgba(0,212,255,.5) 1px,transparent 1px)",backgroundSize:"40px 40px",pointerEvents:"none"}}/>

          {!mapLoaded&&(
            <div style={{position:"absolute",inset:0,display:"flex",flexDirection:"column",alignItems:"center",justifyContent:"center",gap:12,zIndex:100}}
              onMouseDown={e=>e.stopPropagation()} onClick={e=>e.stopPropagation()}>
              <div style={{fontSize:56,opacity:.2}}>◈</div>
              <div style={{fontSize:13,letterSpacing:2,color:"rgba(0,212,255,0.35)"}}>NAV2 MAP EDITOR</div>
              <div style={{fontSize:10,color:"rgba(0,212,255,0.2)",marginBottom:6}}>3계층 시맨틱 맵 (방 → 캐리어 → 객체)</div>
              <div style={{display:"flex",gap:10}}>
                {hasHostAPI ? (
                  <button style={{...btn(true),fontSize:12,padding:"8px 18px"}} onClick={onOpenClick}>📂 PGM/YAML 열기</button>
                ) : (
                  <label style={{...btn(true),fontSize:12,padding:"8px 18px",cursor:"pointer"}}>📂 PGM/YAML 열기<input type="file" accept=".pgm,.yaml,.yml,.json" multiple onChange={handleFiles} style={{display:"none"}}/></label>
                )}
                <button style={{...btn(true),fontSize:12,padding:"8px 18px",borderColor:"#00d4ff"}} onClick={()=>setShowNewDlg(true)}>✦ 새 맵</button>
              </div>
            </div>
          )}

          <div style={{position:"absolute",transform:`translate(${pan.x}px,${pan.y}px) rotate(${rotation}deg) scale(${zoom})`,transformOrigin:"0 0"}}>
            <canvas ref={canvasRef} style={{display:"block",imageRendering:"pixelated",border:"1px solid rgba(0,212,255,0.2)"}}/>
            <canvas ref={overlayRef} style={{position:"absolute",top:0,left:0,display:"block",pointerEvents:"none"}}/>
          </div>

            {/* Tool hint bar */}
            {toolTips[tool]&&mapLoaded&&(
              <div style={{position:"absolute",top:12,left:"50%",transform:"translateX(-50%)",background:"rgba(0,8,20,0.85)",border:"1px solid rgba(0,212,255,0.22)",borderRadius:5,padding:"4px 14px",fontSize:11,color:"rgba(0,212,255,0.7)",pointerEvents:"none",whiteSpace:"nowrap"}}>
                {toolTips[tool]}
              </div>
            )}

            {/* Zoom & rotation controls */}
            <div style={{position:"absolute",bottom:14,right:14,display:"flex",gap:4,flexDirection:"column",alignItems:"center"}}>
              <button style={btn()} onClick={()=>rotateMap(-90)} title="90° 반시계">↺</button>
              <button style={btn()} onClick={()=>rotateMap(90)} title="90° 시계">↻</button>
              <div style={{height:2}}/>
              <button style={btn()} onClick={()=>setZoom(z=>Math.min(z*1.25,16))}>＋</button>
              <button style={btn()} onClick={()=>setZoom(z=>Math.max(z/1.25,.05))}>－</button>
              <button style={btn()} onClick={fitView}>⊞</button>
              {rotation!==0&&<span style={{fontSize:9,color:"rgba(0,212,255,0.5)",marginTop:2}}>{rotation%360}°</span>}
            </div>

            {/* Coord HUD */}
            {cursor.vis&&mapLoaded&&(
              <div style={{position:"absolute",bottom:14,left:14,background:"rgba(0,8,20,0.85)",border:"1px solid rgba(0,212,255,0.15)",borderRadius:5,padding:"4px 10px",fontSize:11,color:"rgba(0,212,255,0.6)",pointerEvents:"none"}}>
                px ({cursor.x}, {cursor.y}) · m ({realX.toFixed(2)}, {realY.toFixed(2)})
              </div>
            )}
          </div>

          {show3DView&&(
            <>
              <div style={{width:5,cursor:"col-resize",background:"rgba(0,212,255,0.08)",flexShrink:0,position:"relative"}}
                onMouseDown={e=>{
                  e.preventDefault();
                  view3DResizing.current=true;
                  const startX=e.clientX,startW=view3DWidth;
                  const onMove=ev=>{
                    if(!view3DResizing.current)return;
                    const nw=Math.max(260,Math.min(900,startW-(ev.clientX-startX)));
                    setView3DWidth(nw);
                  };
                  const onUp=()=>{
                    view3DResizing.current=false;
                    window.removeEventListener("mousemove",onMove);
                    window.removeEventListener("mouseup",onUp);
                  };
                  window.addEventListener("mousemove",onMove);
                  window.addEventListener("mouseup",onUp);
                }}>
                <div style={{position:"absolute",top:"50%",left:1,transform:"translateY(-50%)",width:3,height:30,borderRadius:2,background:"rgba(0,212,255,0.25)"}}/>
              </div>
              <div style={{width:view3DWidth,minWidth:260,flexShrink:0,position:"relative"}}>
                <Ros2View3D
                  mapCanvasRef={canvasRef}
                  meta={meta}
                  canvasSize={canvasSize}
                  lidarWorldPoints={lidarWorldPoints}
                  pathWorldPoints={pathWorldPoints}
                  stats={ros2Stats}
                  fixedFrame={ros2Frames?.fixed || "map"}
                  viewMode={view3DMode}
                  onChangeViewMode={setView3DMode}
                />
              </div>
            </>
          )}
        </div>

        {/* ── MD CATALOG PANEL ── */}
        {showCatalogPanel&&(
          <CatalogPanel
            catalog={semanticCatalog}
            sources={catalogSources}
            placedRooms={rooms}
            placedCarriers={carriers}
            placedObjects={objects}
            hasHostAPI={hasHostAPI}
            onImport={handleCatalogOpen}
            onFileImport={handleCatalogFiles}
            onRemoveSource={removeCatalogSource}
            onReset={resetCatalogSources}
            onAddRoom={addCatalogRoom}
            onUpdateRoom={updateCatalogRoom}
            onDeleteRoom={deleteCatalogRoom}
            onResetRooms={resetCatalogRooms}
            onSelectPlaced={item=>{
              if(!item)return;
              setSelSemId(item.id);
              setSelWpIdx(null);
              setShowSemPanel(true);
              setStatus(`✅ 시맨틱 선택: ${item.label||item.id}`);
            }}
          />
        )}

        {/* ── ROS2 PANEL ── */}
        {showRos2Panel&&(
          <Ros2Panel bridge={ros2Bridge} defaultUrl={defaultRosbridgeUrl(9090)} vis={ros2Vis} onVisChange={setRos2Vis}
            frames={ros2Frames} onFramesChange={setRos2Frames} availableFrames={ros2AvailFrames}
            stats={ros2Stats} meta={meta} canvasSize={canvasSize} cameraDataUrl={cameraDataUrl}/>
        )}

        {/* ── SLAM PANEL ── */}
        {showSlamPanel&&(
          <SlamModePanel
            hasHostAPI={hasHostAPI}
            ros2Connected={ros2State===ROS2_STATES.CONNECTED}
            rosbridgeRunning={rosbridgeRunning}
            rosbridgeBusy={rosbridgeBusy}
            onStartBridge={startRosbridge}
            slamMode={slamMode}
            onEnterMode={enterSlamMode}
            onExitMode={exitSlamMode}
            slamRunning={slamRunning}
            slamBusy={slamBusy}
            slamSaveBusy={slamSaveBusy}
            onStartSlam={startSlamToolbox}
            onStopSlam={stopSlamToolbox}
            onSaveSlamResult={saveSlamResult}
            nav2Running={nav2Running}
            nav2Busy={nav2Busy}
            nav2ParamsFile={nav2ParamsFile}
            onChangeNav2ParamsFile={setNav2ParamsFile}
            onStartNav2={startNav2}
            onStopNav2={stopNav2}
            slamSaveName={slamSaveName}
            onChangeSaveName={setSlamSaveName}
            slamShowFootprint={slamShowFootprint}
            onChangeShowFootprint={setSlamShowFootprint}
            slamFootprintTopic={slamFootprintTopic}
            onChangeFootprintTopic={setSlamFootprintTopic}
            slamShowRobotModel={slamShowRobotModel}
            onChangeShowRobotModel={setSlamShowRobotModel}
            slamRobotDescriptionTopic={slamRobotDescriptionTopic}
            onChangeRobotDescriptionTopic={setSlamRobotDescriptionTopic}
            slamShowCostmaps={slamShowCostmaps}
            onChangeShowCostmaps={setSlamShowCostmaps}
            slamGlobalCostmapTopic={slamGlobalCostmapTopic}
            onChangeGlobalCostmapTopic={setSlamGlobalCostmapTopic}
            slamLocalCostmapTopic={slamLocalCostmapTopic}
            onChangeLocalCostmapTopic={setSlamLocalCostmapTopic}
            ros2Stats={ros2Stats}
            slamMapTopic={slamMapTopic}
            onChangeMapTopic={setSlamMapTopic}
            slamUseSimTime={slamUseSimTime}
            onChangeUseSimTime={setSlamUseSimTime}
            slamParamsFile={slamParamsFile}
            onChangeParamsFile={setSlamParamsFile}
            slamMapStats={slamMapStats}
            defaultMapDir={DEFAULT_MAP_SEARCH_DIR}
            onPickGoalTool={()=>{setActiveTab("semantic");setTool("semGoal");setShowSemPanel(true);}}
            onPickSelectTool={()=>{setActiveTab("semantic");setTool("semSelect");setShowSemPanel(true);}}
            onOpenWorkspaceSync={()=>setShowWorkspaceDlg(true)}
          />
        )}

        {/* ── SEMANTIC PANEL (rooms + carriers + objects + waypoints) ── */}
        {showSemPanel&&(
          <>
          <div style={{width:5,cursor:"col-resize",background:"rgba(0,212,255,0.08)",flexShrink:0,position:"relative"}}
            onMouseDown={e=>{
              e.preventDefault();
              semPanelResizing.current=true;
              const startX=e.clientX,startW=semPanelW;
              const onMove=ev=>{
                if(!semPanelResizing.current)return;
                const nw=Math.max(240,Math.min(620,startW-(ev.clientX-startX)));
                setSemPanelW(nw);
              };
              const onUp=()=>{
                semPanelResizing.current=false;
                window.removeEventListener("mousemove",onMove);
                window.removeEventListener("mouseup",onUp);
              };
              window.addEventListener("mousemove",onMove);
              window.addEventListener("mouseup",onUp);
            }}>
            <div style={{position:"absolute",top:"50%",left:1,transform:"translateY(-50%)",width:3,height:30,borderRadius:2,background:"rgba(0,212,255,0.25)"}}/>
          </div>
          <SemanticPanel
            width={semPanelW}
            maps={maps} rooms={rooms} carriers={carriers} objects={objects} emptySeats={emptySeats} waypoints={waypoints} goals={goals} goalList={goalList}
            startPoses={startPoses} startTasks={START_TASKS} setSelectedStartTask={setSelectedStartTask}
            selId={selSemId} setSelId={setSelSemId}
            selWpIdx={selWpIdx} setSelWpIdx={setSelWpIdx}
            onDeleteMap={id=>{setMaps(p=>p.filter(m=>m.id!==id));if(selSemId===id)setSelSemId(null);}}
            onDeleteRoom={id=>{setRooms(p=>p.filter(r=>r.id!==id));if(selSemId===id)setSelSemId(null);}}
            onDeleteCarrier={id=>{setCarriers(p=>p.filter(c=>c.id!==id));if(selSemId===id)setSelSemId(null);}}
            onDeleteObj={id=>{setObjects(p=>p.filter(o=>o.id!==id));if(selSemId===id)setSelSemId(null);}}
            onDeleteEmptySeat={id=>{setEmptySeats(p=>p.filter(seat=>seat.id!==id));if(selSemId===id)setSelSemId(null);}}
            onDeleteWp={i=>{setWaypoints(p=>p.filter((_,j)=>j!==i));if(selWpIdx===i)setSelWpIdx(null);}}
            onDeleteGoal={id=>{setGoals(p=>p.filter(g=>g.id!==id));if(selSemId===id)setSelSemId(null);}}
            onDeleteStart={task=>{setStartPoseForTask(task||selectedStartTask,null);if(startTaskFromId(startPoses,selSemId))setSelSemId(null);}}
            onAddGoalList={()=>setGoalList(p=>[...p,{id:gluid(),label:`goal_list_${p.length+1}`,goal_id:""}])}
            onUpdateGoalList={(id,patch)=>setGoalList(p=>p.map(item=>item.id===id?{...item,...patch}:item))}
            onDeleteGoalList={id=>setGoalList(p=>p.filter(item=>item.id!==id))}
            onImportGoalList={hasHostAPI?handleGoalListOpen:null}
            onImportGoalListFile={!hasHostAPI?handleGoalListFile:null}
            onExportGoalList={saveGoalListJSON}
            onReassign={(layer,id,field,value)=>{
              if(layer==="room") setRooms(p=>p.map(r=>r.id===id?{...r,[field]:value}:r));
              else if(layer==="carrier") setCarriers(p=>p.map(c=>c.id===id?{...c,[field]:value}:c));
              else if(layer==="object") setObjects(p=>p.map(o=>o.id===id?{...o,[field]:value}:o));
              else if(layer==="emptySeat") setEmptySeats(p=>p.map(seat=>seat.id===id?{...seat,[field]:value}:seat));
              else if(layer==="goal") setGoals(p=>p.map(g=>g.id===id?{...g,[field]:value}:g));
            }}
            onRenameObjId={(oldId,nextRaw)=>{
              const nextId=String(nextRaw||"").trim();
              if(!nextId){setStatus("⚠ 오브젝트 ID는 비워둘 수 없습니다");return false;}
              const usedIds=[
                ...maps.map(m=>m.id),...rooms.map(r=>r.id),...carriers.map(c=>c.id),
                ...objects.filter(o=>o.id!==oldId).map(o=>o.id),
                ...emptySeats.map(seat=>seat.id),
                ...goals.map(g=>g.id),...waypoints.map((wp,i)=>wp.id||`w${i+1}`),
                ...Object.values(startPoses).map(p=>p?.id).filter(Boolean),
              ];
              if(usedIds.includes(nextId)){
                setStatus(`⚠ 이미 있는 ID: ${nextId}`);
                return false;
              }
              const idMatch=nextId.match(/^o(\d+)$/);
              if(idMatch)_oIdx=Math.max(_oIdx,Number(idMatch[1]));
              setObjects(p=>p.map(o=>o.id===oldId?{...o,id:nextId}:o));
              setGoals(p=>p.map(g=>g.target_id===oldId?{...g,target_id:nextId}:g));
              if(selSemId===oldId)setSelSemId(nextId);
              setStatus(`🔹 오브젝트 ID 변경: ${oldId} → ${nextId}`);
              return true;
            }}
            onRenameEmptySeatId={(oldId,nextRaw)=>{
              const nextId=String(nextRaw||"").trim();
              if(!nextId){setStatus("⚠ 빈좌석 ID는 비워둘 수 없습니다");return false;}
              const usedIds=[
                ...maps.map(m=>m.id),...rooms.map(r=>r.id),...carriers.map(c=>c.id),...objects.map(o=>o.id),
                ...emptySeats.filter(seat=>seat.id!==oldId).map(seat=>seat.id),
                ...goals.map(g=>g.id),...waypoints.map((wp,i)=>wp.id||`w${i+1}`),
                ...Object.values(startPoses).map(p=>p?.id).filter(Boolean),
              ];
              if(usedIds.includes(nextId)){
                setStatus(`⚠ 이미 있는 ID: ${nextId}`);
                return false;
              }
              const idMatch=nextId.match(/^e(\d+)$/);
              if(idMatch)_eIdx=Math.max(_eIdx,Number(idMatch[1]));
              setEmptySeats(p=>p.map(seat=>seat.id===oldId?{...seat,id:nextId,label:seat.label===oldId?nextId:seat.label}:seat));
              setGoals(p=>p.map(g=>g.target_id===oldId?{...g,target_id:nextId}:g));
              if(selSemId===oldId)setSelSemId(nextId);
              setStatus(`▧ 빈좌석 ID 변경: ${oldId} → ${nextId}`);
              return true;
            }}
            onRenameGoalId={(oldId,nextRaw)=>{
              const nextId=String(nextRaw||"").trim();
              if(!nextId){setStatus("⚠ 골 ID는 비워둘 수 없습니다");return false;}
              if(goalsRef.current.some(g=>g.id===nextId&&g.id!==oldId)){
                setStatus(`⚠ 이미 있는 골 ID: ${nextId}`);
                return false;
              }
              setGoals(p=>p.map(g=>g.id===oldId?{...g,id:nextId}:g));
              setGoalList(p=>p.map(item=>item.goal_id===oldId?{...item,goal_id:nextId}:item));
              if(selSemId===oldId)setSelSemId(nextId);
              setStatus(`🎯 골 ID 변경: ${oldId} → ${nextId}`);
              return true;
            }}
            onRenameStartPoseId={(task,nextRaw)=>{
              const key=START_TASKS.includes(task)?task:selectedStartTask;
              const oldId=startPoses[key]?.id||"";
              const nextId=String(nextRaw||"").trim();
              if(!nextId){setStatus("⚠ 시작점 ID는 비워둘 수 없습니다");return false;}
              const usedIds=[
                ...maps.map(m=>m.id),...rooms.map(r=>r.id),...carriers.map(c=>c.id),...objects.map(o=>o.id),...emptySeats.map(seat=>seat.id),
                ...goals.map(g=>g.id),...waypoints.map((wp,i)=>wp.id||`w${i+1}`),
                ...Object.entries(startPoses).filter(([t])=>t!==key).map(([,p])=>p?.id).filter(Boolean),
              ];
              if(usedIds.includes(nextId)){
                setStatus(`⚠ 이미 있는 ID: ${nextId}`);
                return false;
              }
              setStartPoseForTask(key,p=>p?{...p,id:nextId,label:key}:p);
              if(selSemId===oldId)setSelSemId(nextId);
              setStatus(`⌂ 시작점 ID 변경: ${oldId} → ${nextId}`);
              return true;
            }}
            setWaypoints={setWaypoints}
            onImportJSON={hasHostAPI?handleSemanticOpen:null}
            onImportFile={!hasHostAPI?handleSemanticFile:null}
            onExportJSON={async()=>{
              const base=cleanMapBaseName(meta.filename||"map")||"map";
              const defaultName=hasHostAPI?`${DEFAULT_MAP_SEARCH_DIR}/${base}_semantic.json`:`${base}_semantic.json`;
              await nativeSave(defaultName,[{name:"JSON",extensions:["json"]}],buildSemanticJSON(),"utf-8",pickHostPath);
            }}
            toWorld={toWorld} resolution={meta.resolution}
            typeOptions={typeOptions}
          />
          </>
        )}
      </div>

      {/* ── STATUS BAR ── */}
      <div style={{padding:"3px 12px",background:"#030a14",borderTop:"1px solid rgba(0,212,255,0.07)",display:"flex",gap:12,alignItems:"center",flexShrink:0,fontSize:11,color:"rgba(0,212,255,0.28)"}}>
        <span style={{color:"rgba(0,212,255,0.5)"}}>{status}</span>
        {mapLoaded&&<><span>{canvasSize.w}×{canvasSize.h}px</span><span>{meta.resolution}m/px</span></>}
        <span style={{marginLeft:"auto"}}>B/E/L/R/C/F=편집 · S=시작 · W=WP · Q=빈좌석 · 1/2=방 · 3/4=캐리어 · 5/6=객체 · 7=포인트 · 8/0=선택 · 9=골 · Backspace/Del=삭제</span>
      </div>

      {/* ── NEW MAP DIALOG ── */}
      {showNewDlg&&(
        <div style={MODAL}><div style={MBOX}>
          <h3 style={{margin:"0 0 16px",color:"#00d4ff",letterSpacing:1}}>✦ 새 맵</h3>
          <div style={{display:"flex",gap:10,marginBottom:10}}>
            {[["너비(px)",newW,setNewW],["높이(px)",newH,setNewH]].map(([l,v,s])=>(
              <label key={l} style={{display:"flex",flexDirection:"column",gap:4,flex:1}}>
                <span style={{fontSize:10,color:"rgba(0,212,255,0.5)"}}>{l}</span>
                <input type="number" value={v} onChange={e=>s(Number(e.target.value))} min={10} max={4096} style={{...INPUT,width:"100%",boxSizing:"border-box"}}/>
              </label>
            ))}
          </div>
          <div style={{fontSize:10,color:"rgba(0,212,255,0.3)",marginBottom:16,padding:"6px 10px",background:"rgba(0,212,255,0.04)",borderRadius:4}}>
            {(newW*meta.resolution).toFixed(1)} × {(newH*meta.resolution).toFixed(1)} m
          </div>
          <div style={{display:"flex",gap:8,justifyContent:"flex-end"}}>
            <button style={btn()} onClick={()=>setShowNewDlg(false)}>취소</button>
            <button style={{...btn(true),borderColor:"#00d4ff",color:"#00d4ff"}} onClick={()=>{initCanvas(newW,newH);setShowNewDlg(false);setStatus(`✦ 새 맵: ${newW}×${newH}px`);}}>만들기</button>
          </div>
        </div></div>
      )}

      {/* ── META DIALOG ── */}
      {showMetaDlg&&(
        <div style={MODAL}><div style={{...MBOX,minWidth:360}}>
          <h3 style={{margin:"0 0 16px",color:"#00d4ff",letterSpacing:1}}>⚙ 맵 메타데이터</h3>
          {[{k:"filename",l:"파일명",t:"text"},{k:"resolution",l:"해상도(m/px)",t:"number",s:.001},{k:"negate",l:"반전",t:"number",s:1},{k:"occupied_thresh",l:"점유 임계값",t:"number",s:.01},{k:"free_thresh",l:"자유 임계값",t:"number",s:.01}].map(f=>(
            <label key={f.k} style={{display:"flex",flexDirection:"column",gap:4,marginBottom:10}}>
              <span style={{fontSize:10,color:"rgba(0,212,255,0.5)"}}>{f.l}</span>
              <input type={f.t} step={f.s} value={meta[f.k]} onChange={e=>setMeta(m=>({...m,[f.k]:f.t==="text"?e.target.value:parseFloat(e.target.value)}))} style={{...INPUT,width:"100%",boxSizing:"border-box"}}/>
            </label>
          ))}
          <label style={{display:"flex",flexDirection:"column",gap:4,marginBottom:16}}>
            <span style={{fontSize:10,color:"rgba(0,212,255,0.5)"}}>Origin [x, y, θ] (m)</span>
            <div style={{display:"flex",gap:6}}>
              {[0,1,2].map(i=>(
                <input key={i} type="number" step=".1" value={meta.origin[i]} onChange={e=>setMeta(m=>({...m,origin:m.origin.map((v,j)=>j===i?parseFloat(e.target.value):v)}))} style={{...INPUT,flex:1,minWidth:0}}/>
              ))}
            </div>
          </label>
          <div style={{display:"flex",justifyContent:"flex-end"}}>
            <button style={{...btn(true),borderColor:"#00d4ff",color:"#00d4ff"}} onClick={()=>setShowMetaDlg(false)}>확인</button>
          </div>
        </div></div>
      )}

      {/* ── WORKSPACE SYNC DIALOG ── */}
      {showWorkspaceDlg&&(
        <div style={MODAL}><div style={{...MBOX,minWidth:520,maxWidth:680,width:"min(680px,92vw)"}}>
          <h3 style={{margin:"0 0 16px",color:"#00d4ff",letterSpacing:1}}>🏗 빌드</h3>
          <div style={{display:"flex",flexDirection:"column",gap:10}}>
            <label style={{display:"flex",flexDirection:"column",gap:4}}>
              <span style={{fontSize:10,color:"rgba(0,212,255,0.5)"}}>맵 YAML 경로</span>
              <div style={{display:"flex",gap:6}}>
                <input value={workspaceSync.mapPath} onChange={e=>setWorkspaceSync(prev=>({...prev,mapPath:e.target.value}))}
                  placeholder={DEFAULT_MAP_SEARCH_DIR}
                  style={{...INPUT,flex:1,minWidth:0}}/>
                <button style={btn()} onClick={chooseMapYamlPath} disabled={workspaceBusy}>찾기</button>
              </div>
            </label>
            <label style={{display:"flex",flexDirection:"column",gap:4}}>
              <span style={{fontSize:10,color:"rgba(0,212,255,0.5)"}}>package 이름</span>
              <input value={workspaceSync.packageName} onChange={e=>setWorkspaceSync(prev=>({...prev,packageName:e.target.value}))}
                placeholder="rby1_nav2"
                style={{...INPUT,width:"100%",boxSizing:"border-box"}}/>
            </label>
            <label style={{display:"flex",flexDirection:"column",gap:4}}>
              <span style={{fontSize:10,color:"rgba(0,212,255,0.5)"}}>ROS2 워크스페이스 루트</span>
              <div style={{display:"flex",gap:6}}>
                <input value={workspaceSync.workspaceRoot} onChange={e=>setWorkspaceSync(prev=>({...prev,workspaceRoot:e.target.value}))}
                  placeholder={DEFAULT_WORKSPACE_ROOT}
                  style={{...INPUT,flex:1,minWidth:0}}/>
                <button style={btn()} onClick={chooseWorkspaceRoot} disabled={workspaceBusy}>찾기</button>
              </div>
            </label>
            <div style={{fontSize:10,color:"rgba(0,212,255,0.3)",lineHeight:1.7,padding:"8px 10px",background:"rgba(0,212,255,0.04)",borderRadius:5}}>
              launch 파일은 수정하지 않고, 지정한 워크스페이스에서 <b>colcon build</b>만 실행합니다. 맵 찾기 기본 폴더는 <b>{DEFAULT_MAP_SEARCH_DIR}</b>입니다.
            </div>
          </div>
          <div style={{display:"flex",gap:8,justifyContent:"flex-end",marginTop:18}}>
            <button style={btn()} onClick={()=>setShowWorkspaceDlg(false)} disabled={workspaceBusy}>닫기</button>
            <button style={{...btn(true),borderColor:"#00d4ff",color:"#00d4ff",opacity:canSyncWorkspace?1:.45}} onClick={buildWorkspaceOnly} disabled={!canSyncWorkspace}>
              {workspaceBusy?"빌드 중...":"빌드"}
            </button>
          </div>
        </div></div>
      )}

      {/* ── SEMANTIC TYPE DIALOG ── */}
      {semDlg&&<SemanticDialog mode={semDlg.mode} typeOptions={typeOptions} onConfirm={onSemConfirm} onCancel={()=>setSemDlg(null)}/>}

      {/* ── GOAL DIALOG ── */}
      {goalDlg&&<GoalDialog rooms={rooms} carriers={carriers} objects={objects} emptySeats={emptySeats} goalList={goalList} roomId={goalDlg.roomId} goalId={goalDlg.goalId} typeOptions={typeOptions} onConfirm={onGoalConfirm} onCancel={()=>{
        // Remove the goal that was placed during drag
        if(goalDlg.goalId) setGoals(p=>p.filter(g=>g.id!==goalDlg.goalId));
        setGoalDlg(null);
      }}/>}

      {/* ── START POSE DIALOG ── */}
      {startDlg&&<StartPoseDialog startTasks={START_TASKS} startPoses={startPoses} pose={startDlg.pose} defaultTask={startDlg.defaultTask} defaultId={startDlg.defaultId} onConfirm={onStartConfirm} onCancel={cancelStartDialog}/>}

      {/* ── ROBOT PC FILE BROWSER ── */}
      {fileDialog&&hostAPI?.isRobotBackend&&(
        <RobotFileBrowser key={fileDialog.key} api={hostAPI} request={fileDialog} onClose={closeFileDialog}/>
      )}

      {/* ── THOR MD PICKER ── */}
      {thorMdPicker&&(
        <ThorMdPicker key={thorMdPicker.key} request={thorMdPicker} onCancel={()=>setThorMdPicker(null)} onConfirm={importThorMdSelection} onBrowse={browseThorMdDir}/>
      )}
    </div>
  );
}
