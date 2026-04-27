# Nav2 Map Editor

![React](https://img.shields.io/badge/React-19-61dafb?logo=react&logoColor=111)
![Vite](https://img.shields.io/badge/Vite-8-646cff?logo=vite&logoColor=fff)
![Electron](https://img.shields.io/badge/Electron-35-47848f?logo=electron&logoColor=fff)
![ROS2](https://img.shields.io/badge/ROS2-Nav2-22314e?logo=ros&logoColor=fff)

Nav2 Map Editor is a desktop/web tool for editing ROS2 Nav2 occupancy maps and semantic navigation data.
It can edit `PGM` maps, load `YAML` metadata, build semantic layers, capture robot poses from ROS2, control rosbag playback, and visualize ROS2 topics through rosbridge.

The app is designed for workflows where a map is not just an occupancy grid, but also contains rooms, carriers, objects, semantic goals, Nav2 initial poses, and waypoints.

## Highlights

- Edit Nav2 `PGM` maps with brush, eraser, line, rectangle, circle, and fill tools
- Load `YAML` map metadata and resolve relative image paths in Electron
- Import and export editable `semantic_map.json`
- Import Markdown catalogs for fixed room names, location/carrier lists, and known object classes
- Draw semantic map, room, carrier, object, point-object, goal, start-pose, and waypoint layers
- Click and drag to set yaw for start pose, waypoint, and semantic goal
- Capture current robot pose from ROS2 or rosbag and save it as a start pose, waypoint, or semantic goal
- Publish Nav2 initial pose to `/initialpose`
- Launch and stop `rosbridge_server` from the Electron UI
- Play, stop, pause, resume, seek, and loop ROS2 bags from the Electron UI
- Visualize ROS2 LaserScan, PointCloud2, odometry, AMCL pose, path, image, and depth topics
- Optional 3D view for map, PointCloud2, path, and robot pose inspection
- Includes a ROS2 `semantic_nav2` package for semantic map serving, waypoint navigation, and semantic costmap experiments

## Screenshot

Add a screenshot here after the UI is running:

```text
docs/images/nav2-map-editor.png
```

## Requirements

For the editor:

- Node.js 18 or newer
- npm 9 or newer

For ROS2 integration:

- ROS2 with Nav2
- `rosbridge_suite`
- A running `rosbridge_websocket` server, or the Electron app's built-in Bridge button

## Quick Start

Install dependencies:

```bash
npm install
```

Run in browser development mode:

```bash
npm run dev
```

Open:

```text
http://localhost:5173
```

Run as an Electron desktop app:

```bash
npm run electron:dev
```

Build the web bundle:

```bash
npm run build
```

Build desktop packages:

```bash
npm run electron:build
```

## Map Workflow

1. Open a `PGM` map or a Nav2 `YAML` map file.
2. If opening from the browser, select `PGM` and `YAML` together so metadata is available.
3. Edit occupancy pixels using the edit tools.
4. Use the semantic tools to add map areas, rooms, carriers, objects, start pose, waypoints, and goals.
5. Save everything with full export, or save only the semantic JSON.

Electron mode can automatically load a sibling semantic file:

- `map_name_semantic.json`
- `semantic_map.json`

## Semantic Editing

The semantic layer supports these editable entities:

| Entity | Purpose |
|---|---|
| Map area | Building, floor, zone, outdoor area, or custom region |
| Room | Bedroom, office, corridor, lab, storage, and other room-like spaces |
| Carrier | Desk, table, shelf, cabinet, counter, sofa, bed, chair, rack |
| Object | Point or polygon objects such as chargers, monitors, tools, boxes, doors |
| Nav2 start pose | Initial pose candidate with position and yaw |
| Waypoint | Navigation waypoint with position and yaw |
| Semantic goal | Named goal pose, optionally linked to a room, carrier, or object |

`semantic_map.json` can be loaded back into the editor, modified, and saved again.
When pixel coordinates exist in the JSON, the editor uses them directly.
When only world coordinates exist, the editor projects them through the map `origin`, `resolution`, and image height.

### Markdown Catalogs

Use the `MD list` button to import Markdown files that define rooms, locations, and known objects.
The editor parses these sections automatically:

```markdown
## Rooms
| Name |
| ------------ |
| kitchen |
| living room |
| bedroom |
| laundry room |
```

```markdown
## Locations
| Number | Name | Object Category |
| ------------ | ----------- | ----------- |
| 1 | fridge (p) |
| 2 | kitchen counter (p) | dishes |
| 6 | cabinet (p) | drinks |
```

`(p)` means objects can be placed at that location.
Imported locations become carrier/location choices in the semantic dialog.
The object category column is saved to semantic JSON as `object_category`.

Known-object files are parsed from sections like this:

```markdown
# Class drinks (drink)

| Objectname | Image |
:-----------:|:-----:
| cola | ![](known_objects/drinks!drink/cola.jpg) |
| water | ![](known_objects/drinks!drink/water.jpg) |
```

Imported objects become object choices in the semantic object dialog.
Their class/type/image metadata is preserved in semantic JSON.

## ROS2 Bridge

The UI communicates with ROS2 through rosbridge.

Install rosbridge:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
```

Run rosbridge manually:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

Or, in Electron mode, use the `Bridge` button in the top ROS2 control row.
It runs:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

Then connect the ROS2 panel to:

```text
ws://localhost:9090
```

## ROS2 Topic Visualization

Open the ROS2 panel and add displays from the topic browser.

Supported topic types include:

| Data | ROS2 message type |
|---|---|
| 2D lidar | `sensor_msgs/msg/LaserScan` |
| 3D lidar | `sensor_msgs/msg/PointCloud2` |
| Odometry | `nav_msgs/msg/Odometry` |
| Pose | `geometry_msgs/msg/PoseStamped` |
| AMCL pose | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| Path | `nav_msgs/msg/Path` |
| TF | `tf2_msgs/msg/TFMessage` |
| Camera image | `sensor_msgs/msg/Image` |
| Compressed camera image | `sensor_msgs/msg/CompressedImage` |
| Depth image | `sensor_msgs/msg/Image` with depth encodings |

The overlay uses message `header.frame_id` and the TF tree to transform data into the selected fixed frame, similar to RViz.
Use `map` as the fixed frame for most Nav2 workflows.

## ROS2 Bag Playback

Electron mode can control rosbag playback directly from the UI:

- Choose a bag directory
- Play and stop
- Pause and resume
- Seek backward and forward
- Drag the timeline slider
- Adjust playback rate
- Enable loop playback
- Publish `/clock`

The underlying command is:

```bash
ros2 bag play <bag_path> --clock
```

with optional flags such as:

```bash
--loop
--rate <rate>
--start-offset <seconds>
```

When a bag reaches the end, the editor resets the playback offset so pressing play again starts from the beginning.

## Capturing Robot Pose

After connecting ROS2 or playing a bag, the current robot pose can be captured into semantic data.

Top ROS2 controls:

| Button | Result |
|---|---|
| `current -> start` | Set Nav2 start pose from current robot pose |
| `current -> WP` | Add a waypoint from current robot pose |
| `current -> goal` | Add a semantic goal from current robot pose |
| `send initial pose` | Publish the selected start pose to `/initialpose` |

The start-pose publisher uses:

```text
geometry_msgs/msg/PoseWithCovarianceStamped
```

on:

```text
/initialpose
```

## Keyboard Shortcuts

| Key | Action |
|---|---|
| `B` | Brush |
| `E` | Eraser |
| `L` | Line |
| `R` | Rectangle |
| `C` | Circle |
| `F` | Fill |
| `S` | Nav2 start pose |
| `W` | Waypoint |
| `1` | Room rectangle |
| `2` | Room polygon |
| `3` | Carrier rectangle |
| `4` | Carrier polygon |
| `5` | Object rectangle |
| `6` | Object polygon |
| `7` | Point object |
| `8` or `0` | Select and move |
| `9` | Semantic goal |
| `Ctrl+Z` | Undo |
| `Ctrl+Y` | Redo |
| `Backspace` or `Delete` | Delete selected item |
| `[` / `]` | Rotate view |

## Exported Files

Full save exports:

```text
map_name.pgm
map_name.yaml
map_name_semantic.json
```

The semantic JSON includes:

- `metadata`
- `start_pose`
- `maps`
- `rooms`
- `carriers`
- `objects`
- `waypoints`
- `goals`
- `_pixel` helper fields for accurate round-trip editing

Example:

```json
{
  "metadata": {
    "resolution": 0.05,
    "origin": [-10, -10, 0],
    "image_size": { "w": 400, "h": 400 }
  },
  "start_pose": {
    "label": "Nav2 start",
    "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
    "theta_rad": 0.0,
    "_pixel": { "x": 200, "y": 200 }
  },
  "rooms": [],
  "carriers": [],
  "objects": [],
  "waypoints": [],
  "goals": []
}
```

## Included ROS2 Package

The repository includes an optional ROS2 package:

```text
src/semantic_nav2_package/pkg/semantic_nav2
```

It contains:

- `semantic_map_server.py`: loads semantic JSON and publishes/query semantic map data
- `waypoint_navigator.py`: loads waypoint data and sends Nav2 FollowWaypoints goals
- `semantic_costmap_layer.py`: experimental semantic costmap layer
- Launch files and Nav2 parameter examples

Install it into a ROS2 workspace:

```bash
cp -r src/semantic_nav2_package/pkg/semantic_nav2 ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select semantic_nav2
source install/setup.bash
```

See:

```text
src/semantic_nav2_package/pkg/semantic_nav2/README.md
```

for package-specific usage.

## Project Structure

```text
map_ui/
├── electron/
│   ├── main.cjs
│   └── preload.cjs
├── src/
│   ├── main.jsx
│   ├── Nav2MapEditor.jsx
│   ├── ros2/
│   │   ├── Ros2Bridge.js
│   │   ├── Ros2Panel.jsx
│   │   ├── Ros2View3D.jsx
│   │   └── useRos2Overlay.js
│   └── semantic_nav2_package/
├── index.html
├── package.json
├── package-lock.json
└── vite.config.js
```

## Notes

- Browser mode cannot launch OS processes, so rosbridge and rosbag controls are Electron-only.
- Browser mode can still connect to an already-running rosbridge server.
- For bag playback with TF and sensor data, include `/clock`, `/tf`, and `/tf_static` in the bag.
- If pose markers remain visible, check whether TF display is enabled. Pose displays and TF displays are controlled independently.
- If world coordinates look shifted, verify that the loaded `YAML` origin and resolution match the `PGM`.

## License

MIT
