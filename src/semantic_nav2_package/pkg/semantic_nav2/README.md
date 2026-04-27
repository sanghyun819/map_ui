# semantic_nav2

ROS2 패키지 – **Nav2 Map Editor** 웹 툴 출력물을 Nav2에서 바로 사용하기 위한 패키지.

---

## 파일 흐름

```
[웹 에디터 출력]                    [이 패키지]
─────────────────────────────────────────────────────────
map.pgm          ──► Nav2 map_server (기존과 동일)
map.yaml         ──► Nav2 map_server
semantic_map.json ──► semantic_map_server  (방·객체 마커 + 쿼리 서비스)
waypoints.yaml   ──► waypoint_navigator   (Nav2 FollowWaypoints 액션 구동)
```

---

## 빌드

```bash
# workspace 안에 패키지 복사
cp -r /home/sh/map_ui/src/semantic_nav2_package/pkg/semantic_nav2 ~/ros2_ws/src/

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select semantic_nav2
source install/setup.bash
```

---

## 실행

### 한 번에 전부 실행 (권장)

```bash
ros2 launch semantic_nav2 semantic_nav2.launch.py \
  map_yaml:=$HOME/maps/map.yaml \
  semantic_map:=$HOME/maps/semantic_map.json \
  waypoints:=$HOME/maps/waypoints.yaml
```

| 인수 | 기본값 | 설명 |
|---|---|---|
| `map_yaml` | `~/maps/map.yaml` | PGM 맵의 YAML 경로 |
| `semantic_map` | `~/maps/semantic_map.json` | 에디터에서 내보낸 JSON |
| `waypoints` | `~/maps/waypoints.yaml` | 에디터에서 내보낸 웨이포인트 |
| `nav2_params` | 패키지 내 기본값 | Nav2 파라미터 YAML |
| `auto_start_nav` | `false` | `true`이면 즉시 웨이포인트 주행 시작 |
| `loop_waypoints` | `false` | `true`이면 반복 주행 |
| `add_semantic_costmap` | `false` | 객체 영역을 추가 장애물로 costmap에 반영 |
| `inflate_types` | `"obstacle"` | costmap에 추가할 객체 타입 (쉼표 구분) |

---

### 개별 실행

```bash
# 1. semantic_map_server만 실행
ros2 run semantic_nav2 semantic_map_server.py \
  --ros-args -p map_file:=/path/to/semantic_map.json

# 2. waypoint_navigator만 실행
ros2 run semantic_nav2 waypoint_navigator.py \
  --ros-args \
  -p waypoints_file:=/path/to/waypoints.yaml \
  -p auto_start:=false

# 3. semantic costmap layer
ros2 run semantic_nav2 semantic_costmap_layer.py \
  --ros-args \
  -p semantic_map_file:=/path/to/semantic_map.json \
  -p base_map_yaml:=/path/to/map.yaml \
  -p inflate_types:="obstacle,cabinet"
```

---

### TurtleBot3 House + Gazebo 실험

```bash
# 0) TurtleBot3 모델 설정
export TURTLEBOT3_MODEL=burger

# 1) Gazebo House + Nav2 + semantic 노드 동시 실행
ros2 launch semantic_nav2 tb3_house_semantic_nav2.launch.py \
  map_yaml:=$HOME/maps/map.yaml \
  semantic_map:=$HOME/maps/semantic_map.json \
  waypoints:=$HOME/maps/waypoints.yaml \
  use_sim_time:=true
```

> `tb3_house_semantic_nav2.launch.py`는 아래를 함께 실행합니다.
> - `turtlebot3_gazebo/launch/turtlebot3_house.launch.py`
> - `semantic_nav2/launch/semantic_nav2.launch.py`

---

## 웨이포인트 주행 제어 (서비스 호출)

```bash
# 주행 시작
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger

# 주행 중지
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger

# 파일 핫 리로드
ros2 service call /waypoint_navigator/reload std_srvs/srv/Trigger
```

---

## 시맨틱 쿼리 서비스

### 좌표 → 방 이름

```bash
ros2 service call /semantic_map/query_room \
  semantic_nav2/srv/QueryRoom \
  "{point: {x: 2.5, y: 1.0, z: 0.0}}"
```

응답:
```yaml
found: true
room:
  id: "s3"
  type: "office"
  label: "연구실 1"
  centroid: {x: 2.8, y: 1.2, z: 0.0}
  area_m2: 12.4
```

### 방 안의 객체 목록

```bash
ros2 service call /semantic_map/query_objects_in_room \
  semantic_nav2/srv/QueryObjectsInRoom \
  "{room_id: 's3', object_type: 'desk'}"
```

### 가장 가까운 객체 찾기

```bash
ros2 service call /semantic_map/query_nearest \
  semantic_nav2/srv/QueryNearestObject \
  "{point: {x: 1.0, y: 2.0, z: 0.0}, object_type: 'charger', max_radius_m: 5.0}"
```

---

## Python에서 시맨틱 쿼리 사용 예시

```python
import rclpy
from rclpy.node import Node
from semantic_nav2.srv import QueryRoom, QueryNearestObject
from geometry_msgs.msg import Point

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot')
        self._qr  = self.create_client(QueryRoom,          '/semantic_map/query_room')
        self._qno = self.create_client(QueryNearestObject, '/semantic_map/query_nearest')

    def what_room_am_i_in(self, x: float, y: float) -> str:
        req = QueryRoom.Request()
        req.point = Point(x=x, y=y, z=0.0)
        future = self._qr.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.found:
            return f"{res.room.label} ({res.room.type})"
        return "Unknown"

    def find_nearest_charger(self, x: float, y: float):
        req = QueryNearestObject.Request()
        req.point = Point(x=x, y=y, z=0.0)
        req.object_type = "charger"
        req.max_radius_m = 20.0
        future = self._qno.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.found:
            pos = res.object.world_pos
            return pos.x, pos.y, res.distance_m
        return None
```

---

## 퍼블리시되는 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/semantic_map/rooms_markers` | `visualization_msgs/MarkerArray` | RViz 방 오버레이 (latched) |
| `/semantic_map/objects_markers` | `visualization_msgs/MarkerArray` | RViz 객체 마커 (latched) |
| `/waypoint_navigator/markers` | `visualization_msgs/MarkerArray` | RViz 웨이포인트 화살표 (latched) |
| `/waypoint_navigator/status` | `std_msgs/String` | 현재 주행 상태 |
| `/semantic_costmap` | `nav_msgs/OccupancyGrid` | 객체 장애물 레이어 (latched, 선택) |

---

## RViz 설정

1. **MarkerArray** 디스플레이 추가 → 토픽: `/semantic_map/rooms_markers`
2. **MarkerArray** 디스플레이 추가 → 토픽: `/semantic_map/objects_markers`
3. **MarkerArray** 디스플레이 추가 → 토픽: `/waypoint_navigator/markers`
4. (선택) **Map** 디스플레이 추가 → 토픽: `/semantic_costmap`

---

## semantic_map.json 포맷 (에디터 출력)

```json
{
  "metadata": {
    "resolution": 0.05,
    "origin": [-10, -10, 0],
    "created": "2025-03-14T12:00:00"
  },
  "rooms": [
    {
      "id": "s1",
      "type": "office",
      "label": "연구실",
      "pixel_rect": {"x": 100, "y": 80, "w": 120, "h": 90},
      "world_rect": {"x": -5.0, "y": -2.0, "w": 6.0, "h": 4.5}
    }
  ],
  "objects": [
    {
      "id": "s2",
      "type": "desk",
      "label": "책상1",
      "room_id": "s1",
      "pixel_rect": {"x": 140, "y": 120, "w": 24, "h": 18},
      "world_rect": {"x": -3.0, "y": -3.2, "w": 1.2, "h": 0.9}
    },
    {
      "id": "s3",
      "type": "charger",
      "label": "충전소",
      "room_id": "s1",
      "is_point": true,
      "pixel_pos": {"x": 150, "y": 120},
      "world_pos": {"x": -2.5, "y": -4.0}
    }
  ]
}
```

`semantic_nav2`는 `world_rect`와 `world_polygon`을 모두 지원합니다.

## waypoints.yaml 포맷 (에디터 출력)

```yaml
waypoints:
  - id: 1
    label: "WP1"
    x: -3.2
    y: -1.5
    theta: 1.57
  - id: 2
    label: "충전소 앞"
    x: -2.5
    y: -4.0
    theta: 0.0
```
