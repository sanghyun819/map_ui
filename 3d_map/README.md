# 3d_map — Livox 높이 맵을 2D occupancy grid 위에 올리기

slam_toolbox로 만든 2D occupancy grid 위에, 같은 bag에 들어있는 Livox lidar
포인트를 쌓아 **2.5D 높이 맵(elevation map) + 병합 포인트클라우드**를 만드는
독립 파이프라인입니다. map_ui 앱과는 분리되어 있고, **어떤 맵·어떤 bag에도**
동작하도록 경로/토픽/프레임을 인자로 받습니다.

## 원리

1. slam_toolbox는 `map` 프레임에서 2D occupancy grid를 줍니다.
2. 같은 bag 안에 `/tf`(+`/tf_static`)가 있어 매 스캔 시점의 `map_T_lidar`를 알 수 있습니다.
3. 모든 Livox 스캔을 `map` 프레임으로 변환해 누적합니다.
4. occupancy grid와 **픽셀 단위로 정렬되는** 격자에 셀별 높이(기본 최댓값)를 래스터화합니다.

ROS를 source 할 필요 없이 pip 패키지만으로 bag을 직접 읽습니다(`rosbags`).

## 설치

```bash
cd 3d_map
python -m venv .venv && source .venv/bin/activate   # 선택
pip install -r requirements.txt
```

`open3d`는 `.pcd` 저장과 3D 뷰어에만 쓰입니다. 설치가 어려우면 빼도 되고,
그 경우 포인트클라우드는 `.xyz`로 저장됩니다.

## 사용법

> **중요 — `/tf_static`이 없는 bag:** rosbag을 녹화할 때 `/tf_static`(latched)을
> 놓치면 lidar·카메라 장착 변환이 TF 트리에 안 들어와 센서 프레임이 끊깁니다.
> 이때는 로봇 URDF를 `--urdf`로 넘기면 URDF의 fixed joint를 static TF로 주입해
> 체인을 복구합니다. (이 저장소엔 rby1용 [`rby1.urdf`](rby1.urdf) 포함 — 검증 완료)

```bash
# 이 저장소에서 검증된 명령 (rby1 로봇, livox)
python build_height_map.py --bag /home/sh/bag/bag_sh --map ../map_0430.yaml \
    --urdf rby1.urdf --z-min 0.0 --z-max 2.5 --range-max 40 --voxel 0.05 --stride 2

# 가장 기본: bag + 2D 맵 yaml (tf_static이 bag에 있으면 --urdf 불필요)
python build_height_map.py --bag /path/to/rosbag2_dir --map ../map_0430.yaml

# 토픽/프레임 수동 지정 (보통 자동 감지됨)
python build_height_map.py --bag BAG --map MAP.yaml \
    --lidar-topic /livox/lidar --map-frame map

# 천장/바닥 제거, 센서 반경 제한, 다운샘플 (대용량 bag 권장)
python build_height_map.py --bag BAG --map MAP.yaml \
    --z-min 0.0 --z-max 2.5 --range-max 30 --voxel 0.03 --stride 2

# 맵 없이 클라우드 경계로 자동 격자 생성
python build_height_map.py --bag BAG --resolution 0.05
```

### 주요 옵션

| 옵션 | 설명 |
|------|------|
| `--bag` | rosbag2 디렉터리 또는 `.mcap`/`.db3` (필수) |
| `--map` | Nav2 맵 `.yaml`. 주면 출력이 2D 격자에 정렬됨 |
| `--lidar-topic` | PointCloud2/Livox 토픽 (미지정 시 자동 감지) |
| `--map-frame` | 목표/월드 프레임 (기본 `map`) |
| `--urdf` | 로봇 URDF. bag에 `/tf_static`이 없을 때 fixed joint를 static TF로 주입 |
| `--ros-distro` | 타입 정의 없는 bag용 typestore (기본 `humble`) |
| `--stat` | 셀당 높이 통계 `max`/`mean`/`min`/`median` (기본 `max`) |
| `--z-min/--z-max` | map 프레임 z로 포인트 필터 (바닥/천장 제거) |
| `--range-max` | 센서로부터 이 거리[m] 초과 포인트 제거 |
| `--voxel` | 병합 클라우드 복셀 다운샘플 leaf[m] |
| `--stride` | N번째 스캔만 사용 (속도/용량 절충) |
| `--icp` | 각 스캔을 누적 맵에 point-to-point ICP로 정합(SLAM drift 보정). `--icp-voxel/--icp-max-dist/--icp-iters`로 조정 |
| `--min-hits N` | 관측 N회 미만 복셀 제거(`--dyn-voxel` 단위) — 따라다니는 사람 등 일시적 포인트를 *대충* 솎아냄 |
| `--save-raw` | 필터 전 누적 원본 클라우드를 `raw_cloud.npy`로 저장(후처리/튜닝용) |
| `--footprint` | 로봇 풋프린트 폴리곤(`"[[x,y],...]"`) 안의 점 제거 — **로봇이 자기 lidar에 찍히는 자기-점 삭제** |
| `--footprint-frame` | 풋프린트 정의 프레임 (기본 `base_nav`) |
| `--footprint-margin` | 풋프린트를 바깥으로 N m 키워서 제거 (기본 0) |

### 로봇 자기-점 제거 예시 (rby1)

```bash
python build_height_map.py --bag BAG --map MAP.yaml --urdf rby1.urdf \
  --footprint "[[0.097,-0.30],[0.097,0.30],[-0.260,0.30],[-0.563,0.15],[-0.563,-0.15],[-0.260,-0.30]]" \
  --footprint-frame base_nav --range-max 40 --voxel 0.05
```

각 스캔을 풋프린트 프레임(`base_nav`)으로 변환해 폴리곤 안 점을 버립니다. 로봇은 늘
풋프린트 안에 있으므로 자기-점만 영구 제거되고, 그 자리의 바닥/천장은 로봇이 다른 위치에
있을 때 본 스캔으로 채워집니다(누적이라 구멍 안 남음). rby1 실측: **자기-점의 ~44% 제거**.

> **ICP vs 동적 제거 (중요):** `--icp`는 *정합*만 합니다 — 움직이는 사람은 **안 지웁니다**(오히려 정합을 방해할 수 있어 max-dist로 outlier 배제). 따라다니는 사람을 솎으려면 `--min-hits`를 쓰세요. 다만 이건 무딘 도구라 멀리서 드물게 보인 정적 구조(벽 끝 등)도 같이 지워질 수 있습니다. 정밀 제거가 필요하면 `--save-raw`로 받아서 직접 후처리하세요.

## 출력 (`out/`)

| 파일 | 내용 |
|------|------|
| `cloud_map.pcd` | map 프레임 병합 포인트클라우드 (높이 컬러) — open3d 없으면 `.xyz` |
| `elevation.npy` | `HxW` float32 높이 격자, 데이터 없는 셀은 NaN |
| `elevation.tif` | 같은 격자를 32-bit TIFF로 |
| `elevation_color.png` | 빠른 확인용 컬러 높이 맵 (RGBA, 빈 셀 투명) |
| `elevation_meta.json` | resolution/origin/size + 높이 범위 |
| `height_view3d.json` | map_ui 3D 뷰용 컬러 높이 포인트 (셀별, map 프레임 월드 좌표) |

## map_ui 3D 뷰에 올리기

map_ui의 3D 패널(우측 3D 뷰)에서 **`Height ⬆`** 버튼을 눌러 `out/height_view3d.json`을
로드하면, 2D 맵 평면 위에 높이 컬러 포인트가 겹쳐 표시됩니다.
- `◉ N pts` 버튼: 표시/숨김 토글
- `z×` 슬라이더: 높이 과장 배율(0.5~5)
좌표는 맵과 동일한 origin/resolution(map 프레임)이라 별도 정렬 없이 바로 겹쳐집니다.

`elevation.npy`는 occupancy grid와 동일한 resolution/origin/size를 가지므로,
나중에 map_ui나 RViz/Three.js에서 2D 맵 위에 그대로 겹쳐 올릴 수 있습니다.

## 확인

```bash
python visualize.py out/cloud_map.pcd      # 3D 포인트클라우드
python visualize.py out/elevation.npy      # 2.5D surface
```

## 한계 / 참고

- TF는 **가장 가까운 시각**으로 조회합니다(선형 보간 아님). 실내 저속 매핑엔 충분.
- Livox 정확도는 slam_toolbox 위치추정 품질에 좌우됩니다. `map->odom`이 흔들리면
  높이 맵도 흔들립니다. 가능하면 루프 클로징이 잘 된 bag을 쓰세요.
- `/livox/lidar`가 `sensor_msgs/PointCloud2`면 바로 처리됩니다. Livox 커스텀
  타입(`CustomMsg`)도 처리하지만, bag에 메시지 정의가 등록돼 있어야 deserialize됩니다.
- 정밀한 3D 복원/정합이 필요하면 이 결과를 초기값으로 두고 `octomap_server`,
  `grid_map`/`elevation_mapping`(ANYbotics) 같은 전용 패키지로 확장하는 걸 권장.
