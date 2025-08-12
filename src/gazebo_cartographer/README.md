# gazebo_cartographer 使用说明

## 功能说明
本包整合了Cartographer的建图（SLAM）和纯定位（Localization-only）功能，通过单一launch文件支持两种模式。

## 1. 建图模式（默认）
```bash
ros2 launch gazebo_cartographer cartographer.launch.py
```

## 2. 纯定位模式
```bash
ros2 launch gazebo_cartographer cartographer.launch.py \
  pure_localization:=true \
  pbstream_filename:=/path/to/your/map.pbstream
```

## 3. 参数说明

### 基础参数
- `use_sim_time`: 是否使用仿真时间（默认: true）
- `resolution`: 占据栅格分辨率（默认: 0.05）
- `publish_period_sec`: 占据栅格发布周期（默认: 1.0）
- `scan_topic`: 激光扫描话题（默认: scan）

### 纯定位参数
- `pure_localization`: 启用纯定位模式（默认: false）
- `pbstream_filename`: .pbstream地图文件路径（默认: /home/hzy/study/gazebo_ws/src/all_launch/maps/map.pbstream）

### 配置文件
- `configuration_basename`: lua配置文件（自动选择）
  - 建图模式: cartographer_2d.lua
  - 纯定位模式: cartogrpher_localization.lua

## 4. 使用示例

### 建图
```bash
# 启动建图
ros2 launch gazebo_cartographer cartographer.launch.py use_sim_time:=true

# 建图完成后保存地图
ros2 run cartographer_ros cartographer_assets_writer -f /path/to/save/map_name -p /path/to/your/bag_file.bag
```

### 纯定位
```bash
# 使用默认pbstream地图
ros2 launch gazebo_cartographer cartographer.launch.py pure_localization:=true

# 指定pbstream地图路径
ros2 launch gazebo_cartographer cartographer.launch.py \
  pure_localization:=true \
  pbstream_filename:=/home/hzy/study/gazebo_ws/src/all_launch/maps/map.pbstream \
  scan_topic:=scan
```

## 5. 注意事项
- 纯定位需要.pbstream格式地图，不能直接使用.pgm/.yaml
- 确保tracking_frame等TF配置与实际机器人一致
- 如果激光话题不是scan，请使用scan_topic参数重映射

## 6. 配置文件
- `config/cartographer_2d.lua`: 建图配置
- `config/cartogrpher_localization.lua`: 纯定位配置（基于建图配置，启用pure_localization_trimmer）