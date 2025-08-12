-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- -- 纯定位配置文件
-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
--   map_frame = "map",
--   tracking_frame = "velodyne_base_link",  -- 根据实际机器人修改
--   published_frame = "odom",
--   odom_frame = "odom",
--   provide_odom_frame = false,
--   publish_frame_projected_to_2d = false,
--   use_pose_extrapolator = false,
--   use_odometry = false,
--   use_nav_sat = false,
--   use_landmarks = false,
--   num_laser_scans = 1,
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 1,
--   num_point_clouds = 0,
--   lookup_transform_timeout_sec = 0.2,
--   submap_publish_period_sec = 1.0,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 1.,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,
-- }

-- MAP_BUILDER.use_trajectory_builder_2d = true
-- -- 纯定位建议参数
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
-- TRAJECTORY_BUILDER_2D.min_range = 0.1
-- TRAJECTORY_BUILDER_2D.max_range = 12.0
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
-- TRAJECTORY_BUILDER_2D.use_imu_data = false
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- -- 关闭全局优化，纯定位只做局部匹配
-- POSE_GRAPH.optimize_every_n_nodes = 0
-- POSE_GRAPH.constraint_builder.min_score = 0.65
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- -- 纯定位轨迹修剪器，防止轨迹无限增长
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 10.

-- return options


include "cartographer_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20
 
return options