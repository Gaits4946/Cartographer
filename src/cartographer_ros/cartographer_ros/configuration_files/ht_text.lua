-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

-- HT: options中定义的参数是 include文件中不包含的参数
options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  
  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "imu_link",              -- 将所有传感器数据转换到这个坐标系下(HT: 有imu的link就写成imu_link, 没有就写成base_link)
                                            -- cartographer在处理数据是会将所有的传感器坐标变换到tracking_frame下,一般imu频率最高(100-400hz),lidar一般比较慢(10-20hz)
  published_frame = "footprint",            -- tf: map -> footprint(HT:bag 自己tf最上面的左边系的名字)
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = false,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
                                            -- 如果为false tf树为map->footprint
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上(HT: 作用不大)
  --use_pose_extrapolator = false,            -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿
                                              -- (HT: 默认是false,一定要设为false)

--HT: 下面3者配置通过Topic的方式订阅, true:订阅(记得在launch中订阅相关topic), false:不订阅. 各自唯一.
  use_odometry = false,                     -- 是否使用里程计,如果使用要求一定要有odom的tf
  use_nav_sat = false,                      -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark

--HT: lidar Topic订阅配置, true:订阅(记得在launch中订阅相关topic), false:不订阅. lidar可以订阅多个.
--单线lidar/多回声波lidar/16线lidar不能都为0.
  num_laser_scans = 0,                      -- 是否使用单线激光数据 (HT: 单线lidar,可以改为2 或大于2)
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据 (HT: 多回声波lidar, 可以改为2 或大于2)
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1 (HT: 一帧传感器数据分几帧处理, cartographer是10)
  num_point_clouds = 1,                     -- 是否使用点云数据 (HT: 16线lidar,可以改为2 或大于2)
  
  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,          -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

--HT: 传感器设置采样率
  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- cartographer常用参数的赋值
MAP_BUILDER.use_trajectory_builder_2d = true --重新赋值MAP_BUILDER中的参数use_trajectory_builder_2d

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 100.
TRAJECTORY_BUILDER_2D.min_z = 0.2
--TRAJECTORY_BUILDER_2D.max_z = 1.4
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02

--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200.
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.

--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12

--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004
--TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1

POSE_GRAPH.optimize_every_n_nodes = 160. -- (HT: num_range_data * 2 后端优化, 一般是2倍子图)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.48
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options
