include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.
}

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.miss_probability = 0.45

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2
SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 5e2
SPARSE_POSE_GRAPH.optimize_every_n_scans = 10
SPARSE_POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
SPARSE_POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.62

return options
