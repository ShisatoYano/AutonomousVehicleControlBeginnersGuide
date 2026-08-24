"""
  generate_examples_gallery.py

  src/simulations配下を走査し、各アルゴリズムのデモ画像(gif/png)を集めて
  doc/EXAMPLES.mdを自動生成する。

  これまでREADME.mdの目次・本文に手動でアルゴリズムを追記していたため、
  新しいシミュレーションを追加しても掲載が漏れることがあった
  (gradient_path_planning, lidar_obstacle_sensing, point_cloud_search等)。
  このスクリプトはディレクトリと画像ファイルの存在だけを根拠に一覧を
  組み立てるため、掲載漏れが起きない。

  新しいシミュレーションを追加した、またはデモ画像を追加/変更したときは、
  このスクリプトを再実行してdoc/EXAMPLES.mdを更新すること。
  再実行を忘れた場合はtest/test_examples_gallery_up_to_date.pyが失敗し、
  CI(Linux/Windows/macOS)で検知される。

  Author: Shisato Yano
  """
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
SIMULATIONS_DIR = PROJECT_ROOT / "src" / "simulations"
OUTPUT_PATH = PROJECT_ROOT / "doc" / "EXAMPLES.md"

IMAGE_EXTENSIONS = (".gif", ".png", ".jpg")

# 表示順とカテゴリ名。ここに無いディレクトリ名(src/simulations直下)はスキップする
CATEGORY_TITLES = {
    "localization": "Localization",
    "mapping": "Mapping",
    "path_planning": "Path Planning",
    "path_tracking": "Path Tracking",
    "perception": "Perception",
    "course": "Course",
}

# ディレクトリ名 -> 表示名。無いものは_default_title()で自動生成した名前になる
# (見た目は少し落ちるが、掲載自体は漏れない)
DISPLAY_NAMES = {
    "extended_kalman_filter_localization": "Extended Kalman Filter Localization",
    "unscented_kalman_filter_localization": "Unscented Kalman Filter Localization",
    "particle_filter_localization": "Particle Filter Localization",
    "ekf_vs_ukf_comparison": "EKF vs UKF Comparison",
    "ekf_ukf_pf_comparison": "EKF vs UKF vs Particle Filter Comparison",
    "binary_grid_map_construction": "Binary Occupancy Grid Map",
    "cost_grid_map_construction": "Cost Map",
    "potential_field_map_construction": "Potential Field Map",
    "ndt_map_construction": "NDT Map",
    "astar_path_planning": "A*",
    "astar_bidirectional_path_planning": "Bidirectional A*",
    "astar_hybrid_path_planning": "Hybrid A*",
    "dstar_path_planning": "D*",
    "dijkstra_path_planning": "Dijkstra",
    "aco_path_planning": "ACO",
    "q_learning_path_planning": "Q-Learning",
    "pso_path_planning": "PSO",
    "prm_path_planning": "PRM",
    "elastic_bands_path_planning": "Elastic Bands",
    "gradient_path_planning": "Gradient Descent",
    "rrt_path_planning": "RRT",
    "rrt_star_bidirectional_path_planning": "Bidirectional RRT*",
    "rrt_star_path_planning": "RRT*",
    "informed_rrt_star_path_planning": "Informed RRT*",
    "pure_pursuit_path_tracking": "Pure pursuit Path Tracking",
    "adaptive_pure_pursuit_path_tracking": "Adaptive Pure pursuit Path Tracking",
    "rear_wheel_feedback_tracking": "Rear wheel feedback Path Tracking",
    "lqr_path_tracking": "LQR(Linear Quadratic Regulator) Path Tracking",
    "stanley_path_tracking": "Stanley steering control Path Tracking",
    "mppi_path_tracking": "MPPI Path Tracking",
    "mpc_path_tracking": "MPC Path Tracking",
    "point_cloud_rectangle_fitting": "Rectangle fitting Detection",
    "sensor_auto_calibration": "Sensor's Extrinsic Parameters Estimation",
    "lidar_obstacle_sensing": "LiDAR Obstacle Sensing",
    "point_cloud_search": "Point Cloud Nearest Neighbor Search (kd-tree)",
    "cubic_spline": "Cubic Spline Course",
}

# 見出しの下に添える一言(旧READMEの本文にあった補足やクレジット表記の移植先)
DESCRIPTIONS = {
    "aco_path_planning": "Ant Colony Optimization  \nAuthor: [Banaan Kiamanesh](https://github.com/BanaanKiamanesh)",
    "q_learning_path_planning": "Reinforcement learning with a Q-table policy",
    "pso_path_planning": "Particle Swarm Optimization",
    "dstar_path_planning": "Planning with dynamic obstacle replanning",
    "dijkstra_path_planning": "Planning (reduce frames by sampling every nth node to prevent memory exhaustion)",
    "elastic_bands_path_planning": "A* seed path smoothed with Elastic Bands optimisation",
    "sensor_auto_calibration": "Estimation by Unscented Kalman Filter",
}

