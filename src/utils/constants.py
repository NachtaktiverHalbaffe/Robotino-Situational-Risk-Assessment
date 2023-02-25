import os
from enum import Enum
from geometry_msgs.msg import PoseStamped


class Topics(Enum):
    ACML = "/amcl_pose"
    ANOMALY_DETECTED = "/anomaly_detected"
    BRUTEFORCE_ENABLED = "/bruteforce_enabled"
    EMERGENCY_BRAKE = "/emergency_break"
    GLOBAL_PATH = "/path_global"
    IMAGE_BB_MOVEABLE = "/image_bb_moveable"
    IMAGE_BB_WS = "/image_bb_ws"
    IMAGE_RAW = "/image_raw"
    IMG_LOCALIZATION = "/camera_pose"
    IR_SENSORS = "/distance_sensors"
    LIDAR_BREAKDOWN = "/lidar_breakdown"
    LOCALIZATION = "/pose"
    FALLBACK_POSE = "/pose_fallback"
    MARKERS = "/target_markers"
    MARKERS_COMMON_TRAJ = "/common_traj_markers"
    MARKERS_NAV = "/target_nav_markers"
    MOVE_BASE_FEEDBACK = "/move_base/feedback"
    MOVE_BASE_RESULT = "/move_base/result"
    NAV_POINT = "/nav_point"
    NAVIGATION_RESPONSE = "/navigation_response"
    NR_OF_RUNS = "/nr_of_runs"
    OBSTACLES = "/obstacles"
    OBSTACLES_GEOFENCED = "/obstacles_geofenced"
    OBSTACLE_MARGIN = "/obstacle_margin"
    OBSTACLES_VISU = "/obstacles_visu"
    ODOM = "/odom"
    ODOM_ROBOTINO = "/odom_robotino"
    PATH_ERRORDIST_DIST = "/path_errordistr_dist"
    PATH_ERRORDISTR_ANGLE = "/path_errordistr_angle"
    RISK_ESTIMATION_BRUTE = "/risk_estimation_brute"
    RISK_ESTIMATION_RL = "/risk_estimation_rl"
    RISK_ESTIMATION_SOTA = "/risk_estimation_sota"
    SOTA_ENABLED = "/sota_enabled"
    STRATEGY_PLANNER_RESPONSE = "/strategy_planner_response"
    TARGET = "/target"
    TARGET_ID = "/target_id"
    USE_ERRORDIST = "/custom_errordist_enabled"
    WORKSTATIONMAPPER_ENABLED = "/workstation_mapper_enabled"


class Nodes(Enum):
    PATH_PLANNER = "path_planner"
    WORKSTATION_MAPPER = "workstation_mapper"
    LOCALIZATION = "localization"
    OBJECT_DETECTION = "object_detection"
    CONTROL = "control"
    STRATEGY_PLANNER = "strategy_planner"
    FLEETIAS_CLIENT = "fleetias_client"
    GUI = "gui"
    RISK_ESTIMATOR = "risk_estimator"
    MOVE_BASE = "move_base"
    MONITORED_SPACE_OBSERVER = "monitored_space_observer"


class Metrics(Enum):
    COLLISIONS = "collisions"
    PRODUCTION_STOPS = "prodStop"
    RISK = "risk"
    STARTED_TASKS = "startedTasks"
    SUCCESSFUL_TASKS = "successfulTasks"
    COLLISION_PROBABILITY = "collisionProbability"


class LoggingLevel(Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    EVAL = "EVAL"


class CommonPositions(Enum):
    __safe_Spot = PoseStamped()
    __safe_Spot.header.frame_id = "map"
    __safe_Spot.pose.position.x = -4.093
    __safe_Spot.pose.position.y = 2.422
    SAFE_SPOT = __safe_Spot


PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))


class Paths(Enum):
    ERRORDIST_DIST = f"{PATH}/logs/error_dist_csvs/localization_error_dist.csv"
    ERRORDIST_ANGLE = f"{PATH}/logs/error_dist_csvs/localization_error_angle.csv"
    RISK_ESTIMATION_OBSTACLES = f"{PATH}/logs/risk_estimation_obstacles.csv"
    RISK_ESTIMATION = f"{PATH}/logs/risk_estimation.csv"
