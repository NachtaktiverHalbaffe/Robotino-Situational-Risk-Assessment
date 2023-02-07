from enum import Enum


class Topics(Enum):
    GLOBAL_PATH = "/path_global"
    TARGET = "/target"
    TARGET_ID = "/target_id"
    NAV_POINT = "/nav_point"
    ACML = "/amcl_pose"
    IMG_LOCALIZATION = "/camera_pose"
    LOCALIZATION_MODE = "/localization_mode"
    LOCALIZATION = "/pose"
    OBSTACLES = "/obstacles"
    OBSTACLES_VISU = "/obstacles_visu"
    IMAGE_RAW = "/image_raw"
    IMAGE_BB_WS = "/image_bb_ws"
    IMAGE_BB_MOVEABLE = "/image_bb_moveable"
    NAVIGATION_RESPONSE = "/navigation_response"
    ODOM = "/odom"
    IR_SENSORS = "/distance_sensors"
    EMERGENCY_BRAKE = "/emergency_break"
    MARKERS = "/target_markers"
    MARKERS_NAV = "/target_nav_markers"
    LIDAR_ENABLED = "/lidar_enabled"
    RISK_ESTIMATION_RL = "/risk_estimation_rl"
    RISK_ESTIMATION_BRUTE = "/risk_estimation_brute"
    RISK_ESTIMATION_SOTA = "/risk_estimation_sota"
    BRUTEFORCE_ENABLED = "/bruteforce_enabled"
    WORKSTATIONMAPPER_ENABLED = "/workstation_mapper_enabled"
    SOTA_ENABLED = "/sota_enabled"


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


class Metrics(Enum):
    COLLISIONS = "collisions"
    PRODUCTION_STOPS = "prodStop"
    RISK = "risk"
    COLLISION_PROBABILITY = "collisionProb"


class LoggingLevel(Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    EVAL = "EVAL"
