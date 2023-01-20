from enum import Enum


class Topics(Enum):
    GLOBAL_PATH = "/path_global"
    LOCAL_PATH = "/path_local"
    TARGET = "/target"
    LOCAL_TARGET = "/target_local"
    TARGET_ID = "/target_id"
    NAV_POINT = "/nav_point"
    ACML = "/acml_pose"
    IMG_LOCALIZATION = "/camera_pose"
    LOCALIZATION_MODE = "/localization_mode"
    LOCALIZATION = "/pose"
    OBSTACLES = "/obstacles"
    IMAGE_RAW = "'/image_raw'"
    NAVIGATION_RESPONSE = "/navigation_response"
    ODOM = "/odom"
    IR_SENSORS = "/distance_sensors"
    EMERGENCY_BRAKE = "/emergency_break"


class Nodes(Enum):
    PATH_PLANNER = "path_planner"
    WORKSTATION_MAPPER = "workstation_mapper"
    LOCALIZATION = "localization"
    OBJECT_DETECTION = "object_detection"
    CONTROL = "control"
    STRATEGY_PLANNER = "strategy_planner"
    FLEETIAS_CLIENT = "fleetias_client"
