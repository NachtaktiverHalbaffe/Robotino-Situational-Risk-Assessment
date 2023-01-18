from enum import Enum


class Topics(Enum):
    GLOBAL_PATH = "/path_global"
    LOCAL_PATH = "/path_local"
    TARGET = "/target"
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


class Nodes(Enum):
    PATH_PLANNER = "path_planner"
    WORKSTATION_MAPPER = "workstation_mapper"
    SLAM = "slam"
    OBJECT_DETECTION = "object_detection"
    CONTROL = "control"
