from enum import Enum


class Topics(Enum):
    PATH = "/path_raw"
    TARGET = "/target"
    TARGET_ID = "/target_id"
    ACML = "/acml_pose"
    OBSTACLES = "/obstacles"
    IMAGE_RAW = "'image_raw'"


class Nodes(Enum):
    PATH_PLANNER = "path_planner"
    WORKSTATION_MAPPER = "workstation_mapper"
    SLAM = "slam"
