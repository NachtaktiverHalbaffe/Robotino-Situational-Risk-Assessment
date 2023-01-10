from enum import Enum


class Topics(Enum):
    PATH = "/path_raw"
    TARGET = "/target"
    TARGET_ID = "/target_id"
    ACML = "/acml_pose"
    OBSTACLES = "/obstacles"
