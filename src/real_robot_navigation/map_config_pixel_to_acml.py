import numpy as np
import os

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))
config_FinalGridMapv2 = {
    "x3_a": -1.896559,  # number 3 is the workstation that has number 4 on the QR code
    "y3_a": 3.651673,
    "x3_p": 106,
    "y3_p": 48,
    "x5_a": -0.4231,
    "y5_a": 0.7527,
    "x5_p": 136,
    "y5_p": 109,
    # 'rot':0.97+np.pi,
    "rot": 1.12 - np.pi,
    "path": f"{PATH}/maps/FinalGridMapv2cleaned.png",
}
config_FinalScannerMap1 = {
    "x3_a": -0.8142565856271811,
    "y3_a": 0.1506975961760537,
    "x3_p": 100,
    "y3_p": 150,
    "x5_a": -2.4831574505182648,
    "y5_a": 2.9521294984392905,
    "x5_p": 65,
    "y5_p": 91,
    "rot": 1.204,
    "path": f"{PATH}/maps/map_cropped.png",
}
