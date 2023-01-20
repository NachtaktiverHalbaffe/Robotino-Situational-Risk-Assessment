from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "gui",
        "yolov7",
        "real_robot_navigation",
        "risk_estimation",
        "utils",
        "situational_awarness",
    ],
    scripts=["/scripts"],
    package_dir={"": "src"},
)
setup(**d)
