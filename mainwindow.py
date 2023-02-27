# This Python file uses the following encoding: utf-8
import os, sys
import time
import cv2
import rospy
import rostopic
import subprocess
import numpy as np
from PySide6.QtWidgets import (
    QGraphicsScene,
    QGraphicsPixmapItem,
    QFileDialog,
    QMessageBox,
)
from PySide6.QtGui import QColor, QPixmap
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QThread, Signal, QProcess
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Bool, String
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from threading import Thread

############ Generation of ui_form.py (GUI-file)
# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py

# setting path
sys.path.insert(1, "../thesis")
from gui.ui_form import Ui_MainWindow
from scripts.velocity import move, drive_backward, drive_forward, stop_robot, rotate
from utils.py_ros_launch import ros_launch_without_core
from risk_estimation.config import configs
from risk_estimation.eval import mains as evaluation
from src.risk_estimation.AutoLabel import AutoLabel, read_data, evaluate_virtual_vs_ida
from utils.constants import Topics, Nodes, Paths
from utils.navigation_utils import pathToTraj
from prototype.msg import Risk, ObstacleMsg, ObstacleList

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "", ""))


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.thread = {}
        self.corners = []
        self.filename = ""
        self.combined_eval = False
        self.speed = 0

        # Textcolors for textboxes
        self.ERROR = QColor(255, 0, 0)
        self.NORMAL = QColor(0, 0, 0)
        self.WARNING = QColor(255, 153, 102)
        self.HIGHLIGHT = QColor(255, 204, 0)
        self.OK = QColor(51, 153, 0)
        self.BLUE = QColor(0, 0, 255)

        self.ui.info_box.setText("GUI Started")
        # ------------------------------ Add Items to dropdown menus ----------------------------------------
        self.ui.shape_selector.addItem("select shape")
        self.ui.shape_selector.addItem("circle")
        self.ui.shape_selector.addItem("rectangle")
        self.ui.shape_selector.addItem("polygon")

        self.ui.tree_selector.addItem("select tree type")
        self.ui.tree_selector.addItem("BRUTE_FORCE")
        self.ui.tree_selector.addItem("BINARY_SEARCH")
        self.ui.tree_selector.addItem("IDA")

        self.ui.ml_algo_selector.addItem("select algorithm")
        self.ui.ml_algo_selector.addItem("Kmeans")
        self.ui.ml_algo_selector.addItem("mixture of gaussian")
        self.ui.ml_algo_selector.addItem("Agglomerative")
        self.ui.ml_algo_selector.addItem("Mini batch Kmeans")

        PATH_ERRORDISTS = f"{PATH}logs/error_dist_csvs"
        self.ui.comboBox_errDistrANglePath.addItems(
            [
                "Set path for angle error CSV",
                Paths.ERRORDIST_ANGLE.value,
                f"{PATH_ERRORDISTS}/localization_error_badlight_angle.csv",
                f"{PATH_ERRORDISTS}/localization_error_faulty_initialpose_angle.csv",
            ]
        )
        self.ui.comboBox_errDistrANglePath.setCurrentIndex(0)
        self.ui.comboBox_errDistrANglePath.currentIndexChanged.connect(
            self.setPathErrorAngleDist
        )
        self.ui.comboBox_errDistrDistPath.addItems(
            [
                "Set path for distance error CSV",
                Paths.ERRORDIST_DIST.value,
                f"{PATH_ERRORDISTS}/localization_error_badlight_dist.csv",
                f"{PATH_ERRORDISTS}/localization_error_faulty_initialpose_dist.csv",
            ]
        )
        self.ui.comboBox_errDistrDistPath.setCurrentIndex(0)
        self.ui.comboBox_errDistrDistPath.currentIndexChanged.connect(
            self.setPathErrorDistrDist
        )

        # -------------------- Enable buttons which are pressable at starttime ------------------------------
        self.ui.virtual_obj_btn.setEnabled(False)
        self.ui.close_window_btn.setEnabled(False)
        self.ui.ML_autolabel.setEnabled(False)
        self.ui.Test_btn.setEnabled(False)

        # ----------Set callback function for button presse/ value changes of inputs in UI -----------------
        self.ui.shape_selector.currentIndexChanged.connect(
            self.onshape_selector_changed
        )
        self.ui.tree_selector.currentIndexChanged.connect(self.treeselector_changed)
        self.ui.ml_algo_selector.currentIndexChanged.connect(self.select_algo)

        self.ui.actionOpen_Map.triggered.connect(
            lambda: self.file_open(map=True),
        )
        self.ui.actionInfo.triggered.connect(self.showDialog)
        self.ui.save_map_btn.clicked.connect(
            lambda: self.show_info(self.ui.path_to_map.text(), new_line=True)
        )
        self.ui.speed_dial.valueChanged.connect(self.show_speed)

        self.ui.traj_btn.clicked.connect(
            lambda: self.show_map(self.ui.path_to_map.text())
        )
        self.ui.start_rob_btn.clicked.connect(lambda: self.start_robot())

        self.ui.Train.clicked.connect(lambda: self.start_worker(mode=False))
        self.ui.stop_train_test.clicked.connect(lambda: self.stop_worker())
        self.ui.Test_btn.clicked.connect(lambda: self.start_worker(mode=True))

        self.ui.btn_start_generateMap.clicked.connect(
            lambda: self.clicklaunch_("generate_map.launch")
        )
        self.ui.btn_start_prototype.clicked.connect(
            lambda: self.clicklaunch_("prototype.launch")
        )
        self.ui.btn_start_identifyAndMap.clicked.connect(
            lambda: self.clicklaunch_("identifyAndMap.launch")
        )
        self.ui.btn_start_autonomous.clicked.connect(
            lambda: self.clicklaunch_("autonomousOperation.launch")
        )
        self.ui.Foward.clicked.connect(lambda: self.driveForward())
        self.ui.Backward.clicked.connect(lambda: self.driveBackward())
        self.ui.Left.clicked.connect(lambda: self.rotateLeft())
        self.ui.Right.clicked.connect(lambda: self.rotateRight())
        self.ui.Break.clicked.connect(lambda: self.stop())

        self.ui.virtual_obj_btn.clicked.connect(lambda: self.create_geofence())
        self.ui.close_window_btn.clicked.connect(lambda: self.destroy_windows())

        self.ui.ML_autolabel.clicked.connect(lambda: self.predict_clusters())
        self.ui.ML_test.clicked.connect(lambda: self.evaluate_ida())

        self.ui.pushButton_driveToWS.clicked.connect(lambda: self.driveToWS())
        self.ui.pushButton_driveToCor.clicked.connect(lambda: self.driveToCor())
        self.ui.pushButton.clicked.connect(lambda: self.runObstacleEstimation())
        self.ui.checkBox_LIDAR.stateChanged.connect(
            lambda: self.activateFeature("lidar")
        )
        self.ui.checkBox_qrScanner.stateChanged.connect(
            lambda: self.activateFeature("qr")
        )
        self.ui.checkBox_bruteforce.stateChanged.connect(
            lambda: self.activateFeature("bruteforce")
        )
        self.ui.checkBox_baselineRisk.stateChanged.connect(
            lambda: self.activateFeature("baseline")
        )
        self.ui.checkBox_errorDIst.stateChanged.connect(
            lambda: self.activateFeature("error_dist")
        )
        self.ui.checkBox_freezeObjects.stateChanged.connect(
            lambda: self.activateFeature("freeze_objects")
        )
        self.ui.checkBox_Offset.stateChanged.connect(
            lambda: self.activateFeature("inject_offset")
        )
        # Make shure roscore is running before starting gui node
        try:
            rostopic.get_topic_class("/rosout")
        except:
            QProcess(self).start("roscore")
        finally:
            rospy.init_node(Nodes.GUI.value)
            self.listener()
            self.corPublisher = rospy.Publisher(
                Topics.TARGET.value, PoseStamped, queue_size=10
            )
            self.wsPublisher = rospy.Publisher(
                Topics.TARGET_ID.value, Int16, queue_size=10
            )
            self.geofencePublisher = rospy.Publisher(
                Topics.OBSTACLES_GEOFENCED.value, ObstacleMsg, queue_size=10
            )
            self.obsRiskEstPublisher = rospy.Publisher(
                Topics.NR_OF_RUNS.value, Int16, queue_size=10
            )
            self.errorDistrDistPathPub = rospy.Publisher(
                Topics.PATH_ERRORDIST_DIST.value, String, queue_size=10, latch=True
            )
            self.errorDistrAnglePathpub = rospy.Publisher(
                Topics.PATH_ERRORDISTR_ANGLE.value, String, queue_size=10, latch=True
            )
            self.LIDARpublisher = rospy.Publisher(
                Topics.LIDAR_BREAKDOWN.value, Bool, queue_size=10, latch=True
            )
            self.qrPublisher = rospy.Publisher(
                Topics.WORKSTATIONMAPPER_ENABLED.value, Bool, queue_size=10, latch=True
            )
            self.brutePublisher = rospy.Publisher(
                Topics.BRUTEFORCE_ENABLED.value, Bool, queue_size=10, latch=True
            )
            self.sotaPublisher = rospy.Publisher(
                Topics.SOTA_ENABLED.value, Bool, queue_size=10, latch=True
            )
            self.errorDistPublisher = rospy.Publisher(
                Topics.USE_ERRORDIST.value, Bool, queue_size=10, latch=True
            )
            self.freezeObjectsPub = rospy.Publisher(
                Topics.FREEZE_OBJECTS.value, Bool, queue_size=10, latch=True
            )
            self.injectOffsetPub = rospy.Publisher(
                Topics.INJECT_OFFSET.value, Bool, queue_size=10, latch=True
            )
            self.ui.checkBox_Offset.setChecked(False)
            self.ui.checkBox_freezeObjects.setChecked(False)
            self.ui.checkBox_LIDAR.setChecked(True)
            self.ui.checkBox_qrScanner.setChecked(True)
            self.ui.checkBox_bruteforce.setChecked(False)
            self.ui.checkBox_baselineRisk.setChecked(False)
            self.ui.checkBox_errorDIst.setChecked(False)

    def start_robot(self):
        """
        Starts the Robotino and let it move so acml can locate it
        """
        # t1=Thread(target=self.Operation, daemon=True)
        path = [
            39.5,
            11,
            39.5,
            11,
            39.5,
            10,
            6,
            39.5,
            10,
            11,
            39.5,
            11,
            39.5,
            10,
            8,
            9,
            11,
            39,
            9,
            39,
        ]  # Time
        direction = [
            "r",
            "f",
            "r",
            "f",
            "r",
            "r",
            "f",
            "r",
            "r",
            "f",
            "r",
            "f",
            "r",
            "r",
            "f",
            "r",
            "f",
            "r",
            "f",
            "r",
        ]  # Move forward or rotation
        orientation = [
            True,
            -1,
            True,
            -1,
            True,
            True,
            -1,
            True,
            True,
            -1,
            True,
            -1,
            True,
            False,
            -1,
            False,
            -1,
            True,
            -1,
            True,
        ]
        rospy.init_node("move_publisher_node", anonymous=True)

        t1 = Thread(
            target=move,
            args=(
                path,
                direction,
                orientation,
            ),
            daemon=True,
        )
        t1.start()
        # ros_launch_without_core()

    def driveForward(self):
        Thread(target=drive_forward, args=[self.speed], daemon=True).start()

    def driveBackward(self):
        Thread(target=drive_backward, args=[self.speed], daemon=True).start()

    def rotateLeft(self):
        Thread(target=rotate, args=[self.speed, True], daemon=True).start()

    def rotateRight(self):
        Thread(target=rotate, args=[self.speed, False], daemon=True).start()

    def stop(self):
        Thread(target=stop_robot, daemon=True).start()

    def clicklaunch_(self, path: str):
        """
        Launches ROS with the launch configuration from /src/IDT/real_nav_control/launch/generate_map.launch
        """
        ROS_PROGRAM = QProcess(self)
        print("Launching...")
        # location of launch file
        launch_files_v = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "launch/", "")
        )
        launch_file = f"{launch_files_v}/{path}"
        # create CLI command
        program = f"roslaunch {launch_file}"
        print(program)
        # Execute command as own process
        # ROS_PROGRAM.start("roslaunch", [launch_file])

        Thread(target=subprocess.call, args=[program.split(" ")]).start()
        # os.system(program)

    def select_algo(self):
        """
        Enabled the AutoLabel button in the localization adversary tab if an algorithm is selected
        """
        algo_type = self.ui.ml_algo_selector.currentText()
        if algo_type == "select algorithm":
            # print("Select circle, rectangle, or polygon")
            self.ui.info_box_ml.setText("Select Kmeans or Mixture of Gaussians")
            self.ui.ML_autolabel.setEnabled(False)
        else:
            self.ui.info_box_ml.append(algo_type)
            self.ui.ML_autolabel.setEnabled(True)

    def treeselector_changed(self):
        """
        Enabled the Test button in the RL section in the  localization adversary tab if an tree-algorithm is selected
        """
        tree_type = self.ui.tree_selector.currentText()
        if tree_type == "select tree type":
            self.ui.info_box_rl.setText("Select BRUTE_FORCE or IDA")
            self.ui.Test_btn.setEnabled(False)
        else:
            self.ui.info_box_rl.setText(tree_type)
            self.print_rl(
                "Combined evaluation: "
                + str(self.ui.combined_ida_and_brute.isChecked()),
                append=True,
            )
            self.combined_eval = self.ui.combined_ida_and_brute.isChecked()
            # print(self.ui.combined_ida_and_brute.isChecked())

            self.ui.Test_btn.setEnabled(True)

    def onshape_selector_changed(self):
        """
        Enable the virtual object & close windows buttons if a shape is selected in the RL section in the localization adversary tab
        """
        object_type = self.ui.shape_selector.currentText()
        if object_type == "select shape":
            object_type
            self.ui.virtual_obj_btn.setEnabled(False)
            self.ui.close_window_btn.setEnabled(False)
        else:
            self.ui.info_box_rl.append(object_type)
            self.ui.virtual_obj_btn.setEnabled(True)
            self.ui.close_window_btn.setEnabled(True)

    def print_ml(self, text, color=QColor(0, 0, 0), append=False):
        """
        Prints a message to the textbox in the ML section in the localization adversary tab

        Args:
            text (str): Message which should be printed
            color (Qcolor(int, int, int)): Textcolor. Defaults to black
            append (bool): If message should be appended to the existing messages (True)  or should replace the existing messages (False)
        """
        self.ui.info_box_ml.setTextColor(color)
        if append:
            self.ui.info_box_ml.append(text)
        else:
            self.ui.info_box_ml.setText(text)
        self.ui.info_box_ml.setTextColor(color)

    def print_rl(self, text, color=QColor(0, 0, 0), append=False):
        """
        Prints a message to the textbox in the RL section in the localization adversary tab

        Args:
            text (str): Message which should be printed
            color (Qcolor(int, int, int)): Textcolor. Defaults to black
            append (bool): If message should be appended to the existing messages (True)  or should replace the existing messages (False)
        """
        self.ui.info_box_rl.setTextColor(color)
        if append:
            self.ui.info_box_rl.append(text)
        else:
            self.ui.info_box_rl.setText(text)
        self.ui.info_box_rl.setTextColor(self.NORMAL)

    def evaluate_ida(self):
        """
        Evaluate IDA and map probability of collisions.

        Args:
            Uses the file the user has opend over the "Open Map" in the file menu
        """
        # Input validation of file
        if len(self.filename) < 10:
            self.print_ml(
                "Please select the data/collision_data_*.csv file",
                append=True,
                color=self.WARNING,
            )
        elif self.filename.split(".")[-1] != "csv":
            self.print_ml(
                "Please select the appropriate file", append=True, color=self.WARNING
            )

        # Start autolabeling
        else:
            self.ui.info_box_ml.append("ML Started")
            data, df = read_data(self.filename)
            y_brute = df["Prob_collision_Brute_force"].values
            y_ida = df["Prob_collision_IDA"].values
            y_exp = df["Expected Probability Collision"].values
            mae, mse, rmse = evaluate_virtual_vs_ida(y_brute, y_ida)
            self.print_ml("MAE: " + str(mae), append=True, color=self.BLUE)
            self.print_ml("MSE: " + str(mse), append=True, color=self.BLUE)
            self.print_ml("RMSE: " + str(rmse), append=True, color=self.BLUE)

    def predict_clusters(self):
        """
        Starts the autolabeling.

        Args:
            Uses the file the user has opend over the "Open Map" in the file menu
            Uses the algorithm the user has selected in the dropdown menu in the ML section in the localization adversary tab
        """
        algo_type = self.ui.ml_algo_selector.currentText()
        # Input validation of file
        if len(self.filename) < 10:
            self.print_ml(
                "Please select the data/collision_data_*.csv file",
                append=True,
                color=self.WARNING,
            )
        elif self.filename.split(".")[-1] != "csv":
            self.print_ml(
                "Please select the appropriate file", append=True, color=self.WARNING
            )

        # Start autolabeling
        else:
            self.ui.info_box_ml.append("ML Started")
            data, df = read_data(self.filename)
            data = df[["N_nodes", "length", "Prob_collision_Brute_force"]].values

            al = AutoLabel(data, n_clusters=3)
            if algo_type == "Kmeans":
                al.K_Means()
            if algo_type == "mixture of gaussian":
                al.MixtureofGaussians()
            if algo_type == "Agglomerative":
                al.agglomerative()
            if algo_type == "Mini batch Kmeans":
                al.MiniBatch_KMeans()

    def train_test(self, risk=-1):
        """
        Enabled the train und test button in the RL section int he localization adversary tab

        Args:
            risk (int):  the risk value which should be printed into the GUI
        """

        indexs = self.sender().index
        print("RISK in GUI:", risk)
        self.print_rl("RISK:" + str(risk), color=self.ERROR)
        if risk >= 0:
            # print(self.thread[1].is_running)
            # print(indexs)
            self.ui.Train.setEnabled(True)
            self.ui.Test_btn.setEnabled(True)
            self.stop_worker()

    def start_worker(self, mode=False):
        """
        Starts a worker in a separate thread which runs the adversaries

        Args:
            mode (bool): If the model should be trained (True) or evaluated (False)
        """
        mcts_eval = self.ui.tree_selector.currentText()
        self.combined_eval = self.ui.combined_ida_and_brute.isChecked()
        # Setup and start thread
        self.thread[1] = ThreadClass(
            parent=None,
            mode=mode,
            mcts_eval=mcts_eval,
            index=1,
            combined_eval=self.combined_eval,
        )
        self.thread[1].start()
        self.thread[1].any_signal.connect(self.train_test)
        # Disable train/test button so it can't be restarted
        self.ui.Train.setEnabled(False)
        self.ui.Test_btn.setEnabled(False)

    def driveToWS(self):
        """
        Sends ROS command to drive the Robot to an resource ID
        """
        id = Int16()
        id.data = self.ui.spinBox_wsId.value()
        try:
            self.wsPublisher.publish(id)
            print(f"Sent command to drive to workstation {id}")
        except:
            print(f"Failed to send command to drive to workstation {id}")

    def driveToCor(self):
        """
        Sends ROS command to drive the Robot to an resource ID
        """
        msg = PoseStamped()
        msg.pose.position.x = self.ui.spinBox_xMan.value()
        msg.pose.position.y = self.ui.spinBox_yMan.value()
        try:
            self.corPublisher.publish(msg)
            print(
                f"Sent command to drive to coordinate ({self.ui.spinBox_xMan.value()},{self.ui.spinBox_yMan.value()})"
            )
        except:
            print(
                f"Failed to send command to drive to coordinate ({self.ui.spinBox_xMan.value()},{self.ui.spinBox_yMan.value()})"
            )

    def activateFeature(self, feature: str):
        if "lidar" in feature.lower():
            try:
                self.LIDARpublisher.publish(not self.ui.checkBox_LIDAR.isChecked())
            except:
                pass
        elif "qr" in feature.lower():
            try:
                self.qrPublisher.publish(self.ui.checkBox_qrScanner.isChecked())
            except:
                pass
        elif "bruteforce" in feature.lower():
            try:
                self.brutePublisher.publish(self.ui.checkBox_bruteforce.isChecked())
            except:
                pass
        elif "baseline" in feature.lower():
            try:
                self.sotaPublisher.publish(self.ui.checkBox_baselineRisk.isChecked())
            except:
                pass
        elif "error_dist" in feature.lower():
            try:
                self.errorDistPublisher.publish(self.ui.checkBox_errorDIst.isChecked())
            except:
                pass
        elif "freeze_objects" in feature.lower():
            try:
                self.freezeObjectsPub.publish(
                    self.ui.checkBox_freezeObjects.isChecked()
                )
            except:
                pass
        elif "inject_offset" in feature.lower():
            try:
                self.injectOffsetPub.publish(self.ui.checkBox_Offset.isChecked())
            except:
                pass

    def runObstacleEstimation(self):
        """
        Starts a risk estimation
        """
        self.obsRiskEstPublisher.publish(self.ui.spinBox_nrOfRuns.value())

    def setPathErrorDistrDist(self):
        path = self.ui.comboBox_errDistrDistPath.currentText()
        try:
            self.errorDistrDistPathPub.publish(path)
        except:
            print("Error: COuldnt publish path of error distribution for distance")

    def setPathErrorAngleDist(self):
        path = self.ui.comboBox_errDistrANglePath.currentText()
        try:
            self.errorDistrAnglePathpub.publish(path)
        except:
            print("Error: COuldnt publish path of error distribution for distance")

    def stop_worker(self):
        """
        Stops the worker which runs the adversary
        """
        for thread in self.thread:
            if thread.is_running == False:
                thread.stop()

        # Enable test/train buttons so adversaries can be restarted
        self.ui.Train.setEnabled(True)
        self.ui.Test_btn.setEnabled(True)

    def show_info(self, text, new_line=True):
        """
        Shows all info in the info box in the mapper tab

        Args:
            text (str): Message to show
            new_line (bool): If message should be appended (True) or replace the existing messages (False). Defaults to True
        """
        if new_line:
            self.ui.info_box.append(text)
        else:
            self.ui.info_box.setText(text)

    def show_speed(self):
        """
        Sets the current speed in the speed lcd in the mapper tab

        Args:
            Takes the value from the GUI. Uses the speed dial in the mapper tab
        """
        val = self.ui.speed_dial.value()
        self.speed = val
        # Linear interpolation of the speed dial value to a velocity value and rounding the value to 2 decimal places
        vel = np.interp(val, [0, 10], [0.01, 0.15]).round(2)
        # Set value in lcd
        self.ui.speed_lcd.display(vel)

    def show_map(self, path):
        """
        Shows the map in the map box in the mapper tab

        Args:
            path (str): Path to the map image.
        """
        # Input validation
        if os.path.isfile(path):
            # Scene where the map should be printed on
            scene = QGraphicsScene(self)
            # Load image
            pixmap = QPixmap(path)
            # Set image onto the scene
            item = QGraphicsPixmapItem(pixmap)
            scene.addItem(item)
            # Print item onto the text box
            self.ui.map_display.setScene(scene)

    def file_open(self, map=True):
        """
        Opens a map file (png)

        Args:
            map (bool): If map should also be showed when opened (True) or not (False). Defaults to True
        """

        name = QFileDialog.getOpenFileName(self)

        if self.ui.tabWidget.tabText(0) == "Mapper":
            self.show_map(name[0])

        self.filename = name[0]
        # return name[0]
        # file = open(name, 'r')
        # print(name[0])

    def showDialog(self):
        """
        Shows a dialog with information about the application
        """
        msgBox = QMessageBox()
        msgBox.setIcon(QMessageBox.Information)
        msgBox.setText("Created by Abdul Rehman \n Under development")
        msgBox.setWindowTitle("Information")
        msgBox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        returnValue = msgBox.exec()
        if returnValue == QMessageBox.Ok:
            print("OK clicked")

    def show_real_speed(self, data):
        """
        Prints the real velocity of the robot into the lcd display in the mapper tab
        """
        val = data.twist.twist.linear.x
        # vel = (val).round(2)
        self.ui.speed_real.display(val)

    ############################################################################
    # ----------------------------- Virtual Objects Functions -------------------------------------
    ############################################################################

    def click_event(self, event, x, y, flags, params):
        """
        Callback function when a corner is placed with a mouseclick. It adds the set corners to the shape and prints a marker on the map

        Args:
            event (cv2.EVENT): type of event
            x (int): x-coordinate
            y (int): y-coordinate
            flags:
            params: image to show
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            points = [x, y]
            self.corners.append(points)
            # print(flags)
            self.ui.info_box_rl.append(str(x) + " " + str(y))
            # self.ui.info_box_rl.setText(str(flags))

            # displaying the coordinates
            # on the image window
            font = cv2.LINE_AA
            cv2.putText(params, "*", (x - 10, y + 8), font, 1, (255, 0, 0), 2)
            cv2.imshow("image", params)

    def create_polygon(self, map_orig):
        """
        Creates a polygon area on the map based on the set corners

        Args:
            map_orig: Image of the map where the polygon should be printed on
            Takes the corners from self.corners
        """
        map_copy = map_orig.copy()
        # displaying the image
        cv2.imshow("image", map_copy)
        self.corners = []

        cv2.setMouseCallback("image", self.click_event, map_copy)

        # wait for a key to be pressed to exit
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        self.ui.info_box_rl.append(str(self.corners))
        pts = np.array(self.corners)
        pts = pts.reshape((-1, 1, 2))

        color = (0, 0, 0)

        image = cv2.fillPoly(map_orig, [pts], color)
        # cv2.imshow("Poly_image", image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        return image

    def draw_geofence(self, img, roi, object="circle"):
        """
        Draws the region of interest (roi) onto the image of the map

        Args:
            img:
            roi ([x1 (int), y1 (int), width (int), height (int)]): The region of object
            object (str): Shape of the object. Can be either "circle" or "rectangle". Defaults to "circle"
        """
        x1 = roi[0]
        y1 = roi[1]
        width = roi[2]
        height = roi[3]

        # width = int(img.shape[1]/2)
        # height = int(img.shape[0]/2)
        centre_x = int(x1 + width / 2)
        centre_y = int(y1 + height / 2)

        # Centre Point
        center_coordinates = (centre_x, centre_y)
        # print(center_coordinates)

        # Radius of circle
        if (width / 2) >= (height / 2):
            radius = int(height / 2)
        else:
            radius = int(width / 2)

        thickness = -1
        start_point = (x1, y1)
        end_point = (x1 + width, y1 + height)
        # print(radius)

        # Red color in BGR
        color = (0, 0, 0)
        self.ui.info_box_rl.append(object)

        if object == "rectangle":
            image = cv2.rectangle(img, start_point, end_point, color, thickness)
        else:
            # Using cv2.circle() method
            # Draw a circle of red color of thickness -1 px
            image = cv2.circle(img, center_coordinates, radius, color, thickness)

        # cv2.imshow("Geofence", image)
        return image

    def create_geofence(self):
        """
        Args:
            Takes the shape from the shape dropdown menu in the RL section of the localization adversary tab
        """
        object_type = self.ui.shape_selector.currentText()
        cfgs = configs[0]
        map = cv2.imread(cfgs["map_path"])  # , cv2.IMREAD_GRAYSCALE)

        scale_percent = 200  # percent of original size
        fracScale = scale_percent / 100
        width = int(map.shape[1] * scale_percent / 100)
        height = int(map.shape[0] * scale_percent / 100)
        dim = (width, height)

        # resize image
        resized = cv2.resize(map, dim, interpolation=cv2.INTER_AREA)
        geofencedObstacle = ObstacleMsg()
        geofencedObstacle.label = "geofenced"
        if object_type == "polygon":
            image = self.create_polygon(resized)

            for corner in self.corners:
                point = Point()
                point.x = corner[0] / fracScale
                point.y = corner[1] / fracScale
                geofencedObstacle.corners.append(point)
        else:
            rect = cv2.selectROI(resized)
            image = self.draw_geofence(resized, rect, object=object_type)

            x1 = rect[0] / fracScale
            y1 = rect[1] / fracScale
            width = rect[2] / fracScale
            height = rect[3] / fracScale
            # Create corners of obstacle
            p = Point()
            p.x = x1
            p.y = y1
            geofencedObstacle.corners.append(p)
            p.x = x1 + width
            p.y = y1
            geofencedObstacle.corners.append(p)
            p.y = y1 - width
            geofencedObstacle.corners.append(p)
            p.x = x1
            geofencedObstacle.corners.append(p)
        try:
            self.geofencePublisher.publish(geofencedObstacle)
        except Exception as e:
            print(f"Couldn't publish geofenced published. Excpetion: {e}")

        # scale_percent = 50 # percent of original size
        # width = int(image.shape[1] * scale_percent / 100)
        # height = int(image.shape[0] * scale_percent / 100)
        dim = (map.shape[1], map.shape[0])

        resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
        cv2.imwrite(cfgs["geo_path"], resized)
        # cv2.imshow("Original", map)
        cv2.imshow("Geo Image", image)
        # cv2.imshow("resized", resized)

    def destroy_windows(self):
        """
        Closes all windows where images of maps are drawn
        """
        cv2.destroyAllWindows()

    def listener(self):
        rospy.Subscriber("/odom", Odometry, self.show_real_speed, queue_size=10)


class ThreadClass(QThread):
    """
    Class for a worker that runs a adversary

    Attributes:
        mode (bool): If model should be trained (False) or evaluated (True). Defaults to False
        is_running (bool): If adversary is running (True) or has finished (False)
        index (bool): Currently unused. Defaults to 0
    """

    any_signal = Signal(float)

    def __init__(
        self, parent=None, mode=False, mcts_eval="IDA", index=0, combined_eval=False
    ):
        super(ThreadClass, self).__init__(parent)
        self.mode = mode
        self.is_running = True
        self.index = index
        self.mcts = mcts_eval
        self.combined_eval = combined_eval

    def run(self):
        """
        Starts the worker.

        Returns:
            Emits an signal with the risk when MAARL is done
        """
        print("Starting Thread...", self.mode)
        # cnt=0
        done, risk = evaluation(
            mode=self.mode,
            mcts_eval=self.mcts,
            combined_eval=self.combined_eval,
        )
        self.is_running = False
        if done:
            self.any_signal.emit(risk)
        # while (True):
        #     self.any_signal.emit(done)

        publisher = rospy.Publisher(
            Topics.RISK_ESTIMATION_SOTA.value, Risk, queue_size=10
        )
        msg = Risk()
        msg.probs_brute.append(risk)
        try:
            publisher.publish(msg)
        except:
            pass

    def stop(self):
        """
        Stops the running worker
        """
        self.is_running = False
        print("Stopping thread... ", self.mode)
        self.terminate()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    try:
        # widget.listener()
        print("GUI Started")
    except rospy.ROSInterruptException:
        pass

    sys.exit(app.exec())
