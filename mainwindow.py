# This Python file uses the following encoding: utf-8
import os
import sys
import time
import cv2

from PySide6.QtWidgets import (
    QGraphicsScene,
    QGraphicsPixmapItem,
    QFileDialog,
    QMessageBox,
)
from PySide6.QtGui import (
    QBrush,
    QColor,
    QConicalGradient,
    QCursor,
    QFont,
    QFontDatabase,
    QGradient,
    QIcon,
    QImage,
    QKeySequence,
    QLinearGradient,
    QPainter,
    QPalette,
    QPixmap,
    QRadialGradient,
    QTransform,
    QAction,
)
from PySide6.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QMenuBar,
    QPushButton,
    QSizePolicy,
    QStatusBar,
    QWidget,
)
from PySide6.QtCore import QThread, Signal, QProcess

############ Generation of ui_form.py (GUI-file)
# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py
# cmd = ['rosrun', 'map_server', 'map_saver', 'map:=dynamic_map', '-f', f1]
#    self.assertTrue(subprocess.call(cmd) == 0)
from Mapper.ui_form import Ui_MainWindow
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from threading import *
from AutoLabel import AutoLabel, read_data, evaluate_virtual_vs_ida

# setting path
sys.path.insert(1, "../thesis")
from scripts.Velocity import move
from scripts.py_ros_launch import ros_launch_without_core
import config as config
from eval import mains as evaluation
import subprocess

prv_txt = "GUI started"
Pose_data = [0, 0, 0]

# def Pose_Callback(data):
#     global Pose_data
#     Pose_data = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z]

# def move():
#     #rospy.init_node('move_publisher_node', anonymous=True) #Creation of node
#     move_publisher = rospy.Publisher('cmd_vel_real', Twist, queue_size=10)

#     rate = rospy.Rate(0.5)
#     t0 = rospy.Time.now().to_sec()
#     rate.sleep()
#     x0 = Pose_data[0]
#     z0 = Pose_data[2]

#     while abs(Pose_data[2] - z0) < 2:
#         print("Z0: ", z0, "Pose_data[2]: ", Pose_data[2], "diff: ", (Pose_data[2] - z0))
#         move_publisher.publish(rotate(vel=0.15))
#         x0 = Pose_data[0]


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.thread = {}
        self.corners = []
        self.filename = ""
        self.combined_eval = False

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

        self.ui.actionOpen_Map.triggered.connect(lambda: self.file_open(map=True),)
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

        self.ui.custom_btn_5.clicked.connect(lambda: self.clicklaunch_())

        self.ui.virtual_obj_btn.clicked.connect(lambda: self.create_geofence())
        self.ui.close_window_btn.clicked.connect(lambda: self.destroy_windows())

        self.ui.ML_autolabel.clicked.connect(lambda: self.predict_clusters())
        self.ui.ML_test.clicked.connect(lambda: self.evaluate_ida())

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
        self.listener()
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

    def clicklaunch_(self):
        """
        Launches ROS with the launch configuration from /src/IDT/real_nav_control/launch/generate_map.launch
        """
        ROS_PROGRAM = QProcess(self)
        print("Launching...")
        # location of launch file
        launch_files_v = "/home/abdul/AbdulRehman_ws/src/IDT/real_nav_control/launch"
        generate_map_launch = f"{launch_files_v}/generate_map.launch"
        # create CLI command
        program = f"roslaunch {generate_map_launch}"
        print(program)
        # Execute command as own process
        ROS_PROGRAM.start("roslaunch", [generate_map_launch])

        # cmd = ['rosrun', 'map_server', 'map_saver', 'map:=dynamic_map', '-f', 'test_map']
        # subprocess.call(cmd) == 0
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
            self.print_rl("Combined evaluation: " + str(self.ui.combined_ida_and_brute.isChecked()), append=True)
            self.combined_eval = self.ui.combined_ida_and_brute.isChecked()
            #print(self.ui.combined_ida_and_brute.isChecked())

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
                "Please select the data/collision_data_*.csv file", append=True, color=self.WARNING
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
            mae,mse,rmse = evaluate_virtual_vs_ida(y_brute,y_ida)
            self.print_ml(
                "MAE: "+str(mae), append=True, color=self.BLUE
            )
            self.print_ml(
                "MSE: "+str(mse), append=True, color=self.BLUE
            )
            self.print_ml(
                "RMSE: "+str(rmse), append=True, color=self.BLUE
            )

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
                "Please select the data/collision_data_*.csv file", append=True, color=self.WARNING

            )
        elif self.filename.split(".")[-1] != "csv":
            self.print_ml(
                "Please select the appropriate file", append=True, color=self.WARNING
            )

        # Start autolabeling
        else:
            self.ui.info_box_ml.append("ML Started")
            data, df = read_data(self.filename)
            data= df[["N_nodes", "length", "Prob_collision_Brute_force"]].values

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
            parent=None, mode=mode, mcts_eval=mcts_eval, index=1, combined_eval=self.combined_eval

        )
        self.thread[1].start()
        self.thread[1].any_signal.connect(self.train_test)
        # Disable train/test button so it can't be restarted
        self.ui.Train.setEnabled(False)
        self.ui.Test_btn.setEnabled(False)

    def stop_worker(self):
        """
        Stops the worker which runs the adversary
        """

        if self.thread[1].is_running==False:
            self.thread[1].stop()


        # # 1 adversary running
        # if len(self.thread) >= 1:
        #     if self.thread[1].is_running == False:
        #         self.thread[1].stop()
        # # 2 adversary running
        # if len(self.thread) >= 2:
        #     if self.thread[2].is_running == False:
        #         self.thread[2].stop()

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
        #print(name[0])


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
            print(x, " ", y)
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
        print(self.corners)
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
        print(object)
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
        configs = config.configs[0]
        map = cv2.imread(configs["map_path"])#, cv2.IMREAD_GRAYSCALE)


        scale_percent = 200  # percent of original size
        width = int(map.shape[1] * scale_percent / 100)
        height = int(map.shape[0] * scale_percent / 100)
        dim = (width, height)

        # resize image
        resized = cv2.resize(map, dim, interpolation=cv2.INTER_AREA)

        if object_type == "polygon":
            image = self.create_polygon(resized)
        else:
            rect = cv2.selectROI(resized)

            image = self.draw_geofence(resized, rect, object=object_type)

        # scale_percent = 50 # percent of original size
        # width = int(image.shape[1] * scale_percent / 100)
        # height = int(image.shape[0] * scale_percent / 100)
        dim = (map.shape[1], map.shape[0])

        resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
        cv2.imwrite(configs["geo_path"], resized)
        # cv2.imshow("Original", map)
        cv2.imshow("Geo Image", image)
        # cv2.imshow("resized", resized)

    def destroy_windows(self):
        """
        Closes all windows where images of maps are drawn
        """
        cv2.destroyAllWindows()

    def listener(self):
        # rospy.init_node("Odometry node", anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.show_real_speed, queue_size=10)
        # rospy.Subscriber('/odom', Odometry, Pose_Callback, queue_size=10)

        # move()

        # rospy.spin()    ##Run the node until we shutdown


class ThreadClass(QThread):
    """
    Class for a worker that runs a adversary

    Attributes:
        mode (bool): If model should be trained (False) or evaluated (True). Defaults to False
        is_running (bool): If adversary is running (True) or has finished (False)
        index (bool): Currently unused. Defaults to 0
    """

    any_signal = Signal(float)

    def __init__(self, parent=None, mode=False, mcts_eval="IDA", index=0, combined_eval = False):
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
        done, risk = evaluation(mode=self.mode, mcts_eval=self.mcts, combined_eval=self.combined_eval)
        self.is_running = False
        if done:
            self.any_signal.emit(risk)
        # while (True):
        #     self.any_signal.emit(done)

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

