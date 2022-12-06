# This Python file uses the following encoding: utf-8
import os
import sys
import time
import cv2

from PySide6.QtWidgets import QGraphicsScene , QGraphicsPixmapItem, QFileDialog, QMessageBox
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform, QAction)
from PySide6.QtWidgets import (QApplication, QLabel, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QStatusBar, QWidget)
from PySide6.QtCore import QThread, Signal, QProcess   
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
from AutoLabel import AutoLabel, read_data

# setting path
sys.path.insert(1, '../thesis')
from scripts.Velocity import move
from scripts.py_ros_launch import ros_launch_without_core
import config as config
from main import mains
import subprocess

prv_txt = "GUI started"
Pose_data = [0,0,0]

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

        self.ERROR = QColor(255, 0, 0)
        self.NORMAL = QColor(0, 0, 0)
        self.WARNING = QColor(255,153,102)
        self.HIGHLIGHT = QColor(255,204,0)
        self.OK = QColor(51,153,0)
        self.BLUE = QColor(0,0,255)

        self.ui.info_box.setText("GUI Started")

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

        self.ui.virtual_obj_btn.setEnabled(False)
        self.ui.close_window_btn.setEnabled(False)
        self.ui.ML_autolabel.setEnabled(False)
        self.ui.Test_btn.setEnabled(False)

        self.ui.shape_selector.currentIndexChanged.connect(self.onshape_selector_changed)
        self.ui.tree_selector.currentIndexChanged.connect(self.treeselector_changed)
        self.ui.ml_algo_selector.currentIndexChanged.connect(self.select_algo)

        self.ui.actionOpen_Map.triggered.connect(self.file_open)
        self.ui.actionInfo.triggered.connect(self.showDialog)
        self.ui.save_map_btn.clicked.connect(lambda: self.show_info(self.ui.path_to_map.text(), new_line=True))
        self.ui.speed_dial.valueChanged.connect(self.show_speed)

        self.ui.traj_btn.clicked.connect(lambda: self.show_map(self.ui.path_to_map.text()))
        self.ui.start_rob_btn.clicked.connect(lambda: self.start_robot())

        self.ui.Train.clicked.connect(lambda: self.start_worker(mode=False))
        self.ui.stop_train_test.clicked.connect(lambda: self.stop_worker())
        self.ui.Test_btn.clicked.connect(lambda: self.start_worker(mode=True))

        self.ui.custom_btn_5.clicked.connect(lambda: self.clicklaunch_())

        self.ui.virtual_obj_btn.clicked.connect(lambda: self.create_geofence())
        self.ui.close_window_btn.clicked.connect(lambda: self.destroy_windows())

        self.ui.ML_autolabel.clicked.connect(lambda: self.predict_clusters())

    def start_robot(self):
        #t1=Thread(target=self.Operation, daemon=True)
        path =        [39.5, 11, 39.5, 11, 39.5,  10,   6, 39.5,  10,  11, 39.5,  11, 39.5,  10,  8,   9,   11,  39,  9, 39] # Time
        direction =   ["r", "f",  "r", "f", "r", "r", "f",  "r", "r", "f",  "r", "f",  "r", "r", "f", "r", "f", "r", "f", "r"]   # Move forward or rotation
        orientation = [True, -1, True,  -1,True,True,  -1, True,True,  -1, True,  -1,  True,False,-1,False, -1,True,  -1, True]
        rospy.init_node('move_publisher_node', anonymous=True)
        self.listener()
        t1=Thread(target=move, args=(path, direction, orientation,), daemon=True)
        t1.start()
        #ros_launch_without_core()
        
    def clicklaunch_(self):
        ROS_PROGRAM = QProcess(self)
        print ("Launching...")
        launch_files_v = "/home/abdul/AbdulRehman_ws/src/IDT/real_nav_control/launch"
        generate_map_launch = launch_files_v + "/" + "generate_map.launch"
        program = "roslaunch" + " " + generate_map_launch
        print(program)
        ROS_PROGRAM.start("roslaunch", [generate_map_launch])
        
        # cmd = ['rosrun', 'map_server', 'map_saver', 'map:=dynamic_map', '-f', 'test_map']
        # subprocess.call(cmd) == 0
        #os.system(program)

    def select_algo(self):
        algo_type = self.ui.ml_algo_selector.currentText()
        if algo_type == "select algorithm":
            #print("Select circle, rectangle, or polygon")
            self.ui.info_box_ml.setText("Select Kmeans or Mixture of Gaussians")
            self.ui.ML_autolabel.setEnabled(False)
        else:
            self.ui.info_box_ml.append(algo_type)    
            self.ui.ML_autolabel.setEnabled(True)

    def treeselector_changed(self):
        tree_type = self.ui.tree_selector.currentText()
        if tree_type == "select tree type":
            #print("Select circle, rectangle, or polygon")
            self.ui.info_box_rl.setText("Select BRUTE_FORCE or IDA")
            self.ui.Test_btn.setEnabled(False)
        else:
            self.ui.info_box_rl.setText(tree_type)    
            self.ui.Test_btn.setEnabled(True)

    def onshape_selector_changed(self):
        object_type = self.ui.shape_selector.currentText()
        if object_type == "select shape":
            #print("Select circle, rectangle, or polygon")
            object_type
            self.ui.virtual_obj_btn.setEnabled(False)
            self.ui.close_window_btn.setEnabled(False)
        else:    
            self.ui.info_box_rl.append(object_type)  
            self.ui.virtual_obj_btn.setEnabled(True)
            self.ui.close_window_btn.setEnabled(True)

    def print_ml(self, text, color=QColor(0, 0, 0), append=False):
        self.ui.info_box_ml.setTextColor(color)
        if append:
            self.ui.info_box_ml.append(text)
        else:
            self.ui.info_box_ml.setText(text)
        self.ui.info_box_ml.setTextColor(color)

    def print_rl(self, text, color=QColor(0, 0, 0), append=False):
        self.ui.info_box_rl.setTextColor(color)
        if append:
            self.ui.info_box_rl.append(text)
        else:
            self.ui.info_box_rl.setText(text)
        self.ui.info_box_rl.setTextColor(self.NORMAL)

    def predict_clusters(self):
        algo_type = self.ui.ml_algo_selector.currentText()
        if len(self.filename) < 10:
            self.print_ml("Please select the collision_data.csv",append=True, color=self.WARNING)
        elif self.filename.split(".")[-1] != "csv":
            self.print_ml("Please select the appropriate file",append=True, color=self.WARNING)
            
        else:
            self.ui.info_box_ml.append("ML Started")
            data, df = read_data(self.filename)
            al = AutoLabel(data, n_clusters = 3)
            if algo_type == "Kmeans":
                al.K_Means()
            if algo_type == "mixture of gaussian":
                al.MixtureofGaussians()
            if algo_type == "Agglomerative":
                al.agglomerative()
            if algo_type == "Mini batch Kmeans":
                al.MiniBatch_KMeans()
        
    def train_test(self, risk=-1):
        #print("Test mode: ")
        indexs = self.sender().index
        print("RISK in GUI:",risk)
        self.print_rl("RISK:"+ str(risk), color = self.ERROR)
        if risk >=0:
            #print(self.thread[1].is_running)
            #print(indexs)
            self.ui.Train.setEnabled(True)
            self.ui.Test_btn.setEnabled(True)
            self.stop_worker()

    def start_worker(self, mode=False):
        #mcts_eval = "BRUTE_FORCE"
        mcts_eval = self.ui.tree_selector.currentText()
        #print(mode)
        self.thread[1] = ThreadClass(parent=None, mode=mode,mcts_eval=mcts_eval, index=1)
        self.thread[1].start()
        self.thread[1].any_signal.connect(self.train_test)
        self.ui.Train.setEnabled(False)
        self.ui.Test_btn.setEnabled(False)
    
    def stop_worker(self):
        #print(self.thread[1].is_running, indexs)
        #print(self.thread[2].is_running, indexs)
        if self.thread[1].is_running==False:
            self.thread[1].stop()
        # if self.thread[2].is_running==False:
        #     self.thread[2].stop()
        self.ui.Train.setEnabled(True)
        self.ui.Test_btn.setEnabled(True)

    # Show all info in the info box
    def show_info(self, text, new_line = True):
        if new_line:
            self.ui.info_box.append(text)
        else:
            self.ui.info_box.setText(text)

    def show_speed(self):
        val = self.ui.speed_dial.value()
        vel = np.interp(val, [0,10], [0.01, 0.15]).round(2)
        self.ui.speed_lcd.display(vel)

    def show_map(self, path):
        if os.path.isfile(path):
            scene = QGraphicsScene(self)
            pixmap = QPixmap(path)
            item = QGraphicsPixmapItem(pixmap)
            scene.addItem(item)
            self.ui.map_display.setScene(scene)

    def file_open(self, map=True):
        name = QFileDialog.getOpenFileName(self)
        if map:
            self.show_map(name[0])
        self.filename = name[0]
        #return name[0]
        #file = open(name, 'r')
        #print(name[0])
        
    def showDialog(self):
        msgBox = QMessageBox()
        msgBox.setIcon(QMessageBox.Information)
        msgBox.setText("Created by Abdul Rehman \n Under development")
        msgBox.setWindowTitle("Information")
        msgBox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        returnValue = msgBox.exec()
        if returnValue == QMessageBox.Ok:
            print('OK clicked')    
    
    def show_real_speed(self, data):
        val = data.twist.twist.linear.x
        #vel = (val).round(2)
        self.ui.speed_real.display(val)

    """" Virtual Objects Functions starts from here"""
    def click_event(self, event, x,y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            points = [x,y]
            self.corners.append(points)
            print(x, ' ', y)
            #print(flags)
            self.ui.info_box_rl.append(str(x) + ' ' + str(y))
            #self.ui.info_box_rl.setText(str(flags))
            
            # displaying the coordinates
            # on the image window
            font = cv2.LINE_AA
            cv2.putText(params, "*",(x-10,y+8), font,
                    1, (255, 0, 0), 2)
            cv2.imshow('image', params)

    def create_polygon(self, map_orig):
        map_copy = map_orig.copy()
        # displaying the image
        cv2.imshow('image', map_copy)
        self.corners = []

        cv2.setMouseCallback('image', self.click_event, map_copy)
 
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

    def draw_geofence(self,img, roi, object="circle"):
        x1=roi[0]
        y1=roi[1]
        width=roi[2]
        height=roi[3]

        # width = int(img.shape[1]/2)
        # height = int(img.shape[0]/2)
        centre_x = int(x1 + width/2)
        centre_y = int(y1 + height/2)

        # Centre Point
        center_coordinates = (centre_x, centre_y)
        #print(center_coordinates)

        # Radius of circle
        if (width/2) >= (height/2):
            radius = int(height/2)
        else:
            radius = int(width/2)
        
        thickness = -1
        start_point = (x1,y1)
        end_point = (x1+width,y1+height)
        #print(radius)

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

        #cv2.imshow("Geofence", image)
        return image

    def create_geofence(self):
        object_type = self.ui.shape_selector.currentText()
        configs = config.configs[0]
        map = cv2.imread(configs['map_path'])
 
        scale_percent = 200 # percent of original size
        width = int(map.shape[1] * scale_percent / 100)
        height = int(map.shape[0] * scale_percent / 100)
        dim = (width, height)
  
        # resize image
        resized = cv2.resize(map, dim, interpolation = cv2.INTER_AREA)
        
        if object_type == "polygon":
            image = self.create_polygon(resized)
        else:
            rect = cv2.selectROI(resized)

            image = self.draw_geofence(resized,rect, object=object_type)

        # scale_percent = 50 # percent of original size
        # width = int(image.shape[1] * scale_percent / 100)
        # height = int(image.shape[0] * scale_percent / 100)
        dim = (map.shape[1], map.shape[0])

        resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        cv2.imwrite(configs['geo_path'], resized)
        #cv2.imshow("Original", map)
        cv2.imshow("Geo Image", image)
        #cv2.imshow("resized", resized)

    def destroy_windows(self):
        cv2.destroyAllWindows()

    """" Virtual Objects Functions ends here"""

    def listener(self):
        #rospy.init_node("Odometry node", anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.show_real_speed, queue_size=10)
        #rospy.Subscriber('/odom', Odometry, Pose_Callback, queue_size=10)

        #move()
        
        #rospy.spin()    ##Run the node until we shutdown

class ThreadClass(QThread):
    any_signal = Signal(float)
    def __init__(self, parent=None, mode=False, mcts_eval="BRUTE_FORCE", index=0):
        super(ThreadClass,self).__init__(parent)
        self.mode = mode
        self.is_running = True
        self.index = index
        self.mcts = mcts_eval
    
    def run(self):
        print('Starting Thread...', self.mode)
        #cnt=0
        done, risk = mains(mode=self.mode, mcts_eval=self.mcts)
        self.is_running=False
        if done:
            self.any_signal.emit(risk)
        # while (True):
        #     self.any_signal.emit(done)
    def stop(self):
        self.is_running=False
        print('Stopping thread... ', self.mode)
        self.terminate()
    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    #sys.exit(app.exec())
    try:
        # widget.listener()
        print("GUI Started") 
    except rospy.ROSInterruptException:
        pass
    
    sys.exit(app.exec())
