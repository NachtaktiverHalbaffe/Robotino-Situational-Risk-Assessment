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
from PySide6.QtCore import QThread, Signal   
# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py
from ui_form import Ui_MainWindow
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

# setting path
sys.path.insert(1, '../thesis')
from scripts.Velocity import drive_backward, drive_forward, listener, rotate, stop_robot
import config as config
from main import mains

prv_txt = "GUI started"
Pose_data = [0,0,0]

# def Pose_Callback(data):
#     global Pose_data
#     Pose_data = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z]

def move():
    #rospy.init_node('move_publisher_node', anonymous=True) #Creation of node
    move_publisher = rospy.Publisher('cmd_vel_real', Twist, queue_size=10)

    rate = rospy.Rate(0.5)
    t0 = rospy.Time.now().to_sec()
    rate.sleep()
    x0 = Pose_data[0]
    z0 = Pose_data[2]

    while abs(Pose_data[2] - z0) < 2:
        print("Z0: ", z0, "Pose_data[2]: ", Pose_data[2], "diff: ", (Pose_data[2] - z0))
        move_publisher.publish(rotate(vel=0.15))
        x0 = Pose_data[0]

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.thread = {}
        self.corners = []

        self.ui.shape_selector.addItem("select shape")
        self.ui.shape_selector.addItem("circle")
        self.ui.shape_selector.addItem("rectangle")
        self.ui.shape_selector.addItem("polygon")
        self.ui.virtual_obj_btn.setEnabled(False)
        self.ui.close_window_btn.setEnabled(False)
        self.ui.shape_selector.currentIndexChanged.connect(self.onshape_selector_changed)
        self.ui.actionOpen_Map.triggered.connect(self.file_open)
        self.ui.actionInfo.triggered.connect(self.showDialog)
        self.ui.save_map_btn.clicked.connect(lambda: self.show_info(self.ui.path_to_map.text(), new_line=True))
        self.ui.speed_dial.valueChanged.connect(self.show_speed)
        self.ui.info_box.setText("GUI Started")
        self.ui.traj_btn.clicked.connect(lambda: self.show_map(self.ui.path_to_map.text()))
        self.ui.custom_btn_1.clicked.connect(lambda: self.start_worker())
        self.ui.custom_btn_2.clicked.connect(lambda: self.stop_worker())
        self.ui.custom_btn_3.clicked.connect(lambda: self.start_worker_2())
        self.ui.custom_btn_4.clicked.connect(lambda: self.stop_worker_2())
        self.ui.virtual_obj_btn.clicked.connect(lambda: self.create_geofence())
        self.ui.close_window_btn.clicked.connect(lambda: self.destroy_windows())

    def onshape_selector_changed(self):
        object_type = self.ui.shape_selector.currentText()
        if object_type == "select shape":
            print("Select circle, rectangle, or polygon")
            self.ui.virtual_obj_btn.setEnabled(False)
            self.ui.close_window_btn.setEnabled(False)
        else:    
            self.ui.virtual_obj_btn.setEnabled(True)
            self.ui.close_window_btn.setEnabled(True)
    def start_worker(self):
        self.thread[1] = ThreadClass(parent=None, index=1)
        self.thread[1].start()
        self.thread[1].any_signal.connect(self.testing_fun)
        self.ui.custom_btn_1.setEnabled(False)
    
    def stop_worker(self):
        self.thread[1].stop()
        self.ui.custom_btn_1.setEnabled(True)

    def start_worker_2(self):
        self.thread[2] = ThreadClass(parent=None, index=2)
        self.thread[2].start()
        self.thread[2].any_signal.connect(self.testing_fun)
        self.ui.custom_btn_3.setEnabled(False)
    
    def stop_worker_2(self):
        self.thread[2].stop()
        self.ui.custom_btn_3.setEnabled(True)

    def testing_fun(self,counter):
        cnt=counter
        index = self.sender().index
        if index==1:
            self.ui.speed_real.display(cnt)
        if index==2:
            self.ui.speed_lcd.display(cnt)

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

    def file_open(self):
        name = QFileDialog.getOpenFileName(self)
        self.show_map(name[0])
        #file = open(name, 'r')
        #print(name[0])
        
    def showDialog(self):
        msgBox = QMessageBox()
        msgBox.setIcon(QMessageBox.Information)
        msgBox.setText("Created by Abdul Rehman \n \
                        Under development")
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
            print(flags)
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

    """" Virtual Objects Functions starts from here"""

    def listener(self):
        rospy.init_node("Odometry node", anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.show_real_speed, queue_size=10)
        #rospy.Subscriber('/odom', Odometry, Pose_Callback, queue_size=10)

        #move()
        
        #rospy.spin()    ##Run the node until we shutdown
    


    # def move_forward(self, text):
    #     self.Speed.setText(text)
    #     self.Speed.adjustSize()
    #     move_publisher = rospy.Publisher('cmd_vel_real', Twist, queue_size=10)
    #     move_publisher.publish(drive_forward())
            
    # def move_backward(self, text):
    #     self.Speed.setText(text)
    #     self.Speed.adjustSize()
    
    # def move_right(self, text):
    #     self.Speed.setText(text)
    #     self.Speed.adjustSize()
    
    # def move_left(self, text):
    #     self.Speed.setText(text)
    #     self.Speed.adjustSize()
    
    # def stop(self, text):
    #     self.Speed.setText(text)
    #     self.Speed.adjustSize()
    #     move_publisher = rospy.Publisher('cmd_vel_real', Twist, queue_size=10)
    #     move_publisher.publish(stop_robot())

class ThreadClass(QThread):
    any_signal = Signal(int)
    def __init__(self, parent=None, index=0):
        super(ThreadClass,self).__init__(parent)
        self.index = index
        self.is_running = True
    
    def run(self):
        print('Starting Thread...', self.index)
        if self.index == 1:
            mains()
        cnt=0
        while (True):
            cnt+=1
            if cnt==99: cnt=0
            time.sleep(0.1)
            self.any_signal.emit(cnt)
    def stop(self):
        self.is_running=False
        print('Stopping thread...', self.index)
        self.terminate()
    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    #sys.exit(app.exec())
    try:
        # widget.listener()
        print("Completed !!!") 
    except rospy.ROSInterruptException:
        pass
    
    sys.exit(app.exec())
