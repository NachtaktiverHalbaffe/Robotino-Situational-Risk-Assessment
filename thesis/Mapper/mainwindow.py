# This Python file uses the following encoding: utf-8
import os
import sys
import time

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
sys.path.append('../thesis')

from scripts.Velocity import drive_backward, drive_forward, listener, rotate, stop_robot

prv_txt = "GUI started"
Pose_data = [0,0,0]

def Pose_Callback(data):
    global Pose_data
    Pose_data = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z]

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
        
        self.ui.actionOpen_Map.triggered.connect(self.file_open)
        self.ui.actionInfo.triggered.connect(self.showDialog)
        self.ui.save_map_btn.clicked.connect(lambda: self.show_info(self.ui.path_to_map.text(), new_line=True))
        self.ui.speed_dial.valueChanged.connect(self.show_speed)
        self.ui.info_box.setText("GUI Started")
        self.ui.traj_btn.clicked.connect(lambda: self.show_map(self.ui.path_to_map.text()))
        self.ui.custom_btn_1.clicked.connect(lambda: self.start_worker())
        self.ui.custom_btn_2.clicked.connect(lambda: self.stop_worker())

    def start_worker(self):
        self.thread[1] = ThreadClass(parent=None, index=1)
        self.thread[1].start()
        self.thread[1].any_signal.connect(self.testing_fun)
        self.ui.custom_btn_1.setEnabled(False)
    
    def stop_worker(self):
        self.thread[1].stop()
        self.ui.custom_btn_1.setEnabled(True)

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
