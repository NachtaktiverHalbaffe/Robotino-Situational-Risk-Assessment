# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'form.ui'
##
## Created by: Qt User Interface Compiler version 6.2.4
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################
import sys

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QLabel, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QStatusBar, QWidget)

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from Velocity import drive_backward, drive_forward, rotate, stop_robot
msg_turn = Twist()

msg_turn.linear.x = 0
msg_turn.linear.y = 0
msg_turn.linear.z = 0
msg_turn.angular.x = 0
msg_turn.angular.y = 0
msg_turn.angular.z = 0


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(778, 533)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.Foward = QPushButton(self.centralwidget)
        self.Foward.setObjectName(u"Foward")
        self.Foward.setGeometry(QRect(270, 120, 91, 51))
        self.Speed = QLabel(self.centralwidget)
        self.Speed.setObjectName(u"Speed")
        self.Speed.setGeometry(QRect(510, 60, 121, 41))
        self.Backward = QPushButton(self.centralwidget)
        self.Backward.setObjectName(u"Backward")
        self.Backward.setGeometry(QRect(270, 240, 91, 51))
        self.Left = QPushButton(self.centralwidget)
        self.Left.setObjectName(u"Left")
        self.Left.setGeometry(QRect(170, 180, 91, 51))
        self.Right = QPushButton(self.centralwidget)
        self.Right.setObjectName(u"Right")
        self.Right.setGeometry(QRect(370, 180, 91, 51))
        self.Break = QPushButton(self.centralwidget)
        self.Break.setObjectName(u"Break")
        self.Break.setGeometry(QRect(270, 180, 91, 51))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 778, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
        # setupUi

        self.Foward.pressed.connect(lambda: self.move_forward("Move Forward"))
        self.Backward.pressed.connect(lambda: self.move_backward("Move Backward"))
        self.Right.pressed.connect(lambda: self.move_right("Move Right"))
        self.Left.pressed.connect(lambda: self.move_left("Move Left"))
        self.Break.pressed.connect(lambda: self.stop("Stop"))

    def move_forward(self, text):
        self.Speed.setText(text)
        self.Speed.adjustSize()
        move_publisher = rospy.Publisher('cmd_vel_real', Twist, queue_size=10)
        move_publisher.publish(drive_forward())
            

    def move_backward(self, text):
        self.Speed.setText(text)
        self.Speed.adjustSize()
    
    def move_right(self, text):
        self.Speed.setText(text)
        self.Speed.adjustSize()
    
    def move_left(self, text):
        self.Speed.setText(text)
        self.Speed.adjustSize()
    
    def stop(self, text):
        self.Speed.setText(text)
        self.Speed.adjustSize()
        move_publisher = rospy.Publisher('cmd_vel_real', Twist, queue_size=10)
        move_publisher.publish(stop_robot())

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.Foward.setText(QCoreApplication.translate("MainWindow", u"Forward", None))
        self.Speed.setText(QCoreApplication.translate("MainWindow", u"0.000", None))
        self.Backward.setText(QCoreApplication.translate("MainWindow", u"Backward", None))
        self.Left.setText(QCoreApplication.translate("MainWindow", u"Left", None))
        self.Right.setText(QCoreApplication.translate("MainWindow", u"Right", None))
        self.Break.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
    # retranslateUi

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

def setup_robotino():
    rospy.init_node('move_publisher_node', anonymous=True) #Creation of node
    #rospy.spin()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    #sys.exit(app.exec())
    try:
        #setup_robotino()
        print("Completed !!!") 
    except rospy.ROSInterruptException:
        pass
    
    sys.exit(app.exec())
