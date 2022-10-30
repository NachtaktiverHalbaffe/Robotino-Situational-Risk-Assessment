# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'form.ui'
##
## Created by: Qt User Interface Compiler version 6.2.4
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(786, 284)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.Train = QPushButton(self.centralwidget)
        self.Train.setObjectName(u"Train")
        self.Train.setGeometry(QRect(30, 30, 181, 91))
        self.Test_btn = QPushButton(self.centralwidget)
        self.Test_btn.setObjectName(u"Test_btn")
        self.Test_btn.setGeometry(QRect(240, 30, 181, 91))
        self.Control_btn = QPushButton(self.centralwidget)
        self.Control_btn.setObjectName(u"Control_btn")
        self.Control_btn.setGeometry(QRect(30, 140, 181, 91))
        self.stop_train_test = QPushButton(self.centralwidget)
        self.stop_train_test.setObjectName(u"stop_train_test")
        self.stop_train_test.setGeometry(QRect(450, 30, 181, 91))
        self.virtual_obj_btn = QPushButton(self.centralwidget)
        self.virtual_obj_btn.setObjectName(u"virtual_obj_btn")
        self.virtual_obj_btn.setGeometry(QRect(240, 140, 181, 91))
        self.close_btn = QPushButton(self.centralwidget)
        self.close_btn.setObjectName(u"close_btn")
        self.close_btn.setGeometry(QRect(450, 140, 181, 91))
        self.shape_selector = QComboBox(self.centralwidget)
        self.shape_selector.setObjectName(u"shape_selector")
        self.shape_selector.setGeometry(QRect(640, 30, 131, 41))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 786, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.Train.setText(QCoreApplication.translate("MainWindow", u"Train", None))
        self.Test_btn.setText(QCoreApplication.translate("MainWindow", u"Test", None))
        self.Control_btn.setText(QCoreApplication.translate("MainWindow", u"Control", None))
        self.stop_train_test.setText(QCoreApplication.translate("MainWindow", u"Stop Train/Test", None))
        self.virtual_obj_btn.setText(QCoreApplication.translate("MainWindow", u"Virtaul Objects", None))
        self.close_btn.setText(QCoreApplication.translate("MainWindow", u"Close windows", None))
    # retranslateUi

