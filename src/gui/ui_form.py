# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'form.ui'
##
## Created by: Qt User Interface Compiler version 6.4.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDial,
    QGraphicsView, QGridLayout, QGroupBox, QHBoxLayout,
    QLCDNumber, QLabel, QLayout, QLineEdit,
    QMainWindow, QMenu, QMenuBar, QPushButton,
    QSizePolicy, QStatusBar, QTabWidget, QTextEdit,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(916, 511)
        MainWindow.setWindowOpacity(1.000000000000000)
        MainWindow.setAutoFillBackground(False)
        MainWindow.setStyleSheet(u"")
        self.actionOpen_Map = QAction(MainWindow)
        self.actionOpen_Map.setObjectName(u"actionOpen_Map")
        self.actionSave_Map = QAction(MainWindow)
        self.actionSave_Map.setObjectName(u"actionSave_Map")
        self.actionUndo = QAction(MainWindow)
        self.actionUndo.setObjectName(u"actionUndo")
        self.actionRedo_Ctrl_Y = QAction(MainWindow)
        self.actionRedo_Ctrl_Y.setObjectName(u"actionRedo_Ctrl_Y")
        self.actionCut_Ctrl_X = QAction(MainWindow)
        self.actionCut_Ctrl_X.setObjectName(u"actionCut_Ctrl_X")
        self.actionCopy_Ctrl_C = QAction(MainWindow)
        self.actionCopy_Ctrl_C.setObjectName(u"actionCopy_Ctrl_C")
        self.actionPaste_Ctrl_V = QAction(MainWindow)
        self.actionPaste_Ctrl_V.setObjectName(u"actionPaste_Ctrl_V")
        self.actionInfo = QAction(MainWindow)
        self.actionInfo.setObjectName(u"actionInfo")
        self.actionTutorial = QAction(MainWindow)
        self.actionTutorial.setObjectName(u"actionTutorial")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.centralwidget.setStyleSheet(u"#centralwidget{\n"
"background-image: url(Mapper/ml_rl_background.jpeg)\n"
"}")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        font = QFont()
        font.setKerning(True)
        self.tabWidget.setFont(font)
        self.tabWidget.setAutoFillBackground(False)
        self.tabWidget.setStyleSheet(u"")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.gridLayout = QGridLayout(self.tab)
        self.gridLayout.setObjectName(u"gridLayout")
        self.horizontalLayout_12 = QHBoxLayout()
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.verticalLayout_12 = QVBoxLayout()
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.info_box = QTextEdit(self.tab)
        self.info_box.setObjectName(u"info_box")
        font1 = QFont()
        font1.setItalic(False)
        self.info_box.setFont(font1)
        self.info_box.setAcceptDrops(True)
        self.info_box.setReadOnly(True)

        self.verticalLayout_12.addWidget(self.info_box)

        self.logo_ias = QLabel(self.tab)
        self.logo_ias.setObjectName(u"logo_ias")
        self.logo_ias.setAutoFillBackground(False)
        self.logo_ias.setStyleSheet(u"")
        self.logo_ias.setPixmap(QPixmap(u"IAS_logo_small.png"))
        self.logo_ias.setAlignment(Qt.AlignCenter)
        self.logo_ias.setWordWrap(False)

        self.verticalLayout_12.addWidget(self.logo_ias)

        self.gridLayout_5 = QGridLayout()
        self.gridLayout_5.setObjectName(u"gridLayout_5")
        self.speed_dial = QDial(self.tab)
        self.speed_dial.setObjectName(u"speed_dial")
        font2 = QFont()
        font2.setPointSize(11)
        self.speed_dial.setFont(font2)
        self.speed_dial.setAutoFillBackground(False)
        self.speed_dial.setMaximum(10)
        self.speed_dial.setSingleStep(1)
        self.speed_dial.setPageStep(10)
        self.speed_dial.setValue(0)
        self.speed_dial.setSliderPosition(0)
        self.speed_dial.setTracking(True)
        self.speed_dial.setOrientation(Qt.Horizontal)
        self.speed_dial.setWrapping(False)
        self.speed_dial.setNotchesVisible(True)

        self.gridLayout_5.addWidget(self.speed_dial, 1, 0, 1, 1)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.Speed_3 = QLabel(self.tab)
        self.Speed_3.setObjectName(u"Speed_3")
        self.Speed_3.setEnabled(True)
        font3 = QFont()
        font3.setPointSize(9)
        font3.setItalic(True)
        font3.setKerning(True)
        self.Speed_3.setFont(font3)

        self.horizontalLayout_11.addWidget(self.Speed_3)

        self.Speed_4 = QLabel(self.tab)
        self.Speed_4.setObjectName(u"Speed_4")
        font4 = QFont()
        font4.setPointSize(9)
        font4.setItalic(True)
        self.Speed_4.setFont(font4)
        self.Speed_4.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.horizontalLayout_11.addWidget(self.Speed_4)


        self.gridLayout_5.addLayout(self.horizontalLayout_11, 2, 0, 1, 1)

        self.gridLayout_4 = QGridLayout()
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.Backward = QPushButton(self.tab)
        self.Backward.setObjectName(u"Backward")

        self.gridLayout_4.addWidget(self.Backward, 2, 1, 1, 1)

        self.Left = QPushButton(self.tab)
        self.Left.setObjectName(u"Left")

        self.gridLayout_4.addWidget(self.Left, 1, 0, 1, 1)

        self.Foward = QPushButton(self.tab)
        self.Foward.setObjectName(u"Foward")

        self.gridLayout_4.addWidget(self.Foward, 0, 1, 1, 1)

        self.Break = QPushButton(self.tab)
        self.Break.setObjectName(u"Break")

        self.gridLayout_4.addWidget(self.Break, 1, 1, 1, 1)

        self.Right = QPushButton(self.tab)
        self.Right.setObjectName(u"Right")

        self.gridLayout_4.addWidget(self.Right, 1, 2, 1, 1)

        self.use_keyboard_chk = QCheckBox(self.tab)
        self.use_keyboard_chk.setObjectName(u"use_keyboard_chk")

        self.gridLayout_4.addWidget(self.use_keyboard_chk, 2, 2, 1, 1)


        self.gridLayout_5.addLayout(self.gridLayout_4, 1, 1, 1, 1)


        self.verticalLayout_12.addLayout(self.gridLayout_5)


        self.horizontalLayout_12.addLayout(self.verticalLayout_12)


        self.gridLayout.addLayout(self.horizontalLayout_12, 10, 2, 1, 1)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.on_off_btn = QPushButton(self.tab)
        self.on_off_btn.setObjectName(u"on_off_btn")
        self.on_off_btn.setAutoFillBackground(False)

        self.horizontalLayout_8.addWidget(self.on_off_btn)

        self.start_rob_btn = QPushButton(self.tab)
        self.start_rob_btn.setObjectName(u"start_rob_btn")

        self.horizontalLayout_8.addWidget(self.start_rob_btn)

        self.gen_map_btn = QPushButton(self.tab)
        self.gen_map_btn.setObjectName(u"gen_map_btn")

        self.horizontalLayout_8.addWidget(self.gen_map_btn)

        self.label = QLabel(self.tab)
        self.label.setObjectName(u"label")
        self.label.setTextFormat(Qt.MarkdownText)
        self.label.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.horizontalLayout_8.addWidget(self.label)

        self.speed_lcd = QLCDNumber(self.tab)
        self.speed_lcd.setObjectName(u"speed_lcd")

        self.horizontalLayout_8.addWidget(self.speed_lcd)


        self.verticalLayout_5.addLayout(self.horizontalLayout_8)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.Speed_2 = QLabel(self.tab)
        self.Speed_2.setObjectName(u"Speed_2")

        self.horizontalLayout_7.addWidget(self.Speed_2)

        self.path_to_map = QLineEdit(self.tab)
        self.path_to_map.setObjectName(u"path_to_map")

        self.horizontalLayout_7.addWidget(self.path_to_map)

        self.Auto_map_chk = QCheckBox(self.tab)
        self.Auto_map_chk.setObjectName(u"Auto_map_chk")

        self.horizontalLayout_7.addWidget(self.Auto_map_chk)


        self.verticalLayout_5.addLayout(self.horizontalLayout_7)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.log_data_btn = QPushButton(self.tab)
        self.log_data_btn.setObjectName(u"log_data_btn")

        self.horizontalLayout_4.addWidget(self.log_data_btn)

        self.save_map_btn = QPushButton(self.tab)
        self.save_map_btn.setObjectName(u"save_map_btn")

        self.horizontalLayout_4.addWidget(self.save_map_btn)

        self.traj_btn = QPushButton(self.tab)
        self.traj_btn.setObjectName(u"traj_btn")

        self.horizontalLayout_4.addWidget(self.traj_btn)

        self.label_2 = QLabel(self.tab)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setTextFormat(Qt.MarkdownText)
        self.label_2.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.horizontalLayout_4.addWidget(self.label_2)

        self.speed_real = QLCDNumber(self.tab)
        self.speed_real.setObjectName(u"speed_real")

        self.horizontalLayout_4.addWidget(self.speed_real)


        self.verticalLayout_5.addLayout(self.horizontalLayout_4)

        self.map_display = QGraphicsView(self.tab)
        self.map_display.setObjectName(u"map_display")

        self.verticalLayout_5.addWidget(self.map_display)

        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.custom_btn_1 = QPushButton(self.tab)
        self.custom_btn_1.setObjectName(u"custom_btn_1")

        self.horizontalLayout_9.addWidget(self.custom_btn_1)

        self.custom_btn_2 = QPushButton(self.tab)
        self.custom_btn_2.setObjectName(u"custom_btn_2")

        self.horizontalLayout_9.addWidget(self.custom_btn_2)

        self.custom_btn_3 = QPushButton(self.tab)
        self.custom_btn_3.setObjectName(u"custom_btn_3")

        self.horizontalLayout_9.addWidget(self.custom_btn_3)

        self.custom_btn_4 = QPushButton(self.tab)
        self.custom_btn_4.setObjectName(u"custom_btn_4")

        self.horizontalLayout_9.addWidget(self.custom_btn_4)

        self.custom_btn_5 = QPushButton(self.tab)
        self.custom_btn_5.setObjectName(u"custom_btn_5")

        self.horizontalLayout_9.addWidget(self.custom_btn_5)


        self.verticalLayout_5.addLayout(self.horizontalLayout_9)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")

        self.verticalLayout_5.addLayout(self.verticalLayout_4)


        self.gridLayout.addLayout(self.verticalLayout_5, 10, 0, 1, 1)

        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.verticalLayout_8 = QVBoxLayout(self.tab_2)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.groupBox = QGroupBox(self.tab_2)
        self.groupBox.setObjectName(u"groupBox")
        self.verticalLayout_7 = QVBoxLayout(self.groupBox)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.gridLayout_3.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.gridLayout_3.setHorizontalSpacing(6)
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")

        self.combined_ida_and_brute = QCheckBox(self.groupBox)
        self.combined_ida_and_brute.setObjectName(u"combined_ida_and_brute")

        self.verticalLayout_2.addWidget(self.combined_ida_and_brute)

        self.tree_selector = QComboBox(self.groupBox)
        self.tree_selector.setObjectName(u"tree_selector")

        self.verticalLayout_2.addWidget(self.tree_selector)

        self.shape_selector = QComboBox(self.groupBox)
        self.shape_selector.setObjectName(u"shape_selector")

        self.shape_selector.setEnabled(True)

        self.verticalLayout_2.addWidget(self.shape_selector)


        self.gridLayout_3.addLayout(self.verticalLayout_2, 1, 2, 1, 1)

        self.virtual_obj_btn = QPushButton(self.groupBox)
        self.virtual_obj_btn.setObjectName(u"virtual_obj_btn")
        self.virtual_obj_btn.setMinimumSize(QSize(190, 90))

        self.gridLayout_3.addWidget(self.virtual_obj_btn, 1, 0, 1, 1)

        self.Test_btn = QPushButton(self.groupBox)
        self.Test_btn.setObjectName(u"Test_btn")
        self.Test_btn.setMinimumSize(QSize(190, 90))

        self.gridLayout_3.addWidget(self.Test_btn, 0, 1, 1, 1)

        self.stop_train_test = QPushButton(self.groupBox)
        self.stop_train_test.setObjectName(u"stop_train_test")
        self.stop_train_test.setMinimumSize(QSize(190, 90))

        self.gridLayout_3.addWidget(self.stop_train_test, 0, 2, 1, 1)

        self.close_window_btn = QPushButton(self.groupBox)
        self.close_window_btn.setObjectName(u"close_window_btn")
        self.close_window_btn.setMinimumSize(QSize(190, 90))

        self.gridLayout_3.addWidget(self.close_window_btn, 1, 1, 1, 1)

        self.Train = QPushButton(self.groupBox)
        self.Train.setObjectName(u"Train")
        self.Train.setMinimumSize(QSize(190, 90))
        self.Train.setIconSize(QSize(16, 16))
        self.Train.setFlat(False)

        self.gridLayout_3.addWidget(self.Train, 0, 0, 1, 1)


        self.horizontalLayout.addLayout(self.gridLayout_3)

        self.info_box_rl = QTextEdit(self.groupBox)
        self.info_box_rl.setObjectName(u"info_box_rl")
        self.info_box_rl.setFont(font1)
        self.info_box_rl.setAcceptDrops(True)
        self.info_box_rl.setReadOnly(True)

        self.horizontalLayout.addWidget(self.info_box_rl)


        self.verticalLayout_7.addLayout(self.horizontalLayout)


        self.verticalLayout_8.addWidget(self.groupBox)

        self.groupBox_2 = QGroupBox(self.tab_2)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.groupBox_2.setAutoFillBackground(False)
        self.verticalLayout_6 = QVBoxLayout(self.groupBox_2)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.ML_train = QPushButton(self.groupBox_2)
        self.ML_train.setObjectName(u"ML_train")

        self.horizontalLayout_2.addWidget(self.ML_train)

        self.ML_test = QPushButton(self.groupBox_2)
        self.ML_test.setObjectName(u"ML_test")

        self.horizontalLayout_2.addWidget(self.ML_test)

        self.ML_autolabel = QPushButton(self.groupBox_2)
        self.ML_autolabel.setObjectName(u"ML_autolabel")

        self.horizontalLayout_2.addWidget(self.ML_autolabel)

        self.ml_algo_selector = QComboBox(self.groupBox_2)
        self.ml_algo_selector.setObjectName(u"ml_algo_selector")

        self.horizontalLayout_2.addWidget(self.ml_algo_selector)


        self.verticalLayout_3.addLayout(self.horizontalLayout_2)

        self.info_box_ml = QTextEdit(self.groupBox_2)
        self.info_box_ml.setObjectName(u"info_box_ml")
        self.info_box_ml.setFont(font1)
        self.info_box_ml.setAcceptDrops(True)
        self.info_box_ml.setReadOnly(True)

        self.verticalLayout_3.addWidget(self.info_box_ml)


        self.verticalLayout_6.addLayout(self.verticalLayout_3)


        self.verticalLayout_8.addWidget(self.groupBox_2)

        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.tabWidget.addTab(self.tab_3, "")
        self.tab_4 = QWidget()
        self.tab_4.setObjectName(u"tab_4")
        self.gridLayout_6 = QGridLayout(self.tab_4)
        self.gridLayout_6.setObjectName(u"gridLayout_6")
        self.verticalLayout_13 = QVBoxLayout()
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.checkBox = QCheckBox(self.tab_4)
        self.checkBox.setObjectName(u"checkBox")

        self.verticalLayout_13.addWidget(self.checkBox)

        self.checkBox_2 = QCheckBox(self.tab_4)
        self.checkBox_2.setObjectName(u"checkBox_2")

        self.verticalLayout_13.addWidget(self.checkBox_2)

        self.custom_ml = QPushButton(self.tab_4)
        self.custom_ml.setObjectName(u"custom_ml")

        self.verticalLayout_13.addWidget(self.custom_ml)


        self.gridLayout_6.addLayout(self.verticalLayout_13, 0, 0, 1, 1)

        self.tabWidget.addTab(self.tab_4, "")
        self.tab_5 = QWidget()
        self.tab_5.setObjectName(u"tab_5")
        self.tabWidget.addTab(self.tab_5, "")
        self.tab_6 = QWidget()
        self.tab_6.setObjectName(u"tab_6")
        self.tabWidget.addTab(self.tab_6, "")
        self.tab_7 = QWidget()
        self.tab_7.setObjectName(u"tab_7")
        self.tabWidget.addTab(self.tab_7, "")

        self.verticalLayout.addWidget(self.tabWidget)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 916, 22))

        self.menuFile = QMenu(self.menubar)
        self.menuFile.setObjectName(u"menuFile")
        self.menuEdit = QMenu(self.menubar)
        self.menuEdit.setObjectName(u"menuEdit")
        self.menuView = QMenu(self.menubar)
        self.menuView.setObjectName(u"menuView")
        self.menuHelp = QMenu(self.menubar)
        self.menuHelp.setObjectName(u"menuHelp")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuView.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())
        self.menuFile.addAction(self.actionOpen_Map)
        self.menuFile.addAction(self.actionSave_Map)
        self.menuEdit.addAction(self.actionUndo)
        self.menuEdit.addAction(self.actionRedo_Ctrl_Y)
        self.menuEdit.addSeparator()
        self.menuEdit.addAction(self.actionCut_Ctrl_X)
        self.menuEdit.addAction(self.actionCopy_Ctrl_C)
        self.menuEdit.addAction(self.actionPaste_Ctrl_V)
        self.menuHelp.addAction(self.actionInfo)
        self.menuHelp.addAction(self.actionTutorial)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.actionOpen_Map.setText(QCoreApplication.translate("MainWindow", u"Open Map", None))
#if QT_CONFIG(shortcut)
        self.actionOpen_Map.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+O", None))
#endif // QT_CONFIG(shortcut)
        self.actionSave_Map.setText(QCoreApplication.translate("MainWindow", u"Save Map", None))
#if QT_CONFIG(shortcut)
        self.actionSave_Map.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+S", None))
#endif // QT_CONFIG(shortcut)
        self.actionUndo.setText(QCoreApplication.translate("MainWindow", u"Undo    Ctrl+Z", None))
        self.actionRedo_Ctrl_Y.setText(QCoreApplication.translate("MainWindow", u"Redo    Ctrl+Y", None))
        self.actionCut_Ctrl_X.setText(QCoreApplication.translate("MainWindow", u"Cut       Ctrl+X", None))
        self.actionCopy_Ctrl_C.setText(QCoreApplication.translate("MainWindow", u"Copy    Ctrl+C", None))
        self.actionPaste_Ctrl_V.setText(QCoreApplication.translate("MainWindow", u"Paste   Ctrl+V", None))
        self.actionInfo.setText(QCoreApplication.translate("MainWindow", u"Info", None))
        self.actionTutorial.setText(QCoreApplication.translate("MainWindow", u"Tutorial", None))
        self.logo_ias.setText("")
        self.Speed_3.setText(QCoreApplication.translate("MainWindow", u"0.01", None))
        self.Speed_4.setText(QCoreApplication.translate("MainWindow", u"0.15", None))
        self.Backward.setText(QCoreApplication.translate("MainWindow", u"Backward(s)", None))
        self.Left.setText(QCoreApplication.translate("MainWindow", u"Left(a)", None))
        self.Foward.setText(QCoreApplication.translate("MainWindow", u"Forward(w)", None))
        self.Break.setText(QCoreApplication.translate("MainWindow", u"Stop(x)", None))
        self.Right.setText(QCoreApplication.translate("MainWindow", u"Right(d)", None))
        self.use_keyboard_chk.setText(QCoreApplication.translate("MainWindow", u"Use Keyboard", None))
        self.on_off_btn.setText(QCoreApplication.translate("MainWindow", u"ON/OFF", None))
        self.start_rob_btn.setText(QCoreApplication.translate("MainWindow", u"Start Robot", None))
        self.gen_map_btn.setText(QCoreApplication.translate("MainWindow", u"Generate Map", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"**Speed Dial:**", None))
        self.Speed_2.setText(QCoreApplication.translate("MainWindow", u"Map Path:", None))
        self.Auto_map_chk.setText(QCoreApplication.translate("MainWindow", u"Auto Map ", None))
        self.log_data_btn.setText(QCoreApplication.translate("MainWindow", u"Log Data", None))
        self.save_map_btn.setText(QCoreApplication.translate("MainWindow", u"Save Map", None))
        self.traj_btn.setText(QCoreApplication.translate("MainWindow", u"Trajectory", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"**Robotino Speed:**", None))
        self.custom_btn_1.setText(QCoreApplication.translate("MainWindow", u"Custom 1", None))
        self.custom_btn_2.setText(QCoreApplication.translate("MainWindow", u"Custom 2", None))
        self.custom_btn_3.setText(QCoreApplication.translate("MainWindow", u"Custom 3", None))
        self.custom_btn_4.setText(QCoreApplication.translate("MainWindow", u"Custom 4", None))
        self.custom_btn_5.setText(QCoreApplication.translate("MainWindow", u"Custom 5", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Mapper", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"Reinforcment Learning", None))
        self.combined_ida_and_brute.setText(QCoreApplication.translate("MainWindow", u"Combined Evaluation", None))

        self.virtual_obj_btn.setText(QCoreApplication.translate("MainWindow", u"Virtual Objects", None))
        self.Test_btn.setText(QCoreApplication.translate("MainWindow", u"Test", None))
        self.stop_train_test.setText(QCoreApplication.translate("MainWindow", u"Stop RL", None))
        self.close_window_btn.setText(QCoreApplication.translate("MainWindow", u"Close Windows", None))
        self.Train.setText(QCoreApplication.translate("MainWindow", u"Train", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("MainWindow", u"Machine Learning", None))
        self.ML_train.setText(QCoreApplication.translate("MainWindow", u"Train", None))
        self.ML_test.setText(QCoreApplication.translate("MainWindow", u"Test", None))
        self.ML_autolabel.setText(QCoreApplication.translate("MainWindow", u"Auto Label", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Localization Adversary", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), QCoreApplication.translate("MainWindow", u"Object Adversary", None))
        self.checkBox.setText(QCoreApplication.translate("MainWindow", u"Localization", None))
        self.checkBox_2.setText(QCoreApplication.translate("MainWindow", u"Object", None))
        self.custom_ml.setText(QCoreApplication.translate("MainWindow", u"Start", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), QCoreApplication.translate("MainWindow", u"ML Control", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_5), QCoreApplication.translate("MainWindow", u"Monitored space localization", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_6), QCoreApplication.translate("MainWindow", u"Monitored space object", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_7), QCoreApplication.translate("MainWindow", u"Evaluation", None))
        self.menuFile.setTitle(QCoreApplication.translate("MainWindow", u"File", None))
        self.menuEdit.setTitle(QCoreApplication.translate("MainWindow", u"Edit", None))
        self.menuView.setTitle(QCoreApplication.translate("MainWindow", u"View", None))
        self.menuHelp.setTitle(QCoreApplication.translate("MainWindow", u"Help", None))
    # retranslateUi

