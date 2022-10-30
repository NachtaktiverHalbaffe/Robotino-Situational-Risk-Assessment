from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys

xpos = 200
ypos = 200
width = 300
height = 300

def window():
    app = QApplication(sys.argv)
    win = QMainWindow()
    win.setGeometry(xpos, ypos, width, height)
    win.setWindowTitle("Mapper")

    label = QtWidgets.QLabel(win)
    label.setText("My label")
    label.move(50, 50)
    win.show()
    sys.exit(app.exec_())

window()