# This Python file uses the following encoding: utf-8
import sys

from PySide6.QtWidgets import QApplication, QMainWindow

# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py
from gui.RL_application.ui_form import Ui_MainWindow
from PySide6.QtCore import QThread, Signal   
import cv2
import config
import numpy as np
# sys.path.append("map_path': '/home/abdul/MasterThesis/Commisioning/Yannik/Stephan_Master_IAS")
from main import mains
indexs = 0

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.thread = {}
        self.corners = []

        self.ui.shape_selector.addItem("circle")
        self.ui.shape_selector.addItem("rectangle")
        self.ui.shape_selector.addItem("polygon")

        self.ui.Train.clicked.connect(lambda: self.start_worker(mode=False))
        self.ui.stop_train_test.clicked.connect(lambda: self.stop_worker())
        self.ui.Test_btn.clicked.connect(lambda: self.start_worker(mode=True))
        #self.ui.stop_train_test.clicked.connect(lambda: self.stop_worker_2())
        #self.ui.Control_btn.clicked.connect(lambda: self.test_fun())

        self.ui.virtual_obj_btn.clicked.connect(lambda: self.create_geofence())
        self.ui.close_btn.clicked.connect(lambda: self.destroy_windows())

    def train_test(self, done=False):
        #print("Test mode: ")
        indexs = self.sender().index
        if done:
            #print(self.thread[1].is_running)
            #print(indexs)
            self.ui.Train.setEnabled(True)
            self.ui.Test_btn.setEnabled(True)
            self.stop_worker()
        
        #mains(mode=mode)

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
        cv2.imshow("Original", map)
        cv2.imshow("Geo Image", image)
        cv2.imshow("resized", resized)

    def destroy_windows(self):
        cv2.destroyAllWindows()



        

    # def start_worker_2(self):
    #     self.thread[2] = ThreadClass(parent=None, mode=True, index=2)
    #     self.thread[2].start()
    #     self.thread[2].any_signal.connect(self.train_test)
    #     self.ui.Train.setEnabled(False)
    #     self.ui.Test_btn.setEnabled(False)

    def start_worker(self, mode=False):
        print(mode)
        self.thread[1] = ThreadClass(parent=None, mode=mode, index=1)
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

    # def stop_worker_2(self):
    #     self.thread[2].stop()
    #     self.ui.Train.setEnabled(True)
    #     self.ui.Test_btn.setEnabled(True)
    
class ThreadClass(QThread):
    any_signal = Signal(int)
    def __init__(self, parent=None, mode=False, index=0):
        super(ThreadClass,self).__init__(parent)
        self.mode = mode
        self.is_running = True
        self.index = index
    
    def run(self):
        print('Starting Thread...', self.mode)
        #cnt=0
        done = mains(mode=self.mode)
        self.is_running=False
        if done:
            self.any_signal.emit(done)
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
    #mains()
    sys.exit(app.exec())
