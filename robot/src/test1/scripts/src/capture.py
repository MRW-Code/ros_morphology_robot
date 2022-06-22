import sys
import cv2
from threading import Thread, Lock
import threading
import time
import serial
import serial.tools.list_ports
import string

from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QSpinBox
from PyQt5.QtWidgets import QDoubleSpinBox
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtCore import QThread
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import Qt
import src.config as config


class QtCapture(QWidget):
    def __init__(self, *args):
        super(QWidget, self).__init__()
        self.imgDirName = ''
        self.fps = 24

        ## I CHANGED THIS TO GET VIDEO WORKING!!!!
        # self.cap = cv2.VideoCapture(*args, cv2.CAP_DSHOW)

        ## HARD CODED THE CAMER ID AS GUI DIDNT WORK?
        self.cap = cv2.VideoCapture(0)

        self.readyToSave = False
        self.imgFileName = ''
        self.isCameraOpen = False

        if self.cap.isOpened():
            self.isCameraOpen = True

        self.video_frame = QLabel()
        lay = QVBoxLayout()
        lay.setContentsMargins(10, 10, 10, 10)
        lay.addWidget(self.video_frame)
        self.setLayout(lay)
        self.setWindowTitle('Camera')
        self.setGeometry(500, 200, 400, 400)

        # ------ Modification ------ #
        self.isCapturing = False
        self.ith_frame = 1
        # ------ Modification ------ #

    def setFPS(self, fps):
        self.fps = fps

    def nextFrameSlot(self):
        if self.isCameraOpen:
            ret, frame = self.cap.read()

            # global pictureSaved
            # global taskReady

            if config.taskReady:
                self.ith_frame = 1

                # ------ Modification ------ #
            # Save images if isCapturing
            if self.isCapturing:
                cv2.imwrite('img_%05d.jpg' % self.ith_frame, frame)
                self.ith_frame += 1
            # ------ Modification ------ #

            if self.readyToSave:
                config.pictureSaved = False
                print("saving")
                # cv2.imwrite('./20-07-21/img_' + self.imgFileName +'%05d.jpg'%self.ith_frame, frame)
                # cv2.imwrite('./' + self.imgFileName + '/' + '%05d.jpg' % self.ith_frame, frame)
                
                ## CAMERA SAVE PATH WASNT WORKING SO HARD CODED IT
                cv2.imwrite('./src/fileName' + '/' + '%05d.jpg' % self.ith_frame, frame)

                #                cv2.imwrite('img_%05d.jpg'%self.ith_frame, frame)
                time.sleep(0.2)
                self.ith_frame += 1
                config.pictureSaved = True
                time.sleep(0.1)

            # My webcam yields frames in BGR format
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            pix = QPixmap.fromImage(img)
            self.video_frame.setPixmap(pix)

    def start(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.nextFrameSlot)
        self.timer.start(int(1000 / self.fps))

    def stop(self):
        self.timer.stop()

    # ------ Modification ------ #
    def capture(self):
        pass

    #        if not self.isCapturing:
    #            self.isCapturing = True
    #        else:
    #            self.isCapturing = False
    # ------ Modification ------ #

    def deleteLater(self):
        if self.isCameraOpen:
            self.cap.release()
            self.isCameraOpen = False
            super(QWidget, self).deleteLater()

    def setFlag(self, val):
        self.readyToSave = val

    def getCameraStatus(self):
        return self.isCameraOpen

    def closeEvent(self, event):
        self.cap.release()
        self.isCameraOpen = False
        super(QWidget, self).deleteLater()
        self.close()

    def setFileName(self, fname):
        self.imgFileName = fname
        print(self.imgFileName)
