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

import rospy
from std_msgs.msg import Bool, String


class explorationThread(QThread):
    explorationUpdate = pyqtSignal(bool)

    def __init__(self, myvar, parent=None):
        QThread.__init__(self, parent)
        self.myvar = myvar
        self.shall_i_move = None

        self.sub_thread = Thread(target=self.create_sub)
        self.sub_thread.start()

        # self.move_sub = rospy.Subscriber('ready_move', Bool, self.callback)
        # rospy.spin()

    def callback(self, data):
        # self.shall_i_move = data.data
        # print(self.shall_i_move)
        print('hi')

    def create_sub(self):
        self.move_sub = rospy.Subscriber('ready_move', Bool, self.callback)
        # rospy.spin()


    def run(self):
        self.ThreadActive = True
        self.speed_explore = 'F200\n'


        if self.ThreadActive:
            config.taskReady = False

            if self.myvar.arduino.isOpen():
                if self.myvar.systemCalibrate:

                    ## YOU CHANGED THIS ALSO CHECK WITH URIEL
                    # initXPosition = self.myvar.spinInitXPosition.value()
                    # initYPosition = self.myvar.spinInitYPosition.value()
                    # initZPosition = self.myvar.spinInitZPosition.value()

                    print("HOME")
                    self.myvar.arduino.write(bytes('G90 G0 X0 Y0\n', 'utf-8'))
                    time.sleep(0.1)

                    print("Start exploration process")
                    w2wDistance = self.myvar.spinStepDistance.value() / 100.0
                    dirXMovement = -1;  # positive X
                    for i in range(0, self.myvar.spinRowsWells.value()):
                        for j in range(0, self.myvar.spinColsWells.value() - 1):

                            config.pictureSaved = False

                            self.explorationUpdate.emit(True)
                            time.sleep(0.1)
                            while config.pictureSaved == False:
                                pass
                            print("figure stored" + str(i + 1) + "," + str(j + 1))
                            self.explorationUpdate.emit(False)
                            config.pictureSaved = False
                            time.sleep(0.5)

                            print('@MOVRX' + str(dirXMovement * w2wDistance) + self.speed_explore)

                            self.myvar.arduino.write(
                                bytes('$J=G21G91X' + str(dirXMovement * w2wDistance) + self.speed_explore,
                                      'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions

                            time.sleep(5.0)

                            if not self.ThreadActive:
                                break
                            print('--')

                        time.sleep(0.8)

                        # invert direction of movement in X axis
                        dirXMovement = dirXMovement * -1;

                        print('@MOVRY- ' + str(w2wDistance) + self.speed_explore)
                        self.myvar.arduino.write(bytes('$J=G21G91Y-' + str(w2wDistance) + self.speed_explore,
                                                       'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions

                        config.pictureSaved = False

                        self.explorationUpdate.emit(True)
                        time.sleep(0.10)
                        while config.pictureSaved == False:
                            pass
                        print("figure stored" + str(i + 1) + "," + str(j + 1))
                        self.explorationUpdate.emit(False)
                        config.pictureSaved = False
                        time.sleep(5)

                        if not self.ThreadActive:
                            break
                        print('--')

                    print("HOME")
                    self.myvar.arduino.write(bytes('G90 G0 X0 Y0\n', 'utf-8'))
                    time.sleep(0.10)

                    print("Exploration process done")
                else:
                    print("Calibration is needed")
            else:
                print("Port is not opened")

            config.taskReady = True
            print('---')

    def stop(self):
        print('Exploration STOPPED')
        self.ThreadActive = False
        self.sub_thread.join(0)
        self.quit()

    def flagStatus(self):
        pass