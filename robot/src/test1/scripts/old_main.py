# -*- coding: utf-8 -*-
"""
Created on Wed Jun  9 22:42:23 2021

@author: Uriel Martinez-Hernandez
date: 09-07-2021

University of Bath
Multimodal inte-R-action lab
Centre for Autonomous Robotics (CENTAUR)

High Throughput Ssystem (HTS) driver version 0.1
"""

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

pictureSaved = None
taskReady = None


class explorationThread(QThread):
    explorationUpdate = pyqtSignal(bool)

    def __init__(self, myvar, parent=None):
        QThread.__init__(self, parent)
        self.myvar = myvar

    def run(self):
        self.ThreadActive = True
        self.speed_explore = 'F200\n'

        global pictureSaved
        global taskReady

        if self.ThreadActive:
            taskReady = False

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

                            pictureSaved = False

                            self.explorationUpdate.emit(True)
                            time.sleep(0.1)
                            while pictureSaved == False:
                                pass
                            print("figure stored" + str(i + 1) + "," + str(j + 1))
                            self.explorationUpdate.emit(False)
                            pictureSaved = False
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

                        pictureSaved = False

                        self.explorationUpdate.emit(True)
                        time.sleep(0.10)
                        while pictureSaved == False:
                            pass
                        print("figure stored" + str(i + 1) + "," + str(j + 1))
                        self.explorationUpdate.emit(False)
                        pictureSaved = False
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

            taskReady = True
            print('---')

    def stop(self):
        print('Exploration STOPPED')
        self.ThreadActive = False
        self.quit()

    def flagStatus(self):
        pass


class QtCapture(QWidget):
    def __init__(self, *args):
        super(QWidget, self).__init__()
        self.imgDirName = ''
        self.fps = 24

        ## I CHANGED THIS TO GET VIDEO WORKING!!!!
        # self.cap = cv2.VideoCapture(*args, cv2.CAP_DSHOW)
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

            global pictureSaved
            global taskReady

            if taskReady:
                self.ith_frame = 1

                # ------ Modification ------ #
            # Save images if isCapturing
            if self.isCapturing:
                cv2.imwrite('img_%05d.jpg' % self.ith_frame, frame)
                self.ith_frame += 1
            # ------ Modification ------ #

            if self.readyToSave:
                pictureSaved = False
                print("saving")
                # cv2.imwrite('./20-07-21/img_' + self.imgFileName +'%05d.jpg'%self.ith_frame, frame)
                cv2.imwrite('./' + self.imgFileName + '/' + '%05d.jpg' % self.ith_frame, frame)

                #                cv2.imwrite('img_%05d.jpg'%self.ith_frame, frame)
                time.sleep(0.2)
                self.ith_frame += 1
                pictureSaved = True
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


class ControlWindow(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.capture = None
        self.isPictureSaved = False
        self.cameraStatus = False

        self.start_button = QPushButton('Start camera')
        self.start_button.clicked.connect(self.startCapture)
        self.pause_button = QPushButton('Stop camera')
        self.quit_button = QPushButton('Close camera')
        self.quit_button.clicked.connect(self.endCapture)

        self.portConnectStatus = False
        self.portConnect_button = QPushButton('Connect')
        self.portConnect_button.setStyleSheet("background-color: red");
        self.portConnect_button.clicked.connect(self.portConnection)
        self.portConnect_button.setEnabled(True)
        self.portDisconnect_button = QPushButton('Disconnect')
        self.portDisconnect_button.setEnabled(False)
        self.portDisconnect_button.clicked.connect(self.portConnection)

        self.arduino = serial.Serial()
        #        self.comPort_line = QLineEdit('COM3')
        self.comPort_line = QComboBox(self)

        ports = serial.tools.list_ports.comports(include_links=False)
        for port in ports:
            print(port.device)
            self.comPort_line.addItem(port.device)
        self.comPort_line.addItem('')

        self.comBaudrate_line = QLineEdit('115200')
        self.comPortRefresh_button = QPushButton('Refresh ports')
        self.comPortRefresh_button.clicked.connect(self.refreshPorts)

        self.systemCalibrate = False
        self.systemCalibrate_button = QPushButton('Calibrate system')
        self.systemCalibrate_button.clicked.connect(self.calibrateMotorSystem)
        self.systemCalibrate_label = QLabel('System NOT calibrated')
        self.systemCalibrate_label.setAlignment(Qt.AlignCenter)
        self.systemCalibrate_label.setStyleSheet("background-color: red");

        self.manualMotorControl_xpositive_button = QPushButton('+X')
        self.manualMotorControl_xpositive_button.clicked.connect(self.manualMotorControlXPositive)
        self.manualMotorControl_xnegative_button = QPushButton('-X')
        self.manualMotorControl_xnegative_button.clicked.connect(self.manualMotorControlXNegative)
        self.manualMotorControl_ypositive_button = QPushButton('+Y')
        self.manualMotorControl_ypositive_button.clicked.connect(self.manualMotorControlYPositive)
        self.manualMotorControl_ynegative_button = QPushButton('-Y')
        self.manualMotorControl_ynegative_button.clicked.connect(self.manualMotorControlYNegative)
        self.manualMotorControl_zpositive_button = QPushButton('+Z')
        self.manualMotorControl_zpositive_button.clicked.connect(self.manualMotorControlZPositive)
        self.manualMotorControl_znegative_button = QPushButton('-Z')
        self.manualMotorControl_znegative_button.clicked.connect(self.manualMotorControlZNegative)
        self.manualMotorControl_home_button = QPushButton('HOME')
        self.manualMotorControl_home_button.setEnabled(False)
        self.manualMotorControl_home_button.clicked.connect(self.manualMotorControlHOME)

        self.processControl_start_button = QPushButton('Start')
        self.processControl_start_button.setEnabled(False)
        self.processControl_start_button.clicked.connect(self.startExplorationProcess)
        self.processControl_stop_button = QPushButton('Stop')
        self.processControl_stop_button.setEnabled(False)
        self.processControl_stop_button.clicked.connect(self.stopExplorationProcess)

        self.monitoringMotorPosition_x_line = QLineEdit('0.0')
        self.monitoringMotorPosition_x_line.setReadOnly(True)
        self.monitoringMotorPosition_y_line = QLineEdit('0.0')
        self.monitoringMotorPosition_y_line.setReadOnly(True)
        self.monitoringMotorPosition_z_line = QLineEdit('0.0')
        self.monitoringMotorPosition_z_line.setReadOnly(True)
        self.monitoringMotorPosition_speed_line = QLineEdit('0.0')
        self.monitoringMotorPosition_speed_line.setReadOnly(True)
        self.monitoringMotorPosition_button = QPushButton('Get values')
        self.monitoringMotorPosition_button.clicked.connect(self.getMonitorSystemValues)

        # ------ Modification ------ #
        # self.capture_button = QPushButton('Capture')
        # self.capture_button.clicked.connect(self.saveCapture)
        # ------ Modification ------ #

        outerLayout = QHBoxLayout(self)

        portLayout = QVBoxLayout(self)
        leftLayout = QVBoxLayout(self)
        middleLayout = QVBoxLayout(self)
        rightLayout = QVBoxLayout(self)

        layoutComPort = QVBoxLayout()
        communicationTitle = QLabel('<h3>Communication</h3>')
        communicationTitle.setAlignment(Qt.AlignCenter)
        layoutComPort.addWidget(communicationTitle)
        #        layoutComPort.addWidget(QLabel('<h3>Communication</h3>'))
        layoutComPortParams = QFormLayout()
        layoutComPortParams.addRow("Port: ", self.comPort_line)
        layoutComPortParams.addRow("Baudrate: ", self.comBaudrate_line)
        layoutComPortParams.addRow(self.comPortRefresh_button)
        layoutComPort.addLayout(layoutComPortParams)
        layoutComPort.addWidget(self.portConnect_button)
        layoutComPort.addWidget(self.portDisconnect_button)

        layoutCameraControl = QVBoxLayout()
        CameraControlTitle = QLabel('<h3>Camera control</h3>')
        CameraControlTitle.setAlignment(Qt.AlignCenter)
        layoutCameraControl.addWidget(CameraControlTitle)
        #        layoutCameraControl.addWidget(QLabel('<h3>Camera control</h3>'))
        layoutCameraControlButtons = QHBoxLayout()
        layoutCameraControlButtons.addWidget(self.start_button)
        layoutCameraControlButtons.addWidget(self.pause_button)
        layoutCameraControlButtons.addWidget(self.quit_button)
        self.spinCameraID = QSpinBox()
        self.spinCameraID.setMinimum(0)
        self.spinCameraID.setMaximum(2)
        layoutCameraControlButtons.addWidget(QLabel('Camera ID'))
        layoutCameraControlButtons.addWidget(self.spinCameraID)
        layoutCameraControl.addLayout(layoutCameraControlButtons)

        layoutSystemCalibration = QVBoxLayout()
        manualMotorControlTitle = QLabel('<br><h3>Manual motor control</h3>')
        manualMotorControlTitle.setAlignment(Qt.AlignCenter)
        layoutSystemCalibration.addWidget(manualMotorControlTitle)
        #        layoutSystemCalibration.addWidget(QLabel('<br><h3>Manual motor control</h3>'))
        layoutCalibrationButtons = QHBoxLayout()
        layoutCalibrationButtons.addWidget(self.systemCalibrate_button)
        layoutCalibrationButtons.addWidget(self.systemCalibrate_label)
        layoutSystemCalibration.addLayout(layoutCalibrationButtons)
        layoutspinManualStepSize = QHBoxLayout()
        self.spinManualStepSize = QDoubleSpinBox()
        self.spinManualStepSize.setMinimum(0.0)
        self.spinManualStepSize.setMaximum(100.0)
        layoutspinManualStepSize.addWidget(QLabel('Step size:'))
        layoutspinManualStepSize.addWidget(self.spinManualStepSize)
        layoutSystemCalibration.addLayout(layoutspinManualStepSize)
        layoutMotorControlButtons = QGridLayout()
        layoutMotorControlButtons.addWidget(self.manualMotorControl_xpositive_button, 0, 0)
        layoutMotorControlButtons.addWidget(self.manualMotorControl_xnegative_button, 0, 1)
        layoutMotorControlButtons.addWidget(self.manualMotorControl_ypositive_button, 1, 0)
        layoutMotorControlButtons.addWidget(self.manualMotorControl_ynegative_button, 1, 1)
        layoutMotorControlButtons.addWidget(self.manualMotorControl_zpositive_button, 2, 0)
        layoutMotorControlButtons.addWidget(self.manualMotorControl_znegative_button, 2, 1)
        layoutMotorControlButtons.addWidget(self.manualMotorControl_home_button, 3, 0, 1, 2)
        layoutSystemCalibration.addLayout(layoutMotorControlButtons)
        #        layoutInfoMotorPositions = QFormLayout()
        #        layoutInfoMotorPositions.addRow("Current motor X: ", self.monitoringMotorPosition_x_line)
        #        layoutInfoMotorPositions.addRow("Current motor Y: ", self.monitoringMotorPosition_y_line)
        #        layoutInfoMotorPositions.addRow("Current motor Z: ", self.monitoringMotorPosition_z_line)
        #        layoutInfoMotorPositions.addRow("Current motor speed: ", self.monitoringMotorPosition_speed_line)
        #        layoutInfoMotorPositions.addRow(self.monitoringMotorPosition_button)
        #        layoutSystemCalibration.addLayout(layoutInfoMotorPositions)

        layoutPatternControl = QVBoxLayout()
        explorationParametersTitle = QLabel('<h3>Exploration parameters</h3>')
        explorationParametersTitle.setAlignment(Qt.AlignCenter)
        layoutPatternControl.addWidget(explorationParametersTitle)
        #        layoutPatternControl.addWidget(QLabel('<h3>Exploration parameters</h3>'))
        #        layoutInitMotorPosition = QFormLayout()
        #        self.spinInitXPosition = QSpinBox()
        #        self.spinInitXPosition.setMinimum(0)
        #        self.spinInitXPosition.setMaximum(200)
        #        self.spinInitYPosition = QSpinBox()
        #        self.spinInitYPosition.setMinimum(0)
        #        self.spinInitYPosition.setMaximum(200)
        #        self.spinInitZPosition = QSpinBox()
        #        self.spinInitZPosition.setMinimum(0)
        #        self.spinInitZPosition.setMaximum(200)
        #        self.spinStepDistance = QDoubleSpinBox()
        #        self.spinStepDistance.setMinimum(0.0)
        #        self.spinStepDistance.setMaximum(100.0)
        #        self.filaname_picture_line = QLineEdit('fileName')
        #        layoutInitMotorPosition.addWidget(QLabel('File name: '))
        #        layoutInitMotorPosition.addWidget(self.filaname_picture_line)
        #        layoutInitMotorPosition.addWidget(QLabel('Init X:'))
        #        layoutInitMotorPosition.addWidget(self.spinInitXPosition)
        #        layoutInitMotorPosition.addWidget(QLabel('Init Y:'))
        #        layoutInitMotorPosition.addWidget(self.spinInitYPosition)
        #        layoutInitMotorPosition.addWidget(QLabel('Init Z:'))
        #        layoutInitMotorPosition.addWidget(self.spinInitZPosition)
        #        layoutInitMotorPosition.addWidget(QLabel('Distance (W2W):'))
        #        layoutInitMotorPosition.addWidget(self.spinStepDistance)
        #        layoutPatternControl.addLayout(layoutInitMotorPosition)

        layoutExplorationInfo = QHBoxLayout()
        self.filaname_picture_line = QLineEdit('fileName')
        layoutExplorationInfo.addWidget(QLabel('File name: '))
        layoutExplorationInfo.addWidget(self.filaname_picture_line)
        self.spinStepDistance = QDoubleSpinBox()
        self.spinStepDistance.setMinimum(0.0)
        self.spinStepDistance.setMaximum(100.0)
        layoutExplorationInfo.addWidget(QLabel('Step size:'))
        layoutExplorationInfo.addWidget(self.spinStepDistance)
        layoutRowsColsWells = QHBoxLayout()
        self.spinRowsWells = QSpinBox()
        self.spinRowsWells.setMinimum(0)
        self.spinRowsWells.setMaximum(20)
        layoutRowsColsWells.addWidget(QLabel('Rows (X axis)'))
        layoutRowsColsWells.addWidget(self.spinRowsWells)
        self.spinColsWells = QSpinBox()
        self.spinColsWells.setMinimum(0)
        self.spinColsWells.setMaximum(20)
        layoutRowsColsWells.addWidget(QLabel('Cols (Y axis)'))
        layoutRowsColsWells.addWidget(self.spinColsWells)
        layoutPatternControl.addLayout(layoutExplorationInfo)
        layoutPatternControl.addLayout(layoutRowsColsWells)
        #        layoutPatternControl.addWidget(QPushButton('Set control parameters'))

        layoutProcessControl = QVBoxLayout()
        processControlTitle = QLabel('<br><h3>Process control</h3>')
        processControlTitle.setAlignment(Qt.AlignCenter)
        layoutProcessControl.addWidget(processControlTitle)
        #        layoutProcessControl.addWidget(QLabel('<br><h3>Process control</h3>'))
        layoutProcessControlButtons = QHBoxLayout()

        self.exit_button = QPushButton('Exit')
        self.exit_button.clicked.connect(self.quitSystem)

        layoutProcessControlButtons.addWidget(self.processControl_start_button)
        layoutProcessControlButtons.addWidget(self.processControl_stop_button)
        layoutProcessControlButtons.addWidget(self.exit_button)
        layoutProcessControl.addLayout(layoutProcessControlButtons)

        self.explorationThread = explorationThread(myvar=self)

        # ------ Modification ------ #
        # vbox.addWidget(self.capture_button)
        # ------ Modification ------ #

        portLayout.addLayout(layoutComPort)
        leftLayout.addLayout(layoutCameraControl)
        leftLayout.addLayout(layoutSystemCalibration)
        middleLayout.addLayout(layoutPatternControl)
        middleLayout.addLayout(layoutProcessControl)

        outerLayout.addLayout(portLayout)
        outerLayout.addLayout(leftLayout)
        outerLayout.addLayout(middleLayout)
        #        outerLayout.addLayout(rightLayout)

        #        self.setLayout(vbox)
        self.setLayout(outerLayout)
        self.setWindowTitle('High Throughput System - v0.1')
        self.setGeometry(200, 200, 600, 300)
        self.show()

        # some variables
        self.speed_rel = 'F50\n'

    def getMonitorSystemValues(self):
        if self.arduino.isOpen():
            print('@GETALLPOS\r')
        else:
            print("Port is not opened")

    def startExplorationProcess(self):
        self.capture.setFileName(self.filaname_picture_line.text())
        self.explorationThread.start()
        self.explorationThread.explorationUpdate.connect(self.capture.setFlag)

    def stopExplorationProcess(self):
        self.explorationThread.stop()

    def closeEvent(self, event):
        super(ControlWindow, self).closeEvent(event)
        self.explorationThread.stop()
        self.arduino.close()
        if self.cameraStatus:
            self.cameraStatus = self.capture.getCameraStatus()
            self.capture.deleteLater()

        self.close()

    def manualMotorControlXPositive(self):
        if self.arduino.isOpen():
            print("+X")
            self.arduino.write(bytes('$J=G21G91X' + self.spinManualStepSize.textFromValue(
                self.spinManualStepSize.value() / 100.0) + self.speed_rel,
                                     'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions
            time.sleep(0.10)
        else:
            print("Port is not opened")

    def manualMotorControlXNegative(self):
        if self.arduino.isOpen():
            print("-X")
            self.arduino.write(bytes('$J=G21G91X-' + self.spinManualStepSize.textFromValue(
                self.spinManualStepSize.value() / 100.0) + self.speed_rel,
                                     'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions
            time.sleep(0.10)
        else:
            print("Port is not opened")

    def manualMotorControlYPositive(self):
        if self.arduino.isOpen():
            print("+Y")
            self.arduino.write(bytes('$J=G21G91Y' + self.spinManualStepSize.textFromValue(
                self.spinManualStepSize.value() / 100.0) + self.speed_rel,
                                     'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions
            time.sleep(0.10)
        else:
            print("Port is not opened")

    def manualMotorControlYNegative(self):
        if self.arduino.isOpen():
            print("-Y")
            self.arduino.write(bytes('$J=G21G91Y-' + self.spinManualStepSize.textFromValue(
                self.spinManualStepSize.value() / 100.0) + self.speed_rel,
                                     'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions
            time.sleep(0.10)
        else:
            print("Port is not opened")

    def manualMotorControlZPositive(self):
        if self.arduino.isOpen():
            print("+Z")
            self.arduino.write(bytes('$J=G21G91Z' + self.spinManualStepSize.textFromValue(
                self.spinManualStepSize.value() / 100.0) + self.speed_rel,
                                     'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions
            time.sleep(0.10)
        else:
            print("Port is not opened")

    def manualMotorControlZNegative(self):
        if self.arduino.isOpen():
            print("-Z")
            self.arduino.write(bytes('$J=G21G91Z-' + self.spinManualStepSize.textFromValue(
                self.spinManualStepSize.value() / 100.0) + self.speed_rel,
                                     'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions
            time.sleep(0.10)
        else:
            print("Port is not opened")

    def manualMotorControlHOME(self):
        if self.arduino.isOpen():
            if self.systemCalibrate:
                print("HOME")
                self.arduino.write(bytes('G90 G0 X0 Y0\n', 'utf-8'))
                time.sleep(0.10)
            else:
                print("Calibration is needed")
        else:
            print("Port is not opened")

    def calibrateMotorSystem(self):
        if self.arduino.isOpen():
            self.arduino.write(
                bytes('$X\n', 'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions
            time.sleep(0.10)
            self.arduino.write(bytes('G10 P0 L20 X0 Y0 Z0\n',
                                     'utf-8'))  # update this by @CALSTART\r to calibrate all motors but use interrumptions
            time.sleep(0.10)
            self.systemCalibrate_label.setText('System calibrated')
            self.systemCalibrate_label.setStyleSheet("background-color: #33ff86");
            self.manualMotorControl_home_button.setEnabled(True)
            self.processControl_start_button.setEnabled(True)
            self.processControl_stop_button.setEnabled(True)
            self.systemCalibrate = True
        else:
            print("Port is not opened")

    def portConnection(self):
        if not self.portConnectStatus:
            self.arduino.port = self.comPort_line.currentText()
            self.arduino.baudrate = self.comBaudrate_line.text()
            self.arduino.open()
            #            self.arduino = serial.Serial(port=comPort, baudrate=comBaudrate, timeout=.1)
            if self.arduino.isOpen():
                self.portConnectStatus = True
                self.portConnect_button.setEnabled(False)
                self.portDisconnect_button.setEnabled(True)
                self.comPort_line.setEnabled(False)
                self.comBaudrate_line.setEnabled(False)
                self.portConnect_button.setText("Connected")
                self.portConnect_button.setStyleSheet("background-color: #33ff86");
                self.comPortRefresh_button.setEnabled(False)


        else:
            self.arduino.close()
            if not self.arduino.isOpen():
                self.portConnectStatus = False
                self.portConnect_button.setEnabled(True)
                self.portDisconnect_button.setEnabled(False)
                self.comPort_line.setEnabled(True)
                self.comBaudrate_line.setEnabled(True)
                self.portConnect_button.setText("Connect")
                self.portConnect_button.setStyleSheet("background-color: red");

                self.systemCalibrate = False
                self.systemCalibrate_label.setText('System NOT calibrated')
                self.systemCalibrate_label.setStyleSheet("background-color: red");

                self.manualMotorControl_home_button.setEnabled(False)
                self.processControl_start_button.setEnabled(False)
                self.processControl_stop_button.setEnabled(False)
                self.comPortRefresh_button.setEnabled(True)

    def refreshPorts(self):
        self.comPort_line.clear()
        ports = serial.tools.list_ports.comports(include_links=False)
        for port in ports:
            print(port.device)
            self.comPort_line.addItem(port.device)
        self.comPort_line.addItem('')

    def show_result_rows(self):
        # getting current value
        value = self.spinRowsWells.value()

    def show_result_cols(self):
        # getting current value
        value = self.spinColsWells.value()

    def startCapture(self):
        if not self.capture:
            self.capture = QtCapture(self.spinCameraID.value())
            self.pause_button.clicked.connect(self.capture.stop)
            # self.capture.setFPS(1)
            self.capture.setParent(self)
            self.capture.setWindowFlags(QtCore.Qt.Tool)
        self.capture.start()
        self.capture.show()
        self.cameraStatus = self.capture.getCameraStatus()

    def endCapture(self):
        if self.cameraStatus:
            self.capture.deleteLater()
            self.cameraStatus = False
            self.capture = None

    def quitSystem(self):
        self.explorationThread.stop()
        self.arduino.close()
        self.close()

    # ------ Modification ------ #
    def saveCapture(self):
        pass


#        if self.capture:
#            self.capture.capture()
# ------ Modification ------ #


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ControlWindow()
    sys.exit(app.exec_())